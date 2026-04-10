#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from collections import Counter

import numpy as np
from PIL import Image


CARTESIAN_RE = re.compile(
    r"CARTESIAN_POINT\('',\(([-+0-9.Ee]+),([-+0-9.Ee]+),([-+0-9.Ee]+)\)\)"
)


@dataclass
class PcaResult:
    mean_x: float
    mean_y: float
    angle_rad: float
    point_count: int


@dataclass
class Bounds2D:
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    @property
    def range_x(self) -> float:
        return self.max_x - self.min_x

    @property
    def range_y(self) -> float:
        return self.max_y - self.min_y


@dataclass
class Transform2D:
    mode: str
    mean_x: float = 0.0
    mean_y: float = 0.0
    angle_rad: float = 0.0


def iter_points(step_path: Path, z_min: float | None, z_max: float | None):
    with step_path.open("r", encoding="utf-8", errors="ignore") as file_obj:
        for line in file_obj:
            match = CARTESIAN_RE.search(line)
            if not match:
                continue
            x_val = float(match.group(1))
            y_val = float(match.group(2))
            z_val = float(match.group(3))
            if z_min is not None and z_val < z_min:
                continue
            if z_max is not None and z_val > z_max:
                continue
            yield x_val, y_val, z_val


def detect_top_z_bin(step_path: Path, progress_every: int) -> tuple[int, int]:
    counter: Counter[int] = Counter()
    count = 0
    with step_path.open("r", encoding="utf-8", errors="ignore") as file_obj:
        for line in file_obj:
            match = CARTESIAN_RE.search(line)
            if not match:
                continue
            z_val = float(match.group(3))
            z_bin = int(round(z_val))
            counter[z_bin] += 1
            count += 1
            if progress_every > 0 and count % progress_every == 0:
                print(f"[z-detect] points={count}", flush=True)
    if not counter:
        raise ValueError("no CARTESIAN_POINT found for z detect")
    top_z, top_count = counter.most_common(1)[0]
    return top_z, top_count


def compute_pca(step_path: Path, z_min: float | None, z_max: float | None, progress_every: int) -> PcaResult:
    count = 0
    mean_x = 0.0
    mean_y = 0.0
    cov_xx = 0.0
    cov_xy = 0.0
    cov_yy = 0.0

    for x_val, y_val, _ in iter_points(step_path, z_min, z_max):
        count += 1
        dx = x_val - mean_x
        dy = y_val - mean_y
        mean_x += dx / count
        mean_y += dy / count
        cov_xx += dx * (x_val - mean_x)
        cov_xy += dx * (y_val - mean_y)
        cov_yy += dy * (y_val - mean_y)

        if progress_every > 0 and count % progress_every == 0:
            print(f"[pass1] points={count}", flush=True)

    if count < 2:
        raise ValueError("not enough points to compute PCA")

    c_xx = cov_xx / (count - 1)
    c_xy = cov_xy / (count - 1)
    c_yy = cov_yy / (count - 1)
    angle = 0.5 * math.atan2(2.0 * c_xy, c_xx - c_yy)
    return PcaResult(mean_x=mean_x, mean_y=mean_y, angle_rad=angle, point_count=count)


def transform_xy(x_val: float, y_val: float, transform: Transform2D) -> tuple[float, float]:
    if transform.mode == "pca":
        dx = x_val - transform.mean_x
        dy = y_val - transform.mean_y
        cos_a = math.cos(-transform.angle_rad)
        sin_a = math.sin(-transform.angle_rad)
        xr = cos_a * dx - sin_a * dy
        yr = sin_a * dx + cos_a * dy
        return xr, yr
    return x_val, y_val


def compute_transformed_bounds(
    step_path: Path,
    transform: Transform2D,
    z_min: float | None,
    z_max: float | None,
    progress_every: int,
) -> Bounds2D:
    min_x = float("inf")
    max_x = float("-inf")
    min_y = float("inf")
    max_y = float("-inf")
    count = 0

    for x_val, y_val, _ in iter_points(step_path, z_min, z_max):
        xr, yr = transform_xy(x_val, y_val, transform)
        min_x = min(min_x, xr)
        max_x = max(max_x, xr)
        min_y = min(min_y, yr)
        max_y = max(max_y, yr)
        count += 1
        if progress_every > 0 and count % progress_every == 0:
            print(f"[pass2] points={count}", flush=True)

    if not math.isfinite(min_x) or not math.isfinite(min_y):
        raise ValueError("failed to compute rotated bounds")
    return Bounds2D(min_x=min_x, max_x=max_x, min_y=min_y, max_y=max_y)


def rasterize_density(
    step_path: Path,
    transform: Transform2D,
    bounds: Bounds2D,
    width: int,
    height: int,
    z_min: float | None,
    z_max: float | None,
    progress_every: int,
) -> np.ndarray:
    density = np.zeros((height, width), dtype=np.uint32)
    count = 0

    range_x = bounds.range_x
    range_y = bounds.range_y
    if range_x <= 0 or range_y <= 0:
        raise ValueError("invalid bounds range")

    scale = min((width - 1) / range_x, (height - 1) / range_y)
    x_pad = ((width - 1) - range_x * scale) * 0.5
    y_pad = ((height - 1) - range_y * scale) * 0.5

    for x_val, y_val, _ in iter_points(step_path, z_min, z_max):
        xr, yr = transform_xy(x_val, y_val, transform)
        px = int(round((xr - bounds.min_x) * scale + x_pad))
        py_bottom = int(round((yr - bounds.min_y) * scale + y_pad))
        py = (height - 1) - py_bottom
        if 0 <= px < width and 0 <= py < height:
            density[py, px] += 1
        count += 1
        if progress_every > 0 and count % progress_every == 0:
            print(f"[pass3] points={count}", flush=True)

    return density


def density_to_image(density: np.ndarray, blur: int, percentile: float) -> np.ndarray:
    if density.max() == 0:
        return np.full(density.shape, 255, dtype=np.uint8)

    values = np.log1p(density.astype(np.float32))
    max_value = float(values.max())
    if max_value <= 0:
        return np.full(density.shape, 255, dtype=np.uint8)

    normalized = values / max_value
    threshold = np.percentile(normalized, percentile)
    normalized = np.clip((normalized - threshold) / max(1e-6, 1.0 - threshold), 0.0, 1.0)
    image = (255.0 * (1.0 - normalized)).astype(np.uint8)

    if blur > 0:
        import cv2

        kernel = blur if blur % 2 == 1 else blur + 1
        image = cv2.GaussianBlur(image, (kernel, kernel), 0)
    return image


def main() -> None:
    parser = argparse.ArgumentParser(description="Extract top-view basemap PNG from STEP point cloud")
    parser.add_argument("--input", required=True, help="STEP file path")
    parser.add_argument("--output", required=True, help="Output PNG path")
    parser.add_argument("--meta-output", default="", help="Optional meta JSON output path")
    parser.add_argument("--width", type=int, default=2800, help="Output width in pixels")
    parser.add_argument("--height", type=int, default=1500, help="Output height in pixels")
    parser.add_argument("--z-min", type=float, default=None, help="Optional Z min filter (STEP unit)")
    parser.add_argument("--z-max", type=float, default=None, help="Optional Z max filter (STEP unit)")
    parser.add_argument("--auto-z-window", type=float, default=10.0, help="When z filter is empty, use dominant z±window")
    parser.add_argument("--rotate", choices=["none", "pca"], default="none", help="Coordinate alignment mode")
    parser.add_argument("--blur", type=int, default=3, help="Gaussian blur kernel size (0=disable)")
    parser.add_argument("--percentile", type=float, default=70.0, help="Contrast percentile (0-99)")
    parser.add_argument("--progress-every", type=int, default=200000, help="Print progress every N points (0=quiet)")
    args = parser.parse_args()

    step_path = Path(args.input).expanduser().resolve()
    if not step_path.exists():
        raise SystemExit(f"STEP not found: {step_path}")

    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    width = max(64, int(args.width))
    height = max(64, int(args.height))
    percentile = min(99.0, max(0.0, float(args.percentile)))

    z_min = args.z_min
    z_max = args.z_max
    auto_z_center = None
    auto_z_count = None
    if z_min is None and z_max is None:
        auto_z_center, auto_z_count = detect_top_z_bin(step_path, args.progress_every)
        z_min = auto_z_center - float(args.auto_z_window)
        z_max = auto_z_center + float(args.auto_z_window)
        print(
            f"[info] auto z slice: center={auto_z_center} count={auto_z_count} "
            f"window=±{args.auto_z_window} => [{z_min}, {z_max}]",
            flush=True,
        )

    transform = Transform2D(mode="none")
    pca = None
    if args.rotate == "pca":
        print("[info] pass1: PCA orientation", flush=True)
        pca = compute_pca(step_path, z_min, z_max, args.progress_every)
        transform = Transform2D(
            mode="pca",
            mean_x=pca.mean_x,
            mean_y=pca.mean_y,
            angle_rad=pca.angle_rad,
        )
        print(f"[info] pca angle(deg)={math.degrees(pca.angle_rad):.6f}, points={pca.point_count}", flush=True)
    else:
        print("[info] transform: none (raw x/y)", flush=True)

    print("[info] pass2: transformed bounds", flush=True)
    bounds = compute_transformed_bounds(step_path, transform, z_min, z_max, args.progress_every)
    print(
        f"[info] bounds x=[{bounds.min_x:.3f},{bounds.max_x:.3f}] "
        f"y=[{bounds.min_y:.3f},{bounds.max_y:.3f}]",
        flush=True,
    )

    print("[info] pass3: rasterize density", flush=True)
    density = rasterize_density(
        step_path=step_path,
        transform=transform,
        bounds=bounds,
        width=width,
        height=height,
        z_min=z_min,
        z_max=z_max,
        progress_every=args.progress_every,
    )

    image = density_to_image(density, blur=args.blur, percentile=percentile)
    Image.fromarray(image, mode="L").save(output_path)
    print(f"[ok] basemap png: {output_path}", flush=True)

    meta = {
        "input": str(step_path),
        "output": str(output_path),
        "width": width,
        "height": height,
        "z_filter": {"min": z_min, "max": z_max},
        "auto_z": {
            "center": auto_z_center,
            "count": auto_z_count,
            "window": args.auto_z_window,
        },
        "transform": {
            "mode": transform.mode,
            "mean_x": transform.mean_x,
            "mean_y": transform.mean_y,
            "angle_rad": transform.angle_rad,
            "angle_deg": math.degrees(transform.angle_rad),
            "point_count": pca.point_count if pca is not None else None,
        },
        "rotated_bounds": {
            "min_x": bounds.min_x,
            "max_x": bounds.max_x,
            "min_y": bounds.min_y,
            "max_y": bounds.max_y,
            "range_x": bounds.range_x,
            "range_y": bounds.range_y,
        },
        "render": {"blur": args.blur, "percentile": percentile},
    }

    meta_output = Path(args.meta_output).expanduser().resolve() if args.meta_output else output_path.with_suffix(".meta.json")
    meta_output.parent.mkdir(parents=True, exist_ok=True)
    meta_output.write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"[ok] meta json: {meta_output}", flush=True)


if __name__ == "__main__":
    main()
