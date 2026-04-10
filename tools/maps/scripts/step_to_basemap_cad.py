#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import multiprocessing
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS


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


def heartbeat_worker(label: str, interval_sec: int, start_time: float, stop_event) -> None:
    while not stop_event.wait(interval_sec):
        elapsed = int(time.monotonic() - start_time)
        print(f"[wait] {label} ... {elapsed}s", flush=True)


class Heartbeat:
    def __init__(self, label: str, interval_sec: int):
        self.label = label
        self.interval_sec = max(1, int(interval_sec))
        self._stop_event = multiprocessing.Event()
        self._process: multiprocessing.Process | None = None
        self._start_time = 0.0

    def __enter__(self):
        self._start_time = time.monotonic()
        self._process = multiprocessing.Process(
            target=heartbeat_worker,
            args=(self.label, self.interval_sec, self._start_time, self._stop_event),
            daemon=True,
        )
        self._process.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self._stop_event.set()
        if self._process is not None:
            self._process.join(timeout=0.2)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=0.2)


def iter_projected_triangles(shape, z_min: float | None, z_max: float | None):
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation_s(face, location)
        if triangulation is not None and triangulation.NbTriangles() > 0:
            transform = location.Transformation()
            for index in range(1, triangulation.NbTriangles() + 1):
                triangle = triangulation.Triangle(index)
                node1, node2, node3 = triangle.Get()
                p1 = triangulation.Node(node1).Transformed(transform)
                p2 = triangulation.Node(node2).Transformed(transform)
                p3 = triangulation.Node(node3).Transformed(transform)
                if z_min is not None or z_max is not None:
                    avg_z = (p1.Z() + p2.Z() + p3.Z()) / 3.0
                    if z_min is not None and avg_z < z_min:
                        continue
                    if z_max is not None and avg_z > z_max:
                        continue
                yield (p1.X(), p1.Y()), (p2.X(), p2.Y()), (p3.X(), p3.Y())
        explorer.Next()


def read_step_shape(step_path: Path):
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(step_path))
    if status != IFSelect_RetDone:
        raise ValueError(f"Read STEP failed with status={status}")
    root_count = int(reader.NbRootsForTransfer())
    return reader, root_count


def parse_root_indices(raw: str, root_count: int) -> list[int]:
    if root_count <= 0:
        raise ValueError("STEP contains no transferable roots")

    spec = (raw or "all").strip().lower()
    if spec in {"all", "*"}:
        return list(range(1, root_count + 1))

    result: set[int] = set()
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token:
            pieces = token.split("-", maxsplit=1)
            if len(pieces) != 2 or not pieces[0].strip().isdigit() or not pieces[1].strip().isdigit():
                raise ValueError(f"invalid root range token: {token}")
            start = int(pieces[0].strip())
            end = int(pieces[1].strip())
            if start > end:
                raise ValueError(f"invalid root range token: {token}")
            for value in range(start, end + 1):
                result.add(value)
            continue
        if not token.isdigit():
            raise ValueError(f"invalid root token: {token}")
        result.add(int(token))

    if not result:
        raise ValueError("no valid roots selected")

    selected = sorted(result)
    out_of_range = [value for value in selected if value < 1 or value > root_count]
    if out_of_range:
        raise ValueError(f"root index out of range: {out_of_range} (valid: 1..{root_count})")
    return selected


def transfer_selected_roots(reader, selected_roots: list[int], heartbeat_sec: int):
    total = len(selected_roots)
    for index, root_id in enumerate(selected_roots, start=1):
        print(f"[info] transfer root {root_id} ({index}/{total})", flush=True)
        with Heartbeat(f"transfer root {root_id}", heartbeat_sec):
            ok = bool(reader.TransferRoot(root_id))
        if not ok:
            print(f"[warn] transfer root {root_id} returned false", flush=True)
    return reader.OneShape()


def compute_bounds(shape, z_min: float | None, z_max: float | None, progress_every: int) -> tuple[Bounds2D, int]:
    min_x = float("inf")
    max_x = float("-inf")
    min_y = float("inf")
    max_y = float("-inf")
    tri_count = 0

    for p1, p2, p3 in iter_projected_triangles(shape, z_min, z_max):
        for x_val, y_val in (p1, p2, p3):
            min_x = min(min_x, x_val)
            max_x = max(max_x, x_val)
            min_y = min(min_y, y_val)
            max_y = max(max_y, y_val)
        tri_count += 1
        if progress_every > 0 and tri_count % progress_every == 0:
            print(f"[bounds] triangles={tri_count}", flush=True)

    if not math.isfinite(min_x):
        raise ValueError("no triangles after z-filter")
    return Bounds2D(min_x=min_x, max_x=max_x, min_y=min_y, max_y=max_y), tri_count


def to_pixel_mapper(bounds: Bounds2D, width: int, height: int):
    range_x = bounds.range_x
    range_y = bounds.range_y
    if range_x <= 0 or range_y <= 0:
        raise ValueError("invalid bounds range")

    scale = min((width - 1) / range_x, (height - 1) / range_y)
    x_pad = ((width - 1) - range_x * scale) * 0.5
    y_pad = ((height - 1) - range_y * scale) * 0.5

    def mapper(x_val: float, y_val: float) -> tuple[int, int]:
        px = int(round((x_val - bounds.min_x) * scale + x_pad))
        py_bottom = int(round((y_val - bounds.min_y) * scale + y_pad))
        py = (height - 1) - py_bottom
        return px, py

    return mapper, {"scale": scale, "x_pad": x_pad, "y_pad": y_pad}


def draw_wireframe(
    shape,
    bounds: Bounds2D,
    width: int,
    height: int,
    z_min: float | None,
    z_max: float | None,
    line_thickness: int,
    blur: int,
    progress_every: int,
) -> tuple[np.ndarray, int, dict]:
    canvas = np.full((height, width), 255, dtype=np.uint8)
    to_pixel, map_meta = to_pixel_mapper(bounds, width, height)
    tri_count = 0

    for p1, p2, p3 in iter_projected_triangles(shape, z_min, z_max):
        pt1 = to_pixel(*p1)
        pt2 = to_pixel(*p2)
        pt3 = to_pixel(*p3)
        cv2.line(canvas, pt1, pt2, color=0, thickness=line_thickness, lineType=cv2.LINE_AA)
        cv2.line(canvas, pt2, pt3, color=0, thickness=line_thickness, lineType=cv2.LINE_AA)
        cv2.line(canvas, pt3, pt1, color=0, thickness=line_thickness, lineType=cv2.LINE_AA)
        tri_count += 1
        if progress_every > 0 and tri_count % progress_every == 0:
            print(f"[draw] triangles={tri_count}", flush=True)

    if blur > 0:
        kernel = blur if blur % 2 == 1 else blur + 1
        canvas = cv2.GaussianBlur(canvas, (kernel, kernel), 0)
    return canvas, tri_count, map_meta


def main() -> None:
    parser = argparse.ArgumentParser(description="CAD-based STEP -> top-view wireframe PNG")
    parser.add_argument("--input", required=True, help="STEP path")
    parser.add_argument("--output", required=True, help="Output PNG")
    parser.add_argument("--meta-output", default="", help="Output meta JSON")
    parser.add_argument("--width", type=int, default=2800, help="Image width")
    parser.add_argument("--height", type=int, default=1500, help="Image height")
    parser.add_argument("--deflection", type=float, default=20.0, help="Meshing deflection (STEP unit)")
    parser.add_argument("--angle", type=float, default=0.5, help="Meshing angle (rad)")
    parser.add_argument("--z-min", type=float, default=None, help="Optional z-min")
    parser.add_argument("--z-max", type=float, default=None, help="Optional z-max")
    parser.add_argument("--line-thickness", type=int, default=1, help="Wireframe line thickness")
    parser.add_argument("--blur", type=int, default=3, help="Post blur kernel (0 disable)")
    parser.add_argument("--progress-every", type=int, default=50000, help="Progress print interval")
    parser.add_argument("--heartbeat", type=int, default=10, help="Seconds between wait logs in long CAD stages")
    parser.add_argument(
        "--crop-field",
        action="store_true",
        help="Center-crop world bounds to target field size before rasterization",
    )
    parser.add_argument(
        "--field-width",
        type=float,
        default=28000.0,
        help="Target field width in STEP unit (used with --crop-field)",
    )
    parser.add_argument(
        "--field-height",
        type=float,
        default=15000.0,
        help="Target field height in STEP unit (used with --crop-field)",
    )
    parser.add_argument(
        "--roots",
        default="all",
        help="STEP roots to transfer: all or comma/range form, e.g. 1,3-5",
    )
    args = parser.parse_args()

    step_path = Path(args.input).expanduser().resolve()
    if not step_path.exists():
        raise SystemExit(f"STEP not found: {step_path}")

    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    width = max(128, int(args.width))
    height = max(128, int(args.height))
    line_thickness = max(1, int(args.line_thickness))
    heartbeat_sec = max(1, int(args.heartbeat))

    size_mb = step_path.stat().st_size / (1024 * 1024)
    print(f"[info] input size={size_mb:.2f}MB", flush=True)
    if size_mb > 300.0:
        print("[warn] large STEP detected: parsing may take several minutes", flush=True)

    print("[info] read step", flush=True)
    with Heartbeat("read step", heartbeat_sec):
        reader, root_count = read_step_shape(step_path)
    print(f"[info] transferable roots={root_count}", flush=True)

    selected_roots = parse_root_indices(args.roots, root_count)
    print(f"[info] selected roots={selected_roots}", flush=True)

    print("[info] transfer roots", flush=True)
    shape = transfer_selected_roots(reader, selected_roots, heartbeat_sec)

    print("[info] mesh shape", flush=True)
    with Heartbeat("mesh shape", heartbeat_sec):
        BRepMesh_IncrementalMesh(shape, float(args.deflection), True, float(args.angle), True)

    print("[info] compute bounds", flush=True)
    raw_bounds, tri_bounds = compute_bounds(shape, args.z_min, args.z_max, args.progress_every)
    bounds = raw_bounds
    if args.crop_field:
        field_w = float(args.field_width)
        field_h = float(args.field_height)
        if field_w <= 0 or field_h <= 0:
            raise ValueError("field width/height must be positive")
        cx = (raw_bounds.min_x + raw_bounds.max_x) * 0.5
        cy = (raw_bounds.min_y + raw_bounds.max_y) * 0.5
        bounds = Bounds2D(
            min_x=cx - field_w * 0.5,
            max_x=cx + field_w * 0.5,
            min_y=cy - field_h * 0.5,
            max_y=cy + field_h * 0.5,
        )
        print(
            f"[info] crop field enabled center=({cx:.3f},{cy:.3f}) "
            f"target=[{field_w:.3f},{field_h:.3f}]",
            flush=True,
        )
    print(
        f"[info] bounds x=[{bounds.min_x:.3f},{bounds.max_x:.3f}] "
        f"y=[{bounds.min_y:.3f},{bounds.max_y:.3f}] triangles={tri_bounds}",
        flush=True,
    )

    print("[info] draw wireframe", flush=True)
    image, tri_drawn, map_meta = draw_wireframe(
        shape=shape,
        bounds=bounds,
        width=width,
        height=height,
        z_min=args.z_min,
        z_max=args.z_max,
        line_thickness=line_thickness,
        blur=max(0, int(args.blur)),
        progress_every=args.progress_every,
    )
    cv2.imwrite(str(output_path), image)
    print(f"[ok] cad basemap png: {output_path}", flush=True)

    meta = {
        "input": str(step_path),
        "output": str(output_path),
        "engine": "cad_wireframe",
        "input_size_mb": size_mb,
        "width": width,
        "height": height,
        "z_filter": {"min": args.z_min, "max": args.z_max},
        "roots": {"total": root_count, "selected": selected_roots},
        "mesh": {"deflection": args.deflection, "angle": args.angle},
        "field_crop": {
            "enabled": bool(args.crop_field),
            "target_width": float(args.field_width),
            "target_height": float(args.field_height),
        },
        "draw": {"line_thickness": line_thickness, "blur": args.blur},
        "raw_bounds": {
            "min_x": raw_bounds.min_x,
            "max_x": raw_bounds.max_x,
            "min_y": raw_bounds.min_y,
            "max_y": raw_bounds.max_y,
            "range_x": raw_bounds.range_x,
            "range_y": raw_bounds.range_y,
        },
        "bounds": {
            "min_x": bounds.min_x,
            "max_x": bounds.max_x,
            "min_y": bounds.min_y,
            "max_y": bounds.max_y,
            "range_x": bounds.range_x,
            "range_y": bounds.range_y,
        },
        "triangles": {"for_bounds": tri_bounds, "drawn": tri_drawn},
        "mapper": map_meta,
    }

    meta_output = Path(args.meta_output).expanduser().resolve() if args.meta_output else output_path.with_suffix(".meta.json")
    meta_output.parent.mkdir(parents=True, exist_ok=True)
    meta_output.write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"[ok] cad basemap meta: {meta_output}", flush=True)


if __name__ == "__main__":
    main()
