#!/usr/bin/env python3

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def prompt(message: str) -> str:
    return input(message).strip()


def choose_index(title: str, options: list[str]) -> int:
    if not options:
        raise ValueError("empty options")
    print(title)
    for index, option in enumerate(options, start=1):
        print(f"  {index}) {option}")
    while True:
        raw = prompt(f"請輸入序號 (1-{len(options)}): ")
        if raw.isdigit():
            idx = int(raw) - 1
            if 0 <= idx < len(options):
                return idx
        print("輸入無效，請重試。")


def find_step_files(step_dir: Path) -> list[Path]:
    files = list(step_dir.rglob("*.step")) + list(step_dir.rglob("*.stp"))
    return sorted(set(path.resolve() for path in files))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate basemap PNG from STEP")
    parser.add_argument("--step-file", default="", help="STEP file path")
    parser.add_argument("--output", default="", help="Output PNG path")
    parser.add_argument("--engine", choices=["cad", "points"], default="cad", help="Basemap engine")
    parser.add_argument("--width", type=int, default=2800, help="Output width")
    parser.add_argument("--height", type=int, default=1500, help="Output height")
    parser.add_argument("--z-min", type=float, default=None, help="Optional z-min filter")
    parser.add_argument("--z-max", type=float, default=None, help="Optional z-max filter")
    parser.add_argument("--percentile", type=float, default=70.0, help="Contrast percentile")
    parser.add_argument("--blur", type=int, default=3, help="Blur kernel size")
    parser.add_argument("--deflection", type=float, default=20.0, help="CAD meshing deflection (engine=cad)")
    parser.add_argument("--angle", type=float, default=0.5, help="CAD meshing angle in rad (engine=cad)")
    parser.add_argument("--line-thickness", type=int, default=1, help="CAD line thickness (engine=cad)")
    parser.add_argument("--crop-field", action="store_true", help="Center-crop CAD world to field size")
    parser.add_argument("--field-width", type=float, default=28000.0, help="Field width in STEP unit (with --crop-field)")
    parser.add_argument("--field-height", type=float, default=15000.0, help="Field height in STEP unit (with --crop-field)")
    parser.add_argument("--roots", default="all", help="CAD roots to transfer: all or 1,3-5")
    parser.add_argument("--heartbeat", type=int, default=10, help="CAD long-stage wait log interval (sec)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    base_dir = Path(__file__).resolve().parent
    step_dir = base_dir / "src"
    basemap_dir = base_dir / "basemaps"
    basemap_dir.mkdir(parents=True, exist_ok=True)

    if args.step_file:
        step_path = Path(args.step_file).expanduser().resolve()
        if not step_path.exists():
            raise SystemExit(f"STEP 不存在：{step_path}")
    else:
        step_files = find_step_files(step_dir)
        if not step_files:
            raise SystemExit(f"找不到 STEP，請放到：{step_dir}")
        selected = choose_index(
            "請選擇 STEP：",
            [str(path.relative_to(base_dir)) if path.is_relative_to(base_dir) else str(path) for path in step_files],
        )
        step_path = step_files[selected]

    if args.output:
        output_png = Path(args.output).expanduser().resolve()
    else:
        output_png = (basemap_dir / f"{step_path.stem}_topview.png").resolve()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    output_meta = output_png.with_suffix(".meta.json")

    if args.engine == "cad":
        script = base_dir / "scripts" / "step_to_basemap_cad.py"
        step_size_mb = step_path.stat().st_size / (1024 * 1024)
        if step_size_mb > 300.0:
            print(f"[warn] large STEP ({step_size_mb:.2f}MB), CAD parse may take several minutes", flush=True)
        command = [
            sys.executable,
            str(script),
            "--input",
            str(step_path),
            "--output",
            str(output_png),
            "--meta-output",
            str(output_meta),
            "--width",
            str(max(256, int(args.width))),
            "--height",
            str(max(256, int(args.height))),
            "--deflection",
            str(float(args.deflection)),
            "--angle",
            str(float(args.angle)),
            "--line-thickness",
            str(max(1, int(args.line_thickness))),
            "--blur",
            str(int(args.blur)),
            "--roots",
            str(args.roots),
            "--heartbeat",
            str(max(1, int(args.heartbeat))),
            "--field-width",
            str(float(args.field_width)),
            "--field-height",
            str(float(args.field_height)),
            "--progress-every",
            "50000",
        ]
        if args.crop_field:
            command.append("--crop-field")
    else:
        script = base_dir / "scripts" / "step_to_basemap.py"
        command = [
            sys.executable,
            str(script),
            "--input",
            str(step_path),
            "--output",
            str(output_png),
            "--meta-output",
            str(output_meta),
            "--width",
            str(max(256, int(args.width))),
            "--height",
            str(max(256, int(args.height))),
            "--percentile",
            str(float(args.percentile)),
            "--blur",
            str(int(args.blur)),
            "--progress-every",
            "250000",
        ]
    if args.z_min is not None:
        command.extend(["--z-min", str(args.z_min)])
    if args.z_max is not None:
        command.extend(["--z-max", str(args.z_max)])

    subprocess.run(command, check=True)

    print("\n=== STEP -> PNG 完成 ===")
    print(f"Engine: {args.engine}")
    print(f"STEP: {step_path}")
    print(f"PNG : {output_png}")
    print(f"META: {output_meta}")
    print("\n下一步可用：")
    print(
        f"python3 tools/maps/mappointer.py --step-file {step_path} --image-file {output_png}"
    )


if __name__ == "__main__":
    main()
