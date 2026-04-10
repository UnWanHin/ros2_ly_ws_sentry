#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import re
from dataclasses import asdict, dataclass
from pathlib import Path


HEADER_FILE_NAME_RE = re.compile(r"/\*\s*name\s*\*/\s*'([^']*)'")
HEADER_TIME_RE = re.compile(r"/\*\s*time_stamp\s*\*/\s*'([^']*)'")
FILE_SCHEMA_RE = re.compile(r"FILE_SCHEMA\s*\(\('([^']*)'\)\)")
CARTESIAN_RE = re.compile(
    r"CARTESIAN_POINT\('',\(([-+0-9.Ee]+),([-+0-9.Ee]+),([-+0-9.Ee]+)\)\)"
)


@dataclass
class Bounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float

    @property
    def range_x(self) -> float:
        return self.max_x - self.min_x

    @property
    def range_y(self) -> float:
        return self.max_y - self.min_y

    @property
    def range_z(self) -> float:
        return self.max_z - self.min_z

    def to_dict(self) -> dict:
        data = asdict(self)
        data["range_x"] = self.range_x
        data["range_y"] = self.range_y
        data["range_z"] = self.range_z
        return data


def detect_length_unit(step_path: Path) -> str:
    block_lines: list[str] = []
    in_block = False
    with step_path.open("r", encoding="utf-8", errors="ignore") as file_obj:
        for raw in file_obj:
            line = raw.strip()
            if line.startswith("#") and line.endswith("=("):
                in_block = True
                block_lines = [line]
                continue
            if not in_block:
                continue
            block_lines.append(line)
            if line == ");":
                in_block = False
                if any("LENGTH_UNIT()" in item for item in block_lines):
                    for item in block_lines:
                        if "SI_UNIT(" in item:
                            if ".MILLI." in item and ".METRE." in item:
                                return "millimeter"
                            if ".CENTI." in item and ".METRE." in item:
                                return "centimeter"
                            if "$,.METRE." in item or ".METRE." in item:
                                return "meter"
                block_lines = []
    return "unknown"


def read_header_info(step_path: Path) -> dict:
    file_name = ""
    timestamp = ""
    schema = ""
    with step_path.open("r", encoding="utf-8", errors="ignore") as file_obj:
        for _ in range(600):
            line = file_obj.readline()
            if not line:
                break
            if not file_name:
                match = HEADER_FILE_NAME_RE.search(line)
                if match:
                    file_name = match.group(1)
            if not timestamp:
                match = HEADER_TIME_RE.search(line)
                if match:
                    timestamp = match.group(1)
            if not schema:
                match = FILE_SCHEMA_RE.search(line)
                if match:
                    schema = match.group(1)
            if file_name and timestamp and schema:
                break
    return {"step_internal_name": file_name, "step_timestamp": timestamp, "file_schema": schema}


def scan_bounds(step_path: Path) -> tuple[Bounds | None, int]:
    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")
    point_count = 0

    with step_path.open("r", encoding="utf-8", errors="ignore") as file_obj:
        for line in file_obj:
            match = CARTESIAN_RE.search(line)
            if not match:
                continue
            x_value = float(match.group(1))
            y_value = float(match.group(2))
            z_value = float(match.group(3))
            min_x = min(min_x, x_value)
            max_x = max(max_x, x_value)
            min_y = min(min_y, y_value)
            max_y = max(max_y, y_value)
            min_z = min(min_z, z_value)
            max_z = max(max_z, z_value)
            point_count += 1

    if point_count == 0:
        return None, 0
    return Bounds(min_x, max_x, min_y, max_y, min_z, max_z), point_count


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect STEP file basics for map workflow.")
    parser.add_argument("step_file", type=Path, help="Path to STEP file")
    parser.add_argument("--output", type=Path, default=None, help="Optional JSON output file")
    args = parser.parse_args()

    step_path = args.step_file.expanduser().resolve()
    if not step_path.exists():
        raise SystemExit(f"STEP file not found: {step_path}")

    header = read_header_info(step_path)
    unit_name = detect_length_unit(step_path)
    bounds, point_count = scan_bounds(step_path)

    result = {
        "path": str(step_path),
        "size_bytes": step_path.stat().st_size,
        "size_mb": round(step_path.stat().st_size / (1024 * 1024), 2),
        "unit": unit_name,
        "cartesian_point_count": point_count,
        **header,
        "bounds": bounds.to_dict() if bounds else None,
    }

    output_text = json.dumps(result, ensure_ascii=False, indent=2)
    print(output_text)

    if args.output is not None:
        output_path = args.output.expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(output_text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

