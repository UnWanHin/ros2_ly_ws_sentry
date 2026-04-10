#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


POINT_ID_NAME = {
    0: "Home",
    1: "Base",
    2: "Recovery",
    3: "BuffShoot",
    4: "LeftHighLand",
    5: "CastleLeft",
    6: "Castle",
    7: "CastleRight1",
    8: "CastleRight2",
    9: "FlyRoad",
    10: "OutpostArea",
    11: "MidShoot",
    12: "LeftShoot",
    13: "OutpostShoot",
    14: "BuffAround1",
    15: "BuffAround2",
    16: "RightShoot",
    17: "HoleRoad",
    18: "OccupyArea",
}


@dataclass
class PointEntry:
    id_value: int
    name: str
    red: tuple[int, int]
    blue: tuple[int, int]


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def normalize_point(raw: dict[str, Any]) -> PointEntry:
    id_value = int(raw["id"])
    name = str(raw.get("name") or POINT_ID_NAME.get(id_value, f"Point{id_value}"))
    red = tuple(int(round(float(value))) for value in raw["red"])
    blue = tuple(int(round(float(value))) for value in raw["blue"])
    return PointEntry(id_value=id_value, name=name, red=(red[0], red[1]), blue=(blue[0], blue[1]))


def validate_plugin(data: dict[str, Any]) -> tuple[list[PointEntry], list[str]]:
    errors: list[str] = []
    points_raw = data.get("points")
    if not isinstance(points_raw, list):
        return [], ["'points' must be a list"]

    entries: list[PointEntry] = []
    seen_ids: set[int] = set()

    for index, raw in enumerate(points_raw):
        if not isinstance(raw, dict):
            errors.append(f"points[{index}] must be an object")
            continue
        for key in ("id", "red", "blue"):
            if key not in raw:
                errors.append(f"points[{index}] missing key '{key}'")
        if "id" not in raw or "red" not in raw or "blue" not in raw:
            continue
        try:
            entry = normalize_point(raw)
        except Exception as error:
            errors.append(f"points[{index}] parse error: {error}")
            continue
        if entry.id_value < 0 or entry.id_value > 18:
            errors.append(f"points[{index}] id={entry.id_value} out of range 0..18")
        if entry.id_value in seen_ids:
            errors.append(f"duplicate point id={entry.id_value}")
        seen_ids.add(entry.id_value)
        entries.append(entry)

    for id_value in sorted(POINT_ID_NAME):
        if id_value not in seen_ids:
            errors.append(f"missing point id={id_value} ({POINT_ID_NAME[id_value]})")

    map_size = data.get("map_size_cm", {})
    if not isinstance(map_size, dict):
        errors.append("'map_size_cm' must be an object")
    else:
        width = map_size.get("width")
        height = map_size.get("height")
        if width is None or height is None:
            errors.append("'map_size_cm.width' and 'map_size_cm.height' are required")
        else:
            try:
                width_value = float(width)
                height_value = float(height)
                if width_value <= 0 or height_value <= 0:
                    errors.append("map_size_cm must be > 0")
            except Exception:
                errors.append("map_size_cm width/height must be numeric")

    return entries, errors


def cmd_init(args: argparse.Namespace) -> None:
    output = Path(args.output).expanduser().resolve()
    data = {
        "map_name": args.map_name,
        "frame": "left_bottom_origin_cm",
        "map_size_cm": {"width": 2800, "height": 1500},
        "points": [
            {
                "id": id_value,
                "name": name,
                "red": [0, 0],
                "blue": [0, 0],
            }
            for id_value, name in POINT_ID_NAME.items()
        ],
    }
    write_json(output, data)
    print(f"created: {output}")


def cmd_validate(args: argparse.Namespace) -> None:
    input_path = Path(args.input).expanduser().resolve()
    data = read_json(input_path)
    _, errors = validate_plugin(data)
    if errors:
        print("validation failed:")
        for error in errors:
            print(f"- {error}")
        raise SystemExit(2)
    print("validation ok")


def cmd_emit_area(args: argparse.Namespace) -> None:
    input_path = Path(args.input).expanduser().resolve()
    data = read_json(input_path)
    entries, errors = validate_plugin(data)
    if errors:
        print("validation failed:")
        for error in errors:
            print(f"- {error}")
        raise SystemExit(2)

    entries_by_id = {entry.id_value: entry for entry in entries}
    for id_value in sorted(POINT_ID_NAME):
        entry = entries_by_id[id_value]
        print(
            f"static const Location<std::uint16_t> {entry.name}"
            f"{{ {{{entry.red[0]}, {entry.red[1]}}}, {{{entry.blue[0]}, {entry.blue[1]}}} }};"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Map plugin helper for BT map points.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    parser_init = subparsers.add_parser("init", help="Create plugin JSON template")
    parser_init.add_argument("--output", required=True, help="Output JSON path")
    parser_init.add_argument("--map-name", default="RMUC2026_custom", help="Map name")
    parser_init.set_defaults(func=cmd_init)

    parser_validate = subparsers.add_parser("validate", help="Validate plugin JSON")
    parser_validate.add_argument("--input", required=True, help="Plugin JSON path")
    parser_validate.set_defaults(func=cmd_validate)

    parser_emit = subparsers.add_parser("emit-area", help="Emit Area.hpp Location lines")
    parser_emit.add_argument("--input", required=True, help="Plugin JSON path")
    parser_emit.set_defaults(func=cmd_emit_area)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()

