#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class Pair:
    src: tuple[float, float]
    dst: tuple[float, float]


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def mat_mul_transpose_a_a(rows: list[list[float]]) -> list[list[float]]:
    size = len(rows[0])
    result = [[0.0 for _ in range(size)] for _ in range(size)]
    for row in rows:
        for i in range(size):
            for j in range(size):
                result[i][j] += row[i] * row[j]
    return result


def mat_mul_transpose_a_b(rows: list[list[float]], values: list[float]) -> list[float]:
    size = len(rows[0])
    result = [0.0 for _ in range(size)]
    for row, value in zip(rows, values):
        for i in range(size):
            result[i] += row[i] * value
    return result


def solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:
    n = len(vector)
    aug = [row[:] + [vector[index]] for index, row in enumerate(matrix)]

    for col in range(n):
        pivot_row = max(range(col, n), key=lambda row_index: abs(aug[row_index][col]))
        if abs(aug[pivot_row][col]) < 1e-12:
            raise ValueError("singular matrix, please provide better control points")
        aug[col], aug[pivot_row] = aug[pivot_row], aug[col]

        pivot = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= pivot

        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]

    return [aug[index][n] for index in range(n)]


def parse_pairs(raw_pairs: list[dict[str, Any]]) -> list[Pair]:
    pairs: list[Pair] = []
    for index, item in enumerate(raw_pairs):
        if not isinstance(item, dict):
            raise ValueError(f"pairs[{index}] must be object")
        src = item.get("src")
        dst = item.get("dst")
        if not isinstance(src, list) or not isinstance(dst, list) or len(src) != 2 or len(dst) != 2:
            raise ValueError(f"pairs[{index}] requires src=[x,y] and dst=[x,y]")
        pairs.append(
            Pair(
                src=(float(src[0]), float(src[1])),
                dst=(float(dst[0]), float(dst[1])),
            )
        )
    return pairs


def fit_affine(pairs: list[Pair]) -> tuple[float, float, float, float, float, float]:
    if len(pairs) < 3:
        raise ValueError("need at least 3 control pairs")

    rows: list[list[float]] = []
    values: list[float] = []
    for pair in pairs:
        x_value, y_value = pair.src
        x_target, y_target = pair.dst
        rows.append([x_value, y_value, 1.0, 0.0, 0.0, 0.0])
        values.append(x_target)
        rows.append([0.0, 0.0, 0.0, x_value, y_value, 1.0])
        values.append(y_target)

    normal_matrix = mat_mul_transpose_a_a(rows)
    normal_vector = mat_mul_transpose_a_b(rows, values)
    solution = solve_linear_system(normal_matrix, normal_vector)
    return solution[0], solution[1], solution[2], solution[3], solution[4], solution[5]


def apply_affine(coeff: tuple[float, float, float, float, float, float], x_value: float, y_value: float) -> tuple[float, float]:
    a_value, b_value, c_value, d_value, e_value, f_value = coeff
    return (
        a_value * x_value + b_value * y_value + c_value,
        d_value * x_value + e_value * y_value + f_value,
    )


def calc_rmse(coeff: tuple[float, float, float, float, float, float], pairs: list[Pair]) -> float:
    if not pairs:
        return 0.0
    sum_sq = 0.0
    for pair in pairs:
        pred_x, pred_y = apply_affine(coeff, pair.src[0], pair.src[1])
        dx = pred_x - pair.dst[0]
        dy = pred_y - pair.dst[1]
        sum_sq += dx * dx + dy * dy
    return (sum_sq / len(pairs)) ** 0.5


def main() -> None:
    parser = argparse.ArgumentParser(description="Solve 2D affine mapping from control point pairs.")
    parser.add_argument("--input", required=True, help="JSON file with pairs")
    parser.add_argument("--output", default="", help="Optional output JSON")
    args = parser.parse_args()

    input_path = Path(args.input).expanduser().resolve()
    payload = read_json(input_path)
    pairs = parse_pairs(payload.get("pairs", []))
    coeff = fit_affine(pairs)
    rmse_value = calc_rmse(coeff, pairs)

    output = {
        "source_frame": payload.get("source_frame", ""),
        "target_frame": payload.get("target_frame", ""),
        "affine": {
            "a": coeff[0],
            "b": coeff[1],
            "c": coeff[2],
            "d": coeff[3],
            "e": coeff[4],
            "f": coeff[5],
        },
        "rmse": rmse_value,
    }

    if isinstance(payload.get("apply_to"), list):
        mapped = []
        for item in payload["apply_to"]:
            if not isinstance(item, list) or len(item) != 2:
                continue
            x_value, y_value = float(item[0]), float(item[1])
            out_x, out_y = apply_affine(coeff, x_value, y_value)
            mapped.append([out_x, out_y])
        output["mapped_points"] = mapped

    text = json.dumps(output, ensure_ascii=False, indent=2)
    print(text)

    if args.output:
        out_path = Path(args.output).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

