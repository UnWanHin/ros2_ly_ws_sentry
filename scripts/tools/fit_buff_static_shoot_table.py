#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import yaml


FEATURE_KEYS = (
    "intercept",
    "coef_z",
    "coef_d",
    "coef_z2",
    "coef_zd",
    "coef_d2",
)


def default_record_dir() -> Path:
    home = os.environ.get("HOME")
    if home:
        return Path(home) / "workspace" / "record"
    return Path("./record")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit buff static shooting-table coefficients from buff_shooting_table CSV records."
    )
    parser.add_argument("csv", nargs="*", help="CSV files to fit.")
    parser.add_argument("--latest", action="store_true", help="Fit newest buff_shooting_table_*.csv in --record-dir.")
    parser.add_argument("--all-in-dir", action="store_true", help="Fit all buff_shooting_table_*.csv in --record-dir.")
    parser.add_argument("--record-dir", default=str(default_record_dir()), help="Directory containing buff_shooting_table_*.csv")
    parser.add_argument("--mode-filter", choices=["all", "small", "big"], default="all", help="Filter samples by buff mode.")
    parser.add_argument("--min-samples", type=int, default=8, help="Minimum sample count required for fitting.")
    parser.add_argument("--output-yaml", help="Write ROS2 parameter override YAML.")
    parser.add_argument("--write-config", help="Rewrite target config YAML in-place and create a timestamped backup.")
    return parser.parse_args()


def resolve_csv_paths(args: argparse.Namespace) -> list[Path]:
    explicit = [Path(p).expanduser().resolve() for p in args.csv]
    if explicit:
        return explicit

    record_dir = Path(args.record_dir).expanduser().resolve()
    candidates = sorted(record_dir.glob("buff_shooting_table_*.csv"))
    if args.latest:
        if not candidates:
            raise FileNotFoundError(f"No buff_shooting_table_*.csv found in {record_dir}")
        return [max(candidates, key=lambda p: p.stat().st_mtime)]
    if args.all_in_dir:
        if not candidates:
            raise FileNotFoundError(f"No buff_shooting_table_*.csv found in {record_dir}")
        return candidates

    raise ValueError("Specify CSV files, or use --latest / --all-in-dir.")


def mode_keep(mode_filter: str, mode_value: int) -> bool:
    if mode_filter == "all":
        return True
    if mode_filter == "small":
        return mode_value == 1
    return mode_value == 2


def load_samples(csv_paths: list[Path], mode_filter: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int]:
    z_values: list[float] = []
    d_values: list[float] = []
    pitch_values: list[float] = []
    yaw_values: list[float] = []
    filtered_out = 0

    required = {"mode", "height_m", "distance_m", "relative_pitch", "relative_yaw"}
    for path in csv_paths:
        if not path.is_file():
            raise FileNotFoundError(f"CSV not found: {path}")
        with path.open(newline="", encoding="utf-8") as fh:
            reader = csv.DictReader(fh)
            if reader.fieldnames is None:
                raise ValueError(f"CSV has no header: {path}")
            missing = required.difference(reader.fieldnames)
            if missing:
                raise ValueError(f"CSV missing columns {sorted(missing)}: {path}")

            for row in reader:
                try:
                    mode = int(float(row["mode"]))
                    if not mode_keep(mode_filter, mode):
                        filtered_out += 1
                        continue
                    z_values.append(float(row["height_m"]))
                    d_values.append(float(row["distance_m"]))
                    pitch_values.append(float(row["relative_pitch"]))
                    yaw_values.append(float(row["relative_yaw"]))
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"Invalid numeric row in {path}: {row}") from exc

    if not z_values:
        raise ValueError("No calibration samples loaded from CSV after filtering.")

    return (
        np.asarray(z_values, dtype=float),
        np.asarray(d_values, dtype=float),
        np.asarray(pitch_values, dtype=float),
        np.asarray(yaw_values, dtype=float),
        filtered_out,
    )


def build_design_matrix(z_values: np.ndarray, d_values: np.ndarray) -> np.ndarray:
    return np.column_stack(
        (
            np.ones_like(z_values),
            z_values,
            d_values,
            z_values * z_values,
            z_values * d_values,
            d_values * d_values,
        )
    )


def fit_axis(matrix: np.ndarray, target: np.ndarray) -> tuple[list[float], dict[str, float]]:
    coeffs, _, rank, _ = np.linalg.lstsq(matrix, target, rcond=None)
    prediction = matrix @ coeffs
    error = prediction - target
    metrics = {
        "rank": float(rank),
        "rmse": float(math.sqrt(np.mean(np.square(error)))),
        "mae": float(np.mean(np.abs(error))),
        "max_abs": float(np.max(np.abs(error))),
    }
    return [float(v) for v in coeffs], metrics


def build_override_yaml(pitch_coeffs: list[float], yaw_coeffs: list[float]) -> dict:
    params = {
        "buff_config.static_shoot_table_adjust_enable": True,
        "buff_config.static_pitch_adjust_coeffs": pitch_coeffs,
        "buff_config.static_yaw_adjust_coeffs": yaw_coeffs,
    }
    return {"/**": {"ros__parameters": params}}


def write_override_yaml(path: Path, pitch_coeffs: list[float], yaw_coeffs: list[float]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(build_override_yaml(pitch_coeffs, yaw_coeffs), fh, sort_keys=False, allow_unicode=False)


def write_config_yaml(path: Path, pitch_coeffs: list[float], yaw_coeffs: list[float]) -> Path:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")

    with path.open(encoding="utf-8") as fh:
        content = yaml.safe_load(fh) or {}

    params_root = content.setdefault("/**", {}).setdefault("ros__parameters", {})
    buff_cfg = params_root.get("buff_config")
    if not isinstance(buff_cfg, dict):
        buff_cfg = {}

    buff_cfg["static_shoot_table_adjust_enable"] = True
    buff_cfg["static_pitch_adjust_coeffs"] = [float(v) for v in pitch_coeffs]
    buff_cfg["static_yaw_adjust_coeffs"] = [float(v) for v in yaw_coeffs]
    params_root["buff_config"] = buff_cfg

    backup = path.with_name(f"{path.name}.bak.{int(time.time())}")
    backup.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    with path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(content, fh, sort_keys=False, allow_unicode=False)
    return backup


def default_output_yaml(csv_paths: list[Path], record_dir: Path) -> Path:
    if len(csv_paths) == 1:
        return csv_paths[0].with_suffix(".static_fit.yaml")
    return record_dir / f"buff_static_fit_{int(time.time())}.yaml"


def print_axis_result(name: str, coeffs: list[float], metrics: dict[str, float]) -> None:
    print(f"{name}:")
    for i, key in enumerate(FEATURE_KEYS):
        print(f"  {key}: {coeffs[i]:.8f}")
    print(
        "  stats:"
        f" rmse={metrics['rmse']:.6f}"
        f" mae={metrics['mae']:.6f}"
        f" max_abs={metrics['max_abs']:.6f}"
        f" rank={metrics['rank']:.0f}"
    )


def main() -> int:
    args = parse_args()
    try:
        csv_paths = resolve_csv_paths(args)
        z_values, d_values, pitch_values, yaw_values, filtered_out = load_samples(csv_paths, args.mode_filter)
        sample_count = len(z_values)
        if sample_count < args.min_samples:
            raise ValueError(
                f"Need at least {args.min_samples} samples for fitting, but only got {sample_count}."
            )

        design = build_design_matrix(z_values, d_values)
        pitch_coeffs, pitch_metrics = fit_axis(design, pitch_values)
        yaw_coeffs, yaw_metrics = fit_axis(design, yaw_values)

        print("Fitted buff static shooting-table coefficients")
        print(f"samples: {sample_count} (filtered_out: {filtered_out})")
        print(f"mode_filter: {args.mode_filter}")
        print("csv:")
        for path in csv_paths:
            print(f"  - {path}")
        print_axis_result("pitch", pitch_coeffs, pitch_metrics)
        print_axis_result("yaw", yaw_coeffs, yaw_metrics)

        record_dir = Path(args.record_dir).expanduser().resolve()
        output_yaml = (
            Path(args.output_yaml).expanduser().resolve()
            if args.output_yaml
            else default_output_yaml(csv_paths, record_dir)
        )
        write_override_yaml(output_yaml, pitch_coeffs, yaw_coeffs)
        print(f"override_yaml: {output_yaml}")

        if args.write_config:
            backup = write_config_yaml(Path(args.write_config), pitch_coeffs, yaw_coeffs)
            print(f"config_updated: {Path(args.write_config).expanduser().resolve()}")
            print(f"config_backup: {backup}")

        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"[fit_buff_static_shoot_table] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
