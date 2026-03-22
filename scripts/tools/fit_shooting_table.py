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
from copy import deepcopy
from pathlib import Path

import numpy as np
import yaml


FEATURE_KEYS = ("intercept", "coef_z", "coef_d", "coef_z2", "coef_zd", "coef_d2")


def default_record_dir() -> Path:
    home = os.environ.get("HOME")
    if home:
        return Path(home) / "workspace" / "record"
    return Path("./record")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit shooting-table compensation coefficients from calibration CSV records."
    )
    parser.add_argument("csv", nargs="*", help="CSV files to fit.")
    parser.add_argument(
        "--latest",
        action="store_true",
        help="Fit the newest shooting_table_*.csv in --record-dir.",
    )
    parser.add_argument(
        "--all-in-dir",
        action="store_true",
        help="Fit all shooting_table_*.csv under --record-dir.",
    )
    parser.add_argument(
        "--record-dir",
        default=str(default_record_dir()),
        help="Directory containing shooting_table_*.csv files.",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=6,
        help="Minimum sample count required for fitting. Default: 6.",
    )
    parser.add_argument(
        "--output-yaml",
        help="Write a ROS2 parameter override YAML with fitted coefficients.",
    )
    parser.add_argument(
        "--write-config",
        help="Rewrite a config YAML in-place with fitted coefficients. A timestamped backup is created.",
    )
    return parser.parse_args()


def resolve_csv_paths(args: argparse.Namespace) -> list[Path]:
    explicit = [Path(p).expanduser().resolve() for p in args.csv]
    if explicit:
        return explicit

    record_dir = Path(args.record_dir).expanduser().resolve()
    candidates = sorted(record_dir.glob("shooting_table_*.csv"))
    if args.latest:
        if not candidates:
            raise FileNotFoundError(f"No shooting_table_*.csv found in {record_dir}")
        return [max(candidates, key=lambda p: p.stat().st_mtime)]
    if args.all_in_dir:
        if not candidates:
            raise FileNotFoundError(f"No shooting_table_*.csv found in {record_dir}")
        return candidates

    raise ValueError("Specify CSV files, or use --latest / --all-in-dir.")


def load_samples(csv_paths: list[Path]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    z_values: list[float] = []
    d_values: list[float] = []
    pitch_values: list[float] = []
    yaw_values: list[float] = []

    required = {"z_height", "horizontal_distance", "relative_pitch", "relative_yaw"}
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
                    z_values.append(float(row["z_height"]))
                    d_values.append(float(row["horizontal_distance"]))
                    pitch_values.append(float(row["relative_pitch"]))
                    yaw_values.append(float(row["relative_yaw"]))
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"Invalid numeric row in {path}: {row}") from exc

    if not z_values:
        raise ValueError("No calibration samples loaded from CSV.")

    return (
        np.asarray(z_values, dtype=float),
        np.asarray(d_values, dtype=float),
        np.asarray(pitch_values, dtype=float),
        np.asarray(yaw_values, dtype=float),
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


def fit_axis(matrix: np.ndarray, target: np.ndarray) -> tuple[dict[str, float], dict[str, float]]:
    coeffs, _, rank, _ = np.linalg.lstsq(matrix, target, rcond=None)
    prediction = matrix @ coeffs
    error = prediction - target
    metrics = {
        "rank": float(rank),
        "rmse": float(math.sqrt(np.mean(np.square(error)))),
        "mae": float(np.mean(np.abs(error))),
        "max_abs": float(np.max(np.abs(error))),
    }
    return {key: float(value) for key, value in zip(FEATURE_KEYS, coeffs)}, metrics


def build_override_yaml(pitch: dict[str, float], yaw: dict[str, float]) -> dict:
    params = {
        "shoot_table_adjust.enable": True,
        "shoot_table_adjust.pitch.intercept": pitch["intercept"],
        "shoot_table_adjust.pitch.coef_z": pitch["coef_z"],
        "shoot_table_adjust.pitch.coef_d": pitch["coef_d"],
        "shoot_table_adjust.pitch.coef_z2": pitch["coef_z2"],
        "shoot_table_adjust.pitch.coef_zd": pitch["coef_zd"],
        "shoot_table_adjust.pitch.coef_d2": pitch["coef_d2"],
        "shoot_table_adjust.yaw.intercept": yaw["intercept"],
        "shoot_table_adjust.yaw.coef_z": yaw["coef_z"],
        "shoot_table_adjust.yaw.coef_d": yaw["coef_d"],
        "shoot_table_adjust.yaw.coef_z2": yaw["coef_z2"],
        "shoot_table_adjust.yaw.coef_zd": yaw["coef_zd"],
        "shoot_table_adjust.yaw.coef_d2": yaw["coef_d2"],
        "controller_config.shoot_table_adjust.enable": True,
        "controller_config.shoot_table_adjust.pitch.intercept": pitch["intercept"],
        "controller_config.shoot_table_adjust.pitch.coef_z": pitch["coef_z"],
        "controller_config.shoot_table_adjust.pitch.coef_d": pitch["coef_d"],
        "controller_config.shoot_table_adjust.pitch.coef_z2": pitch["coef_z2"],
        "controller_config.shoot_table_adjust.pitch.coef_zd": pitch["coef_zd"],
        "controller_config.shoot_table_adjust.pitch.coef_d2": pitch["coef_d2"],
        "controller_config.shoot_table_adjust.yaw.intercept": yaw["intercept"],
        "controller_config.shoot_table_adjust.yaw.coef_z": yaw["coef_z"],
        "controller_config.shoot_table_adjust.yaw.coef_d": yaw["coef_d"],
        "controller_config.shoot_table_adjust.yaw.coef_z2": yaw["coef_z2"],
        "controller_config.shoot_table_adjust.yaw.coef_zd": yaw["coef_zd"],
        "controller_config.shoot_table_adjust.yaw.coef_d2": yaw["coef_d2"],
    }
    return {"/**": {"ros__parameters": params}}


def nested_adjust_block(pitch: dict[str, float], yaw: dict[str, float]) -> dict:
    return {
        "enable": True,
        "pitch": dict(pitch),
        "yaw": dict(yaw),
    }


def write_override_yaml(path: Path, pitch: dict[str, float], yaw: dict[str, float]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(build_override_yaml(pitch, yaw), fh, sort_keys=False, allow_unicode=False)


def write_config_yaml(path: Path, pitch: dict[str, float], yaw: dict[str, float]) -> Path:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")

    with path.open(encoding="utf-8") as fh:
        content = yaml.safe_load(fh) or {}

    params_root = content.setdefault("/**", {}).setdefault("ros__parameters", {})
    adjust_block = nested_adjust_block(pitch, yaw)
    params_root["shoot_table_adjust"] = deepcopy(adjust_block)
    controller = params_root.get("controller_config")
    if not isinstance(controller, dict):
        controller = {}
    controller["shoot_table_adjust"] = deepcopy(adjust_block)
    params_root["controller_config"] = controller

    backup = path.with_name(f"{path.name}.bak.{int(time.time())}")
    backup.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    with path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(content, fh, sort_keys=False, allow_unicode=False)
    return backup


def default_output_yaml(csv_paths: list[Path], record_dir: Path) -> Path:
    if len(csv_paths) == 1:
        return csv_paths[0].with_suffix(".fit.yaml")
    return record_dir / f"shoot_table_fit_{int(time.time())}.yaml"


def print_axis_result(name: str, coeffs: dict[str, float], metrics: dict[str, float]) -> None:
    print(f"{name}:")
    for key in FEATURE_KEYS:
        print(f"  {key}: {coeffs[key]:.8f}")
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
        z_values, d_values, pitch_values, yaw_values = load_samples(csv_paths)
        sample_count = len(z_values)
        if sample_count < args.min_samples:
            raise ValueError(
                f"Need at least {args.min_samples} samples for fitting, but only got {sample_count}."
            )

        design = build_design_matrix(z_values, d_values)
        pitch_coeffs, pitch_metrics = fit_axis(design, pitch_values)
        yaw_coeffs, yaw_metrics = fit_axis(design, yaw_values)

        print("Fitted shooting table coefficients")
        print(f"samples: {sample_count}")
        print("csv:")
        for path in csv_paths:
            print(f"  - {path}")
        print_axis_result("pitch", pitch_coeffs, pitch_metrics)
        print_axis_result("yaw", yaw_coeffs, yaw_metrics)

        record_dir = Path(args.record_dir).expanduser().resolve()
        output_yaml = Path(args.output_yaml).expanduser().resolve() if args.output_yaml else default_output_yaml(csv_paths, record_dir)
        write_override_yaml(output_yaml, pitch_coeffs, yaw_coeffs)
        print(f"override_yaml: {output_yaml}")

        if args.write_config:
            backup = write_config_yaml(Path(args.write_config), pitch_coeffs, yaw_coeffs)
            print(f"config_updated: {Path(args.write_config).expanduser().resolve()}")
            print(f"config_backup: {backup}")

        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"[fit_shooting_table] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
