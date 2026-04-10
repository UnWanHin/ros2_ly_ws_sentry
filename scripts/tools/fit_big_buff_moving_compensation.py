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
    "coef_sin",
    "coef_cos",
    "coef_sin2",
    "coef_cos2",
    "coef_dist",
    "coef_height",
)


def default_record_dir() -> Path:
    home = os.environ.get("HOME")
    if home:
        return Path(home) / "workspace" / "record"
    return Path("./record")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit big-buff periodic compensation coefficients from buff_shooting_table CSV records."
    )
    parser.add_argument("csv", nargs="*", help="CSV files to fit.")
    parser.add_argument("--latest", action="store_true", help="Fit newest buff_shooting_table_*.csv in --record-dir.")
    parser.add_argument("--all-in-dir", action="store_true", help="Fit all buff_shooting_table_*.csv in --record-dir.")
    parser.add_argument("--record-dir", default=str(default_record_dir()), help="Directory containing buff_shooting_table_*.csv")
    parser.add_argument("--include-small-mode", action="store_true", help="Include small-buff samples (mode=1). Default only mode=2.")
    parser.add_argument("--min-samples", type=int, default=12, help="Minimum sample count required for fitting.")
    parser.add_argument("--apply-on-big-only", choices=["true", "false"], default="true", help="Set periodic_apply_on_big_buff_only in output YAML.")
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


def parse_bool_text(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on", "y"}


def load_samples(csv_paths: list[Path], include_small_mode: bool) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, int]:
    theta_values: list[float] = []
    dist_values: list[float] = []
    height_values: list[float] = []
    pitch_values: list[float] = []
    yaw_values: list[float] = []
    filtered_out = 0

    required = {"mode", "rotation_angle", "distance_m", "height_m", "relative_pitch", "relative_yaw"}
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
                    if mode != 2 and (not include_small_mode):
                        filtered_out += 1
                        continue
                    theta_values.append(float(row["rotation_angle"]))
                    dist_values.append(float(row["distance_m"]))
                    height_values.append(float(row["height_m"]))
                    pitch_values.append(float(row["relative_pitch"]))
                    yaw_values.append(float(row["relative_yaw"]))
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"Invalid numeric row in {path}: {row}") from exc

    if not theta_values:
        raise ValueError("No calibration samples loaded from CSV after filtering.")

    return (
        np.asarray(theta_values, dtype=float),
        np.asarray(dist_values, dtype=float),
        np.asarray(height_values, dtype=float),
        np.asarray(pitch_values, dtype=float),
        np.asarray(yaw_values, dtype=float),
        filtered_out,
    )


def build_design_matrix(theta_values: np.ndarray, dist_values: np.ndarray, height_values: np.ndarray) -> np.ndarray:
    sin_theta = np.sin(theta_values)
    cos_theta = np.cos(theta_values)
    sin_2theta = np.sin(2.0 * theta_values)
    cos_2theta = np.cos(2.0 * theta_values)
    return np.column_stack(
        (
            np.ones_like(theta_values),
            sin_theta,
            cos_theta,
            sin_2theta,
            cos_2theta,
            dist_values,
            height_values,
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


def build_override_yaml(pitch_coeffs: list[float], yaw_coeffs: list[float], apply_on_big_only: bool) -> dict:
    params = {
        "buff_config.periodic_shoot_table_adjust_enable": True,
        "buff_config.periodic_apply_on_big_buff_only": bool(apply_on_big_only),
        "buff_config.periodic_pitch_adjust_coeffs": pitch_coeffs,
        "buff_config.periodic_yaw_adjust_coeffs": yaw_coeffs,
    }
    return {"/**": {"ros__parameters": params}}


def write_override_yaml(path: Path, pitch_coeffs: list[float], yaw_coeffs: list[float], apply_on_big_only: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(
            build_override_yaml(pitch_coeffs, yaw_coeffs, apply_on_big_only),
            fh,
            sort_keys=False,
            allow_unicode=False,
        )


def write_config_yaml(path: Path, pitch_coeffs: list[float], yaw_coeffs: list[float], apply_on_big_only: bool) -> Path:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")

    with path.open(encoding="utf-8") as fh:
        content = yaml.safe_load(fh) or {}

    params_root = content.setdefault("/**", {}).setdefault("ros__parameters", {})
    buff_cfg = params_root.get("buff_config")
    if not isinstance(buff_cfg, dict):
        buff_cfg = {}

    buff_cfg["periodic_shoot_table_adjust_enable"] = True
    buff_cfg["periodic_apply_on_big_buff_only"] = bool(apply_on_big_only)
    buff_cfg["periodic_pitch_adjust_coeffs"] = [float(v) for v in pitch_coeffs]
    buff_cfg["periodic_yaw_adjust_coeffs"] = [float(v) for v in yaw_coeffs]
    params_root["buff_config"] = buff_cfg

    backup = path.with_name(f"{path.name}.bak.{int(time.time())}")
    backup.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    with path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(content, fh, sort_keys=False, allow_unicode=False)
    return backup


def default_output_yaml(csv_paths: list[Path], record_dir: Path) -> Path:
    if len(csv_paths) == 1:
        return csv_paths[0].with_suffix(".periodic_fit.yaml")
    return record_dir / f"buff_periodic_fit_{int(time.time())}.yaml"


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
        include_small_mode = bool(args.include_small_mode)
        apply_on_big_only = parse_bool_text(args.apply_on_big_only)
        theta_values, dist_values, height_values, pitch_values, yaw_values, filtered_out = load_samples(
            csv_paths,
            include_small_mode,
        )
        sample_count = len(theta_values)
        if sample_count < args.min_samples:
            raise ValueError(
                f"Need at least {args.min_samples} samples for fitting, but only got {sample_count}."
            )

        design = build_design_matrix(theta_values, dist_values, height_values)
        pitch_coeffs, pitch_metrics = fit_axis(design, pitch_values)
        yaw_coeffs, yaw_metrics = fit_axis(design, yaw_values)

        print("Fitted big-buff periodic compensation coefficients")
        print(f"samples: {sample_count} (filtered_out: {filtered_out})")
        print(f"include_small_mode: {include_small_mode}")
        print(f"apply_on_big_only: {apply_on_big_only}")
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
        write_override_yaml(output_yaml, pitch_coeffs, yaw_coeffs, apply_on_big_only)
        print(f"override_yaml: {output_yaml}")

        if args.write_config:
            backup = write_config_yaml(Path(args.write_config), pitch_coeffs, yaw_coeffs, apply_on_big_only)
            print(f"config_updated: {Path(args.write_config).expanduser().resolve()}")
            print(f"config_backup: {backup}")

        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"[fit_big_buff_moving_compensation] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
