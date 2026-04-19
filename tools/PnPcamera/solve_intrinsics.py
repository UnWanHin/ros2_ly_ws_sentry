#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np


# ===== Quick Settings (edit here) =====
BOARD_INNER_CORNERS = (8, 11)  # (cols, rows), e.g. (8, 11) or (11, 8)
SQUARE_SIZE_METERS = 0.024      # chessboard square size in meters
DISTORTION_OUTPUT_COUNT = 5     # solver_config usually uses 5 values: k1,k2,p1,p2,k3
# =====================================

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SAVE_ROOT = SCRIPT_DIR / "save"
DEFAULT_OUTPUT_NAME = "calibration_result.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Solve camera intrinsics from chessboard images."
    )
    parser.add_argument(
        "--input-dir",
        default="",
        help="Image directory. If not provided, script asks interactively.",
    )
    parser.add_argument(
        "--camera-name",
        default="ly_camera",
        help="Camera name in output yaml. Default: ly_camera",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Output yaml path. Default: <input-dir>/calibration_result.yaml",
    )
    parser.add_argument(
        "--show-detect",
        action="store_true",
        help="Show corner detection preview window.",
    )
    return parser.parse_args()


def ask_input_dir(cli_value: str) -> Path:
    if cli_value:
        return Path(cli_value).expanduser().resolve()

    default_path = DEFAULT_SAVE_ROOT
    raw = input(f"Image folder path [{default_path}]: ").strip()
    if not raw:
        return default_path.resolve()
    return Path(raw).expanduser().resolve()


def collect_images(image_dir: Path) -> list[Path]:
    exts = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")
    files = [p for p in sorted(image_dir.glob("*")) if p.suffix.lower() in exts and p.is_file()]
    return files


def build_object_points() -> np.ndarray:
    cols, rows = BOARD_INNER_CORNERS
    obj = np.zeros((cols * rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj *= float(SQUARE_SIZE_METERS)
    return obj


def mean_reprojection_error(
    objpoints: list[np.ndarray],
    imgpoints: list[np.ndarray],
    rvecs: list[np.ndarray],
    tvecs: list[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> float:
    total_error = 0.0
    for i, obj in enumerate(objpoints):
        projected, _ = cv2.projectPoints(obj, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        err = cv2.norm(imgpoints[i], projected, cv2.NORM_L2) / len(projected)
        total_error += float(err)
    return total_error / max(len(objpoints), 1)


def write_yaml(
    output_path: Path,
    camera_name: str,
    image_size: tuple[int, int],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> None:
    width, height = image_size
    k = camera_matrix.reshape(-1).tolist()
    d = dist_coeffs.reshape(-1).tolist()
    r = np.eye(3, dtype=np.float64).reshape(-1).tolist()
    p = np.hstack([camera_matrix, np.zeros((3, 1), dtype=np.float64)]).reshape(-1).tolist()

    def fmt(values: list[float], precision: int = 8) -> str:
        return ", ".join(f"{float(v):.{precision}f}" for v in values)

    content = (
        f"image_width: {width}\n"
        f"image_height: {height}\n"
        f"camera_name: {camera_name}\n"
        "camera_matrix:\n"
        "  rows: 3\n"
        "  cols: 3\n"
        f"  data: [{fmt(k)}]\n"
        "distortion_model: plumb_bob\n"
        "distortion_coefficients:\n"
        "  rows: 1\n"
        f"  cols: {len(d)}\n"
        f"  data: [{fmt(d)}]\n"
        "rectification_matrix:\n"
        "  rows: 3\n"
        "  cols: 3\n"
        f"  data: [{fmt(r)}]\n"
        "projection_matrix:\n"
        "  rows: 3\n"
        "  cols: 4\n"
        f"  data: [{fmt(p)}]\n"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(content, encoding="utf-8")


def save_solver_snippet(output_dir: Path, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Path:
    k9 = camera_matrix.reshape(-1).tolist()
    d = dist_coeffs.reshape(-1).tolist()
    if len(d) < DISTORTION_OUTPUT_COUNT:
        d = d + [0.0] * (DISTORTION_OUTPUT_COUNT - len(d))
    d = d[:DISTORTION_OUTPUT_COUNT]

    def fmt(values: list[float], precision: int = 8) -> str:
        return ", ".join(f"{float(v):.{precision}f}" for v in values)

    snippet = (
        "solver_config:\n"
        f"  camera_intrinsic_matrix: [{fmt(k9)}]\n"
        f"  camera_distortion_coefficients: [{fmt(d)}]\n"
    )
    path = output_dir / "solver_config_snippet.yaml"
    path.write_text(snippet, encoding="utf-8")
    return path


def main() -> int:
    args = parse_args()

    image_dir = ask_input_dir(args.input_dir)
    if not image_dir.exists() or not image_dir.is_dir():
        print(f"[ERROR] input dir not found: {image_dir}")
        return 1

    images = collect_images(image_dir)
    if not images:
        print(f"[ERROR] no images found in {image_dir}")
        return 1

    print(f"[INFO] input dir: {image_dir}")
    print(f"[INFO] image count: {len(images)}")
    print(f"[INFO] board inner corners: {BOARD_INNER_CORNERS}, square: {SQUARE_SIZE_METERS} m")

    obj_template = build_object_points()
    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None

    # More stable chessboard detection under uneven lighting.
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)

    valid_count = 0
    for img_path in images:
        image = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])

        found, corners = cv2.findChessboardCorners(gray, BOARD_INNER_CORNERS, flags)
        if found:
            refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
            objpoints.append(obj_template.copy())
            imgpoints.append(refined)
            valid_count += 1

            if args.show_detect:
                vis = image.copy()
                cv2.drawChessboardCorners(vis, BOARD_INNER_CORNERS, refined, True)
                cv2.imshow("corner_detect", vis)
                cv2.waitKey(40)

    if args.show_detect:
        cv2.destroyWindow("corner_detect")

    if valid_count < 6:
        print(f"[ERROR] valid chessboard frames too few: {valid_count} (need >= 6)")
        return 1

    assert image_size is not None
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )
    reproj = mean_reprojection_error(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist_coeffs)

    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output
        else (image_dir / DEFAULT_OUTPUT_NAME)
    )
    write_yaml(output_path, args.camera_name, image_size, camera_matrix, dist_coeffs)
    snippet_path = save_solver_snippet(image_dir, camera_matrix, dist_coeffs)

    k9 = camera_matrix.reshape(-1).tolist()
    d = dist_coeffs.reshape(-1).tolist()
    d5 = (d + [0.0] * DISTORTION_OUTPUT_COUNT)[:DISTORTION_OUTPUT_COUNT]

    print("")
    print("[RESULT]")
    print(f"  valid frames: {valid_count}/{len(images)}")
    print(f"  RMS: {rms:.6f}")
    print(f"  mean reprojection error: {reproj:.6f} px")
    print(f"  yaml: {output_path}")
    print(f"  solver snippet: {snippet_path}")
    print("")
    print("solver_config.camera_intrinsic_matrix:")
    print("  [" + ", ".join(f"{v:.8f}" for v in k9) + "]")
    print("solver_config.camera_distortion_coefficients:")
    print("  [" + ", ".join(f"{v:.8f}" for v in d5) + "]")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    legacy_copy = image_dir / f"calibration_result_{stamp}.yaml"
    if legacy_copy != output_path:
        write_yaml(legacy_copy, args.camera_name, image_size, camera_matrix, dist_coeffs)
        print(f"[INFO] timestamp copy: {legacy_copy}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
