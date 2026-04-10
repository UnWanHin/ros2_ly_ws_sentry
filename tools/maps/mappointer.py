#!/usr/bin/env python3

from __future__ import annotations

import argparse
import base64
import json
import re
import subprocess
import sys
import webbrowser
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlencode


@dataclass
class MapPlugin:
    map_name: str
    path: Path


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
        if not raw.isdigit():
            print("請輸入數字。")
            continue
        value = int(raw)
        if 1 <= value <= len(options):
            return value - 1
        print("超出範圍，請重試。")


def slugify(name: str) -> str:
    result = re.sub(r"[^a-zA-Z0-9._-]+", "_", name).strip("_")
    return result or "map"


def find_step_files(step_dir: Path) -> list[Path]:
    files = list(step_dir.rglob("*.step")) + list(step_dir.rglob("*.stp"))
    return sorted(set(path.resolve() for path in files))


def load_map_plugin(path: Path) -> MapPlugin | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    if not isinstance(payload, dict):
        return None
    if not isinstance(payload.get("points"), list):
        return None
    map_name = str(payload.get("map_name") or path.stem)
    return MapPlugin(map_name=map_name, path=path.resolve())


def find_existing_maps(base_dir: Path) -> list[MapPlugin]:
    candidates: list[Path] = []
    plugins_dir = base_dir / "plugins"
    if plugins_dir.exists():
        candidates.extend(sorted(plugins_dir.glob("*.json")))
    candidates.extend(sorted(base_dir.glob("map_plugin*.json")))

    result: list[MapPlugin] = []
    for file_path in candidates:
        plugin = load_map_plugin(file_path)
        if plugin is not None:
            result.append(plugin)

    plugins_dir = (base_dir / "plugins").resolve()
    result.sort(
        key=lambda item: (
            item.map_name.lower(),
            0 if item.path.parent.resolve() == plugins_dir else 1,
            str(item.path),
        )
    )
    return result


def run_cli(script_path: Path, args: list[str]) -> None:
    command = [sys.executable, str(script_path), *args]
    subprocess.run(command, check=True)


def create_or_update_map(base_dir: Path, map_name: str, allow_overwrite: bool) -> Path:
    plugins_dir = base_dir / "plugins"
    plugins_dir.mkdir(parents=True, exist_ok=True)

    file_name = slugify(map_name) + ".json"
    output_path = plugins_dir / file_name

    if output_path.exists() and not allow_overwrite:
        while True:
            answer = prompt(
                f"地圖檔已存在：{output_path}\n"
                "  1) 直接使用\n"
                "  2) 覆蓋重建\n"
                "  3) 重新命名\n"
                "請選擇: "
            )
            if answer == "1":
                return output_path.resolve()
            if answer == "2":
                break
            if answer == "3":
                new_name = prompt("請輸入新的 map name: ")
                if new_name:
                    return create_or_update_map(base_dir, new_name, allow_overwrite=False)
                continue
            print("請輸入 1/2/3。")

    map_cli = base_dir / "scripts" / "map_plugin_cli.py"
    run_cli(
        map_cli,
        ["init", "--output", str(output_path), "--map-name", map_name],
    )
    return output_path.resolve()


def inspect_step(base_dir: Path, step_path: Path) -> Path | None:
    inspect_script = base_dir / "scripts" / "step_inspect.py"
    output_dir = base_dir / "meta"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{step_path.stem}.inspect.json"
    try:
        command = [sys.executable, str(inspect_script), str(step_path), "--output", str(output_path)]
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return output_path.resolve()
    except Exception as error:
        print(f"[WARN] STEP 檢查失敗：{error}")
        return None


def build_marker_url(
    base_dir: Path,
    map_name: str,
    plugin_path: Path | None = None,
    image_path: Path | None = None,
) -> str:
    marker_path = (base_dir / "web" / "map_marker.html").resolve()
    query_params: dict[str, str] = {"map_name": map_name}
    if image_path is not None:
        query_params["image_uri"] = image_path.as_uri()
    if plugin_path is not None and plugin_path.exists():
        query_params["plugin_uri"] = plugin_path.as_uri()
        try:
            plugin_raw = plugin_path.read_bytes()
            plugin_b64 = base64.urlsafe_b64encode(plugin_raw).decode("ascii")
            if len(plugin_b64) <= 12000:
                query_params["plugin_b64"] = plugin_b64
        except Exception:
            pass
    query = urlencode(query_params)
    return f"{marker_path.as_uri()}?{query}"


def open_browser(url: str) -> bool:
    try:
        return bool(webbrowser.open(url, new=2))
    except Exception:
        return False


def select_step_interactive(base_dir: Path, step_files: list[Path]) -> Path:
    index = choose_index(
        "請選擇 STEP 檔案：",
        [str(path.relative_to(base_dir)) if path.is_relative_to(base_dir) else str(path) for path in step_files],
    )
    return step_files[index]


def choose_map_interactive(base_dir: Path) -> tuple[str, Path]:
    existing = find_existing_maps(base_dir)
    if existing:
        mode_index = choose_index(
            "請選擇地圖來源：",
            ["使用既有地圖插件", "新建地圖插件"],
        )
    else:
        mode_index = 1

    if mode_index == 0 and existing:
        selected_index = choose_index(
            "請選擇既有 map：",
            [f"{item.map_name}  ({item.path.name})" for item in existing],
        )
        selected = existing[selected_index]
        return selected.map_name, selected.path

    map_name = prompt("請輸入新地圖名稱 (map_name): ")
    while not map_name:
        map_name = prompt("名稱不可空白，請再輸入 map_name: ")
    plugin_path = create_or_update_map(base_dir, map_name, allow_overwrite=False)
    return map_name, plugin_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive map pointer launcher")
    parser.add_argument("--step-file", default="", help="Direct STEP file path (skip step selection)")
    parser.add_argument("--map-name", default="", help="Map name to create/use")
    parser.add_argument("--image-file", default="", help="Optional existing basemap PNG for marker page")
    parser.add_argument("--new-map", action="store_true", help="Force create new map plugin")
    parser.add_argument("--existing-map", action="store_true", help="Force use existing map plugin")
    parser.add_argument("--force", action="store_true", help="Allow overwrite when creating new map plugin")
    parser.add_argument("--open", action="store_true", help="Try opening browser automatically")
    parser.add_argument("--no-open", action="store_true", help="Legacy alias to disable browser opening")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    base_dir = Path(__file__).resolve().parent
    step_dir = base_dir / "src"

    step_files = find_step_files(step_dir)
    if not step_files:
        raise SystemExit(f"找不到 STEP 檔案，請放到：{step_dir}")

    if args.step_file:
        selected_step = Path(args.step_file).expanduser().resolve()
        if selected_step not in step_files and not selected_step.exists():
            raise SystemExit(f"STEP 檔不存在：{selected_step}")
    else:
        selected_step = select_step_interactive(base_dir, step_files)

    inspect_path = inspect_step(base_dir, selected_step)

    map_name = ""
    plugin_path: Path | None = None
    existing = find_existing_maps(base_dir)

    if args.map_name and (args.new_map or args.existing_map):
        if args.new_map and args.existing_map:
            raise SystemExit("--new-map 與 --existing-map 不能同時使用")
        if args.new_map:
            map_name = args.map_name
            plugin_path = create_or_update_map(base_dir, map_name, allow_overwrite=args.force)
        else:
            matched = [item for item in existing if item.map_name == args.map_name]
            if not matched:
                raise SystemExit(f"找不到既有 map_name={args.map_name}")
            preferred = next((item for item in matched if item.path.parent.name == "plugins"), matched[0])
            map_name = preferred.map_name
            plugin_path = preferred.path
    elif args.map_name and not args.new_map and not args.existing_map:
        matched = [item for item in existing if item.map_name == args.map_name]
        if matched:
            preferred = next((item for item in matched if item.path.parent.name == "plugins"), matched[0])
            map_name = preferred.map_name
            plugin_path = preferred.path
        else:
            map_name = args.map_name
            plugin_path = create_or_update_map(base_dir, map_name, allow_overwrite=args.force)
    else:
        map_name, plugin_path = choose_map_interactive(base_dir)

    assert plugin_path is not None
    image_path: Path | None = None
    if args.image_file:
        image_candidate = Path(args.image_file).expanduser().resolve()
        if not image_candidate.exists():
            raise SystemExit(f"image file not found: {image_candidate}")
        image_path = image_candidate
    marker_url = build_marker_url(base_dir, map_name, plugin_path=plugin_path, image_path=image_path)

    open_requested = args.open and not args.no_open
    opened = open_browser(marker_url) if open_requested else False

    print("\n=== Map Pointer Session ===")
    print(f"STEP: {selected_step}")
    if inspect_path is not None:
        print(f"STEP inspect: {inspect_path}")
    print(f"Map plugin: {plugin_path}")
    if image_path is not None:
        print(f"Basemap PNG: {image_path}")
    print(f"Marker URL: {marker_url}")
    if open_requested:
        print("Browser:", "opened" if opened else "failed (請手動開啟 URL)")
    else:
        print("Browser: skipped (use --open if desktop browser is available)")
    print("提示：URL 已嘗試自動帶入插件點位；若失敗可在 HTML 內用『從 JSON 檔載入』手動選檔。")


if __name__ == "__main__":
    main()
