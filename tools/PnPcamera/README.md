# PnPcamera

Two scripts for camera intrinsic calibration:

- `capture_chessboard.py`: capture chessboard images from ROS2 topic.
- `solve_intrinsics.py`: solve intrinsics and output matrix yaml/snippet.

Default save root:

- `tools/PnPcamera/save/`

Quick start:

```bash
cd /home/unwanhin/ros2_ly_ws_sentary
python3 tools/PnPcamera/capture_chessboard.py
python3 tools/PnPcamera/solve_intrinsics.py
```

You can edit board size in:

- `tools/PnPcamera/solve_intrinsics.py` (top "Quick Settings" lines).
