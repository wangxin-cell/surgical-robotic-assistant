#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
python3.11 -c 'import platform, sys; sys.exit(0 if platform.system() == "Linux" and platform.machine() == "aarch64" else "Expected Linux aarch64")'
python3.11 -m venv .venv
py=.venv/bin/python
if [[ -n "${WHEELHOUSE:-}" ]]; then
  "$py" -m pip install --no-index --find-links "$WHEELHOUSE" -r requirements-o6.txt
else
  "$py" -m pip install --index-url https://download.pytorch.org/whl/cpu 'torch==2.10.0+cpu' 'torchvision==0.25.0+cpu'
  "$py" -m pip install -r requirements-o6.txt
fi
"$py" -c 'import cv2, torch, torchvision, ultralytics; print("O6 vision imports OK")'
"$py" table_plane.py example-plane.json 160 120
