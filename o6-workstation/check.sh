#!/bin/bash
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/app"
exec "$HOME/instrument_vision/.venv/bin/python" -B -u board_check.py
