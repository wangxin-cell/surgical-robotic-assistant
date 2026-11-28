#!/bin/bash
set -euo pipefail
umask 077
release="$(dirname "$(readlink -f "$0")")"
export O6_WORKSTATION_RUNTIME="$HOME/o6_workstation/runtime"
export PYTHONPATH="$release/deps" PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=2
mkdir -p "$O6_WORKSTATION_RUNTIME"
py="$HOME/instrument_vision/.venv/bin/python"
cd "$release/app"
if "$py" -c 'import json,urllib.request,pathlib;assert json.load(urllib.request.urlopen("http://127.0.0.1:8876/api/identity", timeout=1)).get("release") == pathlib.Path.cwd().parent.name' 2>/dev/null; then
  echo "O6 service already running at http://127.0.0.1:8876/"
  exit 0
fi
if [ "${1:-}" = "--background" ]; then
  nohup "$py" -u server.py --port 8876 </dev/null >>"$O6_WORKSTATION_RUNTIME/server.log" 2>&1 &
  pid=$!
  for n in $(seq 1 75); do
    if ! kill -0 "$pid" 2>/dev/null; then tail -n 20 "$O6_WORKSTATION_RUNTIME/server.log"; exit 1; fi
    if "$py" -c 'import json,urllib.request,pathlib;assert json.load(urllib.request.urlopen("http://127.0.0.1:8876/api/identity", timeout=1)).get("release") == pathlib.Path.cwd().parent.name' 2>/dev/null; then
      echo "O6 service listening on http://127.0.0.1:8876/"
      exit 0
    fi
    sleep 1
  done
  tail -n 20 "$O6_WORKSTATION_RUNTIME/server.log"
  exit 1
fi
exec "$py" -u server.py --port 8876
