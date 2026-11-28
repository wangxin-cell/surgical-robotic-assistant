"""Report actual board evidence without enabling motors or submitting a pickup."""
import json
import urllib.error
import urllib.request
from pathlib import Path
import os

url = 'http://127.0.0.1:8876'
def api(path, body=None, mime="application/json"):
    data = body if isinstance(body, bytes) else json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url+path, data=data, headers={'Content-Type':mime})
    try:
        with urllib.request.urlopen(req,timeout=150) as res:
            return json.load(res)
    except urllib.error.HTTPError as e:
        return {"error":e.read().decode("utf-8","replace")}

report = {"service":api("/api/identity"),"health":api("/api/status"),"positions":api("/api/pick/origins"),
          "calibrated":api("/api/pick/calibration").get("configured")}
state = api("/api/pick/status")
if state.get("state") in ("idle","completed","failed","stopped"):
    for arm in ("left","right"):
        # The diagnostic never takes over an existing manual teaching session.
        previous = api("/api/arm/status?arm="+arm)
        if previous.get("session") == "open":
            report[arm] = previous
            continue
        result = api("/api/arm/open",{"arm":arm,"mode":"hardware"})
        report[arm] = result
        if result.get("session") == "open":
            api("/api/arm/action",{"arm":arm,"mode":"hardware","operation":"close"})
    scene = api("/api/lookup",{"ids":[]})
    report["capture"] = {k:scene.get(k) for k in ("source","capture_id","views","error")}
fixture = Path(__file__).parent / 'seed' / 'mandarin-demo.wav'
if fixture.is_file() and state.get("state")=="idle":
    report["voice_sample"] = api("/api/voice",fixture.read_bytes(),"audio/wav")
root = Path(os.environ.get("O6_WORKSTATION_RUNTIME",Path.home()/"o6_workstation/runtime"))
(root/"installation-check.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
print(json.dumps(report, ensure_ascii=False),flush=True)
