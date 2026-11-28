"""Local HTTP interface for the O6 voice and vision workstation."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import secrets
import os
import threading
import time
from urllib.parse import parse_qs, urlsplit
from uuid import uuid4

HERE = Path(__file__).resolve().parent
RUNTIME = Path(os.environ.get("O6_WORKSTATION_RUNTIME", Path.home() / "o6_workstation/runtime"))
STATIC = {"index.html", "style.css", "app.js", "voice-input.js", "favicon.svg", "arm-ui.js", "arms.css", "pick-ui.js"}
from native_motor import ArmSessions, HOME, SERIALS
VERSION = 19


def valid_arm(arm):
    if arm not in SERIALS:
        raise ValueError("请选择左臂或右臂。")
    return arm

from arm_episodes import ArmEpisodes
from native_voice import voice_health, recognize_wav
from native_vision import CAPTURES, TARGETS as VISION_TARGETS, camera_state, last_snapshot, lookup
from action_receipts import ReceiptTracker, ReceiptError
from pick_workflow import ACTIVE, PickWorkflow
from native_pick import HOME_MAX_ATTEMPTS, HOME_TOLERANCE, recorded_origins
TARGETS = {key: value[0] for key, value in VISION_TARGETS.items()}


class Problem(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def media_url(filename, folder):
    if not filename or not folder:
        return None
    directory = Path(folder).resolve()
    path = Path(filename).resolve()
    if directory.parent != CAPTURES.resolve() or path.parent != directory:
        return None
    if not re.fullmatch(r"camera_triplet_\d{8}_\d{6}_\d{6}", directory.name):
        return None
    if not re.fullmatch(r"camera_[1-3]\.jpg|console_head(?:_[0-9a-f]+)?\.jpg", path.name):
        return None
    return f"/frames/{directory.name}/{path.name}"


def public_scene(scene):
    if not scene:
        return None
    folder = scene.get("folder")
    output = {k: scene.get(k) for k in ("source", "captured_at", "results", "objects", "error", "exposure")}
    output["image"] = media_url(scene.get("image"), folder)
    output["views"] = {key: media_url(value, folder) for key, value in (scene.get("views") or {}).items()}
    output["capture_id"] = Path(folder).name if folder else None
    return output


class Desk:
    def __init__(self):
        self.capture_lock = threading.Lock()
        self.receipt_lock = threading.Lock()
        self.receipts = ReceiptTracker()
        self.generation = 0
        self.activity = time.monotonic()
        self.key = secrets.token_hex(32)
        runtime = RUNTIME
        runtime.mkdir(parents=True, exist_ok=True)
        self.arms = ArmSessions()
        self.episodes = ArmEpisodes(runtime)
        self.motion_lock = threading.Lock()
        self.picks = PickWorkflow(runtime / "left-pick-locations.json", self.capture_lock)
        self.last_request = None
        self.last_voice_result = None

    def invalidate(self, *, for_inspection=False):
        with self.receipt_lock:
            if for_inspection:
                job = self.picks.status()
                if job["state"] in ACTIVE:
                    raise Problem(409, "左臂动作正在进行，结束后可重新查看器具。")
            self.generation += 1
            self.receipts.invalidate()
            self.last_request = None
            self.picks.invalidate()
            return self.generation

    def start_home(self, request_id):
        # Use the same lock order as inspect() so a photograph from before a
        # reset cannot make a subsequent pick ready after the arm has moved.
        with self.receipt_lock:
            previous = self.picks.status()
            job = self.picks.start_home(request_id)
            if previous.get("kind") != "home" or previous["request_id"] != request_id:
                self.generation += 1
                self.receipts.invalidate()
                self.last_request = None
            return job

    def start_fixed(self, request_id, slot):
        with self.receipt_lock:
            previous = self.picks.status()
            job = self.picks.start_fixed(request_id, slot, True)
            if previous.get("kind") != "fixed" or previous.get("request_id") != request_id:
                self.generation += 1
                self.receipts.invalidate()
                self.last_request = None
            return job

    def preview_fixed(self, slot):
        with self.receipt_lock:
            ticket = self.picks.fixed_request(slot)
            self.generation += 1
            self.receipts.invalidate()
            self.last_request = None
            return ticket

    def inspect(self, ids, *, slot=None):
        if not isinstance(ids, list) or len(ids) > len(TARGETS) or any(
                not isinstance(item, str) or item not in TARGETS for item in ids) or len(set(ids)) != len(ids):
            raise Problem(400, "器具清单无效")
        if not self.capture_lock.acquire(blocking=False):
            raise Problem(409, "相机正在处理另一项查询，请稍后重试。")
        try:
            request_id = str(uuid4())
            generation = self.invalidate(for_inspection=True)
            raw_scene = lookup(ids)
            scene = public_scene(raw_scene)
            if not scene:
                raise Problem(503, "本次未取得相机画面，请检查 O6 相机会话。")
            eligible = [r["id"] for r in scene["results"] or [] if r["status"] == "found" and r.get("unique")]
            with self.receipt_lock:
                if generation == self.generation:
                    if slot is None:
                        scene["pick"] = self.picks.remember(request_id, raw_scene, ids)
                    else:
                        pick = self.picks.remember_slot(request_id, raw_scene, slot)
                        scene["pick"] = pick
                        scene["results"] = [{"id": f"slot_{slot}", "label": pick.get("label") or "未确认器具",
                                             "status": "found" if pick["ready"] else "uncertain",
                                             "count": int(pick["ready"]), "unique": pick["ready"],
                                             "numbers": [pick["number"]] if pick.get("number") else [],
                                             "message": pick["message"]}]
                if (generation == self.generation and scene["source"] == "live"
                        and len(ids) == 1 and eligible == ids):
                    self.receipts.arm(request_id, ids[0], "found")
                    self.last_request = {"request_id": request_id, "target_id": ids[0],
                                         "purpose": "correlate_feedback", "motion_authorized": False}
            scene["request_id"] = request_id
            return scene
        finally:
            self.capture_lock.release()

    def receipt(self, request_id):
        with self.receipt_lock:
            record = self.receipts.current()
            return record if record and record.get("request_id") == request_id else None


class Handler(BaseHTTPRequestHandler):
    server_version = "O6Desk"
    def log_message(self, fmt, *args):
        if len(args) > 1 and str(args[1]).startswith(("4", "5")):
            super().log_message(fmt, *args)

    @property
    def desk(self):
        return self.server.desk

    def send_data(self, data, mime, code=200):
        self.send_response(code)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Permissions-Policy", "microphone=(self), camera=()")
        self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; frame-ancestors 'none'; base-uri 'none'")
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, obj, code=200):
        self.send_data(json.dumps(obj, ensure_ascii=False).encode(), "application/json; charset=utf-8", code)

    def validate_host(self):
        names = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
        if self.headers.get("Host") not in names:
            raise Problem(403, "只接受本机窗口请求")
        origin = self.headers.get("Origin")
        if origin and origin not in {"http://" + n for n in names}:
            raise Problem(403, "请求来源不匹配")
        self.desk.activity = time.monotonic()

    def read_body(self, max_bytes):
        try:
            size = int(self.headers.get("Content-Length", -1))
        except ValueError:
            size = -1
        if size < 0 or size > max_bytes:
            raise Problem(413, "请求大小无效")
        return self.rfile.read(size)

    def route(self, post):
        self.validate_host()
        u = urlsplit(self.path)
        path = u.path
        if path.startswith("/controller/") and not secrets.compare_digest(
                self.headers.get("X-O6-Controller", ""), self.desk.key):
            raise Problem(403, "回执需要本机控制程序凭据")
        if not post:
            if path == "/api/identity":
                return self.send_json({"app": "o6-instrument-desk", "version": VERSION, "host": "o6", "release": HERE.parent.name})
            if path == "/api/pick/origins":
                return self.send_json(recorded_origins())
            if path == "/api/pick/home":
                return self.send_json({"targets": HOME, "tolerance": HOME_TOLERANCE,
                                       "max_attempts": HOME_MAX_ATTEMPTS,
                                       "request_id": self.desk.picks.home_request()})
            if path == "/api/pick/calibration":
                return self.send_json(self.desk.picks.calibration())
            if path == "/api/pick/status":
                return self.send_json(self.desk.picks.status())
            if path in ("/api/arm/status", "/api/arm/episodes", "/api/arm/episode"):
                query = parse_qs(u.query)
                arm = valid_arm(query.get("arm", [""])[0])
                if path == "/api/arm/status":
                    return self.send_json(self.desk.arms.status(arm))
                if path == "/api/arm/episodes":
                    return self.send_json({"episodes": self.desk.episodes.list(arm)})
                try:
                    detail = self.desk.episodes.detail(arm, query.get("id", [""])[0])
                except (FileNotFoundError, KeyError):
                    raise Problem(404, "未找到该臂的示教记录。")
                return self.send_json(detail)
            if path == "/api/status":
                return self.send_json({"camera": camera_state(), "voice": voice_health(),
                                       "last_voice": self.desk.last_voice_result})
            if path == "/api/initial":
                return self.send_json({"targets": TARGETS, "scene": public_scene(last_snapshot())})
            if path == "/api/receipt":
                return self.send_json({"receipt": self.desk.receipt(parse_qs(u.query).get("id", [""])[0])})
            if path == "/controller/current":
                with self.desk.receipt_lock:
                    request = self.desk.last_request
                    record = self.desk.receipts.current()
                    if not record or not request or record["states"].get(request["target_id"]) != "awaiting_controller":
                        request = None
                return self.send_json({"request": request})
            match = re.fullmatch(r"/frames/(camera_triplet_\d{8}_\d{6}_\d{6})/(camera_[1-3]\.jpg|console_head(?:_[0-9a-f]+)?\.jpg)", path)
            if match:
                photo = CAPTURES / match[1] / match[2]
                if photo.is_file() and photo.resolve().parent.parent == CAPTURES.resolve():
                    return self.send_data(photo.read_bytes(), "image/jpeg")
            name = "index.html" if path == "/" else path.removeprefix("/")
            if name in STATIC and (HERE/name).is_file():
                mime = {"html":"text/html", "css":"text/css", "js":"text/javascript", "svg":"image/svg+xml"}[name.rsplit(".",1)[1]]
                return self.send_data((HERE/name).read_bytes(), mime+"; charset=utf-8")
        elif path == "/api/voice":
            if self.headers.get_content_type() != "audio/wav":
                raise Problem(415, "请使用 WAV 语音")
            self.desk.invalidate()
            result = recognize_wav(self.read_body(3_100_000))
            self.desk.last_voice_result = {
                "time": time.time(), "status": result["status"],
                "targets": [item["id"] for item in result.get("matches", []) if item.get("id") in TARGETS],
            }
            if result["status"] == "no_match":
                self.desk.last_voice_result["reason"] = result.get("input_status") or (
                    "unmatched_text" if result.get("text") else "no_transcript"
                )
            return self.send_json(result)
        elif path in ("/api/lookup", "/api/connect-camera", "/controller/receipt", "/api/arm/open", "/api/arm/action",
                      "/api/pick/calibration", "/api/pick/start", "/api/pick/stop", "/api/pick/slot", "/api/pick/home",
                      "/api/pick/fixed/preview", "/api/pick/fixed/start", "/api/pick/origin", "/api/pick/resume"):
            if self.headers.get_content_type() != "application/json":
                raise Problem(415, "请求格式不匹配")
            obj = json.loads(self.read_body(8192))
            if not isinstance(obj, dict):
                raise Problem(400, "无效请求")
            if path == "/api/pick/calibration":
                return self.send_json(self.desk.picks.calibrate(obj.get("request_id"), obj.get("points")))
            if path == "/api/pick/slot":
                slot = obj.get("slot")
                if type(slot) is not int or slot not in range(1, 5):
                    raise Problem(400, "请选择 1 至 4 号点位。")
                return self.send_json(self.desk.inspect([], slot=slot))
            if path == "/api/pick/start":
                with self.desk.motion_lock:
                    return self.send_json(self.desk.picks.start(obj.get("request_id"), direct=obj.get("direct") is True))
            if path == "/api/pick/resume":
                with self.desk.motion_lock:
                    return self.send_json(self.desk.picks.resume(obj.get("request_id"), obj.get("token")))
            if path == "/api/pick/origin":
                if set(obj)!={"request_id","confirmed"} or obj["confirmed"] is not True:
                    raise Problem(400,"请在工作台确认仅移动到已识别点位的起姿。")
                with self.desk.motion_lock:
                    return self.send_json(self.desk.picks.prepare_origin(obj["request_id"]))
            if path == "/api/pick/fixed/preview":
                if set(obj) != {"slot"}:
                    raise Problem(400, "请选择一个已录制点位。")
                return self.send_json(self.desk.preview_fixed(obj["slot"]))
            if path == "/api/pick/fixed/start":
                if (set(obj) != {"slot", "request_id", "confirmed"} or obj.get("confirmed") is not True
                        or not isinstance(obj.get("request_id"), str) or not obj["request_id"]):
                    raise Problem(400, "请在工作台确认本次操作。")
                with self.desk.motion_lock:
                    return self.send_json(self.desk.start_fixed(obj["request_id"], obj["slot"]))
            if path == "/api/pick/home":
                if set(obj) != {"confirmed", "request_id"} or obj["confirmed"] is not True:
                    raise Problem(400, "请在工作台确认左臂仅复位。")
                with self.desk.motion_lock:
                    return self.send_json(self.desk.start_home(obj["request_id"]))
            if path == "/api/pick/stop":
                return self.send_json(self.desk.picks.stop())
            if path in ("/api/arm/open", "/api/arm/action"):
                arm = valid_arm(obj.get("arm"))
                with self.desk.motion_lock:
                    if arm == "left" and self.desk.picks.active():
                        raise Problem(409, "左臂正在执行动作，请等待结束或停止本次动作。")
                    if path == "/api/arm/open":
                        return self.send_json(self.desk.arms.open(arm, obj.get("mode")))
                    if arm == "left" and obj.get("operation") not in ("read", "refresh"):
                        self.desk.picks.discard_pause()
                    return self.send_json(self.desk.arms.action(arm, obj.get("operation"),
                                          joint=obj.get("joint"), delta=obj.get("delta"),
                                          supported=obj.get("supported") is True, mode=obj.get("mode")))
            if path == "/api/connect-camera":
                return self.send_json({"message": "板端相机状态为 "+str(camera_state())+"，点击重新拍摄可取图。"})
            if path == "/controller/receipt":
                with self.desk.receipt_lock:
                    return self.send_json(self.desk.receipts.acknowledge(obj.get("request_id"), obj.get("target_id"), obj.get("event")))
            return self.send_json(self.desk.inspect(obj.get("ids")))
        raise Problem(404, "找不到此资源")

    def handle_route(self, post):
        try:
            self.route(post)
        except (ConnectionResetError, BrokenPipeError):
            pass
        except Problem as exc:
            self.send_json({"message": str(exc)}, exc.code)
        except (ValueError, ReceiptError) as exc:
            self.send_json({"message": str(exc)}, 400)
        except Exception as exc:
            self.send_json({"message": str(exc) or "设备暂不可用，请重试"}, 503)

    def do_GET(self):
        self.handle_route(False)

    def do_POST(self):
        self.handle_route(True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8876)
    args = parser.parse_args()
    with ThreadingHTTPServer(("127.0.0.1", args.port), Handler) as server:
        server.desk = Desk()
        server.timeout = 2
        print(f"器具工作台 http://127.0.0.1:{args.port}/", flush=True)
        server.serve_forever()
