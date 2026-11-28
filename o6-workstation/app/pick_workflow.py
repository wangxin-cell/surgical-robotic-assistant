"""Bind one fresh visual request to a calibrated, recorded left-arm movement."""
from math import dist
from pathlib import Path
import threading
import time
from uuid import uuid4

from native_pick import ReplayStopped, reset_left_home, prepare_slot_origin
from package_replay import Interrupted, replay as replay_slot
from pick_locations import PickLocations
from native_vision import lookup


ACTIVE = {"homing", "checking", "running", "stopping"}
FIXED_TICKET_SECONDS = 60


class PickWorkflow:
    def __init__(self, calibration_file, capture_lock, *, observe=lookup, replay=replay_slot, home=reset_left_home):
        self.locations = PickLocations(Path(calibration_file))
        self.capture_lock = capture_lock
        self.observe = observe
        self.replay = replay
        self.home = home
        self.lock = threading.RLock()
        self.capture = None
        self.candidate = None
        self.prepared = None
        self.paused = None
        self.stop_event = None
        self.home_ticket = None
        self.fixed_ticket = None
        self.job = {"kind": None, "state": "idle", "message": "", "request_id": None, "slot": None,
                    "target_id": None, "progress": 0.0}

    def status(self):
        with self.lock:
            return dict(self.job, origin_ready=self.prepared is not None and self.candidate is not None)

    def active(self):
        with self.lock:
            return self.job["state"] in ACTIVE

    def invalidate(self):
        with self.lock:
            self.capture = self.candidate = self.fixed_ticket = self.prepared = None

    def discard_pause(self):
        with self.lock:
            self.paused = None
            self.job.pop("resume_token", None)
            self.job.pop("resume_label", None)

    def resume(self, request_id, token):
        with self.lock:
            if (self.active() or not self.paused or not token or token != self.job.get("resume_token")
                    or request_id != self.job["request_id"]):
                raise ValueError("断点已更新，请查看最新回放状态。")
            cursor = self.paused
            kind, target, slot = self.job["kind"], self.job["target_id"], self.job["slot"]
            label = self.job.get("label")
            return self._begin(request_id, kind, target, {"slot":slot,"label":label}, False,
                               direct=True, cursor=cursor)

    def remember(self, request_id, scene, ids):
        with self.lock:
            self.invalidate()
            if scene.get("source") != "live" or scene.get("error") or not scene.get("views", {}).get("head"):
                return {"ready": False, "slot": None, "message": "本次未取得头部实拍。"}
            self.capture = (request_id, scene)
            if len(ids) != 1:
                return {"ready": False, "slot": None, "message": "每次选择一件器具进行夹取。"}
            decision = self.locations.match(scene, ids[0])
            if decision["ready"] and not self.active():
                self.candidate = (request_id, ids[0], decision, False)
            return decision

    def remember_slot(self, request_id, scene, slot):
        with self.lock:
            self.invalidate()
            decision = self.locations.match_slot(scene, slot)
            if decision["ready"] and not self.active():
                self.candidate = (request_id, f"slot_{slot}", decision, True)
            return decision

    def calibration(self):
        with self.lock:
            return self.locations.status()

    def calibrate(self, request_id, points):
        with self.lock:
            if self.active():
                raise ValueError("左臂夹取正在进行，请在结束后设置点位。")
            if not self.capture or self.capture[0] != request_id:
                raise ValueError("标记所用的画面已经更新，请重新拍摄头部画面。")
            saved = self.locations.save(self.capture[1], points)
            self.invalidate()
            return saved

    def start(self, request_id, *, direct=False):
        with self.lock:
            if self.job["kind"] == "pick" and self.job["request_id"] == request_id and request_id:
                return dict(self.job)  # A retry must never repeat a physical action.
            if self.active():
                raise ValueError("左臂正在执行上一项动作。")
            if not self.candidate or self.candidate[0] != request_id:
                raise ValueError("请先重新识别一件器具并取得它对应的点位。")
            _, target, decision, by_slot = self.candidate
            return self._begin(request_id, "pick", target, decision, by_slot, direct=direct)

    def home_request(self):
        with self.lock:
            if self.home_ticket is None:
                self.home_ticket = str(uuid4())
            return self.home_ticket

    def prepare_origin(self, request_id):
        with self.lock:
            if self.job["kind"]=="origin" and self.job["request_id"]==request_id and request_id:
                return dict(self.job)
            if self.active() or not self.candidate or self.candidate[0]!=request_id:
                raise ValueError("请先识别器具并确认唯一点位。")
            _,target,decision,by_slot=self.candidate
            return self._begin(request_id,"origin",target,decision,by_slot)

    def fixed_request(self, slot):
        with self.lock:
            if type(slot) is not int or slot not in range(1, 5):
                raise ValueError("请选择 1 至 4 号点位。")
            if self.active():
                raise ValueError("左臂正在执行上一项动作。")
            self.invalidate()
            ticket = str(uuid4())
            self.fixed_ticket = (ticket, slot, time.monotonic() + FIXED_TICKET_SECONDS)
            return {"request_id": ticket, "slot": slot}

    def start_fixed(self, request_id, slot, confirmed):
        with self.lock:
            if confirmed is not True or type(slot) is not int or slot not in range(1, 5):
                raise ValueError("请确认示教点位、无刃模具和运动路径。")
            if (request_id and self.job["kind"] == "fixed" and self.job["slot"] == slot
                    and self.job["request_id"] == request_id):
                return dict(self.job)
            if self.active():
                raise ValueError("左臂正在执行上一项动作。")
            if (not self.fixed_ticket or self.fixed_ticket[:2] != (request_id, slot)
                    or time.monotonic() >= self.fixed_ticket[2]):
                raise ValueError("本次点位确认已失效，请再次选择并确认点位。")
            return self._begin(request_id, "fixed", None, {"slot": slot, "label": "手动回放"}, False)

    def start_home(self, request_id):
        with self.lock:
            if request_id and self.job["kind"] == "home" and self.job["request_id"] == request_id:
                return dict(self.job)
            if self.active():
                raise ValueError("左臂正在执行上一项动作。")
            if not request_id or request_id != self.home_ticket:
                raise ValueError("复位请求已失效，请重新确认本次复位。")
            return self._begin(request_id, "home", None, None, False)

    def _begin(self, request_id, kind, target, decision, by_slot, *, direct=False, cursor=None):
        self.discard_pause()
        # The caller holds self.lock throughout arbitration and worker startup.
        if kind=="origin":
            self.capture = self.fixed_ticket = self.prepared = None
        else:
            self.invalidate()
        self.home_ticket = None
        stop = self.stop_event = threading.Event()
        self.job = {"kind": kind, "state": "homing" if kind in ("home", "origin") else "checking", "request_id": request_id, "target_id": target,
                    "slot": decision["slot"] if decision else None, "progress": 0.0,
                    "label": decision.get("label") if decision else None,
                    "message": ("正在检查六轴，准备执行左臂公共复位。" if kind == "home"
                                else f"正在对齐点位 {decision['slot']} 的示教起姿。" if kind == "origin"
                                else f"正在只读核对点位 {decision['slot']} 的路径起姿。")}
        try:
            threading.Thread(target=self._run, args=(target, decision, stop, by_slot, kind, direct, cursor),
                             daemon=True, name="left-" + kind).start()
        except RuntimeError as exc:
            self.job.update(state="failed", message=str(exc))
            raise
        return dict(self.job)

    def stop(self):
        with self.lock:
            self.fixed_ticket = None
            if self.active():
                self.stop_event.set()
                self.home_ticket = None
                self.job.update(state="stopping", message="已请求停止，正在等待执行端回报。")
            return dict(self.job)

    def _run(self, target, decision, stop, by_slot, kind, direct, cursor):
        paused = None
        def require_active():
            # A slow camera response must not authorize a stopped or superseded job.
            if self.stop_event is not stop or stop.is_set() or not self.active():
                raise ReplayStopped("已停止本次夹取。")

        def update(message, progress):
            with self.lock:
                if self.stop_event is stop and self.active() and not stop.is_set():
                    self.job.update(message=str(message), progress=max(0.0, min(1.0, float(progress))))
        def verify_target():
            slot = decision["slot"]
            with self.lock:
                require_active()
                if kind == "fixed" or direct:
                    self.job.update(state="running", message=f"正在回放点位 {slot} 的原始路径。")
                    return
                self.job.update(state="checking", message=f"左臂位于点位 {slot} 起姿，正在复拍器具。")
            with self.capture_lock:
                with self.lock:
                    require_active()
                scene = self.observe([] if by_slot else [target])
            with self.lock:
                require_active()
            if scene.get("error") or scene.get("source") != "live":
                raise ValueError(scene.get("error") or "复拍未取得实况画面。")
            matched = (self.locations.match_slot(scene, decision["slot"]) if by_slot
                       else self.locations.match(scene, target))
            if not matched["ready"] or matched["slot"] != decision["slot"]:
                raise ValueError("复拍未确认原点位。"+matched["message"])
            if by_slot and matched.get("class_name") != decision.get("class_name"):
                raise ValueError("点位上的器具外形在两次拍摄间发生变化，请重新查看点位。")
            if not decision.get("center") or not matched.get("center"):
                raise ValueError("复拍缺少器具中心位置。")
            calibration = self.locations.status()
            anchors = calibration["points"]
            width, height = calibration["image_size"]
            def pixel(point):
                return point[0] * width, point[1] * height
            anchor = pixel(anchors[slot - 1])
            spacing = min(dist(anchor, pixel(p)) for i, p in enumerate(anchors, 1) if i != slot)
            if dist(pixel(decision["center"]), pixel(matched["center"])) > spacing * 0.12:
                raise ValueError("两次拍摄间器具位置发生变化，请重新摆放并识别。")
            with self.lock:
                require_active()
                self.job.update(state="running", message=f"已复拍确认点位 {slot}，准备回放夹取轨迹。")

        try:
            if kind=="origin":
                refs = prepare_slot_origin(decision["slot"],stop,update)
                outcome = f"点位 {decision['slot']} 起姿已停稳，可点击夹取。"
            elif decision is None:
                outcome = self.home(stop, update) or "左臂公共复位已完成。"
            else:
                args = (decision["slot"], stop, update, verify_target)
                outcome = self.replay(*args, cursor=cursor) if cursor else self.replay(*args)
                outcome = outcome or "点位轨迹已执行完毕，请核对实物夹取结果。"
            result = dict(state="completed", progress=1.0, message=outcome)
        except Interrupted as exc:
            paused = exc.cursor
            label = (f"继续点位 {paused.slot} · 第 {paused.done + 1}/{paused.total} 步"
                     if paused.done < paused.total else f"点位 {paused.slot} · 重新核对终点")
            result = dict(state="stopped" if stop.is_set() else "failed",
                          message=f"已确认 {paused.done}/{paused.total} 条。{exc}",
                          progress=paused.done / paused.total, resume_label=label, resume_token=str(uuid4()))
        except ReplayStopped as exc:
            result = dict(state="stopped", message=str(exc) or "左臂动作已停止。")
        except Exception as exc:
            result = dict(state="failed", message=str(exc) or "左臂动作执行失败。")
        with self.lock:
            if kind=="origin":
                if result["state"]=="completed" and not stop.is_set() and self.candidate is not None:
                    self.prepared = refs
                else:
                    self.invalidate()
                    if result["state"]=="completed":
                        result=dict(state="stopped",message="起姿请求已停止或被更换，请重新查询器具。")
            self.home_ticket = None
            self.paused = paused
            self.job.update(result)
