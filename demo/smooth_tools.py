"""Local motion telemetry and an encoder-only check of a stationary, held pose."""
import json
import time
from datetime import datetime, timezone
from pathlib import Path


class HoldCheck:
    seconds = 5.0
    max_span = 8
    max_error = 12

    def __init__(self):
        self.started = self.last = None
        self.samples = 0
        self.axes = {}

    def add(self, snapshot, now=None):
        now = time.monotonic() if now is None else now
        if snapshot["state"] != "holding":
            raise ValueError("保持状态已改变，核验已中断。")
        if self.last is not None and not 0 < now - self.last <= 1.5:
            raise ValueError("两次反馈间隔异常，无法核验连续保持。")
        active = {i: row for i, row in snapshot["rows"].items()
                  if row["torque"] == 1 and row["held"] and row["ready"]}
        if not {2, 3, 4, 5}.issubset(active):
            raise ValueError("肩、肘和腕的持续保持尚未全部确认。")
        if self.started is not None and set(active) != set(self.axes):
            raise ValueError("持位关节发生变化，核验已中断。")
        for i, row in active.items():
            pos, goal = row["position"], row["target"]
            if i not in self.axes:
                self.axes[i] = dict(low=pos, high=pos, peak_error=abs(goal-pos), target=goal)
            axis = self.axes[i]
            if goal != axis["target"]:
                raise ValueError("关节目标发生变化，核验已中断。")
            axis["low"] = min(axis["low"], pos)
            axis["high"] = max(axis["high"], pos)
            axis["peak_error"] = max(axis["peak_error"], abs(goal-pos))
        if self.started is None:
            self.started = now
        self.last = now
        self.samples += 1
        return now - self.started >= self.seconds

    def result(self):
        duration = 0 if not self.samples else self.last - self.started
        axes = {i: {"span": a["high"] - a["low"], "peak_error": a["peak_error"]}
                for i, a in self.axes.items()}
        return {"duration": duration, "samples": self.samples, "axes": axes,
                "max_accepted_span": self.max_span, "max_accepted_error": self.max_error,
                "passed": duration >= self.seconds and self.samples >= 5 and all(
                    a["span"] <= self.max_span and a["peak_error"] <= self.max_error for a in axes.values()
                )}


class TracedSmoothBus:
    """Keep real bench runs in separate JSONL files; simulated runs stay in memory."""
    def __init__(self, simulate=False, bus_class=None):
        from smooth_bus import SmoothBus
        self.bus = (bus_class or SmoothBus)(simulate=simulate)
        self.simulate = simulate
        self.path = None
        self.log_error = ""
        self.last_poll = 0.
        self.last_state = None

    def _write(self, event, **details):
        if self.path is None or self.log_error:
            return
        try:
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({"time": datetime.now(timezone.utc).isoformat(),
                                         "event": event, **details}, ensure_ascii=False) + "\n")
        except OSError as exc:
            self.log_error = str(exc)

    def _decorate(self, snapshot):
        return {**snapshot, "observed_at": time.monotonic(),
                "diagnostic_file": str(self.path or ""), "diagnostic_error": self.log_error}

    @property
    def last_status(self):
        return self.bus.last_status

    @property
    def state(self):
        return self.bus.state

    @property
    def reason(self):
        return self.bus.reason

    def record_hold_result(self, result):
        self._write("stationary_encoder_check", result=result)

    def _call(self, name, func, *args, **kwargs):
        try:
            state = func(*args, **kwargs)
        except Exception as exc:
            self._write(name + "_error", error=str(exc))
            raise
        self._write(name, snapshot=state)
        return self._decorate(state)

    def connect(self):
        if not self.simulate:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            self.path = Path(__file__).resolve().parent / "results" / ("smooth_right_" + stamp + ".jsonl")
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
            except OSError as exc:
                self.log_error = str(exc)
        return self._call("connect", self.bus.connect)

    def status(self):
        try:
            snap = self.bus.status()
        except Exception as exc:
            if self.last_state != "unavailable":
                self._write("status_error", error=str(exc))
            self.last_state = "unavailable"
            raise
        now, state = time.monotonic(), snap["state"]
        if state != self.last_state or (state == "holding" and now - self.last_poll >= 5):
            self._write("status", snapshot=snap)
            self.last_state, self.last_poll = state, now
        return self._decorate(snap)

    def enable_hold(self, stop_event=None):
        self._write("request_hold")
        return self._call("enable_hold", self.bus.enable_hold, stop_event=stop_event)

    def move_to(self, targets, speed, stop_event, on_progress=None):
        self._write("request_move", targets=targets, speed_deg_s=speed)
        last = 0.

        def progress(snap):
            nonlocal last
            now = time.monotonic()
            if now - last >= .5:
                self._write("trajectory_sample", snapshot=snap)
                last = now
            if on_progress is not None:
                on_progress(self._decorate(snap))

        return self._call("move_result", self.bus.move_to, targets, speed, stop_event, progress)

    def release(self):
        self._write("request_supported_release")
        return self._call("release", self.bus.release)

    def stop_hold(self):
        self._write("request_stop_at_measured_pose")
        return self._call("stop_hold", self.bus.stop_hold)

    def close(self):
        try:
            self.bus.close()
        except Exception as exc:
            self._write("close_error", error=str(exc))
            raise
        self._write("close")

    def abandon_supported(self):
        self.bus.abandon_supported()
        self._write("close_after_operator_confirmed_power_off")
