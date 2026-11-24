"""Persistent right-arm position control. Public methods run on one I/O worker."""
import math
import time

from jog_bus import BUS_IDS, JOINT_NAMES, MARGIN, PROFILE, RAM_WIDTH, JogBus, _word


TICKS_PER_DEG = 4096 / 360
CONTROL_PERIOD = .040
SETTLE_TOLERANCE = 8
HOLD_ERROR = 18
MAX_FOLLOW_ERROR = 40
FRAME_ADVANCE = 16
# Commissioning ceilings, in the servo RAM limit's native units. Never raise
# a lower setting on this robot, never increase the limit in response to sag.
EFFORT_CAP = {i: (100 if i == 6 else 350) for i in JOINT_NAMES}


class SmoothBus(JogBus):
    passive_drift_limit = 10

    def __init__(self, simulate=False):
        super().__init__(simulate)
        self.state = "passive"
        self.reason = ""
        self.last_status = {}
        self._original = {}  # Any touched axis is owned until verified released/restored.
        self._settings = {}
        self._goals = {}
        self._passive_positions = {}
        self._lag_since = {}

    def connect(self):
        super().connect()
        try:
            return self.status()
        except Exception:
            self.close()
            raise

    def _feedback(self, motor):
        b = self._link.read(motor, 40, 27)
        return dict(torque=b[0], accel=b[1], goal=_word(b, 2), speed=_word(b, 6),
                    effort=_word(b, 8), position=_word(b, 16), load_raw=_word(b, 20),
                    voltage=b[22], temp=b[23], moving=b[26])

    @staticmethod
    def _health(s):
        if not 90 <= s["voltage"] <= 140:
            return "电机电压超出调试范围"
        if s["temp"] >= 60:
            return "电机温度已到调试上限"
        return ""

    def _passive_reason(self, i, states):
        s = states[i]
        lo, hi, _ = PROFILE[i]
        if self._identity_issues[i] or self._health(s):
            return self._identity_issues[i] or self._health(s)
        if any(v["torque"] != 0 for v in states.values()):
            return "总线上存在非本次使能的电机"
        if not 0 < s["effort"] <= 1000:
            return "RAM 输出限值异常"
        if not lo <= s["position"] <= hi:
            return f"当前读数超出硬限 {lo}–{hi}，此轴仅可读"
        return ""

    def _pack(self, states):
        rows = {}
        for i in JOINT_NAMES:
            s = states[i]
            if self.state == "passive":
                issue = self._passive_reason(i, states)
            elif self.state == "fault":
                issue = self.reason
            else:
                issue = "" if i in self._goals else "此轴未使能，保持只读"
            lo, hi, _ = PROFILE[i]
            # Inside the margin the only permitted motion is toward the interior.
            rows[i] = dict(position=s["position"], target=self._goals.get(i, s["position"]),
                           minimum=max(lo, min(lo + MARGIN, s["position"])),
                           maximum=min(hi, max(hi - MARGIN, s["position"])),
                           torque=s["torque"], held=i in self._goals and s["torque"] == 1,
                           ready=not issue, reason=issue, voltage_v=s["voltage"]/10,
                           temperature_c=s["temp"], load_raw=s["load_raw"], moving=s["moving"])
        self.last_status = dict(state=self.state, reason=self.reason, rows=rows, port=self.port_name)
        return self.last_status

    def _faulted(self, exc):
        self.state = "fault"
        self.reason = f"{exc}。请稳固承托机械臂，必要时切断右侧电机电源。"
        if self.last_status:
            rows = {i: dict(s, ready=False, reason=self.reason) for i, s in self.last_status["rows"].items()}
            self.last_status = dict(self.last_status, state=self.state, reason=self.reason, rows=rows)
        return RuntimeError(self.reason)

    def _verify_enabled(self, i, s):
        cfg = self._settings[i]
        detail = self._health(s)
        if detail:
            raise RuntimeError(f"ID{i} {detail}")
        if (s["torque"] != 1 or s["goal"] != self._goals[i] or s["effort"] != cfg[48]
                or s["accel"] != cfg[41] or s["speed"] != cfg[46]):
            raise RuntimeError(f"ID{i} 的使能、目标或限速限力参数回读不符")
        if not PROFILE[i][0] <= s["position"] <= PROFILE[i][1]:
            raise RuntimeError(f"ID{i} 实测已越过硬件限位")

    def _verify(self, states):
        for i in self._goals:
            self._verify_enabled(i, states[i])
            err = abs(states[i]["position"] - self._goals[i])
            if err > MAX_FOLLOW_ERROR:
                raise RuntimeError(f"ID{i} 跟随误差 {err} 格")
            if self.state == "holding" and err > HOLD_ERROR:
                since = self._lag_since.setdefault(i, time.monotonic())
                if time.monotonic() - since > .6:
                    raise RuntimeError(f"ID{i} 不能稳定保持，偏差 {err} 格")
            else:
                self._lag_since.pop(i, None)
        for i, start in self._passive_positions.items():
            s = states[i]
            delta = (s["position"] - start + 2048) % 4096 - 2048
            if s["torque"] or abs(delta) > self.passive_drift_limit:
                raise RuntimeError(f"未接管的 ID{i} 出现使能或位移")

    def status(self):
        try:
            states = self._all()
        except Exception as exc:
            if self._original:
                raise self._faulted(exc) from exc
            raise
        if self.state in ("holding", "moving"):
            try:
                self._verify(states)
            except Exception as exc:
                try:
                    self._freeze()
                except Exception as freeze_error:
                    raise self._faulted(f"{exc}；停止保持未核实：{freeze_error}") from exc
                raise self._faulted(exc) from exc
        return self._pack(states)

    def _verified_ram(self, i, address, value):
        if i not in self._original or address not in RAM_WIDTH:
            raise ValueError("写入不属于本次接管的右臂关节")
        if address == 42 and not PROFILE[i][0] <= value <= PROFILE[i][1]:
            raise ValueError("目标超出右臂硬件限位")
        self._link.write(i, address, value)
        if self._number(i, address, RAM_WIDTH[address]) != value:
            raise OSError(f"ID{i} 寄存器 {address} 回读不符")

    def _hold_axes(self, states):
        axes = []
        for i in JOINT_NAMES:
            issue = self._passive_reason(i, states)
            if issue:
                if i in (1,6) and "超出硬限" in issue:
                    continue
                raise ValueError(f"ID{i} {issue}")
            axes.append(i)
        return axes

    def enable_hold(self, stop_event=None):
        if self.state != "passive" or self._original:
            raise ValueError("请先承托并释放上一次保持。")
        self._identity_issues = {i:self._identity_issue(i) for i in BUS_IDS}
        states = self._all()
        if any(self._identity_issues.values()):
            raise ValueError("总线型号、模式、限位或零位偏置与这台右臂不符。")
        axes = self._hold_axes(states)
        self._passive_positions = {i:s["position"] for i,s in states.items() if i not in axes}
        if stop_event is not None and stop_event.is_set():
            raise InterruptedError("已取消启用保持")
        try:
            # Preload ALL eligible targets with zero available effort. Some firmware
            # implicitly enables torque on every position write, even a same-position write.
            for i in axes:
                if stop_event is not None and stop_event.is_set():
                    raise InterruptedError("启用保持已中止")
                s = self._feedback(i)
                pos = s["position"]
                if (s["torque"] or abs(pos - states[i]["position"]) > 4 or self._health(s)
                        or not PROFILE[i][0] <= pos <= PROFILE[i][1]):
                    raise RuntimeError(f"ID{i} 初始姿态或电机状态已变化")
                eeprom_cap = self._number(i, 16, 2)
                if not 0 < eeprom_cap <= 1000 or not 0 < s["effort"] <= 1000:
                    raise RuntimeError(f"ID{i} 电机输出限值无效")
                self._original[i] = {41:s["accel"], 46:s["speed"], 48:s["effort"]}
                self._settings[i] = {41:min(s["accel"] or 5, 5),
                                     46:min(s["speed"] or 342, 342),
                                     48:min(s["effort"], eeprom_cap, EFFORT_CAP[i])}
                self._verified_ram(i, 48, 0)
                for a in (41,46):
                    self._verified_ram(i, a, self._settings[i][a])
                self._verified_ram(i, 42, pos)
                self._verified_ram(i, 40, 0)
                self._goals[i] = pos
            for i in axes:
                if stop_event is not None and stop_event.is_set():
                    raise InterruptedError("启用保持已中止")
                self._verified_ram(i, 40, 1)
                s = self._feedback(i)
                cfg = self._settings[i]
                if (s["torque"] != 1 or s["goal"] != self._goals[i] or s["effort"] != 0
                        or s["speed"] != cfg[46] or s["accel"] != cfg[41] or self._health(s)
                        or abs(s["position"] - self._goals[i]) > 4):
                    raise RuntimeError(f"ID{i} 零输出预载核对失败")
                self._verified_ram(i, 48, cfg[48])
                self._verify_enabled(i, self._feedback(i))
            self.state, self.reason = "holding", ""
            return self.status()
        except Exception as exc:
            # Some earlier axes may already support weight. Only release after support.
            raise self._faulted(exc) from exc

    def _send(self, changed):
        if not changed:
            return
        if any(i not in self._goals or type(p) is not int or not PROFILE[i][0] <= p <= PROFILE[i][1]
               for i,p in changed.items()):
            raise ValueError("同步目标超出已接管关节范围")
        self._link.write_goals(changed)
        self._goals.update(changed)

    def _freeze(self):
        states = self._all()
        # Never write a goal into a servo that tripped, lost its configuration
        # or powered off: on this firmware that would re-enable it.
        for i in self._goals:
            self._verify_enabled(i, states[i])
        positions = {i:states[i]["position"] for i in self._goals}
        self._send(positions)
        self.state, self.reason = "holding", "轨迹已停止，正保持停止位置。"
        latched = self._all()
        self._verify(latched)
        return self._pack(latched)

    def stop_hold(self):
        """Latch current encoder readings even when the worker was idle in holding."""
        if self.state != "holding":
            raise ValueError("当前状态无法重新锁定实测位置。")
        try:
            return self._freeze()
        except Exception as exc:
            raise self._faulted(exc) from exc

    def move_to(self, targets, speed, stop_event, on_progress=None):
        if self.state != "holding":
            raise ValueError("请先启用姿态保持。")
        if (not isinstance(targets, dict) or not targets
                or any(type(i) is not int or i not in self._goals or type(v) is not int for i,v in targets.items())):
            raise ValueError("目标须是已接管关节的整数刻度。")
        if type(speed) not in (int,float) or not 5 <= speed <= 30:
            raise ValueError("速度可设为每秒 5 至 30 度。")
        snapshot = self.status()
        measured_start = {i:s["position"] for i,s in snapshot["rows"].items()}
        for i,v in targets.items():
            row = snapshot["rows"][i]
            if not row["minimum"] <= v <= row["maximum"]:
                raise ValueError(f"ID{i} 目标应在 {row['minimum']}–{row['maximum']} 格之间")
        start = self._goals.copy()
        delta = {i:v-start[i] for i,v in targets.items() if v != start[i]}
        if not delta:
            return snapshot
        # Quintic smoothstep has peak derivative 1.875. One clock for all six axes.
        duration = max(.4, max(1.875 * abs(d) / min(speed*TICKS_PER_DEG, self._settings[i][46])
                                 for i,d in delta.items()))
        elapsed = 0.
        previous_time = time.monotonic()
        self.state, self.reason = "moving", ""
        settling = None
        quiet_since = None
        last_progress = previous_time
        try:
            while True:
                if stop_event.is_set():
                    return self._freeze()
                states = self._all()
                self._verify(states)
                if stop_event.is_set():
                    return self._freeze()
                now = time.monotonic()
                # A delayed USB read must never cause a jump to the wall-clock target.
                dt = min(now-previous_time, .08, duration*FRAME_ADVANCE/(1.875*max(map(abs,delta.values()))))
                previous_time = now
                if elapsed < duration:
                    # When feedback lags, hold the last setpoint until motion catches up.
                    # Pause all axes together so the path in joint space is preserved.
                    if max(abs(self._goals[i]-states[i]["position"]) for i in self._goals) <= HOLD_ERROR:
                        elapsed = min(duration, elapsed+dt)
                        t = elapsed/duration
                        f = t*t*t*(10+t*(-15+6*t))
                        points = {i:start[i]+round(v*f) for i,v in delta.items()}
                        self._send({i:p for i,p in points.items() if self._goals[i] != p})
                        last_progress = now
                    elif now-last_progress > 1.2:
                        raise RuntimeError("关节跟随停滞，停止轨迹")
                else:
                    if settling is None:
                        settling = now
                    reached = all(
                        abs(states[i]["position"] - goal) <= (SETTLE_TOLERANCE if i in delta else HOLD_ERROR)
                        and not states[i]["moving"] for i, goal in self._goals.items()
                    )
                    if not reached:
                        quiet_since = None
                    elif quiet_since is None:
                        quiet_since = now
                    if quiet_since is not None and now - quiet_since >= .15:
                        unresolved = [i for i in delta
                                      if targets[i] != measured_start[i]
                                      and (targets[i]-measured_start[i])*(states[i]["position"]-measured_start[i]) <= 0]
                        if unresolved:
                            self._freeze()
                            self.reason = "目标在容差内，但 " + "、".join(f"ID{i}" for i in unresolved) + " 未测得向目标移动；已保持实测位置。"
                            return self.status()
                        self.state = "holding"
                        self.reason = "轨迹已结束，继续保持目标。"
                        return self._pack(states)
                    if now-settling > 1.5:
                        raise RuntimeError("终点跟随误差持续过大或关节仍在移动")
                if on_progress:
                    on_progress(self._pack(states))
                stop_event.wait(max(.004, CONTROL_PERIOD-(time.monotonic()-now)))
        except Exception as exc:
            # Try to freeze only a fully verified, responsive set of servos.
            # On a link fault the most recent target remains bounded by the streaming lead.
            try:
                self._freeze()
            except Exception as freeze_error:
                raise self._faulted(f"{exc}；停止保持未核实：{freeze_error}") from exc
            raise self._faulted(exc) from exc

    def release(self):
        failures = []
        for i in list(self._original):
            try:
                try:
                    self._verified_ram(i, 40, 0)
                except OSError:
                    if self._number(i, 40, 1) != 0:
                        raise
                for a in (41,46,48):
                    if self._number(i, 40, 1) != 0:
                        raise RuntimeError("恢复参数时检测到意外使能")
                    self._verified_ram(i, a, self._original[i][a])
                if self._number(i, 40, 1) != 0:
                    raise RuntimeError("卸力回读未确认")
                del self._original[i]
                self._settings.pop(i, None)
                self._goals.pop(i, None)
            except Exception as exc:
                failures.append(f"ID{i}: {exc}")
        if failures:
            raise self._faulted("部分关节释放未核实：" + "，".join(failures))
        self.state, self.reason = "passive", ""
        self._passive_positions.clear()
        self._lag_since.clear()
        return self.status()

    def move(self, *args, **kwargs):
        raise ValueError("姿态保持模式请使用 move_to。")

    def close(self):
        if self._original:
            raise RuntimeError("电机可能仍在保持。请承托并释放后断开。")
        super().close()

    def abandon_supported(self):
        """Operator confirmed mechanical support and physical power removal."""
        if self._link:
            self._link.close()
        self._link, self.port_name = None, ""
        self._original.clear()
        self._goals.clear()
        self._settings.clear()
        self._passive_positions.clear()
        self._lag_since.clear()
        self.state, self.reason, self.last_status = "passive", "", {}
