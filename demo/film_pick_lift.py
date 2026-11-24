"""Single taught wrist arc, supported blunt training prop; default is simulation."""
import argparse
import json
import sys
import threading
import time
from collections import deque

from jog_bus import BUS_IDS, JOINT_NAMES, MARGIN, PROFILE
from pinch_taught import motion_stop, no_stop
from smooth_bus import SmoothBus, TICKS_PER_DEG
from smooth_tools import TracedSmoothBus


WRIST, JAW = 4, 6
ACTIVE = (WRIST, JAW)
FIXED = tuple(i for i in BUS_IDS if i not in ACTIVE)
FIXED_DRIFT_LIMIT = 6
WRIST_TRAVEL = (35, 145)
JAW_OPENING = (12, 70)


def fixture_drift(reference, positions, *, from_hold=False):
    displaced = []
    for i in FIXED:
        delta = positions[i] - reference[i]
        if from_hold:
            delta = (delta + 2048) % 4096 - 2048
        if abs(delta) > FIXED_DRIFT_LIMIT:
            displaced.append(f"ID{i} {JOINT_NAMES.get(i, '底盘驱动轮')} {delta:+d} 格")
    if displaced:
        origin = "使能起点" if from_hold else "夹持低位"
        return f"相对{origin}，{'、'.join(displaced)}。固定轴允许 ±{FIXED_DRIFT_LIMIT} 格，请检查机械支撑。"
    return ""


class SupportedWristBus(SmoothBus):
    """Only wrist pitch and jaws can ever receive a write in this fixture."""
    passive_drift_limit = FIXED_DRIFT_LIMIT
    fixture_reference = None

    def bind_fixture(self, contact):
        self.fixture_reference = {i: contact[i] for i in FIXED}

    def _verify(self, states):
        positions = {i: states[i]["position"] for i in FIXED}
        if self.fixture_reference is not None:
            drift = fixture_drift(self.fixture_reference, positions)
            if drift:
                raise RuntimeError(drift)
        if self._passive_positions:
            drift = fixture_drift(self._passive_positions, positions, from_hold=True)
            if drift:
                raise RuntimeError(drift)
        super()._verify(states)

    def _fixture_issue(self, states, *, check_motion=True):
        for i, s in states.items():
            if self._identity_issues[i] or s["torque"] or self._health(s):
                return f"ID{i} 身份、静止状态、电压、温度或卸力反馈异常。"
            if check_motion and s["moving"]:
                return f"ID{i} 仍在运动，请松手等它停稳。"
        for i in ACTIVE:
            issue = self._passive_reason(i, states)
            if issue:
                return f"{JOINT_NAMES[i]}：{issue}"
        return ""

    def _hold_axes(self, states):
        issue = self._fixture_issue(states)
        if issue:
            raise ValueError(issue)
        if self.fixture_reference is None:
            raise ValueError("固定支撑未通过本次示教定位核对。")
        drift = fixture_drift(self.fixture_reference, {i: states[i]["position"] for i in FIXED})
        if drift:
            raise ValueError(drift)
        return ACTIVE

    def _verified_ram(self, i, address, value):
        if i not in ACTIVE:
            raise ValueError(f"夹具模式禁止向 ID{i} 写入。")
        super()._verified_ram(i, address, value)

    def _send(self, changed):
        if set(changed).difference(ACTIVE):
            raise ValueError("夹具模式只允许腕俯仰和夹口目标。")
        super()._send(changed)

    def _pack(self, states):
        s = super()._pack(states)
        s["fixture_positions"] = {i: states[i]["position"] for i in FIXED}
        s["fixture_torques"] = {i: states[i]["torque"] for i in FIXED}
        if self.state == "passive":
            s["fixture_issue"] = self._fixture_issue(states)
            s["teaching_fault"] = self._fixture_issue(states, check_motion=False)
        return s


class DirectPickBus(SupportedWristBus):
    """Keep the jaw preload active while only the wrist moves."""
    jaw_band = None
    retained_goal = None
    contact_pose = None
    start_stop = None

    def bind_fixture(self, contact):
        super().bind_fixture(contact)
        self.contact_pose = dict(contact)

    def wait_passive(self, expected, timeout, stop=None, first=None):
        """Before the first write, qualify four consecutive quiet full-bus samples."""
        if self.state != "passive" or self._original:
            raise RuntimeError("只读等待仅可用于尚未接管的总线。")
        deadline = time.monotonic() + timeout
        low, high = {}, {}
        count, quiet_since = 0, 0.
        flagged = set()
        while True:
            if stop is not None and stop.is_set():
                raise InterruptedError("现场已停止。")
            states = self._all() if first is None else first
            first = None
            issue = self._fixture_issue(states, check_motion=False)
            if issue:
                raise RuntimeError(issue)
            now = time.monotonic()
            for i in BUS_IDS:
                value = states[i]["position"]
                lo = low[i] = min(low.get(i, value), value)
                hi = high[i] = max(high.get(i, value), value)
                if hi - lo > 5:
                    raise UnsteadyPose(f"只读等待中 ID{i} 的位置从最低 {lo} 到最高 {hi} 格，支撑仍在移动。")
                if expected is not None and abs(value - expected[i]) > 5:
                    raise UnsteadyPose(f"ID{i} 已离开本轮起点：参考 {expected[i]}，实测 {value} 格。")
            moving = {i for i in BUS_IDS if states[i]["moving"]}
            if moving:
                if not flagged:
                    ids = "、".join(f"ID{i}" for i in sorted(moving))
                    print(f"{ids} 报告运动，正在只读等待静止。", flush=True)
                flagged.update(moving)
                count = 0
            else:
                if count == 0:
                    quiet_since = now
                count += 1
                if count >= 4 and now - quiet_since >= .28 and now <= deadline:
                    return states
            if now >= deadline:
                ids = "、".join(f"ID{i}" for i in sorted(flagged))
                raise UnsteadyPose(f"只读等待 {timeout:g} 秒仍未取得连续静止读数，运动标志涉及 {ids or '未知轴'}。")
            time.sleep(.1)

    def enable_hold(self, stop_event=None):
        self.start_stop = stop_event
        try:
            return super().enable_hold(stop_event)
        finally:
            self.start_stop = None

    def _hold_axes(self, states):
        # The base class selects axes before it saves settings or writes any RAM.
        # Replace the initial raw snapshot with the newly qualified one so the
        # following preloads and passive references are based on the same sample.
        fresh = self.wait_passive(self.contact_pose, 8., self.start_stop, first=states)
        states.update(fresh)
        return super()._hold_axes(states)

    def _verify(self, states):
        super()._verify(states)
        if self.jaw_band is not None:
            lo, hi = self.jaw_band
            jaw = states[JAW]
            if not lo <= jaw["position"] <= hi or abs(jaw["position"] - jaw["goal"]) > 12:
                raise RuntimeError("夹口偏离本轮实测接触位置，已停止。")
            if self.retained_goal is not None and jaw["goal"] != self.retained_goal:
                raise RuntimeError("夹紧目标发生变化，已停止。")

    def step_goal(self, axis, goal, stop):
        """One incremental RAM position write, bounded against fresh feedback."""
        snap = self.status()
        held_pose(snap)
        if stop.is_set():
            raise InterruptedError("现场已停止。")
        row = snap["rows"][axis]
        if (axis not in ACTIVE or not row["minimum"] <= goal <= row["maximum"]
                or abs(goal - row["position"]) > 12
                or abs(goal - row["target"]) > 3):
            raise ValueError(f"ID{axis} 小步目标超出本轮移动范围。")
        self._send({axis: goal})

    def _freeze(self):
        if self.retained_goal is None:
            return super()._freeze()
        # Hold the wrist where it stopped without opening the jaws under load.
        states = self._all()
        for i in ACTIVE:
            self._verify_enabled(i, states[i])
        self._send({WRIST: states[WRIST]["position"]})
        self.state, self.reason = "holding", "腕部已停止，夹爪保持到模型获得承托。"
        latched = self._all()
        self._verify(latched)
        return self._pack(latched)


class UnsteadyPose(ValueError):
    """A recording needs another stationary sample."""


def direct_read_pose(bus, expected=None, timeout=8., stop=None):
    states = bus.bus.wait_passive(expected, timeout, stop)
    return {i: states[i]["position"] for i in BUS_IDS}


def read_pose(bus):
    samples = []
    for _ in range(4):
        s = bus.status()
        if s["state"] != "passive":
            raise ValueError("示教时需要只读状态。")
        if s["teaching_fault"]:
            raise ValueError(s["teaching_fault"])
        if s["fixture_issue"]:
            raise UnsteadyPose(s["fixture_issue"])
        samples.append({**s["fixture_positions"], **{i: s["rows"][i]["position"] for i in ACTIVE}})
        time.sleep(.1)
    for i in BUS_IDS:
        values = [s[i] for s in samples]
        if max(values) - min(values) > 5:
            raise UnsteadyPose(f"ID{i} 松手后未停稳。")
    return samples[-1]


class AdjustHover(ValueError):
    """The low pose is still valid; only the hand-taught high pose needs adjustment."""


def pose_difference(contact, hover):
    wrist = abs(hover[WRIST] - contact[WRIST])
    jaw = hover[JAW] - contact[JAW]
    return wrist, jaw


def validate_pair(contact, hover):
    drift = fixture_drift(contact, hover)
    if drift:
        raise ValueError(drift)
    for pose in (contact, hover):
        for i in (WRIST, JAW):
            lo, hi, _ = PROFILE[i]
            if not lo + MARGIN <= pose[i] <= hi - MARGIN:
                raise ValueError(f"{JOINT_NAMES[i]}的示教位置接近行程边缘。")
    wrist, jaw = pose_difference(contact, hover)
    if wrist < WRIST_TRAVEL[0]:
        raise AdjustHover("腕部略抬高些")
    if wrist > WRIST_TRAVEL[1]:
        raise AdjustHover("腕部往低位收回一些")
    if jaw < JAW_OPENING[0]:
        raise AdjustHover("夹口再张开一点")
    if jaw > JAW_OPENING[1]:
        raise AdjustHover("夹口稍收小些")


def hover_keys():
    """Read a short console command without stopping the live encoder display."""
    import msvcrt
    command = ""
    while True:
        while msvcrt.kbhit():
            key = msvcrt.getwch()
            if key in ("\x00", "\xe0"):
                msvcrt.getwch()
            elif key in ("\x1b", "q", "Q"):
                yield "quit"
            elif key == "\r":
                if command == "2":
                    command = ""
                    yield "capture"
                command = ""
            elif key == "\b":
                command = command[:-1]
            else:
                command = (command + key)[-2:]
        yield None


def teach_hover(bus, contact, recorded_at, report, keys):
    print("第①点已保留。现在是手动示教，电机不会主动抬起。看夹尖确实向上、练习剪仍在垫块上。")
    print("两处松手能停稳、窗口显示「可记录」时，按 2 再回车。只有之后输入 G 才启动自动回放。按 Q 或 Esc 可退出。")
    previous = ""
    while time.monotonic() - recorded_at <= 900:
        s = bus.status()
        if s["state"] != "passive" or s["teaching_fault"]:
            raise RuntimeError(s.get("teaching_fault") or "示教只读状态中断。")
        pose = {**s["fixture_positions"], **{i: s["rows"][i]["position"] for i in ACTIVE}}
        report["last_teaching_sample"] = pose
        wrist, jaw = pose_difference(contact, pose)
        try:
            validate_pair(contact, pose)
            hint = s["fixture_issue"] or "可记录"
        except AdjustHover as exc:
            hint = str(exc)
        message = f"抬腕幅度 {wrist/TICKS_PER_DEG:4.1f}°  |  夹口比低位 {jaw:+d} 格  |  {hint}"
        if message != previous:
            print("\r" + message + " " * 14, end="", flush=True)
            previous = message
        command = next(keys)
        if command == "quit":
            print()
            report["teaching_exit"] = "cancelled_before_high_point"
            print("已退出第②步，未录入高位。本轮没有发送运动指令。")
            return None
        if command == "capture":
            print()
            report["hover_attempt"] = pose
            if hint == "可记录":
                try:
                    hover = read_pose(bus)
                    report["hover_attempt"] = hover
                    validate_pair(contact, hover)
                except (AdjustHover, UnsteadyPose) as exc:
                    print("读数变化，第①点已保留。", exc)
                else:
                    return hover
            else:
                print(f"第②点还未记录：{hint}。可继续调整。")
            previous = ""
        time.sleep(.18)
    raise ValueError("两点示教已超过 15 分钟。请重新录入。")


def require_near(live, expected, tolerance=5):
    if any(abs(live[i] - expected[i]) > tolerance for i in BUS_IDS):
        raise RuntimeError("当前姿态已偏离刚录下的起点，请重新录入。")


class Stop(threading.Event):
    def __init__(self, live, native_check=no_stop):
        super().__init__()
        self.live = live
        self.native_check = native_check

    def is_set(self):
        try:
            self.native_check()
        except InterruptedError:
            self.set()
        if self.live:
            import msvcrt
            while msvcrt.kbhit():
                if msvcrt.getwch().lower() in ("\x1b", "q"):
                    self.set()
        return super().is_set()


def check_pause(bus, stop, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if stop.is_set():
            raise InterruptedError("现场已停止。")
        if bus.status()["state"] != "holding":
            raise RuntimeError("保持反馈中断。")
        time.sleep(.08)


def held_pose(snapshot):
    if (snapshot["state"] != "holding"
            or {i for i, r in snapshot["rows"].items() if r["held"]} != set(ACTIVE)
            or any(snapshot["fixture_torques"].values())):
        raise RuntimeError("腕与夹爪的专属使能范围核验失败。")
    return {**snapshot["fixture_positions"], **{i: snapshot["rows"][i]["position"] for i in ACTIVE}}


def enable_at(bus, contact, expected, stop):
    if isinstance(bus.bus, DirectPickBus):
        direct_read_pose(bus, expected, stop=stop)
    else:
        require_near(read_pose(bus), expected)
    bus.bus.bind_fixture(contact)
    require_near(held_pose(bus.enable_hold(stop_event=stop)), expected)
    check_pause(bus, stop, .6)


def measured_move(bus, axis, target, label, stop, log):
    if stop.is_set():
        raise InterruptedError("现场已停止。")
    print(label, flush=True)
    s = bus.move_to({axis: target}, 8, stop)
    if stop.is_set():
        raise InterruptedError("现场已停止。")
    positions = held_pose(s)
    row = s["rows"][axis]
    tolerance = 4 if axis == JAW else 8
    if row["target"] != target or abs(positions[axis] - target) > tolerance or row["moving"]:
        raise RuntimeError(f"{JOINT_NAMES[axis]}未到位：指令 {target} 格，"
                           f"保持目标 {row['target']} 格，实测 {positions[axis]} 格。")
    log.append({"stage": label, "id": axis, "goal": target, "actual": positions[axis]})
    return positions


def wrist_direction_pilot(bus, opened, sign, stop, moves):
    """Latch a small observed free-air displacement; this probe only identifies direction."""
    now_pose = held_pose(bus.status())
    require_near(now_pose, opened)
    start = now_pose[WRIST]
    if sign not in (-1, 1):
        raise ValueError("腕部试动方向无效。")
    goal = start + sign * 16
    lo, hi, _ = PROFILE[WRIST]
    if not lo + MARGIN <= goal <= hi - MARGIN:
        raise ValueError("试动角度接近腕部行程边缘。")
    quiet = deque()
    acquired = False

    class PilotStop:
        def is_set(self):
            return stop.is_set() or acquired

        def wait(self, seconds):
            return stop.wait(seconds)

    def sample(snapshot):
        nonlocal acquired
        w, j = snapshot["rows"][WRIST], snapshot["rows"][JAW]
        pos = w["position"]
        if (w["target"] != goal or w["moving"] or j["moving"]
                or not 8 <= sign * (pos - start) <= 16
                or abs(j["position"] - opened[JAW]) > 5):
            quiet.clear()
            return
        # A slow creep through successive ticks must not count as a settled position.
        if any(abs(pos - p) > 1 for _, p in quiet):
            quiet.clear()
        now = time.monotonic()
        quiet.append((now, pos))
        acquired = len(quiet) >= 3 and now - quiet[0][0] >= .12

    print(f"空爪方向试动：从实测 {start} 格出发，目标 {goal} 格。", flush=True)
    s = bus.move_to({WRIST: goal}, 8, PilotStop(), on_progress=sample)
    if stop.is_set():
        raise InterruptedError("现场已停止。")
    pos = held_pose(s)
    if not 8 <= sign * (pos[WRIST] - start) <= 16 or abs(pos[JAW] - opened[JAW]) > 5:
        raise RuntimeError(f"方向试动目标 {goal} 格，起点 {start} 格，实测 {pos[WRIST]} 格，未取得稳定方向。")
    if not acquired:
        # A normal endpoint already passed the trajectory's stationary settling check.
        if s["rows"][WRIST]["target"] != goal or any(s["rows"][i]["moving"] for i in ACTIVE):
            raise RuntimeError("试动未完成正常到位或静止位置锁定。")
        s = bus.stop_hold()
    # Both the early-exit path and the normal path now hold the measured position.
    anchor = s["rows"][WRIST]["target"]
    if not 8 <= sign * (anchor - start) <= 16:
        raise RuntimeError("方向试动的实测锁定点不在试动范围内。")
    stable = []
    for _ in range(4):
        check_pause(bus, stop, .08)
        s = bus.status()
        pos = held_pose(s)
        w = s["rows"][WRIST]
        if (w["target"] != anchor or abs(pos[WRIST] - anchor) > 2
                or not 8 <= sign * (pos[WRIST] - start) <= 16
                or abs(pos[JAW] - opened[JAW]) > 5
                or any(s["rows"][i]["moving"] for i in ACTIVE)):
            raise RuntimeError("腕部试动后未能稳定保持实测位置。")
        stable.append(pos[WRIST])
    if max(stable) - min(stable) > 1:
        raise RuntimeError("试动锁定点仍在漂移。")
    delta = pos[WRIST] - start
    moves.append({"stage": "空爪方向试动", "id": WRIST, "start": start, "goal": goal,
                  "held": anchor, "actual": pos[WRIST], "delta": delta})
    print(f"试动实测：{start} → {pos[WRIST]} 格，位移 {delta:+d} 格，已保持。", flush=True)
    return pos


def replay_arc(bus, contact, hover, stop, report):
    validate_pair(contact, hover)
    require_near(held_pose(bus.status()), hover)
    for label, axis, target in (
            ("下降到剪柄", WRIST, contact[WRIST]),
            ("合到实测夹持点", JAW, contact[JAW]),
            ("抬起，停留两秒", WRIST, hover[WRIST]),
            ("放回原垫块", WRIST, contact[WRIST]),
            ("张开夹口", JAW, hover[JAW]),
            ("空爪回到起点", WRIST, hover[WRIST])):
        report["last_requested_stage"] = label
        measured_move(bus, axis, target, label, stop, report["encoder_stages"])
        if label == "抬起，停留两秒":
            check_pause(bus, stop, 2)
    report["encoder_cycle_completed"] = True
    print("关节轨迹已完成。请以实物是否离开垫块并放回判断夹取结果。", flush=True)


def run_arc(bus, contact, hover, stop, report):
    validate_pair(contact, hover)
    enable_at(bus, contact, hover, stop)
    replay_arc(bus, contact, hover, stop, report)


def held_command(bus, stop, prompt, allowed, timeout=90):
    """While awaiting a deliberate command, monitor the bus and native console events."""
    import msvcrt
    # Discard keys typed during the preceding stage. Preserve any emergency stop.
    if stop.is_set():
        raise InterruptedError("现场已停止。")
    print(prompt + "  输入后回车（Esc 或 Q 停止）：", flush=True)
    typed = ""
    expires = time.monotonic() + timeout
    while time.monotonic() < expires:
        stop.native_check()
        held_pose(bus.status())
        while msvcrt.kbhit():
            key = msvcrt.getwch()
            if key in ("\x00", "\xe0"):
                msvcrt.getwch()
            elif key in ("\x1b", "Q", "q"):
                stop.set()
                raise InterruptedError("现场已停止。")
            elif key == "\r":
                print(flush=True)
                if typed in allowed:
                    return typed
                typed = ""
                print("请输入 " + " / ".join(allowed) + " 后回车。", flush=True)
            elif key == "\b":
                if typed:
                    typed = typed[:-1]
                    print("\b \b", end="", flush=True)
            elif key.isprintable() and len(typed) < 3:
                typed += key.upper()
                print(key.upper(), end="", flush=True)
        time.sleep(.06)
    raise ValueError("等待按键超时。请承托并卸力。")


def direct_stage(bus, axis, goal, stop, report, label, *, tolerance, minimum_motion):
    """Advance slowly, require a stable measured displacement in the commanded direction."""
    report["last_requested_stage"] = label
    print(label, flush=True)
    snapshot = bus.status()
    start = held_pose(snapshot)[axis]
    sign = 1 if goal >= start else -1
    deadline = time.monotonic() + 7
    best = 0
    progressed = time.monotonic()
    quiet_low = quiet_high = quiet_at = None
    while True:
        if stop.is_set():
            raise InterruptedError("现场已停止。")
        snapshot = bus.status()
        actual = held_pose(snapshot)[axis]
        row = snapshot["rows"][axis]
        now = time.monotonic()
        travel = sign * (actual - start)
        if travel < -3 or travel > abs(goal - start) + 4:
            raise RuntimeError(f"{label}：反馈移出本次短行程，实测 {actual} 格。")
        if travel > best:
            best, progressed = travel, now
        if (row["target"] == goal and abs(actual - goal) <= tolerance
                and travel >= minimum_motion and all(not snapshot["rows"][i]["moving"] for i in ACTIVE)):
            if quiet_at is None or max(quiet_high, actual) - min(quiet_low, actual) > 2:
                quiet_low = quiet_high = actual
                quiet_at = now
            quiet_low, quiet_high = min(quiet_low, actual), max(quiet_high, actual)
            if now - quiet_at >= .24:
                report.setdefault("actions", []).append(dict(stage=label, id=axis, start=start,
                                                             goal=goal, actual=actual))
                return actual
        else:
            quiet_at = None
        if now >= deadline or now - progressed > 1.8:
            # status() verifies the tracked goal against the actual servo register.
            last_command = row["target"]
            report["stage_timeout"] = dict(stage=label, id=axis, start=start, goal=goal,
                                           last_command=last_command, actual=actual,
                                           load_raw=row["load_raw"], moving=row["moving"],
                                           seconds_without_progress=round(now - progressed, 2))
            raise RuntimeError(f"{label}跟随停止：ID{axis}，起点 {start}，阶段目标 {goal}，"
                               f"最后写入并回读 {last_command}，实测 {actual} 格。")
        previous = row["target"]
        delta = max(-3, min(3, goal - previous))
        bounded = min(actual + 12, max(actual - 12, previous + delta))
        if delta * (bounded - previous) > 0:
            bus.bus.step_goal(axis, bounded, stop)
        stop.wait(.06)


def direct_pick(bus, low, stop, report, ask):
    """One live run uses a newly observed empty-wrist direction and the supported prop."""
    report["released_at_support"] = False
    report.setdefault("actions", [])
    touch = low[JAW]
    opened = touch + 30
    firm = touch - 4
    if not PROFILE[JAW][0] + MARGIN <= firm < opened <= PROFILE[JAW][1] - MARGIN:
        raise ValueError("此夹口位置未留出张开及轻收紧所需行程。")
    enable_at(bus, low, low, stop)
    try:
        direct_stage(bus, JAW, opened, stop, report, "张开夹口，模型留在垫块上",
                     tolerance=6, minimum_motion=24)
        key = ask("看清两夹垫均脱离剪柄、空爪上下约 2° 往返均有净空。选 + 或 - 做一次空腕试动。"
                  "此前 + 方向曾抬起，请从侧面看这一次。", ("+", "-"))
        if key not in ("+", "-"):
            raise InterruptedError("未确认试动净空。")
        sign = 1 if key == "+" else -1
        upper = low[WRIST] + sign * 64
        if not PROFILE[WRIST][0] + MARGIN <= upper <= PROFILE[WRIST][1] - MARGIN:
            raise ValueError("所选抬腕方向靠近限位。")
        direct_stage(bus, WRIST, low[WRIST] + sign * 24, stop, report, "观察空夹尖的试动方向",
                     tolerance=10, minimum_motion=12)
        check_pause(bus, stop, 1.2)
        direct_stage(bus, WRIST, low[WRIST], stop, report, "空腕沿原路回到剪柄旁",
                     tolerance=12, minimum_motion=4)
        seated = direct_stage(bus, JAW, touch, stop, report, "夹垫合到本轮剪柄接触位",
                              tolerance=8, minimum_motion=12)
        if ask("确认刚才空夹尖确实向上，约 6° 全程有空间，且现在两夹垫都贴住实心柄。"
               "看清后输入 U 开始收紧、抬起和放回；未看清或方向向下请按 Q。", ("U",)) != "U":
            raise InterruptedError("未确认方向和双垫贴柄。")
        actual = held_pose(bus.status())
        # A stationary backlash offset (observed at 9–10 ticks) is acceptable
        # only on the approach side and with actual pad contact confirmed.
        returned_offset = sign * (actual[WRIST] - low[WRIST])
        if abs(actual[JAW] - seated) > 2 or not -4 <= returned_offset <= 12:
            raise RuntimeError("等待期间夹持位置已变化。")
        report["confirmed_touch_actual"] = seated
        report["firm_target"] = firm
        print("两级微收紧，保持夹持目标。", flush=True)
        bus.bus.jaw_band = (seated - 6, seated + 2)
        for goal in (touch - 2, firm):
            bus.bus.step_goal(JAW, goal, stop)
            check_pause(bus, stop, .16)
        bus.bus.retained_goal = firm
        readings = []
        for _ in range(5):
            check_pause(bus, stop, .08)
            snapshot = bus.status()
            readings.append(held_pose(snapshot)[JAW])
            if snapshot["rows"][JAW]["moving"]:
                raise RuntimeError("预紧后夹垫尚在移动。")
        if max(readings) - min(readings) > 2:
            raise RuntimeError("预紧后夹垫未停稳。")
        report["actions"].append(dict(stage="夹柄预紧", id=JAW, goal=firm, actual=readings[-1]))
        direct_stage(bus, WRIST, upper, stop, report, "夹住模型向上抬，侧拍垫块间隙",
                     tolerance=12, minimum_motion=40)
        check_pause(bus, stop, 2)
        direct_stage(bus, WRIST, low[WRIST], stop, report, "夹持模型沿原路放回垫块",
                     tolerance=12, minimum_motion=35)
        if ask("目视模型已由垫块或固定软垫托住，输入 R 松夹。若还悬空，按 Q 停止。", ("R",)) != "R":
            raise InterruptedError("尚未确认模型有承托。")
        current = held_pose(bus.status())
        if not -4 <= sign * (current[WRIST] - low[WRIST]) <= 12:
            raise RuntimeError("松夹前腕部已偏离放回位置。")
        bus.bus.retained_goal = bus.bus.jaw_band = None
        direct_stage(bus, JAW, opened, stop, report, "在承托面上松夹", tolerance=6, minimum_motion=20)
        report["released_at_support"] = True
        report["encoder_cycle_completed"] = True
    except BaseException:
        if bus.state == "holding":
            bus.stop_hold()
        raise


def keyboard_cycle(bus, contact, stop, report, ask):
    """Choose one upward wrist arc under direct observation, with the prop supported."""
    # Reserve room in both directions before holding or making the first motion.
    lo, hi, _ = PROFILE[WRIST]
    if not lo + MARGIN + 16 <= contact[WRIST] <= hi - MARGIN - 16:
        raise ValueError("当前腕角离行程边缘太近。")
    lo, hi, _ = PROFILE[JAW]
    open_goal = min(contact[JAW] + 40, hi - MARGIN)
    if contact[JAW] < lo + MARGIN or open_goal - contact[JAW] < 24:
        raise ValueError("当前夹口张开余量不足。")
    moves = report["teaching_moves"] = []
    enable_at(bus, contact, contact, stop)
    opened = measured_move(bus, JAW, open_goal, "键盘示教：张开夹口，练习剪留在垫块上。", stop, moves)
    if not JAW_OPENING[0] <= opened[JAW] - contact[JAW] <= JAW_OPENING[1]:
        raise RuntimeError("张开幅度回读不足。")
    require_near(opened, {**contact, JAW: opened[JAW]})

    unused = ["+", "-"]
    while unused:
        key = ask("从侧面确认两夹垫已离开剪柄，夹尖上下两侧的小幅试动都不会碰到剪子或台面。"
                  "选 " + " 或 ".join(unused) + " 试动一次约 1.4°。不清楚请按 Q。", tuple(unused))
        if key not in unused:
            raise InterruptedError("未选定试动方向。")
        unused.remove(key)
        sign = 1 if key == "+" else -1
        pilot = wrist_direction_pilot(bus, opened, sign, stop, moves)
        pilot_start = moves[-1]["start"]
        answer = ask("看夹尖：明显抬起且同方向至约 5.6° 的全程有净空，输入 U。"
                     "若方向向下且刚走过的短弧可原路返回，输入 R。分辨不清请按 Q。", ("U", "R"))
        require_near(held_pose(bus.status()), pilot)
        if answer == "U":
            break
        if answer != "R":
            raise InterruptedError("尚未确认抬腕方向。")
        back = measured_move(bus, WRIST, pilot_start, "空爪沿试动短弧回位。", stop, moves)
        require_near(back, opened)
    else:
        raise ValueError("两个方向均未确认为抬起，已停止键盘示教。")

    upper = contact[WRIST] + sign * 64
    if not PROFILE[WRIST][0] + MARGIN <= upper <= PROFILE[WRIST][1] - MARGIN:
        raise ValueError("所选方向的抬腕位置接近行程边缘。")
    hover = measured_move(bus, WRIST, upper, "空爪沿已确认的方向抬到录像高位。", stop, moves)
    validate_pair(contact, hover)
    report["hover"] = hover
    answer = ask("确认夹尖在剪柄上方、空爪扫过全程无碰撞，模型仍在原处。开录像，手离开后输入 G 回放。", ("G",))
    if answer != "G":
        raise InterruptedError("未启动回放。")
    report["replay_requested"] = True
    replay_arc(bus, contact, hover, stop, report)


def confirm_supported(bus, native_check):
    import msvcrt
    print("请从夹口以外承托好右臂，并确认模型已由垫块或承接面托住。若仍夹着模型，请让模型也有承接。", flush=True)
    print("当前等待卸力，运动按键已结束。支撑就位后按 S（不用回车）；若已承托并物理关闭右侧电机电源，按 F。", flush=True)
    warned = False
    last_key_notice = None
    while True:
        try:
            native_check()
        except InterruptedError:
            # The operator verified continuous mechanical support before G.
            try:
                bus.release()
                return "torque_off_verified"
            except Exception as exc:
                print(f"卸力反馈未确认：{exc}。保持承托，关闭右侧电机电源后按 F。", flush=True)
                native_check = no_stop
        if msvcrt.kbhit():
            key = msvcrt.getwch().lower()
            if key == "f":
                bus.abandon_supported()
                return "power_off_operator_confirmed"
            if key == "s":
                try:
                    s = bus.release()
                    if any(r["torque"] for r in s["rows"].values()) or any(s["fixture_torques"].values()):
                        raise RuntimeError("卸力反馈未确认")
                    print("已核实腕和夹爪卸力，其余七轴保持未使能。")
                    return "torque_off_verified"
                except Exception as exc:
                    print(f"{exc}。保持支撑，关闭右侧电机电源后按 F。", flush=True)
            elif key.isprintable():
                now = time.monotonic()
                if last_key_notice is None or now - last_key_notice >= 1:
                    print(f"已收到 {key.upper()}。当前等待卸力；支撑就位后请按 S。", flush=True)
                    last_key_notice = now
        try:
            bus.status()
        except Exception as exc:
            if not warned:
                print(f"{exc}。保持支撑，必要时关闭右侧电机电源。", flush=True)
                warned = True
        time.sleep(.16)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true", help="在 Windows 本机交互示教并运行")
    parser.add_argument("--direct", action="store_true", help="夹紧、抬离、放回，一次完成")
    options = parser.parse_args()
    live, direct = options.live, options.direct
    if live and (sys.platform != "win32" or not sys.stdin.isatty()):
        raise ValueError("实机模式需要 Windows 本地交互窗口。")
    bus = TracedSmoothBus(simulate=not live, bus_class=DirectPickBus if direct else SupportedWristBus)
    report = dict(mode="hardware" if live else "simulation", controlled_ids=ACTIVE, motion_requested=False,
                  encoder_stages=[], encoder_cycle_completed=False, physical_outcome="unconfirmed")
    bus.connect()
    try:
        if live:
            capture = direct_read_pose if direct else read_pose
            capture(bus)
            print(f"已连接 {bus.bus.port_name}，正在只读检查。电脑旁留人，可随时切断电机电源。")
            print("本入口仅向右腕俯仰 ID4 和夹爪 ID6 写入。请用独立稳固的机械支撑固定底座、大臂、肘和前臂，锁住小车位置。")
            print("腕部在整个短弧内有承接保护，夹口范围内只放无刃轻质模型。如需手扶、会滑动或没有合适固定支撑，请退出。")
            print("① 模型在稳固垫块上。低位两夹垫轻贴实心柄；腕和夹指与台面有间隙。")
            if direct:
                print("实机夹取：手机侧拍剪柄与垫块。让两夹垫宽面轻贴粉色实心柄，确认张开时两侧有空间。")
                print("手和衣袖离开；前臂固定，短弧下方有承接，练习剪单独放稳。Q 或 Esc 可停止。")
                selection = input("录像已开始、支撑和净空已确认，输入 G 回车开始：").strip().upper()
                if selection != "G":
                    return
            else:
                print("推荐 K 键盘模式：先从侧面确认夹口两侧张开有净空，前臂和模型都能靠支撑停稳。")
                print("手和衣袖离开后，输入 K 回车就会使能右腕和右夹爪，并电动张开夹口。之后只用键盘。")
                selection = input("K 键盘示教   |   1 原手动只读示教   |   其他键退出：").strip().upper()
                if selection not in ("K", "1"):
                    return
            contact = capture(bus)
            report["contact"] = contact
            report["teaching_mode"] = {"K": "keyboard", "1": "manual", "G": "direct_pick"}[selection]
            captured_at = time.monotonic()
            if selection == "1":
                print("② 手动示范一次抬腕：模型留在垫块上，从夹指外侧轻轻张开夹口让它脱离模型。")
                print("从外侧托住夹爪后面的黑色腕壳，让夹尖像点头后抬头一样朝上；前臂留在支撑上。遇到阻力请停。")
                print("确认刚经过的弧线全程有净空，其余关节保持原位，手离开后能停稳。")
                hover = teach_hover(bus, contact, captured_at, report, hover_keys())
                if hover is None:
                    return
                report["hover"] = hover
                print("按下开始后会下降、夹持、抬起、放回和张开。运动中按 Esc 停止。")
                print("确认全程有机械承接，断电也能托住前臂和腕。Ctrl+C 或关闭窗口时按已承托处理并卸力。")
                if input("开录像，确认模型与垫块原位、支撑稳固、人员离开路径，输入 G 回车开始：").strip().upper() != "G":
                    return
            if time.monotonic() - captured_at > 900:
                raise ValueError("低位示教已超过 15 分钟。请重新录入当前两点。")
        else:
            print("模拟模式")
            hover = direct_read_pose(bus) if direct else read_pose(bus)
            contact = dict(hover)
            if not direct:
                contact[WRIST] -= 65
                contact[JAW] -= 35
        if live:
            with motion_stop() as native_check:
                try:
                    stop = Stop(True, native_check)
                    report["motion_requested"] = True
                    if direct:
                        def ask_direct(prompt, choices):
                            return held_command(bus, stop, prompt, choices, timeout=25)
                        direct_pick(bus, contact, stop, report, ask_direct)
                    elif selection == "K":
                        def ask(prompt, choices):
                            return held_command(bus, stop, prompt, choices)
                        keyboard_cycle(bus, contact, stop, report, ask)
                    else:
                        report["replay_requested"] = True
                        run_arc(bus, contact, hover, stop, report)
                except (Exception, KeyboardInterrupt) as exc:
                    if bus.state != "passive":
                        print(f"轨迹已停止：{exc}\nU 和 G 已退出，下面请按提示卸力。", flush=True)
                    raise
                finally:
                    if bus.state != "passive":
                        report["release"] = confirm_supported(bus, native_check)
            print("录像中模型是否完整离开垫块并放回？1 看到了；2 没有或滑落；其他键 未确认。")
            answer = input().strip()
            report["physical_outcome"] = {"1": "operator_observed_lift_and_return",
                                          "2": "operator_observed_failure"}.get(answer, "unconfirmed")
        else:
            try:
                if direct:
                    direct_pick(bus, contact, Stop(False), report, lambda _, choices: choices[0])
                    print("模拟夹持与腕部往返完成；实物夹取结果须由现场录像确认。")
                else:
                    run_arc(bus, contact, hover, Stop(False), report)
            finally:
                if bus.state != "passive":
                    bus.release()
    except BaseException as exc:
        report["error"] = str(exc)
        raise
    finally:
        try:
            bus.close()
        finally:
            if live and bus.path is not None:
                path = bus.path.with_name(bus.path.stem.replace("smooth_right_", "film_lift_") + ".json")
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
                print("录像核验记录：", path)


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print("本轮已停止。", exc, file=sys.stderr)
        sys.exit(1)
