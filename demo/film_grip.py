"""One supported pinch-and-release for filming, using the local contact pose."""
import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from jog_bus import JogBus, MARGIN, PROFILE, TRACKING_TOLERANCE
from pinch_taught import motion_stop
from right_table_state import SERIAL_NUMBER


JAW = 6
BEARING_SECONDS = 25
MAX_TARGET_LEAD = 12
FIRM_PRELOAD = 4  # Servo encoder ticks beyond this run's light-touch reference.


def support_keys(expected="R"):
    """Discard early commands, then require the key for the current phase and Enter."""
    import msvcrt
    pending = ""
    while msvcrt.kbhit():
        if msvcrt.getwch().upper() in ("Q", "\x1b"):
            raise InterruptedError("现场已停止。")
    yield False  # Armed just before the support-transfer prompt.
    while True:
        while msvcrt.kbhit():
            key = msvcrt.getwch().upper()
            if key in ("\x00", "\xe0".upper()):
                msvcrt.getwch()
            elif key in ("Q", "\x1b"):
                raise InterruptedError("现场已停止。")
            elif key == "\r":
                if pending == expected:
                    yield True
                elif pending:
                    print(f"本步请按 {expected} 再回车，或按 Q 停止。", flush=True)
                pending = ""
            elif key == "\b":
                pending = pending[:-1]
            else:
                pending = (pending + key)[-2:]
                if pending == expected:
                    print(f"{expected}：回车确认。", flush=True)
        yield False


class FilmingJaw(JogBus):
    """The sole writable device in this demonstration is the physical right gripper."""

    def _write(self, i, address, value):
        if i != JAW:
            raise ValueError("录像演示只能写右夹爪 ID6。")
        super()._write(i, address, value)

    def passive(self):
        states = self._all()
        row = self._rows(states)[JAW]
        if not row["ready"]:
            raise ValueError(f"右夹爪实测 {row['position']} 格：{row['reason']}")
        return states

    def setup_snapshot(self, result):
        """Observe the supported jaw, even just outside its commanded travel window."""
        states = self._all()
        pos = states[JAW]["position"]
        result.setdefault("setup_readings", []).append(dict(
            position=pos, torque_by_id={i: s["torque"] for i, s in states.items()},
            voltage_raw=states[JAW]["voltage"], temp_c=states[JAW]["temp"],
            effort=states[JAW]["effort"]))
        for i, s in states.items():
            identity = self._identity_issue(i)
            if identity:
                raise ValueError(f"实测夹口 {pos} 格，ID{i} 身份核对失败：{identity}。")
            if s["torque"] or not 90 <= s["voltage"] <= 140 or s["temp"] >= 60 or not 0 < s["effort"] <= 1000:
                raise ValueError(f"实测夹口 {pos} 格，ID{i} 状态异常：力矩 {s['torque']}、"
                                 f"电压 {s['voltage']/10:.1f} V、温度 {s['temp']} °C、输出限值 {s['effort']}。")
        low, high, _ = PROFILE[JAW]
        if not low <= pos <= high:
            raise ValueError(f"实测夹口 {pos} 格，超出本机 EEPROM 硬限 {low}–{high} 格。"
                             "保持当前姿态，先核对夹爪零位和机械装配。")
        return states

    def prepare_contact(self, result, initial, prompt=input, *, bearing=False):
        current = initial
        low, high, _ = PROFILE[JAW]
        soft_low, soft_high = low + MARGIN, high - MARGIN
        minimum_touch = soft_low + (FIRM_PRELOAD if bearing else 0)
        while True:
            pos = current[JAW]["position"]
            print(f"只读摆位：实测夹口 {pos} 格，可动范围 {soft_low}–{soft_high} 格，"
                  f"本轮需向张开方向留出 24 格。", flush=True)
            if bearing:
                print(f"轻贴位置需向闭合方向留出 {FIRM_PRELOAD} 格，用于确认接触后的微量收紧。")
            if not minimum_touch <= pos <= soft_high - 24:
                print("当前开口不适合启动。模型和手腕各自托稳，夹垫从外侧轻轻贴到实心柄段即可。"
                      "若轻贴时仍没有余量或有阻力，请按 Q 退出。", flush=True)
            if prompt("夹垫已轻贴、手已离开，回车复查并开始；Q 回车退出：").strip():
                raise InterruptedError("已取消，只读摆位期间没有发出运动指令。")
            current = self.setup_snapshot(result)
            if minimum_touch <= current[JAW]["position"] <= soft_high - 24:
                return current

    def run_cycle(self, result, stop, support_input=None, confirmed=None, *, from_contact=False):
        if from_contact and support_input is None:
            raise ValueError("从贴柄开始需要现场确认接触和承托。")
        baseline = self.passive()
        if confirmed is not None:
            self._watch_passive(baseline, confirmed, None)
            if abs(baseline[JAW]["position"] - confirmed[JAW]["position"]) > 2:
                raise ValueError("回车确认后夹口位置发生变化，尚未启动。")
        contact = baseline[JAW]["position"]
        firm_target = contact - FIRM_PRELOAD
        if support_input is not None and firm_target < PROFILE[JAW][0] + MARGIN:
            raise ValueError("当前轻贴位置离闭合软限太近，没有收紧余量。未使能夹爪。")
        opened = min(contact + 52, PROFILE[JAW][1] - MARGIN)
        if opened - contact < 24:
            raise ValueError(f"当前夹爪 {contact} 格，余下张幅不足。请检查两垫是否轻贴模型实心柄段。")
        result.update(contact=contact, open_target=opened, events=[], samples=[],
                      initial_feedback={i: {key: row[key] for key in ("position", "torque", "voltage", "temp")}
                                        for i, row in baseline.items()},
                      voltage_unit="0.1 V", temperature_unit="deg C")
        print(f"本轮接触参考 {contact} 格，张开目标 {opened} 格。", flush=True)
        for count in (3, 2, 1):
            print(count, flush=True)
            for _ in range(5):
                time.sleep(.18)
                stop()
                measured = self.passive()
                self._watch_passive(measured, baseline, None)
                if abs(measured[JAW]["position"] - contact) > 2:
                    raise ValueError("倒计时期间夹指位置变化，请稳固模型和机械臂。")

        maximum_effort = self._number(JAW, 16, 2)
        if not 0 < maximum_effort <= 1000:
            raise ValueError("夹爪 EEPROM 输出上限读数异常。")
        jaw = measured[JAW]
        self._saved = {41: jaw["accel"], 46: jaw["speed"], 48: jaw["effort"]}
        accel = min(jaw["accel"] or 5, 5)
        speed = min(jaw["speed"] or 60, 60)
        effort = min(jaw["effort"], maximum_effort, 100)
        goal = contact
        started = time.monotonic()
        phase = "零输出预载"
        self._path = contact, opened
        self._active = JAW

        def write(address, value):
            stop()
            if address == 42:
                result["last_attempted_goal"] = value
            self._write(JAW, address, value)

        def read_checked(expected_effort):
            stop()
            polled = time.monotonic()
            state = self._all()
            j = state[JAW]
            now = time.monotonic()
            sample = dict(stage=phase, elapsed=round(now-started, 3), read_ms=round((now-polled)*1000),
                          requested=goal, register_goal=j["goal"], actual=j["position"],
                          torque=j["torque"], effort=j["effort"], voltage_raw=j["voltage"], temp_c=j["temp"])
            result["samples"].append(sample)
            result["last_feedback"] = sample
            self._watch_passive(state, baseline, JAW)
            if ((j["torque"], j["goal"], j["effort"], j["speed"], j["accel"])
                    != (1, goal, expected_effort, speed, accel)):
                raise RuntimeError("夹爪使能、目标或输出参数发生变化。")
            if not (90 <= j["voltage"] <= 140 and j["temp"] < 60):
                raise RuntimeError("夹爪电压或温度读数异常。")
            if not self._path[0] - 3 <= j["position"] <= opened + 3:
                raise RuntimeError("夹指已超出本轮设定的张合区间。")
            return j["position"]

        def event(label, actual):
            result["events"].append(dict(stage=label, elapsed=round(time.monotonic()-started, 3),
                                         goal=goal, actual=actual))

        def move(label, destination, opening):
            nonlocal goal, phase
            phase = label
            print(label, flush=True)
            direction = 1 if opening else -1
            progress_at = time.monotonic()
            deadline = progress_at + 16
            best = read_checked(effort)
            recent = []
            quiet_contact = []
            while True:
                actual = read_checked(effort)
                now = time.monotonic()
                recent.append((now, actual))
                recent = [(t, pos) for t, pos in recent if now - t <= 1.8]
                if direction * (actual - best) >= 2:
                    best = actual
                    progress_at = now
                sufficient_opening = actual - contact >= 24
                if support_input is not None and not opening:
                    if (goal == destination and -TRACKING_TOLERANCE <= actual - contact <= TRACKING_TOLERANCE + 2
                            and result["first_measured_open"] - actual >= 17):
                        if any(abs(p - actual) > 2 for _, p in quiet_contact):
                            quiet_contact.clear()
                        quiet_contact.append((now, actual))
                        if len(quiet_contact) >= 3 and now - quiet_contact[0][0] >= .5:
                            event("合到剪柄附近并停稳，等待目视确认", actual)
                            result["candidate_contact_actual"] = actual
                            return actual
                    else:
                        quiet_contact.clear()
                elif (goal == destination and abs(destination - actual) <= TRACKING_TOLERANCE
                        and (not opening or sufficient_opening)):
                    event(label, actual)
                    return actual
                # A backed-off open pose only needs enough real clearance. Latch the observed
                # plateau and verify it before using that measured opening for the release.
                stable = len(recent) >= 3 and now-recent[0][0] >= .6 and max(p for _,p in recent)-min(p for _,p in recent) <= 2
                if opening and sufficient_opening and stable and now-progress_at >= 1.2:
                    write(42, actual)
                    goal = actual
                    latched = read_checked(effort)
                    if abs(latched-goal) > TRACKING_TOLERANCE or latched-contact < 24:
                        raise RuntimeError("实测开位锁存后发生移动。")
                    event(label + "（实测开位）", latched)
                    return latched
                if now >= deadline or now-progress_at >= 3.5:
                    raise TimeoutError(f"{label}跟随停止。目的位置 {destination}，最后指令 {goal}，"
                                       f"实测 {actual} 格，最近有效进展距今 {now-progress_at:.1f} 秒。")
                # Limit the ramp step, then bound target lead relative to measured feedback.
                planned = goal + max(-8, min(8, destination-goal))
                bounded = min(actual + MAX_TARGET_LEAD, max(actual - MAX_TARGET_LEAD, planned))
                # Wait for lagging feedback without reversing a monotonic segment.
                if (destination - goal) * (bounded - goal) > 0 and min(goal, destination) <= bounded <= max(goal, destination):
                    write(42, bounded)
                    goal = bounded
                time.sleep(.08)

        def dwell(seconds):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                if abs(read_checked(effort) - goal) > TRACKING_TOLERANCE + 2:
                    raise RuntimeError("保持期间夹指实际位置偏离目标。")
                time.sleep(.08)

        def read_seated(touch):
            actual = read_checked(effort)
            if (abs(actual - goal) > MAX_TARGET_LEAD or actual > touch + 2
                    or actual < touch - FIRM_PRELOAD - 2):
                raise RuntimeError("夹垫相对已确认的剪柄位置变化过大。请保留软垫承托，正在卸力。")
            return actual

        def apply_preload(touch):
            nonlocal goal, phase
            phase = "夹柄后微量收紧"
            self._path = firm_target, opened
            result.update(confirmed_touch_actual=touch, firm_target=firm_target)
            print(f"已确认贴柄。保持垫片承托，闭合目标从 {contact} 调到 {firm_target} 格。", flush=True)
            for next_goal in (contact - 2, firm_target):
                actual = read_seated(touch)
                if abs(actual - next_goal) > MAX_TARGET_LEAD:
                    raise RuntimeError("新目标与实测位置差值过大，停止收紧。")
                write(42, next_goal)
                goal = next_goal
                for _ in range(3):
                    read_seated(touch)
                    time.sleep(.08)
            stable = []
            for _ in range(5):
                time.sleep(.08)
                stable.append(read_seated(touch))
            if max(stable) - min(stable) > 2:
                raise RuntimeError("收紧后夹垫仍在移动，停止承重演示。")
            result["firm_actual"] = stable[-1]
            event("收紧位置已稳定", stable[-1])
            return stable[-1]

        def await_observation(key, message, seconds, reference=None, seated_touch=None):
            keys = support_input(key)
            next(keys)  # Discard any keys entered before this phase.
            print(message, flush=True)
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                actual = read_checked(effort) if seated_touch is None else read_seated(seated_touch)
                if (seated_touch is None and abs(actual - goal) > TRACKING_TOLERANCE + 2) or (reference is not None
                        and abs(actual - reference) > 2):
                    raise RuntimeError("夹指在现场确认期间偏离保持位置，正在卸力。")
                if next(keys):
                    return actual
                time.sleep(.08)
            raise TimeoutError(f"{seconds} 秒内未收到 {key} 确认，正在卸力。")

        try:
            write(48, 0)
            write(41, accel)
            write(46, speed)
            write(42, contact)  # Some firmware auto-enables on any position write.
            write(40, 0)
            pre = self._feedback(JAW)
            if (pre["torque"] or pre["effort"] or pre["goal"] != contact
                    or abs(pre["position"] - contact) > 2):
                raise RuntimeError("夹指零输出预载反馈异常。")
            write(40, 1)
            if abs(read_checked(0) - contact) > 2:
                raise RuntimeError("刚使能时夹指位置异常。")
            write(48, effort)
            if from_contact:
                release_target = opened
                phase = "保持现场摆好的轻贴位置"
                dwell(.6)
                closing_actual = read_checked(effort)
                if abs(closing_actual - contact) > 2:
                    raise RuntimeError("开始保持后夹口偏离了刚录下的贴柄位置，请留好承托。")
                event("已保持现场摆好的位置，等待目视确认", closing_actual)
            else:
                release_target = move("张开夹爪", opened, opening=True)
                result["first_measured_open"] = release_target
                phase = "开位保持"
                dwell(1)
                closing_actual = move("合到剪柄", contact, opening=False)
            phase = "接触位置保持"
            if support_input is None:
                print("夹持展示 3 秒", flush=True)
                dwell(3)
            else:
                phase = "目视核对接触"
                actual = await_observation("C",
                    f"实测已稳定在 {closing_actual} 格，目标 {contact} 格。先让薄垫继续承托练习剪。"
                    "若亲眼看清两块夹垫都贴住实心柄，输入 C 回车；有缝或看不清按 Q 停止。15 秒后自动卸力。",
                    15, closing_actual)
                result["contact_confirmation_received"] = True
                event("已收到 C 确认两垫贴柄", actual)
                touch = actual
                seated = apply_preload(touch)
                phase = "抽垫观察"
                event("开始抽垫观察", seated)
                actual = await_observation("R",
                    "现在只拉夹口外的长拉片，缓慢水平抽出薄垫。侧拍模型与下方软垫之间的间隙。"
                    "把薄垫送回模型下方，或确认模型已平稳落在软垫上，再输入 R 回车松开。"
                    "25 秒未确认将自动卸力，Ctrl+C 可随时卸力。", BEARING_SECONDS,
                    reference=seated, seated_touch=touch)
                result["support_restored_by_operator"] = True
                event("现场已确认模型有承托", actual)
            event("接触位置保持结束", read_checked(effort))
            move("张开释放", release_target, opening=True)
            phase = "释放位置保持"
            dwell(.35)
        finally:
            self._release()
            result.update(jaw_torque_off=True, temporary_settings_restored=True)
        final = self.passive()
        self._watch_passive(final, baseline, JAW)
        result["released_position"] = final[JAW]["position"]
        result["encoder_cycle_completed"] = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--simulate", action="store_true", help="memory-only rehearsal")
    ap.add_argument("--bearing", action="store_true", help="用带长拉片的薄垫演示固定夹爪承重")
    ap.add_argument("--from-contact", action="store_true",
                    help="从两垫已轻贴剪柄的位置直接做承重试验，承托复位后才张开")
    args = ap.parse_args()
    bearing = args.bearing or args.from_contact
    bus = FilmingJaw(simulate=args.simulate)
    result = dict(simulated=args.simulate, serial=SERIAL_NUMBER, encoder_cycle_completed=False,
                  jaw_torque_off=None)
    if bearing:
        result.update(mode="supported_bearing", support_restored_by_operator=False,
                      physical_outcome="unconfirmed", from_contact=args.from_contact)
    results = Path(__file__).resolve().parent / "results"
    results.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    log = results / ("film_grip_" + ("sim_" if args.simulate else "usb_") + stamp + ".json")
    result["utc"] = datetime.now(timezone.utc).isoformat()
    try:
        bus.connect()
        result["port"] = bus.port_name
        initial = bus.setup_snapshot(result)
        print(f"已连接 {bus.port_name}，夹爪和其余八轴均未使能。", flush=True)
        confirmed = None
        if not args.simulate:
            if not sys.stdin.isatty():
                raise RuntimeError("需要在现场交互窗口启动。")
            if bearing:
                if args.from_contact:
                    print("贴柄起步：现在摆好两垫与实心剪柄的接触位置。按提示确认后将微收紧，承托复位后再张开。")
                print("承重镜头：用固定软垫承接整个轻质模型，软垫上放一片约 2～4 毫米厚的薄垫。"
                      "薄垫的长拉片伸到机械臂外，拉片能平顺抽出和放回。")
                print("练习剪只由薄垫承托；腕和前臂另有稳固支撑。超时或停止会卸力，模型可落在正下方软垫上。")
            print("手机开始录像。稳固承托空臂和无刃练习模型，让两块夹垫刚好轻贴实心柄段。\n"
                  "模型在夹爪松开时也要自行稳住；两侧整个张开范围留空。手和衣袖离开，电源开关可及。"
                  "运动时按 Ctrl+C 可停止。", flush=True)
            confirmed = bus.prepare_contact(result, initial, bearing=bearing)
        with motion_stop() as stop:
            support_input = None
            if bearing:
                support_input = (lambda key: iter((False, False, True))) if args.simulate else support_keys
            bus.run_cycle(result, stop, support_input, confirmed, from_contact=args.from_contact)
    except (Exception, KeyboardInterrupt) as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        print(result["error"], flush=True)
    finally:
        try:
            bus.close()
        except Exception as exc:
            result["close_error"] = str(exc)
            print(f"停止状态未确认，请关闭右侧电机电源。{exc}", flush=True)
        log.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print("演示记录", log, flush=True)
    if result["encoder_cycle_completed"] and "close_error" not in result:
        print("夹爪已完成张合，卸力回读正常。请从视频确认夹垫与模型的接触。", flush=True)
        if bearing and not args.simulate and result["jaw_torque_off"]:
            print("现场观察：整把练习剪是否在垫片完全抽出后悬空、与软垫和其他物体均有间隙？")
            try:
                answer = input("1 看清悬空    2 滑落或仍有接触    回车 未确认：").strip()
            except (EOFError, KeyboardInterrupt):
                answer = ""
            result["physical_outcome"] = {"1": "operator_observed_unsupported_prop",
                                          "2": "operator_observed_slip_or_contact"}.get(answer, "unconfirmed")
            log.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
