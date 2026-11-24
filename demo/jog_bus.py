"""Bounded manual point-to-point jogging for the right USB arm. All calls use one worker."""
import time


JOINT_NAMES = {1: "底座旋转", 2: "肩部俯仰", 3: "肘部屈伸", 4: "腕部俯仰", 5: "腕部旋转", 6: "夹爪"}
MAX_STEP = {i: 24 if i == 6 else 57 for i in JOINT_NAMES}
# Right board 5AAF288004, matched between the October Windows and November O6 readbacks.
# (stored min, stored max, raw homing offset). These are never written.
PROFILE = {1: (1115, 2965, 898), 2: (909, 3198, 210), 3: (906, 3095, 904),
           4: (907, 3217, 275), 5: (123, 3971, 1028), 6: (705, 924, 154)}
BUS_IDS = range(1, 10)
MARGIN = 20
# The archived right gripper run reached 815/807/801 for goals 812/804/796.
# Allow observed lag, but require real encoder progress on every new goal.
TRACKING_TOLERANCE = 5
RAM_WIDTH = {40: 1, 41: 1, 42: 2, 46: 2, 48: 2}


class UnresolvedStep(ValueError):
    """Small requested increment left no conclusive motion; output is verified off."""


class _SerialLink:
    def __init__(self):
        import serial
        from right_table_state import read_register
        from teach_grip_points import controller_port

        self.read_register = read_register
        self.name = controller_port(True)
        try:
            self.port = serial.Serial(self.name, 1_000_000, timeout=.08, write_timeout=.08,
                                      exclusive=True)
        except OSError as exc:
            raise OSError(f"{self.name} 打开失败。请先关闭占用右臂串口的示教窗口或串口助手。{exc}") from exc

    def read(self, motor, address, size):
        return self.read_register(self.port, motor, address, size).to_bytes(size, "little")

    def write(self, motor, address, value):
        size = RAM_WIDTH[address]
        body = bytes([motor, size + 3, 3, address]) + value.to_bytes(size, "little")
        self.port.reset_input_buffer()
        self.port.write(b"\xff\xff" + body + bytes([(~sum(body)) & 255]))
        if self.port.read(6) != bytes([255, 255, motor, 2, 0, (~(motor + 2)) & 255]):
            raise OSError(f"ID{motor} 写入寄存器 {address} 时未收到有效应答。")

    def close(self):
        self.port.close()

    def write_goals(self, goals):
        """Feetech SYNC_WRITE, position register only. SYNC_WRITE has no reply."""
        if not goals or any(type(i) is not int or i not in PROFILE or
                            type(v) is not int or not PROFILE[i][0] <= v <= PROFILE[i][1]
                            for i, v in goals.items()):
            raise ValueError("同步目标包含未确认的右臂关节或位置。")
        payload = b"".join(bytes([i]) + v.to_bytes(2, "little") for i, v in goals.items())
        body = bytes([254, 4 + len(payload), 0x83, 42, 2]) + payload
        packet = b"\xff\xff" + body + bytes([(~sum(body)) & 255])
        if self.port.write(packet) != len(packet):
            raise OSError("同步目标数据未完整写入。")


class _SimulatedLink:
    """Memory register bank; the simulated servo reproduces goal write auto-enable."""
    name = "离线模拟"

    def __init__(self):
        self.registers = {i: bytearray(74) for i in BUS_IDS}
        for i in BUS_IDS:
            low, high, offset = PROFILE.get(i, (0, 4095, 0))
            effort = 500 if i == 6 else 1000
            start = (low + high) // 2
            for address, size, value in (
                (3, 2, 777), (9, 2, low), (11, 2, high), (16, 2, effort), (18, 1, 12),
                (31, 2, offset), (33, 1, int(i > 6)), (42, 2, start), (48, 2, effort),
                (56, 2, start), (62, 1, 123), (63, 1, 34)
            ):
                self.put(i, address, size, value)

    def put(self, motor, address, size, value):
        self.registers[motor][address:address + size] = value.to_bytes(size, "little")

    def read(self, motor, address, size):
        return bytes(self.registers[motor][address:address + size])

    def write(self, motor, address, value):
        self.put(motor, address, RAM_WIDTH[address], value)
        if address == 42:
            self.put(motor, 40, 1, 1)
            if any(self.read(motor, 48, 2)):
                self.put(motor, 56, 2, value)

    def close(self):
        pass

    def write_goals(self, goals):
        for i, v in goals.items():
            self.write(i, 42, v)


def _word(data, offset):
    return int.from_bytes(data[offset:offset + 2], "little")


class JogBus:
    """One connection, owned by one worker. stop_event can be set from the UI thread."""
    def __init__(self, simulate=False):
        self.simulate = simulate
        self.port_name = ""
        self._link = None
        self._active = None
        self._path = ()
        self._saved = {}
        self._identity_issues = {}
        self._fault = ""

    def _number(self, motor, addr, width):
        return int.from_bytes(self._link.read(motor, addr, width), "little")

    def _identity_issue(self, motor):
        block = self._link.read(motor, 3, 31)
        if (_word(block, 0) != 777 or block[30] != int(motor > 6) or block[15] != 12):
            return "电机型号、工作模式或相位与右臂记录不符"
        if motor in PROFILE:
            calibration = _word(block, 6), _word(block, 8), _word(block, 28)
            if calibration != PROFILE[motor]:
                return "限位或零位偏置与本机记录不符"
        return ""

    def connect(self):
        if self._link is not None:
            raise RuntimeError("连接已打开。")
        self._link = _SimulatedLink() if self.simulate else _SerialLink()
        self.port_name = self._link.name
        self._fault = ""
        try:
            self._identity_issues = {i: self._identity_issue(i) for i in BUS_IDS}
            if any(self._identity_issues[i] for i in (7, 8, 9)):
                raise RuntimeError("右侧总线的底盘电机身份或模式不符，请检查 USB 接线。")
            return self.snapshot()
        except BaseException:
            self.close()
            raise

    def _feedback(self, i):
        # One validated reply provides a time-consistent target, encoder and enable flag.
        b = self._link.read(i, 40, 24)
        return dict(torque=b[0], accel=b[1], goal=_word(b, 2), speed=_word(b, 6),
                    effort=_word(b, 8), position=_word(b, 16), voltage=b[22], temp=b[23])

    def _all(self):
        if self._link is None:
            raise RuntimeError("请先连接右臂。")
        return {i: self._feedback(i) for i in BUS_IDS}

    def _rows(self, states):
        enabled = [str(i) for i in BUS_IDS if states[i]["torque"] != 0]
        rows = {}
        for i in JOINT_NAMES:
            s = states[i]
            low, high, _ = PROFILE[i]
            low, high = low + MARGIN, high - MARGIN
            reason = self._fault or self._identity_issues[i]
            if not reason:
                if enabled:
                    reason = "已有电机使能，请先结束其他控制程序（ID " + "、".join(enabled) + "）"
                elif not low <= s["position"] <= high:
                    reason = f"当前读数在点动范围 {low}–{high} 以外"
                elif not 90 <= s["voltage"] <= 140 or s["temp"] >= 60:
                    reason = "当前电压或温度超出调试范围"
                elif not 0 < s["effort"] <= 1000:
                    reason = "当前力矩上限异常"
            rows[i] = dict(position=s["position"], minimum=low, maximum=high,
                           torque=s["torque"], ready=not reason, reason=reason)
        return rows

    def snapshot(self):
        """Pure read. No motor is ever enabled by connecting or refreshing."""
        return self._rows(self._all())

    def _write(self, i, address, value):
        if i != self._active or i not in JOINT_NAMES or address not in RAM_WIDTH:
            raise ValueError("写入超出当前关节或允许的 RAM 寄存器。")
        if address == 40 and value not in (0, 1):
            raise ValueError("无效的使能标志。")
        if address == 42:
            low, high, _ = PROFILE[i]
            if not low + MARGIN <= value <= high - MARGIN or not self._path[0] <= value <= self._path[1]:
                raise ValueError("目标超出点动范围。")
        self._link.write(i, address, value)
        if self._number(i, address, RAM_WIDTH[address]) != value:
            raise OSError(f"ID{i} 寄存器 {address} 回读值不符。")

    def _release(self):
        """Restore effort last, only with torque demonstrably OFF."""
        i = self._active
        if i is None:
            return
        try:
            for _ in range(3):
                try:
                    self._write(i, 40, 0)
                    break
                except OSError:
                    # A lost acknowledgement does not tell us whether the write took effect.
                    try:
                        if self._number(i, 40, 1) == 0:
                            break
                    except OSError:
                        pass
            else:
                raise OSError("力矩关闭回读失败")
            for addr in (41, 46, 48):
                if self._number(i, 40, 1) != 0:
                    raise OSError("还原临时参数时使能标志发生变化")
                self._write(i, addr, self._saved[addr])
            if self._number(i, 40, 1) != 0:
                raise OSError("还原后使能标志异常")
        except Exception as exc:
            self._fault = f"ID{i} 停机状态或 RAM 恢复无法核实，请立即关闭右侧电机电源。"
            for addr in (48, 40):
                try:
                    self._write(i, addr, 0)
                except Exception:
                    pass
            raise RuntimeError(self._fault) from exc
        self._active, self._path, self._saved = None, (), {}

    @staticmethod
    def _watch_passive(states, baseline, active):
        for i, row in states.items():
            if i == active:
                continue
            delta = (row["position"] - baseline[i]["position"] + 2048) % 4096 - 2048
            if row["torque"] or abs(delta) > 6:
                raise RuntimeError(f"ID{i} 的使能或位置发生变化：实测 {baseline[i]['position']} → "
                                   f"{row['position']} 格（{delta:+d} 格），使能标志 {row['torque']}。"
                                   "请检查机械支撑。")

    def move(self, motor_id, target_raw, stop_event, expected_raw=None):
        """Move one measured increment. No queue, dwell/hold or retry on a movement error."""
        if type(motor_id) is not int or motor_id not in JOINT_NAMES or type(target_raw) is not int:
            raise ValueError("请选择右臂 ID1–6 和整数编码器目标。")
        i = motor_id
        if self._active is not None or self._fault:
            raise RuntimeError(self._fault or "上一动作尚未结束。")
        deadline = time.monotonic() + 8

        def check():
            if stop_event.is_set():
                raise InterruptedError("点动已停止。")
            if time.monotonic() >= deadline:
                raise TimeoutError("点动超时。")

        check()
        baseline = self._all()
        self._identity_issues[i] = self._identity_issue(i)
        row = self._rows(baseline)[i]
        if not row["ready"]:
            raise ValueError(row["reason"])
        start = row["position"]
        if expected_raw is not None:
            if (type(expected_raw) is not int or abs(start - expected_raw) > 2
                    or (target_raw - expected_raw) * (target_raw - start) < 0):
                raise ValueError("关节位置与界面显示已不同，请刷新读数后重新点动。")
        if not row["minimum"] <= target_raw <= row["maximum"] or abs(target_raw - start) > MAX_STEP[i]:
            raise ValueError(f"ID{i} 单次最多 {MAX_STEP[i]} 刻度，目标须在 {row['minimum']}–{row['maximum']}。")
        if start == target_raw:
            return self._rows(baseline)
        if stop_event.wait(.08):
            check()
        still = self._all()
        self._watch_passive(still, baseline, None)
        if abs(still[i]["position"] - start) > 1:
            raise RuntimeError("点动前关节仍在移动，请先稳固承托。")
        s = still[i]
        if not self._rows(still)[i]["ready"]:
            raise RuntimeError("点动前状态发生变化，请刷新。")
        self._saved = {41: s["accel"], 46: s["speed"], 48: s["effort"]}
        accel = min(s["accel"] or 5, 5)
        speed = min(s["speed"] or 60, 60)
        effort = min(s["effort"], 100)
        self._active = i
        self._path = min(start, target_raw), max(start, target_raw)
        tracking_warning = ""

        def write(address, value):
            check()
            self._write(i, address, value)

        try:
            # Some STS firmware enables torque on ANY goal write, even at the same position.
            write(48, 0)
            write(41, accel)
            write(46, speed)
            write(42, start)
            write(40, 0)
            preloaded = self._feedback(i)
            if (preloaded["torque"] or preloaded["effort"] or preloaded["goal"] != start
                    or abs(preloaded["position"] - start) > 2):
                raise RuntimeError("零输出准备时的关节位置、目标或使能状态发生变化。")
            write(40, 1)  # Output limit is still zero.
            armed = self._all()
            self._watch_passive(armed, baseline, i)
            feedback = armed[i]
            if ((feedback["torque"], feedback["goal"], feedback["effort"], feedback["speed"],
                 feedback["accel"]) != (1, start, 0, speed, accel)
                    or abs(feedback["position"] - start) > 2):
                raise RuntimeError("使能后目标或临时参数发生变化，已停止输出。")
            write(48, effort)
            bounded = self._feedback(i)
            if (bounded["torque"] != 1 or bounded["goal"] != start or bounded["effort"] != effort
                    or abs(bounded["position"] - start) > 2):
                raise RuntimeError("设置点动输出后目标或关节状态发生变化。")
            goal = start
            direction = 1 if target_raw > start else -1
            previous_feedback = start
            while goal != target_raw:
                goal += direction * min(8, abs(target_raw - goal))
                write(42, goal)
                reach_by = min(deadline, time.monotonic() + .85)
                while True:
                    stop_event.wait(.04)
                    check()
                    measured = self._all()
                    self._watch_passive(measured, baseline, i)
                    m = measured[i]
                    if ((m["goal"], m["torque"], m["effort"], m["speed"], m["accel"]) !=
                            (goal, 1, effort, speed, accel) or m["temp"] >= 60
                            or not 90 <= m["voltage"] <= 140):
                        raise RuntimeError(f"ID{i} 的反馈或输出参数异常。")
                    if not min(start, goal) - 3 <= m["position"] <= max(start, goal) + 3:
                        raise RuntimeError(f"ID{i} 超出了点动区间。")
                    tolerance = min(TRACKING_TOLERANCE, abs(goal - start) - 1)
                    if (direction * (m["position"] - previous_feedback) > 0
                            and abs(m["position"] - goal) <= tolerance):
                        previous_feedback = m["position"]
                        break
                    if time.monotonic() >= reach_by:
                        message = f"ID{i} 目标 {goal}，实测 {m['position']}；关节未跟上。"
                        if abs(target_raw - start) <= 5 or (goal == target_raw and abs(m["position"] - goal) <= 5):
                            raise UnresolvedStep(message)
                        raise RuntimeError(message)
        except UnresolvedStep as exc:
            tracking_warning = str(exc)
        finally:
            self._release()
        stop_event.wait(.15)
        settled = self._all()
        self._watch_passive(settled, baseline, i)
        actual = settled[i]["position"]
        if settled[i]["torque"] or not min(start, target_raw) - 3 <= actual <= max(start, target_raw) + 3:
            raise RuntimeError(f"ID{i} 停止后出现意外使能或漂移，当前读数 {actual}。")
        check()
        if tracking_warning:
            raise UnresolvedStep(f"{tracking_warning} 已核对卸力，停在 {actual}。请检查支撑与间隙。")
        if direction * (actual - start) <= 0 or abs(actual - target_raw) > min(TRACKING_TOLERANCE, abs(target_raw - start) - 1):
            if abs(target_raw - start) <= 5:
                raise UnresolvedStep(f"ID{i} 已卸力，目标 {target_raw}，实测 {actual}；未能保持该微小位移。")
            raise RuntimeError(f"卸力后目标 {target_raw}，实测 {actual}，请检查承托。")
        return self._rows(settled)

    def close(self):
        if self._link is not None:
            try:
                self._release()
            finally:
                self._link.close()
                self._link = None
                self.port_name = ""
                self._active, self._path, self._saved = None, (), {}
