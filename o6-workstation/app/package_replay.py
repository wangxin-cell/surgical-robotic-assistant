"""Host the supplied PowerShell forward player on the O6's exclusive serial lease."""
import base64
import os
from pathlib import Path
from queue import Empty, Queue
import shutil
import subprocess
import threading
import time

from native_motor import in_range, lease, read_quiet, snapshot
from native_pick import RECORDINGS, ReplayStopped, check_held, hold_on_failure, plan_for, stop_check, wait_for_recorded_step

HERE = Path(__file__).resolve().parent
PWSH = Path.home() / "o6_workstation/tools/powershell-7.4.22/pwsh"


class Interrupted(RuntimeError):
    def __init__(self, reason, cursor):
        super().__init__(str(reason))
        self.cursor = cursor


class Cursor:
    def __init__(self, slot, goals):
        self.slot = slot
        self.done = 0
        self.expected = list(goals)
        self.pending = None
        self.parked = None
        self.total = len(plan_for(slot)[1])

    def commit(self):
        axis, target = self.pending
        self.expected[axis - 1] = target
        if self.parked:
            self.parked[axis-1]["goal"] = target
        self.done += 1
        self.pending = None


def check_restart(bus, cursor, stop):
    joints = read_quiet(bus, snapshot(bus))
    for j, goal, parked in zip(joints, cursor.expected, cursor.parked):
        stop_check(stop)
        check_held(j, parked["goal"], 36)
        if abs(j["position"] - parked["position"]) > 30 or abs(j["position"] - goal) > 30:
            raise ValueError(f"ID{j['id']} 与断点姿态不符，实测 {j['position']}，断点目标 {goal}。")
    return joints


def exchange(bus, fields, cursor, actions, touched, stop, report):
    op, *args = fields
    stop_check(stop)
    if op == "open":
        return ""
    if op == "confirm":
        # The caller already holds the workbench start/resume authorization.
        return "mayo-scissors-straight"
    if op == "wait":
        delay = int(args[0])
        if not 60 <= delay <= 300:
            raise ValueError("原程序返回无效步间隔。")
        if not bus.simulated:
            stop.wait(delay / 1000)
        return ""
    if op == "read":
        axis, address, count = map(int, args)
        if axis not in range(1, 7) or (address,count) not in ((9,4),(56,2),(40,1),(33,1),(42,2)):
            raise ValueError("原程序请求了未知寄存器。")
        data = bus.read(axis, address, count)
        if cursor.pending and address == 42 and cursor.pending[0] == axis:
            if int.from_bytes(data, "little") != cursor.pending[1]:
                raise RuntimeError(f"ID{axis} 指令回读不符。")
            cursor.commit()
            report(f"点位 {cursor.slot} · 已确认 {cursor.done}/{cursor.total} 条", cursor.done / cursor.total)
        return base64.b64encode(data).decode()
    if op != "write" or len(args) != 4 or cursor.pending or cursor.done >= len(actions):
        raise ValueError("回放程序的指令顺序异常。")
    axis, opcode, address = map(int, args[:3])
    packet = base64.b64decode(args[3], validate=True)
    _, expected_axis, target, _ = actions[cursor.done]
    if (axis, opcode, address, packet) != (expected_axis, 3, 41, bytes((5,target & 255,target >> 8,0,0,45,0))):
        raise ValueError("原程序发出的目标与对应录制行不一致。")
    j = bus.joint(axis)
    # On resume a stopped axis may be holding its measured stop position.
    held = cursor.parked[axis-1]["goal"] if cursor.parked else cursor.expected[axis-1]
    wait_for_recorded_step(bus, j, held, target, stop)
    stop_check(stop)
    touched.add(axis)
    cursor.pending = (axis,target)
    bus.send(axis,opcode,address,packet)
    return ""


def run_script(bus, path, cursor, actions, touched, stop, report):
    exe = shutil.which("pwsh") if os.name == "nt" else str(PWSH)
    args = [exe, "-NoLogo", "-NoProfile", "-NonInteractive", "-File", str(HERE/"o6_replay.ps1"),
            "-Episode", str(path), "-Offset", str(cursor.done), "-Pose",
            ",".join(str(j["position"]) for j in cursor.parked) if cursor.done and cursor.parked else "original"]
    options = {"creationflags": subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {}
    with subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, encoding="utf-8", errors="replace", bufsize=1, **options) as proc:
        lines = Queue()
        def read_output():
            try:
                for line in proc.stdout:
                    lines.put(line.rstrip())
            finally:
                lines.put(None)
        reader = threading.Thread(target=read_output, daemon=True)
        reader.start()
        last = time.monotonic()
        messages = []
        try:
            while True:
                stop_check(stop)
                try:
                    line = lines.get(timeout=.10)
                except Empty:
                    if time.monotonic() - last > 25:
                        raise TimeoutError("原回放进程超过 25 秒没有响应。")
                    continue
                last = time.monotonic()
                if line is None:
                    break
                if line.startswith("BUS "):
                    reply = exchange(bus, line[4:].split(), cursor, actions, touched, stop, report)
                    proc.stdin.write("OK " + reply + "\n")
                    proc.stdin.flush()
                else:
                    messages = (messages + [line])[-12:]
            if proc.wait(timeout=3) or cursor.done != len(actions):
                raise RuntimeError("原回放程序中止。" + " ".join(messages)[-1000:])
        finally:
            if proc.poll() is None:
                proc.kill()
            proc.wait()
            reader.join(timeout=2)


def verify_end(bus, plan, cursor, stop):
    deadline = time.monotonic() + 4
    while True:
        stop_check(stop)
        joints = snapshot(bus)
        for j, goal in zip(joints,cursor.parked or [{"goal": g} for g in cursor.expected]):
            check_held(j, goal["goal"], 36)
        if all(not j["moving"] and abs(j["position"] - end["position"]) <= 24 for j,end in zip(joints,plan[2])):
            read_quiet(bus,joints)
            stop_check(stop)
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("路径指令已全部确认，终点姿态尚未核实。")
        stop.wait(.08)


def play(bus, slot, stop, report, verify, cursor=None):
    plan = plan_for(slot)
    actions = plan[1]
    path = HERE/"arm_tools"/"pick_points"/f"left_manual_instrument_pick_{RECORDINGS[slot][0]}.jsonl"
    if cursor is not None:
        if cursor.slot != slot or cursor.total != len(actions):
            raise ValueError("断点与所选路径不符。")
        try:
            check_restart(bus,cursor,stop)
        except Exception as exc:
            raise Interrupted(exc,cursor) from exc
    else:
        cursor = Cursor(slot, [j["goal"] for j in read_quiet(bus,snapshot(bus))])
    verify()
    touched = set()
    try:
        with hold_on_failure(bus) as touched:
            try:
                if cursor.done < len(actions):
                    run_script(bus,path,cursor,actions,touched,stop,report)
                verify_end(bus,plan,cursor,stop)
            except Exception:
                # Resolve an interrupted write before parking changes the register.
                if cursor.pending:
                    axis, target = cursor.pending
                    value = bus.number(axis,42,2)
                    old = cursor.parked[axis-1]["goal"] if cursor.parked else cursor.expected[axis-1]
                    if value == target:
                        cursor.commit()
                    elif value == old:
                        cursor.pending = None
                raise
    except Exception as exc:
        if cursor.pending:
            raise RuntimeError(f"{exc} 上一条写入的回读结果未知，无法确定断点。") from exc
        try:
            parked = snapshot(bus)
        except Exception:
            # When communication returns, require every original goal to still
            # match. Never adopt an arbitrary live pose as a known breakpoint.
            parked = [dict(position=g,goal=g) for g in cursor.expected]
        # At step 1 the package itself will check against the original start.
        cursor.parked = parked
        raise Interrupted(exc,cursor) from exc
    return f"点位 {slot} 的 {cursor.total} 条原始指令已确认，终点姿态已核对。"


def replay(slot,stop,report,verify,cursor=None):
    acquired = False
    try:
        with lease("left",pick=True) as bus:
            acquired = True
            return play(bus,slot,stop,report,verify,cursor)
    except Exception as exc:
        if cursor is not None and not acquired:
            raise Interrupted(exc,cursor) from exc
        raise

