"""One supported right-jaw contact on a blunt training prop, using a fresh observed pair of stops."""
import base64
import json
import os
import signal
import socket
import statistics
import sys
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

import serial
from right_table_state import SERIAL_NUMBER, read_register


MINIMUM, MAXIMUM = 705, 924
SETTINGS = {41: (1, 5), 46: (2, 60), 48: (2, 100)}
WIDTHS = {40: 1, 41: 1, 42: 2, 46: 2, 48: 2}
_retained_callbacks = []  # Keep the native callback alive if Windows reports unregister failure.


def no_stop():
    pass


@contextmanager
def motion_stop():
    if os.name != "nt":
        yield no_stop  # The SSH process uses Python signal handlers on Linux.
        return
    import ctypes
    import threading
    from ctypes import wintypes

    requested, finished = threading.Event(), threading.Event()
    routine = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.DWORD)
    install = ctypes.WinDLL("kernel32", use_last_error=True).SetConsoleCtrlHandler
    install.argtypes = (routine, wintypes.BOOL)
    install.restype = wintypes.BOOL

    @routine
    def handler(event):
        if event not in (0, 1, 2, 5, 6):  # CTRL-C, BREAK, CLOSE, LOGOFF, SHUTDOWN
            return False
        requested.set()
        # Closing a console can end its processes as soon as this handler returns.
        # Only the main thread touches the serial port. Give it a bounded cleanup interval.
        finished.wait(4)
        return True

    def check():
        if requested.is_set():
            raise InterruptedError("Console stop requested.")

    if not install(handler, True):
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        yield check
    finally:
        finished.set()
        if not install(handler, False):
            _retained_callbacks.append(handler)
            raise ctypes.WinError(ctypes.get_last_error())


def taught_endpoints(report):
    if (report.get("serial") != SERIAL_NUMBER or report.get("host") != socket.gethostname()
            or report.get("completed") is not True or report.get("read_only") is not True
            or report.get("error") or report.get("cancelled")):
        raise ValueError("A complete passive teaching record from this host and right controller is required.")
    stages = report["stages"]
    poses = {}
    for name in ("empty_pregrasp", "handle_contact", "open_at_grasp"):
        stage = stages[name]
        age = (datetime.now(timezone.utc) - datetime.fromisoformat(stage["utc"])).total_seconds()
        if not 0 <= age <= 900:
            raise ValueError(f"{name} is outside the 15-minute teaching window.")
        samples = stage["samples"]
        if (len(samples) != 3 or stage["id6_limits"] != [MINIMUM, MAXIMUM]
                or any(len(s["positions"]) != 9 or s["torques"] != [0] * 9 for s in samples)):
            raise ValueError(f"{name}: unexpected samples, torque state or jaw limits")
        columns = list(zip(*(s["positions"] for s in samples)))
        if any(type(v) is not int or not 0 <= v <= 4095 for col in columns for v in col):
            raise ValueError(f"{name}: invalid raw encoder position")
        pose = [int(statistics.median(c)) for c in columns]
        if pose != stage["median_by_id_1_to_9"] or any(max(c) - min(c) > 6 for c in columns):
            raise ValueError(f"{name}: teaching samples moved or disagree with the stored pose")
        poses[name] = pose
    contact = poses["handle_contact"]
    opened = poses["open_at_grasp"]
    if max(abs(a-b) for a, b in zip(contact[:5], opened[:5])) > 6:
        raise ValueError("Arm pose changed between handle contact and opening.")
    if not (MINIMUM+20 <= contact[5] < opened[5] <= MAXIMUM-20 and 7 <= opened[5]-contact[5] <= 24):
        raise ValueError(f"Contact {contact[5]}, open {opened[5]}: need 7..24 closing ticks and both within 725..904.")
    return opened, contact[5]


def check_passive(port, expected, jaw_enable, stop=no_stop):
    def read(motor, address, size):
        stop()
        return read_register(port, motor, address, size)
    flags = [read(i, 40, 1) for i in range(1, 10)]
    if flags != [0, 0, 0, 0, 0, jaw_enable, 0, 0, 0]:
        raise RuntimeError(f"Unexpected torque enables: {flags}")
    arm = [read(i, 56, 2) for i in range(1, 6)]
    if any(abs(a-b) > 6 for a, b in zip(arm, expected[:5])):
        raise RuntimeError(f"Supported arm moved: live {arm}, taught {expected[:5]}")


def plan(port, teach):
    opened, target = taught_endpoints(teach)
    registers = [(3, 2, 777), (33, 1, 0), (9, 2, MINIMUM), (11, 2, MAXIMUM), (31, 2, 154), (18, 1, 12)]
    for address, width, required in registers:
        if read_register(port, 6, address, width) != required:
            raise RuntimeError(f"Right jaw identity, operating mode or calibration mismatch at register {address}.")
    starts = []
    for _ in range(2):
        check_passive(port, opened, 0)
        starts.append(read_register(port, 6, 56, 2))
        time.sleep(.05)
    if (abs(starts[0]-starts[1]) > 3 or any(abs(v-opened[5]) > 3 or not 0 < v-target <= 24
            or not MINIMUM+20 <= v <= MAXIMUM-20 for v in starts)):
        raise RuntimeError(f"Jaw moved since teaching: live {starts}, taught open {opened[5]}, contact {target}.")
    return opened, starts[-1], target


def write_ram(port, address, value):
    # The device ID and the only writable register widths are fixed here.
    width = WIDTHS[address]
    if address == 40 and value not in (0, 1):
        raise ValueError("Invalid torque enable")
    if address == 42 and not MINIMUM+20 <= value <= MAXIMUM-20:
        raise ValueError("Goal outside supported jaw window")
    body = bytes([6, 3+width, 3, address]) + value.to_bytes(width, "little")
    port.reset_input_buffer()
    port.write(b"\xff\xff" + body + bytes([(~sum(body)) & 255]))
    if port.read(6) != b"\xff\xff\x06\x02\x00\xf7":
        raise OSError(f"Missing ID6 acknowledgement for register {address}")
    if read_register(port, 6, address, width) != value:
        raise OSError(f"ID6 readback mismatch at register {address}")


def release(port, original, result):
    for _ in range(3):
        try:
            write_ram(port, 40, 0)
        except OSError:
            pass  # A command with a lost acknowledgement may already have disabled torque.
        try:
            if read_register(port, 6, 40, 1) == 0:
                break
        except OSError:
            pass
    else:
        raise RuntimeError("Unable to verify ID6 torque off. Switch off RIGHT motor power now.")
    result["jaw_torque_off"] = True
    failures = []
    for address, value in original.items():
        try:
            write_ram(port, address, value)
        except (OSError, ValueError) as exc:
            failures.append(str(exc))
    if failures:
        raise RuntimeError("Jaw disabled; RAM restoration incomplete: " + ", ".join(failures))
    result["temporary_settings_restored"] = True


def run(port, report, execute, result, validate_source=no_stop):
    opened, start, target = plan(port, report)
    result.update(start_raw=start, taught_contact_raw=target, movement_requested=execute)
    print(f"Right ID6 only. Start {start}, taught open {opened[5]}, just-contact goal {target}.", flush=True)
    if not execute:
        print("Read-only plan completed. Motor registers were not changed.", flush=True)
        return
    if not sys.stdin.isatty():
        raise RuntimeError("An operator at an interactive terminal is required.")
    prompt = ("无刃练习剪和右臂须各自有稳固支撑，夹指合拢时两侧都不碰台面。"
              "手和衣袖离开，留在右侧电机电源开关旁。若条件满足，输入 YES 并回车，"
              "只让右夹指合到示教接触位置，随后解除力矩。其余输入将停止。"
              if os.name == "nt" else
              "Keep the same BLUNT unbladed prop and arm independently supported. Clear hands and sleeves, "
              "stay by the RIGHT motor switch. Type YES then Enter for one small jaw closure and release.")
    print(prompt, flush=True)
    if input().strip() != "YES":
        raise RuntimeError("Operator did not confirm supported pinch.")
    validate_source()
    opened, start, target = plan(port, report)
    original = {a: read_register(port, 6, a, w) for a, (w, _) in SETTINGS.items()}
    if original[48] == 0:
        raise RuntimeError("Existing zero torque limit; leave motor disabled.")
    settings = {a: min(original[a] or cap, cap) for a, (_, cap) in SETTINGS.items()}
    result.update(start_raw=start, samples=[])
    with motion_stop() as stop:
        try:
            move_to_contact(port, opened, start, target, settings, result, stop)
        finally:
            if os.name == "nt":
                release(port, original, result)  # Console handler waits for this block.
            else:
                signals = {signal.SIGINT, signal.SIGTERM, signal.SIGHUP}
                previous = signal.pthread_sigmask(signal.SIG_BLOCK, signals)
                try:
                    release(port, original, result)
                finally:
                    signal.pthread_sigmask(signal.SIG_SETMASK, previous)


def move_to_contact(port, opened, start, target, settings, result, stop):
    stop()
    write_ram(port, 42, start)
    for address, value in settings.items():
        stop()
        write_ram(port, address, value)
    check_passive(port, opened, 0, stop)
    if abs(read_register(port, 6, 56, 2)-start) > 2:
        raise RuntimeError("Jaw moved after goal preload.")
    stop()
    write_ram(port, 40, 1)
    if read_register(port, 6, 42, 2) != start or any(
        read_register(port, 6, a, WIDTHS[a]) != value for a, value in settings.items()
    ):
        raise RuntimeError("Goal or effort/speed/acceleration changed when torque was enabled.")
    requested = start
    while requested > target:
        check_passive(port, opened, 1, stop)
        stop()
        requested = max(target, requested-8)
        write_ram(port, 42, requested)
        time.sleep(.22)
        stop()
        position = read_register(port, 6, 56, 2)
        result["samples"].append([requested, position])
        if not target-2 <= position <= start+2 or abs(position-requested) > 8:
            raise RuntimeError(f"Tracking departed from bounded jaw path at {requested}, observed {position}.")
    time.sleep(.16)
    stop()
    position = read_register(port, 6, 56, 2)
    check_passive(port, opened, 1, stop)
    result["last_position_raw"] = position
    if abs(position-target) > 3 or start-position < 4:
        raise RuntimeError("Encoder did not reach the just-contact reference.")
    result["encoder_reached_taught_reference"] = True


def interrupt(signum, _frame):
    raise InterruptedError(f"Stopped by signal {signum}")


def latest_teaching(results):
    candidates = []
    for directory in results.glob("passive_teach_*"):
        if not directory.is_dir():
            continue
        path = directory / "result.json"
        pending = directory / "result.tmp"
        if pending.exists():
            candidates.append((pending.stat().st_mtime_ns, None))
        if path.is_file():
            candidates.append((path.stat().st_mtime_ns, path))
        elif not pending.exists():
            candidates.append((directory.stat().st_ctime_ns, None))  # Windows creation time
    if not candidates:
        raise ValueError("No passive teaching record on this computer.")
    newest = max(stamp for stamp, _ in candidates)
    matches = [path for stamp, path in candidates if stamp == newest]
    if len(matches) != 1 or matches[0] is None:
        raise ValueError("Latest teaching session is unfinished or ambiguous.")
    age = time.time() - newest / 1e9
    if not 0 <= age <= 900:
        raise ValueError("Latest teaching file is outside the 15-minute window.")
    return matches[0]


def main(*, taught_path=None):
    result = {"completed": False, "motor_id": 6, "serial": SERIAL_NUMBER}
    output_path = None
    validate_source = no_stop
    try:
        if os.name == "nt":
            from teach_grip_points import controller_port

            if taught_path is None and sys.argv[1:] not in ([], ["--execute"]):
                raise ValueError("Usage: pinch_taught_usb.cmd [--execute]")
            execute = taught_path is not None or sys.argv[1:] == ["--execute"]
            path = latest_teaching(Path(__file__).resolve().parent / "results")
            if taught_path is not None and path.resolve() != Path(taught_path).resolve():
                raise RuntimeError("本窗口的示教记录已被另一轮替代。本次夹指动作已停止。")
            mode = "execute" if execute else "plan"
            output_path = path.parent / datetime.now().strftime(f"pinch_usb_{mode}_%Y%m%d_%H%M%S_%f.json")
            teach = json.loads(path.read_text(encoding="utf-8-sig"))
            def validate_source():
                if latest_teaching(path.parent.parent) != path or json.loads(path.read_text(encoding="utf-8-sig")) != teach:
                    raise RuntimeError("Teaching record changed while waiting for confirmation.")
            taught_endpoints(teach)  # Refuse incomplete, stale or foreign-host records before opening USB.
            device = controller_port(True)
            result.update(teaching_folder=path.parent.name, host=socket.gethostname(), device=device)
            print(f"{mode}: {path.parent.name}, right controller {SERIAL_NUMBER} on {device}.", flush=True)
        else:
            for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGINT):
                signal.signal(sig, interrupt)
            teach = json.loads(base64.b64decode(sys.argv[2], validate=True))
            if sys.argv[3] not in ("plan", "execute"):
                raise ValueError("Specify plan or execute.")
            execute = sys.argv[3] == "execute"
            devices = list(Path("/dev/serial/by-id").glob(f"*_{SERIAL_NUMBER}-*"))
            if len(devices) != 1:
                raise RuntimeError(f"Expected one right controller; found {len(devices)}.")
            device = str(devices[0].resolve(strict=True))
        with serial.Serial(device, 1_000_000, timeout=.12, write_timeout=.12, exclusive=True) as port:
            run(port, teach, execute, result, validate_source)
        result["completed"] = True
    except (Exception, KeyboardInterrupt) as exc:
        result["error"] = str(exc)
    if output_path is not None:
        pending = output_path.with_suffix(".tmp")
        pending.write_text(json.dumps(result, indent=2), encoding="utf-8")
        pending.replace(output_path)
        print(f"Saved: {output_path}", flush=True)
    print("PINCH_REPORT " + json.dumps(result, separators=(",", ":")), flush=True)
    return 0 if result["completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
