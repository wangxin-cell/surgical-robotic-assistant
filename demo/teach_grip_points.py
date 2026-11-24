"""Record three hand-positioned poses on the right bus, using read instructions only."""
import errno
import json
import os
import socket
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import serial
from right_table_state import SERIAL_NUMBER, read_register


STAGES = (
    ("empty_pregrasp", "Empty jaws at the supported, stationary pregrasp pose; the handle location is fixed."),
    ("handle_contact", "At the grasp pose, pads just touch the supported blunt practice handle. No squeezing."),
    ("open_at_grasp", "Same stationary grasp pose, jaws open slightly clear of the handle."),
)
MAX_STILL_SPAN = 6


class TeachingCancelled(Exception):
    pass


def check_torque(port):
    values = [read_register(port, i, 40, 1) for i in range(1, 10)]
    if any(values):
        raise RuntimeError(f"Unexpected enabled torque: {values}. Stop the session.")
    return values


def capture(port):
    limits = [read_register(port, 6, a, 2) for a in (9, 11)]
    if not 0 <= limits[0] < limits[1] <= 4095:
        raise RuntimeError(f"Unexpected ID6 position limits: {limits}")
    samples = []
    for _ in range(3):
        torques = check_torque(port)
        positions = [read_register(port, i, 56, 2) for i in range(1, 10)]
        samples.append({"positions": positions, "torques": torques})
        time.sleep(0.15)
    check_torque(port)
    columns = list(zip(*(row["positions"] for row in samples)))
    spans = [max(column) - min(column) for column in columns]
    if max(spans) > MAX_STILL_SPAN:
        raise RuntimeError(f"Positions changed during stationary capture. Spans ID1..9: {spans}")
    pose = [int(statistics.median(column)) for column in columns]
    return {"utc": datetime.now(timezone.utc).isoformat(), "samples": samples,
            "median_by_id_1_to_9": pose, "span": spans, "id6_limits": limits,
            "jaw_inside_limits": limits[0] <= pose[5] <= limits[1]}

def controller_port(local_usb):
    if local_usb:
        from serial.tools.list_ports import comports

        matches = [entry.device for entry in comports() if entry.serial_number == SERIAL_NUMBER]
        if len(matches) != 1:
            raise RuntimeError(f"需要一个序列号 {SERIAL_NUMBER} 的右臂 USB 控制板，当前找到 {len(matches)} 个。")
        return matches[0]
    paths = list(Path("/dev/serial/by-id").glob(f"*_{SERIAL_NUMBER}-*"))
    if len(paths) != 1:
        raise RuntimeError(f"Expected one right controller, found {len(paths)}.")
    return str(paths[0])


def open_for_teaching(local_usb, report):
    for attempt in range(2):
        device = controller_port(local_usb)
        report["device"] = device if local_usb else str(Path(device).resolve(strict=True))
        try:
            return serial.Serial(device, 1_000_000, timeout=.12, write_timeout=.12, exclusive=True)
        except OSError as exc:
            if not local_usb or attempt > 0:
                raise
            # pyserial 3.5 on Windows embeds repr(ctypes.WinError()) in its open exception.
            denied = any(getattr(err, "winerror", None) == 5 or getattr(err, "errno", None) == errno.EACCES
                         for err in (exc, exc.__cause__, exc.__context__))
            if not denied and not (isinstance(exc, serial.SerialException) and "PermissionError(13," in str(exc)):
                raise
        print(f"Windows 拒绝打开右臂串口 {report['device']}。串口可能已被占用，或当前窗口没有访问权限。", flush=True)
        print("若已有示教窗口停在 [1/3]，请在本窗口输入 q 回车，再回到原窗口继续。", flush=True)
        while True:
            print("若已关闭其他占用串口的程序，输入 r 回车重试一次；输入 q 回车退出本窗口。", flush=True)
            try:
                choice = input().strip().lower()
            except EOFError:
                raise RuntimeError("Terminal input closed while waiting for a USB retry.") from None
            if choice == "q":
                raise TeachingCancelled("open_port") from None
            if choice == "r":
                break


def pair_issue(contact, opened, fixed_demo):
    changed = [f"ID{i} {a}->{b} ({b-a:+d})" for i, (a, b) in
               enumerate(zip(contact[:5], opened[:5]), 1) if abs(b-a) > MAX_STILL_SPAN]
    issues = []
    if changed:
        issues.append("Arm pose changed between contact and opening: " + ", ".join(changed)
                      + f". Maximum allowed change per arm joint: {MAX_STILL_SPAN} ticks.")
    travel = opened[5] - contact[5]
    if fixed_demo:
        if not (725 <= contact[5] <= 904 and 725 <= opened[5] <= 904 and 7 <= travel <= 24):
            issues.append(f"夹指轻贴 {contact[5]}、张开 {opened[5]}，变化 {travel:+d}。"
                          "本演示要求两次都在 725～904，张开变化 +7～+24。")
    elif travel <= MAX_STILL_SPAN:
        issues.append("Jaw opening was not resolved in the expected direction.")
    return " ".join(issues)


def collect(port, stages, local_usb=False, fixed_demo=False):
    if read_register(port, 6, 3, 2) != 777 or read_register(port, 6, 33, 1) != 0:
        raise RuntimeError("Unexpected ID6 model or mode.")
    number = 1
    while number <= len(STAGES):
        label, condition = STAGES[number - 1]
        check_torque(port)
        if local_usb:
            condition = (
                "空夹爪停在实心剪柄旁，机械臂和练习剪各自有稳定支撑。",
                "从外侧轻轻调整两块夹垫，刚碰到实心剪柄就停。",
                "腕部保持原位，从外侧稍稍张开夹爪至两垫刚离开剪柄。",
            )[number - 1]
        print(f"\n[{number}/3] {label}  {condition}", flush=True)
        if local_usb and label == "open_at_grasp":
            print("输入 p 并回车可先查看相对轻贴位置的变化。固定点夹指微动使用 7～24 个开合刻度。", flush=True)
            if fixed_demo:
                print("需要重新取轻贴位置时，输入 2 返回上一步，按新提示重新摆好后再记录。", flush=True)
        while True:
            prompt = (f"松手后保持稳定，手离开夹口，再输入 {number} 并回车记录。输入 q 并回车可停止。"
                      if local_usb else
                      f"Once supported and hands clear, type {number} then Enter to record. Type q then Enter to stop.")
            print(prompt, flush=True)
            try:
                answer = input().strip().lower()
            except EOFError:
                raise RuntimeError("Terminal input closed before this position was recorded.") from None
            if answer == "q":
                raise TeachingCancelled(label)
            if fixed_demo and number == 3 and answer == "2":
                break
            if local_usb and label == "open_at_grasp" and answer == "p":
                contact = stages["handle_contact"]["median_by_id_1_to_9"]
                current = capture(port)["median_by_id_1_to_9"]
                shifts = [b-a for a, b in zip(contact[:6], current[:6])]
                print("ID1～5 相对变化（各自最多 ±6）：" + ", ".join(f"{v:+d}" for v in shifts[:5])
                      + f"；夹指 ID6 张开变化：{shifts[5]:+d}。仍停在第 3 步。", flush=True)
                continue
            if answer == str(number):
                break
            print("Still waiting. This stage has not been recorded.", flush=True)
        if fixed_demo and number == 3 and answer == "2":
            del stages["handle_contact"]
            number = 2
            print("返回第 2 步。请将夹垫轻贴到有支撑的实心柄段，松手停稳，再输入 2 记录。", flush=True)
            continue
        sample = capture(port)
        pose = sample["median_by_id_1_to_9"]
        if fixed_demo and sample["id6_limits"] != [705, 924]:
            raise RuntimeError(f"Unexpected ID6 limits: {sample['id6_limits']}")
        if number == 3:
            issue = pair_issue(stages["handle_contact"]["median_by_id_1_to_9"], pose, fixed_demo)
            if issue and fixed_demo:
                print(f"这次张开位置未采用。{issue}\n仍停在第 3 步。", flush=True)
                continue
            if issue:
                stages[label] = sample  # Keep the original read-only diagnostic report.
                raise RuntimeError(issue)
        stages[label] = sample
        print(f"Recorded {label}: {pose}", flush=True)
        number += 1


def main(fixed_demo=False):
    local_usb = os.name == "nt"
    out = {"host": socket.gethostname(), "utc": datetime.now(timezone.utc).isoformat(),
           "serial": SERIAL_NUMBER, "read_only": True, "completed": False, "motion_ready": False,
           "stages": {}, "cancelled": False}
    try:
        print("Read-only teaching. Support arm and blunt prop independently throughout.", flush=True)
        print("Keep fingers outside jaws. Stop on resistance or loss of clearance.", flush=True)
        if fixed_demo:
            print("三步示教后，程序在同一窗口核对实时位置。执行夹指微动前还需你在现场输入 YES。", flush=True)
        with open_for_teaching(local_usb, out) as port:
            collect(port, out["stages"], local_usb, fixed_demo)
        out["completed"] = True
    except TeachingCancelled as exc:
        out.update(cancelled=True, stopped_at=str(exc))
        print("Cancelled at " + str(exc) + ". Saving the readings collected so far.", flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        out["error"] = f"{type(exc).__name__}: {exc}"
        print(out["error"], flush=True)
    if local_usb:
        folder = Path(__file__).resolve().parent / "results" / datetime.now().strftime("passive_teach_usb_%Y%m%d_%H%M%S_%f")
        folder.mkdir(parents=True)
        pending = folder / "result.tmp"
        pending.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
        pending.replace(folder / "result.json")
        print(f"Saved: {folder}  completed={out['completed']}  cancelled={out['cancelled']}", flush=True)
        if fixed_demo and out["completed"]:
            from pinch_taught import main as pinch_main

            return pinch_main(taught_path=folder / "result.json")
    else:
        print("TEACH_REPORT " + json.dumps(out, separators=(",", ":")), flush=True)
    return 0 if out["completed"] or out["cancelled"] else 1


if __name__ == "__main__":
    if os.name == "nt" and len(sys.argv) > 1:
        if sys.argv[1:] == ["--fixed-demo"]:
            raise SystemExit(main(fixed_demo=True))
        if sys.argv[1:] != ["--check"]:
            raise SystemExit("Usage: teach_grip_usb.cmd [--check | --fixed-demo]")
        try:
            print(f"Right USB controller {SERIAL_NUMBER}: {controller_port(True)}")
        except RuntimeError as exc:
            raise SystemExit(str(exc))
        raise SystemExit(0)
    raise SystemExit(main())
