"""Verify the common left-arm pose from a nearby held pose, independently of grasping."""
import argparse
import json
import os
import signal
import threading
import time
import urllib.request
from pathlib import Path

from native_motor import HOME, SERIALS, in_range, lease, snapshot
from native_pick import HOME_TOLERANCE, move_pose


def save_record(path, record):
    temp = path.with_name(f".{path.name}-{os.getpid()}.tmp")
    try:
        temp.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


def check(confirmed, workstation=False):
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    runtime = Path(os.environ.get("O6_WORKSTATION_RUNTIME", Path.home() / "o6_workstation/runtime"))
    receipt = runtime / "left-home-trim.json"
    if confirmed and not workstation:
        receipt.unlink(missing_ok=True)
    touched = set()
    exercised = set()
    tolerance = HOME_TOLERANCE if workstation else 8
    evidence = {"targets": list(HOME), "passed": False, "verified_axes": [],
                "policy": "workstation" if workstation else "precision", "tolerance": tolerance}
    try:
        with urllib.request.urlopen("http://127.0.0.1:8876/api/pick/status", timeout=3) as response:
            if json.load(response).get("state") not in ("idle", "failed", "completed", "stopped"):
                raise ValueError("工作台仍有动作，请等待结束。")
        with lease("left") as bus:
            initial = snapshot(bus)
            evidence["before"] = initial
            if not confirmed:
                print(json.dumps(evidence, ensure_ascii=False))
                return
            if bus.simulated or [j["id"] for j in initial] != list(range(1, 7)):
                raise ValueError("实机复位检查需要左臂六轴的完整反馈。")
            if any(j["torque"] != 1 or j["moving"] != 0 or
                   abs(j["position"] - target) > 32 or abs(j["goal"] - j["position"]) > 16
                   for j, target in zip(initial, HOME)):
                raise ValueError("本次仅验证目标附近已保持的空臂。实测超出范围，本次未启动。")
            try:
                move_pose(bus, HOME, stop, "公共复位", lambda msg, _: print(msg, flush=True),
                          touched, trim_axes=None if workstation else {3, 4}, trimmed_axes=exercised)
                evidence["final_scans"] = []
                for scan in range(3):
                    if scan:
                        stop.wait(.12)
                    measured = snapshot(bus)
                    evidence["final_scans"].append(measured)
                    evidence["after"] = measured
                    if stop.is_set():
                        raise RuntimeError("本次复位验证已停止。")
                    if [j["id"] for j in measured] != list(range(1, 7)):
                        raise RuntimeError("复位后未读全左臂六轴。")
                    for j, target in zip(measured, HOME):
                        if (j["torque"] != 1 or j["moving"] != 0 or j["goal"] != target
                                or abs(j["position"] - target) > tolerance):
                            raise RuntimeError(
                                f"第 {scan + 1} 次复读 ID{j['id']} 实测 {j['position']}，"
                                f"目标 {target}（回读 {j['goal']}），力矩 {j['torque']}，运动 {j['moving']}。"
                            )
            except Exception:
                try:
                    evidence["at_failure"] = snapshot(bus)
                finally:
                    for axis in sorted(touched):
                        try:
                            j = bus.joint(axis)
                            if j["torque"] == 1:
                                in_range(j, j["position"])
                                bus.goal(axis, j["position"])
                        except Exception as hold_error:
                            evidence.setdefault("hold_errors", {})[axis] = str(hold_error)
                raise
            verified_axes = sorted(exercised)
            if not exercised.issubset({3, 4}):
                raise RuntimeError("本次微调记录含有意外关节，无法用于后续复位。")
            evidence["verified_axes"] = verified_axes
            evidence["passed"] = True
    except Exception as exc:
        evidence["error"] = str(exc)
        raise
    finally:
        if confirmed:
            evidence["trim_attempted_axes"] = sorted(exercised)
            evidence["timestamp"] = time.time()
            save_record(runtime / "home-check.json", evidence)
            print("HOME_RESULT " + json.dumps(evidence, ensure_ascii=False), flush=True)
    if workstation:
        print(f"工作台公共复位已通过，六轴连续三次复读均在 ±{tolerance} 格内。", flush=True)
    elif verified_axes:
        save_record(receipt, {
            "release": Path(__file__).resolve().parents[1].name,
            "policy": "left-home-trim-6-v1", "controller": SERIALS["left"], "home": list(HOME),
            "verified_axes": verified_axes, "passed": True,
        })
        print("已保存精密复位诊断记录，关节 ID", *verified_axes, flush=True)
    else:
        print("本次按原目标直接复位通过，未启用后续微调。", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--confirmed", action="store_true")
    parser.add_argument("--workstation", action="store_true")
    args = parser.parse_args()
    if not args.confirmed:
        return check(False, workstation=args.workstation)
    # Serialize field checks and upgrades; an old login may still be waiting at H.
    import fcntl
    runtime = Path(os.environ.get("O6_WORKSTATION_RUNTIME", Path.home() / "o6_workstation/runtime"))
    with (runtime / ".settle-update.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError("已有更新或复位验证正在进行。") from exc
        if (runtime.parent / "current").resolve() != Path(__file__).resolve().parents[1]:
            raise ValueError("工作台版本已更换，请从当前版本打开复位验证。")
        check(True, workstation=args.workstation)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, TimeoutError, RuntimeError) as exc:
        print("HOME_CHECK_REJECTED", exc, flush=True)
        raise SystemExit(3) from None
