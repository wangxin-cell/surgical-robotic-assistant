"""Meter each O6 USB camera, then time-match the frames used for recognition."""
import base64
import contextlib
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import itertools
import json
from pathlib import Path
import socket
import struct
import time

import cv2
from exposure_view import AUTO, ABSOLUTE, MANUAL, control, write_control
from triplet_exposure import meter, next_exposure


# Mapping confirmed on this hub. Device node numbers can change after a reboot.
def camera_ports(root=Path("/dev/v4l/by-path")):
    groups = {}
    for path in root.glob("platform-xhci-hcd.*.auto-usb-0:1.*:1.0-video-index0"):
        prefix, port = path.name.removesuffix(":1.0-video-index0").rsplit(".", 1)
        if port.isdigit() and path.exists():
            groups.setdefault(prefix, {})[int(port)] = path
    return next(iter(groups.values())) if len(groups) == 1 else {}

ROLES = (("head", 2), ("left_wrist", 4), ("right_wrist", 3))


def utc():
    return datetime.now(timezone.utc).isoformat()


def exposure(path):
    import fcntl
    with path.open("rb", buffering=0) as fd:
        values = {"unit_us": 100, "read_utc": utc()}
        for name, control in (("mode", 0x009A0901), ("absolute_100us", 0x009A0902)):
            buf = bytearray(struct.pack("=Ii", control, 0))
            fcntl.ioctl(fd.fileno(), 0xC008561B, buf, True)
            values[name] = struct.unpack("=Ii", buf)[1]
        return values


def configure(view, path, streams, head_size):
    view["node"] = str(path.resolve(strict=True))
    cap = cv2.VideoCapture(str(path), cv2.CAP_V4L2)
    streams.callback(cap.release)
    if not cap.isOpened():
        raise RuntimeError("Failed to open USB stream")
    width, height = head_size if view["role"] == "head" else (640, 480)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    code = int(cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little", signed=False)
    view["pixel_format"] = code.decode("ascii", "replace")
    view["raw_mjpeg"] = code in (b"MJPG", b"JPEG") and cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    view["requested_size"] = [width, height]
    view["width"] = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    view["height"] = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    try:
        view["exposure_readback"] = exposure(path)
    except OSError as exc:
        view["exposure_readback"] = {"error": str(exc)}
    return cap


def grab(cap):
    start_ms = time.monotonic_ns() / 1e6
    if not cap.grab():
        raise RuntimeError("Camera did not deliver a frame")
    received_ms = time.monotonic_ns() / 1e6
    driver_ms = cap.get(cv2.CAP_PROP_POS_MSEC)
    ok, frame = cap.retrieve()
    if not ok or frame is None or not frame.size:
        raise RuntimeError("Empty camera frame")
    return {"read_start_ms": start_ms, "host_received_ms": received_ms,
            "v4l2_frame_ms": driver_ms if 0 < driver_ms <= received_ms + 1 and received_ms-driver_ms < 2000 else None,
            "frame": frame}


def settled_meter(cap, role, reference=None):
    # Discard queued images as well as the first frames after the UVC setting changes.
    for _ in range(4):
        grab(cap)
    observations = []
    for _ in range(3):
        observed = meter(grab(cap)["frame"], role, reference)
        if reference is None:
            reference = observed
        observations.append(observed)
    return sorted(observations, key=lambda obs: obs["p80"])[1]


def tune_camera(view, cap):
    role = view["role"]
    record = view["auto_exposure"] = {"status": "unavailable", "trials": []}
    def restore(fd, original_mode, original_value):
        try:
            if control(fd, AUTO)["value"] == MANUAL:
                write_control(fd, ABSOLUTE, original_value)
        finally:
            write_control(fd, AUTO, original_mode)

    try:
        before = settled_meter(cap, role)
        record["before"] = before
        if before["level"] == "ok":
            record["status"] = "balanced"
            return
        with open(view["device_by_path"], "rb+", buffering=0) as device:
            fd = device.fileno()
            mode = control(fd, AUTO)["value"]
            shutter = control(fd, ABSOLUTE)
            original = shutter["value"]
            record["original"] = {"mode": mode, "absolute_100us": original}
            touched = False
            try:
                if mode != MANUAL:
                    touched = True
                    write_control(fd, AUTO, MANUAL)
                    shutter = control(fd, ABSOLUTE)
                    reading = settled_meter(cap, role, before)
                else:
                    reading = before
                current = shutter["value"]
                best_value, best = current, reading
                visited = {current}
                for _ in range(7):
                    target = next_exposure(reading, current, shutter["min"], shutter["max"], shutter["step"])
                    if target in visited:
                        break
                    visited.add(target)
                    touched = True
                    write_control(fd, ABSOLUTE, target)
                    reading = settled_meter(cap, role, before)
                    current = target
                    record["trials"].append({"value": current, "level": reading["level"], "score": reading["score"]})
                    if reading["level"] == "ok" or reading["score"] < best["score"]:
                        best_value, best = current, reading
                    if reading["level"] == "ok":
                        break
                if best["level"] != "ok" and best["score"] >= before["score"]:
                    if touched:
                        restore(fd, mode, original)
                    record["status"] = "limited"
                    return
                if current != best_value:
                    write_control(fd, ABSOLUTE, best_value)
                record["expected_manual"] = best_value
                record["status"] = "adjusted" if best["level"] == "ok" else "limited"
            except Exception:
                if touched:
                    restore(fd, mode, original)
                raise
    except (OSError, ValueError, RuntimeError, cv2.error) as exc:
        record["error"] = str(exc)
        record["status"] = "unavailable"


def verify_exposure(view, frame):
    record = view["auto_exposure"]
    try:
        after = meter(frame, view["role"], record.get("before"))
        record["after"] = after
        if record["status"] != "unavailable" and after["level"] != "ok":
            record["status"] = "limited"
    except (ValueError, RuntimeError, cv2.error) as exc:
        record.update(status="unavailable", error=str(exc))
    try:
        readback = exposure(Path(view["device_by_path"]))
        view["exposure_readback"] = readback
        # In automatic mode this register can be a stored manual setting.
        if readback["mode"] == MANUAL:
            record["value"] = readback["absolute_100us"]
        if "expected_manual" in record and (readback["mode"] != MANUAL or
                                             record.get("value") != record["expected_manual"]):
            record.update(status="unavailable", error="Final exposure setting differs from the verified command")
    except (OSError, ValueError, RuntimeError) as exc:
        view["exposure_readback"] = {"error": str(exc)}
        record.pop("value", None)
        record.update(status="unavailable", error=str(exc))


def save_frame(view, sample):
    frame = sample["frame"]
    if view["raw_mjpeg"] and (frame.ndim < 3 or frame.shape[-1] == 1):
        jpeg = frame.tobytes()
        end = jpeg.rfind(b"\xff\xd9")
        if not jpeg.startswith(b"\xff\xd8") or end < 2:
            raise ValueError("Malformed MJPEG frame")
        jpeg = jpeg[:end+2]
        view["jpeg_source"] = "native_mjpeg"
    else:
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            raise ValueError("JPEG encode failed")
        jpeg = encoded.tobytes()
        view["jpeg_source"] = "full_frame_reencoded"
    view["jpeg"] = base64.b64encode(jpeg).decode("ascii")
    view["frame_timing"] = {k: v for k, v in sample.items() if k != "frame"}


def match_samples(samples):
    # POS_MSEC from OpenCV's V4L2 backend is the kernel buffer timestamp.
    # Prefer that clock only when it advances and aligns with CLOCK_MONOTONIC.
    driver_clock = all(len(s) >= 2 and all(r["v4l2_frame_ms"] is not None for r in s)
                       and all(a["v4l2_frame_ms"] < b["v4l2_frame_ms"] for a, b in zip(s, s[1:]))
                       for s in samples)
    field = "v4l2_frame_ms" if driver_clock else "host_received_ms"
    def span(group):
        ts = [s[field] for s in group]
        return max(ts) - min(ts)
    selected = min(itertools.product(*samples), key=span)
    return selected, field


def acquire(head_size=(1920, 1080)):
    views = []
    ports = camera_ports()
    for role, port in ROLES:
        views.append({"role": role, "device_by_path": str(ports[port]) if port in ports else "", "kind": "full",
                      "role_source": "operator_verified_usb_hub_port"})
    samples = [[] for _ in views]
    with contextlib.ExitStack() as streams, ThreadPoolExecutor(max_workers=len(views)) as workers:
        active = {}
        nodes = set()
        for i, view in enumerate(views):
            try:
                if not view["device_by_path"]:
                    raise ValueError(f"未定位到 {ROLES[i][0]} 镜头。所需集线器端口 {ROLES[i][1]}，已定位的端口 {sorted(ports)}。")
                path = Path(view["device_by_path"])
                node = path.resolve(strict=True)
                if node in nodes:
                    raise ValueError("Multiple camera roles resolve to one device")
                nodes.add(node)
                active[i] = configure(view, path, streams, head_size)
            except (OSError, ValueError, RuntimeError, cv2.error) as exc:
                view["error"] = str(exc)
        pending = [workers.submit(tune_camera, views[i], cap) for i, cap in active.items()]
        for task in pending:
            task.result()
        for sequence in range(7):
            if not active:
                break
            pending = {i: workers.submit(grab, cap) for i, cap in active.items()}
            for i, future in pending.items():
                try:
                    s = future.result()
                    if sequence >= 3:
                        samples[i].append(s)
                except (OSError, ValueError, RuntimeError, cv2.error) as exc:
                    del active[i]
                    views[i]["error"] = str(exc)
        participants = [i for i in active if samples[i]]
        if participants:
            selected, clock = match_samples([samples[i] for i in participants])
            for i, sample in zip(participants, selected):
                verify_exposure(views[i], sample["frame"])

    result = dict(host=socket.gethostname(), utc=utc(), views=views,
                  timing={"method": "concurrent_free_running_time_matched", "hardware_trigger": False,
                          "driver_timestamp_skew_ms": None, "host_receive_skew_ms": None})
    if participants:
        for i, s in zip(participants, selected):
            try:
                save_frame(views[i], s)
            except (ValueError, cv2.error) as exc:
                views[i]["error"] = str(exc)
        if len(participants) == 3 and all("jpeg" in v for v in views):
            def spread(key):
                values = [s[key] for s in selected]
                return round(max(values)-min(values), 3)
            result["timing"].update(match_clock=clock, host_receive_skew_ms=spread("host_received_ms"),
                                    driver_timestamp_skew_ms=spread(clock) if clock == "v4l2_frame_ms" else None)
    result["complete"] = all("jpeg" in v for v in views)
    return result


if __name__ == "__main__":
    report = acquire()
    # The shared USB controller may refuse the highest-bandwidth head mode.
    if not report["complete"] and all(Path(v["device_by_path"]).exists() for v in report["views"]):
        fallback = acquire((640, 480))
        if sum("jpeg" in v for v in fallback["views"]) >= sum("jpeg" in v for v in report["views"]):
            fallback["first_attempt_errors"] = {v["role"]: v["error"] for v in report["views"] if "error" in v}
            report = fallback
    print(json.dumps(report), flush=True)
