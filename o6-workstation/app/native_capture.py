"""Capture and classify the O6's local USB streams using the evaluated tabletop model."""
import base64
from datetime import datetime
from functools import lru_cache
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from threading import Lock

from detection_groups import ranked_shape_groups
from multiview_recognition import recognize
from predict import annotate
from shape_proposals import close_prop_roi, predict_table_crop
from table_region import find_table
from triplet_capture import camera_ports, ROLES


HERE = Path(__file__).resolve().parent
CAPTURES = (Path(os.environ.get("O6_WORKSTATION_RUNTIME",
                                 Path.home() / "o6_workstation/runtime")) / "captures").resolve()
WEIGHTS = HERE / "weights" / "props.pt"
MODEL_SHA256 = "7bd1a5f38d6e8a975ed5131e8b9c87f40a1dcd691234af51d28db6a998cf7291"
SHAPES = ["blade_shape", "ring_broad_split", "ring_narrow_tip", "tweezer_shape"]
_capture_lock = Lock()


@lru_cache(maxsize=1)
def detector():
    from ultralytics import YOLO

    if hashlib.sha256(WEIGHTS.read_bytes()).hexdigest() != MODEL_SHA256:
        raise ValueError("O6 上的视觉模型与已验证的模型不符。")
    model = YOLO(str(WEIGHTS), task="detect")
    if [model.names[i] for i in range(len(model.names))] != SHAPES:
        raise ValueError("模具模型的类别表与工作台不符。")
    return model


def camera_state():
    if _capture_lock.locked():
        return "busy"
    head = next(port for role, port in ROLES if role == "head")
    return "ready" if head in camera_ports() else "offline"


def save_report(report):
    CAPTURES.mkdir(parents=True, exist_ok=True)
    final = CAPTURES / ("camera_triplet_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    with TemporaryDirectory(dir=CAPTURES, prefix=".capture-") as tmp:
        staging = Path(tmp) / "report"
        staging.mkdir()
        for index, view in enumerate(report["views"], 1):
            for key, field, suffix in (("jpeg", "image_file", ""),
                                       ("annotated_jpeg", "annotated_image_file", "_detections")):
                if key not in view:
                    continue
                data = base64.b64decode(view.pop(key), validate=True)
                if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
                    raise ValueError(f"相机 {index} 的 JPEG 不完整")
                filename = f"camera_{index}{suffix}.jpg"
                (staging / filename).write_bytes(data)
                view[field] = filename
        (staging / "result.json").write_text(json.dumps(report, ensure_ascii=False), encoding="utf-8")
        staging.rename(final)
    return final


def capture_scene():
    if not _capture_lock.acquire(blocking=False):
        raise RuntimeError("O6 正在拍摄上一组画面，请稍后重试。")
    try:
        # Bound driver reads in a separate process, so a disconnected camera
        # cannot strand the server in a blocked ioctl.
        try:
            burst = subprocess.run([sys.executable, str(HERE / "triplet_capture.py")],
                                   capture_output=True, timeout=60, check=False)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("三路取帧超时，请检查 O6 的相机连接。") from exc
        if burst.returncode:
            raise RuntimeError("O6 本机取帧失败。"+burst.stderr.decode(errors="replace")[-450:])
        report = json.loads(burst.stdout)
        report["model"] = {"weights": str(WEIGHTS), "sha256": MODEL_SHA256,
                           "class_names": dict(enumerate(SHAPES)), "device": "cpu",
                           "imgsz": 640, "confidence_threshold": .25}
        if any(view.get("jpeg") for view in report["views"]):
            recognize(report, detector(), predict_table_crop, close_prop_roi, find_table, annotate)
            for view in report["views"]:
                if "detection_record" in view:
                    groups = ranked_shape_groups(view["detection_record"]["detections"])
                    view["ranked_groups"] = {"indices": groups, "iou_threshold": .55}
        return save_report(report)
    finally:
        _capture_lock.release()
