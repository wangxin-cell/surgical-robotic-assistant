"""Bind four fixed left-arm trajectories to positions in the uncropped head view."""
import json
import math
from pathlib import Path
import tempfile

import cv2
import numpy as np


def _points(values):
    if (not isinstance(values, list) or len(values) != 4
            or not all(isinstance(p, (list, tuple)) and len(p) == 2 for p in values)
            or not all(type(v) in (int, float) and math.isfinite(v) and 0 <= v <= 1
                       for p in values for v in p)):
        raise ValueError("请在头部完整原图中依次点击四个示教位置的器具中心。")
    arr = np.array(values, dtype=float)
    if np.any(np.diff(arr[:, 0]) <= 0):
        raise ValueError("点位 1 至 4 必须在头部原图上从左到右排列。")
    return arr


def _frame(scene):
    if scene.get("source") != "live" or scene.get("error"):
        raise ValueError("需要本次拍摄的头部原图，请重新拍摄。")
    meta = scene.get("head_camera") or {}
    path = (scene.get("views") or {}).get("head")
    if not path or not meta.get("device"):
        raise ValueError("本次拍摄缺少头部镜头的原图或设备标识。")
    try:
        image = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    except (OSError, cv2.error):
        image = None
    if image is None or list(image.shape[1::-1]) != meta.get("size"):
        raise ValueError("头部镜头的原图或分辨率信息有误，请重新拍摄。")
    return meta, image


def _spacing(points):
    d = np.linalg.norm(points[:, None] - points, axis=2)
    np.fill_diagonal(d, np.inf)
    return d.min(axis=1)


def _paper_dots(image, slots):
    """Locate dark ink on light paper near the fixtures, independent of YOLO or occupancy."""
    h, w = image.shape[:2]
    gap = np.median(_spacing(slots))
    x, y = np.maximum(0, np.floor(slots.min(0) - [gap, 2.5 * gap])).astype(int)
    r, b = np.minimum([w, h], np.ceil(slots.max(0) + [gap, 2.5 * gap])).astype(int)
    # Analyse at a common scale so that dot size and search windows have pixel units.
    scale = 1920 / w
    paper = cv2.resize(image[y:b, x:r].max(axis=2), None, fx=scale, fy=scale)
    mask = cv2.adaptiveThreshold(paper, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 31, 30)
    dots = []
    for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        left, top, width, height = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        if not (3 <= min(width, height) and max(width, height) <= 16 and
                min(width, height) >= .52 * max(width, height) and area > max(7, .45 * width * height)):
            continue
        u, v = left + width // 2, top + height // 2
        ring = paper[v-12:v+13, u-12:u+13]
        if ring.shape == (25, 25) and np.median(ring) > max(130, float(paper[v, u]) + 35):
            dots.append([x + u / scale, y + v / scale])
    return np.asarray(dots, dtype=np.float32).reshape(-1, 2)


def _area(dots):
    return cv2.contourArea(cv2.convexHull(dots)) if len(dots) >= 3 else 0.


def _same_sheet(reference, current, radius):
    if len(current) == 0:
        return False
    # Each reference dot needs its own nearby dot. A surviving single row cannot
    # distinguish a rotated/translated sheet from its commissioned geometry.
    near = np.linalg.norm(reference[:, None] - current, axis=2) <= radius
    kept = reference[(near.sum(axis=1) == 1) & ((near & (near.sum(axis=0) == 1)).any(axis=1))]
    return len(kept) >= max(4, math.ceil(len(reference) * .65)) and _area(kept) >= .55 * _area(reference)


def _box(obj):
    raw = obj.get("box")
    if not isinstance(raw, (list, tuple)) or len(raw) != 4:
        return None
    box = np.asarray(raw, dtype=float)
    if not np.isfinite(box).all() or np.any(box[2:] <= box[:2]):
        return None
    return box


def _axis(box):
    w, h = box[2:] - box[:2]
    return "horizontal" if w > 1.6 * h else "vertical" if h > 1.6 * w else None


class PickLocations:
    def __init__(self, path):
        self.path = Path(path)

    def _read(self):
        if not self.path.exists():
            raise ValueError("请先在头部原图中设置四个夹取点位。")
        try:
            cfg = json.loads(self.path.read_bytes())
            _points(cfg["points"])
            dots = np.asarray(cfg["dots"], dtype=np.float32)
            if (cfg["schema"] != 1 or len(cfg["axes"]) != 4 or not cfg["device"]
                    or len(cfg["image_size"]) != 2 or min(cfg["image_size"]) <= 0
                    or dots.ndim != 2 or dots.shape[1] != 2 or not np.isfinite(dots).all()
                    or len(dots) < 4 or _area(dots) <= 0):
                raise ValueError
            return cfg
        except (KeyError, TypeError, ValueError, OSError, cv2.error) as exc:
            raise ValueError("四点位标定文件无法读取，请重新设置点位。") from exc

    def status(self):
        try:
            cfg = self._read()
        except ValueError as exc:
            return {"configured": False, "points": [], "message": str(exc)}
        return {"configured": True, "points": cfg["points"], "image_size": cfg["image_size"],
                "message": "已保存四个点位。器具和夹具请按示教时的高度及方向摆放。"}

    def save(self, scene, points):
        coords = _points(points)
        meta, image = _frame(scene)
        slots = coords * meta["size"]
        spacing = _spacing(slots)
        if np.min(np.diff(slots[:, 0])) < max(8., image.shape[1] * .008):
            raise ValueError("相邻点在水平方向过近，请核对左至右四个不同的示教位置。")
        dots = _paper_dots(image, slots)
        if len(dots) < 4 or _area(dots) < .20 * np.median(spacing) ** 2:
            raise ValueError("定位纸上的黑点不足，请露出分布在至少两行、两列的四个黑点后重新拍摄。")
        boxes = [b for obj in scene.get("objects", []) if (b := _box(obj)) is not None]
        axes = []
        for p, gap in zip(slots, spacing):
            near = [box for box in boxes if np.linalg.norm((box[:2] + box[2:]) / 2 - p) <= .28 * gap]
            axes.append(_axis(near[0]) if len(near) == 1 else None)
        cfg = {"schema": 1, "points": coords.tolist(), "device": meta["device"], "image_size": meta["size"],
               "dots": dots.tolist(), "axes": axes, "captured_at": scene.get("captured_at")}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=self.path.parent, suffix=".tmp", delete=False) as file:
            staged = Path(file.name)
        try:
            staged.write_text(json.dumps(cfg, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
            staged.replace(self.path)
        finally:
            staged.unlink(missing_ok=True)
        return self.status()

    def match(self, scene, target_id):
        return self._match(scene, target_id=target_id)

    def match_slot(self, scene, slot):
        if type(slot) is not int or not 1 <= slot <= 4:
            return {"ready": False, "slot": None, "message": "点位编号应为 1、2、3 或 4。"}
        return self._match(scene, selected_slot=slot)

    def _match(self, scene, target_id=None, selected_slot=None):
        def blocked(message):
            return {"ready": False, "slot": None, "message": message}

        try:
            cfg = self._read()
            meta, image = _frame(scene)
        except ValueError as exc:
            return blocked(str(exc))
        if meta["device"] != cfg["device"] or meta["size"] != cfg["image_size"]:
            return blocked("头部镜头或图像分辨率已改变，请重新设置点位。")
        size = meta["size"]
        slots = np.asarray(cfg["points"]) * size
        spacing = _spacing(slots)
        reference = np.asarray(cfg["dots"], dtype=np.float32)
        radius = min(.10 * spacing.min(), .004 * size[0])
        if not _same_sheet(reference, _paper_dots(image, slots), radius):
            return blocked("当前纸面黑点与保存的点位不符，请检查遮挡、镜头及台面位置。")
        objects = scene.get("objects", [])
        def nearby(obj, i):
            box = _box(obj)
            center = (box[:2] + box[2:]) / 2 if box is not None else obj.get("anchor")
            return center is not None and np.linalg.norm(slots[i] - center) <= .28 * spacing[i]

        if selected_slot is not None:
            matching = [obj for obj in objects if nearby(obj, selected_slot-1)]
        else:
            results = [r for r in scene.get("results", []) if r.get("id") == target_id]
            if len(results) != 1:
                return blocked("本次画面没有所请求器具的识别结果。")
            result = results[0]
            if result.get("status") != "found" or not result.get("unique") or len(result.get("numbers", [])) != 1:
                return blocked(result.get("message") or "当前画面无法确定唯一器具。")
            matching = [obj for obj in objects if obj.get("number") == result["numbers"][0]]
        if len(matching) > 1:
            return blocked("同一点位还有其他器具轮廓，请分开后重拍。")
        if not matching or matching[0].get("evidence") != "found" or not matching[0].get("class_name"):
            return blocked("当前画面没有可靠的目标框。")
        obj = matching[0]
        box = _box(obj)
        if box is None:
            return blocked("仅有腕部镜头的局部线索，头部原图中未定位到完整器具。")
        center = (box[:2] + box[2:]) / 2
        distances = np.linalg.norm(slots - center, axis=1)
        nearest = int(np.argmin(distances))
        if distances[nearest] > .28 * spacing[nearest]:
            return blocked("器具中心偏离已示教点位，请按示教摆位放置后重拍。")
        if cfg["axes"][nearest] and _axis(box) and _axis(box) != cfg["axes"][nearest]:
            return blocked("器具的横竖方向与该点位示教摆位不同，请按示教方向放置。")
        for other in objects:
            if other is obj:
                continue
            other_box = _box(other)
            overlaps = other_box is not None and np.all((other_box[:2] <= center) & (center <= other_box[2:]))
            if nearby(other, nearest) or overlaps:
                return blocked("同一点位还有其他器具轮廓，请分开后重拍。")
        return {"ready": True, "slot": nearest + 1, "center": (center / size).tolist(),
                "image_size": size, "class_name": obj["class_name"], "label": obj.get("label"), "number": obj["number"],
                "message": f"目标位于点位 {nearest + 1}。"}
