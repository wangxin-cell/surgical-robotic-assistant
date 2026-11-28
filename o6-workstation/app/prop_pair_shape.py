"""Compare the longitudinal profiles of the two straight pink props in a detector box."""
import json
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
MODEL = ROOT / "weights/prop_pair_shape.json"
CLASSES = ("blade_shape", "tweezer_shape")


def shape_profile(image_bgr, box):
    left, top, right, bottom = np.asarray(box, np.float64)
    width, height = right - left, bottom - top
    if min(width, height) < 3 or max(width, height) < 24:
        return None

    padding = max(3, round(.05 * max(width, height)))
    x0 = max(0, int(np.floor(left)) - padding)
    y0 = max(0, int(np.floor(top)) - padding)
    x1 = min(image_bgr.shape[1], int(np.ceil(right)) + padding)
    y1 = min(image_bgr.shape[0], int(np.ceil(bottom)) + padding)
    if x1 - x0 < 3 or y1 - y0 < 3:
        return None

    patch = image_bgr[y0:y1, x0:x1]
    blue, green, red = cv2.split(patch.astype(np.float32))
    chroma = red - green
    hue = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)[:, :, 0]
    # Estimate contrast inside the YOLO box before searching its padded neighborhood.
    inside = chroma[
        max(0, int(top) - y0):min(y1 - y0, int(bottom) - y0 + 1),
        max(0, int(left) - x0):min(x1 - x0, int(right) - x0 + 1),
    ]
    if not inside.size:
        return None
    threshold = max(7., .18 * float(np.percentile(inside, 96)))
    mask = (chroma > threshold) & (blue - green > -2) & (hue >= 122) & (hue < 179)
    mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    candidates = []
    for i in range(1, count):
        x, y, cw, ch, area = stats[i]
        if area < 40 or max(cw, ch) < .64 * max(width, height):
            continue
        inter_w = max(0, min(x1, right, x + x0 + cw) - max(left, x + x0))
        inter_h = max(0, min(y1, bottom, y + y0 + ch) - max(top, y + y0))
        intersection = inter_w * inter_h
        overlap = intersection / max(1, width * height + cw * ch - intersection)
        center_x = x + x0 + cw / 2
        center_y = y + y0 + ch / 2
        near_center = (abs(center_x - (left + right) / 2) < .32 * width
                       and abs(center_y - (top + bottom) / 2) < .32 * height)
        if overlap >= .18 and near_center:
            candidates.append((overlap, i))
    if not candidates:
        return None
    candidates.sort(reverse=True)
    if len(candidates) > 1 and candidates[1][0] > .8 * candidates[0][0]:
        return None

    ys, xs = np.nonzero(labels == candidates[0][1])
    points = np.column_stack([xs, ys]).astype(np.float64)
    _, axes = np.linalg.eigh(np.cov(points.T))
    along, across = ((points - points.mean(0)) @ axes[:, ::-1]).T
    length = np.ptp(along) + 1.
    thickness = np.ptp(across) + 1.
    if length < 3.2 * thickness:
        return None

    # Trim a few isolated contour pixels per section, then normalize out scale.
    bins = np.minimum(15, ((along - along.min()) / length * 16).astype(np.int32))
    widths = []
    for i in range(16):
        section = across[bins == i]
        if not len(section):
            return None
        low, high = np.percentile(section, [3, 97])
        widths.append(high - low + 1.)
    widths = np.array(widths)
    profile = (widths / np.mean(widths)).tolist()
    profile.append(float(np.log(np.mean(widths) / length)))
    return profile


def distances(profile, library):
    features = np.asarray(profile)
    result = {}
    for name in CLASSES:
        refs = np.asarray([r["profile"] for r in library if r["class_name"] == name])
        if not len(refs):
            raise ValueError(f"No shape templates for {name}")
        direct = np.abs(refs[:, :-1] - features[:-1]).mean(axis=1)
        reverse = np.abs(refs[:, :-1] - features[:-1][::-1]).mean(axis=1)
        result[name] = float((np.minimum(direct, reverse) + .08 * np.abs(refs[:, -1] - features[-1])).min())
    return result


@lru_cache(maxsize=1)
def read_model():
    return json.loads(MODEL.read_text(encoding="utf-8"))


def classify_pair(image_bgr, xyxy):
    """Return an optional second opinion. Margin measures template separation."""
    profile = shape_profile(image_bgr, xyxy)
    if profile is None:
        return dict(class_name=None, nearest_class=None, reason="incomplete_or_ambiguous_silhouette",
                    distances={}, margin=None)
    model = read_model()
    scores = distances(profile, model["templates"])
    first, second = sorted(scores, key=scores.get)
    gap = (scores[second] - scores[first]) / max(scores[second], 1e-6)
    good = gap >= model["min_margin"] and scores[first] <= model["max_distance"]
    return dict(class_name=first if good else None, nearest_class=first, distances=scores, margin=gap,
                reason="profile_match" if good else "weak_profile_match")


def training_examples():
    phone = ROOT / "datasets/o6_props_20261125/reference"
    o6 = ROOT / "datasets/o6_props_finetune"
    supplement = ROOT / "datasets/o6_props_supplement_20261126"

    def read(path):
        return cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)

    for row in json.loads((phone / "phone_regions.json").read_bytes())["images"]:
        image = read(phone / "phone" / row["source_filename"])
        for index, region in enumerate(row["regions"]):
            if region["shape_tag"] in CLASSES:
                yield f"phone_{row['source_id']}", index, region["shape_tag"], image, region["bbox_xyxy"]
    for row in json.loads((o6 / "o6_manifest.json").read_bytes())["images"]:
        if row["split"] != "train":
            continue
        image = read(o6 / row["image"])
        height, width = image.shape[:2]
        for index, line in enumerate((o6 / row["label_file"]).read_text().splitlines()):
            cls, x, y, w, h = map(float, line.split())
            if cls in (0, 3):
                name = "blade_shape" if cls == 0 else "tweezer_shape"
                box = [(x - w / 2) * width, (y - h / 2) * height,
                       (x + w / 2) * width, (y + h / 2) * height]
                yield row["placement"], index, name, image, box
    for row in json.loads((supplement / "manifest.json").read_bytes())["samples"]:
        if row["scene_group"] not in ("01_horizontal_ring", "02_horizontal_tweezer"):
            continue
        image = read(supplement / row["image"])
        for index, region in enumerate(row["regions"]):
            if region["shape_tag"] in CLASSES:
                yield row["scene_group"], index, region["shape_tag"], image, region["bbox_crop_xyxy"]


def train():
    templates = []
    for group, obj, name, image, box in training_examples():
        for scale, sigma in ((1., 0.), (.67, 1.6), (.45, 2.)):
            if sigma:
                sample = cv2.GaussianBlur(image, (0, 0), sigma)
                sample = cv2.resize(sample, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            else:
                sample = image
            profile = shape_profile(sample, np.asarray(box)*scale)
            if profile is not None:
                templates.append(dict(source=group, instance=obj, class_name=name,
                                      augmentation=[scale,sigma], profile=profile))
    result = dict(templates=templates, min_margin=.20, max_distance=.26)
    for ref in templates:
        others = [r for r in templates if r["source"] != ref["source"]]
        scores = distances(ref["profile"], others)
        first, second = sorted(scores, key=scores.get)
        gap = (scores[second] - scores[first]) / max(scores[second], 1e-6)
        print(ref["source"], ref["instance"], ref["class_name"], first, round(scores[first], 3), round(gap, 3))
    MODEL.parent.mkdir(exist_ok=True)
    MODEL.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    train()
