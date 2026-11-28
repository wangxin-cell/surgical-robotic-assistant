"""Review YOLO proposals for the pink tabletop props using native head-camera pixels."""
import cv2
import numpy as np

from prop_pair_shape import CLASSES, classify_pair


def pink_evidence(image, box):
    left, top, right, bottom = np.rint(box).astype(int)
    height, width = image.shape[:2]
    region = image[max(0, top):max(0, min(height, bottom)),
                   max(0, left):max(0, min(width, right))]
    if region.size == 0:
        return {"fraction": 0., "span_fraction": 0.}
    hue, saturation, _ = cv2.split(cv2.cvtColor(region, cv2.COLOR_BGR2HSV))
    blue, green, red = cv2.split(region.astype(np.int16))
    mask = ((hue >= 125) & (hue <= 176) & (saturation >= 30) &
            (red-green >= 12) & (blue-green >= 4))
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    span = max((max(w, h) for _, _, w, h, area in stats[1:] if area >= 5), default=0)
    return {"fraction": float(mask.mean()), "span_fraction": float(span/max(region.shape[:2]))}


def review_items(image, items):
    """Keep raw classes and scores. Apply the pink-material check only with scene evidence."""
    evidence = {i: pink_evidence(image, item["head_box"]) for i, item in enumerate(items)
                if item.get("source") == "head" and item.get("head_box") is not None}
    pink_scene = sum(e["fraction"] > .05 and e["span_fraction"] > .40 for e in evidence.values()) >= 2
    accepted, rejected = [], []
    for i, original in enumerate(items):
        item = dict(original, recognized_class=original["class_name"])
        if i in evidence and original["class_name"] in CLASSES:
            color = evidence[i]
            item["pink_evidence"] = color
            if pink_scene and color["fraction"] < .005 and color["span_fraction"] < .08:
                item.update(recognized_class=None, review_reason="no_corresponding_pink_prop")
                rejected.append(item)
                continue
            shape = classify_pair(image, original["head_box"])
            item["shape_review"] = shape
            if shape["class_name"] is not None:
                item["recognized_class"] = shape["class_name"]
        accepted.append(item)
    return accepted, rejected

