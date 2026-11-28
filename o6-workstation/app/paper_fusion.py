"""Pixel-level wrist supplementation for stationary pink props with visible ring apertures."""
import json
from pathlib import Path

import cv2
import numpy as np

from paper_registration import register_paper


def primary(view):
    raw = view["detection_record"]["detections"]
    picks = []
    for group in view["ranked_groups"]["indices"]:
        idx = group[0]
        row = raw[idx]
        other = next((raw[i] for i in group[1:] if raw[i]["class_name"] != row["class_name"]), None)
        alternative = {key: other[key] for key in ("class_name", "confidence")} if other else None
        picks.append({"index": idx, "box": row["box_xyxy_pixels"], "class_name": row["class_name"],
                      "confidence": row["confidence"], "alternative": alternative})
    return picks


def apertures(image, roi):
    """Find enclosed oval openings in a pink outline; no YOLO classes are consulted."""
    x, y, r, b = roi
    hue, sat, val = cv2.split(cv2.cvtColor(image[y:b, x:r], cv2.COLOR_BGR2HSV))
    mask = ((hue >= 130) & (hue < 178) & (sat > 25) & (val > 80)).astype(np.uint8)
    contours, tree = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    holes = []
    for i, c in enumerate(contours):
        if tree[0, i, 3] < 0 or len(c) < 5:
            continue
        area = cv2.contourArea(c)
        if not 30 <= area <= .035 * mask.size:
            continue
        (cx, cy), axes, _ = cv2.fitEllipse(c)
        major, minor = max(axes), min(axes)
        if minor < 4 or minor < .36 * major:
            continue
        fill = area / (np.pi * major * minor / 4)
        if .83 <= fill <= 1.15:
            holes.append({"xy": [cx+x, cy+y], "area": area})
    return holes


def links(head_holes, wrist_holes, registration):
    if not head_holes or not wrist_holes:
        return []
    reference = np.asarray(registration["head_points"], np.float32)
    hull = cv2.convexHull(reference)
    spacing = np.linalg.norm(reference[:, None] - reference[None, :], axis=2)
    np.fill_diagonal(spacing, np.inf)
    margin = float(np.median(np.min(spacing, axis=1)))
    # Limit extrapolation to one fiducial spacing outside the observed paper patch.
    head_holes = [h for h in head_holes if cv2.pointPolygonTest(hull, tuple(h["xy"]), True) >= -margin]
    if not head_holes:
        return []
    points = np.float32([h["xy"] for h in head_holes]).reshape(-1, 1, 2)
    warped = cv2.perspectiveTransform(points, np.asarray(registration["head_to_right_h"])).reshape(-1, 2)
    wrist_xy = np.float32([h["xy"] for h in wrist_holes])
    distances = np.linalg.norm(warped[:, None] - wrist_xy[None, :], axis=2)
    gate = min(10.0, max(5.0, 3 * registration["check_error_right_px"]))
    close = distances <= gate
    pairs = []
    for i, j in zip(*np.where(close)):
        if close[i].sum() == 1 and close[:, j].sum() == 1:
            pairs.append({"head": head_holes[i], "wrist": wrist_holes[j],
                          "projection_px": warped[i].tolist(), "residual_px": float(distances[i, j])})
    return pairs


def contains(box, xy):
    l, t, r, b = box
    return l <= xy[0] <= r and t <= xy[1] <= b


def span(box, xy):
    l, t, r, b = box
    return max(np.hypot(cx-xy[0], cy-xy[1]) for cx, cy in ((l,t), (r,t), (l,b), (r,b)))


def head_item(det):
    return {"source": "head", "source_index": det["index"], "class_name": det["class_name"],
            "confidence": det["confidence"], "alternative": det["alternative"], "head_box": det["box"], "head_anchor": None,
            "wrist_box": None, "wrist_anchor": None, "evidence": {"method": "head_yolo"}}


def fuse_triplet(folder):
    folder = Path(folder)
    report = json.loads((folder/"result.json").read_bytes())
    views = {v["role"]: v for v in report["views"]}
    head, wrist = views.get("head", {}), views.get("right_wrist", {})
    hd = primary(head) if "ranked_groups" in head else []
    items = {d["index"]: head_item(d) for d in hd}
    reg = register_paper(folder)
    out = {"status": "head_only", "reasons": [], "registration": reg, "head_primary": hd,
           "items": list(items.values()), "wrist_reviews": [], "supplements": 0}
    timing = report.get("timing", {})
    skew = timing.get("driver_timestamp_skew_ms")
    if not reg["valid"]:
        out["reasons"].append(reg["reason"])
    if skew is None or not 0 <= skew <= 50:
        out["reasons"].append("missing driver timestamps or inter-camera skew exceeds 50 ms")
    if not all("ranked_groups" in v for v in (head, wrist)):
        out["reasons"].append("head or right-wrist detection record missing")
    if out["reasons"]:
        return out
    hd_image, wr_image = [cv2.imdecode(np.fromfile(folder/v["image_file"], np.uint8), cv2.IMREAD_COLOR)
                          for v in (head, wrist)]
    hd_roi, wr_roi = [v["detection_record"]["inference_roi_xyxy"] for v in (head,wrist)]
    matches = links(apertures(hd_image, hd_roi), apertures(wr_image, wr_roi), reg)
    wr = primary(wrist)
    out.update(status="registered", landmark_matches=matches)
    supplements = []
    for det in wr:
        review = {"wrist_index": det["index"], "class_name": det["class_name"], "confidence": det["confidence"]}
        out["wrist_reviews"].append(review)
        # A landmark shared by overlapping wrist proposals cannot identify a particular prop.
        hits = [m for m in matches if contains(det["box"], m["wrist"]["xy"]) and
                sum(contains(p["box"], m["wrist"]["xy"]) for p in wr) == 1]
        if not hits or det["confidence"] < .5:
            review["decision"] = "no_unique_landmark_or_low_confidence"
            continue
        covered = {}
        for m in hits:
            h, w = m["head"], m["wrist"]
            for p in hd:
                if contains(p["box"], h["xy"]):
                    # Relative to the same visible opening, the extent of a fragment is much
                    # shorter than the wrist detection. Ambiguous extent preserves the head result.
                    ratio = span(p["box"], h["xy"]) / max(span(det["box"], w["xy"]), 1)
                    ratio *= np.sqrt(w["area"] / h["area"])
                    covered[p["index"]] = max(covered.get(p["index"], 0), float(ratio))
        m = min(hits, key=lambda h: h["residual_px"])
        evidence = {"method": "unique_pink_aperture_on_registered_surface",
                    "landmarks": hits, "head_extent_ratios": covered}
        review.update(evidence=evidence)
        if any(ratio >= .60 for ratio in covered.values()):
            review["decision"] = "existing_head_extent"
            continue
        if any(idx not in items for idx in covered):
            review["decision"] = "fragment_claimed_by_another_candidate"
            continue
        for idx in covered:
            del items[idx]
        supplements.append({"source": "right_wrist", "source_index": det["index"],
                            "class_name": det["class_name"], "confidence": det["confidence"], "alternative": det["alternative"],
                            "head_box": None, "head_anchor": m["head"]["xy"], "wrist_box": det["box"],
                            "wrist_anchor": m["wrist"]["xy"], "evidence": evidence})
        review["decision"] = "supplement"
    out.update(items=list(items.values()) + supplements, supplements=len(supplements))
    return out
