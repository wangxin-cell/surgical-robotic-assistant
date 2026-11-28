"""Verify a local head-to-right-wrist mapping using the stationary dotted work sheet.

The fifth reviewed dot checks the four-dot fit. Coordinates are image pixels on
the plane of the sheet. A wrist movement outside the reference windows requires
a new independent correspondence review.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


REFERENCE = Path(__file__).with_name("paper_reference.json")


def intensity(image):
    # White paper and pink plastic both have a bright channel. Dark ink has none.
    return image.max(axis=2)


def read_image(path):
    data = path.read_bytes()
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"Cannot decode {path.name}")
    return data, img


def find_dot(ref, frame, point, estimate=(0,0), search=8):
    x, y = (round(v) for v in point)
    radius = 8
    height, width = frame.shape
    px, py = x+round(estimate[0]), y+round(estimate[1])
    x0, y0 = max(0, px-radius-search), max(0, py-radius-search)
    patch = ref[y-radius:y+radius+1, x-radius:x+radius+1]
    if patch.shape != (2*radius+1, 2*radius+1):
        raise ValueError("Reference dot is too close to the edge")
    area = frame[y0:min(height, py+radius+search+1), x0:min(width, px+radius+search+1)]
    if min(area.shape) < 2*radius+1:
        raise ValueError("Dot window moved outside image")
    scores = cv2.matchTemplate(area, patch, cv2.TM_CCOEFF_NORMED)
    _, best, _, (u,v) = cv2.minMaxLoc(scores)
    others = scores.copy()
    others[max(0,v-5):v+6, max(0,u-5):u+6] = -1
    if best < .78 or (others.max() > .70 and best - others.max() < .10):
        raise ValueError(f"Missing or ambiguous ink dot near ({x}, {y}), score {best:.2f}")
    obs = area[v:v+patch.shape[0],u:u+patch.shape[1]]
    if np.std(obs) < max(5., .48*np.std(patch)):
        raise ValueError(f"Ink contrast lost near ({x}, {y})")
    cx, cy = x0+u+radius, y0+v+radius
    if max(abs(cx-px), abs(cy-py)) >= search:
        raise ValueError("Dot drift exceeds the fixed-pose search range")
    def parabola(l, c, r):
        denom = float(l)+float(r)-2*float(c)
        return float(np.clip((float(l)-float(r))/(2*denom), -.5, .5)) if denom < -1e-6 else 0.
    du = parabola(*scores[v,u-1:u+2]) if 0 < u < scores.shape[1]-1 else 0.
    dv = parabola(*scores[v-1:v+2,u]) if 0 < v < scores.shape[0]-1 else 0.
    return [float(point[0]+cx-x+du), float(point[1]+cy-y+dv)], round(float(best), 4)


def track_reference_dots(ref, frame, seeds, initial=(0, 0), search=32):
    coarse = [find_dot(ref, frame, xy, estimate=initial, search=search)[0] for xy in seeds[:4]]
    shifts = np.asarray(coarse) - seeds[:4]
    offset = np.median(shifts, axis=0)
    if np.max(np.linalg.norm(shifts-offset, axis=1)) > 8:
        raise ValueError("The four paper-dot shifts are inconsistent")
    return [find_dot(ref, frame, xy, estimate=offset) for xy in seeds]


FIT_GRID = np.float32([[0,0], [1,0], [0,1], [1,1]])


def reference_left_column(ref, seeds):
    # The hand-marked paper's column spacing is unequal. Estimate it from
    # the unused near-left dot in the original head photograph only.
    h = cv2.getPerspectiveTransform(FIT_GRID, seeds[:4])
    xy = cv2.perspectiveTransform(np.float32([[[-1,1]]]), h)[0,0]
    point, _ = find_dot(ref, ref, seeds[2], estimate=xy-seeds[2], search=16)
    gx, gy = cv2.perspectiveTransform(np.float32([[point]]), np.linalg.inv(h))[0,0]
    if not (-1.16 < gx < -.84 and abs(gy-1) < .08):
        raise ValueError("Could not verify the unused left-column dot in the reference head image")
    return float(gx)


def grid_cross_checks(ref, frame, seeds, matches, left_x):
    # In the 3-by-3 grid the four fit dots occupy middle/right columns and
    # middle/near rows. Seek a dark ink dot independently in the left column.
    fit = np.float32([m[0] for m in matches[:4]])
    lattice = cv2.getPerspectiveTransform(FIT_GRID, fit)
    checks = np.float32([[left_x,-1], [left_x,0], [left_x,1]])
    predicted = cv2.perspectiveTransform(checks[:,None,:], lattice).reshape(-1,2)
    verified = []
    for row, position in enumerate(predicted):
        x, y = position
        if not (12 <= x < frame.shape[1]-12 and 12 <= y < frame.shape[0]-12):
            continue
        for seed in seeds[:4]:
            try:
                point, score = find_dot(ref, frame, seed, estimate=position-seed, search=8)
            except ValueError:
                continue
            err = np.linalg.norm(position-point)
            if score >= .80 and err <= 5:
                verified.append({"grid_xy": [-1, row-1], "image_xy": point,
                                 "prediction_error_px": round(float(err), 3), "score": score})
                break
    return verified


def recover_grid(ref, frame, seeds, left_x):
    # Generate alternative lattice origins. Identical dots recur one cell apart,
    # so accepting the largest correlation peak alone would be unsafe.
    radius, search = 8, 108
    x, y = np.rint(seeds[0]).astype(int)
    x0, y0 = max(0,x-search-radius), max(0,y-search-radius)
    roi = frame[y0:y+search+radius+1, x0:x+search+radius+1]
    patch = ref[y-radius:y+radius+1, x-radius:x+radius+1]
    scores = cv2.matchTemplate(roi, patch, cv2.TM_CCOEFF_NORMED)
    candidates = []
    for _ in range(40):
        _, score, _, (u,v) = cv2.minMaxLoc(scores)
        if score < .78:
            break
        scores[max(0,v-6):v+7, max(0,u-6):u+7] = -1
        offset = np.array([x0+u+radius-x, y0+v+radius-y])
        if max(abs(offset)) >= search:
            continue
        try:
            matches = track_reference_dots(ref, frame, seeds, initial=offset, search=14)
        except ValueError:
            continue
        checks = grid_cross_checks(ref, frame, seeds, matches, left_x)
        if checks and all(np.linalg.norm(np.array(matches[0][0])-m[0][0]) > 5
                          for m, _ in candidates):
            candidates.append((matches, checks))
    else:
        if scores.max() >= .78:
            raise ValueError("Too many plausible lattice origins")
    if len(candidates) != 1:
        raise ValueError(f"Expected one verified three-column lattice, found {len(candidates)}")
    return candidates[0]


def register_paper(folder, config=REFERENCE):
    """Return a checked image-plane homography, or valid=False with an explanation."""
    folder, config = Path(folder), Path(config)
    try:
        setup = json.loads(config.read_bytes())
        cap = json.loads((folder/"result.json").read_bytes())
        timing = cap["timing"]
        skew_key = ("driver_timestamp_skew_ms" if timing.get("match_clock") == "v4l2_frame_ms"
                    else "host_receive_skew_ms")
        skew = timing.get(skew_key)
        if not cap.get("complete") or skew is None or not 0 <= skew <= 50:
            raise ValueError("Triplet incomplete or time spread exceeds 50 ms")
        points, strengths, tracking, inputs = {}, {}, {}, {}
        for role in ("head", "right_wrist"):
            spec = setup["roles"][role]
            candidates = [v for v in cap["views"] if v["role"] == role]
            if len(candidates) != 1 or candidates[0]["device_by_path"] != spec["device_by_path"]:
                raise ValueError(f"{role}: camera identity changed")
            reference_dir = config.parent / setup["reference_folder"]
            data, reference = read_image(reference_dir / spec["reference_image"])
            if hashlib.sha256(data).hexdigest() != spec["sha256"]:
                raise ValueError(f"{role}: reference JPEG changed")
            _, image = read_image(folder / candidates[0]["image_file"])
            if spec["size_wh"] != list(image.shape[1::-1]) or reference.shape != image.shape:
                raise ValueError(f"{role}: image size differs from the reference")
            ref, frame = intensity(reference), intensity(image)
            seeds = np.asarray(spec["points_xy"], np.float32)
            inputs[role] = (ref, frame, seeds)
            try:
                matches = track_reference_dots(ref, frame, seeds)
                tracking[role] = {"method": "fixed_pose"}
            except ValueError as local:
                try:
                    head_ref, _, head_seeds = inputs["head"]
                    left_x = reference_left_column(head_ref, head_seeds)
                    matches, checks = recover_grid(ref, frame, seeds, left_x)
                except ValueError as recovery:
                    raise ValueError(f"{role}: {local}. Grid recovery: {recovery}") from recovery
                tracking[role] = {"method": "grid_reacquisition", "left_column_checks": checks}
            if len(matches) != 5:
                raise ValueError("Need four fit points and one held-out point")
            points[role] = np.array([m[0] for m in matches], np.float32)
            strengths[role] = [m[1] for m in matches]
        head, right = points["head"], points["right_wrist"]
        h = cv2.getPerspectiveTransform(head[:4], right[:4])
        check = cv2.perspectiveTransform(head[4:].reshape(-1,1,2), h).reshape(-1,2)[0]
        err = float(np.linalg.norm(check-right[4]))
        if not np.isfinite(err) or err > 5.:
            raise ValueError(f"Held-out paper dot error {err:.2f} px exceeds 5 px")
        extra = []
        if any(v["method"] == "grid_reacquisition" for v in tracking.values()):
            for role, method in tracking.items():
                if "left_column_checks" not in method:
                    method["left_column_checks"] = grid_cross_checks(
                        *inputs[role], list(zip(points[role], strengths[role])), left_x)
            for hc in tracking["head"]["left_column_checks"]:
                for rc in tracking["right_wrist"]["left_column_checks"]:
                    if hc["grid_xy"] == rc["grid_xy"]:
                        projected = cv2.perspectiveTransform(np.float32([[hc["image_xy"]]]), h)[0,0]
                        error = float(np.linalg.norm(projected-rc["image_xy"]))
                        if not np.isfinite(error) or error > 5:
                            raise ValueError("Independent left-column cross-view check exceeds 5 px")
                        extra.append({"grid_xy": hc["grid_xy"], "error_right_px": round(error,3)})
            if not extra:
                raise ValueError("Grid recovery needs an independent left-column dot in both cameras")
        return {"valid": True, "reason": "paper_dots_verified", "head_to_right_h": h.tolist(),
                "head_points": head.tolist(), "right_points": right.tolist(),
                "check_error_right_px": round(err,3), "match_scores": strengths,
                "scope": "fixed_pose_sheet_plane_pixels", "skew_ms": skew, "tracking": tracking,
                "extra_cross_view_checks": extra}
    except (OSError, ValueError, KeyError, TypeError, cv2.error) as exc:
        return {"valid": False, "reason": str(exc)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--reference", type=Path, default=REFERENCE)
    args = parser.parse_args()
    result = register_paper(args.capture, args.reference)
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result["valid"] else 1)
