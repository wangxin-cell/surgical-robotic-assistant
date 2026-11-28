"""Image-only exposure metering for the three fixed-role O6 USB cameras.

All reports are JSON-serializable. Exposure values use the camera's native
100-microsecond V4L2 units. This module never opens a device.
"""
import math

import cv2
import numpy as np


# Ignore the distant room above the table and the nearest part of the gripper.
# The head sees a small sheet near the centre; a wrist can see only part of it.
WORK_AREA = {
    "head": (.25, .25, .75, .72),
    "left_wrist": (.04, .25, .96, .78),
    "right_wrist": (.04, .25, .96, .78),
}


def _small_bgr(frame):
    if isinstance(frame, (bytes, bytearray, memoryview)):
        frame = np.frombuffer(frame, np.uint8)
    if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or not frame.size:
        raise ValueError("Expected a nonempty uint8 MJPEG or camera image")
    if frame.ndim == 1 or (frame.ndim in (2, 3) and 1 in frame.shape[:2]):
        frame = cv2.imdecode(frame.reshape(-1), cv2.IMREAD_REDUCED_COLOR_2)
        if frame is None:
            raise ValueError("Could not decode MJPEG exposure sample")
    if frame.ndim == 2 or (frame.ndim == 3 and frame.shape[-1] == 1):
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.ndim != 3 or frame.shape[2] != 3 or min(frame.shape[:2]) < 8:
        raise ValueError("Unsupported exposure sample shape")
    h, w = frame.shape[:2]
    scale = min(1, 384 / max(h, w))
    return cv2.resize(frame, (max(8, round(w*scale)), max(8, round(h*scale))),
                      interpolation=cv2.INTER_AREA)


def _meter_box(img, role):
    h, w = img.shape[:2]
    x, y, r, b = [round(v*n) for v, n in zip(WORK_AREA[role], (w, h, w, h))]
    area = img[y:b, x:r]
    saturation = cv2.cvtColor(area, cv2.COLOR_BGR2HSV)[:, :, 1]
    gray = cv2.cvtColor(area, cv2.COLOR_BGR2GRAY)
    # Use a broad neutral surface even if the hand or pink instruments obscure
    # part of it. Small window highlights and individual white jaw tips do not
    # have enough contiguous area to qualify as the work surface.
    median, p90 = np.percentile(gray, (50, 90))
    # In a dim image the same white sheet may only be gray. Downward adaptation
    # still requires contrast with its surroundings, avoiding flat dark walls.
    threshold = max(23., min(138., max(median + max(16., .45*median), .80*p90)))
    neutral = ((saturation <= 72) & (gray >= threshold)).astype(np.uint8)
    neutral = cv2.morphologyEx(neutral, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(neutral)
    if n > 1:
        left, top, width, height, count = map(int, stats[1 + np.argmax(stats[1:, 4])])
        if count >= .07 * gray.size and width >= .18 * (r-x) and height >= .15 * (b-y):
            return "work_surface", (x+left, y+top, x+left+width, y+top+height)
    return "central_scene", (x, y, r, b)


def meter(frame, role, reference=None):
    """Measure native BGR/gray/MJPEG. Reuse a previous report as reference
    within a stationary burst to compare exactly the same rectangle.
    """
    if role not in WORK_AREA:
        raise ValueError(f"Unknown camera role: {role}")
    img = _small_bgr(frame)
    h, w = img.shape[:2]
    if reference is None:
        method, box = _meter_box(img, role)
    else:
        if reference["role"] != role or reference["meter_size"] != [w, h]:
            raise ValueError("Exposure reference does not match camera or dimensions")
        method, box = reference["method"], reference["box"]
    x, y, r, b = box
    if method not in ("work_surface", "central_scene") or not (0 <= x < r <= w and 0 <= y < b <= h):
        raise ValueError("Invalid exposure metering rectangle")
    # The whole bounding box is measured, including darker paper detail and the
    # props. Selecting only pixels which are bright in each new frame would
    # make a darker trial change its own measurement population.
    pixels = cv2.cvtColor(img[y:b, x:r], cv2.COLOR_BGR2GRAY)
    median, p80, p90 = map(float, np.percentile(pixels, (50, 80, 90)))
    clipped = float(np.mean(pixels >= 248))
    surface = method == "work_surface"
    signal = p80 if surface else max(p80, .8*p90)
    target, low, high = (207, 162, 226) if surface else (150, 94, 199)
    # A small specular highlight or a white jaw tip in the scene must not
    # drive an otherwise legible view into the dark.
    clip_limit = .035 if surface else .09
    highlight_penalty = max(0., clipped - clip_limit) if p90 > 230 else 0.
    if signal > high or highlight_penalty > 0:
        level = "bright"
    elif signal < low and clipped < clip_limit:
        level = "dark"
    else:
        level = "ok"
    score = max(0., low - signal, signal - high) + 90 * highlight_penalty
    return dict(role=role, method=method, meter_size=[w, h], box=list(box),
                median=round(median, 2), p80=round(p80, 2), p90=round(p90, 2),
                clipped_fraction=round(clipped, 5), level=level, score=round(score, 3),
                signal=round(signal, 2), target=target)


def next_exposure(measurement, current, minimum, maximum, step):
    """Return a bounded, supported shutter step; hold within the dead band."""
    current, minimum, maximum, step = map(int, (current, minimum, maximum, step))
    if minimum < 0 or maximum < minimum or step <= 0 or not minimum <= current <= maximum:
        raise ValueError("Invalid exposure control range or current value")
    level = measurement["level"]
    if level == "ok":
        return current
    if level not in ("bright", "dark"):
        raise ValueError("Unknown exposure level")
    ratio = (measurement["target"] / max(1, measurement["signal"])) ** 1.45
    if level == "bright":
        clipped = measurement["clipped_fraction"]
        ratio = max(.45, min(ratio, .90, 1.0 - .75 * clipped))
    else:
        ratio = max(1.12, min(1.85, ratio))
    last = (maximum-minimum) // step
    index = int(round((current*ratio-minimum)/step))
    if level == "bright":
        index = min(index, math.ceil((current-minimum)/step)-1)
    else:
        index = max(index, math.floor((current-minimum)/step)+1)
    return minimum + min(last, max(0, index))*step
