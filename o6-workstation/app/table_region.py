"""Locate the work surface in a full head image."""
import cv2
import numpy as np

def find_table(frame):
    """Find the bright sheet in the fixed head-camera work area at any input size."""
    height, width = frame.shape[:2]
    scale = min(1.0, 640 / width)
    small = cv2.resize(frame, (round(width * scale), round(height * scale)), interpolation=cv2.INTER_AREA)
    sh, sw = small.shape[:2]
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    saturation = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)[:, :, 1]
    neutral = ((gray >= 150) & (saturation < 83)).astype(np.uint8) * 255
    neutral = cv2.morphologyEx(neutral, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    neutral = cv2.morphologyEx(neutral, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(neutral, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not .025 * sh * sw < area < .65 * sh * sw:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        cx, cy = (x + w / 2) / sw, (y + h / 2) / sh
        if not (.17 < cx < .84 and .40 < cy < .91 and 1.08 < w / h < 4.4):
            continue
        hull = cv2.convexHull(contour)
        if area / (w * h) < .65 or area / cv2.contourArea(hull) < .83:
            continue
        if x <= 0 or y <= 0 or x + w >= sw or y + h >= sh:
            continue
        candidates.append((area, x, y, w, h, hull))
    candidates.sort(key=lambda c: c[0], reverse=True)
    if not candidates or (len(candidates) > 1 and candidates[1][0] > .65 * candidates[0][0]):
        return {"verified": False, "reason": "Work sheet missing or multiple bright work surfaces match"}
    _, x, y, w, h, hull = candidates[0]
    sx, sy = width / sw, height / sh
    paper = [int(np.floor(x*sx)), int(np.floor(y*sy)),
             int(np.ceil((x+w)*sx)), int(np.ceil((y+h)*sy))]
    # Include props which extend just beyond the visible border.
    mx, my = round(.045*w*sx), round(.075*h*sy)
    crop = [max(0, paper[0]-mx), max(0, paper[1]-my),
            min(width, paper[2]+mx), min(height, paper[3]+my)]
    outline = np.rint(hull.reshape(-1, 2) * [sx, sy]).astype(int).tolist()
    return {"verified": True, "sheet_bbox_xyxy": paper, "crop_xyxy": crop,
            "sheet_hull_xy": outline, "method": "isolated central bright low-saturation sheet"}
