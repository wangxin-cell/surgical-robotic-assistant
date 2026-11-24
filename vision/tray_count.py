"""Fixed tray counting zone, measured in the head camera's 640 x 480 view."""
import cv2
import numpy as np


POLYGON = [(248, 127), (438, 119), (456, 306), (238, 311)]
MIN_TRAY_FRACTION = 0.65
DUPLICATE_IOU = 0.80
SCISSOR_CLASSES = {'straight_mayo_scissor', 'curved_mayo_scissor'}


class TrayVisibility:
    """Flag broad foreground objects against the empty, fixed dark board."""
    def __init__(self, reference):
        if reference is None or reference.shape != (480, 640, 3):
            raise ValueError('Empty-board reference must be a 640 x 480 BGR frame')
        polygon = np.array(POLYGON, np.int32)
        x, y, w, h = cv2.boundingRect(polygon)
        self.crop = (slice(y, y+h), slice(x, x+w))
        self.reference = reference[self.crop].copy()
        mask = np.zeros((h, w), np.uint8)
        cv2.fillConvexPoly(mask, polygon - (x, y), 255)
        self.mask = cv2.erode(mask, np.ones((9, 9), np.uint8), borderType=cv2.BORDER_CONSTANT,
                              borderValue=0)
        self.area = cv2.countNonZero(self.mask)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (27, 27))

    def measure(self, frame):
        if frame.shape != (480, 640, 3):
            raise ValueError('Unexpected size for fixed tray visibility check')
        difference = cv2.absdiff(frame[self.crop], self.reference).max(axis=2)
        foreground = cv2.medianBlur(np.uint8(difference > 36) * 255, 5) & self.mask
        # Thin scissor blades and handles vanish after erosion. A palm keeps a broad interior.
        interior = cv2.erode(foreground, self.kernel, borderType=cv2.BORDER_CONSTANT, borderValue=0)
        _, _, stats, _ = cv2.connectedComponentsWithStats(interior)
        fraction = float(stats[1:, cv2.CC_STAT_AREA].max()) / self.area if len(stats) > 1 else 0.0
        return dict(blocked=fraction >= 0.003, broad_fraction=round(fraction, 5))


def intersection_area(a, b):
    return max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0, min(a[3], b[3]) - max(a[1], b[1]))


def iou(a, b):
    intersection = intersection_area(a, b)
    union = ((a[2] - a[0]) * (a[3] - a[1]) +
             (b[2] - b[0]) * (b[3] - b[1]) - intersection)
    return intersection / union if union > 0 else 0


def repeats_scissors(candidate, accepted):
    # A weaker prediction can widen around one tool, or enclose two separately detected tools.
    enclosed = 0
    for detection in accepted:
        if detection['class_name'] not in SCISSOR_CLASSES:
            continue
        box = detection['box_xyxy_pixels']
        area = (box[2] - box[0]) * (box[3] - box[1])
        if intersection_area(candidate, box) >= 0.90 * area:
            if iou(candidate, box) >= 0.50:
                return True
            enclosed += 1
    return enclosed >= 2


def tray_result(record):
    if (record['original_width'], record['original_height']) != (640, 480):
        raise ValueError('Tray calibration requires 640 x 480')
    polygon = np.array(POLYGON, np.float32)
    detections = record['detections']
    kept = []
    for index in sorted(range(len(detections)), key=lambda i: -detections[i]['confidence']):
        box = detections[index]['box_xyxy_pixels']
        x1, y1, x2, y2 = box
        area = (x2 - x1) * (y2 - y1)
        center = ((x1 + x2) / 2, (y1 + y2) / 2)
        if area <= 0 or cv2.pointPolygonTest(polygon, center, False) < 0:
            continue
        rect = np.array([(x1, y1), (x2, y1), (x2, y2), (x1, y2)], np.float32)
        overlap, _ = cv2.intersectConvexConvex(polygon, rect)
        if overlap < area * MIN_TRAY_FRACTION:
            continue
        if any(iou(box, detections[j]['box_xyxy_pixels']) >= DUPLICATE_IOU for j in kept):
            continue
        if detections[index]['class_name'] in SCISSOR_CLASSES and repeats_scissors(
                box, (detections[j] for j in kept)):
            continue
        kept.append(index)
    scissors = sum(detections[i]['class_name'] in SCISSOR_CLASSES for i in kept)
    return dict(kept_indices=kept, total=len(kept), scissors=scissors)


def overlay(frame, record, counted, annotate):
    kept = [record['detections'][i] for i in counted['kept_indices']]
    image = annotate(frame, {'detections': kept}, cv2)
    cv2.polylines(image, [np.array(POLYGON, np.int32)], True, (50, 235, 90), 2)
    cv2.rectangle(image, (0, 0), (305, 32), (25, 25, 25), -1)
    cv2.putText(image, f"Tray: {counted['total']}    Scissors: {counted['scissors']}",
                (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    return image
