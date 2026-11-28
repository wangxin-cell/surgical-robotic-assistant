"""Record YOLO detections and draw the detector overlays."""
import math

def result_record(result, source, source_index, frame_index):
    height, width = (int(v) for v in result.orig_shape)
    names = result.names
    if not isinstance(names, dict):
        names = dict(enumerate(names))
    names = {int(key): str(value) for key, value in names.items()}
    counts = {name: 0 for name in names.values()}
    detections = []
    if result.boxes is not None:
        boxes = result.boxes.xyxy.cpu().tolist()
        scores = result.boxes.conf.cpu().tolist()
        class_ids = result.boxes.cls.cpu().tolist()
        for coords, score, class_id in zip(boxes, scores, class_ids):
            class_id = int(class_id)
            name = names.get(class_id, str(class_id))
            detections.append({
                "class_id": class_id,
                "class_name": name,
                "confidence": float(score),
                "box_xyxy_pixels": [float(v) for v in coords],
            })
            counts[name] = counts.get(name, 0) + 1
    return {
        "source": str(source),
        "source_index": source_index,
        "frame_index": frame_index,
        "original_width": width,
        "original_height": height,
        "detections": detections,
        "class_counts": counts,
    }


def annotate(frame, record, cv2):
    """Use OpenCV's bundled fonts so plotting never needs a font download."""
    canvas = frame.copy()
    height, width = canvas.shape[:2]
    thickness = max(1, round(min(height, width) / 400))
    for detection in record["detections"]:
        class_id = detection["class_id"]
        color = ((67 * class_id + 70) % 200 + 55,
                 (113 * class_id + 30) % 200 + 55,
                 (157 * class_id + 110) % 200 + 55)
        x1, y1, x2, y2 = (round(v) for v in detection["box_xyxy_pixels"])
        x1, x2 = (min(max(x, 0), width - 1) for x in (x1, x2))
        y1, y2 = (min(max(y, 0), height - 1) for y in (y1, y2))
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)
        name = detection["class_name"].encode("ascii", "replace").decode("ascii")
        label = f"{name} {detection['confidence']:.2f}"
        cv2.putText(canvas, label, (x1, max(16, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, max(1, thickness), cv2.LINE_AA)
    return canvas
