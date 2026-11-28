"""Optional YOLO localization for pink tabletop practice props in BGR crops."""
import time

import cv2
import numpy as np


def _iou(box, other):
    left = max(box[0], other[0])
    top = max(box[1], other[1])
    right = min(box[2], other[2])
    bottom = min(box[3], other[3])
    intersection = max(0, right-left) * max(0, bottom-top)
    area = (box[2]-box[0])*(box[3]-box[1]) + (other[2]-other[0])*(other[3]-other[1])
    return intersection / max(1., area-intersection)


def _pink_components(image):
    blue, green, red = cv2.split(image.astype(np.int16))
    hue, sat, value = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))
    mask = (red-green > 10) & (blue-green > 1) & (hue > 125) & (hue < 178) & (sat > 16) & (value > 45)
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    return stats[1:]


def _pink_regions(image, components=None):
    height, width = image.shape[:2]
    short_side = min(height, width)
    boxes = []
    for x, y, w, h, area in (_pink_components(image) if components is None else components):
        if (area < max(28, short_side**2*.0007) or max(w,h) < .12*short_side
                or min(w,h) < 3 or area > height*width*.23 or area/(w*h) < .072):
            continue
        margin = max(1, round(.006*max(w,h)))
        boxes.append([max(0,int(x-margin)), max(0,int(y-margin)),
                      min(width,int(x+w+margin)), min(height,int(y+h+margin))])
    return boxes


def close_prop_roi(frame, base_roi):
    """Select a pixel-only inference crop for pink props on a light work surface."""
    x, y, right, bottom = base_roi
    patch = frame[y:bottom, x:right]
    height, width = patch.shape[:2]
    components = _pink_components(patch)
    candidates = _pink_regions(patch, components)
    detail = {"method": "base_region", "candidate_count": len(candidates)}
    if not candidates:
        return list(base_roi), detail

    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    sat = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)[:, :, 1]
    light = (gray >= 137) & (sat < 88)
    boxes = []
    on_light_surface = 0
    for box in candidates:
        left, top, r, b = box
        margin = max(5, round(.12 * max(r-left, b-top)))
        surround = light[max(0, top-margin):b+margin, max(0, left-margin):r+margin]
        supported = surround.mean() >= .15
        on_light_surface += int(supported)
        # A clipped pink cable or prop in the dark foreground should not set the
        # zoom when complete props lie on the bright table. Include complete
        # silhouettes even on dark supports to avoid losing nearby objects.
        if supported or (left > 0 and top > 0 and r < width and b < height):
            boxes.append(box)
    if not on_light_surface:
        return list(base_roi), detail

    # Leave about half a prop length of background beyond the union on each
    # side, including room for faint tips and gaps in a partially occluded prop.
    length = max(max(r-l, b-t) for l, t, r, b in boxes)
    pad = max(8, round(.5 * length))
    roi = [x + max(0, min(l for l, t, r, b in boxes) - pad),
           y + max(0, min(t for l, t, r, b in boxes) - pad),
           x + min(width, max(r for l, t, r, b in boxes) + pad),
           y + min(height, max(b for l, t, r, b in boxes) + pad)]
    # A desaturated tool may split into several components. Smaller tip
    # fragments cannot initiate a zoom but must not be sliced by its boundary.
    fragments = [(x+int(l), y+int(t), x+int(l+w), y+int(t+h))
                 for l, t, w, h, area in components
                 if area >= max(12, .00006 * min(height, width)**2)]
    edge_margin = max(3, round(.05 * length))
    while True:
        left, top, r, b = roi
        crossing = [f for f in fragments
                    if f[0] < r and f[2] > left and f[1] < b and f[3] > top
                    and (f[0] < left or f[1] < top or f[2] > r or f[3] > b)]
        if not crossing:
            break
        for fl, ft, fr, fb in crossing:
            if fl < left:
                roi[0] = min(roi[0], max(x, fl-edge_margin))
            if ft < top:
                roi[1] = min(roi[1], max(y, ft-edge_margin))
            if fr > r:
                roi[2] = max(roi[2], min(right, fr+edge_margin))
            if fb > b:
                roi[3] = max(roi[3], min(bottom, fb+edge_margin))
    detail.update(used_components=len(boxes), light_surface_components=on_light_surface)
    if (roi[2]-roi[0]) * (roi[3]-roi[1]) >= .95 * width * height:
        return list(base_roi), detail
    detail["method"] = "padded_pink_silhouettes"
    return roi, detail


def _predict(model, image, conf, imgsz, device):
    result = model.predict(image, imgsz=imgsz, conf=conf, device=device, verbose=False)[0]
    return [{"class_id": int(cls), "confidence": float(score), "box_xyxy_pixels": row, "source": "yolo"}
            for row, score, cls in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(), result.boxes.cls.tolist())]


def _centered_views(image, box, fill_color):
    x1, y1, x2, y2 = box
    cx, cy = (x1+x2)/2, (y1+y2)/2
    length = max(x2-x1,y2-y1)
    height, width = image.shape[:2]
    for multiplier, rotate in ((1.65,False), (1.65,True), (2.2,False)):
        size = max(30, round(multiplier*length))
        x, y = round(cx-size/2), round(cy-size/2)
        left, right = max(0,x), min(width,x+size)
        top, bottom = max(0,y), min(height,y+size)
        view = np.empty((size,size,3), dtype=np.uint8)
        view[:] = fill_color
        view[top-y:bottom-y,left-x:right-x] = image[top:bottom,left:right]
        target = [x1-x,y1-y,x2-x,y2-y]
        if rotate:
            view = cv2.rotate(view,cv2.ROTATE_90_CLOCKWISE)
            a,b,c,d = target
            target = [size-d,a,size-b,c]
        yield view,target


def predict_shapes(model, crop, *, conf=.25, imgsz=640, device="cpu", max_extra_passes=6):
    """Run a local Ultralytics YOLO model on an OpenCV BGR tabletop crop.

    Returns {detections, proposal_count, yolo_passes, elapsed_ms}. Each detection has
    class_id, confidence, box_xyxy_pixels (crop coordinates), source. Only the model
    supplies classes. max_extra_passes=0 performs a single full-crop pass with box
    refinement. Six extra views is a frame-wide budget for optional local recovery.
    """
    if crop is None or crop.dtype != np.uint8 or crop.ndim != 3 or crop.shape[2] != 3 or not crop.size:
        raise ValueError("Expected a nonempty uint8 BGR crop")
    if max_extra_passes < 0:
        raise ValueError("max_extra_passes must be nonnegative")
    start = time.perf_counter()
    detections = _predict(model,crop,conf,imgsz,device)
    regions = _pink_regions(crop)
    original = [row["box_xyxy_pixels"] for row in detections]
    candidates = sorted(((_iou(box,p),i,j) for i,box in enumerate(original)
                         for j,p in enumerate(regions)), reverse=True)
    used_detections, used_regions = set(), set()
    for overlap, i, j in candidates:
        if overlap < .32:
            break
        if i in used_detections or j in used_regions:
            continue
        # One connected blob shared by several YOLO objects is a poor boundary cue.
        if sum(_iou(box,regions[j]) > .30 for box in original) > 1:
            continue
        detections[i]["box_xyxy_pixels"] = regions[j]
        detections[i]["source"] = "yolo_refined"
        used_detections.add(i)
        used_regions.add(j)

    extra = 0
    if max_extra_passes >= 2:
        fill_color = np.median(crop[::13,::13],axis=(0,1)).astype(np.uint8)
        for j, region in enumerate(regions):
            if extra+2 > max_extra_passes:
                break
            if j in used_regions or any(_iou(row["box_xyxy_pixels"],region) >= .27 for row in detections):
                continue
            votes = {}
            for view_no, (view,target) in enumerate(_centered_views(crop,region,fill_color), 1):
                if extra >= max_extra_passes:
                    break
                pred = _predict(model,view,conf,imgsz,device)
                extra += 1
                hits = [(row["confidence"]*overlap,row) for row in pred
                        if (overlap := _iou(row["box_xyxy_pixels"],target)) >= .36]
                if hits:
                    row = max(hits,key=lambda pair: pair[0])[1]
                    cls = row["class_id"]
                    if cls in votes:
                        detections.append({"class_id": cls, "confidence": min(votes[cls], row["confidence"]),
                                           "box_xyxy_pixels": region, "source": "two_local_yolo_views"})
                        break
                    votes[cls] = row["confidence"]
                if view_no == 2 and not votes:
                    break
    return {"detections": detections, "proposal_count": len(regions), "yolo_passes": 1+extra,
            "elapsed_ms": round((time.perf_counter()-start)*1000,1)}


def predict_table_crop(model, crop, pipeline, source):
    """Produce the same detection record for a stored O6 crop or a live camera crop."""
    from predict import result_record

    if pipeline == "plain":
        start = time.perf_counter()
        pred = model.predict(crop, conf=.25, imgsz=640, device="cpu", verbose=False, save=False)[0]
        record = result_record(pred, source, 0, 0)
        record.update(yolo_passes=1, elapsed_ms=round(1000*(time.perf_counter()-start), 1))
        return record
    if pipeline not in ("snap", "refine"):
        raise ValueError(f"Unknown tabletop detector mode: {pipeline}")
    aux = predict_shapes(model, crop, max_extra_passes=0 if pipeline == "snap" else 6)
    names = model.names
    counts = dict.fromkeys(names.values(), 0)
    for det in aux["detections"]:
        name = names[det["class_id"]]
        det["class_name"] = name
        counts[name] += 1
    return dict(source=str(source), source_index=0, frame_index=0, original_width=crop.shape[1],
                original_height=crop.shape[0], class_counts=counts, **aux)

