def ranked_shape_groups(detections):
    """Group substantially overlapping boxes for display. Keep all raw YOLO detections."""
    def overlap(a, b):
        w = max(0, min(a[2], b[2]) - max(a[0], b[0]))
        h = max(0, min(a[3], b[3]) - max(a[1], b[1]))
        area = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1])
        return w*h / max(area - w*h, 1)

    groups = []
    for idx in sorted(range(len(detections)), key=lambda i: detections[i]["confidence"], reverse=True):
        box = detections[idx]["box_xyxy_pixels"]
        scores = [overlap(box, detections[group[0]]["box_xyxy_pixels"]) for group in groups]
        if scores and max(scores) >= .55:
            groups[scores.index(max(scores))].append(idx)
        else:
            groups.append([idx])
    return sorted(groups, key=lambda g: sum(detections[g[0]]["box_xyxy_pixels"][::2]))
