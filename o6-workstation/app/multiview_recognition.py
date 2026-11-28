"""Run the same shape detector on individual frames from a completed O6 capture."""
import base64

import cv2
import numpy as np


def recognize(report, model, predict, roi_selector, table_finder, annotate):
    """Attach independent, full-image pixel detections without assuming object correspondence."""
    for index, view in enumerate(report["views"]):
        if not view.get("jpeg"):
            continue
        try:
            raw = np.frombuffer(base64.b64decode(view["jpeg"], validate=True), np.uint8)
            frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("Invalid JPEG")
            h, w = frame.shape[:2]
            base = [0, 0, w, h]
            if view["role"] == "head":
                table = table_finder(frame)
                view["table_region"] = table
                if table["verified"]:
                    base = table["crop_xyxy"]
                else:
                    dx, dy = round(.225*w), round(.19*h)
                    base = [dx, dy, w-dx, h-dy]
            roi, detail = roi_selector(frame, base)
            x, y, right, bottom = roi
            record = predict(model, frame[y:bottom, x:right], "plain", view["device_by_path"])
            for det in record["detections"]:
                l, t, r, b = det["box_xyxy_pixels"]
                det["box_xyxy_pixels"] = [x+l, y+t, x+r, y+b]
            record.update(original_width=w, original_height=h, source_index=index,
                          coordinate_system="original_image_pixels", inference_roi_xyxy=roi)
            view.update(detection_record=record, roi_detail=detail)
            overlay = annotate(frame, record, cv2)
            ok, jpg = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if not ok:
                raise ValueError("Could not encode annotated JPEG")
            view["annotated_jpeg"] = base64.b64encode(jpg).decode("ascii")
        except Exception as exc:
            view["inference_error"] = str(exc)
