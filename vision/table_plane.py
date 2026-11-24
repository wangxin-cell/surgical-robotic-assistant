"""Convert native head-camera pixels to millimetres on the marked tabletop plane."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def table_xy(calibration, u, v):
    point = (float(u), float(v))
    boundary = np.array(calibration['corner_pixels_tl_tr_br_bl'], dtype=np.float32)
    if not np.isfinite(point).all() or cv2.pointPolygonTest(boundary, point, False) < 0:
        raise ValueError('Point is outside the calibrated quadrilateral')
    matrix = np.array(calibration['pixel_to_mm'], dtype=np.float64)
    mapped = cv2.perspectiveTransform(np.array([[point]], dtype=np.float64), matrix)[0, 0]
    return float(mapped[0]), float(mapped[1])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('calibration', type=Path)
    parser.add_argument('u', type=float)
    parser.add_argument('v', type=float)
    args = parser.parse_args()
    data = json.loads(args.calibration.read_text(encoding='utf-8'))
    try:
        x, y = table_xy(data, args.u, args.v)
    except ValueError as error:
        parser.error(str(error))
    print(json.dumps(dict(u=args.u, v=args.v, x_mm=round(x, 3), y_mm=round(y, 3))))
