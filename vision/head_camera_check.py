"""Measure a short, headless capture/detection loop on the O6 and save its last frame."""
import argparse
import json
import os
from pathlib import Path
import statistics
import tempfile
import time


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True, type=Path, help='Verified head camera, V4L2 by-path index 0')
    parser.add_argument('--seconds', default=12, type=int, choices=range(1, 601), metavar='1..600')
    parser.add_argument('--tray', action='store_true', help='Count instruments on the fixed dark board')
    parser.add_argument('--events', action='store_true', help='Continuously log stable tray count changes')
    parser.add_argument('--expected-scissors', type=int, help='Enable on-demand closing check against this inventory')
    parser.add_argument('--empty-reference', type=Path, help='Guard counts using a matching empty-board photograph')
    parser.add_argument('--pick-left', action='store_true', help='Verify a request for the left scissors on a fixed tray')
    args = parser.parse_args()
    if args.pick_left and not args.empty_reference:
        parser.error('Left-target verification needs the empty-board reference to suspend decisions during occlusion')
    if args.empty_reference:
        args.events = True
    if args.expected_scissors is not None:
        if args.expected_scissors < 0:
            parser.error('Expected scissors must be nonnegative')
        args.events = True
    if args.events:
        args.tray = True
    device = args.device.resolve(strict=True)
    node = Path('/sys/class/video4linux') / device.name
    if (node / 'device/driver').resolve().name != 'uvcvideo' or (node / 'index').read_text().strip() != '0':
        parser.error('Expected primary USB video capture node')
    weights = root / 'weights/best.pt'
    if not weights.is_file():
        parser.error('Missing weights/best.pt')
    os.environ.update(YOLO_OFFLINE='true', YOLO_AUTOINSTALL='false',
                      YOLO_CONFIG_DIR=str(root/'.ultralytics'), MPLCONFIGDIR=str(root/'.matplotlib'))
    import cv2
    from ultralytics import YOLO
    from predict import annotate, result_record
    if args.tray:
        import tray_count
    guard = None
    if args.empty_reference:
        guard = tray_count.TrayVisibility(cv2.imread(str(args.empty_reference)))
    pick = None
    if args.pick_left:
        from target_pick import LeftPick, save_pick_event
        pick = LeftPick()
        pick_events = []

    model = YOLO(str(weights), task='detect')
    folder = root/'results'
    folder.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='tray_session_' if args.events else 'head_camera_', dir=folder))
    session = None
    camera = cv2.VideoCapture(str(device), cv2.CAP_V4L2)
    try:
        if not camera.isOpened():
            raise RuntimeError(f'Cannot open {device}')
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        options = dict(device='cpu', conf=0.25, imgsz=640, verbose=False, save=False)
        for _ in range(3):
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError('Camera warmup read failed')
            model.predict(frame, **options)
        prediction_times = []
        frame_counts = []
        tray_frames = []
        latest_detection_frame = None
        latest_detection_record = None
        started = time.perf_counter()
        if args.events:
            from tray_events import TraySession
            session = TraySession(output, expected_scissors=args.expected_scissors)
        while time.perf_counter() - started < args.seconds:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError('Camera read failed during timed run')
            begin_prediction = time.perf_counter()
            result = model.predict(frame, **options)[0]
            prediction_times.append((time.perf_counter() - begin_prediction) * 1000)
            record = result_record(result, device, 0, len(prediction_times) - 1)
            if args.tray:
                counted = tray_count.tray_result(record)
                last_frame = tray_count.overlay(frame, record, counted, annotate)
                tray_frames.append(dict(**counted, detections=record['detections']))
                if session:
                    visibility = guard.measure(frame) if guard else None
                    sample = session.observe(len(prediction_times) - 1, counted, last_frame, visibility)
                    if pick:
                        boxes = [record['detections'][i]['box_xyxy_pixels'] for i in counted['kept_indices']]
                        valid = not sample['blocked'] and counted['scissors'] == counted['total']
                        event = pick.observe(boxes, valid, sample['elapsed_s'])
                        pick.draw(last_frame)
                        if event:
                            pick_events.append(save_pick_event(output, sample, event, last_frame))
            else:
                last_frame = annotate(frame, record, cv2)
            frame_counts.append(record['class_counts'])
            if record['detections']:
                latest_detection_frame = last_frame
                latest_detection_record = record
            if session and (output/'stop').exists():
                break
        elapsed = time.perf_counter() - started
    finally:
        camera.release()
        if session:
            session.close()
    if not cv2.imwrite(str(output/'preview.jpg'), last_frame):
        raise RuntimeError('Could not save the final annotated frame')
    if args.tray and not cv2.imwrite(str(output/'raw.jpg'), frame):
        raise RuntimeError('Could not save the unannotated tray frame')
    if latest_detection_frame is not None:
        if not cv2.imwrite(str(output/'latest_detection.jpg'), latest_detection_frame):
            raise RuntimeError('Could not save the last frame with a detection')
    report = dict(device=str(device), by_path=str(args.device), output=str(output),
                  width=record['original_width'], height=record['original_height'],
                  frames=len(prediction_times), seconds=elapsed, processed_fps=len(prediction_times)/elapsed,
                  median_predict_ms=statistics.median(prediction_times), confidence=0.25,
                  timing_scope='capture + CPU prediction + counts + annotation; after warmup, before disk write',
                  frames_with_class={name:sum(counts[name] > 0 for counts in frame_counts)
                                     for name in record['class_counts']},
                  frame_counts=frame_counts, latest_detection_record=latest_detection_record,
                  last_frame=record)
    if args.tray:
        report.update(tray_polygon=tray_count.POLYGON, min_tray_fraction=tray_count.MIN_TRAY_FRACTION,
                      duplicate_iou=tray_count.DUPLICATE_IOU, tray_frames=tray_frames,
                      single_scissors_frames=sum(f['total'] == f['scissors'] == 1 for f in tray_frames),
                      empty_frames=sum(f['total'] == 0 for f in tray_frames))
    if session:
        report.update(events=session.events, final_confirmed_scissors=session.stability.confirmed)
        if pick:
            report.update(requested_target='image_left', pick_status=pick.status,
                          task_completed=pick.status == 'completed', pick_events=pick_events)
        if guard:
            report.update(empty_reference=str(args.empty_reference), visibility_events=session.visibility_events)
        if args.expected_scissors is not None:
            report.update(expected_scissors=args.expected_scissors, check_requested_at=session.check_requested_at,
                          closing_checks=session.checks)
    (output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items()
                      if k not in {'frame_counts','latest_detection_record','last_frame','tray_frames','visibility_events'}}), flush=True)


if __name__ == '__main__':
    main()
