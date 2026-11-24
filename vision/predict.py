"""Run local instrument detection and save annotated media plus frame-level JSONL.

Only existing local image/video files or folders are accepted. This program does
not open cameras or send robot commands. Folder inputs are not searched recursively.
"""

import argparse
import json
import math
import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent
IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}
VIDEO_SUFFIXES = {".avi", ".m4v", ".mkv", ".mov", ".mp4", ".mpeg", ".mpg", ".ts", ".webm", ".wmv"}
MEDIA_SUFFIXES = IMAGE_SUFFIXES | VIDEO_SUFFIXES


def local_path(value, parser, label):
    """Reject URLs, UNC shares and camera identifiers before loading any model."""
    value = str(value)
    if "://" in value or value.startswith(("\\\\", "//")):
        parser.error(f"{label} must be a local filesystem path, not a URL or network share.")
    path = Path(value).expanduser().resolve()
    if str(path).startswith(("\\\\", "//")):
        parser.error(f"{label} resolves to a network share; use a local path.")
    return path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", required=True,
        help="Existing local image, video, or folder of images/videos (nonrecursive); no camera IDs or URLs.",
    )
    parser.add_argument(
        "--weights", default=str(ROOT / "weights/best.pt"),
        help="Existing local .pt weights; defaults to the 50-epoch model.",
    )
    parser.add_argument("--conf", type=float, default=0.25, help="Detection score threshold, greater than 0 and at most 1 (default: 0.25).")
    parser.add_argument("--device", default="cpu", help="Inference device, e.g. cpu or 0 for the first CUDA GPU (default: cpu).")
    parser.add_argument(
        "--output", default=str(ROOT / "predictions/run"),
        help="Result directory; an existing directory is preserved and a numbered sibling is created.",
    )
    args = parser.parse_args()
    if not math.isfinite(args.conf) or not 0 < args.conf <= 1:
        parser.error("--conf must be greater than 0 and at most 1.")
    if not args.device.strip():
        parser.error("--device cannot be empty.")
    args.source = local_path(args.source, parser, "--source")
    args.weights = local_path(args.weights, parser, "--weights")
    args.output = local_path(args.output, parser, "--output")
    if args.source.is_file():
        if args.source.suffix.lower() not in MEDIA_SUFFIXES:
            parser.error("--source must be a supported image/video file, not a manifest or camera identifier.")
        args.sources = [args.source]
    elif args.source.is_dir():
        args.sources = sorted(
            (p.resolve() for p in args.source.iterdir() if p.is_file() and p.suffix.lower() in MEDIA_SUFFIXES),
            key=lambda p: str(p).casefold(),
        )
        if not args.sources:
            parser.error("No supported images/videos found directly inside --source.")
        for path in args.sources:
            local_path(path, parser, "Source file")
    else:
        parser.error(f"--source does not exist: {args.source}")
    if not args.weights.is_file() or args.weights.suffix.lower() != ".pt":
        parser.error(f"--weights must be an existing local .pt file: {args.weights}")
    return args


def reserve_output(requested):
    """Atomically reserve a new run directory without replacing previous results."""
    requested.parent.mkdir(parents=True, exist_ok=True)
    index = 1
    while True:
        candidate = requested if index == 1 else requested.with_name(f"{requested.name}_{index}")
        try:
            candidate.mkdir(exist_ok=False)
            return candidate
        except FileExistsError:
            index += 1


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


def video_fps(source, cv2):
    cap = cv2.VideoCapture(str(source))
    try:
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open local video: {source}")
        fps = float(cap.get(cv2.CAP_PROP_FPS))
    finally:
        cap.release()
    return fps if math.isfinite(fps) and fps > 0 else 30.0


def open_video_writer(stem, fps, dimensions, cv2):
    for suffix, codec in ((".mp4", "mp4v"), (".avi", "MJPG")):
        target = stem.with_suffix(suffix)
        writer = cv2.VideoWriter(str(target), cv2.VideoWriter_fourcc(*codec), fps, dimensions)
        if writer.isOpened():
            return writer, target
        writer.release()
        # A failed writer may create an empty file, only inside this new run.
        if target.exists() and target.stat().st_size == 0:
            target.unlink()
    raise RuntimeError(f"No usable video encoder for output: {stem}")


def process_source(model, source, source_index, output, records, args, cv2):
    is_video = source.suffix.lower() in VIDEO_SUFFIXES
    # Numeric prefix avoids collisions between same-stem files with different extensions.
    stem = output / f"{source_index + 1:04d}_{source.stem}_annotated"
    fps = video_fps(source, cv2) if is_video else None
    writer = None
    target = None
    dimensions = None
    frames = 0
    results = model.predict(
        source=str(source), stream=True, conf=args.conf, device=args.device,
        imgsz=640, save=False, show=False, verbose=False, vid_stride=1,
    )
    try:
        for result in results:
            record = result_record(result, source, source_index, frames)
            canvas = annotate(result.orig_img, record, cv2)
            current_dimensions = (canvas.shape[1], canvas.shape[0])
            if is_video:
                if writer is None:
                    dimensions = current_dimensions
                    writer, target = open_video_writer(stem, fps, dimensions, cv2)
                elif current_dimensions != dimensions:
                    raise RuntimeError(f"Video dimensions changed during decoding: {source}")
                writer.write(canvas)
            else:
                target = stem.with_suffix(".jpg")
                success, encoded = cv2.imencode(".jpg", canvas)
                if not success:
                    raise RuntimeError(f"Cannot encode annotated image: {source}")
                # numpy.tofile supports Windows Unicode paths consistently.
                encoded.tofile(str(target))
            records.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            records.flush()
            frames += 1
    finally:
        if writer is not None:
            writer.release()
        results.close()
    if frames == 0:
        raise RuntimeError(f"No frames decoded from: {source}")
    return {
        "source": str(source), "annotated_media": str(target),
        "frames": frames, "video_fps": fps,
    }


def main():
    # Parse help and validate paths before importing any GPU/vision dependency.
    args = parse_args()
    os.environ["YOLO_OFFLINE"] = "true"
    os.environ["YOLO_AUTOINSTALL"] = "false"
    os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / ".ultralytics"))
    os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".matplotlib"))
    import cv2
    from ultralytics import YOLO

    output = reserve_output(args.output)
    summary = {
        "status": "running", "weights": str(args.weights), "source": str(args.source),
        "confidence_threshold": args.conf, "device": args.device,
        "frame_index_base": 0, "box_format": "[x1, y1, x2, y2] in original-image pixels",
        "jsonl": str(output / "detections.jsonl"), "outputs": [],
    }
    print(f"Output directory: {output}", flush=True)
    try:
        model = YOLO(str(args.weights), task="detect")
        with (output / "detections.jsonl").open("x", encoding="utf-8") as records:
            for index, source in enumerate(args.sources):
                item = process_source(model, source, index, output, records, args, cv2)
                summary["outputs"].append(item)
                print(f"[{index + 1}/{len(args.sources)}] {source.name}: {item['frames']} frame(s)", flush=True)
        summary["status"] = "complete"
        print(f"Finished. Detection records: {summary['jsonl']}", flush=True)
    except BaseException as exc:
        summary["status"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
        summary["error"] = str(exc) or type(exc).__name__
        raise
    finally:
        (output / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8",
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted; completed frame records remain in the output directory.", file=sys.stderr)
        sys.exit(130)
