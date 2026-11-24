"""Train a local YOLO instrument dataset; no robot or camera control."""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, required=True, help='Local YOLO dataset YAML')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--batch', type=int, default=8)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--name', default='instruments')
    parser.add_argument('--no-amp', action='store_true')
    args = parser.parse_args()
    if not args.data.is_file():
        parser.error('Dataset YAML does not exist; prepare and inspect labels first.')
    import torch
    from ultralytics import YOLO
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA unavailable. Verify the GPU environment before training.')
    weights = Path(__file__).resolve().parent / 'weights/yolov8n.pt'
    if not weights.is_file():
        raise FileNotFoundError(f'Pretrained weights missing: {weights}')
    model = YOLO(str(weights))
    model.train(
        data=str(args.data.resolve()), epochs=args.epochs, batch=args.batch,
        imgsz=args.imgsz, device=0, workers=0, patience=20, seed=42,
        project=str(Path(__file__).resolve().parent / 'runs'), name=args.name,
        exist_ok=False, cache=False, amp=not args.no_amp,
    )


if __name__ == '__main__':
    main()
