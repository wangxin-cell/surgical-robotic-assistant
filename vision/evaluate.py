"""Evaluate the selected checkpoint once on the untouched source test split."""
import argparse
import json
import os
from pathlib import Path


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=Path, default=root/'runs/instruments_50epochs/weights/best.pt')
    args = parser.parse_args()
    if not args.weights.is_file():
        parser.error('Checkpoint does not exist')
    for key, folder in [('YOLO_CONFIG_DIR','.ultralytics'),('MPLCONFIGDIR','.matplotlib')]:
        path=root/folder
        path.mkdir(exist_ok=True)
        os.environ[key]=str(path)
    from ultralytics import YOLO
    model = YOLO(str(args.weights.resolve()))
    result = model.val(data=str(root/'datasets/labeled_surgical_tools/yolo/data.yaml'),
        split='test', imgsz=640, batch=8, device=0, workers=0, plots=True,
        conf=0.001, iou=0.7, save_txt=True, save_conf=True,
        project=str(root/'runs'), name='test_evaluation', exist_ok=False)
    classes=[]
    for i,c in enumerate(result.box.ap_class_index):
        p,r,ap50,ap=result.class_result(i)
        classes.append({'class_id':int(c),'name':result.names[int(c)],
            'precision':float(p),'recall':float(r),'mAP50':float(ap50),'mAP50_95':float(ap)})
    summary={'weights':str(args.weights.resolve()),'split':'test',
        'metrics':{k:float(v) for k,v in result.results_dict.items()},
        'per_class':classes,'speed_ms_per_image':result.speed,'output_dir':str(result.save_dir),
        'precision_recall_definition':'Ultralytics reports P/R at the best smoothed mean-F1 operating point on this evaluation split (IoU=0.5); these are not fixed-confidence deployment metrics.',
        'confusion_matrix':{
            'values':result.confusion_matrix.matrix.astype(int).tolist(),
            'labels':[result.names[i] for i in range(len(result.names))]+['background'],
            'rows':'predicted','columns':'true',
            'confidence_threshold':0.25,'matching_iou_threshold':0.45,
            'nms_iou_threshold':0.7,
            'note':'Ultralytics 8.4.0 ConfusionMatrix maps validation conf=0.001 to 0.25 internally. Distinct from AP/P/R operating points.'},
        'limitations':['Source-dataset test only; not an independent camera/clinical test.',
            'Exact pixel duplicates removed across prepared splits; spot review found two train-test pairs with highly similar instrument arrangements. Performance may be optimistic; capture-session identities are unavailable.',
            'Four source instrument classes, not blade subtype identification.',
            'Each source image contains at most one annotated object per class; multiple-instance counting is unvalidated.',
            'Timing from batched laptop validation is not live camera or Orion O6 throughput.']}
    (root/'test_metrics.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary),flush=True)


if __name__=='__main__':
    main()
