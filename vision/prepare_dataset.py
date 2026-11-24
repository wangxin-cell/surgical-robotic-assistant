"""Audit the source archive's dataset and prepare a reproducible YOLO layout."""
from pathlib import Path
import collections
import hashlib
import json
import random
import shutil
from PIL import Image

ROOT = Path(__file__).resolve().parent / 'datasets/labeled_surgical_tools'
RAW = ROOT / 'raw/Surgical-Dataset'
NAMES = ['scalpel_no4', 'straight_dissection_clamp', 'straight_mayo_scissor', 'curved_mayo_scissor']


def main():
    images = {p.name: p for p in (RAW/'Images/All/images').glob('*.jpg')}
    hashes, sizes, counts, errors = {}, collections.Counter(), collections.Counter(), []
    for name, path in images.items():
        with Image.open(path) as img:
            img.load()
            sizes[str(img.size)] += 1
            hashes[name] = hashlib.sha256(img.convert('RGB').tobytes()).hexdigest()
        label = RAW/'Labels/label object names'/f'{path.stem}.txt'
        if not label.is_file():
            errors.append([name, 'missing label'])
            continue
        for line in label.read_text().splitlines():
            vals = line.split()
            if len(vals) != 5:
                errors.append([name, 'bad field count', line])
                continue
            c, x, y, w, h = map(float, vals)
            if c != int(c) or int(c) not in range(4) or not (0 <= x <= 1 and 0 <= y <= 1 and 0 < w <= 1 and 0 < h <= 1):
                errors.append([name, 'invalid values', line])
            if min(x-w/2, y-h/2) < -1e-5 or max(x+w/2, y+h/2) > 1.00001:
                errors.append([name, 'box outside image', line])
            counts[NAMES[int(c)]] += 1
    groups = {}
    for split in ['train', 'test']:
        lines = (RAW/f'Test-Train Groups/{split}-obj_detector.txt').read_text().splitlines()
        groups[split] = [line.replace('\\', '/').split('/')[-1] for line in lines if line.strip()]
    missing = [n for ns in groups.values() for n in ns if n not in images]
    if missing:
        raise ValueError(f'Split references missing images: {missing[:10]}')
    overlap = set(hashes[n] for n in groups['train']) & set(hashes[n] for n in groups['test'])
    report = {'images':len(images), 'resolution_counts':dict(sizes), 'objects_per_class':dict(counts),
        'invalid_labels':errors, 'original_split_sizes':{k:len(v) for k,v in groups.items()},
        'unique_decoded_images':len(set(hashes.values())), 'cross_split_duplicate_hashes':len(overlap),
        'class_names':NAMES,
        'note':'No blade subtype labels. Scalpel no4 preserves source class wording; not a verified blade number. Near-duplicates and capture-session leakage have not been exhaustively audited.'}
    (ROOT/'audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    if errors:
        print(json.dumps(report),flush=True)
        return
    # Preserve original test assignment; exclude train copies of test pixels.
    test_hashes = {hashes[n] for n in groups['test']}
    seen = set()
    test = []
    for n in groups['test']:
        if hashes[n] not in seen:
            test.append(n)
            seen.add(hashes[n])
    train_candidates = []
    for n in groups['train']:
        if hashes[n] not in seen:
            train_candidates.append(n)
            seen.add(hashes[n])
    # Derive validation from original train only, stratified by class combination.
    strata = collections.defaultdict(list)
    for n in train_candidates:
        lines = (RAW/'Labels/label object names'/f'{Path(n).stem}.txt').read_text().splitlines()
        key = tuple(sorted(int(line.split()[0]) for line in lines if line.strip()))
        strata[key].append(n)
    train, val = [], []
    rng = random.Random(42)
    for key in sorted(strata):
        ns = sorted(strata[key])
        rng.shuffle(ns)
        k = max(1, round(len(ns)*0.15)) if len(ns)>1 else 0
        val.extend(ns[:k]); train.extend(ns[k:])
    out = ROOT/'yolo'
    split_lists = {'train':train,'val':val,'test':test}
    for split, ns in split_lists.items():
        idir, ldir = out/'images'/split, out/'labels'/split
        idir.mkdir(parents=True,exist_ok=True); ldir.mkdir(parents=True,exist_ok=True)
        for n in ns:
            shutil.copyfile(images[n], idir/n)
            shutil.copyfile(RAW/'Labels/label object names'/f'{Path(n).stem}.txt', ldir/f'{Path(n).stem}.txt')
    yaml = f'path: {out.as_posix()}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n'
    yaml += ''.join(f'  {i}: {n}\n' for i,n in enumerate(NAMES))
    (out/'data.yaml').write_text(yaml,encoding='utf-8')
    (out/'split_manifest.json').write_text(json.dumps(split_lists,indent=2),encoding='utf-8')
    report['prepared_split_sizes'] = {k:len(v) for k,v in split_lists.items()}
    report['data_yaml'] = str(out/'data.yaml')
    (ROOT/'audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report),flush=True)


if __name__ == '__main__':
    main()
