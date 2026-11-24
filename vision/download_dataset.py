import concurrent.futures as cf
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time
import zipfile

ROOT = Path(__file__).resolve().parent / 'datasets/labeled_surgical_tools'
URL = 'https://www.kaggle.com/api/v1/datasets/download/dilavado/labeled-surgical-tools?datasetVersionNumber=1'
TOTAL = 769368262


def main():
    ROOT.mkdir(parents=True, exist_ok=True)
    target = ROOT / 'source.zip'
    start = target.stat().st_size if target.exists() else 0
    if start > TOTAL:
        raise ValueError('Unexpected existing archive size')
    chunks = []
    for a in range(start, TOTAL, 8 * 1024 * 1024):
        b = min(a + 8 * 1024 * 1024, TOTAL) - 1
        chunks.append((a, b, ROOT / f'chunk_{a}_{b}.part'))

    def fetch(item):
        a, b, dest = item
        headers = dest.with_suffix('.headers')
        for attempt in range(4):
            result = subprocess.run(['curl.exe', '-sS', '-L', '--fail', '--connect-timeout', '30',
                '--max-time', '180', '-r', f'{a}-{b}', '-D', str(headers), URL, '-o', str(dest)],
                capture_output=True)
            h = headers.read_text(errors='replace').lower() if headers.exists() else ''
            if result.returncode == 0 and dest.stat().st_size == b-a+1 and f'content-range: bytes {a}-{b}/{TOTAL}' in h:
                return b-a+1
            time.sleep(2)
        raise RuntimeError(f'Failed byte range {a}-{b}')

    received = start
    last = 0
    with cf.ThreadPoolExecutor(max_workers=8) as pool:
        for job in cf.as_completed([pool.submit(fetch, item) for item in chunks]):
            received += job.result()
            if time.monotonic()-last > 15 or received == TOTAL:
                print(f'Downloaded {received/1048576:.1f}/{TOTAL/1048576:.1f} MiB', flush=True)
                last = time.monotonic()
    with target.open('ab') as out:
        for _, _, part in chunks:
            with part.open('rb') as src:
                shutil.copyfileobj(src, out)
    assert target.stat().st_size == TOTAL
    sha = hashlib.file_digest(target.open('rb'), 'sha256').hexdigest()
    with zipfile.ZipFile(target) as z:
        bad = z.testzip()
        if bad:
            raise ValueError(f'ZIP checksum failed: {bad}')
        root = (ROOT / 'raw').resolve()
        for entry in z.infolist():
            resolved = (root / entry.filename).resolve()
            if not resolved.is_relative_to(root):
                raise ValueError('Unsafe archive path')
        z.extractall(root)
        info = {'source': URL, 'bytes': TOTAL, 'sha256': sha, 'archive_files': len(z.infolist()), 'crc_check': 'passed'}
    (ROOT / 'download_verified.json').write_text(json.dumps(info, indent=2), encoding='utf-8')
    print(json.dumps(info), flush=True)


if __name__ == '__main__':
    main()
