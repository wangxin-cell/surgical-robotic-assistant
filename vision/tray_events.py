"""Record confirmed changes in the number of scissors on a fixed tray."""
import json
import time
from datetime import datetime, timedelta, timezone

import cv2


class StableCount:
    def __init__(self):
        self.confirmed = None
        self.candidate = None
        self.samples = 0
        self.since = 0.0
        self.last = None

    def ready(self):
        return (self.candidate is not None and self.samples >= 4
                and self.last - self.since >= 1.0)

    def observe(self, value, elapsed):
        if value is None or value != self.candidate or self.last is None or elapsed - self.last > 1.5:
            self.candidate = value
            self.samples = 0
            self.since = elapsed
        self.last = elapsed
        self.samples += 1
        if value == self.confirmed or not self.ready():
            return False
        self.confirmed = value
        return True


class TraySession:
    def __init__(self, folder, expected_scissors=None):
        self.folder = folder
        self.clock = time.monotonic()
        self.stability = StableCount()
        self.events = []
        self.expected = expected_scissors
        self.check_requested_at = None
        self.checks = []
        self.visibility_events = []
        self.blocked_since = None
        self.held_snapshot = False
        self.timeline = (folder / 'timeline.jsonl').open('x', encoding='utf-8', buffering=1)
        print(f'RECORDING {folder}', flush=True)

    def observe(self, frame_index, counted, image, visibility=None):
        elapsed = time.monotonic() - self.clock
        timestamp = datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='milliseconds')
        total, scissors = counted['total'], counted['scissors']
        # Other instrument labels make this single-prop scene ambiguous.
        blocked = visibility is not None and visibility['blocked']
        value = scissors if total == scissors and not blocked else None
        previous = self.stability.confirmed
        changed = self.stability.observe(value, elapsed)
        sample = dict(frame=frame_index, elapsed_s=round(elapsed, 3), time=timestamp,
                      total=total, scissors=scissors, confirmed=self.stability.confirmed)
        if visibility is not None:
            sample.update(**visibility, ready=self.stability.ready())
            self.record_visibility(sample, image, elapsed)
        self.timeline.write(json.dumps(sample) + '\n')
        if changed:
            filename = f'event_{len(self.events):02d}.jpg'
            if not cv2.imwrite(str(self.folder / filename), image):
                raise RuntimeError('Failed to save event image')
            event = dict(**sample, previous=previous, image=filename)
            self.events.append(event)
            with (self.folder / 'events.jsonl').open('a', encoding='utf-8') as log:
                log.write(json.dumps(event) + '\n')
            print('EVENT ' + json.dumps(event), flush=True)
        if self.expected is not None:
            self.update_check(sample, image)
        return sample

    def record_visibility(self, sample, image, elapsed):
        blocked = sample['blocked']
        confirmed = sample['confirmed']
        label = 'VIEW BLOCKED - WAIT' if blocked else ('VISIBLE' if sample['ready'] else 'CONFIRMING')
        cv2.rectangle(image, (0, 0), (image.shape[1]-1, 32), (24, 24, 24), -1)
        cv2.putText(image, f"{label}   Last confirmed: {confirmed if confirmed is not None else '?'}",
                    (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (70, 200, 255) if blocked else (130, 240, 140), 2)
        transition = not self.visibility_events or self.visibility_events[-1]['blocked'] != blocked
        if transition:
            self.blocked_since = elapsed if blocked else None
            self.held_snapshot = False
        held = blocked and not self.held_snapshot and elapsed - self.blocked_since >= 1.0
        if not transition and not held:
            return
        filename = f'view_{len(self.visibility_events):03d}.jpg'
        if not cv2.imwrite(str(self.folder / filename), image):
            raise RuntimeError('Could not save visibility evidence')
        item = dict(**sample, kind='held' if held else 'transition', image=filename)
        self.visibility_events.append(item)
        if held:
            self.held_snapshot = True
        print('VIEW ' + json.dumps(item), flush=True)

    def update_check(self, sample, image):
        if self.check_requested_at is None:
            if not (self.folder / 'request_check').is_file():
                return
            self.check_requested_at = sample['time']
            print('CHECK_REQUESTED ' + self.check_requested_at, flush=True)
        # Keep an existing alert through occlusions. Clear only on a fresh stable reading.
        if not self.stability.ready():
            return
        actual = self.stability.confirmed
        if self.checks and self.checks[-1]['actual'] == actual:
            return
        missing = max(0, self.expected - actual)
        extra = max(0, actual - self.expected)
        status = 'missing' if missing else ('extra' if extra else 'matched')
        text = f'应有 {self.expected} 把，实有 {actual} 把，待归还 {missing} 把'
        if extra:
            text += f'，多出 {extra} 把'
        filename = f'check_{len(self.checks):02d}.jpg'
        snapshot = image.copy()
        cv2.rectangle(snapshot, (0, 32), (snapshot.shape[1]-1, 63), (30, 30, 30), -1)
        caption = f'{status.upper()}    Expected {self.expected}    On tray {actual}    Missing {missing}'
        cv2.putText(snapshot, caption, (8, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                    (80, 230, 90) if status == 'matched' else (45, 180, 255), 2)
        if not cv2.imwrite(str(self.folder / filename), snapshot):
            raise RuntimeError('Failed to save closing check image')
        result = dict(frame=sample['frame'], time=sample['time'], requested_at=self.check_requested_at,
                      status=status, expected=self.expected, actual=actual, missing=missing, extra=extra,
                      message=text, image=filename)
        with (self.folder / 'closing_checks.jsonl').open('a', encoding='utf-8') as log:
            log.write(json.dumps(result, ensure_ascii=False) + '\n')
        self.checks.append(result)
        print('CHECK ' + json.dumps(result), flush=True)

    def close(self):
        self.timeline.close()
