"""Verify a request for the left scissors using two fixed, separated tray positions."""
import json
from math import dist

import cv2

from tray_events import StableCount


def center(box):
    x1, y1, x2, y2 = box
    return ((x1 + x2) / 2, (y1 + y2) / 2)


class LeftPick:
    def __init__(self):
        self.status = 'waiting_pair'
        self.positions = None
        self.target_box = None
        self.initial_pair = None
        self.evidence = StableCount()

    def observe(self, boxes, valid, elapsed):
        if self.status == 'completed':
            return None
        points = sorted((center(box) for box in boxes)) if valid else []
        candidate = None
        if self.positions is None:
            if len(points) == 2 and points[1][0] - points[0][0] >= 35:
                if self.initial_pair is None or any(dist(a, b) > 8 for a, b in zip(points, self.initial_pair)):
                    self.initial_pair = points
                    self.evidence = StableCount()
                candidate = 'pair'
            else:
                self.initial_pair = None
        else:
            radius = min(28, 0.35 * dist(*self.positions))
            if len(points) == 1:
                for side, position in zip(('left_remains', 'right_remains'), self.positions):
                    if dist(points[0], position) <= radius:
                        candidate = side
            elif len(points) == 2 and all(dist(a, b) <= radius for a, b in zip(points, self.positions)):
                candidate = 'pair'
        self.evidence.observe(candidate, elapsed)
        if not self.evidence.ready():
            return None
        if self.status == 'waiting_pair' and candidate == 'pair':
            self.positions = points
            self.target_box = min(boxes, key=lambda box: center(box)[0])
            self.status = 'selected'
        elif self.status == 'selected' and candidate == 'right_remains':
            self.status = 'completed'
        elif self.status == 'selected' and candidate == 'left_remains':
            self.status = 'wrong_pick'
        elif self.status == 'wrong_pick' and candidate == 'pair':
            self.status = 'selected'
        else:
            return None
        return dict(status=self.status, completed=self.status == 'completed', target='image_left',
                    observation=candidate, centers=points, reference_centers=self.positions)

    def draw(self, image):
        if self.target_box is None:
            return
        x1, y1, x2, y2 = map(round, self.target_box)
        color = (0, 200, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, 'TARGET L', (x1, max(85, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        labels = {'selected': 'Target: LEFT | pending',
                  'wrong_pick': 'WRONG: right removed | left target pending',
                  'completed': 'CORRECT: left target removed | complete'}
        cv2.rectangle(image, (0, 33), (image.shape[1]-1, 62), (24, 24, 24), -1)
        cv2.putText(image, labels[self.status], (8, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2)


def save_pick_event(output, sample, event, image):
    filename = f"pick_{sample['frame']:05d}_{event['status']}.jpg"
    if not cv2.imwrite(str(output / filename), image):
        raise RuntimeError('Could not save target selection evidence')
    item = dict(**event, frame=sample['frame'], time=sample['time'], image=filename)
    with (output/'picks.jsonl').open('a', encoding='utf-8') as log:
        log.write(json.dumps(item) + '\n')
    print('PICK ' + json.dumps(item), flush=True)
    return item
