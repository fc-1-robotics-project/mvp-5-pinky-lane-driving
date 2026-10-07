#!/usr/bin/env python3
"""Collect frames worth labeling from a recorded lane run.

    python3 lane_label_candidates.py ~/pinky_runs/<time>/robot2/run_1.jsonl \
        [--until 60] [--model best.pt]

A frame is a candidate when the nearest detector result (within 0.5 s) missed a
lane line, saw only one side, or had low confidence. Raw camera JPEGs saved by
lane_run_recorder.py are used when present; otherwise frames come from the
overlay video and are NOT suitable for training (banner, re-encoding).
With --model, current predictions are written as YOLO segmentation pre-labels
to correct by hand; frames the model misses get an empty label file.
"""

import argparse
import bisect
import json
from pathlib import Path
import re
import shutil

LOW_CONFIDENCE = .6
MATCH_S = .5
NAMES = ('crosswalk', 'left line', 'right line')


def weakness(row):
    """Return why one detector result is worth a label, or '' when it is fine."""
    lines = [(c, p) for c, p in zip(row['classes'], row['confidence']) if c in (1, 2)]
    sides = {c for c, _ in lines}
    if not sides:
        return 'none'
    if len(sides) == 1:
        return 'left_only' if 1 in sides else 'right_only'
    return 'low_confidence' if min(p for _, p in lines) < LOW_CONFIDENCE else ''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('log')
    parser.add_argument('--until', type=float, help='ignore seconds after this (lane end)')
    parser.add_argument('--model', help='YOLO segmentation weights for pre-labels')
    args = parser.parse_args()
    log = Path(args.log).expanduser()
    rows = [json.loads(line) for line in log.open()]
    active = [r['t'] for r in rows if r['kind'] == 'mission' and r.get('active')]
    if not active:
        raise SystemExit('no active mission in this log')
    start, end = active[0], active[-1]
    if args.until is not None:
        end = min(end, start + args.until)
    obs = [r for r in rows if r['kind'] == 'obs' and start <= r['t'] <= end]
    obs_t = [r['t'] for r in obs]
    if not obs:
        raise SystemExit('no detector results in this log')

    def reason_at(t):
        index = bisect.bisect_left(obs_t, t)
        near = [obs[i] for i in (index - 1, index) if 0 <= i < len(obs)
                and abs(obs_t[i] - t) <= MATCH_S]
        return next((why for why in map(weakness, near) if why), '')

    out = log.parent / 'label_candidates' / log.stem
    images, labels = out / 'images', out / 'labels'
    if out.exists():
        shutil.rmtree(out)
    images.mkdir(parents=True)
    raw = log.with_name(log.stem + '_raw')
    picked = []
    if raw.is_dir():
        for source in sorted(raw.glob('*.jpg')):
            t = start + float(re.search(r'_(\d+\.\d+)s\.jpg$', source.name)[1])
            why = reason_at(t) if start <= t <= end else ''
            if why:
                target = images / f'{source.stem}_{why}.jpg'
                shutil.copy2(source, target)
                picked.append((target, why))
    else:
        import cv2
        print('WARNING: no raw frames; using overlay video. Do not train on these images.')
        frames = [r['t'] for r in rows if r['kind'] == 'frame']
        capture = cv2.VideoCapture(str(log.with_suffix('.mp4')))
        for index, t in enumerate(frames):
            ok, frame = capture.read()
            if not ok:
                break
            why = reason_at(t) if start <= t <= end else ''
            if why and index % 3 == 0:
                target = images / f'{log.parent.name}_{log.stem}_{t - start:06.2f}s_{why}.png'
                cv2.imwrite(str(target), frame)
                picked.append((target, why))

    counts = {}
    for _, why in picked:
        counts[why] = counts.get(why, 0) + 1
    print(f'{len(picked)} candidate frames -> {images}')
    print('  ' + ', '.join(f'{k} {v}' for k, v in sorted(counts.items())))

    if args.model and picked:
        from ultralytics import YOLO
        model = YOLO(args.model)
        labels.mkdir()
        (out / 'classes.txt').write_text('\n'.join(NAMES) + '\n')
        empty = 0
        for target, _ in picked:
            result = model.predict(str(target), imgsz=448, conf=.25, device='cpu', verbose=False)[0]
            lines = []
            if result.masks is not None:
                for cls, polygon in zip(result.boxes.cls.tolist(), result.masks.xyn):
                    if len(polygon) >= 3:
                        lines.append(f'{int(cls)} ' + ' '.join(f'{v:.5f}' for v in polygon.flatten()))
            empty += not lines
            (labels / f'{target.stem}.txt').write_text('\n'.join(lines) + ('\n' if lines else ''))
        print(f'pre-labels -> {labels} ({empty} empty: the model found nothing there)')


if __name__ == '__main__':
    main()
