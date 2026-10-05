#!/usr/bin/env python3
"""Summarize one recorded lane run: where the lane was lost and where it stopped.

    python3 lane_run_report.py ~/pinky_runs/<time>/robot1/run_1.jsonl

Reads the JSONL written by lane_run_recorder.py and, when the matching mp4
exists, saves one PNG per reported interval next to it.
"""

import bisect
import json
import math
from pathlib import Path
import sys
import time

OK_LANE = ('paired',)
MOVING = ('follow', 'tracking', 'crosswalk')


def intervals(samples, bad, end, min_s=.4):
    """Merge consecutive bad samples (t, label) into (start, stop, labels)."""
    out, current = [], None
    for (t, label), (following, _) in zip(samples, samples[1:] + [(end, None)]):
        if bad(label):
            if current is None:
                current = [t, following, {}]
            current[1] = following
            current[2][label] = current[2].get(label, 0.) + following - t
        elif current is not None:
            out.append(current)
            current = None
    if current is not None:
        out.append(current)
    return [item for item in out if item[1] - item[0] >= min_s]


def main(path):
    path = Path(path).expanduser()
    rows = [json.loads(line) for line in path.open()]
    if not rows:
        print('empty log')
        return
    mission = [r for r in rows if r['kind'] == 'mission']
    active = [r['t'] for r in mission if r.get('active')]
    start, end = (active[0], active[-1]) if active else (rows[0]['t'], rows[-1]['t'])
    odom = [r for r in rows if r['kind'] == 'odom']
    times, distance = [], []
    total = 0.
    for a, b in zip(odom, odom[1:]):
        total += math.dist((a['x'], a['y']), (b['x'], b['y']))
        times.append(b['t'])
        distance.append(total)

    def where(t):
        if not times:
            return 0., None
        index = min(bisect.bisect_left(times, t), len(times) - 1)
        row = odom[index + 1]
        return distance[index], (row['x'], row['y'])

    def show(title, found):
        print(f'\n{title}: {len(found)} interval(s), {sum(b - a for a, b, _ in found):.1f} s total')
        for a, b, labels in found:
            at, xy = where(a)
            detail = ', '.join(f'{k} {v:.1f}s' for k, v in sorted(labels.items(),
                                                                key=lambda item: -item[1]))
            place = f'odom ({xy[0]:+.2f}, {xy[1]:+.2f})' if xy else ''
            print(f'  +{a - start:6.1f}s  {b - a:5.1f}s  at {at:5.2f} m  {place}  {detail}')
        return found

    final = mission[-1] if mission else {}
    print(f'== {path.parent.name} {path.stem}: {time.strftime("%H:%M:%S", time.localtime(start))}'
          f' ~ {time.strftime("%H:%M:%S", time.localtime(end))} ({end - start:.0f} s),'
          f' odom {total:.2f} m, mission distance {final.get("distance_m")} m')
    print(f'   end: {final.get("state")} / {final.get("detail")}')

    inside = [r for r in rows if start <= r['t'] <= end]
    diag = [(r['t'], r) for r in inside if r['kind'] == 'diag']
    lane = [(t, 'no_valid_path:' + str(r.get('lane_reason')) if r.get('lane_valid') is not True
             else str(r.get('lane_reason'))) for t, r in diag]
    lost = show('No valid lane path', intervals(lane, lambda v: v.startswith('no_valid_path'), end))
    show('Degraded lane path (one boundary / remembered path)',
         intervals(lane, lambda v: not v.startswith('no_valid_path') and v not in OK_LANE, end, 1.))
    control = [(t, str(r.get('reason'))) for t, r in diag]
    stops = show('Stopped by control', intervals(control, lambda v: v not in MOVING, end))
    obs = [(r['t'], ('none' if not {1, 2} & set(r['classes']) else
                     'left_only' if 2 not in r['classes'] else
                     'right_only' if 1 not in r['classes'] else 'both'))
           for r in inside if r['kind'] == 'obs']
    show('Detector saw no lane line at all', intervals(obs, lambda v: v == 'none', end))
    counts = {}
    for _, label in obs:
        counts[label] = counts.get(label, 0) + 1
    print('\nDetections per frame:', ', '.join(f'{k} {v}' for k, v in sorted(counts.items())),
          f'of {len(obs)}')
    ages = sorted(r['capture_age_s'] for _, r in diag if isinstance(r.get('capture_age_s'), float))
    proc = sorted(r['processing_time_s'] for r in inside
                  if r['kind'] == 'obs' and r.get('processing_time_s'))
    if ages and proc:
        print(f'Capture age median {ages[len(ages) // 2]:.2f} s, max {ages[-1]:.2f} s'
              f' (limit 1.1); inference median {proc[len(proc) // 2]:.2f} s, max {proc[-1]:.2f} s')
    speeds = [r['v'] for r in odom if start <= r['t'] <= end]
    if speeds:
        print(f'Speed mean {sum(speeds) / len(speeds):.3f} m/s, max {max(speeds):.3f} m/s')

    video = path.with_suffix('.mp4')
    frames = [r['t'] for r in rows if r['kind'] == 'frame']
    if video.exists() and frames:
        import cv2
        capture = cv2.VideoCapture(str(video))
        for kind, found in (('lost', lost), ('stop', stops)):
            for a, _, _ in found[:8]:
                index = min(bisect.bisect_left(frames, a), len(frames) - 1)
                capture.set(cv2.CAP_PROP_POS_FRAMES, index)
                ok, frame = capture.read()
                if ok:
                    name = path.with_name(f'{path.stem}_{kind}_{a - start:05.1f}s.png')
                    cv2.imwrite(str(name), frame)
        print(f'\nvideo: {video} ({len(frames)} frames)')


if __name__ == '__main__':
    main(sys.argv[1])
