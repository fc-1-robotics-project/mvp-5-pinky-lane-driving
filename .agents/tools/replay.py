#!/usr/bin/env python3
# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Replay trusted local segmentation weights on video windows; no ROS commands."""

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import tempfile
import time
import sys
from dataclasses import asdict


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'pinky_lane_driving'))
from pinky_lane_driving.vision import CLASSES, observe_result, trace_observation, validate_model
from pinky_lane_driving.calibration import Calibration
from pinky_lane_driving.path import LanePath, PathSettings
from pinky_lane_driving.tracking import LaneTracker


def frame_plan(fps, total, start, end, sample_fps):
    """Select source frames in [start, end), retaining source playback timing."""
    if (not all(math.isfinite(v) for v in (fps, total, start, end, sample_fps))
            or fps <= 0 or total <= 0 or sample_fps <= 0
            or not 0 <= start < end <= total / fps):
        raise ValueError('Invalid FPS, frame count, or video interval')
    stride = max(1, round(fps / sample_fps))
    frames = range(math.ceil(start * fps), math.ceil(end * fps), stride)
    if not frames:
        raise ValueError('Interval contains no frames')
    return frames, fps / stride


def sha256(path):
    """Identify the exact local artifact used for a run."""
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def replay_case(cv2, model, case, output, conf, imgsz, device, sample_fps, tracker=None):
    """Produce one annotated clip and machine-readable frame observations."""
    source = Path(case['video'])
    cap = cv2.VideoCapture(str(source))
    writer = None
    latencies = []
    counts = Counter()
    empty = 0
    clip = output / f"{case['name']}.mp4"
    try:
        if not cap.isOpened():
            raise RuntimeError(f'Cannot open video: {source}')
        fps = cap.get(cv2.CAP_PROP_FPS)
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        frames, output_fps = frame_plan(fps, total, case['start_s'], case['end_s'], sample_fps)
        size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        writer = cv2.VideoWriter(str(clip), cv2.VideoWriter_fourcc(*'mp4v'), output_fps, size)
        if not writer.isOpened():
            raise RuntimeError('MP4 writer could not open')
        if not cap.set(cv2.CAP_PROP_POS_FRAMES, frames.start):
            raise RuntimeError('Video seek failed')
        with (output / f"{case['name']}.jsonl").open('w') as log:
            for index in range(frames.start, frames[-1] + 1):
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError(f'Unexpected decode failure at frame {index}')
                if (index - frames.start) % frames.step:
                    continue
                started = time.perf_counter()
                result = model.predict(frame, imgsz=imgsz, conf=conf, device=device,
                                       retina_masks=True, verbose=False)[0]
                elapsed = time.perf_counter() - started
                latencies.append(elapsed)
                ids = result.boxes.cls.int().cpu().tolist()
                scores = result.boxes.conf.cpu().tolist()
                if ids and (result.masks is None or len(result.masks.data) != len(ids)):
                    raise RuntimeError('Detections without matching segmentation masks')
                counts.update(CLASSES[i] for i in ids)
                empty += not ids
                row = {'source_frame': index, 'source_time_s': index / fps,
                       'inference_s': elapsed, 'class_ids': ids, 'confidence': scores}
                row['observation'] = trace_observation(observe_result(result, index / fps))
                canvas = result.plot(boxes=True, labels=True, conf=True)
                for detection in row['observation']['detections']:
                    if detection['class_id'] == 0:
                        continue
                    trace = detection.get('boundary_px', ())
                    for a, b in zip(trace, trace[1:]):
                        cv2.line(canvas, tuple(map(round, a)), tuple(map(round, b)),
                                 (0, 255, 255), 2)
                lane = (LanePath(reason='uncalibrated') if tracker is None else
                        tracker.update(row['observation'], now=index / fps))
                row['lane_path'] = dict(asdict(lane), frame_id='base_footprint', units='m')
                if lane.valid:
                    overlay = tracker.calibration.image_points(lane.points)
                    for a, b in zip(overlay, overlay[1:]):
                        cv2.line(canvas, tuple(map(round, a)), tuple(map(round, b)),
                                 (0, 255, 0), 2)
                log.write(json.dumps(row, allow_nan=False) + '\n')
                cv2.putText(canvas, f'yellow: boundary | green: metric path | {lane.reason}',
                            (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 255), 1)
                cv2.putText(canvas, f"{case['name']} | {index / fps:.2f}s | perception only",
                            (8, size[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, .45,
                            (255, 255, 255), 1)
                writer.write(canvas)
    finally:
        cap.release()
        if writer is not None:
            writer.release()
    # Validate the encoded result, not only the inference loop.
    check = cv2.VideoCapture(str(clip))
    decoded = 0
    try:
        while check.read()[0]:
            decoded += 1
    finally:
        check.release()
    if decoded != len(frames):
        raise RuntimeError(f'Output frame mismatch: {decoded} != {len(frames)}')
    return {**case, 'video_sha256': sha256(source), 'source_fps': fps,
            'sampled_frames': len(frames), 'output_fps': output_fps,
            'empty_detection_frames': empty, 'detection_instances': dict(counts),
            'mean_inference_s': sum(latencies) / len(latencies),
            'max_inference_s': max(latencies), 'output_clip': clip.name,
            'visual_review': 'pending', 'status': 'execution_passed'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / '.agents' / 'replay.local.json')
    parser.add_argument('--case', action='append', help='Run named case(s) only')
    args = parser.parse_args()
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text())
    conf = config.get('conf', .25)
    sample_fps = config.get('sample_fps', 5.)
    imgsz = config.get('imgsz', 640)
    if not math.isfinite(conf) or not 0 <= conf <= 1:
        raise ValueError('conf must be finite and within 0..1')
    if not math.isfinite(sample_fps) or sample_fps <= 0:
        raise ValueError('sample_fps must be finite and positive')
    if not isinstance(imgsz, int) or isinstance(imgsz, bool) or imgsz <= 0 or imgsz % 32:
        raise ValueError('imgsz must be a positive multiple of 32')

    def local_file(value):
        path = Path(value).expanduser()
        path = (config_path.parent / path).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        return path

    weights = local_file(config['model'])
    calibration = None
    calibration_path = None
    if config.get('calibration_file'):
        calibration_path = local_file(config['calibration_file'])
        calibration = Calibration(json.loads(calibration_path.read_text()))
        path_settings = PathSettings(**config['path_settings'])
    cases = config['cases']
    names = [c['name'] for c in cases]
    if (not cases or len(set(names)) != len(names)
            or any(not n or any(ch not in 'abcdefghijklmnopqrstuvwxyz0123456789_-' for ch in n)
                   for n in names)):
        raise ValueError('Case names must be unique lowercase ASCII names')
    if args.case and not set(args.case) <= set(names):
        raise ValueError(f'Unknown case; choose from {names}')
    cases = [dict(c, video=str(local_file(c['video']))) for c in cases
             if not args.case or c['name'] in args.case]
    import cv2
    import torch
    import ultralytics
    from ultralytics import YOLO

    torch.set_num_threads(1)
    cv2.setNumThreads(1)
    model = YOLO(str(weights))
    validate_model(model.task, model.names)
    base = ROOT / '.agents' / 'output' / 'replay'
    base.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='run-', dir=base))
    report = {'status': 'running', 'model': str(weights), 'model_sha256': sha256(weights),
              'classes': CLASSES, 'conf': conf, 'imgsz': imgsz, 'sample_fps': sample_fps,
              'device': config.get('device', 'cpu'),
              'calibration_sha256': sha256(calibration_path) if calibration_path else None,
              'path_settings': config.get('path_settings') if calibration else None,
              'mounting_id': config.get('mounting_id') if calibration else None,
              'versions': {'opencv': cv2.__version__, 'torch': torch.__version__,
                           'ultralytics': ultralytics.__version__}, 'cases': [],
              'scope': 'Perception replay only; no ground-truth accuracy or driving validation'}
    print(f'Output: {output}', flush=True)
    try:
        for case in cases:
            tracker = (None if calibration is None else
                       LaneTracker(calibration, path_settings,
                                   mounting_id=config['mounting_id'],
                                   timeout=config['capture_timeout_s']))
            result = replay_case(cv2, model, case, output, conf, imgsz,
                                 report['device'], sample_fps, tracker)
            report['cases'].append(result)
            print(f"{case['name']}: {result['sampled_frames']} frames saved", flush=True)
        report['status'] = 'execution_passed'
    except Exception as error:
        report.update(status='failed', error=str(error))
        raise
    finally:
        (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
