# Implementation evidence

Design authority: GitHub issues #1 and #2. This is partial implementation, not
acceptance of autonomous driving. Keep ROS-independent algorithms separate from
perception adapters, behavior/control, and robot-side watchdog.

## Completed checks (2026-09-16)

- Perception input preserves capture time, optical frame, per-instance polygons,
  confidences and classes (0 crosswalk, 1 left line, 2 right line).
- Projection primitive rejects missing calibration, unrectified pixels, singular
  matrices, invalid pixels and horizon/behind-robot projections. Synthetic tests
  use base_footprint x forward/y left, metres; no real calibration is inferred.
- Replay imports that contract and records it for each sampled source frame.
- Pure Pursuit proposals: polyline arc lookahead, signed curvature, speed/angular/
  lateral-acceleration limits, acceleration ramp, visible-path braking bound,
  and zero output on invalid/unmeasured/stale input. No ROS command publication.
- `.agents/tools/harness.sh fast`: 20 tests passed. RED and GREEN commits are
  separate for perception, replay adaptation, and control.
- Actual best (2).pt replay: 3 windows across both training recordings, 45 sampled
  frames and decoded output frames. `.agents/output/replay/run-nuaoy7fe/report.json`
  records model/video hashes and versions; per-frame JSONL now contains polygons.
  This proves execution, not mask correctness or generalization.

## Actual local inputs

Ignored `.agents/replay.local.json` points to:

- `/home/seunghoon/Downloads/best (2).pt`
- `/home/seunghoon/dev_ws/ros/pinky_recordings/drive_upright.mp4`
- `/home/seunghoon/dev_ws/ros/pinky_recordings/drive_20260912_184852.mp4`

The extracted `best (2)/best/` metadata matches the original checkpoint. Reuse the
checkpoint rather than loading extracted pickle files. Media remains local; the
user permits inclusion but duplication is unnecessary for these tests.

## Remaining acceptance work

- Measured camera intrinsics/distortion, floor correspondences and lane width;
  validate calibration resolution/pose/domain before metric video processing.
- Current-lane association (not largest mask), boundary tracing, corresponding
  cross-sections, normal-offset single-boundary fallback with time/speed bounds;
  current-frame odometry transforms for temporal reuse.
- Connect calibrated paths and target overlay to both videos; evaluate straight,
  S-turn, sharp turns, opposite-lane markings and missing boundaries.
- Crosswalk path overlap, FOLLOW/APPROACH/WAIT/PASS latch and nonblocking timer.
- LiDAR TF/footprint/path collision and stable-clear hold; stale scan is not clear.
- Single command arbiter, fresh-input policy, independent robot watchdog and
  dry-run ROS adapters; validate graph/QoS, process failure and simulation.
- Selected-package deployment/rollback instructions and explicitly authorized
  physical verification with fall protection and emergency stop.

Do not mark issues complete from unit tests or training-video replay alone.
