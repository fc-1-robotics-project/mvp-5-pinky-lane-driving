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
- Nonblocking crosswalk state and final proposal arbitration: measured stop before
  WAIT, repeated-event latch through PASS, explicit passage evidence, priority of
  emergency/sensor failure/obstacles, stable-clear hold before restart.
- Robot-local watchdog core: disabled by default, receiver-monotonic expiry,
  remaining source-age budget, strictly ordered sequences, invalid-input stop.
- Planar laser-return transform and swept circumscribed-footprint collision checks.
  No-hit is not proof of unobserved free space; adapter coverage validation remains.
- Metric boundary selection and normal-section center polylines reject wrong width,
  near-field disconnection and ambiguity; synthetic 120-degree bend passes.
  Single-boundary normal offsets require a bounded fallback age and speed cap.
- Instance mask thinning/geodesic tracing runs in the existing replay environment.
  45 actual frames produce 75 lane traces and 30 crosswalk instances in
  `.agents/output/replay/run-u6vtafwn/`. One overlay sample was inspected, not a full
  mask-quality evaluation; metric_valid remains false without measured calibration.
- `.agents/tools/harness.sh fast`: 44 tests passed; `vision`: 3 tests passed.
  RED and GREEN commits are
  separate for perception, replay adaptation, control, behavior, watchdog and scans.
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
- Connect tested pixel tracing and calibrated projection to metric boundary
  association, maintain fallback age/speed state across frames, and transform
  temporal hints with current-frame odometry. Test S-curves and tracing edge cases.
- Connect calibrated paths and target overlay to both videos; evaluate straight,
  S-turn, sharp turns, opposite-lane markings and missing boundaries.
- Crosswalk path overlap and stable event identity/positive passage association;
  connect these to the tested FOLLOW/APPROACH/WAIT/PASS timer/latch core.
- Validate actual LiDAR FOV/angular coverage, timestamped TF and footprint;
  integrate return collision checks and tested stable-clear behavior hold.
- Connect command arbiter and robot watchdog to dry-run ROS adapters and fresh
  inputs; validate graph/QoS, process failure, downstream timeout and simulation.
- Selected-package deployment/rollback instructions and explicitly authorized
  physical verification with fall protection and emergency stop.

Do not mark issues complete from unit tests or training-video replay alone.
