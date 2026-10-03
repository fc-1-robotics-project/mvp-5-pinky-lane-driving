# Calibration gate

No measured calibration is available in this repository. Checkpoint weights are
not camera calibration. Do not copy synthetic test values onto the robot.

For metric replay, supply a JSON calibration file with these required fields:

- `image_size`: exact source `[width, height]` in pixels.
- `camera_frame`: source optical frame; `ground_frame`: `base_footprint`.
- `mounting_id`: versioned identity of the measured physical camera mounting.
- `source`: provenance/location of measurement records, not an accuracy claim.
- `lane_width_m`: physically measured current-lane width.
- `camera_matrix`: measured 3x3 pinhole K; `distortion`: measured OpenCV pinhole
  coefficients (4/5/8/12/14 values). Fisheye calibration is not supported.
- `homography`: 3x3 mapping from **undistorted pixels using P=K** to robot ground
  x forward/y left in metres. Raw distorted pixel correspondences are not valid.
- `rectified_roi`: `[xmin, ymin, xmax, ymax]` within the independently validated
  ground region. Do not extend it above the horizon or to unmeasured floor areas.

After measuring intrinsics/distortion and floor correspondences, validate held-out
ground points, lane-width estimates and camera mounting stability. Changing image
resolution, cropping, image rotation, lens or mounting requires a matching reviewed
calibration. Software identity checks cannot detect a physically moved camera.

In ignored `.agents/replay.local.json`, add `calibration_file`, `mounting_id`,
`capture_timeout_s`, and `path_settings`. The latter uses the exact names from
`PathSettings`: `width`, `width_tolerance`, `max_near`, `sample_step`,
`fallback_timeout`, `fallback_speed`, `ambiguity_margin`, optionally `min_coverage`.
Distances are metres, times seconds, fallback speed m/s. `width` must equal the
measured calibration width. No hardware values are supplied by default.

Without `calibration_file`, replay remains pixel-only and records an invalid metric
path (`reason=uncalibrated`). With valid settings it logs metric polylines in
`lane_path` and overlays them in green. Yellow remains a pixel boundary trace.
Playback uses source video time for geometry; this does not benchmark live safety
latency. Original videos have no synchronized odometry, so temporal hints are not
reused; the live adapter must provide pose at capture and transform again at control.

Checks: `fast` tests synthetic metric geometry without external libraries; `vision`
also checks distorted-image projection roundtrip using the replay environment.
Neither constitutes real calibration or physical driving acceptance.
