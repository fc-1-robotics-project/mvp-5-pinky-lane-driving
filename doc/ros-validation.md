# ROS validation boundary

Implemented ROS endpoints: `lane_watchdog` and `lane_perception` in package
`pinky_lane_driving`. The control adapter is still pending; this is not an autonomous
driving launch. The pure algorithms live under the standard ament Python package
path `pinky_lane_driving/pinky_lane_driving/`.

## Reproducible checks

```bash
./.agents/tools/harness.sh fast
./.agents/tools/harness.sh ros pinky_lane_driving
./.agents/tools/harness.sh ros-smoke
REPLAY_PYTHON=/home/seunghoon/dev_ws/ros/.venv_yolo/bin/python ./.agents/tools/harness.sh ros-model
```

`ros-smoke` uses domain 177, localhost discovery and a process-specific namespace.
It never starts bringup or publishes the motor `cmd_vel` topic. The graph test
proves valid command delivery, timeout zero output after publisher silence,
rejection of stale/malformed messages and read-only runtime safety parameters.

## Perception transport

`lane_perception` subscribes to configurable `image_topic` (default relative
`camera/image_raw`), BEST_EFFORT depth 1. It runs one inference worker and replaces
the single pending image with the newest capture. A separate steady-clock timer
publishes reliable depth-1 JSON observations on `lane/observation` only while both
source age and receiver-monotonic elapsed age remain within `max_age_s` (default .3).
Failed inference publishes no observation; downstream freshness gates must stop.
The source optical frame and capture stamp are preserved. Supported uint8 image
encodings are bgr8/rgb8/mono8 with validated row stride.

Provide trusted local `model_path`; no automatic model download. Runtime needs the
existing PyTorch/Ultralytics environment in addition to ROS, OpenCV and NumPy.
Source ROS before running with that Python. Default device is CPU. Model loading
and first inference may require warmup; an expired first result is discarded.

`ros-model` uses both configured recordings and actual best (2).pt on isolated
camera/observation topics. It explicitly allows **5 seconds only for this offline
warmup check**, not live driving. Latest result `.agents/output/ros_model/run-jp8b7j8u/`
preserved timestamps; first inference/transport took 2.09 s, second .14 s. This is
not a throughput guarantee or a reason to increase the live safety timeout.

## Watchdog transport

Input: reliable/volatile depth-1 `std_msgs/String` on relative `lane/command`:
`sequence` (session-increasing integer), `capture_stamp` (source ROS time seconds),
`speed` (m/s), `omega` (rad/s). No other fields accepted; payload is limited to 4096
characters. Source clocks must be synchronized and source age validated. Receiver
expiry uses monotonic time and an independent steady-clock 20 ms timer.

Default output is relative `lane/dry_run_cmd_vel` (`geometry_msgs/Twist`).
`dry_run` is read-only. Hardware output requires an explicitly configured restart
with `dry_run=false` AND `hardware_watchdog_confirmed=true`; neither has been used
or verified on hardware. Test defaults are not calibrated speed/braking limits.

This watchdog cannot protect against its own process/robot computer dying. A
separately verified lower-level motor timeout and physical emergency stop remain
mandatory. No SSH deployment or real robot command publication has occurred.

## Verified (2026-09-16)

- ament package discovered; `ros2 pkg executables` lists `lane_watchdog`.
- colcon build passed (environment setuptools emitted pytest-repeat egg warning).
- colcon test: one pytest bridge passed, executing all 63 core tests.
- fast 63, vision 5, localhost ROS graph 4 tests passed independently.
- Actual model/frames from both recordings reached the ROS observation topic.

Still pending: live camera/scan/odom interfaces and QoS matching, timestamped TF,
coverage validation, control node, process-kill integration scenarios,
simulation, measured calibration and physical acceptance.
