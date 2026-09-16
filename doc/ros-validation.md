# ROS validation boundary

Current implemented ROS endpoint: `pinky_lane_driving lane_watchdog`. Perception
and control adapters are still pending; do not interpret this as an autonomous
driving launch. The pure algorithms live under the standard ament Python package
path `pinky_lane_driving/pinky_lane_driving/`.

## Reproducible checks

```bash
./.agents/tools/harness.sh fast
./.agents/tools/harness.sh ros pinky_lane_driving
./.agents/tools/harness.sh ros-smoke
```

`ros-smoke` uses domain 177, localhost discovery and a process-specific namespace.
It never starts bringup or publishes the motor `cmd_vel` topic. The graph test
proves valid command delivery, timeout zero output after publisher silence,
rejection of stale/malformed messages and read-only runtime safety parameters.

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
- fast 63, vision 4, localhost ROS graph 2 tests passed independently.

Still pending: live camera/scan/odom interfaces and QoS matching, timestamped TF,
coverage validation, perception/control nodes, process-kill integration scenarios,
simulation, measured calibration and physical acceptance.
