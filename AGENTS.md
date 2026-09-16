# Repository Guidelines

## Project Structure

This repository is a ROS 2 Jazzy workspace for Pinky Pro and lane-driving
experiments. Keep ROS packages at the repository root:

- `pinky_bringup/`, `pinky_description/`, `pinky_gz_sim/`: robot bringup,
  URDF/meshes, and Gazebo simulation.
- `pinky_navigation/`: Nav2, SLAM, maps, parameters, RViz, and web launch files.
- `pinky_interfaces/`, `pinky_imu_bno055/`, `pinky_sensor_adc/`,
  `pinky_led/`, `pinky_emotion/`, `pinky_lamp_control/`: interfaces and
  hardware-facing nodes.
- `tests/`: hardware-free Python tests; `.agents/tools/`: repository validation scripts.
- `.agents/`: shared harness code and configuration for coding agents.
  `CLAUDE.md` imports this file so project guidance has a single source.
- `doc/`, `README.md`, and `PROJECT.md`: user documentation and lane-driving scope.

## Build, Test, and Development

Run the fast, hardware-free checks before every change:

```bash
./.agents/tools/harness.sh fast
```

For affected ROS packages, use a clean terminal with ROS 2 Jazzy available:

```bash
./.agents/tools/harness.sh ros pinky_description
```

Replace the package name with each affected package. This builds and tests in
`.agents/output/` without launching nodes, moving motors, or deploying remotely.
Use `bash -n .agents/tools/harness.sh` when changing the shell harness.

For perception replay, configure ignored `.agents/replay.local.json` from
`.agents/replay.example.json`, then run `./.agents/tools/harness.sh replay`. Set `REPLAY_PYTHON`
to a Python with OpenCV, PyTorch and Ultralytics installed. See `doc/replay.md`.
Replay success verifies execution and encoded output, not detection accuracy or driving.

## Coding Style and Safety

Use standard library and existing project patterns before adding dependencies.
Use four-space Python indentation, conventional C++ formatting, and descriptive
`snake_case` Python names. Preserve metres, radians, seconds, and the ROS frame
convention `x` forward, `y` left, `z` up. Keep pixel and metric coordinates
separate, validate parameter bounds, and preserve timestamps and QoS semantics.

Never commit models, recordings, datasets, credentials, or generated build
artifacts. Do not run hardware bringup, publish motor commands, SSH-deploy, or
force-push without explicit approval. Unit tests, ROS builds, simulation, and
physical tests are separate evidence.

## Testing and Pull Requests

Name Python tests `test_*.py` and keep logic hardware-free where possible.
Tests must fail on errors, import failures, and empty discovery. Pull requests
should explain the change, list commands and results, link the relevant issue,
and call out unverified simulation or hardware behavior. Use concise imperative
commit subjects such as `feat: add lane center estimator` or `test: cover stop timeout`.
