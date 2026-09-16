# Development baseline

## Repository

- Local checkout: `/home/seunghoon/ros/pinky-lane-driving`
- Branch: `feat/lane-driving-tdd`
- origin: `git@github.com:jsh0116/pinky-lane-driving.git`
- upstream: `https://github.com/pinklab-art/pinky_pro.git`
- Imported main: `75f76e8b7cd971c32c07233f7accd418dab1d6b4`

This is an independent private repository with upstream Git history, not a GitHub
Fork. Original README and Apache-2.0 LICENSE are preserved. See PROJECT.md and
GitHub issues #1/#2 for the lane mission. Do not nest this checkout inside another
Pinky source tree: colcon can discover duplicate package names.

## Fast checks (no ROS, network or motors)

```bash
./tools/harness.sh fast
```

Python 3 standard-library unittest; no additional dependency installation.
The runner fails on failed tests, import errors and zero tests. It also checks
Git diff whitespace. This is not a full style/type/lint checker.
Harness self-tests use generated temporary test modules to prove failure propagation.
Mission fixtures and calibration have not been created yet.

## ROS build/tests (explicit packages, no launch)

In a fresh terminal without the old Pinky overlay sourced:

```bash
./tools/harness.sh ros pinky_description
```

Uses ROS 2 Jazzy and colcon, builds selected packages plus dependencies, then runs
selected package tests. Outputs stay in ignored `.harness/`. It never installs
system dependencies or launches nodes. Missing dependencies are failures to report,
not reasons to invoke sudo automatically. Upstream lint failures are recorded
separately from new lane tests. A build is not physical validation.

## Simulation and hardware gates

Simulation automation is deferred until the lane package exists. Use a separate
ROS domain and test topic; ensure no real robot bridge is connected. Add a bounded
process lifecycle and assertions on stop behavior before enabling that harness mode.
Do not use generic hardware launch files as a test runner.

Before motor tests: measured camera/floor calibration and lane width, latency and
braking limits, single command owner, robot-side timeout, physical emergency stop
and fall protection. Deployment and motor operation require explicit approval.

## TDD workflow

1. Add a focused test; run fast and confirm the intended failure.
2. Commit `test: ... (RED)` with the failure reason.
3. Implement the minimum; run fast and commit `feat: ... (GREEN)`.
4. Refactor only while green; add package-specific ROS validation when relevant.

The earlier control draft is preserved in stash `pre-bootstrap control test draft`.
Do not apply the whole stash over the imported .gitignore. Restore its test file
selectively when starting controller TDD. No lane implementation exists yet.
Do not push until the user explicitly authorizes it.

## Bootstrap verification (2026-09-16)

- Python 3.12.3, ROS 2 Jazzy, local x86_64 development environment.
- `./tools/harness.sh fast`: 2 tests passed, covering four runner outcomes and
  unique upstream package names.
- `bash -n tools/harness.sh`: passed.
- `./tools/harness.sh ros pinky_description`: build passed; CTest 3/4 passed.
  Upstream flake8 failed; lint_cmake, pep257 and xmllint passed. Upstream source
  is unchanged; this is a baseline failure, not a passing ROS suite.
- No simulation, camera inference, motor operation or SSH deployment performed.
