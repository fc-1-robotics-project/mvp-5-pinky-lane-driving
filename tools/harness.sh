#!/usr/bin/env bash
# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
mode="${1:-fast}"
if (( $# )); then shift; fi
case "$mode" in
  fast)
    if (( $# )); then echo 'fast takes no arguments' >&2; exit 2; fi
    git diff --check
    git diff --cached --check
    python3 tools/check.py
    ;;
  ros)
    if (( $# == 0 )); then echo 'Usage: tools/harness.sh ros PACKAGE...' >&2; exit 2; fi
    source /opt/ros/jazzy/setup.bash
    available="$(colcon list --names-only)"
    for package in "$@"; do
      if ! grep -Fxq -- "$package" <<< "$available"; then
        echo "Unknown local package: $package" >&2; exit 2
      fi
    done
    colcon --log-base .harness/log build --base-paths . \
      --build-base .harness/build --install-base .harness/install \
      --packages-up-to "$@" --symlink-install
    source .harness/install/setup.bash
    test_status=0
    colcon --log-base .harness/log test --base-paths . \
      --build-base .harness/build --install-base .harness/install \
      --packages-select "$@" --return-code-on-test-failure \
      --event-handlers console_direct+ || test_status=$?
    result_status=0
    colcon test-result --test-result-base .harness/build --verbose || result_status=$?
    if (( test_status != 0 )); then exit "$test_status"; fi
    exit "$result_status"
    ;;
  replay)
    "${REPLAY_PYTHON:-python3}" tools/replay.py "$@"
    ;;
  *) echo 'Usage: tools/harness.sh [fast | ros PACKAGE... | replay --config FILE]' >&2; exit 2 ;;
esac
