# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Expose repository behavioral suite to colcon/ament's pytest runner."""

from pathlib import Path
import subprocess
import sys


def test_core_suite():
    root = Path(__file__).resolve().parents[2]
    result = subprocess.run([sys.executable, str(root / '.agents/tools/check.py')],
                            cwd=root, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
