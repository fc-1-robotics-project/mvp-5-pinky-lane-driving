#!/usr/bin/env python3
# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Hardware-free unittest runner; an empty suite is not a passing suite."""

from pathlib import Path
import sys
import unittest


def run_suite(directory):
    """Return a shell status, including nonzero for zero discovered tests."""
    suite = unittest.defaultTestLoader.discover(str(directory))
    if suite.countTestCases() == 0:
        print('ERROR: no tests discovered', file=sys.stderr)
        return 2
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    sys.exit(run_suite(root / 'tests'))
