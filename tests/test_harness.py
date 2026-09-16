# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
"""Exercise failure propagation without importing any robot drivers."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


class HarnessTest(unittest.TestCase):
    def test_runner_exit_codes(self):
        cases = [('', 2),
                 ('import unittest\nclass T(unittest.TestCase):\n'
                  ' def test_ok(self): self.assertTrue(True)\n', 0),
                 ('import unittest\nclass T(unittest.TestCase):\n'
                  ' def test_bad(self): self.fail("intentional red")\n', 1),
                 ('raise ImportError("missing dependency")\n', 1)]
        for source, expected in cases:
            with self.subTest(expected=expected), tempfile.TemporaryDirectory() as directory:
                if source:
                    (Path(directory) / 'test_sample.py').write_text(source)
                result = subprocess.run(
                    [sys.executable, '-c',
                     'from tools.check import run_suite; import sys; '
                     'sys.exit(run_suite(sys.argv[1]))', directory],
                    cwd=ROOT, capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, expected, result.stderr)

    def test_upstream_manifests_unique(self):
        manifests = list(ROOT.glob('*/package.xml'))
        names = [ET.parse(path).getroot().findtext('name') for path in manifests]
        self.assertTrue(names)
        self.assertTrue(all(names))
        self.assertEqual(len(names), len(set(names)))


if __name__ == '__main__':
    unittest.main()
