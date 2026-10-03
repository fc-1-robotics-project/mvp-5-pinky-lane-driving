"""Expose the pytest-style policy tests to setuptools test discovery."""

from pathlib import Path
import subprocess
import sys
import unittest


class FleetSafetySuiteTest(unittest.TestCase):
    """Run the package's hardware-free policy tests."""

    def test_policy_suite(self):
        """Require every pytest-style policy test to pass."""
        test_directory = Path(__file__).resolve().parent
        test_files = sorted(
            path
            for path in test_directory.glob('test_*.py')
            if path.name != Path(__file__).name
        )
        result = subprocess.run(
            [sys.executable, '-m', 'pytest', '-q', *map(str, test_files)],
            cwd=test_directory.parent,
            capture_output=True,
            text=True,
            timeout=60,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
