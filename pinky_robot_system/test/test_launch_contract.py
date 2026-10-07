"""Tests for the integrated robot launch's safety contract."""

from pathlib import Path
import unittest


class RobotSystemLaunchTest(unittest.TestCase):
    """Protect the single-entry launch and its fail-closed defaults."""

    def setUp(self):
        """Load the launch source from this package."""
        package_root = Path(__file__).resolve().parents[1]
        self.launch_source = (
            package_root / 'launch' / 'robot_system.launch.py'
        ).read_text()

    def test_launch_source_compiles(self):
        """The installed launch entry point must remain valid Python."""
        compile(self.launch_source, 'robot_system.launch.py', 'exec')

    def test_motion_defaults_are_fail_closed(self):
        """Motors and live lane control require explicit operator opt-in."""
        self.assertIn(
            "DeclareLaunchArgument('start_motors', default_value='false')",
            self.launch_source,
        )
        self.assertIn(
            "DeclareLaunchArgument('start_lane_control', default_value='false')",
            self.launch_source,
        )
        self.assertIn(
            "DeclareLaunchArgument('lane_dry_run', default_value='true')",
            self.launch_source,
        )


if __name__ == '__main__':
    unittest.main()
