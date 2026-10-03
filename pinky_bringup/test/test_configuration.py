"""Hardware-free validation of the robot-local bringup configuration."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

from pinky_bringup.camera import load_camera_info
import yaml


class BringupConfigurationTest(unittest.TestCase):
    """Check that launch and installed YAML inputs remain parseable."""

    def test_launch_and_yaml_parse(self):
        package = Path(__file__).resolve().parents[1]
        ElementTree.parse(package / 'launch' / 'bringup_robot.launch.xml')
        for path in sorted((package / 'config').glob('*.yaml')):
            with path.open() as stream:
                self.assertIsNotNone(yaml.safe_load(stream), str(path))

    def test_camera_calibration_loads(self):
        package = Path(__file__).resolve().parents[1]
        info = load_camera_info(
            package / 'config' / 'pinky_camera.yaml', 640, 480
        )
        self.assertEqual(info.distortion_model, 'plumb_bob')
        self.assertEqual(len(info.k), 9)
        self.assertEqual(len(info.p), 12)


if __name__ == '__main__':
    unittest.main()
