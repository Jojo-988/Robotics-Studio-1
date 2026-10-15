"""Regression checks for installed launch and the expanded flight model (ROS 2)."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import xacro
from ament_index_python.packages import get_package_share_directory


class ModelTests(unittest.TestCase):
    def model(self, pitch=None):
        share = Path(get_package_share_directory('41068_ignition_bringup'))
        args = {'prefix': 'parrot1_', 'gz_model_name': 'parrot1'}
        if pitch is not None:
            args['camera_pitch'] = str(pitch)
        return ET.fromstring(xacro.process_file(
            str(share/'urdf_parrot/parrot.urdf.xacro'), mappings=args).toxml())

    def test_odometry_reports_three_dimensions_on_the_bridged_topic(self):
        plugin = self.model().find(
            ".//plugin[@name='ignition::gazebo::systems::OdometryPublisher']")
        self.assertEqual(plugin.findtext('dimensions'), '3')
        self.assertEqual(plugin.findtext('odom_topic'), '/model/parrot1/odometry')
        self.assertEqual(plugin.findtext('odom_frame'), 'parrot1_odom')
        self.assertEqual(plugin.findtext('robot_base_frame'), 'parrot1_base_link')

    def test_coverage_camera_pitch_reaches_generated_model(self):
        joint = self.model(math.pi/2).find("joint[@name='parrot1_camera_joint']/origin")
        self.assertAlmostEqual(float(joint.attrib['rpy'].split()[1]), math.pi/2)

    def test_other_launches_keep_the_default_camera_angle(self):
        joint = self.model().find("joint[@name='parrot1_camera_joint']/origin")
        self.assertAlmostEqual(float(joint.attrib['rpy'].split()[1]), .785398)


class InstalledLaunchTests(unittest.TestCase):
    def test_installed_launch_starts_patrol_without_odometry(self):
        # Running the real installed executable detects missing shebangs, unlike import tests.
        rclpy.init()
        observer = Node('patrol_launch_test_observer')
        statuses = []
        subscription = observer.create_subscription(
            String, '/parrot1/patrol/status', lambda m: statuses.append(json.loads(m.data)), 10)
        process = None
        try:
            with tempfile.TemporaryDirectory(prefix='patrol-launch-test-') as directory:
                env = dict(os.environ, ROS_LOG_DIR=directory)
                with open(Path(directory)/'launch.log', 'w+') as log:
                    process = subprocess.Popen(
                        ['ros2', 'launch', '41068_ignition_bringup',
                         '41068_coverage_patrol.launch.py', 'start_sim:=false'],
                        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
                    deadline = time.monotonic()+12
                    try:
                        while not statuses and time.monotonic() < deadline and process.poll() is None:
                            rclpy.spin_once(observer, timeout_sec=.1)
                        log.flush()
                        log.seek(0)
                        output = log.read()
                        self.assertTrue(statuses, output)
                        self.assertEqual(statuses[-1]['state'], 'WAITING')
                        self.assertFalse(statuses[-1]['odometry_ready'])
                        self.assertIsNone(process.poll(), output)
                    finally:
                        if process.poll() is None:
                            os.killpg(process.pid, signal.SIGINT)
                            try:
                                process.wait(timeout=10)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGKILL)
                                process.wait(timeout=5)
        finally:
            observer.destroy_subscription(subscription)
            observer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
