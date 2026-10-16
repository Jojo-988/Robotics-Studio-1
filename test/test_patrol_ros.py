"""Exercise the ROS adapter over real topics/services without Gazebo."""

from pathlib import Path
import sys
import time
import unittest
import struct

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
    from std_srvs.srv import SetBool, Trigger
except ImportError:
    rclpy = None
else:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
    from coverage_patrol import CoveragePatrol


@unittest.skipIf(rclpy is None, 'ROS 2 is not installed in this Python environment')
class RosAdapterTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args', '-p', 'odom_timeout:=0.2', '-p', 'control_rate:=40.0',
                        '-p', 'ready_hold:=0.03', '-p', 'sensor_timeout:=0.2'])
        self.patrol = CoveragePatrol()
        # Do not let host NTP / WSL wall-clock corrections impersonate a simulator
        # reset. Use a monotonic test timebase; /clock rollback is tested separately.
        self.time_origin = time.monotonic()
        self.patrol._seconds = lambda: 10.0 + time.monotonic() - self.time_origin
        self.observer = Node('patrol_test_observer')
        self.publisher = self.observer.create_publisher(Odometry, 'odometry', 10)
        self.rgb_pub = self.observer.create_publisher(Image, 'camera/image', 10)
        self.depth_pub = self.observer.create_publisher(Image, 'camera/depth/image', 10)
        self.lidar_pub = self.observer.create_publisher(PointCloud2, 'scan/points', 10)
        self.imu_pub = self.observer.create_publisher(Imu, 'imu', 10)
        self.commands = []
        self.subscription = self.observer.create_subscription(Twist, 'cmd_vel', self.commands.append, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.patrol)
        self.executor.add_node(self.observer)
        self.run_for(0.2)

    def tearDown(self):
        self.executor.shutdown()
        self.patrol.destroy_node()
        self.observer.destroy_node()
        rclpy.shutdown()

    def run_for(self, seconds, publish=False, frame='parrot1_odom', sensors=True, valid_depth=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if publish:
                msg = Odometry()
                msg.header.stamp = Time(nanoseconds=int(self.patrol._seconds() * 1e9)).to_msg()
                msg.header.frame_id = frame
                msg.child_frame_id = 'parrot1_base_link'
                msg.pose.pose.position.x = 2.0
                msg.pose.pose.position.z = 0.8
                msg.pose.pose.orientation.w = 1.0
                self.publisher.publish(msg)
                if sensors:
                    rgb = Image(height=1, width=1, encoding='rgb8', step=3, data=[10, 20, 30])
                    rgb.header.stamp = msg.header.stamp
                    rgb.header.frame_id = 'parrot1_camera_link'
                    self.rgb_pub.publish(rgb)
                    depth = Image(height=1, width=1, encoding='32FC1', step=4,
                                  data=struct.pack('<f', 15.0 if valid_depth else float('nan')))
                    depth.header = rgb.header
                    self.depth_pub.publish(depth)
                    cloud = PointCloud2(height=1, width=1, point_step=12, row_step=12,
                                        data=struct.pack('<fff', 0, 0, -15),
                                        fields=[PointField(name=n, offset=i*4, datatype=7, count=1)
                                                for i, n in enumerate('xyz')])
                    cloud.header.stamp = msg.header.stamp
                    cloud.header.frame_id = 'parrot1_base_scan'
                    self.lidar_pub.publish(cloud)
                    imu = Imu()
                    imu.header.stamp = msg.header.stamp
                    imu.header.frame_id = 'parrot1_imu_link'
                    imu.orientation.w = 1.0
                    imu.linear_acceleration.z = 9.81
                    self.imu_pub.publish(imu)
            self.executor.spin_once(timeout_sec=0.01)

    def call_service(self, kind, name, request):
        client = self.observer.create_client(kind, name)
        self.assertTrue(client.wait_for_service(timeout_sec=1))
        future = client.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=1)
        self.assertTrue(future.done())
        self.assertTrue(future.result().success)
        self.observer.destroy_client(client)

    def assert_stopped(self):
        self.assertTrue(self.commands)
        for msg in self.commands[-3:]:
            self.assertEqual((msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z), (0, 0, 0, 0))

    def test_odometry_watchdog_stops_and_recovers_same_target(self):
        self.run_for(0.3, publish=True)
        self.assertGreater(self.commands[-1].linear.z, 0)
        target = self.patrol.mission.target
        self.run_for(0.4)
        self.assert_stopped()
        self.assertTrue(self.patrol.wait_reason)
        self.assertEqual(self.patrol.mission.grid.fraction, 0)
        self.run_for(0.3, publish=True)
        self.assertGreater(self.commands[-1].linear.z, 0)
        self.assertEqual(self.patrol.mission.target, target)

    def test_pause_resume_and_latched_stop_services(self):
        self.run_for(0.3, publish=True)
        self.call_service(SetBool, 'patrol/pause', SetBool.Request(data=True))
        self.run_for(0.2, publish=True)
        self.assert_stopped()
        self.call_service(SetBool, 'patrol/pause', SetBool.Request(data=False))
        self.run_for(0.2, publish=True)
        self.assertGreater(self.commands[-1].linear.z, 0)
        self.call_service(Trigger, 'patrol/stop', Trigger.Request())
        self.run_for(0.2, publish=True)
        self.assert_stopped()
        self.assertEqual(self.patrol.mission.state, 'STOPPED')

    def test_wrong_coordinate_frame_stops_flight(self):
        self.run_for(0.2, publish=True)
        self.run_for(0.2, publish=True, frame='wrong_frame')
        self.assert_stopped()
        self.assertEqual(self.patrol.mission.state, 'ERROR')

    def test_missing_or_invalid_depth_blocks_takeoff(self):
        self.run_for(.4, publish=True, sensors=False)
        self.assert_stopped()
        self.assertIsNone(self.patrol.mission.home)
        self.run_for(.4, publish=True, valid_depth=False)
        self.assert_stopped()
        self.assertIn('depth', self.patrol.wait_reason)
        self.run_for(.4, publish=True)
        self.assertGreater(self.commands[-1].linear.z, 0)

    def test_sensor_loss_preserves_target_and_operator_pause(self):
        self.run_for(.4, publish=True)
        target = self.patrol.mission.target
        deadline = self.patrol.mission.deadline
        self.run_for(.4, publish=True, sensors=False)
        self.assert_stopped()
        self.assertTrue(self.patrol.mission.paused)
        self.call_service(SetBool, 'patrol/pause', SetBool.Request(data=True))
        self.run_for(.4, publish=True)
        self.assert_stopped()  # Healthy data must not override the operator's pause.
        self.call_service(SetBool, 'patrol/pause', SetBool.Request(data=False))
        self.run_for(.3, publish=True)
        self.assertGreater(self.commands[-1].linear.z, 0)
        self.assertEqual(self.patrol.mission.target, target)
        self.assertGreater(self.patrol.mission.deadline, deadline + .3)

    def test_startup_and_persistent_loss_have_bounded_timeouts(self):
        self.patrol.set_parameters([Parameter('startup_timeout', value=0.3)])
        self.run_for(.4, publish=True, sensors=False)
        self.assertEqual(self.patrol.mission.state, 'ERROR')
        self.assertIn('Startup timeout', self.patrol.mission.reason)

    def test_persistent_sensor_loss_latches_error(self):
        self.patrol.set_parameters([Parameter('health_loss_timeout', value=0.25)])
        self.run_for(.4, publish=True)
        self.run_for(.7, publish=True, sensors=False)
        self.assertEqual(self.patrol.mission.state, 'ERROR')
        self.assertFalse(self.patrol.mission.paused)
        self.run_for(.3, publish=True)
        self.assert_stopped()

    def test_second_velocity_controller_is_rejected(self):
        other = self.observer.create_publisher(Twist, 'cmd_vel', 10)
        try:
            self.run_for(.5, publish=True)
            self.assertEqual(self.patrol.mission.state, 'ERROR')
            self.assertIn('Multiple publishers', self.patrol.mission.reason)
        finally:
            self.observer.destroy_publisher(other)

    def test_clock_subscription_receives_data_and_latches_rollback(self):
        clock_pub = self.observer.create_publisher(Clock, '/clock', 10)
        try:
            end = time.monotonic() + .3
            value = 10
            while time.monotonic() < end:
                message = Clock()
                message.clock.sec = value
                value += 1
                clock_pub.publish(message)
                self.executor.spin_once(timeout_sec=.01)
            self.assertGreater(self.patrol.streams['clock'].accepted, 1)
            message.clock.sec = 1
            clock_pub.publish(message)
            self.run_for(.2)
            self.assertEqual(self.patrol.mission.state, 'ERROR')
            self.assertIn('clock reset', self.patrol.mission.reason)
            self.assert_stopped()
        finally:
            self.observer.destroy_publisher(clock_pub)


if __name__ == '__main__':
    unittest.main()
