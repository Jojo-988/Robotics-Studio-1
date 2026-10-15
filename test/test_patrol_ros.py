"""Exercise the ROS adapter over real topics/services without Gazebo."""

from pathlib import Path
import sys
import time
import unittest

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_srvs.srv import SetBool, Trigger
except ImportError:
    rclpy = None
else:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
    from coverage_patrol import CoveragePatrol


@unittest.skipIf(rclpy is None, 'ROS 2 is not installed in this Python environment')
class RosAdapterTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args', '-p', 'odom_timeout:=0.2', '-p', 'control_rate:=40.0'])
        self.patrol = CoveragePatrol()
        self.observer = Node('patrol_test_observer')
        self.publisher = self.observer.create_publisher(Odometry, 'odometry', 10)
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

    def run_for(self, seconds, publish=False, frame='parrot1_odom'):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if publish:
                msg = Odometry()
                msg.header.stamp = self.observer.get_clock().now().to_msg()
                msg.header.frame_id = frame
                msg.child_frame_id = 'parrot1_base_link'
                msg.pose.pose.position.x = 2.0
                msg.pose.pose.position.z = 0.8
                msg.pose.pose.orientation.w = 1.0
                self.publisher.publish(msg)
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


if __name__ == '__main__':
    unittest.main()
