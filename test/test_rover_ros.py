
from copy import deepcopy
import math
import time
import unittest

try:
    import rclpy
except ImportError:
    rclpy = None
else:
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from nav_msgs.msg import Path
    from nav2_msgs.action import ComputePathToPose, FollowPath
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.task import Future
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool, Trigger
    from tf2_ros import TransformBroadcaster
    from rover_navigation.node import RoverNavigation


if rclpy is not None:
    class StackFixture(Node):
        def __init__(self, external=False):
            super().__init__('rover_test_fixture')
            self.pose = (0., 0., 0.)
            self.sensors = True
            self.commands = True
            self.heartbeat = True
            self.blocked = False
            self.defer_plan = False
            self.empty_path = False
            self.velocity = .2
            self.plan_requests = []
            self.follow_requests = []
            self.sessions = []
            self.last_output = None
            self.tf = TransformBroadcaster(self)
            self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
            self.vel_pub = self.create_publisher(
                Twist, 'avoidance/cmd_vel' if external else 'rover/nav2_cmd_vel', 10)
            self.block_pub = self.create_publisher(Bool, 'avoidance/blocked', 10)
            self.goal_pub = self.create_publisher(PoseStamped, 'rover/goal', 10)
            self.cmd_sub = self.create_subscription(
                Twist, 'cmd_vel', lambda msg: setattr(self, 'last_output', msg), 10)
            self.plan_server = ActionServer(
                self, ComputePathToPose, 'compute_path_to_pose', self.plan,
                cancel_callback=lambda handle: CancelResponse.ACCEPT)
            self.follow_server = ActionServer(
                self, FollowPath, 'avoidance/follow_path' if external else 'follow_path', self.follow,
                cancel_callback=lambda handle: CancelResponse.ACCEPT)
            self.timer = self.create_timer(.02, self.tick)

        async def plan(self, handle):
            self.plan_requests.append(deepcopy(handle.request))
            if self.defer_plan:
                future = Future()
                self.sessions.append((handle, future, 'plan'))
                await future
            result = ComputePathToPose.Result()
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.succeed()
                result.path.header = deepcopy(handle.request.goal.header)
                if not self.empty_path:
                    result.path.poses = [deepcopy(handle.request.start), deepcopy(handle.request.goal)]
            return result

        async def follow(self, handle):
            self.follow_requests.append(deepcopy(handle.request))
            future = Future()
            self.sessions.append((handle, future, 'follow'))
            outcome = await future
            if handle.is_cancel_requested:
                handle.canceled()
            elif outcome == 'success':
                handle.succeed()
            else:
                handle.abort()
            return FollowPath.Result()

        def complete_follow(self):
            for handle, future, kind in self.sessions:
                if kind == 'follow' and not future.done():
                    future.set_result('success')

        def tick(self):
            stamp = self.get_clock().now().to_msg()
            if self.sensors:
                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = 'husky1_map'
                transform.child_frame_id = 'husky1_base_link'
                transform.transform.translation.x, transform.transform.translation.y = self.pose[:2]
                transform.transform.rotation.z = math.sin(self.pose[2]/2)
                transform.transform.rotation.w = math.cos(self.pose[2]/2)
                self.tf.sendTransform(transform)
                scan = LaserScan()
                scan.header.stamp = stamp
                scan.header.frame_id = 'husky1_base_scan'
                scan.range_min, scan.range_max = .2, 40.
                scan.ranges = [10.] * 10
                self.scan_pub.publish(scan)
            if self.heartbeat:
                self.block_pub.publish(Bool(data=self.blocked))
            for handle, future, kind in self.sessions:
                if not future.done() and handle.is_cancel_requested:
                    future.set_result('canceled')
                if kind == 'follow' and not future.done() and self.commands:
                    msg = Twist()
                    msg.linear.x = self.velocity
                    self.vel_pub.publish(msg)


@unittest.skipIf(rclpy is None, 'Requires ROS 2 Humble')
class RoverRosTests(unittest.TestCase):
    def setUp(self):
        external = self._testMethodName.startswith('test_external_')
        args = ['--ros-args', '-r', '__ns:=/husky1',
                        '-r', '/tf:=/husky1/tf', '-r', '/tf_static:=/husky1/tf_static',
                        '-p', 'sensor_timeout:=0.25', '-p', 'tf_timeout:=0.25',
                        '-p', 'command_timeout:=0.15', '-p', 'avoidance_timeout:=0.15']
        if external:
            args.extend(['-p', 'controller_action:=avoidance/follow_path',
                         '-p', 'controller_cmd_topic:=avoidance/cmd_vel',
                         '-p', 'require_avoidance_heartbeat:=true'])
        rclpy.init(args=args)
        self.nav = RoverNavigation()
        self.stack = StackFixture(external=external)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.nav)
        self.executor.add_node(self.stack)
        self.until(lambda: self.nav.current_pose is not None and self.nav.scan_received is not None)

    def tearDown(self):
        self.nav.goal = None
        self.nav._invalidate('CANCELED', 'Test complete')
        self.until(lambda: self.nav.operation is None, timeout=2.)
        self.executor.shutdown()
        self.stack.plan_server.destroy()
        self.stack.follow_server.destroy()
        self.nav.destroy_node()
        self.stack.destroy_node()
        rclpy.shutdown()

    def until(self, predicate, timeout=4.):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        self.fail(f'Timeout: {self.nav.state}, {self.nav.reason}')

    def run_for(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)

    def goal(self, x=2., y=1., yaw=.4, frame='husky1_map'):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.stack.goal_pub.publish(msg)

    def moving(self):
        return self.stack.last_output is not None and self.stack.last_output.linear.x > 0

    def stopped(self):
        return self.stack.last_output is not None and self.stack.last_output.linear.x == 0

    def service(self, srv_type, name, request):
        client = self.stack.create_client(srv_type, name)
        self.until(client.service_is_ready)
        future = client.call_async(request)
        self.until(future.done)
        response = future.result()
        self.stack.destroy_client(client)
        self.assertTrue(response.success, response.message)

    def test_plans_from_current_pose_and_verifies_goal_heading(self):
        self.goal()
        self.until(self.moving)
        request = self.stack.plan_requests[0]
        self.assertTrue(request.use_start)
        self.assertEqual(request.start.pose.position.x, 0.)
        self.assertEqual(request.goal.header.frame_id, 'husky1_map')
        self.stack.pose = (2., 1., .4)
        self.until(lambda: self.nav.current_pose is not None and all(
            abs(a-b) < 1e-8 for a, b in zip(self.nav.current_pose, self.stack.pose)))
        self.stack.complete_follow()
        self.until(lambda: self.nav.state == 'SUCCEEDED' and self.stopped())

    def test_wrong_frame_is_rejected_without_planning(self):
        self.goal(frame='parrot1_map')
        self.until(lambda: bool(self.nav.last_rejection))
        self.assertFalse(self.stack.plan_requests)
        self.until(self.stopped)

    def test_empty_plan_never_moves(self):
        self.stack.empty_path = True
        self.goal()
        self.until(lambda: self.nav.state == 'FAILED')
        self.assertFalse(self.stack.follow_requests)
        self.assertTrue(self.stopped())

    def test_preemption_discards_canceled_plan(self):
        self.stack.defer_plan = True
        self.goal(2., 1.)
        self.until(lambda: len(self.stack.plan_requests) == 1)
        self.stack.defer_plan = False
        self.goal(3., 2.)
        self.until(self.moving)
        self.assertEqual(len(self.stack.follow_requests), 1)
        self.assertEqual(self.stack.follow_requests[0].path.poses[-1].pose.position.x, 3.)

    def test_pause_then_resume_replans_from_new_pose(self):
        self.goal()
        self.until(self.moving)
        self.service(SetBool, 'rover/pause', SetBool.Request(data=True))
        self.until(lambda: self.nav.operation is None and self.stopped())
        self.stack.pose = (.5, .2, 0.)
        self.until(lambda: self.nav.current_pose == self.stack.pose)
        self.service(SetBool, 'rover/pause', SetBool.Request(data=False))
        self.until(lambda: len(self.stack.plan_requests) >= 2 and self.moving())
        self.assertAlmostEqual(self.stack.plan_requests[-1].start.pose.position.x, .5)

    def test_blocked_stops_and_unblocked_replans(self):
        self.goal()
        self.until(self.moving)
        self.stack.blocked = True
        self.until(lambda: self.nav.state == 'BLOCKED' and self.stopped())
        self.run_for(.1)
        self.assertTrue(self.stopped())
        self.stack.blocked = False
        self.until(lambda: len(self.stack.plan_requests) >= 2 and self.moving())

    def test_stale_command_stops_without_reusing_last_velocity(self):
        self.goal()
        self.until(self.moving)
        self.stack.commands = False
        self.run_for(.3)
        self.assertTrue(self.stopped())

    def test_lost_scan_stops_and_cancels_controller(self):
        self.goal()
        self.until(self.moving)
        self.stack.sensors = False
        self.until(lambda: self.nav.state == 'WAITING' and self.stopped())
        self.until(lambda: self.nav.operation is None)

    def test_external_heartbeat_loss_stops(self):
        self.goal()
        self.until(self.moving)
        self.assertTrue(self.stack.follow_requests)
        self.stack.heartbeat = False
        self.until(lambda: 'heartbeat' in self.nav.reason and self.stopped())

    def test_cancel_and_replan_services(self):
        self.goal()
        self.until(self.moving)
        self.service(Trigger, 'rover/replan', Trigger.Request())
        self.until(lambda: len(self.stack.plan_requests) >= 2 and self.moving())
        self.service(Trigger, 'rover/cancel', Trigger.Request())
        self.until(lambda: self.nav.operation is None and self.stopped())
        self.assertIsNone(self.nav.goal)

    def test_false_controller_success_does_not_claim_arrival(self):
        self.goal()
        self.until(self.moving)
        self.stack.complete_follow()
        self.until(lambda: self.nav.state == 'FAILED')
        self.assertIn('outside goal pose tolerance', self.nav.reason)


if __name__ == '__main__':
    unittest.main()
