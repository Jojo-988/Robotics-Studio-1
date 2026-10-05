
from copy import deepcopy
import json
import math
import signal
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .core import bounded_command, fresh, planar_pose, pose_errors


class RoverNavigation(Node):
    def __init__(self):
        super().__init__('rover_navigation')
        defaults = {
            'global_frame': 'husky1_map', 'base_frame': 'husky1_base_link',
            'planner_action': 'compute_path_to_pose', 'planner_id': 'GridBased',
            'controller_action': 'follow_path', 'controller_id': 'FollowPath',
            'goal_checker_id': 'general_goal_checker',
            'controller_cmd_topic': 'rover/nav2_cmd_vel',
            'require_avoidance_heartbeat': False,
            'max_speed': 0.6, 'max_yaw_rate': 0.8, 'control_rate': 20.0,
            'command_timeout': 0.5, 'sensor_timeout': 1.5, 'tf_timeout': 1.0,
            'avoidance_timeout': 0.5, 'server_timeout': 30.0,
            'planning_timeout': 15.0, 'following_timeout': 180.0,
            'cancel_timeout': 5.0, 'xy_tolerance': 0.25,
            'yaw_tolerance': 0.20, 'max_replans': 3,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        for key, value in self.p.items():
            if isinstance(defaults[key], float) and (
                    not math.isfinite(value) or value <= 0):
                raise ValueError(f'{key} must be finite and positive')
        if self.p['max_replans'] < 0:
            raise ValueError('max_replans must be nonnegative')

        self.state, self.reason = 'IDLE', 'Waiting for a goal pose'
        self.last_rejection = ''
        self.goal = None
        self.goal_xyyaw = None
        self.generation = 0
        self.operation = None
        self.pending_path = None
        self.paused = False
        self.fault = False
        self.blocked = False
        self.blocked_received = None
        self.scan_received = self.scan_stamp = None
        self.command = (0.0, 0.0)
        self.command_received = None
        self.current_pose = None
        self.replans = 0
        self.wait_started = time.monotonic()
        self.last_ros_time = None
        self.feedback = {}
        self.stop_requested = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.planner = ActionClient(self, ComputePathToPose, self.p['planner_action'])
        self.controller = ActionClient(self, FollowPath, self.p['controller_action'])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, 'rover/path', latched)
        self.status_pub = self.create_publisher(String, 'rover/status', latched)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.goal_sub = self.create_subscription(PoseStamped, 'rover/goal', self._goal, 10)
        self.cmd_sub = self.create_subscription(
            Twist, self.p['controller_cmd_topic'], self._command, 1)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self._scan, qos_profile_sensor_data)
        self.blocked_sub = self.create_subscription(
            Bool, 'avoidance/blocked', self._blocked, 1)
        self.pause_srv = self.create_service(SetBool, 'rover/pause', self._pause)
        self.cancel_srv = self.create_service(Trigger, 'rover/cancel', self._cancel)
        self.replan_srv = self.create_service(Trigger, 'rover/replan', self._replan)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1/self.p['control_rate'], self._tick, clock=steady)
        self.status_timer = self.create_timer(0.5, self._report, clock=steady)
        self.get_logger().info('Rover ready: send PoseStamped to rover/goal in ' +
                               self.p['global_frame'])

    def _seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp(header):
        return header.stamp.sec + header.stamp.nanosec/1e9

    @staticmethod
    def _pose_tuple(pose):
        p, q = pose.position, pose.orientation
        return planar_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def _set_state(self, state, reason):
        if (state, reason) != (self.state, self.reason):
            self.get_logger().info(f'{state}: {reason}')
        self.state, self.reason = state, reason

    def _zero(self):
        self.command_received = None
        self.command = (0.0, 0.0)
        self.cmd_pub.publish(Twist())

    def _clear_path(self):
        self.pending_path = None
        msg = Path()
        msg.header.frame_id = self.p['global_frame']
        msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(msg)

    def _invalidate(self, state, reason):
        self.generation += 1
        self._zero()
        self._clear_path()
        self.wait_started = time.monotonic()
        self._set_state(state, reason)
        if self.operation is not None:
            self.operation.setdefault('cancel_started', time.monotonic())
            self._request_cancel(self.operation)

    def _fail(self, reason):
        self.fault = True
        self._invalidate('FAILED', reason)

    def _goal(self, msg):
        try:
            if msg.header.frame_id != self.p['global_frame']:
                raise ValueError('Goal must be transformed into ' + self.p['global_frame'])
            goal_xyyaw = self._pose_tuple(msg.pose)
        except ValueError as error:
            self.last_rejection = str(error)
            self.get_logger().warning('Goal rejected: ' + str(error))
            return
        # A timed-out old controller may still be executing: require it to end.
        if self.fault and self.operation is not None:
            self.last_rejection = 'Old action has not terminated; restore/restart its server'
            return
        self.goal = deepcopy(msg)
        self.goal_xyyaw = goal_xyyaw
        self.goal.pose.position.z = 0.0
        self.goal.pose.orientation.x = self.goal.pose.orientation.y = 0.0
        self.goal.pose.orientation.z = math.sin(goal_xyyaw[2]/2)
        self.goal.pose.orientation.w = math.cos(goal_xyyaw[2]/2)
        self.goal.header.stamp = self.get_clock().now().to_msg()
        self.replans, self.fault, self.last_rejection = 0, False, ''
        self.feedback = {}
        self._invalidate('WAITING', 'New goal accepted; waiting for sensors and servers')

    def _scan(self, msg):
        stamp = self._stamp(msg.header)
        if stamp == self.scan_stamp:
            return
        # +inf is a valid no-return range; an entirely NaN/invalid scan is not.
        if not msg.ranges or not any(
                (math.isfinite(v) and v >= msg.range_min) or v == math.inf
                for v in msg.ranges):
            return
        self.scan_stamp, self.scan_received = stamp, time.monotonic()

    def _blocked(self, msg):
        self.blocked_received = time.monotonic()
        self.blocked = msg.data
        if msg.data:
            self._zero()

    def _command(self, msg):
        # Discard controller output outside the current accepted action.
        op = self.operation
        if (self.state != 'FOLLOWING' or op is None or op['kind'] != 'follow'
                or op['generation'] != self.generation):
            return
        try:
            self.command = bounded_command(
                (msg.linear.x, msg.linear.y, msg.linear.z,
                 msg.angular.x, msg.angular.y, msg.angular.z),
                self.p['max_speed'], self.p['max_yaw_rate'])
            self.command_received = time.monotonic()
        except ValueError as error:
            self._fail(str(error))

    def _read_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.p['global_frame'], self.p['base_frame'], Time())
            age = self._seconds()-self._stamp(transform.header)
            if not -0.5 <= age <= self.p['tf_timeout']:
                return None
            t, q = transform.transform.translation, transform.transform.rotation
            # The base has physical height; navigation uses its ground projection.
            return planar_pose(t.x, t.y, 0.0, q.x, q.y, q.z, q.w)
        except (TransformException, ValueError):
            return None

    def _readiness(self, now):
        if self.paused:
            return 'PAUSED', 'Paused by operator'
        if self.blocked:
            return 'BLOCKED', 'Avoidance module reports no safe motion'
        if (self.p['require_avoidance_heartbeat'] and
                not fresh(self.blocked_received, now, self.p['avoidance_timeout'])):
            return 'WAITING', 'Waiting for avoidance/blocked heartbeat'
        if (not fresh(self.scan_received, now, self.p['sensor_timeout']) or
                self.scan_stamp is None or not -0.5 <= self._seconds()-self.scan_stamp
                <= self.p['sensor_timeout']):
            return 'WAITING', 'Waiting for fresh LaserScan'
        if self.current_pose is None:
            return 'WAITING', 'Waiting for fresh map-to-base TF'
        return None

    def _request_cancel(self, op):
        if op.get('handle') is not None and not op.get('cancel_sent'):
            op['cancel_sent'] = True
            try:
                future = op['handle'].cancel_goal_async()
                future.add_done_callback(lambda f: self._cancel_response(f, op))
            except Exception as error:
                self._fail('Cannot cancel action: ' + str(error))

    def _cancel_response(self, future, op):
        if self.operation is not op:
            return
        try:
            future.result()
        except Exception as error:
            self._fail('Cancellation failed: ' + str(error))
        # A cancellation response is not a terminal result. Keep the operation.

    def _send(self, kind, request):
        op = {'kind': kind, 'generation': self.generation, 'sent': time.monotonic()}
        self.operation = op
        self._set_state('PLANNING' if kind == 'plan' else 'STARTING',
                        'Computing global path' if kind == 'plan' else 'Sending path to controller')
        client = self.planner if kind == 'plan' else self.controller
        try:
            if kind == 'follow':
                future = client.send_goal_async(
                    request, feedback_callback=lambda f: self._feedback(f, op))
            else:
                future = client.send_goal_async(request)
            future.add_done_callback(lambda f: self._accepted(f, op))
        except Exception as error:
            self.operation = None
            self._fail('Action send failed: ' + str(error))

    def _accepted(self, future, op):
        try:
            handle = future.result()
        except Exception as error:
            if self.operation is op:
                self.operation = None
                if op['generation'] == self.generation:
                    self._fail('Action response failed: ' + str(error))
            return
        if not handle.accepted:
            if self.operation is op:
                self.operation = None
                if op['generation'] == self.generation:
                    self._fail(op['kind'] + ' goal was rejected')
            return
        op['handle'] = handle
        handle.get_result_async().add_done_callback(lambda f: self._result(f, op))
        if op['generation'] != self.generation or self.fault:
            self._request_cancel(op)
        elif op['kind'] == 'follow':
            self._zero()
            self._set_state('FOLLOWING', 'Following the planned path')

    def _feedback(self, msg, op):
        if self.operation is op and op['generation'] == self.generation:
            f = msg.feedback
            if math.isfinite(f.distance_to_goal) and math.isfinite(f.speed):
                self.feedback = {'distance_to_goal': float(f.distance_to_goal),
                                 'controller_speed': float(f.speed)}

    def _validate_path(self, path):
        if path.header.frame_id != self.p['global_frame'] or not path.poses:
            raise ValueError('Planner returned an empty path or an unexpected frame')
        for pose in path.poses:
            if pose.header.frame_id not in ('', self.p['global_frame']):
                raise ValueError('Path contains mixed coordinate frames')
            self._pose_tuple(pose.pose)
        xy, yaw = pose_errors(self._pose_tuple(path.poses[-1].pose), self.goal_xyyaw)
        if xy > self.p['xy_tolerance'] or yaw > self.p['yaw_tolerance']:
            raise ValueError('Path does not end at the requested position and heading')

    def _result(self, future, op):
        if self.operation is not op:
            return
        self.operation = None
        self._zero()
        if op['generation'] != self.generation:
            return
        try:
            response = future.result()
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                if op['kind'] == 'follow' and self.replans < self.p['max_replans']:
                    self.replans += 1
                    self._invalidate('WAITING', 'Controller failed; replanning from current pose')
                    return
                raise ValueError(f"{op['kind']} action ended with status {response.status}")
            if op['kind'] == 'plan':
                path = response.result.path
                self._validate_path(path)
                self.pending_path = path
                self.path_pub.publish(path)
                self.wait_started = time.monotonic()
                self._set_state('READY', 'Global path ready')
            else:
                pose = self._read_pose()
                if pose is None:
                    raise ValueError('Cannot verify arrival without a fresh pose')
                xy, yaw = pose_errors(pose, self.goal_xyyaw)
                if xy > self.p['xy_tolerance'] or yaw > self.p['yaw_tolerance']:
                    raise ValueError('Controller claimed success outside goal pose tolerance')
                self.goal = None
                self._set_state('SUCCEEDED', 'Goal position and heading reached')
        except Exception as error:
            self._fail(str(error))

    def _tick(self):
        now, ros_now = time.monotonic(), self._seconds()
        if self.last_ros_time is not None and ros_now < self.last_ros_time-1e-6:
            self._fail('ROS clock moved backwards; send a new goal after reset')
        self.last_ros_time = ros_now
        self.current_pose = self._read_pose()
        wait = self._readiness(now)
        op = self.operation
        if op is not None:
            if op['generation'] != self.generation:
                self._request_cancel(op)
                if (not self.fault and now-op['cancel_started'] > self.p['cancel_timeout']):
                    self._fail('Old action did not terminate after cancellation')
            elif wait is not None:
                self._invalidate(*wait)
            elif now-op['sent'] > self.p[
                    'planning_timeout' if op['kind'] == 'plan' else 'following_timeout']:
                self._fail(op['kind'] + ' action timed out')

        if self.operation is None and self.goal is not None and not self.fault:
            if wait:
                self._set_state(*wait)
                self.wait_started = now
            elif self.pending_path is None:
                if self.planner.server_is_ready():
                    request = ComputePathToPose.Goal()
                    request.goal = deepcopy(self.goal)
                    request.goal.header.stamp = self.get_clock().now().to_msg()
                    request.start = PoseStamped()
                    request.start.header = deepcopy(request.goal.header)
                    x, y, yaw = self.current_pose
                    request.start.pose.position.x, request.start.pose.position.y = x, y
                    request.start.pose.orientation.z = math.sin(yaw/2)
                    request.start.pose.orientation.w = math.cos(yaw/2)
                    request.use_start = True
                    request.planner_id = self.p['planner_id']
                    self._send('plan', request)
                else:
                    self._wait_server(now, 'planner')
            elif self.controller.server_is_ready():
                request = FollowPath.Goal()
                request.path = self.pending_path
                request.controller_id = self.p['controller_id']
                request.goal_checker_id = self.p['goal_checker_id']
                self._send('follow', request)
            else:
                self._wait_server(now, 'controller')

        command = Twist()
        op = self.operation
        if (not self.fault and wait is None and self.state == 'FOLLOWING'
                and op is not None and op['generation'] == self.generation
                and fresh(self.command_received, now, self.p['command_timeout'])):
            command.linear.x, command.angular.z = self.command
        self.cmd_pub.publish(command)

    def _wait_server(self, now, name):
        if now-self.wait_started > self.p['server_timeout']:
            self._fail(name + ' action server unavailable')
        else:
            self._set_state('WAITING', 'Waiting for ' + name + ' action server')

    def _pause(self, request, response):
        self.paused = request.data
        if self.goal is not None and not self.fault:
            self._invalidate('PAUSED' if self.paused else 'WAITING',
                             'Paused' if self.paused else 'Resuming with a fresh plan')
        response.success, response.message = True, 'Paused' if self.paused else 'Resumed'
        return response

    def _cancel(self, request, response):
        self.goal = None
        self._invalidate('CANCELED', 'Goal canceled; stopping old action')
        response.success, response.message = True, 'Stop requested; see rover/status'
        return response

    def _replan(self, request, response):
        if self.goal is None or self.fault:
            response.success, response.message = False, 'No active goal; send a new goal'
        else:
            self._invalidate('WAITING', 'Replan requested; stopping before computing new path')
            response.success, response.message = True, 'Replan requested'
        return response

    def _report(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state, 'reason': self.reason, 'generation': self.generation,
            'goal': self.goal_xyyaw if self.goal is not None else None,
            'position': self.current_pose, 'frame_id': self.p['global_frame'],
            'replans': self.replans, 'blocked': self.blocked, 'paused': self.paused,
            'action_pending': self.operation is not None,
            'last_rejection': self.last_rejection, **self.feedback,
        }, allow_nan=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RoverNavigation()
    def stop(signum, frame):
        node.stop_requested = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.goal = None
        node._invalidate('CANCELED', 'Shutting down')
        node.timer.cancel()
        for _ in range(5):
            if rclpy.ok():
                node._zero()
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
