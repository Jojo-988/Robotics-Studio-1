
import json
import math
import signal
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from patrol.mission import FlightConfig, PatrolMission
from patrol.planner import CoverageConfig


class CoveragePatrol(Node):
    def __init__(self):
        super().__init__('coverage_patrol')
        for defaults in (CoverageConfig(), FlightConfig()):
            for name, value in vars(defaults).items():
                self.declare_parameter(name, value)
        for name, value in {
            'odom_topic': 'odometry', 'cmd_vel_topic': 'cmd_vel',
            'odom_frame': 'parrot1_odom', 'base_frame': 'parrot1_base_link',
            'mission_frame': 'world', 'odom_timeout': 1.0, 'control_rate': 20.0,
            'max_tilt': 0.2,
        }.items():
            self.declare_parameter(name, value)
        self.value = lambda name: self.get_parameter(name).value
        self.mission = PatrolMission(
            CoverageConfig(**{k: self.value(k) for k in vars(CoverageConfig())}),
            FlightConfig(**{k: self.value(k) for k in vars(FlightConfig())}),
        )
        for name in ('odom_timeout', 'control_rate', 'max_tilt'):
            if not math.isfinite(self.value(name)) or self.value(name) <= 0:
                raise ValueError(f'{name} must be finite and positive.')
        self.pose = None
        self.last_received = None
        self.odom_stamp = None
        self.last_sim_time = None
        self.last_status = ''
        self.wait_reason = 'Waiting for 3D Gazebo odometry'
        self.stop_requested = False
        self.route_key = None
        self.trace = Path()
        self.trace.header.frame_id = self.value('mission_frame')
        self.cmd_pub = self.create_publisher(Twist, self.value('cmd_vel_topic'), 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.plan_pub = self.create_publisher(Path, 'patrol/planned_path', latched)
        self.trace_pub = self.create_publisher(Path, 'patrol/flown_path', latched)
        self.status_pub = self.create_publisher(String, 'patrol/status', latched)
        self.odom_sub = self.create_subscription(
            Odometry, self.value('odom_topic'), self._odom, qos_profile_sensor_data)
        self.pause_service = self.create_service(SetBool, 'patrol/pause', self._pause)
        self.stop_service = self.create_service(Trigger, 'patrol/stop', self._stop)
        # Wall timer also sends zero velocity when /clock or odometry stops.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.control_timer = self.create_timer(
            1.0 / self.value('control_rate'), self._tick, clock=self.wall_clock)
        self.report_timer = self.create_timer(1.0, self._report, clock=self.wall_clock)
        self.get_logger().info('Coverage patrol ready. This node must be the only cmd_vel controller.')

    def _seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    # Ensure that the current recorded pose is real-time and valid.
    def _odom(self, msg):
        if (msg.header.frame_id != self.value('odom_frame')
                or msg.child_frame_id != self.value('base_frame')):
            self.mission.stop('Unexpected odometry frame; use raw Gazebo odometry', 'ERROR')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = math.sqrt(sum(v * v for v in (q.x, q.y, q.z, q.w)))
        if not all(math.isfinite(v) for v in values) or norm < 1e-6:
            self.mission.stop('Invalid odometry pose', 'ERROR')
            return
        qx, qy, qz, qw = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        if max(abs(roll), abs(pitch)) > self.value('max_tilt'):
            self.mission.stop('Tilt exceeds simplified level-flight controller limit', 'ERROR')
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.odom_stamp is not None and stamp < self.odom_stamp:
            self.mission.stop('Simulation time reset; restart patrol', 'ERROR')
        # Repeated/stuck timestamps do not keep the watchdog alive.
        if stamp == self.odom_stamp:
            return
        self.odom_stamp = stamp
        self.pose = (p.x, p.y, p.z, math.atan2(2 * (qw * qz + qx * qy),
                                           1 - 2 * (qy * qy + qz * qz)))
        self.last_received = time.monotonic()

    def _publish_velocity(self, command=(0.0, 0.0, 0.0, 0.0)):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z = command
        self.cmd_pub.publish(msg)

    # Timely verify whether the updated data has expired
    def _tick(self):
        now = self._seconds()
        if self.last_sim_time is not None and now < self.last_sim_time:
            self.mission.stop('Simulation clock reset; restart patrol', 'ERROR')
        self.last_sim_time = now
        stale = (self.last_received is None
                 or time.monotonic() - self.last_received > self.value('odom_timeout')
                 or self.odom_stamp is None
                 or now - self.odom_stamp > self.value('odom_timeout')
                 or self.odom_stamp - now > self.value('odom_timeout'))
        if stale:
            self.wait_reason = 'Waiting for fresh 3D Gazebo odometry'
            self.mission.break_trace()
            self._publish_velocity()
            return
        self.wait_reason = ''
        self._publish_velocity(self.mission.step(self.pose, now))
        self._update_paths()

    def _pose_message(self, x, y, z, yaw=0.0):
        msg = PoseStamped()
        msg.header.frame_id = self.value('mission_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return msg

    def _update_paths(self):
        key = (len(self.mission.route), self.mission.completed_passes)
        if key != self.route_key:
            path = Path()
            path.header.frame_id = self.value('mission_frame')
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = [self._pose_message(x, y, self.mission.flight.altitude)
                          for x, y in self.mission.route]
            self.plan_pub.publish(path)
            self.route_key = key
        x, y, z, yaw = self.pose
        last = self.trace.poses[-1].pose.position if self.trace.poses else None
        if last is None or math.dist((x, y, z), (last.x, last.y, last.z)) >= 0.25:
            self.trace.poses.append(self._pose_message(x, y, z, yaw))
            # Bound memory for continuous missions. Coverage retains its own grid.
            self.trace.poses = self.trace.poses[-10000:]

    def _report(self):
        status = self.mission.status()
        status['odometry_ready'] = not bool(self.wait_reason)
        status['waiting_reason'] = self.wait_reason
        status['position'] = self.pose
        status['frame_id'] = self.value('mission_frame')
        msg = String()
        msg.data = json.dumps(status, allow_nan=False)
        self.status_pub.publish(msg)
        self.trace.header.stamp = self.get_clock().now().to_msg()
        self.trace_pub.publish(self.trace)
        summary = (f"{status['state']} | pass {status['completed_passes']} | "
                   f"waypoint {status['waypoint_index']}/{status['waypoints_total']} | "
                   f"coverage {status['coverage_fraction']:.1%} | "
                   f"{status['reason'] or self.wait_reason}")
        if summary != self.last_status:
            self.get_logger().info(summary)
            self.last_status = summary

    # Pause, stop, and status output are reserved for future manual control and GUI interface.
    def _pause(self, request, response):
        if self.mission.state in ('ERROR', 'STOPPED', 'COMPLETE'):
            response.success, response.message = False, 'Mission ended; restart node for a new mission.'
            return response
        self.mission.pause(request.data, self._seconds())
        self._publish_velocity()
        response.success = True
        response.message = 'Paused' if request.data else 'Resumed'
        return response

    def _stop(self, request, response):
        self.mission.stop()
        self._publish_velocity()
        response.success, response.message = True, 'Stopped; restart node for a new mission.'
        return response


def main(args=None):
    # Keep ROS alive briefly on SIGINT/SIGTERM to deliver zero cmd_vel to Gazebo.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = CoveragePatrol()

        def request_shutdown(signum, frame):
            node.stop_requested = True

        signal.signal(signal.SIGINT, request_shutdown)
        signal.signal(signal.SIGTERM, request_shutdown)
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.control_timer.cancel()
            for _ in range(3):
                if rclpy.ok():
                    node._publish_velocity()
                    rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
