#!/usr/bin/env python3
"""Parrot1 ROS 2 Nav2 bridge: PoseStamped goals and pause/resume/cancel services.

Uses /parrot1/navigate_to_pose. Standard Nav2 NavigateToPose is planar;
this node deliberately rejects nonzero altitude goals rather than promising 3D flight.
"""
import json
import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ParrotNavigation(Node):
    def __init__(self):
        super().__init__('parrot_navigation')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('nav_action', '/parrot1/navigate_to_pose')
        self.frame = self.get_parameter('global_frame').value
        action_name = self.get_parameter('nav_action').value

        self.nav_client = ActionClient(self, NavigateToPose, action_name)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/parrot1/goal', self.on_goal, 10)
        self.cancel_service = self.create_service(
            Trigger, '/parrot1/cancel', self.on_cancel)
        self.pause_service = self.create_service(
            Trigger, '/parrot1/pause', self.on_pause)
        self.resume_service = self.create_service(
            Trigger, '/parrot1/resume', self.on_resume)
        self.status_pub = self.create_publisher(String, '/parrot1/status', 10)

        self.lock = threading.RLock()
        self.target = None
        self.goal_handle = None
        self.generation = 0
        self.paused = False
        self.canceling = False
        self.state = 'IDLE'
        self.reason = 'Waiting for goal'
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'Parrot navigation bridge ready; frame={self.frame}, action={action_name}')

    def set_state(self, state, reason):
        self.state, self.reason = state, reason
        self.get_logger().info(f'{state}: {reason}')

    def publish_status(self):
        with self.lock:
            message = {'state': self.state, 'reason': self.reason,
                       'paused': self.paused, 'action_pending': self.goal_handle is not None,
                       'frame_id': self.frame,
                       'goal': None if self.target is None else {
                           'x': self.target.pose.position.x,
                           'y': self.target.pose.position.y,
                           'z': self.target.pose.position.z}}
        self.status_pub.publish(String(data=json.dumps(message)))

    def validate_goal(self, msg):
        if msg.header.frame_id != self.frame:
            raise ValueError(f'Goal frame must be {self.frame}; got {msg.header.frame_id!r}')
        p, q = msg.pose.position, msg.pose.orientation
        vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in vals):
            raise ValueError('Goal contains non-finite coordinates')
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-6:
            raise ValueError('Quaternion cannot be zero')
        if abs(p.z) > 0.05:
            raise ValueError('This Nav2 bridge supports XY/yaw only, not altitude control')
        if math.hypot(q.x, q.y) / norm > 0.01:
            raise ValueError('Nav2 goal must use yaw rotation only')

    def on_goal(self, msg):
        try:
            self.validate_goal(msg)
        except ValueError as exc:
            with self.lock:
                self.set_state('REJECTED', str(exc))
            return
        with self.lock:
            # Do not silently preempt an action; cancel first, then submit another goal.
            if self.goal_handle is not None or self.state in ('SENDING', 'CANCELING'):
                self.set_state('REJECTED', 'Navigation active; cancel before sending a new goal')
                return
            self.target = msg
            self.paused = False
            self.generation += 1
            generation = self.generation
            self.send_goal(generation)

    def send_goal(self, generation):
        if self.target is None:
            return
        if not self.nav_client.server_is_ready():
            self.set_state('WAITING', 'Nav2 action server unavailable; use resume to retry')
            return
        request = NavigateToPose.Goal()
        request.pose = self.target
        request.pose.header.stamp = self.get_clock().now().to_msg()
        self.set_state('SENDING', 'Sending goal to Nav2')
        try:
            future = self.nav_client.send_goal_async(request)
            future.add_done_callback(lambda f: self.goal_response(f, generation))
        except Exception as exc:
            self.set_state('FAILED', f'Cannot send goal: {exc}')

    def goal_response(self, future, generation):
        with self.lock:
            try:
                handle = future.result()
            except Exception as exc:
                if generation == self.generation:
                    self.set_state('FAILED', f'Nav2 goal request failed: {exc}')
                return
            if generation != self.generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.set_state('REJECTED', 'Nav2 rejected the goal')
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(
                lambda f: self.goal_result(f, generation))
            if self.paused or self.canceling:
                self.request_cancel()
            else:
                self.set_state('NAVIGATING', 'Nav2 accepted goal')

    def goal_result(self, future, generation):
        with self.lock:
            if generation != self.generation:
                return
            try:
                status = future.result().status
            except Exception as exc:
                status = None
                error = str(exc)
            self.goal_handle = None
            self.canceling = False
            if self.paused:
                self.set_state('PAUSED', 'Goal stopped; resume to resend destination')
            elif self.target is None:
                self.set_state('CANCELED', 'Navigation canceled')
            elif status == GoalStatus.STATUS_SUCCEEDED:
                self.target = None
                self.set_state('SUCCEEDED', 'Destination reached')
            else:
                self.set_state('FAILED', f'Nav2 finished with status {status}' if status is not None else error)

    def request_cancel(self):
        if self.goal_handle is None:
            return
        self.canceling = True
        self.set_state('CANCELING', 'Requesting Nav2 cancellation')
        try:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_response)
        except Exception as exc:
            self.set_state('FAILED', f'Cancel request failed: {exc}')

    def cancel_response(self, future):
        with self.lock:
            try:
                response = future.result()
                if not response.goals_canceling:
                    self.set_state('FAILED', 'Nav2 did not accept cancellation')
            except Exception as exc:
                self.set_state('FAILED', f'Cancel response failed: {exc}')

    def on_pause(self, request, response):
        with self.lock:
            if self.target is None:
                response.success, response.message = False, 'No goal to pause'
            else:
                self.paused = True
                if self.goal_handle is not None:
                    self.request_cancel()
                else:
                    self.set_state('PAUSED', 'Paused before goal acceptance')
                response.success, response.message = True, 'Pause requested'
        return response

    def on_resume(self, request, response):
        with self.lock:
            if self.target is None:
                response.success, response.message = False, 'No saved destination'
            elif self.goal_handle is not None or self.canceling or self.state == 'SENDING':
                response.success, response.message = False, 'Wait for previous action to finish'
            else:
                self.paused = False
                self.generation += 1
                self.send_goal(self.generation)
                response.success, response.message = True, 'Goal resubmission requested'
        return response

    def on_cancel(self, request, response):
        with self.lock:
            self.target = None
            self.paused = False
            if self.goal_handle is not None:
                self.request_cancel()
            elif self.state == 'SENDING':
                self.canceling = True
                self.set_state('CANCELING', 'Will cancel when goal is accepted')
            else:
                self.set_state('CANCELED', 'No active goal')
            response.success, response.message = True, 'Cancel requested'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ParrotNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
