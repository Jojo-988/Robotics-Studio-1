#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class ParrotMappingMission(Node):

    def __init__(self):
        super().__init__('parrot_mapping_mission')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/parrot1/navigate_to_pose'
        )

        # Example zig-zag survey path.
        # Adjust these coordinates to fit your large_demo world.
        self.waypoints = [
            (2.0, 0.0),
            (6.0, 0.0),
            (6.0, 4.0),
            (2.0, 4.0),
            (2.0, 8.0),
            (6.0, 8.0),
        ]

        self.current_waypoint = 0
        self.started = False

        # Give Nav2 a few seconds to become ready
        self.start_timer = self.create_timer(
            3.0,
            self.start_mission
        )

        self.get_logger().info(
            'Parrot mapping mission ready.'
        )

    def start_mission(self):

        if self.started:
            return

        self.started = True
        self.start_timer.cancel()

        self.get_logger().info(
            'Starting autonomous Parrot survey...'
        )

        self.send_next_waypoint()

    def send_next_waypoint(self):

        if self.current_waypoint >= len(self.waypoints):

            self.get_logger().warn(
                '================================'
            )
            self.get_logger().warn(
                'PARROT SURVEY COMPLETE'
            )
            self.get_logger().warn(
                '================================'
            )

            return

        if not self.nav_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                'Parrot Nav2 action server unavailable.'
            )
            return

        x, y = self.waypoints[self.current_waypoint]

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'parrot1_map'
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # Nav2 is 2D, so leave z = 0
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Going to waypoint '
            f'{self.current_waypoint + 1}/'
            f'{len(self.waypoints)}: '
            f'({x:.2f}, {y:.2f})'
        )

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'Parrot rejected waypoint.'
            )

            return

        self.get_logger().info(
            'Waypoint accepted.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.goal_result_callback
        )

    def feedback_callback(self, feedback_msg):

        try:
            distance = (
                feedback_msg.feedback.distance_remaining
            )

            self.get_logger().info(
                f'Distance remaining: '
                f'{distance:.2f} m',
                throttle_duration_sec=2.0
            )

        except Exception:
            pass

    def goal_result_callback(self, future):

        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().info(
                f'Waypoint '
                f'{self.current_waypoint + 1} reached.'
            )

            self.current_waypoint += 1

            self.send_next_waypoint()

        else:

            self.get_logger().error(
                f'Navigation failed. '
                f'Status = {result.status}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = ParrotMappingMission()

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