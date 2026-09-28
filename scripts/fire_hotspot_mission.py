#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

import tf2_ros

from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String


class FireHotspotMission(Node):

    def __init__(self):
        super().__init__('fire_hotspot_mission')

        # =====================================================
        # FIRE HOTSPOT SETTINGS
        # =====================================================

        # Temporary simulated fire location.
        # Later this can be replaced by thermal-camera detection.
        self.fire_x = 4.0
        self.fire_y = 6.0

        # Fire is considered detected when Parrot is within
        # this distance from the hotspot.
        self.detection_radius = 1.5  # meters

        # Mission states
        self.fire_detected = False
        self.husky_goal_active = False
        self.mission_complete = False
        self.parrot_goal_sent = False

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =====================================================
        # FIRE HOTSPOT TOPIC
        # Parrot -> Husky
        # =====================================================

        self.hotspot_pub = self.create_publisher(
            PointStamped,
            '/fire_hotspot',
            10
        )

        self.hotspot_sub = self.create_subscription(
            PointStamped,
            '/fire_hotspot',
            self.hotspot_callback,
            10
        )

        # =====================================================
        # STATUS TOPIC
        # Can later be connected to your GUI
        # =====================================================

        self.status_pub = self.create_publisher(
            String,
            '/fire_mission_status',
            10
        )

        # =====================================================
        # CONFIRMATION TOPIC
        # =====================================================

        self.confirm_pub = self.create_publisher(
            Bool,
            '/fire_hotspot_confirmed',
            10
        )

        # =====================================================
        # PARROT NAV2
        # =====================================================

        self.parrot_nav_client = ActionClient(
            self,
            NavigateToPose,
            '/parrot1/navigate_to_pose'
        )

        # =====================================================
        # HUSKY NAV2
        # =====================================================

        self.husky_nav_client = ActionClient(
            self,
            NavigateToPose,
            '/husky1/navigate_to_pose'
        )

        # =====================================================
        # FIRE DETECTION TIMER
        # =====================================================

        # Check Parrot position twice per second.
        self.fire_check_timer = self.create_timer(
            0.5,
            self.check_parrot_for_fire
        )

        # =====================================================
        # START MISSION TIMER
        # =====================================================

        # Give Gazebo/Nav2 a few seconds before sending
        # the Parrot navigation goal.
        self.start_timer = self.create_timer(
            3.0,
            self.start_parrot_mission
        )

        # =====================================================
        # STARTUP MESSAGE
        # =====================================================

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'FIRE HOTSPOT MISSION STARTED'
        )

        self.get_logger().info(
            f'Fire hotspot: '
            f'({self.fire_x:.2f}, {self.fire_y:.2f})'
        )

        self.get_logger().info(
            'Waiting 3 seconds before starting Parrot...'
        )

        self.get_logger().info(
            '=========================================='
        )

    # =========================================================
    # STATUS MESSAGE
    # =========================================================

    def publish_status(self, text):

        msg = String()
        msg.data = text

        self.status_pub.publish(msg)

        self.get_logger().warn(text)

    # =========================================================
    # START PARROT MISSION
    # =========================================================

    def start_parrot_mission(self):

        # Prevent timer from repeatedly starting mission.
        if self.parrot_goal_sent:
            return

        self.parrot_goal_sent = True

        # Timer only needs to run once.
        self.start_timer.cancel()

        self.publish_status(
            'Starting Parrot fire survey mission...'
        )

        # -----------------------------------------------------
        # Check Parrot Nav2
        # -----------------------------------------------------

        if not self.parrot_nav_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                'Parrot Nav2 action server unavailable.'
            )

            self.parrot_goal_sent = False
            return

        # -----------------------------------------------------
        # Create Parrot goal
        # -----------------------------------------------------

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = 'parrot1_map'

        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # -----------------------------------------------------
        # Send Parrot toward fire area
        # -----------------------------------------------------

        goal.pose.pose.position.x = self.fire_x
        goal.pose.pose.position.y = self.fire_y
        goal.pose.pose.position.z = 7.0  # Fly at 7 meters altitude

        # Valid neutral quaternion
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.publish_status(
            f'Parrot going to survey coordinates '
            f'({self.fire_x:.2f}, {self.fire_y:.2f}, {goal.pose.pose.position.z:.2f})'
        )

        # -----------------------------------------------------
        # Send goal
        # -----------------------------------------------------

        future = self.parrot_nav_client.send_goal_async(
            goal,
            feedback_callback=self.parrot_feedback
        )

        future.add_done_callback(
            self.parrot_goal_response
        )

    # =========================================================
    # PARROT GOAL RESPONSE
    # =========================================================

    def parrot_goal_response(self, future):

        try:

            goal_handle = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Failed to send Parrot goal: {error}'
            )

            return

        if not goal_handle.accepted:

            self.get_logger().error(
                'Parrot rejected navigation goal.'
            )

            return

        self.get_logger().info(
            'Parrot accepted navigation goal.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.parrot_goal_result
        )

    # =========================================================
    # PARROT NAVIGATION FEEDBACK
    # =========================================================

    def parrot_feedback(self, feedback_msg):

        try:

            distance = (
                feedback_msg.feedback.distance_remaining
            )

            self.get_logger().info(
                f'Parrot distance remaining: '
                f'{distance:.2f} m',
                throttle_duration_sec=2.0
            )

        except Exception:
            pass

    # =========================================================
    # PARROT GOAL RESULT
    # =========================================================

    def parrot_goal_result(self, future):

        try:

            result = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Parrot navigation error: {error}'
            )

            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().info(
                'Parrot reached survey destination.'
            )

        else:

            self.get_logger().error(
                f'Parrot navigation failed. '
                f'Status = {result.status}'
            )

    # =========================================================
    # GET PARROT POSITION
    # =========================================================

    def get_parrot_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                'parrot1_map',
                'parrot1_base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return x, y

        except Exception as error:

            self.get_logger().debug(
                f'Waiting for Parrot TF: {error}'
            )

            return None

    # =========================================================
    # CHECK FOR FIRE
    # =========================================================

    def check_parrot_for_fire(self):

        # Fire already found.
        if self.fire_detected:
            return

        pose = self.get_parrot_pose()

        if pose is None:
            return

        parrot_x, parrot_y = pose

        # -----------------------------------------------------
        # Calculate distance between Parrot and fire
        # -----------------------------------------------------

        distance = math.hypot(
            parrot_x - self.fire_x,
            parrot_y - self.fire_y
        )

        self.get_logger().info(
            f'Parrot: '
            f'({parrot_x:.2f}, {parrot_y:.2f}) '
            f'| Fire distance: {distance:.2f} m',
            throttle_duration_sec=2.0
        )

        # -----------------------------------------------------
        # FIRE DETECTED
        # -----------------------------------------------------

        if distance <= self.detection_radius:

            self.fire_detected = True

            self.get_logger().warn(
                '=========================================='
            )

            self.publish_status(
                'FIRE DETECTED! '
                'Send Husky for confirmation.'
            )

            self.get_logger().warn(
                '=========================================='
            )

            # -------------------------------------------------
            # Create fire hotspot coordinate
            # -------------------------------------------------

            hotspot = PointStamped()

            hotspot.header.stamp = (
                self.get_clock().now().to_msg()
            )

            hotspot.header.frame_id = 'map'

            hotspot.point.x = self.fire_x
            hotspot.point.y = self.fire_y
            hotspot.point.z = 0.0

            # -------------------------------------------------
            # Publish hotspot
            # -------------------------------------------------

            self.hotspot_pub.publish(hotspot)

            self.get_logger().warn(
                f'Fire coordinates sent: '
                f'({self.fire_x:.2f}, '
                f'{self.fire_y:.2f})'
            )

    # =========================================================
    # HUSKY RECEIVES HOTSPOT
    # =========================================================

    def hotspot_callback(self, msg):

        # Don't send another goal while Husky is moving.
        if self.husky_goal_active:
            return

        if self.mission_complete:
            return

        x = msg.point.x
        y = msg.point.y

        self.publish_status(
            f'Husky going to coordinates '
            f'({x:.2f}, {y:.2f})'
        )

        self.send_husky_goal(
            x,
            y
        )

    # =========================================================
    # SEND HUSKY GOAL
    # =========================================================

    def send_husky_goal(self, x, y):

        self.get_logger().info(
            'Waiting for Husky Nav2...'
        )

        # -----------------------------------------------------
        # Check Husky Nav2
        # -----------------------------------------------------

        if not self.husky_nav_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.publish_status(
                'ERROR: Husky Nav2 action server unavailable.'
            )

            return

        # -----------------------------------------------------
        # Create Nav2 goal
        # -----------------------------------------------------

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = 'husky1_map'

        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.husky_goal_active = True

        self.get_logger().info(
            'Sending fire location to Husky Nav2...'
        )

        # -----------------------------------------------------
        # Send goal
        # -----------------------------------------------------

        future = self.husky_nav_client.send_goal_async(
            goal,
            feedback_callback=self.husky_feedback
        )

        future.add_done_callback(
            self.husky_goal_response
        )

    # =========================================================
    # HUSKY GOAL RESPONSE
    # =========================================================

    def husky_goal_response(self, future):

        try:

            goal_handle = future.result()

        except Exception as error:

            self.husky_goal_active = False

            self.publish_status(
                f'Failed to send Husky goal: {error}'
            )

            return

        if not goal_handle.accepted:

            self.husky_goal_active = False

            self.publish_status(
                'Husky rejected fire hotspot goal.'
            )

            return

        self.get_logger().info(
            'Husky accepted fire hotspot goal.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.husky_goal_result
        )

    # =========================================================
    # HUSKY NAVIGATION FEEDBACK
    # =========================================================

    def husky_feedback(self, feedback_msg):

        try:

            distance = (
                feedback_msg.feedback.distance_remaining
            )

            self.get_logger().info(
                f'Husky distance remaining: '
                f'{distance:.2f} m',
                throttle_duration_sec=2.0
            )

        except Exception:
            pass

    # =========================================================
    # HUSKY GOAL RESULT
    # =========================================================

    def husky_goal_result(self, future):

        try:

            result = future.result()

        except Exception as error:

            self.husky_goal_active = False

            self.publish_status(
                f'Husky navigation error: {error}'
            )

            return

        self.husky_goal_active = False

        # -----------------------------------------------------
        # SUCCESS
        # -----------------------------------------------------

        if result.status == GoalStatus.STATUS_SUCCEEDED:

            self.mission_complete = True

            self.get_logger().warn(
                '=========================================='
            )

            self.publish_status(
                'FIRE HOTSPOT CONFIRMED! '
                'Husky reached the fire location.'
            )

            self.get_logger().warn(
                '=========================================='
            )

            # Publish confirmation
            confirmation = Bool()
            confirmation.data = True

            self.confirm_pub.publish(
                confirmation
            )

        # -----------------------------------------------------
        # FAILED
        # -----------------------------------------------------

        else:

            self.publish_status(
                f'Husky failed to reach hotspot. '
                f'Nav2 status = {result.status}'
            )


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = FireHotspotMission()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Fire hotspot mission stopped.'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()