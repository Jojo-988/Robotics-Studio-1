#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped

import cv2
import numpy as np


class FireDetector(Node):

    def __init__(self):
        super().__init__('fire_detector')

        self.bridge = CvBridge()

        # ---------------------------------------------------------
        # FIRE LOCATION
        # First test: known fire position in the Gazebo world
        # Fake-Simulation: fire is at (-9.68, -7.70) in the Gazebo world
        # Works when Aerial detects fire 
        # ---------------------------------------------------------

        self.fire_x = -9.68
        self.fire_y = -7.70

        # Prevent continuously sending the same navigation goal
        self.fire_detected = False
        self.goal_sent = False

        # Minimum fire area in camera image
        self.min_fire_area = 300

        # ---------------------------------------------------------
        # PARROT CAMERA
        # ---------------------------------------------------------

        self.camera_sub = self.create_subscription(
            Image,
            '/parrot1/camera/image',
            self.image_callback,
            10
        )

        # ---------------------------------------------------------
        # NAV2 ACTION CLIENT
        # ---------------------------------------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/husky1/navigate_to_pose'
        )

        self.get_logger().info(
            'Fire detector started.'
        )

        self.get_logger().info(
            'Waiting for fire detection from Parrot camera...'
        )

    # =============================================================
    # CAMERA CALLBACK
    # =============================================================

    def image_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'Image conversion failed: {e}'
            )

            return

        # Slight blur
        blurred = cv2.GaussianBlur(
            frame,
            (5, 5),
            0
        )

        # BGR -> HSV
        hsv = cv2.cvtColor(
            blurred,
            cv2.COLOR_BGR2HSV
        )

        # =========================================================
        # FIRE COLOUR DETECTION
        #
        # Detect orange/red colours
        # =========================================================

        # RED RANGE 1
        lower_red1 = np.array([0, 150, 150])
        upper_red1 = np.array([10, 255, 255])

        # RED RANGE 2
        lower_red2 = np.array([170, 150, 150])
        upper_red2 = np.array([179, 255, 255])

        # ORANGE/YELLOW
        lower_orange = np.array([10, 120, 150])
        upper_orange = np.array([35, 255, 255])

        red_mask1 = cv2.inRange(
            hsv,
            lower_red1,
            upper_red1
        )

        red_mask2 = cv2.inRange(
            hsv,
            lower_red2,
            upper_red2
        )

        orange_mask = cv2.inRange(
            hsv,
            lower_orange,
            upper_orange
        )

        # Combine fire colours
        fire_mask = (
            red_mask1 |
            red_mask2 |
            orange_mask
        )

        # =========================================================
        # REMOVE SMALL NOISE
        # =========================================================

        kernel = np.ones(
            (5, 5),
            np.uint8
        )

        fire_mask = cv2.morphologyEx(
            fire_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        fire_mask = cv2.morphologyEx(
            fire_mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # =========================================================
        # FIND FIRE REGIONS
        # =========================================================

        contours, _ = cv2.findContours(
            fire_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        largest_fire = None
        largest_area = 0

        for contour in contours:

            area = cv2.contourArea(contour)

            if area > largest_area:

                largest_area = area
                largest_fire = contour

        # =========================================================
        # FIRE FOUND
        # =========================================================

        if (
            largest_fire is not None
            and largest_area > self.min_fire_area
        ):

            x, y, w, h = cv2.boundingRect(
                largest_fire
            )

            # Fire centre pixel
            fire_u = x + w // 2
            fire_v = y + h // 2

            # Bounding box
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 0, 255),
                3
            )

            cv2.circle(
                frame,
                (fire_u, fire_v),
                5,
                (255, 255, 255),
                -1
            )

            cv2.putText(
                frame,
                'FIRE DETECTED',
                (x, max(y - 10, 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            cv2.putText(
                frame,
                f'Area: {int(largest_area)}',
                (x, y + h + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2
            )

            # Only trigger navigation once
            if not self.fire_detected:

                self.fire_detected = True

                self.get_logger().warn(
                    'FIRE DETECTED!'
                )

                self.get_logger().info(
                    f'Fire image pixel: '
                    f'({fire_u}, {fire_v})'
                )

                self.get_logger().info(
                    f'Fire location: '
                    f'({self.fire_x:.2f}, '
                    f'{self.fire_y:.2f})'
                )

                self.send_husky_to_fire()

        # =========================================================
        # DISPLAY CAMERA
        # =========================================================

        cv2.imshow(
            'Parrot Fire Detection',
            frame
        )

        cv2.imshow(
            'Fire Mask',
            fire_mask
        )

        cv2.waitKey(1)

    # =============================================================
    # SEND HUSKY
    # =============================================================

    def send_husky_to_fire(self):

        if self.goal_sent:
            return

        self.get_logger().info(
            'Waiting for Husky Nav2...'
        )

        if not self.nav_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                'NavigateToPose action server '
                'is not available.'
            )

            # Allow retry
            self.fire_detected = False

            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()

        # IMPORTANT:
        # This assumes your navigation goals use "map".
        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.pose.pose.position.x = (
            self.fire_x
        )

        goal_msg.pose.pose.position.y = (
            self.fire_y
        )

        goal_msg.pose.pose.position.z = 0.0

        # No rotation
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            'Sending Husky to fire location...'
        )

        self.goal_sent = True

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # =============================================================
    # GOAL RESPONSE
    # =============================================================

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'Husky navigation goal rejected.'
            )

            self.goal_sent = False
            return

        self.get_logger().info(
            'Husky navigation goal accepted.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.navigation_result_callback
        )

    # =============================================================
    # NAVIGATION FEEDBACK
    # =============================================================

    def navigation_feedback(self, feedback_msg):

        feedback = feedback_msg.feedback

        try:

            distance = (
                feedback.distance_remaining
            )

            self.get_logger().info(
                f'Distance to fire: '
                f'{distance:.2f} m',
                throttle_duration_sec=2.0
            )

        except Exception:
            pass

    # =============================================================
    # NAVIGATION RESULT
    # =============================================================

    def navigation_result_callback(self, future):

        result = future.result()

        self.get_logger().info(
            'Husky navigation finished.'
        )

        self.get_logger().info(
            f'Husky arrived near fire location '
            f'({self.fire_x:.2f}, '
            f'{self.fire_y:.2f})'
        )


def main(args=None):

    rclpy.init(args=args)

    node = FireDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        cv2.destroyAllWindows()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()