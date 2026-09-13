#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class VegetationDetector(Node):

    def __init__(self):
        super().__init__('vegetation_detector')

        # Convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Subscribe to Parrot RGB camera
        self.subscription = self.create_subscription(
            Image,
            '/parrot1/camera/image',
            self.image_callback,
            10
        )

        self.get_logger().info(
            'Vegetation Detector started - listening to /parrot1/camera/image'
        )

    def image_callback(self, msg):

        # ---------------------------------------------------------
        # 1. ROS IMAGE -> OPENCV
        # ---------------------------------------------------------
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

        # ---------------------------------------------------------
        # 2. BLUR IMAGE
        # Helps reduce tiny texture/noise detections
        # ---------------------------------------------------------
        blurred = cv2.GaussianBlur(
            frame,
            (7, 7),
            0
        )

        # ---------------------------------------------------------
        # 3. BGR -> HSV
        # HSV is easier for vegetation colour classification
        # ---------------------------------------------------------
        hsv = cv2.cvtColor(
            blurred,
            cv2.COLOR_BGR2HSV
        )

        # ---------------------------------------------------------
        # 4. COLOUR RANGES
        #
        # These are STARTING VALUES.
        # We will tune them for your Gazebo tree textures.
        # ---------------------------------------------------------

        # HEALTHY
        # Dark / strong green
        healthy_lower = np.array([30, 70, 25])
        healthy_upper = np.array([95, 255, 210])

        # STRESSED
        # Pale / desaturated green-grey
        stressed_lower = np.array([25, 15, 60])
        stressed_upper = np.array([100, 110, 230])

        # DEAD
        # Very low saturation + relatively bright
        dead_lower = np.array([0, 0, 100])
        dead_upper = np.array([179, 45, 255])

        # ---------------------------------------------------------
        # 5. CREATE MASKS
        # ---------------------------------------------------------

        healthy_mask = cv2.inRange(
            hsv,
            healthy_lower,
            healthy_upper
        )

        stressed_mask = cv2.inRange(
            hsv,
            stressed_lower,
            stressed_upper
        )

        dead_mask = cv2.inRange(
            hsv,
            dead_lower,
            dead_upper
        )

        # ---------------------------------------------------------
        # 6. CLEAN MASKS
        #
        # Removes tiny isolated pixels.
        # ---------------------------------------------------------

        kernel = np.ones((5, 5), np.uint8)

        healthy_mask = cv2.morphologyEx(
            healthy_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        healthy_mask = cv2.morphologyEx(
            healthy_mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        stressed_mask = cv2.morphologyEx(
            stressed_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        stressed_mask = cv2.morphologyEx(
            stressed_mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        dead_mask = cv2.morphologyEx(
            dead_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        dead_mask = cv2.morphologyEx(
            dead_mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # ---------------------------------------------------------
        # 7. DETECT VEGETATION REGIONS
        # ---------------------------------------------------------

        healthy_count = self.detect_regions(
            frame,
            healthy_mask,
            'HEALTHY',
            (0, 255, 0)
        )

        stressed_count = self.detect_regions(
            frame,
            stressed_mask,
            'STRESSED',
            (0, 255, 255)
        )

        dead_count = self.detect_regions(
            frame,
            dead_mask,
            'DEAD',
            (0, 0, 255)
        )

        # ---------------------------------------------------------
        # 8. CURRENT FRAME RESULTS
        # ---------------------------------------------------------

        total = (
            healthy_count
            + stressed_count
            + dead_count
        )

        # Background panel
        cv2.rectangle(
            frame,
            (10, 10),
            (270, 145),
            (0, 0, 0),
            -1
        )

        cv2.putText(
            frame,
            f'Total Vegetation: {total}',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f'Healthy: {healthy_count}',
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f'Stressed: {stressed_count}',
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f'Dead: {dead_count}',
            (20, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2
        )

        # ---------------------------------------------------------
        # 9. DISPLAY RESULT
        # ---------------------------------------------------------

        cv2.imshow(
            'Parrot Vegetation Detector',
            frame
        )

        cv2.waitKey(1)

    # =============================================================
    # DETECT REGIONS
    # =============================================================

    def detect_regions(
        self,
        frame,
        mask,
        label,
        colour
    ):

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        count = 0

        for contour in contours:

            area = cv2.contourArea(contour)

            # Ignore small colour patches
            if area < 800:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            # Ignore very small bounding boxes
            if w < 20 or h < 30:
                continue

            count += 1

            # Draw bounding box
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                colour,
                2
            )

            # Draw classification
            cv2.putText(
                frame,
                label,
                (x, max(y - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                colour,
                2
            )

        return count


def main(args=None):

    rclpy.init(args=args)

    node = VegetationDetector()

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