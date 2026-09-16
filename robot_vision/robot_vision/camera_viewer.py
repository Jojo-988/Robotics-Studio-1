import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")

        # implent parrot here
        self.declare_parameter("image_topic", "/husky1/camera/image")
        topic = self.get_parameter("image_topic").value

        self.bridge = CvBridge()
        self.received_first_frame = False

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"Waiting for images on: {topic}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except CvBridgeError as error:
            self.get_logger().error(
                f"Image conversion failed: {error}",
                throttle_duration_sec=5.0,
            )
            return

        if not self.received_first_frame:
            self.get_logger().info(
                f"First image received: {msg.width} x {msg.height}"
            )
            self.received_first_frame = True

        # vege or fire detection starts from here
        cv2.imshow("Robot Vision - RGB Camera", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()