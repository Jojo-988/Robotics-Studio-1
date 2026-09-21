import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class Lidar_Handler(Node):

    def __init__(self):
        super().__init__('lidar_handler')
        
        # create subscription to lidar scanner
        self.point_cloud_sub = self.create_subscription(PointCloud2, 'parrot1/scan/points', self.callback, qos_profile=10)

        # create publisher of 3d map
        self.map_pub = self.create_publisher(PointCloud2, '/parrot1/lidar_map_3d', qos_profile=10)

        # log initialisation
        self.get_logger().info(
            '======================================'
        )
        self.get_logger().info(
            'PARROT LIDAR 3D MAPPING STARTED'
        )
        self.get_logger().info(
            'Input: /parrot1/scan/points'
        )
        self.get_logger().info(
            'Output: /parrot1/lidar_map_3d'
        )
        # self.get_logger().info(
        #     f'Fixed frame: {self.map_frame}'
        # )
        self.get_logger().info(
            '======================================'
        )

    def callback(self, msg):
        self.publish_point_cloud(msg)

    def publish_point_cloud(self, msg):
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = Lidar_Handler()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'LiDAR 3D mapping stopped.'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()