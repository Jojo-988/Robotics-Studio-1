import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import open3d as o3d

class Lidar_Handler(Node):

    def __init__(self):
        super().__init__('lidar_handler')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # create subscription to lidar scanner
        self.point_cloud_sub = self.create_subscription(PointCloud2, 'parrot1/scan/points', self.callback, qos_profile=qos_profile)

        # create publisher of 3d map
        self.map_pub = self.create_publisher(PointCloud2, '/parrot1/processed_point_cloud', qos_profile=qos_profile)

        self.voxel_size = 0.05

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
            'Output: /parrot1/processed_point_cloud'
        )
        self.get_logger().info(
            '======================================'
        )

    def callback(self, msg):

        # === Step 1: convert raw lidar readings to numpy array of points ===
        raw_points = point_cloud2.read_points_numpy(msg, field_names=['x','y','z'],skip_nans=True) # note: idk if field names is string or tuple, need to test

        # === Step 2: Crop Bounds ===
        # (crop bounds if having issues with points going out of the environment)

        # === Step 3: Voxel Downsampling ===
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(raw_points)

        downsampled_pcd = point_cloud.voxel_down_sample(self.voxel_size)
        processed_points = np.asarray(downsampled_pcd.points)

        # === Step 4: Convert back to ROS2 Point Cloud and publish ===
        output_msg = point_cloud2.create_cloud_xyz32(header=msg.header, points=processed_points)
        self.publish_point_cloud(output_msg)

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