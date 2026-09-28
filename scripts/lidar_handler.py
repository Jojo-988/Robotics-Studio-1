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
             f'Voxel Size: {self.voxel_size}'
        )
        self.get_logger().info(
            '======================================'
        )

    def callback(self, msg):

        # === Step 1: convert raw lidar readings to numpy array of points ===
        raw_points = point_cloud2.read_points(msg, field_names=['x','y','z'],skip_nans=True) # note: can be list or tuple
        raw_points_list = list(raw_points)

        if not raw_points_list:
            self.get_logger().warning(
                    "Point Cloud List is empty"
                )
            return
        
        raw_points_numpy = np.array(raw_points_list, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        raw_points_numpy = np.column_stack((raw_points_numpy['x'], raw_points_numpy['y'], raw_points_numpy['z']))

        # remove any points that are infinite
        finite_mask = np.isfinite(raw_points_numpy).all(axis=1)
        raw_points_numpy = raw_points_numpy[finite_mask]

        if raw_points_numpy.shape[0] < 2:
            self.get_logger().warning(
                "Point Cloud is empty"
            )
            return

        self.get_logger().info(
            f"Points shape: {raw_points_numpy.shape}"
            )
        self.get_logger().info(
            f"Min: {np.min(raw_points_numpy, axis=0)}, Max: {np.max(raw_points_numpy, axis=0)}"
            )

        # === Step 2: Crop Bounds ===
        # (crop bounds if having issues with points going out of the environment)

        # === Step 3: Voxel Downsampling ===
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(raw_points_numpy)

        downsampled_pcd = point_cloud.voxel_down_sample(self.voxel_size)
        processed_points = np.asarray(downsampled_pcd.points)
        self.get_logger().info(
            f"Original points: {raw_points_numpy.shape[0]} -> Downsampled points: {processed_points.shape[0]}"
            )
        print("\n")

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