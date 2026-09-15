#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ParrotLidar3DMapping(Node):

    def __init__(self):
        super().__init__('parrot_lidar_3d_mapping')

        # Fixed map frame
        self.map_frame = 'parrot1_map'

        # Voxel size for downsampling
        self.voxel_size = 0.10

        # Limit map size
        self.max_points = 500000

        # Store accumulated point cloud
        self.voxels = {}

        self.cloud_counter = 0

        # TF
        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # Subscribe to LiDAR PointCloud2
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/parrot1/scan/points',
            self.cloud_callback,
            10
        )

        # Publish accumulated 3D map
        self.map_pub = self.create_publisher(
            PointCloud2,
            '/parrot1/lidar_map_3d',
            10
        )

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
        self.get_logger().info(
            f'Fixed frame: {self.map_frame}'
        )
        self.get_logger().info(
            '======================================'
        )

    def rotate_point(
        self,
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw
    ):

        # Quaternion -> rotation matrix

        r00 = 1 - 2 * (qy*qy + qz*qz)
        r01 = 2 * (qx*qy - qz*qw)
        r02 = 2 * (qx*qz + qy*qw)

        r10 = 2 * (qx*qy + qz*qw)
        r11 = 1 - 2 * (qx*qx + qz*qz)
        r12 = 2 * (qy*qz - qx*qw)

        r20 = 2 * (qx*qz - qy*qw)
        r21 = 2 * (qy*qz + qx*qw)
        r22 = 1 - 2 * (qx*qx + qy*qy)

        rx = r00*x + r01*y + r02*z
        ry = r10*x + r11*y + r12*z
        rz = r20*x + r21*y + r22*z

        return rx, ry, rz

    def cloud_callback(self, msg):

        self.cloud_counter += 1

        source_frame = msg.header.frame_id

        # Get transform:
        # LiDAR frame -> map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )

        except Exception:

            self.get_logger().warn(
                f'Waiting for TF: '
                f'{self.map_frame} <- {source_frame}',
                throttle_duration_sec=2.0
            )

            return

        # Translation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        # Rotation quaternion
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        added_points = 0

        try:

            points = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )

            for point in points:

                x = float(point[0])
                y = float(point[1])
                z = float(point[2])

                if not (
                    math.isfinite(x)
                    and math.isfinite(y)
                    and math.isfinite(z)
                ):
                    continue

                # Transform point into map frame
                rx, ry, rz = self.rotate_point(
                    x,
                    y,
                    z,
                    qx,
                    qy,
                    qz,
                    qw
                )

                map_x = rx + tx
                map_y = ry + ty
                map_z = rz + tz

                # Downsample into voxels
                vx = int(
                    math.floor(
                        map_x / self.voxel_size
                    )
                )

                vy = int(
                    math.floor(
                        map_y / self.voxel_size
                    )
                )

                vz = int(
                    math.floor(
                        map_z / self.voxel_size
                    )
                )

                key = (
                    vx,
                    vy,
                    vz
                )

                if key not in self.voxels:

                    self.voxels[key] = (
                        map_x,
                        map_y,
                        map_z
                    )

                    added_points += 1

                    if (
                        len(self.voxels)
                        >= self.max_points
                    ):
                        break

        except Exception as error:

            self.get_logger().error(
                f'Point processing error: {error}'
            )

            return

        self.publish_map(msg)

        self.get_logger().info(
            f'Cloud #{self.cloud_counter} | '
            f'Added: {added_points} | '
            f'Map points: {len(self.voxels)}',
            throttle_duration_sec=2.0
        )

    def publish_map(self, original_msg):

        if not self.voxels:
            return

        points = list(
            self.voxels.values()
        )

        header = original_msg.header

        header.frame_id = self.map_frame

        header.stamp = (
            self.get_clock().now().to_msg()
        )

        cloud_msg = (
            point_cloud2.create_cloud_xyz32(
                header,
                points
            )
        )

        self.map_pub.publish(
            cloud_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = ParrotLidar3DMapping()

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


if __name__ == '__main__':
    main()