import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import open3d as o3d
import tf2_ros as tf2
from scipy.spatial.transform import Rotation

class Environment_Reconstruction(Node):
    def __init__(self):
        super().__init__('env_reconstructor')

        # tf2 buffer and listener to convert between coordinate frames
        self.tf_buffer = tf2.Buffer()
        self.tf_listener = tf2.TransformListener(buffer=self.tf_buffer, node=self)

        self.global_frame = 'map'
        self.sensor_frame = 'lidar_frame'
        self.frame_count = 0

        self.tsdf_volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=0.04, 
            sdf_trunc=0.12, 
            color_type=o3d.pipelines.integration.NoColor
            )

        qos_profile = QoSProfile(
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST,
                            depth=5
                        )

        # create subscription to point cloud
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, 
            'parrot1/processed_point_cloud', 
            self.callback, 
            qos_profile=qos_profile
            )

        # Timer to periodically extract mesh or save 3D reconstruction model
        self.save_timer = self.create_timer(10.0, self.export_mesh_callback)
        self.frame_count = 0

        self.get_logger().info('TSDF Environment Reconstruction Node Active.')

    def get_transform(self, target_frame, source_frame, time_stamp):

        # Get transforms from tf
        transform = self.tf_buffer.lookup_transform(
            source_frame=source_frame, 
            target_frame=target_frame, 
            time=time_stamp, 
            timeout=0.1
            )

        # extract translations
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        # extrcat quarternions
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # build 4x4 matrix
        rot_matrix = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        transform_4x4 = np.eye(4)
        transform_4x4[:3,:3] = rot_matrix
        transform_4x4[:3,3] = [tx, ty, tz]

        return transform_4x4

    def callback(self, msg: PointCloud2):

        # generate pose matrix between global and sensor frame
        pose_matrix = self.get_transform(
            self.global_frame, 
            msg.header.frame_id, 
            msg.header.stamp
            )

        # covnert ros2 pointcloud2 into o3d pointcloud
        raw_points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(raw_points)

        # estimate and orient normals to begin creating mesh
        point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
        point_cloud.orient_normals_towards_camera_location(camera_location=pose_matrix[:3,3])

        #set the pointcloud to the global coordinate frame
        point_cloud.transform(pose_matrix)

        # integrate pointcloud into mesh
        self.tsdf_volume.integrate(
            o3d.geometry.RGBDImage(), 
            intrinstic=o3d.camera.PinholeCameraIntrinsic(),
            extrinsic=np.eye(4)
            )

        self.frame_count += 1
        self.get_logger().info(f'Integrated frame {self.frame_count} into global map.', throttle_duration_sec=2.0)

    def export_mesh_callback(self):
        if self.frame_count == 0:
            return

        # get mesh from buffer
        mesh = self.tsdf_volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()

        # export mesh
        filename = 'Environment Reconstruction.ply'
        o3d.io.write_triangle_mesh(filename=filename, mesh=mesh)
        self.get_logger().info(f'Mesh successfully exported to {filename}')

def main(args=None):
    rclpy.init(args=args)

    node = Environment_Reconstruction()

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
