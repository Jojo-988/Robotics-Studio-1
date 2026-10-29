import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import open3d as o3d
import tf2_ros as tf2
from scipy.spatial.transform import Rotation
from rclpy.duration import Duration
from rclpy.time import Time
import threading
from std_msgs.msg import String
from std_msgs.msg import Bool
import json

# python3 scripts/environment_reconstruction.py --ros-args -r /tf:=/parrot1/tf -r /tf_static:=/parrot1/tf_static

class Environment_Reconstruction(Node):
    def __init__(self):
        super().__init__('env_reconstructor', namespace='parrot1')

        # tf2 buffer and listener to convert between coordinate frames
        self.tf_buffer = tf2.Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = tf2.TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)

        self.global_frame = 'parrot1_odom'
        self.sensor_frame = 'lidar_frame'
        self.frame_count = 0

        self.tsdf_volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=0.04, 
            sdf_trunc=0.12, 
            color_type=o3d.pipelines.integration.NoColor
            )

        self.global_point_cloud = o3d.geometry.PointCloud()

        self.is_meshing = False # flag to see whether async mesh thread is running
        
        qos_profile = QoSProfile(
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST,
                            depth=5
                        )

        # create subscription to point cloud
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, 
            'processed_point_cloud', 
            self.callback, 
            qos_profile=qos_profile
            )

        self.status_sub = self.create_subscription(
            String, 
            'patrol/status', 
            self.status_callback, 
            qos_profile=qos_profile
            )
        self.patrol_status = None

        # Timer to periodically extract mesh or save 3D reconstruction model
        self.save_timer = self.create_timer(20.0, self.export_mesh_callback)
        self.frame_count = 0

        self.get_logger().info('TSDF Environment Reconstruction Node Active.')

        self.mesh_export_pub = self.create_publisher(Bool, '/parrot1/mesh/exported', 10)

    def get_transform(self, target_frame, source_frame, stamp):

        # Get transforms from tf
        self.get_logger().info(
            'Attempting to read tf buffer...'
        )
        transform = self.tf_buffer.lookup_transform(
            source_frame=source_frame, 
            target_frame=target_frame, 
            time=stamp, 
            timeout= Duration(nanoseconds=50 * 10 **6) # 50ms timeout
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

        self.get_logger().info(
            'Recieved Transform'
        )

        return transform_4x4

    def callback(self, msg: PointCloud2):

        if self.patrol_status != 'PATROL':
            self.get_logger().info(
                'Awaiting Patrol...'
            )
            return
        
        try:
            # generate pose matrix between global and sensor frame
            pose_matrix = self.get_transform(
                self.global_frame, 
                msg.header.frame_id, 
                msg.header.stamp
                )
        except Exception as e:
            # if frame isnt buffered, safely drop it
            self.get_logger().warning(
                'Failed to recieve transform, frame dropped.'
            )
            return

        # covnert ros2 pointcloud2 into o3d pointcloud
        raw_points = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(raw_points)

        # estimate and orient normals to begin creating mesh
        # point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
        # point_cloud.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0])) #orients pc to lidar coordinate frame NOT global

        #set the pointcloud to the global coordinate frame
        point_cloud.transform(pose_matrix)

        # collect points in point cloud to later export when the timer loops
        self.global_point_cloud += point_cloud

        self.frame_count += 1
        self.get_logger().info(
            f'Integrated frame {self.frame_count} into global map.', throttle_duration_sec=2.0
            )

        # Keep global point cloud size bounded by downsampling every 10 frames
        if self.frame_count % 10 == 0:
            self.global_point_cloud = self.global_point_cloud.voxel_down_sample(0.04)

    def export_mesh_callback(self):
        if (self.frame_count == 0 or self.is_meshing == True) and self.patrol_status == 'COMPLETE':
            return

        self.get_logger().info(
            'Attempting to build mesh...'
        )

        mesh_thread = threading.Thread(target=self.async_mesh)
        mesh_thread.start()

    def async_mesh(self):

        self.is_meshing = True

        pcd = self.global_point_cloud.voxel_down_sample(voxel_size=0.04)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
        # pcd.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0])) #orients pc to lidar coordinate frame NOT global
        pcd.orient_normals_consistent_tangent_plane(k=15)

        # get mesh from point cloud
        radii = o3d.utility.DoubleVector([0.06, 0.12, 0.24, 0.40])
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd=pcd, radii=radii)
        mesh.compute_vertex_normals()

        # export mesh
        filename = 'Environment Reconstruction.ply'
        o3d.io.write_triangle_mesh(filename=filename, mesh=mesh, print_progress=True)
        self.is_meshing = False

        self.get_logger().info(
            f'Mesh successfully exported to {filename}'
            )

        msg = Bool()
        msg.data = True
        self.mesh_export_pub.publish(msg)

    def status_callback(self, msg):

        data = json.loads(msg.data)
        status = data.get('state', 'UNKNOWN')
        self.patrol_status = status
        # self.get_logger().info(
        #     f'Patrol Status: {status}'
        # )
        return

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
