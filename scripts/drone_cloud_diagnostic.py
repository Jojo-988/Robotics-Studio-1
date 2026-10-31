#!/usr/bin/env python3
"""Inspect one cloud at a time in a fixed coordinate frame."""

import time
from copy import deepcopy

import json

from std_msgs.msg import String
from map_data.bird_grid import (
    StructureWindow,
    classify_counts,
    classify_window_counts,
    project_hits,
    summarize_cells,
)
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger



def transform_xyz(points, transform):
    """Apply a rigid transform to an N-by-3 array."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation

    q = np.array(
        [rotation.x, rotation.y, rotation.z, rotation.w],
        dtype=np.float64,
    )
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid TF quaternion')

    x, y, z, w = q / norm

    matrix = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])

    offset = np.array(
        [translation.x, translation.y, translation.z],
        dtype=np.float64,
    )
    if not np.isfinite(offset).all():
        raise ValueError('Invalid TF translation')

    return points @ matrix.T + offset


class DroneCloudDiagnostic(Node):
    def __init__(self):
        super().__init__('drone_cloud_diagnostic')

        self.declare_parameter('cloud_topic', '/parrot1/scan/points')
        self.declare_parameter('target_frame', 'parrot1_odom')

        # A fixed square region in the target frame.
        self.declare_parameter('roi_x', 2.0)
        self.declare_parameter('roi_y', 0.0)
        self.declare_parameter('roi_half_size', 1.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.roi_x = self.get_parameter('roi_x').value
        self.roi_y = self.get_parameter('roi_y').value
        self.roi_half_size = self.get_parameter('roi_half_size').value

        if self.roi_half_size <= 0:
            raise ValueError('roi_half_size must be positive')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pending = None
        self.pending_since = 0.0
        self.processed = 0
        self.busy_skips = 0
        self.tf_drops = 0

         # These settings are fixed for one mapping session.
        config_defaults = {
            'grid_resolution': 0.10,
            'grid_origin_x': -10.0,
            'grid_origin_y': -10.0,
            'grid_width': 200,
            'grid_height': 200,
            'ground_z': 0.0,
            'ground_tolerance': 0.05,
            'structure_min_height': 0.15,
            'min_ground_frames': 3,
            'min_structure_frames': 3,
            'structure_confirmation_mode': 'cumulative',
            'structure_window_frames': 5,
        }

        for name, default in config_defaults.items():
            self.declare_parameter(
                name,
                default,
                descriptor=ParameterDescriptor(read_only=True),
            )
            setattr(self, name, self.get_parameter(name).value)

        finite_values = [
            self.grid_resolution,
            self.grid_origin_x,
            self.grid_origin_y,
            self.ground_z,
            self.ground_tolerance,
            self.structure_min_height,
        ]
        if not np.isfinite(finite_values).all():
            raise ValueError('Mapping parameters must be finite')

        if self.grid_resolution <= 0:
            raise ValueError('grid_resolution must be positive')

        for name in ('grid_width', 'grid_height'):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f'{name} must be a positive integer')

        if self.ground_tolerance <= 0:
            raise ValueError('ground_tolerance must be positive')

        if self.structure_min_height <= self.ground_tolerance:
            raise ValueError(
                'structure_min_height must exceed ground_tolerance'
            )

        self.count_limit = 1_000_000

        for name in ('min_ground_frames', 'min_structure_frames'):
            value = getattr(self, name)
            if (
                type(value) is not int
                or not 1 <= value <= self.count_limit
            ):
                raise ValueError(
                    f'{name} must be an integer from 1 '
                    f'to {self.count_limit}'
                )

        if self.structure_confirmation_mode not in (
            'cumulative',
            'window',
        ):
            raise ValueError(
                'structure_confirmation_mode must be cumulative or window'
            )

        if (
            type(self.structure_window_frames) is not int
            or self.structure_window_frames <= 0
        ):
            raise ValueError(
                'structure_window_frames must be a positive integer'
            )

        if (
            self.structure_confirmation_mode == 'window'
            and self.min_structure_frames > self.structure_window_frames
        ):
            raise ValueError(
                'min_structure_frames must not exceed '
                'structure_window_frames in window mode'
            )
        cell_count = self.grid_width * self.grid_height

        self.ground_observations = np.zeros(
            cell_count, dtype=np.uint32
        )
        self.structure_observations = np.zeros(
            cell_count, dtype=np.uint32
        )
        self.structure_window = None

        if self.structure_confirmation_mode == 'window':
            self.structure_window = StructureWindow(
                cell_count=cell_count,
                window_frames=self.structure_window_frames,
                min_hits=self.min_structure_frames,
            )

        self.accumulated_frames = 0
        self.last_accumulated_stamp_ns = None
        self.accumulation_start_stamp = None
        self.reset_after_stamp_ns = None

        topic = self.get_parameter('cloud_topic').value
        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        preview_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.preview_publisher = self.create_publisher(
            OccupancyGrid,
            '/parrot1/bird_map_preview',
            preview_qos,
        )

        self.accumulated_publisher = self.create_publisher(
            OccupancyGrid,
            '/parrot1/bird_map_accumulated_preview',
            preview_qos,
        )
        self.declare_parameter('map_topic', '/parrot1/bird_map')
        self.declare_parameter(
            'statistics_topic', '/parrot1/bird_map_statistics'
        )

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            preview_qos,
        )

        self.statistics_publisher = self.create_publisher(
            String,
            self.get_parameter('statistics_topic').value,
            preview_qos,
        )

        self.reset_service = self.create_service(
            Trigger,
            '~/reset_map',
            self.reset_map_callback,
        )

        # Retry TF without blocking the listener callbacks.
        self.timer = self.create_timer(0.05, self.process_pending)

        self.get_logger().info(
            f'Cloud: {topic}; target frame: {self.target_frame}; '
            f'structure mode={self.structure_confirmation_mode}; '
            f'min hits={self.min_structure_frames}; '
            f'window frames={self.structure_window_frames}'
        )

    def publish_delivery(self, map_message, mixed_cells=0):
        """Publish the map and statistics derived from the same snapshot."""
        summary = summarize_cells(map_message.data)

        payload = {
            'schema_version': 1,
            'map_kind': 'lidar_bird_eye_classification',
            'frame_id': map_message.header.frame_id,
            'source_stamp': {
                'sec': int(map_message.header.stamp.sec),
                'nanosec': int(map_message.header.stamp.nanosec),
            },
            'width': int(map_message.info.width),
            'height': int(map_message.info.height),
            'resolution': float(map_message.info.resolution),
            'origin': {
                'x': float(map_message.info.origin.position.x),
                'y': float(map_message.info.origin.position.y),
                'yaw': 0.0,
            },
            'accumulated_frames': int(self.accumulated_frames),
            'structure_confirmation_mode':
                self.structure_confirmation_mode,
            'structure_window_frames':
                int(self.structure_window_frames),
            'min_structure_frames':
                int(self.min_structure_frames),
            'structure_window_fill': (
                self.structure_window.frames_in_window
                if self.structure_window is not None
                else 0
            ),
            'mixed_cells': int(mixed_cells),
            'tf_drops': int(self.tf_drops),
            'busy_skips': int(self.busy_skips),
            **summary,
        }

        statistics_message = String()
        statistics_message.data = json.dumps(
            payload, allow_nan=False
        )

        self.map_publisher.publish(map_message)
        self.statistics_publisher.publish(statistics_message)

    def reset_map_callback(self, request, response):
        """Start a new accumulation session in the same simulation."""
        now = self.get_clock().now()

        # Discard pending and delayed observations from before reset.
        self.pending = None
        self.pending_since = 0.0
        self.reset_after_stamp_ns = now.nanoseconds

        self.ground_observations.fill(0)
        self.structure_observations.fill(0)
        if self.structure_window is not None:
            self.structure_window.reset()

        self.accumulated_frames = 0
        self.last_accumulated_stamp_ns = None
        self.accumulation_start_stamp = None

        self.processed = 0
        self.busy_skips = 0
        self.tf_drops = 0

        # Publish an empty map immediately so RViz does not retain
        # the previous experiment's displayed map.
        empty_map = OccupancyGrid()
        empty_map.header.stamp = now.to_msg()
        empty_map.header.frame_id = self.target_frame

        empty_map.info.map_load_time = now.to_msg()
        empty_map.info.resolution = self.grid_resolution
        empty_map.info.width = self.grid_width
        empty_map.info.height = self.grid_height

        empty_map.info.origin.position.x = self.grid_origin_x
        empty_map.info.origin.position.y = self.grid_origin_y
        empty_map.info.origin.position.z = 0.0
        empty_map.info.origin.orientation.w = 1.0

        cell_count = self.grid_width * self.grid_height
        empty_map.data = [-1] * cell_count

        self.preview_publisher.publish(empty_map)
        self.accumulated_publisher.publish(deepcopy(empty_map))
        self.publish_delivery(empty_map)

        response.success = True
        response.message = (
            'Map cleared. Waiting for clouds captured after reset.'
        )
        self.get_logger().info(response.message)
        return response

    def cloud_callback(self, msg):
        if not msg.header.frame_id:
            self.get_logger().warning(
                'Dropped cloud with empty frame_id',
                throttle_duration_sec=2.0,
            )
            return

        # A zero TF time means "latest", which we do not want here.
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.get_logger().warning(
                'Dropped cloud with zero timestamp',
                throttle_duration_sec=2.0,
            )
            return

        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        if (
            self.reset_after_stamp_ns is not None
            and stamp_ns <= self.reset_after_stamp_ns
        ):
            return

        if self.pending is not None:
            self.busy_skips += 1
            return

        self.pending = msg
        self.pending_since = time.monotonic()

    def process_pending(self):
        if self.pending is None:
            return

        msg = self.pending

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
            )
        except TransformException as error:
            if time.monotonic() - self.pending_since > 1.0:
                self.tf_drops += 1
                self.pending = None
                self.get_logger().warning(
                    f'TF timeout; dropped cloud: {error}',
                    throttle_duration_sec=2.0,
                )
            return

        self.pending = None

        try:
            records = point_cloud2.read_points(
                msg,
                field_names=['x', 'y', 'z'],
                skip_nans=False,
            )

            points = np.array(
                [
                    [float(point[0]), float(point[1]), float(point[2])]
                    for point in records
                ],
                dtype=np.float64,
            ).reshape(-1, 3)

            total = len(points)
            points = points[np.isfinite(points).all(axis=1)]

            if len(points) == 0:
                self.get_logger().warning(
                    'Cloud contains no finite XYZ points',
                    throttle_duration_sec=2.0,
                )
                return

            world_points = transform_xyz(points, transform)

        except (ValueError, TypeError, AssertionError) as error:
            self.get_logger().error(
                f'Cannot process cloud: {error}',
                throttle_duration_sec=2.0,
            )
            return

        self.processed += 1

        z_values = world_points[:, 2]
        ground_z = self.ground_z
        ground_tolerance = self.ground_tolerance
        structure_min_height = self.structure_min_height

        relative_z = z_values - ground_z

        ground_mask = np.abs(relative_z) <= ground_tolerance
        structure_mask = relative_z >= structure_min_height
        unclassified_mask = ~(ground_mask | structure_mask)

        ground_count = int(np.count_nonzero(ground_mask))
        structure_count = int(np.count_nonzero(structure_mask))
        unclassified_count = int(np.count_nonzero(unclassified_mask))
        resolution = self.grid_resolution
        origin_x = self.grid_origin_x
        origin_y = self.grid_origin_y
        width = self.grid_width
        height = self.grid_height

        ground_hits, structure_hits, outside_points = project_hits(
            world_points,
            ground_mask,
            structure_mask,
            resolution,
            origin_x,
            origin_y,
            width,
            height,
        )

        ground_only = ground_hits & ~structure_hits
        mixed = ground_hits & structure_hits
        no_classified_evidence = ~(ground_hits | structure_hits)

        ground_only_cells = int(np.count_nonzero(ground_only))
        structure_cells = int(np.count_nonzero(structure_hits))
        mixed_cells = int(np.count_nonzero(mixed))
        unknown_cells = int(np.count_nonzero(no_classified_evidence))
        # Diagnostic encoding:
        # -1: no classified evidence
        #  0: ground-only candidate
        # 100: structure candidate, including mixed cells
        cells = np.full(width * height, -1, dtype=np.int8)
        cells[ground_only] = 0
        cells[structure_hits] = 100

        preview = OccupancyGrid()

        # Preserve the acquisition time of the source cloud.
        preview.header.stamp = msg.header.stamp
        preview.header.frame_id = self.target_frame

        preview.info.map_load_time = msg.header.stamp
        preview.info.resolution = resolution
        preview.info.width = width
        preview.info.height = height

        preview.info.origin.position.x = origin_x
        preview.info.origin.position.y = origin_y
        preview.info.origin.position.z = 0.0
        preview.info.origin.orientation.w = 1.0

        preview.data = cells.tolist()
        self.preview_publisher.publish(preview)
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        if (
            self.last_accumulated_stamp_ns is not None
            and stamp_ns <= self.last_accumulated_stamp_ns
        ):
            self.get_logger().warning(
                'Skipped duplicate or older cloud. '
                'Restart this node if the simulation was reset.',
                throttle_duration_sec=2.0,
            )
            return

        if self.accumulation_start_stamp is None:
            self.accumulation_start_stamp = deepcopy(msg.header.stamp)

        self.last_accumulated_stamp_ns = stamp_ns

        # Count each cell at most once per category per frame.
        self.ground_observations[ground_hits] = np.minimum(
            self.ground_observations[ground_hits] + 1,
            self.count_limit,
        )
        self.structure_observations[structure_hits] = np.minimum(
            self.structure_observations[structure_hits] + 1,
            self.count_limit,
        )
        self.accumulated_frames += 1

        if self.structure_window is not None:
            confirmed_structure = self.structure_window.update(
                structure_hits
            )

            accumulated_cells, supported_mixed = classify_window_counts(
                self.ground_observations,
                self.structure_observations,
                self.min_ground_frames,
                confirmed_structure,
            )
        else:
            accumulated_cells, supported_mixed = classify_counts(
                self.ground_observations,
                self.structure_observations,
                self.min_ground_frames,
                self.min_structure_frames,
            )
        accumulated = OccupancyGrid()
        accumulated.header = deepcopy(preview.header)
        accumulated.info = deepcopy(preview.info)
        accumulated.info.map_load_time = deepcopy(
            self.accumulation_start_stamp
        )
        accumulated.data = accumulated_cells.tolist()
        self.accumulated_publisher.publish(accumulated)
        self.publish_delivery(
            accumulated,
            mixed_cells=int(np.count_nonzero(supported_mixed)),
        )

        accumulated_ground = int(
            np.count_nonzero(accumulated_cells == 0)
        )
        accumulated_structure = int(
            np.count_nonzero(accumulated_cells == 100)
        )
        accumulated_unknown = int(
            np.count_nonzero(accumulated_cells == -1)
        )
        accumulated_mixed = int(
            np.count_nonzero(supported_mixed)
        )

        self.get_logger().info(
            f'ACCUM frames={self.accumulated_frames}; '
            f'ground={accumulated_ground}, '
            f'structure={accumulated_structure}, '
            f'mixed={accumulated_mixed}, '
            f'unknown={accumulated_unknown}; '
            f'max_ground_frames={int(self.ground_observations.max())}, '
            f'max_structure_frames='
            f'{int(self.structure_observations.max())}',
            throttle_duration_sec=2.0,
        )
        p05, p50, p95 = np.percentile(z_values, [5, 50, 95])

        roi_mask = (
            (np.abs(world_points[:, 0] - self.roi_x) <= self.roi_half_size)
            & (np.abs(world_points[:, 1] - self.roi_y) <= self.roi_half_size)
        )
        roi_z = z_values[roi_mask]

        roi_text = 'ROI: no points'
        if len(roi_z) > 0:
            r05, r50, r95 = np.percentile(roi_z, [5, 50, 95])
            roi_text = (
                f'ROI: n={len(roi_z)}, '
                f'z[p05,p50,p95]=[{r05:.3f},{r50:.3f},{r95:.3f}] m'
            )

        stamp = msg.header.stamp
        self.get_logger().info(
            f't={stamp.sec}.{stamp.nanosec:09d}; '
            f'{msg.header.frame_id}->{self.target_frame}; '
            f'valid={len(points)}/{total}; '
            f'x=[{world_points[:, 0].min():.2f},'
            f'{world_points[:, 0].max():.2f}] m; '
            f'y=[{world_points[:, 1].min():.2f},'
            f'{world_points[:, 1].max():.2f}] m; '
            f'z[min,p05,p50,p95,max]='
            f'[{z_values.min():.3f},{p05:.3f},{p50:.3f},'
            f'{p95:.3f},{z_values.max():.3f}] m; '
            f'{roi_text};'
            f'ground_candidates={ground_count}, '
            f'structure_candidates={structure_count}, '
            f'unclassified={unclassified_count}; '
            f'grid: ground_only={ground_only_cells}, '
            f'structure={structure_cells}, '
            f'mixed={mixed_cells}, '
            f'unknown={unknown_cells}, '
            f'outside_points={outside_points}; '
            f'processed={self.processed}, '
            f'busy_skips={self.busy_skips}, tf_drops={self.tf_drops}',
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = DroneCloudDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()