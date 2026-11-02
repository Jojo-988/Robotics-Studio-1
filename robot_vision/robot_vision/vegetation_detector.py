#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import message_filters

from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class VegetationDetector(Node):
    def __init__(self):
        super().__init__('vegetation_detector')

        defaults = {
            'image_topic': '/parrot1/camera/image',
            'depth_topic': '/parrot1/camera/depth/image',
            'odom_topic': '/parrot1/odometry',
            'min_height': 1.0,
            'region_limit': 12.0,
            'dead_min_area': 120,
            'dead_match_distance': 1.0,
            'dead_confirm_frames': 5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if int(self.get_parameter('dead_min_area').value) < 1:
            raise ValueError('dead_min_area must be positive')
        if float(self.get_parameter('dead_match_distance').value) <= 0:
            raise ValueError('dead_match_distance must be positive')
        if int(self.get_parameter('dead_confirm_frames').value) < 1:
            raise ValueError('dead_confirm_frames must be positive')

        self.bridge = CvBridge()
        self.first_frame = True
        self.dead_tracks = {}
        self.next_dead_id = 1
        self.min_height = float(
            self.get_parameter('min_height').value
        )
        self.region_limit = float(
            self.get_parameter('region_limit').value
        )
        if self.min_height < 0 or self.region_limit <= 0:
            raise ValueError('Invalid height or region limit')

        # Intrinsics measured from the current 720 x 480 camera.
        self.fx = 207.8449214842941
        self.fy = 207.84492148429413
        self.cx = 360.0
        self.cy = 240.0

        v, u = np.indices((480, 720), dtype=np.float32)
        self.ray_x = (u - self.cx) / self.fx
        self.ray_y = (v - self.cy) / self.fy

        # Camera position in the robot base frame.
        self.offset = np.array([0.25, 0.0, -0.15])

        self.classes = [
            ('HEALTHY', (30, 70, 25), (95, 255, 210), (0, 255, 0)),
            ('STRESSED', (25, 15, 60), (100, 110, 230), (0, 255, 255)),
            ('DEAD', (0, 0, 15), (179, 45, 95), (0, 0, 255)),
        ]

        self.rgb_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('image_topic').value,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('depth_topic').value,
            qos_profile=qos_profile_sensor_data,
        )
        self.odom_sub = message_filters.Subscriber(
            self, Odometry, self.get_parameter('odom_topic').value,
            qos_profile=qos_profile_sensor_data,
        )

        # Match RGB, depth and pose by message timestamps.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.odom_sub],
            queue_size=30,
            slop=0.1,
        )
        self.sync.registerCallback(self.callback)

        self.get_logger().warning(
            'Requires downward camera at offset (0.25, 0, -0.15), '
            '720x480 images, flat ground Z=0, and scene-aligned odometry.'
        )
        self.get_logger().info(
            'Waiting for synchronized RGB, depth and odometry...'
        )

        # Publish the vegetation labels as an image topic for GUI display
        self.labels_pub = self.create_publisher(
            Image,
            '/parrot1/vegetation/labels',
            10
        )
        self.dead_preview_pub = self.create_publisher(
            Image,
            '/parrot1/vegetation/dead_preview',
            10
        )
    def update_dead_tracks(self, boxes):
        radius = float(self.get_parameter('dead_match_distance').value)
        required = int(self.get_parameter('dead_confirm_frames').value)

        observations = [(box[4], box[5]) for box in boxes]
        assignments = [None] * len(observations)

        # Match nearby observations and stored targets one-to-one.
        pairs = []
        for index, (x, y) in enumerate(observations):
            for track_id, track in self.dead_tracks.items():
                distance2 = (
                    (x - track['x'])**2
                    + (y - track['y'])**2
                )
                if distance2 <= radius**2:
                    pairs.append((distance2, index, track_id))

        used_tracks = set()

        for _, index, track_id in sorted(pairs):
            if assignments[index] is not None:
                continue
            if track_id in used_tracks:
                continue

            assignments[index] = track_id
            used_tracks.add(track_id)

            track = self.dead_tracks[track_id]
            x, y = observations[index]

            # Smooth small position fluctuations.
            track['x'] = 0.8 * track['x'] + 0.2 * x
            track['y'] = 0.8 * track['y'] + 0.2 * y
            track['hits'] += 1
            track['misses'] = 0

        for track_id in list(self.dead_tracks):
            if track_id in used_tracks:
                continue

            track = self.dead_tracks[track_id]
            track['hits'] = 0
            track['misses'] += 1

            # Discard short-lived candidates, but retain confirmed ones.
            if not track['confirmed'] and track['misses'] >= 3:
                del self.dead_tracks[track_id]

        for index, (x, y) in enumerate(observations):
            if assignments[index] is not None:
                continue

            track_id = self.next_dead_id
            self.next_dead_id += 1

            self.dead_tracks[track_id] = {
                'x': x,
                'y': y,
                'hits': 1,
                'misses': 0,
                'confirmed': False,
            }
            assignments[index] = track_id

        for track in self.dead_tracks.values():
            if track['hits'] >= required:
                track['confirmed'] = True

        total = sum(
            track['confirmed']
            for track in self.dead_tracks.values()
        )

        return assignments, total

    @staticmethod
    def assess_dead_count(total):
        if total == 0:
            return (
                'No Mortality Detected',
                'Continue monitoring',
                (0, 200, 0),
            )

        if total < 5:
            return (
                'Low Mortality Concern',
                'Record locations; monitor changes',
                (0, 220, 220),
            )

        if total < 10:
            return (
                'Moderate Mortality Concern',
                'Schedule on-site inspection',
                (0, 165, 255),
            )

        return (
            'High Mortality Concern',
            'Prioritise on-site investigation',
            (0, 0, 255),
        )
    
    def make_dead_preview(
        self, overlay, dead_mask, has_surfaces, points
    ):
        """Preview connected DEAD regions in this frame only."""
        preview = overlay.copy()
        binary = dead_mask.astype(np.uint8) * 255
        min_area = int(self.get_parameter('dead_min_area').value)

        count, component_labels, stats, _ = (
            cv2.connectedComponentsWithStats(binary, connectivity=8)
        )

        boxes = []
        for component in range(1, count):
            x, y, w, h, area = [
                int(value) for value in stats[component]
            ]

            if area < min_area or w < 8 or h < 8:
                continue

            # Use the actual DEAD pixels belonging to this component.
            selected = component_labels == component
            region_points = points[selected]

            valid_points = region_points[
                np.all(np.isfinite(region_points), axis=1)
            ]
            if len(valid_points) == 0:
                continue

            # Median position reduces the effect of isolated depth errors.
            world_x, world_y = np.median(
                valid_points[:, :2], axis=0
            )

            boxes.append(
                (x, y, w, h, float(world_x), float(world_y))
            )

        boxes.sort(key=lambda box: (box[0], box[1]))

        track_ids, total = self.update_dead_tracks(boxes)

        for track_id, box in zip(track_ids, boxes):
            x, y, w, h, world_x, world_y = box

            confirmed = self.dead_tracks[track_id]['confirmed']
            marker = '' if confirmed else '?'

            caption = (
                f'T{track_id}{marker} '
                f'({world_x:.1f}, {world_y:.1f})'
            )
            cv2.rectangle(
                preview,
                (x, y),
                (x + w - 1, y + h - 1),
                (0, 255, 255),
                2,
            )

            text_y = max(18, y - 6)

            # Black outline keeps the text readable.
            cv2.putText(
                preview,
                caption,
                (x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )
            cv2.putText(
                preview,
                caption,
                (x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        dead_count = len(boxes)

        assessment, advice, status_colour = self.assess_dead_count(total)

        # No usable retained surfaces: do not present a health conclusion.
        if not has_surfaces:
            current_text = 'Current dead candidates: N/A'
            assessment = 'Cannot Assess Current View'
            advice = 'Observe the vegetation from a suitable position'
            status_colour = (180, 180, 180)

        else:
            current_text = f'Current dead candidates: {dead_count}'

            # Avoid reporting zero mortality while candidates await confirmation.
            if total == 0 and dead_count > 0:
                assessment = 'Confirming Candidates'
                advice = 'Keep the target in view'
                status_colour = (0, 220, 220)

        display_lines = [
            (current_text, 27, (255, 255, 255)),
            (
                f'Confirmed candidates since start: {total}',
                53,
                (255, 255, 255),
            ),
            (
                f'Preliminary: {assessment}',
                79,
                status_colour,
            ),
            (
                f'Advice: {advice}',
                105,
                (255, 255, 255),
            ),
        ]

        for text, y, colour in display_lines:
            # Black outline for readability on the camera image.
            cv2.putText(
                preview,
                text,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )

            cv2.putText(
                preview,
                text,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                colour,
                2,
                cv2.LINE_AA,
            )

        return preview, dead_count

    @staticmethod
    def rotation_matrix(q):
        values = np.array([q.x, q.y, q.z, q.w], dtype=float)
        norm = np.linalg.norm(values)
        if not np.isfinite(norm) or norm < 1e-8:
            raise ValueError('Invalid odometry orientation')

        x, y, z, w = values / norm
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])

    def callback(self, rgb_msg, depth_msg, odom_msg):
        try:
            if depth_msg.encoding != '32FC1':
                raise ValueError('Expected 32FC1 depth')
            if odom_msg.header.frame_id != 'parrot1_odom':
                raise ValueError('Unexpected odometry frame')
            if odom_msg.child_frame_id != 'parrot1_base_link':
                raise ValueError('Unexpected odometry child frame')

            frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, 'passthrough'
            )
            if depth.shape != (480, 720):
                raise ValueError('Expected 720 x 480 depth')
            if frame.shape[:2] != depth.shape:
                raise ValueError('RGB and depth sizes differ')

            pose = odom_msg.pose.pose
            position = np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ])
            if not np.all(np.isfinite(position)):
                raise ValueError('Invalid odometry position')
            rotation = self.rotation_matrix(pose.orientation)

            valid = np.isfinite(depth) & (depth > 0)
            d = np.where(valid, depth, 0.0)

            # Optical coordinates: right, down, forward.
            optical_x = self.ray_x * d
            optical_y = self.ray_y * d

            # Downward camera -> robot base coordinates.
            # Camera pitch relative to base is +90 degrees.
            base_points = np.stack(
                (-optical_y, -optical_x, -d), axis=-1
            )
            base_points += self.offset

            # Robot base -> odometry coordinates.
            points = base_points @ rotation.T + position
            px = points[:, :, 0]
            py = points[:, :, 1]
            pz = points[:, :, 2]

            kept = (
                valid
                & (pz > self.min_height)
                & (np.abs(px) < self.region_limit)
                & (np.abs(py) < self.region_limit)
            )
            gate = kept.astype(np.uint8) * 255

            retained = np.zeros_like(frame)
            retained[kept] = frame[kept]

            hsv = cv2.cvtColor(
                cv2.GaussianBlur(frame, (7, 7), 0),
                cv2.COLOR_BGR2HSV,
            )
            kernel = np.ones((5, 5), np.uint8)
            masks = []

            for name, lower, upper, colour in self.classes:
                mask = cv2.inRange(
                    hsv,
                    np.array(lower, dtype=np.uint8),
                    np.array(upper, dtype=np.uint8),
                )
                mask = cv2.bitwise_and(mask, gate)
                mask = cv2.morphologyEx(
                    mask, cv2.MORPH_OPEN, kernel
                )
                mask = cv2.morphologyEx(
                    mask, cv2.MORPH_CLOSE, kernel
                )
                masks.append(
                    (cv2.bitwise_and(mask, gate)) > 0
                )

            matches = np.stack(masks).sum(axis=0)
            exclusive = [
                mask & (matches == 1) for mask in masks
            ]
            overlap = kept & (matches > 1)
            unknown = kept & (matches == 0)

            labels = np.zeros_like(frame)
            labels[unknown] = (100, 100, 100)
            labels[overlap] = (255, 0, 255)

            overlay = frame.copy()
            for selected, item in zip(exclusive, self.classes):
                colour = np.array(item[3], dtype=np.float32)
                labels[selected] = item[3]
                overlay[selected] = (
                    0.45 * frame[selected].astype(np.float32)
                    + 0.55 * colour
                ).astype(np.uint8)

            overlay[overlap] = (
                0.45 * frame[overlap].astype(np.float32)
                + 0.55 * np.array((255, 0, 255), dtype=np.float32)
            ).astype(np.uint8)

            # Class order: HEALTHY, STRESSED, DEAD.
            # Use only pixels classified exclusively as DEAD.
            dead_mask = exclusive[2]

            dead_preview, dead_count = self.make_dead_preview(
                overlay,
                dead_mask,
                bool(kept.any()),
                points,
            )

            panel = np.zeros((125, 720, 3), dtype=np.uint8)
            lines = [
                f'Image-time pose: X={position[0]:.2f} '
                f'Y={position[1]:.2f} Z={position[2]:.2f} m',
                f'Height > {self.min_height:.1f} m | '
                f'Region +/-{self.region_limit:.1f} m',
                f'Dead candidates in current frame: {dead_count} (not total)',
                'Green=Healthy Yellow=Stressed Red=Dead',
                'Magenta=Ambiguous Grey=Unclassified Black=Excluded',
            ]
            if not kept.any():
                lines[2] = 'NO RETAINED SURFACES - cannot assess'

            for index, text in enumerate(lines):
                cv2.putText(
                    panel, text, (8, 20 + index * 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.44, (255, 255, 255), 1,
                )

            # display CV2 windows are disabled for now, instead it will displayed the labels on the GUI
            # cv2.imshow(
            #     'Vegetation - Overlay',
            #     np.vstack((overlay, panel)),
            # )
            # cv2.imshow('Vegetation - Labels', labels)
            # cv2.imshow('Vegetation - Retained Surfaces', retained)
            cv2.waitKey(1)

            if self.first_frame:
                self.get_logger().info(
                    'First synchronized RGB-D and pose processed'
                )
                self.first_frame = False

            labels_msg = self.bridge.cv2_to_imgmsg(
                labels,
                encoding='bgr8'
            )
            labels_msg.header = rgb_msg.header
            self.labels_pub.publish(labels_msg)

            preview_msg = self.bridge.cv2_to_imgmsg(
                dead_preview,
                encoding='bgr8',
            )
            preview_msg.header = rgb_msg.header
            self.dead_preview_pub.publish(preview_msg)

        except Exception as error:
            self.get_logger().error(
                f'Processing failed: {error}',
                throttle_duration_sec=5.0,
            )


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