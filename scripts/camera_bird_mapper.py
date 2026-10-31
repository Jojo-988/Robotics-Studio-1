#!/usr/bin/env python3
"""Live simulation camera ground-plane mosaic; no movement commands."""
from collections import OrderedDict
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from map_data.camera_projection import project_rgb
from map_data.camera_mosaic import CameraMosaic


def plain(message):
    return json.loads(json.dumps(message_to_ordereddict(message)))


def ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def stamp_dict(value):
    return {'sec': value // 1_000_000_000, 'nanosec': value % 1_000_000_000}


def decode_rgb(message):
    formats = {'rgb8': ('RGB', 'RGB', 3), 'bgr8': ('RGB', 'BGR', 3),
               'rgba8': ('RGBA', 'RGBA', 4), 'bgra8': ('RGBA', 'BGRA', 4)}
    if message.encoding not in formats:
        raise ValueError('Unsupported colour encoding: ' + message.encoding)
    mode, raw, channels = formats[message.encoding]
    if message.width < 2 or message.height < 2:
        raise ValueError('Invalid image size')
    if message.step < message.width * channels or len(message.data) != message.step*message.height:
        raise ValueError('Invalid image step/byte count')
    image = PILImage.frombytes(mode, (message.width, message.height), bytes(message.data),
                              'raw', raw, message.step, 1).convert('RGB')
    return np.frombuffer(image.tobytes(), np.uint8).reshape(message.height, message.width, 3).copy()


class CameraBirdMapper(Node):
    def __init__(self):
        super().__init__('camera_bird_mapper')
        defaults = {
            'image_topic': '/parrot1/camera/image',
            'camera_info_topic': '/parrot1/camera/camera_info',
            'camera_frame': 'parrot1_camera_link', 'target_frame': 'parrot1_odom',
            'output_prefix': '/parrot1/camera_bird_map',
            'resolution': 0.01, 'origin_x': -2.0, 'origin_y': -4.0,
            'width': 800, 'height': 800, 'ground_z': 0.0, 'exclude_bottom': 0.10,
            'process_hz': 1.0, 'pair_timeout_sec': 2.0,
            'save_directory': '~/team-results/drone-camera/live',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.mosaic = CameraMosaic(self.p['resolution'], self.p['origin_x'],
                                   self.p['origin_y'], self.p['width'], self.p['height'])
        if not 0 < self.p['process_hz'] <= 10 or not 0 < self.p['pair_timeout_sec'] <= 10:
            raise ValueError('Invalid rate/timeout')
        if not 0 <= self.p['exclude_bottom'] < 1 or not np.isfinite(self.p['ground_z']):
            raise ValueError('Invalid projection parameters')
        self.images, self.infos = OrderedDict(), OrderedDict()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.image_sub = self.create_subscription(Image, self.p['image_topic'],
            lambda msg: self.cache(self.images, msg), qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, self.p['camera_info_topic'],
            lambda msg: self.cache(self.infos, msg), qos_profile_sensor_data)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        prefix = self.p['output_prefix'].rstrip('/')
        self.image_pub = self.create_publisher(Image, prefix+'/image', qos)
        self.mask_pub = self.create_publisher(Image, prefix+'/valid_mask', qos)
        self.meta_pub = self.create_publisher(String, prefix+'/metadata', qos)
        self.reset_srv = self.create_service(Trigger, '~/reset_map', self.reset_service)
        self.save_srv = self.create_service(Trigger, '~/save_map', self.save_service)
        self.epoch = str(uuid.uuid4())
        self.version = 0
        self.last_input = 0
        self.last_output_stamp = 0
        self.floor = 0
        self.frames = 0
        self.drops = {'unmatched_images': 0, 'tf_timeout': 0, 'invalid': 0,
                      'superseded_pairs': 0, 'outside_frames': 0}
        self.last_outside = 0
        self.last_error = ''
        self.last_metadata = None
        self.last_tick_clock = self.get_clock().now().nanoseconds
        self.next_process = 0.0
        self.last_received_stamp = 0
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.tick, clock=self.steady)
        self.publish(0, 'waiting_for_data')
        self.get_logger().info('Camera mapper ready. Needs running simulation and CameraInfo. '
                               f'Output: {prefix}; canvas {self.p["width"]}x{self.p["height"]}')

    def check_clock(self):
        now = self.get_clock().now().nanoseconds
        if now < self.last_tick_clock:
            self.clear(now, 'clock_reset')
            # Buffer's ROS clock-jump handler clears transforms; do not manually
            # clear static TF here, since /tf_static may only have been sent once.
        self.last_tick_clock = now

    def cache(self, cache, message):
        self.check_clock()
        key = ns(message)
        if key <= max(self.floor, self.last_input) or key <= 0:
            return
        cache[key] = (message, time.monotonic())
        while len(cache) > 8:
            old, _ = cache.popitem(last=False)
            if cache is self.images:
                self.drops['unmatched_images'] += 1

    def clear(self, floor, reason):
        self.images.clear()
        self.infos.clear()
        self.mosaic.clear()
        self.floor = floor
        self.last_input = 0
        self.frames = 0
        self.last_outside = 0
        self.last_error = ''
        self.drops = dict.fromkeys(self.drops, 0)
        self.epoch = str(uuid.uuid4())
        self.version = 0
        self.last_output_stamp = 0
        self.next_process = 0.0
        self.publish(floor, reason)

    def reset_service(self, request, response):
        self.clear(self.get_clock().now().nanoseconds, 'reset')
        response.success = True
        response.message = 'Cleared. Waiting for images captured after reset.'
        return response

    def tick(self):
        self.check_clock()
        wall = time.monotonic()
        expired = [key for key, (_, arrival) in self.images.items()
                   if wall-arrival > self.p['pair_timeout_sec']]
        for key in expired:
            self.drops['tf_timeout' if key in self.infos else 'unmatched_images'] += 1
            self.images.pop(key, None)
            self.infos.pop(key, None)
        for key in list(self.infos):
            if wall-self.infos[key][1] > self.p['pair_timeout_sec']:
                self.infos.pop(key)
        if wall < self.next_process:
            return
        common = sorted(self.images.keys() & self.infos.keys(), reverse=True)
        if not common:
            return
        # Try newest pair with available exact-time TF, without blocking executor.
        for key in common:
            image = self.images[key][0]
            info = self.infos[key][0]
            try:
                tf = self.tf_buffer.lookup_transform(self.p['target_frame'],
                    self.p['camera_frame'], Time.from_msg(image.header.stamp))
            except TransformException:
                continue
            try:
                if image.header.frame_id != self.p['camera_frame']:
                    raise ValueError('Image frame differs from configured camera_frame')
                stamp = plain(image.header)['stamp']
                pose = {
                    'image_stamp': stamp, 'camera_info_stamp': plain(info.header)['stamp'],
                    'target_frame': self.p['target_frame'],
                    'source_frame': self.p['camera_frame'], 'transform': plain(tf),
                    'optical_convention': {
                        'status': 'model_based_convention_not_independently_calibrated',
                        'basis': 'Parrot RGBD sensor at camera_link with no sensor pose offset',
                        'T_camera_link_from_optical': [
                            [0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]],
                    },
                }
                tile, mask, metadata = project_rgb(decode_rgb(image), plain(info), pose,
                    {'header': plain(image.header), 'width': image.width, 'height': image.height},
                    self.p['resolution'], self.p['ground_z'], self.p['exclude_bottom'])
                inside, self.last_outside = self.mosaic.add(tile, mask, metadata)
                if inside == 0:
                    self.drops['outside_frames'] += 1
                self.frames += 1
                self.last_error = ''
                self.publish(key, 'updated' if inside else 'outside_canvas')
            except (ValueError, KeyError, TypeError, np.linalg.LinAlgError) as error:
                self.drops['invalid'] += 1
                self.last_error = str(error)
                self.get_logger().warning(f'Rejected image: {error}', throttle_duration_sec=5.0)
            self.last_input = key
            self.next_process = wall + 1.0/self.p['process_hz']
            self.drops['superseded_pairs'] += sum(k < key for k in common)
            for cache in (self.images, self.infos):
                for old in list(cache):
                    if old <= key:
                        cache.pop(old)
            return

    def publish(self, source_stamp, status):
        # Unique output stamp within epoch, shared by all three output messages.
        out_stamp = max(source_stamp, self.last_output_stamp+1)
        self.last_output_stamp = out_stamp
        self.version += 1
        rgba = self.mosaic.rgba
        for array, encoding, publisher in (
                (rgba, 'rgba8', self.image_pub),
                (rgba[:, :, 3], 'mono8', self.mask_pub)):
            msg = Image()
            msg.header.frame_id = self.p['target_frame']
            msg.header.stamp.sec = out_stamp // 1_000_000_000
            msg.header.stamp.nanosec = out_stamp % 1_000_000_000
            msg.height, msg.width = array.shape[:2]
            msg.encoding = encoding
            msg.is_bigendian = 0
            msg.step = msg.width * (4 if encoding == 'rgba8' else 1)
            msg.data = np.ascontiguousarray(array).tobytes()
            publisher.publish(msg)
        count = int(np.count_nonzero(rgba[:, :, 3]))
        meta = {
            'schema_version': 1, 'map_kind': 'camera_ground_plane_mosaic',
            'epoch_id': self.epoch, 'map_version': self.version,
            'output_stamp': stamp_dict(out_stamp), 'source_stamp': stamp_dict(source_stamp),
            'frame_id': self.p['target_frame'], 'status': status,
            'width': self.p['width'], 'height': self.p['height'],
            'resolution_m_per_pixel': self.p['resolution'],
            'origin_lower_left': {'x': self.p['origin_x'], 'y': self.p['origin_y'], 'yaw': 0.0},
            'image_axes': 'right=+X, up=+Y; row 0 has highest Y',
            'ground_z': self.p['ground_z'], 'exclude_bottom_fraction': self.p['exclude_bottom'],
            'blend_policy': 'latest_valid_pixel_wins',
            'processed_frames': self.frames, 'valid_pixels': count,
            'valid_area_m2': count*self.p['resolution']**2,
            'last_outside_pixels': self.last_outside, 'drops': self.drops.copy(),
            'last_error': self.last_error,
            'limitations': 'Planar approximation, model-based optical axes, '
                           'no SLAM/orthorectification, mask does not certify free ground.',
        }
        self.last_metadata = meta
        self.meta_pub.publish(String(data=json.dumps(meta, allow_nan=False)))

    def save_service(self, request, response):
        if not self.frames:
            response.success = False
            response.message = 'No processed images; nothing saved'
            return response
        root = Path(self.p['save_directory']).expanduser()
        folder = root / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
                         + '_' + self.epoch[:8])
        try:
            folder.mkdir(parents=True, exist_ok=False)
            PILImage.fromarray(self.mosaic.rgba).save(folder/'bird_view.png')
            PILImage.fromarray(self.mosaic.rgba[:, :, 3]).save(folder/'valid_mask.png')
            (folder/'metadata.json').write_text(
                json.dumps(self.last_metadata, indent=2, allow_nan=False), encoding='utf-8')
            (folder/'COMPLETE').write_text('Saved one coherent map version\n')
            response.success = True
            response.message = str(folder)
        except (OSError, ValueError) as error:
            response.success = False
            response.message = f'Save failed (incomplete output may remain): {error}'
        return response


def main():
    rclpy.init()
    node = CameraBirdMapper()
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
