"""Bounded, ROS-independent validation of camera, lidar and IMU payloads."""
from dataclasses import dataclass, field
import math

import numpy as np


def imu_quality(msg):
    vectors = (msg.angular_velocity, msg.linear_acceleration)
    values = [getattr(v, axis) for v in vectors for axis in 'xyz']
    if (msg.angular_velocity_covariance[0] == -1 or msg.linear_acceleration_covariance[0] == -1
            or not all(math.isfinite(v) for v in values)):
        return False, 'IMU angular velocity / acceleration unavailable or non-finite', {}
    acceleration = math.hypot(*values[3:])
    if not math.isfinite(acceleration):
        return False, 'IMU acceleration magnitude overflow', {}
    metrics = {'acceleration_m_s2': acceleration}
    if msg.orientation_covariance[0] != -1:
        q = msg.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in quaternion):
            return False, 'Non-finite IMU orientation', {}
        norm = math.sqrt(sum(v * v for v in quaternion))
        if abs(norm - 1.0) > 0.05:
            return False, 'IMU orientation quaternion is not normalized', {}
        metrics['quaternion_norm'] = norm
    return True, '', metrics


def image_quality(msg, *, depth=False, minimum=0.4, maximum=40.0, fraction=0.05):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    if depth:
        formats = {'32FC1': ('f4', 4, 1.0), '16UC1': ('u2', 2, 0.001)}
        if msg.encoding not in formats:
            return False, 'Unsupported depth encoding: ' + msg.encoding, {}
        kind, size, scale = formats[msg.encoding]
    else:
        if msg.encoding not in channels:
            return False, 'Unsupported RGB encoding: ' + msg.encoding, {}
        size = channels[msg.encoding]
    if (msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * size
            or len(msg.data) != msg.height * msg.step):
        return False, 'Malformed image dimensions, stride or payload', {}
    if not depth:
        # A dark/uniform image is still a valid image; quality of recognition is separate.
        return True, '', {'width': msg.width, 'height': msg.height, 'encoding': msg.encoding}
    pixels = np.ndarray((msg.height, msg.width),
                        dtype=np.dtype(('>' if msg.is_bigendian else '<') + kind),
                        buffer=bytes(msg.data), strides=(msg.step, size)).astype(float) * scale
    valid = np.isfinite(pixels) & (pixels >= minimum) & (pixels <= maximum)
    ratio = float(valid.mean())
    metrics = {'valid_fraction': ratio, 'width': msg.width, 'height': msg.height,
               'encoding': msg.encoding}
    if valid.any():
        metrics.update(min_m=float(pixels[valid].min()), max_m=float(pixels[valid].max()))
    return ratio >= fraction, '' if ratio >= fraction else 'Too few usable depth pixels', metrics


def cloud_quality(msg, *, minimum=0.2, maximum=40.0, fraction=0.05):
    if (msg.width <= 0 or msg.height <= 0 or msg.point_step <= 0
            or msg.row_step < msg.width * msg.point_step
            or len(msg.data) != msg.height * msg.row_step):
        return False, 'Malformed point cloud dimensions, stride or payload', {}
    fields = {f.name: f for f in msg.fields}
    coordinates = []
    payload = bytes(msg.data)
    for name in ('x', 'y', 'z'):
        f = fields.get(name)
        # PointField.FLOAT32=7, FLOAT64=8. Honor offsets, endianness and row padding.
        if f is None or f.datatype not in (7, 8) or f.count != 1:
            return False, 'Point cloud requires scalar floating-point x/y/z fields', {}
        size = 4 if f.datatype == 7 else 8
        if f.offset < 0 or f.offset + size > msg.point_step:
            return False, 'Point field exceeds point_step', {}
        coordinates.append(np.ndarray(
            (msg.height, msg.width), dtype=np.dtype(('>' if msg.is_bigendian else '<') + f'f{size}'),
            buffer=payload, offset=f.offset, strides=(msg.row_step, msg.point_step)).astype(float))
    with np.errstate(invalid='ignore', over='ignore'):
        ranges = np.sqrt(sum(c * c for c in coordinates))
    valid = np.isfinite(ranges) & (ranges >= minimum) & (ranges <= maximum)
    ratio = float(valid.mean())
    metrics = {'valid_fraction': ratio, 'points': msg.width * msg.height}
    if valid.any():
        metrics.update(min_m=float(ranges[valid].min()), max_m=float(ranges[valid].max()))
    return ratio >= fraction, '' if ratio >= fraction else 'Too few usable lidar returns', metrics


@dataclass
class StreamHealth:
    stamp: float = None
    received: float = None
    max_gap_wall: float = 0.0
    consecutive: int = 0
    accepted: int = 0
    rejected: int = 0
    reason: str = 'No data received'
    metrics: dict = field(default_factory=dict)
    last_rejection: dict = field(default_factory=dict)

    def update(self, stamp, wall, valid=True, reason='', metrics=None):
        self.metrics = metrics or {}
        if not math.isfinite(stamp) or stamp < 0:
            valid, reason = False, 'Invalid timestamp'
        elif self.stamp is not None and stamp <= self.stamp:
            valid, reason = False, 'Timestamp did not advance'
        if not valid:
            self.consecutive = 0
            self.rejected += 1
            self.reason = reason
            self.last_rejection = {'reason': reason, 'stamp': stamp if math.isfinite(stamp) else None,
                                   'previous_stamp': self.stamp,
                                   'metrics': self.metrics}
            return
        if self.received is not None:
            self.max_gap_wall = max(self.max_gap_wall, wall - self.received)
        self.stamp, self.received = stamp, wall
        self.consecutive += 1
        self.accepted += 1
        self.reason = ''

    def issue(self, now, wall, timeout, samples=2):
        if self.reason:
            return self.reason
        if self.received is None or wall - self.received > timeout:
            self.consecutive = 0
            return 'Data stopped arriving (wall-clock timeout)'
        if abs(now - self.stamp) > timeout:
            self.consecutive = 0
            return 'Timestamp is outside the current ROS clock window'
        if self.consecutive < samples:
            return f'Waiting for {samples} consecutive valid samples'
        return ''

    def report(self, now, wall, timeout, samples=2):
        issue = self.issue(now, wall, timeout, samples)
        return {'ready': not bool(issue), 'reason': issue, 'accepted': self.accepted,
                'rejected': self.rejected, 'consecutive': self.consecutive,
                'age_wall_s': None if self.received is None else max(0.0, wall - self.received),
                'age_ros_s': None if self.stamp is None else now - self.stamp,
                'max_gap_wall_s': self.max_gap_wall,
                'last_rejection': self.last_rejection,
                'metrics': self.metrics}
