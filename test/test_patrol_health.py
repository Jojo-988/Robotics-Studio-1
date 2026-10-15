"""Sensor payloads may be present but unusable: exercise those failure modes."""
from types import SimpleNamespace as Msg
import unittest

import numpy as np

from patrol.health import StreamHealth, cloud_quality, image_quality, imu_quality


class SensorQualityTests(unittest.TestCase):
    def test_imu_normalization_and_unavailable_measurements(self):
        msg = Msg(orientation=Msg(x=0, y=0, z=0, w=1), orientation_covariance=[0] * 9,
                  angular_velocity=Msg(x=0, y=0, z=0), angular_velocity_covariance=[0] * 9,
                  linear_acceleration=Msg(x=0, y=0, z=9.81), linear_acceleration_covariance=[0] * 9)
        self.assertTrue(imu_quality(msg)[0])
        msg.orientation.w = 0
        self.assertFalse(imu_quality(msg)[0])
        msg.orientation_covariance[0] = -1  # Message explicitly omits orientation.
        self.assertTrue(imu_quality(msg)[0])
        msg.linear_acceleration.x = float('nan')
        self.assertFalse(imu_quality(msg)[0])

    def depth(self, data, encoding='32FC1', big=False, padding=0):
        array = np.array(data, dtype=('>' if big else '<') + ('f4' if encoding == '32FC1' else 'u2'))
        payload = b''.join(row.tobytes() + b'\0' * padding for row in array)
        return Msg(height=array.shape[0], width=array.shape[1], encoding=encoding,
                   is_bigendian=big, step=array.shape[1] * array.itemsize + padding, data=payload)

    def test_depth_15m_is_valid_with_padding_and_endianness(self):
        for big in (False, True):
            valid, _, metrics = image_quality(self.depth([[15, 16], [17, 18]], big=big, padding=8), depth=True)
            self.assertTrue(valid)
            self.assertEqual(metrics['valid_fraction'], 1)
            self.assertEqual(metrics['max_m'], 18)

    def test_millimetre_depth_and_invalid_returns(self):
        valid, _, metrics = image_quality(self.depth([[15000, 0]], '16UC1'), depth=True)
        self.assertTrue(valid)
        self.assertEqual(metrics['min_m'], 15)
        self.assertEqual(metrics['valid_fraction'], .5)
        self.assertFalse(image_quality(self.depth([[float('nan'), float('inf'), 0, 41]]), depth=True)[0])

    def test_truncated_image_and_unsupported_encoding(self):
        msg = self.depth([[15]])
        msg.data = msg.data[:-1]
        self.assertFalse(image_quality(msg, depth=True)[0])
        msg.encoding = 'bogus'
        self.assertFalse(image_quality(msg, depth=True)[0])
        # Uniform black is a structurally valid RGB frame.
        self.assertTrue(image_quality(Msg(width=2, height=2, encoding='rgb8', step=6, data=b'\0' * 12))[0])

    def test_cloud_field_offsets_row_padding_and_nonfinite_points(self):
        fields = [Msg(name=n, offset=o, count=1, datatype=7) for n, o in zip('xyz', (4, 8, 12))]
        row = np.array([99, 3, 4, 0, 99, np.nan, np.inf, 0], dtype='>f4').tobytes() + b'\0' * 8
        msg = Msg(width=2, height=1, point_step=16, row_step=40, fields=fields,
                  is_bigendian=True, data=row)
        valid, _, metrics = cloud_quality(msg)
        self.assertTrue(valid)
        self.assertEqual(metrics['valid_fraction'], .5)
        self.assertEqual(metrics['min_m'], 5)
        msg.fields[0].offset = 16
        self.assertFalse(cloud_quality(msg)[0])

    def test_missing_xyz_or_empty_cloud_fails(self):
        msg = Msg(width=1, height=1, point_step=12, row_step=12, fields=[], is_bigendian=False, data=b'\0' * 12)
        self.assertFalse(cloud_quality(msg)[0])
        msg.width = 0
        self.assertFalse(cloud_quality(msg)[0])

    def test_frozen_reversed_stale_and_future_timestamps(self):
        stream = StreamHealth()
        self.assertTrue(stream.issue(1, 1, 1))
        stream.update(1, 1)
        stream.update(1.1, 1.1)
        self.assertFalse(stream.issue(1.1, 1.1, 1))
        stream.update(1.1, 1.2)
        self.assertTrue(stream.issue(1.2, 1.2, 1))
        self.assertEqual(stream.received, 1.1)
        stream.update(1.0, 1.3)
        self.assertEqual(stream.rejected, 2)
        stream.update(1.4, 1.4)
        stream.update(1.5, 1.5)
        self.assertTrue(stream.issue(1.5, 3, 1))
        stream.update(9, 3.1)
        stream.update(9.1, 3.2)
        self.assertTrue(stream.issue(3.2, 3.2, 1))


if __name__ == '__main__':
    unittest.main()
