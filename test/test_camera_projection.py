"""Shared projector and mosaic tests, independent of running ROS."""
import unittest
import numpy as np
from map_data.camera_projection import self_test
from map_data.camera_mosaic import CameraMosaic


class CameraProjectionTests(unittest.TestCase):
    def test_known_geometry(self):
        self_test()

    @staticmethod
    def tile(x=0, y=2):
        rgba = np.full((2, 2, 4), 100, dtype=np.uint8)
        rgba[:, :, 3] = 255
        return rgba, np.full((2, 2), 255, np.uint8), {
            'resolution_m_per_pixel': 1.0,
            'bounds': {'xmin': x, 'ymax': y},
        }

    def test_placement_and_invalid_pixels_preserve_history(self):
        m = CameraMosaic(1.0, 0, 0, 4, 4)
        rgba, mask, meta = self.tile()
        self.assertEqual(m.add(rgba, mask, meta), (4, 0))
        self.assertEqual(m.rgba[2, 0, 0], 100)
        rgba[:, :, :3] = 220
        rgba[0, 0, 3] = 0
        mask[0, 0] = 0
        m.add(rgba, mask, meta)
        self.assertEqual(m.rgba[2, 0, 0], 100)
        self.assertEqual(m.rgba[2, 1, 0], 220)
        m.clear()
        self.assertFalse(m.rgba.any())

    def test_clipping_and_outside(self):
        m = CameraMosaic(1.0, 0, 0, 4, 4)
        self.assertEqual(m.add(*self.tile(x=-1)), (2, 2))
        self.assertEqual(m.add(*self.tile(x=10)), (0, 4))

    def test_invalid_geometry(self):
        with self.assertRaises(ValueError):
            CameraMosaic(1.0, 0.3, 0, 4, 4)
        with self.assertRaises(ValueError):
            CameraMosaic(1.0, 0, 0, 10000, 10000)


if __name__ == '__main__':
    unittest.main()
