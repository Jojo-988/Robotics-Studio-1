"""Regression tests for bounded live sampling; no ROS or flight required."""
import math
import unittest
import numpy as np
from map_data.camera_projection import project_rgb
from map_data.camera_mosaic import CameraMosaic


def fixture(height=1.2, x=2.25, yaw=0.0, tilt=0.0):
    stamp = {'sec': 1, 'nanosec': 0}
    header = {'stamp': stamp, 'frame_id': 'camera'}
    info = {'header': header, 'width': 720, 'height': 480,
            'k': [207.8449214842941, 0, 360, 0, 207.8449214842941, 240, 0, 0, 1],
            'd': [0] * 5}
    meta = {'header': header, 'width': 720, 'height': 480}
    p = math.radians(90 - tilt) / 2
    y = yaw / 2
    pose = {
        'image_stamp': stamp, 'camera_info_stamp': stamp,
        'source_frame': 'camera', 'target_frame': 'odom',
        'optical_convention': {'T_camera_link_from_optical':
            [[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]]},
        'transform': {'header': {'stamp': stamp, 'frame_id': 'odom'},
            'child_frame_id': 'camera', 'transform': {
                'translation': {'x': x, 'y': 0., 'z': height - .15},
                'rotation': {'x': -math.sin(y)*math.sin(p),
                             'y': math.cos(y)*math.sin(p),
                             'z': math.sin(y)*math.cos(p),
                             'w': math.cos(y)*math.cos(p)}}}}
    v, u = np.indices((480, 720))
    rgb = np.stack((u % 251, v % 251, (u + 2*v) % 251), axis=2).astype(np.uint8)
    return rgb, info, pose, meta


class CanvasProjectionTests(unittest.TestCase):
    def check_tile(self, tile, grid):
        rgba, mask, meta = tile
        self.assertEqual(rgba.shape, (grid[3], grid[2], 4))
        np.testing.assert_array_equal(mask, rgba[:, :, 3])
        self.assertTrue(set(np.unique(mask)) <= {0, 255})
        self.assertEqual(meta['valid_pixels'], np.count_nonzero(mask))
        self.assertEqual(meta['projection_domain'], 'fixed_canvas')

    def test_matches_full_footprint_then_clip(self):
        grid = (-2., -4., 80, 80)
        for kwargs in ({}, {'yaw': .6}, {'tilt': 20}, {'x': 5.9}):
            with self.subTest(**kwargs):
                args = fixture(**kwargs)
                old = CameraMosaic(.1, *grid)
                old.add(*project_rgb(*args, resolution=.1))
                bounded = project_rgb(*args, resolution=.1, target_grid=grid)
                self.check_tile(bounded, grid)
                np.testing.assert_array_equal(old.rgba[:, :, 3], bounded[1])
                np.testing.assert_allclose(old.rgba.astype(int), bounded[0].astype(int), atol=1)

    def test_fine_resolution_large_footprint_is_bounded(self):
        args = fixture(height=10)
        with self.assertRaisesRegex(ValueError, 'Output is too large'):
            project_rgb(*args, resolution=.01)
        grid = (-2., -4., 800, 800)
        tile = project_rgb(*args, resolution=.01, target_grid=grid)
        self.check_tile(tile, grid)
        self.assertGreater(tile[2]['valid_pixels'], 0)

    def test_tilted_large_footprint_is_bounded(self):
        args = fixture(height=15, tilt=35)
        with self.assertRaisesRegex(ValueError, 'Output is too large'):
            project_rgb(*args, resolution=.1)
        grid = (-40., -40., 800, 800)
        tile = project_rgb(*args, resolution=.1, target_grid=grid)
        self.check_tile(tile, grid)
        self.assertGreater(tile[2]['valid_pixels'], 0)

    def test_empty_frame_does_not_erase_history(self):
        grid = (-2., -4., 80, 80)
        mosaic = CameraMosaic(.1, *grid)
        mosaic.add(*project_rgb(*fixture(), resolution=.1, target_grid=grid))
        before = mosaic.rgba.copy()
        empty = project_rgb(*fixture(x=100), resolution=.1, target_grid=grid)
        self.assertEqual(empty[2]['valid_pixels'], 0)
        self.assertEqual(mosaic.add(*empty), (0, 0))
        np.testing.assert_array_equal(mosaic.rgba, before)

    def test_height_sequence_preserves_old_valid_pixels(self):
        grid = (-2., -4., 80, 80)
        mosaic = CameraMosaic(.1, *grid)
        for height in (.8, 1., 1.2, 1.5, 2., 1.):
            before = mosaic.rgba.copy()
            tile = project_rgb(*fixture(height=height), resolution=.1, target_grid=grid)
            mosaic.add(*tile)
            np.testing.assert_array_equal(mosaic.rgba[tile[1] == 0], before[tile[1] == 0])
            self.assertFalse(np.any((before[:, :, 3] > 0) & (mosaic.rgba[:, :, 3] == 0)))

    def test_invalid_grid_rejected(self):
        for grid in ((0, 0, 0, 80), (0, 0, 4001, 1), (0, 0, 3000, 3000),
                     (0, 0, 80.5, 80), (0, 0, True, 80),
                     (float('nan'), 0, 80, 80), (.03, 0, 80, 80)):
            with self.subTest(grid=grid), self.assertRaises(ValueError):
                project_rgb(*fixture(), resolution=.1, target_grid=grid)

    def test_horizon_rejection_is_preserved(self):
        with self.assertRaises(ValueError):
            project_rgb(*fixture(height=15, tilt=45), resolution=.1,
                        target_grid=(-40., -40., 800, 800))


if __name__ == '__main__':
    unittest.main()
