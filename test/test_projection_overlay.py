"""Offline synthetic alignment tests; importing this module does not run tests."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image


class ProjectionOverlayTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = root = Path(temporary.name)
        for name, xmin in [('a', 0), ('b', 2)]:
            folder = root / name
            folder.mkdir()
            # Equal colours at equal world coordinates, despite different image origins.
            yy, xx = np.indices((6, 8))
            rgb = np.stack(((xx+xmin)*20, yy*30, np.full_like(xx, 80)), axis=2)
            rgba = np.dstack((rgb, np.full_like(xx, 255))).astype(np.uint8)
            Image.fromarray(rgba).save(folder / 'bird_view.png')
            Image.fromarray(np.full((6, 8), 255, np.uint8)).save(folder / 'valid_mask.png')
            meta = {
                'width': 8, 'height': 6, 'frame_id': 'test_map',
                'source_stamp': {'sec': 1 if name == 'a' else 2, 'nanosec': 0},
                'image_axes': 'right=+X, up=+Y', 'resolution_m_per_pixel': 0.01,
                'origin_lower_left': {'x': xmin*.01, 'y': 0, 'yaw': 0},
                'bounds': {'xmin': xmin*.01, 'xmax': (xmin+8)*.01,
                           'ymin': 0, 'ymax': .06},
                'ground_z': 0, 'optical_convention': {}, 'exclude_bottom_fraction': 0,
                'camera_optical_center': [xmin*.01, 0, 1],
            }
            (folder / 'metadata.json').write_text(json.dumps(meta))

        self.tool = (
            Path(__file__).resolve().parents[1]
            / 'scripts'
            / 'compare_camera_projections.py'
        )

    def run_comparison(self, second, output):
        return subprocess.run(
            [sys.executable, str(self.tool), '--a', str(self.root / 'a'),
             '--b', str(self.root / second), '--output', str(self.root / output)],
            capture_output=True, text=True, timeout=30,
        )

    def test_world_alignment_and_overlap(self):
        completed = self.run_comparison('b', 'out')
        self.assertEqual(completed.returncode, 0, completed.stderr)
        result = json.loads((self.root / 'out' / 'comparison.json').read_text())
        self.assertEqual(result['canvas_width'], 10)
        self.assertEqual(result['canvas_height'], 6)
        self.assertEqual(result['offset_b_column_row'], [2, 0])
        self.assertEqual(result['overlap_pixels'], 36)
        self.assertEqual(result['overlap_mean_absolute_rgb_difference_0_255'], 0)
        self.assertTrue((self.root / 'out' / 'blink.gif').is_file())

    def test_same_sample_rejected(self):
        completed = self.run_comparison('a', 'bad')
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn('Same image timestamp', completed.stderr)


if __name__ == '__main__':
    unittest.main()
