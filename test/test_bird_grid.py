"""Tests for bird's-eye grid projection and classification."""

import unittest

import numpy as np

from map_data.bird_grid import (
    project_hits,
    classify_counts,
    summarize_cells,
)


class BirdGridTests(unittest.TestCase):
    def test_repeated_points_are_one_hit(self):
        points = np.array([
            [0.1, 0.1, 0.0],
            [0.2, 0.2, 0.0],
            [0.3, 0.3, 0.0],
        ])

        ground, structure, outside = project_hits(
            points,
            [True, True, True],
            [False, False, False],
            1.0, 0.0, 0.0, 2, 2,
        )

        self.assertEqual(int(ground.sum()), 1)
        self.assertEqual(int(structure.sum()), 0)
        self.assertEqual(outside, 0)

    def test_upper_boundary_and_negative_coordinates(self):
        points = np.array([
            [-1.0, -1.0, 0.0],
            [0.99, 0.99, 0.0],
            [1.0, 0.0, 0.0],
            [-1.01, 0.0, 0.0],
        ])

        ground, _, outside = project_hits(
            points,
            [True] * 4,
            [False] * 4,
            1.0, -1.0, -1.0, 2, 2,
        )

        np.testing.assert_array_equal(
            ground, [True, False, False, True]
        )
        self.assertEqual(outside, 2)

    def test_thresholds_and_structure_priority(self):
        cells, mixed = classify_counts(
            np.array([0, 2, 3, 10, 10]),
            np.array([0, 0, 0, 1, 3]),
            3, 3,
        )

        # Tentative structure remains unknown in this conservative version.
        np.testing.assert_array_equal(
            cells, [-1, -1, 0, -1, 100]
        )
        np.testing.assert_array_equal(
            mixed, [False, False, False, False, True]
        )

    def test_empty_counts_produce_unknown(self):
        cells, mixed = classify_counts(
            np.zeros(4, dtype=np.uint32),
            np.zeros(4, dtype=np.uint32),
            3, 3,
        )

        np.testing.assert_array_equal(cells, [-1] * 4)
        self.assertFalse(mixed.any())

    def test_summary_does_not_double_count(self):
        summary = summarize_cells([-1, 0, 100, 0])

        self.assertEqual(summary['total_cells'], 4)
        self.assertEqual(summary['ground_candidate_cells'], 2)
        self.assertEqual(summary['structure_cells'], 1)
        self.assertEqual(summary['unknown_cells'], 1)
        self.assertAlmostEqual(
            summary['classified_fraction'], 0.75
        )

    def test_reject_nonfinite_point(self):
        with self.assertRaises(ValueError):
            project_hits(
                [[float('nan'), 0.0, 0.0]],
                [True], [False],
                1.0, 0.0, 0.0, 2, 2,
            )


if __name__ == '__main__':
    unittest.main()