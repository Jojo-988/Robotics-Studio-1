"""Tests for temporal structure confirmation and reset."""

import unittest

import numpy as np

from map_data.bird_grid import (
    StructureWindow,
    classify_window_counts,
)


class StructureWindowTests(unittest.TestCase):
    def test_sparse_hits_do_not_confirm(self):
        window = StructureWindow(1)

        # Three lifetime hits, but never three within five frames.
        for value in [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]:
            result = window.update(np.array([bool(value)]))

        self.assertFalse(result[0])

    def test_waits_for_full_window(self):
        window = StructureWindow(1)

        for value in [True, True, True, False]:
            result = window.update(np.array([value]))
            self.assertFalse(result[0])

        result = window.update(np.array([False]))
        self.assertTrue(result[0])

    def test_oldest_frame_is_removed(self):
        window = StructureWindow(1)

        for value in [True, True, False, False, False]:
            window.update(np.array([value]))

        np.testing.assert_array_equal(window.counts, [2])

        result = window.update(np.array([True]))

        # The first True has left the window: count remains two.
        np.testing.assert_array_equal(window.counts, [2])
        self.assertFalse(result[0])

    def test_confirmation_is_retained(self):
        window = StructureWindow(1)

        for value in [True, True, True, False, False]:
            window.update(np.array([value]))

        for _ in range(6):
            result = window.update(np.array([False]))

        self.assertTrue(result[0])
        np.testing.assert_array_equal(window.counts, [0])

    def test_reset_clears_all_state(self):
        window = StructureWindow(2)

        for _ in range(5):
            window.update(np.array([True, False]))

        self.assertTrue(window.confirmed[0])

        window.reset()

        self.assertEqual(window.frames_in_window, 0)
        self.assertFalse(window.confirmed.any())
        np.testing.assert_array_equal(window.counts, [0, 0])

        # Evidence from the previous session must not survive.
        result = window.update(np.array([False, True]))
        self.assertFalse(result.any())

    def test_arrays_do_not_expose_internal_state(self):
        window = StructureWindow(1)
        hits = np.array([True])

        window.update(hits)
        hits[0] = False

        np.testing.assert_array_equal(window.counts, [1])

        counts = window.counts
        counts[0] = 99

        confirmed = window.confirmed
        confirmed[0] = True

        np.testing.assert_array_equal(window.counts, [1])
        self.assertFalse(window.confirmed[0])

    def test_invalid_configuration_and_input(self):
        for arguments in [(0, 5, 3), (1, 0, 1), (1, 5, 6),
                          (True, 5, 3), (1, 5, 0)]:
            with self.subTest(arguments=arguments):
                with self.assertRaises(ValueError):
                    StructureWindow(*arguments)

        window = StructureWindow(2)

        with self.assertRaises(ValueError):
            window.update(np.array([True]))

        with self.assertRaises(ValueError):
            window.update(np.array([1, 0]))

        self.assertEqual(window.frames_in_window, 0)

    def test_classification_and_structure_priority(self):
        cells, mixed = classify_window_counts(
            ground_counts=np.array([3, 10, 10, 0]),
            structure_counts=np.array([0, 1, 3, 3]),
            min_ground_frames=3,
            confirmed_structure=np.array(
                [False, False, True, True]
            ),
        )

        np.testing.assert_array_equal(cells, [0, -1, 100, 100])
        np.testing.assert_array_equal(
            mixed, [False, False, True, False]
        )

    def test_confirmation_requires_history(self):
        with self.assertRaises(ValueError):
            classify_window_counts(
                np.array([3]),
                np.array([0]),
                3,
                np.array([True]),
            )


if __name__ == '__main__':
    unittest.main()