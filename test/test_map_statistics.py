"""Small, hand-calculated case for map statistics; no ROS node"""

import json
import math
import unittest
from array import array

from map_data.statistics import calculate_map_statistic

class MapStatisticsTests(unittest.TestCase):
    def test_mixed_map(self):
        result = calculate_map_statistic(2, 2, 0.5, [-1, 0, 100, 50])
        self.assertEqual(result, {'total_cells': 4, 'known_cells': 3, 'unknown_cells': 1,
                                  'free_cells': 1, 'occupied_cells': 1, 'intermediate_cells':1,
                                  'known_fraction': 0.75, 'known_area_m2': 0.75,
                                  })

        self.assertEqual(json.loads(json.dumps(result, allow_nan=False)), result)

    def test_all_unknown(self):
        result = calculate_map_statistic(2, 2, 0.5, [-1] * 4)
        self.assertEqual(result['known_fraction'], 0.0)
        self.assertEqual(result['known_area_m2'], 0.0)

    def test_all_known_and_intermediate_boundaries(self):
        result = calculate_map_statistic(2, 2, 0.5,[0,1,99,100])
        self.assertEqual(result['known_fraction'], 1.0)
        self.assertEqual(result['intermediate_cells'], 2)
        self.assertEqual(result['known_area_m2'], 1.0)

    def test_map_growth_can_reduce_fraction_without_losing_known_area(self):
        before = calculate_map_statistic(2,2,0.5, [0, 0, 0, -1])
        after = calculate_map_statistic(3, 2, 0.5, [0, 0, 0, -1, -1, -1])
        self.assertEqual(before['known_area_m2'], after['known_area_m2'])
        self.assertLess(after['known_fraction'], before['known_fraction'])

    def test_invalid_dimensions_and_data_length(self):
        cases = [
            (0, 2, []),
            (-1, 2, []),
            (1.5,2, [0]* 3),
            (True, 1, [0]),
            (2,2,[0] *3),
        ]

        for width, height, data in cases:
            with self.subTest(width = width, height = height):
                with self.assertRaises(ValueError):
                    calculate_map_statistic(
                        width=width,
                        height=height,
                        resolution=0.5,
                        data =data,
                    )


    def test_invalid_resolution(self):
        for resolution in [0, -0.1, math.nan, math.inf, 1e308, 1e-300]:
            with self.subTest(resolution=resolution):
                with self.assertRaises(ValueError):
                    calculate_map_statistic(1,1, resolution, [0])

    def test_invalid_cell_values(self):
        for value in[-2, 101, 0.5, math.nan, True]:
            with self.subTest(value = value):
                with self.assertRaises(ValueError):
                    calculate_map_statistic(1,1, 0.5, [value])

if __name__ == '__main__':
    unittest.main()
