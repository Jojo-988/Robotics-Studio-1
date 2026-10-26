"""Statistics for this project's SLAM map encoding: -1 and 0..100"""

import math
from numbers import Integral

def calculate_map_statistic(width, height, resolution,data):
    """return counts and area, or reject invalid input with ValueError"""
    if any(isinstance(value, bool) or not isinstance(value, Integral) or value <= 0
           for value in(width, height)):
        raise ValueError('Map width and height must be positive')

    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError(
            'Map resolution must be finite and positive'
        )
    total_cells = width * height
    if len(data) != total_cells:
        raise ValueError('Map data length does not match widthh* height')

    cell_area = resolution * resolution
    if not math.isfinite(total_cells *cell_area) or cell_area <= 0:
        raise ValueError('Map area is outside thr suppported numeric range')

    unknown_cells = 0
    free_cells = 0
    occupied_cells = 0
    intermediate_cells = 0

    for value in data:
        if isinstance(value, bool) or not isinstance(value, Integral):
            raise ValueError('Map cells must be integer values')
        if value == -1:
            unknown_cells +=1
        elif value ==0:
            free_cells +=1
        elif value == 100:
            occupied_cells +=1
        elif 1<= value <= 99:
            intermediate_cells += 1
        else:
            raise ValueError(f'Unsupported SLAM cell value: {value}')

    known_cells = total_cells - unknown_cells
    return{
        'total_cells': int(total_cells),
        'known_cells': known_cells,
        'unknown_cells': unknown_cells,
        'free_cells': free_cells,
        'occupied_cells': occupied_cells,
        'intermediate_cells': intermediate_cells,
        'known_fraction': known_cells / total_cells,
        'known_area_m2': known_cells * cell_area,

    }