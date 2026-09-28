
from dataclasses import dataclass
import math
from typing import Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class CoverageConfig:
    x_min: float = -12.5
    x_max: float = 12.5
    y_min: float = -12.5
    y_max: float = 12.5
    footprint_width: float = 6.0
    overlap: float = 0.2
    waypoint_spacing: float = 2.0
    position_tolerance: float = 0.15
    grid_resolution: float = 0.5
    sweep_axis: str = 'auto'

    # Check whether the area configuration is incorrect before running
    def __post_init__(self):
        numbers = [v for v in vars(self).values() if isinstance(v, (float, int))]
        if not all(math.isfinite(v) for v in numbers):
            raise ValueError('Coverage parameters must be finite.')
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise ValueError('Area bounds must satisfy min < max on both axes.')
        for name in ('footprint_width', 'waypoint_spacing', 'position_tolerance',
                     'grid_resolution'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive.')
        if not 0 <= self.overlap < 1:
            raise ValueError('overlap must be in [0, 1).')
        if self.sweep_axis not in ('auto', 'x', 'y'):
            raise ValueError('sweep_axis must be auto, x or y.')
        if self.lane_spacing <= 0:
            raise ValueError('Footprint too small for tracking tolerance/grid resolution.')
        nx = math.ceil((self.x_max - self.x_min) / self.grid_resolution)
        ny = math.ceil((self.y_max - self.y_min) / self.grid_resolution)
        if nx * ny > 250000:
            raise ValueError('Coverage grid exceeds 250000 cells; increase grid_resolution.')

    @property
    def lane_spacing(self):
        # Reserve enough footprint for endpoint error and a whole grid cell.
        usable = (self.footprint_width - 2 * self.position_tolerance
                  - math.sqrt(2) * self.grid_resolution)
        return usable * (1 - self.overlap)


def distance_to_segment(point: Point, start: Point, end: Point) -> float:   # Find the closest point on a line segment using vector projection
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    t = 0.0 if length_sq == 0 else max(0.0, min(1.0, (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq))  # t indicates the proportion of the projection position along the line segment
    return math.hypot(point[0] - start[0] - t * dx, point[1] - start[1] - t * dy)


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def plan_coverage(config: CoverageConfig, start: Point = (0.0, 0.0)):
    """Choose axis and entry corner by total sweep + initial transit length."""
    if not all(math.isfinite(v) for v in start):
        raise ValueError('Start position must be finite.')
    candidates = []
    axes = ('x', 'y') if config.sweep_axis == 'auto' else (config.sweep_axis,)
    for axis in axes:
        along = (config.x_min, config.x_max) if axis == 'x' else (config.y_min, config.y_max)   # fly follow the scan line
        across = (config.y_min, config.y_max) if axis == 'x' else (config.x_min, config.x_max)  # change to the next scan line
        intervals = max(1, math.ceil((across[1] - across[0]) / config.lane_spacing))    # calcualte the maxmum gaps between scan lines
        if intervals > 10000:
            raise ValueError('Route exceeds 10000 scan intervals; reduce overlap or increase footprint.')
        for reverse_rows in (False, True):
            for reverse_first in (False, True):
                corners = []
                rows = range(intervals, -1, -1) if reverse_rows else range(intervals + 1)
                for i, row in enumerate(rows):
                    cross = across[0] + row * (across[1] - across[0]) / intervals   # set the location of scan lines
                    ends = along[::-1] if (i % 2 == 1) != reverse_first else along  # decided to start from which side
                    for end in ends:
                        corners.append((end, cross) if axis == 'x' else (cross, end))
                cost = math.dist(start, corners[0]) + path_length(corners)  # The length from current position to the start position + the whole length of the scan line
                candidates.append((cost, corners))
    corners = min(candidates, key=lambda item: item[0])[1]
    points = [corners[0]]
    for a, b in zip(corners, corners[1:]):
        count = max(1, math.ceil(math.dist(a, b) / config.waypoint_spacing)) # Trans the scan line to be the position points
        if len(points) + count > 100000:
            raise ValueError('Route exceeds 100000 waypoints; increase spacing.')
        points.extend((a[0] + (b[0] - a[0]) * j / count,
                       a[1] + (b[1] - a[1]) * j / count) for j in range(1, count + 1))  # the mid point = start + the whole movement * the percentage of fly for now 
    return points


class CoverageGrid:
    """Conservative whole-cell coverage from actual, contiguous flight segments.

    Mark a cell only if its centre is within radius - half-cell-diagonal of a
    flown segment. This guarantees the entire cell is inside the model footprint.
    Equal-sized cells give an area-weighted fraction, including boundary cells.
    """

    def __init__(self, config: CoverageConfig):
        self.config = config
        self.nx = math.ceil((config.x_max - config.x_min) / config.grid_resolution)
        self.ny = math.ceil((config.y_max - config.y_min) / config.grid_resolution)
        self.dx = (config.x_max - config.x_min) / self.nx
        self.dy = (config.y_max - config.y_min) / self.ny
        self.radius = config.footprint_width / 2 - math.hypot(self.dx, self.dy) / 2
        self.visited = set()

    def mark_segment(self, start: Point, end: Point):
        c, r = self.config, self.radius
        ix0 = max(0, math.floor((min(start[0], end[0]) - r - c.x_min) / self.dx))
        ix1 = min(self.nx - 1, math.floor((max(start[0], end[0]) + r - c.x_min) / self.dx))
        iy0 = max(0, math.floor((min(start[1], end[1]) - r - c.y_min) / self.dy))
        iy1 = min(self.ny - 1, math.floor((max(start[1], end[1]) + r - c.y_min) / self.dy))
        for iy in range(iy0, iy1 + 1):
            for ix in range(ix0, ix1 + 1):
                cell = iy * self.nx + ix
                if cell not in self.visited:
                    centre = (c.x_min + (ix + 0.5) * self.dx, c.y_min + (iy + 0.5) * self.dy)
                    if distance_to_segment(centre, start, end) <= r + 1e-9:
                        self.visited.add(cell)

    @property
    def fraction(self):
        return len(self.visited) / (self.nx * self.ny)  # calculate the coverage