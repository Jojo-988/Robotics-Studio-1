"""Pure grid operations for a LiDAR bird's-eye classification map."""

from collections import deque
import numpy as np


def project_hits(points, ground_mask, structure_mask,
                 resolution, origin_x, origin_y, width, height):
    """Convert classified points into one boolean hit per cell."""
    points = np.asarray(points, dtype=np.float64)
    ground_mask = np.asarray(ground_mask, dtype=bool)
    structure_mask = np.asarray(structure_mask, dtype=bool)

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')

    if ground_mask.shape != (len(points),):
        raise ValueError('ground_mask must match the point count')

    if structure_mask.shape != (len(points),):
        raise ValueError('structure_mask must match the point count')

    if not np.isfinite(points).all():
        raise ValueError('points must contain only finite values')

    if not np.isfinite([resolution, origin_x, origin_y]).all():
        raise ValueError('Grid geometry must be finite')

    if resolution <= 0:
        raise ValueError('resolution must be positive')

    if any(type(value) is not int or value <= 0
           for value in (width, height)):
        raise ValueError('width and height must be positive integers')

    # Check bounds before converting coordinates into integer indices.
    inside = (
        (points[:, 0] >= origin_x)
        & (points[:, 0] < origin_x + width * resolution)
        & (points[:, 1] >= origin_y)
        & (points[:, 1] < origin_y + height * resolution)
    )

    selected = points[inside]
    columns = np.floor(
        (selected[:, 0] - origin_x) / resolution
    ).astype(np.int64)
    rows = np.floor(
        (selected[:, 1] - origin_y) / resolution
    ).astype(np.int64)

    # Guard against floating-point rounding at an upper boundary.
    index_valid = (
        (columns >= 0) & (columns < width)
        & (rows >= 0) & (rows < height)
    )
    indices = rows[index_valid] * width + columns[index_valid]

    ground_hits = np.zeros(width * height, dtype=bool)
    structure_hits = np.zeros(width * height, dtype=bool)

    ground_hits[indices[ground_mask[inside][index_valid]]] = True
    structure_hits[indices[structure_mask[inside][index_valid]]] = True

    outside_points = (
        int(np.count_nonzero(~inside))
        + int(np.count_nonzero(~index_valid))
    )
    return ground_hits, structure_hits, outside_points


def classify_counts(ground_counts, structure_counts,
                    min_ground_frames, min_structure_frames):
    """Produce conservative classifications, not occupancy probabilities."""
    ground_counts = np.asarray(ground_counts)
    structure_counts = np.asarray(structure_counts)

    if (
        ground_counts.ndim != 1
        or ground_counts.shape != structure_counts.shape
    ):
        raise ValueError('Count arrays must be matching 1-D arrays')

    for counts in (ground_counts, structure_counts):
        if not np.issubdtype(counts.dtype, np.integer):
            raise ValueError('Counts must be integers')
        if np.any(counts < 0):
            raise ValueError('Counts must be non-negative')

    for threshold in (min_ground_frames, min_structure_frames):
        if type(threshold) is not int or threshold <= 0:
            raise ValueError('Thresholds must be positive integers')

    supported_structure = structure_counts >= min_structure_frames
    supported_ground = (
        (ground_counts >= min_ground_frames)
        & (structure_counts == 0)
    )
    supported_mixed = (
        (ground_counts >= min_ground_frames)
        & supported_structure
    )

    cells = np.full(ground_counts.shape, -1, dtype=np.int8)
    cells[supported_ground] = 0
    cells[supported_structure] = 100

    return cells, supported_mixed


def summarize_cells(cells):
    """Count the three published classes."""
    cells = np.asarray(cells)

    if cells.ndim != 1 or cells.size == 0:
        raise ValueError('cells must be a non-empty 1-D array')

    if not np.isin(cells, [-1, 0, 100]).all():
        raise ValueError('Unexpected cell classification')

    ground = int(np.count_nonzero(cells == 0))
    structure = int(np.count_nonzero(cells == 100))
    unknown = int(np.count_nonzero(cells == -1))

    return {
        'total_cells': int(cells.size),
        'ground_candidate_cells': ground,
        'structure_cells': structure,
        'unknown_cells': unknown,
        'classified_fraction': (ground + structure) / int(cells.size),
    }


class StructureWindow:
    """Confirm structures using a rolling window of frame hits."""

    def __init__(self, cell_count, window_frames=5, min_hits=3):
        for name, value in (
            ('cell_count', cell_count),
            ('window_frames', window_frames),
            ('min_hits', min_hits),
        ):
            if type(value) is not int or value <= 0:
                raise ValueError(f'{name} must be a positive integer')

        if min_hits > window_frames:
            raise ValueError('min_hits must not exceed window_frames')

        self.cell_count = cell_count
        self.window_frames = window_frames
        self.min_hits = min_hits

        self._history = deque()
        self._counts = np.zeros(cell_count, dtype=np.int64)
        self._confirmed = np.zeros(cell_count, dtype=bool)

    def update(self, structure_hits):
        """Accept one valid frame and return a copy of confirmed cells."""
        hits = np.asarray(structure_hits)

        if hits.shape != (self.cell_count,):
            raise ValueError('structure_hits must match the cell count')

        if hits.dtype != np.dtype(bool):
            raise ValueError('structure_hits must be a boolean array')

        # Own the stored data: the caller may reuse its input array.
        hits = hits.copy()

        if len(self._history) == self.window_frames:
            oldest = self._history.popleft()
            self._counts -= oldest

        self._history.append(hits)
        self._counts += hits

        # Match the offline experiment: wait for a complete window.
        if len(self._history) == self.window_frames:
            self._confirmed |= self._counts >= self.min_hits

        return self.confirmed

    @property
    def confirmed(self):
        """Return a copy so callers cannot modify internal state."""
        return self._confirmed.copy()

    @property
    def counts(self):
        """Return the current window hit counts."""
        return self._counts.copy()

    @property
    def frames_in_window(self):
        return len(self._history)

    def reset(self):
        """Clear both recent evidence and retained confirmations."""
        self._history.clear()
        self._counts.fill(0)
        self._confirmed.fill(False)


def classify_window_counts(
    ground_counts,
    structure_counts,
    min_ground_frames,
    confirmed_structure,
):
    """Combine retained structure confirmations with ground evidence."""
    # Reuse existing validation for count arrays and ground threshold.
    cells, _ = classify_counts(
        ground_counts,
        structure_counts,
        min_ground_frames,
        1,
    )

    confirmed = np.asarray(confirmed_structure)

    if confirmed.shape != cells.shape:
        raise ValueError('confirmed_structure must match count arrays')

    if confirmed.dtype != np.dtype(bool):
        raise ValueError('confirmed_structure must be a boolean array')

    structure_counts = np.asarray(structure_counts)
    ground_counts = np.asarray(ground_counts)

    if np.any(confirmed & (structure_counts == 0)):
        raise ValueError(
            'Confirmed structures require historical structure evidence'
        )

    # A historical hit alone is insufficient for window confirmation.
    cells[structure_counts > 0] = -1
    cells[confirmed] = 100

    mixed = confirmed & (ground_counts >= min_ground_frames)

    return cells, mixed