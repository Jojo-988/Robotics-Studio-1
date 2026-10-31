"""Bounded, exact-stamp pairing for single-session map verification tools."""
from collections import Counter
from copy import deepcopy
import math
import time


def stamp_ns(stamp):
    sec, nano = stamp['sec'], stamp['nanosec']
    if type(sec) is not int or type(nano) is not int or not 0 <= nano < 1_000_000_000:
        raise ValueError('Invalid integer timestamp')
    return sec * 1_000_000_000 + nano


def validate_pair(grid, stats):
    if stamp_ns(grid['header']['stamp']) != stamp_ns(stats['source_stamp']):
        raise ValueError('Map/statistics timestamp mismatch')
    if grid['header']['frame_id'] != stats['frame_id']:
        raise ValueError('Map/statistics frame mismatch')
    info = grid['info']
    for name in ('width', 'height'):
        if type(info[name]) is not int or info[name] <= 0 or info[name] != stats[name]:
            raise ValueError(f'Invalid or mismatched {name}')
    if not math.isclose(info['resolution'], stats['resolution'], rel_tol=0, abs_tol=1e-6):
        raise ValueError('Resolution mismatch')
    if not math.isfinite(info['resolution']) or info['resolution'] <= 0:
        raise ValueError('Invalid resolution')
    for axis in ('x', 'y'):
        if not math.isclose(info['origin']['position'][axis], stats['origin'][axis], rel_tol=0, abs_tol=1e-6):
            raise ValueError(f'Origin {axis} mismatch')
    total = info['width'] * info['height']
    if len(grid['data']) != total:
        raise ValueError('Map data length mismatch')
    counts = Counter(grid['data'])
    if set(counts) - {-1, 0, 100}:
        raise ValueError('Unexpected cell class')
    expected = {'total_cells': total, 'unknown_cells': counts[-1],
                'ground_candidate_cells': counts[0], 'structure_cells': counts[100]}
    for name, value in expected.items():
        if type(stats[name]) is not int or stats[name] != value:
            raise ValueError(f'{name}: map={value}, statistics={stats[name]}')
    fraction = (counts[0] + counts[100]) / total
    if not math.isclose(fraction, stats['classified_fraction'], rel_tol=0, abs_tol=1e-9):
        raise ValueError('Classified fraction mismatch')
    # The class image cannot independently establish the mixed evidence count.
    mixed = stats['mixed_cells']
    if type(mixed) is not int or not 0 <= mixed <= counts[100]:
        raise ValueError('Invalid mixed_cells range')
    return expected


class SnapshotPairs:
    def __init__(self, capacity=32):
        if type(capacity) is not int or capacity <= 0:
            raise ValueError('capacity must be positive')
        self.capacity = capacity
        self.maps, self.statistics = {}, {}

    def _put(self, cache, stamp, message):
        key = stamp_ns(stamp)
        cache[key] = deepcopy(message)
        while len(cache) > self.capacity:
            del cache[min(cache)]

    def add_map(self, grid):
        self._put(self.maps, grid['header']['stamp'], grid)

    def add_statistics(self, stats):
        self._put(self.statistics, stats['source_stamp'], stats)

    def latest_seen(self):
        return max([*self.maps, *self.statistics], default=None)

    def get_pair(self, minimum_stamp=None):
        common = self.maps.keys() & self.statistics.keys()
        if minimum_stamp is not None:
            common = {key for key in common if key >= minimum_stamp}
        if not common:
            return None
        key = max(common)
        grid, stats = self.maps[key], self.statistics[key]
        counts = validate_pair(grid, stats)
        return deepcopy(grid), deepcopy(stats), counts

    def wait(self, spin_once, timeout=10.0, clock=time.monotonic):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive and finite')
        # Do not fall back to an older pair if one stream has advanced.
        minimum = self.latest_seen()
        deadline = clock() + timeout
        while True:
            pair = self.get_pair(minimum)
            if pair is not None:
                return pair
            remaining = deadline - clock()
            if remaining <= 0:
                raise TimeoutError(
                    f'No matched snapshot within {timeout}s; '
                    f'minimum_stamp={minimum}; '
                    f'latest_map={max(self.maps, default=None)}; '
                    f'latest_statistics={max(self.statistics, default=None)}')
            spin_once(min(0.1, remaining))

    def clear(self):
        self.maps.clear()
        self.statistics.clear()
