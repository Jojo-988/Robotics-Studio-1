import unittest
from copy import deepcopy
from map_data.snapshot_pairing import SnapshotPairs, validate_pair


def messages(sec=1):
    stamp = {'sec': sec, 'nanosec': 123}
    grid = {'header': {'stamp': stamp, 'frame_id': 'parrot1_odom'},
            'info': {'width': 3, 'height': 1, 'resolution': .1,
                     'origin': {'position': {'x': -16., 'y': -16.}}},
            'data': [-1, 0, 100]}
    stats = {'source_stamp': stamp, 'frame_id': 'parrot1_odom',
             'width': 3, 'height': 1, 'resolution': .1,
             'origin': {'x': -16., 'y': -16.},
             'total_cells': 3, 'unknown_cells': 1,
             'ground_candidate_cells': 1, 'structure_cells': 1,
             'classified_fraction': 2/3, 'mixed_cells': 0}
    return grid, stats


class PairingTests(unittest.TestCase):
    def test_both_arrival_orders(self):
        for stats_first in (True, False):
            p = SnapshotPairs(); g, s = messages()
            if stats_first:
                p.add_statistics(s); self.assertIsNone(p.get_pair()); p.add_map(g)
            else:
                p.add_map(g); self.assertIsNone(p.get_pair()); p.add_statistics(s)
            self.assertEqual(p.get_pair()[2]['structure_cells'], 1)

    def test_does_not_mix_adjacent_frames(self):
        p = SnapshotPairs(); g, _ = messages(1); _, s = messages(2)
        p.add_map(g); p.add_statistics(s)
        self.assertIsNone(p.get_pair())

    def test_counts_mismatch_fails(self):
        p = SnapshotPairs(); g, s = messages(); s['structure_cells'] = 2
        p.add_map(g); p.add_statistics(s)
        with self.assertRaises(ValueError):
            p.get_pair()

    def test_metadata_and_fraction_mismatch(self):
        g, s = messages()
        for field, value in [('frame_id', 'wrong'), ('resolution', .2),
                             ('classified_fraction', 1), ('mixed_cells', 2)]:
            modified = deepcopy(s); modified[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_pair(g, modified)

    def test_timeout_instead_of_old_pair(self):
        p = SnapshotPairs(); g, s = messages(); p.add_map(g); p.add_statistics(s)
        _, newer = messages(2); p.add_statistics(newer)
        now = [0.]
        def spin(dt): now[0] += dt
        with self.assertRaises(TimeoutError):
            p.wait(spin, timeout=.5, clock=lambda: now[0])

    def test_wait_accepts_delayed_match(self):
        p = SnapshotPairs(); g, s = messages(2); p.add_statistics(s)
        pair = p.wait(lambda dt: p.add_map(g))
        self.assertEqual(pair[0]['header']['stamp']['sec'], 2)

    def test_capacity_copies_and_clear(self):
        p = SnapshotPairs(capacity=2)
        for sec in (1, 3, 2):
            g, s = messages(sec); p.add_map(g); p.add_statistics(s)
        self.assertEqual(len(p.maps), 2)
        pair = p.get_pair(); pair[0]['data'][0] = 100
        self.assertEqual(p.get_pair()[0]['data'][0], -1)
        p.clear(); self.assertIsNone(p.get_pair())


if __name__ == '__main__':
    unittest.main()
