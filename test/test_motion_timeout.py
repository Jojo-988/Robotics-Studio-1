import unittest
from map_data.motion_timeout import MotionTimeout


class TimeoutTests(unittest.TestCase):
    def test_slow_simulation_survives_old_wall_limit(self):
        guard = MotionTimeout([100, 0], 0.)
        for tick in range(1, 1201):
            # 60 wall seconds, 12 simulation seconds; fresh samples throughout.
            guard.check([100 + tick // 100, (tick % 100) * 10_000_000], tick * .05)
        self.assertAlmostEqual(guard.sim_elapsed, 12.)
        self.assertAlmostEqual(guard.wall_elapsed, 60.)

    def test_stalled_stamp_even_with_repeated_messages(self):
        guard = MotionTimeout([100, 0], 0.)
        guard.check([100, 0], .5)
        with self.assertRaisesRegex(RuntimeError, 'ODOMETRY_TIME_STALLED'):
            guard.check([100, 0], 1.)

    def test_backward_stamp(self):
        guard = MotionTimeout([100, 0], 0.)
        with self.assertRaisesRegex(RuntimeError, 'ODOMETRY_TIME_BACKWARDS'):
            guard.check([99, 999999999], .1)

    def test_simulation_budget(self):
        guard = MotionTimeout([100, 0], 0.)
        with self.assertRaisesRegex(RuntimeError, 'SIM_TIMEOUT'):
            guard.check([145, 0], .1)

    def test_wall_cap_despite_advancing_simulation(self):
        guard = MotionTimeout([100, 0], 0.)
        for tick in range(1, 360):
            guard.check([100, tick * 1_000_000], tick * .5)
        with self.assertRaisesRegex(RuntimeError, 'WALL_TIMEOUT'):
            guard.check([100, 360_000_000], 180.)

    def test_nanosecond_rollover(self):
        guard = MotionTimeout([100, 990000000], 0.)
        guard.check([101, 10000000], .1)
        self.assertAlmostEqual(guard.sim_elapsed, .02)

    def test_invalid_timeout(self):
        for value in (0, -1, float('nan'), float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                MotionTimeout([0, 0], 0., wall_timeout=value)


if __name__ == '__main__':
    unittest.main()
