import math
import unittest

from rover_navigation.core import bounded_command, fresh, planar_pose, pose_errors


class RoverCoreTests(unittest.TestCase):
    def test_ground_goal_normalizes_quaternion(self):
        pose = planar_pose(3, -2, 0, 0, 0, 2, 2)
        self.assertEqual(pose[:2], (3, -2))
        self.assertAlmostEqual(pose[2], math.pi/2)

    def test_invalid_and_nonplanar_goals(self):
        for pose in ((math.nan, 0, 0, 0, 0, 0, 1), (0, 0, 0, 0, 0, 0, 0),
                     (0, 0, 7, 0, 0, 0, 1), (0, 0, 0, .5, 0, 0, 1)):
            with self.subTest(pose=pose), self.assertRaises(ValueError):
                planar_pose(*pose)

    def test_heading_error_wraps_at_pi(self):
        xy, yaw = pose_errors((0, 0, math.pi-.01), (3, 4, -math.pi+.01))
        self.assertEqual(xy, 5)
        self.assertAlmostEqual(yaw, .02)

    def test_command_limit_preserves_curvature(self):
        vx, wz = bounded_command((1.2, 0, 0, 0, 0, .8), .6, .8)
        self.assertEqual((vx, wz), (.6, .4))
        self.assertEqual(bounded_command((-.3, 0, 0, 0, 0, 0), .6, .8), (-.3, 0))

    def test_invalid_velocity_rejected(self):
        for values in ((math.inf, 0, 0, 0, 0, 0), (0, .1, 0, 0, 0, 0),
                       (0, 0, 1, 0, 0, 0), (0, 0, 0, .1, 0, 0)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                bounded_command(values, .6, .8)

    def test_missing_old_and_future_inputs_expire(self):
        self.assertFalse(fresh(None, 10, .5))
        self.assertFalse(fresh(9, 10, .5))
        self.assertFalse(fresh(11, 10, .5))
        self.assertTrue(fresh(9.8, 10, .5))


if __name__ == '__main__':
    unittest.main()
