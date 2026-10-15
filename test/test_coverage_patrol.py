"""Geometry and closed-loop mission tests; executable without ROS or Gazebo."""

from dataclasses import replace
import math
import random
import unittest

from patrol.mission import FlightConfig, PatrolMission
from patrol.planner import CoverageConfig, CoverageGrid, distance_to_segment, plan_coverage


def integrate(pose, command, dt=0.05):
    x, y, z, yaw = pose
    vx, vy, vz, wz = command
    return (x + (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt,
            y + (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt,
            z + vz * dt, yaw + wz * dt)


class CoverageTests(unittest.TestCase):
    def test_boundary_and_interior_are_covered(self):
        rng = random.Random(41068)
        for axis in ('x', 'y', 'auto'):
            for bounds in ((-12.5, 12.5, -12.5, 12.5), (2, 2.3, -4, 17), (-8, 23, 1, 1.2)):
                with self.subTest(axis=axis, bounds=bounds):
                    c = CoverageConfig(*bounds, sweep_axis=axis)
                    route = plan_coverage(c, (3, -1))
                    grid = CoverageGrid(c)
                    for a, b in zip(route, route[1:]):
                        self.assertLessEqual(math.dist(a, b), c.waypoint_spacing + 1e-9)
                        grid.mark_segment(a, b)
                    self.assertEqual(grid.fraction, 1.0)
                    points = [(c.x_min, c.y_min), (c.x_min, c.y_max),
                              (c.x_max, c.y_min), (c.x_max, c.y_max)]
                    points += [(rng.uniform(c.x_min, c.x_max), rng.uniform(c.y_min, c.y_max))
                               for _ in range(100)]
                    for p in points:
                        nearest = min(distance_to_segment(p, a, b) for a, b in zip(route, route[1:]))
                        self.assertLessEqual(nearest, c.footprint_width / 2)
                    for x, y in route:
                        self.assertTrue(c.x_min - 1e-9 <= x <= c.x_max + 1e-9)
                        self.assertTrue(c.y_min - 1e-9 <= y <= c.y_max + 1e-9)

    def test_invalid_parameters_fail_before_flight(self):
        for values in ({'x_max': -20}, {'overlap': 1}, {'footprint_width': 0},
                       {'footprint_width': 0.5}, {'grid_resolution': 0},
                       {'x_min': math.nan}, {'sweep_axis': 'z'}, {'waypoint_spacing': -2},
                       {'grid_resolution': 0.001}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                CoverageConfig(**values)
        with self.assertRaises(ValueError):
            FlightConfig(max_speed=0)
        with self.assertRaises(ValueError):
            plan_coverage(CoverageConfig(overlap=0.999999999))

    def test_near_entry_is_chosen_and_partial_route_is_not_complete(self):
        c = CoverageConfig(sweep_axis='x')
        route = plan_coverage(c, (12.5, 12.5))
        self.assertEqual(route[0], (12.5, 12.5))
        grid = CoverageGrid(c)
        grid.mark_segment(route[0], route[1])
        self.assertGreater(grid.fraction, 0)
        self.assertLess(grid.fraction, 1)


class MissionTests(unittest.TestCase):
    def test_full_flight_and_return_from_offset_spawn(self):
        mission = PatrolMission()
        pose = (2.0, 0.0, 0.8, 1.2)
        states = set()
        for i in range(24000):
            cmd = mission.step(pose, i * 0.05)
            states.add(mission.state)
            self.assertLessEqual(math.hypot(cmd[0], cmd[1]), mission.flight.max_speed + 1e-9)
            if mission.state == 'TAKEOFF':
                self.assertAlmostEqual(pose[0], 2)
                self.assertAlmostEqual(pose[1], 0)
                self.assertEqual(mission.grid.fraction, 0)
            pose = integrate(pose, cmd)
            if mission.state in ('COMPLETE', 'ERROR'):
                break
        self.assertEqual(mission.state, 'COMPLETE', mission.reason)
        self.assertEqual(mission.completed_passes, 1)
        self.assertEqual(mission.grid.fraction, 1)
        self.assertLess(math.dist(pose[:2], (2, 0)), mission.coverage.position_tolerance)
        self.assertTrue({'TAKEOFF', 'PATROL', 'RETURNING', 'COMPLETE'} <= states)

    def test_pause_resumes_same_waypoint_without_counting_gap(self):
        mission = PatrolMission()
        mission.step((2, 0, 15, 0), 0)
        mission.step((2, 0, 15, 0), 0.05)
        target, index, fraction = mission.target, mission.index, mission.grid.fraction
        mission.pause(True, 0.1)
        self.assertEqual(mission.step((4, 4, 15, 0), 100), (0, 0, 0, 0))
        self.assertEqual(mission.grid.fraction, fraction)
        mission.pause(False, 100)
        mission.step((2, 0, 15, 0), 100.05)
        self.assertEqual((mission.target, mission.index), (target, index))
        self.assertEqual(mission.state, 'PATROL')

    def test_stuck_robot_times_out_without_skipping(self):
        mission = PatrolMission()
        mission.step((2, 0, 0.8, 0), 0)
        cmd = mission.step((2, 0, 0.8, 0), 100)
        self.assertEqual(cmd, (0, 0, 0, 0))
        self.assertEqual(mission.state, 'ERROR')
        self.assertEqual(mission.index, 0)
        self.assertEqual(mission.grid.fraction, 0)

    def test_teleport_does_not_count_as_flown_coverage(self):
        mission = PatrolMission()
        mission.step((2, 0, 15, 0), 0)
        mission.step((2, 0, 15, 0), 0.05)
        before = mission.grid.fraction
        mission.step((-12, -12, 15, 0), 0.1)
        self.assertEqual(mission.state, 'ERROR')
        self.assertEqual(mission.grid.fraction, before)

    def test_repeat_completes_two_passes_then_stops(self):
        c = replace(CoverageConfig(), x_min=-2, x_max=2, y_min=-2, y_max=2)
        mission = PatrolMission(c, FlightConfig(altitude=3, repeat=True))
        pose = (0.0, 0.0, 0.8, 0.0)
        for i in range(8000):
            pose = integrate(pose, mission.step(pose, i * 0.05))
            if mission.completed_passes == 2 or mission.state == 'ERROR':
                break
        self.assertEqual(mission.completed_passes, 2, mission.reason)
        self.assertEqual(mission.state, 'PATROL')
        mission.stop()
        self.assertEqual(mission.step(pose, 500), (0, 0, 0, 0))

    def test_no_coverage_outside_altitude_band(self):
        mission = PatrolMission()
        mission.step((2, 0, 15, 0), 0)
        mission.step((2, 0, 3, 0), 1)
        self.assertEqual(mission.grid.fraction, 0)
        self.assertEqual(mission.step((2, 0, 3, 0), 1.1)[:2], (0, 0))


if __name__ == '__main__':
    unittest.main()
