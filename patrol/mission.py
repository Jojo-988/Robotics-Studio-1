
from dataclasses import dataclass
import math

from .planner import CoverageConfig, CoverageGrid, plan_coverage


@dataclass(frozen=True)
class FlightConfig:
    altitude: float = 15.0
    max_speed: float = 1.5
    vertical_speed: float = 1.0
    max_yaw_rate: float = 0.8
    position_gain: float = 1.0
    altitude_tolerance: float = 0.2
    waypoint_timeout: float = 90.0
    repeat: bool = False

    def __post_init__(self):
        for name, value in vars(self).items():
            if name != 'repeat' and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} must be finite and positive.')
        if self.altitude <= self.altitude_tolerance:
            raise ValueError('altitude must exceed altitude_tolerance.')


class PatrolMission:

    def __init__(self, coverage=None, flight=None):
        self.coverage = coverage or CoverageConfig()
        self.flight = flight or FlightConfig()
        self.grid = CoverageGrid(self.coverage)
        self.route = [] # Set all the target waypoints
        self.index = 0  # The current target waypoint
        self.completed_passes = 0
        self.state = 'WAITING'
        self.reason = ''
        self.home = None    # The start position 
        self.target = None  # The current position
        self.deadline = None
        self.previous = None
        self.previous_time = None
        self.paused = False
        self.pause_started = None

    def pause(self, enabled, now):
        if enabled and not self.paused:
            self.pause_started = now
        elif not enabled and self.paused and self.deadline is not None:
            self.deadline += max(0, now - self.pause_started)
        self.paused = enabled
        self.break_trace()

    def break_trace(self):
        self.previous = None
        self.previous_time = None

    def stop(self, reason='Stopped by operator', state='STOPPED'):
        self.state, self.reason = state, reason
        self.break_trace()

    def _set_target(self, xy, now):
        self.target = (xy[0], xy[1], self.flight.altitude)
        self.deadline = now + self.flight.waypoint_timeout

    def step(self, pose, now):
        zero = (0.0, 0.0, 0.0, 0.0)
        if not all(math.isfinite(v) for v in (*pose, now)):
            self.stop('Non-finite pose/time', 'ERROR')
        if self.state in ('ERROR', 'STOPPED') or self.paused:
            return zero
        x, y, z, yaw = pose
        if self.home is None:   # record the current position as the return point, then find the start point append on this location
            self.home = (x, y)
            self.route = plan_coverage(self.coverage, self.home)
            self.state = 'TAKEOFF'
            self._set_target(self.home, now)    # add the hight of the drone

        if self.state == 'PATROL' and abs(z - self.flight.altitude) <= self.flight.altitude_tolerance:  # The drone enters recording mode once it reaches the patrol altitude.
            if self.previous is not None:
                dt = now - self.previous_time
                if dt < 0 or math.dist((x, y), self.previous) > 2 * self.flight.max_speed * dt + 0.5:   # Record two position changes within a short time; if abnormal movement is detected, stop the simulation.
                    self.stop('Pose discontinuity; restart mission after simulator reset', 'ERROR')
                    return zero
                self.grid.mark_segment(self.previous, (x, y))
            else:
                self.grid.mark_segment((x, y), (x, y))
            self.previous, self.previous_time = (x, y), now
        else:
            self.break_trace()

        # Record the required conditions, position, and altitude for reaching the cruise point.
        reached = (math.hypot(x - self.target[0], y - self.target[1]) <= self.coverage.position_tolerance
                   and abs(z - self.target[2]) <= self.flight.altitude_tolerance)
        if reached:
            if self.state == 'TAKEOFF':
                self.state = 'PATROL'
                self._set_target(self.route[0], now)
            elif self.state == 'PATROL':
                self.index += 1
                if self.index < len(self.route):
                    self._set_target(self.route[self.index], now)
                elif self.grid.fraction < 1.0:  # After reaching the final cruising point, it is still necessary to verify whether the scan coverage meets the required standard.
                    self.stop('Route finished with uncovered cells; inspect footprint/tracking', 'ERROR')
                    return zero
                else:
                    self.completed_passes += 1
                    if self.flight.repeat:  # Can patrol continuously or return to home
                        self.route.reverse()
                        self.index = 0
                        self.grid = CoverageGrid(self.coverage)
                        self.break_trace()
                        self._set_target(self.route[0], now)
                    else:
                        self.state = 'RETURNING'
                        self._set_target(self.home, now)
            elif self.state == 'RETURNING':
                self.state = 'COMPLETE'
        elif self.state != 'COMPLETE' and now > self.deadline:  # If the destination cannot be reached, report an error directly.
            self.stop('Waypoint timeout; no waypoint was skipped', 'ERROR')
            return zero

        dx, dy, dz = self.target[0] - x, self.target[1] - y, self.target[2] - z
        distance = math.hypot(dx, dy)
        speed = min(self.flight.max_speed, self.flight.position_gain * distance)
        # Hold horizontal position while regaining survey altitude.
        if abs(dz) > self.flight.altitude_tolerance and self.state == 'PATROL':
            speed = 0.0
        vx = speed * dx / distance if distance else 0.0
        vy = speed * dy / distance if distance else 0.0
        vz = max(-self.flight.vertical_speed, min(self.flight.vertical_speed, self.flight.position_gain * dz))
        yaw_error = 0.0 if distance <= self.coverage.position_tolerance else math.atan2(
            math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
        wz = max(-self.flight.max_yaw_rate, min(self.flight.max_yaw_rate, yaw_error))
        return (math.cos(yaw) * vx + math.sin(yaw) * vy,
                -math.sin(yaw) * vx + math.cos(yaw) * vy, vz, wz)

    def status(self):
        return {'state': 'PAUSED' if self.paused else self.state,
                'mission_state': self.state, 'reason': self.reason,
                'waypoint_index': self.index, 'waypoints_total': len(self.route),
                'coverage_fraction': self.grid.fraction,
                'coverage_model': 'geometric circular footprint; no occlusion modelling',
                'completed_passes': self.completed_passes, 'target': self.target}
