"""Timeout policy using simulation odometry timestamps and monotonic wall time."""
import math


class MotionTimeout:
    def __init__(self, stamp, now, sim_timeout=45., wall_timeout=180., stall_timeout=1.):
        if not all(math.isfinite(v) and v > 0 for v in
                   (sim_timeout, wall_timeout, stall_timeout)):
            raise ValueError('Timeouts must be finite and positive')
        self.start_ns = self.previous_ns = self.to_ns(stamp)
        self.start_wall = self.last_advance_wall = now
        self.sim_timeout = sim_timeout
        self.wall_timeout = wall_timeout
        self.stall_timeout = stall_timeout
        self.sim_elapsed = self.wall_elapsed = 0.

    @staticmethod
    def to_ns(stamp):
        return int(stamp[0]) * 1_000_000_000 + int(stamp[1])

    def check(self, stamp, now):
        current = self.to_ns(stamp)
        self.wall_elapsed = now - self.start_wall
        self.sim_elapsed = (current - self.start_ns) / 1e9
        if current < self.previous_ns:
            raise RuntimeError('ODOMETRY_TIME_BACKWARDS')
        if current > self.previous_ns:
            self.previous_ns = current
            self.last_advance_wall = now
        if now - self.last_advance_wall >= self.stall_timeout:
            raise RuntimeError('ODOMETRY_TIME_STALLED')
        if self.wall_elapsed >= self.wall_timeout:
            raise RuntimeError('WALL_TIMEOUT')
        if self.sim_elapsed >= self.sim_timeout:
            raise RuntimeError('SIM_TIMEOUT')

    def report(self):
        return dict(time_source='odometry_header_stamp',
                    sim_elapsed_s=self.sim_elapsed, wall_elapsed_s=self.wall_elapsed,
                    sim_timeout_s=self.sim_timeout, wall_timeout_s=self.wall_timeout,
                    stall_timeout_s=self.stall_timeout)
