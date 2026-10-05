
import math


def planar_pose(x, y, z, qx, qy, qz, qw):
    """Validate a ground-robot pose; return x, y and normalized yaw."""
    if not all(math.isfinite(v) for v in (x, y, z, qx, qy, qz, qw)):
        raise ValueError('Pose contains NaN or infinity')
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-6:
        raise ValueError('Orientation must be a nonzero quaternion')
    qx, qy, qz, qw = (v / norm for v in (qx, qy, qz, qw))
    if abs(z) > 0.05 or math.hypot(qx, qy) > 0.01:
        raise ValueError('Ground goal must have z=0 and only a yaw rotation')
    return x, y, math.atan2(2*qw*qz, 1-2*qz*qz)


def pose_errors(current, target):
    return (math.hypot(current[0]-target[0], current[1]-target[1]),
            abs(math.atan2(math.sin(current[2]-target[2]),
                           math.cos(current[2]-target[2]))))


def fresh(received, now, timeout):
    return received is not None and 0 <= now-received <= timeout


def bounded_command(values, max_speed, max_yaw_rate):
    """Reject invalid/non-planar commands and preserve v/w curvature when limiting."""
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Velocity contains NaN or infinity')
    vx, vy, vz, wx, wy, wz = values
    if any(abs(v) > 1e-6 for v in (vy, vz, wx, wy)):
        raise ValueError('Husky accepts only linear.x and angular.z')
    scale = max(1.0, abs(vx)/max_speed, abs(wz)/max_yaw_rate)
    return vx/scale, wz/scale
