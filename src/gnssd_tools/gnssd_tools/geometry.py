"""Small 2D geometry helpers shared by the project nodes."""

import math


def yaw_from_quaternion(q):
    """Return the heading angle (rotation about z) of a quaternion, in radians."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(angle):
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
