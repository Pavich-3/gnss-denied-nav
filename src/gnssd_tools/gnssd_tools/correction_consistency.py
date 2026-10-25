"""
Consistency check of the correction layer (no ROS inside; used by sensor_monitor_node).

What it detects. slam_toolbox corrects the base layer through the transform
map -> odom. A legitimate correction (scan matching, loop closure) only
removes the drift the base layer has accumulated, so map -> odom changes by
small amounts: in all recorded runs (Steps 4-5) at most 0.67 deg of heading
per update. A wrong correction - e.g. the first scan after a long lidar
outage registered at the wrong angle (Step 5b) - rotates map -> odom by
15-40 deg at once. After that the whole corrected estimate is wrong.

The test (heading only):
    change   = |yaw(map->odom now) - yaw(reference)|
    allowed  = floor + k * sigma_drift * dt
    INVALID if change > allowed
where `reference` is the last ACCEPTED map->odom state at least `window`
seconds old (or the last accepted one if updates stopped, e.g. during an
outage), dt the time between the two, and sigma_drift the heading drift rate
the base layer can have (the gyro bias of the sensor model, rad/s). The
heading drift of the base layer is what a correction legitimately removes, so
it bounds a legitimate heading correction; `floor` covers scan-matching
quantisation (0.2 deg steps) and loop closures.

Why not the EKF covariance: with the default (untuned) process noise the EKF
yaw sigma grows by ~4.4 deg per 0.1 s and by ~154 deg during a 120 s outage
(Step 5c analysis) - far above any real correction, so it could never fire.
Why not translation: the shift of the corrected position caused by a wrong
heading at the first update is small (0.09-0.36 m) and overlaps with
legitimate corrections (up to 0.15 m); it is reported, not tested.

States: VALID -> INVALID (latched). Once a wrong correction has entered
slam_toolbox's graph, the map and every later correction are built on it, so
the estimate does not become right again by itself; only the operator
(after e.g. restarting the mapping) clears the state with reset().
"""

import math

VALID = 'VALID'
INVALID = 'INVALID'


def wrap_angle(angle):
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class CorrectionConsistencyCheck:
    """Decides whether the stream of map -> odom updates is still plausible."""

    def __init__(self, k, floor_deg, drift_sigma, window):
        self.k = k
        self.floor = math.radians(floor_deg)
        self.drift_sigma = drift_sigma      # rad/s
        self.window = window                # s

        self.state = VALID
        self.accepted = []                  # (t, yaw) of accepted updates, newest last
        self.last_update_time = None
        self.alarm = None                   # details of the update that made it INVALID
        self.largest_ratio = 0.0            # largest change / allowed seen while VALID

    def allowed_change(self, dt):
        """Largest heading change (rad) explainable after dt seconds of base-layer drift."""
        return self.floor + self.k * self.drift_sigma * dt

    def add_update(self, t, yaw):
        """
        Register one NEW map -> odom update (time stamp t in s, heading in rad).

        Returns a dict describing the alarm if this update makes the check
        INVALID, otherwise None. Repeated stamps must be filtered by the caller.
        """
        if self.last_update_time is not None:
            if t == self.last_update_time:
                return None                 # the same update re-sent
            if t < self.last_update_time:
                self.accepted.clear()       # clock went back (simulation reset)
        gap = None if self.last_update_time is None else t - self.last_update_time
        self.last_update_time = t

        if self.state == INVALID:
            return None
        if not self.accepted:               # first update (or after a reset)
            self.accepted.append((t, yaw))
            return None

        reference_time, reference_yaw = self.reference(t)
        dt = t - reference_time
        change = abs(wrap_angle(yaw - reference_yaw))
        allowed = self.allowed_change(dt)

        if change > allowed:
            self.state = INVALID
            self.alarm = {
                'time': t,
                'heading_change_deg': math.degrees(change),
                'allowed_deg': math.degrees(allowed),
                'dt': dt,
                'gap_before_update': gap,
            }
            return self.alarm

        self.largest_ratio = max(self.largest_ratio, change / allowed)
        self.accepted.append((t, yaw))
        # Keep only what the window needs (plus one older reference).
        while len(self.accepted) > 2 and t - self.accepted[1][0] >= self.window:
            self.accepted.pop(0)
        return None

    def reference(self, t):
        """Return the newest accepted state at least `window` old, else the oldest kept."""
        candidates = [entry for entry in self.accepted if t - entry[0] >= self.window]
        return candidates[-1] if candidates else self.accepted[0]

    def reset(self):
        """Operator reset: accept the current map -> odom as the new reference."""
        self.state = VALID
        self.alarm = None
        self.accepted.clear()
