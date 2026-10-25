"""
State machine that decides whether one sensor stream is alive (no ROS inside).

Used by sensor_monitor_node, one `SensorWatch` per sensor. Kept free of ROS
so that it can be unit-tested with plain numbers (test/test_sensor_health.py).

States:
    WAITING  no message has arrived yet since start (or since a clock reset)
    OK       the last message is younger than the timeout
    LOST     no message for longer than the timeout

Transitions:
    WAITING -> OK    first message
    OK      -> LOST  check(now): now - last message > timeout
    LOST    -> OK    a message arrives again
    any     -> WAITING  the clock jumped backwards (simulation reset):
                        all history is forgotten

Every transition is returned exactly once as a `Transition`, so the caller
can log it once instead of repeating the same warning at every check.

Times are plain floats in seconds (the node passes its ROS clock, i.e. sim time).
"""

from collections import deque
from dataclasses import dataclass

WAITING = 'WAITING'
OK = 'OK'
LOST = 'LOST'


@dataclass
class Transition:
    """A change of state, reported once."""

    sensor: str
    old_state: str
    new_state: str
    time: float
    # LOST: how long the sensor had been silent when the loss was detected.
    # back to OK: how long the outage lasted (gap between the two messages).
    duration: float = 0.0


class SensorWatch:
    """Tracks the message arrivals of one sensor and its state."""

    def __init__(self, name, timeout, expected_rate, rate_window=2.0):
        self.name = name
        self.timeout = timeout
        self.expected_rate = expected_rate
        self.rate_window = rate_window

        self.state = WAITING
        self.last_time = None
        self.ok_since = None      # time of the last transition to OK
        self.arrivals = deque()   # arrival times inside the rate window

    def on_message(self, now):
        """Register one message arriving at time `now`; return a Transition or None."""
        # A backwards clock jump is normally reported by check(), which runs
        # at 10 Hz; here the history is just reset and the message counts as
        # the first one (WAITING -> OK).
        self._reset_if_clock_went_back(now)

        previous_time = self.last_time
        self.last_time = now
        self.arrivals.append(now)
        self._forget_old_arrivals(now)

        if self.state == WAITING:
            return self._change(OK, now)
        if self.state == LOST:
            return self._change(OK, now, duration=now - previous_time)
        return None

    def check(self, now):
        """Re-evaluate the state at time `now` (called periodically); return Transition/None."""
        transition = self._reset_if_clock_went_back(now)
        if transition is not None:
            return transition

        self._forget_old_arrivals(now)
        if self.state == OK and now - self.last_time > self.timeout:
            return self._change(LOST, now, duration=now - self.last_time)
        return None

    def age(self, now):
        """Seconds since the last message, or None if there was none yet."""
        return None if self.last_time is None else now - self.last_time

    def rate_known(self, now):
        """
        Return True once the sensor has been OK for a full rate window.

        Right after the start or after an outage the window holds only a few
        messages and the measured rate is meaningless (e.g. 0 Hz).
        """
        return self.state == OK and now - self.ok_since >= self.rate_window

    def measured_rate(self):
        """Message rate over the last `rate_window` seconds (Hz), or 0 if unknown."""
        if len(self.arrivals) < 2:
            return 0.0
        span = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / span if span > 0 else 0.0

    def _forget_old_arrivals(self, now):
        while self.arrivals and now - self.arrivals[0] > self.rate_window:
            self.arrivals.popleft()

    def _reset_if_clock_went_back(self, now):
        """After a backwards clock jump the old times mean nothing: start again."""
        if self.last_time is not None and now < self.last_time:
            self.last_time = None
            self.arrivals.clear()
            if self.state != WAITING:
                return self._change(WAITING, now)
        return None

    def _change(self, new_state, now, duration=0.0):
        transition = Transition(self.name, self.state, new_state, now, duration)
        self.state = new_state
        if new_state == OK:
            self.ok_since = now
        return transition
