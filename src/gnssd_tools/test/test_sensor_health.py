"""Unit tests of the sensor state machine (gnssd_tools.sensor_health), no ROS needed."""

import pytest

from gnssd_tools.sensor_health import LOST, OK, SensorWatch, WAITING


def feed(watch, start, end, period):
    """Deliver messages every `period` seconds in [start, end); return the transitions."""
    transitions = []
    t = start
    while t < end - 1e-9:
        result = watch.on_message(t)
        if result is not None:
            transitions.append(result)
        t += period
    return transitions


def test_starts_waiting_and_first_message_makes_it_ok():
    watch = SensorWatch('scan', timeout=0.5, expected_rate=10.0)
    assert watch.state == WAITING
    assert watch.check(1.0) is None          # still nothing: no transition, no spam

    transition = watch.on_message(1.2)
    assert transition.old_state == WAITING and transition.new_state == OK
    assert watch.state == OK


def test_no_message_ever_stays_waiting():
    watch = SensorWatch('imu', timeout=0.05, expected_rate=100.0)
    for i in range(100):
        assert watch.check(i * 0.1) is None
    assert watch.state == WAITING
    assert watch.age(10.0) is None


def test_timeout_reports_lost_once():
    watch = SensorWatch('odom', timeout=0.1, expected_rate=50.0)
    feed(watch, 0.0, 1.0, 0.02)             # last message at 0.98 s
    assert watch.check(1.05) is None         # silence 0.07 s < timeout

    transition = watch.check(1.10)           # silence 0.12 s > timeout
    assert transition.new_state == LOST
    assert transition.duration == pytest.approx(0.12)

    for t in (1.2, 1.5, 3.0):                # no repeated reports while lost
        assert watch.check(t) is None
    assert watch.state == LOST


def test_recovery_reports_outage_duration():
    watch = SensorWatch('scan', timeout=0.5, expected_rate=10.0)
    feed(watch, 0.0, 10.0, 0.1)              # last message at 9.9 s
    assert watch.check(10.5).new_state == LOST

    transition = watch.on_message(70.0)      # lidar back after a 60 s gap
    assert transition.old_state == LOST and transition.new_state == OK
    assert transition.duration == pytest.approx(60.1)
    assert watch.state == OK


def test_measured_rate():
    watch = SensorWatch('imu', timeout=0.05, expected_rate=100.0, rate_window=2.0)
    feed(watch, 0.0, 3.0, 0.01)
    assert watch.measured_rate() == pytest.approx(100.0, rel=0.01)


def test_clock_jump_back_resets_to_waiting():
    watch = SensorWatch('odom', timeout=0.1, expected_rate=50.0)
    feed(watch, 100.0, 101.0, 0.02)
    assert watch.state == OK

    transition = watch.check(5.0)            # simulation was reset
    assert transition.new_state == WAITING
    assert watch.age(5.0) is None and watch.measured_rate() == 0.0

    assert watch.on_message(5.1).new_state == OK   # and it recovers normally
