"""Unit tests of the correction consistency check (gnssd_tools.correction_consistency), no ROS."""

import math

from gnssd_tools.correction_consistency import CorrectionConsistencyCheck, INVALID, VALID

DRIFT = 1.0e-4   # rad/s, gyro bias sigma of the sensor model


def make_check():
    return CorrectionConsistencyCheck(k=3.0, floor_deg=3.0, drift_sigma=DRIFT, window=1.0)


def feed(check, yaws_deg, start=0.0, period=0.1):
    """Feed map->odom updates every `period` s with the given headings; return the alarms."""
    alarms = []
    for i, yaw in enumerate(yaws_deg):
        alarm = check.add_update(start + i * period, math.radians(yaw))
        if alarm is not None:
            alarms.append(alarm)
    return alarms


def test_valid_run_with_quantised_scan_matching():
    check = make_check()
    # scan matching moves map->odom in 0.2 deg steps back and forth
    yaws = [0.2 * ((i // 7) % 3 - 1) for i in range(2000)]
    assert feed(check, yaws) == []
    assert check.state == VALID


def test_slow_drift_correction_is_accepted():
    check = make_check()
    # 0.02 deg per update = 0.2 deg/s, i.e. 2 deg per 10 s: corrected continuously
    assert feed(check, [0.02 * i for i in range(500)]) == []
    assert check.state == VALID


def test_legitimate_loop_closure_is_accepted():
    check = make_check()
    yaws = [0.0] * 100 + [0.6] * 100      # one 0.6 deg step (largest seen in the bags)
    assert feed(check, yaws) == []


def test_jump_after_outage_is_detected():
    check = make_check()
    feed(check, [0.0] * 100)              # 10 s of normal updates, last at 9.9 s
    alarm = check.add_update(9.9 + 120.0, math.radians(24.0))   # first update after 120 s
    assert alarm is not None and check.state == INVALID
    assert alarm['heading_change_deg'] > alarm['allowed_deg']
    # allowed after 120 s: 3 deg + 3 * 1e-4 rad/s * 120 s = 5.06 deg
    assert abs(alarm['allowed_deg'] - (3.0 + math.degrees(3 * DRIFT * 120.0))) < 0.1


def test_invalid_is_latched_until_reset():
    check = make_check()
    feed(check, [0.0] * 50)
    assert check.add_update(10.0, math.radians(40.0)) is not None
    # later updates agree with the wrong map: still INVALID
    for i in range(100):
        assert check.add_update(10.1 + 0.1 * i, math.radians(40.0)) is None
    assert check.state == INVALID

    check.reset()                          # operator
    assert check.state == VALID
    assert feed(check, [40.0] * 50, start=30.0) == []   # new reference: no alarm
    assert check.state == VALID


def test_repeated_stamp_is_ignored():
    check = make_check()
    feed(check, [0.0] * 20)
    t = check.last_update_time
    assert check.add_update(t, math.radians(30.0)) is None   # same stamp re-sent
    assert check.state == VALID
