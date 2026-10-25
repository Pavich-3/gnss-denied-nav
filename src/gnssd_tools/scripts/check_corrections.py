#!/usr/bin/env python3
"""
Replay the correction consistency check on recorded bags (offline validation).

For every bag: reads the map -> odom updates from /tf (one per new stamp),
feeds them to CorrectionConsistencyCheck with the given parameters and prints
  * the alarms (sim time, heading change vs allowed, the gap before the update);
  * the largest heading change accepted as normal and the threshold at that
    moment (their ratio is the margin to a false alarm).
Bags without map -> odom (no correction layer) are skipped.

The drift rate defaults to the gyro bias of the sensor model, read from
robot_params.xacro (static and dynamic bias combined), like the live node.

Run with the Python of evo's venv (rosbags):
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/check_corrections.py --index bags/index.csv
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/check_corrections.py bags/<bag> [bags/<bag> ...]
"""

import argparse
import csv
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ElementTree

from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'src' / 'gnssd_tools'))
from gnssd_tools.correction_consistency import CorrectionConsistencyCheck  # noqa: E402

ROBOT_PARAMS = REPO / 'src' / 'gnssd_description' / 'urdf' / 'robot_params.xacro'


def xacro_property(name):
    root = ElementTree.parse(ROBOT_PARAMS).getroot()
    for element in root.iter('{http://www.ros.org/wiki/xacro}property'):
        if element.get('name') == name:
            return float(element.get('value'))
    raise KeyError(name)


def gyro_drift_sigma():
    """Heading drift rate of the base layer: static and dynamic gyro bias combined (rad/s)."""
    return math.hypot(xacro_property('imu_gyro_bias_stddev'),
                      xacro_property('imu_gyro_dynamic_bias_stddev'))


def map_to_odom_updates(bag):
    """Return [(stamp, yaw)] of map -> odom, one per new stamp, sorted; None if absent."""
    updates = {}
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        connections = [c for c in reader.connections if c.topic == '/tf']
        for connection, _, raw in reader.messages(connections=connections):
            for transform in reader.deserialize(raw, connection.msgtype).transforms:
                if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                    stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                    q = transform.transform.rotation
                    updates[stamp] = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                1 - 2 * (q.y * q.y + q.z * q.z))
    return sorted(updates.items()) if updates else None


def check_bag(bag, args):
    updates = map_to_odom_updates(bag)
    if updates is None:
        return None
    check = CorrectionConsistencyCheck(args.k, args.floor_deg, args.drift, args.window)
    alarms = []
    largest = (0.0, 0.0, 0.0)          # (change, allowed, time) of the largest accepted change
    for t, yaw in updates:
        reference = check.reference(t) if check.accepted and check.state == 'VALID' else None
        alarm = check.add_update(t, yaw)
        if alarm is not None:
            alarms.append(alarm)
        elif reference is not None and check.state == 'VALID':
            change = abs(math.atan2(math.sin(yaw - reference[1]), math.cos(yaw - reference[1])))
            allowed = check.allowed_change(t - reference[0])
            if change / allowed > largest[0] / max(largest[1], 1e-12):
                largest = (change, allowed, t)
    return len(updates), alarms, largest


def bags_from_index(index):
    with open(index) as f:
        return [Path(index).parent / row['bag'] for row in csv.DictReader(f)]


def main():
    parser = argparse.ArgumentParser(description='Offline correction consistency check')
    parser.add_argument('bags', nargs='*')
    parser.add_argument('--index', help='bags/index.csv: check every bag listed there')
    parser.add_argument('--k', type=float, default=3.0)
    parser.add_argument('--floor-deg', type=float, default=3.0)
    parser.add_argument('--window', type=float, default=1.0)
    parser.add_argument('--drift', type=float, default=None,
                        help='heading drift rate, rad/s (default: gyro bias from robot_params)')
    args = parser.parse_args()
    if args.drift is None:
        args.drift = gyro_drift_sigma()
    bags = (bags_from_index(args.index) if args.index else []) + [Path(b) for b in args.bags]

    print(f'parameters: k {args.k}, floor {args.floor_deg} deg, window {args.window} s, '
          f'drift {args.drift:.3e} rad/s')
    for bag in bags:
        result = check_bag(bag, args)
        if result is None:
            print(f'{bag.name}: no map->odom (no correction layer), skipped')
            continue
        count, alarms, (change, allowed, t) = result
        print(f'{bag.name}: {count} updates; largest accepted change '
              f'{math.degrees(change):.3f} deg '
              f'at t={t:.1f} s vs allowed {math.degrees(allowed):.2f} deg '
              f'(ratio {change / allowed if allowed else 0:.3f}); alarms: {len(alarms)}')
        for a in alarms:
            gap = a['gap_before_update']
            print(f'    ALARM t={a["time"]:.1f} s: heading change '
                  f'{a["heading_change_deg"]:.2f} deg > '
                  f'allowed {a["allowed_deg"]:.2f} deg (dt {a["dt"]:.1f} s, '
                  f'gap before the update {gap:.1f} s)')


if __name__ == '__main__':
    main()
