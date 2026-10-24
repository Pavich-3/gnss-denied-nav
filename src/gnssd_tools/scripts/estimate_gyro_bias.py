#!/usr/bin/env python3
"""
Estimate the gyro z bias of a recorded run.

Gazebo draws the gyro bias once per run (seeded by `gz sim --seed`) but does
not publish it. It can be recovered from the bag by comparing the heading
change measured by the gyro with the true heading change:

    bias = ( integral of w_gyro dt  -  (true yaw at the end - true yaw at the start) ) / T

The true yaw comes from the orientation in /ground_truth (unwrapped), not from
its angular velocity: Gazebo smooths that velocity with a rolling mean, which
gave a timing-dependent error of ~1e-4 rad/s in an earlier version of this
script. The white noise of the gyro contributes only
sigma * sqrt(dt * T) / T ~ 1e-5 rad/s for a 2-lap run. The in-run (dynamic)
bias is included in the result (it is its time average).

Run with the Python of evo's venv (needs only rosbags + numpy):
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/estimate_gyro_bias.py <bag>
"""

import argparse
import math
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser(description='Estimate the gyro z bias of a run')
    parser.add_argument('bag', help='rosbag2 folder')
    args = parser.parse_args()

    gyro_t, gyro_w, true_t, true_yaw = [], [], [], []
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(args.bag)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections
                       if c.topic in ('/imu_raw', '/ground_truth')]
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            if connection.topic == '/imu_raw':
                gyro_t.append(stamp_seconds(msg.header))
                gyro_w.append(msg.angular_velocity.z)
            else:
                true_t.append(stamp_seconds(msg.header))
                true_yaw.append(yaw_from_quaternion(msg.pose.pose.orientation))

    order = np.argsort(gyro_t)
    gyro_t, gyro_w = np.array(gyro_t)[order], np.array(gyro_w)[order]
    order = np.argsort(true_t)
    true_t, true_yaw = np.array(true_t)[order], np.unwrap(np.array(true_yaw)[order])

    # Use the time span covered by both topics.
    start, end = max(gyro_t[0], true_t[0]), min(gyro_t[-1], true_t[-1])
    inside = (gyro_t >= start) & (gyro_t <= end)
    t, w = gyro_t[inside], gyro_w[inside]

    # np.trapezoid in numpy >= 2 (evo's venv), np.trapz in older versions.
    integrate = getattr(np, 'trapezoid', None) or np.trapz
    gyro_heading_change = integrate(w, t)
    true_heading_change = np.interp(t[-1], true_t, true_yaw) - np.interp(t[0], true_t, true_yaw)
    duration = t[-1] - t[0]
    bias = (gyro_heading_change - true_heading_change) / duration

    print(f'gyro z bias estimate: {bias:+.3e} rad/s '
          f'(heading change: gyro {math.degrees(gyro_heading_change):.2f} deg, '
          f'true {math.degrees(true_heading_change):.2f} deg, over {duration:.1f} s)')


if __name__ == '__main__':
    main()
