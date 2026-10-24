#!/usr/bin/env python3
"""
Estimate the gyro z bias of a recorded run.

Gazebo draws the gyro bias once per run (seeded by `gz sim --seed`) but does
not publish it. It can be recovered from the bag: the gyro measures
    w_gyro = w_true + bias + white noise,
so the mean of (w_gyro - w_true) over the whole run is the bias; the white
noise averages out (sigma 0.0017 rad/s over ~20 000 samples -> ~1e-5 rad/s).

w_true is the yaw rate of /ground_truth, matched to each /imu_raw sample by
the nearest stamp. The in-run (dynamic) bias also ends up in the mean.

Run with the Python of evo's venv (needs only rosbags + numpy):
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/estimate_gyro_bias.py <bag>
"""

import argparse
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description='Estimate the gyro z bias of a run')
    parser.add_argument('bag', help='rosbag2 folder')
    args = parser.parse_args()

    gyro_t, gyro_w, true_t, true_w = [], [], [], []
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
                true_w.append(msg.twist.twist.angular.z)

    gyro_t, gyro_w = np.array(gyro_t), np.array(gyro_w)
    true_t, true_w = np.array(true_t), np.array(true_w)

    # For every gyro sample, the ground-truth sample with the nearest stamp.
    index = np.clip(np.searchsorted(true_t, gyro_t), 1, len(true_t) - 1)
    previous_is_closer = (gyro_t - true_t[index - 1]) < (true_t[index] - gyro_t)
    index = index - previous_is_closer
    error = gyro_w - true_w[index]

    print(f'gyro z bias estimate: {error.mean():+.3e} rad/s '
          f'(std of the difference {error.std():.2e} rad/s, {len(error)} samples)')


if __name__ == '__main__':
    main()
