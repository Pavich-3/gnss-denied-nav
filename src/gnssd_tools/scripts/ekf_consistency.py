#!/usr/bin/env python3
"""
Check whether the base layer's covariance matches its real errors (Step 6b).

For every /odometry/filtered message of a bag, the true error against
/ground_truth (nearest stamp, NO alignment - both start at the world origin)
is compared with the EKF's own pose covariance:
  * position: squared Mahalanobis distance d2 = e^T * Sigma_xy^-1 * e;
      inside the 2-sigma ellipse    d2 < 4.00  (expected 86.5 % if consistent, 2D)
      inside the 95 % ellipse       d2 < 5.99  (expected 95 %)
  * heading: |yaw error| < 2 * sigma_yaw       (expected 95.4 %)
It also prints sigma_xy (sqrt of the largest eigenvalue of Sigma_xy) and
sigma_yaw at the end of the run next to the real errors at that moment.

A consistent filter shows roughly the expected fractions. Much lower =
over-confident (covariance too small), much higher = pessimistic.

Run with the Python of evo's venv (reads the bag through rosbags, no ROS):
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/ekf_consistency.py <bag> [<bag> ...]
"""

import argparse
import math
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores

MAX_TIME_DIFFERENCE = 0.02   # s, estimate and ground truth must be this close
SKIP_START = 1.0             # s, ignore the first second (covariance still ~1e-9)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read(bag):
    """
    Return the rows of the estimate and of the ground truth.

    Row: t, x, y, yaw; estimate rows also cov_xx, cov_xy, cov_yy, cov_yaw.
    """
    estimate, truth = [], []
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        connections = [c for c in reader.connections
                       if c.topic in ('/odometry/filtered', '/ground_truth')]
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            pose = msg.pose.pose
            row = [t, pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)]
            if connection.topic == '/odometry/filtered':
                c = msg.pose.covariance          # 6x6 row-major: x y z roll pitch yaw
                estimate.append(row + [c[0], c[1], c[7], c[35]])
            else:
                truth.append(row)
    return np.array(estimate), np.array(truth)


def analyse(bag):
    estimate, truth = read(bag)
    index = np.clip(np.searchsorted(truth[:, 0], estimate[:, 0]), 1, len(truth) - 1)
    t = estimate[:, 0]
    previous_is_closer = (t - truth[index - 1, 0]) < (truth[index, 0] - t)
    index = index - previous_is_closer
    keep = ((np.abs(truth[index, 0] - estimate[:, 0]) <= MAX_TIME_DIFFERENCE)
            & (estimate[:, 0] > estimate[0, 0] + SKIP_START))
    est, gt = estimate[keep], truth[index[keep]]

    ex, ey = est[:, 1] - gt[:, 1], est[:, 2] - gt[:, 2]
    eyaw = np.arctan2(np.sin(est[:, 3] - gt[:, 3]), np.cos(est[:, 3] - gt[:, 3]))
    sxx, sxy, syy, syaw2 = est[:, 4], est[:, 5], est[:, 6], est[:, 7]
    det = sxx * syy - sxy * sxy
    d2 = (syy * ex * ex - 2 * sxy * ex * ey + sxx * ey * ey) / det
    sigma_yaw = np.sqrt(syaw2)
    sigma_xy = np.sqrt((sxx + syy) / 2 + np.sqrt(((sxx - syy) / 2) ** 2 + sxy ** 2))

    print(f'== {Path(bag).name}  ({len(est)} samples, {est[-1, 0] - est[0, 0]:.0f} s)')
    inside_2sigma = 100 * np.mean(d2 < 4.0)
    inside_95 = 100 * np.mean(d2 < 5.991)
    heading_inside = 100 * np.mean(np.abs(eyaw) < 2 * sigma_yaw)
    print(f'  position inside 2-sigma ellipse : {inside_2sigma:5.1f} %  (expected 86.5 %)')
    print(f'  position inside 95 % ellipse    : {inside_95:5.1f} %  (expected 95 %)')
    print(f'  |heading error| < 2 sigma_yaw   : {heading_inside:5.1f} %  (expected 95.4 %)')
    print(f'  end of run: sigma_xy {sigma_xy[-1]:.3f} m, real position error '
          f'{math.hypot(ex[-1], ey[-1]):.3f} m (max during run {np.max(np.hypot(ex, ey)):.3f} m)')
    print(f'              sigma_yaw {math.degrees(sigma_yaw[-1]):.3f} deg, real heading error '
          f'{math.degrees(eyaw[-1]):+.3f} deg (max |.| during run '
          f'{math.degrees(np.max(np.abs(eyaw))):.3f} deg)')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('bags', nargs='+', help='rosbag2 folders')
    for bag in parser.parse_args().bags:
        analyse(bag)


if __name__ == '__main__':
    main()
