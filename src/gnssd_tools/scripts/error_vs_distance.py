#!/usr/bin/env python3
"""
Plot the position error of trajectory estimates against distance.

Two panels:
  left   error vs distance of the robot from the start point, |p_true|
         (shows whether the error grows the farther the robot goes);
  right  error vs travelled distance along the true path.

Inputs are TUM files (timestamp x y z qx qy qz qw), e.g. made with
`evo_traj bag2 <bag> /ground_truth /odometry/filtered --save_as_tum` and
`tf_to_tum.py <bag> map base_link corrected.tum`. Every estimate is matched
to the ground truth by the nearest time stamp (max 0.02 s apart). No
alignment: all trajectories start at the world origin.

Run with the Python of evo's venv:
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/error_vs_distance.py
      ground_truth.tum out.png base=odometry_filtered.tum corrected=corrected.tum
  (one command line)
"""

import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402  (must come after matplotlib.use)
import numpy as np  # noqa: E402

MAX_TIME_DIFFERENCE = 0.02  # s


def load_tum(path):
    """Return (times, xy) arrays from a TUM file."""
    data = np.loadtxt(path)
    return data[:, 0], data[:, 1:3]


def main():
    parser = argparse.ArgumentParser(description='Position error vs distance plots')
    parser.add_argument('ground_truth', help='TUM file of the ground truth')
    parser.add_argument('output', help='PNG file to write')
    parser.add_argument('estimates', nargs='+', help='label=file.tum for every estimate')
    args = parser.parse_args()

    true_t, true_xy = load_tum(args.ground_truth)
    distance_from_start = np.hypot(true_xy[:, 0], true_xy[:, 1])
    step_lengths = np.hypot(*np.diff(true_xy, axis=0).T)
    travelled = np.concatenate([[0.0], np.cumsum(step_lengths)])

    figure, (ax_start, ax_path) = plt.subplots(1, 2, figsize=(13, 5))
    for item in args.estimates:
        label, path = item.split('=', 1)
        est_t, est_xy = load_tum(path)

        # Nearest ground-truth sample for every estimate sample.
        index = np.clip(np.searchsorted(true_t, est_t), 1, len(true_t) - 1)
        previous_is_closer = (est_t - true_t[index - 1]) < (true_t[index] - est_t)
        index = index - previous_is_closer
        close = np.abs(true_t[index] - est_t) <= MAX_TIME_DIFFERENCE
        index, est_xy = index[close], est_xy[close]

        error = np.hypot(*(est_xy - true_xy[index]).T)
        ax_start.scatter(distance_from_start[index], error, s=1, label=label)
        ax_path.plot(travelled[index], error, linewidth=0.8, label=label)
        print(f'{label}: {len(error)} matched poses, RMSE {np.sqrt(np.mean(error ** 2)):.3f} m, '
              f'max {error.max():.3f} m')

    ax_start.set_xlabel('distance of the robot from the start |p_true|, m')
    ax_start.set_ylabel('position error, m')
    ax_start.set_title('Error vs distance from the start')
    ax_path.set_xlabel('travelled distance, m')
    ax_path.set_ylabel('position error, m')
    ax_path.set_title('Error vs travelled distance')
    for axes in (ax_start, ax_path):
        axes.grid(True)
        axes.legend(markerscale=6)
    figure.tight_layout()
    figure.savefig(args.output, dpi=130)
    print(f'plot saved to {args.output}')


if __name__ == '__main__':
    main()
