#!/usr/bin/env python3
"""
Compare noisy wheel odometry (/odom) with the simulator's exact odometry (/odom_raw).

Reads a rosbag2 recording and draws two plots:
  left   the two trajectories in the x-y plane;
  right  the distance between the two positions over time (the drift).

With the noise node enabled the drift grows over the run; with
odom_noise:=false the two trajectories must coincide (drift ~ 0).

Usage (after `source install/setup.bash`):
  python3 src/gnssd_tools/scripts/compare_odom.py bags/<bag_folder>
  python3 src/gnssd_tools/scripts/compare_odom.py bags/<bag_folder> -o drift.png
"""

import argparse
import math
import os

import matplotlib
matplotlib.use('Agg')  # draw into a file; no window needed
import matplotlib.pyplot as plt  # noqa: E402  (must come after matplotlib.use)
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
import rosbag2_py  # noqa: E402
import yaml  # noqa: E402


def read_positions(bag_path, topics):
    """Return {topic: (times, xs, ys)} for the given Odometry topics in a bag."""
    # The storage plugin (sqlite3 or mcap) is written in the bag's metadata.
    with open(os.path.join(bag_path, 'metadata.yaml')) as f:
        storage_id = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))

    data = {topic: ([], [], []) for topic in topics}
    while reader.has_next():
        topic, raw_bytes, _ = reader.read_next()
        if topic not in data:
            continue
        msg = deserialize_message(raw_bytes, Odometry)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        times, xs, ys = data[topic]
        times.append(stamp)
        xs.append(msg.pose.pose.position.x)
        ys.append(msg.pose.pose.position.y)
    return data


def drift_over_time(reference, measured):
    """
    Return (times, distances) between two trajectories.

    For every measured sample, the reference sample with the closest time stamp
    is used. Both topics come from the same 50 Hz source, so the stamps match.
    """
    ref_times, ref_xs, ref_ys = reference
    times, xs, ys = measured
    distances = []
    j = 0
    for t, x, y in zip(times, xs, ys):
        while j + 1 < len(ref_times) and abs(ref_times[j + 1] - t) <= abs(ref_times[j] - t):
            j += 1
        distances.append(math.hypot(x - ref_xs[j], y - ref_ys[j]))
    return times, distances


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('bag', help='rosbag2 folder')
    parser.add_argument('-o', '--output', default=None,
                        help='output image (default: <bag>/compare_odom.png)')
    args = parser.parse_args()
    output = args.output or os.path.join(args.bag, 'compare_odom.png')

    data = read_positions(args.bag, ['/odom_raw', '/odom'])
    for topic, (times, _, _) in data.items():
        if not times:
            raise SystemExit(f'No messages on {topic} in {args.bag}')

    times, drift = drift_over_time(data['/odom_raw'], data['/odom'])
    start = times[0]
    elapsed = [t - start for t in times]

    figure, (ax_path, ax_drift) = plt.subplots(1, 2, figsize=(12, 5))

    _, raw_x, raw_y = data['/odom_raw']
    _, odom_x, odom_y = data['/odom']
    ax_path.plot(raw_x, raw_y, label='/odom_raw (exact)')
    ax_path.plot(odom_x, odom_y, '--', label='/odom (with noise)')
    ax_path.plot(raw_x[0], raw_y[0], 'ko', label='start')
    ax_path.set_xlabel('x, m')
    ax_path.set_ylabel('y, m')
    ax_path.set_title('Trajectories in the odom frame')
    ax_path.axis('equal')
    ax_path.grid(True)
    ax_path.legend()

    ax_drift.plot(elapsed, drift)
    ax_drift.set_xlabel('time, s')
    ax_drift.set_ylabel('position difference, m')
    ax_drift.set_title('Drift of /odom relative to /odom_raw')
    ax_drift.grid(True)

    figure.tight_layout()
    figure.savefig(output, dpi=120)

    print(f'messages: /odom_raw {len(raw_x)}, /odom {len(odom_x)}')
    print(f'duration: {elapsed[-1]:.1f} s')
    print(f'final drift: {drift[-1]:.3f} m, max drift: {max(drift):.3f} m')
    print(f'plot saved to {output}')


if __name__ == '__main__':
    main()
