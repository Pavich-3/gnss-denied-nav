#!/usr/bin/env python3
"""
Save the last /map of a bag as a PNG figure (for the thesis).

The occupancy grid is drawn in its map coordinates: free cells white,
occupied black, unknown grey. Optionally the /ground_truth trajectory is drawn
on top: because every run starts at the world origin, the map frame and the
world frame coincide, so no transformation is needed.

Run with the Python of evo's venv (rosbags, numpy, matplotlib):
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/map_to_png.py <bag> out.png
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/map_to_png.py <bag> out.png --ground-truth
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402  (must come after matplotlib.use)
import numpy as np  # noqa: E402
from rosbags.highlevel import AnyReader  # noqa: E402
from rosbags.typesys import get_typestore, Stores  # noqa: E402


def read_bag(bag_path, with_ground_truth):
    """Return the last OccupancyGrid message and the ground-truth x, y lists."""
    last_map = None
    xs, ys = [], []
    topics = ['/map'] + (['/ground_truth'] if with_ground_truth else [])
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            if connection.topic == '/map':
                last_map = msg
            else:
                xs.append(msg.pose.pose.position.x)
                ys.append(msg.pose.pose.position.y)
    if last_map is None:
        raise SystemExit(f'No /map message in {bag_path}')
    return last_map, xs, ys


def main():
    parser = argparse.ArgumentParser(description='Last /map of a bag -> PNG')
    parser.add_argument('bag', help='rosbag2 folder')
    parser.add_argument('output', help='PNG file to write')
    parser.add_argument('--ground-truth', action='store_true',
                        help='draw the /ground_truth trajectory on the map')
    args = parser.parse_args()

    grid, xs, ys = read_bag(args.bag, args.ground_truth)
    info = grid.info
    cells = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)

    # -1 unknown -> grey, 0 free -> white, 100 occupied -> black.
    image = np.full(cells.shape, 0.75)
    known = cells >= 0
    image[known] = 1.0 - cells[known] / 100.0

    x0 = info.origin.position.x
    y0 = info.origin.position.y
    extent = [x0, x0 + info.width * info.resolution, y0, y0 + info.height * info.resolution]

    figure, axes = plt.subplots(figsize=(8, 7))
    axes.imshow(image, cmap='gray', vmin=0.0, vmax=1.0, origin='lower', extent=extent)
    if args.ground_truth:
        axes.plot(xs, ys, linewidth=0.8, color='tab:blue', label='/ground_truth')
        axes.plot(0, 0, 'o', color='tab:red', label='start (origin)')
        axes.legend(loc='upper right')
    axes.set_xlabel('x, m')
    axes.set_ylabel('y, m')
    axes.set_title(f'slam_toolbox map, {info.width} x {info.height} cells '
                   f'of {info.resolution:.2f} m')
    figure.tight_layout()
    figure.savefig(args.output, dpi=150)
    print(f'map {info.width} x {info.height} cells, resolution {info.resolution:.2f} m, '
          f'origin ({x0:.2f}, {y0:.2f}) -> {args.output}')


if __name__ == '__main__':
    main()
