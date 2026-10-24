#!/usr/bin/env python3
"""
Extract a trajectory parent -> child from the TF messages of a bag, as a TUM file.

Why this script exists: some estimates exist only as TF (e.g. map -> odom
from slam_toolbox combined with odom -> base_link from the EKF). evo can read
TF from a bag (`/tf:parent.child`) only with ROS's tf2_py, which is not
available in evo's own venv (~/.venvs/evo). This script needs only `rosbags`,
numpy and scipy, which evo already brings, and writes a TUM file that any
evo command reads.

How it works:
  1. read all transforms from /tf and /tf_static, grouped by edge
     (parent frame -> child frame);
  2. find the chain of edges from the requested parent down to the child;
  3. for every time stamp of the busiest dynamic edge in the chain,
     look up every edge at that time (static edges are constant, dynamic
     ones are interpolated: translation linearly, rotation by slerp; no
     extrapolation), and multiply the transforms along the chain;
  4. write "t x y z qx qy qz qw" lines (TUM format, header stamps in seconds).

Run with the Python of evo's venv:
  ~/.venvs/evo/bin/python src/gnssd_tools/scripts/tf_to_tum.py <bag> odom base_link out.tum
Then e.g.:
  ~/.venvs/evo/bin/evo_ape tum ground_truth.tum out.tum
"""

import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
from scipy.spatial.transform import Rotation, Slerp


class Edge:
    """All recorded transforms of one TF edge parent -> child, sorted by time."""

    def __init__(self, parent, child, static):
        self.parent = parent
        self.child = child
        self.static = static
        self.samples = []  # (time, translation[3], quaternion xyzw[4])

    def finish(self):
        """Sort the samples and prepare them for interpolation."""
        self.samples.sort(key=lambda sample: sample[0])
        self.times = np.array([s[0] for s in self.samples])
        self.translations = np.array([s[1] for s in self.samples])
        self.rotations = Rotation.from_quat([s[2] for s in self.samples])
        if not self.static and len(self.samples) > 1:
            self.slerp = Slerp(self.times, self.rotations)

    def lookup(self, t):
        """Return the 4x4 transform at time t, or None if t is outside the recorded span."""
        if self.static:
            return to_matrix(self.translations[-1], self.rotations[-1])
        if t < self.times[0] or t > self.times[-1]:
            return None
        translation = np.array([np.interp(t, self.times, self.translations[:, axis])
                                for axis in range(3)])
        return to_matrix(translation, self.slerp([t])[0])


def to_matrix(translation, rotation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation.as_matrix()
    matrix[:3, 3] = translation
    return matrix


def read_edges(bag_path):
    """Read /tf and /tf_static into {(parent, child): Edge}."""
    edges = {}
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic in ('/tf', '/tf_static')]
        for connection, _, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            static = connection.topic == '/tf_static'
            for transform in message.transforms:
                key = (transform.header.frame_id, transform.child_frame_id)
                if key not in edges:
                    edges[key] = Edge(key[0], key[1], static)
                stamp = transform.header.stamp
                t = transform.transform.translation
                q = transform.transform.rotation
                edges[key].samples.append((stamp.sec + stamp.nanosec * 1e-9,
                                           (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)))
    for edge in edges.values():
        edge.finish()
    return edges


def find_chain(edges, parent, child):
    """Return the list of edges leading from `parent` down to `child`."""
    parent_of = defaultdict(list)
    for edge in edges.values():
        parent_of[edge.child].append(edge)

    chain = []
    frame = child
    while frame != parent:
        if not parent_of[frame]:
            raise SystemExit(f'No TF path from {parent} to {child} (stuck at {frame})')
        edge = parent_of[frame][0]
        chain.append(edge)
        frame = edge.parent
    chain.reverse()
    return chain


def main():
    parser = argparse.ArgumentParser(description='TF trajectory from a rosbag2 bag -> TUM file')
    parser.add_argument('bag', help='rosbag2 folder')
    parser.add_argument('parent', help='parent frame, e.g. odom or map')
    parser.add_argument('child', help='child frame, e.g. base_link')
    parser.add_argument('output', help='TUM file to write')
    args = parser.parse_args()

    edges = read_edges(args.bag)
    chain = find_chain(edges, args.parent, args.child)
    dynamic = [edge for edge in chain if not edge.static]
    if not dynamic:
        raise SystemExit('The chain has only static transforms; there is no trajectory')

    # Output times: the stamps of the edge that is published most often.
    clock = max(dynamic, key=lambda edge: len(edge.times))

    lines = []
    for t in clock.times:
        pose = np.eye(4)
        for edge in chain:
            transform = edge.lookup(t)
            if transform is None:  # some edge has no data at this time yet
                break
            pose = pose @ transform
        else:
            x, y, z = pose[:3, 3]
            qx, qy, qz, qw = Rotation.from_matrix(pose[:3, :3]).as_quat()
            lines.append(f'{t:.9f} {x:.9f} {y:.9f} {z:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}')

    with open(args.output, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    chain_text = ' -> '.join([chain[0].parent] + [edge.child for edge in chain])
    print(f'{chain_text}: {len(lines)} poses written to {args.output} '
          f'(time base: {clock.parent} -> {clock.child}, {len(clock.times)} stamps)')


if __name__ == '__main__':
    main()
