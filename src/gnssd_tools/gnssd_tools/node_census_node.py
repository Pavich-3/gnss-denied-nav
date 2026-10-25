"""
Node census: writes the list of running ROS nodes every few seconds (evaluation tool).

Used to check node liveness (FR-14, NFR-8): during a run with a sensor
dropout no node of the system may exit or restart. Every `period` seconds of
sim time one line is appended to <output_file> (CSV):
    sim_time, node_count, nodes (sorted, separated by ';')
Comparing the first complete line with the last one before shutdown shows
whether any node disappeared or appeared during the run.

Started by record.launch.py; part of the evaluation tooling, not of the system.
"""

import csv
import os

from rclpy.node import Node

from gnssd_tools.node_runner import run_node


class NodeCensusNode(Node):
    """Appends the current node list to a CSV file periodically."""

    def __init__(self):
        super().__init__('node_census_node')
        self.output_file = self.declare_parameter('output_file', '').value
        period = self.declare_parameter('period', 5.0).value
        self.writer = None
        self.file = None
        self.create_timer(period, self.write_census)

    def write_census(self):
        if self.writer is None:
            if not os.path.isdir(os.path.dirname(self.output_file)):
                return   # the bag folder (created by ros2 bag record) does not exist yet
            self.file = open(self.output_file, 'w', newline='')
            self.writer = csv.writer(self.file)
            self.writer.writerow(['sim_time', 'node_count', 'nodes'])

        names = sorted(f'{namespace.rstrip("/")}/{name}'
                       for name, namespace in self.get_node_names_and_namespaces())
        t = self.get_clock().now().nanoseconds * 1e-9
        self.writer.writerow([f'{t:.1f}', len(names), ';'.join(names)])
        self.file.flush()

    def close(self):
        if self.file is not None:
            self.file.close()


def main():
    run_node(NodeCensusNode, on_shutdown=NodeCensusNode.close)


if __name__ == '__main__':
    main()
