"""
Scan gate: switches the lidar off during chosen time windows (part of the STAND).

Used for the dropout scenarios (FR-14, NFR-8): what happens to the
localization when the lidar stops sending data for a while?

The bridge publishes the simulated lidar on /scan_raw; this node relays every
message unchanged (same header, same QoS class) to /scan, EXCEPT while the
simulation time is inside one of the dropout windows - then the messages are
dropped. The gate is always in the chain (with no windows it is a plain
pass-through), so every run has the same topology and the same small delay.

On a real robot this node does not exist; the lidar driver publishes /scan.

Parameter `dropout_windows` is a YAML string with a list of [t_start, t_end]
pairs in seconds of sim time; t_end = -1 means "until the end", e.g.
    "[]"                      pass-through (default)
    "[[140, 200]]"            lidar off from 140 s to 200 s
    "[[50, 60], [300, -1]]"   two windows, the second one never ends
The gate logs once when it closes and once when it opens (with sim time).

Time base: the header stamp of each scan. It is the simulation time at which
Gazebo produced the scan, so the node does not need the ROS clock at all and
runs WITHOUT use_sim_time: with use_sim_time every rclpy node processes all
/clock messages (Gazebo publishes 1000 per second), which alone costs ~45 % of
a CPU core in Python - far more than relaying 10 scans per second.

Subscribes:  /scan_raw (sensor_msgs/LaserScan, from the bridge)
Publishes:   /scan     (sensor_msgs/LaserScan)
"""

import math

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import yaml

from gnssd_tools.node_runner import run_node


def parse_windows(text):
    """Turn "[[t1, t2], ...]" into a list of (start, end) floats; end -1 -> infinity."""
    windows = []
    for pair in yaml.safe_load(text) or []:
        start, end = float(pair[0]), float(pair[1])
        windows.append((start, math.inf if end < 0 else end))
    return windows


class ScanGateNode(Node):
    """Relays /scan_raw to /scan except inside the dropout windows."""

    def __init__(self):
        super().__init__('scan_gate_node')
        self.windows = parse_windows(self.declare_parameter('dropout_windows', '[]').value)
        self.is_open = True

        # The same QoS class as the bridge's publisher (reliable, depth 10),
        # so the subscribers of /scan see exactly what they saw before.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(LaserScan, 'scan', qos)
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos)

        if self.windows:
            self.get_logger().info(f'Lidar dropout windows (sim time, s): {self.windows}')
        else:
            self.get_logger().info('No dropout windows: pass-through')

    def inside_window(self, t):
        return any(start <= t < end for start, end in self.windows)

    def on_scan(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9   # sim time of the scan
        should_be_open = not self.inside_window(t)

        if should_be_open != self.is_open:
            self.is_open = should_be_open
            state = 'OPEN (lidar back)' if should_be_open else 'CLOSED (lidar dropout)'
            self.get_logger().info(f'Scan gate {state} at t = {t:.2f} s')

        if self.is_open:
            self.publisher.publish(msg)


def main():
    run_node(ScanGateNode)


if __name__ == '__main__':
    main()
