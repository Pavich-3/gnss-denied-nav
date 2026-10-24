"""
Resource logger: CPU and memory use of the SYSTEM processes (for NFR-6, NFR-7).

Once per second (wall clock: CPU load is a wall-clock quantity) it samples,
for every process in the `processes` parameter:

* cpu_core_pct  CPU use normalised to ONE core (100 % = one core fully busy)
* cpu_total_pct the same as a share of ALL cores (cpu_core_pct / core count)
* rss_mb        resident memory (RSS), MiB

Only the localization system itself is measured (by default ekf_filter_node
and async_slam_toolbox_node). The simulation stand (gz sim, bridge, odom_noise_node,
imu_covariance_node, route_driver_node, recorder, RViz) is excluded on
purpose: on a real robot it would not exist. Later steps add their nodes
(sensor_monitor_node, map_manager_node) to the parameter list.

A process matches a name if its command line contains `__node:=<name>`
(node renamed in a launch file) or its executable is called <name>.
Processes that start later are picked up when they appear.

Output: <output_file> (CSV), one row per process and sample:
    wall_time, sim_time, process, pid, cpu_core_pct, cpu_total_pct, rss_mb

Started by record.launch.py; part of the evaluation tooling, not of the system.
"""

import csv
import os
import time

import psutil
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

FIELDS = ['wall_time', 'sim_time', 'process', 'pid', 'cpu_core_pct', 'cpu_total_pct', 'rss_mb']


def matches(process, name):
    """Return True if `process` runs the ROS node or executable called `name`."""
    try:
        command_line = process.cmdline()
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return False
    if f'__node:={name}' in command_line:
        return True
    return bool(command_line) and os.path.basename(command_line[0]) == name


class ResourceLoggerNode(Node):
    """Samples CPU and RSS of the configured processes at 1 Hz."""

    def __init__(self):
        super().__init__('resource_logger_node')
        self.names = self.declare_parameter(
            'processes', ['ekf_filter_node', 'async_slam_toolbox_node']).value
        self.output_file = self.declare_parameter('output_file', '').value
        period = self.declare_parameter('period', 1.0).value

        self.core_count = psutil.cpu_count()
        self.tracked = {}   # name -> psutil.Process

        # The CSV goes into the bag folder, which `ros2 bag record` creates.
        self.file = None
        self.writer = None

        # Wall-clock timer even though use_sim_time is true: CPU % is
        # measured over real time.
        self.create_timer(period, self.sample, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Logging {self.names} ({self.core_count} cores) to '
                               f'{self.output_file}')

    def find_processes(self):
        """Attach to every configured process that is running and not yet tracked."""
        missing = [name for name in self.names if name not in self.tracked]
        if not missing:
            return
        for process in psutil.process_iter():
            for name in missing:
                if name not in self.tracked and matches(process, name):
                    process.cpu_percent(None)   # first call only starts the measurement
                    self.tracked[name] = process

    def open_output(self):
        """Open the CSV once the bag folder exists. Returns False while it does not."""
        if self.writer is not None:
            return True
        if not os.path.isdir(os.path.dirname(self.output_file)):
            return False
        self.file = open(self.output_file, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(FIELDS)
        return True

    def sample(self):
        self.find_processes()
        if not self.open_output():
            return

        wall_time = time.time()
        sim_time = self.get_clock().now().nanoseconds * 1e-9
        for name, process in list(self.tracked.items()):
            try:
                cpu = process.cpu_percent(None)   # % of one core since the last call
                rss = process.memory_info().rss / (1024 * 1024)
            except psutil.NoSuchProcess:
                del self.tracked[name]   # it ended (e.g. at shutdown)
                continue
            self.writer.writerow([f'{wall_time:.3f}', f'{sim_time:.3f}', name, process.pid,
                                  f'{cpu:.1f}', f'{cpu / self.core_count:.2f}', f'{rss:.1f}'])
        self.file.flush()

    def close(self):
        if self.file is not None:
            self.file.close()


def main():
    rclpy.init()
    node = ResourceLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
