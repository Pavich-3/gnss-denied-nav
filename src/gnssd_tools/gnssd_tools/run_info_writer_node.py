"""
Run info writer: stores everything that identifies a recorded run.

Started by record.launch.py next to `ros2 bag record`. It writes
<bag folder>/run_info.yaml with

* world, scenario name and seed (the same seed is given to `gz sim --seed`, so it
  also fixes the Gazebo sensor noise, e.g. the gyro bias);
* the odometry noise parameters that odom_noise_node ACTUALLY used, including
  the scale errors it drew at start-up (read from the node's parameters);
* the number of laps of route_driver_node and the lidar dropout windows
  of scan_gate_node;
* the git commit of the repository (with "-dirty" if there are uncommitted
  changes), whether that was allowed, and the Gazebo Sim version.

With this file any run can be repeated exactly. The node exits after writing.
"""

import os
import subprocess
import time

from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
import yaml

from gnssd_tools.node_runner import init_with_stop_flag

NOISE_NODE = 'odom_noise_node'
DRIVER_NODE = 'route_driver_node'
GATE_NODE = 'scan_gate_node'
NOISE_PARAMETERS = [
    'enabled',
    'random_scale',
    'linear_scale_error',
    'angular_scale_error',
    'applied_linear_scale_error',
    'applied_angular_scale_error',
    'linear_noise_stddev',
    'angular_noise_stddev',
    'standstill_linear_speed',
    'standstill_angular_speed',
    'seed',
]


def command_output(command, cwd=None):
    """Run a shell command and return its stripped output, or 'unknown' on failure."""
    try:
        return subprocess.run(command, cwd=cwd, capture_output=True, text=True,
                              timeout=10, check=True).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return 'unknown'


def git_commit(repo_dir):
    commit = command_output(['git', 'rev-parse', 'HEAD'], cwd=repo_dir)
    if commit != 'unknown' and command_output(['git', 'status', '--porcelain'], cwd=repo_dir):
        commit += '-dirty'
    return commit


def gz_sim_version():
    # `gz sim --version` prints e.g. "Gazebo Sim, version 8.15.0" plus copyright lines.
    first_line = command_output(['gz', 'sim', '--version']).splitlines()[0:1]
    return first_line[0].split('version')[-1].strip() if first_line else 'unknown'


class RunInfoWriterNode(Node):
    """Collects run information once and writes it to a YAML file."""

    def __init__(self):
        super().__init__('run_info_writer_node')
        self.output_file = self.declare_parameter('output_file', '').value
        self.scenario = self.declare_parameter('scenario', '').value
        self.seed = self.declare_parameter('seed', 0).value
        self.repo_dir = self.declare_parameter('repo_dir', '').value
        self.allow_dirty = self.declare_parameter('allow_dirty', False).value
        self.world = self.declare_parameter('world', '').value
        self.timeout = self.declare_parameter('timeout', 30.0).value

    def read_parameters(self, node_name, names):
        """Ask another node for some of its parameters; return a dict, or None if no answer."""
        client = self.create_client(GetParameters, f'/{node_name}/get_parameters')
        if not client.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().warning(f'{node_name} did not answer; its parameters are not stored')
            return None
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if future.result() is None:
            return None
        return {name: parameter_value_to_python(value)
                for name, value in zip(names, future.result().values)}

    def write(self):
        noise = self.read_parameters(NOISE_NODE, NOISE_PARAMETERS)
        driver = self.read_parameters(DRIVER_NODE, ['laps'])
        gate = self.read_parameters(GATE_NODE, ['dropout_windows'])

        info = {
            'world': self.world,
            'scenario': self.scenario,
            'seed': self.seed,
            'gz_sim_seed': self.seed,
            'laps': driver['laps'] if driver is not None else 'unavailable',
            'scan_dropout_windows': (yaml.safe_load(gate['dropout_windows'])
                                     if gate is not None else 'unavailable'),
            'odom_noise': noise if noise is not None else 'unavailable',
            'git_commit': git_commit(self.repo_dir),
            'allow_dirty': self.allow_dirty,
            'gz_sim_version': gz_sim_version(),
        }
        # The bag folder is created by `ros2 bag record`, which refuses to
        # record into a folder that already exists. So we never create it
        # ourselves; we wait until the recorder has done it.
        bag_folder = os.path.dirname(self.output_file)
        deadline = time.monotonic() + self.timeout
        while not os.path.isdir(bag_folder):
            if time.monotonic() > deadline:
                self.get_logger().error(f'Bag folder {bag_folder} never appeared')
                return
            time.sleep(0.2)

        with open(self.output_file, 'w') as f:
            yaml.safe_dump(info, f, sort_keys=False)
        self.get_logger().info(f'Run info written to {self.output_file}')


def main():
    # Same clean-shutdown setup as node_runner.run_node (see its docstring).
    # The node runs once and exits; a signal just stops it early.
    init_with_stop_flag()
    node = RunInfoWriterNode()
    try:
        node.write()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
