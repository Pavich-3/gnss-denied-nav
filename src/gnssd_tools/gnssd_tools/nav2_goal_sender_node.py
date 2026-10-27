"""
Nav2 goal sender: sends navigation goals one after another and records the outcome (STAND, Step 8).

Plays the user of Nav2 in the NFR-9 demonstration (Nav2 is an external
consumer of the localization system, not part of it). For every goal of the
`goals` parameter (flat list x, y, yaw in the map frame) it

  * sends a NavigateToPose action goal and waits for the result (or goal_timeout);
  * records: result, time to goal (simulation time), path length driven
    (integrated /ground_truth), the final TRUE position error to the goal
    (ground truth vs goal), and the final error of the system's estimate
    (TF map -> base_link vs ground truth);
  * optionally takes one screenshot of the RViz window (xwininfo finds the
    window whose title starts with `screenshot_window`, xwd captures it)
    `screenshot_delay` s after the goal was accepted, when the global path
    is shown.

At the end it writes `output_file` (YAML) and stops itself (SIGINT through
node_runner), so the demo launch file can shut everything down.

Subscribes:   /ground_truth (nav_msgs/Odometry, stand only: evaluation)
Uses:         TF map -> base_link (the system's corrected estimate)
Action:       navigate_to_pose (nav2_msgs/NavigateToPose)
"""

import math
import os
import signal
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import yaml

from gnssd_tools.geometry import normalize_angle, yaw_from_quaternion
from gnssd_tools.node_runner import run_node

RESULT_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}


class Nav2GoalSenderNode(Node):
    """Sends NavigateToPose goals in sequence and writes a result table."""

    def __init__(self):
        super().__init__('nav2_goal_sender_node')
        flat = list(self.declare_parameter('goals', [0.0, 0.0, 0.0]).value)
        self.goals = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]
        self.names = list(self.declare_parameter('goal_names', ['']).value)
        self.output_file = self.declare_parameter('output_file', '').value
        self.goal_timeout = self.declare_parameter('goal_timeout', 300.0).value
        self.start_delay = self.declare_parameter('start_delay', 20.0).value
        self.screenshot_prefix = self.declare_parameter('screenshot_prefix', '').value
        self.screenshot_window = self.declare_parameter('screenshot_window',
                                                        'gnssd_nav2_rviz').value
        self.screenshot_delay = self.declare_parameter('screenshot_delay', 10.0).value

        group = ReentrantCallbackGroup()
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                   callback_group=group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.true_pose = None          # (x, y, yaw) of the latest /ground_truth
        self.path_length = 0.0         # driven since the current goal was sent
        self.create_subscription(Odometry, '/ground_truth', self.on_ground_truth, 50,
                                 callback_group=group)
        threading.Thread(target=self.run, daemon=True).start()

    def on_ground_truth(self, msg):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        with self.lock:
            if self.true_pose is not None:
                self.path_length += math.hypot(pose[0] - self.true_pose[0],
                                               pose[1] - self.true_pose[1])
            self.true_pose = pose

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep_sim(self, seconds):
        end = self.now_s() + seconds
        while self.now_s() < end:
            time.sleep(0.05)

    def wait(self, future, timeout):
        """Wait (simulation time) until an rclpy future is done; False on timeout."""
        end = self.now_s() + timeout
        while not future.done():
            if self.now_s() > end:
                return False
            time.sleep(0.05)
        return True

    def estimate(self):
        """Return the system's corrected pose (TF map -> base_link), or None."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time(),
                                                 timeout=Duration(seconds=1.0))
        except Exception as error:   # noqa: B902 (any tf2 lookup error)
            self.get_logger().warning(f'map -> base_link not available: {error}')
            return None
        t = tf.transform
        return (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))

    def screenshot(self, index):
        if not self.screenshot_prefix:
            return None
        path = f'{self.screenshot_prefix}_goal{index + 1}.xwd'
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        # Find the window by a substring of its title: RViz appends '*' to the title as soon
        # as the scene changes, and `xwd -name` would need the exact title.
        tree = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True)
        ids = [line.split()[0] for line in tree.stdout.splitlines()
               if f'"{self.screenshot_window}' in line]
        if not ids:
            self.get_logger().warning(f'screenshot: no window titled {self.screenshot_window}')
            return None
        result = subprocess.run(['xwd', '-silent', '-id', ids[0], '-out', path],
                                capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().warning(f'screenshot failed: {result.stderr.strip()}')
            return None
        return path

    def send(self, index, goal):
        x, y, yaw = goal
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y = x, y
        target.pose.orientation.z, target.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        with self.lock:
            self.path_length = 0.0
        start = self.now_s()
        accepted = self.client.send_goal_async(NavigateToPose.Goal(pose=target))
        row = {'goal': index + 1, 'name': self.names[index] if index < len(self.names) else '',
               'target': [round(v, 3) for v in goal]}
        if not self.wait(accepted, 30.0) or not accepted.result().accepted:
            row['result'] = 'REJECTED'
            return row
        handle = accepted.result()
        result_future = handle.get_result_async()
        shot_at = start + self.screenshot_delay
        shot = None
        end = start + self.goal_timeout
        while not result_future.done() and self.now_s() < end:
            if shot is None and self.now_s() >= shot_at:
                shot = self.screenshot(index) or ''
            time.sleep(0.05)
        if not result_future.done():
            handle.cancel_goal_async()
            row['result'] = 'TIMEOUT'
        else:
            status = result_future.result().status
            row['result'] = RESULT_NAMES.get(status, f'status {status}')
        row['time_s'] = round(self.now_s() - start, 2)
        with self.lock:
            row['path_m'] = round(self.path_length, 3)
            true = self.true_pose
        est = self.estimate()
        row['true_final'] = [round(v, 3) for v in true]
        row['goal_pos_err_m'] = round(math.hypot(true[0] - x, true[1] - y), 3)
        row['goal_yaw_err_deg'] = round(math.degrees(normalize_angle(true[2] - yaw)), 2)
        if est is not None:
            row['estimate_final'] = [round(v, 3) for v in est]
            row['estimate_pos_err_m'] = round(math.hypot(est[0] - true[0], est[1] - true[1]), 3)
            row['estimate_yaw_err_deg'] = round(math.degrees(normalize_angle(est[2] - true[2])), 2)
        if shot:
            row['screenshot'] = shot
        return row

    def run(self):
        self.get_logger().info('Waiting for the navigate_to_pose action server')
        while not self.client.wait_for_server(timeout_sec=1.0):
            pass
        self.sleep_sim(self.start_delay)   # costmaps fill, map -> odom settles
        rows = []
        for index, goal in enumerate(self.goals):
            self.get_logger().info(f'Goal {index + 1}/{len(self.goals)}: {goal}')
            rows.append(self.send(index, goal))
            self.get_logger().info(f'Goal {index + 1}: {rows[-1]}')
        if self.output_file:
            os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
            with open(self.output_file, 'w') as f:
                yaml.safe_dump({'goals': rows}, f, sort_keys=False)
            self.get_logger().info(f'Results written to {self.output_file}')
        os.kill(os.getpid(), signal.SIGINT)   # clean stop through node_runner


def main():
    run_node(Nav2GoalSenderNode, threads=4)


if __name__ == '__main__':
    main()
