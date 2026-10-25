"""
Map manager: turns operator commands into actions of the correction layer (part of the SYSTEM).

As described in the thesis (chapter 2): map_manager_node receives operator
commands and turns them into actions of the correction layer - it starts
slam_toolbox in the required mode, and calls the package services to save the
pose graph with the map to a file and to load them at the next start.

Requirements covered:
  FR-10  the operator selects the mode: mapping or localization on a saved map (~/set_mode)
  FR-11  save the built map to a file on operator command (~/save_map)
  FR-12  load a previously saved map at start-up (parameters startup_mode / startup_map)
  FR-13  localize on the loaded map without changing it (localization_slam_toolbox_node
         refuses to serialize; map_manager never saves in localization mode)

How slam_toolbox is run. The slam_toolbox 2.6.10 nodes are plain (not
lifecycle) nodes and the mode is fixed when a node starts, so a mode switch
means: stop the running node, start the other one. map_manager supervises it
as a CHILD PROCESS (`ros2 launch gnssd_bringup slam_<mode>.launch.py`) in its
own process group; stopping sends SIGINT, then SIGTERM, then SIGKILL to the
whole group, so no orphan process is left (map_manager_core.ChildProcess).
After every switch sensor_monitor's correction check is reset (the new
map -> odom is a new reference).

Optional safety action (checkpoint_on_scan_lost, default off): when
sensor_monitor reports the lidar LOST while mapping, the current pose graph is
serialized to maps/checkpoint_<time>, i.e. before any post-outage scan can
corrupt it. The operator can later localize on it (map_name "last_checkpoint").

Initial pose for localization (use_base_pose): the last VALID corrected pose
propagated with the base layer, i.e. the map -> odom of the newest update
before the correction became INVALID, composed with the current EKF pose in
odom.

Services:   ~/save_map (gnssd_interfaces/SaveMap), ~/set_mode (SetMode), ~/status (GetStatus)
Publishes:  /map_manager/events (std_msgs/String), one line per mode switch / save / load
Subscribes: /tf (map -> odom), /odometry/filtered, /correction_valid, /diagnostics
Calls:      /slam_toolbox/serialize_map, /sensor_monitor_node/reset_correction_check
Runs:       nav2_map_server map_saver_cli for the occupancy grid (.pgm/.yaml)

Why map_saver_cli is run here and not through /slam_toolbox/save_map: the
slam_toolbox service starts the same tool with its default 2 s timeout, which
is too short on this machine while the simulation is running (the saver did
not receive the latched /map in time, "Failed to spin map subscription").
"""

from collections import deque
import math
import os
import subprocess
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from gnssd_interfaces.srv import GetStatus, SaveMap, SetMode
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from gnssd_tools.map_manager_core import (ChildProcess, LOCALIZATION, MapStore, MAPPING,
                                          validate_mode_command)
from gnssd_tools.node_runner import run_node

HISTORY_SECONDS = 300.0   # how much map -> odom history is kept for use_base_pose


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """Pose composition a (+) b of 2D poses (x, y, yaw)."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    return (ax + math.cos(ayaw) * bx - math.sin(ayaw) * by,
            ay + math.sin(ayaw) * bx + math.cos(ayaw) * by,
            math.atan2(math.sin(ayaw + byaw), math.cos(ayaw + byaw)))


class MapManagerNode(Node):
    """Supervises slam_toolbox and serves the operator commands."""

    def __init__(self):
        super().__init__('map_manager_node')
        self.store = MapStore(self.declare_parameter('maps_dir', 'maps').value)
        self.startup_mode = self.declare_parameter('startup_mode', MAPPING).value
        self.startup_map = self.declare_parameter('startup_map', '').value
        self.startup_pose = list(self.declare_parameter('startup_pose', [0.0, 0.0, 0.0]).value)
        self.checkpoint_on_scan_lost = self.declare_parameter(
            'checkpoint_on_scan_lost', False).value
        self.service_timeout = self.declare_parameter('service_timeout', 30.0).value
        self.grid_save_timeout = self.declare_parameter('grid_save_timeout', 10.0).value
        log_dir = self.declare_parameter('child_log_dir', os.path.expanduser('~/.ros/log')).value
        self.child_log = os.path.join(log_dir, f'map_manager_slam_{int(time.time())}.log')

        self.child = ChildProcess(self.declare_parameter('stop_timeout', 10.0).value)
        self.mode = 'stopped'
        self.map_name = ''
        self.switch_lock = threading.Lock()

        # Inputs for use_base_pose and the checkpoint.
        self.map_to_odom_history = deque()   # (stamp, (x, y, yaw)), one per new stamp
        self.base_pose = None                # EKF pose in odom (x, y, yaw)
        self.correction_valid = True
        self.alarm_time = None               # stamp of the invalid map -> odom update
        self.scan_lost = False

        group = ReentrantCallbackGroup()
        self.events = self.create_publisher(
            String, '/map_manager/events', QoSProfile(depth=50,
                                                      durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.serialize_client = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map', callback_group=group)
        self.reset_client = self.create_client(
            Trigger, '/sensor_monitor_node/reset_correction_check', callback_group=group)

        self.create_service(SaveMap, '~/save_map', self.on_save_map, callback_group=group)
        self.create_service(SetMode, '~/set_mode', self.on_set_mode, callback_group=group)
        self.create_service(GetStatus, '~/status', self.on_status, callback_group=group)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100, callback_group=group)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_base_pose, 10,
                                 callback_group=group)
        self.create_subscription(Bool, '/correction_valid', self.on_correction_valid, latched,
                                 callback_group=group)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10,
                                 callback_group=group)

        # FR-12: start in the configured mode once the node is up.
        self.startup_timer = self.create_timer(0.5, self.start_up, callback_group=group)

    # ---- start-up and mode switching -------------------------------------

    def start_up(self):
        self.startup_timer.cancel()
        x, y, yaw = self.startup_pose
        ok, message = self.switch_mode(self.startup_mode, self.startup_map, (x, y, yaw),
                                       'start-up')
        if not ok:
            self.get_logger().error(f'Start-up failed: {message}')

    def switch_mode(self, mode, map_name, pose, reason):
        """Stop the running slam_toolbox, start the requested one. Returns (ok, message)."""
        ok, message, resolved_map = validate_mode_command(
            mode, map_name, False, True, self.store)
        if not ok:
            return False, message
        with self.switch_lock:
            stopped_by = self.child.stop()
            if mode == MAPPING:
                command = ['ros2', 'launch', 'gnssd_bringup', 'slam_mapping.launch.py']
            else:
                command = ['ros2', 'launch', 'gnssd_bringup', 'slam_localization.launch.py',
                           f'map_file:={self.store.base_path(resolved_map)}',
                           f'x:={pose[0]:.4f}', f'y:={pose[1]:.4f}', f'yaw:={pose[2]:.5f}']
            pid = self.child.start(command, log_file=self.child_log)
            previous = self.mode
            self.mode, self.map_name = mode, resolved_map
            if mode == LOCALIZATION:
                self.event(f'load map={resolved_map} pose=({pose[0]:.3f}, {pose[1]:.3f}, '
                           f'{math.degrees(pose[2]):.2f} deg)')
            self.event(f'mode {previous} -> {mode} ({reason}); previous slam stopped by '
                       f'{stopped_by}; new pid {pid}')
        self.reset_correction_check()
        return True, message

    def reset_correction_check(self):
        """Tell sensor_monitor that the new slam_toolbox starts a new map -> odom."""
        if self.reset_client.wait_for_service(timeout_sec=2.0):
            self.call(self.reset_client, Trigger.Request(), timeout=5.0)
        self.correction_valid, self.alarm_time = True, None
        self.map_to_odom_history.clear()

    # ---- operator services --------------------------------------------------

    def on_set_mode(self, request, response):
        pose = (request.x, request.y, request.yaw)
        if request.mode == LOCALIZATION and request.use_base_pose:
            pose = self.last_valid_corrected_pose()
            if pose is None:
                response.success = False
                response.message = 'No valid corrected pose known yet, give x/y/yaw explicitly'
                return response
        response.success, response.message = self.switch_mode(
            request.mode, request.map_name, pose,
            'operator' + (' (initial pose from the base layer)' if request.use_base_pose else ''))
        return response

    def on_save_map(self, request, response):
        if self.mode != MAPPING:
            response.success = False
            response.message = f'Saving is only possible in mapping mode (now: {self.mode})'
            return response
        if not self.store.is_valid_name(request.name):
            response.success = False
            response.message = f'Invalid map name "{request.name}"'
            return response
        ok, files = self.save(request.name, with_grid=True)
        response.success = ok
        response.files = files
        response.message = 'saved' if ok else 'saving failed, see the log'
        return response

    def save(self, name, with_grid):
        """Serialize the pose graph (and optionally the occupancy grid). Returns (ok, files)."""
        os.makedirs(self.store.folder, exist_ok=True)
        base = self.store.base_path(name)
        ok = True
        result = self.call(self.serialize_client, SerializePoseGraph.Request(filename=base))
        ok &= result is not None and result.result == SerializePoseGraph.Response.RESULT_SUCCESS
        files = self.store.pose_graph_files(name)
        if with_grid:
            ok &= self.save_grid(base)
            files += self.store.grid_files(name)
        present = [f for f in files if os.path.isfile(f)]
        ok &= len(present) == len(files)
        file_names = ','.join(os.path.basename(f) for f in present)
        self.event(f'save map={name} ok={ok} files={file_names}')
        return ok, present

    def save_grid(self, base):
        """Write the latched /map to <base>.pgm/.yaml with nav2's map_saver_cli."""
        command = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', base,
                   '--ros-args', '-p', f'save_map_timeout:={self.grid_save_timeout}',
                   '-p', 'map_subscribe_transient_local:=true']
        try:
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    timeout=self.grid_save_timeout + 20.0, text=True)
        except subprocess.TimeoutExpired:
            self.get_logger().error('map_saver_cli did not finish')
            return False
        if result.returncode != 0:
            self.get_logger().error(f'map_saver_cli failed:\n{result.stdout}')
        return result.returncode == 0

    def on_status(self, _request, response):
        response.mode = self.mode
        response.map_name = self.map_name
        response.pid = self.child.pid
        response.uptime = self.child.uptime()
        response.message = 'running' if self.child.is_running() else 'slam_toolbox not running'
        return response

    # ---- inputs -------------------------------------------------------------

    def on_tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id != 'map' or transform.child_frame_id != 'odom':
                continue
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            if self.map_to_odom_history and stamp <= self.map_to_odom_history[-1][0]:
                continue   # the same update re-sent
            pose = (transform.transform.translation.x, transform.transform.translation.y,
                    yaw_from_quaternion(transform.transform.rotation))
            self.map_to_odom_history.append((stamp, pose))
            while stamp - self.map_to_odom_history[0][0] > HISTORY_SECONDS:
                self.map_to_odom_history.popleft()

    def on_base_pose(self, msg):
        p = msg.pose.pose
        self.base_pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_correction_valid(self, msg):
        self.correction_valid = msg.data

    def on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == 'sensor_monitor: correction consistency':
                values = {kv.key: kv.value for kv in status.values}
                if 'alarm_time_s' in values:
                    self.alarm_time = float(values['alarm_time_s'])
            elif status.name == 'sensor_monitor: scan':
                lost = status.level == DiagnosticStatus.ERROR
                if lost and not self.scan_lost:
                    self.on_scan_lost()
                self.scan_lost = lost

    def on_scan_lost(self):
        if not self.checkpoint_on_scan_lost or self.mode != MAPPING:
            return
        name = f'checkpoint_{int(time.time())}'
        threading.Thread(target=self.save, args=(name, False), daemon=True).start()
        self.event(f'checkpoint requested map={name} (lidar LOST)')

    def last_valid_corrected_pose(self):
        """Compose map -> odom of the last valid update (before the alarm) with the EKF pose."""
        if self.base_pose is None or not self.map_to_odom_history:
            return None
        candidates = self.map_to_odom_history
        if not self.correction_valid and self.alarm_time is not None:
            candidates = [entry for entry in candidates if entry[0] < self.alarm_time - 1e-6]
        if not candidates:
            return None
        return compose(candidates[-1][1], self.base_pose)

    # ---- helpers --------------------------------------------------------------

    def call(self, client, request, timeout=None):
        """Call a service from inside a callback (multi-threaded executor); None on failure."""
        timeout = self.service_timeout if timeout is None else timeout
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout):
            self.get_logger().error(f'Service {client.srv_name} did not answer in {timeout} s')
            return None
        return future.result()

    def event(self, text):
        self.get_logger().info(f'[event] {text}')
        self.events.publish(String(data=text))

    def shutdown(self):
        stopped_by = self.child.stop()
        self.get_logger().info(f'slam_toolbox stopped by {stopped_by}')


def main():
    run_node(MapManagerNode, on_shutdown=MapManagerNode.shutdown, threads=4)


if __name__ == '__main__':
    main()
