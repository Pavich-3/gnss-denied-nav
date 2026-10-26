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

Modes (~/set_mode):
  mapping                       new map from the given pose (normally the origin)
  mapping + map_name            CONTINUE mapping on a saved pose graph, starting at the
                                given pose (slam_toolbox START_AT_GIVEN_POSE, Step 6b):
                                the new scans extend the old graph
  localization + map_name       localize on a saved graph without changing it

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

Initial pose with use_base_pose: the last VALID corrected pose propagated with
the base layer, i.e. map -> odom of the last valid update composed with the
current EKF pose in odom. While the correction is INVALID, "last valid" comes
from sensor_monitor's diagnostics (last_valid_map_to_odom_*: the update before
the alarm); otherwise it is the current map -> odom.

CPU (Step 6b): /tf and /odometry/filtered are NOT subscribed permanently (that
cost ~20 % of a core in Python). Both are read once, at the moment a switch
with use_base_pose needs them: a temporary subscription waits for one message
and is destroyed again.

Services:   ~/save_map (gnssd_interfaces/SaveMap), ~/set_mode (SetMode), ~/status (GetStatus)
Publishes:  /map_manager/events (std_msgs/String), one line per mode switch / save / load
Subscribes: /correction_valid, /diagnostics; /tf and /odometry/filtered once per switch
Calls:      /slam_toolbox/serialize_map, /sensor_monitor_node/reset_correction_check
Runs:       nav2_map_server map_saver_cli for the occupancy grid (.pgm/.yaml)

Why map_saver_cli is run here and not through /slam_toolbox/save_map: the
slam_toolbox service starts the same tool with its default 2 s timeout, which
is too short on this machine while the simulation is running (the saver did
not receive the latched /map in time, "Failed to spin map subscription").
"""

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

from gnssd_tools.map_manager_core import (ChildProcess, MapStore, MAPPING,
                                          validate_mode_command)
from gnssd_tools.node_runner import run_node

SAMPLE_TIMEOUT = 2.0      # s, wait for one /tf or /odometry/filtered message at a switch


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """Pose composition a (+) b of 2D poses (x, y, yaw)."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    return (ax + math.cos(ayaw) * bx - math.sin(ayaw) * by,
            ay + math.sin(ayaw) * bx + math.cos(ayaw) * by,
            math.atan2(math.sin(ayaw + byaw), math.cos(ayaw + byaw)))


def map_to_odom_from_tf(msg):
    """(x, y, yaw) of map -> odom if this /tf message contains it, else None."""
    for transform in msg.transforms:
        if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
            t = transform.transform
            return (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
    return None


def pose_from_odometry(msg):
    """(x, y, yaw) of an Odometry message."""
    p = msg.pose.pose
    return (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))


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
        self.correction_valid = True
        self.last_valid_map_to_odom = None   # from sensor_monitor while INVALID
        self.scan_lost = False
        # /diagnostics callbacks can run in parallel (multi-threaded executor):
        # the lock makes "scan just became LOST" true for exactly one of them,
        # otherwise two messages could both request a checkpoint.
        self.scan_state_lock = threading.Lock()

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

        self.group = group
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
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
            launch_file = 'slam_mapping.launch.py' if mode == MAPPING else \
                'slam_localization.launch.py'
            command = ['ros2', 'launch', 'gnssd_bringup', launch_file]
            if resolved_map:
                # Load the saved graph and start at `pose` (mapping: continue the graph).
                command += [f'map_file:={self.store.base_path(resolved_map)}',
                            f'x:={pose[0]:.4f}', f'y:={pose[1]:.4f}', f'yaw:={pose[2]:.5f}']
            pid = self.child.start(command, log_file=self.child_log)
            previous = self.mode
            self.mode, self.map_name = mode, resolved_map
            if resolved_map:
                self.event(f'load map={resolved_map} for {mode} pose=({pose[0]:.3f}, '
                           f'{pose[1]:.3f}, {math.degrees(pose[2]):.2f} deg)')
            self.event(f'mode {previous} -> {mode} ({reason}); previous slam stopped by '
                       f'{stopped_by}; new pid {pid}')
        self.reset_correction_check()
        return True, message

    def reset_correction_check(self):
        """Tell sensor_monitor that the new slam_toolbox starts a new map -> odom."""
        if self.reset_client.wait_for_service(timeout_sec=2.0):
            self.call(self.reset_client, Trigger.Request(), timeout=5.0)
        self.correction_valid, self.last_valid_map_to_odom = True, None

    # ---- operator services --------------------------------------------------

    def on_set_mode(self, request, response):
        pose = (request.x, request.y, request.yaw)
        if request.use_base_pose and request.map_name:
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

    def on_correction_valid(self, msg):
        self.correction_valid = msg.data

    def on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == 'sensor_monitor: correction consistency':
                values = {kv.key: kv.value for kv in status.values}
                if 'last_valid_map_to_odom_x' in values:
                    self.last_valid_map_to_odom = tuple(
                        float(values[f'last_valid_map_to_odom_{key}'])
                        for key in ('x', 'y', 'yaw'))
            elif status.name == 'sensor_monitor: scan':
                lost = status.level == DiagnosticStatus.ERROR
                with self.scan_state_lock:
                    just_lost = lost and not self.scan_lost
                    self.scan_lost = lost
                if just_lost:
                    self.on_scan_lost()

    def on_scan_lost(self):
        if not self.checkpoint_on_scan_lost or self.mode != MAPPING:
            return
        name = f'checkpoint_{int(time.time())}'
        threading.Thread(target=self.save, args=(name, False), daemon=True).start()
        self.event(f'checkpoint requested map={name} (lidar LOST)')

    def last_valid_corrected_pose(self):
        """Compose map -> odom of the last valid update with the current EKF pose."""
        if not self.correction_valid:
            map_to_odom = self.last_valid_map_to_odom   # the update before the alarm
        else:
            map_to_odom = self.read_one(TFMessage, '/tf', map_to_odom_from_tf)
        base_pose = self.read_one(Odometry, '/odometry/filtered', pose_from_odometry)
        if map_to_odom is None or base_pose is None:
            return None
        return compose(map_to_odom, base_pose)

    def read_one(self, message_type, topic, extract):
        """
        Subscribe, wait for the first message for which extract(msg) is not None, unsubscribe.

        Used instead of permanent subscriptions: the value is needed only at a
        switch. Returns None after SAMPLE_TIMEOUT.
        """
        result = []
        done = threading.Event()

        def callback(msg):
            value = extract(msg)
            if value is not None and not done.is_set():
                result.append(value)
                done.set()

        subscription = self.create_subscription(message_type, topic, callback, 100,
                                                callback_group=self.group)
        done.wait(SAMPLE_TIMEOUT)
        self.destroy_subscription(subscription)
        if not result:
            self.get_logger().error(f'No usable message on {topic} within {SAMPLE_TIMEOUT} s')
        return result[0] if result else None

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
