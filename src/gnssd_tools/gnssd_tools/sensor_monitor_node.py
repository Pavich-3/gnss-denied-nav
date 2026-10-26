"""
Sensor monitor: tells the operator when a sensor stream is lost (part of the SYSTEM).

As described in the thesis (chapter 2): the node subscribes to the wheel
odometry, IMU and lidar topics, records the arrival time of the last message
of each, and if the silence exceeds that sensor's timeout, reports the loss
of the source to the operator. It only OBSERVES the streams - it is not
between the sensors and the other nodes - so if it fails, the rest of the
system keeps working. Timeouts are parameters (config/sensor_monitor.yaml).

How it works:
  * one subscription per sensor; the callback stores the arrival time in a
    SensorWatch (gnssd_tools/sensor_health.py, plain Python, unit-tested);
  * a 10 Hz timer re-checks every sensor; state changes (OK -> LOST,
    LOST -> OK) are logged exactly once: WARN with the measured silence,
    INFO with the outage duration;
  * the report to the operator is a diagnostic_msgs/DiagnosticArray on
    /diagnostics, one DiagnosticStatus per sensor.

Clock: the STEADY (monotonic system) clock, not the simulation clock.
  * On the robot the monitor would use the system clock anyway: a sensor
    that has been silent for 0.5 s of real time is lost, whatever any
    other clock says.
  * In simulation the real-time factor is ~1 (measured 0.993-1.001), so a
    timeout in wall time equals the same timeout in sim time.
  * With use_sim_time an rclpy node handles every /clock message (1 kHz from
    Gazebo), which costs ~43 % of a CPU core - a simulation artifact that
    would distort the resource measurements (NFR-6).
  So the node runs with use_sim_time false. To make sure the equivalence of
  wall and sim time holds, it samples ONE /clock message every 10 s
  (subscribe, take one message, unsubscribe) and logs a WARN if the
  real-time factor deviates from 1 by more than `rtf_tolerance`.

Correction consistency (Step 5c): the node also watches the correction
layer. Every new map -> odom update from /tf goes through
CorrectionConsistencyCheck (gnssd_tools/correction_consistency.py, plain
Python, unit-tested): a heading change of map -> odom larger than the drift
the base layer can have (floor + k * gyro bias * dt) marks the corrected
estimate INVALID (latched until the operator calls ~/reset_correction_check).
The state is published on /correction_valid (std_msgs/Bool, transient local)
for consumers that can fall back to the base layer, and as the status
"sensor_monitor: correction consistency" on /diagnostics. The node still only
observes: it never changes map -> odom or any other estimate.

Why /diagnostics: it is the standard ROS channel for health reports. The
operator can watch it with existing tools (rqt_robot_monitor,
`ros2 topic echo /diagnostics`), and diagnostic_aggregator / logging tools
already understand it, so no custom message or GUI is needed.

Subscriptions use the sensor-data QoS profile (best effort): it is compatible
with the reliable publishers of the stand and never slows them down.

CPU (Step 6b): the sensor topics are subscribed RAW (serialized bytes, not
deserialized): only the arrival time of a message matters, and deserializing
100 Hz IMU, 50 Hz odometry and 10 Hz scans in Python was most of the node's
cost. /odometry/filtered is no longer subscribed: it was used only for the
informational "position shift" of a correction, the check needs the heading
of map -> odom alone.

When the check becomes INVALID, the map -> odom of the last valid update is
added to the diagnostic status (last_valid_map_to_odom_x/_y/_yaw):
map_manager_node uses it as the reference for a recovery with use_base_pose,
so it does not need its own /tf history.

Subscribes:  the topics in config/sensor_monitor.yaml (/odom, /imu, /scan), raw;
             /tf (map -> odom) for the consistency check;
             /clock for one message every rtf_check_period seconds
Publishes:   /diagnostics (diagnostic_msgs/DiagnosticArray)
             /correction_valid (std_msgs/Bool, transient local)
Services:    ~/reset_correction_check (std_srvs/Trigger), operator reset
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from gnssd_tools.correction_consistency import CorrectionConsistencyCheck, INVALID
from gnssd_tools.node_runner import run_node
from gnssd_tools.sensor_health import LOST, OK, RealTimeFactorCheck, SensorWatch, WAITING

MESSAGE_TYPES = {'odometry': Odometry, 'imu': Imu, 'scan': LaserScan}

# Diagnostic level for each state of a sensor.
LEVELS = {WAITING: DiagnosticStatus.STALE, OK: DiagnosticStatus.OK, LOST: DiagnosticStatus.ERROR}


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def steady_now():
    """Seconds on the monotonic system clock (never jumps, unaffected by /clock)."""
    return time.monotonic()


class SensorMonitorNode(Node):
    """Watches the sensor topics and reports their state on /diagnostics."""

    def __init__(self):
        super().__init__('sensor_monitor_node')

        sensor_names = self.declare_parameter('sensors', ['odom', 'imu', 'scan']).value
        check_rate = self.declare_parameter('check_rate', 10.0).value
        report_rate = self.declare_parameter('report_rate', 1.0).value
        rate_window = self.declare_parameter('rate_window', 2.0).value
        # Below this fraction of the expected rate an OK sensor is reported as WARN.
        self.low_rate_fraction = self.declare_parameter('low_rate_fraction', 0.5).value
        rtf_check_period = self.declare_parameter('rtf_check_period', 10.0).value
        self.rtf_check = RealTimeFactorCheck(self.declare_parameter('rtf_tolerance', 0.05).value)

        self.watches = {}
        self.topics = {}
        for name in sensor_names:
            topic = self.declare_parameter(f'{name}.topic', '').value
            message_type = self.declare_parameter(f'{name}.type', '').value
            timeout = self.declare_parameter(f'{name}.timeout', 0.0).value
            expected_rate = self.declare_parameter(f'{name}.expected_rate', 0.0).value
            if not topic or message_type not in MESSAGE_TYPES or timeout <= 0.0:
                raise ValueError(f'Sensor {name}: set topic, type (odometry/imu/scan) and a '
                                 'positive timeout in sensor_monitor.yaml')

            self.watches[name] = SensorWatch(name, timeout, expected_rate, rate_window)
            self.topics[name] = topic
            # raw=True: the callback gets the serialized bytes, nothing is deserialized.
            self.create_subscription(MESSAGE_TYPES[message_type], topic,
                                     self.make_callback(name), qos_profile_sensor_data,
                                     raw=True)
            self.get_logger().info(f'Watching {name} on {topic}: timeout {timeout} s, '
                                   f'expected {expected_rate} Hz (steady clock)')

        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.setup_correction_check()

        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / check_rate, self.check_all, clock=steady_clock)
        self.create_timer(1.0 / report_rate, self.publish_diagnostics, clock=steady_clock)
        self.create_timer(rtf_check_period, self.sample_sim_clock, clock=steady_clock)
        self.clock_subscription = None

    # ---- correction consistency -------------------------------------------

    def setup_correction_check(self):
        drift_sigma = self.declare_parameter('correction_check.drift_sigma', 0.0).value
        if drift_sigma <= 0.0:
            raise ValueError('correction_check.drift_sigma must be set (sim.launch.py passes '
                             'the gyro bias from robot_params.xacro)')
        self.correction_check = CorrectionConsistencyCheck(
            k=self.declare_parameter('correction_check.k', 3.0).value,
            floor_deg=self.declare_parameter('correction_check.floor_deg', 3.0).value,
            drift_sigma=drift_sigma,
            window=self.declare_parameter('correction_check.window', 1.0).value)
        # An alarm within this time after the lidar came back is reported as
        # "preceded by a lidar outage" (the check itself does not use it).
        self.outage_memory = self.declare_parameter('correction_check.outage_memory', 30.0).value
        self.last_scan_outage_end = None      # steady time of the last scan LOST -> OK

        self.map_to_odom = None               # (x, y, yaw) of the last update
        self.last_valid_map_to_odom = None    # the last update before the alarm (for recovery)
        self.last_heading_change = 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.valid_publisher = self.create_publisher(Bool, '/correction_valid', latched)
        self.valid_publisher.publish(Bool(data=True))
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_service(Trigger, '~/reset_correction_check', self.on_reset_request)

    def on_tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                self.on_map_to_odom(transform)

    def on_map_to_odom(self, transform):
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        x, y = transform.transform.translation.x, transform.transform.translation.y
        yaw = yaw_from_quaternion(transform.transform.rotation)

        previous = self.map_to_odom
        if previous is not None and (x, y, yaw) != previous:
            old_yaw = previous[2]
            self.last_heading_change = math.degrees(
                abs(math.atan2(math.sin(yaw - old_yaw), math.cos(yaw - old_yaw))))
        self.map_to_odom = (x, y, yaw)

        alarm = self.correction_check.add_update(stamp, yaw)
        # Remember this map -> odom as "last valid" only if the check really
        # ACCEPTED it (it is then its newest accepted state). add_update() also
        # returns None for an update it ignores: slam_toolbox can re-send a NEW
        # value under an already seen stamp - after the Step 6b outage the wrong
        # 39 deg correction came with the stamp of the update before it, was
        # skipped as a re-send and caught 0.1 s later (next stamp).
        accepted = self.correction_check.accepted
        if alarm is None and accepted and accepted[-1] == (stamp, yaw):
            self.last_valid_map_to_odom = (x, y, yaw)
        if alarm is not None:
            after_outage = (self.last_scan_outage_end is not None and
                            steady_now() - self.last_scan_outage_end < self.outage_memory)
            self.get_logger().error(
                f'Correction INVALID at t = {alarm["time"]:.2f} s: map->odom heading changed by '
                f'{alarm["heading_change_deg"]:.2f} deg in {alarm["dt"]:.1f} s, allowed '
                f'{alarm["allowed_deg"]:.2f} deg; '
                + ('preceded by a lidar outage' if after_outage else 'no lidar outage before it'))
            self.valid_publisher.publish(Bool(data=False))
            self.publish_diagnostics()

    def on_reset_request(self, _request, response):
        was_invalid = self.correction_check.state == INVALID
        self.correction_check.reset()
        self.last_valid_map_to_odom = None
        self.valid_publisher.publish(Bool(data=True))
        self.get_logger().info('Correction consistency reset by the operator'
                               + (' (was INVALID)' if was_invalid else ''))
        response.success = True
        response.message = 'correction check reset'
        self.publish_diagnostics()
        return response

    def correction_status(self):
        check = self.correction_check
        status = DiagnosticStatus()
        status.name = 'sensor_monitor: correction consistency'
        status.hardware_id = 'map->odom'
        values = [
            KeyValue(key='state', value=check.state),
            KeyValue(key='last_heading_change_deg', value=f'{self.last_heading_change:.3f}'),
            KeyValue(key='floor_deg', value=f'{math.degrees(check.floor):.2f}'),
            KeyValue(key='k', value=f'{check.k:.1f}'),
            KeyValue(key='drift_sigma_rad_s', value=f'{check.drift_sigma:.2e}'),
        ]
        if check.state == INVALID:
            status.level = DiagnosticStatus.ERROR
            status.message = (f'INVALID since t = {check.alarm["time"]:.2f} s: heading jump '
                              f'{check.alarm["heading_change_deg"]:.1f} deg > allowed '
                              f'{check.alarm["allowed_deg"]:.1f} deg')
            values.append(KeyValue(key='alarm_time_s', value=f'{check.alarm["time"]:.3f}'))
            values.append(KeyValue(key='alarm_heading_change_deg',
                                   value=f'{check.alarm["heading_change_deg"]:.2f}'))
            values.append(KeyValue(key='alarm_allowed_deg',
                                   value=f'{check.alarm["allowed_deg"]:.2f}'))
            if self.last_valid_map_to_odom is not None:
                for key, value in zip(('x', 'y', 'yaw'), self.last_valid_map_to_odom):
                    values.append(KeyValue(key=f'last_valid_map_to_odom_{key}',
                                           value=f'{value:.6f}'))
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'VALID'
        status.values = values
        return status

    def make_callback(self, name):
        """Return a subscription callback that registers a message of sensor `name`."""
        def callback(_msg):
            self.report(self.watches[name].on_message(steady_now()))
        return callback

    def check_all(self):
        now = steady_now()
        for watch in self.watches.values():
            self.report(watch.check(now))

    # ---- real-time factor: one /clock message every rtf_check_period -------

    def sample_sim_clock(self):
        if self.clock_subscription is None:
            self.clock_subscription = self.create_subscription(
                ClockMessage, '/clock', self.on_clock_sample, 1)

    def on_clock_sample(self, msg):
        # Unsubscribe right away: only this one message is needed.
        self.destroy_subscription(self.clock_subscription)
        self.clock_subscription = None
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        change = self.rtf_check.add_sample(steady_now(), sim)
        if change == 'deviation':
            self.get_logger().warning(
                f'Real-time factor {self.rtf_check.rtf:.3f} deviates from 1 by more than '
                f'{self.rtf_check.tolerance:.0%}: steady-clock timeouts no longer equal sim time')
        elif change == 'back to normal':
            self.get_logger().info(f'Real-time factor back to {self.rtf_check.rtf:.3f}')

    # ---- reporting ----------------------------------------------------------

    def report(self, transition):
        """Log a state change once; publish the diagnostics immediately."""
        if transition is None:
            return
        name = transition.sensor
        if transition.new_state == LOST:
            self.get_logger().warning(
                f'Sensor {name} LOST: no message for {transition.duration:.2f} s '
                f'(timeout {self.watches[name].timeout} s)')
        elif transition.new_state == OK and transition.old_state == LOST:
            if name == 'scan':
                self.last_scan_outage_end = steady_now()
            self.get_logger().info(
                f'Sensor {name} OK again after an outage of {transition.duration:.2f} s')
        elif transition.new_state == OK:
            self.get_logger().info(f'Sensor {name} OK (first message)')
        else:
            self.get_logger().info(f'Sensor {name}: clock jumped back, waiting')
        self.publish_diagnostics()

    def publish_diagnostics(self):
        now = steady_now()
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()   # system time
        for name, watch in self.watches.items():
            status = DiagnosticStatus()
            status.name = f'sensor_monitor: {name}'
            status.hardware_id = self.topics[name]
            status.level = LEVELS[watch.state]
            rate = watch.measured_rate()
            age = watch.age(now)

            if watch.state == WAITING:
                status.message = 'no message received yet'
            elif watch.state == LOST:
                status.message = f'LOST: no message for {age:.2f} s'
            elif watch.rate_known(now) and rate < self.low_rate_fraction * watch.expected_rate:
                status.level = DiagnosticStatus.WARN
                status.message = f'rate low: {rate:.1f} Hz of {watch.expected_rate:.1f} Hz'
            else:
                status.message = 'OK'

            status.values = [
                KeyValue(key='last_age_s', value='none' if age is None else f'{age:.3f}'),
                KeyValue(key='timeout_s', value=f'{watch.timeout:.3f}'),
                KeyValue(key='expected_rate_hz', value=f'{watch.expected_rate:.1f}'),
                KeyValue(key='measured_rate_hz', value=f'{rate:.1f}'),
                KeyValue(key='real_time_factor',
                         value='unknown' if self.rtf_check.rtf is None
                         else f'{self.rtf_check.rtf:.3f}'),
            ]
            array.status.append(status)
        array.status.append(self.correction_status())
        self.diagnostics_publisher.publish(array)


def main():
    run_node(SensorMonitorNode)


if __name__ == '__main__':
    main()
