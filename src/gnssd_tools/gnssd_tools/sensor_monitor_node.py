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

Why /diagnostics: it is the standard ROS channel for health reports. The
operator can watch it with existing tools (rqt_robot_monitor,
`ros2 topic echo /diagnostics`), and diagnostic_aggregator / logging tools
already understand it, so no custom message or GUI is needed.

Subscriptions use the sensor-data QoS profile (best effort): it is compatible
with the reliable publishers of the stand and never slows them down.

Subscribes:  the topics in config/sensor_monitor.yaml (/odom, /imu, /scan);
             /clock for one message every rtf_check_period seconds
Publishes:   /diagnostics (diagnostic_msgs/DiagnosticArray)
"""

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu, LaserScan

from gnssd_tools.node_runner import run_node
from gnssd_tools.sensor_health import LOST, OK, RealTimeFactorCheck, SensorWatch, WAITING

MESSAGE_TYPES = {'odometry': Odometry, 'imu': Imu, 'scan': LaserScan}

# Diagnostic level for each state of a sensor.
LEVELS = {WAITING: DiagnosticStatus.STALE, OK: DiagnosticStatus.OK, LOST: DiagnosticStatus.ERROR}


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
            self.create_subscription(MESSAGE_TYPES[message_type], topic,
                                     self.make_callback(name), qos_profile_sensor_data)
            self.get_logger().info(f'Watching {name} on {topic}: timeout {timeout} s, '
                                   f'expected {expected_rate} Hz (steady clock)')

        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / check_rate, self.check_all, clock=steady_clock)
        self.create_timer(1.0 / report_rate, self.publish_diagnostics, clock=steady_clock)
        self.create_timer(rtf_check_period, self.sample_sim_clock, clock=steady_clock)
        self.clock_subscription = None

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
        self.diagnostics_publisher.publish(array)


def main():
    run_node(SensorMonitorNode)


if __name__ == '__main__':
    main()
