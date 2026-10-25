"""
Sensor monitor: tells the operator when a sensor stream is lost (part of the SYSTEM).

As described in the thesis (chapter 2): the node subscribes to the wheel
odometry, IMU and lidar topics, records the arrival time of the last message
of each, and if the silence exceeds that sensor's timeout, reports the loss
of the source to the operator. It only OBSERVES the streams - it is not
between the sensors and the other nodes - so if it fails, the rest of the
system keeps working. Timeouts are parameters (config/sensor_monitor.yaml).

How it works:
  * one subscription per sensor; the callback stores the arrival time
    (node clock = sim time) in a SensorWatch (gnssd_tools/sensor_health.py,
    plain Python, unit-tested);
  * a 10 Hz timer re-checks every sensor; state changes (OK -> LOST,
    LOST -> OK) are logged exactly once: WARN with the measured silence,
    INFO with the outage duration;
  * the report to the operator is a diagnostic_msgs/DiagnosticArray on
    /diagnostics, one DiagnosticStatus per sensor.

Why /diagnostics: it is the standard ROS channel for health reports. The
operator can watch it with existing tools (rqt_robot_monitor,
`ros2 topic echo /diagnostics`), and diagnostic_aggregator / logging tools
already understand it, so no custom message or GUI is needed.

Subscriptions use the sensor-data QoS profile (best effort): it is compatible
with the reliable publishers of the stand and never slows them down.

Subscribes:  the topics in config/sensor_monitor.yaml (/odom, /imu, /scan)
Publishes:   /diagnostics (diagnostic_msgs/DiagnosticArray)
"""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

from gnssd_tools.node_runner import run_node
from gnssd_tools.sensor_health import LOST, OK, SensorWatch, WAITING

MESSAGE_TYPES = {'odometry': Odometry, 'imu': Imu, 'scan': LaserScan}

# Diagnostic level for each state of a sensor.
LEVELS = {WAITING: DiagnosticStatus.STALE, OK: DiagnosticStatus.OK, LOST: DiagnosticStatus.ERROR}


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
                                   f'expected {expected_rate} Hz')

        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0 / check_rate, self.check_all)
        self.create_timer(1.0 / report_rate, self.publish_diagnostics)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def make_callback(self, name):
        """Return a subscription callback that registers a message of sensor `name`."""
        def callback(_msg):
            now = self.now_seconds()
            if now > 0.0:   # see check_all()
                self.report(self.watches[name].on_message(now))
        return callback

    def check_all(self):
        now = self.now_seconds()
        if now == 0.0:
            # With use_sim_time the clock reads 0 until the first /clock message.
            # Messages arriving before that would get time 0 and then look
            # "1 s old" as soon as the clock starts - so nothing is evaluated yet.
            return
        for watch in self.watches.values():
            self.report(watch.check(now))

    def report(self, transition):
        """Log a state change once; publish the diagnostics immediately."""
        if transition is None:
            return
        name, t = transition.sensor, transition.time
        if transition.new_state == LOST:
            self.get_logger().warning(
                f'Sensor {name} LOST at t = {t:.2f} s: no message for '
                f'{transition.duration:.2f} s (timeout {self.watches[name].timeout} s)')
        elif transition.new_state == OK and transition.old_state == LOST:
            self.get_logger().info(
                f'Sensor {name} OK again at t = {t:.2f} s after an outage of '
                f'{transition.duration:.2f} s')
        elif transition.new_state == OK:
            self.get_logger().info(f'Sensor {name} OK (first message at t = {t:.2f} s)')
        else:
            self.get_logger().info(f'Sensor {name}: clock jumped back at t = {t:.2f} s, waiting')
        self.publish_diagnostics()

    def publish_diagnostics(self):
        now = self.now_seconds()
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
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
            ]
            array.status.append(status)
        self.diagnostics_publisher.publish(array)


def main():
    run_node(SensorMonitorNode)


if __name__ == '__main__':
    main()
