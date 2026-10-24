"""
Odometry noise node: makes perfect simulator odometry look like real wheel odometry.

The DiffDrive plugin in Gazebo reports the robot's motion without any error
(/odom_raw). Real wheel odometry is never that good:

* systematic error: the real wheel radius and track width differ slightly
  from the nominal values, so every measured velocity is off by a constant
  factor;
* random error: encoder quantisation, wheel slip, vibrations.

This node adds both kinds of error to the VELOCITIES and then integrates the
corrupted velocities into a pose, the same way the robot's own dead
reckoning would. The pose in /odom therefore drifts away from the truth more
and more over time. That drift is what the EKF and SLAM layers must deal with.

The node belongs to the simulation stand, not to the navigation system: on
the real robot it is not started, and /odom comes straight from the encoders.

Subscribes:  /odom_raw  (nav_msgs/Odometry)
Publishes:   /odom      (nav_msgs/Odometry, frame odom, child base_link)
TF:          none (odom -> base_link belongs to the EKF)
"""

import math
import random

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from gnssd_tools.geometry import normalize_angle, yaw_from_quaternion

# Variance written for the axes a differential-drive robot cannot move along
# (sideways, vertical, roll, pitch). Tiny = "we are sure this is zero".
UNUSED_AXIS_VARIANCE = 1e-9


def diagonal_covariance(variances):
    """Build a row-major 6x6 covariance matrix with the given diagonal."""
    matrix = [0.0] * 36
    for i, variance in enumerate(variances):
        matrix[i * 6 + i] = variance
    return matrix


class OdomNoiseNode(Node):
    """Corrupts /odom_raw velocities and integrates them into /odom."""

    def __init__(self):
        super().__init__('odom_noise_node')

        self.enabled = self.declare_parameter('enabled', True).value
        linear_scale_error = self.declare_parameter('linear_scale_error', 0.02).value
        angular_scale_error = self.declare_parameter('angular_scale_error', 0.03).value
        random_scale = self.declare_parameter('random_scale', False).value
        self.linear_noise_stddev = self.declare_parameter('linear_noise_stddev', 0.02).value
        self.angular_noise_stddev = self.declare_parameter('angular_noise_stddev', 0.02).value
        self.standstill_linear_speed = self.declare_parameter(
            'standstill_linear_speed', 0.001).value
        self.standstill_angular_speed = self.declare_parameter(
            'standstill_angular_speed', 0.001).value
        seed = self.declare_parameter('seed', 42).value

        # A private generator with a fixed seed makes every run repeatable.
        self.rng = random.Random(seed)

        if random_scale:
            # The parameter values are the limits; the actual errors are drawn once.
            linear_scale_error = self.rng.uniform(-linear_scale_error, linear_scale_error)
            angular_scale_error = self.rng.uniform(-angular_scale_error, angular_scale_error)
        self.linear_scale_error = linear_scale_error
        self.angular_scale_error = angular_scale_error

        # The values actually used are exposed as read-only parameters, so
        # that run_info_writer_node can store them next to the recorded bag.
        # With the noise disabled nothing is applied, so they are reported as 0.
        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter('applied_linear_scale_error',
                               linear_scale_error if self.enabled else 0.0, read_only)
        self.declare_parameter('applied_angular_scale_error',
                               angular_scale_error if self.enabled else 0.0, read_only)

        # Dead-reckoning state. It is initialised from the first /odom_raw
        # message so that /odom and /odom_raw start at the same pose.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_stamp = None

        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Odometry, 'odom_raw', self.on_raw_odometry, 10)

        if self.enabled:
            self.get_logger().info(
                f'Odometry noise ON: linear scale error {self.linear_scale_error:+.4f}, '
                f'angular scale error {self.angular_scale_error:+.4f}, '
                f'noise stddev {self.linear_noise_stddev} m/s, '
                f'{self.angular_noise_stddev} rad/s, seed {seed}')
        else:
            self.get_logger().info('Odometry noise OFF: velocities pass through unchanged')

    def on_raw_odometry(self, raw):
        """Handle one /odom_raw message: corrupt, integrate, publish."""
        stamp = Time.from_msg(raw.header.stamp)

        if self.last_stamp is None or stamp < self.last_stamp:
            # First message, or simulation time jumped back (world reset):
            # start dead reckoning again from the simulator's pose.
            self.reset_pose(raw)
            self.last_stamp = stamp
            return

        dt = (stamp - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = stamp

        true_linear = raw.twist.twist.linear.x
        true_angular = raw.twist.twist.angular.z
        linear, angular = self.measured_velocities(true_linear, true_angular)

        # Unicycle model, simple Euler step.
        self.x += linear * math.cos(self.yaw) * dt
        self.y += linear * math.sin(self.yaw) * dt
        self.yaw = normalize_angle(self.yaw + angular * dt)

        self.publish(raw.header.stamp, linear, angular)

    def reset_pose(self, raw):
        """Copy the pose from a raw odometry message into the dead-reckoning state."""
        self.x = raw.pose.pose.position.x
        self.y = raw.pose.pose.position.y
        self.yaw = yaw_from_quaternion(raw.pose.pose.orientation)

    def measured_velocities(self, true_linear, true_angular):
        """Return (v, w) as a real wheel odometry would measure them."""
        if not self.enabled:
            return true_linear, true_angular

        linear = true_linear * (1.0 + self.linear_scale_error)
        angular = true_angular * (1.0 + self.angular_scale_error)

        robot_is_moving = (abs(true_linear) > self.standstill_linear_speed or
                           abs(true_angular) > self.standstill_angular_speed)
        if robot_is_moving:
            linear += self.rng.gauss(0.0, self.linear_noise_stddev)
            angular += self.rng.gauss(0.0, self.angular_noise_stddev)

        return linear, angular

    def publish(self, stamp, linear, angular):
        """Publish the current dead-reckoning pose and the measured velocities."""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular

        # Diagonal covariances taken directly from the noise parameters.
        # The EKF will fuse only the twist part of /odom; the pose covariance
        # is filled the same way just so the message is complete.
        linear_variance = max(self.linear_noise_stddev ** 2, UNUSED_AXIS_VARIANCE)
        angular_variance = max(self.angular_noise_stddev ** 2, UNUSED_AXIS_VARIANCE)
        small = UNUSED_AXIS_VARIANCE
        msg.twist.covariance = diagonal_covariance(
            [linear_variance, small, small, small, small, angular_variance])
        msg.pose.covariance = diagonal_covariance(
            [linear_variance, linear_variance, small, small, small, angular_variance])

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = OdomNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
