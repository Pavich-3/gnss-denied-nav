"""
IMU covariance node: adds the sensor's accuracy to the simulated IMU messages.

Gazebo publishes IMU messages with all covariances set to zero. For the EKF
a zero variance means "this sensor is perfect": robot_localization replaces
it with 1e-9, and the gyro would then completely override the wheel
odometry instead of being fused with it.

A real IMU driver fills the covariances from the datasheet. This node does
the same for the simulator, so it is part of the simulation stand (like
odom_noise_node), not of the navigation system:

* angular_velocity_covariance    diagonal = gyro sigma^2
* linear_acceleration_covariance diagonal = accelerometer sigma^2
* orientation_covariance[0] = -1 (REP 145: "orientation not provided";
  the simulated IMU has no magnetometer and its orientation is not fused)

Everything else in the message is passed through unchanged. The sigmas are
the white-noise values of the Gazebo IMU model; sim.launch.py reads them
from robot_params.xacro, so the numbers exist in one place only. The
per-run bias is not part of the covariance (it is a slowly varying offset,
not white noise).

Subscribes:  /imu_raw (sensor_msgs/Imu, straight from Gazebo)
Publishes:   /imu     (sensor_msgs/Imu, with covariances)
"""

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def diagonal_3x3(variance):
    """Row-major 3x3 matrix with `variance` on the diagonal."""
    return [variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance]


class ImuCovarianceNode(Node):
    """Republishes /imu_raw as /imu with covariances filled in."""

    def __init__(self):
        super().__init__('imu_covariance_node')

        gyro_stddev = self.required_parameter('gyro_noise_stddev')
        accel_stddev = self.required_parameter('accel_noise_stddev')

        self.angular_velocity_covariance = diagonal_3x3(gyro_stddev ** 2)
        self.linear_acceleration_covariance = diagonal_3x3(accel_stddev ** 2)

        self.publisher = self.create_publisher(Imu, 'imu', 10)
        self.create_subscription(Imu, 'imu_raw', self.on_imu, 10)

        self.get_logger().info(
            f'IMU covariance: gyro sigma {gyro_stddev} rad/s '
            f'(variance {gyro_stddev ** 2:.3e}), accel sigma {accel_stddev} m/s^2 '
            f'(variance {accel_stddev ** 2:.3e}), orientation not provided')

    def required_parameter(self, name):
        """
        Declare a float parameter that has no default value.

        There is deliberately no default: the sigmas must come from
        robot_params.xacro (passed in by sim.launch.py), never from a second
        copy of the numbers in this file.
        """
        value = self.declare_parameter(
            name, None, ParameterDescriptor(dynamic_typing=True)).value
        if value is None:
            raise ValueError(f'Parameter {name} is not set; sim.launch.py passes it '
                             'from robot_params.xacro')
        return float(value)

    def on_imu(self, msg):
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance = self.angular_velocity_covariance
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ImuCovarianceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
