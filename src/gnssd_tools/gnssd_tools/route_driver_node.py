"""
Route driver: drives the robot along the same closed route in every run.

Experiments are compared by repeating the same run with different noise seeds.
For that comparison to be fair, the robot must physically drive the same
path every time. A human with a keyboard cannot do that; this node can.

The node is part of the simulation stand, NOT of the navigation system.

It steers using /ground_truth (the exact pose from Gazebo) on purpose: the
estimators under test (odometry, EKF, SLAM) are allowed to be wrong, but the
route itself must not depend on their errors. If the driver steered on the
estimate, a worse estimator would also drive a different route, and the
runs would no longer be comparable.

Controller: pure pursuit. The route is a polyline. On every control step the
node finds where the robot is along the polyline, picks a target point
`lookahead_distance` further along it and turns towards that point along a
circular arc:

    curvature = 2 * sin(alpha) / L
    w = v * curvature

where alpha is the angle to the target point in the robot frame and L the
distance to it.

When the last lap is finished the node publishes a zero velocity, logs the
driven path length and the duration, and exits. run.launch.py uses this exit
to stop the whole experiment.

Subscribes:  /ground_truth (nav_msgs/Odometry)
Publishes:   /cmd_vel      (geometry_msgs/Twist)
Parameters:  gnssd_bringup/config/route.yaml
"""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from gnssd_tools.geometry import yaw_from_quaternion


class Route:
    """A polyline that can be queried by distance travelled along it (arc length)."""

    def __init__(self, points):
        self.points = points
        # cumulative[i] = distance along the route from points[0] to points[i]
        self.cumulative = [0.0]
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            self.cumulative.append(self.cumulative[-1] + math.hypot(x1 - x0, y1 - y0))
        self.length = self.cumulative[-1]

    @classmethod
    def closed_loop(cls, waypoints, laps):
        """Repeat a closed loop of waypoints `laps` times and end at the first point."""
        points = []
        for _ in range(laps):
            points.extend(waypoints)
        points.append(waypoints[0])
        return cls(points)

    def point_at(self, s):
        """Return the (x, y) point that lies `s` metres along the route."""
        s = min(max(s, 0.0), self.length)
        for i in range(len(self.points) - 1):
            if s <= self.cumulative[i + 1]:
                segment_length = self.cumulative[i + 1] - self.cumulative[i]
                t = 0.0 if segment_length == 0.0 else (s - self.cumulative[i]) / segment_length
                (x0, y0), (x1, y1) = self.points[i], self.points[i + 1]
                return x0 + t * (x1 - x0), y0 + t * (y1 - y0)
        return self.points[-1]

    def project(self, x, y, first_segment, progress, search_distance):
        """
        Find the point of the route closest to (x, y).

        Only the part of the route from the current segment up to
        `search_distance` metres beyond the current `progress` is searched.
        The robot only moves forward along the route, and the loop passes the
        same places on every lap, so a global search could jump to the wrong lap.

        Returns (segment index, arc length s of the closest point, distance to it).
        """
        best = (first_segment, progress, math.inf)
        limit = progress + search_distance
        for i in range(first_segment, len(self.points) - 1):
            if self.cumulative[i] > limit:
                break
            (x0, y0), (x1, y1) = self.points[i], self.points[i + 1]
            dx, dy = x1 - x0, y1 - y0
            segment_length_sq = dx * dx + dy * dy
            if segment_length_sq == 0.0:
                continue
            # Position of the foot of the perpendicular, as a fraction of the segment.
            t = ((x - x0) * dx + (y - y0) * dy) / segment_length_sq
            t = min(max(t, 0.0), 1.0)
            distance = math.hypot(x - (x0 + t * dx), y - (y0 + t * dy))
            if distance < best[2]:
                s = self.cumulative[i] + t * math.sqrt(segment_length_sq)
                best = (i, s, distance)
        return best


class RouteDriverNode(Node):
    """Follows the configured route with pure pursuit and exits when done."""

    def __init__(self):
        super().__init__('route_driver_node')

        flat_waypoints = self.declare_parameter('waypoints', [0.0, 0.0]).value
        laps = self.declare_parameter('laps', 2).value
        self.linear_speed = self.declare_parameter('linear_speed', 0.2).value
        self.max_angular_speed = self.declare_parameter('max_angular_speed', 0.6).value
        self.lookahead = self.declare_parameter('lookahead_distance', 0.5).value
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.05).value
        control_rate = self.declare_parameter('control_rate', 20.0).value
        self.start_delay = self.declare_parameter('start_delay', 3.0).value

        if len(flat_waypoints) % 2 != 0 or len(flat_waypoints) < 6:
            raise ValueError('waypoints must be a flat list x0, y0, x1, y1, ... '
                             'with at least 3 points')
        waypoints = list(zip(flat_waypoints[0::2], flat_waypoints[1::2]))
        self.route = Route.closed_loop(waypoints, laps)

        # Latest ground truth pose; None until the first message arrives.
        self.pose = None
        self.first_pose_time = None

        # Progress along the route.
        self.segment = 0
        self.progress = 0.0

        # Statistics for the final log message.
        self.start_time = None
        self.driven_length = 0.0
        self.last_position = None
        self.max_cross_track_error = 0.0

        self.done = False

        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'ground_truth', self.on_ground_truth, 10)
        self.timer = self.create_timer(1.0 / control_rate, self.control_step)

        self.get_logger().info(
            f'Route: {len(waypoints)} waypoints, {laps} laps, '
            f'{self.route.length:.2f} m planned, speed {self.linear_speed} m/s')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_ground_truth(self, msg):
        position = msg.pose.pose.position
        self.pose = (position.x, position.y, yaw_from_quaternion(msg.pose.pose.orientation))
        if self.first_pose_time is None:
            self.first_pose_time = self.now_seconds()

        # Path length actually driven, measured on the ground truth.
        if self.start_time is not None and not self.done:
            if self.last_position is not None:
                self.driven_length += math.hypot(position.x - self.last_position[0],
                                                 position.y - self.last_position[1])
            self.last_position = (position.x, position.y)

    def control_step(self):
        if self.done or self.pose is None:
            return

        if self.start_time is None:
            if self.now_seconds() - self.first_pose_time < self.start_delay:
                return
            self.start_time = self.now_seconds()
            self.get_logger().info('Starting the route')

        x, y, yaw = self.pose

        # 1. Where along the route are we?
        self.segment, self.progress, cross_track = self.route.project(
            x, y, self.segment, self.progress, search_distance=2.0 * self.lookahead + 1.0)
        self.max_cross_track_error = max(self.max_cross_track_error, cross_track)

        goal_x, goal_y = self.route.points[-1]
        distance_to_goal = math.hypot(goal_x - x, goal_y - y)
        on_last_stretch = self.progress > self.route.length - self.lookahead
        if on_last_stretch and (distance_to_goal < self.goal_tolerance or
                                self.progress >= self.route.length - 1e-6):
            self.finish()
            return

        # 2. Target point a look-ahead distance further along the route.
        target_x, target_y = self.route.point_at(self.progress + self.lookahead)

        # 3. Angle to the target as seen from the robot, and the arc to reach it.
        dx, dy = target_x - x, target_y - y
        alpha = math.atan2(dy, dx) - yaw
        distance = max(math.hypot(dx, dy), 1e-3)
        curvature = 2.0 * math.sin(alpha) / distance

        # Slow down on the last few decimetres so the robot stops on the goal
        # instead of overshooting it.
        speed = self.linear_speed
        if on_last_stretch:
            speed = max(0.05, min(speed, distance_to_goal))

        angular = max(-self.max_angular_speed,
                      min(self.max_angular_speed, speed * curvature))
        self.publish_velocity(speed, angular)

    def finish(self):
        self.publish_velocity(0.0, 0.0)
        self.done = True
        self.timer.cancel()
        duration = self.now_seconds() - self.start_time
        x, y, _ = self.pose
        self.get_logger().info(
            f'Route finished: driven {self.driven_length:.2f} m '
            f'(planned {self.route.length:.2f} m) in {duration:.1f} s, '
            f'final position ({x:.3f}, {y:.3f}), '
            f'max distance from the route {self.max_cross_track_error:.3f} m')

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_publisher.publish(msg)


def main():
    rclpy.init()
    node = RouteDriverNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Give the last zero-velocity message a moment to leave the process.
        time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
