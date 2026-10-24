"""
Loop closure monitor: counts the loop closures of slam_toolbox during a run.

slam_toolbox does not publish or log automatic loop closures. But its graph
optimisation (Karto `CorrectPoses`) runs only when a loop is closed, and an
optimisation moves the poses of OLD graph vertices. Adding a new scan never
moves old vertices.

So this node watches the pose graph that slam_toolbox publishes for RViz
(/slam_toolbox/graph_visualization, one sphere marker per vertex, id =
vertex id) and counts an "optimisation event" whenever a vertex that was
already known has moved by more than `move_threshold`. One event is counted
per graph message, so a closure is detected with the time resolution of the
graph publication (it is published together with the map, every
map_update_interval = 5 s of sim time).

At every event, and on shutdown, it writes <output_file> (YAML):
    loop_closures: <count>
    times: [<sim time of each event, s>, ...]
    max_vertex_shift: [<largest vertex displacement of each event, m>, ...]

Started by record.launch.py; part of the evaluation tooling, not of the system.
"""

import math

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml

VERTEX_NAMESPACE = 'slam_toolbox'   # edges use 'slam_toolbox_edges'


class LoopClosureMonitorNode(Node):
    """Detects graph optimisations (= loop closures) from moved graph vertices."""

    def __init__(self):
        super().__init__('loop_closure_monitor_node')
        self.output_file = self.declare_parameter('output_file', '').value
        self.move_threshold = self.declare_parameter('move_threshold', 0.005).value

        self.vertices = {}       # vertex id -> (x, y) at the last graph message
        self.event_times = []
        self.event_shifts = []

        self.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization',
                                 self.on_graph, 10)
        self.get_logger().info(f'Watching the slam_toolbox graph, writing {self.output_file}')

    def on_graph(self, msg):
        current = {}
        stamp = None
        for marker in msg.markers:
            if marker.ns == VERTEX_NAMESPACE and marker.action == Marker.ADD:
                current[marker.id] = (marker.pose.position.x, marker.pose.position.y)
                stamp = marker.header.stamp

        # Largest displacement among the vertices that existed before.
        largest_shift = 0.0
        for vertex_id, (x, y) in current.items():
            if vertex_id in self.vertices:
                old_x, old_y = self.vertices[vertex_id]
                largest_shift = max(largest_shift, math.hypot(x - old_x, y - old_y))

        if largest_shift > self.move_threshold and stamp is not None:
            t = stamp.sec + stamp.nanosec * 1e-9
            self.event_times.append(round(t, 3))
            self.event_shifts.append(round(largest_shift, 4))
            self.get_logger().info(
                f'Loop closure #{len(self.event_times)} at t = {t:.1f} s '
                f'(old vertices moved up to {largest_shift:.3f} m)')
            self.write()

        if current:
            self.vertices = current

    def write(self):
        if not self.output_file:
            return
        with open(self.output_file, 'w') as f:
            yaml.safe_dump({'loop_closures': len(self.event_times),
                            'times': self.event_times,
                            'max_vertex_shift': self.event_shifts}, f, sort_keys=False)


def main():
    rclpy.init()
    node = LoopClosureMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write()   # also records "0 loop closures" if none happened
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
