"""
Scripted operator: performs the recovery an operator would do after an invalid correction (STAND).

Used only for the Step 6 recovery experiment. It waits until sensor_monitor
reports the corrected estimate invalid (/correction_valid false), waits
`reaction_delay` seconds (a human needs some time) and then asks
map_manager_node to switch to localization on the last checkpoint, with the
initial pose taken from the base layer (use_base_pose). Everything it does is
something an operator could do by hand with `ros2 service call`.

Subscribes: /correction_valid (std_msgs/Bool, transient local)
Calls:      /map_manager_node/set_mode (gnssd_interfaces/SetMode)
"""

from gnssd_interfaces.srv import SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from gnssd_tools.node_runner import run_node


class ScriptedOperatorNode(Node):
    """Switches to localization on the last checkpoint after an INVALID alarm."""

    def __init__(self):
        super().__init__('scripted_operator_node')
        self.reaction_delay = self.declare_parameter('reaction_delay', 3.0).value
        self.map_name = self.declare_parameter('map_name', 'last_checkpoint').value
        self.client = self.create_client(SetMode, '/map_manager_node/set_mode')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/correction_valid', self.on_valid, latched)
        self.timer = None
        self.done = False

    def on_valid(self, msg):
        if msg.data or self.timer is not None or self.done:
            return
        self.get_logger().info(f'Correction INVALID: switching to localization in '
                               f'{self.reaction_delay} s')
        self.timer = self.create_timer(self.reaction_delay, self.act)

    def act(self):
        self.timer.cancel()
        self.done = True
        request = SetMode.Request(mode='localization', map_name=self.map_name,
                                  use_base_pose=True)
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'set_mode answered: {f.result().success}, '
                                             f'"{f.result().message}"'))


def main():
    run_node(ScriptedOperatorNode)


if __name__ == '__main__':
    main()
