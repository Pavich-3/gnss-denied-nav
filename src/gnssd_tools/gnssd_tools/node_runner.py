"""
One way to start and stop every project node cleanly.

The problem it solves: by default rclpy installs its own SIGINT handler,
which shuts the ROS context down immediately - possibly while a
subscription callback is running. The callback then fails with
"RuntimeError: Unable to convert call argument to Python object" and the
node prints a Traceback on every Ctrl+C / launch shutdown.

Here rclpy's handler is switched off (SignalHandlerOptions.NO). Ctrl+C then
raises an ordinary KeyboardInterrupt in the main thread, between two
callbacks, and the node is destroyed and the context shut down in order.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions


def run_node(node_class, on_shutdown=None):
    """
    Create `node_class()`, spin it until Ctrl+C, then shut down cleanly.

    `on_shutdown(node)` is called before the node is destroyed (e.g. to write
    a final file); it runs even when the node is stopped by a signal.
    """
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = node_class()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if on_shutdown is not None:
            on_shutdown(node)
        node.destroy_node()
        rclpy.try_shutdown()
