"""
One way to start and stop every project node cleanly.

The problem it solves. With rclpy's own SIGINT handler the ROS context is
shut down immediately - possibly while a subscription callback is running -
and the node prints "RuntimeError: Unable to convert call argument to Python
object". Switching that handler off and relying on Python's default handler
is not enough either: KeyboardInterrupt can then be raised in the middle of a
call into rclpy's C++ code, which pybind11 turns into the same RuntimeError.

The solution here:
  * rclpy's handler is off (SignalHandlerOptions.NO);
  * our own SIGINT/SIGTERM handler only sets a "stop" flag - it never raises;
  * the node spins with spin_once(timeout) and checks the flag between
    callbacks, so it stops within SPIN_TIMEOUT even if no message arrives
    (a node without timers or /clock would otherwise wait forever).
"""

import signal
import threading

import rclpy
from rclpy.executors import (ExternalShutdownException, MultiThreadedExecutor,
                             SingleThreadedExecutor)
from rclpy.signals import SignalHandlerOptions

SPIN_TIMEOUT = 0.1  # s, longest time between two looks at the stop flag


def init_with_stop_flag():
    """Initialise rclpy without its signal handler; return the event set by SIGINT/SIGTERM."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()

    def request_stop(_signal_number, _frame):
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    return stop


def run_node(node_class, on_shutdown=None, threads=1):
    """
    Create `node_class()`, spin it until SIGINT/SIGTERM, then shut down cleanly.

    `on_shutdown(node)` is called before the node is destroyed (e.g. to write
    a final file); it runs even when the node is stopped by a signal.
    `threads` > 1 uses a multi-threaded executor, needed by nodes whose
    service callbacks wait for other services (e.g. map_manager_node).
    """
    stop = init_with_stop_flag()
    node = node_class()
    executor = (MultiThreadedExecutor(num_threads=threads) if threads > 1
                else SingleThreadedExecutor())
    executor.add_node(node)
    try:
        while not stop.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=SPIN_TIMEOUT)
    except ExternalShutdownException:
        pass
    finally:
        if on_shutdown is not None:
            on_shutdown(node)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()
