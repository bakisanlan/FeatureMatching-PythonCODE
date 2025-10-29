#!/usr/bin/env python3
# vio_process_manager_node.py
import os
import signal
import subprocess
import sys
import time
from functools import partial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool                      # see note ①

class VIOProcessManager(Node):
    """
    A ROS 2 node that starts / stops the OV_MSCKF tmux window.

    External control
    ────────────────
    • 🟢  Publish `True` on  «vio_manager/start»  to ask the node to launch the
      tmux window.
    • 🔴  Publish `True` on  «vio_manager/stop»   to ask the node to kill it.

    Instead of topics you can, of course, flip `self.should_start / should_stop`
    from another thread as you did before; the attributes remain public.
    """

    # ---- configuration ------------------------------------------------- #
    CHECK_PERIOD = 2.0   # seconds between polls
    START_DELAY  = 3.0   # let nodes come up
    STOP_DELAY   = 2.0   # give tmux a moment

    def __init__(self,
                 session: str = "ros2stack",
                 window:  str = "OV_MSCKF") -> None:
        super().__init__("vio_process_manager")

        # ---- configuration ------------------------------------------------
        self.TMUX_SESSION = session
        self.TMUX_WINDOW  = window

        # --------- public state you can access from outside --------------- #
        self.should_start: bool = False
        self.should_stop:  bool = False
        self.running:      bool = False

        # --------- ROS interfaces ----------------------------------------- #
        self.create_subscription(Bool, "vio_manager/start",
                                 self._start_msg_cb,   1)
        self.create_subscription(Bool, "vio_manager/stop",
                                 self._stop_msg_cb,    1)

        # periodic polling timer
        self.create_timer(self.CHECK_PERIOD, self._main_loop)

        # --------- handle SIGINT / SIGTERM so Ctrl‑C works ----------------- #
        signal.signal(signal.SIGINT,  partial(self._sig_handler, "SIGINT"))
        signal.signal(signal.SIGTERM, partial(self._sig_handler, "SIGTERM"))

        self.get_logger().info("🛫  VIOProcessManager node started")

    # =======================================================================
    # ROS topic callbacks
    # =======================================================================
    def _start_msg_cb(self, msg: Bool):
        self.should_start = bool(msg.data)

    def _stop_msg_cb(self, msg: Bool):
        self.should_stop = bool(msg.data)

    # =======================================================================
    # Internal helpers  (identical to your original code)
    # =======================================================================
    def _is_running(self) -> bool:
        """Return True iff the tmux window for OV_MSCKF is alive."""
        result = subprocess.run(["tmux", "list-windows",
                                 "-t", self.TMUX_SESSION],
                                capture_output=True, text=True)
        return self.TMUX_WINDOW in result.stdout

    def _start_window(self) -> None:
        """Create the session if needed and spawn the ROS2 launch window."""
        subprocess.run(
            f"tmux has-session -t {self.TMUX_SESSION} "
            f"|| tmux new-session -d -s {self.TMUX_SESSION}",
            shell=True,
        )
        cmd = (f"tmux new-window -t {self.TMUX_SESSION} -n {self.TMUX_WINDOW} "
               f"'ros2 launch ov_msckf subscribe.launch.py "
               f"config:=my_config max_cameras:=1'")
        subprocess.run(cmd, shell=True)
        self.get_logger().info(
            f"[+] tmux window '{self.TMUX_WINDOW}' created in "
            f"session '{self.TMUX_SESSION}'")

    def _stop_window(self) -> None:
        """Kill the tmux window that is running OV_MSCKF."""
        self.get_logger().info("[-] Stopping ROS2 tmux window…")
        subprocess.run(f"tmux kill-window -t {self.TMUX_SESSION}:{self.TMUX_WINDOW}",
                       shell=True, check=False)
        self.get_logger().info("[-] ROS2 tmux window terminated")

    # =======================================================================
    # Main loop (called by timer)
    # =======================================================================
    def _main_loop(self):
        running = self._is_running()
        self.running = running

        if self.should_start and not running:
            self._start_window()
            self.should_stop = False
            time.sleep(self.START_DELAY)            # minimal blocking is fine

        elif self.should_stop and running:
            self._stop_window()
            self.should_start = False  # reset the flag
            time.sleep(self.STOP_DELAY)

        print(f"running={running}, should_start={self.should_start}, should_stop={self.should_stop}")

    # =======================================================================
    # Shutdown handling
    # =======================================================================
    def _sig_handler(self, sig_name, *_):
        self.get_logger().warn(f"Caught {sig_name}.  Shutting down gracefully…")
        self._graceful_shutdown()

    def _graceful_shutdown(self):
        if self._is_running():
            self._stop_window()
        rclpy.shutdown()
        sys.exit(0)


# # ─────────────────────────────────────────────────────────────────────────────
# # Stand‑alone demo
# # ─────────────────────────────────────────────────────────────────────────────
# if __name__ == "__main__":
#     rclpy.init()
#     node = VIOProcessManagerNode()          # defaults: ros2stack / OV_MSCKF
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)

#     # example: flip the flag from another thread if you wish, e.g.:
#     # node.should_start = True

#     try:
#         executor.spin()
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
