#!/usr/bin/env python3
# tmux_ros2_manager.py
import os
import signal
import subprocess
import sys
import time
from functools import partial


class VIOProcessManager:
    """
    should_start and should_stop are public attributes that control
    the behavior of the process manager.  They can be set from outside
    this class to start or stop the OV_MSCKF tmux window.
    
    The public attributes `should_start` and `should_stop` are updated
    on every cycle so external code can query them.
    """

    # ---- configuration ------------------------------------------------- #
    CHECK_PERIOD   = 2       # seconds between polls
    START_DELAY    = 3       # let nodes come up
    STOP_DELAY     = 2       # give tmux a moment

    def __init__(self,
                 session: str = "ros2stack",
                 window: str  = "OV_MSCKF") -> None:

        self.TMUX_SESSION = session
        self.TMUX_WINDOW  = window

        # --------- public state you can access from outside -------------- #
        self.should_start: bool = False
        self.should_stop:  bool = False

        # --------- install signal handlers --------------------------------
        signal.signal(signal.SIGINT,  partial(self._sig_handler, "SIGINT"))
        signal.signal(signal.SIGTERM, partial(self._sig_handler, "SIGTERM"))


    def _is_running(self) -> bool:
        """Return True iff the tmux window for OV_MSCKF is alive."""
        proc = subprocess.run(["tmux", "list-windows", "-t", self.TMUX_SESSION],
                              capture_output=True, text=True)
        return self.TMUX_WINDOW in proc.stdout

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
        print(f"[+] tmux window '{self.TMUX_WINDOW}' created in "
              f"session '{self.TMUX_SESSION}'")

    def _stop_window(self) -> None:
        """Kill the tmux window that is running OV_MSCKF."""
        print("[-] Stopping ROS2 tmux window…")
        subprocess.run(f"tmux kill-window -t {self.TMUX_SESSION}:{self.TMUX_WINDOW}",
                       shell=True, check=False)
        print("[-] ROS2 tmux window terminated")

    # =====================================================================#
    # Main loop                                                            #
    # =====================================================================#
    def run(self) -> None:
        print("[i] Process manager started.  Press Ctrl+C to exit.")
        try:
            while True:
                # update public state
                running          = self._is_running()

                if self.should_start and not running:
                    self._start_window()
                    time.sleep(self.START_DELAY)
                    print(self.should_start, running)

                elif self.should_stop and running:
                    self._stop_window()
                    time.sleep(self.STOP_DELAY)

                time.sleep(self.CHECK_PERIOD)

        except KeyboardInterrupt:
            self._graceful_shutdown()

    # =====================================================================#
    # Signal handling & cleanup                                            #
    # =====================================================================#
    def _sig_handler(self, sig_name, *_):
        print(f"\n[!] Caught {sig_name}.  Shutting down gracefully…")
        self._graceful_shutdown()

    def _graceful_shutdown(self):
        if self._is_running():
            self._stop_window()
        print("Yönetim döngüsü sonlandı.")
        sys.exit(0)


# # ----------------------------------------------------------------------- #
# # Stand‑alone usage                                                       #
# # ----------------------------------------------------------------------- #
# if __name__ == "__main__":
#     TmuxRos2Manager().run()
