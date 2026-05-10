ngcp2025@frs-pi:~/rf_pcb/integration_member/pi4_tracker_test $ cat ./servo_control.py
"""
servo_control.py
----------------
Python wrapper around the servo_control C binary.

Keeps the compiled C program running as a persistent subprocess and sends
commands to it over stdin, reading responses from stdout.

The C binary handles all calibration interpolation, PWM ramping, and
sysfs writes — this file just drives it.

SETUP (once per boot — same as running the C binary directly):
    sudo pinctrl set 18 a3
    echo 0 | sudo tee /sys/class/pwm/pwmchip0/export

BUILD THE C BINARY (once):
    gcc servo_control.c -o servo_control -lm
    sudo chown root:root servo_control
    sudo chmod u+s servo_control      # setuid so Python can run it without sudo

USAGE:
    from servo_control import ServoController

    servo = ServoController()
    servo.start()           # launches servo_control binary, moves to 0°

    servo.goto(45.0)        # absolute move to +45°  (uses C cal tables)
    servo.goto(-60.0)       # absolute move to -60°

    servo.stop()            # sends 'quit', terminates subprocess
"""

import subprocess
import threading
import logging
import time
import os

log = logging.getLogger("servo_control")

# Path to the compiled C binary — adjust if yours is elsewhere
SERVO_BINARY = os.path.join(os.path.dirname(__file__), "servo_control")

# How long to wait for the C binary to confirm a move is complete
MOVE_TIMEOUT_S = 5.0

# Seconds per degree — used to estimate move duration for goto()
# Tune this to match your gear ratio / servo speed
SEC_PER_DEG = 0.004     # e.g. 0.004 s/deg → 90° move takes ~0.36s


class ServoController:
    """
    Drives the servo_control C binary as a persistent subprocess.

    The C process stays running for the entire mission — one launch,
    zero restarts. Commands are sent as text lines to its stdin and
    responses are read from its stdout.

    Supported C commands used here:
        goto <deg> <sec>    — absolute move using calibration table
        stop                — cancel current motion immediately
        quit                — clean shutdown
    """

    def __init__(self,
                 binary:          str   = SERVO_BINARY,
                 sec_per_deg:     float = SEC_PER_DEG,
                 move_timeout_s:  float = MOVE_TIMEOUT_S,
                 settle_s:        float = 0.05):
        """
        Args:
            binary          : path to compiled servo_control binary
            sec_per_deg     : move speed estimate (seconds per degree)
            move_timeout_s  : max seconds to wait for a move to complete
            settle_s        : extra settle time after each goto()
        """
        self.binary         = binary
        self.sec_per_deg    = sec_per_deg
        self.move_timeout_s = move_timeout_s
        self.settle_s       = settle_s

        self._proc          = None
        self._lock          = threading.Lock()
        self._running       = False
        self._current_deg   = 0.0

    # ------------------------------------------------------------------ #
    #  Lifecycle
    # ------------------------------------------------------------------ #

    def start(self):
        """
        Launch the servo_control binary and wait for it to be ready.
        Moves servo to 0° on startup.
        """
        if self._running:
            return

        if not os.path.isfile(self.binary):
            raise FileNotFoundError(
                f"servo_control binary not found at '{self.binary}'\n"
                f"Build it with:  gcc servo_control.c -o servo_control -lm"
            )

        log.info(f"Launching servo_control binary: {self.binary}")

        self._proc = subprocess.Popen(
            [self.binary],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,          # line-buffered
        )

        # Wait for "Ready." prompt from C binary
        if not self._wait_for_output("Ready", timeout=3.0):
            raise RuntimeError(
                "servo_control binary did not print 'Ready' — "
                "check PWM setup:\n"
                "  sudo pinctrl set 18 a3\n"
                "  echo 0 | sudo tee /sys/class/pwm/pwmchip0/export"
            )

        self._running     = True
        self._current_deg = 0.0
        log.info("ServoController ready at 0°")

    def stop(self):
        """Send 'quit' to the C binary and terminate the subprocess."""
        if not self._running:
            return

        try:
            self._send("quit")
            self._proc.wait(timeout=2.0)
        except Exception:
            pass
        finally:
            try:
                self._proc.kill()
            except Exception:
                pass
            self._proc    = None
            self._running = False

        log.info("ServoController stopped")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()

    # ------------------------------------------------------------------ #
    #  Public movement commands
    # ------------------------------------------------------------------ #

    def goto(self, deg: float, duration_s: float = None):
        """
        Move servo to an absolute angle using the C binary's 'goto' command.

        The C binary uses its calibration tables (forward/reverse) to
        convert degrees → pulse width with nonlinear interpolation.

        Format sent to C binary:
            goto <deg> <sec>

        Args:
            deg        : target angle in degrees
            duration_s : move duration; auto-calculated from sec_per_deg if None
        """
        if not self._running:
            raise RuntimeError("ServoController not started — call start() first")

        if duration_s is None:
            travel    = abs(deg - self._current_deg)
            duration_s = max(0.1, travel * self.sec_per_deg)

        cmd = f"goto {deg:.3f} {duration_s:.3f}"

        with self._lock:
            log.debug(f"→ C: '{cmd}'")
            self._send(cmd)

            # Wait for the C binary to confirm the move finished.
            # The C binary prints one of:
            #   "Goto X.XX deg using ... -> target pulse ..."  (move started)
            #   "Stopped by sensor."  (sensor triggered)
            #   "Stopped."            (stop_requested)
            # We wait for any output line then settle.
            self._wait_for_output(None, timeout=duration_s + self.move_timeout_s)

            self._current_deg = deg

        if self.settle_s > 0:
            time.sleep(self.settle_s)

    def stop_motion(self):
        """
        Immediately cancel any in-progress movement.
        Sends the 'stop' command to the C binary.
        """
        if self._running:
            self._send("stop")
            log.info("Motion stopped")

    @property
    def position_deg(self) -> float:
        """Last commanded position in degrees."""
        return self._current_deg

    # ------------------------------------------------------------------ #
    #  Internal helpers
    # ------------------------------------------------------------------ #

    def _send(self, cmd: str):
        """Write one command line to the C binary's stdin."""
        try:
            self._proc.stdin.write(cmd + "\n")
            self._proc.stdin.flush()
        except OSError as e:
            log.error(f"Failed to send command '{cmd}': {e}")
            raise

    def _wait_for_output(self, keyword, timeout: float) -> bool:
        """
        Read lines from the C binary's stdout until a line containing
        keyword is found, or until timeout expires.

        If keyword is None, returns True as soon as any line arrives.
        Returns False on timeout.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            # Non-blocking readline with timeout
            self._proc.stdout
            try:
                import select
                ready, _, _ = select.select([self._proc.stdout], [], [], 0.05)
                if ready:
                    line = self._proc.stdout.readline().strip()
                    if line:
                        log.debug(f"← C: '{line}'")
                        if keyword is None or keyword in line:
                            return True
            except Exception:
                time.sleep(0.05)
        return False
