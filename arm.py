"""
Chainable API for the 5-DOF robot arm.
Each function does one thing and returns self so you can chain: arm.move_base(90).wait(1).close_gripper()
"""
import time
from setup import ServoCtrl

# Servo indices for the 5-DOF arm
BASE = 0
SHOULDER = 1
ELBOW = 2
WRIST = 3
GRIPPER = 4

NUM_ARM_SERVOS = 5

# Default gripper angles (tune to your hardware)
GRIPPER_OPEN = 120
GRIPPER_CLOSED = 60


def _clamp(angle: float, lo: int = 0, hi: int = 180) -> int:
    """Clamp angle to valid servo range."""
    return max(lo, min(hi, int(round(angle))))


class Arm:
    """
    Fluent interface for the robot arm. All movement and wait methods return self for chaining.
    """

    def __init__(self):
        self._ctrl = ServoCtrl()
        self._ctrl.start()
        self._ctrl.moveInit()

    def _position(self) -> list:
        """Current angles of the 5 arm servos [base, shoulder, elbow, wrist, gripper]."""
        return list(self._ctrl.servoAngle()[:NUM_ARM_SERVOS])

    def _move_to(self, angles: list) -> "Arm":
        """Move all 5 arm servos to the given angles. Expects a list of 5 integers 0–180."""
        if len(angles) != NUM_ARM_SERVOS:
            raise ValueError(f"angles must have {NUM_ARM_SERVOS} elements, got {len(angles)}")
        goal = [_clamp(a) for a in angles]
        self._ctrl.moveToPos(NUM_ARM_SERVOS, goal)
        return self

    # --- Single-joint moves (each does one thing, returns self) ---

    def move_base(self, angle: float) -> "Arm":
        """Set base rotation (0–180). Other joints stay put."""
        pos = self._position()
        pos[BASE] = _clamp(angle)
        return self._move_to(pos)

    def move_shoulder(self, angle: float) -> "Arm":
        """Set shoulder angle (0–180). Other joints stay put."""
        pos = self._position()
        pos[SHOULDER] = _clamp(angle)
        return self._move_to(pos)

    def move_elbow(self, angle: float) -> "Arm":
        """Set elbow angle (0–180). Other joints stay put."""
        pos = self._position()
        pos[ELBOW] = _clamp(angle)
        return self._move_to(pos)

    def move_wrist(self, angle: float) -> "Arm":
        """Set wrist angle (0–180). Other joints stay put."""
        pos = self._position()
        pos[WRIST] = _clamp(angle)
        return self._move_to(pos)

    def move_gripper(self, angle: float) -> "Arm":
        """Set gripper angle (0–180). Other joints stay put."""
        pos = self._position()
        pos[GRIPPER] = _clamp(angle)
        return self._move_to(pos)

    # --- Multi-joint / convenience ---

    def move_to(self, base=None, shoulder=None, elbow=None, wrist=None, gripper=None) -> "Arm":
        """
        Move only the joints you specify; others keep current position.
        Example: arm.move_to(shoulder=45, gripper=80)
        """
        pos = self._position()
        if base is not None:
            pos[BASE] = _clamp(base)
        if shoulder is not None:
            pos[SHOULDER] = _clamp(shoulder)
        if elbow is not None:
            pos[ELBOW] = _clamp(elbow)
        if wrist is not None:
            pos[WRIST] = _clamp(wrist)
        if gripper is not None:
            pos[GRIPPER] = _clamp(gripper)
        return self._move_to(pos)

    def open_gripper(self, angle: float = GRIPPER_OPEN) -> "Arm":
        """Open gripper to given angle (default GRIPPER_OPEN)."""
        return self.move_gripper(angle)

    def close_gripper(self, angle: float = GRIPPER_CLOSED) -> "Arm":
        """Close gripper to given angle (default GRIPPER_CLOSED)."""
        return self.move_gripper(angle)

    def wait(self, seconds: float) -> "Arm":
        """Pause for the given time. Chainable."""
        time.sleep(seconds)
        return self

    def home(self) -> "Arm":
        """Move all 5 arm servos to 90° (neutral)."""
        return self._move_to([90] * NUM_ARM_SERVOS)

    def position(self) -> list:
        """Return current [base, shoulder, elbow, wrist, gripper] (read-only, not chainable)."""
        return self._position()

    def stop(self) -> None:
        """Stop any continuous motion (e.g. wiggle)."""
        self._ctrl.stopWiggle()
