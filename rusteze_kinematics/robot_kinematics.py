"""
robot_kinematics.py

Object-oriented kinematics library for Task 11.3 (Multi-Module Vehicle
Convoy & Kinematics).

Shared interface (agreed contract - do not change signatures):
    Kinematics(L, W, R)
        L : track width / radius
        W : wheelbase length
        R : wheel radius

    .inverse(vx, vy, wz) -> list[float]   # chassis vel -> wheel speeds (rad/s)
    .forward(w: list[float]) -> (vx, vy, wz)  # wheel speeds -> chassis vel

Owners:
    Kinematics (base) + DiffDriveKinematics ......... Member 2
    MecanumKinematics, ThreeWheelOmniKinematics,
    FourWheelOmniKinematics .......................... Member 3
"""

import numpy as np


class Kinematics:
    """Base class holding shared chassis geometry and the matrix contract."""

    def __init__(self, L: float, W: float, R: float):
        self.L = L  # track width / radius
        self.W = W  # wheelbase length
        self.R = R  # wheel radius

        # Each subclass must populate these in its own __init__.
        self.M_forward = None  # wheel speeds -> chassis velocity
        self.M_inverse = None  # chassis velocity -> wheel speeds

    def inverse(self, vx: float, vy: float, wz: float):
        """Convert chassis velocities into target wheel angular speeds.

        Returns a 4-element array/list of wheel speeds (rad/s).
        """
        raise NotImplementedError("Subclasses must implement inverse()")

    def forward(self, w):
        """Convert measured wheel rotational speeds into chassis velocities.

        Args:
            w: list/array of measured wheel speeds (rad/s)
        Returns:
            (vx, vy, wz)
        """
        raise NotImplementedError("Subclasses must implement forward()")


# ---------------------------------------------------------------------------
# Member 2
# ---------------------------------------------------------------------------
class DiffDriveKinematics(Kinematics):
    """2-wheel differential drive (report as a 4-element array: e.g.
    [left, right, left, right] or [left, right, 0, 0] - confirm the
    convention with Member 4/5 and document it here once decided)."""

    def __init__(self, L: float, W: float, R: float):
        super().__init__(L, W, R)
        # TODO(Member 2): define self.M_inverse / self.M_forward, e.g.
        # M_inverse maps [vx, wz] -> [w_left, w_right] using L and R.
        self.M_inverse = None
        self.M_forward = None

    def inverse(self, vx: float, vy: float, wz: float):
        # TODO(Member 2): implement using self.M_inverse
        raise NotImplementedError("Member 2: implement DiffDriveKinematics.inverse")

    def forward(self, w):
        # TODO(Member 2): implement using self.M_forward
        raise NotImplementedError("Member 2: implement DiffDriveKinematics.forward")


# ---------------------------------------------------------------------------
# Member 3
# ---------------------------------------------------------------------------
class MecanumKinematics(Kinematics):
    """4-wheel Mecanum drive - supports vx, vy (strafe), and wz."""

    def __init__(self, L: float, W: float, R: float):
        super().__init__(L, W, R)
        # TODO(Member 3): define self.M_inverse / self.M_forward
        self.M_inverse = None
        self.M_forward = None

    def inverse(self, vx: float, vy: float, wz: float):
        raise NotImplementedError("Member 3: implement MecanumKinematics.inverse")

    def forward(self, w):
        raise NotImplementedError("Member 3: implement MecanumKinematics.forward")


class ThreeWheelOmniKinematics(Kinematics):
    """3-wheel omni drive, wheels typically at 120 degrees apart."""

    def __init__(self, L: float, W: float, R: float):
        super().__init__(L, W, R)
        # TODO(Member 3): define self.M_inverse / self.M_forward
        self.M_inverse = None
        self.M_forward = None

    def inverse(self, vx: float, vy: float, wz: float):
        raise NotImplementedError("Member 3: implement ThreeWheelOmniKinematics.inverse")

    def forward(self, w):
        raise NotImplementedError("Member 3: implement ThreeWheelOmniKinematics.forward")


class FourWheelOmniKinematics(Kinematics):
    """4-wheel omni drive, wheels typically at 45 degree rollers."""

    def __init__(self, L: float, W: float, R: float):
        super().__init__(L, W, R)
        # TODO(Member 3): define self.M_inverse / self.M_forward
        self.M_inverse = None
        self.M_forward = None

    def inverse(self, vx: float, vy: float, wz: float):
        raise NotImplementedError("Member 3: implement FourWheelOmniKinematics.inverse")

    def forward(self, w):
        raise NotImplementedError("Member 3: implement FourWheelOmniKinematics.forward")


DRIVE_TYPE_MAP = {
    "diff": DiffDriveKinematics,
    "mecanum": MecanumKinematics,
    "omni3": ThreeWheelOmniKinematics,
    "omni4": FourWheelOmniKinematics,
}


def build_kinematics(drive_type: str, L: float, W: float, R: float) -> Kinematics:
    """Factory used by both ROS2 nodes so drive-type selection lives in one place."""
    cls = DRIVE_TYPE_MAP.get(drive_type)
    if cls is None:
        raise ValueError(
            f"Unknown drive_type '{drive_type}'. Valid options: {list(DRIVE_TYPE_MAP)}"
        )
    return cls(L, W, R)
