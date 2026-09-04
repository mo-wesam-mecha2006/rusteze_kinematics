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



# Member 2
# ---------------------------------------------------------------------------
class DiffDriveKinematics(Kinematics):
    """
    Differential drive kinematics for a 4-wheel robot.
    
    Convention: wheel speed order is [front_left, front_right, rear_left, rear_right].
    Left wheels (FL, RL) share the same speed.
    Right wheels (FR, RR) share the same speed.
    """

    def __init__(self, L: float, W: float, R: float):
        super().__init__(L, W, R)


        # --- Inverse Kinematics Matrix ---
        # Maps [vx, wz] -> [w_left, w_right]
        # w_left  = (vx - (wz * L/2)) / R
        # w_right = (vx + (wz * L/2)) / R
        self.M_inverse = np.array([
            [1.0 / self.R, -self.L / (2.0 * self.R)],
            [1.0 / self.R,  self.L / (2.0 * self.R)]
        ])

        # --- Forward Kinematics Matrix ---
        # Maps [w_left, w_right] -> [vx, wz]
        # vx = (w_left + w_right) * R / 2
        # wz = (w_right - w_left) * R / L
        self.M_forward = np.array([
            [self.R / 2.0, self.R / 2.0],         # vx coefficients
            [-self.R / self.L, self.R / self.L]   # wz coefficients
        ])

    def inverse(self, vx: float, vy: float, wz: float):
        """
        Convert chassis velocity [vx, vy, wz] to wheel speeds.
        Note: vy is ignored for differential drive.
        Returns: [w_FL, w_FR, w_RL, w_RR] in rad/s.
        """
        # Compute left and right wheel speeds using the inverse matrix
        w_left, w_right = self.M_inverse.dot([vx, wz])

        # Return speeds for 4 wheels: [FL, FR, RL, RR]
        # Left wheels (FL, RL) get w_left, Right wheels (FR, RR) get w_right
        return [w_left, w_right, w_left, w_right]

    def forward(self, w):
        """
        Convert measured wheel speeds [w_FL, w_FR, w_RL, w_RR] to chassis velocity.
        Returns: (vx, vy, wz) where vy is always 0 for diff drive.
        """
        # Extract individual wheel speeds
        w_FL, w_FR, w_RL, w_RR = w

        # Average left and right wheel speeds for robustness
        w_left = (w_FL + w_RL) / 2.0
        w_right = (w_FR + w_RR) / 2.0

        # Compute chassis velocities using the forward matrix
        vx, wz = self.M_forward.dot([w_left, w_right])
        vy = 0.0  # No lateral velocity for diff drive

        return vx, vy, wz

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
        w1 = (vx - vy -(self.L + self.W)*wz)/self.R #angular velocuty_for_front_left_wheel
        w2 = (vx + vy +(self.L + self.W)*wz)/self.R #angular velocuty_for_front_right_wheel
        w3 = (vx + vy - (self.L + self.W)*wz)/self.R #angular velocuty_for_rear_left_wheel
        w4 = (vx - vy +(self.L + self.W)*wz)/self.R #angular velocuty_for_rear_right_wheel
        w = [w1,w2,w3,w4]
        return w

    def forward(self, w):
        vx = (w[0] + w[1] + w[2] + w[3])*(self.R/4)
        vy = (-w[0] + w[1] + w[2] - w[3])*(self.R/4)
        wz = (-w[0] + w[1] - w[2] + w[3])*(self.R/(4*(self.L+self.W)))
        chassis_vel = [vx,vy,wz]
        return chassis_vel


class ThreeWheelOmniKinematics(Kinematics):
    """3-wheel omni drive, wheels typically at 120 degrees apart."""

    def __init__(self, L: float, W: float, R: float):
        super().__init__(L, W, R)
        # TODO(Member 3): define self.M_inverse / self.M_forward
        self.M_inverse = None
        self.M_forward = None

    def inverse(self, vx: float, vy: float, wz: float):
        w1 = (-vx + self.L * wz) /self.R
        w2 = (0.5 * vx - ((3**0.5)/2) * vy + self.L *wz ) /self.R
        w3 = (0.5 * vx + ((3**0.5)/2) *vy +self.L *wz)/self.R
        w = [w1,w2,w3]
        return w

    def forward(self, w):
        vx = (-2 * w[0] + w[1] +w[2])*(self.R/3)
        vy = (-w[1] + w[2])*(self.R/(3**0.5))
        wz = (w[0] + w[1] + w[2])*(self.R / (3*self.L))
        chassis_vel = [vx,vy,wz]
        return chassis_vel
        


class FourWheelOmniKinematics(Kinematics):
    """4-wheel omni drive, wheels typically at 45 degree rollers."""

    def __init__(self, L: float, W: float, R: float):
        super().__init__(L, W, R)
        # TODO(Member 3): define self.M_inverse / self.M_forward
        self.M_inverse = None
        self.M_forward = None

    def inverse(self, vx: float, vy: float, wz: float):
        w1 = (- vx *((2**0.5)/2)+vy* ((2**0.5)/2)+ self.L * wz) / self.R
        w2 = ( vx *((2**0.5)/2)+vy* ((2**0.5)/2)- self.L * wz) / self.R
        w3 = ( vx *((2**0.5)/2)+vy* ((2**0.5)/2)+ self.L * wz) / self.R
        w4 = (- vx *((2**0.5)/2)+vy* ((2**0.5)/2)- self.L * wz) / self.R
        w = [w1,w2,w3,w4]
        return w
    
    def forward(self, w):
        vx = (-w[0] + w[1] + w[2] - w[3])*(self.R*(2**0.5)/4)
        vy = (w[0] + w[1] + w[2] + w[3])*(self.R*(2**0.5)/4)
        wz = (w[0] - w[1] + w[2] - w[3])*(self.R/(4*self.L))
        chassis_vel = [vx,vy,wz]
        return chassis_vel



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


