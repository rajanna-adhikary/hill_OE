"""
Brake Actuator Model for Software Defined Vehicle (SDV) - Phase 4.

This module models a simplified software-controlled brake actuator that exerts
positive (uphill) braking force to counteract downhill gravitational force.

Forces & Constraints:
--------------------
- F_gravity = -m * g * sin(theta)  [acts downhill, negative direction]
- F_brake   = positive force acting uphill
- F_net     = F_gravity + F_brake

Holding Constraint:
-------------------
When holding a stationary vehicle, the brake acts as a constraint against rollback:
  effective_brake = min(applied_force, abs(F_gravity))
The brake will never accelerate the vehicle uphill (no positive motion from brakes alone).
"""

import math
from config import GRAVITY, MAX_BRAKE_FORCE


class BrakeActuator:
    """
    Simulated software-controlled brake actuator.

    Attributes:
        max_brake_force (float): Maximum available braking force in Newtons (N).
        current_brake_force (float): Currently exerted braking force in Newtons (N).
    """

    def __init__(self, max_brake_force: float = MAX_BRAKE_FORCE):
        """
        Initialize the brake actuator.

        Args:
            max_brake_force: Maximum allowable clamping force in Newtons (N).
        """
        self.max_brake_force = float(max_brake_force)
        self.current_brake_force = 0.0
        self._active = False

    def apply(self, force: float) -> float:
        """
        Apply requested braking force, clamped between 0 and max_brake_force.

        Args:
            force: Desired braking force in Newtons (N).

        Returns:
            float: Clamped brake force exerted in Newtons (N).
        """
        clamped = max(0.0, min(float(force), self.max_brake_force))
        self.current_brake_force = clamped
        self._active = (self.current_brake_force > 0.0)
        return self.current_brake_force

    def release(self) -> None:
        """
        Release the brake actuator, resetting applied force to 0 N and disarming.
        """
        self.current_brake_force = 0.0
        self._active = False

    def get_force(self) -> float:
        """
        Get the currently applied brake force.

        Returns:
            float: Applied force in Newtons (N).
        """
        return self.current_brake_force

    def is_active(self) -> bool:
        """
        Check if the actuator is currently exerting braking force.

        Returns:
            bool: True if brake force > 0, False otherwise.
        """
        return self._active

    def check_rollback_risk(self, required_force: float) -> bool:
        """
        Check whether the required brake force exceeds the maximum available brake capacity.

        Args:
            required_force: Required uphill force to counteract gravity in Newtons (N).

        Returns:
            bool: True if required_force > max_brake_force (insufficient capacity), False otherwise.
        """
        return float(required_force) > self.max_brake_force

    @staticmethod
    def calculate_required_force(mass: float, slope_rad: float) -> float:
        """
        Calculate the required uphill brake force to balance downhill gravity:
            F_req = m * g * sin(theta)

        Args:
            mass: Vehicle mass in kg.
            slope_rad: Road incline angle in radians.

        Returns:
            float: Required uphill force in Newtons (N).
        """
        return float(mass * GRAVITY * math.sin(slope_rad))
