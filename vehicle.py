"""
Vehicle Model for 1D Longitudinal Motion on an Incline.

This module defines the Vehicle class, representing the physical vehicle state
(mass, position, velocity, acceleration) and updating its motion using Euler
numerical integration.

Coordinate & Sign Convention:
-----------------------------
- Longitudinal coordinate axis x runs parallel to the road surface.
- Positive direction (+x): UPHILL.
- Negative direction (-x): DOWNHILL.
- Road incline angle theta is positive for an uphill slope (theta > 0).
- Gravity acts downward; resolving along the slope produces a force directed
  DOWNHILL, hence F_gravity is negative for theta > 0:
      F_gravity = -m * g * sin(theta)
- Brake actuator exerts positive (uphill) force: F_brake >= 0.
- Net force: F_net = F_gravity + F_brake.
- Holding Constraint: When stationary, brake force counteracts downhill gravity
  up to available capacity, but cannot propel the vehicle uphill:
      effective_brake = min(F_brake, abs(F_gravity))
"""

import math
from config import GRAVITY, INITIAL_POSITION, INITIAL_VELOCITY, VEHICLE_MASS


class Vehicle:
    """
    A 1D longitudinal vehicle model subject to gravitational force and braking on an incline.
    """

    def __init__(
        self,
        mass: float = VEHICLE_MASS,
        initial_position: float = INITIAL_POSITION,
        initial_velocity: float = INITIAL_VELOCITY,
    ):
        """
        Initialize the vehicle with mass, initial position, and initial velocity.

        Args:
            mass: Vehicle mass in kilograms (kg).
            initial_position: Initial longitudinal position in meters (m).
            initial_velocity: Initial longitudinal velocity in meters/second (m/s).
        """
        self.mass = float(mass)
        self.position = float(initial_position)
        self.velocity = float(initial_velocity)
        self.acceleration = 0.0

    def calculate_gravitational_force(self, slope_angle_rad: float) -> float:
        """
        Calculate the gravitational force component acting along the slope.

        Equation:
            F_gravity = -m * g * sin(theta)

        Args:
            slope_angle_rad: Road incline angle in radians (positive = uphill).

        Returns:
            Gravitational force along the slope in Newtons (N).
            Negative sign indicates downhill pull.
        """
        return -self.mass * GRAVITY * math.sin(slope_angle_rad)

    def calculate_acceleration(self, net_force: float) -> float:
        """
        Calculate vehicle longitudinal acceleration using Newton's Second Law:
            a = F_net / m

        Args:
            net_force: Total net longitudinal force in Newtons (N).

        Returns:
            Longitudinal acceleration in m/s^2.
        """
        self.acceleration = net_force / self.mass
        return self.acceleration

    def update_velocity(self, dt: float) -> float:
        """
        Update vehicle velocity using Euler integration:
            v_new = v_current + a * dt

        Args:
            dt: Timestep duration in seconds (s).

        Returns:
            Updated velocity in m/s.
        """
        self.velocity += self.acceleration * dt
        return self.velocity

    def update_position(self, dt: float) -> float:
        """
        Update vehicle position using Euler integration:
            x_new = x_current + v * dt

        Args:
            dt: Timestep duration in seconds (s).

        Returns:
            Updated position in meters (m).
        """
        self.position += self.velocity * dt
        return self.position

    def update_state(
        self,
        slope_angle_rad: float,
        dt: float,
        brake_force: float = 0.0,
    ) -> None:
        """
        Update the complete vehicle state for one simulation timestep dt.

        Process:
            1. Calculate downhill gravitational force along the slope.
            2. When brake_force > 0, apply static holding constraint:
               effective_brake = min(brake_force, abs(F_gravity))
               F_net = F_gravity + effective_brake
            3. Calculate acceleration: a = F_net / m
            4. Update velocity: v = v + a * dt
            5. Update position: x = x + v * dt

        Args:
            slope_angle_rad: Road incline angle in radians.
            dt: Timestep duration in seconds (s).
            brake_force: Applied uphill braking force in Newtons (N). Defaults to 0.0.
        """
        f_gravity = self.calculate_gravitational_force(slope_angle_rad)

        if brake_force > 0.0:
            # Brake opposes downhill gravity. Static holding constraint ensures
            # the brake cannot propel the vehicle uphill.
            effective_brake = min(float(brake_force), abs(f_gravity))
            net_force = f_gravity + effective_brake
        else:
            net_force = f_gravity

        # Clean numerical precision when forces perfectly cancel
        if abs(net_force) < 1e-9:
            net_force = 0.0

        self.calculate_acceleration(net_force)

        # Holding constraint: if acceleration is zero and vehicle was at rest, stay at rest
        if abs(self.acceleration) == 0.0 and abs(self.velocity) < 1e-9:
            self.velocity = 0.0
            return

        self.update_velocity(dt)
        self.update_position(dt)

    def reset(
        self,
        position: float = INITIAL_POSITION,
        velocity: float = INITIAL_VELOCITY,
    ) -> None:
        """
        Reset dynamic state variables to initial conditions.
        """
        self.position = float(position)
        self.velocity = float(velocity)
        self.acceleration = 0.0

