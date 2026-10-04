"""
Sensor Models for Software Defined Vehicle (SDV) - Phase 2.

This module provides simulated sensor models for vehicle state estimation.
In Phase 2, we simulate an Incline Sensor (Inclinometer / IMU pitch estimation)
that measures the road slope angle with optional Gaussian measurement noise.

Key Principle:
--------------
Actual road slope (ground truth from the physical environment) is distinct from
the measured slope (the noisy signal perceived by on-board electronic control units).
"""

import random
from typing import Optional
from config import SENSOR_NOISE_ENABLED, SENSOR_NOISE_STD_DEG, SENSOR_RANDOM_SEED


class InclineSensor:
    """
    Simulated Incline Sensor (Inclinometer / IMU pitch estimator).

    Converts ground truth road slope angle (degrees) into a measured angle,
    with configurable zero-mean Gaussian measurement noise.
    """

    def __init__(
        self,
        noise_std_deg: float = SENSOR_NOISE_STD_DEG,
        noise_enabled: bool = SENSOR_NOISE_ENABLED,
        seed: Optional[int] = SENSOR_RANDOM_SEED,
    ):
        """
        Initialize the incline sensor.

        Args:
            noise_std_deg: Standard deviation of Gaussian measurement noise in degrees.
            noise_enabled: Whether measurement noise is added to the reading.
            seed: Random seed for reproducible noise generation.
        """
        self.noise_std_deg = float(noise_std_deg)
        self.noise_enabled = bool(noise_enabled)
        # Use an isolated random instance to prevent interfering with global RNG state
        self._rng = random.Random(seed)

    def read_angle(self, actual_angle_deg: float) -> float:
        """
        Read the simulated measured road incline angle.

        Args:
            actual_angle_deg: Ground truth road incline angle in degrees.

        Returns:
            Measured road incline angle in degrees (with noise if enabled).
        """
        if self.noise_enabled and self.noise_std_deg > 0.0:
            noise = self._rng.gauss(0.0, self.noise_std_deg)
            return actual_angle_deg + noise
        return float(actual_angle_deg)

    def enable_noise(self) -> None:
        """Enable sensor measurement noise."""
        self.noise_enabled = True

    def disable_noise(self) -> None:
        """Disable sensor measurement noise (returns perfect ground truth)."""
        self.noise_enabled = False

    def set_seed(self, seed: Optional[int]) -> None:
        """Reset the random seed for reproducible testing."""
        self._rng.seed(seed)
