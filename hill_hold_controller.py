"""
Hill Hold Assist (HHA) Controller for Software Defined Vehicle (SDV) - Phases 3 & 4.

This module implements the software controller that decides WHEN Hill Hold Assist
should be activated based on:
  1. Measured road incline angle (from InclineSensor)
  2. Vehicle longitudinal speed (|v|)
  3. Driver brake pedal state (PRESSED / RELEASED)
  4. Pedal transition history (specifically detecting PRESSED -> RELEASED)
  5. Hold timer management (Phase 4: holding for HILL_HOLD_DURATION seconds)

State Transitions:
------------------
  IDLE
    │
    │ vehicle stopped + slope >= threshold + brake pressed
    ▼
  READY
    │
    │ brake released (PRESSED -> RELEASED)
    ▼
  HOLD_ACTIVE  (starts hold timer)
    │
    │ hold_timer >= HILL_HOLD_DURATION
    ▼
  RELEASED     (disarms brake actuator)
    │
    ▼
  IDLE
"""

from typing import Optional
from config import HILL_HOLD_DURATION, MIN_SLOPE_DEG, STOP_SPEED_THRESHOLD


class HillHoldController:
    """
    Finite State Machine (FSM) controller for Hill Hold Assist activation and hold timing.

    States:
      - IDLE: System inactive or conditions not met.
      - READY: Primed/armed (vehicle stopped, on incline >= threshold, brake pressed).
      - HOLD_ACTIVE: Hill Hold command issued (triggered upon PRESSED -> RELEASED).
      - RELEASED: Hold duration expired or disarmed (actuator released).
    """

    STATE_IDLE = "IDLE"
    STATE_READY = "READY"
    STATE_HOLD_ACTIVE = "HOLD_ACTIVE"
    STATE_RELEASED = "RELEASED"

    def __init__(
        self,
        min_slope_deg: float = MIN_SLOPE_DEG,
        stop_speed_threshold: float = STOP_SPEED_THRESHOLD,
        hold_duration: float = HILL_HOLD_DURATION,
        initial_brake_pressed: bool = False,
    ):
        """
        Initialize the Hill Hold Controller.

        Args:
            min_slope_deg: Minimum road incline angle (deg) required to arm/activate.
            stop_speed_threshold: Maximum speed (m/s) below which vehicle is considered stopped.
            hold_duration: Automatic hold duration in seconds (s). Defaults to HILL_HOLD_DURATION.
            initial_brake_pressed: Initial brake pedal state at t = 0.
        """
        self.min_slope_deg = float(min_slope_deg)
        self.stop_speed_threshold = float(stop_speed_threshold)
        self.hold_duration = float(hold_duration)

        self.state = self.STATE_IDLE
        self.prev_brake_pressed = bool(initial_brake_pressed)
        self.hold_timer = 0.0

    def update(
        self,
        measured_slope_deg: float,
        vehicle_speed: float,
        brake_pressed: bool,
        dt: float = 0.0,
    ) -> str:
        """
        Process inputs for one simulation timestep and update controller state.

        Decision Rules:
          - Vehicle is stopped when: |vehicle_speed| <= stop_speed_threshold
          - Meaningful incline when: measured_slope_deg >= min_slope_deg
          - Armed (READY) when: stopped + on incline + brake is actively PRESSED
          - Hold Activated (HOLD_ACTIVE) when: armed + driver RELEASES brake (PRESSED -> RELEASED)
          - Hold Released (RELEASED) when: elapsed hold time >= hold_duration

        Args:
            measured_slope_deg: Measured road incline angle from InclineSensor (deg).
            vehicle_speed: Current vehicle longitudinal velocity or speed (m/s).
            brake_pressed: Current brake pedal state (True if PRESSED, False if RELEASED).
            dt: Timestep duration (s) to advance the hold timer. Defaults to 0.0.

        Returns:
            str: Updated controller state (IDLE, READY, HOLD_ACTIVE, or RELEASED).
        """
        speed = abs(vehicle_speed)
        is_stopped = speed <= self.stop_speed_threshold
        is_on_incline = measured_slope_deg >= self.min_slope_deg
        brake_released_transition = self.prev_brake_pressed and (not brake_pressed)

        # -------------------------------------------------------------
        # State Machine Transitions
        # -------------------------------------------------------------
        if self.state == self.STATE_IDLE:
            # Arm system if stopped on a slope with brake pressed
            if is_stopped and is_on_incline and brake_pressed:
                self.state = self.STATE_READY

        elif self.state == self.STATE_READY:
            # Trigger hold upon driver releasing the brake pedal (PRESSED -> RELEASED)
            if brake_released_transition:
                self.state = self.STATE_HOLD_ACTIVE
                self.hold_timer = 0.0
            elif not brake_pressed:
                self.state = self.STATE_IDLE
            elif not is_on_incline:
                self.state = self.STATE_IDLE

        elif self.state == self.STATE_HOLD_ACTIVE:
            # Advance hold timer in simulation time (Phase 4)
            if dt > 0.0:
                self.hold_timer += dt
                if (self.hold_timer + 1e-9) >= self.hold_duration:
                    self.state = self.STATE_RELEASED

        elif self.state == self.STATE_RELEASED:
            # Return to IDLE after release cycle
            self.state = self.STATE_IDLE
            self.hold_timer = 0.0

        # Record history for transition detection on next timestep
        self.prev_brake_pressed = bool(brake_pressed)
        return self.state

    def is_hold_active(self) -> bool:
        """
        Check if Hill Hold command is currently active.

        Returns:
            bool: True if state is HOLD_ACTIVE, False otherwise.
        """
        return self.state == self.STATE_HOLD_ACTIVE

    def get_state(self) -> str:
        """
        Get the current controller state string.

        Returns:
            str: IDLE, READY, HOLD_ACTIVE, or RELEASED.
        """
        return self.state

    def get_hold_elapsed_time(self) -> float:
        """
        Get the elapsed hold duration in seconds.

        Returns:
            float: Elapsed hold time in seconds.
        """
        return self.hold_timer

    def is_hold_expired(self) -> bool:
        """
        Check if the hold timer has expired.

        Returns:
            bool: True if elapsed time >= hold_duration.
        """
        return self.hold_timer >= self.hold_duration

    def release_hold(self) -> None:
        """
        Manually or timer-driven release of Hill Hold.
        """
        if self.state == self.STATE_HOLD_ACTIVE:
            self.state = self.STATE_RELEASED
            self.hold_timer = self.hold_duration

    def reset(self, initial_brake_pressed: bool = False) -> None:
        """
        Reset controller state, timer, and pedal history.
        """
        self.state = self.STATE_IDLE
        self.prev_brake_pressed = bool(initial_brake_pressed)
        self.hold_timer = 0.0

