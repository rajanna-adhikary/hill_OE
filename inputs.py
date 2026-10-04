"""
Driver Input Models for Software Defined Vehicle (SDV) - Phase 2.

This module models driver inputs. In Phase 2, we implement a simple binary
Brake Pedal input with two discrete states:
  - PRESSED
  - RELEASED

IMPORTANT ARCHITECTURAL NOTE:
-----------------------------
The brake pedal represents ONLY a human/system input signal.
It is NOT connected to any brake actuator, hydraulic caliper, or braking force
in this phase. Connecting this input signal to control logic and brake actuation
will occur in Phases 3 and 4.
"""


class BrakePedal:
    """
    Simulated driver brake pedal input.

    Maintains discrete state: PRESSED (True) or RELEASED (False).
    """

    STATE_PRESSED = "PRESSED"
    STATE_RELEASED = "RELEASED"

    def __init__(self, initially_pressed: bool = False):
        """
        Initialize the brake pedal state.

        Args:
            initially_pressed: Initial state (True for PRESSED, False for RELEASED).
        """
        self._pressed = bool(initially_pressed)

    def press(self) -> None:
        """Simulate driver pressing the brake pedal."""
        self._pressed = True

    def release(self) -> None:
        """Simulate driver releasing the brake pedal."""
        self._pressed = False

    def is_pressed(self) -> bool:
        """
        Check if the brake pedal is currently pressed.

        Returns:
            bool: True if PRESSED, False if RELEASED.
        """
        return self._pressed

    def get_state_string(self) -> str:
        """
        Get human-readable pedal state string ("PRESSED" or "RELEASED").

        Returns:
            str: "PRESSED" or "RELEASED".
        """
        return self.STATE_PRESSED if self._pressed else self.STATE_RELEASED

    def __repr__(self) -> str:
        return f"BrakePedal(state='{self.get_state_string()}')"
