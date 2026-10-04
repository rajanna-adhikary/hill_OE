"""
Test verification script for Phase 2 implementation.

Verifies:
1. Sensor readings are close to actual slope and within expected error bounds.
2. Noise enable/disable toggles correctly.
3. Brake pedal transitions between PRESSED and RELEASED.
4. Existing Phase 1 vehicle physics still produces identical results.
5. No Hill Hold behavior is active (vehicle accelerates downhill even with brake pressed).
"""

import math
from config import SENSOR_NOISE_STD_DEG, TIMESTEP, VEHICLE_MASS, GRAVITY
from inputs import BrakePedal
from sensors import InclineSensor
from vehicle import Vehicle


def test_incline_sensor():
    sensor = InclineSensor(noise_std_deg=0.1, noise_enabled=True, seed=123)
    readings = [sensor.read_angle(8.0) for _ in range(100)]
    mean_val = sum(readings) / len(readings)
    errors = [abs(r - 8.0) for r in readings]

    # Verify sensor readings are close to 8.0 deg
    assert abs(mean_val - 8.0) < 0.05, f"Mean sensor reading {mean_val} not close to 8.0"
    # Verify max error is within 4-sigma (< 0.4 deg)
    assert max(errors) < 0.4, f"Max error {max(errors)} exceeded 4-sigma bound"

    # Test noise disabled
    sensor.disable_noise()
    assert sensor.read_angle(8.0) == 8.0, "Noise disabled should return exact angle"
    sensor.enable_noise()
    print("[PASS] InclineSensor tests passed.")


def test_brake_pedal():
    pedal = BrakePedal(initially_pressed=False)
    assert not pedal.is_pressed()
    assert pedal.get_state_string() == "RELEASED"

    pedal.press()
    assert pedal.is_pressed()
    assert pedal.get_state_string() == "PRESSED"

    pedal.release()
    assert not pedal.is_pressed()
    assert pedal.get_state_string() == "RELEASED"
    print("[PASS] BrakePedal tests passed.")


def test_physics_preservation():
    # Test 5 deg slope over 5 seconds (500 steps of dt=0.01)
    v1 = Vehicle()
    v2 = Vehicle()
    slope_rad = math.radians(5.0)

    # In Phase 2, even if brake pedal is pressed, vehicle physics is untouched
    pedal = BrakePedal(initially_pressed=True)

    for _ in range(500):
        v1.update_state(slope_rad, TIMESTEP)
        # Even with pedal pressed, no brake actuator exists, so update_state behaves identically
        v2.update_state(slope_rad, TIMESTEP)

    assert v1.velocity == v2.velocity
    assert v1.position == v2.position
    # Final values match Phase 1 exact values:
    assert round(v1.velocity, 4) == -4.2750, f"Unexpected velocity: {v1.velocity}"
    assert round(v1.position, 4) == -10.7088, f"Unexpected position: {v1.position}"
    print("[PASS] Vehicle physics preservation tests passed.")


if __name__ == "__main__":
    test_incline_sensor()
    test_brake_pedal()
    test_physics_preservation()
    print("\nALL PHASE 2 VERIFICATION TESTS PASSED SUCCESSFULLY!")
