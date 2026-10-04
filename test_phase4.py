"""
Test verification script for Phase 4:
Brake Actuator, Force Balance, Hold Timer, and Vehicle Response.

Tests:
1. Brake actuator can apply force.
2. Brake actuator can release.
3. Brake force is clamped to maximum.
4. Required brake force calculation is correct: F_req = m * g * sin(theta).
5. Hill Hold applies brake when controller becomes HOLD_ACTIVE.
6. Vehicle remains approximately stationary during a successful hold (v ≈ 0, x ≈ 0).
7. Hold timer lasts approximately 3.0 seconds of simulation time.
8. Brake releases after timer expiration.
9. Vehicle begins rolling downhill after release.
10. Insufficient brake force produces ROLLBACK_RISK.
11. Regression verification: Phase 2 tests still pass.
12. Regression verification: Phase 3 tests still pass.
"""

import math
import subprocess
import sys
from brake_actuator import BrakeActuator
from config import (
    GRAVITY,
    HILL_HOLD_DURATION,
    INSUFFICIENT_MAX_BRAKE_FORCE,
    MAX_BRAKE_FORCE,
    TIMESTEP,
    VEHICLE_MASS,
)
from hill_hold_controller import HillHoldController
from inputs import BrakePedal
from sensors import InclineSensor
from vehicle import Vehicle


def test_actuator_apply():
    """1. Brake actuator can apply force."""
    actuator = BrakeActuator(max_brake_force=3000.0)
    applied = actuator.apply(1500.0)
    assert applied == 1500.0
    assert actuator.get_force() == 1500.0
    assert actuator.is_active()
    print("[PASS] Test 1: Brake actuator can apply force.")


def test_actuator_release():
    """2. Brake actuator can release."""
    actuator = BrakeActuator(max_brake_force=3000.0)
    actuator.apply(2000.0)
    actuator.release()
    assert actuator.get_force() == 0.0
    assert not actuator.is_active()
    print("[PASS] Test 2: Brake actuator can release.")


def test_actuator_clamping():
    """3. Brake force is clamped to maximum."""
    actuator = BrakeActuator(max_brake_force=2500.0)
    applied = actuator.apply(4000.0)
    assert applied == 2500.0
    assert actuator.get_force() == 2500.0

    # Negative force clamped to 0
    applied_neg = actuator.apply(-100.0)
    assert applied_neg == 0.0
    print("[PASS] Test 3: Brake force is clamped to maximum.")


def test_required_force_calculation():
    """4. Required brake force calculation is correct: F_req = m * g * sin(theta)."""
    mass = 1500.0
    slope_rad = math.radians(8.0)
    expected_force = mass * GRAVITY * math.sin(slope_rad)  # ~2048.0 N
    calculated_force = BrakeActuator.calculate_required_force(mass, slope_rad)
    assert abs(calculated_force - expected_force) < 1e-4
    assert abs(calculated_force - 2048.0) < 5.0
    print("[PASS] Test 4: Required brake force calculation is correct.")


def test_controller_actuator_engagement():
    """5. Hill Hold applies brake when controller becomes HOLD_ACTIVE."""
    ctrl = HillHoldController(min_slope_deg=3.0, stop_speed_threshold=0.1)
    actuator = BrakeActuator(max_brake_force=3000.0)

    # Arm controller
    ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=True)
    assert ctrl.get_state() == HillHoldController.STATE_READY

    # Release pedal -> triggers HOLD_ACTIVE
    ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=False)
    assert ctrl.is_hold_active()

    # Engagement: Actuator applies required force
    f_req = BrakeActuator.calculate_required_force(1500.0, math.radians(8.0))
    actuator.apply(f_req)
    assert actuator.is_active()
    assert abs(actuator.get_force() - f_req) < 1e-4
    print("[PASS] Test 5: Hill Hold applies brake when controller becomes HOLD_ACTIVE.")


def test_vehicle_held_stationary():
    """6. Vehicle remains approximately stationary during a successful hold."""
    vehicle = Vehicle(mass=1500.0, initial_position=0.0, initial_velocity=0.0)
    slope_rad = math.radians(8.0)
    f_req = BrakeActuator.calculate_required_force(vehicle.mass, slope_rad)

    # Run 100 timesteps with brake applied
    for _ in range(100):
        vehicle.update_state(slope_rad, TIMESTEP, brake_force=f_req)

    assert abs(vehicle.velocity) < 1e-6
    assert abs(vehicle.position) < 1e-6
    print("[PASS] Test 6: Vehicle remains stationary during hold.")


def test_hold_timer_duration():
    """7. Hold timer lasts approximately 3 seconds of simulation time."""
    ctrl = HillHoldController(min_slope_deg=3.0, stop_speed_threshold=0.1, hold_duration=3.0)
    ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=True)
    ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=False)  # Becomes HOLD_ACTIVE

    dt = 0.01
    # Run 2.9 seconds -> should still be active
    for _ in range(290):
        ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=False, dt=dt)
    assert ctrl.is_hold_active()
    assert abs(ctrl.get_hold_elapsed_time() - 2.9) < 1e-3

    # Run remaining 0.11 seconds -> should expire
    for _ in range(11):
        ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=False, dt=dt)
    assert not ctrl.is_hold_active()
    print("[PASS] Test 7: Hold timer lasts exactly 3.0 seconds.")


def test_brake_releases_after_timer():
    """8. Brake releases after timer expiration."""
    ctrl = HillHoldController(min_slope_deg=3.0, stop_speed_threshold=0.1, hold_duration=3.0)
    actuator = BrakeActuator(max_brake_force=3000.0)

    ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=True)
    ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=False)
    actuator.apply(2048.0)

    # Step through 3.0 seconds
    for _ in range(300):
        ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=False, dt=0.01)

    assert not ctrl.is_hold_active()
    actuator.release()
    assert actuator.get_force() == 0.0
    assert not actuator.is_active()
    print("[PASS] Test 8: Brake releases after timer expiration.")


def test_vehicle_rolls_after_release():
    """9. Vehicle begins rolling downhill after release."""
    vehicle = Vehicle(mass=1500.0, initial_position=0.0, initial_velocity=0.0)
    slope_rad = math.radians(8.0)
    f_req = BrakeActuator.calculate_required_force(vehicle.mass, slope_rad)

    # 3.0 seconds held (brake active)
    for _ in range(300):
        vehicle.update_state(slope_rad, TIMESTEP, brake_force=f_req)
    assert abs(vehicle.velocity) < 1e-6

    # 1.0 second post-release (brake = 0)
    for _ in range(100):
        vehicle.update_state(slope_rad, TIMESTEP, brake_force=0.0)

    # Vehicle should have rolled downhill (negative velocity and position)
    assert vehicle.velocity < -1.0
    assert vehicle.position < -0.5
    print("[PASS] Test 9: Vehicle begins rolling downhill after release.")


def test_rollback_risk():
    """10. Insufficient brake force produces ROLLBACK_RISK."""
    actuator = BrakeActuator(max_brake_force=INSUFFICIENT_MAX_BRAKE_FORCE)  # 1000 N
    f_req = BrakeActuator.calculate_required_force(1500.0, math.radians(8.0))  # ~2048 N

    assert actuator.check_rollback_risk(f_req)

    # Apply insufficient brake to vehicle
    vehicle = Vehicle(mass=1500.0, initial_position=0.0, initial_velocity=0.0)
    applied = actuator.apply(f_req)
    assert applied == 1000.0

    # Step vehicle: net force is -2048 + 1000 = -1048 N -> vehicle rolls back!
    for _ in range(100):
        vehicle.update_state(math.radians(8.0), TIMESTEP, brake_force=applied)

    assert vehicle.velocity < 0.0
    assert vehicle.position < 0.0
    print("[PASS] Test 10: Insufficient brake force produces ROLLBACK_RISK.")


def test_regression_phase2():
    """11. Phase 2 tests still pass."""
    result = subprocess.run([sys.executable, "test_phase2.py"], capture_output=True, text=True)
    assert result.returncode == 0, f"Phase 2 tests failed:\n{result.stderr}"
    print("[PASS] Test 11: Phase 2 regression tests pass.")


def test_regression_phase3():
    """12. Phase 3 tests still pass."""
    result = subprocess.run([sys.executable, "test_phase3.py"], capture_output=True, text=True)
    assert result.returncode == 0, f"Phase 3 tests failed:\n{result.stderr}"
    print("[PASS] Test 12: Phase 3 regression tests pass.")


if __name__ == "__main__":
    test_actuator_apply()
    test_actuator_release()
    test_actuator_clamping()
    test_required_force_calculation()
    test_controller_actuator_engagement()
    test_vehicle_held_stationary()
    test_hold_timer_duration()
    test_brake_releases_after_timer()
    test_vehicle_rolls_after_release()
    test_rollback_risk()
    test_regression_phase2()
    test_regression_phase3()
    print("\nALL 12 PHASE 4 VERIFICATION TESTS PASSED SUCCESSFULLY!")
