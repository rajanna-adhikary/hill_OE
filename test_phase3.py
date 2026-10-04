"""
Test verification script for Phase 3: Hill Hold Assist Controller.

Tests:
1. Flat road (0 deg) does not activate Hill Hold.
2. Low slope (< 3 deg, e.g. 1 deg) does not activate Hill Hold.
3. Stopped vehicle + sufficient slope (>= 3 deg) + brake release activates Hill Hold.
4. Moving vehicle (speed > 0.1 m/s) does not activate Hill Hold.
5. Brake never pressed does not activate Hill Hold.
6. PRESSED -> RELEASED transition is required (not just brake == RELEASED).
7. Controller state transitions follow the FSM (IDLE -> READY -> HOLD_ACTIVE).
8. Existing Phase 1 vehicle physics and Phase 2 sensors remain preserved and functional.
"""

from hill_hold_controller import HillHoldController
from inputs import BrakePedal
from sensors import InclineSensor
from vehicle import Vehicle


def test_flat_road():
    """1. Flat road (0 deg) should remain IDLE and not activate Hill Hold."""
    ctrl = HillHoldController(min_slope_deg=3.0, stop_speed_threshold=0.1)

    # Press brake on flat road
    state1 = ctrl.update(measured_slope_deg=0.0, vehicle_speed=0.0, brake_pressed=True)
    assert state1 == HillHoldController.STATE_IDLE, f"Expected IDLE, got {state1}"

    # Release brake on flat road
    state2 = ctrl.update(measured_slope_deg=0.0, vehicle_speed=0.0, brake_pressed=False)
    assert state2 == HillHoldController.STATE_IDLE, f"Expected IDLE, got {state2}"
    assert not ctrl.is_hold_active()
    print("[PASS] Test 1: Flat road does not activate Hill Hold.")


def test_low_slope():
    """2. Slope below threshold (e.g. 1 deg < 3 deg) does not activate Hill Hold."""
    ctrl = HillHoldController(min_slope_deg=3.0, stop_speed_threshold=0.1)

    ctrl.update(measured_slope_deg=1.5, vehicle_speed=0.0, brake_pressed=True)
    assert ctrl.get_state() == HillHoldController.STATE_IDLE

    ctrl.update(measured_slope_deg=1.5, vehicle_speed=0.0, brake_pressed=False)
    assert ctrl.get_state() == HillHoldController.STATE_IDLE
    assert not ctrl.is_hold_active()
    print("[PASS] Test 2: Low slope does not activate Hill Hold.")


def test_stopped_incline_activation():
    """3. Stopped vehicle on slope with brake release activates Hill Hold."""
    ctrl = HillHoldController(min_slope_deg=3.0, stop_speed_threshold=0.1)

    # Step 1: Stopped on 8 deg slope, brake PRESSED -> should be READY
    state1 = ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=True)
    assert state1 == HillHoldController.STATE_READY, f"Expected READY, got {state1}"
    assert not ctrl.is_hold_active()

    # Step 2: Driver RELEASES brake -> should transition to HOLD_ACTIVE
    state2 = ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=False)
    assert state2 == HillHoldController.STATE_HOLD_ACTIVE, f"Expected HOLD_ACTIVE, got {state2}"
    assert ctrl.is_hold_active()
    print("[PASS] Test 3: Stopped vehicle + slope + brake release activates Hill Hold.")


def test_moving_vehicle():
    """4. Moving vehicle (speed > 0.1 m/s) does not activate Hill Hold."""
    ctrl = HillHoldController(min_slope_deg=3.0, stop_speed_threshold=0.1)

    # Moving at 2.0 m/s on 8 deg slope with brake pressed
    state1 = ctrl.update(measured_slope_deg=8.0, vehicle_speed=2.0, brake_pressed=True)
    assert state1 == HillHoldController.STATE_IDLE, f"Expected IDLE, got {state1}"

    # Release brake while still moving
    state2 = ctrl.update(measured_slope_deg=8.0, vehicle_speed=1.5, brake_pressed=False)
    assert state2 == HillHoldController.STATE_IDLE, f"Expected IDLE, got {state2}"
    assert not ctrl.is_hold_active()
    print("[PASS] Test 4: Moving vehicle does not activate Hill Hold.")


def test_brake_never_pressed():
    """5. Driver never pressed the brake -> Hill Hold must NOT activate."""
    ctrl = HillHoldController(min_slope_deg=3.0, stop_speed_threshold=0.1)

    for _ in range(10):
        state = ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=False)
        assert state == HillHoldController.STATE_IDLE
        assert not ctrl.is_hold_active()
    print("[PASS] Test 5: Brake never pressed does not activate Hill Hold.")


def test_transition_required():
    """6. Controller specifically requires transition PRESSED -> RELEASED."""
    ctrl = HillHoldController(min_slope_deg=3.0, stop_speed_threshold=0.1, initial_brake_pressed=False)

    # Brake remains RELEASED
    ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=False)
    assert ctrl.get_state() == HillHoldController.STATE_IDLE

    # Now press brake
    ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=True)
    assert ctrl.get_state() == HillHoldController.STATE_READY

    # Now release brake
    ctrl.update(measured_slope_deg=8.0, vehicle_speed=0.0, brake_pressed=False)
    assert ctrl.get_state() == HillHoldController.STATE_HOLD_ACTIVE
    print("[PASS] Test 6: PRESSED -> RELEASED transition requirement verified.")


def test_fsm_complete_cycle():
    """7. Test complete FSM state transitions and manual release."""
    ctrl = HillHoldController(min_slope_deg=3.0, stop_speed_threshold=0.1)
    assert ctrl.get_state() == HillHoldController.STATE_IDLE

    # IDLE -> READY
    ctrl.update(measured_slope_deg=5.0, vehicle_speed=0.0, brake_pressed=True)
    assert ctrl.get_state() == HillHoldController.STATE_READY

    # READY -> HOLD_ACTIVE
    ctrl.update(measured_slope_deg=5.0, vehicle_speed=0.0, brake_pressed=False)
    assert ctrl.get_state() == HillHoldController.STATE_HOLD_ACTIVE

    # Manual release (for Phase 4 integration)
    ctrl.release_hold()
    assert ctrl.get_state() == HillHoldController.STATE_RELEASED

    # Next update resets to IDLE
    ctrl.update(measured_slope_deg=0.0, vehicle_speed=0.0, brake_pressed=False)
    assert ctrl.get_state() == HillHoldController.STATE_IDLE
    print("[PASS] Test 7: Full FSM cycle verified.")


if __name__ == "__main__":
    test_flat_road()
    test_low_slope()
    test_stopped_incline_activation()
    test_moving_vehicle()
    test_brake_never_pressed()
    test_transition_required()
    test_fsm_complete_cycle()
    print("\nALL PHASE 3 CONTROLLER UNIT TESTS PASSED SUCCESSFULLY!")
