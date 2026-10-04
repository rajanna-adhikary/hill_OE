"""
Simulation Runner for Phase 1: 1D Longitudinal Vehicle on Sloped Road.

This module executes the longitudinal vehicle simulation across multiple road
incline angles (e.g., 0°, 5°, 8°), outputs numerical summary metrics, and generates
separate visualization plots for acceleration, velocity, and position over time.

Sign Convention Reminder:
-------------------------
- Positive (+) : Uphill direction
- Negative (-) : Downhill direction
- On an uphill incline, gravity produces negative acceleration, leading to
  downhill rollback (negative velocity and negative position).
"""

import math
import sys
import matplotlib.pyplot as plt

from config import (
    GRAVITY,
    INITIAL_POSITION,
    INITIAL_VELOCITY,
    MAX_BRAKE_FORCE,
    SIMULATION_DURATION,
    TEST_SLOPES_DEG,
    TIMESTEP,
    VEHICLE_MASS,
)
from vehicle import Vehicle


def run_phase4_scenario(
    scenario: dict,
    sensor,
    duration: float = SIMULATION_DURATION,
    dt: float = TIMESTEP,
) -> dict:
    """
    Run a full Phase 4 closed-loop Hill Hold Assist scenario.

    Args:
        scenario: Dict with keys: slope_deg, initial_velocity, brake_behavior, max_brake_force.
        sensor: InclineSensor instance.
        duration: Simulation duration in seconds.
        dt: Integration timestep in seconds.

    Returns:
        dict with time-series arrays and scalar summary fields.
    """
    import math
    from brake_actuator import BrakeActuator
    from hill_hold_controller import HillHoldController
    from inputs import BrakePedal

    slope_deg = float(scenario["slope_deg"])
    slope_rad = math.radians(slope_deg)
    max_brake_force = float(scenario.get("max_brake_force", MAX_BRAKE_FORCE))
    brake_behavior = scenario.get("brake_behavior", "PRESSED_THEN_RELEASED")
    init_v = float(scenario.get("initial_velocity", INITIAL_VELOCITY))

    from config import BRAKE_RELEASE_TIME

    vehicle = Vehicle(mass=VEHICLE_MASS, initial_position=INITIAL_POSITION, initial_velocity=init_v)
    actuator = BrakeActuator(max_brake_force=max_brake_force)
    controller = HillHoldController()
    pedal = BrakePedal(initially_pressed=False)

    f_req = BrakeActuator.calculate_required_force(VEHICLE_MASS, slope_rad)
    rollback_risk = actuator.check_rollback_risk(f_req)

    # Sample slope once for dashboard display (first sensor read)
    measured_slope_deg = sensor.read_angle(slope_deg)

    time_series, vel_series, pos_series, acc_series = [0.0], [vehicle.velocity], [vehicle.position], [0.0]
    brake_series, state_series, hold_series, pedal_series = [0.0], [controller.get_state()], [False], ["RELEASED"]

    current_time = 0.0
    num_steps = int(round(duration / dt))

    for _ in range(num_steps):
        current_time += dt

        # Driver pedal logic: pressed before BRAKE_RELEASE_TIME, released after
        if brake_behavior == "PRESSED_THEN_RELEASED":
            brake_pressed = current_time <= BRAKE_RELEASE_TIME
        else:
            brake_pressed = False

        if brake_pressed:
            pedal.press()
        else:
            pedal.release()

        meas = sensor.read_angle(slope_deg)
        ctrl_state = controller.update(
            measured_slope_deg=meas,
            vehicle_speed=vehicle.velocity,
            brake_pressed=pedal.is_pressed(),
            dt=dt,
        )

        hold_active = controller.is_hold_active()
        if hold_active:
            applied_force = actuator.apply(f_req)
        else:
            actuator.release()
            applied_force = 0.0

        vehicle.update_state(slope_rad, dt, brake_force=applied_force)

        time_series.append(current_time)
        vel_series.append(vehicle.velocity)
        pos_series.append(vehicle.position)
        acc_series.append(vehicle.acceleration)
        brake_series.append(applied_force)
        state_series.append(ctrl_state)
        hold_series.append(hold_active)
        pedal_series.append(pedal.get_state_string())

    return {
        "time": time_series,
        "velocity": vel_series,
        "position": pos_series,
        "acceleration": acc_series,
        "brake_force": brake_series,
        "controller_state": state_series,
        "hold_active": hold_series,
        "brake_pedal": pedal_series,
        "measured_slope_deg": measured_slope_deg,
        "rollback_risk": rollback_risk,
    }


def run_single_simulation(slope_deg: float, duration: float = SIMULATION_DURATION, dt: float = TIMESTEP) -> dict:
    """
    Run the vehicle simulation for a single incline angle.

    Args:
        slope_deg: Road incline angle in degrees.
        duration: Total simulation time in seconds.
        dt: Integration timestep in seconds.

    Returns:
        dict: Time series data containing 'time', 'acceleration', 'velocity',
              'position', and configuration metadata.
    """
    vehicle = Vehicle(
        mass=VEHICLE_MASS,
        initial_position=INITIAL_POSITION,
        initial_velocity=INITIAL_VELOCITY,
    )
    slope_rad = math.radians(slope_deg)

    # Pre-allocate lists for recording simulation trajectory
    time_series = [0.0]
    acc_series = [0.0]  # Vehicle starts at rest with 0 initial acceleration before physics step
    vel_series = [vehicle.velocity]
    pos_series = [vehicle.position]

    current_time = 0.0
    num_steps = int(round(duration / dt))

    for _ in range(num_steps):
        vehicle.update_state(slope_rad, dt)
        current_time += dt

        time_series.append(current_time)
        acc_series.append(vehicle.acceleration)
        vel_series.append(vehicle.velocity)
        pos_series.append(vehicle.position)

    return {
        "slope_deg": slope_deg,
        "slope_rad": slope_rad,
        "time": time_series,
        "acceleration": acc_series,
        "velocity": vel_series,
        "position": pos_series,
    }


def print_simulation_summary(all_results: list[dict]) -> None:
    """
    Print a formatted summary table of simulation results for all slopes.
    """
    print("\n" + "=" * 78)
    print("      SOFTWARE DEFINED VEHICLE (SDV) - PHASE 1 SIMULATION SUMMARY")
    print("=" * 78)
    print(f"Vehicle Mass: {VEHICLE_MASS:.1f} kg | Gravity: {GRAVITY:.2f} m/s^2 | Timestep dt: {TIMESTEP} s")
    print(f"Duration: {SIMULATION_DURATION:.1f} s | Initial Position: {INITIAL_POSITION:.1f} m | Initial Velocity: {INITIAL_VELOCITY:.1f} m/s")
    print("-" * 78)
    print("Sign Convention: (+) Uphill  |  (-) Downhill")
    print("-" * 78)
    header = (
        f"{'Slope (deg)':^12} | "
        f"{'Slope (rad)':^12} | "
        f"{'Accel (m/s^2)':^14} | "
        f"{'Final Vel (m/s)':^16} | "
        f"{'Final Pos (m)':^14}"
    )
    print(header)
    print("-" * 78)

    for res in all_results:
        slope = res["slope_deg"]
        slope_rad = res["slope_rad"]
        # Acceleration after initial step is steady: -g * sin(theta)
        final_acc = res["acceleration"][-1]
        final_vel = res["velocity"][-1]
        final_pos = res["position"][-1]

        row = (
            f"{slope:^12.1f} | "
            f"{slope_rad:^12.4f} | "
            f"{final_acc:^14.4f} | "
            f"{final_vel:^16.4f} | "
            f"{final_pos:^14.4f}"
        )
        print(row)

    print("-" * 78)
    print("Key Observations:")
    print("  * 0.0 deg slope: No downhill gravity component. Vehicle stays stationary at origin.")
    print("  * > 0.0 deg slope: Negative acceleration causes downhill rollback.")
    print("  * Steeper slope (8 deg vs 5 deg): Larger downhill acceleration, velocity, & displacement.")
    print("=" * 78 + "\n")


def plot_results(all_results: list[dict], save_figures: bool = True, show_plots: bool = True) -> None:
    """
    Plot velocity vs time, position vs time, and acceleration vs time.
    As per requirements, each graph is created in its own separate figure (NO subplots).

    Args:
        all_results: List of result dictionaries from `run_single_simulation`.
        save_figures: If True, saves PNG images to disk.
        show_plots: If True, calls plt.show() to display interactive windows.
    """
    colors = ["#2ca02c", "#1f77b4", "#d62728", "#9467bd"]

    # -------------------------------------------------------------
    # Figure 1: Velocity vs Time
    # -------------------------------------------------------------
    plt.figure(figsize=(8, 5))
    for idx, res in enumerate(all_results):
        color = colors[idx % len(colors)]
        label = f"Slope = {res['slope_deg']}° (a = {res['acceleration'][-1]:.3f} m/s²)"
        plt.plot(res["time"], res["velocity"], label=label, color=color, linewidth=2)

    plt.title("Vehicle Velocity vs Time on Various Road Inclines", fontsize=13, fontweight="bold")
    plt.xlabel("Time (s)", fontsize=11)
    plt.ylabel("Velocity (m/s)\n[ (+) Uphill  |  (-) Downhill ]", fontsize=11)
    plt.axhline(0, color="black", linestyle="--", linewidth=0.8, alpha=0.7)
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.legend(loc="lower left", fontsize=10)
    plt.tight_layout()
    if save_figures:
        plt.savefig("velocity_vs_time.png", dpi=300)
        print("Saved plot: velocity_vs_time.png")

    # -------------------------------------------------------------
    # Figure 2: Position vs Time
    # -------------------------------------------------------------
    plt.figure(figsize=(8, 5))
    for idx, res in enumerate(all_results):
        color = colors[idx % len(colors)]
        label = f"Slope = {res['slope_deg']}° (final x = {res['position'][-1]:.2f} m)"
        plt.plot(res["time"], res["position"], label=label, color=color, linewidth=2)

    plt.title("Vehicle Position vs Time on Various Road Inclines", fontsize=13, fontweight="bold")
    plt.xlabel("Time (s)", fontsize=11)
    plt.ylabel("Position (m)\n[ (+) Uphill  |  (-) Downhill ]", fontsize=11)
    plt.axhline(0, color="black", linestyle="--", linewidth=0.8, alpha=0.7)
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.legend(loc="lower left", fontsize=10)
    plt.tight_layout()
    if save_figures:
        plt.savefig("position_vs_time.png", dpi=300)
        print("Saved plot: position_vs_time.png")

    # -------------------------------------------------------------
    # Figure 3: Acceleration vs Time
    # -------------------------------------------------------------
    plt.figure(figsize=(8, 5))
    for idx, res in enumerate(all_results):
        color = colors[idx % len(colors)]
        label = f"Slope = {res['slope_deg']}° (a = {res['acceleration'][-1]:.3f} m/s²)"
        plt.plot(res["time"], res["acceleration"], label=label, color=color, linewidth=2)

    plt.title("Vehicle Acceleration vs Time on Various Road Inclines", fontsize=13, fontweight="bold")
    plt.xlabel("Time (s)", fontsize=11)
    plt.ylabel("Acceleration (m/s²)\n[ (+) Uphill  |  (-) Downhill ]", fontsize=11)
    plt.axhline(0, color="black", linestyle="--", linewidth=0.8, alpha=0.7)
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.legend(loc="lower left", fontsize=10)
    plt.tight_layout()
    if save_figures:
        plt.savefig("acceleration_vs_time.png", dpi=300)
        print("Saved plot: acceleration_vs_time.png")

    if show_plots:
        plt.show()


def main() -> None:
    """
    Main entry point for running Phase 1 simulation experiments.
    """
    # Check command line flags (e.g. --no-gui for automated or headless test runs)
    show_gui = "--no-gui" not in sys.argv

    all_results = []
    print(f"Running Phase 1 simulation for slopes: {TEST_SLOPES_DEG} ...")
    for slope in TEST_SLOPES_DEG:
        res = run_single_simulation(slope_deg=slope, duration=SIMULATION_DURATION, dt=TIMESTEP)
        all_results.append(res)

    # 1. Print summary table
    print_simulation_summary(all_results)

    # 2. Generate and display separate plots
    plot_results(all_results, save_figures=True, show_plots=show_gui)


if __name__ == "__main__":
    main()
