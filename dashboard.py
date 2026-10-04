"""
Software Defined Vehicle (SDV) - Hill Hold Assist Simulation Dashboard (Phase 5).

This interactive Streamlit dashboard demonstrates the complete closed-loop Hill Hold Assist system:
  1. Incline Sensor (measures slope with Gaussian noise)
  2. Driver Brake Pedal (PRESSED / RELEASED transition)
  3. Hill Hold Controller FSM (IDLE, READY, HOLD_ACTIVE, RELEASED)
  4. Brake Actuator (required force calculation, clamping, and rollback risk)
  5. Vehicle Physics (Euler longitudinal motion with holding constraint)
  6. Hold Timer (3.0s holding duration)

Run Command:
  streamlit run dashboard.py
"""

import math
import matplotlib.pyplot as plt
import numpy as np
import streamlit as st

from brake_actuator import BrakeActuator
from config import (
    BRAKE_RELEASE_TIME,
    GRAVITY,
    HILL_HOLD_DURATION,
    INSUFFICIENT_MAX_BRAKE_FORCE,
    MAX_BRAKE_FORCE,
    MIN_SLOPE_DEG,
    SENSOR_NOISE_ENABLED,
    SENSOR_NOISE_STD_DEG,
    SENSOR_RANDOM_SEED,
    SIMULATION_DURATION,
    STOP_SPEED_THRESHOLD,
    TIMESTEP,
    VEHICLE_MASS,
)
from sensors import InclineSensor
from simulation import run_phase4_scenario

# ==============================================================================
# Page Configuration
# ==============================================================================
st.set_page_config(
    page_title="SDV - Hill Hold Assist Dashboard",
    page_icon="🚗",
    layout="wide",
    initial_sidebar_state="expanded",
)

# Custom Styling for automotive styling
st.markdown(
    """
    <style>
    .main-title {
        font-size: 2.2rem;
        font-weight: 800;
        color: #1E3A8A;
        margin-bottom: 0.2rem;
    }
    .sub-title {
        font-size: 1.1rem;
        color: #4B5563;
        margin-bottom: 1.5rem;
    }
    .kpi-card {
        background-color: #F8FAFC;
        border-radius: 8px;
        padding: 14px;
        border: 1px solid #E2E8F0;
        text-align: center;
    }
    .kpi-label {
        font-size: 0.82rem;
        font-weight: 600;
        color: #64748B;
        text-transform: uppercase;
        letter-spacing: 0.05em;
    }
    .kpi-value {
        font-size: 1.6rem;
        font-weight: 700;
        color: #0F172A;
        margin-top: 4px;
    }
    .badge-active {
        background-color: #DCFCE7;
        color: #166534;
        padding: 6px 14px;
        border-radius: 20px;
        font-weight: 700;
        display: inline-block;
        font-size: 1.1rem;
        border: 1px solid #86EFAC;
    }
    .badge-inactive {
        background-color: #F1F5F9;
        color: #475569;
        padding: 6px 14px;
        border-radius: 20px;
        font-weight: 700;
        display: inline-block;
        font-size: 1.1rem;
        border: 1px solid #CBD5E1;
    }
    .badge-safe {
        background-color: #DCFCE7;
        color: #166534;
        padding: 6px 14px;
        border-radius: 20px;
        font-weight: 700;
        display: inline-block;
        font-size: 1.1rem;
        border: 1px solid #86EFAC;
    }
    .badge-risk {
        background-color: #FEE2E2;
        color: #991B1B;
        padding: 6px 14px;
        border-radius: 20px;
        font-weight: 700;
        display: inline-block;
        font-size: 1.1rem;
        border: 1px solid #FCA5A5;
        animation: pulse 1.5s infinite;
    }
    </style>
    """,
    unsafe_allow_html=True,
)

# ==============================================================================
# Header Section
# ==============================================================================
st.markdown('<div class="main-title">🚗 Software Defined Vehicle (SDV) — Hill Hold Assist</div>', unsafe_allow_html=True)
st.markdown('<div class="sub-title">Phase 5 Final Demonstration Dashboard | Closed-Loop Incline, Controller, Actuator & Vehicle Simulation</div>', unsafe_allow_html=True)

# ==============================================================================
# Sidebar: Presets and User Configuration
# ==============================================================================
st.sidebar.header("🕹️ Demo Presets")

preset = st.sidebar.radio(
    "Select Quick Demo Scenario:",
    options=[
        "Custom Configuration",
        "Scenario 1: Flat Road (0°, Inactive)",
        "Scenario 2: Normal Hill Hold (5°, 3000N)",
        "Scenario 3: Steeper Hill (8°, 3000N)",
        "Scenario 4: Insufficient Brake Capacity (8°, 1000N - Rollback Risk)",
    ],
    index=2,  # Default to Scenario 2
)

# Initialize defaults based on preset
if preset == "Scenario 1: Flat Road (0°, Inactive)":
    default_slope = 0.0
    default_max_brake = 3000.0
    default_brake_mode = "PRESSED_THEN_RELEASED"
    default_init_v = 0.0
elif preset == "Scenario 2: Normal Hill Hold (5°, 3000N)":
    default_slope = 5.0
    default_max_brake = 3000.0
    default_brake_mode = "PRESSED_THEN_RELEASED"
    default_init_v = 0.0
elif preset == "Scenario 3: Steeper Hill (8°, 3000N)":
    default_slope = 8.0
    default_max_brake = 3000.0
    default_brake_mode = "PRESSED_THEN_RELEASED"
    default_init_v = 0.0
elif preset == "Scenario 4: Insufficient Brake Capacity (8°, 1000N - Rollback Risk)":
    default_slope = 8.0
    default_max_brake = 1000.0
    default_brake_mode = "PRESSED_THEN_RELEASED"
    default_init_v = 0.0
else:
    default_slope = 5.0
    default_max_brake = 3000.0
    default_brake_mode = "PRESSED_THEN_RELEASED"
    default_init_v = 0.0

st.sidebar.markdown("---")
st.sidebar.header("⚙️ Simulation Inputs")

# 1. Incline angle selector
slope_options = [0.0, 5.0, 8.0]
slope_idx = slope_options.index(default_slope) if default_slope in slope_options else 1
selected_slope = st.sidebar.selectbox(
    "Road Incline Angle (θ):",
    options=slope_options,
    index=slope_idx,
    format_func=lambda x: f"{x:.1f}° {'(Flat)' if x == 0 else ('(Gentle)' if x == 5 else '(Steep)')}",
)

# 2. Brake capacity selector
brake_options = [3000.0, 1000.0]
brake_idx = brake_options.index(default_max_brake) if default_max_brake in brake_options else 0
selected_max_brake = st.sidebar.selectbox(
    "Brake Actuator Capacity:",
    options=brake_options,
    index=brake_idx,
    format_func=lambda x: f"{x:.0f} N {'(Normal Capacity)' if x >= 3000 else '(Insufficient Capacity)'}",
)

# 3. Brake pedal behavior
pedal_behavior = st.sidebar.selectbox(
    "Driver Brake Pedal Input:",
    options=["PRESSED_THEN_RELEASED", "NEVER_PRESSED"],
    index=0 if default_brake_mode == "PRESSED_THEN_RELEASED" else 1,
    format_func=lambda x: "Pressed (0.0s-1.0s) → Released at 1.0s" if x == "PRESSED_THEN_RELEASED" else "Never Pressed (Always Released)",
)

# 4. Simulation duration
sim_duration = st.sidebar.slider(
    "Simulation Duration (seconds):",
    min_value=4.0,
    max_value=10.0,
    value=6.0,
    step=0.5,
)

# Execute Simulation via existing simulation/model layer
scenario_config = {
    "id": 99,
    "name": "Live Dashboard Run",
    "description": f"Slope {selected_slope:.1f}°, Max Brake {selected_max_brake:.0f} N",
    "slope_deg": selected_slope,
    "initial_velocity": default_init_v,
    "brake_behavior": pedal_behavior,
    "max_brake_force": selected_max_brake,
    "expected_hold": selected_slope >= MIN_SLOPE_DEG and pedal_behavior == "PRESSED_THEN_RELEASED",
    "expected_risk": selected_max_brake < BrakeActuator.calculate_required_force(VEHICLE_MASS, math.radians(selected_slope)),
}

sensor = InclineSensor(
    noise_std_deg=SENSOR_NOISE_STD_DEG,
    noise_enabled=SENSOR_NOISE_ENABLED,
    seed=SENSOR_RANDOM_SEED,
)

sim_data = run_phase4_scenario(
    scenario=scenario_config,
    sensor=sensor,
    duration=sim_duration,
    dt=TIMESTEP,
)

times = sim_data["time"]
vels = sim_data["velocity"]
poss = sim_data["position"]
accs = sim_data["acceleration"]
brakes = sim_data["brake_force"]
states = sim_data["controller_state"]
holds = sim_data["hold_active"]
pedals = sim_data["brake_pedal"]

# Calculate required force
measured_slope_deg = sim_data["measured_slope_deg"]
req_force = BrakeActuator.calculate_required_force(VEHICLE_MASS, math.radians(measured_slope_deg))
has_risk = sim_data["rollback_risk"]

# ==============================================================================
# Interactive Timeline Inspection Slider
# ==============================================================================
st.markdown("### ⏱️ Timeline Inspector")
inspect_time = st.slider(
    "Scrub through simulation timeline to inspect instantaneous state:",
    min_value=0.0,
    max_value=float(sim_duration),
    value=2.5,
    step=TIMESTEP,
    format="%.2f s",
)

# Find closest sample index
sample_idx = int(round(inspect_time / TIMESTEP))
sample_idx = min(sample_idx, len(times) - 1)

cur_t = times[sample_idx]
cur_v = vels[sample_idx]
cur_x = poss[sample_idx]
cur_a = accs[sample_idx]
cur_f = brakes[sample_idx]
cur_state = states[sample_idx]
cur_hold = holds[sample_idx]
cur_pedal = pedals[sample_idx]

# Calculate hold timer elapsed at current time
if cur_t < BRAKE_RELEASE_TIME:
    hold_elapsed = 0.0
elif cur_hold:
    hold_elapsed = min(cur_t - BRAKE_RELEASE_TIME, HILL_HOLD_DURATION)
elif cur_t >= (BRAKE_RELEASE_TIME + HILL_HOLD_DURATION):
    hold_elapsed = HILL_HOLD_DURATION
else:
    hold_elapsed = 0.0

# ==============================================================================
# Main KPI Status Section (10 Core Signals)
# ==============================================================================
st.markdown("### 📊 Instantaneous Vehicle & Control State")

col1, col2, col3, col4, col5 = st.columns(5)

with col1:
    st.markdown(
        f"""
        <div class="kpi-card">
            <div class="kpi-label">1. Incline Angle</div>
            <div class="kpi-value">{selected_slope:.1f}°</div>
            <div style="font-size: 0.75rem; color: #64748B;">Sensor: {measured_slope_deg:.2f}°</div>
        </div>
        """,
        unsafe_allow_html=True,
    )

with col2:
    st.markdown(
        f"""
        <div class="kpi-card">
            <div class="kpi-label">2. Vehicle Speed</div>
            <div class="kpi-value">{cur_v:+.2f} m/s</div>
            <div style="font-size: 0.75rem; color: #64748B;">Accel: {cur_a:+.2f} m/s²</div>
        </div>
        """,
        unsafe_allow_html=True,
    )

with col3:
    st.markdown(
        f"""
        <div class="kpi-card">
            <div class="kpi-label">3. Position (Displacement)</div>
            <div class="kpi-value">{cur_x:+.2f} m</div>
            <div style="font-size: 0.75rem; color: #64748B;">{'Rollback' if cur_x < -0.05 else ('Stationary' if abs(cur_x) <= 0.05 else 'Forward')}</div>
        </div>
        """,
        unsafe_allow_html=True,
    )

with col4:
    pedal_color = "#DC2626" if cur_pedal == "PRESSED" else "#16A34A"
    st.markdown(
        f"""
        <div class="kpi-card">
            <div class="kpi-label">4. Brake Pedal State</div>
            <div class="kpi-value" style="color: {pedal_color};">{cur_pedal}</div>
            <div style="font-size: 0.75rem; color: #64748B;">Driver Input Signal</div>
        </div>
        """,
        unsafe_allow_html=True,
    )

with col5:
    st.markdown(
        f"""
        <div class="kpi-card">
            <div class="kpi-label">5. Required Brake Force</div>
            <div class="kpi-value">{req_force:.0f} N</div>
            <div style="font-size: 0.75rem; color: #64748B;">m × g × sin(θ)</div>
        </div>
        """,
        unsafe_allow_html=True,
    )

st.markdown("<div style='height: 12px;'></div>", unsafe_allow_html=True)

col6, col7, col8, col9, col10 = st.columns(5)

with col6:
    st.markdown(
        f"""
        <div class="kpi-card">
            <div class="kpi-label">6. Max Brake Capacity</div>
            <div class="kpi-value">{selected_max_brake:.0f} N</div>
            <div style="font-size: 0.75rem; color: #64748B;">Actuator Limit</div>
        </div>
        """,
        unsafe_allow_html=True,
    )

with col7:
    st.markdown(
        f"""
        <div class="kpi-card">
            <div class="kpi-label">7. Actual Brake Force</div>
            <div class="kpi-value">{cur_f:.0f} N</div>
            <div style="font-size: 0.75rem; color: #64748B;">Software Clamped</div>
        </div>
        """,
        unsafe_allow_html=True,
    )

with col8:
    hha_badge = '<span class="badge-active">ACTIVE</span>' if cur_hold else '<span class="badge-inactive">INACTIVE</span>'
    st.markdown(
        f"""
        <div class="kpi-card">
            <div class="kpi-label">8. Hill Hold State</div>
            <div style="margin-top: 6px;">{hha_badge}</div>
            <div style="font-size: 0.75rem; color: #64748B; margin-top: 4px;">FSM: {cur_state}</div>
        </div>
        """,
        unsafe_allow_html=True,
    )

with col9:
    st.markdown(
        f"""
        <div class="kpi-card">
            <div class="kpi-label">9. Hold Timer</div>
            <div class="kpi-value">{hold_elapsed:.1f} / {HILL_HOLD_DURATION:.1f} s</div>
            <div style="font-size: 0.75rem; color: #64748B;">Rem: {max(0.0, HILL_HOLD_DURATION - hold_elapsed):.1f} s</div>
        </div>
        """,
        unsafe_allow_html=True,
    )

with col10:
    if has_risk and cur_hold:
        risk_badge = '<span class="badge-risk">⚠️ ROLLBACK RISK</span>'
        risk_sub = "Brake Force Insufficient!"
    elif has_risk and not cur_hold:
        risk_badge = '<span class="badge-risk">⚠️ CAPACITY DEFICIT</span>'
        risk_sub = "Max < Required"
    else:
        risk_badge = '<span class="badge-safe">✅ SAFE</span>'
        risk_sub = "Capacity Sufficient"

    st.markdown(
        f"""
        <div class="kpi-card">
            <div class="kpi-label">10. Rollback Status</div>
            <div style="margin-top: 6px;">{risk_badge}</div>
            <div style="font-size: 0.75rem; color: #64748B; margin-top: 4px;">{risk_sub}</div>
        </div>
        """,
        unsafe_allow_html=True,
    )

st.markdown("---")

# ==============================================================================
# Simulation Plots Section
# ==============================================================================
st.markdown("### 📈 Engineering Simulation Charts")

# Grid layout: 2 rows of figures
row1_col1, row1_col2 = st.columns(2)

with row1_col1:
    fig1, ax1 = plt.subplots(figsize=(7, 3.8))
    ax1.plot(times, vels, color="#1E40AF", linewidth=2.2, label="Longitudinal Velocity")
    ax1.axvline(BRAKE_RELEASE_TIME, color="gray", linestyle="--", linewidth=1.2, label="Brake Release (1.0s)")
    ax1.axvline(BRAKE_RELEASE_TIME + HILL_HOLD_DURATION, color="black", linestyle=":", linewidth=1.2, label="Hold Expired (4.0s)")
    ax1.axvline(cur_t, color="#DC2626", linestyle="-", linewidth=1.5, label=f"Inspector (t={cur_t:.2f}s)")
    ax1.axhline(0, color="black", linestyle="-", linewidth=0.7, alpha=0.5)
    if selected_slope >= MIN_SLOPE_DEG and pedal_behavior == "PRESSED_THEN_RELEASED":
        ax1.axvspan(BRAKE_RELEASE_TIME, BRAKE_RELEASE_TIME + HILL_HOLD_DURATION, color="#86EFAC", alpha=0.2, label="3s Hold Window")
    ax1.set_title("1. Vehicle Velocity vs Time", fontsize=11, fontweight="bold")
    ax1.set_xlabel("Time (s)", fontsize=9)
    ax1.set_ylabel("Velocity (m/s)\n[(+) Uphill | (-) Downhill]", fontsize=9)
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="lower left", fontsize=7.5)
    plt.tight_layout()
    st.pyplot(fig1)

with row1_col2:
    fig2, ax2 = plt.subplots(figsize=(7, 3.8))
    ax2.plot(times, poss, color="#B91C1C", linewidth=2.2, label="Longitudinal Position")
    ax2.axvline(BRAKE_RELEASE_TIME, color="gray", linestyle="--", linewidth=1.2, label="Brake Release (1.0s)")
    ax2.axvline(BRAKE_RELEASE_TIME + HILL_HOLD_DURATION, color="black", linestyle=":", linewidth=1.2, label="Hold Expired (4.0s)")
    ax2.axvline(cur_t, color="#DC2626", linestyle="-", linewidth=1.5, label=f"Inspector (t={cur_t:.2f}s)")
    ax2.axhline(0, color="black", linestyle="-", linewidth=0.7, alpha=0.5)
    if selected_slope >= MIN_SLOPE_DEG and pedal_behavior == "PRESSED_THEN_RELEASED":
        ax2.axvspan(BRAKE_RELEASE_TIME, BRAKE_RELEASE_TIME + HILL_HOLD_DURATION, color="#86EFAC", alpha=0.2, label="3s Hold Window")
    ax2.set_title("2. Vehicle Position vs Time", fontsize=11, fontweight="bold")
    ax2.set_xlabel("Time (s)", fontsize=9)
    ax2.set_ylabel("Position (m)\n[(+) Uphill | (-) Downhill]", fontsize=9)
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="lower left", fontsize=7.5)
    plt.tight_layout()
    st.pyplot(fig2)

row2_col1, row2_col2 = st.columns(2)

with row2_col1:
    fig3, ax3 = plt.subplots(figsize=(7, 3.8))
    ax3.plot(times, accs, color="#7C3AED", linewidth=2.2, label="Longitudinal Accel")
    ax3.axvline(BRAKE_RELEASE_TIME, color="gray", linestyle="--", linewidth=1.2, label="Brake Release (1.0s)")
    ax3.axvline(BRAKE_RELEASE_TIME + HILL_HOLD_DURATION, color="black", linestyle=":", linewidth=1.2, label="Hold Expired (4.0s)")
    ax3.axvline(cur_t, color="#DC2626", linestyle="-", linewidth=1.5, label=f"Inspector (t={cur_t:.2f}s)")
    ax3.axhline(0, color="black", linestyle="-", linewidth=0.7, alpha=0.5)
    if selected_slope >= MIN_SLOPE_DEG and pedal_behavior == "PRESSED_THEN_RELEASED":
        ax3.axvspan(BRAKE_RELEASE_TIME, BRAKE_RELEASE_TIME + HILL_HOLD_DURATION, color="#86EFAC", alpha=0.2, label="3s Hold Window")
    ax3.set_title("3. Acceleration vs Time", fontsize=11, fontweight="bold")
    ax3.set_xlabel("Time (s)", fontsize=9)
    ax3.set_ylabel("Acceleration (m/s²)", fontsize=9)
    ax3.grid(True, linestyle=":", alpha=0.6)
    ax3.legend(loc="lower left", fontsize=7.5)
    plt.tight_layout()
    st.pyplot(fig3)

with row2_col2:
    fig4, ax4 = plt.subplots(figsize=(7, 3.8))
    ax4.plot(times, brakes, color="#059669", linewidth=2.2, label="Actual Brake Force")
    ax4.axhline(selected_max_brake, color="#DC2626", linestyle="--", linewidth=1.2, label=f"Max Capacity ({selected_max_brake:.0f}N)")
    ax4.axhline(req_force, color="#D97706", linestyle=":", linewidth=1.4, label=f"Required Force ({req_force:.0f}N)")
    ax4.axvline(BRAKE_RELEASE_TIME, color="gray", linestyle="--", linewidth=1.2, label="Brake Release (1.0s)")
    ax4.axvline(BRAKE_RELEASE_TIME + HILL_HOLD_DURATION, color="black", linestyle=":", linewidth=1.2, label="Hold Expired (4.0s)")
    ax4.axvline(cur_t, color="#DC2626", linestyle="-", linewidth=1.5, label=f"Inspector (t={cur_t:.2f}s)")
    if selected_slope >= MIN_SLOPE_DEG and pedal_behavior == "PRESSED_THEN_RELEASED":
        ax4.axvspan(BRAKE_RELEASE_TIME, BRAKE_RELEASE_TIME + HILL_HOLD_DURATION, color="#86EFAC", alpha=0.2, label="3s Hold Window")
    ax4.set_title("4. Brake Force vs Time & Capacity", fontsize=11, fontweight="bold")
    ax4.set_xlabel("Time (s)", fontsize=9)
    ax4.set_ylabel("Brake Force (N) [Uphill]", fontsize=9)
    ax4.grid(True, linestyle=":", alpha=0.6)
    ax4.legend(loc="upper right", fontsize=7.5)
    plt.tight_layout()
    st.pyplot(fig4)

# ==============================================================================
# Chart 5: Force Balance Comparison Chart
# ==============================================================================
st.markdown("### ⚖️ Force Balance & Actuator Capacity Comparison")
fig5, ax5 = plt.subplots(figsize=(12, 2.8))

labels = ["Downhill Gravity (|F_g|)", "Required Brake (F_req)", "Max Available Capacity (F_max)", "Currently Applied (F_brake)"]
f_gravity_mag = abs(VEHICLE_MASS * GRAVITY * math.sin(math.radians(selected_slope)))
values = [f_gravity_mag, req_force, selected_max_brake, cur_f]
bar_colors = ["#475569", "#D97706", "#2563EB" if selected_max_brake >= req_force else "#DC2626", "#059669"]

bars = ax5.barh(labels, values, color=bar_colors, height=0.55)
ax5.set_xlabel("Force Magnitude (Newtons)", fontsize=10)
ax5.set_title(f"Force Comparison at Current Inspector Time t = {cur_t:.2f}s", fontsize=11, fontweight="bold")
ax5.grid(True, linestyle=":", alpha=0.6, axis="x")

for bar in bars:
    width = bar.get_width()
    ax5.text(width + 25, bar.get_y() + bar.get_height() / 2, f"{width:.0f} N", ha="left", va="center", fontsize=9, fontweight="bold")

ax5.set_xlim(0, max(values + [3200]) * 1.15)
plt.tight_layout()
st.pyplot(fig5)

# ==============================================================================
# Engineering Explanation Section
# ==============================================================================
st.markdown("---")
st.markdown("### 🎓 Engineering Principles: How Hill Hold Assist Works")

exp_col1, exp_col2 = st.columns(2)

with exp_col1:
    st.markdown(
        """
        #### 1. Closed-Loop Execution Sequence:
        1. **Incline Measurement:** The simulated Incline Sensor (Inclinometer / IMU) samples road slope angle $\\theta$ with Gaussian noise $\\mathcal{N}(0, \\sigma^2)$.
        2. **Condition Checks:** The controller checks if vehicle speed is below stop threshold ($|v| \\le 0.1\\text{ m/s}$) and slope exceeds minimum grade ($\\theta \\ge 3.0^\\circ$).
        3. **Driver Engagement:** The driver actively holds the brake pedal down (`PRESSED`), priming the controller into the `READY` state.
        4. **Transition Detection:** The driver lifts their foot off the pedal. The controller detects the edge transition **`PRESSED` $\\rightarrow$ `RELEASED`**.
        5. **Hold Activation:** The controller transitions to `HOLD_ACTIVE` and starts the **3.0-second hold timer**.
        6. **Actuator Counteraction:** The software brake actuator clamps with required force $F_{\\text{req}} = m \\cdot g \\cdot \\sin(\\theta)$ to balance gravity.
        7. **Stationary Holding:** The static holding constraint ensures $v \\approx 0\\text{ m/s}$ and $x = \\text{constant}$ without propelling the car uphill.
        8. **Timer Expiration:** After $3.0$ seconds, the controller transitions to `RELEASED`, the actuator drops brake pressure to $0\\text{ N}$, and the car rolls downhill under gravity.
        """
    )

with exp_col2:
    st.markdown(
        """
        #### 2. Physical Force Balance Principle:
        $$\\text{Required Brake Force: } F_{\\text{req}} = m \\times g \\times \\sin(\\theta)$$
        
        Where:
        - $m = 1500\\text{ kg}$ (Vehicle Mass)
        - $g = 9.81\\text{ m/s}^2$ (Gravitational Acceleration)
        - $\\theta$ = Road Incline Angle
        
        #### 3. Safety Criterion & Rollback Risk:
        - **Safe Condition:**
          $$\\text{Available Brake Force } (F_{\\text{max}}) \\ge \\text{Required Brake Force } (F_{\\text{req}})$$
          The brake has adequate clamping capacity to hold the vehicle stationary at standstill.
        - **Rollback Risk Condition:**
          $$\\text{Available Brake Force } (F_{\\text{max}}) < \\text{Required Brake Force } (F_{\\text{req}})$$
          The brake actuator cannot produce sufficient torque to balance gravity. A **`ROLLBACK RISK`** warning is raised, and the vehicle begins rolling downhill even while Hill Hold is commanded.
        """
    )

st.markdown("---")
st.caption("College Software Defined Vehicle (SDV) Project | Phase 5 Final Presentation Dashboard | Architecture: InclineSensor → BrakePedal → HillHoldController → BrakeActuator → Vehicle")
