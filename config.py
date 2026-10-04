"""
Simulation Configuration Parameters
Hill Hold Assist Project - Phase 1 to Phase 5
"""

# ==========================================
# Physical Constants
# ==========================================
GRAVITY = 9.81  # Gravitational acceleration [m/s^2]


# ==========================================
# Vehicle Parameters
# ==========================================
VEHICLE_MASS = 1500.0  # Vehicle mass [kg]

INITIAL_POSITION = 0.0  # Initial position [m]
INITIAL_VELOCITY = 0.0  # Initial velocity [m/s]


# ==========================================
# Simulation Parameters
# ==========================================
TIMESTEP = 0.01  # Simulation timestep [seconds]

# Phase 4 requires enough time to show:
# 3-second Hill Hold + post-release rollback
SIMULATION_DURATION = 6.0


# ==========================================
# Sensor Parameters - Phase 2
# ==========================================
SENSOR_NOISE_STD_DEG = 0.1
SENSOR_NOISE_ENABLED = True
SENSOR_RANDOM_SEED = 42


# ==========================================
# Hill Hold Controller Parameters - Phase 3
# ==========================================

# Minimum slope required for Hill Hold
MIN_SLOPE_DEG = 3.0

# Vehicle is considered stopped below this speed
STOP_SPEED_THRESHOLD = 0.1

# Brake must be pressed before Hill Hold can become ready
BRAKE_RELEASE_TIME = 1.0


# ==========================================
# Brake Actuator Parameters - Phase 4
# ==========================================

# Maximum brake force available in normal condition
MAX_BRAKE_FORCE = 3000.0  # Newtons

# Used to demonstrate insufficient brake capacity
INSUFFICIENT_MAX_BRAKE_FORCE = 1000.0  # Newtons


# ==========================================
# Hill Hold Timing - Phase 4
# ==========================================

# How long Hill Hold maintains brake force
HILL_HOLD_DURATION = 3.0  # seconds


# ==========================================
# Test Slopes
# ==========================================
TEST_SLOPES_DEG = [0.0, 5.0, 8.0]