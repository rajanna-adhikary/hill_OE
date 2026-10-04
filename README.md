# Hill Hold Assist (HHA) — Software Simulation

---

## 1. Project in One Paragraph 

A 1500 kg virtual vehicle is stopped on an inclined road. Gravity tries to pull it downhill. A simulated incline sensor measures the road angle. The driver presses the brake pedal, then releases it. The Hill Hold controller detects this — vehicle is stopped, slope is sufficient, brake was pressed then released — and activates automatic braking. The brake actuator generates exactly the force needed to cancel gravity, so the vehicle stays stationary for 3 seconds. After 3 seconds, the brake releases automatically and the vehicle rolls downhill. If the actuator cannot produce enough force, a **ROLLBACK RISK** is flagged and the vehicle rolls backward even during the hold attempt.

**This is a Python software simulation. It is NOT a real ECU, NOT connected to physical sensors, and NOT running on real vehicle hardware.**

---

## 2. Real-World Problem

A car stops on an uphill road at a traffic signal. The driver needs to move their foot from the brake pedal to the accelerator. During that brief moment — maybe 0.5 to 1 second — there is no braking force. Gravity pulls the car backward. This is called **rollback**.

Hill Hold Assist solves this by automatically holding the brake for a few seconds after the driver releases it, giving them time to press the accelerator without the car rolling back.

---

## 3. Basic Vehicle Physics

Gravity acts vertically downward. On an inclined road, only the component **along the road surface** causes the vehicle to move.

```
F_gravity = -m × g × sin(θ)
```

| Symbol | Value | Meaning |
|--------|-------|---------|
| m | 1500 kg | Vehicle mass |
| g | 9.81 m/s² | Gravitational acceleration |
| θ | Road incline angle | In radians |
| − sign | Downhill direction | Negative = downhill |

**Sign convention:** Uphill = positive (+), Downhill = negative (−)

**To hold the vehicle stationary, the brake must apply an equal and opposite uphill force:**

```
F_required = m × g × sin(θ)
```

**Worked examples:**

| Slope | sin(θ) | F_required |
|-------|--------|------------|
| 5° | 0.0872 | 1500 × 9.81 × 0.0872 ≈ **1278 N** |
| 8° | 0.1392 | 1500 × 9.81 × 0.1392 ≈ **2048 N** |

**Why sin(θ)?** Because sin(θ) gives the component of gravity *along* the road surface. cos(θ) gives the component *perpendicular* to the road, which is balanced by the road's normal reaction and causes no longitudinal motion.

**Steeper slope → larger sin(θ) → larger downhill force → more brake force required.**

---

## 4. How the Complete System Works

```
ROAD SLOPE (ground truth)
        ↓
INCLINE SENSOR  →  measures slope angle (adds small noise)
        ↓
HILL HOLD CONTROLLER  →  decides when to activate/deactivate hold
        ↓
BRAKE ACTUATOR  →  applies required uphill force (clamped to max capacity)
        ↓
VEHICLE  →  physics update: F_net → acceleration → velocity → position
        ↓
SIMULATION DATA  →  displayed on Streamlit dashboard
```

- **Incline Sensor:** Reads slope angle with small Gaussian noise (±0.1°). Simulates a real IMU.
- **Hill Hold Controller:** A 4-state finite state machine. Decides when conditions are right to hold.
- **Brake Actuator:** Applies the calculated force. Cannot exceed its maximum capacity.
- **Vehicle:** Updates speed and position every 0.01 seconds using Newton's second law.

---

## 5. Hill Hold Controller — Four States

```
        IDLE
          ↓  vehicle stopped (speed ≤ 0.1 m/s)
          ↓  slope ≥ 3°
          ↓  brake PRESSED
        READY
          ↓  driver releases brake (PRESSED → RELEASED transition)
      HOLD_ACTIVE  ←── brake force applied, timer running
          ↓  3 seconds elapsed
        RELEASED
          ↓  (next step)
        IDLE
```

**Why must the vehicle be stopped?**
Hill Hold is for a stationary vehicle about to move uphill. If the vehicle is already rolling, applying sudden brake force would be dangerous and abrupt.

**Why must the brake have been pressed first?**
The controller looks for the PRESSED→RELEASED *transition*, not just the RELEASED state. This prevents false activation if the driver never touched the brake at all.

**Why slope ≥ 3°?**
On a nearly flat road, rollback risk is negligible. The 3° threshold avoids unnecessary activation on flat roads or very gentle inclines.

**Why RELEASED state before IDLE?**
It gives the simulation one clean step to release the actuator before resetting, ensuring the brake force drops to zero before the next cycle begins.

---

## 6. Force Balance During Hold

When HOLD_ACTIVE and the actuator applies F_required:

```
F_gravity  = −1278 N   (downhill, 5° slope)
F_brake    = +1278 N   (uphill, applied by actuator)
─────────────────────
F_net      =     0 N

a = F_net / m = 0 / 1500 = 0 m/s²
→ velocity stays 0
→ position stays 0
→ vehicle is stationary
```

The code in `vehicle.py` enforces a **holding constraint**:
```
effective_brake = min(brake_force, abs(F_gravity))
```
This ensures the brake can cancel gravity but **cannot push the vehicle uphill**.

---

## 7. Rollback Risk

**Setup:** 8° slope, actuator maximum = 1000 N

```
F_required = 1500 × 9.81 × sin(8°) = 2048 N
F_available = 1000 N

1000 < 2048  →  ROLLBACK RISK
```

**What happens during hold attempt:**
```
F_gravity  = −2048 N
F_brake    = +1000 N   (clamped at max)
─────────────────────
F_net      = −1048 N   (still downhill!)

a = −1048 / 1500 = −0.699 m/s²
→ vehicle accelerates backward even while Hill Hold is commanded
```

> **Key viva point:** Rollback risk is an actuator-capacity problem, not a controller failure. The FSM and actuator work correctly — the actuator simply does not have enough physical capacity to counteract gravity on a steep slope.

---

## 8. The 3-Second Hold Timer

```
Driver releases brake
        ↓
FSM → HOLD_ACTIVE, timer = 0.0 s
        ↓
Every 0.01 s: timer += 0.01
Brake force maintained at F_required
Vehicle stays stationary
        ↓
timer reaches 3.0 s
        ↓
FSM → RELEASED → IDLE
Actuator releases → brake force = 0 N
        ↓
F_net = F_gravity (full gravity, no brake)
Vehicle accelerates downhill
```

The timer is purely software — it counts simulation timesteps, not real wall-clock time.

---

## 9. Software Components

| File | Job |
|------|-----|
| `config.py` | All constants: mass, gravity, thresholds, durations |
| `vehicle.py` | 1D physics: F_gravity, acceleration, velocity, position |
| `sensors.py` | Simulated incline sensor with Gaussian noise |
| `inputs.py` | Brake pedal signal: PRESSED or RELEASED |
| `hill_hold_controller.py` | FSM: IDLE → READY → HOLD_ACTIVE → RELEASED |
| `brake_actuator.py` | Applies/releases brake force, checks rollback risk |
| `simulation.py` | Runs the full scenario, returns time-series data |
| `dashboard.py` | Streamlit web dashboard for visualization |

---

## 10. Dashboard

Run with:
```bash
python -m streamlit run dashboard.py
```

**What it shows:**

| KPI Card | What it means |
|----------|---------------|
| Incline Angle | Road slope + sensor reading |
| Vehicle Speed | Current velocity (0 during hold, negative after release) |
| Position | Displacement (0 during hold, negative after release) |
| Brake Pedal | PRESSED or RELEASED |
| Required Brake Force | m × g × sin(θ) |
| Max Brake Capacity | Actuator limit |
| Actual Brake Force | Force currently applied |
| Hill Hold State | ACTIVE / INACTIVE + FSM state |
| Hold Timer | Elapsed / 3.0 s |
| Rollback Status | ✅ SAFE or ⚠️ ROLLBACK RISK |

**During demo:** Use the timeline slider to scrub to t = 2.5 s (during hold) and show speed = 0, position = 0, brake force = F_required. Then scrub to t = 5.0 s and show the vehicle rolling.

---

## 11. Three Demo Scenarios

| Scenario | Slope | Max Brake | Required Force | Result |
|----------|-------|-----------|----------------|--------|
| Normal Hill Hold | 5° | 3000 N | ~1278 N | ✅ SAFE — vehicle holds 3 s, then rolls |
| Steeper Hill | 8° | 3000 N | ~2048 N | ✅ SAFE — more force needed, still sufficient |
| Rollback Risk | 8° | 1000 N | ~2048 N | ⚠️ ROLLBACK RISK — rolls even during hold |

---

## 12. Testing

| Test Suite | Tests | Result | What it validates |
|------------|-------|--------|-------------------|
| test_phase2.py | 3/3 | ✅ PASS | Sensor noise, brake pedal, physics accuracy |
| test_phase3.py | 7/7 | ✅ PASS | All FSM state transitions |
| test_phase4.py | 12/12 | ✅ PASS | Actuator, force balance, timer, rollback risk |
| **Total** | **22/22** | **✅ ALL PASS** | |

---

## 13. Limitations

- Software simulation only — no real vehicle hardware
- 1D model — no lateral, yaw, or suspension dynamics
- No real ECU or embedded microcontroller
- No CAN bus communication
- No hydraulic brake pressure dynamics
- No tire slip or road friction model
- Sensor noise is synthetic Gaussian, not real IMU behavior

---

## 14. Viva — 15 Key Questions

**1. What is Hill Hold Assist?**
A safety feature that automatically holds brake pressure for a few seconds after the driver releases the brake on a slope, preventing rollback during hill starts.

**2. Why does a vehicle roll on a slope?**
Gravity has a component along the road surface: F = m × g × sin(θ). With no brake force, this accelerates the vehicle downhill.

**3. What force causes rollback?**
The longitudinal component of gravity: F_gravity = −m × g × sin(θ). Negative means downhill.

**4. Why sin(θ) and not cos(θ)?**
sin(θ) gives the force component *along* the road surface. cos(θ) gives the perpendicular component, which is balanced by the road's normal reaction.

**5. What is the required brake force?**
F_required = m × g × sin(θ). At 5°: ~1278 N. At 8°: ~2048 N.

**6. Why does a steeper slope need more brake force?**
Because sin(θ) increases with angle. Steeper slope → larger downhill force → more uphill brake force needed to cancel it.

**7. What happens at 5° with 3000 N capacity?**
Required force ≈ 1278 N, available = 3000 N. Sufficient. Vehicle holds stationary for 3 seconds, then rolls after release.

**8. What happens at 8° with 3000 N capacity?**
Required force ≈ 2048 N, available = 3000 N. Still sufficient. Vehicle holds, then rolls faster than at 5° because acceleration after release is larger.

**9. What happens if brake capacity is insufficient (1000 N at 8°)?**
Net force = −2048 + 1000 = −1048 N. Acceleration = −0.699 m/s². Vehicle rolls backward even while Hill Hold is commanded. ROLLBACK RISK.

**10. Why must the vehicle be stopped before Hill Hold activates?**
Applying sudden brake force to a moving vehicle would be abrupt and dangerous. The system only arms when speed ≤ 0.1 m/s.

**11. Why must the brake have been pressed before release?**
The controller detects the PRESSED→RELEASED *transition*, not just the RELEASED state. This prevents false activation if the driver never touched the brake.

**12. Why is there a 3-second timer?**
To give the driver enough time to move their foot from brake to accelerator and begin moving. After 3 seconds, the system assumes the driver has taken control and releases the brake.

**13. What is rollback risk?**
When the actuator's maximum force is less than the force required to hold the vehicle. The actuator cannot cancel gravity, so the vehicle rolls backward despite Hill Hold being active. It is a capacity problem, not a controller failure.

**14. What is the role of the incline sensor?**
It measures the road slope angle. The controller uses this to check if the slope is above the 3° threshold before arming. Gaussian noise is added to simulate real sensor imperfection.

**15. What are the limitations of your simulation?**
It is a 1D software simulation — no real hardware, no hydraulic dynamics, no tire model, no CAN bus, no real IMU. It demonstrates the logic and physics correctly but cannot replace a real ECU.

---

## 15. 30-Second Explanation

"This project simulates Hill Hold Assist in Python. A 1500 kg vehicle is stopped on a slope. Gravity pulls it downhill with force F = m × g × sin(θ). When the driver releases the brake, the controller detects this and activates automatic braking. The brake actuator applies exactly the force needed to cancel gravity, so the vehicle stays stationary for 3 seconds. After 3 seconds, the brake releases and the vehicle rolls downhill. If the actuator's maximum force is less than what gravity requires — for example, 1000 N on an 8° slope where 2048 N is needed — the vehicle rolls backward even during the hold. That is the rollback risk condition. All of this is verified with 22 passing unit tests."

---

## 16. 2-Minute Explanation

**The problem:** When a car stops on a hill and the driver lifts their foot off the brake to press the accelerator, there is a brief moment with no braking force. Gravity pulls the car backward. Hill Hold Assist prevents this.

**The physics:** On a slope of angle θ, gravity produces a downhill force of m × g × sin(θ). For our 1500 kg vehicle at 5°, that is about 1278 N. To hold the vehicle stationary, the brake must apply exactly 1278 N uphill. At 8°, it needs 2048 N. Steeper slope means larger sin(θ), means more force required.

**The controller:** A finite state machine with four states. It starts IDLE. When the vehicle is stopped, the slope is above 3°, and the driver presses the brake, it moves to READY. When the driver releases the brake, it detects the PRESSED→RELEASED transition and moves to HOLD_ACTIVE. The brake actuator applies the required force. After 3 seconds, it moves to RELEASED and the brake drops to zero.

**The force balance:** During HOLD_ACTIVE, F_gravity + F_brake = 0. Net force is zero. Acceleration is zero. The vehicle stays exactly where it is.

**The failure case:** If the actuator's maximum capacity is less than the required force — say 1000 N when 2048 N is needed — the net force is still −1048 N downhill. The vehicle rolls backward even while Hill Hold is commanded. This is the rollback risk. It is a hardware capacity problem, not a software bug.

**The simulation:** Everything is implemented in Python across 8 modules. The dashboard lets you select scenarios, scrub through the timeline, and see the force balance live. 22 unit tests verify every critical behavior — sensor accuracy, FSM transitions, force calculations, hold timer, and rollback detection.
