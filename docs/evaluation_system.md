# SAMTRUCK Real-Data Evaluation System Documentation

This document describes the design, data schemas, mathematical formulas, command-line interface, and results structure of the SAMTRUCK evaluation system.

---

## 1. Created Files & Architecture

The evaluation system adds a non-destructive logging, metrics calculation, plotting, reporting, and comparison suite:

```
E:\SAMSUNGTRUCK
├── evaluation/
│   ├── __init__.py        # Evaluation package initializer
│   ├── metrics.py         # Real-data statistics, jerk, recovery time, string stability & PASS/FAIL evaluator
│   ├── plotter.py         # Headless Matplotlib graph generator for base & conditional plots
│   └── report.py          # HTML report generator (report.html)
├── run_all_scenarios.py   # Standalone TraCI evaluation scenario runner CLI
├── generate_comparison.py # Baseline vs AI-Digital Twin comparison framework
└── docs/
    └── evaluation_system.md # System documentation (this file)
```

---

## 2. Output Data Schemas

All results are written to `E:\SAMSUNGTRUCK\results\<scenario_name>\`.

### `telemetry.csv`
Logged at every SUMO simulation step ($0.1\text{ s}$) for all three platoon trucks (`samtruck_leader`, `samtruck_follower_1`, `samtruck_follower_2`):

| Column Header | Type | Description |
|---|---|---|
| `simulation_time_s` | float | Current simulation time in seconds |
| `vehicle_id` | string | Vehicle ID (`samtruck_leader`, `samtruck_follower_1`, `samtruck_follower_2`) |
| `x_m` | float | 2D X coordinate in meters |
| `y_m` | float | 2D Y coordinate in meters |
| `lane_id` | string | Current SUMO lane ID |
| `lane_position_m` | float | Position along current lane in meters |
| `longitudinal_position_m` | float | Cumulative 2D distance travelled since spawn |
| `speed_mps` | float | Vehicle forward speed in m/s |
| `acceleration_mps2` | float | Vehicle acceleration in m/s² |
| `leader_id` | string | Sensed leader vehicle ID from `traci.vehicle.getLeader` |
| `predecessor_id` | string | Immediate platoon predecessor ID |
| `follower_id` | string | Immediate platoon follower ID |
| `actual_gap_m` | float | Sensed bumper-to-bumper gap in meters |
| `desired_gap_m` | float | Controller target CTG gap ($4.0 + 1.2 \cdot v$) |
| `gap_error_m` | float | $\text{actual\_gap} - \text{desired\_gap}$ in meters |
| `resilience_state` | string | `CACC`, `DEGRADED_CACC`, `ACC_FALLBACK`, or `EMERGENCY_BRAKE` |
| `controller_reason` | string | Reason string explaining controller decision |
| `v2v_message_id` | string | ID of last received V2V message |
| `v2v_age_ms` | float | Age of last received V2V message in milliseconds |

### `messages.csv`
Logged for every V2V message attempt:

| Column Header | Type | Description |
|---|---|---|
| `message_id` | string | Unique message identifier |
| `sender` | string | Sender vehicle ID |
| `receiver` | string | Receiver vehicle ID |
| `generation_time_s` | float | Timestamp when message was transmitted |
| `delivery_time_s` | float/blank | Timestamp when message arrived (blank if dropped) |
| `latency_ms` | float/blank | Network latency in milliseconds (blank if dropped) |
| `status` | string | `received` or `dropped` |

### `events.json`
Recorded scenario events:
```json
[
  {
    "time_s": 20.0,
    "event": "emergency_brake",
    "duration_s": 4.0,
    "applied": true,
    "detail": "Emergency braking applied in execution loop"
  }
]
```

---

## 3. Metric Formulas & Calculations

### A. Jerk ($m/s^3$)
Jerk represents the rate of change of acceleration per vehicle:
$$\text{jerk}_t = \frac{a_t - a_{t-\Delta t}}{\Delta t}$$

### B. Recovery Time ($s$)
For applied disturbances, recovery time is calculated from the end of the last applied disturbance ($t_{\text{end}}$) until **both followers** maintain an absolute gap error $\le 1.0\text{ m}$ continuously for at least $1.0\text{ s}$:
$$\text{Recovery Time} = t_{\text{stable}} - t_{\text{end}}$$
If this condition is not met before the scenario ends, `recovery_time_s` is recorded as `null`.

### C. Quantitative String Stability ($D_v$ and $R_v$)
Disturbance propagation $D_v$ for a signal $v(t)$ (speed or acceleration) relative to pre-event baseline:
$$D_v = \max_{t \ge t_{\text{event}}} |v(t) - \text{median}(v(t < t_{\text{event}}))|$$

Speed propagation ratio $R_v$ for followers:
$$R_v = \frac{D_v^{(\text{follower})}}{D_v^{(\text{leader})}}$$
If leader disturbance $D_v^{(\text{leader})} == 0$, $R_v$ is recorded as `null`.

### D. PASS / FAIL Criteria
A scenario is marked **PASS** if and only if:
1. Telemetry is present for all three platoon trucks (`samtruck_leader`, `samtruck_follower_1`, `samtruck_follower_2`).
2. Every expected scenario event in the catalog specification was applied (`applied == true`).
3. `collision_count == 0`.

Otherwise, the scenario is marked **FAIL**.

---

## 4. Command Line Interface

Runs are executed using `run_all_scenarios.py`:

```powershell
# Run a specific scenario in headless mode
python run_all_scenarios.py --scenario emergency_braking

# Run with GUI visualization
python run_all_scenarios.py --scenario emergency_braking --gui

# Run all 20 catalog scenarios sequentially and print summary
python run_all_scenarios.py --scenario all

# Override scenario duration (e.g. 30 seconds)
python run_all_scenarios.py --scenario normal_platooning --duration 30

# List available scenarios
python run_all_scenarios.py --list

# Run bounded random journey
python run_all_scenarios.py --scenario random_journey --duration 60
```

---

## 5. Known Limitations

1. **Fuel / Energy Model**: SUMO fuel consumption models are not configured for `platoon_truck`. The metric field returns:
   ```json
   "fuel_energy": {
     "status": "unavailable",
     "reason": "No SUMO fuel or energy model is configured for platoon_truck."
   }
   ```
2. **PASS Status Scope**: The `PASS` status represents execution and event validation only (verifying telemetry logging, event application, and collision absence). It is not a formal safety-certification proof.
3. **Master / Random Journey**: `--scenario random_journey` requires an explicit `--duration` argument because the master journey has no natural end.
