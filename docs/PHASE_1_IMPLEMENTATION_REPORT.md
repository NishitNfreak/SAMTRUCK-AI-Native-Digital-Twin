# SAMTRUCK Phase 1 Digital Twin Core Report

## Guide Compatibility and Mapping

The revised Digital Twin Implementation Guide was read before implementation. The relevant sections are Phase 1 goal, Step 1.1 (Twin core), Step 1.2 (unit tests), Step 1.3 (parallel integration), and its strict data-boundary guidance.

| Guide intent | SAMTRUCK city-project implementation |
| --- | --- |
| `PlatoonTwin` history/gap/relative-speed/message-age core | New `samtruck\twin_runtime.py` `PlatoonTwin`. |
| Extend `digital_twin.py` | Existing `VehicleState` remains compatible; new `EstimatedVehicleState` represents measurement-derived state. |
| SensorInterface pattern | New `samtruck\sensors.py` typed DTOs and seeded `SensorInterface`. |
| Integrate in scenario loop | `run_all_scenarios.py`, the active 20-scenario evaluation runner, updates the Twin after each SUMO step. |
| Separate truth and Twin logs | `twin_ground_truth.csv` and `twin_estimates.csv` inside every scenario results directory. |

**Guide conflict resolved:** the guide demonstrates direct TraCI-to-Twin examples and names `scenario_suite.py` as the integration point. The mandated strict boundary supersedes that example. This project uses `run_all_scenarios.py` as the active evaluation runner, so it owns the sole scalar TraCI-to-sensor conversion boundary. The Twin itself never imports TraCI/SUMOlib or accepts legacy raw `VehicleState` objects.

## Files Created

- `samtruck\sensors.py`: `RadarMeasurement`, `OdometryMeasurement`, `VehicleSensorInput`, seeded `SensorInterface`.
- `samtruck\twin_runtime.py`: measurement-only rolling-history `PlatoonTwin`.
- `tests\test_sensors.py`: deterministic seeded noise/dropout tests.
- `tests\test_ground_truth_boundary.py`: simulator-import and raw-state rejection tests.
- `docs\PHASE_0_PROJECT_AUDIT.md`, `docs\RESEARCH_DECISIONS.md`, and this report.

## Files Modified

- `samtruck\digital_twin.py`: `EstimatedVehicleState`; original `VehicleState` and `time_gap()` preserved.
- `samtruck\config.py`: Phase 1 Twin/sensor configuration defaults.
- `samtruck\v2x.py`: optional receiver/sequence metadata with backward-compatible defaults.
- `run_all_scenarios.py`: passive Twin input/logging branch only.
- `tests\test_digital_twin.py`: history, relative-speed, stale-message, and missing-radar tests.

## Data Flow

```text
SUMO/TraCI
  ├─ run_all_scenarios simulator boundary -> scalar observations -> SensorInterface DTOs -> PlatoonTwin
  ├─ inline V2X delivery logic -> delivered V2XMessage DTOs --------------------^ 
  └─ independent truth rows -> results/<scenario>/twin_ground_truth.csv

Existing direct CACC inputs -> unchanged CACCController.command() -> TraCI vehicle commands
PlatoonTwin estimates -> results/<scenario>/twin_estimates.csv only
```

The Twin uses odometry and radar DTOs only. Missing radar is represented by `None` plus `radar_missing=True`, not a zero gap. A V2X message is missing when no predecessor message was delivered and stale when its age exceeds the configured limit. Phase 1 does not estimate/extrapolate missing data.

## CACC Preservation

`samtruck\controller.py` was not modified. `run_all_scenarios.py` keeps the existing controller construction, call arguments, message-age rules, event handling, and TraCI speed commands unchanged. `twin.update()` is called after the existing per-step controller processing, and no Twin estimate is sent into CACC.

## Verification Performed

Commands run in `E:\SAMSUNGTRUCK`:

```powershell
python -m compileall -q samtruck run_all_scenarios.py
python -m unittest discover -s tests -v
python run_all_scenarios.py --scenario emergency_braking --fast-demo
python run_all_scenarios.py --scenario v2x_latency_packet_loss --fast-demo
python run_all_scenarios.py --scenario platoon_merge --fast-demo
```

Results:

| Check | Actual result |
| --- | --- |
| Complete unit suite | PASS: 15/15 tests |
| Emergency braking run | PASS: evaluation runner completed and wrote reports/logs |
| Combined latency/packet-loss run | PASS: evaluation runner completed and wrote reports/logs |
| Catalog merge run (`platoon_merge`) | PASS: completed; 750 Twin rows and 750 truth rows |
| Emergency Twin/truth alignment | PASS: 750 rows in each stream, all three platoon IDs |
| V2X Twin/truth alignment | PASS: 1050 rows in each stream, all three platoon IDs |
| Packet-loss evidence | PASS: `messages.csv` contains both `received` and `dropped` rows |
| Twin simulator imports | PASS: no TraCI/SUMOlib import in Twin modules; unit test enforces it |
| Raw-state leakage | PASS: Twin rejects legacy `VehicleState` at runtime |

## Known Limitations and Next Step

- This Twin is a transparent measurement-history core, not an estimator, Kalman filter, predictor, AI model, or adaptive controller.
- The pre-existing CACC path still reads live TraCI radar/predecessor data; this was intentionally preserved because Phase 1 requires baseline behavior to remain unchanged. That direct path does not enter the Twin.
- Sensor configuration is available through an optional in-memory `spec["twin"]` block; catalog defaults remain zero noise/dropout for reproducible baseline behavior.
- Consolidating the legacy `SimulationConfig` CTG defaults with the active city controller is a later compatibility task, not a Phase 1 controller change.

Recommended next phase: implement V2X message validation and bounded stale-data estimation inside `PlatoonTwin`, retaining the same measurement-only boundary and unchanged baseline CACC.
