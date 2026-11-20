# Phase 2.2A.2 — Measurement Reliability and Age Tracking Implementation Report

## 1. Scope

Implemented the approved component-specific own-state measurement freshness design from `PHASE_2_2A_2_MEASUREMENT_AGE_DESIGN.md`.

This implementation is passive and observational only.  It does not change the Kalman transition/process/measurement equations, covariance tuning, sensor simulation, radar tracking, V2X delivery or fusion, CACC, scenarios, SUMO configuration, or offline-evaluator matching/metrics.

## 2. Files changed

| File | Change |
| --- | --- |
| `samtruck/estimation.py` | Added additive freshness fields to `FilteredVehicleState`; `VehicleStateEstimator` now retains independent accepted-measurement timestamps for position, speed, and acceleration and emits component ages/provenance. |
| `samtruck/twin_runtime.py` | Added the new own-state freshness fields to `PlatoonTwin.filtered_as_rows()`.  Lane-local position remains excluded. |
| `run_all_scenarios.py` | Appended the approved columns to `FILTERED_STATE_FIELDS`; simulation loop and evaluation invocation were not changed. |
| `tests/test_estimation.py` | Added deterministic unit tests for initialization, per-component refresh, prediction aging, position freshness, invalid/out-of-order timestamps, same-time updates, and unavailable states. |
| `tests/test_twin_estimator_integration.py` | Added Twin/CSV serialization tests and invalid-odometry freshness assertions. |

## 3. Data model and timestamp semantics

`VehicleStateEstimator` now owns independent last-valid timestamps for `position`, `speed`, and `acceleration`.  A timestamp changes only after the matching Kalman measurement row has been accepted:

- a valid initialization or update row sets that component timestamp to the accepted estimator timestamp;
- prediction does not change any component timestamp;
- a missing component in a partial update continues to age independently;
- a component never accepted as a measurement has a null timestamp and null age;
- `partially_initialized` acceleration remains a broad prior, not a measured acceleration;
- invalid/out-of-order timestamps return the existing estimator timestamp and do not mutate freshness state;
- same-timestamp partial updates refresh only the supplied measurement component.

For every observed component, `measurement_age_s = max(0, accepted_estimator_timestamp - last_measurement_timestamp_s)`.  The implementation does not derive age from radar tracks, V2X message age, SUMO truth, or sensor transport delay.

Component provenance is one of:

- `measurement_updated`
- `prediction_only`
- `not_meaningfully_initialized`
- `invalid_timestamp_no_update`
- `unavailable`

Existing row-level `estimator_status` and `measurement_update_mask` are unchanged and remain separate from the component-level provenance.

## 4. CSV fields added

The following fields are appended to `results/<scenario>/twin_filtered_states.csv` without removing or renaming an existing field:

```
last_independent_position_measurement_timestamp_s
independent_position_measurement_age_s
independent_position_measurement_ever_received
last_speed_measurement_timestamp_s
speed_measurement_age_s
speed_measurement_ever_received
last_acceleration_measurement_timestamp_s
acceleration_measurement_age_s
acceleration_measurement_ever_received
position_estimate_provenance
speed_estimate_provenance
acceleration_estimate_provenance
```

The current runner intentionally never supplies an independent continuous position measurement.  Its independent-position timestamp and age are therefore empty/null and its availability is false.  Existing predecessor V2X CSV fields remain unchanged.

## 5. Tests executed

All commands were executed from `E:\SAMSUNGTRUCK`.

| Command | Result |
| --- | --- |
| `python -m unittest tests.test_estimation -v` | PASS — 27 tests |
| `python -m unittest tests.test_twin_estimator_integration -v` | PASS — 13 tests |
| `python -m unittest discover -s tests -v` | PASS — 71 tests |

The new coverage verifies full and speed-only initialization, speed-only and acceleration-only updates, repeated prediction-only intervals, independent position metadata, never-measured metadata, invalid and out-of-order timestamps, same-timestamp partial updates, invalid odometry, lane-local position exclusion, and filtered CSV serialization while retaining existing V2X fields.

## 6. Official benchmark runner smoke test

The runner CLI accepts a named scenario argument; the supported equivalent of the requested positional invocation was used:

```
python run_all_scenarios.py --scenario normal_platooning --fast-demo --seed 20260929
```

Result: **PASS** — `normal_platooning` completed successfully.

Verified output: `results/normal_platooning/twin_filtered_states.csv` contains 750 rows and all appended fields.  In the fixed-seed run, the first leader record reports speed and acceleration age `0.0`, both with `measurement_updated` provenance; independent-position timestamp and age are empty.  The existing predecessor V2X fields are present.  `results/normal_platooning/twin_filtered_evaluation.json` was generated with schema version `2.2A.1` and 750 matched truth/filtered records, confirming the evaluation remains post-run and compatible.

## 7. Preservation confirmation

- CACC controller, inputs, gains, and actuation: **not modified**.
- Kalman equations, process/measurement noise, and covariance mathematics: **not modified**.
- Sensor noise/dropout behavior: **not modified**.
- Radar tracking and radar age: **not modified**.
- V2X scheduling, validation, age, and fusion: **not modified**.
- Scenario definitions, benchmark configuration, and SUMO files: **not modified**.
- Phase 2.2A.1 offline evaluator matching and metrics: **not modified**.

## 8. Limitations and next boundary

Measurement age is freshness metadata, not proof of accuracy.  Future Phase 2.2B may use the appended speed/acceleration ages with separately logged truth to evaluate error and covariance versus measurement age.  Position accuracy still requires an approved independent continuous localization reference; lane-local SUMO position remains excluded.  No Phase 2.2B work was started here.
