# SAMTRUCK Phase 2.1B — Passive Estimator Integration Report

## Scope and integration plan

Phase 2.1B integrates the completed Phase 2.1A estimators into the existing **passive** `PlatoonTwin`.  It does not change CACC, controller inputs, vehicle commands, SUMO routes, scenario JSON, `PendingV2XChannel`, or benchmark parameters.

The inspected interfaces established two important constraints:

1. `OdometryMeasurement.position_m` is populated from `traci.vehicle.getLanePosition()` by the simulation boundary.  It is lane-local, not a continuous route coordinate, so it cannot be an independent position observation.
2. The passive Twin receives only `VehicleSensorInput` DTOs and the `V2XMessage` list returned by `PendingV2XChannel.deliver_ready()`.  Current V2X speed/acceleration payloads are captured from the simulator boundary, so they are retained as separate delayed V2X observations and are not fused into the own-state or radar filters.

## Files changed

| File | Change |
| --- | --- |
| `samtruck/twin_runtime.py` | Integrated independent `VehicleStateEstimator` and per-ego `RadarTargetTracker` instances into the passive Twin; added filtered/radar row serializers. |
| `run_all_scenarios.py` | Writes separate filtered-state/radar-track CSVs and an **offline-only** filtered evaluation JSON after a run.  Existing raw Twin CSV and evaluation CSV are preserved. |
| `tests/test_twin_estimator_integration.py` | Created deterministic integration tests without SUMO. |
| `docs/PHASE_2_1B_IMPLEMENTATION_REPORT.md` | Created this report. |

No changes were made to `samtruck/controller.py`, `samtruck/v2x.py`, `samtruck/sensors.py`, `samtruck/config.py`, scenarios, city/network files, or CACC logic.

### Odometry-validity correction

The passive filter integration now gates both odometry channels on `OdometryMeasurement.valid`.  When `valid` is `False`, `PlatoonTwin._update_filtered_state()` passes `None` for both speed and acceleration, even if the DTO contains numerical payload values.  It does not substitute zero.  An already initialized filter therefore performs its normal timestamp-based prediction-only step; an invalid initial DTO remains `uninitialized_missing_speed`.

This correction changed only:

- `samtruck/twin_runtime.py`
- `tests/test_twin_estimator_integration.py`
- this report

It did not alter estimator equations, covariance handling, lane-local position exclusion, CACC, V2X, runner behavior, scenarios, or SUMO configuration.

## Architecture and data flow

```text
SUMO/TraCI (simulation boundary only)
  ├─ SensorInterface -> VehicleSensorInput DTOs -> PlatoonTwin
  │                                             ├─ VehicleStateEstimator (per truck)
  │                                             └─ RadarTargetTracker (per ego truck, per target ID)
  ├─ PendingV2XChannel -> delivered/validated V2XMessage -> PlatoonTwin
  │                                             └─ separate delayed predecessor observation only
  └─ independent truth rows -> twin_ground_truth.csv -> offline comparison only

Existing inline CACC branch -> CACCController -> SUMO actuation
```

The Twin remains observational.  Filtered output is never supplied to CACC and cannot alter speed, braking, gap, route, or vehicle actuation.

## Estimator integration details

`PlatoonTwin` now creates one `VehicleStateEstimator` per truck in `vehicle_estimators`.  It updates each using only `OdometryMeasurement.speed_mps` and `OdometryMeasurement.acceleration_mps2` at the DTO timestamp.  The filter therefore derives `dt` from measurement timestamps, not a hardcoded timestep.

The Twin deliberately does **not** pass `OdometryMeasurement.position_m` to `independent_position_m`.  The filter's position starts from its broad-uncertainty prior and propagates from the state model.  This avoids both lane/edge discontinuities and double-counting speed-integrated distance as an independent measurement.

Missing speed/acceleration values pass through as `None`; they cause the estimator to remain uninitialized or perform prediction-only propagation.  No missing measurement is replaced with a SUMO value.

## Radar behavior and expiration policy

Each ego truck owns a separate `RadarTargetTracker`.  Tracks are keyed by `RadarMeasurement.sensed_leader_id`; a cut-in target is therefore distinct from the assigned platoon predecessor.  A valid radar update refreshes only its matching target.  At every subsequent sensor timestamp, known targets absent from the current radar measurement call `predict_missing()`.

The Phase 2.1B expiration policy is deliberately conservative: tracks are retained for the duration of a run and visibly become prediction-only with increasing `track_age_s`; they are not silently deleted after one missed detection.  This prevents a missing radar detection from being misrepresented as a fresh measurement.  A bounded time-to-live/track-management policy is deferred until requirements for lateral/scene tracking are approved.

## V2X handling

`PendingV2XChannel` remains the sole delivery, scheduling, packet-loss, and validation authority.  `PlatoonTwin` sees only the runner's post-`deliver_ready()` list.  A message is interpreted as a predecessor observation only when its receiver's configured predecessor matches `message.sender_id`.

The Twin uses `propagate_predecessor_v2x()` to retain the original generation timestamp, computed message age, and stale/fresh status.  It does not treat delayed data as current, does not create another message queue, and does not fuse V2X values into an ego-state filter or radar track.

## CSV and offline evaluation outputs

For each official-run scenario, new files are created in `results/<scenario>/`:

| File | Contents |
| --- | --- |
| `twin_filtered_states.csv` | Timestamp, truck ID, filtered position/speed/acceleration, variances, estimator status, measurement mask, and separate predecessor-V2X metadata.  It contains no ground-truth or lane-position fields. |
| `twin_radar_tracks.csv` | Ego ID, sensed target ID, estimated range/relative speed, uncertainty, measurement timestamp, track age, component provenance, and measurement/prediction status. |
| `twin_filtered_evaluation.json` | Offline comparison counts and speed/acceleration error summaries joined only after completion with `twin_ground_truth.csv`. |

Existing `twin_estimates.csv`, `twin_ground_truth.csv`, and `twin_v2x_messages.csv` retain their prior role and column meanings.

## Tests executed

```powershell
python -m unittest tests.test_estimation tests.test_digital_twin tests.test_twin_estimator_integration -v
python -m unittest discover -s tests -v
```

| Check | Result |
| --- | --- |
| Focused estimator + Twin integration suite | **37/37 passed** |
| Full regression suite | **52/52 passed** |

After the odometry-validity correction:

| Check | Result |
| --- | --- |
| Focused estimator + Twin integration suite | **40/40 passed** |
| Full regression suite | **55/55 passed** |

The additional focused tests prove that invalid numerical speed/acceleration do not update a previously initialized filter, invalid initial odometry does not initialize it, valid odometry continues to update normally, and existing missing-measurement prediction behavior remains intact.

The new integration tests verify independent filters, no cross-truck state leakage, timestamp-derived prediction, safe missing measurements, exclusion of lane-local position, identity-keyed cut-in radar tracks, prediction-only radar aging, delivered-only predecessor V2X handling, original V2X generation time retention, separation of raw versus filtered Twin rows, and absence of Twin imports for simulator/controller/evaluation modules.

## Fixed-seed headless SUMO validation

The official `run_all_scenarios.py` runner was executed headlessly with `--fast-demo --seed 20260929`.  Fast-demo shifts timing in memory but does not modify benchmark configuration or scenario files.

| Scenario | Result | Filtered rows | Radar rows | Speed MAE / RMSE (m/s) | Acceleration MAE / RMSE (m/s²) |
| --- | --- | ---: | ---: | ---: | ---: |
| `normal_platooning` | PASS | 750 | 500 | 0.0968 / 0.1583 | 0.3859 / 0.7751 |
| `emergency_braking` | PASS | 750 | 500 | 0.1085 / 0.1666 | 0.4259 / 0.7964 |
| `v2x_latency_packet_loss` | PASS | 1,050 | 728 | 0.0640 / 0.1181 | 0.2675 / 0.6223 |

For `v2x_latency_packet_loss`, the Twin V2X log recorded 562 delivered messages, 93 packet-loss drops, and 3 duplicate/out-of-order rejections.  The radar CSV recorded 69 prediction-only rows, confirming that missed detections are logged as predictions rather than fabricated measurements.

Position error is explicitly **not evaluated**: the independent truth log currently has lane-local position and the filter has no independent continuous localization coordinate.  The reported speed/acceleration errors are real post-run comparisons, not a claim that the estimator is calibrated or accurate under all conditions.

## Ground-truth separation verification

`samtruck/twin_runtime.py` imports sensor DTOs, estimator types, and `V2XMessage` only.  It does not import TraCI, SUMOlib, an evaluator, a scenario runner, or `CACCController`.  The filtered-state CSV is generated entirely from Twin estimates and metadata.  Ground truth is joined only by `evaluate_filtered_outputs()` in the runner after the simulation loop and after filtered rows have already been produced.

## CACC and benchmark preservation

`CACCController` was not modified.  Its source checksum remains:

```text
FE3170BABBB1CBF24F92AEA5E0A6885FAB52396B762445038127EC2893DF34FC
```

The controller's existing regression tests pass.  No filtered state is passed into its invocation, and no controller configuration, gains, CTG settings, scenario definition, route, or SUMO configuration was changed.

## Remaining limitations and recommended next phase

1. Filtered position is unanchored because independent continuous localization is intentionally absent; it must not be compared with lane-local route position.
2. Current position/acceleration sensor channels are ideal/pass-through at the boundary; covariance values remain simulation assumptions rather than calibrated specifications.
3. V2X is stored separately and not fused.  Its current payload is simulator-boundary data, not a sender-local filtered estimate.
4. Radar target retention is conservative and has no bounded lifecycle policy yet.
5. No adaptive control, prediction, AI, or actuator path exists.

The recommended next step is a separately approved Phase 2.1C/2.2 design for continuous sensor-side dead reckoning or an independent localization DTO, V2X measurement semantics, and offline covariance calibration.  CACC must remain the baseline until an explicitly separate adaptive-control phase.
