# SAMTRUCK Phase 2.2A — Estimation Accuracy Audit

**Status:** design and audit only.  This document records verified current behavior and proposes a reproducible accuracy-evaluation method.  No estimator, controller, scenario, SUMO, configuration, test, or dependency change was made.

## 1. Executive summary

SAMTRUCK has a passive three-truck estimation pipeline.  `VehicleStateEstimator` reconstructs an own-vehicle state `[position, speed, acceleration]` using a constant-acceleration Kalman filter.  In the current Phase 2.1B integration, only odometry speed and acceleration are supplied as measurement updates; lane-local position is intentionally excluded.  `RadarTargetTracker` separately tracks range and relative speed by sensed target ID.

Verified current output permits meaningful **offline speed and acceleration** comparisons against separately logged SUMO data.  It does **not** permit a mathematically meaningful position-accuracy claim because the filtered position has no independent continuous localization reference and the truth log position is lane-local.  Radar accuracy is only partly evaluable: the truth log contains the sensed leader ID and range, but it does not log target-relative speed, and it does not explicitly record a target identity at the same granularity as every radar-track row.

The current offline evaluator in `run_all_scenarios.py` reports only sample count, MAE, and RMSE for speed and acceleration.  It uses an in-memory dictionary keyed by rounded `(simulation_time_s, vehicle_id)`.  This works for current normal fixed-step output but is not yet robust enough for a formal accuracy study because duplicate keys overwrite silently and unmatched records are not reported.

Passing unit tests or one fixed-seed fast-demo run establishes functional behavior, not estimator reliability or calibrated accuracy.

## 2. Verified existing estimator architecture

### Live passive data flow

```text
SUMO/TraCI after simulationStep()                         (simulation boundary only)
  -> run_all_scenarios.build_twin_inputs_and_truth()
      -> SensorInterface.make_input() -> VehicleSensorInput
          -> PlatoonTwin.update()
              -> VehicleStateEstimator per truck
              -> RadarTargetTracker per ego truck, keyed by sensed_leader_id
      -> independent truth rows -> twin_ground_truth.csv

PendingV2XChannel.deliver_ready()
  -> delivered/validated V2XMessage list -> PlatoonTwin.update()
      -> separate predecessor V2X observation only; no filter fusion

Existing inline CACC branch -> CACCController -> SUMO vehicle actuation
```

Verified implementation references:

- `samtruck/estimation.py`
  - `VehicleStateEstimator`: own-state linear KF with `[s, v, a]`.
  - `RadarTargetTracker`: relative-target state `[range, ego_speed_minus_target_speed]`.
  - `propagate_predecessor_v2x()`: preserves message generation time and propagates delayed V2X speed/acceleration without queueing or fusion.
- `samtruck/twin_runtime.py`
  - `PlatoonTwin.vehicle_estimators`: one independent estimator per truck.
  - `PlatoonTwin.radar_trackers`: one independent tracker per ego vehicle.
  - `_update_filtered_state()`: supplies speed/acceleration only if `OdometryMeasurement.valid`; intentionally never passes lane-local `position_m` as `independent_position_m`.
- `samtruck/sensors.py`
  - `SensorInterface` applies configurable noise to radar range and odometry speed, supports radar dropout, and currently passes position, acceleration, and radar relative speed through unchanged.
- `samtruck/v2x.py`
  - `PendingV2XChannel` is still the sole authority for Twin-branch delivery, packet loss, timestamp/sequence validation, and stale rejection.
- `run_all_scenarios.py`
  - `evaluate_filtered_outputs()` performs its post-run, offline comparison only after the live Twin has written filtered output.

No estimator output reaches `samtruck/controller.py` or vehicle actuation.

## 3. Available ground-truth signals and current output files

The official runner currently writes these relevant files under `results/<scenario>/`:

| File | Relevant columns | Role |
| --- | --- | --- |
| `twin_ground_truth.csv` | `simulation_time_s`, `vehicle_id`, `lane_id`, `lane_position_m`, `speed_mps`, `acceleration_mps2`, `sensed_leader_id`, `actual_gap_m` | Independent evaluation-only log. |
| `twin_filtered_states.csv` | `simulation_time_s`, `vehicle_id`, `estimated_position_m`, `estimated_speed_mps`, `estimated_acceleration_mps2`, three diagonal variance columns, `estimator_status`, `measurement_update_mask`, predecessor V2X metadata | Passive own-state estimator output. |
| `twin_radar_tracks.csv` | `simulation_time_s`, `ego_vehicle_id`, `sensed_target_id`, `estimated_range_m`, `estimated_relative_speed_mps`, variances, `track_age_s`, `last_measurement_timestamp_s`, provenance/status | Passive identity-keyed radar tracks. |
| `twin_v2x_messages.csv` | sender/receiver, generation/scheduled/delivery time, status, reason | Delivered/dropped/rejected Twin communication evidence. |
| `twin_filtered_evaluation.json` | currently position availability flag, speed/acceleration MAE/RMSE and row counts | Current minimal offline summary. |

The truth log is produced alongside DTO construction but is not passed to `PlatoonTwin`, estimators, or CACC.  That separation is required for a valid offline accuracy study.

## 4. Signal-by-signal comparability

| Estimated quantity | Current truth quantity | Units/frame | Timestamp key | Meaningful now? | Verified limitation |
| --- | --- | --- | --- | --- | --- |
| `estimated_speed_mps` | `twin_ground_truth.speed_mps` | m/s, vehicle longitudinal speed | `(simulation_time_s, vehicle_id)` | Yes, offline | Sensor speed may be noisy if configured; normal default currently has zero speed noise. |
| `estimated_acceleration_mps2` | `twin_ground_truth.acceleration_mps2` | m/s², vehicle longitudinal acceleration | Same | Yes, offline | Sensor acceleration is currently ideal/pass-through when valid, so this is not yet a realistic noisy-accelerometer assessment. |
| `estimated_position_m` | `twin_ground_truth.lane_position_m` | Filter position is unanchored propagated distance; truth is lane-local distance | Superficially same timestamp, incompatible coordinate | **No** | Lane position can reset/change at lane, edge, junction, and loop transitions.  The filter starts at `0` with broad position uncertainty and does not ingest an independent position observation. |
| `estimated_range_m` | `twin_ground_truth.actual_gap_m` | m, ego-to-current SUMO leader | Needs `(time, ego ID, target ID)` identity proof | Conditionally, not implemented | Truth has `sensed_leader_id` and range but no dedicated radar truth row.  A comparison is valid only when truth `sensed_leader_id == sensed_target_id` and timestamps align. |
| `estimated_relative_speed_mps` | No dedicated truth column | m/s, ego minus radar-target speed | Needs target speed at identical time | Not currently | Truth lacks target speed and target identity-specific relative speed.  It must be derived offline only after target-ID matching, or logged independently in a future approved evaluation change. |
| predecessor V2X propagated speed/acceleration | Sender ground truth at message generation/propagated timestamp | m/s, m/s² | sender, receiver, generation timestamp | Diagnostic only | Current payload is simulator-boundary data, not a sender-local sensor estimate; it is deliberately not fused. |

### Position comparison requirement

Before position accuracy can be evaluated, the project needs an independent, continuous localization reference with a defined origin and frame.  Examples are a sensor-side dead-reckoned distance with documented uncertainty, a map-matched route-progress sensor DTO, or a separately logged continuous route-distance truth field.  The live Twin must receive only the sensor/localization DTO; truth remains offline.  Until then, position MAE/RMSE must be reported as unavailable, as the current `twin_filtered_evaluation.json` correctly does.

## 5. Accuracy metric definitions

For an aligned estimate `hat(x)_i` and independent truth `x_i`, define error `e_i = hat(x)_i - x_i`.

| Metric | Definition | What it shows |
| --- | --- | --- |
| MAE | `mean(abs(e))` | Typical absolute error, less dominated by rare spikes than RMSE. |
| RMSE | `sqrt(mean(e²))` | Error magnitude with stronger penalty for large transient failures. |
| Mean error / bias | `mean(e)` | Persistent over- or under-estimation direction. |
| Maximum absolute error | `max(abs(e))` | Worst observed aligned sample; always report sample count and scenario context. |
| 95th percentile absolute error | `percentile(abs(e), 95)` | Near-worst routine error without allowing one outlier to define the result. |
| Sample count | number of valid aligned pairs | Essential context for every metric. |

All speed errors use m/s, acceleration errors use m/s², range/position errors use m.  Do not report a metric when the coordinate frame or target identity is not compatible.

Report every applicable metric both overall and stratified by update status:

- measurement-updated: non-empty `measurement_update_mask`;
- prediction-only: empty `measurement_update_mask` and status such as `predicted_only`/`degraded_long_gap_predicted`;
- short gap versus long gap, using elapsed time since last valid observation rather than only a categorical status.

Scenario/event partitions should also include normal driving, leader acceleration/deceleration, emergency braking, radar target change/cut-in, V2X latency/loss/outage, and sensor-noise/dropout variants when those variants actually exist.

## 6. Timestamp alignment audit

### Verified current behavior

`run_all_scenarios.evaluate_filtered_outputs()` currently builds a truth dictionary with:

```python
(round(float(simulation_time_s), 6), vehicle_id)
```

and looks up filtered rows with the same rounded key.  Current sample output uses simulation times such as `0.1`, and each active truck has one truth and one filtered row per step.  This makes current speed/acceleration summaries plausible for normal fixed-step runs.

### Risks in the current join

1. Rounding to six decimal places is a practical floating-point guard, but it is not an explicit alignment policy.
2. The dictionary silently overwrites duplicate truth `(time, vehicle)` keys.  It does not count duplicate or unmatched rows.
3. It does not verify that an estimate and truth record belong to the same simulation step beyond rounded time and vehicle ID.
4. It does not distinguish filter initialization or prediction-only status when aggregating error.
5. Radar tracks need a three-part identity `(time, ego_vehicle_id, sensed_target_id)`; current truth does not create that dedicated structure.

### Recommended robust matching strategy (Phase 2.2B, not implemented)

1. Parse timestamps as numeric values; retain raw time and a monotonic simulation-step index if available.
2. Validate uniqueness of `(vehicle_id, simulation_time_s)` in each own-state file; fail/report duplicates rather than overwriting.
3. Exact-match by vehicle ID and a declared tolerance no greater than half the SUMO step length, while recording matched, unmatched, duplicate, and tolerance-exceeded counts.
4. Prefer an explicit simulation-step index in future evaluation logs; use it with vehicle ID as the primary key and timestamp as a validation field.
5. For radar, require matching ego ID, target ID, and time.  Reject comparisons when truth target identity differs or is absent.
6. Keep truth joining in a post-run evaluator module only; never expose joined rows to the live Twin.

## 7. Measurement-versus-prediction analysis

The current own-state CSV has `measurement_update_mask` and `estimator_status`.  The radar CSV has `last_update_was_measurement`, `track_age_s`, `last_measurement_timestamp_s`, and `track_status`.

Recommended reporting groups:

| Group | Selection | Required reporting |
| --- | --- | --- |
| Own-state update | non-empty `measurement_update_mask` | Speed/acceleration MAE, RMSE, bias, p95, max, n. |
| Own-state prediction-only | empty mask | Same metrics, separately. |
| Short prediction gap | elapsed age from last valid odometry within approved short-gap bins | Error versus age bin and n. |
| Long prediction gap | gap exceeds the approved `long_gap_s`/larger bins | Error, covariance, n, and reason. |
| Radar measurement update | `last_update_was_measurement=True` | Range error when target identity matches. |
| Radar prediction-only | `last_update_was_measurement=False` | Range/relative-speed error (if truth becomes available) versus `track_age_s`. |

Important unresolved data issue: `twin_filtered_states.csv` does not currently include own-odometry last-valid timestamp/age.  `estimator_status` can separate prediction-only at the current row, but cannot reconstruct a precise cumulative time since the last valid own measurement across arbitrary gaps.  A future evaluation design should add that estimator metadata without changing the live-control boundary.

Recommended plots for this analysis:

- error versus simulation time, colored by update/prediction state;
- absolute-error box/violin or ECDF distributions by group;
- error and covariance/standard deviation versus time since last measurement;
- a table with `n`, MAE, RMSE, bias, p95, and maximum per group.

## 8. Scenario selection

The verified official catalog in `scenarios/catalog/` has twenty JSON scenarios.  Recommended minimum estimator-evaluation set:

| Catalog scenario | Existing event(s) | Estimator behavior tested |
| --- | --- | --- |
| `normal_platooning` | none | Nominal baseline and steady-state error. |
| `leader_acceleration` | `leader_speed` | Speed/acceleration transient response. |
| `emergency_braking` | `emergency_brake` | High-deceleration response, peak error, and covariance behavior. |
| `v2x_latency` | `v2x_delay` | Timestamp/generation-age logging under delayed communication; own-state filter remains sensor-only. |
| `v2x_packet_loss` | `packet_loss` | Delivered/drop/rejected communication accounting; no false V2X freshness. |
| `v2x_outage` | `v2x_outage` | Missing predecessor V2X observation status. |
| `cut_in` | `cut_in` | Radar target identity change versus assigned predecessor. |
| `cut_out` | `cut_in`, `cut_out` | Target disappearance and conservative prediction-only track aging. |
| `multiple_disturbances` | latency, cut-in, packet loss, braking | Compound behavior and event-stratified reporting. |

### Sensor noise and dropout gap

No current catalog JSON has a `twin` sensor configuration; all inspected catalog entries report `twin=(none)`.  `SensorInterface` supports radar noise, odometry speed noise, and radar dropout, but these are not presently represented as official catalog sensor-noise or sensor-dropout scenarios.  Therefore, neither noise nor dropout estimator accuracy is established by the existing catalog.  A future approved Phase 2.2B evaluation plan should define controlled immutable evaluation variants using the runner's existing `twin` fields, without changing base scenarios or calling the result an official baseline until approved.

## 9. Repeatability and statistical reliability protocol

### Recommended protocol

1. Preserve the same SUMO version, network, runner, duration, controller configuration, and scenario JSON for all compared runs.
2. Record the scenario ID, source JSON checksum, runner command, SUMO version, project revision/checksum, seed, sensor configuration, estimator configuration, and step length alongside every result.
3. First use one fixed seed per selected scenario for a reproducibility check: repeated executions must produce equivalent communication outcomes and matching summary metrics within a documented floating-point tolerance.
4. Then use multiple independent seeds for stochastic variants (packet loss, radar dropout, sensor noise).  An initial engineering recommendation is **10 seeds** for smoke-level variability characterization and **30 seeds** before drawing comparative conclusions; these are recommendations, not current project requirements.
5. Use the same seed set and scenario durations for any later estimator tuning or controller comparison.  Keep any tuning/calibration seed set separate from final evaluation seeds.
6. Report seed-wise metrics plus median, mean, spread/interval, and failure/unmatched sample counts; do not only pool all samples because long runs can dominate.

One successful `--fast-demo` run is insufficient: it changes event timing in memory, exercises one random realization, and cannot characterize sensitivity to measurement loss/noise, edge cases, or seed-to-seed variation.

## 10. Proposed acceptance-criteria framework

No numerical thresholds are proposed here.  Thresholds require project-team/mentor approval because sensor models are not calibrated and safety/application objectives have not specified an allowed estimation error.

| Area | Evidence required | How threshold should be chosen | Approval needed |
| --- | --- | --- | --- |
| Speed estimation | MAE/RMSE/bias/p95/max with n by scenario and update state | Absolute m/s target linked to intended control/prediction use and sensor model. | Team/mentor. |
| Acceleration estimation | Same, including emergency-brake partition | Scenario-dependent m/s² target; peak braking accuracy may need stricter treatment. | Team/mentor. |
| Prediction during loss | Error versus measurement age and covariance trend | Relative degradation limit versus updated state and maximum permitted prediction horizon. | Team/mentor. |
| Radar range/relative speed | Target-identity-validated metrics, once suitable truth exists | Absolute range/relative-speed bounds conditioned on range, cut-in, and target continuity. | Team/mentor. |
| Uncertainty consistency | Empirical fraction of errors within 1σ/2σ/3σ and normalized innovation/error diagnostics | Statistical consistency expectations, after noise covariance calibration. | Estimation owner/mentor. |
| Data integrity | Duplicate/unmatched rows, invalid identifiers/timestamps, accidental truth leakage | Normally zero or explicitly explained. | Engineering owner. |
| Reproducibility | Repeated fixed-seed results and seed-wise dispersion | Deterministic equivalence/tolerance and approved sample size. | Engineering owner. |

Passing criteria should be scenario-specific where braking, target changes, and communication loss have different operating conditions.  No criterion should be satisfied merely because the Python process exits successfully or unit tests pass.

## 11. Visualization plan (not implemented)

| Graph | Purpose | Required inputs |
| --- | --- | --- |
| Estimated vs truth speed | Inspect tracking and transient lag | filtered/truth time, vehicle ID, `estimated_speed_mps`, `speed_mps`, event markers. |
| Estimated vs truth acceleration | Inspect braking/acceleration response and smoothing | Same with acceleration columns. |
| Signed/absolute error vs time | Find temporal/event-local failures | aligned error, scenario event times, estimator status. |
| Absolute-error distribution | Show median, tails, and outliers | aligned absolute errors, group labels. |
| Update vs prediction-only comparison | Prevent aggregate metrics hiding loss behavior | error, `measurement_update_mask`, own measurement-age metadata (future). |
| Error vs time since last measurement | Quantify drift through loss | error, age since last valid odometry/radar measurement. |
| Scenario-wise MAE/RMSE table/bar chart | Compare conditions without conflating them | seed-wise scenario summaries and sample counts. |
| Radar range error and target timeline | Verify cut-in/cut-out identity management | radar tracks, identity-matched truth range, target IDs. |
| Covariance vs squared error | Assess uncertainty calibration in Phase 2.2B | filtered covariance diagonal, aligned squared error, status. |

Every plot must label units, scenario/seed, sample count, update/prediction state, and event timing.  Never graph incompatible lane-local position against unanchored filtered position.

## 12. Identified limitations

1. Position accuracy is unavailable without a compatible continuous localization truth/reference.
2. The normal current sensor configuration has no configured noise/dropout in catalog JSON; baseline speed/acceleration comparisons do not prove resilience to imperfect sensing.
3. Acceleration and radar relative-speed DTO channels are currently pass-through when present, which limits realism of those accuracy claims.
4. Current offline evaluator provides MAE/RMSE only and hides bias, tails, duplicates, unmatched samples, status stratification, and seed variability.
5. Radar truth is not yet logged as a target-identity-specific reference; relative-speed truth is unavailable as a direct column.
6. V2X is separate and not fused into the own-state or radar filters; its degradation tests demonstrate communication accounting, not V2X-aided estimation accuracy.
7. `twin_filtered_states.csv` lacks a continuous own-measurement age field needed for full gap-duration stratification.
8. Current radar retention keeps tracks to end-of-run; that is conservative for observability but not a final lifecycle policy.

## 13. Recommended Phase 2.2A implementation tasks (future, not performed)

1. Create an offline-only evaluator that validates one-to-one timestamp/vehicle matches, reports duplicate/unmatched counts, and calculates MAE, RMSE, bias, maximum, p95, and n.
2. Add status-stratified speed/acceleration reports for measurement updates and prediction-only intervals.
3. Add event-aware segmentation from `events.json` without allowing it into the live Twin.
4. Add an offline radar evaluator only after defining identity-compatible truth range/relative-speed records.
5. Add visualization functions and report tables using the proposed input columns.
6. Establish a versioned evaluation manifest and fixed multi-seed protocol.
7. Design, then separately approve, independent continuous localization and sensor-noise/dropout evaluation variants.

## 14. Open questions requiring team approval

1. What continuous localization sensor/reference and coordinate origin should define position accuracy?
2. What sensor noise/dropout configurations are representative enough to become approved evaluation variants?
3. Should future V2X payloads represent sender-local measured state, sender filtered state, or another defined communicated state?
4. What safety/application use will consume estimator output, and therefore what speed/acceleration/range accuracy thresholds are acceptable?
5. What prediction horizon and track-expiration policy are acceptable during sustained measurement loss?
6. Which seeds, durations, and number of repetitions are mandatory for project conclusions?
7. Is a future absolute radar-target truth log permitted in the offline evaluation branch, with target-ID matching, while preserving the live ground-truth boundary?
