# Phase 2.2A.2 — Measurement Reliability and Age Tracking Design

## 1. Scope and executive summary

This is a design-only audit for the SAMTRUCK workspace at `E:\SAMSUNGTRUCK`.  No estimator, Twin, controller, scenario, SUMO, or evaluator code was changed.

The current own-vehicle Kalman filter can identify measurements used *in the current update* through `FilteredVehicleState.measurement_update_mask`.  It cannot identify, for speed, acceleration, and independent position separately, when the component was last measured or how long it has been propagated without a fresh measurement.  This document proposes component-specific freshness metadata while preserving the passive Twin and existing output fields.

## 2. Verified current implementation findings

### 2.1 Estimator state and timestamps

`samtruck/estimation.py` implements `VehicleStateEstimator` with state `[position, speed, acceleration]`.  Its `update()` method maintains one estimator timestamp (`_timestamp_s`) and emits immutable `FilteredVehicleState` snapshots containing:

- `timestamp_s`, state estimate, covariance and standard deviations;
- `initialized` and `status`; and
- `measurement_update_mask`, whose current tokens are `position`, `speed`, and `acceleration`.

At initialization, a valid speed is required.  A supplied acceleration is observed; a missing acceleration is represented by a zero-valued **prior with broad covariance**, and the output status is `partially_initialized`.  Therefore that prior is not evidence of an acceleration measurement.  A supplied independent position is represented by the `position` token only when actually passed to `update()`.

For an initialized estimator, a positive timestamp interval predicts the state before any available measurement rows are applied.  `predicted_only`, `updated`, `updated_same_timestamp`, `degraded_long_gap_predicted`, and `degraded_long_gap_updated` distinguish broad update cases.  An out-of-order timestamp returns `invalid_timestamp` without advancing the estimator timestamp.  None of these mechanisms retains per-component last-measurement timestamps.

### 2.2 Twin input and filtered-state logging

`samtruck/twin_runtime.py` owns one `VehicleStateEstimator` per truck.  In `PlatoonTwin._update_filtered_state()`, valid odometry supplies speed and acceleration; invalid odometry supplies `None` for both.  The existing lane-local `odometry.position_m` is deliberately excluded because it is not a continuous route coordinate and therefore is not an independent position measurement.

`PlatoonTwin.filtered_as_rows()` serializes the filtered output.  `run_all_scenarios.py` writes it to `results/<scenario>/twin_filtered_states.csv` using `FILTERED_STATE_FIELDS`.  The current columns include estimator state, diagonal covariance, `estimator_status`, `measurement_update_mask`, a fixed `independent_position_measurement_used=False`, and predecessor V2X metadata.  They do not include own-state last-valid measurement timestamps or ages.

### 2.3 Existing age-like values are not interchangeable

The radar tracker already emits target-specific `last_measurement_timestamp_s`, `track_age_s`, and `last_update_was_measurement` in `twin_radar_tracks.csv`.  Those values describe a radar **target track**, not an own-state speed or acceleration measurement.

The filtered-state CSV also emits `predecessor_v2x_generation_time_s` and `predecessor_v2x_age_s`.  These describe delivered predecessor V2X message freshness, not an odometry measurement or filter prediction duration.  V2X is not fused into the own-state filter in this phase.

`samtruck/offline_estimator_evaluator.py` currently classifies speed and acceleration separately from `measurement_update_mask` plus status.  This is useful for current-row update provenance but cannot calculate elapsed time since the last valid component measurement.

## 3. Identified gaps

1. No component-specific last-valid timestamp exists for own speed, own acceleration, or independent position.
2. A non-empty row mask does not establish that every component is fresh.  For example, a speed-only update must not refresh acceleration freshness.
3. `partially_initialized` must not imply that the broad acceleration prior is a fresh measurement.
4. `updated_same_timestamp` can update one component while another component was neither predicted nor measured; a row-level status cannot determine its freshness.
5. Current CSV output cannot support accuracy-versus-own-measurement-age or covariance-consistency analyses required later in Phase 2.2B.

## 4. Proposed measurement-age data model

### 4.1 Per-component internal metadata

In a future approved implementation, `VehicleStateEstimator` should maintain the following independently for `position`, `speed`, and `acceleration`:

- `last_<component>_measurement_timestamp_s: float | None`
- `<component>_measurement_ever_received: bool`

The boolean is explicit for clarity, although it is logically equivalent to a non-`None` timestamp.  It avoids overloading `0.0` and makes a never-measured component unambiguous.

`position` refers strictly to an explicitly supplied **independent** position observation.  It must not be set from lane-local `OdometryMeasurement.position_m` or speed-integrated dead reckoning.

### 4.2 Snapshot/output metadata

Extend `FilteredVehicleState` only by additive, optional fields (with defaults if necessary for compatibility):

- `last_independent_position_measurement_timestamp_s`
- `independent_position_measurement_age_s`
- `independent_position_measurement_ever_received`
- `last_speed_measurement_timestamp_s`
- `speed_measurement_age_s`
- `speed_measurement_ever_received`
- `last_acceleration_measurement_timestamp_s`
- `acceleration_measurement_age_s`
- `acceleration_measurement_ever_received`
- `position_estimate_provenance`, `speed_estimate_provenance`, `acceleration_estimate_provenance`

Recommended provenance values are:

- `measurement_updated` — this component had a valid row in the current Kalman update;
- `prediction_only` — a valid, initialized component was advanced or retained without a fresh measurement at this estimator timestamp;
- `not_meaningfully_initialized` — no valid observation has yet established this component (including the acceleration prior in `partially_initialized`);
- `invalid_timestamp_no_update` — a rejected timestamp produced no state progression or component update;
- `unavailable` — no state output exists for the component.

These are component-level labels.  They do not replace the existing row-level `estimator_status` or `measurement_update_mask`.

## 5. Exact timestamp and age semantics

Let `t_current` be the accepted estimator snapshot timestamp, not simply any timestamp passed by a caller.  For component `c`, let `t_last[c]` be its last valid measurement timestamp.

```
age[c] = null                         if t_last[c] is null
age[c] = max(0, t_current - t_last[c]) otherwise
```

The `max` guard is defensive; the implementation must reject out-of-order timestamps rather than manufacture a negative age.

| Situation | Timestamp/age rule |
| --- | --- |
| First valid speed / acceleration / independent-position measurement | Set only that component's `t_last` to `t_current`; its age is `0.0`. |
| Initial speed-only state | Speed age is `0.0`; acceleration and independent-position ages are `null`. The acceleration prior is `not_meaningfully_initialized`. |
| Time-advancing speed-only update | Refresh only speed timestamp/age. Acceleration and position timestamps remain unchanged; their non-null ages grow by elapsed accepted estimator time. |
| Time-advancing acceleration-only update | Symmetric to speed-only. |
| Prediction-only update | Do not refresh any component timestamp. Non-null ages increase relative to the new `t_current`; never-measured ages remain `null`. |
| Same-timestamp partial update | Refresh only supplied components to age `0.0`. An omitted component retains its prior timestamp and age; it must not be called fresh merely because the row was updated. |
| Invalid/out-of-order timestamp | Do not change component timestamps. Snapshot/age calculations remain relative to the estimator's existing accepted timestamp; provenance is `invalid_timestamp_no_update`. |
| Invalid/missing odometry component | It is absent (`None`) and cannot refresh a timestamp. It must never be replaced by zero. |

The future implementation should evaluate measurement validity and timestamp validity before mutating component freshness metadata.  A valid measurement token in the Kalman update mask is the sole condition that refreshes that component.

## 6. Ages that must remain separate

| Quantity | Meaning | Must not be used as own component measurement age |
| --- | --- | --- |
| Estimator prediction interval | `dt` between accepted estimator timestamps for a particular update | It is per-update, not time since last valid component observation. |
| Sensor packet/DTO time or transport delay | When a sensor observation was sampled or delivered | A packet delay is not evidence of the filter's last accepted measurement. |
| `predecessor_v2x_age_s` | Age of an actually delivered predecessor V2X message | V2X has its own source, identity, and no own-state fusion here. |
| Radar `track_age_s` | Age since a target-specific radar track was measured | It concerns a sensed target, not ego vehicle speed/acceleration/position. |

## 7. Proposed CSV fields

Append the following fields to `twin_filtered_states.csv`; do not rename or remove current columns.

| Field | Unit/type | Meaning |
| --- | --- | --- |
| `last_independent_position_measurement_timestamp_s` | s or empty | Last accepted independent position measurement; expected empty in the current runner. |
| `independent_position_measurement_age_s` | s or empty | Age at `simulation_time_s`; empty if never independently observed. |
| `independent_position_measurement_ever_received` | boolean | Explicit independent-position availability. |
| `last_speed_measurement_timestamp_s` | s or empty | Last accepted valid odometry-speed measurement. |
| `speed_measurement_age_s` | s or empty | Age at `simulation_time_s`; empty if speed has never been measured. |
| `speed_measurement_ever_received` | boolean | Explicit speed availability. |
| `last_acceleration_measurement_timestamp_s` | s or empty | Last accepted valid odometry-acceleration measurement. |
| `acceleration_measurement_age_s` | s or empty | Age at `simulation_time_s`; empty if acceleration has never been measured. |
| `acceleration_measurement_ever_received` | boolean | Explicit acceleration availability. |
| `position_estimate_provenance` | string | Component-level current-row provenance. |
| `speed_estimate_provenance` | string | Component-level current-row provenance. |
| `acceleration_estimate_provenance` | string | Component-level current-row provenance. |

Empty CSV values represent unavailable/null values; zero is only valid for an observed component at the current estimator timestamp.  Existing `measurement_update_mask`, `estimator_status`, V2X fields, and radar CSV fields remain authoritative for their original meanings.

## 8. Future integration points (not implemented in this phase)

1. **`samtruck/estimation.py`** — add private component timestamp state, compute ages in snapshots, and add the output metadata to `FilteredVehicleState`.  No Kalman equations, covariance mathematics, or measurement selection changes are needed.
2. **`samtruck/twin_runtime.py`** — expose the new snapshot metadata from `filtered_as_rows()`.  Continue excluding lane-local position and invalid odometry.
3. **`run_all_scenarios.py`** — append the CSV field names to `FILTERED_STATE_FIELDS`; no changes to the simulation loop, CACC invocation, V2X handling, scenarios, or SUMO configuration.
4. **Tests** — extend `tests/test_estimation.py` and `tests/test_twin_estimator_integration.py` with deterministic freshness and CSV serialization tests.
5. **Later offline analysis, separately approved** — `samtruck/offline_estimator_evaluator.py` may consume the appended age fields for speed/acceleration error stratification.  Phase 2.2A.1 matching and all-sample metrics must remain unchanged unless an explicit later task authorizes extensions.

This data remains inside the passive Twin output path.  It does not access evaluation truth, alter vehicle actuation, or feed the CACC controller.

## 9. Test plan for the future implementation

1. **Initial full observation:** speed and acceleration timestamps equal initialization time and ages are both zero.
2. **Speed-only initialization:** speed age is zero; acceleration and independent-position timestamps/ages are null; acceleration provenance is `not_meaningfully_initialized`.
3. **Speed-only time-advancing update:** only speed resets to zero; acceleration age advances if previously observed.
4. **Acceleration-only time-advancing update:** symmetric check for acceleration.
5. **Independent position:** only an explicit independent-position argument sets/reset its timestamp.  Lane-local `position_m` must not do so in Twin integration.
6. **Prediction-only intervals:** all observed-component ages advance by timestamp-derived `dt`; no timestamp refresh occurs.
7. **Repeated predictions:** age accumulates across multiple prediction-only updates.
8. **Partial update after a gap:** supplied component resets; omitted components retain their own accumulated age.
9. **Never-measured components:** age stays null, never zero, across predictions.
10. **Invalid odometry:** numeric values in an invalid DTO do not reset speed or acceleration timestamps.
11. **Invalid/out-of-order timestamp:** no component timestamp mutates and rejected snapshot provenance is explicit.
12. **Same-timestamp partial update:** supplied component becomes fresh; omitted component remains at its previous age/provenance and is not presented as freshly measured.
13. **CSV serialization:** all appended fields serialize correctly, while existing filtered CSV fields and V2X metadata are unchanged.
14. **Boundary regression:** static/import checks confirm no TraCI or evaluation truth enters the estimator or Twin; CACC behavior remains unchanged.

## 10. Use in later Phase 2.2B validation

Component ages enable offline plots and metrics that are impossible from the current row mask alone:

- speed/acceleration MAE and RMSE by age bins;
- error and squared error versus covariance as observations become stale;
- counts of fresh, short-gap, long-gap, and never-observed estimates;
- scenario/event comparisons during radar/odometry loss, braking, and V2X degradation.

Age is a reliability descriptor, not a claim of accuracy.  It must be reported alongside estimator covariance, status, measurement provenance, scenario identity, and independently logged ground truth.  Position-accuracy analysis remains unavailable until an independent continuous localization reference is designed.

## 11. Risks and backward compatibility

- Adding fields to the frozen `FilteredVehicleState` must use defaults or otherwise preserve external construction compatibility.
- Existing logs and the offline evaluator may not contain the new columns.  Future readers must treat missing new columns as unavailable, not as zero-age measurements.
- Component provenance must not be inferred only from row-level `estimator_status`; `updated_same_timestamp` and `partially_initialized` demonstrate why.
- Float comparisons should use the accepted estimator timestamp and existing timestamp validation, not rounded CSV values.
- No ages should be derived from ground-truth logs, radar track fields, or V2X metadata.

## 12. Confirmation

This Phase 2.2A.2 activity created only this design document.  No existing source files, tests, estimator equations, sensor behavior, CACC behavior, V2X delivery/fusion behavior, scenarios, SUMO configuration, or Phase 2.2A.1 evaluator matching/metrics were modified.
