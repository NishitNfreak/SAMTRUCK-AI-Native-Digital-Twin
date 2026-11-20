# SAMTRUCK Phase 2.0 — Estimation Layer Design

**Status:** design audit only.  No estimator, controller, scenario, configuration, or source-code change was made in this phase.

## 1. Inspected implementation and current architecture

The active benchmark runner is `run_all_scenarios.py`.  It starts SUMO/TraCI, runs the Montreal scenario catalog, and writes scenario outputs under `results/<scenario>/`.  The relevant current implementation is:

| Component | Current implementation | Phase 2.0 finding |
| --- | --- | --- |
| Sensor boundary | `build_twin_inputs_and_truth()` in `run_all_scenarios.py` reads TraCI scalars, calls `SensorInterface.make_input()`, and separately appends ground-truth CSV rows. | The DTO/evaluation split exists, but `position_m` is currently lane-local. |
| Sensor DTOs | `samtruck/sensors.py`: `RadarMeasurement`, `OdometryMeasurement`, and `VehicleSensorInput`. | Radar range and odometry speed can be noisy; radar can drop out; position, acceleration, and radar relative speed are ideal/pass-through. |
| Twin | `samtruck/twin_runtime.py`: `PlatoonTwin` keeps bounded `deque` histories of `EstimatedVehicleState`. | It assembles measurement-derived records and flags missing/stale data, but does not predict, filter, or fuse V2X into a numerical state. |
| Communication | `samtruck/v2x.py`: `PendingV2XChannel`. | Timestamped queue, packet loss, sender/receiver/sequence/staleness validation, and deterministic seed already exist for the passive Twin branch. |
| Baseline CACC | `samtruck/controller.py`: `CACCController.command()`. | Must remain unchanged.  It retains its own inline communication/sensing path in `run_all_scenarios.py`; the Twin has no actuation path. |
| Phase 1.1 evidence | `docs/PHASE_1_1_CORRECTION_REPORT.md`. | The passive Twin branch receives only post-delivery, validated V2X DTOs.  `PendingV2XChannel` is authoritative for that branch, not for legacy CACC. |

Existing tests in `tests/` cover controller determinism, sensor DTO behavior, Twin history/flags, V2X queue validation, and rejection of non-DTO Twin input.  They do not yet test a numerical estimator or estimation accuracy.

### Current live data flow

```text
SUMO/TraCI after simulationStep()
   ├─ build_twin_inputs_and_truth()
   │    ├─ SensorInterface -> VehicleSensorInput DTOs -> PlatoonTwin.update()
   │    └─ ground-truth rows -> twin_ground_truth.csv (evaluation only)
   ├─ Twin V2X payload -> PendingV2XChannel -> delivered/validated V2XMessage
   │                                              -> PlatoonTwin.update()
   └─ legacy inline sensing/V2X -> CACCController -> SUMO vehicle commands
```

The Twin imports neither TraCI nor SUMOlib.  `PlatoonTwin.update()` type-checks `VehicleSensorInput` and `V2XMessage`.  This boundary must be retained.

## 2. Guide compatibility and scope decision

The revised implementation guide's Phase 2 proposes four ideas: message validation, stale-data extrapolation, optional Kalman gap filtering, and brake-health estimation.  Message validation and stale-message rejection are already implemented in `PendingV2XChannel`, so Phase 2.1 must reuse rather than duplicate them.

The guide's simple one-dimensional gap filter is a useful teaching starting point, but it is insufficient as the primary estimator here: it cannot estimate own longitudinal state, cannot retain uncertainty for speed/acceleration, and can accidentally conflate a radar cut-in target with the V2X platoon predecessor.  Phase 2.1 should retain the guide's transparent constant-acceleration principle while using a small linear Kalman filter for the ego state and separate target tracks.

Brake-health estimation requires commanded acceleration and a verified measurement model.  It is not required by this prompt's stated Phase 2 estimator objective and should remain a separately approved extension after the passive state estimator is validated.

## 3. Current limitations requiring estimation design

1. **No estimator:** `PlatoonTwin._estimate()` copies the latest odometry and radar values into `EstimatedVehicleState`; it does not propagate state through a missed measurement or quantify uncertainty.
2. **Non-continuous position:** the runner supplies `traci.vehicle.getLanePosition()` as `OdometryMeasurement.position_m`.  That coordinate is local to a lane/edge and can reset at lane, edge, junction, and route transitions.  It is not safe as a continuous longitudinal state.
3. **Target identity ambiguity:** `RadarMeasurement.sensed_leader_id` can be a cut-in vehicle, while `VehicleSensorInput.platoon_predecessor_id` and the V2X sender identify the assigned platoon predecessor.  A single `estimated_gap_m` must not imply these are the same object.
4. **V2X state is not fused:** accepted messages are retained to compute age only.  Their speed and acceleration do not update a state estimate, which is correct for Phase 1 but incomplete for Phase 2.
5. **Ideal channels:** position, acceleration, and radar relative speed are passed through unchanged.  They still need explicit non-zero uncertainty assumptions in a filter; zero covariance would claim unrealistic perfect knowledge.
6. **Passive Twin V2X payload provenance:** the current runner captures V2X speed/acceleration from TraCI at the simulation boundary.  It is a typed communication DTO and does not enter CACC, but Phase 2.1 should decide whether transmitted state represents the sender's local odometry/estimate rather than a simulator-perfect value.  This is needed to make the communication model semantically consistent with strict measurement-only estimation.

## 4. Recommended initial estimator

### Decision: linear Kalman filter (KF), with a separate relative-target filter

Use a **per-truck linear constant-acceleration KF** for own longitudinal state.  Use a separate, identity-keyed relative-target filter for radar observations.  This is recommended over:

- **Alpha-beta filter:** simpler, but has no explicit acceleration state or full covariance and is less suitable for uncertainty-aware future prediction/control.
- **EKF:** not justified initially; the proposed vehicle and direct odometry measurements are linear.  Introducing an EKF would add complexity without an identified nonlinear measurement model.

The KF is transparent, deterministic, lightweight for three trucks, supports missing observations naturally, and exposes covariance for later risk and adaptive-control work.  It is not an AI controller and does not influence the current CACC baseline.

### 4.1 Own-vehicle state and motion model

For each truck, maintain the estimated state in the truck's forward travel coordinate:

\[
\mathbf{x}_k = [s_k,\ v_k,\ a_k]^T
\]

where `s` is continuous locally estimated travelled distance (m), `v` is longitudinal speed (m/s), and `a` is longitudinal acceleration (m/s²).

For the actual elapsed Twin timestamp `dt = t_k - t_(k-1)`, use:

\[
\mathbf{x}_{k|k-1}=F(dt)\mathbf{x}_{k-1|k-1},\quad
F(dt)=\begin{bmatrix}
1&dt&0.5dt^2\\
0&1&dt\\
0&0&1
\end{bmatrix}
\]

Use an integrated white-jerk process-noise model, with configurable jerk spectral-density `q_j`:

\[
Q(dt)=q_j\begin{bmatrix}
dt^5/20&dt^4/8&dt^3/6\\
dt^4/8&dt^3/3&dt^2/2\\
dt^3/6&dt^2/2
\end{bmatrix}.
\]

`dt` must be calculated from measurement timestamps, not hard-coded to the controller's current `0.1`-second update assumption.  A zero or negative timestamp interval must not run a predict step; it should be recorded as invalid/out-of-order.  Long measurement gaps should predict forward while increasing covariance, with a documented maximum horizon/status rather than silently presenting an old state as current.

### 4.2 Measurement model

Perform only updates for valid measurements present at the current timestamp.  Use a row-selected linear measurement model:

| Measurement | Proposed observation | H row | Current availability |
| --- | --- | --- | --- |
| Continuous odometry distance | `s_odo` | `[1, 0, 0]` | Must replace lane-local position in Phase 2.1. |
| Odometry speed | `v_odo` | `[0, 1, 0]` | Noisy when `odometry_speed_noise_std_mps > 0`. |
| Odometry acceleration | `a_odo` | `[0, 0, 1]` | Currently ideal/pass-through when supplied. |

For each update, build `R` only for available rows.  `R` must be explicitly configured for continuous odometry distance, speed, and acceleration.  The known configured speed-noise standard deviation can initialize the speed variance.  Position and acceleration need conservative declared variances even while the present simulator adapter passes them through.  Numerical values require calibration/approval; they must not be inferred as zero merely because the current adapter is ideal.

Initialize a filter from the first valid odometry observation:

\[
\hat{x}_0=[s_{odo},v_{odo},a_{odo}]^T
\]

using a configurable diagonal `P_0`.  If acceleration is absent, initialize it to zero with a large acceleration variance and mark the state as partially initialized.

### 4.3 Radar target state (separate from own state)

Maintain a track per `sensed_leader_id`, not one ambiguous global leader field.  For a target track:

\[
\mathbf{r}_k=[d_k,\ \Delta v_k]^T,
\]

where `d` is positive range and `Δv = v_ego - v_target`; positive relative speed means closing, matching `RadarMeasurement`.

A transparent initial prediction is:

\[
\mathbf{r}_{k|k-1}=\begin{bmatrix}1&-dt\\0&1\end{bmatrix}\mathbf{r}_{k-1|k-1},
\]

with configurable process covariance, and an observation `z_r=[range, relative_speed]^T` when those radar fields are valid.  Missing relative speed simply omits the second row.  A radar dropout/no target performs no measurement update; it must set a target-track degraded or predicted-only status.

Radar range must **not** update the assigned predecessor's V2X track unless `radar.sensed_leader_id == platoon_predecessor_id`.  A cut-in therefore creates/updates a separate target track and remains observable as such.

## 5. Coordinate-system decision

### Decision: use continuous local odometry distance for live estimation

Do not use current `getLanePosition()` as the KF's absolute longitudinal coordinate.  It is lane-local and discontinuous at exactly the map events that matter for this project: lane changes, junction connections, edge transitions, and an endless route loop.

In Phase 2.1, `SensorInterface` should own a per-vehicle **continuous odometry distance** (`s_odo`) that is initialized at vehicle appearance and propagated from timestamped odometry speed, for example:

\[
s_{odo,k}=s_{odo,k-1}+0.5(v_{odo,k-1}+v_{odo,k})dt.
\]

This is a sensor-side dead-reckoning measurement, so drift is expected and represented in covariance.  It avoids passing evaluator route progress or raw TraCI lane position into the Twin.  `lane_id`/`road_id` should be carried as non-numerical context metadata for reset/transition diagnostics, not used to reset `s_odo`.

If an absolute route-progress measurement is required later, it must be introduced as an explicitly modelled, map-matched sensor DTO at the boundary with its own validity and covariance.  It must not reuse `twin_ground_truth.csv` or evaluator-only cumulative distance.  Global X/Y should remain independent visualization/evaluation data unless a future localization sensor is designed.

## 6. V2X identity, delay, and fusion rules

`PendingV2XChannel` already validates sender, receiver, timestamp, staleness, and per sender/receiver sequence.  Phase 2.1 should keep it authoritative and add no second delivery queue or second validator.

For each accepted message:

1. Keep identity keys separate: `(receiver_id, expected_predecessor_id)` for the platoon link and `sensed_leader_id` for radar target tracks.
2. Fuse a message only into a state/track associated with `message.sender_id`; never use a predecessor message to label or update a cut-in radar object.
3. Compute age using `t_now - message.sim_time`, preserving the generation-time payload and scheduled-delivery semantics.
4. Do not treat an old message as a current measurement.  For a fresh delivered message, propagate its speed/acceleration forward from generation time using the same constant-acceleration model and increase uncertainty according to elapsed age.  Messages beyond the configured stale threshold remain rejected by the channel.
5. Do not overwrite a newer accepted estimate with duplicate/out-of-order sequence data.  Existing queue validation enforces this before Twin input.

Current `V2XMessage` contains speed and acceleration but no position.  Phase 2.1 can initially use it as a delayed predecessor speed/acceleration observation.  If predecessor position is needed later, add an explicitly timestamped position measurement captured from the sender's local odometry/estimate at generation time—not read from evaluation telemetry at delivery time.

## 7. Proposed estimated-state interface

Retain `EstimatedVehicleState` as the Phase 1 compatibility record.  Phase 2.1 should introduce a distinct filtered-state object, for example `FilteredVehicleState` in `samtruck/estimation.py`, rather than changing legacy CSV semantics without a migration.

Recommended fields:

| Group | Fields |
| --- | --- |
| Identity/time | `timestamp_s: float`, `vehicle_id: str`, `sequence_index: int` |
| Own-state estimate | `position_m: float | None`, `speed_mps: float | None`, `acceleration_mps2: float | None` |
| Uncertainty | immutable `covariance_3x3`, `position_std_m`, `speed_std_mps`, `acceleration_std_mps2` |
| Assigned platoon relation | `platoon_predecessor_id: str | None`, `predecessor_v2x_sender_id: str | None`, `v2x_message_age_s: float | None`, `v2x_fresh: bool` |
| Radar target relation | `radar_target_id: str | None`, `estimated_range_m: float | None`, `estimated_relative_speed_mps: float | None`, target covariance/standard deviations |
| Measurement health | `odometry_valid`, `radar_valid`, `v2x_valid`, `measurement_missing`, `estimator_status` |
| Provenance | `state_source`/update mask such as `predicted`, `odometry_update`, `radar_update`, `v2x_assisted` |

`estimator_status` should distinguish at least `uninitialized`, `initialized`, `predicted_only`, `updated`, `degraded`, and `invalid_timestamp`.  Missing data must remain `None` or explicitly flagged—not encoded as zero.

## 8. Strict ground-truth separation

```text
                   LIVE ESTIMATION PATH (no evaluator objects)
SUMO/TraCI -> simulation boundary -> SensorInterface DTOs -> estimator / PlatoonTwin
                         |                    |
                         |                    +-> local noisy/dead-reckoned measurements only
                         +-> V2X generation -> PendingV2XChannel -> validated delivered DTOs

                   OFFLINE EVALUATION PATH
SUMO/TraCI -> build_twin_inputs_and_truth() truth rows -> twin_ground_truth.csv
                                              +-> later offline estimator accuracy metrics
```

The estimator module and `twin_runtime.py` must not import TraCI/SUMOlib or read CSV evaluation rows.  No live prediction may use future trajectory data.  Ground truth can be joined with filtered logs only after a run for offline MAE/RMSE analysis.  The existing CACC path must continue to bypass Twin estimates until a separately approved adaptive-controller phase.

## 9. Phase 2.1 validation plan

Add deterministic unit tests before broad scenario testing:

1. Constant-speed and constant-acceleration state propagation with timestamp-derived `dt`.
2. Valid odometry update reduces covariance; predict-only step increases it.
3. Radar range/relative-speed update respects the positive-closing sign convention.
4. Radar dropout, no target, missing odometry, and invalid timestamp remain explicit and never substitute ground truth.
5. A delayed fresh V2X message is propagated from its generation timestamp with uncertainty growth.
6. Packet loss and stale, duplicate, and out-of-order messages do not update a target/predecessor estimate.
7. A radar cut-in ID and predecessor V2X sender ID produce separate tracks.
8. Lane/edge transition metadata does not reset continuous local `s_odo`.
9. Seeded sensor/V2X runs remain reproducible.
10. A static source-boundary test confirms estimator/Twin modules do not import TraCI/SUMOlib or evaluation logger code.
11. Existing controller tests still pass unchanged; an identical-input CACC regression remains byte-for-byte external to the estimator.

Offline metrics, calculated only by joining independently logged truth and filtered output after the run:

- position, speed, and acceleration MAE/RMSE;
- radar relative-range and relative-speed error;
- error versus measurement age/dropout duration/V2X latency;
- covariance consistency, e.g. fraction of truth errors within 1σ/2σ/3σ where the coordinate is comparable;
- recovery time from dropout, without claiming a controller improvement.

The first integration checks should be short, headless fixed-seed runs for baseline, braking, V2X latency/loss, lane/edge transitions, and a cut-in.  Physical CACC metrics must remain unchanged because estimator output is passive.

## 10. Phase 2.1 file-by-file implementation plan (not executed)

| File | Planned change | Constraint |
| --- | --- | --- |
| `samtruck/estimation.py` | New linear own-state KF, identity-keyed relative-target track, covariance/status DTOs, timestamp validation helpers. | No TraCI/SUMOlib; no controller imports. |
| `samtruck/sensors.py` | Add continuous sensor-side odometry distance and optional lane/road context metadata; preserve existing defaults. | Do not derive state from evaluator CSV. |
| `samtruck/twin_runtime.py` | Own estimator instances, feed only DTOs/delivered V2X, retain bounded histories, expose filtered output. | No CACC/actuation path. |
| `samtruck/digital_twin.py` | Add a compatible filtered-state data model only if necessary. | Preserve existing `VehicleState`, `EstimatedVehicleState`, and `time_gap()` compatibility. |
| `samtruck/v2x.py` | Reuse existing `PendingV2XChannel`; optionally extend the DTO with an explicitly timestamped sender-local position measurement after approval. | No second queue; keep timing/validation behavior. |
| `run_all_scenarios.py` | Wire estimator output/logging alongside current Twin logs. | Preserve CACC invocation, controller inputs, events, routes, and scenario JSON. |
| `samtruck/config.py` | Add explicit estimation covariance/noise configuration only after parameter approval. | Do not alter current CACC CTG/min-gap values. |
| `tests/test_estimation.py` and focused existing tests | Add deterministic estimator and boundary tests. | Retain all current coverage. |

## 11. Risks, assumptions, and decisions needing approval

1. **Noise/covariance calibration:** no measured sensor specification exists.  Phase 2.1 needs approved initial `P0`, process jerk variance, and non-zero position/acceleration measurement variances; they should be labelled assumptions, not validated physical values.
2. **Continuous coordinate:** this design recommends dead-reckoned local `s_odo`.  Approve this as the Phase 2 baseline, or provide a map/localization requirement for a route-progress sensor.
3. **V2X payload semantics:** decide whether Phase 2.1 may change the passive Twin V2X payload from simulator-perfect boundary scalars to sender-local odometry/estimated fields.  That is recommended for measurement realism, but must leave the legacy CACC branch untouched.
4. **Delayed-message policy:** recommend forward propagation with covariance inflation for bounded fresh messages, not out-of-sequence history rewind/replay in the initial implementation.  This is simpler and deterministic; OOSM smoothing can be evaluated later.
5. **Configuration ambiguity remains:** `run_all_scenarios.py`/`scenario_suite.py` use CACC CTG `0.8 s`, min gap `10 m`, while `montreal_demo.py` uses `1.2 s`, `4 m` and `SimulationConfig` defaults align with the demo.  Phase 2.1 must not harmonize these without identifying an approved benchmark configuration.

## 12. Future compatibility

Phase 3 prediction can consume timestamped filtered states and covariance, while offline SUMO truth may generate training labels only outside the live path.  Phase 4 can add a separate adaptive-controller experiment that consumes the filtered state.  The present `CACCController` remains the unmodified baseline for any comparison.

## Recommendation

Proceed with Phase 2.1 only after approving the continuous-odometry coordinate and initial covariance assumptions.  Implement the small linear KF plus separate identity-keyed radar target tracks first, keep the Twin passive, and validate against independently logged ground truth before exposing estimates to any controller.
