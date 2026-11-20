# SAMTRUCK Phase 2.1A — Kalman Filter Estimator Implementation Report

## Scope and result

Phase 2.1A implements a standalone, passive numerical estimation module.  It is **not** connected to `run_all_scenarios.py`, SUMO, vehicle actuation, CACC, or scenario definitions.  No packages were installed; NumPy `2.5.3` was already available.

The implementation follows the Phase 2.0 design and the compatible Phase 2 guidance in the revised implementation guide.  The guide's simple gap-filter concept was extended to a small, transparent own-vehicle constant-acceleration filter plus separate identity-keyed radar tracks.  Existing `PendingV2XChannel` validation/timing is reused conceptually and was not changed or duplicated.

## Files created and modified

| File | Change |
| --- | --- |
| `samtruck/estimation.py` | **Created.** Standalone Kalman estimator, relative radar tracker, delayed-V2X propagation helper, typed immutable outputs, and local configuration dataclasses. |
| `tests/test_estimation.py` | **Created.** Deterministic estimator, radar, V2X-compatibility, numerical, and boundary tests. |
| `docs/PHASE_2_1A_IMPLEMENTATION_REPORT.md` | **Created.** This report. |

No existing source file was modified.  In particular, `samtruck/controller.py`, `run_all_scenarios.py`, `samtruck/v2x.py`, `samtruck/twin_runtime.py`, `samtruck/sensors.py`, `samtruck/config.py`, and scenario files were left unchanged.

## Phase 2.1A.1 estimator review corrections

This narrowly scoped correction pass modified only `samtruck/estimation.py`, `tests/test_estimation.py`, and this report.  It did not begin Phase 2.1B.

### V2X acceleration-process-variance validation

`propagate_predecessor_v2x()` now validates `acceleration_process_variance` before using it in the unchanged covariance equations.  It accepts finite values greater than or equal to zero.  Negative values, `NaN`, `+∞`, and `-∞` raise:

```text
ValueError: acceleration_process_variance must be finite and non-negative.
```

Zero remains an intentional valid boundary: it preserves the existing propagation equations while adding no age-dependent process uncertainty.  Tests cover each rejected value and the zero boundary.  No V2X timing, message delivery, timestamp, covariance formula, or return-status behavior was changed.

### Corrected integrated white-jerk process covariance

`VehicleStateEstimator.process_covariance()` now implements the complete specified covariance:

\[
Q(dt)=q_j\begin{bmatrix}
dt^5/20&dt^4/8&dt^3/6\\
dt^4/8&dt^3/3&dt^2/2\\
dt^3/6&dt^2/2&dt
\end{bmatrix}.
\]

The final diagonal term was corrected from `dt²` to `dt`.  Focused tests evaluate exact matrices at `dt=0.5` and `dt=2.0`, confirm the `Q[2,2]` linear timestep scaling, verify symmetry/positive-semidefinite behavior, and verify prediction-only covariance growth.

### Radar initialization provenance and freshness

Partial radar initialization is still mathematically valid: an absent state component uses a zero mean with a deliberately large configured prior covariance.  It is no longer presented as an observation.

`RelativeTargetState` now retains backward-compatible existing fields and adds defaulted metadata fields:

- `observed_components`: components in the last actual radar update;
- `unobserved_prior_components`: components that still only have an initialization prior;
- `last_measurement_timestamp_s`: timestamp of the last true radar measurement;
- `prediction_timestamp_s`: timestamp of the current stored filter state;
- `track_age_s`: time since the last actual measurement;
- `last_update_was_measurement`: whether the current state resulted from a radar update rather than prediction-only propagation.

Range-only initialization is marked `initialized_range_only_prior_relative_speed`; relative-speed-only initialization is marked `initialized_relative_speed_only_prior_range`.  `RadarTargetTracker.state()` now returns `latest_measurement` only when the latest stored state genuinely came from radar data.  A state advanced through `predict_missing()` is returned as `prediction_only` with `radar_valid=False`, its old measurement timestamp, and a nonzero track age.

## Estimator architecture

### Own-vehicle filter

`VehicleStateEstimator` owns a `LinearKalmanFilter` for one vehicle using:

\[
x=[s,v,a]^T
\]

where `s` is longitudinal position (m), `v` speed (m/s), and `a` acceleration (m/s²).  It uses the supplied timestamps to compute `dt`, then predicts with:

\[
F(dt)=\begin{bmatrix}1&dt&0.5dt^2\\0&1&dt\\0&0&1\end{bmatrix}
\]

and the integrated white-jerk process covariance:

\[
Q(dt)=q_j\begin{bmatrix}
dt^5/20&dt^4/8&dt^3/6\\
dt^4/8&dt^3/3&dt^2/2\\
dt^3/6&dt^2/2&dt
\end{bmatrix}.
\]

The update is dynamically constructed from only valid supplied rows:

- independent position: `[1, 0, 0]`;
- speed: `[0, 1, 0]`;
- acceleration: `[0, 0, 1]`.

`LinearKalmanFilter` uses the Joseph covariance form and symmetrizes/checks covariance after prediction and update.  Output is the immutable `FilteredVehicleState`, which provides the state, 3x3 covariance, standard deviations, update mask, and status.

### Initialization and missing data

Initialization requires a finite speed measurement.  Position initializes from an explicitly independent position measurement when available; otherwise it begins at `0 m` with very large position uncertainty.  Acceleration initializes from a valid acceleration measurement, otherwise at `0 m/s²` with large acceleration uncertainty and a `partially_initialized` status.

Missing measurements are never converted to zero.  Prediction-only output is marked `predicted_only`, and a gap longer than the configurable `long_gap_s` is marked `degraded_long_gap_predicted` or `degraded_long_gap_updated`.  Negative timestamps return `invalid_timestamp` without advancing the filter.  Same-timestamp measurement updates are allowed without a prediction and are marked `updated_same_timestamp`.

### Noise and covariance assumptions

`EstimatorConfig` contains local Phase 2.1A defaults:

- white-jerk process variance: `1.0`;
- position measurement variance: `4.0 m²`;
- speed measurement variance: `1.0 (m/s)²`;
- acceleration measurement variance: `4.0 (m/s²)²`;
- deliberately large unobserved initial-position variance: `1,000,000 m²`;
- long-gap threshold: `1.0 s`.

These are simulation assumptions, not calibrated truck sensor specifications.  They are intentionally local to `samtruck/estimation.py`; `SimulationConfig` was not changed.  Phase 2.1B must determine how validated estimator assumptions should be configured centrally without altering CACC parameters.

### Double-counting prevention

The `independent_position_m` argument is deliberately named to prevent accidental fusion of speed-integrated odometry distance with the same speed measurement as if they were independent.  A dead-reckoned position can be used as a diagnostic or prediction aid in a later integration step, but must not be passed through the independent-position update row without a correlation-aware sensor model or a truly independent localization measurement.

## Separate radar target tracking

`RadarTargetTracker` keeps a distinct two-state filter for every `target_vehicle_id`:

\[
r=[d,\Delta v]^T
\]

where range `d` is positive and `Δv = v_ego - v_target`; positive relative speed means the ego truck is closing.  Its motion model is:

\[
F_r(dt)=\begin{bmatrix}1&-dt\\0&1\end{bmatrix}.
\]

It supports range-only, relative-speed-only, joint updates, explicit target disappearance (`predict_missing()`), timestamp rejection, covariance propagation, and degraded long-gap states.  The target ID is mandatory, so a radar cut-in target and a platoon predecessor remain separate tracks.

## V2X handling and limitation

`propagate_predecessor_v2x()` is a pure compatibility helper.  It accepts an already delivered `V2XMessage`, calculates age from the original `message.sim_time`, propagates speed with constant acceleration, and inflates uncertainty with age.  It returns `stale` rather than treating expired information as current.

It deliberately does **not**:

- enqueue, validate, or replace `PendingV2XChannel`;
- fuse a V2X message into an own-state or radar track;
- alter CACC communication;
- add position to the current V2X payload.

This preserves the Phase 1.1 delivery/validation authority and avoids silently presenting simulator-perfect values as a measurement-estimate.  Delayed V2X fusion and sender-local payload semantics belong to Phase 2.1B.

## Ground-truth and CACC boundary

`samtruck/estimation.py` imports only NumPy and the typed `V2XMessage` DTO.  It imports no TraCI, SUMOlib, evaluator, scenario runner, or controller module.  It does not read ground-truth CSV data.  A static test enforces the simulator/evaluation import boundary.

The estimator has no SUMO integration, so it has no control-to-actuation path.  `CACCController` source was not modified; its post-test SHA-256 was recorded as:

```text
FE3170BABBB1CBF24F92AEA5E0A6885FAB52396B762445038127EC2893DF34FC
```

The existing CACC regression tests passed.

## Tests actually executed

Commands run:

```powershell
python -m unittest tests.test_estimation -v
python -m unittest discover -s tests -v
```

Results:

| Suite | Result |
| --- | --- |
| New Phase 2.1A estimator tests | **16/16 passed** |
| Full existing + new regression suite | **37/37 passed** |

After the Phase 2.1A.1 correction pass, the commands required by the review were executed again:

| Suite | Result |
| --- | --- |
| `python -m unittest tests.test_estimation -v` | **20/20 passed** |
| `python -m unittest discover -s tests -v` | **41/41 passed** |

For this validation-only follow-up:

| Command | Result |
| --- | --- |
| `python -m pytest tests/test_estimation.py -v` | **Could not run:** `pytest` is not installed in the active Python environment (`No module named pytest`). No package was installed. |
| `python -m unittest tests.test_estimation -v` | **22/22 passed** |
| `python -m unittest discover -s tests -v` | **43/43 passed** |

The tests cover constant speed/acceleration prediction, variable timestep, exact white-jerk covariance values/scaling, speed/acceleration/independent-position updates, missing data, NaN/invalid variance rejection, out-of-order and same-timestamp handling, covariance symmetry/PSD behavior, missing acceleration initialization, long-gap status, radar sign convention, range-only/relative-speed-only provenance, dropout/disappearance/freshness, identity-separated tracks, delayed/stale V2X treatment, no queue duplication, and no simulator/evaluation import.

No SUMO scenario was run in this phase because the estimator is intentionally not connected to the benchmark runner yet.

## Known limitations

1. The new estimator is standalone; `PlatoonTwin` does not yet own or log it.
2. The current `SensorInterface` supplies lane-local `position_m`.  It must not be fused as a continuous position until Phase 2.1B introduces a sensor-side dead-reckoning/localization design.
3. Sensor/process variances are documented simulation assumptions and require calibration.
4. Current V2X messages contain speed and acceleration but no sender-local position measurement.
5. The helper does forward propagation only; it does not implement out-of-sequence measurement rewind/smoothing.
6. Brake-health estimation, adaptive control, AI, and prediction remain out of scope.

## Exact Phase 2.1B next steps

1. Add a sensor-side continuous dead-reckoning coordinate or approved independent localization DTO; do not feed lane-local SUMO position as a continuous observation.
2. Integrate `VehicleStateEstimator` and `RadarTargetTracker` inside `PlatoonTwin` only, after `SensorInterface` DTO generation and after validated V2X delivery.
3. Add separate filtered-state logging alongside existing Twin and ground-truth logs; keep the evaluator branch separate.
4. Decide and document whether V2X payloads should carry sender-local odometry/estimated state, then implement delayed message fusion with age-based covariance inflation.
5. Run fixed-seed passive SUMO scenarios and compare filtered outputs with the independent ground-truth logs offline.
6. Keep CACC invocation, inputs, equations, gains, scenarios, and vehicle actuation unchanged.
