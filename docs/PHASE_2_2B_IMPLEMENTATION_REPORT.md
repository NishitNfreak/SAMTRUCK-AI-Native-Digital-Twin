# Phase 2.2B — Estimator Uncertainty and Covariance Validation Report

## 1. Objective

Phase 2.2B adds a strictly offline, reproducible check of whether the own-state Kalman filter's exported marginal covariance is consistent with independently logged SUMO truth for **speed** and **acceleration**.  The result is a covariance-consistency evaluation, not a claim that covariance is calibrated merely because it exists.

## 2. Stage 1: Phase 2.2A closure audit

The authoritative workspace `E:\SAMSUNGTRUCK` was verified before work.  Inspected files included:

- `samtruck/estimation.py`
- `samtruck/twin_runtime.py`
- `samtruck/offline_estimator_evaluator.py`
- `run_all_scenarios.py`
- `tests/test_estimation.py`
- `tests/test_twin_estimator_integration.py`
- `tests/test_offline_estimator_evaluator.py`
- `docs/PHASE_2_2A_1_IMPLEMENTATION_REPORT.md`
- `docs/PHASE_2_2A_2_MEASUREMENT_AGE_DESIGN.md`
- `docs/PHASE_2_2A_2_IMPLEMENTATION_REPORT.md`

### Phase 2.2A findings

- `VehicleStateEstimator` records own position/speed/acceleration timestamps only after the corresponding measurement row is accepted.
- Component ages are independent and use the accepted estimator timestamp.
- Missing initial acceleration remains a broad prior and is labelled `not_meaningfully_initialized`, not a measurement.
- Invalid odometry is converted to missing speed/acceleration in `PlatoonTwin._update_filtered_state()` and cannot refresh freshness metadata.
- The Twin receives sensor DTOs and delivered V2X DTOs only.  It does not import TraCI or consume evaluation truth.
- The Phase 2.2A.1 accuracy evaluator and its exact normalized timestamp-matching policy were not modified.

### Same-timestamp partial-update semantic edge case

In `VehicleStateEstimator._freshness_metadata()`, a previously measured component omitted from an `updated_same_timestamp` row receives `prediction_only` provenance.  This is consistent with the approved Phase 2.2A.2 design: that provenance includes a valid initialized component that is **retained without a fresh measurement** at the current timestamp.  It is explicitly not labelled fresh.

This does not change the Phase 2.2A.1 metric grouping: its existing evaluator deliberately classifies omitted components on `updated_same_timestamp` rows as `other_no_measurement_update`, because no time-advancing propagation occurred.  The two fields serve different purposes: per-component freshness telemetry versus established evaluation grouping.  No correction was required.

## 3. Architecture and changes

### Files changed

| File | Change |
| --- | --- |
| `samtruck/offline_uncertainty_evaluator.py` | New post-run-only scalar covariance-consistency evaluator. |
| `run_all_scenarios.py` | Writes a separate post-run `twin_uncertainty_evaluation.json` after existing CSVs and the unchanged Phase 2.2A.1 evaluation are complete. |
| `tests/test_offline_uncertainty_evaluator.py` | New deterministic evaluator tests. |
| `tests/test_estimation.py` | Added measurement-recovery covariance-reduction regression test. |

The new evaluator imports only standard-library modules and the established file-based matching helpers from `samtruck.offline_estimator_evaluator`.  It never imports TraCI, SUMOlib, a controller, `PlatoonTwin`, sensors, or V2X delivery.  The live simulation path remains:

```text
SUMO -> SensorInterface / delivered V2X -> passive Twin -> filtered CSV
SUMO -> independent ground-truth CSV
completed CSVs -> offline accuracy + uncertainty evaluation JSON
```

## 4. Available signals and mathematical definitions

`twin_filtered_states.csv` already contains scalar marginal variances:

- `speed_variance_m2ps2`
- `acceleration_variance_m2ps4`

It also contains `speed_estimate_provenance`, `acceleration_estimate_provenance`, and the corresponding measurement ages added in Phase 2.2A.2.  `twin_ground_truth.csv` independently contains `speed_mps` and `acceleration_mps2`.  Records are matched by exact normalized `(vehicle_id, simulation_time_s)` keys, with duplicate keys excluded exactly as in Phase 2.2A.1.

For a valid scalar pair:

```text
error = estimate - truth
sigma = sqrt(exported marginal variance)
standardized_error = error / sigma
scalar_normalized_squared_error = error^2 / variance
```

The evaluator reports MAE, RMSE, mean predicted standard deviation, mean absolute standardized error, mean scalar normalized squared error, and Pearson correlation between absolute error and predicted sigma.  It reports coverage for `|error| <= sigma`, `<= 2*sigma`, and `<= 1.959963984540054*sigma`.

For a calibrated independent Gaussian scalar reference, nominal coverage is approximately 68.27%, 95.45%, and 95%, respectively.  These are reference values, not pass/fail thresholds: sequential simulation rows are correlated and a single scenario is not a calibration study.

Invalid/non-finite/non-positive variances, invalid truth, and invalid estimates are counted and excluded; no substitute values are invented.

## 5. NIS, NEES, and exclusions

- **NIS:** unavailable.  Innovations and innovation covariance are not logged, so NIS cannot be calculated validly.
- **Full-state NEES:** unavailable.  The only position truth is lane-local, while the estimator position is unanchored; their coordinate frames are incompatible.
- **Scalar normalized squared error:** available for speed and acceleration using independently logged truth and marginal variance.  It is intentionally not presented as full-vector NEES.
- **Position/radar uncertainty calibration:** excluded for the same coordinate/identity-compatible-truth limitations.

## 6. Generated output

Every supported scenario run now also writes:

```text
results/<scenario>/twin_uncertainty_evaluation.json
```

It contains matching diagnostics, scalar speed/acceleration metrics, provenance groups (`measurement_updated`, `prediction_only`, `other_or_unavailable`), measurement-age groups (`fresh`, `aged`, `unavailable_age`), and explicit NIS/NEES availability declarations.

## 7. Validation results

### Automated tests

| Command | Result |
| --- | --- |
| `python -m unittest tests.test_offline_uncertainty_evaluator -v` | PASS — 6 tests |
| `python -m unittest tests.test_estimation -v` | PASS — 28 tests |
| `python -m unittest tests.test_twin_estimator_integration -v` | PASS — 13 tests |
| `python -m unittest discover -s tests -v` | PASS — 78 tests |

The new tests cover covariance extraction/math, confidence coverage, provenance/age grouping across a prediction gap and recovery, invalid truth/estimate/variance handling, duplicate and timestamp-mismatch exclusion, insufficient-sample status, no live-system imports, and covariance reduction after a measurement recovery.

### Official runner smoke test

Command executed:

```powershell
python run_all_scenarios.py --scenario normal_platooning --fast-demo --seed 20260929
```

Result: **PASS**.  Generated outputs include both the existing `twin_filtered_evaluation.json` (schema `2.2A.1`) and the new `twin_uncertainty_evaluation.json` (schema `2.2B`).  Both recorded 750 exact matched records, zero duplicates, zero unmatched records, and zero invalid values for this run.

| Metric | Speed | Acceleration |
| --- | ---:| ---:|
| Valid scalar pairs | 750 | 750 |
| Mean normalized squared error | 0.1664 | 1.2209 |
| 1-sigma coverage | 95.73% | 84.13% |
| 2-sigma coverage | 100.00% | 93.07% |
| 95% coverage | 100.00% | 92.67% |
| Absolute-error vs sigma correlation | -0.0578 | 0.0366 |

All 750 speed and acceleration rows were `measurement_updated` with age zero in this normal scenario; therefore it does not exercise the aged or prediction-only evaluator groups.  The values are descriptive for this one fixed-seed run only.  In particular, they do not establish a calibrated uncertainty model or demonstrate covariance behavior under sensor loss.

## 8. Preservation confirmation

- CACC/CTG controller logic, gains, inputs, actuation, and `samtruck/controller.py`: **not modified**.
- Kalman state transition, process-noise/measurement-noise equations, covariance mathematics, and tuning: **not modified**.
- Live Twin data boundary: **preserved**; no SUMO truth entered the Twin.
- Sensor behavior, radar behavior, V2X queue/validation/age/fusion: **not modified**.
- Scenario definitions, SUMO network/configuration, and benchmark configuration: **not modified**.
- Phase 2.2A.1 accuracy evaluator matching and metrics: **not modified**.

## 9. Limitations and Phase 2.2C recommendation

Phase 2.2B implementation is complete as an offline covariance-consistency capability.  It is not yet possible to claim calibrated uncertainty across faults because the validated normal run has no prediction-only rows.  Phase 2.2C should design and run fixed-seed, repeated sensor-dropout/measurement-loss experiments, report age-binned speed and acceleration consistency, and assess covariance versus squared error across independent runs.  It should not tune covariance or change the controller without a separate approved experiment plan.
