# Phase 2.2C — Controlled Sensor and V2X Degradation Experiments Report

## 1. Objective and scope

Phase 2.2C establishes reproducible, controlled degradation experiments for the **passive Digital Twin**.  The objective is to measure estimator behavior when Twin sensor or Twin V2X information becomes imperfect, while preserving the official CACC baseline.

All work was completed in `E:\SAMSUNGTRUCK`.  No files were inspected or changed in `E:\TruckPlatooning`.

## 2. Stage 1 audit and architecture decision

The following were inspected before implementation: `samtruck/sensors.py`, `samtruck/v2x.py`, `samtruck/estimation.py`, `samtruck/twin_runtime.py`, `samtruck/config.py`, both offline evaluators, `run_all_scenarios.py`, catalog scenarios, tests, and Phase 2.2A/2.2B reports.

### Verified existing capabilities

- `SensorInterface` already supports deterministic radar-range noise, odometry-speed noise, and random radar dropout with a local seeded RNG.
- `PendingV2XChannel` already supports timestamped delivery, packet loss, outage, validation, stale-message checks, and deterministic seeded outcomes.
- `PlatoonTwin` already converts invalid odometry into absent Kalman speed/acceleration measurements; this preserves component timestamps and causes prediction-only operation.
- The Phase 2.2A.1 and Phase 2.2B evaluators operate only after CSV output is complete and retain exact normalized `(vehicle_id, simulation_time_s)` matching.

### Isolation issue and resolution

The legacy `v2x_state()` in `run_all_scenarios.py` affects the inline CACC V2X path and was also previously used to configure the Twin queue.  Reusing catalog V2X events for Phase 2.2C would therefore alter CACC behavior.

The implementation adds opt-in `twin_only_degradations`.  When present, its V2X schedule is used only for `PendingV2XChannel`; the CACC path continues using the unmodified legacy `v2x_state()` result.  When absent, the existing behavior remains unchanged.  This keeps official catalog semantics intact and makes Phase 2.2C experiments passive-Twin-only.

## 3. Changes made

| File | Change |
| --- | --- |
| `samtruck/twin_degradation.py` | New deterministic DTO-only Twin degradation schedule for timed radar suppression, timed odometry suppression, and Twin-only V2X latency/loss/outage. |
| `run_all_scenarios.py` | Applies the opt-in schedule only after sensor DTO construction and only to the passive Twin branch. Writes `twin_degradation_summary.json` post-run. Existing CACC invocation is unchanged. |
| `scenarios/experiments/phase_2_2c.json` | New controlled 23-run experiment matrix, separate from the official catalog. |
| `run_degradation_experiments.py` | New matrix runner and `--summarize-existing` mode for rebuilding a consolidated report without rerunning SUMO. |
| `tests/test_twin_degradation.py` | New deterministic unit tests for radar/odometry loss, Twin-only V2X isolation/recovery, and configuration validation. |
| `tests/test_estimation.py` | Added covariance recovery regression test after prediction-only operation. |

## 4. Experiment matrix and reproducibility

The matrix uses `normal_platooning`, 12-second runs, fixed seeds `20260929` and/or `20260930`, and no extra legacy CACC V2X event.  It includes:

- baseline;
- low/medium/high sensor noise: radar range standard deviation `0.25/0.75/1.50 m` and odometry-speed standard deviation `0.10/0.30/0.60 m/s`;
- radar dropout windows of `1/3/6 s`;
- own-odometry dropout windows of `1/3/6 s`;
- Twin-only V2X latency of `200/600 ms`;
- Twin-only packet-loss probabilities of `20/30/40%`;
- a 4-second Twin-only V2X outage and recovery.

Noise levels are controlled simulation assumptions, not claimed hardware specifications.  Each result folder records seed, requested sensor settings, event windows, applied sensor-suppression counters, and V2X channel counts.  Consolidated output:

```text
results/phase_2_2c_experiments_summary.json
```

All **23/23** configured runs have a persisted `metrics.json` verdict of `PASS`; the final summary reconstruction now reads this original artifact rather than assuming a folder is passed.

## 5. Evidence that degradation was actually applied

The following observed counts are from the generated per-run summaries.

| Example run | Applied evidence |
| --- | --- |
| `p22c_radar_short_s20260929` | 20 radar measurements suppressed. |
| `p22c_radar_medium_s20260929` | 60 radar measurements suppressed. |
| `p22c_radar_extended_s20260929` | 120 radar measurements suppressed. |
| `p22c_odometry_short_s20260929` | 30 odometry measurements suppressed. |
| `p22c_odometry_medium_s20260929` | 90 odometry measurements suppressed. |
| `p22c_odometry_extended_s20260929` | 180 odometry measurements suppressed. |
| `p22c_v2x_loss_20_s20260929` | 240 generated, 208 delivered, 30 packet-loss drops. |
| `p22c_v2x_loss_30_s20260929` | 240 generated, 195 delivered, 43 packet-loss drops. |
| `p22c_v2x_loss_30_s20260930` | 240 generated, 209 delivered, 29 packet-loss drops. |
| `p22c_v2x_loss_40_s20260929` | 240 generated, 184 delivered, 54 packet-loss drops. |
| `p22c_v2x_outage_s20260929` | 80 outage attempts and 80 outage drops; recovery delivered messages afterwards. |
| `p22c_v2x_latency_600_s20260929` | 230 delivered and 8 rejected duplicate/out-of-order messages after delayed-message reordering; no fabricated delivery. |

Requested loss probabilities are not claimed as exact finite-run loss rates.  The counts above are the realised outcomes.  Some last generated packets remain pending at a bounded run end; this explains baseline `240 generated / 238 delivered` rather than being packet loss.

## 6. Estimator/uncertainty results

Existing `twin_filtered_evaluation.json` and `twin_uncertainty_evaluation.json` are generated for every run.  They preserve speed/acceleration MAE, RMSE, covariance consistency, coverage, matching diagnostics, component provenance, and measurement-age grouping.

Representative seed `20260929` results:

| Run | Speed MAE (m/s) | Speed mean normalized squared error | Speed updated / prediction-only | Speed fresh / aged | Acceleration MAE (m/s²) | Acceleration prediction-only |
| --- | ---:| ---:| ---:| ---:| ---:| ---:|
| baseline | 0.15 | 0.26 | 360 / 0 | 360 / 0 | 0.62 | 0 |
| high sensor noise | 0.23 | 0.52 | 360 / 0 | 360 / 0 | 0.64 | 0 |
| medium odometry loss | 0.54 | 0.50 | 270 / 90 | 270 / 90 | 0.93 | 90 |
| extended odometry loss | 2.42 | 0.98 | 180 / 180 | 180 / 180 | 1.36 | 180 |

For `p22c_odometry_medium_s20260929`, all 90 window rows were speed and acceleration `prediction_only`; maximum observed component age was `3.0 s`, and post-window rows returned to `measurement_updated`.  This confirms that missing own measurements do not refresh timestamps and that recovery is recorded.

Radar loss is represented as missing radar DTO data—not zeros—and preserves the radar tracker’s separate missing/prediction behavior.  It does not directly change the own-state speed/acceleration Kalman update because that filter uses odometry, not radar.

V2X degradation changes Twin communication freshness and channel statistics, but does not directly change own-state Kalman covariance in this phase because predecessor V2X is not fused into the own-state filter.  This is an architectural fact, not an experimental failure.

## 7. Tests and validation

| Command | Result |
| --- | --- |
| `python -m unittest tests.test_twin_degradation -v` | PASS — 5 tests |
| `python -m unittest tests.test_sensors -v` | PASS — 2 tests |
| `python -m unittest tests.test_v2x -v` | PASS — 7 tests |
| `python -m unittest tests.test_estimation -v` | PASS — 28 tests |
| `python -m unittest tests.test_twin_estimator_integration -v` | PASS — 13 tests |
| `python -m unittest discover -s tests -v` | PASS — 83 tests |
| `python run_all_scenarios.py --scenario normal_platooning --fast-demo --seed 20260929` | PASS |

The official smoke test generated the normal output/evaluation files successfully after the Phase 2.2C changes.

## 8. Preservation confirmation

- `samtruck/controller.py`, CACC gains, CTG, controller inputs, and actuation: **not modified**.
- Kalman equations, covariance tuning, radar tracking, `SensorInterface`, and `PendingV2XChannel`: **not modified**.
- Official scenario catalog and SUMO configuration/network: **not modified**.
- Phase 2.2A.1 accuracy evaluator matching and metrics: **not modified**.
- Ground-truth boundary: **preserved**.  The Twin sees only degraded DTOs and delivered/validated Twin V2X messages; evaluation truth remains separate.

## 9. Limitations and recommended next step

- Two seeds and short fixed-route runs establish deterministic repeatability, not statistical calibration across traffic populations.
- The 600 ms latency case shows delayed-message reordering can cause duplicate/out-of-order validation rejections.  This is correctly logged and should be analyzed before any future communication-policy change.
- Position uncertainty remains excluded due to lack of coordinate-compatible independent truth.
- V2X robustness currently measures communication freshness/logging rather than V2X-aided own-state estimation, because V2X is not fused into the own-state Kalman filter.

Phase 2.2C is complete as a controlled passive-Twin robustness evaluation.  A future Phase 2.2D should be separately approved to analyse repeated-run confidence intervals and/or design valid V2X measurement fusion; it must not tune covariance or alter CACC without a dedicated experiment plan.

## 10. Final consistency audit

The final audit corrected `run_degradation_experiments.py --summarize-existing` so it no longer hardcodes every reconstructed run as passed.

- It reads `results/<run_id>/metrics.json` and accepts only `pass_fail.status == "PASS"` or `"FAIL"` as an original run verdict.
- Missing, malformed, missing-status, or unrecognized-status artifacts are recorded as `pass: null` with an explicit `pass_status_source`; they are never inferred as pass.
- Missing degradation or uncertainty output is represented as `result_completeness: "incomplete"`, without rerunning SUMO.
- The final reconstruction of the existing matrix found 23 runs, 23 persisted passes, zero fails, zero unknowns, and zero incomplete results.

The Phase 2.2C design was synchronized to `scenarios/experiments/phase_2_2c.json`: 12.0-second duration; 3.0-second event start; actual 1/3/6-second sensor windows, 6-second latency/loss windows, 4-second outage window; and the exact one- versus two-seed allocation.
