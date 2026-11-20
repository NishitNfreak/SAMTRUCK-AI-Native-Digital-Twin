# Phase 2.2C — Controlled Sensor and V2X Degradation Design

## 1. Audit findings

All findings below were inspected in the authoritative workspace `E:\SAMSUNGTRUCK`.

| Requirement | Existing capability | Decision |
| --- | --- | --- |
| Radar range noise | `SensorInterface(radar_noise_std_m=...)`, deterministic local RNG | Reuse through per-experiment `twin` configuration. |
| Odometry speed noise | `SensorInterface(odometry_speed_noise_std_mps=...)`, deterministic local RNG | Reuse through per-experiment `twin` configuration. |
| Radar dropout | `radar_dropout_probability`, explicit invalid `RadarMeasurement`, deterministic local RNG | Reuse for random dropout; add deterministic time-window suppression for short/medium/extended/recovery experiments. |
| Own odometry loss | No scenario-controlled availability switch. `PlatoonTwin` already treats invalid DTOs as missing measurements. | Add a Twin-only DTO suppression layer; it returns invalid/missing odometry, never zero. |
| Twin V2X latency/loss/outage | `PendingV2XChannel` supports latency, loss, outage, validation, and seeded RNG. | Reuse with a Twin-only schedule. |
| Legacy CACC V2X | `run_all_scenarios.v2x_state()` affects the inline CACC communication state. | Preserve unchanged. Existing catalog events retain their existing meaning. |
| Twin V2X in current runner | Shares the *scenario event state* with CACC but uses an independent pending queue/RNG. | Add a separate schedule only when an experiment explicitly requests `twin_only_degradations`; otherwise preserve current behavior. |
| Accuracy/uncertainty grouping | Offline evaluators consume finished CSVs; uncertainty evaluator already groups provenance and age. | Reuse unchanged. |

## 2. Safety boundary

The experiment path will be:

```text
SUMO/TraCI -> SensorInterface -> Twin-only degradation adapter -> passive Twin
SUMO/TraCI -> independent truth CSV -> offline evaluators
Twin-only V2X schedule -> PendingV2XChannel -> passive Twin
Legacy scenario V2X state -> existing CACC path -> actuation
```

The adapter operates only on already-created sensor DTOs at the runner boundary.  It does not pass truth into the Twin, does not alter the CACC call, and does not modify legacy scenario event handling.  `twin_only_degradations` is opt-in.  Omitted configuration preserves the existing runner behavior exactly.

## 3. Required minimal additions

1. `samtruck/twin_degradation.py`: a deterministic, DTO-only schedule that can:
   - suppress radar measurements during configured windows;
   - suppress own speed/acceleration/position DTO fields during configured windows;
   - resolve Twin-only V2X latency, loss probability, and outage state;
   - record requested/applied counters and event windows.
2. `run_all_scenarios.py`: apply the schedule to the Twin branch only when present, write degradation metadata/statistics, and use its V2X state for `PendingV2XChannel` only.
3. `scenarios/experiments/phase_2_2c.json`: experiment matrix separate from the official catalog.
4. `run_degradation_experiments.py`: reproducibly run the matrix, preserve per-run results, and write a consolidated summary.

## 4. Experiment matrix

All experiments use `normal_platooning`, a **12.0-second** controlled duration, and no added legacy CACC V2X event. Timing uses a 0.1 s simulation step. Every degradation event starts at **3.0 s**; sensor/V2X recovery occurs at the configured end time.

| Group | ID pattern | Twin-only condition | Window(s) |
| --- | --- | --- | --- |
| A | `p22c_baseline_s<seed>` | no added degradation | n/a |
| B | `p22c_noise_low/medium/high_s<seed>` | radar range std 0.25/0.75/1.50 m; odometry-speed std 0.10/0.30/0.60 m/s | whole 12.0 s run |
| C | `p22c_radar_short/medium/extended_s<seed>` | deterministic radar suppression | 3–4 s / 3–6 s / 3–9 s |
| D | `p22c_odometry_short/medium/extended_s<seed>` | deterministic own-odometry suppression | 3–4 s / 3–6 s / 3–9 s |
| E | `p22c_v2x_latency_200/600_s<seed>` | Twin-only 200/600 ms latency | 3–9 s |
| F | `p22c_v2x_loss_20/30/40_s<seed>` | Twin-only 0.2/0.3/0.4 loss | 3–9 s |
| G | `p22c_v2x_outage_s<seed>` | Twin-only outage | 3–7 s |

The chosen noise levels are explicitly experiment assumptions—not hardware specifications—and are anchored to the current zero-noise defaults.  Packet-loss cases implement the required 20%, 30%, and 40% requested probabilities.  The logged generated/delivered/dropped/stale counts determine realised loss; requested probability is never reported as a realised rate.

Actual seed allocation from `scenarios/experiments/phase_2_2c.json` is:

- both seeds (`20260929`, `20260930`): baseline, medium noise, medium radar dropout, medium odometry dropout, 600 ms latency, 30% loss, and outage;
- seed `20260929` only: low/high noise, short/extended radar dropout, short/extended odometry dropout, 200 ms latency, and 20%/40% loss.

This allocation yields 23 configured runs. It is a practical reproducibility check, not a statistically sufficient calibration study.

## 5. Measurements and success evidence

Each run must produce normal telemetry plus:

- `twin_ground_truth.csv`, `twin_filtered_states.csv`, and V2X event rows;
- existing `twin_filtered_evaluation.json` and `twin_uncertainty_evaluation.json`;
- `twin_degradation_summary.json` containing requested configuration, event windows, applied sensor-suppression counts, and Twin V2X channel statistics;
- a consolidated `results/phase_2_2c_experiments_summary.json`.

Evidence of application is required:

- radar dropout: suppression count > 0 and radar track prediction-only evidence where a target exists;
- odometry loss: suppressed count > 0, own-state age/provenance shows prediction-only, and covariance grows versus the preceding fresh period;
- latency/loss/outage: Twin V2X event rows show the configured scheduled delivery/drop/outage behavior and channel statistics agree;
- recovery: rows following the window show fresh sensor measurement provenance or delivered V2X events as appropriate.

## 6. Evaluation constraints

The existing accuracy and uncertainty evaluator schemas remain unchanged.  Their output is used as-is for speed/acceleration MAE, RMSE, covariance consistency, coverage, provenance, and age groups.  Position, full-state NEES, and NIS remain unavailable for the documented reasons.  No metric is invented for absent data.

## 7. Files expected to change

- New: `samtruck/twin_degradation.py`, `run_degradation_experiments.py`, `scenarios/experiments/phase_2_2c.json`, targeted tests, implementation report.
- Modified: `run_all_scenarios.py` only at the Twin sensor/V2X boundary and post-run summary write.

No changes are planned for `samtruck/controller.py`, `samtruck/estimation.py`, `samtruck/twin_runtime.py`, `samtruck/v2x.py`, `samtruck/sensors.py`, official catalog entries, SUMO files, or evaluator matching/metric definitions.

## 8. Risks and limitations

- Radar loss does not remove own odometry; it primarily validates radar-track and raw Twin behavior, not own-state speed/acceleration uncertainty.
- V2X is not fused into the current own-state Kalman filter, so V2X degradation validates communication freshness/statistics rather than directly changing own-state covariance.
- A normal fixed-route simulation may occasionally have no radar target on a particular window; the summary will report that rather than claim radar degradation was observed.
- Two seeds and short runs demonstrate reproducibility, not calibrated confidence intervals.

## 9. Implementation decision

The required changes are isolated to the passive Twin branch and do not require modifying CACC behavior or the live ground-truth boundary.  Implementation may proceed.
