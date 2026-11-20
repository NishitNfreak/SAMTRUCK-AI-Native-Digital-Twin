# Phase 2.2D — Single High-Sensor-Noise Demonstration Report

## Scope

This is one fresh, bounded 12.0 s demonstration run. It is not a Phase 2.2D D1 matrix run and does not establish multi-seed robustness.

## Configuration

- Base scenario: `normal_platooning` with the normal three-truck platoon.
- Result directory: `results/phase_2_2d_high_noise_demo`.
- Vehicle evaluated: `samtruck_follower_1`.
- High-noise settings reused from `scenarios/experiments/phase_2_2c.json` / `p22c_noise_high`:
  - radar range standard deviation: 1.50 m;
  - odometry speed standard deviation: 0.60 m/s.
- Explicit seeds: SUMO `20260929`, scenario `20260929`, sensor `20260929`, Twin V2X `20260929`.
- The Phase 2.2D opt-in path appended the explicit SUMO `--seed` and persisted sensor DTOs. The seed manifest is recorded in `twin_degradation_summary.json`.

Command executed:

```powershell
python run_phase_2_2d_high_noise_demo.py --duration 12 --vehicle samtruck_follower_1 --sumo-seed 20260929 --scenario-seed 20260929 --sensor-seed 20260929 --twin-v2x-seed 20260929
```

Runner verdict: **PASS**.

## Data boundary and matching

- Blue data: `twin_ground_truth.csv`, used only as an offline SUMO evaluation reference.
- Orange data: `twin_sensor_measurements.csv`, the noisy odometry DTO persisted after the sensor boundary.
- Green data: `twin_filtered_states.csv`, the passive Kalman Twin estimate.

The live Twin receives sensor DTOs and delivered V2X only; this comparison reads completed files after simulation. It does not route truth into the Twin or CACC.

Records were joined by exact `vehicle_id` and `simulation_time_s`, normalized to six decimal places. For `samtruck_follower_1`: 120 truth rows, 120 sensor rows, and 120 filtered rows matched; there were zero duplicate keys and zero invalid rows.

## Quantitative result

| Metric | Raw noisy odometry vs SUMO truth | Filtered Twin vs SUMO truth | Improvement from filtering |
| --- | ---: | ---: | ---: |
| MAE (m/s) | 0.485243 | 0.182746 | 0.302496 m/s / 62.34% |
| RMSE (m/s) | 0.604439 | 0.217284 | 0.387155 m/s / 64.05% |
| Matched samples | 120 | 120 | n/a |

Both MAE and RMSE are lower for the filtered output in this run. Therefore, this single demonstration supports the limited claim that the passive filter reduced speed error relative to the persisted high-noise odometry for this fixed configuration and seed. It does not establish general performance across seeds, scenarios, or tuning settings.

## Graph verification

`results/phase_2_2d_high_noise_demo/high_noise_speed_comparison.png` contains all required series for the same 120 timestamps and vehicle:

1. blue **SUMO ground-truth speed (offline reference)**, line width 2.2 and z-order 5;
2. orange **noisy odometry speed (persisted DTO)**, visible point markers;
3. green **Kalman-filtered Twin speed**.

The blue series is plotted last with the highest z-order, so it is present in the image data even where its values overlap another curve. No values were offset or altered for visibility.

## Changes and validation

Created:

- `samtruck/high_noise_demo.py` — offline exact-match metrics and the three-series plotter;
- `run_phase_2_2d_high_noise_demo.py` — one-run-only, non-overwriting demonstration launcher;
- `tests/test_high_noise_demo.py` — exact-match, metrics, and three-series-plot tests;
- this report.

Validation completed before the simulation:

- focused high-noise and Phase 2.2D tests: **7 passed**;
- full regression suite: **94 passed**.

No CACC/CTG settings, Kalman equations/covariance, SensorInterface implementation, PendingV2XChannel implementation, Phase 2.2C configuration/results, official scenario catalog, or SUMO network files were modified.
