# Milestone A — Live Evaluator Demonstration Console

## Purpose

Milestone A makes the passive Digital Twin pipeline visible during a SUMO run without changing vehicle control. At each simulation step the console records three independently labelled views per truck:

1. **SUMO truth** — an offline evaluation reference only;
2. **Sensor** — the post-degradation odometry/radar DTO that was actually supplied to the Twin;
3. **Twin** — the post-update Kalman filtered state, covariance-derived standard deviation, freshness, provenance, and available predecessor-V2X status.

The presentation join occurs after `twin.update(...)`. The console never returns truth or display values to the Twin, CACC controller, V2X channel, or vehicle actuation path.

## Implementation

- `samtruck/evaluator_console.py`: JSONL writer and compact period-limited terminal renderer.
- `run_all_scenarios.py`: opt-in `--evaluator-console` and `--evaluator-console-period`; disabled by default. The hook reads `current_truth_rows`, `twin_sensor_inputs`, and `twin.filtered_as_rows(...)` after the passive Twin update.
- `run_evaluator_live_demo.py`: non-overwriting one-command high-noise demo launcher.
- `tests/test_evaluator_console.py`: console formatting, JSONL, period, and truth/sensor-separation checks.

The Windows console uses ASCII `sigma` rather than the Unicode Greek character so it remains compatible with the host CP1252 terminal. JSONL retains raw numeric variances.

## Exact completed execution

The requested default folder `results/evaluator_live_demo` was created by an initial attempted run but the process stopped at 0.1 s because CP1252 could not encode the Unicode sigma character. That partial folder was **preserved and not overwritten**. After the display-only ASCII correction, the completed fresh result used a new folder:

`results/evaluator_live_demo_retry/`

Command executed:

```powershell
python run_evaluator_live_demo.py --run-id evaluator_live_demo_retry --duration 12 --vehicle samtruck_follower_1 --sumo-seed 20260929 --scenario-seed 20260929 --sensor-seed 20260929 --twin-v2x-seed 20260929 --evaluator-console-period 1.0
```

Configuration:

| Setting | Value |
| --- | ---: |
| Base scenario | `normal_platooning` |
| Duration | 12.0 s |
| SUMO seed | 20260929 |
| Scenario seed | 20260929 |
| Sensor seed | 20260929 |
| Twin V2X seed | 20260929 |
| Radar range noise | 1.50 m |
| Odometry speed noise | 0.60 m/s |
| Console period | 1.0 s |

Runner verdict: **PASS**.

## Example real terminal block

```text
t = 3.1s | LIVE DIGITAL TWIN EVALUATOR CONSOLE
Truth = SUMO offline reference | Sensor = Twin-side DTO | Twin = passive filter
Vehicle                Truth spd  Sensor spd    Twin spd   sigma     Age  Flag
samtruck_leader            4.99     3.81     5.33     0.39   0.00  MEAS
samtruck_follower_1        7.25     7.56     7.46     0.39   0.00  MEAS
samtruck_follower_2        8.63     9.77     9.01     0.39   0.00  MEAS
------------------------------------------------------------------------------
Notes: DROP = invalid/missing DTO | sigma = sqrt(speed variance) | samtruck_follower_1 V2X=fresh_propagated age=0.10s | samtruck_follower_2 V2X=fresh_propagated age=0.10s
```

## Persisted console evidence

`results/evaluator_live_demo_retry/evaluator_console.jsonl` has **360 lines**: one JSON object for every active truck at each of the 120 simulation steps. It records truth, sensor, filtered, uncertainty, provenance, and V2X fields. The first `samtruck_follower_1` row, for example, records distinct truth `11.639947 m/s` and noisy DTO `12.298014 m/s` values, demonstrating that the console does not overwrite sensor data with truth.

## Quantitative speed evaluation

The three series for `samtruck_follower_1` were exact-joined by vehicle ID and simulation time normalized to six decimals. There were 120 truth rows, 120 sensor rows, and 120 filtered rows; all 120 matched with zero duplicates and zero invalid rows.

| Metric | Persisted noisy odometry vs SUMO truth | Filtered Twin vs SUMO truth | Filter improvement |
| --- | ---: | ---: | ---: |
| MAE | 0.485243 m/s | 0.182746 m/s | 0.302496 m/s (62.34%) |
| RMSE | 0.604439 m/s | 0.217284 m/s | 0.387155 m/s (64.05%) |

For this fixed high-noise seed, the filtered speed reduced both MAE and RMSE. This is a single demonstration, not a multi-seed robustness conclusion.

## Evaluator plot verification

`results/evaluator_live_demo_retry/evaluator_speed_comparison.png` contains the same 120 matched samples from:

- blue **SUMO ground-truth speed (offline reference)**, drawn last at z-order 5;
- orange **noisy odometry speed (persisted DTO)** with point markers;
- green **Kalman-filtered Twin speed**.

The plot does not offset or alter truth data. Visual inspection confirmed all three traces are present.

## Validation

| Command | Actual result |
| --- | --- |
| `python -m unittest tests.test_evaluator_console tests.test_high_noise_demo tests.test_phase_2_2d_reproducibility -v` | PASS — 11 tests |
| `python -m unittest discover -s tests -v` | PASS — 98 tests |
| Completed evaluator live demo command above | PASS |

## Architecture confirmation and limitations

- CACCController, CTG/min-gap parameters, and controller call inputs were not changed.
- Kalman equations, Q/R/covariance settings, SensorInterface internals, and PendingV2XChannel internals were not changed.
- The console is passive observability only; it has no AI or vehicle actuation.
- SUMO truth stays in the evaluation branch and is not provided to the live Twin.
- Historical results and Phase 2.2C artifacts were not overwritten.
- The completed result lives in `evaluator_live_demo_retry` because the original requested name is a preserved partial failure folder.

## Recommended next milestone

Implement a **Passive Decision Advisor** that consumes only Twin estimates, covariance/freshness, and delivered V2X observations to issue explanatory *advisories* and risk labels. It must remain separate from CACC vehicle actuation until independently validated.
