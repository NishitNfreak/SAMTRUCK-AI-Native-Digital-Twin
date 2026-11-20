# V2X Observability Fix

## Problem and cause

In the earlier passive-advisor demonstration, `twin_filtered_states.csv`
stored the `PredecessorV2XObservation` created only when a V2X message was
delivered. Its `message_age_s` and `fresh_propagated` status were therefore
frozen until another delivery arrived. During an outage, this incorrectly
looked fresh to the CSV, evaluator console, and advisor.

## Implementation

`samtruck/twin_runtime.py` now stores the last *accepted delivery timestamp*
for each predecessor/receiver pair. On every `filtered_as_rows()` call it
calculates receiver-side age as:

```text
current_filtered_timestamp_s - last_accepted_delivery_timestamp_s
```

The status is dynamically classified:

- platoon leader: `not_applicable`, age `null`;
- no accepted predecessor delivery: `not_available`, age `null`;
- age <= `v2x_stale_after_s`: `fresh_propagated`;
- age > `v2x_stale_after_s`: `stale`;
- age > `v2x_outage_after_s` (2.0 s): `outage`.

The default passive-Twin stale threshold is now `0.5 s`; this does not affect
the frozen CACC path. The V2X delivery channel remains the only delivery and
validation authority. No pending message or SUMO truth is exposed to the
Twin.

The passive advisor now prioritizes the explicit combined
`SENSOR_AND_V2X_DEGRADED` rule over its generic high-uncertainty gap advisory,
so a verified simultaneous sensor/V2X degradation is visible to evaluators.
It remains advisory only and always records `command_to_executive: "NONE"`.

## Files changed

- Modified `samtruck/twin_runtime.py`
- Modified `samtruck/config.py`
- Modified `samtruck/twin_advisor.py`
- Modified `tests/test_twin_estimator_integration.py` to reflect delivery-age
  semantics.
- Created `tests/test_v2x_observability.py`
- Created `run_v2x_observability_fix_demo.py`
- Created this report.

## New deterministic tests

`tests/test_v2x_observability.py` verifies:

1. receiver-side V2X age advances when there is no delivery;
2. `fresh_propagated -> stale -> outage` transitions;
3. leader `not_applicable` and never-delivered follower `not_available` states.

## Fresh demonstration

Executed from `E:\SAMSUNGTRUCK`:

```powershell
python run_v2x_observability_fix_demo.py --duration 12 --sumo-seed 20260929 --scenario-seed 20260929 --sensor-seed 20260929 --twin-v2x-seed 20260929 --evaluator-console-period 1.0 --advisor-period 1.0
```

Fresh, non-overwritten result folder:
`results/v2x_observability_fix_demo/`.

Configuration:

- duration 12.0 s;
- all four seeds `20260929`;
- radar noise 1.50 m and odometry-speed noise 0.60 m/s;
- Twin-only odometry dropout and Twin-only V2X outage from 3.0 s to 7.0 s;
- evaluator console and passive advisor enabled.

Actual `samtruck_follower_1` CSV evidence:

| Simulation time (s) | Last V2X generation (s) | V2X age (s) | Status |
| ---: | ---: | ---: | --- |
| 3.1 | 2.9 | 0.1 | fresh_propagated |
| 4.1 | 2.9 | 1.1 | stale |
| 5.1 | 2.9 | 2.1 | outage |
| 6.1 | 2.9 | 3.1 | outage |
| 7.1 | 7.0 | 0.0 | fresh_propagated |

The follower advisor transitioned to `SENSOR_AND_V2X_DEGRADED` at 4.1 s and
5.1 s, and to `CRITICAL_AWARENESS` at 6.1 s when both measurement age and V2X
age became critical. All JSONL records had `command_to_executive: "NONE"`.

### Decision-histogram comparison

| Decision | Earlier passive demo | Fixed demonstration |
| --- | ---: | ---: |
| MONITOR_ELEVATED_UNCERTAINTY | 238 | 236 |
| PREDICTION_ONLY_DEGRADED | 15 | 15 |
| INCREASE_GAP_RECOMMENDED | 107 | 41 |
| SENSOR_AND_V2X_DEGRADED | 0 | 46 |
| CRITICAL_AWARENESS | 0 | 22 |

For followers inside the configured outage window, 46 records were explicitly
V2X-relevant combined-degradation decisions. The launcher wrote the exact
counts and verification verdict to `v2x_observability_demo_summary.json`.

## Validation

```text
python -m unittest tests.test_v2x_observability tests.test_twin_estimator_integration tests.test_twin_advisor -v
25 passed

python -m unittest discover -s tests -v
110 passed
```

The fresh scenario itself returned `PASS` and the launcher verified a
non-zero V2X-relevant follower advisory count plus an all-`NONE` command set.

## Safety audit

- No `CACCController`, CTG/minimum gap, controller call site, or vehicle
  actuation was changed.
- No Kalman, sensor-noise, or PendingV2XChannel equation/implementation was
  changed.
- SUMO truth remains in `twin_ground_truth.csv` as offline evaluation data;
  it is not passed to `PlatoonTwin` or `AdvisorInput`.
- Console output uses ASCII text (`sigma`, not a Greek character), preserving
  CP1252 terminal safety.
