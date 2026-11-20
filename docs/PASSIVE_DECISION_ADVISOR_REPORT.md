# Milestone B — Passive Decision Advisor

## Purpose

This milestone adds an explanation-only advisor after `PlatoonTwin.update()`.
It reads Twin-side sensor DTOs, filtered-state uncertainty/freshness metadata,
and Twin V2X status. It never sends a command to SUMO or the CACC controller.
Every persisted decision has `command_to_executive: "NONE"`.

## Files changed

- Created `samtruck/twin_advisor.py`: deterministic rule engine and terminal/JSONL sink.
- Modified `run_all_scenarios.py`: opt-in `--twin-advisor` and
  `--twin-advisor-period` hook, invoked after `twin.update()`.
- Created `run_passive_advisor_demo.py`: fresh, non-overwriting visible demo.
- Created `tests/test_twin_advisor.py`.

No CACC controller, Kalman filter, sensor-interface internals, V2X-channel
internals, SUMO configuration, or existing result folder was changed.

## Data boundary

`_advisor_inputs_from_twin_snapshots()` explicitly maps only filtered-state,
sensor-DTO, freshness, and predecessor-V2X fields to `AdvisorInput`. Although
the evaluator-console display snapshot also contains `truth_*` fields, the
advisor mapping never reads them. SUMO truth remains evaluation/display-only;
it is not a live Twin or advisor input.

## Rules

The rule order is deterministic: combined critical awareness, degraded
confidence/gap recommendation, combined sensor-plus-V2X degradation, own
measurement degradation, V2X degradation, elevated uncertainty, then healthy
hold. Advisory-only thresholds are speed sigma 0.35/0.70 m/s, measurement age
1.0/3.0 s, and V2X age 0.5/2.0 s. They do not change filter or controller
tuning.

## Actual demonstration

Command executed from `E:\SAMSUNGTRUCK`:

```powershell
python run_passive_advisor_demo.py --duration 12 --sumo-seed 20260929 --scenario-seed 20260929 --sensor-seed 20260929 --twin-v2x-seed 20260929 --evaluator-console-period 1.0 --advisor-period 1.0
```

Result folder: `results/passive_advisor_demo/`.

- SUMO/scenario/sensor/Twin-V2X seeds: all `20260929`.
- Sensor noise: radar range `1.50 m`; odometry speed `0.60 m/s`.
- Twin-only events: odometry dropout `3.0–7.0 s`; V2X outage `3.0–7.0 s`.
- Scenario verdict: **PASS**.
- Decision records: `360`; all executive commands were `NONE`.
- Observed decisions: `MONITOR_ELEVATED_UNCERTAINTY=238`,
  `PREDICTION_ONLY_DEGRADED=15`, `INCREASE_GAP_RECOMMENDED=107`.

Real console excerpt at `t=3.1 s`:

```text
samtruck_follower_1    PREDICTION_ONLY_DEGRADED       severity=warn rule=R4_OWN_MEASUREMENT
  Why: Odometry is invalid/dropout, estimate is prediction-only, or measurement age is stale.
  Advisory: Monitor prediction-only operation until a valid measurement recovers; no command is applied.
  Executive command: NONE
```

The degradation summary recorded `120` suppressed Twin odometry DTOs and `80`
Twin V2X outage attempts/dropped messages. The current persisted predecessor
V2X freshness field remained `fresh_propagated` with a `0.1 s` age during this
outage, so the demo did **not** claim a V2X-degraded decision from that field.
This is an existing V2X freshness-observability limitation to resolve in a
separately approved task; the advisor preserves and reports the metadata it is
given rather than inventing stale status.

## Validation

```text
python -m unittest tests.test_twin_advisor -v   -> 9 passed
python -m unittest discover -s tests -v         -> 107 passed
```

The advisor unit tests cover healthy, odometry-dropout, V2X-only, combined,
and critical decisions; JSONL validity; terminal print-period suppression; and
a static check for no TraCI, CACC, or `setSpeed` dependency.

## Use

For any existing scenario without changing its baseline behaviour:

```powershell
python run_all_scenarios.py --scenario normal_platooning --fast-demo --evaluator-console --twin-advisor
```

The advisor records every step to `results/<scenario>/advisor_decisions.jsonl`
and prints once per simulation second by default. It is passive only. A future
milestone may use these persisted decisions as evidence for a separately
validated Decision Advisor / executive interface, but must not reuse this
milestone as authority for vehicle actuation.
