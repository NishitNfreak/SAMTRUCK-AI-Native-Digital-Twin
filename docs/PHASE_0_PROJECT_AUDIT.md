# SAMTRUCK Phase 0 Project Audit

## Audited Workspace and Environment

The audited project root is `E:\SAMSUNGTRUCK`. It is the Montreal city implementation, with 20 catalog scenarios and a real-data reporting pipeline. It is distinct from the older `E:\TruckPlatooning` straight-road workspace.

Verified during the audit:

- Portable SUMO 1.27.1 is available at `runtime\sumo-1.27.1`.
- Python can import TraCI through `samtruck.run.prepare_traci()`.
- The existing test suite initially contained 8 tests and passed.
- `run_all_scenarios.py` already generates real TraCI telemetry, V2X message logs, metrics, plots, and HTML reports under `results\<scenario>\`.

## Actual Architecture

| Area | Files | Observed behavior |
| --- | --- | --- |
| City simulation | `city\montreal\`, `samtruck\montreal_demo.py` | Imports a Montreal network; creates a three-truck route loop. |
| Scenario control | `samtruck\scenario_suite.py`, `scenarios\catalog\` | Defines 20 scenarios and event scheduling. |
| Evaluation runner | `run_all_scenarios.py`, `evaluation\` | Uses TraCI each step to write telemetry/messages/events, metrics, plots and HTML. |
| CACC resilience | `samtruck\controller.py` | CTG CACC plus degraded, ACC fallback, and radar AEB states. |
| Existing V2X helper | `samtruck\v2x.py` | Basic delayed/lossy queue used by legacy tests; city scenario runners implement V2X timing inline. |
| Existing Twin | `samtruck\digital_twin.py` | Before Phase 1: a single immutable state snapshot and `time_gap()` helper only. |

`samtruck.scenario_suite.run_spec()` and `run_all_scenarios.run_single_scenario()` use the city identifiers `samtruck_leader`, `samtruck_follower_1`, and `samtruck_follower_2`. The evaluation runner is the active target because it is the runner used to execute all catalog scenarios and generate reports.

## CACC and V2X Findings

`CACCController` implements a normal CTG target of `10 m + 0.8 * speed` in the city scenario runners. It selects `CACC` below 100 ms, `DEGRADED_CACC` below 1000 ms, `ACC_FALLBACK` for unavailable/stale V2X, and `EMERGENCY_BRAKE` for critical gap/TTC. Its code and parameters must remain unchanged in Phase 1.

The active runners directly query TraCI for sensed speed/gap and construct their V2X delivery records. This is acceptable for the preserved baseline CACC path but is not a measurement-only Digital Twin input pipeline.

## Digital Twin Requirement Audit Before Phase 1

1. **Structured state:** existing `VehicleState` had time, speed, acceleration, lane, sensed leader, gap, and time gap. Missing were bounded history, relative speed, message age/staleness, quality flags, and estimated state separation.
2. **Sensor interface:** no radar/odometry DTOs, noise, dropout, or seed-controlled sensor model existed.
3. **Ground truth separation:** missing. `digital_twin.py` did not import TraCI, but the only state consumed by live logic came directly from TraCI. No interface prevented raw-state leakage.
4. **Minimal integration:** no Twin was integrated into `run_all_scenarios.py`; the evaluation logger and live controller shared direct TraCI data.
5. **Tests:** covered `time_gap()`, braking flag, CACC, and basic V2X queue behavior. No history, stale data, sensor dropout/noise, or no-leakage tests existed.
6. **Integration point:** immediately after `traci.simulationStep()` in `run_all_scenarios.run_single_scenario()`. Sensor DTO creation and evaluation logging can branch at this point while CACC remains untouched.

## Phase Roadmap Status

| Phase | Status before Phase 1 | Notes |
| --- | --- | --- |
| 0 Audit | Complete | This document records actual city-project state. |
| 1 Twin core | Missing before this work | Implemented separately in Phase 1. |
| 2 Estimation/stale handling | Missing | No extrapolation or Kalman filter. |
| 3 AI/prediction | Missing | No AI result may be claimed. |
| 4 Adaptive control | Missing | Baseline CACC is retained. |
| 5 Fault scenarios | Partial/strong | 20 catalog scenarios exist; not all are Twin-specific faults. |
| 6 Evaluation | Implemented | Real TraCI telemetry/report system exists. |
| 7 Documentation/validation | Partial | Scenario and evaluation docs exist. |

## Risks and Compatibility Notes

- `samtruck/config.py` has legacy defaults (`4 m + 1.2 s`) while the active city runners instantiate controller parameters as `10 m + 0.8 s`. Phase 1 must not change either controller path; document this discrepancy for later consolidation.
- `samtruck.v2x.V2XChannel` is not the active city scenario V2X implementation. It remains a testable helper, while the live runner builds message timing inline.
- Current CACC can use direct predecessor TraCI values when generating/handling its legacy V2X stream. Phase 1 must not change this behavior; the new Twin is a passive observer with a stricter measurement-only boundary.
- Evaluation ground truth must remain independent from Twin inputs, even though both originate at the simulator boundary.
