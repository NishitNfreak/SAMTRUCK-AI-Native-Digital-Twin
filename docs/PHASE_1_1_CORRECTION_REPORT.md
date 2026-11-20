# SAMTRUCK Phase 1.1 Targeted Digital Twin Integration Corrections

## Executive Summary

The active benchmark workspace is `E:\SAMSUNGTRUCK`. The benchmark runner is `run_all_scenarios.py`, which executes the 20 Montreal catalog scenarios and creates reports in `results\<scenario>\`. The previously identified Twin V2X timing issue was confirmed: messages were constructed with a backdated timestamp and appended to the Twin-delivery list in the generation step.

Phase 1.1 replaces only the **passive Twin communication branch** with one timestamped, validated pending queue. The CACC controller, its invocation arguments, its inline V2X path, all scenario JSON definitions, and controller parameters were not changed.

## Pre-implementation Verification

- The project has no `.git` metadata, so a Git checkpoint/status could not be created. No existing files were deleted or reverted.
- Active benchmark: `run_all_scenarios.py`.
- `samtruck\scenario_suite.py` is a legacy/lightweight catalog runner. `samtruck\v2x.py` originally contained a discrete-step helper used by tests, while the active runner had its own inline V2X generation/delivery behavior.
- The timing problem remained in `run_all_scenarios.py`: generation used current sender values, assigned a future CSV delivery timestamp, created a backdated `V2XMessage.sim_time`, and appended it immediately to `delivered_twin_messages`.

## Changes Implemented

| File | Change | Reason |
| --- | --- | --- |
| `samtruck\v2x.py` | Added `scheduled_delivery_time_s`, `V2XDeliveryEvent`, and `PendingV2XChannel`. | One authoritative queue for the Twin path; generation payload, delay, loss, validation, and statistics. |
| `run_all_scenarios.py` | Queues Twin messages at generation, drains only timestamp-eligible messages after scenario processing, and writes `twin_v2x_messages.csv`. | Prevent premature Twin delivery while leaving CACC intact. |
| `tests\test_v2x.py` | Added pending-delivery, loss, duplicate/order, receiver, timestamp, stale, and seed-reproducibility tests. | Regression coverage for actual communication rules. |
| `tests\test_controller.py` | Added identical-input command determinism test. | Regression guard for baseline controller behavior. |
| `docs\PHASE_1_1_CORRECTION_REPORT.md` | Added this report. | Phase 1.1 record. |

## V2X Communication Flow

1. The runner captures sender ID, receiver ID, current simulation time, speed, acceleration, braking flag, and per-sender sequence number when a message is generated.
2. `PendingV2XChannel.enqueue()` freezes that payload and assigns `scheduled_delivery_time_s = generation_time + latency`.
3. At every simulation step, `deliver_ready(now)` removes only messages with `scheduled_delivery_time_s <= now + 1e-9`.
4. Eligible messages undergo seeded packet-loss sampling, then sender, receiver, timestamp, stale-age, and sequence validation.
5. Only accepted messages are passed to `PlatoonTwin.update()`.
6. Each outcome is recorded in `results\<scenario>\twin_v2x_messages.csv` as `delivered`, `dropped`, or `rejected` with a reason.

Packet-loss sampling uses a channel-local `random.Random(seed)`, seeded from the scenario seed or optional `spec["twin"]["v2x_seed"]`. No global random state is used.

## Message Validation

The Twin communication queue rejects messages for unknown sender, unknown/wrong receiver, future/invalid generation time, stale age, missing sequence, duplicate sequence, and out-of-order sequence. Accepted sequence numbers are tracked per `(sender, receiver)` pair. The configured stale threshold defaults to one second and is configurable through the optional `spec["twin"]["v2x_stale_after_s"]` field.

## Digital Twin Data Boundary

```text
SUMO/TraCI -> SensorInterface DTOs -> PlatoonTwin
SUMO/TraCI -> independent twin_ground_truth.csv (evaluation only)
V2X generation -> PendingV2XChannel -> validated delivered messages -> PlatoonTwin

existing inline CACC branch -> CACCController -> SUMO commands
```

The Twin modules do not import TraCI/SUMOlib and reject legacy `VehicleState` input. The sensor boundary creates `VehicleSensorInput` DTOs. Ground-truth records are written separately and are never passed to the Twin. CACC does not consume Twin output.

## Sensor Model

| Measurement | Current treatment |
| --- | --- |
| Radar range | Configurable Gaussian noise; default 0 m |
| Radar availability | Configurable dropout; default 0 probability |
| Odometry speed | Configurable Gaussian noise; default 0 m/s |
| Position | Passed through unchanged |
| Acceleration | Passed through unchanged |
| Radar relative speed | Passed through unchanged |

Sensor noise uses the locally owned seeded generator in `SensorInterface`. Phase 1.1 did not alter defaults or add noise channels.

## Configuration Findings

| Path | Desired time gap | Minimum gap | Role |
| --- | ---: | ---: | --- |
| `SimulationConfig` defaults | 1.2 s | 4 m | Legacy/simple runner defaults and Twin defaults container |
| `scenario_suite.py` | 0.8 s | 10 m | Catalog city runner |
| `run_all_scenarios.py` | 0.8 s | 10 m | Official benchmark/evaluation runner |
| `montreal_demo.py` | 1.2 s | 4 m | Interactive Montreal demonstration |

The code/docs do not establish a single authority for harmonizing these controller settings. Phase 1.1 intentionally did not change them. Future baseline-versus-adaptive experiments must explicitly select the benchmark runner and use identical 0.8 s/10 m settings, or receive approval to formalize a different standard.

## Tests and Executed Verification

Commands actually run:

```powershell
python -m compileall -q samtruck run_all_scenarios.py tests
python -m unittest discover -s tests -v
python run_all_scenarios.py --scenario emergency_braking --fast-demo
python run_all_scenarios.py --scenario v2x_latency_packet_loss --fast-demo
```

Results:

| Check | Result |
| --- | --- |
| Unit tests | PASS: 21/21 |
| Not-before-delivery test | PASS |
| Delivery-at-scheduled-time test | PASS |
| Packet-loss prevention | PASS |
| Duplicate/older sequence rejection | PASS |
| Wrong receiver and future timestamp rejection | PASS |
| Stale-message rejection | PASS |
| Seeded communication reproducibility | PASS |
| Raw ground-truth object rejection | PASS |
| Emergency braking scenario | PASS: completed; 498 Twin messages delivered; 0 early deliveries |
| Latency + packet-loss scenario | PASS: completed; 562 delivered, 93 dropped, 3 rejected; 0 early deliveries |

In both scenarios, `twin_ground_truth.csv` and `twin_estimates.csv` had matching row counts for all three platoon vehicles. Existing evaluation telemetry, plots, metrics, and reports continued to be generated.

## Regression Results

`samtruck\controller.py` was not modified. Existing controller tests and a new identical-input determinism test pass. Scenario completion and evaluation outputs remain working.

An exact numerical before/after physical comparison is **unverified** because no immutable pre-Phase-1.1 benchmark snapshot was available in this non-Git project. The corrected queue is passive and has no control-to-actuation path, so it is designed not to change physical CACC behavior; this is supported by unchanged controller code and scenario completion, but is not a substitute for a stored A/B baseline.

## Remaining Limitations

- The CACC branch retains its original inline communication behavior and direct TraCI sensing by strict requirement. The Twin queue is separate and passive; it must not be described as the CACC input path.
- `V2XChannel` remains a discrete-step legacy helper; `PendingV2XChannel` is the sole authoritative queue for the Twin branch of `run_all_scenarios.py`.
- Sensor position, acceleration, and radar relative speed are ideal/pass-through channels in this phase.
- No filtering, stale-data extrapolation, prediction, AI, adaptive control, new scenarios, or parameter harmonization was added.

## Phase 2 Readiness

The Digital Twin Core is ready for Phase 2 estimation work: it has bounded histories, explicit missing/stale flags, deterministic sensor/V2X inputs, validated time-correct message delivery, separate ground truth, and regression coverage. Phase 2 should add estimation only inside the measurement-only Twin and must retain the current baseline CACC path unless separately approved.
