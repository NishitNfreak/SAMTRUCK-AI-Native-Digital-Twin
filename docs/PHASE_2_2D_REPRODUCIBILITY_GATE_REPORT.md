# Phase 2.2D — Reproducibility Gate and Twin Trace Inspection Report

## Scope

This implementation performs only the approved Phase 2.2D reproducibility gate and read-only trace inspection. It does not run the 80-run matrix, alter CACC/CTG, change Kalman equations, modify the SensorInterface or PendingV2XChannel implementations, or modify Phase 2.2C configurations/results.

## SUMO randomness audit

### Active Montreal benchmark inputs

`run_all_scenarios.py` launches `city/montreal/montreal.sumocfg`, which references:

- `concordia_sgw_region.net.xml` — static network;
- `vehicle_types.rou.xml` — the active passenger type has `sigma="0.5"`, so SUMO car-following contains a stochastic source; the platoon truck type has `sigma="0.0"`;
- `city_traffic.rou.xml` — ten fixed-ID passenger `<trip>` records with fixed departure times (25–43 s) and fixed origin/destination edges; it has no active `<flow>`, random distribution, or route distribution;
- `concordia_sgw_region.poly.xml` — static visual/geometry data.

`montreal.sumocfg` sets the network, input files, 0.1 s step length, collision/teleport processing, and reporting options. Before this gate, it did not specify a seed and the `run_all_scenarios.py` command did not pass `--seed`. Therefore, Python-level scenario/sensor/V2X seeds did not control SUMO's `sigma` randomness.

Inactive files in `city/montreal` include `platoon_route.rou.alt.xml` (a route distribution with probability 1.0) and `random_trips_3.xml`; neither is referenced by `montreal.sumocfg` or the official runner. They are not randomness sources for this benchmark path.

### Seeded subsystems

| Subsystem | Random source | Resolved seed in Phase 2.2D gate |
| --- | --- | --- |
| SUMO physical simulation | SUMO PRNG, including passenger `sigma=0.5` behavior | explicit `--seed 20260929` |
| Legacy inline CACC V2X event path | local `random.Random(spec["seed"])` | scenario seed `20260929` |
| Sensor noise/random radar dropout | `SensorInterface` local RNG | explicit sensor seed `20260929` |
| Twin pending-channel packet loss | `PendingV2XChannel` local RNG | explicit Twin V2X seed `20260929` |
| Twin-only time windows | `TwinDegradationSchedule` | deterministic; no RNG |

The gate does not prove universal determinism on every platform or for every future SUMO configuration. It verifies this project, binary, inputs, and explicit seed manifest for one short normal-platooning execution.

## Changes

| File | Change |
| --- | --- |
| `run_all_scenarios.py` | Added opt-in `phase_2_2d.sumo_seed`; only when present, appends `--seed` to the SUMO command, resolves/logs SUMO/scenario/sensor/Twin-V2X/legacy CACC seeds, and writes Phase 2.2D-only audit CSVs. Existing scenarios without the opt-in block retain their command and output behavior. |
| `samtruck/reproducibility.py` | Added read-only completed-artifact comparator that reports the first mismatch. |
| `run_phase_2_2d_reproducibility_gate.py` | Added two-run isolated scratch gate. It uses `normal_platooning`, separate `results/phase_2_2d_gate/run_a` and `run_b` folders, and an identical explicit manifest. |
| `inspect_twin_trace.py` | Added read-only CSV trace inspector with vehicle/time filters and optional speed plot. |
| `tests/test_phase_2_2d_reproducibility.py` | Added focused tests for opt-in seed resolution, DTO audit rows, result comparison, and historical no-sensor-log behavior. |

### Phase 2.2D-only observability

New opt-in Phase 2.2D runs write:

- `sumo_vehicle_lifecycle.csv`: active, departed, and arrived IDs per step;
- `twin_sensor_measurements.csv`: the measurement DTO fields delivered to the Twin after any Twin-only degradation.

These are audit outputs only. They do not feed truth into the Twin, change the sensor model, alter estimation, or retroactively manufacture raw sensors for historical results.

## Reproducibility gate procedure and result

Executed command:

```powershell
python run_phase_2_2d_reproducibility_gate.py --duration 12 --sumo-seed 20260929 --scenario-seed 20260929 --sensor-seed 20260929 --twin-v2x-seed 20260929
```

Both isolated 12.0 s scratch runs returned the runner's PASS verdict. The report is `results/phase_2_2d_gate/reproducibility_gate_report.json`.

The comparator performed exact CSV comparisons (after identical explicit inputs) and JSON comparisons that ignore only the scratch-specific `scenario_id` label. It compared:

1. `sumo_vehicle_lifecycle.csv` — vehicle IDs, active/departed/arrived behavior;
2. `twin_ground_truth.csv` — independently logged SUMO trajectories;
3. `twin_sensor_measurements.csv` — DTOs actually delivered to the Twin;
4. `twin_v2x_messages.csv` — generation, scheduling, delivery/drop/rejection outcomes;
5. `twin_filtered_states.csv` — estimator state, covariance, provenance, and age;
6. `twin_radar_tracks.csv`;
7. degradation, accuracy-evaluation, and uncertainty-evaluation JSON artifacts.

Result: **all nine artifacts matched exactly; no first mismatch exists.** This is evidence of fixed-manifest reproducibility for the gate run, not evidence of multi-seed robustness.

## Trace inspection

Usage:

```powershell
python inspect_twin_trace.py --results-dir "results/p22c_odometry_medium_s20260929" --vehicle samtruck_follower_1 --start 2.9 --end 3.2
python inspect_twin_trace.py --results-dir "results/phase_2_2d_gate/run_a" --vehicle samtruck_follower_1 --start 0.1 --end 3.0 --plot "results/phase_2_2d_gate/follower_1_speed.png"
```

The inspector joins only persisted files by vehicle ID and normalized simulation timestamp. It prints SUMO truth explicitly as an **offline evaluation reference**, sensor/odometry DTO values when persisted, radar tracks, Twin estimates/covariance/provenance/age, and associated Twin V2X event rows. Empty fields are shown as `NA`; the utility never writes to the selected result directory.

Sample historical inspection (`p22c_odometry_medium_s20260929`, `samtruck_follower_1`, 2.9–3.2 s) found four truth/filtered/radar records and eight V2X events, but zero raw sensor DTO records. At 3.0 s the filtered speed became `prediction_only` and its speed-measurement age increased from 0.0 to 0.1 s. This is expected for the configured odometry-loss window, but raw historical DTO values are unavailable.

Sample gate inspection (`phase_2_2d_gate/run_a`, `samtruck_follower_1`, 0.1–0.3 s) found persisted sensor DTOs. At 0.1 s, it showed truth speed and odometry speed `11.639947 m/s`, valid radar range `16.5 m`, and a measurement-updated Twin speed with age 0.0 s. The equality is a property of this zero-noise gate configuration, not a claim that truth entered the Twin.

## Validation

| Check | Result |
| --- | --- |
| Focused Phase 2.2D tests | PASS — 5 tests |
| Full unit regression suite | PASS — 92 tests |
| Two-run reproducibility gate | PASS — both runner verdicts PASS and all nine output artifacts matched |
| Official normal-platooning smoke test | Not run; the gate exercised the same base scenario with an explicit Phase 2.2D-only seed contract. |

## Historical Phase 2.2C compatibility decision

The historical Phase 2.2C runs did not receive an explicit SUMO `--seed` and do not contain the Phase 2.2D seed manifest/lifecycle/DTO artifacts. They cannot currently be certified as compatible with the new strict Phase 2.2D reproducibility contract. They remain useful pilot evidence, but the Phase 2.2D matrix must use the **fresh-dataset path** unless a separately approved, evidence-based compatibility procedure can establish equivalent SUMO seed behavior without silently assuming it.

## Limitations and next approved boundary

- The gate uses one fixed seed and 12.0 s duration; it does not substitute for the planned repeated-run matrix.
- Existing historical runs do not contain raw sensor DTO CSVs. A future observability decision is needed only if historical raw-sensor reconstruction is required; no such reconstruction was attempted.
- The optional plot requires `matplotlib`, already used by the project plotting path; no dependency was added.
- Phase 2.2D ends here. The 80-run matrix and Phase 2.3 were not started.
