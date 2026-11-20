# SAMTRUCK Phase 2.2A.1 — Robust Offline Estimator Evaluator

## Scope and outcome

Phase 2.2A.1 replaces the runner's minimal in-memory estimator summary with a standalone, file-based post-run evaluator.  It evaluates completed `twin_ground_truth.csv` and `twin_filtered_states.csv` files only.  It never runs in the simulation control loop and has no live path back to the Twin, CACC, SUMO, sensors, or V2X delivery.

## Files created and modified

| File | Change |
| --- | --- |
| `samtruck/offline_estimator_evaluator.py` | **Created.** Pure CSV/record evaluator with matching diagnostics and structured speed/acceleration metrics. |
| `tests/test_offline_estimator_evaluator.py` | **Created.** Deterministic synthetic evaluator tests. |
| `run_all_scenarios.py` | **Modified minimally.** Imports and invokes the evaluator only after the truth and filtered CSV files have been written.  The prior in-memory helper was removed. |
| `docs/PHASE_2_2A_1_IMPLEMENTATION_REPORT.md` | **Created.** This report. |

No changes were made to `samtruck/estimation.py`, `samtruck/twin_runtime.py`, `samtruck/sensors.py`, `samtruck/controller.py`, `samtruck/v2x.py`, scenario JSON files, SUMO files, or controller/estimator parameters.

### Component-specific metric classification correction

The evaluator initially grouped an entire row as measurement-updated whenever `measurement_update_mask` was non-empty.  That could incorrectly place an unmeasured acceleration estimate in the acceleration measurement-updated group when only the `speed` token was present, or vice versa.

The corrected evaluator reads the verified existing tokens emitted by `VehicleStateEstimator` and serialized by `PlatoonTwin.filtered_as_rows()`:

- `speed`
- `acceleration`
- optional `position` (not evaluated in this module)

For each metric independently:

- a matching component token means `measurement_updated`;
- a missing component token with a valid initialized/updated/predicted estimator state means `prediction_only` for that component;
- `uninitialized_missing_speed` and `invalid_timestamp` mean `other_no_measurement_update`;
- all-samples metrics remain unchanged.

The JSON now adds `component_classification_counts.speed_mps` and `component_classification_counts.acceleration_mps2`.  Existing row-level `classification_counts` remains as a backward-compatible row summary.

The diagnostic field formerly named `nearby_timestamp_mismatch_candidates` is now named `nearby_unmatched_truth_with_filtered_candidate_count`.  It is deliberately asymmetric: it counts only unmatched **truth** rows which have at least one same-vehicle unmatched filtered timestamp within one configured simulation step.  It is a diagnostic candidate count, not a count of unique mismatch pairs and never causes a match.

## Evaluator architecture

`samtruck.offline_estimator_evaluator` uses Python standard-library `csv`, `math`, and `statistics` modules only.  It imports no TraCI, SUMOlib, controller, Twin runtime, sensor interface, or V2X delivery module.

`evaluate_csv_files()` reads completed CSVs and delegates to `evaluate_records()`.  The runner calls it after closing `twin_ground_truth.csv` and `twin_filtered_states.csv`, then writes the returned JSON to:

```text
results/<scenario>/twin_filtered_evaluation.json
```

The evaluator has no function that receives a simulator handle, modifies a live DTO, or returns a value to the Twin/controller.

## Matching policy

Current CSVs have no simulation-step index.  The evaluator therefore uses the documented key:

```text
(vehicle_id, round(simulation_time_s, 6))
```

This is exact normalized matching at one microsecond (`1e-6 s`), not unconstrained nearest-neighbour matching.  The tolerance is explicitly reported and is below the runner's configured `0.1 s` simulation step.  Rows that are only nearby in time are reported as `nearby_unmatched_truth_with_filtered_candidate_count`; they are never paired.

The evaluator reports and does not silently overwrite:

- invalid/missing timestamp or vehicle-ID records;
- duplicate truth keys and duplicate filtered keys;
- matched records;
- unmatched truth and filtered records;
- nearby-but-unpaired timestamp candidates;
- non-finite truth/estimate speed and acceleration values.

Duplicate keys are excluded from matching entirely.  This prevents an arbitrary duplicate from becoming a false accuracy sample.

## Metrics and definitions

For valid aligned pairs, error is `estimate - ground_truth`.

For both speed (m/s) and acceleration (m/s²), the evaluator reports:

- `samples` / `n`;
- MAE;
- RMSE;
- bias (mean signed error);
- maximum absolute error;
- inclusive-interpolated 95th percentile absolute error.

Empty groups report `samples: 0` and `null` metrics rather than misleading zero values.  Position remains explicitly unavailable because unanchored filtered position and lane-local ground-truth position have incompatible frames.  Radar metrics remain outside scope because identity-compatible radar truth/relative-speed truth is not presently logged.

## Measurement-versus-prediction classification

The evaluator preserves current `measurement_update_mask` semantics.  **Component-specific classification is the authoritative policy for speed and acceleration metrics;** the legacy `classification_counts` field is only a backward-compatible row summary:

- a `speed` token -> speed `measurement_updated` only;
- an `acceleration` token -> acceleration `measurement_updated` only;
- a component absent during a valid time-advancing update or a `predicted_only`/`degraded_long_gap_predicted` row -> that component's `prediction_only` group;
- `partially_initialized` with a missing component -> `other_no_measurement_update`, because the absent component is an uncertain initialization prior rather than a propagated estimate;
- `updated_same_timestamp` with a missing component -> `other_no_measurement_update`, because `dt=0` means no propagation occurred for that component;
- `uninitialized_missing_speed` and `invalid_timestamp` -> `other_no_measurement_update` for both components.

It reports all matched estimates, measurement-updated estimates, and prediction-only estimates separately per component.  It does not claim that a non-empty mask means both channels were measured; masks such as `speed` remain preserved in source CSVs.  The third category is counted but excluded from update/prediction metric subgroups, which avoids inventing a classification for initialization, zero-`dt`, or invalid states.

## Output JSON structure

The existing `speed_error_mps`, `acceleration_error_mps2`, and unavailable `position_error` keys are retained for practical compatibility, with added metrics.  New structured keys are:

```text
schema_version
scenario_id
matching_policy
matching_diagnostics
classification_counts
component_classification_counts
metrics.speed_mps.{all,measurement_updated,prediction_only}
metrics.acceleration_mps2.{all,measurement_updated,prediction_only}
limitations
```

Example verified output from fixed-seed `normal_platooning` fast-demo:

```json
{
  "scenario_id": "normal_platooning",
  "matching_diagnostics": {
    "matched_records": 750,
    "unmatched_truth_records": 0,
    "unmatched_filtered_records": 0
  },
  "metrics": {
    "speed_mps": {
      "units": "m/s",
      "all": {"samples": 750, "mae": 0.09678632045904201, "rmse": 0.15828439407159017}
    }
  }
}
```

## Tests executed

```powershell
python -m unittest tests.test_offline_estimator_evaluator -v
python -m unittest discover -s tests -v
python run_all_scenarios.py --scenario normal_platooning --fast-demo --seed 20260929
```

| Check | Result |
| --- | --- |
| New focused offline evaluator tests | **10/10 passed** |
| Full project regression suite | **65/65 passed** |
| Official headless runner smoke validation | **PASS** (`normal_platooning`) |

Focused tests cover manually verifiable MAE/RMSE/bias/max/p95 values, empty records, duplicate truth/filtered keys, unmatched records, timestamp mismatch without nearest-neighbour pairing, invalid timestamps/vehicle IDs, NaN/infinite data, multiple vehicle IDs/scenarios, update-versus-prediction classification, and forbidden live-system imports.

The component-specific correction adds explicit synthetic checks for both components updated, speed-only update with acceleration prediction, acceleration-only update with speed prediction, both prediction-only, and uninitialized/invalid no-update rows.  After this correction, focused evaluator tests remain **10/10 passed**, the complete suite remains **65/65 passed**, and the official `normal_platooning` runner smoke test passed.  Its output reports 750 measurement-updated speed samples and 750 measurement-updated acceleration samples, with zero prediction-only samples, as expected for that nominal run.

### Final partially-initialized and same-timestamp correction

The final Phase 2.2A.1 correction changes only `samtruck/offline_estimator_evaluator.py`, `tests/test_offline_estimator_evaluator.py`, and this report.  It does not alter all-samples errors, timestamp matching, duplicate handling, runner behavior, or any live estimation/control component.

`VehicleStateEstimator` has the following verified own-state status semantics relevant to component metrics:

- `partially_initialized`: speed was measured, but absent acceleration is a zero-valued prior with broad uncertainty.  A missing acceleration token is therefore `other_no_measurement_update`, not prediction-only.
- `updated_same_timestamp`: the named mask component was measurement-updated, but `dt=0` means a missing component was not propagated.  A missing component is therefore `other_no_measurement_update`.
- `uninitialized_missing_speed` and `invalid_timestamp`: both components are `other_no_measurement_update`.
- `predicted_only`, `degraded_long_gap_predicted`, and components absent during time-advancing `updated`/`degraded_long_gap_updated` rows remain valid component-level `prediction_only` classifications.

The authoritative speed/acceleration grouping remains `component_classification_counts` and the corresponding component metric groups.  `classification_counts` remains only a backward-compatible row-level summary.

Regression coverage now explicitly exercises a full mask, time-advancing speed-only and acceleration-only masks, both-component prediction-only, partial initialization, same-timestamp single-component update, and invalid-timestamp behavior.  Results after this final correction:

| Check | Result |
| --- | --- |
| Focused evaluator tests | **10/10 passed** |
| Full project regression suite | **65/65 passed** |
| Official `normal_platooning --fast-demo --seed 20260929` smoke run | **PASS** |

The nominal smoke result retains the same all-samples speed/acceleration values and the same exact normalized timestamp matching policy.  It has no prediction-only rows, so this classification correction does not change its nominal aggregate metrics.

## Existing-output diagnostic audit

The new evaluator was read-only applied to existing outputs for `normal_platooning`, `emergency_braking`, and `v2x_latency_packet_loss`:

| Scenario | Matched | Unmatched truth | Unmatched filtered | Truth duplicates | Filtered duplicates |
| --- | ---: | ---: | ---: | ---: | ---: |
| `normal_platooning` | 750 | 0 | 0 | 0 | 0 |
| `emergency_braking` | 750 | 0 | 0 | 0 | 0 |
| `v2x_latency_packet_loss` | 1,050 | 0 | 0 | 0 | 0 |

These are verified clean matching diagnostics for the inspected result directories, not a general claim about every future run.

## Preservation checks

- The offline module has no live-system imports.
- The runner invokes evaluation only after CSV writes complete.
- CACC invocation, equations, gains, inputs, and behavior are unchanged.
- Estimator/Twin equations and output semantics are unchanged.
- Sensor and V2X behavior are unchanged.
- Scenario and SUMO configuration files are unchanged.

## Known limitations

1. Timestamp matching uses normalized timestamps because no step index is logged; a future evaluator should prefer an explicit immutable step index.
2. Position and radar accuracy remain intentionally unavailable.
3. Current CSVs do not expose own-measurement age, so prediction-error versus elapsed measurement-gap evaluation remains a future task.
4. The smoke scenario uses `--fast-demo`, which shifts events in memory; it confirms integration, not multi-seed estimator reliability.
