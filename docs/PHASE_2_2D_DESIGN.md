# Phase 2.2D — Repeated-Run Robustness and Statistical Repeatability Design

## 1. Objective and research questions

Phase 2.2C established that controlled, Twin-only degradation can be applied and observed in short 12.0 s runs. It recorded 23/23 `PASS` verdicts, but a `PASS` means the configured run completed its verification checks; it does **not** establish that an estimator observation is repeatable across a traffic/randomness population.

Phase 2.2D should therefore answer, without altering CACC, the estimator, or the degradation mechanisms:

1. Which speed/acceleration accuracy and uncertainty observations reproduce across matched random seeds?
2. How large is seed-to-seed variability relative to the observed effect of each degradation?
3. Which effects remain inconclusive because there are too few runs, too few prediction-only samples, or only a short 12.0 s window?
4. Are the recorded degradation counters, message outcomes, measurement ages, and recovery behavior consistently realised?

This document is a design only. It neither runs experiments nor changes existing artifacts.

## 2. Verified existing capabilities and reproducibility audit

### 2.1 Existing Phase 2.2C experiment contract

Verified from `scenarios/experiments/phase_2_2c.json` and `run_degradation_experiments.py`:

- Every Phase 2.2C condition starts from official `normal_platooning`, uses a 12.0 s duration, and writes a unique result directory named `<experiment_id>_s<seed>`.
- All additional faults are declared in `twin_only_degradations`; the normal scenario's physical events are copied unchanged.
- Degradation windows begin at 3.0 s. The controlled windows are deterministic: the schedule evaluates `at_s <= now_s < at_s + duration_s` and has no random-number source.
- `run_degradation_experiments.py` writes/reconstructs `results/phase_2_2c_experiments_summary.json`. Historical PASS/FAIL is recovered only from `results/<run>/metrics.json` (`pass_fail.status`); absent or malformed evidence is represented as unknown, not as PASS.
- The existing summary has 23 result directories, one for each configuration/seed entry, with recorded PASS statuses. This is useful pilot evidence, not a statistical result.

### 2.2 Randomness paths

Verified from `run_all_scenarios.py`, `samtruck/sensors.py`, and `samtruck/v2x.py`:

| Randomness source | Implementation | Seed source | Scope |
| --- | --- | --- | --- |
| Legacy runner V2X state | `random.Random(spec["seed"])` | scenario `seed`, default `20260929` | legacy inline CACC V2X event path |
| Sensor noise / random radar dropout | `SensorInterface._rng` | `twin.sensor_seed`, else scenario `seed`, else `SimulationConfig.sensor_seed` | Twin sensor DTO generation |
| Twin V2X loss queue | `PendingV2XChannel._rng` | `twin.v2x_seed`, else scenario `seed`, else `SimulationConfig.sensor_seed` | Twin-only pending-message channel |
| Scheduled degradation | `TwinDegradationSchedule` | none | deterministic time windows |

The RNG instances are locally owned rather than global state. By default they receive the same numeric scenario seed but remain separate PRNG streams. A future manifest should record every resolved seed explicitly rather than relying on fallback precedence.

The current SUMO command is `sumo[(-gui)] -c city/montreal.sumocfg --start --delay 50` (and `--quit-on-end` headlessly). It does **not** pass an explicit SUMO seed. Phase 2.2D must not assume that Python-level seeds control SUMO demand/randomness. Before a repeated-run dataset is accepted, the implementation must determine whether the Montreal configuration has stochastic demand and, if so, pass and record an explicit SUMO seed. This is a prerequisite for including Phase 2.2C historical results in a strict matched-seed analysis.

### 2.3 Available per-run evidence

Each completed run can provide:

- `twin_ground_truth.csv` and `twin_filtered_states.csv`, matched offline exactly by vehicle ID and normalized simulation timestamp;
- `twin_filtered_evaluation.json`: speed and acceleration MAE, RMSE, bias, maximum/95th-percentile absolute error, plus measurement-updated and prediction-only groups;
- `twin_uncertainty_evaluation.json`: scalar normalized squared error, predicted standard deviation, 1-sigma, 2-sigma, and 95% coverage, absolute-error/sigma correlation, provenance groups, and age groups;
- `twin_degradation_summary.json`: requested window/configuration, sensor-suppression counters, and Twin V2X channel `generated`, `delivered`, `dropped`, `rejected`, `duplicates`, and `stale` statistics;
- `twin_v2x_messages.csv` and filtered-state freshness/provenance fields for recovery and message-age analysis.

`samtruck/offline_estimator_evaluator.py` and `samtruck/offline_uncertainty_evaluator.py` are offline-only. Position remains excluded: the estimator's unanchored longitudinal position does not share a valid coordinate reference with SUMO lane-local truth. NIS is unavailable because innovations and innovation covariance are not persisted; full-state NEES is unavailable because position truth is not coordinate compatible.

## 3. Proposed focused matrix

The following eight conditions are appropriate. They reuse the Phase 2.2C definitions exactly; no new controller, estimator, or sensor behavior is proposed.

| Condition | Existing Phase 2.2C ID | Evidence value | Existing seeds |
| --- | --- | --- | --- |
| Baseline | `p22c_baseline` | matched reference for all paired comparisons | 20260929, 20260930 |
| Medium sensor noise | `p22c_noise_medium` | combined radar-range 0.75 m and odometry-speed 0.30 m/s noise | 20260929, 20260930 |
| Medium radar dropout | `p22c_radar_medium` | deterministic Twin radar suppression, 3–6 s | 20260929, 20260930 |
| Medium own-odometry dropout | `p22c_odometry_medium` | prediction-only own-state interval, 3–6 s | 20260929, 20260930 |
| Extended own-odometry dropout | `p22c_odometry_extended` | stronger prediction/recovery stress, 3–9 s | 20260929 only |
| 600 ms Twin-only V2X latency | `p22c_v2x_latency_600` | delayed delivery/reordering evidence, 3–9 s | 20260929, 20260930 |
| 30% Twin-only V2X packet loss | `p22c_v2x_loss_30` | roadmap-relevant loss rate, 3–9 s | 20260929, 20260930 |
| Twin-only V2X outage | `p22c_v2x_outage` | complete delivery interruption and recovery, 3–7 s | 20260929, 20260930 |

Radar dropout is retained because its suppression/tracking evidence is meaningful, but it should not be expected to affect own-state speed/acceleration accuracy in the same way as odometry loss; report radar-track evidence separately. Likewise, the passive Twin does not feed V2X into the own-state filter, so latency/loss/outage results should primarily establish communication reliability/freshness behavior rather than claim a causal own-state-KF accuracy effect.

## 4. Recommended seed plan and run count

### Recommendation: 10 fully matched seeds per condition

Use the matched seed set:

`[20260929, 20260930, 20260931, 20260932, 20260933, 20260934, 20260935, 20260936, 20260937, 20260938]`

Run every one of the eight conditions for every seed. Ten repeated runs per condition is an exploratory but defensible balance: it supports paired seed-level deltas, dispersion summaries, and bootstrap/t-based interval estimates while remaining far short of a definitive traffic-population study. Two runs cannot estimate a useful distribution; ten still requires cautious, descriptive interpretation.

The desired balanced matrix contains **80 runs**.

### Two mutually exclusive dataset paths

The 15 existing Phase 2.2C runs relevant to this focused matrix are:

- two historical seeds (`20260929`, `20260930`) for seven conditions; and
- one historical seed (`20260929`) for extended odometry dropout.

They are pilot evidence by default. They may become part of the Phase 2.2D dataset only after the implementation verifies the reproducibility contract: identical normal scenario, duration, simulation step, SUMO network/configuration, code revision, Twin configuration, result completeness, and SUMO randomness/seed behavior.

#### Path A — fresh dataset: historical runs incompatible or unverifiable

Use this path if any required historical compatibility check fails, including an inability to establish compatible SUMO randomness/seed behavior. Do not mix the failing historical result into the new matrix.

1. **Reproducibility gate:** before accepting the matrix, verify the new manifest, explicit SUMO-seed policy, message outcomes, and deterministic DTO behavior using duplicate scratch runs.
2. **D1:** run five matched fresh seeds across all eight conditions: **40 fresh runs**.
3. **D2:** run the remaining five matched fresh seeds across all eight conditions: **40 additional fresh runs**.

Total: **80 fresh runs**.

#### Path B — historical runs verified compatible

Use this path only after every one of the 15 historical runs passes the full compatibility check, including SUMO randomness and seed behavior.

1. **Reproducibility gate:** verify that the historical and new manifests are compatible before reusing any historical data.
2. **D1:** complete the first five matched seeds across eight conditions. The historical set already contributes 15 eligible runs, so this requires **25 additional runs** to reach 40 first-stage runs.
3. **D2:** add seeds six through ten across all eight conditions: **40 additional runs**.

Total: **65 additional runs**, yielding the same 80-run, ten-seed balanced matrix.

If a compatibility check fails for any historical run, it must be excluded rather than silently retained. The implementation must either use Path A or document the exact reduced eligible historical subset and recalculate the remaining workload as `80 - eligible_historical_runs`; the staged seed layout must remain explicitly reported.

### Practical staged execution

No early stopping should be used to claim an effect. Staging limits wasted runs if the seed contract or artifact collection is defective. D1 and D2 must follow the count rules of the selected Path A or Path B above; they are not interchangeable.


## 5. Statistical analysis methodology

### 5.1 Unit of analysis and descriptive summaries

The unit of comparison is one completed **run**, not an individual correlated 0.1 s telemetry sample. For each condition and each existing scalar metric, report across seeds:

- valid-run count, unknown/failed/incomplete count, and excluded reason;
- mean and sample standard deviation;
- median and interquartile range;
- minimum and maximum;
- a 95% confidence interval for the run-level mean when assumptions are disclosed (Student-t interval) and/or a fixed-seed bootstrap interval for descriptive robustness.

With only ten runs, intervals are uncertainty descriptions, not evidence of population-normality. Preserve raw per-seed values in the summary so any aggregate is auditable.

### 5.2 Metrics to aggregate without redefining them

Aggregate existing values separately for `speed_mps` and `acceleration_mps2`:

- MAE and RMSE from the accuracy evaluator;
- scalar mean normalized squared error, mean predicted standard deviation, and absolute-error-versus-predicted-sigma correlation from the uncertainty evaluator;
- 1-sigma, 2-sigma, and 95% coverage;
- sample count and `sufficient_for_interpretation` flag;
- results split by estimate provenance (`measurement_updated`, `prediction_only`, `other_or_unavailable`) and measurement age (`fresh`, `aged`, `unavailable_age`) where the existing per-run group has valid samples.

Coverage is recorded as a per-run rate and summarized across seeds. It must not be presented as independent Bernoulli evidence across all time samples because adjacent samples are temporally correlated.

For degradation realization/recovery, aggregate existing counters and derive only transparent secondary descriptors from saved CSVs:

- requested versus observed radar/odometry suppression counts and rates;
- V2X generated, delivered, dropped, rejected, duplicate, stale, and outage-attempt counts; realised loss rate must state its denominator and exclude undelivered-at-termination messages explicitly;
- maximum/median component measurement age, prediction-only sample count, and first valid post-window fresh update/recovery time;
- radar-track presence/age only when a target existed; a no-target window is an unavailable observation, not evidence of a successful radar stress effect.

### 5.3 Matched baseline comparisons

For each degradation and seed `s`, calculate the same-seed delta:

`delta_m(s) = metric_degradation(s) - metric_baseline(s)`.

Report the paired delta's mean, standard deviation, median, IQR, range, and interval. Matched seeds reduce variance from shared simulator/route effects. A paired sign-permutation or Wilcoxon signed-rank result may be reported only as an exploratory sensitivity check and only when the number of valid complete pairs is adequate; no statistical-significance claim should be made from a short ten-seed matrix alone.

For metrics where lower is preferable (for example MAE/RMSE), positive deltas indicate worse values. For coverage, compare signed deviation from its nominal reference as well as raw coverage, while keeping the existing evaluator's definitions unchanged. Communication observations should be compared against the requested degradation and their own realized outcome counts—not against an invented accuracy causal effect.

### 5.4 Interpretation safeguards

- Do not pool measurement-updated and prediction-only groups into one conclusion without presenting them separately.
- Do not replace unavailable groups with zero.
- Do not infer covariance calibration solely from coverage or MNSSE in a short, correlated simulation.
- Do not claim a V2X degradation changes own-state filter accuracy unless the passive architecture provides a valid causal path and the paired data support it.
- Flag a condition as inconclusive when fewer than a predeclared minimum number of complete paired runs or relevant grouped samples exists.

## 6. Expected future output schema

Phase 2.2D implementation should not overwrite Phase 2.2C artifacts. It should add a versioned manifest and summary namespace, for example:

```text
scenarios/experiments/phase_2_2d.json
results/phase_2_2d/<condition>_s<seed>/     # per-run standard outputs
results/phase_2_2d_manifest.json
results/phase_2_2d_repeated_run_summary.json
results/phase_2_2d_paired_comparisons.json
```

The manifest should record: project revision/hash when available; SUMO binary/version and explicit SUMO seed; scenario seed; resolved sensor, Twin-V2X, and legacy-runner seeds; duration/step; network/configuration identifiers; condition/degradation window; and source artifact paths. The summary should contain raw run records, completion/verdict provenance, metrics by condition, realization/recovery evidence, aggregation metadata, and exclusions. The paired-comparisons file should retain each baseline/degradation seed pair and its delta.

## 7. Risks, limits, and computational cost

- The absent explicit SUMO seed is the principal reproducibility risk. It must be resolved before historical and fresh runs are pooled.
- The three local Python PRNGs use the same number by default but are independent streams. Future work should log resolved seeds rather than interpreting them as one shared generator.
- Twelve seconds yields only 120 simulation steps and short recovery windows. It supports controlled checks, not broad traffic-generalization claims.
- Output folders and SUMO/TraCI connections should be serialised or use isolated ports/IDs; parallel runs risk file/path and process collisions unless explicitly engineered.
- Some metrics can be unavailable because no radar target is present, an estimator group has insufficient samples, or run artifacts fail matching validation. These outcomes must remain explicit.
- Position, NIS, and full-vector NEES remain out of scope until compatible truth/reference or persisted innovations are available.
- Phase 2.2D must preserve the Twin-only degradation boundary: CACC/CTG behavior remains the official baseline and must not be used as an adjustable variable.

## 8. Later implementation plan (requires approval)

1. Verify whether SUMO traffic/demand uses randomness and establish an explicit recorded SUMO-seed policy without changing CACC or estimator settings.
2. Create a new Phase 2.2D manifest/configuration using the eight conditions and matched seed set; preserve Phase 2.2C config/results unchanged.
3. Add a runner/aggregator that invokes existing per-run evaluation only after simulation completion, validates artifacts, and retains raw values and failure evidence.
4. Run the reproducibility gate, then the staged 5-seed and 10-seed matrix.
5. Generate descriptive and paired summaries from existing metric schemas, including realization and recovery evidence.
6. Review whether evidence is sufficient for a Phase 2.3 design. Do not tune covariance, add V2X fusion, or adapt CACC as part of repeated-run analysis.

## 9. Unresolved decisions requiring approval

1. Should Phase 2.2D introduce an explicit SUMO seed, which would make the historical Phase 2.2C runs pilot-only rather than pooled evidence?
2. Is the recommended 10-seed, 80-run fresh matrix acceptable, or should the team approve the conditional 65-run incremental plan after compatibility verification?
3. Should the project predeclare a minimum count of complete matched pairs (recommended: at least 8 of 10) before reporting a paired comparison as interpretable?
4. Is a longer duration warranted for a later robustness phase, recognizing that it would define a different experiment family rather than extend this controlled 12.0 s matrix?

## 10. Scope confirmation

Only this design document was created. No SUMO run was started; no Python source, test, scenario, evaluator, existing result, Phase 2.2C documentation, CACC, Kalman, sensor, V2X, or Twin architecture file was modified.
