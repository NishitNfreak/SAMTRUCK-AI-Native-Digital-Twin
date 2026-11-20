# Milestone C — Active Decision Executive

## Scope

Milestone C adds an opt-in executive that applies bounded changes to follower
controller parameters. The default runner remains unchanged because the
executive is disabled unless both `--twin-advisor` and `--twin-executive` are
supplied.

## Architecture

```text
Twin-side DTOs + filtered state + delivered V2X
  -> TwinAdvisor -> AdvisorDecision command recommendation
  -> ActiveExecutive -> bounded follower CACC parameters
  -> existing CACCController.command() -> existing SUMO setSpeed path
```

SUMO truth is not on this path. It remains an offline evaluation CSV only.
The executive has no TraCI dependency and does not call any SUMO API.

## Implemented controls

`samtruck/executive.py` maintains independent follower state and maps advisor
recommendations to these settings:

| Advisor code | Executive command | CTG target | Mode | V2V | Cap |
| --- | --- | ---: | --- | --- | ---: |
| HOLD_NORMAL_CACC | HOLD_BASELINE | 0.8 s | CACC | enabled | none |
| INCREASE_GAP_RECOMMENDED | INCREASE_GAP_1_5S | 1.5 s | CACC | enabled | none |
| V2X_DEGRADED | FALLBACK_ACC | 1.2 s | ACC | disabled | none |
| SENSOR_AND_V2X_DEGRADED | FALLBACK_ACC_WIDE_GAP | 1.8 s | ACC | disabled | none |
| CRITICAL_AWARENESS | EMERGENCY_CAUTION | 2.0 s | speed override | disabled | 12.0 m/s |

Guardrails:

- CTG is bounded to `[0.8, 2.0] s`.
- CTG slew is bounded to `0.5 s/s`.
- Any lower-safety recommendation must wait 2.0 s before reducing the active
  safety level. This prevents a momentary recovered reading from immediately
  clearing ACC fallback or emergency caution.
- The controller interface validates CTG and cap inputs. Baseline CACC gains,
  gap-error terms, and normal non-executive behavior are preserved.

## Files changed

- Created `samtruck/executive.py`
- Modified `samtruck/controller.py`
- Modified `samtruck/twin_advisor.py`
- Modified `run_all_scenarios.py`
- Created `run_active_executive_demo.py`
- Created `tests/test_executive.py`
- Modified `tests/test_twin_advisor.py`
- Created this report.

## Fresh demonstration and result

The initial `results/active_executive_demo/` run exposed an early
de-escalation guard issue. It was preserved, not overwritten. The corrected,
fresh validation run is:

```powershell
python run_active_executive_demo.py --run-id active_executive_demo_retry --duration 12 --sumo-seed 20260929 --scenario-seed 20260929 --sensor-seed 20260929 --twin-v2x-seed 20260929 --advisor-period 1.0 --executive-period 1.0
```

Validated output: `results/active_executive_demo_retry/`.

- Duration: 12.0 s; all seeds: `20260929`.
- Twin-only odometry dropout and V2X outage: 3.0–7.0 s.
- Sensor noise: radar 1.50 m, odometry speed 0.60 m/s.
- Scenario result: **PASS**, collision count **0**.
- Actions: 240; applied CTG range: **0.8–2.0 s**.
- ACC actions: 46; speed-override actions: 40; speed-cap events: 40.
- Recovery guard active: 56 action records.

For `samtruck_follower_1`, actual actions include:

| Time | Requested command | Effective mode | Applied CTG | V2V | Cap |
| ---: | --- | --- | ---: | --- | ---: |
| 4.1 s | FALLBACK_ACC_WIDE_GAP | ACC | 1.65 s | off | none |
| 5.1 s | FALLBACK_ACC_WIDE_GAP | ACC | 1.80 s | off | none |
| 6.1 s | EMERGENCY_CAUTION | speed override | 1.95 s | off | 12.0 m/s |
| 7.1 s | HOLD_BASELINE | speed override (held) | 2.00 s | off | 12.0 m/s |
| 8.1 s | HOLD_BASELINE | CACC | 1.85 s | on | none |

The 7.1 s record proves the two-second recovery hold blocked a premature
de-escalation; later recovery used the CTG slew rate rather than a jump.

## Validation

```text
python -m unittest tests.test_executive tests.test_controller tests.test_twin_advisor -v
19 passed

python -m unittest discover -s tests -v
116 passed
```

`tests/test_executive.py` covers CTG slew limits, ACC fallback, emergency cap,
recovery hold, invalid commands, and the controller parameter interface.

## Safety boundaries

- Ground truth remains evaluation-only and is never passed to the Twin,
  advisor, or executive.
- The executive acts only through validated controller parameter methods; it
  does not call TraCI directly.
- ASCII-only live console output is retained.
- Existing result folders were not overwritten.

## Limitation

This is a deterministic rule-based executive evaluated in one controlled
scenario. It is not an AI controller, has not been statistically validated
across the D1/D2 matrix, and should not be represented as production-ready
adaptive control.
