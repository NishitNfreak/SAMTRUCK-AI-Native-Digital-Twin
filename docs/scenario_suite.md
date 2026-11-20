# Scenario Suite

`samtruck.scenario_suite` is the Phase 1 SUMO + TraCI experiment runner. It uses a deterministic controller, not ML or AI.

## Controller and Measurements

The normal CACC constant-time-gap reference from the VARAPU presentation is `d_des = d0 + h * v_follower`, with `d0 = 10 m` and `h = 0.8 s`.

It uses the relative-gap error and relative-speed error in the CACC acceleration command. Fresh V2X data below 100 ms uses `CACC`; 100-1000 ms uses `DEGRADED_CACC`; an outage or stale data uses radar-only `ACC_FALLBACK` with a 5 m + 1.8 s headway. Local radar AEB has priority at TTC below 2 s or a gap below 5 m.

Each run records gap, desired gap, spacing error, speed, TTC, V2X age, resilience state, collisions, packet statistics, and acceleration into `outputs/scenario_<id>.csv` with a matching summary JSON. The stated project targets, such as TTC above 3.5 s, remain validation targets rather than claimed results.

## Catalog

There are 20 individual definitions under `scenarios/catalog`, matching the requested list. The runner maps them to SUMO actions:

| Group | Scenario actions |
| --- | --- |
| Platoon motion | Normal, acceleration, deceleration, emergency braking, stop/restart |
| Interaction | Cut-in, cut-out, slow vehicle, merge, split, lane change, congestion, on-ramp merge |
| V2X resilience | Latency, packet loss, combined loss/delay, temporary outage/recovery |
| ITS | Roadwork/lane blockage, traffic-light stop/restart, multiple disturbances |

The imported Montreal map has no terrain model, so grades are reserved for a later network-elevation experiment rather than being mislabeled as physically validated slope behavior.

## Commands

Run all checks:

```powershell
cd E:\SAMSUNGTRUCK
python -m samtruck.scenario_suite --validate-catalog
```

Run any one scenario:

```powershell
python -m samtruck.scenario_suite --scenario emergency_braking --gui
python -m samtruck.scenario_suite --scenario v2x_latency_packet_loss --gui
python -m samtruck.scenario_suite --scenario cut_in --gui
```

Run the endless randomized journey. It continues through the 45-edge route and cycles through all 20 test patterns in a seeded randomized order, with 18-45 seconds between patterns. Stop it with `Ctrl+C`.

```powershell
python -m samtruck.scenario_suite --master --gui
```

Use `--duration 120` for a bounded test, or `--seed 42` to reproduce a different random sequence.
