# Resilience Scenario Matrix

The uploaded project material defines Phase 1 as SUMO plus TraCI with deterministic CACC, V2V resilience, and local sensing fallback. AI is deliberately deferred until a later abnormal-event phase.

| Scenario | Phase 1 intent | Demo status |
| --- | --- | --- |
| Baseline | 3-truck CACC/CTG formation | Implemented |
| Leader acceleration | Follow the leader speed increase | Implemented |
| Emergency braking | Brake-wave response and local safety guard | Implemented |
| V2V latency | Degraded CACC with conservative gain | Implemented inside blackout/recovery |
| Blackout | Radar-only ACC fallback | Implemented |
| Recovery | Return from ACC fallback to CACC | Implemented |
| Uphill | Grade-aware spacing and power limits | Next |
| Downhill | Grade-aware braking and spacing | Next |
| Low friction | Reduced deceleration and larger safety gap | Next |
| Obstacle | Detect, brake, and re-form | Next |
| Slow vehicle | Follow/overtake policy | Next |
| Cut-in | Safe gap response to merging traffic | Next |
| Cut-out | Reacquire predecessor and close formation safely | Next |
| Compound failure | Combine link degradation with an environmental event | Next |

## Resilience state machine

`CACC -> DEGRADED_CACC -> ACC_FALLBACK -> CACC`

An `EMERGENCY_BRAKE` state can be entered from any state when local radar reports a critical gap or TTC below 2 seconds. It is a local safety override, not AI-driven driving.
