# SAMTRUCK: SUMO Phase 1 Platoon Demo

This is the runnable Phase 1 demo for the truck-platooning presentation: three heavy vehicles drive an automatic origin-to-destination route in SUMO. No driver input and no ML model is used.

## Prerequisites checked

- SUMO 1.27.1 is installed at `E:\SUMO` (`sumo.exe`, `sumo-gui.exe`, `netconvert.exe`, and TraCI tools).
- Python is installed. No pip package is required; TraCI is supplied by SUMO.
- If SUMO is installed elsewhere, set the `SUMO_HOME` environment variable before running.

## Run

From `E:\SAMSUNGTRUCK`:

```powershell
python -m samtruck.run --scenario baseline --gui
python -m samtruck.run --scenario acceleration --gui
python -m samtruck.run --scenario emergency_brake --gui
python -m samtruck.run --scenario blackout_recovery --gui
```

### Montreal city integration

```powershell
python -m samtruck.montreal_demo --scenario baseline --gui
python -m samtruck.montreal_demo --scenario blackout_recovery --gui
```

This launches the imported downtown Montreal road network, building polygons, staged city traffic, and the three SAMTRUCK vehicles. The trucks follow a 45-edge, screenshot-aligned perimeter-and-central corridor loop. It has no default time limit; stop it with `Ctrl+C`. Add `--duration 180` for a bounded test. The blackout demo visibly exercises `CACC -> DEGRADED_CACC -> ACC_FALLBACK -> CACC`; red lights and surrounding traffic can then trigger the local radar/AEB safety layer.

Omit `--gui` for a quick headless run. Telemetry CSV and a scenario summary JSON appear in `outputs`.

### Full Scenario Suite

The 20 requested experiment definitions and the endless random-journey runner are ready under `scenarios/catalog`:

```powershell
python -m samtruck.scenario_suite --validate-catalog
python -m samtruck.scenario_suite --scenario emergency_braking --gui
python -m samtruck.scenario_suite --scenario cut_in --gui
python -m samtruck.scenario_suite --master --gui
```

See `docs/scenario_suite.md` for the scenario mapping, logged metrics, and reproducibility commands.

## What the demo shows

The leader follows a scripted path. Follower trucks use CACC with the presentation's constant-time-gap target of `10 m + 0.8 v`. With healthy V2X (under 100 ms), the state is `CACC`. At 100--1000 ms it becomes `DEGRADED_CACC`; an outage or stale message uses radar-only `ACC_FALLBACK` with `5 m + 1.8 v`. Local AEB commands take priority when sensed TTC is below 2 seconds or the gap is critical.

The fast demo implements baseline, leader acceleration, emergency braking, and V2V latency/blackout/recovery. The complete runnable catalog is recorded in `docs/scenario_suite.md`.

## 3D city and learning resources

Start with the official [2026 SUMO tutorial video](https://www.youtube.com/watch?v=zrMRpP47T_M), then use the [GUI 3D guide](https://sumo.dlr.de/docs/sumo-gui.html). The installed GUI runs the verified 2D visualization but does not expose the OpenSceneGraph 3D-view flag; a SUMO Windows "extra" build is required for true OSG 3D. For a larger city-scale network after this demo, use [LuSTScenario](https://github.com/lcodeca/LuSTScenario); it is a SUMO-ready Luxembourg network, but its maintainers note that its real-data validation used an old SUMO release. It is intentionally not pulled into this time-boxed starter. SUMO's [OpenStreetMap import guide](https://sumo.dlr.de/docs/Networks/Import/OpenStreetMap.html) is the next practical way to make the selected city local to your demo.
