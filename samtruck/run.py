from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

from .config import OUTPUT_DIR, SCENARIO_DIR, SUMO_HOME, SimulationConfig
from .controller import CACCController


SCENARIOS = ("baseline", "acceleration", "emergency_brake", "blackout_recovery")


def prepare_traci() -> None:
    tools = SUMO_HOME / "tools"
    if not (SUMO_HOME / "bin" / "sumo.exe").exists():
        raise FileNotFoundError(f"SUMO was not found at {SUMO_HOME}. Set SUMO_HOME to your SUMO installation.")
    if str(tools) not in sys.path:
        sys.path.append(str(tools))


def leader_speed(scenario: str, time_s: float) -> float:
    if scenario == "acceleration":
        if 25.0 <= time_s < 29.0:
            return 18.0 + (time_s - 25.0)
        if 29.0 <= time_s < 42.0:
            return 22.0
        if 42.0 <= time_s < 43.0:
            return 22.0 - 4.0 * (time_s - 42.0)
    if scenario == "emergency_brake":
        if 42.0 <= time_s < 46.0:
            return max(0.0, 18.0 - 4.5 * (time_s - 42.0))
        if time_s >= 46.0:
            return 16.0
    return 18.0


def link_latency(scenario: str, time_s: float) -> float | None:
    if scenario != "blackout_recovery":
        return 40.0
    if 12.0 <= time_s < 22.0:
        return 160.0
    if 22.0 <= time_s < 34.0:
        return None
    return 40.0


def run_simulation(cfg: SimulationConfig, use_gui: bool = False) -> Path:
    prepare_traci()
    import traci  # type: ignore

    OUTPUT_DIR.mkdir(exist_ok=True)
    telemetry = OUTPUT_DIR / f"{cfg.scenario}_telemetry.csv"
    summary_file = OUTPUT_DIR / f"{cfg.scenario}_summary.json"
    binary = SUMO_HOME / "bin" / ("sumo-gui.exe" if use_gui else "sumo.exe")
    sumocfg = SCENARIO_DIR / "city_platoon.sumocfg"
    if not sumocfg.exists():
        sumocfg = SCENARIO_DIR / "platoon.sumocfg"
    command = [str(binary), "-c", str(sumocfg), "--step-length", str(cfg.step_length)]

    if use_gui:
        command.extend(["--start", "--delay", "50"])
    else:
        command.append("--quit-on-end")

    controller = CACCController(cfg.desired_time_gap, cfg.min_gap, cfg.max_speed)
    rows: list[dict[str, object]] = []
    transitions: list[dict[str, object]] = []
    prior_state: dict[str, str] = {}
    aeb_events = 0
    min_gap = float("inf")
    min_ttc = float("inf")
    traci.start(command)
    try:
        while traci.simulation.getTime() < cfg.duration and traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            now = traci.simulation.getTime()
            active = set(traci.vehicle.getIDList())
            if "truck_0" not in active:
                continue
            # The scripted leader is the trajectory source, so bypass SUMO's
            # insertion acceleration guard from the first active step.
            traci.vehicle.setSpeedMode("truck_0", 0)
            traci.vehicle.setSpeed("truck_0", leader_speed(cfg.scenario, now))
            for preceding, follower in zip(cfg.truck_ids, cfg.truck_ids[1:]):
                if preceding not in active or follower not in active:
                    continue
                # A platoon must retain a fixed ordering; the scenario's road
                # supports multiple lanes, but autonomous passing is out of scope.
                traci.vehicle.setLaneChangeMode(follower, 0)
                sensed = traci.vehicle.getLeader(follower, 150.0)
                if sensed is None:
                    continue
                sensed_id, gap = sensed
                follower_speed = traci.vehicle.getSpeed(follower)
                sensed_speed = traci.vehicle.getSpeed(sensed_id)
                predecessor_accel = traci.vehicle.getAcceleration(preceding)
                latency = link_latency(cfg.scenario, now)
                command_out = controller.command(
                    follower_speed, sensed_speed, gap,
                    sensed_speed if latency is not None else None,
                    predecessor_accel if latency is not None else None,
                    latency,
                )
                traci.vehicle.setSpeed(follower, command_out.target_speed)
                closing_speed = follower_speed - sensed_speed
                ttc = gap / closing_speed if closing_speed > 0.05 else float("inf")
                min_gap = min(min_gap, gap)
                min_ttc = min(min_ttc, ttc)
                if command_out.reason == "radar_aeb":
                    aeb_events += 1
                if prior_state.get(follower) != command_out.state.value:
                    transitions.append({"time_s": round(now, 1), "vehicle": follower, "state": command_out.state.value})
                    prior_state[follower] = command_out.state.value
                rows.append({
                    "time_s": f"{now:.1f}", "vehicle_id": follower, "preceding_vehicle": preceding,
                    "sensed_vehicle": sensed_id, "speed_mps": f"{follower_speed:.2f}",
                    "gap_m": f"{gap:.2f}", "ttc_s": "" if ttc == float("inf") else f"{ttc:.2f}",
                    "v2v_latency_ms": "blackout" if latency is None else f"{latency:.0f}",
                    "resilience_state": command_out.state.value, "target_speed_mps": f"{command_out.target_speed:.2f}",
                    "desired_gap_m": f"{command_out.desired_gap:.2f}", "controller_reason": command_out.reason,
                })
    finally:
        traci.close(False)

    with telemetry.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else [])
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "scenario": cfg.scenario, "telemetry_csv": str(telemetry), "samples": len(rows),
        "min_gap_m": round(min_gap, 2) if min_gap != float("inf") else None,
        "min_ttc_s": round(min_ttc, 2) if min_ttc != float("inf") else None,
        "aeb_command_count": aeb_events, "state_transitions": transitions,
    }
    summary_file.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return telemetry


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the deterministic SAMTRUCK SUMO platoon demo.")
    parser.add_argument("--scenario", choices=SCENARIOS, default="baseline")
    parser.add_argument("--gui", action="store_true", help="Open SUMO GUI for the 3D visual demo.")
    args = parser.parse_args()
    run_simulation(SimulationConfig(scenario=args.scenario), args.gui)


if __name__ == "__main__":
    main()
