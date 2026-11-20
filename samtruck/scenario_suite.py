from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from pathlib import Path

from .config import OUTPUT_DIR, SUMO_HOME
from .controller import CACCController
from .montreal_demo import CITY_DIR, TRUCK_IDS, add_platoon, select_truck_loop
from .run import prepare_traci


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CATALOG_DIR = PROJECT_ROOT / "scenarios" / "catalog"
MASTER_FILE = PROJECT_ROOT / "scenarios" / "random_journey.json"

REQUIRED_SCENARIOS = {
    "normal_platooning", "leader_acceleration", "leader_deceleration", "emergency_braking",
    "cut_in", "cut_out", "slow_vehicle_ahead", "traffic_stop_restart", "platoon_merge",
    "platoon_split", "lane_change_merge", "traffic_congestion", "v2x_latency",
    "v2x_packet_loss", "v2x_latency_packet_loss", "v2x_outage", "lane_blockage_roadwork",
    "traffic_light_stop_restart", "onramp_vehicle_merge", "multiple_disturbances",
}


def load_specs() -> dict[str, dict]:
    specs = {}
    for path in sorted(CATALOG_DIR.glob("*.json")):
        spec = json.loads(path.read_text(encoding="utf-8"))
        specs[spec["id"]] = spec
    return specs


def validate_catalog(specs: dict[str, dict]) -> list[str]:
    missing = sorted(REQUIRED_SCENARIOS - set(specs))
    unexpected = sorted(set(specs) - REQUIRED_SCENARIOS)
    errors = [f"missing scenario definitions: {', '.join(missing)}"] if missing else []
    if unexpected:
        errors.append(f"unexpected scenario definitions: {', '.join(unexpected)}")
    for scenario_id, spec in specs.items():
        if not isinstance(spec.get("events"), list) or "title" not in spec or "duration_s" not in spec:
            errors.append(f"invalid schema: {scenario_id}")
    return errors


def active_events(events: list[dict], now: float) -> list[dict]:
    return [event for event in events if event["at_s"] <= now < event["at_s"] + event.get("duration_s", 0.1)]


def add_city_vehicle(traci, route_edges: list[str], vehicle_id: str, position: float, speed: float) -> bool:
    try:
        route_id = f"{vehicle_id}_route"
        traci.route.add(route_id, route_edges)
        # A departure at speed can be rejected when an imported network has a
        # connection that never receives green. Spawn stopped, then take over.
        traci.vehicle.add(vehicle_id, route_id, typeID="passenger", depart="now", departLane="best",
                          departPos=str(max(1.0, position)), departSpeed="0")
        traci.vehicle.setSpeedMode(vehicle_id, 7)
        traci.vehicle.setSpeed(vehicle_id, max(0.0, speed))
        return True
    except Exception:
        return False


def inject_relative_vehicle(traci, event_id: str, leader_id: str, distance_ahead: float, speed: float) -> str | None:
    if leader_id not in traci.vehicle.getIDList():
        return None
    route = list(traci.vehicle.getRoute(leader_id))
    route_index = traci.vehicle.getRouteIndex(leader_id)
    road = traci.vehicle.getRoadID(leader_id)
    if road.startswith(":") or route_index < 0:
        return None
    remaining = route[route_index:]
    if not remaining or remaining[0] != road:
        remaining = [road] + remaining
    lane_length = traci.lane.getLength(f"{road}_0")
    position = min(lane_length - 6.0, traci.vehicle.getLanePosition(leader_id) + distance_ahead)
    if position <= 4.0:
        return None
    vehicle_id = f"event_{event_id}"
    return vehicle_id if add_city_vehicle(traci, remaining, vehicle_id, position, speed) else None


def schedule_master(seed: int, specs: dict[str, dict]) -> dict:
    master = json.loads(MASTER_FILE.read_text(encoding="utf-8"))
    rng = random.Random(seed)
    events = []
    current = master["random_window_s"][0]
    patterns = [specs[scenario_id] for scenario_id in sorted(REQUIRED_SCENARIOS)]
    while current < 3600:
        cycle = patterns[:]
        rng.shuffle(cycle)
        for pattern in cycle:
            source_events = pattern["events"] or [{"at_s": 0, "type": "baseline", "duration_s": 8}]
            first_time = min(event["at_s"] for event in source_events)
            for source_event in source_events:
                event = dict(source_event)
                event["at_s"] = round(current + source_event["at_s"] - first_time, 1)
                event["scenario_id"] = pattern["id"]
                events.append(event)
            latest_end = max(event["at_s"] + event.get("duration_s", 0.1) for event in source_events)
            current += latest_end - first_time + rng.uniform(*master["random_window_s"])
            if current >= 3600:
                break
    return {"id": "random_journey", "title": master["title"], "duration_s": 0, "events": events, "seed": seed}


def event_speed(now: float, events: list[dict]) -> float:
    speed = 10.0
    for event in active_events(events, now):
        kind = event["type"]
        if kind == "leader_speed":
            speed = event["speed_mps"]
        elif kind == "traffic_stop":
            speed = 0.0
        elif kind == "emergency_brake":
            elapsed = now - event["at_s"]
            speed = max(0.0, 10.0 - 5.0 * elapsed)
    return speed


def v2v_state(now: float, events: list[dict], rng: random.Random) -> tuple[float | None, bool]:
    delay = 40.0
    loss_rate = 0.0
    outage = False
    for event in active_events(events, now):
        if event["type"] == "v2x_delay":
            delay = event["latency_ms"]
        elif event["type"] == "packet_loss":
            loss_rate = max(loss_rate, event["loss_rate"])
        elif event["type"] == "v2x_outage":
            outage = True
    return (None if outage else delay), rng.random() < loss_rate


_loop_replacement_failed: set[tuple[str, int]] = set()


def ensure_loop(traci, vehicle_id: str, loop_edges: list[str]) -> None:
    try:
        route = list(traci.vehicle.getRoute(vehicle_id))
        route_index = traci.vehicle.getRouteIndex(vehicle_id)
        if route_index < 0 or route_index >= len(route):
            return
        current_edge = route[route_index]
        if current_edge != loop_edges[-1] or route_index != len(route) - 1:
            return

        guard_key = (vehicle_id, route_index)
        if guard_key in _loop_replacement_failed:
            return

        replacement = [current_edge] + loop_edges
        traci.vehicle.setRoute(vehicle_id, replacement)
    except Exception:
        _loop_replacement_failed.add((vehicle_id, route_index))


def coordinated_leader_speed(traci, nominal_target_speed: float) -> float:
    leader_id = TRUCK_IDS[0]
    follower_1_id = TRUCK_IDS[1]

    active = set(traci.vehicle.getIDList())
    if leader_id not in active or follower_1_id not in active:
        return nominal_target_speed

    sensed = traci.vehicle.getLeader(follower_1_id, 150.0)
    truck2_speed = traci.vehicle.getSpeed(follower_1_id)

    sensed_leader_id = sensed[0] if sensed else None
    needs_coordination = (sensed_leader_id != leader_id) or (nominal_target_speed - truck2_speed > 1.5)

    if needs_coordination:
        capped_speed = max(4.0, truck2_speed + 1.0)
        return min(nominal_target_speed, capped_speed)

    return nominal_target_speed



def run_spec(spec: dict, gui: bool, duration_override: float | None) -> Path:
    prepare_traci()
    import traci  # type: ignore

    rng = random.Random(spec.get("seed", 20260929))
    duration = spec["duration_s"] if duration_override is None else duration_override
    OUTPUT_DIR.mkdir(exist_ok=True)
    telemetry_file = OUTPUT_DIR / f"scenario_{spec['id']}.csv"
    summary_file = OUTPUT_DIR / f"scenario_{spec['id']}_summary.json"
    binary = SUMO_HOME / "bin" / ("sumo-gui.exe" if gui else "sumo.exe")
    command = [str(binary), "-c", str(CITY_DIR / "montreal.sumocfg"), "--start", "--delay", "50"]
    if not gui:
        command.append("--quit-on-end")
    controller = CACCController(desired_time_gap=0.8, min_gap=10.0, max_speed=18.0)
    rows, event_log, injected = [], [], {}
    last_v2v: dict[str, tuple[float, float, float]] = {}
    state_changes: list[dict] = []
    prior_state: dict[str, str] = {}
    metrics = {"min_gap_m": float("inf"), "min_ttc_s": float("inf"), "collisions": 0, "dropped_packets": 0,
               "sent_packets": 0, "max_accel": {vehicle: 0.0 for vehicle in TRUCK_IDS}}

    traci.start(command)
    try:
        route_id, loop_edges = select_truck_loop(traci)
        add_platoon(traci, route_id, loop_edges[0])
        print(f"Scenario '{spec['id']}', loop length {len(loop_edges)} edges. Ctrl+C stops endless runs.")
        triggered: set[int] = set()
        while (duration <= 0 or traci.simulation.getTime() < duration) and traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            now = traci.simulation.getTime()
            active = set(traci.vehicle.getIDList())
            for index, event in enumerate(spec["events"]):
                if index in triggered or now < event["at_s"]:
                    continue
                triggered.add(index)
                kind = event["type"]
                event_log.append({"time_s": round(now, 1), "event": kind, "status": "triggered"})
                if kind in {"cut_in", "slow_vehicle", "roadwork", "onramp_merge"}:
                    speed = 4.0 if kind in {"slow_vehicle", "roadwork"} else 8.0
                    injected[kind] = inject_relative_vehicle(traci, f"{kind}_{index}", TRUCK_IDS[0], 32.0, speed)
                elif kind == "platoon_merge":
                    injected[kind] = inject_relative_vehicle(traci, f"merge_{index}", TRUCK_IDS[-1], -5.0, 8.0)
                elif kind == "traffic_burst":
                    created = 0
                    for number in range(event.get("count", 7)):
                        if inject_relative_vehicle(traci, f"traffic_{index}_{number}", TRUCK_IDS[0], 45 + number * 12, 5.0):
                            created += 1
                    event_log[-1]["created"] = created
                elif kind == "lane_change" and TRUCK_IDS[0] in active:
                    road = traci.vehicle.getRoadID(TRUCK_IDS[0])
                    lanes = traci.edge.getLaneNumber(road) if not road.startswith(":") else 1
                    if lanes > 1:
                        traci.vehicle.changeLane(TRUCK_IDS[0], 1, 4.0)
                elif kind == "cut_out":
                    vehicle_id = injected.get("cut_in")
                    if vehicle_id in active:
                        traci.vehicle.remove(vehicle_id)
            if TRUCK_IDS[0] not in active:
                continue
            for vehicle_id in TRUCK_IDS:
                if vehicle_id in active:
                    traci.vehicle.setSpeedMode(vehicle_id, 7)
                    traci.vehicle.setLaneChangeMode(vehicle_id, 1621)
                    ensure_loop(traci, vehicle_id, loop_edges)
            target_sp = event_speed(now, spec["events"])
            coord_sp = coordinated_leader_speed(traci, target_sp)
            traci.vehicle.setSpeed(TRUCK_IDS[0], coord_sp)
            link_latency, packet_lost = v2v_state(now, spec["events"], rng)
            for preceding, follower in zip(TRUCK_IDS, TRUCK_IDS[1:]):
                if preceding not in active or follower not in active:
                    continue
                sensed = traci.vehicle.getLeader(follower, 150.0)
                if sensed is None:
                    traci.vehicle.setSpeed(follower, max(0.0, traci.vehicle.getSpeed(preceding) - 1.0))
                    continue
                sensed_id, gap = sensed
                follower_speed = traci.vehicle.getSpeed(follower)
                sensed_speed = traci.vehicle.getSpeed(sensed_id)
                metrics["sent_packets"] += 1
                if link_latency is not None and not packet_lost:
                    last_v2v[follower] = (now, traci.vehicle.getSpeed(preceding), traci.vehicle.getAcceleration(preceding))
                elif packet_lost:
                    metrics["dropped_packets"] += 1
                message = last_v2v.get(follower)
                age_ms = None if message is None else (now - message[0]) * 1000 + (link_latency or 0)
                usable = message is not None and link_latency is not None and age_ms is not None and age_ms <= 1000
                command_out = controller.command(
                    follower_speed=follower_speed,
                    sensed_leader_speed=sensed_speed,
                    sensed_gap=gap,
                    v2v_leader_speed=message[1] if usable else None,
                    v2v_leader_accel=message[2] if usable else None,
                    link_latency_ms=age_ms if usable else None,
                )
                if any(event["type"] == "platoon_split" for event in active_events(spec["events"], now)) and follower == TRUCK_IDS[-1]:
                    command_out = command_out.__class__(max(0.0, command_out.target_speed - 3.0), command_out.acceleration,
                                                        command_out.state, "split_separation", command_out.desired_gap)
                traci.vehicle.setSpeed(follower, command_out.target_speed)
                closing_speed = follower_speed - sensed_speed
                ttc = gap / closing_speed if closing_speed > 0.05 else float("inf")
                metrics["min_gap_m"] = min(metrics["min_gap_m"], gap)
                metrics["min_ttc_s"] = min(metrics["min_ttc_s"], ttc)
                metrics["max_accel"][follower] = max(metrics["max_accel"][follower], abs(traci.vehicle.getAcceleration(follower)))
                if prior_state.get(follower) != command_out.state.value:
                    state_changes.append({"time_s": round(now, 1), "vehicle": follower, "state": command_out.state.value})
                    prior_state[follower] = command_out.state.value
                rows.append({"time_s": f"{now:.1f}", "vehicle_id": follower, "preceding_vehicle": preceding,
                             "gap_m": f"{gap:.2f}", "desired_gap_m": f"{command_out.desired_gap:.2f}",
                             "spacing_error_m": f"{gap - command_out.desired_gap:.2f}", "speed_mps": f"{follower_speed:.2f}",
                             "ttc_s": "" if math.isinf(ttc) else f"{ttc:.2f}",
                             "v2v_age_ms": "outage" if link_latency is None else f"{age_ms or 0:.0f}",
                             "resilience_state": command_out.state.value, "controller_reason": command_out.reason})
            metrics["collisions"] += len(traci.simulation.getCollidingVehiclesIDList())
    except KeyboardInterrupt:
        print("Scenario stopped by user.")
    finally:
        traci.close(False)
    with telemetry_file.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]) if rows else [])
        writer.writeheader()
        writer.writerows(rows)
    errors = [float(row["spacing_error_m"]) for row in rows]
    metrics["spacing_error_rms_m"] = round(math.sqrt(sum(error * error for error in errors) / len(errors)), 3) if errors else None
    metrics["min_gap_m"] = round(metrics["min_gap_m"], 3) if not math.isinf(metrics["min_gap_m"]) else None
    metrics["min_ttc_s"] = round(metrics["min_ttc_s"], 3) if not math.isinf(metrics["min_ttc_s"]) else None
    summary = {"scenario": spec["id"], "title": spec["title"], "telemetry_csv": str(telemetry_file),
               "events": event_log, "state_changes": state_changes, "metrics": metrics}
    summary_file.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return telemetry_file


def main() -> None:
    specs = load_specs()
    parser = argparse.ArgumentParser(description="Run individual SAMTRUCK city scenarios or an endless randomized journey.")
    parser.add_argument("--scenario", choices=sorted(specs), help="Scenario ID from scenarios/catalog")
    parser.add_argument("--master", action="store_true", help="Run the endless seeded master journey until Ctrl+C.")
    parser.add_argument("--validate-catalog", action="store_true", help="Check that all 20 required scenario files are present.")
    parser.add_argument("--seed", type=int, default=20260929)
    parser.add_argument("--duration", type=float, default=None, help="Override duration; 0 means run until Ctrl+C.")
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    if args.validate_catalog:
        errors = validate_catalog(specs)
        if errors:
            parser.error("; ".join(errors))
        print(f"Scenario catalog valid: {len(specs)} required scenarios found.")
        return
    if args.master:
        spec = schedule_master(args.seed, specs)
    elif args.scenario:
        spec = specs[args.scenario]
    else:
        parser.error("choose --scenario or --master")
    run_spec(spec, args.gui, args.duration)


if __name__ == "__main__":
    main()
