from __future__ import annotations

import argparse
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from .config import OUTPUT_DIR, SUMO_HOME
from .controller import CACCController
from .run import SCENARIOS, leader_speed, link_latency, prepare_traci


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CITY_DIR = PROJECT_ROOT / "city" / "montreal"
TRUCK_IDS = ("samtruck_leader", "samtruck_follower_1", "samtruck_follower_2")
LOOP_CACHE = CITY_DIR / "samtruck_loop_edges.txt"


def truck_route(traci, start: str, end: str) -> list[str]:
    try:
        return list(traci.simulation.findRoute(start, end, vType="platoon_truck").edges)
    except Exception:
        return []


def city_link_latency(scenario: str, time_s: float) -> float | None:
    if scenario != "blackout_recovery":
        return link_latency(scenario, time_s)
    if 1.0 <= time_s < 3.0:
        return 160.0
    if 3.0 <= time_s < 6.0:
        return None
    return 40.0


def candidate_pairs() -> list[tuple[str, str]]:
    root = ET.parse(CITY_DIR / "random_trips_3.xml").getroot()
    return [(trip.attrib["from"], trip.attrib["to"]) for trip in root.findall("trip")]


def city_anchor_candidates(traci) -> list[list[str]]:
    """Pick map-relative anchors matching the requested outer-and-central loop."""
    candidates: list[tuple[str, float, float]] = []
    for edge_id in traci.edge.getIDList():
        if edge_id.startswith(":"):
            continue
        lane_id = f"{edge_id}_0"
        if lane_id not in traci.lane.getIDList():
            continue
        allowed = traci.lane.getAllowed(lane_id)
        if allowed and "truck" not in allowed:
            continue
        shape = traci.lane.getShape(lane_id)
        if len(shape) < 2:
            continue
        x = sum(point[0] for point in shape) / len(shape)
        y = sum(point[1] for point in shape) / len(shape)
        candidates.append((edge_id, x, y))
    if not candidates:
        raise RuntimeError("No drivable road edges were found in the Montreal network.")
    min_x, max_x = min(item[1] for item in candidates), max(item[1] for item in candidates)
    min_y, max_y = min(item[2] for item in candidates), max(item[2] for item in candidates)
    targets = ((0.24, 0.82), (0.55, 0.83), (0.84, 0.60), (0.64, 0.17), (0.22, 0.46), (0.49, 0.58))
    anchor_options: list[list[str]] = []
    for fraction_x, fraction_y in targets:
        target_x = min_x + fraction_x * (max_x - min_x)
        target_y = min_y + fraction_y * (max_y - min_y)
        nearest = sorted(candidates, key=lambda edge: math.hypot(edge[1] - target_x, edge[2] - target_y))[:40]
        legal = []
        for edge_id, _, _ in nearest:
            if not anchor_options and traci.lane.getLength(f"{edge_id}_0") < 90:
                continue
            if truck_route(traci, edge_id, edge_id):
                legal.append(edge_id)
        anchor_options.append(legal)
    return anchor_options


def select_truck_loop(traci) -> tuple[str, list[str]]:
    if LOOP_CACHE.exists():
        cached_edges = LOOP_CACHE.read_text(encoding="utf-8").split()
        if cached_edges and all(edge in traci.edge.getIDList() for edge in cached_edges):
            route_id = "samtruck_montreal_loop"
            traci.route.add(route_id, cached_edges)
            return route_id, cached_edges
    anchor_options = city_anchor_candidates(traci)
    loop_edges: list[str] = []
    for start in anchor_options[0]:
        anchors = [start]
        segments: list[list[str]] = []
        valid = True
        for options in anchor_options[1:]:
            segment = next((truck_route(traci, anchors[-1], end)
                            for end in options
                            if truck_route(traci, anchors[-1], end)), None)
            if segment is None:
                valid = False
                break
            segments.append(segment)
            anchors.append(segment[-1])
        if not valid:
            continue
        closing = truck_route(traci, anchors[-1], start)
        if not closing:
            continue
        loop_edges = segments[0]
        for segment in segments[1:] + [closing]:
            loop_edges.extend(segment[1:])
        break
    if len(loop_edges) < 8:
        raise RuntimeError("No truck-legal loop could be formed from the requested city corridor.")
    route_id = "samtruck_montreal_loop"
    traci.route.add(route_id, loop_edges)
    LOOP_CACHE.write_text("\n".join(loop_edges) + "\n", encoding="utf-8")
    return route_id, loop_edges


def add_platoon(traci, route_id: str, first_edge: str) -> None:
    first_lane_length = traci.lane.getLength(f"{first_edge}_0")
    # Start with approximately 20 m of physical clearance.  The controller's
    # 10 m + 0.8 s CTG target can then settle without an artificial AEB event.
    leader_position = min(120.0, first_lane_length - 14.0)
    for index, vehicle_id in enumerate(TRUCK_IDS):
        traci.vehicle.add(
            vehicle_id,
            route_id,
            typeID="platoon_truck",
            depart="now",
            departLane="0",
            departPos=str(leader_position - index * 32.0),
            departSpeed="max",
        )
        traci.vehicle.setColor(vehicle_id, (0, 135 + index * 45, 255 - index * 45, 255))


def run(scenario: str, gui: bool, duration: float) -> Path:
    prepare_traci()
    import traci  # type: ignore

    OUTPUT_DIR.mkdir(exist_ok=True)
    output = OUTPUT_DIR / f"montreal_{scenario}_telemetry.csv"
    binary = SUMO_HOME / "bin" / ("sumo-gui.exe" if gui else "sumo.exe")
    command = [str(binary), "-c", str(CITY_DIR / "montreal.sumocfg"), "--start", "--delay", "50"]
    if not gui:
        command.append("--quit-on-end")
    controller = CACCController(desired_time_gap=1.2, min_gap=4.0, max_speed=25.0)
    rows: list[dict[str, object]] = []
    transitions: dict[str, str] = {}

    traci.start(command)
    try:
        route_id, edges = select_truck_loop(traci)
        add_platoon(traci, route_id, edges[0])
        print(f"Platoon loop: {len(edges)} edges. Press Ctrl+C to stop.")
        while (duration <= 0 or traci.simulation.getTime() < duration) and traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            now = traci.simulation.getTime()
            active = set(traci.vehicle.getIDList())
            if TRUCK_IDS[0] not in active:
                continue
            # The imported map contains legacy signal-connection metadata. The
            # demo platoon keeps SUMO's speed/gap safeguards but does not wait
            # for those signal permissions along its pre-authorized corridor.
            for vehicle_id in TRUCK_IDS:
                if vehicle_id in active:
                    traci.vehicle.setSpeedMode(vehicle_id, 7)
            # Default strategic lane-change behavior lets the leader get into
            # the lane required by the next connected turn.
            traci.vehicle.setLaneChangeMode(TRUCK_IDS[0], 1621)
            traci.vehicle.setSpeed(TRUCK_IDS[0], leader_speed(scenario, now))
            # Reapply the closed route on its final edge, before SUMO can mark
            # the leader as arrived. Followers inherit the same loop route.
            for vehicle_id in TRUCK_IDS:
                if vehicle_id in active and traci.vehicle.getRouteIndex(vehicle_id) >= len(edges) - 1:
                    traci.vehicle.setRoute(vehicle_id, edges)
            for preceding, follower in zip(TRUCK_IDS, TRUCK_IDS[1:]):
                if preceding not in active or follower not in active:
                    continue
                traci.vehicle.setLaneChangeMode(follower, 1621)
                sensed = traci.vehicle.getLeader(follower, 150.0)
                if sensed is None:
                    # The predecessor may be in an adjacent lane while it is
                    # positioning for a turn. Use a conservative speed
                    # reference until it regains radar line-of-sight.
                    predecessor_speed = traci.vehicle.getSpeed(preceding)
                    traci.vehicle.setSpeed(follower, max(0.0, predecessor_speed - 1.0))
                    continue
                sensed_id, gap = sensed
                follower_speed = traci.vehicle.getSpeed(follower)
                sensed_speed = traci.vehicle.getSpeed(sensed_id)
                latency = city_link_latency(scenario, now)
                command_out = controller.command(
                    follower_speed=follower_speed,
                    sensed_leader_speed=sensed_speed,
                    sensed_gap=gap,
                    v2v_leader_speed=sensed_speed if latency is not None else None,
                    v2v_leader_accel=traci.vehicle.getAcceleration(preceding) if latency is not None else None,
                    link_latency_ms=latency,
                )
                traci.vehicle.setSpeed(follower, command_out.target_speed)
                if transitions.get(follower) != command_out.state.value:
                    transitions[follower] = command_out.state.value
                    print(f"{now:5.1f}s {follower}: {command_out.state.value}")
                rows.append({
                    "time_s": f"{now:.1f}", "vehicle_id": follower, "preceding_vehicle": preceding,
                    "sensed_vehicle": sensed_id, "gap_m": f"{gap:.2f}",
                    "speed_mps": f"{follower_speed:.2f}", "v2v_latency_ms": "blackout" if latency is None else latency,
                    "resilience_state": command_out.state.value, "controller_reason": command_out.reason,
                })
    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    finally:
        traci.close(False)

    with output.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]) if rows else [])
        writer.writeheader()
        writer.writerows(rows)
    print(f"Telemetry: {output}")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the SAMTRUCK platoon in the Montreal city scenario.")
    parser.add_argument("--scenario", choices=SCENARIOS, default="baseline")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--duration", type=float, default=0, help="Seconds to run; 0 means loop until Ctrl+C.")
    args = parser.parse_args()
    run(args.scenario, args.gui, args.duration)


if __name__ == "__main__":
    main()
