"""
SAMTRUCK Standalone TraCI Evaluation Scenario Runner
Runs scenario catalog, logs real TraCI telemetry/messages/events, computes metrics, generates plots, and creates HTML reports.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from samtruck.config import SUMO_HOME, SimulationConfig
from samtruck.ambient_traffic import SeededAmbientTraffic
from samtruck.controller import CACCController
from samtruck.evaluator_console import EvaluatorConsole
from samtruck.executive import ActiveExecutive
from samtruck.twin_advisor import AdvisorDecisionConsole, AdvisorInput, TwinAdvisor
from samtruck.sensors import SensorInterface, VehicleSensorInput
from samtruck.twin_runtime import PlatoonTwin
from samtruck.twin_degradation import TwinDegradationSchedule
from samtruck.v2x import PendingV2XChannel, V2XDeliveryEvent, V2XMessage
from samtruck.montreal_demo import CITY_DIR, TRUCK_IDS, add_platoon, select_truck_loop
from samtruck.offline_estimator_evaluator import evaluate_csv_files
from samtruck.offline_uncertainty_evaluator import evaluate_uncertainty_csv_files
from samtruck.run import prepare_traci
from samtruck.scenario_suite import CATALOG_DIR, load_specs, schedule_master

from evaluation.metrics import calculate_metrics
from evaluation.plotter import generate_all_plots
from evaluation.report import generate_report


RESULTS_DIR = PROJECT_ROOT / "results"

FILTERED_STATE_FIELDS = [
    "simulation_time_s", "vehicle_id", "estimated_position_m", "estimated_speed_mps",
    "estimated_acceleration_mps2", "position_variance_m2", "speed_variance_m2ps2",
    "acceleration_variance_m2ps4", "estimator_status", "measurement_update_mask",
    "independent_position_measurement_used",
    "last_independent_position_measurement_timestamp_s", "independent_position_measurement_age_s",
    "independent_position_measurement_ever_received",
    "last_speed_measurement_timestamp_s", "speed_measurement_age_s", "speed_measurement_ever_received",
    "last_acceleration_measurement_timestamp_s", "acceleration_measurement_age_s",
    "acceleration_measurement_ever_received",
    "position_estimate_provenance", "speed_estimate_provenance", "acceleration_estimate_provenance",
    "predecessor_v2x_generation_time_s",
    "predecessor_v2x_age_s", "predecessor_v2x_status",
]
RADAR_TRACK_FIELDS = [
    "simulation_time_s", "ego_vehicle_id", "sensed_target_id", "estimated_range_m",
    "estimated_relative_speed_mps", "range_variance_m2", "relative_speed_variance_m2ps2",
    "track_age_s", "last_measurement_timestamp_s", "observed_components",
    "unobserved_prior_components", "track_status", "last_update_was_measurement",
]


def build_twin_inputs_and_truth(
    traci, timestamp_s: float, active_vehs: set[str], sensor_interface: SensorInterface
) -> tuple[list[VehicleSensorInput], list[dict[str, object]]]:
    """Narrow simulator boundary: TraCI values become DTOs or evaluation-only rows.

    The DTOs are the only live data passed to the Twin.  The ground-truth rows
    are returned separately and never enter the Twin or controller.
    """

    sensor_inputs: list[VehicleSensorInput] = []
    truth_rows: list[dict[str, object]] = []
    for index, vehicle_id in enumerate(TRUCK_IDS):
        if vehicle_id not in active_vehs:
            continue
        position = traci.vehicle.getLanePosition(vehicle_id)
        speed = traci.vehicle.getSpeed(vehicle_id)
        acceleration = traci.vehicle.getAcceleration(vehicle_id)
        lane_id = traci.vehicle.getLaneID(vehicle_id)
        sensed = traci.vehicle.getLeader(vehicle_id, 150.0)
        sensed_leader_id = sensed[0] if sensed else None
        gap = sensed[1] if sensed else None
        relative_speed = None if sensed_leader_id is None else speed - traci.vehicle.getSpeed(sensed_leader_id)
        predecessor_id = TRUCK_IDS[index - 1] if index else None
        sensor_inputs.append(sensor_interface.make_input(
            timestamp_s=timestamp_s, vehicle_id=vehicle_id, platoon_predecessor_id=predecessor_id,
            position_m=position, speed_mps=speed, acceleration_mps2=acceleration,
            sensed_leader_id=sensed_leader_id, radar_range_m=gap,
            radar_relative_speed_mps=relative_speed,
        ))
        truth_rows.append({
            "simulation_time_s": timestamp_s, "vehicle_id": vehicle_id,
            "lane_id": lane_id, "lane_position_m": position, "speed_mps": speed,
            "acceleration_mps2": acceleration, "sensed_leader_id": sensed_leader_id or "",
            "actual_gap_m": "" if gap is None else gap,
        })
    return sensor_inputs, truth_rows


def _twin_message_row(event: V2XDeliveryEvent) -> dict[str, object]:
    """Serialize Twin-channel outcomes without exposing simulator state objects."""

    message = event.message
    return {
        "sender": message.sender_id,
        "receiver": message.receiver_id or "",
        "sequence": "" if message.sequence is None else message.sequence,
        "generation_time_s": message.sim_time,
        "scheduled_delivery_time_s": "" if message.scheduled_delivery_time_s is None else message.scheduled_delivery_time_s,
        "delivery_time_s": "" if event.delivery_time_s is None else event.delivery_time_s,
        "status": event.status,
        "reason": event.reason,
    }


def _phase_2_2d_seed_policy(spec: dict, phase_config: SimulationConfig) -> dict[str, int] | None:
    """Resolve explicit Phase 2.2D seeds without changing legacy scenarios.

    The opt-in block is deliberately separate from the official scenario
    catalog and Phase 2.2C configuration.  When absent, the SUMO command and
    all existing seed fallback behavior remain byte-for-byte equivalent.
    """

    phase_2_2d = spec.get("phase_2_2d")
    if phase_2_2d is None:
        return None
    if not isinstance(phase_2_2d, dict) or "sumo_seed" not in phase_2_2d:
        raise ValueError("Phase 2.2D runs require phase_2_2d.sumo_seed.")

    def as_seed(value: object, name: str) -> int:
        if isinstance(value, bool):
            raise ValueError(f"{name} must be a non-negative integer.")
        try:
            seed = int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a non-negative integer.") from exc
        if seed < 0:
            raise ValueError(f"{name} must be a non-negative integer.")
        return seed

    scenario_seed = as_seed(spec.get("seed", 20260929), "scenario seed")
    twin_config = spec.get("twin", {})
    if not isinstance(twin_config, dict):
        raise ValueError("Scenario twin configuration must be an object.")
    return {
        "sumo_seed": as_seed(phase_2_2d["sumo_seed"], "Phase 2.2D SUMO seed"),
        "scenario_seed": scenario_seed,
        "sensor_seed": as_seed(twin_config.get("sensor_seed", scenario_seed), "sensor seed"),
        "twin_v2x_seed": as_seed(twin_config.get("v2x_seed", scenario_seed), "Twin V2X seed"),
        "legacy_cacc_v2x_seed": scenario_seed,
    }


def _sensor_measurement_rows(sensor_inputs: list[VehicleSensorInput]) -> list[dict[str, object]]:
    """Serialize the DTOs delivered to the Twin for Phase 2.2D auditing only.

    This is measurement-only observability.  It does not expose evaluation
    rows to the Twin, alter the SensorInterface, or persist retroactive data
    for historical results.
    """

    rows: list[dict[str, object]] = []
    for sensor_input in sensor_inputs:
        odometry, radar = sensor_input.odometry, sensor_input.radar
        rows.append({
            "simulation_time_s": sensor_input.timestamp_s,
            "vehicle_id": sensor_input.vehicle_id,
            "platoon_predecessor_id": sensor_input.platoon_predecessor_id or "",
            "odometry_position_m": odometry.position_m,
            "odometry_speed_mps": odometry.speed_mps,
            "odometry_acceleration_mps2": odometry.acceleration_mps2,
            "odometry_valid": odometry.valid,
            "odometry_status": odometry.status,
            "radar_sensed_leader_id": "" if radar is None or radar.sensed_leader_id is None else radar.sensed_leader_id,
            "radar_range_m": "" if radar is None or radar.range_m is None else radar.range_m,
            "radar_relative_speed_mps": "" if radar is None or radar.relative_speed_mps is None else radar.relative_speed_mps,
            "radar_valid": "" if radar is None else radar.valid,
            "radar_status": "no_target" if radar is None else radar.status,
        })
    return rows


def _evaluator_console_snapshots(
    truth_rows: list[dict[str, object]], sensor_inputs: list[VehicleSensorInput], filtered_rows: list[dict[str, object]],
) -> list[dict[str, object]]:
    """Assemble display-only records after the Twin update.

    The rows are joined locally for presentation.  Truth originates from the
    independent evaluation branch and is never supplied to ``twin.update``.
    Sensor values are the exact post-degradation DTOs that reached the Twin.
    """

    truth_by_vehicle = {str(row["vehicle_id"]): row for row in truth_rows}
    filtered_by_vehicle = {str(row["vehicle_id"]): row for row in filtered_rows}
    snapshots: list[dict[str, object]] = []
    for sensor_input in sensor_inputs:
        vehicle_id = sensor_input.vehicle_id
        truth = truth_by_vehicle.get(vehicle_id, {})
        filtered = filtered_by_vehicle.get(vehicle_id, {})
        odometry, radar = sensor_input.odometry, sensor_input.radar
        snapshots.append({
            "simulation_time_s": sensor_input.timestamp_s,
            "vehicle_id": vehicle_id,
            "truth_speed_mps": truth.get("speed_mps"),
            "truth_acceleration_mps2": truth.get("acceleration_mps2"),
            "truth_gap_m": truth.get("actual_gap_m"),
            "sensor_odometry_speed_mps": odometry.speed_mps,
            "sensor_odometry_valid": odometry.valid,
            "sensor_odometry_status": odometry.status,
            "sensor_radar_range_m": None if radar is None else radar.range_m,
            "sensor_radar_valid": None if radar is None else radar.valid,
            "sensor_radar_status": "no_target" if radar is None else radar.status,
            "twin_filtered_speed_mps": filtered.get("estimated_speed_mps"),
            "twin_filtered_acceleration_mps2": filtered.get("estimated_acceleration_mps2"),
            "twin_speed_variance": filtered.get("speed_variance_m2ps2"),
            "twin_accel_variance": filtered.get("acceleration_variance_m2ps4"),
            "speed_measurement_age_s": filtered.get("speed_measurement_age_s"),
            "speed_estimate_provenance": filtered.get("speed_estimate_provenance"),
            "predecessor_v2x_status": filtered.get("predecessor_v2x_status"),
            "predecessor_v2x_age_s": filtered.get("predecessor_v2x_age_s"),
        })
    return snapshots


def _advisor_inputs_from_twin_snapshots(
    snapshots: list[dict[str, object]], sensor_inputs: list[VehicleSensorInput],
) -> list[AdvisorInput]:
    """Map post-update Twin-side observations into passive advisor DTOs.

    This deliberately selects only sensor DTO, filtered-state, freshness, and
    delivered-V2X fields.  ``truth_*`` display fields in the combined console
    snapshots are never read here, preserving the evaluation/Twin boundary.
    """

    predecessor_by_vehicle = {
        sensor_input.vehicle_id: sensor_input.platoon_predecessor_id
        for sensor_input in sensor_inputs
    }
    return [
        AdvisorInput(
            simulation_time_s=float(snapshot["simulation_time_s"]),
            vehicle_id=str(snapshot["vehicle_id"]),
            twin_filtered_speed_mps=snapshot.get("twin_filtered_speed_mps"),
            twin_filtered_acceleration_mps2=snapshot.get("twin_filtered_acceleration_mps2"),
            twin_speed_variance=snapshot.get("twin_speed_variance"),
            twin_accel_variance=snapshot.get("twin_accel_variance"),
            speed_measurement_age_s=snapshot.get("speed_measurement_age_s"),
            speed_estimate_provenance=snapshot.get("speed_estimate_provenance"),
            sensor_odometry_valid=snapshot.get("sensor_odometry_valid"),
            sensor_odometry_status=snapshot.get("sensor_odometry_status"),
            sensor_radar_valid=snapshot.get("sensor_radar_valid"),
            sensor_radar_status=snapshot.get("sensor_radar_status"),
            sensor_radar_range_m=snapshot.get("sensor_radar_range_m"),
            predecessor_v2x_status=snapshot.get("predecessor_v2x_status"),
            predecessor_v2x_age_s=snapshot.get("predecessor_v2x_age_s"),
            has_platoon_predecessor=predecessor_by_vehicle.get(str(snapshot["vehicle_id"])) is not None,
        )
        for snapshot in snapshots
    ]


def apply_fast_demo(spec: dict, duration_override: float | None) -> tuple[dict, float | None]:
    """
    In-memory fast demo transformation:
    - Deep copy spec so original JSON files remain unchanged.
    - Shift first scenario event to 5.0s while preserving time differences between later events.
    - Bounded scenario duration: last event end time + 15 seconds recovery (minimum 25 seconds).
    """
    spec_copy = json.loads(json.dumps(spec))
    events = spec_copy.get("events", [])
    if events:
        first_at_s = min(float(e.get("at_s", 0.0)) for e in events)
        delta_s = 5.0 - first_at_s
        for e in events:
            e["at_s"] = round(float(e.get("at_s", 0.0)) + delta_s, 1)
        last_event_end = max(float(e["at_s"]) + float(e.get("duration_s", 0.1)) for e in events)
        fast_duration = max(25.0, last_event_end + 15.0)
    else:
        fast_duration = 25.0

    if duration_override is None:
        duration_override = fast_duration

    return spec_copy, duration_override


def add_city_vehicle_safe(traci, route_edges: list[str], vehicle_id: str, position: float, speed: float, now: float) -> bool:
    try:
        if vehicle_id in traci.vehicle.getIDList() or vehicle_id in traci.vehicle.getLoadedIDList():
            return True
        route_id = f"{vehicle_id}_route_{int(now * 10)}"
        traci.route.add(route_id, route_edges)
        traci.vehicle.add(vehicle_id, route_id, typeID="passenger", depart="now", departLane="best",
                          departPos=str(max(1.0, position)), departSpeed="0")
        traci.vehicle.setSpeedMode(vehicle_id, 7)
        traci.vehicle.setSpeed(vehicle_id, max(0.0, speed))
        return True
    except Exception:
        return False


def load_ambient_edge_pairs() -> list[tuple[str, str]]:
    """Load valid city origin/destination candidates without touching routes."""
    route_file = CITY_DIR / "city_traffic.rou.xml"
    root = ET.parse(route_file).getroot()
    return [
        (trip.attrib["from"], trip.attrib["to"])
        for trip in root.findall("trip")
        if trip.attrib.get("from") and trip.attrib.get("to")
    ]


def add_ambient_vehicle_safe(traci, request, now: float) -> bool:
    """Resolve and insert one SUMO-controlled ambient car, or safely skip it."""
    try:
        route_result = traci.simulation.findRoute(request.origin_edge, request.destination_edge)
        edges = list(route_result.edges)
        if not edges:
            return False
        route_id = f"{request.vehicle_id}_route"
        traci.route.add(route_id, edges)
        traci.vehicle.add(request.vehicle_id, route_id, typeID="passenger", depart="now", departLane="best", departSpeed="max")
        return True
    except Exception:
        return False


def inject_relative_vehicle_safe(traci, event_id: str, leader_id: str, distance_ahead: float, speed: float, now: float) -> str | None:
    vehicle_id = f"event_{event_id}"
    if vehicle_id in traci.vehicle.getIDList() or vehicle_id in traci.vehicle.getLoadedIDList():
        return vehicle_id

    if leader_id not in traci.vehicle.getIDList():
        return None
    try:
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
        return vehicle_id if add_city_vehicle_safe(traci, remaining, vehicle_id, position, speed, now) else None
    except Exception:
        return None


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



def event_speed(now: float, events: list[dict]) -> float:
    speed = 10.0
    for event in events:
        at_s = float(event.get("at_s", 0.0))
        dur_s = float(event.get("duration_s", 0.1))
        if at_s <= now < at_s + dur_s:
            kind = event["type"]
            if kind == "leader_speed":
                speed = float(event["speed_mps"])
            elif kind == "traffic_stop":
                speed = 0.0
            elif kind == "emergency_brake":
                elapsed = now - at_s
                speed = max(0.0, 10.0 - 5.0 * elapsed)
    return speed


def v2x_state(now: float, events: list[dict], rng: random.Random) -> tuple[float | None, bool, float, float]:
    """Returns (latency_ms_if_not_outage, is_packet_lost, base_delay_ms, loss_rate)."""
    delay = 40.0
    loss_rate = 0.0
    outage = False
    for event in events:
        at_s = float(event.get("at_s", 0.0))
        dur_s = float(event.get("duration_s", 0.1))
        if at_s <= now < at_s + dur_s:
            kind = event["type"]
            if kind == "v2x_delay":
                delay = float(event["latency_ms"])
            elif kind == "packet_loss":
                loss_rate = max(loss_rate, float(event["loss_rate"]))
            elif kind == "v2x_outage":
                outage = True
    is_lost = rng.random() < loss_rate
    return (None if outage else delay), is_lost, delay, loss_rate


def twin_v2x_state(
    now: float,
    schedule: TwinDegradationSchedule,
    legacy_latency_ms: float | None,
    legacy_loss_rate: float,
) -> tuple[float | None, float]:
    """Resolve Twin V2X conditions without changing the legacy CACC path.

    Existing catalog scenarios omit ``twin_only_degradations`` and retain the
    established legacy event-derived behavior.  Phase 2.2C experiments opt in
    to a distinct Twin-only schedule.
    """

    if schedule.has_v2x_events:
        return schedule.v2x_state(now)
    return legacy_latency_ms, legacy_loss_rate


def run_single_scenario(
    spec: dict,
    gui: bool = False,
    duration_override: float | None = None,
    *,
    evaluator_console: bool = False,
    evaluator_console_period_s: float = 1.0,
    twin_advisor: bool = False,
    twin_advisor_period_s: float = 1.0,
    twin_executive: bool = False,
    twin_executive_period_s: float = 1.0,
    ambient_random_traffic: bool = False,
    ambient_traffic_rate_vpm: float = 18.0,
    ambient_traffic_seed: int | None = None,
    event_console: bool = False,
    output_id: str | None = None,
    refuse_existing_results: bool = False,
) -> tuple[bool, dict]:
    prepare_traci()
    import traci  # type: ignore

    scenario_id = spec["id"]
    output_dir = RESULTS_DIR / (output_id or scenario_id)
    if refuse_existing_results and output_dir.exists():
        raise FileExistsError(f"Refusing to overwrite existing results directory: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    telemetry_file = output_dir / "telemetry.csv"
    messages_file = output_dir / "messages.csv"
    events_file = output_dir / "events.json"
    truth_file = output_dir / "twin_ground_truth.csv"
    twin_file = output_dir / "twin_estimates.csv"
    filtered_file = output_dir / "twin_filtered_states.csv"
    radar_tracks_file = output_dir / "twin_radar_tracks.csv"
    estimator_evaluation_file = output_dir / "twin_filtered_evaluation.json"
    uncertainty_evaluation_file = output_dir / "twin_uncertainty_evaluation.json"
    twin_messages_file = output_dir / "twin_v2x_messages.csv"
    degradation_summary_file = output_dir / "twin_degradation_summary.json"
    sensor_measurements_file = output_dir / "twin_sensor_measurements.csv"
    vehicle_lifecycle_file = output_dir / "sumo_vehicle_lifecycle.csv"
    ambient_traffic_file = output_dir / "ambient_traffic.csv"
    evaluator_console_file = output_dir / "evaluator_console.jsonl"
    live_console = EvaluatorConsole(evaluator_console_file, period_s=evaluator_console_period_s) if evaluator_console else None
    advisor_file = output_dir / "advisor_decisions.jsonl"
    advisor_engine = TwinAdvisor() if twin_advisor else None
    advisor_console = AdvisorDecisionConsole(advisor_file, period_s=twin_advisor_period_s) if twin_advisor else None
    if twin_executive and not twin_advisor:
        raise ValueError("The active executive requires TwinAdvisor decisions.")
    executive_file = output_dir / "executive_actions.jsonl"

    duration = spec["duration_s"] if duration_override is None else duration_override
    rng = random.Random(spec.get("seed", 20260929))
    phase_config = SimulationConfig()
    phase_2_2d_seed_policy = _phase_2_2d_seed_policy(spec, phase_config)

    binary = SUMO_HOME / "bin" / ("sumo-gui.exe" if gui else "sumo.exe")
    command = [str(binary), "-c", str(CITY_DIR / "montreal.sumocfg"), "--start", "--delay", "50"]
    # The city config has a 90 s end time.  A bounded long run overrides it.
    # Fully unbounded runs are deliberately excluded because this runner keeps
    # complete telemetry rows in memory for its post-run evaluation pipeline.
    if duration > 90.0:
        command.extend(["--end", str(int(math.ceil(duration + 30.0)))])
    if phase_2_2d_seed_policy is not None:
        command.extend(["--seed", str(phase_2_2d_seed_policy["sumo_seed"])])
    if not gui:
        command.append("--quit-on-end")

    controller = CACCController(desired_time_gap=0.8, min_gap=10.0, max_speed=18.0)
    follower_controllers = {
        truck_id: CACCController(desired_time_gap=0.8, min_gap=10.0, max_speed=18.0)
        for truck_id in TRUCK_IDS[1:]
    } if twin_executive else {}
    executive = ActiveExecutive(TRUCK_IDS[1:]) if twin_executive else None
    next_executive_print_s: float | None = None
    twin_config = spec.get("twin", {})
    sensor_interface = SensorInterface(
        radar_noise_std_m=float(twin_config.get("radar_noise_std_m", phase_config.radar_noise_std_m)),
        odometry_speed_noise_std_mps=float(twin_config.get("odometry_speed_noise_std_mps", phase_config.odometry_speed_noise_std_mps)),
        radar_dropout_probability=float(twin_config.get("radar_dropout_probability", phase_config.radar_dropout_probability)),
        seed=int(twin_config.get("sensor_seed", spec.get("seed", phase_config.sensor_seed))),
    )
    twin = PlatoonTwin(
        TRUCK_IDS,
        history_len=int(twin_config.get("history_len", phase_config.twin_history_len)),
        v2x_stale_after_s=float(twin_config.get("v2x_stale_after_s", phase_config.twin_v2x_stale_after_s)),
    )
    twin_channel = PendingV2XChannel(
        TRUCK_IDS,
        {TRUCK_IDS[0]: TRUCK_IDS[1], TRUCK_IDS[1]: TRUCK_IDS[2]},
        stale_after_s=float(twin_config.get("v2x_stale_after_s", phase_config.twin_v2x_stale_after_s)),
        seed=int(twin_config.get("v2x_seed", spec.get("seed", phase_config.sensor_seed))),
    )
    twin_degradation = TwinDegradationSchedule(spec.get("twin_only_degradations", ()))

    # Data structures
    telemetry_rows: list[dict] = []
    message_rows: list[dict] = []
    events_log: list[dict] = []
    truth_rows: list[dict] = []
    twin_rows: list[dict] = []
    filtered_rows: list[dict] = []
    radar_track_rows: list[dict] = []
    twin_message_rows: list[dict] = []
    sensor_measurement_rows: list[dict] = []
    vehicle_lifecycle_rows: list[dict] = []
    executive_action_rows: list[dict] = []
    ambient_traffic_rows: list[dict[str, object]] = []
    ambient_generator = (
        SeededAmbientTraffic(
            load_ambient_edge_pairs(),
            seed=int(spec.get("seed", 20260929) if ambient_traffic_seed is None else ambient_traffic_seed),
            rate_vehicles_per_minute=ambient_traffic_rate_vpm,
        )
        if ambient_random_traffic else None
    )

    injected_vehicles: dict[str, str] = {}
    event_statuses: dict[int, dict] = {}  # index -> event_log_record
    lane_change_pending: dict[str, dict] = {}

    prev_positions: dict[str, tuple[float, float]] = {}
    cum_distances: dict[str, float] = {v: 0.0 for v in TRUCK_IDS}
    last_v2v_received: dict[str, dict] = {}
    msg_counter = 0
    twin_sequences = {vehicle_id: 0 for vehicle_id in TRUCK_IDS}

    collisions_count = 0

    traci.start(command)
    try:
        route_id, loop_edges = select_truck_loop(traci)
        add_platoon(traci, route_id, loop_edges[0])

        step_count = 0
        while (duration <= 0 or traci.simulation.getTime() < duration) and traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            now = traci.simulation.getTime()
            active_vehs = set(traci.vehicle.getIDList())
            if phase_2_2d_seed_policy is not None:
                vehicle_lifecycle_rows.append({
                    "simulation_time_s": now,
                    "active_vehicle_ids": "|".join(sorted(active_vehs)),
                    "departed_vehicle_ids": "|".join(sorted(traci.simulation.getDepartedIDList())),
                    "arrived_vehicle_ids": "|".join(sorted(traci.simulation.getArrivedIDList())),
                })
            collisions_count += len(traci.simulation.getCollidingVehiclesIDList())

            if ambient_generator is not None:
                for request in ambient_generator.due_requests(now):
                    inserted = add_ambient_vehicle_safe(traci, request, now)
                    ambient_traffic_rows.append({
                        "simulation_time_s": now,
                        "vehicle_id": request.vehicle_id,
                        "origin_edge": request.origin_edge,
                        "destination_edge": request.destination_edge,
                        "scheduled_time_s": request.scheduled_time_s,
                        "inserted": inserted,
                    })
                    if event_console:
                        state = "INSERTED" if inserted else "SKIPPED"
                        print(f"t = {now:.1f}s | AMBIENT {state} {request.vehicle_id}")

            # Handle Event Triggers & Continuous Retry during Active Window
            for index, event in enumerate(spec.get("events", [])):
                at_s = float(event.get("at_s", 0.0))
                dur_s = float(event.get("duration_s", 0.1))
                kind = event["type"]

                if index not in event_statuses and now >= at_s:
                    # Initialize event log record
                    event_rec = {
                        "time_s": round(now, 1),
                        "event": kind,
                        "duration_s": dur_s,
                        "applied": False,
                        "detail": "Event initialized"
                    }
                    event_statuses[index] = event_rec
                    events_log.append(event_rec)
                    if event_console:
                        print(f"t = {now:.1f}s | EVENT DETECTED type={kind} scheduled={at_s:.1f}s duration={dur_s:.1f}s")

                # Retry pending unapplied events during active window
                event_rec = event_statuses.get(index)
                if event_rec and not event_rec["applied"] and at_s <= now <= at_s + max(20.0, dur_s):
                    if kind in {"cut_in", "slow_vehicle", "roadwork", "onramp_merge"}:
                        spd = 4.0 if kind in {"slow_vehicle", "roadwork"} else 8.0
                        injected_id = inject_relative_vehicle_safe(traci, f"{kind}_{index}", TRUCK_IDS[0], 32.0, spd, now)
                        if injected_id and injected_id in traci.vehicle.getIDList():
                            injected_vehicles[kind] = injected_id
                            event_rec["applied"] = True
                            event_rec["time_s"] = round(now, 1)
                            event_rec["detail"] = f"Injected vehicle {injected_id}"

                    elif kind == "platoon_merge":
                        injected_id = inject_relative_vehicle_safe(traci, f"merge_{index}", TRUCK_IDS[-1], -5.0, 8.0, now)
                        if injected_id and injected_id in traci.vehicle.getIDList():
                            injected_vehicles[kind] = injected_id
                            event_rec["applied"] = True
                            event_rec["time_s"] = round(now, 1)
                            event_rec["detail"] = f"Injected merge vehicle {injected_id}"

                    elif kind == "traffic_burst":
                        created_count = 0
                        for n in range(event.get("count", 7)):
                            inj_id = inject_relative_vehicle_safe(traci, f"traffic_{index}_{n}", TRUCK_IDS[0], 45 + n * 12, 5.0, now)
                            if inj_id:
                                created_count += 1
                        if created_count > 0:
                            event_rec["applied"] = True
                            event_rec["time_s"] = round(now, 1)
                            event_rec["detail"] = f"Created {created_count} burst vehicles"

                    elif kind == "lane_change" and TRUCK_IDS[0] in active_vehs:
                        road = traci.vehicle.getRoadID(TRUCK_IDS[0])
                        if not road.startswith(":"):
                            lanes = traci.edge.getLaneNumber(road)
                            if lanes > 1:
                                target_lane = 1
                                initial_lane = traci.vehicle.getLaneIndex(TRUCK_IDS[0])
                                traci.vehicle.changeLane(TRUCK_IDS[0], target_lane, 4.0)
                                lane_change_pending[kind] = {
                                    "event_rec": event_rec,
                                    "target_lane": target_lane,
                                    "initial_lane": initial_lane,
                                    "timeout_s": now + 6.0
                                }
                            else:
                                event_rec["detail"] = "Single lane edge; waiting for multi-lane edge"

                    elif kind == "cut_out":
                        inj_id = injected_vehicles.get("cut_in")
                        if inj_id and inj_id in active_vehs:
                            traci.vehicle.remove(inj_id)
                            event_rec["applied"] = True
                            event_rec["time_s"] = round(now, 1)
                            event_rec["detail"] = f"Removed cut-in vehicle {inj_id}"

                    elif kind in {"leader_speed", "traffic_stop", "emergency_brake", "v2x_delay", "packet_loss", "v2x_outage", "platoon_split"}:
                        event_rec["applied"] = True
                        event_rec["detail"] = f"{kind} event activated in scenario execution loop"

                if event_console and event_rec and event_rec["applied"] and not event_rec.get("console_applied_reported", False):
                    print(f"t = {now:.1f}s | EVENT APPLIED type={kind} detail={event_rec['detail']}")
                    event_rec["console_applied_reported"] = True

            # Final status update for expired events
            for index, event in enumerate(spec.get("events", [])):
                at_s = float(event.get("at_s", 0.0))
                dur_s = float(event.get("duration_s", 0.1))
                event_rec = event_statuses.get(index)
                if event_rec and not event_rec["applied"] and now > at_s + max(20.0, dur_s):
                    event_rec["detail"] = f"Event window expired without map/route opportunity to apply"

            # Verify pending lane changes
            for lc_kind, lc_data in list(lane_change_pending.items()):
                if TRUCK_IDS[0] in active_vehs:
                    current_lane = traci.vehicle.getLaneIndex(TRUCK_IDS[0])
                    if current_lane == lc_data["target_lane"] or current_lane != lc_data["initial_lane"]:
                        lc_data["event_rec"]["applied"] = True
                        lc_data["event_rec"]["detail"] = f"Lane changed to lane index {current_lane}"
                        del lane_change_pending[lc_kind]
                    elif now >= lc_data["timeout_s"]:
                        lc_data["event_rec"]["applied"] = False
                        lc_data["event_rec"]["detail"] = "Lane change command timed out without lane shift"
                        del lane_change_pending[lc_kind]

            if TRUCK_IDS[0] not in active_vehs:
                continue

            # Maintain vehicle modes & loops
            for vehicle_id in TRUCK_IDS:
                if vehicle_id in active_vehs:
                    traci.vehicle.setSpeedMode(vehicle_id, 7)
                    traci.vehicle.setLaneChangeMode(vehicle_id, 1621)
                    ensure_loop(traci, vehicle_id, loop_edges)

            # Apply Leader Speed
            target_sp = event_speed(now, spec.get("events", []))
            coord_sp = coordinated_leader_speed(traci, target_sp)
            traci.vehicle.setSpeed(TRUCK_IDS[0], coord_sp)

            # V2X Channel State
            current_latency_ms, is_packet_lost, base_delay, loss_rate = v2x_state(now, spec.get("events", []), rng)
            sensor_inputs, current_truth_rows = build_twin_inputs_and_truth(traci, now, active_vehs, sensor_interface)
            twin_sensor_inputs = twin_degradation.apply_sensor_inputs(now, sensor_inputs)
            if phase_2_2d_seed_policy is not None:
                sensor_measurement_rows.extend(_sensor_measurement_rows(twin_sensor_inputs))
            twin_latency_ms, twin_loss_rate = twin_v2x_state(
                now, twin_degradation, current_latency_ms, loss_rate
            )
            truth_rows.extend(current_truth_rows)

            # Process Follower Trucks & Telemetry
            for idx, truck_id in enumerate(TRUCK_IDS):
                if truck_id not in active_vehs:
                    continue

                pos_x, pos_y = traci.vehicle.getPosition(truck_id)
                lane_id = traci.vehicle.getLaneID(truck_id)
                lane_pos = traci.vehicle.getLanePosition(truck_id)
                speed = traci.vehicle.getSpeed(truck_id)
                accel = traci.vehicle.getAcceleration(truck_id)

                # Accumulated Distance Calculation
                if truck_id in prev_positions:
                    px, py = prev_positions[truck_id]
                    dist_step = math.hypot(pos_x - px, pos_y - py)
                    cum_distances[truck_id] += dist_step
                prev_positions[truck_id] = (pos_x, pos_y)
                long_pos = cum_distances[truck_id]

                predecessor_id = TRUCK_IDS[idx - 1] if idx > 0 else ""
                follower_id = TRUCK_IDS[idx + 1] if idx < len(TRUCK_IDS) - 1 else ""

                if idx == 0:
                    # Leader Vehicle
                    telemetry_rows.append({
                        "simulation_time_s": f"{now:.1f}",
                        "vehicle_id": truck_id,
                        "x_m": f"{pos_x:.2f}",
                        "y_m": f"{pos_y:.2f}",
                        "lane_id": lane_id,
                        "lane_position_m": f"{lane_pos:.2f}",
                        "longitudinal_position_m": f"{long_pos:.2f}",
                        "speed_mps": f"{speed:.2f}",
                        "acceleration_mps2": f"{accel:.2f}",
                        "leader_id": "",
                        "predecessor_id": "",
                        "follower_id": follower_id,
                        "actual_gap_m": "",
                        "desired_gap_m": "",
                        "gap_error_m": "",
                        "resilience_state": "LEADER",
                        "controller_reason": "scripted_leader",
                        "v2v_message_id": "",
                        "v2v_age_ms": ""
                    })
                else:
                    # Follower Vehicle
                    sensed = traci.vehicle.getLeader(truck_id, 150.0)
                    if sensed is None:
                        traci.vehicle.setSpeed(truck_id, max(0.0, traci.vehicle.getSpeed(predecessor_id) - 1.0))
                        continue

                    sensed_id, gap = sensed
                    sensed_speed = traci.vehicle.getSpeed(sensed_id)

                    # Simulate V2X Message Transmission Attempt
                    msg_counter += 1
                    msg_id = f"msg_{predecessor_id}_{truck_id}_{msg_counter}"

                    if current_latency_ms is None:
                        # Outage
                        msg_status = "dropped"
                        delivery_time = ""
                        lat_val = ""
                    elif is_packet_lost:
                        # Packet loss
                        msg_status = "dropped"
                        delivery_time = ""
                        lat_val = ""
                    else:
                        # Successful reception
                        msg_status = "received"
                        lat_val = current_latency_ms
                        delivery_time = f"{(now + current_latency_ms / 1000.0):.3f}"
                        last_v2v_received[truck_id] = {
                            "time_s": now,
                            "speed": traci.vehicle.getSpeed(predecessor_id),
                            "accel": traci.vehicle.getAcceleration(predecessor_id),
                            "msg_id": msg_id
                        }

                    message_rows.append({
                        "message_id": msg_id,
                        "sender": predecessor_id,
                        "receiver": truck_id,
                        "generation_time_s": f"{now:.3f}",
                        "delivery_time_s": delivery_time,
                        "latency_ms": f"{lat_val}" if lat_val != "" else "",
                        "status": msg_status
                    })

                    # Twin communication branch: capture payload at generation
                    # time and queue it. It is independent from the preserved
                    # inline CACC V2X path above.
                    twin_sequences[predecessor_id] += 1
                    payload_accel = traci.vehicle.getAcceleration(predecessor_id)
                    payload = V2XMessage(
                        sender_id=predecessor_id,
                        sim_time=now,
                        speed=traci.vehicle.getSpeed(predecessor_id),
                        acceleration=payload_accel,
                        braking=payload_accel < -0.5,
                        receiver_id=truck_id,
                        sequence=twin_sequences[predecessor_id],
                    )
                    twin_degradation.record_v2x_attempt(twin_latency_ms)
                    for event in twin_channel.enqueue(payload, twin_latency_ms, twin_loss_rate):
                        twin_message_rows.append(_twin_message_row(event))

                    # Evaluate V2V Message Age for Controller
                    v2v_info = last_v2v_received.get(truck_id)
                    if v2v_info is not None and current_latency_ms is not None:
                        age_ms = (now - v2v_info["time_s"]) * 1000.0 + current_latency_ms
                        usable_v2v = (age_ms <= 1000.0)
                    else:
                        age_ms = None
                        usable_v2v = False

                    active_controller = follower_controllers.get(truck_id, controller)
                    if executive is not None:
                        executive.apply_to_controller(truck_id, active_controller)
                    command_out = active_controller.command(
                        follower_speed=speed,
                        sensed_leader_speed=sensed_speed,
                        sensed_gap=gap,
                        v2v_leader_speed=v2v_info["speed"] if usable_v2v else None,
                        v2v_leader_accel=v2v_info["accel"] if usable_v2v else None,
                        link_latency_ms=age_ms if usable_v2v else None
                    )

                    # Platoon Split modification if active
                    active_split = any(
                        e["type"] == "platoon_split" and float(e.get("at_s", 0.0)) <= now < float(e.get("at_s", 0.0)) + float(e.get("duration_s", 0.1))
                        for e in spec.get("events", [])
                    )
                    if active_split and truck_id == TRUCK_IDS[-1]:
                        command_out = command_out.__class__(
                            max(0.0, command_out.target_speed - 3.0),
                            command_out.acceleration,
                            command_out.state,
                            "split_separation",
                            command_out.desired_gap
                        )

                    traci.vehicle.setSpeed(truck_id, command_out.target_speed)
                    gap_error = gap - command_out.desired_gap

                    telemetry_rows.append({
                        "simulation_time_s": f"{now:.1f}",
                        "vehicle_id": truck_id,
                        "x_m": f"{pos_x:.2f}",
                        "y_m": f"{pos_y:.2f}",
                        "lane_id": lane_id,
                        "lane_position_m": f"{lane_pos:.2f}",
                        "longitudinal_position_m": f"{long_pos:.2f}",
                        "speed_mps": f"{speed:.2f}",
                        "acceleration_mps2": f"{accel:.2f}",
                        "leader_id": sensed_id,
                        "predecessor_id": predecessor_id,
                        "follower_id": follower_id,
                        "actual_gap_m": f"{gap:.2f}",
                        "desired_gap_m": f"{command_out.desired_gap:.2f}",
                        "gap_error_m": f"{gap_error:.2f}",
                        "resilience_state": command_out.state.value,
                        "controller_reason": command_out.reason,
                        "v2v_message_id": v2v_info["msg_id"] if (v2v_info and usable_v2v) else "",
                        "v2v_age_ms": f"{age_ms:.0f}" if age_ms is not None else ""
                    })

            # Only now can timestamp-eligible pending messages reach the Twin.
            delivered_twin_messages, twin_events = twin_channel.deliver_ready(now)
            twin_message_rows.extend(_twin_message_row(event) for event in twin_events)
            # Passive Phase 1 Twin update. It cannot influence baseline CACC.
            twin.update(now, twin_sensor_inputs, delivered_twin_messages)
            twin_rows.extend(twin.as_rows(sensor_input.vehicle_id for sensor_input in twin_sensor_inputs))
            current_filtered_rows = twin.filtered_as_rows(sensor_input.vehicle_id for sensor_input in twin_sensor_inputs)
            filtered_rows.extend(current_filtered_rows)
            radar_track_rows.extend(twin.radar_as_rows(sensor_input.vehicle_id for sensor_input in twin_sensor_inputs))
            if live_console is not None or advisor_engine is not None:
                snapshots = _evaluator_console_snapshots(current_truth_rows, twin_sensor_inputs, current_filtered_rows)
                if live_console is not None:
                    live_console.record(snapshots)
                    live_console.maybe_print(now, snapshots)
                if advisor_engine is not None and advisor_console is not None:
                    decisions = advisor_engine.evaluate_many(
                        _advisor_inputs_from_twin_snapshots(snapshots, twin_sensor_inputs)
                    )
                    advisor_console.record(decisions)
                    advisor_console.maybe_print(now, decisions)
                    if executive is not None:
                        actions = executive.process_decisions(
                            decision for decision in decisions if decision.vehicle_id in follower_controllers
                        )
                        executive_action_rows.extend(asdict(action) for action in actions)
                        if next_executive_print_s is None:
                            next_executive_print_s = now
                        if now + 1e-9 >= next_executive_print_s:
                            while next_executive_print_s <= now + 1e-9:
                                next_executive_print_s += twin_executive_period_s
                            for action in actions:
                                print(
                                    f"t = {now:.1f}s | EXECUTIVE {action.vehicle_id} "
                                    f"command={action.command} ctg={action.applied_time_gap_s:.2f}s "
                                    f"mode={action.control_mode} v2v={action.v2v_enabled} "
                                    f"cap={action.max_speed_cap_mps}"
                                )

            step_count += 1

    except KeyboardInterrupt:
        print(f"Scenario {scenario_id} interrupted by user.")
    finally:
        traci.close(False)
        if live_console is not None:
            live_console.close()
        if advisor_console is not None:
            advisor_console.close()

    # Save Output CSVs and JSONs
    if telemetry_rows:
        with telemetry_file.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(telemetry_rows[0].keys()))
            writer.writeheader()
            writer.writerows(telemetry_rows)

    if message_rows:
        with messages_file.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(message_rows[0].keys()))
            writer.writeheader()
            writer.writerows(message_rows)
    else:
        # Empty messages.csv with headers
        with messages_file.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["message_id", "sender", "receiver", "generation_time_s", "delivery_time_s", "latency_ms", "status"])
            writer.writeheader()

    if truth_rows:
        with truth_file.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(truth_rows[0].keys()))
            writer.writeheader()
            writer.writerows(truth_rows)

    if twin_rows:
        with twin_file.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(twin_rows[0].keys()))
            writer.writeheader()
            writer.writerows(twin_rows)

    with filtered_file.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FILTERED_STATE_FIELDS)
        writer.writeheader()
        writer.writerows(filtered_rows)

    with radar_tracks_file.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=RADAR_TRACK_FIELDS)
        writer.writeheader()
        writer.writerows(radar_track_rows)

    with twin_messages_file.open("w", newline="", encoding="utf-8") as f:
        fields = ["sender", "receiver", "sequence", "generation_time_s", "scheduled_delivery_time_s", "delivery_time_s", "status", "reason"]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(twin_message_rows)

    if executive is not None:
        with executive_file.open("w", encoding="utf-8") as f:
            for row in executive_action_rows:
                f.write(json.dumps(row, allow_nan=False) + "\n")

    if phase_2_2d_seed_policy is not None:
        with sensor_measurements_file.open("w", newline="", encoding="utf-8") as f:
            fields = [
                "simulation_time_s", "vehicle_id", "platoon_predecessor_id",
                "odometry_position_m", "odometry_speed_mps", "odometry_acceleration_mps2",
                "odometry_valid", "odometry_status", "radar_sensed_leader_id",
                "radar_range_m", "radar_relative_speed_mps", "radar_valid", "radar_status",
            ]
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(sensor_measurement_rows)
        with vehicle_lifecycle_file.open("w", newline="", encoding="utf-8") as f:
            fields = ["simulation_time_s", "active_vehicle_ids", "departed_vehicle_ids", "arrived_vehicle_ids"]
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(vehicle_lifecycle_rows)

    if ambient_generator is not None:
        with ambient_traffic_file.open("w", newline="", encoding="utf-8") as f:
            fields = ["simulation_time_s", "vehicle_id", "origin_edge", "destination_edge", "scheduled_time_s", "inserted"]
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(ambient_traffic_rows)

    events_file.write_text(json.dumps(events_log, indent=2), encoding="utf-8")
    degradation_summary = {
        "scenario_id": scenario_id,
        "seed": spec.get("seed", 20260929),
        "twin_sensor_configuration": {
            "radar_noise_std_m": sensor_interface.radar_noise_std_m,
            "odometry_speed_noise_std_mps": sensor_interface.odometry_speed_noise_std_mps,
            "radar_dropout_probability": sensor_interface.radar_dropout_probability,
        },
        "twin_only_degradation": twin_degradation.summary(),
        "twin_v2x_channel_statistics": twin_channel.statistics,
    }
    if phase_2_2d_seed_policy is not None:
        degradation_summary["phase_2_2d_reproducibility"] = phase_2_2d_seed_policy
    degradation_summary_file.write_text(json.dumps(degradation_summary, indent=2), encoding="utf-8")
    estimator_evaluation_file.write_text(
        json.dumps(
            evaluate_csv_files(
                truth_file, filtered_file, scenario_id=scenario_id, simulation_step_s=phase_config.step_length
            ),
            indent=2,
        ),
        encoding="utf-8",
    )
    uncertainty_evaluation_file.write_text(
        json.dumps(
            evaluate_uncertainty_csv_files(
                truth_file, filtered_file, scenario_id=scenario_id, simulation_step_s=phase_config.step_length
            ),
            indent=2,
        ),
        encoding="utf-8",
    )

    # Pass collision metadata in spec
    spec["collisions"] = collisions_count

    # Calculate Metrics & Generate Graphs & HTML Report
    metrics_dict, pass_status, pass_reason = calculate_metrics(telemetry_file, messages_file, events_file, spec, output_dir)
    generate_all_plots(telemetry_file, messages_file, events_file, spec, output_dir)
    generate_report(output_dir, spec, metrics_dict, events_log)

    print(f"[{'PASS' if pass_status else 'FAIL'}] Scenario '{scenario_id}' completed -> Results in {output_dir}")
    return pass_status, metrics_dict


def main() -> None:
    specs = load_specs()

    parser = argparse.ArgumentParser(description="SAMTRUCK Standalone TraCI Evaluation Scenario Runner.")
    parser.add_argument("--scenario", help="Scenario ID from scenarios/catalog, 'all', or 'random_journey'")
    parser.add_argument("--gui", action="store_true", help="Launch SUMO GUI for visualization")
    parser.add_argument("--duration", type=float, default=None, help="Override scenario duration in seconds")
    parser.add_argument("--fast-demo", action="store_true", help="Shift first event to 5.0s and use bounded fast duration")
    parser.add_argument("--list", action="store_true", help="List all available scenarios in catalog")
    parser.add_argument("--seed", type=int, default=20260929)
    parser.add_argument("--evaluator-console", action="store_true", help="Print passive Twin/evaluation snapshots and write evaluator_console.jsonl")
    parser.add_argument("--evaluator-console-period", type=float, default=1.0, help="Simulation-time seconds between console blocks")
    parser.add_argument("--twin-advisor", action="store_true", help="Print passive Twin advisory reasoning and write advisor_decisions.jsonl")
    parser.add_argument("--twin-advisor-period", type=float, default=1.0, help="Simulation-time seconds between passive advisor blocks")
    parser.add_argument("--twin-executive", action="store_true", help="Opt in to bounded executive controller parameter changes")
    parser.add_argument("--twin-executive-period", type=float, default=1.0, help="Simulation-time seconds between executive status blocks")
    parser.add_argument("--ambient-random-traffic", action="store_true", help="Inject seeded SUMO-controlled ambient traffic; platoon route remains fixed")
    parser.add_argument("--ambient-traffic-rate", type=float, default=18.0, help="Ambient traffic requests per simulated minute (opt-in)")
    parser.add_argument("--ambient-traffic-seed", type=int, default=None, help="Optional seed isolated from scenario, sensor, and V2X RNGs")
    parser.add_argument("--event-console", action="store_true", help="Print scenario-event and ambient-traffic transitions in the live terminal")
    parser.add_argument("--output-id", default=None, help="Write one run to results/<output-id> instead of results/<scenario>")
    parser.add_argument("--refuse-existing-results", action="store_true", help="Fail instead of overwriting an existing results directory")

    args = parser.parse_args()

    if args.twin_executive and not args.twin_advisor:
        parser.error("--twin-executive requires --twin-advisor")
    if args.ambient_traffic_rate <= 0:
        parser.error("--ambient-traffic-rate must be positive")
    if args.scenario == "all" and args.output_id:
        parser.error("--output-id is only supported for one scenario at a time")

    if args.list:
        print(f"Available Scenarios in Catalog ({len(specs)} total):")
        for s_id, s_spec in sorted(specs.items()):
            print(f"  - {s_id}: {s_spec.get('title')} (duration: {s_spec.get('duration_s')}s, events: {len(s_spec.get('events', []))})")
        return

    if not args.scenario:
        parser.error("Please specify --scenario <name|all|random_journey> or --list")

    if args.scenario == "random_journey":
        if args.duration is None or args.duration <= 0:
            parser.error("--scenario random_journey requires a finite --duration <seconds> argument")
        master_spec = schedule_master(args.seed, specs)
        master_spec["duration_s"] = args.duration
        # The long randomized path has an explicit four-way seed contract:
        # SUMO demand, scheduled disturbances, sensors, and Twin V2X can be
        # replayed exactly with the same --seed value.
        master_spec["phase_2_2d"] = {"sumo_seed": args.seed}
        master_spec["twin"] = {"sensor_seed": args.seed, "v2x_seed": args.seed}
        # A bounded demonstration validates only events that can actually
        # start during its requested horizon.  The Twin never receives this
        # schedule; it still reacts exclusively to DTOs and delivered V2X.
        master_spec["events"] = [
            event for event in master_spec["events"]
            if float(event.get("at_s", 0.0)) < args.duration
        ]
        if args.fast_demo:
            master_spec, args.duration = apply_fast_demo(master_spec, args.duration)
        run_single_scenario(master_spec, gui=args.gui, duration_override=args.duration,
                            evaluator_console=args.evaluator_console, evaluator_console_period_s=args.evaluator_console_period,
                            twin_advisor=args.twin_advisor, twin_advisor_period_s=args.twin_advisor_period,
                            twin_executive=args.twin_executive, twin_executive_period_s=args.twin_executive_period,
                            ambient_random_traffic=args.ambient_random_traffic, ambient_traffic_rate_vpm=args.ambient_traffic_rate,
                            ambient_traffic_seed=args.ambient_traffic_seed, event_console=args.event_console,
                            output_id=args.output_id, refuse_existing_results=args.refuse_existing_results)
        return

    if args.scenario == "all":
        summary_results = []
        mode_str = " (Fast-Demo Mode)" if args.fast_demo else ""
        print(f"Running all {len(specs)} catalog scenarios sequentially{mode_str}...\n" + "=" * 60)
        for s_id, raw_spec in sorted(specs.items()):
            spec = raw_spec
            duration_override = args.duration
            if args.fast_demo:
                spec, duration_override = apply_fast_demo(raw_spec, duration_override)
            print(f"\n---> Running scenario: {s_id} (duration: {duration_override}s)")
            try:
                pass_status, metrics = run_single_scenario(spec, gui=args.gui, duration_override=duration_override,
                                                           evaluator_console=args.evaluator_console,
                                                           evaluator_console_period_s=args.evaluator_console_period,
                                                           twin_advisor=args.twin_advisor,
                                                           twin_advisor_period_s=args.twin_advisor_period,
                                                           twin_executive=args.twin_executive,
                                                           twin_executive_period_s=args.twin_executive_period,
                                                           ambient_random_traffic=args.ambient_random_traffic,
                                                           ambient_traffic_rate_vpm=args.ambient_traffic_rate,
                                                           ambient_traffic_seed=args.ambient_traffic_seed,
                                                           event_console=args.event_console)
                summary_results.append((s_id, "PASS" if pass_status else "FAIL", metrics["pass_fail"]["reason"]))
            except Exception as ex:
                print(f"Scenario {s_id} encountered an error: {ex}")
                summary_results.append((s_id, "FAIL", f"Runtime exception: {ex}"))

        print("\n" + "=" * 60)
        print("SCENARIO-BY-SCENARIO SUMMARY RESULTS:")
        print("=" * 60)
        passed_count = 0
        for s_id, status, reason in summary_results:
            if status == "PASS":
                passed_count += 1
            print(f"  [{status:4s}] {s_id:<32s} : {reason}")
        print("-" * 60)
        print(f"Total: {passed_count}/{len(summary_results)} Scenarios PASSED.")
        return

    if args.scenario not in specs:
        parser.error(f"Unknown scenario '{args.scenario}'. Use --list to see available scenarios.")

    spec = specs[args.scenario]
    duration_override = args.duration
    if args.fast_demo:
        spec, duration_override = apply_fast_demo(spec, duration_override)

    run_single_scenario(spec, gui=args.gui, duration_override=duration_override,
                        evaluator_console=args.evaluator_console, evaluator_console_period_s=args.evaluator_console_period,
                        twin_advisor=args.twin_advisor, twin_advisor_period_s=args.twin_advisor_period,
                        twin_executive=args.twin_executive, twin_executive_period_s=args.twin_executive_period,
                        ambient_random_traffic=args.ambient_random_traffic, ambient_traffic_rate_vpm=args.ambient_traffic_rate,
                        ambient_traffic_seed=args.ambient_traffic_seed, event_console=args.event_console,
                        output_id=args.output_id, refuse_existing_results=args.refuse_existing_results)


if __name__ == "__main__":
    main()
