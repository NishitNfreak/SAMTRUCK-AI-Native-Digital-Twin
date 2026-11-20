"""Read-only inspector for completed SAMTRUCK Twin result directories.

SUMO truth displayed by this tool is an independent offline evaluation
reference.  It was not, and must not be, a live input to the Digital Twin.
"""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path
from typing import Iterable


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _time_key(row: dict[str, str]) -> str:
    return f"{float(row['simulation_time_s']):.6f}"


def _selected(rows: Iterable[dict[str, str]], vehicle_id: str, start: float | None, end: float | None,
              vehicle_field: str = "vehicle_id") -> list[dict[str, str]]:
    selected: list[dict[str, str]] = []
    for row in rows:
        if row.get(vehicle_field) != vehicle_id:
            continue
        timestamp = float(row["simulation_time_s"])
        if (start is not None and timestamp < start) or (end is not None and timestamp > end):
            continue
        selected.append(row)
    return selected


def collect_trace_records(results_dir: Path, vehicle_id: str, start: float | None = None,
                          end: float | None = None) -> tuple[list[dict[str, object]], dict[str, object]]:
    """Join persisted artifacts by exact normalized simulation timestamp.

    Raw sensor DTO rows exist only for opt-in Phase 2.2D runs.  Older result
    folders legitimately return ``sensor_dtos_persisted=False`` rather than a
    fabricated measurement value.
    """

    truth = _selected(read_csv(results_dir / "twin_ground_truth.csv"), vehicle_id, start, end)
    filtered = _selected(read_csv(results_dir / "twin_filtered_states.csv"), vehicle_id, start, end)
    sensors_file = results_dir / "twin_sensor_measurements.csv"
    sensors = _selected(read_csv(sensors_file), vehicle_id, start, end)
    radar = _selected(read_csv(results_dir / "twin_radar_tracks.csv"), vehicle_id, start, end, "ego_vehicle_id")
    v2x = [
        row for row in read_csv(results_dir / "twin_v2x_messages.csv")
        if row.get("sender") == vehicle_id or row.get("receiver") == vehicle_id
    ]
    if start is not None or end is not None:
        v2x = [
            row for row in v2x
            if (start is None or float(row["generation_time_s"]) >= start)
            and (end is None or float(row["generation_time_s"]) <= end)
        ]

    by_time: dict[str, dict[str, object]] = defaultdict(dict)
    for label, rows in (("truth", truth), ("sensor", sensors), ("filtered", filtered), ("radar", radar)):
        for row in rows:
            by_time[_time_key(row)][label] = row
    for row in v2x:
        by_time[f"{float(row['generation_time_s']):.6f}"].setdefault("v2x", []).append(row)

    records: list[dict[str, object]] = []
    for timestamp, data in sorted(by_time.items(), key=lambda item: float(item[0])):
        records.append({"simulation_time_s": float(timestamp), **data})
    summary = {
        "results_dir": str(results_dir),
        "vehicle_id": vehicle_id,
        "records": len(records),
        "truth_rows": len(truth),
        "filtered_rows": len(filtered),
        "sensor_dto_rows": len(sensors),
        "sensor_dtos_persisted": sensors_file.exists(),
        "radar_track_rows": len(radar),
        "v2x_event_rows": len(v2x),
    }
    return records, summary


def _show(value: object | None) -> str:
    return "NA" if value is None or value == "" else str(value)


def print_records(records: list[dict[str, object]], limit: int) -> None:
    shown = records if limit == 0 else records[:limit]
    for record in shown:
        truth = record.get("truth", {})
        sensor = record.get("sensor", {})
        filtered = record.get("filtered", {})
        radar = record.get("radar", {})
        v2x = record.get("v2x", [])
        print(
            f"t={record['simulation_time_s']:.3f} "
            f"truth(v={_show(truth.get('speed_mps'))}, a={_show(truth.get('acceleration_mps2'))}, "
            f"pos_lane={_show(truth.get('lane_position_m'))}) | "
            f"sensor(v={_show(sensor.get('odometry_speed_mps'))}, a={_show(sensor.get('odometry_acceleration_mps2'))}, "
            f"valid={_show(sensor.get('odometry_valid'))}, radar={_show(sensor.get('radar_range_m'))}, "
            f"radar_valid={_show(sensor.get('radar_valid'))}) | "
            f"twin(v={_show(filtered.get('estimated_speed_mps'))}, a={_show(filtered.get('estimated_acceleration_mps2'))}, "
            f"pos={_show(filtered.get('estimated_position_m'))}, speed_var={_show(filtered.get('speed_variance_m2ps2'))}, "
            f"accel_var={_show(filtered.get('acceleration_variance_m2ps4'))}, "
            f"speed_state={_show(filtered.get('speed_estimate_provenance'))}, "
            f"speed_age={_show(filtered.get('speed_measurement_age_s'))}, "
            f"radar_track={_show(radar.get('estimated_range_m'))}, v2x_events={len(v2x)})"
        )
        for event in v2x:
            print(
                f"  V2X {event.get('sender')}->{event.get('receiver')} seq={event.get('sequence')} "
                f"status={event.get('status')} reason={event.get('reason')} "
                f"generated={event.get('generation_time_s')} scheduled={event.get('scheduled_delivery_time_s')} "
                f"delivered={_show(event.get('delivery_time_s'))}"
            )
    if limit and len(records) > limit:
        print(f"... {len(records) - limit} additional selected timestamps omitted; use --limit 0 to print all.")


def plot_speed(records: list[dict[str, object]], output: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("Optional --plot requires the already-used matplotlib package.") from exc

    timestamps = [record["simulation_time_s"] for record in records]
    truth = [float(record["truth"]["speed_mps"]) if record.get("truth", {}).get("speed_mps") else float("nan") for record in records]
    estimated = [float(record["filtered"]["estimated_speed_mps"]) if record.get("filtered", {}).get("estimated_speed_mps") else float("nan") for record in records]
    sensor = [float(record["sensor"]["odometry_speed_mps"]) if record.get("sensor", {}).get("odometry_speed_mps") else float("nan") for record in records]
    plt.figure(figsize=(10, 5))
    plt.plot(timestamps, truth, label="SUMO truth (offline reference)")
    if any(value == value for value in sensor):
        plt.plot(timestamps, sensor, label="Persisted odometry DTO", alpha=0.7)
    plt.plot(timestamps, estimated, label="Twin filtered speed", alpha=0.8)
    plt.xlabel("Simulation time (s)")
    plt.ylabel("Speed (m/s)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output, dpi=150)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only inspector for completed SAMTRUCK Twin artifacts.")
    parser.add_argument("--results-dir", required=True, type=Path)
    parser.add_argument("--vehicle", required=True)
    parser.add_argument("--start", type=float, default=None, help="Inclusive simulation-time lower bound (s)")
    parser.add_argument("--end", type=float, default=None, help="Inclusive simulation-time upper bound (s)")
    parser.add_argument("--limit", type=int, default=30, help="Rows to print; 0 prints all selected rows")
    parser.add_argument("--plot", type=Path, default=None, help="Optional output PNG for truth/sensor/filtered speed")
    args = parser.parse_args()
    if not args.results_dir.is_dir():
        parser.error("--results-dir must be an existing completed result directory.")
    if args.limit < 0:
        parser.error("--limit must be non-negative.")
    records, summary = collect_trace_records(args.results_dir, args.vehicle, args.start, args.end)
    print("Trace summary:")
    for key, value in summary.items():
        print(f"  {key}: {value}")
    if not summary["sensor_dtos_persisted"]:
        print("  NOTE: raw sensor DTOs were not persisted for this historical run; unavailable values are shown as NA.")
    print("\nRecords (SUMO truth is an offline evaluation reference, not a live Twin input):")
    print_records(records, args.limit)
    if args.plot is not None:
        plot_speed(records, args.plot)
        print(f"Wrote read-only speed plot: {args.plot}")


if __name__ == "__main__":
    main()
