"""Run exactly one fresh Phase 2.2D high-sensor-noise demonstration."""

from __future__ import annotations

import argparse
import json
from copy import deepcopy
from pathlib import Path

from run_all_scenarios import RESULTS_DIR, run_single_scenario
from samtruck.high_noise_demo import calculate_speed_comparison, matched_speed_records, plot_speed_comparison
from samtruck.scenario_suite import load_specs


RUN_ID = "phase_2_2d_high_noise_demo"
HIGH_NOISE_CONFIGURATION = {
    "radar_noise_std_m": 1.50,
    "odometry_speed_noise_std_mps": 0.60,
}


def build_demo_spec(*, duration_s: float, sumo_seed: int, scenario_seed: int,
                    sensor_seed: int, twin_v2x_seed: int) -> dict:
    """Clone normal platooning and add Phase 2.2D-only explicit settings."""

    base = deepcopy(load_specs()["normal_platooning"])
    base.update({
        "id": RUN_ID,
        "title": "Phase 2.2D high sensor-noise Kalman demonstration",
        "duration_s": duration_s,
        "seed": scenario_seed,
        "twin": {
            **deepcopy(base.get("twin", {})),
            **HIGH_NOISE_CONFIGURATION,
            "sensor_seed": sensor_seed,
            "v2x_seed": twin_v2x_seed,
        },
        "phase_2_2d": {"sumo_seed": sumo_seed},
    })
    return base


def main() -> int:
    parser = argparse.ArgumentParser(description="Run one fresh Phase 2.2D high-noise demonstration.")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--vehicle", default="samtruck_follower_1")
    parser.add_argument("--sumo-seed", type=int, default=20260929)
    parser.add_argument("--scenario-seed", type=int, default=20260929)
    parser.add_argument("--sensor-seed", type=int, default=20260929)
    parser.add_argument("--twin-v2x-seed", type=int, default=20260929)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive.")
    if any(seed < 0 for seed in (args.sumo_seed, args.scenario_seed, args.sensor_seed, args.twin_v2x_seed)):
        parser.error("All seeds must be non-negative integers.")

    output_dir = RESULTS_DIR / RUN_ID
    if output_dir.exists():
        parser.error(f"Refusing to overwrite an existing demonstration result: {output_dir}")
    spec = build_demo_spec(
        duration_s=args.duration, sumo_seed=args.sumo_seed, scenario_seed=args.scenario_seed,
        sensor_seed=args.sensor_seed, twin_v2x_seed=args.twin_v2x_seed,
    )
    passed, _ = run_single_scenario(spec, gui=False, duration_override=args.duration)
    records, diagnostics = matched_speed_records(
        output_dir / "twin_ground_truth.csv",
        output_dir / "twin_sensor_measurements.csv",
        output_dir / "twin_filtered_states.csv",
        args.vehicle,
    )
    metrics = calculate_speed_comparison(records, diagnostics)
    plot_counts = plot_speed_comparison(records, output_dir / "high_noise_speed_comparison.png", args.vehicle)
    if not all(plot_counts.values()) or len({*plot_counts.values()}) != 1:
        raise RuntimeError(f"Three-series graph validation failed: {plot_counts}")
    report = {
        "experiment": "Phase 2.2D single high-sensor-noise demonstration",
        "run_verdict": "PASS" if passed else "FAIL",
        "vehicle_id": args.vehicle,
        "configuration": {
            "base_scenario": "normal_platooning",
            "duration_s": args.duration,
            **HIGH_NOISE_CONFIGURATION,
            "sumo_seed": args.sumo_seed,
            "scenario_seed": args.scenario_seed,
            "sensor_seed": args.sensor_seed,
            "twin_v2x_seed": args.twin_v2x_seed,
        },
        "data_sources": {
            "sumo_truth": "twin_ground_truth.csv (offline evaluation reference)",
            "noisy_odometry": "twin_sensor_measurements.csv (persisted DTO)",
            "filtered_twin": "twin_filtered_states.csv",
        },
        "plot": {"file": "high_noise_speed_comparison.png", "series_points": plot_counts},
        "metrics": metrics,
    }
    report_path = output_dir / "high_noise_speed_metrics.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
