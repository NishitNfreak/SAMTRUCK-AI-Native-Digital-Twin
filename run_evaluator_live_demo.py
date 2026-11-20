"""Run one fresh high-noise evaluator-console demonstration for SAMTRUCK."""

from __future__ import annotations

import argparse
import json
from copy import deepcopy
from pathlib import Path

from run_all_scenarios import RESULTS_DIR, run_single_scenario
from samtruck.high_noise_demo import calculate_speed_comparison, matched_speed_records, plot_speed_comparison
from samtruck.scenario_suite import load_specs


RUN_ID = "evaluator_live_demo"
HIGH_NOISE_CONFIGURATION = {"radar_noise_std_m": 1.50, "odometry_speed_noise_std_mps": 0.60}


def build_demo_spec(*, run_id: str, duration_s: float, sumo_seed: int, scenario_seed: int,
                    sensor_seed: int, twin_v2x_seed: int) -> dict:
    """Clone normal platooning with only opt-in Twin observability/noise settings."""

    base = deepcopy(load_specs()["normal_platooning"])
    base.update({
        "id": run_id,
        "title": "Live Digital Twin evaluator console demonstration",
        "duration_s": duration_s,
        "seed": scenario_seed,
        "twin": {**deepcopy(base.get("twin", {})), **HIGH_NOISE_CONFIGURATION,
                 "sensor_seed": sensor_seed, "v2x_seed": twin_v2x_seed},
        "phase_2_2d": {"sumo_seed": sumo_seed},
    })
    return base


def main() -> int:
    parser = argparse.ArgumentParser(description="Run one fresh SAMTRUCK live evaluator-console demonstration.")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--vehicle", default="samtruck_follower_1")
    parser.add_argument("--sumo-seed", type=int, default=20260929)
    parser.add_argument("--scenario-seed", type=int, default=20260929)
    parser.add_argument("--sensor-seed", type=int, default=20260929)
    parser.add_argument("--twin-v2x-seed", type=int, default=20260929)
    parser.add_argument("--evaluator-console-period", type=float, default=1.0)
    parser.add_argument("--run-id", default=RUN_ID, help="Fresh results subdirectory name; never overwritten")
    args = parser.parse_args()
    if args.duration <= 0.0 or args.evaluator_console_period <= 0.0:
        parser.error("--duration and --evaluator-console-period must be positive.")
    if any(seed < 0 for seed in (args.sumo_seed, args.scenario_seed, args.sensor_seed, args.twin_v2x_seed)):
        parser.error("All seeds must be non-negative integers.")

    if not args.run_id or Path(args.run_id).name != args.run_id:
        parser.error("--run-id must be a simple results subdirectory name.")
    output_dir = RESULTS_DIR / args.run_id
    if output_dir.exists():
        parser.error(f"Refusing to overwrite an existing evaluator demonstration: {output_dir}")
    spec = build_demo_spec(run_id=args.run_id, duration_s=args.duration, sumo_seed=args.sumo_seed, scenario_seed=args.scenario_seed,
                           sensor_seed=args.sensor_seed, twin_v2x_seed=args.twin_v2x_seed)
    passed, _ = run_single_scenario(
        spec, gui=False, duration_override=args.duration,
        evaluator_console=True, evaluator_console_period_s=args.evaluator_console_period,
    )
    records, diagnostics = matched_speed_records(
        output_dir / "twin_ground_truth.csv", output_dir / "twin_sensor_measurements.csv",
        output_dir / "twin_filtered_states.csv", args.vehicle,
    )
    metrics = calculate_speed_comparison(records, diagnostics)
    points = plot_speed_comparison(records, output_dir / "evaluator_speed_comparison.png", args.vehicle)
    if not all(points.values()) or len(set(points.values())) != 1:
        raise RuntimeError(f"Required three-series graph validation failed: {points}")
    report = {
        "run_verdict": "PASS" if passed else "FAIL",
        "vehicle_id": args.vehicle,
        "configuration": {"duration_s": args.duration, **HIGH_NOISE_CONFIGURATION, "sumo_seed": args.sumo_seed,
                            "scenario_seed": args.scenario_seed, "sensor_seed": args.sensor_seed,
                            "twin_v2x_seed": args.twin_v2x_seed, "evaluator_console_period_s": args.evaluator_console_period},
        "sources": {"truth": "twin_ground_truth.csv (offline reference)", "sensor": "twin_sensor_measurements.csv (Twin-side DTO)",
                    "filtered": "twin_filtered_states.csv"},
        "plot": {"file": "evaluator_speed_comparison.png", "series_points": points},
        "metrics": metrics,
    }
    metrics_path = output_dir / "evaluator_live_demo_metrics.json"
    metrics_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    raw = metrics["raw_noisy_odometry_vs_sumo_truth"]
    filtered = metrics["filtered_twin_vs_sumo_truth"]
    improvement = metrics["improvement_filtered_over_raw"]
    print("\nLIVE EVALUATOR DEMO SUMMARY")
    print(f"Sensor speed:   MAE={raw['mae_mps']:.4f} m/s, RMSE={raw['rmse_mps']:.4f} m/s")
    print(f"Filtered speed: MAE={filtered['mae_mps']:.4f} m/s, RMSE={filtered['rmse_mps']:.4f} m/s")
    print(f"Improvement:    MAE={improvement['mae_percent']:.2f}%, RMSE={improvement['rmse_percent']:.2f}%")
    print(f"PNG: {output_dir / 'evaluator_speed_comparison.png'}")
    print(f"JSONL: {output_dir / 'evaluator_console.jsonl'}")
    print(f"Evaluation: {output_dir / 'twin_filtered_evaluation.json'}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
