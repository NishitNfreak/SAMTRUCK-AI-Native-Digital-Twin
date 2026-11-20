"""Run one fresh opt-in active-executive safety demonstration."""

from __future__ import annotations

import argparse
import json
from collections import Counter

from run_all_scenarios import RESULTS_DIR, run_single_scenario
from run_passive_advisor_demo import build_demo_spec


RUN_ID = "active_executive_demo"


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the opt-in SAMTRUCK active executive demonstration.")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--run-id", default=RUN_ID, help="Fresh result directory name under results/")
    parser.add_argument("--sumo-seed", type=int, default=20260929)
    parser.add_argument("--scenario-seed", type=int, default=20260929)
    parser.add_argument("--sensor-seed", type=int, default=20260929)
    parser.add_argument("--twin-v2x-seed", type=int, default=20260929)
    parser.add_argument("--advisor-period", type=float, default=1.0)
    parser.add_argument("--executive-period", type=float, default=1.0)
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive.")
    if any(seed < 0 for seed in (args.sumo_seed, args.scenario_seed, args.sensor_seed, args.twin_v2x_seed)):
        parser.error("All seeds must be non-negative integers.")
    output_dir = RESULTS_DIR / args.run_id
    if output_dir.exists():
        parser.error(f"Refusing to overwrite an existing result: {output_dir}")

    spec = build_demo_spec(
        run_id=args.run_id, duration_s=args.duration, sumo_seed=args.sumo_seed,
        scenario_seed=args.scenario_seed, sensor_seed=args.sensor_seed,
        twin_v2x_seed=args.twin_v2x_seed,
    )
    spec["title"] = "Milestone C active executive demonstration"
    passed, _ = run_single_scenario(
        spec, gui=False, duration_override=args.duration, evaluator_console=True,
        evaluator_console_period_s=args.advisor_period, twin_advisor=True,
        twin_advisor_period_s=args.advisor_period, twin_executive=True,
        twin_executive_period_s=args.executive_period,
    )
    actions = [
        json.loads(line) for line in (output_dir / "executive_actions.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    metrics = json.loads((output_dir / "metrics.json").read_text(encoding="utf-8"))
    applied_gaps = [float(row["applied_time_gap_s"]) for row in actions]
    summary = {
        "run_verdict": "PASS" if passed else "FAIL",
        "seeds": {
            "sumo_seed": args.sumo_seed, "scenario_seed": args.scenario_seed,
            "sensor_seed": args.sensor_seed, "twin_v2x_seed": args.twin_v2x_seed,
        },
        "action_count": len(actions),
        "command_counts": dict(sorted(Counter(row["command"] for row in actions).items())),
        "control_mode_counts": dict(sorted(Counter(row["control_mode"] for row in actions).items())),
        "applied_time_gap_range_s": [min(applied_gaps), max(applied_gaps)] if applied_gaps else None,
        "speed_cap_events": sum(row["max_speed_cap_mps"] is not None for row in actions),
        "collision_count": metrics.get("collision_count"),
        "collision_free": metrics.get("collision_count") == 0,
        "twin_only_degradations": spec["twin_only_degradations"],
        "ground_truth_boundary": "SUMO truth remains offline evaluation data and is not an executive input.",
    }
    (output_dir / "executive_demo_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if passed and summary["collision_free"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
