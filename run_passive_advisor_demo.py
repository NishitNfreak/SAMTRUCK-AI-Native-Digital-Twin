"""Run a visible, passive-only Twin advisor demonstration without actuation."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from copy import deepcopy

from run_all_scenarios import RESULTS_DIR, run_single_scenario
from samtruck.scenario_suite import load_specs


RUN_ID = "passive_advisor_demo"
HIGH_NOISE_CONFIGURATION = {
    "radar_noise_std_m": 1.50,
    "odometry_speed_noise_std_mps": 0.60,
}


def build_demo_spec(*, run_id: str, duration_s: float, sumo_seed: int, scenario_seed: int,
                    sensor_seed: int, twin_v2x_seed: int) -> dict:
    """Clone normal platooning with Twin-only adversity for advisory visibility."""

    base = deepcopy(load_specs()["normal_platooning"])
    base.update({
        "id": run_id,
        "title": "Milestone B passive decision advisor demonstration",
        "duration_s": duration_s,
        "seed": scenario_seed,
        "twin": {
            **deepcopy(base.get("twin", {})),
            **HIGH_NOISE_CONFIGURATION,
            "sensor_seed": sensor_seed,
            "v2x_seed": twin_v2x_seed,
        },
        "phase_2_2d": {"sumo_seed": sumo_seed},
        # These affect only post-boundary Twin DTO/V2X observations.  The
        # frozen baseline CACC channel and vehicle actuation are unaffected.
        "twin_only_degradations": [
            {"type": "twin_odometry_dropout", "at_s": 3.0, "duration_s": 4.0},
            {"type": "twin_v2x_outage", "at_s": 3.0, "duration_s": 4.0},
        ],
    })
    return base


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the Milestone B passive decision-advisor demonstration.")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--run-id", default=RUN_ID)
    parser.add_argument("--sumo-seed", type=int, default=20260929)
    parser.add_argument("--scenario-seed", type=int, default=20260929)
    parser.add_argument("--sensor-seed", type=int, default=20260929)
    parser.add_argument("--twin-v2x-seed", type=int, default=20260929)
    parser.add_argument("--evaluator-console-period", type=float, default=1.0)
    parser.add_argument("--advisor-period", type=float, default=1.0)
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive.")
    if any(seed < 0 for seed in (args.sumo_seed, args.scenario_seed, args.sensor_seed, args.twin_v2x_seed)):
        parser.error("All seeds must be non-negative integers.")
    output_dir = RESULTS_DIR / args.run_id
    if output_dir.exists():
        parser.error(f"Refusing to overwrite an existing advisor demonstration result: {output_dir}")

    spec = build_demo_spec(
        run_id=args.run_id, duration_s=args.duration, sumo_seed=args.sumo_seed,
        scenario_seed=args.scenario_seed, sensor_seed=args.sensor_seed, twin_v2x_seed=args.twin_v2x_seed,
    )
    passed, _ = run_single_scenario(
        spec, gui=False, duration_override=args.duration, evaluator_console=True,
        evaluator_console_period_s=args.evaluator_console_period, twin_advisor=True,
        twin_advisor_period_s=args.advisor_period,
    )
    decisions_file = output_dir / "advisor_decisions.jsonl"
    decisions = [json.loads(line) for line in decisions_file.read_text(encoding="utf-8").splitlines() if line.strip()]
    summary = {
        "run_verdict": "PASS" if passed else "FAIL",
        "decision_records": len(decisions),
        "decision_counts": dict(sorted(Counter(row["decision_code"] for row in decisions).items())),
        "all_commands_none": all(row.get("command_to_executive") == "NONE" for row in decisions),
        "data_boundary": "Advisor receives Twin-side snapshots only; SUMO truth is never mapped into AdvisorInput.",
        "seeds": {
            "sumo_seed": args.sumo_seed, "scenario_seed": args.scenario_seed,
            "sensor_seed": args.sensor_seed, "twin_v2x_seed": args.twin_v2x_seed,
        },
        "twin_only_degradations": spec["twin_only_degradations"],
    }
    (output_dir / "advisor_demo_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if passed and summary["all_commands_none"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
