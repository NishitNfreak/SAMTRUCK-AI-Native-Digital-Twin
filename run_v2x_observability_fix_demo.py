"""Run one fresh, passive demonstration of advancing Twin V2X freshness."""

from __future__ import annotations

import argparse
import json
from collections import Counter

from run_all_scenarios import RESULTS_DIR, run_single_scenario
from run_passive_advisor_demo import build_demo_spec


RUN_ID = "v2x_observability_fix_demo"
OUTAGE_START_S = 3.0
OUTAGE_END_S = 7.0


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the fresh V2X-observability fix demonstration.")
    parser.add_argument("--duration", type=float, default=12.0)
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

    output_dir = RESULTS_DIR / RUN_ID
    if output_dir.exists():
        parser.error(f"Refusing to overwrite an existing result: {output_dir}")
    spec = build_demo_spec(
        run_id=RUN_ID, duration_s=args.duration, sumo_seed=args.sumo_seed,
        scenario_seed=args.scenario_seed, sensor_seed=args.sensor_seed,
        twin_v2x_seed=args.twin_v2x_seed,
    )
    passed, _ = run_single_scenario(
        spec, gui=False, duration_override=args.duration, evaluator_console=True,
        evaluator_console_period_s=args.evaluator_console_period, twin_advisor=True,
        twin_advisor_period_s=args.advisor_period,
    )

    decisions = [
        json.loads(line) for line in (output_dir / "advisor_decisions.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    outage_decisions = [
        item for item in decisions
        if OUTAGE_START_S <= float(item["simulation_time_s"]) < OUTAGE_END_S
        and item["vehicle_id"] in {"samtruck_follower_1", "samtruck_follower_2"}
    ]
    relevant_codes = {"V2X_DEGRADED", "SENSOR_AND_V2X_DEGRADED"}
    relevant_count = sum(item["decision_code"] in relevant_codes for item in outage_decisions)
    summary = {
        "run_verdict": "PASS" if passed else "FAIL",
        "seeds": {
            "sumo_seed": args.sumo_seed, "scenario_seed": args.scenario_seed,
            "sensor_seed": args.sensor_seed, "twin_v2x_seed": args.twin_v2x_seed,
        },
        "outage_window_s": [OUTAGE_START_S, OUTAGE_END_S],
        "all_decision_counts": dict(sorted(Counter(row["decision_code"] for row in decisions).items())),
        "outage_follower_decision_counts": dict(sorted(Counter(row["decision_code"] for row in outage_decisions).items())),
        "outage_v2x_relevant_decision_count": relevant_count,
        "all_commands_none": all(row.get("command_to_executive") == "NONE" for row in decisions),
        "verified": bool(passed and relevant_count > 0 and all(row.get("command_to_executive") == "NONE" for row in decisions)),
        "data_boundary": "Advisor input is Twin-side only; SUMO truth remains offline evaluation/display data.",
    }
    (output_dir / "v2x_observability_demo_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["verified"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
