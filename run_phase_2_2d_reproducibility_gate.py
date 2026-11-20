"""Run the approved two-run Phase 2.2D reproducibility gate only.

This script intentionally does not run the 80-run matrix.  It executes two
isolated scratch copies of the same normal-platooning specification with
explicit SUMO, scenario, sensor, and Twin-V2X seeds, then compares completed
offline artifacts.
"""

from __future__ import annotations

import argparse
import json
from copy import deepcopy
from pathlib import Path

from run_all_scenarios import RESULTS_DIR, run_single_scenario
from samtruck.reproducibility import compare_result_directories
from samtruck.scenario_suite import load_specs


def build_gate_spec(run_name: str, *, duration_s: float, sumo_seed: int, scenario_seed: int,
                    sensor_seed: int, twin_v2x_seed: int) -> dict:
    """Return an isolated normal-platooning scratch spec with explicit seeds."""

    base = deepcopy(load_specs()["normal_platooning"])
    base.update({
        "id": f"phase_2_2d_gate/{run_name}",
        "title": f"Phase 2.2D reproducibility gate — {run_name}",
        "duration_s": duration_s,
        "seed": scenario_seed,
        "twin": {
            **deepcopy(base.get("twin", {})),
            "sensor_seed": sensor_seed,
            "v2x_seed": twin_v2x_seed,
        },
        "phase_2_2d": {"sumo_seed": sumo_seed},
    })
    return base


def main() -> int:
    parser = argparse.ArgumentParser(description="Run only the two-scratch-run Phase 2.2D reproducibility gate.")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--sumo-seed", type=int, default=20260929)
    parser.add_argument("--scenario-seed", type=int, default=20260929)
    parser.add_argument("--sensor-seed", type=int, default=20260929)
    parser.add_argument("--twin-v2x-seed", type=int, default=20260929)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive.")
    if any(value < 0 for value in (args.sumo_seed, args.scenario_seed, args.sensor_seed, args.twin_v2x_seed)):
        parser.error("All seeds must be non-negative integers.")

    specs = [
        build_gate_spec("run_a", duration_s=args.duration, sumo_seed=args.sumo_seed,
                        scenario_seed=args.scenario_seed, sensor_seed=args.sensor_seed,
                        twin_v2x_seed=args.twin_v2x_seed),
        build_gate_spec("run_b", duration_s=args.duration, sumo_seed=args.sumo_seed,
                        scenario_seed=args.scenario_seed, sensor_seed=args.sensor_seed,
                        twin_v2x_seed=args.twin_v2x_seed),
    ]
    run_verdicts = []
    for spec in specs:
        passed, _ = run_single_scenario(spec, gui=False, duration_override=args.duration)
        run_verdicts.append({"scenario_id": spec["id"], "passed": passed})

    left_dir = RESULTS_DIR / specs[0]["id"]
    right_dir = RESULTS_DIR / specs[1]["id"]
    comparison = compare_result_directories(left_dir, right_dir)
    report = {
        "phase": "2.2D_reproducibility_gate",
        "manifest": {
            "sumo_seed": args.sumo_seed,
            "scenario_seed": args.scenario_seed,
            "sensor_seed": args.sensor_seed,
            "twin_v2x_seed": args.twin_v2x_seed,
            "duration_s": args.duration,
            "base_scenario": "normal_platooning",
        },
        "runs": run_verdicts,
        "comparison": comparison,
    }
    report_path = RESULTS_DIR / "phase_2_2d_gate" / "reproducibility_gate_report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if all(item["passed"] for item in run_verdicts) and comparison["matched"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
