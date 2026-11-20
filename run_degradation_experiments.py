"""Run the Phase 2.2C Twin-only degradation matrix reproducibly."""

from __future__ import annotations

import argparse
import json
from copy import deepcopy
from pathlib import Path

from run_all_scenarios import RESULTS_DIR, run_single_scenario
from samtruck.scenario_suite import load_specs


PROJECT_ROOT = Path(__file__).resolve().parent
CONFIG_FILE = PROJECT_ROOT / "scenarios" / "experiments" / "phase_2_2c.json"


def _recorded_pass_status(output_dir: Path) -> tuple[bool | None, str]:
    """Read the original runner verdict from a completed metrics artifact.

    Summary reconstruction must not infer success merely from a folder or CSV.
    Only the runner's persisted ``metrics.json.pass_fail.status`` is an
    authoritative historical verdict.  Missing, malformed, or unrecognized
    records remain explicitly unknown.
    """

    metrics_file = output_dir / "metrics.json"
    if not metrics_file.exists():
        return None, "unknown_missing_metrics_json"
    try:
        metrics = json.loads(metrics_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None, "unknown_invalid_metrics_json"
    status = str(metrics.get("pass_fail", {}).get("status", "")).upper()
    if status == "PASS":
        return True, "recorded_pass"
    if status == "FAIL":
        return False, "recorded_fail"
    return None, "unknown_missing_or_unrecognized_pass_status"


def _result_summary(run_id: str, experiment: dict, seed: int, passed: bool | None, status_source: str) -> dict:
    output_dir = RESULTS_DIR / run_id
    degradation_file = output_dir / "twin_degradation_summary.json"
    uncertainty_file = output_dir / "twin_uncertainty_evaluation.json"
    try:
        degradation = json.loads(degradation_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        degradation = None
    try:
        uncertainty = json.loads(uncertainty_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        uncertainty = None
    return {
        "run_id": run_id,
        "experiment_id": experiment["id"],
        "seed": seed,
        "pass": passed,
        "pass_status_source": status_source,
        "result_completeness": "complete" if degradation is not None and uncertainty is not None else "incomplete",
        "requested_twin_configuration": experiment.get("twin", {}),
        "requested_twin_only_degradations": experiment.get("twin_only_degradations", []),
        "degradation_summary": degradation,
        "uncertainty_metrics": None if uncertainty is None else uncertainty.get("metrics"),
        "matching_diagnostics": None if uncertainty is None else uncertainty.get("matching_diagnostics"),
    }


def run_matrix(config: dict, selected_ids: set[str] | None = None, limit: int | None = None) -> list[dict]:
    specs = load_specs()
    base_id = config["base_scenario"]
    if base_id not in specs:
        raise ValueError(f"Experiment base scenario {base_id!r} is not in the official catalog.")
    records: list[dict] = []
    for experiment in config["experiments"]:
        if selected_ids and experiment["id"] not in selected_ids:
            continue
        for seed in experiment["seeds"]:
            if limit is not None and len(records) >= limit:
                return records
            spec = deepcopy(specs[base_id])
            run_id = f"{experiment['id']}_s{seed}"
            spec.update({
                "id": run_id,
                "title": f"Phase 2.2C: {experiment['id']} (seed {seed})",
                "duration_s": float(config["duration_s"]),
                "seed": int(seed),
                "twin": deepcopy(experiment.get("twin", {})),
                "twin_only_degradations": deepcopy(experiment.get("twin_only_degradations", [])),
                # Preserve the normal scenario's physical events.  All added
                # degradation remains in the passive Twin branch.
                "events": deepcopy(specs[base_id].get("events", [])),
            })
            passed, _ = run_single_scenario(spec, gui=False, duration_override=float(config["duration_s"]))
            records.append(_result_summary(
                run_id, experiment, int(seed), passed, "live_runner_return"
            ))
    return records


def summarize_existing(config: dict, selected_ids: set[str] | None = None) -> list[dict]:
    """Rebuild the matrix summary from completed runs without rerunning SUMO."""

    records: list[dict] = []
    for experiment in config["experiments"]:
        if selected_ids and experiment["id"] not in selected_ids:
            continue
        for seed in experiment["seeds"]:
            run_id = f"{experiment['id']}_s{seed}"
            output_dir = RESULTS_DIR / run_id
            passed, source = _recorded_pass_status(output_dir)
            records.append(_result_summary(run_id, experiment, int(seed), passed, source))
    return records


def main() -> None:
    parser = argparse.ArgumentParser(description="Run reproducible Phase 2.2C Twin-only degradation experiments.")
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--ids", nargs="*", help="Optional experiment IDs to run")
    parser.add_argument("--limit", type=int, default=None, help="Optional maximum number of runs")
    parser.add_argument("--summarize-existing", action="store_true", help="Write summary from completed result folders without running SUMO")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    records = (
        summarize_existing(config, set(args.ids) if args.ids else None)
        if args.summarize_existing
        else run_matrix(config, set(args.ids) if args.ids else None, args.limit)
    )
    summary_path = RESULTS_DIR / "phase_2_2c_experiments_summary.json"
    summary_path.write_text(json.dumps({"config": config, "runs": records}, indent=2), encoding="utf-8")
    print(f"Phase 2.2C runs completed: {len(records)} -> {summary_path}")


if __name__ == "__main__":
    main()
