"""
SAMTRUCK Baseline vs. AI-Digital Twin Comparison Framework
Compares baseline scenario evaluation data against AI-Digital Twin results when available.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
import json
import pandas as pd


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate Baseline vs AI-Digital Twin comparison report.")
    parser.add_argument("--baseline-dir", help="Path to baseline scenario results directory")
    parser.add_argument("--ai-dt-dir", help="Path to AI-Digital Twin scenario results directory")
    parser.add_argument("--output-dir", default="results/comparison", help="Output directory for comparison artifacts")

    args = parser.parse_args()

    baseline_path = Path(args.baseline_dir) if args.baseline_dir else None
    ai_dt_path = Path(args.ai_dt_dir) if args.ai_dt_dir else None

    # Verify if AI-DT data exists
    if ai_dt_path is None or not ai_dt_path.exists() or not any(ai_dt_path.iterdir()):
        print("AI-DT result directory does not exist or contains no data. Comparison skipped successfully.")
        sys.exit(0)

    if baseline_path is None or not baseline_path.exists():
        print("Baseline result directory does not exist. Comparison cannot be performed.")
        sys.exit(1)

    print(f"Comparing Baseline ({baseline_path}) with AI-DT ({ai_dt_path})...")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Read metrics
    base_metrics_file = baseline_path / "metrics.json"
    ai_metrics_file = ai_dt_path / "metrics.json"

    if base_metrics_file.exists() and ai_metrics_file.exists():
        base_m = json.loads(base_metrics_file.read_text(encoding="utf-8"))
        ai_m = json.loads(ai_metrics_file.read_text(encoding="utf-8"))

        comparison = {
            "metrics_compared": [
                "gap_error", "speed_response", "minimum_gap", "recovery_time",
                "jerk", "string_stability", "v2x_robustness", "fuel_energy"
            ],
            "baseline": base_m,
            "ai_digital_twin": ai_m
        }

        comp_json = output_dir / "comparison_metrics.json"
        comp_json.write_text(json.dumps(comparison, indent=2), encoding="utf-8")
        print(f"Comparison metrics written to {comp_json}")

    print("Comparison execution completed successfully.")


if __name__ == "__main__":
    main()
