"""Read and format SAMTRUCK JSONL logs while a simulation is running."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from samtruck.live_log_viewer import format_advisor, format_evaluator, format_executive


FILES = {
    "evaluator": ("evaluator_console.jsonl", format_evaluator),
    "advisor": ("advisor_decisions.jsonl", format_advisor),
    "executive": ("executive_actions.jsonl", format_executive),
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only readable live SAMTRUCK log viewer.")
    parser.add_argument("--results-dir", required=True, type=Path)
    parser.add_argument("--stream", choices=sorted(FILES), required=True)
    parser.add_argument("--every-s", type=float, default=1.0, help="Minimum simulation-time interval per vehicle to display")
    parser.add_argument("--vehicle", default=None, help="Optional vehicle ID filter")
    args = parser.parse_args()
    if args.every_s <= 0.0:
        parser.error("--every-s must be positive")

    filename, formatter = FILES[args.stream]
    path = args.results_dir / filename
    print(f"Waiting for {path} ...")
    while not path.exists():
        time.sleep(0.2)

    last_shown: dict[str, float] = {}
    with path.open("r", encoding="utf-8") as stream:
        while True:
            line = stream.readline()
            if not line:
                time.sleep(0.15)
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                print("[Skipped malformed partial JSONL record]")
                continue
            vehicle_id = str(record.get("vehicle_id", "NA"))
            if args.vehicle and vehicle_id != args.vehicle:
                continue
            try:
                now_s = float(record.get("simulation_time_s"))
            except (TypeError, ValueError):
                now_s = float("inf")
            if now_s - last_shown.get(vehicle_id, -float("inf")) + 1e-9 < args.every_s:
                continue
            last_shown[vehicle_id] = now_s
            print(formatter(record), flush=True)


if __name__ == "__main__":
    main()
