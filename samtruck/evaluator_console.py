"""Passive live display and JSONL logger for the SAMTRUCK evaluator demo.

This helper has no TraCI, controller, estimator, or random-state dependency.
It only renders already-produced evaluation truth, Twin-side sensor DTO, and
post-update filtered-state snapshots.  SUMO truth remains an offline
evaluation reference and is never returned to the live Twin.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Callable, Iterable


def _display_number(value: object | None, width: int = 9) -> str:
    if value is None or value == "":
        return f"{'NA':>{width}}"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return f"{str(value):>{width}}"
    return f"{number:>{width}.2f}" if math.isfinite(number) else f"{'NA':>{width}}"


class EvaluatorConsole:
    """Write every snapshot to JSONL and print compact blocks at a sim-time period."""

    def __init__(self, jsonl_path: Path, *, period_s: float = 1.0, printer: Callable[[str], None] = print) -> None:
        if not math.isfinite(period_s) or period_s <= 0.0:
            raise ValueError("Evaluator-console period must be a positive finite number of seconds.")
        self.period_s = period_s
        self._printer = printer
        self._handle = jsonl_path.open("w", encoding="utf-8")
        self._next_print_s: float | None = None

    def should_print(self, simulation_time_s: float) -> bool:
        """Return whether this simulation timestamp should render a terminal block."""

        if self._next_print_s is None:
            self._next_print_s = simulation_time_s
        if simulation_time_s + 1e-9 < self._next_print_s:
            return False
        while self._next_print_s <= simulation_time_s + 1e-9:
            self._next_print_s += self.period_s
        return True

    def record(self, snapshots: Iterable[dict[str, Any]]) -> None:
        """Append one JSON object per vehicle at every simulation timestamp."""

        for snapshot in snapshots:
            self._handle.write(json.dumps(snapshot, allow_nan=False) + "\n")
        self._handle.flush()

    def maybe_print(self, simulation_time_s: float, snapshots: list[dict[str, Any]]) -> None:
        """Print a compact evaluator block only when the configured period expires."""

        if not self.should_print(simulation_time_s):
            return
        self._printer("=" * 78)
        self._printer(f"t = {simulation_time_s:.1f}s | LIVE DIGITAL TWIN EVALUATOR CONSOLE")
        self._printer("Truth = SUMO offline reference | Sensor = Twin-side DTO | Twin = passive filter")
        # ASCII label keeps the live display compatible with Windows CP1252
        # terminals while the JSONL retains numeric variance values.
        self._printer("Vehicle                Truth spd  Sensor spd    Twin spd   sigma     Age  Flag")
        for snapshot in snapshots:
            sensor_valid = snapshot.get("sensor_odometry_valid")
            sensor_speed = _display_number(snapshot.get("sensor_odometry_speed_mps")) if sensor_valid else f"{'DROP':>9}"
            variance = snapshot.get("twin_speed_variance")
            sigma = None if variance is None else math.sqrt(max(0.0, float(variance)))
            provenance = str(snapshot.get("speed_estimate_provenance") or "NA")
            flag = "PRED" if provenance == "prediction_only" else "MEAS" if provenance == "measurement_updated" else provenance[:4].upper()
            self._printer(
                f"{str(snapshot.get('vehicle_id', 'NA')):<22}"
                f"{_display_number(snapshot.get('truth_speed_mps'))}"
                f"{sensor_speed}"
                f"{_display_number(snapshot.get('twin_filtered_speed_mps'))}"
                f"{_display_number(sigma, 9)}"
                f"{_display_number(snapshot.get('speed_measurement_age_s'), 7)}  {flag}"
            )
        notes = []
        for snapshot in snapshots:
            status = snapshot.get("predecessor_v2x_status")
            if status and status != "not_available":
                notes.append(
                    f"{snapshot['vehicle_id']} V2X={status} age={_display_number(snapshot.get('predecessor_v2x_age_s'), 0).strip()}s"
                )
        self._printer("-" * 78)
        self._printer("Notes: DROP = invalid/missing DTO | sigma = sqrt(speed variance) | " + (" | ".join(notes) if notes else "V2X: no predecessor message"))
        self._printer("=" * 78)

    def close(self) -> None:
        if not self._handle.closed:
            self._handle.flush()
            self._handle.close()

    def __enter__(self) -> "EvaluatorConsole":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
