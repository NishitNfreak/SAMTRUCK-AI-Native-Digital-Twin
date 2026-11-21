"""Human-readable renderers for SAMTRUCK's append-only live JSONL logs.

These functions are intentionally read-only.  They make demonstration traces
readable without changing the simulation, Twin, advisor, or executive.
"""
from __future__ import annotations

import math
from typing import Any


def _number(value: object, digits: int = 2) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "NA"
    return f"{number:.{digits}f}" if math.isfinite(number) else "NA"


def format_evaluator(record: dict[str, Any]) -> str:
    """Render one Twin-side measurement/estimate row in one terminal line."""
    sensor = _number(record.get("sensor_odometry_speed_mps")) if record.get("sensor_odometry_valid") else "DROP"
    variance = record.get("twin_speed_variance")
    sigma = "NA"
    try:
        sigma = _number(math.sqrt(max(0.0, float(variance))))
    except (TypeError, ValueError):
        pass
    return (
        f"t={_number(record.get('simulation_time_s'), 1):>5}s "
        f"{str(record.get('vehicle_id', 'NA')):<22} "
        f"truth={_number(record.get('truth_speed_mps')):>5} "
        f"sensor={sensor:>5} twin={_number(record.get('twin_filtered_speed_mps')):>5} "
        f"sigma={sigma:>5} age={_number(record.get('speed_measurement_age_s'), 1):>4} "
        f"state={str(record.get('speed_estimate_provenance') or 'NA')} "
        f"v2x={str(record.get('predecessor_v2x_status') or 'NA')}"
    )


def format_advisor(record: dict[str, Any]) -> str:
    """Render the decision and its first factual reason without JSON noise."""
    reasoning = record.get("reasoning") or []
    reason = str(reasoning[0]) if reasoning else "No rule explanation recorded."
    return (
        f"t={_number(record.get('simulation_time_s'), 1):>5}s "
        f"{str(record.get('vehicle_id', 'NA')):<22} "
        f"{str(record.get('severity', 'NA')).upper():<8} "
        f"decision={str(record.get('decision_code', 'NA')):<32} "
        f"command={str(record.get('command_to_executive', 'NA'))} | {reason}"
    )


def format_executive(record: dict[str, Any]) -> str:
    """Render one bounded executive action compactly."""
    cap = _number(record.get("max_speed_cap_mps")) if record.get("max_speed_cap_mps") is not None else "none"
    return (
        f"t={_number(record.get('simulation_time_s'), 1):>5}s "
        f"{str(record.get('vehicle_id', 'NA')):<22} "
        f"command={str(record.get('command', 'NA')):<27} "
        f"mode={str(record.get('control_mode', 'NA')):<14} "
        f"ctg={_number(record.get('applied_time_gap_s'))}s "
        f"v2v={record.get('v2v_enabled')} cap={cap} "
        f"hold={record.get('recovery_hold_active')}"
    )
