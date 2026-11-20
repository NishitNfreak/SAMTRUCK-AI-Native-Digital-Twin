"""Post-run, file-based accuracy evaluation for passive Twin estimates.

This module never imports TraCI, SUMO, controllers, Twin runtime, sensors, or
V2X delivery code.  It evaluates completed CSV records only and returns JSON-
serializable diagnostics.  The matching policy is exact after normalizing
timestamps to six decimal places (one microsecond); nearby rows are reported
as mismatches, never paired by nearest-neighbour matching.
"""

from __future__ import annotations

import csv
import math
from collections import defaultdict
from pathlib import Path
from statistics import quantiles
from typing import Any, Iterable


TIMESTAMP_DECIMALS = 6
TIMESTAMP_TOLERANCE_S = 10 ** -TIMESTAMP_DECIMALS
PREDICTION_ONLY_STATUSES = {"predicted_only", "degraded_long_gap_predicted"}
NON_ESTIMATE_STATUSES = {"uninitialized_missing_speed", "invalid_timestamp"}
# In a partially initialized state, the absent acceleration component is a
# zero-valued prior with broad uncertainty, not a propagated estimate.  With
# zero elapsed time, an updated_same_timestamp row only updates its mask token;
# an omitted component has not been propagated in that row either.
NON_PROPAGATED_MISSING_COMPONENT_STATUSES = {"partially_initialized", "updated_same_timestamp"}


def _empty_metrics() -> dict[str, int | float | None]:
    return {
        "samples": 0,
        "mae": None,
        "rmse": None,
        "bias": None,
        "maximum_absolute_error": None,
        "p95_absolute_error": None,
    }


def _metrics(errors: list[float]) -> dict[str, int | float | None]:
    if not errors:
        return _empty_metrics()
    absolute = sorted(abs(error) for error in errors)
    if len(absolute) == 1:
        p95 = absolute[0]
    else:
        # Inclusive interpolation makes percentile behavior deterministic for
        # small evaluation samples and needs no external dependency.
        p95 = quantiles(absolute, n=100, method="inclusive")[94]
    return {
        "samples": len(errors),
        "mae": sum(absolute) / len(absolute),
        "rmse": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "bias": sum(errors) / len(errors),
        "maximum_absolute_error": absolute[-1],
        "p95_absolute_error": p95,
    }


def _parse_timestamp(record: dict[str, Any]) -> float | None:
    try:
        value = float(record.get("simulation_time_s", ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _key(record: dict[str, Any]) -> tuple[str, float] | None:
    timestamp = _parse_timestamp(record)
    vehicle_id = str(record.get("vehicle_id", "")).strip()
    if timestamp is None or not vehicle_id:
        return None
    return vehicle_id, round(timestamp, TIMESTAMP_DECIMALS)


def _finite_value(record: dict[str, Any], field: str) -> float | None:
    try:
        value = float(record.get(field, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _measurement_components(record: dict[str, Any]) -> set[str]:
    """Parse the existing pipe-delimited update mask without inventing tokens."""

    mask = str(record.get("measurement_update_mask", "")).strip()
    return {token.strip() for token in mask.split("|") if token.strip()}


def _classify_filtered_row(record: dict[str, Any]) -> str:
    """Retain the prior row-level summary for backward-compatible diagnostics."""

    if _measurement_components(record):
        return "measurement_updated"
    if str(record.get("estimator_status", "")).strip() in PREDICTION_ONLY_STATUSES:
        return "prediction_only"
    return "other_no_measurement_update"


def _classify_component(record: dict[str, Any], component: str) -> str:
    """Classify one estimate component, not the whole row.

    ``VehicleStateEstimator`` emits `speed` and `acceleration` tokens.  A
    component omitted from a valid initialized/updated row was propagated by
    the filter rather than measurement-updated, even if another component was
    updated in that same row.
    """

    status = str(record.get("estimator_status", "")).strip()
    if status in NON_ESTIMATE_STATUSES:
        return "other_no_measurement_update"
    if component in _measurement_components(record):
        return "measurement_updated"
    if status in NON_PROPAGATED_MISSING_COMPONENT_STATUSES:
        return "other_no_measurement_update"
    return "prediction_only"


def _index_records(records: Iterable[dict[str, Any]]) -> tuple[dict[tuple[str, float], dict[str, Any]], dict[tuple[str, float], list[dict[str, Any]]], dict[str, int]]:
    grouped: dict[tuple[str, float], list[dict[str, Any]]] = defaultdict(list)
    diagnostics = {"records": 0, "invalid_timestamp_or_vehicle_id": 0}
    for record in records:
        diagnostics["records"] += 1
        key = _key(record)
        if key is None:
            diagnostics["invalid_timestamp_or_vehicle_id"] += 1
            continue
        grouped[key].append(record)
    duplicates = {key: group for key, group in grouped.items() if len(group) > 1}
    # Duplicate keys are intentionally excluded so no arbitrary overwrite/pair
    # can be mistaken for a valid accuracy sample.
    unique = {key: group[0] for key, group in grouped.items() if len(group) == 1}
    diagnostics["duplicate_keys"] = len(duplicates)
    diagnostics["duplicate_records"] = sum(len(group) for group in duplicates.values())
    diagnostics["unique_eligible_records"] = len(unique)
    return unique, duplicates, diagnostics


def _nearby_mismatch_count(
    truth_keys: Iterable[tuple[str, float]], filtered_keys: Iterable[tuple[str, float]], simulation_step_s: float
) -> int:
    """Count unpaired same-vehicle rows within one step; do not pair them."""

    by_vehicle: dict[str, list[float]] = defaultdict(list)
    for vehicle_id, timestamp in filtered_keys:
        by_vehicle[vehicle_id].append(timestamp)
    count = 0
    for vehicle_id, timestamp in truth_keys:
        if any(abs(timestamp - candidate) <= simulation_step_s for candidate in by_vehicle.get(vehicle_id, ())):
            count += 1
    return count


def evaluate_records(
    truth_records: Iterable[dict[str, Any]],
    filtered_records: Iterable[dict[str, Any]],
    *,
    scenario_id: str | None = None,
    simulation_step_s: float = 0.1,
) -> dict[str, Any]:
    """Evaluate completed records with exact normalized timestamp matching.

    ``simulation_step_s`` is diagnostic-only: it bounds reporting of nearby
    unpaired timestamps.  It is never used to create a nearest-neighbour pair.
    """

    if not math.isfinite(simulation_step_s) or simulation_step_s <= 0.0:
        raise ValueError("simulation_step_s must be finite and positive.")
    if TIMESTAMP_TOLERANCE_S > simulation_step_s:
        raise ValueError("Timestamp normalization tolerance must not exceed the simulation step.")

    truth, truth_duplicates, truth_diagnostics = _index_records(truth_records)
    filtered, filtered_duplicates, filtered_diagnostics = _index_records(filtered_records)
    truth_keys, filtered_keys = set(truth), set(filtered)
    matched_keys = truth_keys & filtered_keys
    unmatched_truth = truth_keys - filtered_keys
    unmatched_filtered = filtered_keys - truth_keys

    errors = {
        "speed_mps": {"all": [], "measurement_updated": [], "prediction_only": []},
        "acceleration_mps2": {"all": [], "measurement_updated": [], "prediction_only": []},
    }
    invalid_values = {
        "truth_speed_mps": 0, "filtered_speed_mps": 0,
        "truth_acceleration_mps2": 0, "filtered_acceleration_mps2": 0,
    }
    row_classifications = {"measurement_updated": 0, "prediction_only": 0, "other_no_measurement_update": 0}
    component_classifications = {
        "speed_mps": {"measurement_updated": 0, "prediction_only": 0, "other_no_measurement_update": 0},
        "acceleration_mps2": {"measurement_updated": 0, "prediction_only": 0, "other_no_measurement_update": 0},
    }
    for key in sorted(matched_keys):
        truth_row, filtered_row = truth[key], filtered[key]
        row_group = _classify_filtered_row(filtered_row)
        row_classifications[row_group] += 1
        for label, component, truth_field, filtered_field in (
            ("speed_mps", "speed", "speed_mps", "estimated_speed_mps"),
            ("acceleration_mps2", "acceleration", "acceleration_mps2", "estimated_acceleration_mps2"),
        ):
            actual = _finite_value(truth_row, truth_field)
            estimate = _finite_value(filtered_row, filtered_field)
            if actual is None:
                invalid_values[f"truth_{label}"] += 1
                continue
            if estimate is None:
                invalid_values[f"filtered_{label}"] += 1
                continue
            error = estimate - actual
            errors[label]["all"].append(error)
            component_group = _classify_component(filtered_row, component)
            component_classifications[label][component_group] += 1
            if component_group in ("measurement_updated", "prediction_only"):
                errors[label][component_group].append(error)

    return {
        "schema_version": "2.2A.1",
        "scenario_id": scenario_id,
        "matching_policy": {
            "key": ["vehicle_id", "simulation_time_s_normalized"],
            "timestamp_decimals": TIMESTAMP_DECIMALS,
            "timestamp_tolerance_s": TIMESTAMP_TOLERANCE_S,
            "simulation_step_s_for_diagnostics": simulation_step_s,
            "nearest_neighbour_matching": False,
            "duplicate_key_policy": "exclude_duplicate_keys_from_matching",
        },
        "matching_diagnostics": {
            "truth": truth_diagnostics,
            "filtered": filtered_diagnostics,
            "matched_records": len(matched_keys),
            "unmatched_truth_records": len(unmatched_truth) + sum(len(group) for group in truth_duplicates.values()),
            "unmatched_filtered_records": len(unmatched_filtered) + sum(len(group) for group in filtered_duplicates.values()),
            "nearby_unmatched_truth_with_filtered_candidate_count": _nearby_mismatch_count(
                unmatched_truth, unmatched_filtered, simulation_step_s
            ),
            "invalid_numeric_values": invalid_values,
        },
        "classification_counts": row_classifications,
        "component_classification_counts": component_classifications,
        "metrics": {
            "speed_mps": {
                "units": "m/s",
                "all": _metrics(errors["speed_mps"]["all"]),
                "measurement_updated": _metrics(errors["speed_mps"]["measurement_updated"]),
                "prediction_only": _metrics(errors["speed_mps"]["prediction_only"]),
            },
            "acceleration_mps2": {
                "units": "m/s^2",
                "all": _metrics(errors["acceleration_mps2"]["all"]),
                "measurement_updated": _metrics(errors["acceleration_mps2"]["measurement_updated"]),
                "prediction_only": _metrics(errors["acceleration_mps2"]["prediction_only"]),
            },
        },
        # Keep prior top-level scalar summaries available to existing readers.
        "position_error": {"available": False, "reason": "lane-local ground truth is not coordinate-compatible"},
        "speed_error_mps": _metrics(errors["speed_mps"]["all"]),
        "acceleration_error_mps2": _metrics(errors["acceleration_mps2"]["all"]),
        "limitations": [
            "Position accuracy is unavailable: filtered position is unanchored while truth position is lane-local.",
            "Radar accuracy is outside this evaluator because target-identity-compatible truth and relative-speed truth are not logged.",
            "Component metrics classify speed and acceleration independently from their exact update-mask tokens.",
            "Rows with no measurement update but an uninitialized/invalid status are counted but excluded from component update/prediction metric groups.",
        ],
    }


def evaluate_csv_files(
    truth_csv: str | Path,
    filtered_csv: str | Path,
    *,
    scenario_id: str | None = None,
    simulation_step_s: float = 0.1,
) -> dict[str, Any]:
    """Read completed CSV files and return a JSON-serializable evaluation."""

    with Path(truth_csv).open(newline="", encoding="utf-8") as file:
        truth = list(csv.DictReader(file))
    with Path(filtered_csv).open(newline="", encoding="utf-8") as file:
        filtered = list(csv.DictReader(file))
    return evaluate_records(truth, filtered, scenario_id=scenario_id, simulation_step_s=simulation_step_s)
