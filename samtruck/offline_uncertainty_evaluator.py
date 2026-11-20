"""Post-run covariance consistency checks for passive SAMTRUCK estimates.

The evaluator consumes completed ground-truth and filtered-state CSV records
only.  It deliberately imports neither SUMO/TraCI nor the live Twin, sensors,
controller, or V2X channel.  Its exact duplicate/timestamp policy reuses the
established Phase 2.2A.1 offline evaluator helpers without changing them.

Position is intentionally excluded: the current estimator position is
unanchored while the available SUMO truth position is lane-local.  The scalar
speed and acceleration calculations below are covariance-consistency
diagnostics; they are not a claim of full-vector NEES calibration.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from statistics import mean
from typing import Any, Iterable

from .offline_estimator_evaluator import (
    TIMESTAMP_DECIMALS,
    TIMESTAMP_TOLERANCE_S,
    _finite_value,
    _index_records,
    _nearby_mismatch_count,
)


Z_95 = 1.959963984540054
MINIMUM_SAMPLES_FOR_INTERPRETATION = 30


def _empty_consistency_metrics() -> dict[str, int | float | bool | None | str]:
    return {
        "samples": 0,
        "mean_absolute_error": None,
        "root_mean_square_error": None,
        "mean_predicted_standard_deviation": None,
        "mean_absolute_standardized_error": None,
        "mean_normalized_squared_error": None,
        "one_sigma_coverage": None,
        "two_sigma_coverage": None,
        "ninety_five_percent_coverage": None,
        "absolute_error_vs_sigma_correlation": None,
        "sufficient_for_interpretation": False,
        "interpretation": "unavailable_no_valid_pairs",
    }


def _pearson_correlation(values_x: list[float], values_y: list[float]) -> float | None:
    """Return Pearson correlation, or None when a variable has no spread."""

    if len(values_x) < 2 or len(values_x) != len(values_y):
        return None
    mean_x, mean_y = mean(values_x), mean(values_y)
    deviations_x = [value - mean_x for value in values_x]
    deviations_y = [value - mean_y for value in values_y]
    denominator = math.sqrt(sum(value * value for value in deviations_x) * sum(value * value for value in deviations_y))
    if denominator <= 0.0:
        return None
    return sum(x * y for x, y in zip(deviations_x, deviations_y)) / denominator


def _consistency_metrics(pairs: list[tuple[float, float]]) -> dict[str, int | float | bool | None | str]:
    """Evaluate scalar errors against positive marginal variances.

    Each pair is ``(estimate - truth, variance)``.  With a calibrated scalar
    Gaussian estimator, nominal coverage is about 68.27% inside one sigma,
    95.45% inside two sigma, and 95% inside ±1.95996 sigma.  These values are
    references only; correlated time samples and simulator assumptions limit
    inference from a single run.
    """

    if not pairs:
        return _empty_consistency_metrics()
    errors = [pair[0] for pair in pairs]
    variances = [pair[1] for pair in pairs]
    sigmas = [math.sqrt(variance) for variance in variances]
    absolute_errors = [abs(error) for error in errors]
    standardized = [error / sigma for error, sigma in zip(errors, sigmas)]
    normalized_squared = [value * value for value in standardized]
    count = len(pairs)
    sufficient = count >= MINIMUM_SAMPLES_FOR_INTERPRETATION
    return {
        "samples": count,
        "mean_absolute_error": sum(absolute_errors) / count,
        "root_mean_square_error": math.sqrt(sum(error * error for error in errors) / count),
        "mean_predicted_standard_deviation": sum(sigmas) / count,
        "mean_absolute_standardized_error": sum(abs(value) for value in standardized) / count,
        "mean_normalized_squared_error": sum(normalized_squared) / count,
        "one_sigma_coverage": sum(abs(error) <= sigma for error, sigma in zip(errors, sigmas)) / count,
        "two_sigma_coverage": sum(abs(error) <= 2.0 * sigma for error, sigma in zip(errors, sigmas)) / count,
        "ninety_five_percent_coverage": sum(abs(error) <= Z_95 * sigma for error, sigma in zip(errors, sigmas)) / count,
        "absolute_error_vs_sigma_correlation": _pearson_correlation(absolute_errors, sigmas),
        "sufficient_for_interpretation": sufficient,
        "interpretation": "descriptive_only_insufficient_samples" if not sufficient else "descriptive_scalar_consistency_metric",
    }


def _component_group(record: dict[str, Any], component: str) -> str:
    """Group records by new freshness provenance without changing Phase 2.2A.1.

    Older CSV files without the additive provenance field are represented as
    unavailable rather than guessed from row-level status/mask.
    """

    value = str(record.get(f"{component}_estimate_provenance", "")).strip()
    if value == "measurement_updated":
        return "measurement_updated"
    if value == "prediction_only":
        return "prediction_only"
    return "other_or_unavailable"


def _age_group(record: dict[str, Any], component: str) -> str:
    age = _finite_value(record, f"{component}_measurement_age_s")
    if age is None:
        return "unavailable"
    return "fresh" if age == 0.0 else "aged"


def evaluate_uncertainty_records(
    truth_records: Iterable[dict[str, Any]],
    filtered_records: Iterable[dict[str, Any]],
    *,
    scenario_id: str | None = None,
    simulation_step_s: float = 0.1,
) -> dict[str, Any]:
    """Evaluate offline scalar uncertainty consistency with exact key matching.

    Invalid/non-positive variances are excluded from uncertainty metrics and
    reported.  They are not replaced by arbitrary values.  NIS is unavailable
    because innovations and innovation covariance are not persisted; full NEES
    is unavailable because coordinate-compatible position truth is absent.
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

    component_specs = {
        "speed_mps": ("speed", "speed_mps", "estimated_speed_mps", "speed_variance_m2ps2", "m/s", "m^2/s^2"),
        "acceleration_mps2": (
            "acceleration", "acceleration_mps2", "estimated_acceleration_mps2",
            "acceleration_variance_m2ps4", "m/s^2", "m^2/s^4",
        ),
    }
    results: dict[str, Any] = {}
    invalid_values = {"truth": 0, "estimate": 0, "variance": 0}
    for label, (component, truth_field, estimate_field, variance_field, units, variance_units) in component_specs.items():
        all_pairs: list[tuple[float, float]] = []
        grouped_pairs = {
            "measurement_updated": [], "prediction_only": [], "other_or_unavailable": [],
            "fresh": [], "aged": [], "unavailable_age": [],
        }
        for key in sorted(matched_keys):
            truth_row, filtered_row = truth[key], filtered[key]
            actual = _finite_value(truth_row, truth_field)
            estimate = _finite_value(filtered_row, estimate_field)
            variance = _finite_value(filtered_row, variance_field)
            if actual is None:
                invalid_values["truth"] += 1
                continue
            if estimate is None:
                invalid_values["estimate"] += 1
                continue
            if variance is None or variance <= 0.0:
                invalid_values["variance"] += 1
                continue
            pair = (estimate - actual, variance)
            all_pairs.append(pair)
            grouped_pairs[_component_group(filtered_row, component)].append(pair)
            age_group = _age_group(filtered_row, component)
            grouped_pairs["unavailable_age" if age_group == "unavailable" else age_group].append(pair)
        results[label] = {
            "units": units,
            "variance_units": variance_units,
            "all": _consistency_metrics(all_pairs),
            "by_estimate_provenance": {
                name: _consistency_metrics(pairs)
                for name, pairs in grouped_pairs.items()
                if name in {"measurement_updated", "prediction_only", "other_or_unavailable"}
            },
            "by_measurement_age": {
                name: _consistency_metrics(pairs)
                for name, pairs in grouped_pairs.items()
                if name in {"fresh", "aged", "unavailable_age"}
            },
        }

    return {
        "schema_version": "2.2B",
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
            "invalid_or_nonpositive_values": invalid_values,
        },
        "metrics": results,
        "statistical_availability": {
            "nis": {"available": False, "reason": "innovation and innovation covariance are not logged"},
            "full_state_nees": {
                "available": False,
                "reason": "position truth is lane-local and not coordinate-compatible with unanchored estimator position",
            },
            "scalar_speed_acceleration_normalized_squared_error": {
                "available": True,
                "definition": "(estimate - truth)^2 / marginal_variance for each valid scalar matched pair",
            },
        },
        "limitations": [
            "Coverage references assume calibrated approximately Gaussian scalar errors; sequential samples are correlated.",
            "This is offline evaluation only and does not feed a controller or live Twin.",
            "Position and radar uncertainty calibration are excluded because coordinate/identity-compatible truth is unavailable.",
        ],
    }


def evaluate_uncertainty_csv_files(
    truth_csv: str | Path,
    filtered_csv: str | Path,
    *,
    scenario_id: str | None = None,
    simulation_step_s: float = 0.1,
) -> dict[str, Any]:
    """Read completed CSV files and return JSON-serializable uncertainty results."""

    with Path(truth_csv).open(newline="", encoding="utf-8") as file:
        truth = list(csv.DictReader(file))
    with Path(filtered_csv).open(newline="", encoding="utf-8") as file:
        filtered = list(csv.DictReader(file))
    return evaluate_uncertainty_records(
        truth, filtered, scenario_id=scenario_id, simulation_step_s=simulation_step_s
    )
