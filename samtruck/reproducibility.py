"""Offline comparison helpers for the opt-in Phase 2.2D reproducibility gate.

These functions read completed artifacts only.  They do not import TraCI,
construct a Twin, or alter any result file.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any


CSV_ARTIFACTS = (
    "sumo_vehicle_lifecycle.csv",
    "twin_ground_truth.csv",
    "twin_sensor_measurements.csv",
    "twin_v2x_messages.csv",
    "twin_filtered_states.csv",
    "twin_radar_tracks.csv",
)
JSON_ARTIFACTS = (
    "twin_degradation_summary.json",
    "twin_filtered_evaluation.json",
    "twin_uncertainty_evaluation.json",
)


def _read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or ()), list(reader)


def _normalise_json(value: Any) -> Any:
    """Remove only per-run labels that intentionally differ between scratch runs."""

    if isinstance(value, dict):
        return {
            key: _normalise_json(item)
            for key, item in value.items()
            if key not in {"scenario_id"}
        }
    if isinstance(value, list):
        return [_normalise_json(item) for item in value]
    return value


def _first_sequence_mismatch(left: list[Any], right: list[Any]) -> dict[str, Any] | None:
    for index, (left_item, right_item) in enumerate(zip(left, right)):
        if left_item != right_item:
            return {"row_or_item_index": index, "left": left_item, "right": right_item}
    if len(left) != len(right):
        return {
            "row_or_item_index": min(len(left), len(right)),
            "left": None if len(left) <= len(right) else left[len(right)],
            "right": None if len(right) <= len(left) else right[len(left)],
            "reason": "length_mismatch",
        }
    return None


def compare_result_directories(left_dir: Path, right_dir: Path) -> dict[str, Any]:
    """Compare reproducibility-gate artifacts and return the first mismatch.

    CSV values are compared exactly because both gate runs use an identical
    explicit manifest.  JSON scenario IDs are intentionally ignored: scratch
    output directories require distinct IDs, while those labels cannot affect
    physical or Twin behavior.
    """

    result: dict[str, Any] = {"matched": True, "checked": [], "first_mismatch": None}
    for filename in CSV_ARTIFACTS:
        left_path, right_path = left_dir / filename, right_dir / filename
        if not left_path.exists() or not right_path.exists():
            result.update({
                "matched": False,
                "first_mismatch": {
                    "artifact": filename,
                    "reason": "missing_artifact",
                    "left_exists": left_path.exists(),
                    "right_exists": right_path.exists(),
                },
            })
            return result
        left_headers, left_rows = _read_csv(left_path)
        right_headers, right_rows = _read_csv(right_path)
        if left_headers != right_headers:
            result.update({
                "matched": False,
                "first_mismatch": {"artifact": filename, "reason": "header_mismatch", "left": left_headers, "right": right_headers},
            })
            return result
        mismatch = _first_sequence_mismatch(left_rows, right_rows)
        if mismatch is not None:
            mismatch["artifact"] = filename
            result.update({"matched": False, "first_mismatch": mismatch})
            return result
        result["checked"].append(filename)

    for filename in JSON_ARTIFACTS:
        left_path, right_path = left_dir / filename, right_dir / filename
        if not left_path.exists() or not right_path.exists():
            result.update({
                "matched": False,
                "first_mismatch": {
                    "artifact": filename,
                    "reason": "missing_artifact",
                    "left_exists": left_path.exists(),
                    "right_exists": right_path.exists(),
                },
            })
            return result
        try:
            left = _normalise_json(json.loads(left_path.read_text(encoding="utf-8")))
            right = _normalise_json(json.loads(right_path.read_text(encoding="utf-8")))
        except json.JSONDecodeError as exc:
            result.update({"matched": False, "first_mismatch": {"artifact": filename, "reason": "invalid_json", "detail": str(exc)}})
            return result
        if left != right:
            result.update({
                "matched": False,
                "first_mismatch": {"artifact": filename, "reason": "json_value_mismatch", "left": left, "right": right},
            })
            return result
        result["checked"].append(filename)
    return result
