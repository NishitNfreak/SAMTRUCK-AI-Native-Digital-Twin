"""Synthetic, post-run-only tests for the estimator accuracy evaluator."""

from __future__ import annotations

import ast
import inspect
import math
import unittest

import samtruck.offline_estimator_evaluator as evaluator
from samtruck.offline_estimator_evaluator import evaluate_records


def truth(time: object, vehicle: object = "truck_0", speed: object = 10.0, acceleration: object = 1.0) -> dict:
    return {
        "simulation_time_s": time, "vehicle_id": vehicle,
        "speed_mps": speed, "acceleration_mps2": acceleration,
    }


def filtered(
    time: object,
    vehicle: object = "truck_0",
    speed: object = 10.0,
    acceleration: object = 1.0,
    mask: str = "speed|acceleration",
    status: str = "updated",
) -> dict:
    return {
        "simulation_time_s": time, "vehicle_id": vehicle,
        "estimated_speed_mps": speed, "estimated_acceleration_mps2": acceleration,
        "measurement_update_mask": mask, "estimator_status": status,
    }


class OfflineEstimatorEvaluatorTests(unittest.TestCase):
    def test_metrics_include_mae_rmse_bias_max_and_p95(self) -> None:
        truth_rows = [truth(index) for index in range(4)]
        filtered_rows = [filtered(index, speed=10.0 + error, acceleration=1.0 - error) for index, error in enumerate((-1, 2, -3, 4))]
        result = evaluate_records(truth_rows, filtered_rows, scenario_id="math")
        speed = result["metrics"]["speed_mps"]["all"]
        self.assertEqual(speed["samples"], 4)
        self.assertAlmostEqual(speed["mae"], 2.5)
        self.assertAlmostEqual(speed["rmse"], math.sqrt(7.5))
        self.assertAlmostEqual(speed["bias"], 0.5)
        self.assertEqual(speed["maximum_absolute_error"], 4.0)
        self.assertAlmostEqual(speed["p95_absolute_error"], 3.85)
        self.assertEqual(result["scenario_id"], "math")

    def test_component_specific_update_and_prediction_classification(self) -> None:
        rows = [
            filtered(0.0, speed=11.0, acceleration=2.0, mask="speed|acceleration"),
            filtered(0.1, speed=12.0, acceleration=3.0, mask="speed", status="updated"),
            filtered(0.2, speed=13.0, acceleration=4.0, mask="acceleration", status="updated"),
            filtered(0.3, speed=14.0, acceleration=5.0, mask="", status="predicted_only"),
            filtered(0.4, speed=15.0, acceleration=6.0, mask="speed", status="partially_initialized"),
            filtered(0.5, speed=16.0, acceleration=7.0, mask="speed", status="updated_same_timestamp"),
            # An otherwise fully initialized filter returns this status with
            # no valid measurement update after an out-of-order timestamp.
            filtered(0.6, speed=17.0, acceleration=8.0, mask="", status="invalid_timestamp"),
        ]
        result = evaluate_records([truth(index / 10.0) for index in range(7)], rows)
        speed = result["metrics"]["speed_mps"]
        acceleration = result["metrics"]["acceleration_mps2"]
        self.assertEqual(speed["all"]["samples"], 7)
        self.assertEqual(acceleration["all"]["samples"], 7)
        # Rows 0/1/4/5 update speed; rows 2/3 are valid predictions.
        self.assertEqual(speed["measurement_updated"]["samples"], 4)
        self.assertEqual(speed["prediction_only"]["samples"], 2)
        # Rows 0/2 update acceleration; rows 1/3 are valid predictions.
        # Acceleration is not prediction-only in partial or same-time rows.
        self.assertEqual(acceleration["measurement_updated"]["samples"], 2)
        self.assertEqual(acceleration["prediction_only"]["samples"], 2)
        counts = result["component_classification_counts"]
        self.assertEqual(counts["speed_mps"], {"measurement_updated": 4, "prediction_only": 2, "other_no_measurement_update": 1})
        self.assertEqual(counts["acceleration_mps2"], {"measurement_updated": 2, "prediction_only": 2, "other_no_measurement_update": 3})
        # Existing full-mask row remains measurement-updated for both components.
        self.assertEqual(result["classification_counts"]["measurement_updated"], 5)

    def test_empty_dataset_returns_unavailable_metrics(self) -> None:
        result = evaluate_records([], [], scenario_id="empty")
        metrics = result["metrics"]["speed_mps"]["all"]
        self.assertEqual(metrics["samples"], 0)
        self.assertIsNone(metrics["mae"])
        self.assertFalse(result["position_error"]["available"])

    def test_duplicates_are_reported_and_not_paired(self) -> None:
        result = evaluate_records(
            [truth(0.0), truth(0.0)], [filtered(0.0)]
        )
        diagnostics = result["matching_diagnostics"]
        self.assertEqual(diagnostics["truth"]["duplicate_keys"], 1)
        self.assertEqual(diagnostics["truth"]["duplicate_records"], 2)
        self.assertEqual(diagnostics["matched_records"], 0)
        self.assertEqual(diagnostics["unmatched_truth_records"], 2)
        self.assertEqual(diagnostics["unmatched_filtered_records"], 1)

    def test_duplicate_filtered_records_are_reported_and_not_paired(self) -> None:
        result = evaluate_records(
            [truth(0.0)], [filtered(0.0), filtered(0.0)]
        )
        diagnostics = result["matching_diagnostics"]
        self.assertEqual(diagnostics["filtered"]["duplicate_keys"], 1)
        self.assertEqual(diagnostics["filtered"]["duplicate_records"], 2)
        self.assertEqual(diagnostics["matched_records"], 0)
        self.assertEqual(diagnostics["unmatched_truth_records"], 1)
        self.assertEqual(diagnostics["unmatched_filtered_records"], 2)

    def test_unmatched_timestamp_is_not_nearest_neighbour_paired(self) -> None:
        result = evaluate_records([truth(0.0)], [filtered(0.05)], simulation_step_s=0.1)
        diagnostics = result["matching_diagnostics"]
        self.assertEqual(diagnostics["matched_records"], 0)
        self.assertEqual(diagnostics["unmatched_truth_records"], 1)
        self.assertEqual(diagnostics["unmatched_filtered_records"], 1)
        self.assertEqual(diagnostics["nearby_unmatched_truth_with_filtered_candidate_count"], 1)

    def test_invalid_identifiers_timestamps_and_nonfinite_values_are_reported(self) -> None:
        result = evaluate_records(
            [truth("bad"), truth(1.0, vehicle=""), truth(2.0, speed=float("nan"))],
            [filtered(2.0, speed=float("inf"))],
        )
        diagnostics = result["matching_diagnostics"]
        self.assertEqual(diagnostics["truth"]["invalid_timestamp_or_vehicle_id"], 2)
        self.assertEqual(diagnostics["invalid_numeric_values"]["truth_speed_mps"], 1)
        self.assertEqual(diagnostics["invalid_numeric_values"]["filtered_speed_mps"], 0)
        # Invalid truth prevents comparison before estimate needs to be read.
        self.assertEqual(result["metrics"]["speed_mps"]["all"]["samples"], 0)

    def test_nonfinite_filtered_value_is_reported(self) -> None:
        result = evaluate_records([truth(0.0)], [filtered(0.0, speed=float("inf"))])
        self.assertEqual(result["matching_diagnostics"]["invalid_numeric_values"]["filtered_speed_mps"], 1)
        self.assertEqual(result["metrics"]["speed_mps"]["all"]["samples"], 0)

    def test_vehicle_identity_and_multiple_scenarios_stay_separate(self) -> None:
        result_a = evaluate_records([truth(0.0, "truck_a")], [filtered(0.0, "truck_a", speed=11.0)], scenario_id="a")
        result_b = evaluate_records([truth(0.0, "truck_b")], [filtered(0.0, "truck_b", speed=8.0)], scenario_id="b")
        mismatch = evaluate_records([truth(0.0, "truck_a")], [filtered(0.0, "truck_b")])
        self.assertEqual(result_a["scenario_id"], "a")
        self.assertEqual(result_b["scenario_id"], "b")
        self.assertEqual(result_a["metrics"]["speed_mps"]["all"]["bias"], 1.0)
        self.assertEqual(result_b["metrics"]["speed_mps"]["all"]["bias"], -2.0)
        self.assertEqual(mismatch["matching_diagnostics"]["matched_records"], 0)

    def test_evaluator_has_no_live_system_imports(self) -> None:
        source = inspect.getsource(evaluator)
        imports = []
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.Import):
                imports.extend(alias.name.lower() for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module.lower())
        for forbidden in ("traci", "sumolib", "controller", "twin_runtime", "sensors", "v2x"):
            self.assertFalse(any(name == forbidden or name.startswith(f"{forbidden}.") for name in imports))


if __name__ == "__main__":
    unittest.main()
