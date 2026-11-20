"""Deterministic Phase 2.2B tests for post-run uncertainty evaluation."""

from __future__ import annotations

import ast
import inspect
import unittest

import samtruck.offline_uncertainty_evaluator as uncertainty
from samtruck.offline_uncertainty_evaluator import evaluate_uncertainty_records


def truth(time: float, vehicle: str = "truck_0", speed: float = 10.0, acceleration: float = 0.0) -> dict[str, object]:
    return {
        "simulation_time_s": time,
        "vehicle_id": vehicle,
        "speed_mps": speed,
        "acceleration_mps2": acceleration,
    }


def filtered(
    time: float,
    vehicle: str = "truck_0",
    speed: float = 10.0,
    acceleration: float = 0.0,
    speed_variance: float = 1.0,
    acceleration_variance: float = 1.0,
    speed_provenance: str = "measurement_updated",
    acceleration_provenance: str = "measurement_updated",
    speed_age: float | None = 0.0,
    acceleration_age: float | None = 0.0,
) -> dict[str, object]:
    return {
        "simulation_time_s": time,
        "vehicle_id": vehicle,
        "estimated_speed_mps": speed,
        "estimated_acceleration_mps2": acceleration,
        "speed_variance_m2ps2": speed_variance,
        "acceleration_variance_m2ps4": acceleration_variance,
        "speed_estimate_provenance": speed_provenance,
        "acceleration_estimate_provenance": acceleration_provenance,
        "speed_measurement_age_s": speed_age,
        "acceleration_measurement_age_s": acceleration_age,
    }


class OfflineUncertaintyEvaluatorTests(unittest.TestCase):
    def test_scalar_error_uncertainty_metrics_and_coverage(self) -> None:
        truths = [truth(0.0, speed=10.0), truth(1.0, speed=10.0), truth(2.0, speed=10.0)]
        estimates = [
            filtered(0.0, speed=10.0, speed_variance=1.0),
            filtered(1.0, speed=12.0, speed_variance=4.0),
            filtered(2.0, speed=13.0, speed_variance=9.0),
        ]
        result = evaluate_uncertainty_records(truths, estimates)
        metrics = result["metrics"]["speed_mps"]["all"]
        self.assertEqual(metrics["samples"], 3)
        self.assertAlmostEqual(metrics["mean_absolute_error"], 5.0 / 3.0)
        self.assertAlmostEqual(metrics["root_mean_square_error"], (13.0 / 3.0) ** 0.5)
        self.assertAlmostEqual(metrics["mean_normalized_squared_error"], 2.0 / 3.0)
        self.assertEqual(metrics["one_sigma_coverage"], 1.0)
        self.assertEqual(metrics["two_sigma_coverage"], 1.0)
        self.assertEqual(metrics["ninety_five_percent_coverage"], 1.0)
        self.assertAlmostEqual(metrics["absolute_error_vs_sigma_correlation"], 0.9819805060619656)

    def test_prediction_only_and_measurement_recovery_are_grouped_by_provenance_and_age(self) -> None:
        result = evaluate_uncertainty_records(
            [truth(0.0), truth(1.0), truth(2.0)],
            [
                filtered(0.0, speed_provenance="measurement_updated", speed_age=0.0),
                filtered(1.0, speed=11.0, speed_variance=4.0, speed_provenance="prediction_only", speed_age=1.0),
                filtered(2.0, speed_provenance="measurement_updated", speed_age=0.0),
            ],
        )
        speed = result["metrics"]["speed_mps"]
        self.assertEqual(speed["by_estimate_provenance"]["measurement_updated"]["samples"], 2)
        self.assertEqual(speed["by_estimate_provenance"]["prediction_only"]["samples"], 1)
        self.assertEqual(speed["by_measurement_age"]["fresh"]["samples"], 2)
        self.assertEqual(speed["by_measurement_age"]["aged"]["samples"], 1)

    def test_invalid_truth_estimate_and_variance_are_excluded_not_fabricated(self) -> None:
        result = evaluate_uncertainty_records(
            [truth(0.0), truth(1.0, speed=float("nan")), truth(2.0)],
            [
                filtered(0.0, speed_variance=0.0),
                filtered(1.0),
                filtered(2.0, speed=float("inf")),
            ],
        )
        speed = result["metrics"]["speed_mps"]["all"]
        self.assertEqual(speed["samples"], 0)
        self.assertIsNone(speed["mean_normalized_squared_error"])
        diagnostics = result["matching_diagnostics"]["invalid_or_nonpositive_values"]
        self.assertEqual(diagnostics["variance"], 1)
        self.assertEqual(diagnostics["truth"], 1)
        self.assertEqual(diagnostics["estimate"], 1)

    def test_duplicate_and_timestamp_mismatch_records_are_not_paired(self) -> None:
        result = evaluate_uncertainty_records(
            [truth(0.0), truth(0.0), truth(1.0)],
            [filtered(0.0), filtered(1.05)],
            simulation_step_s=0.1,
        )
        self.assertEqual(result["matching_diagnostics"]["matched_records"], 0)
        self.assertEqual(result["matching_diagnostics"]["truth"]["duplicate_keys"], 1)
        self.assertEqual(result["matching_diagnostics"]["unmatched_truth_records"], 3)
        self.assertEqual(result["matching_diagnostics"]["unmatched_filtered_records"], 2)

    def test_insufficient_sample_and_statistical_availability_are_explicit(self) -> None:
        result = evaluate_uncertainty_records([truth(0.0)], [filtered(0.0)])
        metrics = result["metrics"]["speed_mps"]["all"]
        self.assertFalse(metrics["sufficient_for_interpretation"])
        self.assertEqual(metrics["interpretation"], "descriptive_only_insufficient_samples")
        self.assertFalse(result["statistical_availability"]["nis"]["available"])
        self.assertFalse(result["statistical_availability"]["full_state_nees"]["available"])
        self.assertTrue(result["statistical_availability"]["scalar_speed_acceleration_normalized_squared_error"]["available"])

    def test_uncertainty_evaluator_has_no_live_system_imports(self) -> None:
        source = inspect.getsource(uncertainty)
        imports = []
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.Import):
                imports.extend(alias.name.lower() for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module.lower())
        for forbidden in ("traci", "sumolib", "twin_runtime", "controller", "sensors", "v2x"):
            self.assertFalse(any(name == forbidden or name.startswith(f"{forbidden}.") for name in imports))


if __name__ == "__main__":
    unittest.main()
