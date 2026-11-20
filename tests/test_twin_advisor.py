"""Unit tests for the passive, non-actuating Twin decision advisor."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from samtruck.twin_advisor import AdvisorDecisionConsole, AdvisorInput, TwinAdvisor


def healthy_input(**changes: object) -> AdvisorInput:
    values: dict[str, object] = {
        "simulation_time_s": 1.0,
        "vehicle_id": "truck_1",
        "twin_filtered_speed_mps": 10.0,
        "twin_filtered_acceleration_mps2": 0.0,
        "twin_speed_variance": 0.04,
        "twin_accel_variance": 0.09,
        "speed_measurement_age_s": 0.0,
        "speed_estimate_provenance": "measurement_updated",
        "sensor_odometry_valid": True,
        "sensor_odometry_status": "valid",
        "sensor_radar_valid": True,
        "sensor_radar_status": "valid",
        "sensor_radar_range_m": 15.0,
        "predecessor_v2x_status": "delivered",
        "predecessor_v2x_age_s": 0.05,
        "has_platoon_predecessor": True,
    }
    values.update(changes)
    return AdvisorInput(**values)  # type: ignore[arg-type]


class TwinAdvisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.advisor = TwinAdvisor()

    def test_healthy_input_maps_to_baseline_command(self) -> None:
        decision = self.advisor.evaluate(healthy_input())
        self.assertEqual("HOLD_NORMAL_CACC", decision.decision_code)
        self.assertEqual("HOLD_BASELINE", decision.command_to_executive)
        self.assertEqual("R1_HEALTHY", decision.rule_id)

    def test_invalid_odometry_is_prediction_only_degraded(self) -> None:
        decision = self.advisor.evaluate(healthy_input(
            sensor_odometry_valid=False, sensor_odometry_status="scheduled_dropout",
            speed_estimate_provenance="prediction_only", speed_measurement_age_s=1.1,
        ))
        self.assertEqual("PREDICTION_ONLY_DEGRADED", decision.decision_code)
        self.assertEqual("INCREASE_GAP_1_5S", decision.command_to_executive)

    def test_v2x_outage_is_distinct_from_own_measurement_health(self) -> None:
        decision = self.advisor.evaluate(healthy_input(predecessor_v2x_status="outage", predecessor_v2x_age_s=0.1))
        self.assertEqual("V2X_DEGRADED", decision.decision_code)
        self.assertEqual("FALLBACK_ACC", decision.command_to_executive)

    def test_sensor_and_v2x_degradation_has_combined_rule(self) -> None:
        decision = self.advisor.evaluate(healthy_input(
            sensor_odometry_valid=False, sensor_odometry_status="scheduled_dropout",
            speed_estimate_provenance="prediction_only", speed_measurement_age_s=1.2,
            predecessor_v2x_status="outage", predecessor_v2x_age_s=0.7,
        ))
        self.assertEqual("SENSOR_AND_V2X_DEGRADED", decision.decision_code)
        self.assertEqual("high", decision.severity)

    def test_critical_rule_and_no_actuation(self) -> None:
        decision = self.advisor.evaluate(healthy_input(
            twin_speed_variance=0.64, sensor_odometry_valid=False,
            speed_estimate_provenance="prediction_only", speed_measurement_age_s=3.1,
            predecessor_v2x_status="outage", predecessor_v2x_age_s=2.1,
        ))
        self.assertEqual("CRITICAL_AWARENESS", decision.decision_code)
        self.assertEqual("critical", decision.severity)
        self.assertEqual("EMERGENCY_CAUTION", decision.command_to_executive)

    def test_leader_has_no_v2x_requirement(self) -> None:
        decision = self.advisor.evaluate(healthy_input(
            vehicle_id="truck_0", has_platoon_predecessor=False,
            predecessor_v2x_status=None, predecessor_v2x_age_s=None,
        ))
        self.assertEqual("HOLD_NORMAL_CACC", decision.decision_code)


class AdvisorDecisionConsoleTests(unittest.TestCase):
    def test_jsonl_is_valid_and_contains_executive_recommendation(self) -> None:
        decision = TwinAdvisor().evaluate(healthy_input())
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "advisor.jsonl"
            console = AdvisorDecisionConsole(path, printer=lambda _: None)
            console.record([decision])
            console.close()
            record = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual("HOLD_BASELINE", record["command_to_executive"])
        self.assertEqual("truck_1", record["vehicle_id"])
        self.assertIn("metrics_used", record)

    def test_print_period_suppresses_intermediate_blocks(self) -> None:
        output: list[str] = []
        decision = TwinAdvisor().evaluate(healthy_input())
        with tempfile.TemporaryDirectory() as temp_dir:
            console = AdvisorDecisionConsole(Path(temp_dir) / "advisor.jsonl", period_s=1.0, printer=output.append)
            console.maybe_print(0.0, [decision])
            first_count = len(output)
            console.maybe_print(0.5, [decision])
            self.assertEqual(first_count, len(output))
            console.maybe_print(1.0, [decision])
            console.close()
        self.assertGreater(len(output), first_count)
        self.assertTrue(any("EXECUTIVE OPTIONAL" in line for line in output))

    def test_advisor_module_has_no_truth_or_actuation_dependency(self) -> None:
        source = Path(__file__).parents[1] / "samtruck" / "twin_advisor.py"
        text = source.read_text(encoding="utf-8")
        self.assertNotIn("import traci", text)
        self.assertNotIn("setSpeed", text)
        self.assertNotIn("CACCController", text)


if __name__ == "__main__":
    unittest.main()
