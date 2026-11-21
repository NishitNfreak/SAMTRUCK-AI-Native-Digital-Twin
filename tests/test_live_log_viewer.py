from __future__ import annotations

import unittest

from samtruck.live_log_viewer import format_advisor, format_evaluator, format_executive


class LiveLogViewerTests(unittest.TestCase):
    def test_evaluator_output_preserves_truth_and_sensor_difference(self) -> None:
        line = format_evaluator({
            "simulation_time_s": 1.0, "vehicle_id": "truck_1", "truth_speed_mps": 10.0,
            "sensor_odometry_speed_mps": 11.0, "sensor_odometry_valid": True,
            "twin_filtered_speed_mps": 10.2, "twin_speed_variance": 0.16,
            "speed_measurement_age_s": 0.0, "speed_estimate_provenance": "measurement_updated",
            "predecessor_v2x_status": "fresh_propagated",
        })
        self.assertIn("truth=10.00", line)
        self.assertIn("sensor=11.00", line)
        self.assertIn("twin=10.20", line)

    def test_evaluator_marks_invalid_odometry_as_drop(self) -> None:
        line = format_evaluator({"simulation_time_s": 1.0, "vehicle_id": "truck_1", "sensor_odometry_valid": False})
        self.assertIn("sensor= DROP", line)

    def test_advisor_and_executive_are_compact(self) -> None:
        advisor = format_advisor({"simulation_time_s": 2.0, "vehicle_id": "truck_1", "severity": "high", "decision_code": "V2X_DEGRADED", "command_to_executive": "FALLBACK_ACC", "reasoning": ["V2X is stale."]})
        executive = format_executive({"simulation_time_s": 2.0, "vehicle_id": "truck_1", "command": "FALLBACK_ACC", "control_mode": "ACC", "applied_time_gap_s": 1.2, "v2v_enabled": False, "max_speed_cap_mps": None, "recovery_hold_active": False})
        self.assertIn("V2X_DEGRADED", advisor)
        self.assertIn("ctg=1.20s", executive)


if __name__ == "__main__":
    unittest.main()
