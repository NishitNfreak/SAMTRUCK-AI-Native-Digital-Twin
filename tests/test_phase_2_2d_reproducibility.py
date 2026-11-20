"""Focused tests for the Phase 2.2D opt-in reproducibility utilities."""

from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path

from inspect_twin_trace import collect_trace_records
from run_all_scenarios import _phase_2_2d_seed_policy, _sensor_measurement_rows
from samtruck.config import SimulationConfig
from samtruck.reproducibility import CSV_ARTIFACTS, JSON_ARTIFACTS, compare_result_directories
from samtruck.sensors import SensorInterface


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


class Phase22DReproducibilityTests(unittest.TestCase):
    def test_opt_in_seed_policy_resolves_and_leaves_legacy_absent(self) -> None:
        config = SimulationConfig()
        self.assertIsNone(_phase_2_2d_seed_policy({"seed": 9}, config))
        policy = _phase_2_2d_seed_policy(
            {"seed": 9, "twin": {"sensor_seed": 10, "v2x_seed": 11}, "phase_2_2d": {"sumo_seed": 12}}, config
        )
        self.assertEqual(policy, {
            "sumo_seed": 12, "scenario_seed": 9, "sensor_seed": 10,
            "twin_v2x_seed": 11, "legacy_cacc_v2x_seed": 9,
        })

    def test_opt_in_seed_policy_requires_non_negative_sumo_seed(self) -> None:
        with self.assertRaises(ValueError):
            _phase_2_2d_seed_policy({"phase_2_2d": {"sumo_seed": -1}}, SimulationConfig())
        with self.assertRaises(ValueError):
            _phase_2_2d_seed_policy({"phase_2_2d": {}}, SimulationConfig())

    def test_sensor_dto_rows_are_measurement_only(self) -> None:
        dto = SensorInterface(seed=3).make_input(
            timestamp_s=1.0, vehicle_id="truck_0", platoon_predecessor_id=None,
            position_m=7.0, speed_mps=8.0, acceleration_mps2=0.5,
            sensed_leader_id=None, radar_range_m=None, radar_relative_speed_mps=None,
        )
        row = _sensor_measurement_rows([dto])[0]
        self.assertEqual(row["odometry_speed_mps"], 8.0)
        self.assertEqual(row["radar_status"], "no_target")
        self.assertNotIn("truth", row)

    def _write_complete_artifacts(self, root: Path, *, speed: str = "1.0") -> None:
        root.mkdir()
        for filename in CSV_ARTIFACTS:
            write_csv(root / filename, [{"simulation_time_s": "0.1", "value": speed}])
        for filename in JSON_ARTIFACTS:
            (root / filename).write_text(json.dumps({"scenario_id": root.name, "value": speed}), encoding="utf-8")

    def test_result_comparison_ignores_scratch_labels_but_detects_first_csv_difference(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left, right = root / "a", root / "b"
            self._write_complete_artifacts(left)
            self._write_complete_artifacts(right)
            self.assertTrue(compare_result_directories(left, right)["matched"])
            write_csv(right / "twin_filtered_states.csv", [{"simulation_time_s": "0.1", "value": "DIFFERENT"}])
            comparison = compare_result_directories(left, right)
        self.assertFalse(comparison["matched"])
        self.assertEqual(comparison["first_mismatch"]["artifact"], "twin_filtered_states.csv")

    def test_trace_reports_absent_historical_sensor_dtos_as_unavailable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_csv(root / "twin_ground_truth.csv", [{
                "simulation_time_s": "0.1", "vehicle_id": "truck_0", "lane_position_m": "1",
                "speed_mps": "2", "acceleration_mps2": "3",
            }])
            write_csv(root / "twin_filtered_states.csv", [{
                "simulation_time_s": "0.1", "vehicle_id": "truck_0", "estimated_position_m": "0",
                "estimated_speed_mps": "2", "estimated_acceleration_mps2": "3",
            }])
            records, summary = collect_trace_records(root, "truck_0")
        self.assertEqual(len(records), 1)
        self.assertFalse(summary["sensor_dtos_persisted"])
        self.assertEqual(summary["sensor_dto_rows"], 0)


if __name__ == "__main__":
    unittest.main()
