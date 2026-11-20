"""Unit tests for passive evaluator-console formatting and logging."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from samtruck.evaluator_console import EvaluatorConsole


def snapshot(*, timestamp: float = 1.0, sensor_speed: float | None = 10.4, valid: bool = True) -> dict:
    return {
        "simulation_time_s": timestamp,
        "vehicle_id": "samtruck_follower_1",
        "truth_speed_mps": 10.0,
        "truth_acceleration_mps2": -0.2,
        "truth_gap_m": 16.5,
        "sensor_odometry_speed_mps": sensor_speed,
        "sensor_odometry_valid": valid,
        "sensor_odometry_status": "dropout" if not valid else "measured",
        "sensor_radar_range_m": None,
        "sensor_radar_valid": False,
        "sensor_radar_status": "dropout",
        "twin_filtered_speed_mps": 10.1,
        "twin_filtered_acceleration_mps2": -0.1,
        "twin_speed_variance": 0.04,
        "twin_accel_variance": 0.09,
        "speed_measurement_age_s": 0.1,
        "speed_estimate_provenance": "prediction_only" if not valid else "measurement_updated",
        "predecessor_v2x_status": "accepted",
        "predecessor_v2x_age_s": 0.05,
    }


class EvaluatorConsoleTests(unittest.TestCase):
    def test_formats_invalid_sensor_as_drop_without_crashing(self) -> None:
        output: list[str] = []
        with tempfile.TemporaryDirectory() as directory:
            with EvaluatorConsole(Path(directory) / "console.jsonl", period_s=1.0, printer=output.append) as console:
                console.maybe_print(1.0, [snapshot(sensor_speed=99.0, valid=False)])
        rendered = "\n".join(output)
        self.assertIn("DROP", rendered)
        self.assertIn("PRED", rendered)
        self.assertIn("SUMO offline reference", rendered)

    def test_jsonl_records_valid_json_and_preserves_truth_sensor_difference(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "console.jsonl"
            with EvaluatorConsole(path, printer=lambda _: None) as console:
                console.record([snapshot()])
            record = json.loads(path.read_text(encoding="utf-8").strip())
        self.assertEqual(record["vehicle_id"], "samtruck_follower_1")
        self.assertEqual(record["truth_speed_mps"], 10.0)
        self.assertEqual(record["sensor_odometry_speed_mps"], 10.4)
        self.assertNotEqual(record["truth_speed_mps"], record["sensor_odometry_speed_mps"])

    def test_period_suppresses_intermediate_blocks(self) -> None:
        output: list[str] = []
        with tempfile.TemporaryDirectory() as directory:
            with EvaluatorConsole(Path(directory) / "console.jsonl", period_s=1.0, printer=output.append) as console:
                self.assertTrue(console.should_print(0.1))
                self.assertFalse(console.should_print(0.9))
                self.assertTrue(console.should_print(1.1))

    def test_invalid_period_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                EvaluatorConsole(Path(directory) / "console.jsonl", period_s=0.0)


if __name__ == "__main__":
    unittest.main()
