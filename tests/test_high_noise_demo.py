"""Focused offline tests for the Phase 2.2D high-noise comparison."""

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

from samtruck.high_noise_demo import calculate_speed_comparison, matched_speed_records, plot_speed_comparison


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


class HighNoiseDemoTests(unittest.TestCase):
    def test_exact_timestamp_matching_metrics_and_three_series_plot(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_csv(root / "truth.csv", [
                {"simulation_time_s": "0.1", "vehicle_id": "truck", "speed_mps": "10.0"},
                {"simulation_time_s": "0.2", "vehicle_id": "truck", "speed_mps": "10.0"},
            ])
            write_csv(root / "sensor.csv", [
                {"simulation_time_s": "0.1", "vehicle_id": "truck", "odometry_speed_mps": "11.0"},
                {"simulation_time_s": "0.2", "vehicle_id": "truck", "odometry_speed_mps": "9.0"},
            ])
            write_csv(root / "filtered.csv", [
                {"simulation_time_s": "0.1", "vehicle_id": "truck", "estimated_speed_mps": "10.5"},
                {"simulation_time_s": "0.2", "vehicle_id": "truck", "estimated_speed_mps": "9.5"},
            ])
            records, diagnostics = matched_speed_records(root / "truth.csv", root / "sensor.csv", root / "filtered.csv", "truck")
            comparison = calculate_speed_comparison(records, diagnostics)
            counts = plot_speed_comparison(records, root / "comparison.png", "truck")
            self.assertTrue((root / "comparison.png").exists())
        self.assertEqual(diagnostics["matched_samples"], 2)
        self.assertEqual(counts, {"truth_points": 2, "sensor_points": 2, "filtered_points": 2})
        self.assertAlmostEqual(comparison["raw_noisy_odometry_vs_sumo_truth"]["mae_mps"], 1.0)
        self.assertAlmostEqual(comparison["filtered_twin_vs_sumo_truth"]["mae_mps"], 0.5)
        self.assertAlmostEqual(comparison["improvement_filtered_over_raw"]["mae_percent"], 50.0)

    def test_unmatched_or_nonfinite_records_are_not_paired(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_csv(root / "truth.csv", [{"simulation_time_s": "0.1", "vehicle_id": "truck", "speed_mps": "10"}])
            write_csv(root / "sensor.csv", [{"simulation_time_s": "0.2", "vehicle_id": "truck", "odometry_speed_mps": "nan"}])
            write_csv(root / "filtered.csv", [{"simulation_time_s": "0.1", "vehicle_id": "truck", "estimated_speed_mps": "10"}])
            records, diagnostics = matched_speed_records(root / "truth.csv", root / "sensor.csv", root / "filtered.csv", "truck")
        self.assertEqual(records, [])
        self.assertEqual(diagnostics["matched_samples"], 0)


if __name__ == "__main__":
    unittest.main()
