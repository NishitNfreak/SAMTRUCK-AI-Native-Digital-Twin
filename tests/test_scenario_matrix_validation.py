"""
Unit tests for Scenario Matrix D1/D2 Statistical Validation Engine.
Tests confidence interval computation, matrix aggregation, and artifact generation.
"""

import unittest
import tempfile
import shutil
from pathlib import Path

from run_scenario_matrix_validation import (
    compute_mean_std_ci95,
    ScenarioMatrixRunner,
    SCENARIO_CATALOG,
    BASE_SEEDS,
)


class TestScenarioMatrixValidation(unittest.TestCase):

    def test_confidence_interval_calculation(self):
        """Test Student's t 95% confidence interval computation on sample data."""
        # Known dataset with mean = 10.0, std = 2.0, N = 10
        data = [10.0, 12.0, 8.0, 11.0, 9.0, 10.5, 9.5, 11.5, 8.5, 10.0]
        stats = compute_mean_std_ci95(data)
        self.assertEqual(stats["n"], 10)
        self.assertAlmostEqual(stats["mean"], 10.0, places=1)
        self.assertGreater(stats["std"], 0.0)
        self.assertGreater(stats["ci95_half"], 0.0)
        self.assertLess(stats["ci95_lower"], stats["mean"])
        self.assertGreater(stats["ci95_upper"], stats["mean"])

    def test_single_element_confidence_interval(self):
        """Test confidence interval fallback when N=1."""
        stats = compute_mean_std_ci95([42.0])
        self.assertEqual(stats["n"], 1)
        self.assertEqual(stats["mean"], 42.0)
        self.assertEqual(stats["std"], 0.0)
        self.assertEqual(stats["ci95_half"], 0.0)

    def test_dry_run_matrix_execution(self):
        """Test end-to-end dry-run execution of the 80-run matrix."""
        temp_dir = Path(tempfile.mkdtemp(prefix="samtruck_matrix_test_"))
        try:
            runner = ScenarioMatrixRunner(output_base_dir=temp_dir, dry_run=True, fast_demo=True)
            summary = runner.execute_matrix()

            self.assertEqual(summary["metadata"]["total_runs"], len(SCENARIO_CATALOG) * len(BASE_SEEDS))
            self.assertEqual(summary["metadata"]["total_collisions"], 0)
            self.assertTrue(summary["global_averages"]["zero_collision_integrity"])

            # Verify files written
            json_file = temp_dir / "scenario_matrix_summary.json"
            csv_file = temp_dir / "scenario_matrix_summary.csv"
            self.assertTrue(json_file.exists())
            self.assertTrue(csv_file.exists())

            # Verify MAE reduction is in valid range
            glob_mae = summary["global_averages"]["mean_mae_reduction_pct"]["mean"]
            self.assertGreater(glob_mae, 50.0)
            self.assertLess(glob_mae, 80.0)
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()