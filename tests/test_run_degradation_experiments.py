"""Regression tests for safe Phase 2.2C summary reconstruction."""
from __future__ import annotations
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import run_degradation_experiments as experiments

EXPERIMENT = {"id": "example", "seeds": [7], "twin": {}, "twin_only_degradations": []}
CONFIG = {"experiments": [EXPERIMENT]}


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value), encoding="utf-8")

class SummaryReconstructionTests(unittest.TestCase):
    def _create_run(self, root: Path, *, status: str | None, complete: bool = True) -> None:
        output = root / "example_s7"
        output.mkdir()
        if status is not None:
            write_json(output / "metrics.json", {"pass_fail": {"status": status}})
        if complete:
            write_json(output / "twin_degradation_summary.json", {"statistics": {}})
            write_json(output / "twin_uncertainty_evaluation.json", {"metrics": {}, "matching_diagnostics": {}})

    def test_previously_passed_run_uses_recorded_metrics_verdict(self) -> None:
        with tempfile.TemporaryDirectory() as directory, patch.object(experiments, "RESULTS_DIR", Path(directory)):
            self._create_run(Path(directory), status="PASS")
            record = experiments.summarize_existing(CONFIG)[0]
        self.assertIs(record["pass"], True)
        self.assertEqual(record["pass_status_source"], "recorded_pass")
        self.assertEqual(record["result_completeness"], "complete")

    def test_previously_failed_run_cannot_be_labelled_passed(self) -> None:
        with tempfile.TemporaryDirectory() as directory, patch.object(experiments, "RESULTS_DIR", Path(directory)):
            self._create_run(Path(directory), status="FAIL")
            record = experiments.summarize_existing(CONFIG)[0]
        self.assertIs(record["pass"], False)
        self.assertEqual(record["pass_status_source"], "recorded_fail")

    def test_missing_or_incomplete_folder_is_unknown_not_an_exception_or_pass(self) -> None:
        with tempfile.TemporaryDirectory() as directory, patch.object(experiments, "RESULTS_DIR", Path(directory)):
            record = experiments.summarize_existing(CONFIG)[0]
        self.assertIsNone(record["pass"])
        self.assertEqual(record["pass_status_source"], "unknown_missing_metrics_json")
        self.assertEqual(record["result_completeness"], "incomplete")

    def test_unverifiable_status_is_unknown(self) -> None:
        with tempfile.TemporaryDirectory() as directory, patch.object(experiments, "RESULTS_DIR", Path(directory)):
            self._create_run(Path(directory), status="PENDING")
            record = experiments.summarize_existing(CONFIG)[0]
        self.assertIsNone(record["pass"])
        self.assertEqual(record["pass_status_source"], "unknown_missing_or_unrecognized_pass_status")

if __name__ == "__main__":
    unittest.main()
