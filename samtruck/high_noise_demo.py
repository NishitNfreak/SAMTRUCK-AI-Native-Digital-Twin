"""Offline metrics and plotting for the single Phase 2.2D high-noise demo.

This module reads completed result CSVs only.  SUMO truth is used strictly as
an offline evaluation reference and never enters the live Twin.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _key(row: dict[str, str]) -> tuple[str, str]:
    return row["vehicle_id"], f"{float(row['simulation_time_s']):.6f}"


def _finite(row: dict[str, str], field: str) -> float | None:
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def matched_speed_records(truth_path: Path, sensor_path: Path, filtered_path: Path, vehicle_id: str) -> tuple[list[dict[str, float]], dict[str, int]]:
    """Return only exact vehicle/time matches with three finite speed values."""

    sources = {
        "truth": _read_csv(truth_path),
        "sensor": _read_csv(sensor_path),
        "filtered": _read_csv(filtered_path),
    }
    indexed: dict[str, dict[tuple[str, str], dict[str, str]]] = {}
    diagnostics = {"truth_rows": 0, "sensor_rows": 0, "filtered_rows": 0, "duplicate_keys": 0, "invalid_rows": 0}
    for name, rows in sources.items():
        index: dict[tuple[str, str], dict[str, str]] = {}
        diagnostics[f"{name}_rows"] = sum(row.get("vehicle_id") == vehicle_id for row in rows)
        for row in rows:
            if row.get("vehicle_id") != vehicle_id:
                continue
            try:
                key = _key(row)
            except (KeyError, TypeError, ValueError):
                diagnostics["invalid_rows"] += 1
                continue
            if key in index:
                diagnostics["duplicate_keys"] += 1
                continue
            index[key] = row
        indexed[name] = index

    records: list[dict[str, float]] = []
    for key in sorted(set(indexed["truth"]).intersection(indexed["sensor"], indexed["filtered"]), key=lambda item: float(item[1])):
        truth_speed = _finite(indexed["truth"][key], "speed_mps")
        raw_speed = _finite(indexed["sensor"][key], "odometry_speed_mps")
        filtered_speed = _finite(indexed["filtered"][key], "estimated_speed_mps")
        if truth_speed is None or raw_speed is None or filtered_speed is None:
            diagnostics["invalid_rows"] += 1
            continue
        records.append({
            "simulation_time_s": float(key[1]),
            "sumo_truth_speed_mps": truth_speed,
            "noisy_odometry_speed_mps": raw_speed,
            "filtered_twin_speed_mps": filtered_speed,
        })
    diagnostics["matched_samples"] = len(records)
    return records, diagnostics


def _error_metrics(estimates: list[float], truth: list[float]) -> dict[str, float | int]:
    errors = [estimate - actual for estimate, actual in zip(estimates, truth)]
    count = len(errors)
    if not count:
        raise ValueError("No exact, finite timestamp-matched speed samples are available.")
    return {
        "samples": count,
        "mae_mps": sum(abs(error) for error in errors) / count,
        "rmse_mps": math.sqrt(sum(error * error for error in errors) / count),
    }


def calculate_speed_comparison(records: list[dict[str, float]], diagnostics: dict[str, int]) -> dict[str, Any]:
    """Calculate raw-vs-filtered speed error without changing evaluator schemas."""

    truth = [record["sumo_truth_speed_mps"] for record in records]
    raw = [record["noisy_odometry_speed_mps"] for record in records]
    filtered = [record["filtered_twin_speed_mps"] for record in records]
    raw_metrics = _error_metrics(raw, truth)
    filtered_metrics = _error_metrics(filtered, truth)
    mae_absolute = raw_metrics["mae_mps"] - filtered_metrics["mae_mps"]
    rmse_absolute = raw_metrics["rmse_mps"] - filtered_metrics["rmse_mps"]
    improvement = {
        "mae_absolute_mps": mae_absolute,
        "rmse_absolute_mps": rmse_absolute,
        "mae_percent": None if raw_metrics["mae_mps"] == 0 else mae_absolute / raw_metrics["mae_mps"] * 100.0,
        "rmse_percent": None if raw_metrics["rmse_mps"] == 0 else rmse_absolute / raw_metrics["rmse_mps"] * 100.0,
    }
    improved = mae_absolute > 0 and rmse_absolute > 0
    return {
        "matching": {**diagnostics, "policy": "exact vehicle_id + simulation_time_s normalized to six decimals"},
        "raw_noisy_odometry_vs_sumo_truth": raw_metrics,
        "filtered_twin_vs_sumo_truth": filtered_metrics,
        "improvement_filtered_over_raw": improvement,
        "conclusion": (
            "Filtered Twin speed has lower MAE and RMSE than the persisted noisy odometry."
            if improved else
            "This single run does not show lower filtered MAE and RMSE than persisted noisy odometry."
        ),
    }


def plot_speed_comparison(records: list[dict[str, float]], output_path: Path, vehicle_id: str) -> dict[str, int]:
    """Write the required three-series comparison with truth drawn visibly on top."""

    if not records:
        raise ValueError("Cannot plot a speed comparison without matched records.")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [record["simulation_time_s"] for record in records]
    truth = [record["sumo_truth_speed_mps"] for record in records]
    raw = [record["noisy_odometry_speed_mps"] for record in records]
    filtered = [record["filtered_twin_speed_mps"] for record in records]
    figure, axis = plt.subplots(figsize=(11, 5.5))
    # Draw orange samples first, then filtered response, then blue truth last
    # at the highest z-order.  Truth remains visible even on overlap.
    axis.plot(times, raw, color="tab:orange", marker="o", markersize=2.8, linewidth=0.7,
              alpha=0.75, label="Noisy odometry speed (persisted DTO)", zorder=2)
    axis.plot(times, filtered, color="tab:green", linewidth=1.8,
              label="Kalman-filtered Twin speed", zorder=3)
    axis.plot(times, truth, color="tab:blue", linewidth=2.2,
              label="SUMO ground-truth speed (offline reference)", zorder=5)
    axis.set_xlabel("Simulation time (s)")
    axis.set_ylabel("Speed (m/s)")
    axis.set_title(f"Phase 2.2D high sensor-noise speed comparison — {vehicle_id}")
    axis.grid(True, linestyle=":", alpha=0.65)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)
    return {"truth_points": len(truth), "sensor_points": len(raw), "filtered_points": len(filtered)}
