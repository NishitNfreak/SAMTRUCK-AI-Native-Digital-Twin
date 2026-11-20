"""Transparent passive decision advisor for the SAMTRUCK Digital Twin.

The advisor consumes only Twin-side sensor/filtered/V2X observations. It has
no TraCI, controller, estimator, or actuation dependency. It emits an
explainable executive recommendation; only an explicit opt-in executive may
apply it to a controller.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Callable, Iterable


@dataclass(frozen=True)
class AdvisorThresholds:
    """Advisor-only thresholds; these do not modify estimator/controller tuning."""

    SPEED_SIGMA_WARN_MPS: float = 0.35
    SPEED_SIGMA_HIGH_MPS: float = 0.70
    MEASUREMENT_AGE_STALE_S: float = 1.0
    MEASUREMENT_AGE_CRITICAL_S: float = 3.0
    V2X_AGE_STALE_S: float = 0.5
    V2X_AGE_CRITICAL_S: float = 2.0


@dataclass(frozen=True)
class AdvisorInput:
    simulation_time_s: float
    vehicle_id: str
    twin_filtered_speed_mps: float | None
    twin_filtered_acceleration_mps2: float | None
    twin_speed_variance: float | None
    twin_accel_variance: float | None
    speed_measurement_age_s: float | None
    speed_estimate_provenance: str | None
    sensor_odometry_valid: bool | None
    sensor_odometry_status: str | None
    sensor_radar_valid: bool | None
    sensor_radar_status: str | None
    sensor_radar_range_m: float | None
    predecessor_v2x_status: str | None
    predecessor_v2x_age_s: float | None
    has_platoon_predecessor: bool = True


@dataclass(frozen=True)
class AdvisorDecision:
    simulation_time_s: float
    vehicle_id: str
    decision_code: str
    severity: str
    observations: list[str]
    reasoning: list[str]
    recommended_action: str
    command_to_executive: str
    metrics_used: dict[str, float | str | bool | None]
    rule_id: str


class TwinAdvisor:
    """Apply deterministic health rules to one post-update Twin observation."""

    def __init__(self, thresholds: AdvisorThresholds | None = None) -> None:
        self.thresholds = thresholds or AdvisorThresholds()

    @staticmethod
    def _finite_nonnegative(value: float | None) -> float | None:
        if value is None or not math.isfinite(value) or value < 0.0:
            return None
        return value

    def evaluate(self, item: AdvisorInput) -> AdvisorDecision:
        """Return an explanation-only decision; no execution command exists here."""

        speed_variance = self._finite_nonnegative(item.twin_speed_variance)
        speed_sigma = None if speed_variance is None else math.sqrt(speed_variance)
        age = self._finite_nonnegative(item.speed_measurement_age_s)
        v2x_age = self._finite_nonnegative(item.predecessor_v2x_age_s)
        provenance = item.speed_estimate_provenance or "unavailable"
        sensor_invalid = item.sensor_odometry_valid is False or (item.sensor_odometry_status or "").lower() in {"dropout", "invalid"}
        prediction_only = provenance == "prediction_only"
        own_degraded = sensor_invalid or prediction_only or age is None or age > self.thresholds.MEASUREMENT_AGE_STALE_S
        own_critical = age is None or (age is not None and age > self.thresholds.MEASUREMENT_AGE_CRITICAL_S)
        v2x_status = item.predecessor_v2x_status or "not_available"
        v2x_bad_status = v2x_status in {"not_available", "stale", "rejected", "outage", "missing", "invalid"}
        v2x_degraded = item.has_platoon_predecessor and (v2x_bad_status or v2x_age is None or v2x_age > self.thresholds.V2X_AGE_STALE_S)
        v2x_critical = item.has_platoon_predecessor and (v2x_age is None or v2x_age > self.thresholds.V2X_AGE_CRITICAL_S or v2x_status in {"outage", "rejected", "stale"})
        sigma_warn = speed_sigma is not None and speed_sigma > self.thresholds.SPEED_SIGMA_WARN_MPS
        sigma_high = speed_sigma is not None and speed_sigma > self.thresholds.SPEED_SIGMA_HIGH_MPS

        observations = [
            f"speed provenance={provenance}",
            f"odometry status={item.sensor_odometry_status or 'unavailable'} valid={item.sensor_odometry_valid}",
            f"speed sigma={'unavailable' if speed_sigma is None else f'{speed_sigma:.3f} m/s'}",
            f"speed measurement age={'unavailable' if age is None else f'{age:.3f} s'}",
        ]
        if item.has_platoon_predecessor:
            observations.append(f"predecessor V2X status={v2x_status} age={'unavailable' if v2x_age is None else f'{v2x_age:.3f} s'}")
        else:
            observations.append("predecessor V2X not applicable for platoon leader")
        metrics: dict[str, float | str | bool | None] = {
            "filtered_speed_mps": item.twin_filtered_speed_mps,
            "filtered_acceleration_mps2": item.twin_filtered_acceleration_mps2,
            "speed_variance_m2ps2": speed_variance,
            "speed_sigma_mps": speed_sigma,
            "speed_measurement_age_s": age,
            "speed_estimate_provenance": provenance,
            "sensor_odometry_valid": item.sensor_odometry_valid,
            "sensor_odometry_status": item.sensor_odometry_status,
            "sensor_radar_valid": item.sensor_radar_valid,
            "sensor_radar_status": item.sensor_radar_status,
            "sensor_radar_range_m": item.sensor_radar_range_m,
            "predecessor_v2x_status": v2x_status if item.has_platoon_predecessor else None,
            "predecessor_v2x_age_s": v2x_age if item.has_platoon_predecessor else None,
        }

        if own_critical and v2x_critical and sigma_high:
            return self._decision(item, "CRITICAL_AWARENESS", "critical", observations,
                ["Own-state measurement age is critical.", "Predecessor V2X is critically unavailable/stale.", "Speed uncertainty is high."],
                "Escalate human awareness and apply emergency caution.", metrics, "R7_CRITICAL_COMBINED", "EMERGENCY_CAUTION")
        if own_degraded and v2x_degraded:
            return self._decision(item, "SENSOR_AND_V2X_DEGRADED", "high", observations,
                ["Own-state measurement path is degraded.", "Predecessor V2X path is degraded."],
                "Use radar-only ACC fallback with a wider gap.", metrics, "R5_SENSOR_V2X", "FALLBACK_ACC_WIDE_GAP")
        if (own_degraded or v2x_degraded) and sigma_high:
            return self._decision(item, "INCREASE_GAP_RECOMMENDED", "high", observations,
                ["Confidence is degraded and speed uncertainty exceeds the high advisory threshold."],
                "Increase the CACC time gap.", metrics, "R6_GAP_FOR_CONFIDENCE", "INCREASE_GAP_1_5S")
        if own_degraded:
            return self._decision(item, "PREDICTION_ONLY_DEGRADED", "warn", observations,
                ["Odometry is invalid/dropout, estimate is prediction-only, or measurement age is stale."],
                "Use a cautious wider-gap CACC mode until valid measurement recovery.", metrics, "R4_OWN_MEASUREMENT", "INCREASE_GAP_1_5S")
        if v2x_degraded:
            return self._decision(item, "V2X_DEGRADED", "warn", observations,
                ["Predecessor communication is missing, stale, rejected, or above the advisory age threshold."],
                "Use radar-only ACC fallback while predecessor communication recovers.", metrics, "R3_V2X", "FALLBACK_ACC")
        if sigma_warn:
            return self._decision(item, "MONITOR_ELEVATED_UNCERTAINTY", "warn", observations,
                ["Speed uncertainty exceeds the warning advisory threshold while measurements remain fresh."],
                "Continue baseline CACC and monitor estimator uncertainty.", metrics, "R2_SIGMA_WARN", "HOLD_BASELINE")
        return self._decision(item, "HOLD_NORMAL_CACC", "info", observations,
            ["Own measurements are fresh, uncertainty is acceptable, and predecessor V2X is healthy or not applicable."],
            "Continue baseline CACC behavior.", metrics, "R1_HEALTHY", "HOLD_BASELINE")

    @staticmethod
    def _decision(item: AdvisorInput, code: str, severity: str, observations: list[str], reasoning: list[str],
                  action: str, metrics: dict[str, float | str | bool | None], rule_id: str,
                  command: str) -> AdvisorDecision:
        return AdvisorDecision(item.simulation_time_s, item.vehicle_id, code, severity, observations, reasoning,
                               action, command, metrics, rule_id)

    def evaluate_many(self, items: list[AdvisorInput]) -> list[AdvisorDecision]:
        return [self.evaluate(item) for item in items]


class AdvisorDecisionConsole:
    """Terminal/JSONL sink for advisor recommendations, without actuation."""

    def __init__(self, jsonl_path: Path, *, period_s: float = 1.0, printer: Callable[[str], None] = print) -> None:
        if not math.isfinite(period_s) or period_s <= 0.0:
            raise ValueError("Advisor period must be a positive finite number of seconds.")
        self.period_s = period_s
        self._printer = printer
        self._handle = jsonl_path.open("w", encoding="utf-8")
        self._next_print_s: float | None = None

    def record(self, decisions: Iterable[AdvisorDecision]) -> None:
        for decision in decisions:
            self._handle.write(json.dumps(asdict(decision), allow_nan=False) + "\n")
        self._handle.flush()

    def maybe_print(self, simulation_time_s: float, decisions: list[AdvisorDecision]) -> None:
        if self._next_print_s is None:
            self._next_print_s = simulation_time_s
        if simulation_time_s + 1e-9 < self._next_print_s:
            return
        while self._next_print_s <= simulation_time_s + 1e-9:
            self._next_print_s += self.period_s
        self._printer("-" * 78)
        self._printer(f"t = {simulation_time_s:.1f}s | TWIN DECISION ADVISOR (EXECUTIVE OPTIONAL)")
        for decision in decisions:
            self._printer(f"{decision.vehicle_id:<22} {decision.decision_code:<30} severity={decision.severity} rule={decision.rule_id}")
            self._printer(f"  Why: {'; '.join(decision.reasoning)}")
            self._printer(f"  Advisory: {decision.recommended_action}")
            self._printer(f"  Executive recommendation: {decision.command_to_executive}")
        self._printer("-" * 78)

    def close(self) -> None:
        if not self._handle.closed:
            self._handle.flush()
            self._handle.close()
