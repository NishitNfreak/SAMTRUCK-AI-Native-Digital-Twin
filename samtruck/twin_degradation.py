"""Deterministic, Twin-only sensor and V2X degradation schedules.

This module operates on already-created measurement DTOs at the simulator
boundary.  It has no TraCI, controller, evaluator, or V2X-channel dependency.
It is deliberately opt-in: callers that do not supply a schedule retain their
existing sensor and communication behavior.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Iterable

from .sensors import VehicleSensorInput


SENSOR_EVENT_TYPES = {"twin_radar_dropout", "twin_odometry_dropout"}
V2X_EVENT_TYPES = {"twin_v2x_latency", "twin_v2x_packet_loss", "twin_v2x_outage"}


class TwinDegradationSchedule:
    """Apply deterministic configured degradation only to the passive Twin path."""

    def __init__(self, events: Iterable[dict[str, object]] = ()) -> None:
        self.events = tuple(dict(event) for event in events)
        for event in self.events:
            kind = str(event.get("type", ""))
            if kind not in SENSOR_EVENT_TYPES | V2X_EVENT_TYPES:
                raise ValueError(f"Unsupported Twin-only degradation event: {kind!r}")
            at_s = float(event.get("at_s", 0.0))
            duration_s = float(event.get("duration_s", 0.0))
            if at_s < 0.0 or duration_s < 0.0:
                raise ValueError("Twin-only degradation timing must be non-negative.")
            if kind == "twin_v2x_latency" and float(event.get("latency_ms", 0.0)) < 0.0:
                raise ValueError("Twin-only V2X latency must be non-negative.")
            if kind == "twin_v2x_packet_loss":
                loss = float(event.get("loss_rate", 0.0))
                if not 0.0 <= loss <= 1.0:
                    raise ValueError("Twin-only V2X loss rate must be in [0, 1].")
        self.statistics = {
            "sensor_inputs_seen": 0,
            "radar_measurements_seen": 0,
            "radar_measurements_suppressed": 0,
            "odometry_measurements_seen": 0,
            "odometry_measurements_suppressed": 0,
            "twin_v2x_attempts": 0,
            "twin_v2x_outage_attempts": 0,
        }

    @property
    def has_v2x_events(self) -> bool:
        return any(str(event.get("type")) in V2X_EVENT_TYPES for event in self.events)

    def active_events(self, now_s: float) -> tuple[dict[str, object], ...]:
        return tuple(
            event for event in self.events
            if float(event.get("at_s", 0.0)) <= now_s < float(event.get("at_s", 0.0)) + float(event.get("duration_s", 0.0))
        )

    def apply_sensor_inputs(self, now_s: float, inputs: Iterable[VehicleSensorInput]) -> list[VehicleSensorInput]:
        """Return degraded Twin DTOs without altering input DTOs or truth data."""

        active_types = {str(event.get("type")) for event in self.active_events(now_s)}
        radar_dropout = "twin_radar_dropout" in active_types
        odometry_dropout = "twin_odometry_dropout" in active_types
        degraded: list[VehicleSensorInput] = []
        for sensor_input in inputs:
            self.statistics["sensor_inputs_seen"] += 1
            radar = sensor_input.radar
            odometry = sensor_input.odometry
            if radar is not None and radar.valid:
                self.statistics["radar_measurements_seen"] += 1
            if odometry.valid:
                self.statistics["odometry_measurements_seen"] += 1
            if radar_dropout and radar is not None and radar.valid:
                radar = replace(
                    radar, range_m=None, relative_speed_mps=None, valid=False, status="scheduled_dropout"
                )
                self.statistics["radar_measurements_suppressed"] += 1
            if odometry_dropout and odometry.valid:
                # Explicit absence prevents numerical payloads from refreshing
                # Twin component timestamps or Kalman measurement rows.
                odometry = replace(
                    odometry, position_m=None, speed_mps=None, acceleration_mps2=None,
                    valid=False, status="scheduled_dropout",
                )
                self.statistics["odometry_measurements_suppressed"] += 1
            degraded.append(replace(sensor_input, odometry=odometry, radar=radar))
        return degraded

    def v2x_state(self, now_s: float) -> tuple[float | None, float]:
        """Return Twin-only ``(latency_ms, loss_probability)`` for this time."""

        latency_ms = 40.0
        loss_rate = 0.0
        outage = False
        for event in self.active_events(now_s):
            kind = str(event.get("type"))
            if kind == "twin_v2x_latency":
                latency_ms = float(event["latency_ms"])
            elif kind == "twin_v2x_packet_loss":
                loss_rate = max(loss_rate, float(event["loss_rate"]))
            elif kind == "twin_v2x_outage":
                outage = True
        return (None if outage else latency_ms), loss_rate

    def record_v2x_attempt(self, latency_ms: float | None) -> None:
        self.statistics["twin_v2x_attempts"] += 1
        if latency_ms is None:
            self.statistics["twin_v2x_outage_attempts"] += 1

    def summary(self) -> dict[str, object]:
        return {
            "events": list(self.events),
            "statistics": dict(self.statistics),
            "has_twin_only_v2x_events": self.has_v2x_events,
        }
