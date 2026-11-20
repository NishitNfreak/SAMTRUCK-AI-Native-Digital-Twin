from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleState:
    sim_time: float
    vehicle_id: str
    position: float
    speed: float
    acceleration: float
    lane: int
    gap: float | None
    time_gap: float | None
    leader_id: str | None

    @property
    def braking(self) -> bool:
        return self.acceleration < -0.5


@dataclass(frozen=True)
class EstimatedVehicleState:
    """Measurement-derived Twin state; it never contains a simulator handle."""

    timestamp_s: float
    vehicle_id: str
    estimated_position_m: float | None
    estimated_speed_mps: float | None
    estimated_acceleration_mps2: float | None
    platoon_predecessor_id: str | None
    sensed_leader_id: str | None
    estimated_gap_m: float | None
    relative_speed_mps: float | None
    estimated_time_gap_s: float | None
    v2x_message_age_s: float | None
    radar_missing: bool
    v2x_missing: bool
    v2x_stale: bool
    braking: bool
    accelerating: bool
    source: str
    quality: str


def time_gap(gap: float | None, speed: float) -> float | None:
    if gap is None or speed <= 0.1:
        return None
    return gap / speed
