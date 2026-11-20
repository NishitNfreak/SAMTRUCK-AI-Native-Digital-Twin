"""Sensor DTOs at the simulator boundary.

This module accepts scalar observations and returns measurement-only objects. It
never imports TraCI; only the simulation runner may read TraCI and pass scalar
values here. The Digital Twin sees only the DTOs declared in this module.
"""

from __future__ import annotations

from dataclasses import dataclass
import random


@dataclass(frozen=True)
class RadarMeasurement:
    """SI-unit radar measurement; positive relative speed means closing."""

    timestamp_s: float
    vehicle_id: str
    sensed_leader_id: str | None
    range_m: float | None
    relative_speed_mps: float | None
    valid: bool
    status: str


@dataclass(frozen=True)
class OdometryMeasurement:
    """SI-unit own-vehicle odometry measurement."""

    timestamp_s: float
    vehicle_id: str
    position_m: float | None
    speed_mps: float | None
    acceleration_mps2: float | None
    valid: bool
    status: str


@dataclass(frozen=True)
class VehicleSensorInput:
    """Typed local measurements provided to the Twin for one vehicle/timestep."""

    timestamp_s: float
    vehicle_id: str
    platoon_predecessor_id: str | None
    odometry: OdometryMeasurement
    radar: RadarMeasurement | None


class SensorInterface:
    """Generate reproducible noisy/dropout-prone sensor DTOs from scalar data."""

    def __init__(
        self,
        radar_noise_std_m: float = 0.0,
        odometry_speed_noise_std_mps: float = 0.0,
        radar_dropout_probability: float = 0.0,
        seed: int = 42,
    ) -> None:
        if radar_noise_std_m < 0 or odometry_speed_noise_std_mps < 0:
            raise ValueError("Sensor noise standard deviations must be non-negative.")
        if not 0.0 <= radar_dropout_probability <= 1.0:
            raise ValueError("Radar dropout probability must be in [0, 1].")
        self.radar_noise_std_m = radar_noise_std_m
        self.odometry_speed_noise_std_mps = odometry_speed_noise_std_mps
        self.radar_dropout_probability = radar_dropout_probability
        self._rng = random.Random(seed)

    def make_input(
        self, *, timestamp_s: float, vehicle_id: str, platoon_predecessor_id: str | None,
        position_m: float | None, speed_mps: float | None, acceleration_mps2: float | None,
        sensed_leader_id: str | None, radar_range_m: float | None,
        radar_relative_speed_mps: float | None,
    ) -> VehicleSensorInput:
        """Create a DTO; radar dropout is explicit and is never encoded as zero."""

        measured_speed = None if speed_mps is None else max(
            0.0, speed_mps + self._rng.gauss(0.0, self.odometry_speed_noise_std_mps)
        )
        odometry = OdometryMeasurement(
            timestamp_s, vehicle_id, position_m, measured_speed, acceleration_mps2,
            position_m is not None and measured_speed is not None, "measured",
        )
        if radar_range_m is None:
            radar = None
        elif self._rng.random() < self.radar_dropout_probability:
            radar = RadarMeasurement(timestamp_s, vehicle_id, sensed_leader_id, None, None, False, "dropout")
        else:
            radar = RadarMeasurement(
                timestamp_s, vehicle_id, sensed_leader_id,
                max(0.0, radar_range_m + self._rng.gauss(0.0, self.radar_noise_std_m)),
                radar_relative_speed_mps, True, "measured",
            )
        return VehicleSensorInput(timestamp_s, vehicle_id, platoon_predecessor_id, odometry, radar)
