"""Passive, measurement-only estimation primitives for SAMTRUCK Phase 2.1A.

This module deliberately has no TraCI, SUMOlib, evaluator, controller, or
scenario dependency.  It accepts scalar sensor observations and already
delivered V2X DTOs only.  Nothing here actuates a vehicle or changes CACC.

The own-vehicle filter is a linear constant-acceleration Kalman filter with
state ``[longitudinal_position_m, speed_mps, acceleration_mps2]``.  Position
may only be supplied when it is an *independent* measurement.  In particular,
an odometry distance reconstructed from the same speed samples must not be fed
back as an independent position update because that would double-count noise.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .v2x import V2XMessage


_EPSILON = 1e-10


@dataclass(frozen=True)
class EstimatorConfig:
    """Simulation assumptions for the own-state Kalman filter.

    The values are deliberately conservative defaults, not calibrated truck
    sensor specifications.  They are kept in this standalone configuration so
    Phase 2.1A does not alter the established SUMO/CACC configuration.
    """

    jerk_process_variance: float = 1.0
    position_measurement_variance: float = 4.0
    speed_measurement_variance: float = 1.0
    acceleration_measurement_variance: float = 4.0
    initial_position_variance: float = 1_000_000.0
    initial_speed_variance: float = 25.0
    initial_acceleration_variance: float = 25.0
    long_gap_s: float = 1.0

    def __post_init__(self) -> None:
        for name, value in self.__dict__.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")


@dataclass(frozen=True)
class FilteredVehicleState:
    """Immutable own-vehicle filter output in SI units.

    ``covariance`` is a row-major immutable 3x3 tuple.  Missing or
    uninitialized components are represented by ``None`` and status explains
    why; a zero is never used to stand in for a missing measurement.
    """

    vehicle_id: str
    timestamp_s: float | None
    estimated_position_m: float | None
    estimated_speed_mps: float | None
    estimated_acceleration_mps2: float | None
    covariance: tuple[tuple[float, float, float], ...] | None
    position_std_m: float | None
    speed_std_mps: float | None
    acceleration_std_mps2: float | None
    initialized: bool
    status: str
    measurement_update_mask: tuple[str, ...]
    # Phase 2.2A.2: own-state measurement freshness.  These optional,
    # additive fields preserve construction compatibility with earlier output
    # objects.  They describe accepted measurement rows, never SUMO truth.
    last_independent_position_measurement_timestamp_s: float | None = None
    independent_position_measurement_age_s: float | None = None
    independent_position_measurement_ever_received: bool = False
    last_speed_measurement_timestamp_s: float | None = None
    speed_measurement_age_s: float | None = None
    speed_measurement_ever_received: bool = False
    last_acceleration_measurement_timestamp_s: float | None = None
    acceleration_measurement_age_s: float | None = None
    acceleration_measurement_ever_received: bool = False
    position_estimate_provenance: str = "unavailable"
    speed_estimate_provenance: str = "unavailable"
    acceleration_estimate_provenance: str = "unavailable"


@dataclass(frozen=True)
class RadarTrackerConfig:
    """Simulation assumptions for a relative radar target track."""

    relative_acceleration_process_variance: float = 1.0
    range_measurement_variance: float = 1.0
    relative_speed_measurement_variance: float = 1.0
    initial_range_variance: float = 100.0
    initial_relative_speed_variance: float = 25.0
    long_gap_s: float = 1.0

    def __post_init__(self) -> None:
        for name, value in self.__dict__.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")


@dataclass(frozen=True)
class RelativeTargetState:
    """Immutable radar target output; positive relative speed means closing."""

    target_vehicle_id: str
    timestamp_s: float | None
    estimated_range_m: float | None
    estimated_relative_speed_mps: float | None
    covariance: tuple[tuple[float, float], tuple[float, float]] | None
    range_std_m: float | None
    relative_speed_std_mps: float | None
    radar_valid: bool
    status: str
    measurement_update_mask: tuple[str, ...]
    # Added in Phase 2.1A.1.  Defaults preserve the original positional
    # constructor contract while making partial radar initialization/freshness
    # explicit to callers.
    observed_components: tuple[str, ...] = ()
    unobserved_prior_components: tuple[str, ...] = ()
    last_measurement_timestamp_s: float | None = None
    prediction_timestamp_s: float | None = None
    track_age_s: float | None = None
    last_update_was_measurement: bool = False


@dataclass(frozen=True)
class PredecessorV2XObservation:
    """A delayed predecessor observation propagated from message generation.

    This is a compatibility helper, not a delivery queue and not a filter
    update.  The caller must pass only an already delivered/validated message
    from ``PendingV2XChannel``.  Since a current V2X payload has no position,
    it exposes only propagated speed and acceleration.
    """

    sender_id: str
    receiver_id: str | None
    generation_timestamp_s: float
    estimator_timestamp_s: float
    message_age_s: float
    propagated_speed_mps: float | None
    propagated_acceleration_mps2: float | None
    speed_variance: float | None
    acceleration_variance: float | None
    status: str


def _finite(value: float | None, name: str) -> float | None:
    if value is None:
        return None
    numeric = float(value)
    if not np.isfinite(numeric):
        raise ValueError(f"{name} must be finite when supplied.")
    return numeric


def _positive_variance(value: float | None, default: float, name: str) -> float:
    variance = default if value is None else float(value)
    if not np.isfinite(variance) or variance <= 0.0:
        raise ValueError(f"{name} must be finite and positive.")
    return variance


def _symmetrize(covariance: np.ndarray) -> np.ndarray:
    """Return a finite symmetric covariance or raise a numerical error."""

    symmetric = (covariance + covariance.T) * 0.5
    if not np.all(np.isfinite(symmetric)):
        raise FloatingPointError("Covariance became non-finite.")
    eigenvalues = np.linalg.eigvalsh(symmetric)
    if float(eigenvalues.min()) < -1e-8:
        raise FloatingPointError("Covariance is not positive semidefinite.")
    return symmetric


def _covariance_tuple(covariance: np.ndarray) -> tuple[tuple[float, ...], ...]:
    return tuple(tuple(float(value) for value in row) for row in covariance)


class LinearKalmanFilter:
    """Small validated linear Kalman filter used by the two estimator wrappers."""

    def __init__(self, state: np.ndarray, covariance: np.ndarray) -> None:
        self.state = np.asarray(state, dtype=float).reshape(-1)
        self.covariance = _symmetrize(np.asarray(covariance, dtype=float))
        if self.covariance.shape != (len(self.state), len(self.state)):
            raise ValueError("State and covariance dimensions do not match.")
        if not np.all(np.isfinite(self.state)):
            raise ValueError("Initial state must be finite.")

    def predict(self, transition: np.ndarray, process_covariance: np.ndarray) -> None:
        transition = np.asarray(transition, dtype=float)
        process_covariance = np.asarray(process_covariance, dtype=float)
        size = len(self.state)
        if transition.shape != (size, size) or process_covariance.shape != (size, size):
            raise ValueError("Prediction matrices have invalid dimensions.")
        self.state = transition @ self.state
        self.covariance = _symmetrize(transition @ self.covariance @ transition.T + process_covariance)

    def update(self, measurement: Iterable[float], observation: np.ndarray, measurement_covariance: np.ndarray) -> None:
        measurement_vector = np.asarray(tuple(measurement), dtype=float).reshape(-1)
        observation = np.asarray(observation, dtype=float)
        measurement_covariance = np.asarray(measurement_covariance, dtype=float)
        size = len(self.state)
        rows = len(measurement_vector)
        if rows == 0:
            return
        if observation.shape != (rows, size) or measurement_covariance.shape != (rows, rows):
            raise ValueError("Measurement matrices have invalid dimensions.")
        if not np.all(np.isfinite(measurement_vector)):
            raise ValueError("Measurement must be finite.")
        _symmetrize(measurement_covariance)

        innovation = measurement_vector - observation @ self.state
        innovation_covariance = observation @ self.covariance @ observation.T + measurement_covariance
        try:
            # Solve S K^T = (P H^T)^T instead of explicitly computing inv(S).
            gain = np.linalg.solve(innovation_covariance, (self.covariance @ observation.T).T).T
        except np.linalg.LinAlgError as error:
            raise FloatingPointError("Innovation covariance is singular.") from error
        self.state = self.state + gain @ innovation
        identity = np.eye(size)
        residual = identity - gain @ observation
        # Joseph form maintains positive semidefiniteness more reliably.
        self.covariance = _symmetrize(
            residual @ self.covariance @ residual.T + gain @ measurement_covariance @ gain.T
        )


class VehicleStateEstimator:
    """Timestamp-aware own-vehicle constant-acceleration Kalman filter.

    Position updates are accepted only through the ``independent_position_m``
    argument.  A speed-integrated dead-reckoning position must remain a
    prediction aid/diagnostic and cannot be supplied as this independent row.
    """

    def __init__(self, vehicle_id: str, config: EstimatorConfig | None = None) -> None:
        self.vehicle_id = vehicle_id
        self.config = config or EstimatorConfig()
        self._filter: LinearKalmanFilter | None = None
        self._timestamp_s: float | None = None
        self._last_output: FilteredVehicleState | None = None
        # Each timestamp is refreshed only after its matching Kalman
        # measurement row is accepted.  A prior, prediction, radar track, or
        # V2X message cannot make an own-state component fresh.
        self._last_measurement_timestamps: dict[str, float | None] = {
            "position": None,
            "speed": None,
            "acceleration": None,
        }

    @staticmethod
    def transition(dt_s: float) -> np.ndarray:
        return np.array([[1.0, dt_s, 0.5 * dt_s**2], [0.0, 1.0, dt_s], [0.0, 0.0, 1.0]])

    def process_covariance(self, dt_s: float) -> np.ndarray:
        q = self.config.jerk_process_variance
        return q * np.array([
            [dt_s**5 / 20.0, dt_s**4 / 8.0, dt_s**3 / 6.0],
            [dt_s**4 / 8.0, dt_s**3 / 3.0, dt_s**2 / 2.0],
            [dt_s**3 / 6.0, dt_s**2 / 2.0, dt_s],
        ])

    def update(
        self,
        timestamp_s: float,
        *,
        independent_position_m: float | None = None,
        speed_mps: float | None = None,
        acceleration_mps2: float | None = None,
        position_variance: float | None = None,
        speed_variance: float | None = None,
        acceleration_variance: float | None = None,
    ) -> FilteredVehicleState:
        """Predict then update from available independent finite measurements."""

        timestamp = _finite(timestamp_s, "timestamp_s")
        position = _finite(independent_position_m, "independent_position_m")
        speed = _finite(speed_mps, "speed_mps")
        acceleration = _finite(acceleration_mps2, "acceleration_mps2")
        if self._timestamp_s is not None and timestamp < self._timestamp_s - _EPSILON:
            return self._snapshot("invalid_timestamp", ())

        if self._filter is None:
            if speed is None:
                self._timestamp_s = timestamp
                return self._snapshot("uninitialized_missing_speed", ())
            initial_position = position if position is not None else 0.0
            initial_acceleration = acceleration if acceleration is not None else 0.0
            covariance = np.diag([
                _positive_variance(position_variance, self.config.position_measurement_variance, "position_variance")
                if position is not None else self.config.initial_position_variance,
                _positive_variance(speed_variance, self.config.speed_measurement_variance, "speed_variance"),
                _positive_variance(acceleration_variance, self.config.acceleration_measurement_variance, "acceleration_variance")
                if acceleration is not None else self.config.initial_acceleration_variance,
            ])
            self._filter = LinearKalmanFilter(np.array([initial_position, speed, initial_acceleration]), covariance)
            self._timestamp_s = timestamp
            mask = tuple(name for name, value in (
                ("position", position), ("speed", speed), ("acceleration", acceleration)
            ) if value is not None)
            self._record_accepted_measurements(timestamp, mask)
            return self._snapshot("initialized" if acceleration is not None else "partially_initialized", mask)

        dt_s = timestamp - self._timestamp_s
        if dt_s > _EPSILON:
            self._filter.predict(self.transition(dt_s), self.process_covariance(dt_s))
        self._timestamp_s = timestamp
        mask: list[str] = []
        rows: list[np.ndarray] = []
        values: list[float] = []
        variances: list[float] = []
        if position is not None:
            rows.append(np.array([1.0, 0.0, 0.0]))
            values.append(position)
            variances.append(_positive_variance(position_variance, self.config.position_measurement_variance, "position_variance"))
            mask.append("position")
        if speed is not None:
            rows.append(np.array([0.0, 1.0, 0.0]))
            values.append(speed)
            variances.append(_positive_variance(speed_variance, self.config.speed_measurement_variance, "speed_variance"))
            mask.append("speed")
        if acceleration is not None:
            rows.append(np.array([0.0, 0.0, 1.0]))
            values.append(acceleration)
            variances.append(_positive_variance(acceleration_variance, self.config.acceleration_measurement_variance, "acceleration_variance"))
            mask.append("acceleration")
        if rows:
            self._filter.update(values, np.vstack(rows), np.diag(variances))
            self._record_accepted_measurements(timestamp, tuple(mask))
        if dt_s > self.config.long_gap_s:
            status = "degraded_long_gap_updated" if rows else "degraded_long_gap_predicted"
        elif rows:
            status = "updated" if dt_s > _EPSILON else "updated_same_timestamp"
        else:
            status = "predicted_only"
        return self._snapshot(status, tuple(mask))

    def predict_only(self, timestamp_s: float) -> FilteredVehicleState:
        """Advance without inventing a measurement."""

        return self.update(timestamp_s)

    def _snapshot(self, status: str, mask: tuple[str, ...]) -> FilteredVehicleState:
        freshness = self._freshness_metadata(status, mask)
        if self._filter is None:
            result = FilteredVehicleState(
                self.vehicle_id, self._timestamp_s, None, None, None, None, None, None, None,
                False, status, mask, **freshness,
            )
        else:
            covariance = _symmetrize(self._filter.covariance)
            result = FilteredVehicleState(
                self.vehicle_id, self._timestamp_s,
                float(self._filter.state[0]), float(self._filter.state[1]), float(self._filter.state[2]),
                _covariance_tuple(covariance),
                float(np.sqrt(max(0.0, covariance[0, 0]))),
                float(np.sqrt(max(0.0, covariance[1, 1]))),
                float(np.sqrt(max(0.0, covariance[2, 2]))),
                True, status, mask, **freshness,
            )
        self._last_output = result
        return result

    def _record_accepted_measurements(self, timestamp_s: float, mask: tuple[str, ...]) -> None:
        """Record only measurement rows accepted by the own-state filter."""

        for component in mask:
            self._last_measurement_timestamps[component] = timestamp_s

    def _freshness_metadata(self, status: str, mask: tuple[str, ...]) -> dict[str, object]:
        """Return per-component accepted-measurement timestamps, ages, and provenance.

        Ages are relative to the accepted estimator timestamp.  In particular,
        an invalid/out-of-order input leaves the internal timestamp and all
        component timestamps unchanged; its snapshot is visibly labelled as a
        no-update event rather than as a fresh or predicted measurement.
        """

        timestamp = self._timestamp_s
        metadata: dict[str, object] = {}
        names = {
            "position": "independent_position",
            "speed": "speed",
            "acceleration": "acceleration",
        }
        for component, output_name in names.items():
            last_timestamp = self._last_measurement_timestamps[component]
            age = None if timestamp is None or last_timestamp is None else max(0.0, timestamp - last_timestamp)
            if status == "invalid_timestamp":
                provenance = "invalid_timestamp_no_update"
            elif component in mask:
                provenance = "measurement_updated"
            elif self._filter is None:
                provenance = "unavailable"
            elif last_timestamp is None:
                # A zero-valued initialization prior is not a measurement.
                provenance = "not_meaningfully_initialized"
            else:
                provenance = "prediction_only"
            metadata[f"last_{output_name}_measurement_timestamp_s"] = last_timestamp
            metadata[f"{output_name}_measurement_age_s"] = age
            metadata[f"{output_name}_measurement_ever_received"] = last_timestamp is not None
            metadata[f"{component}_estimate_provenance"] = provenance
        return metadata


class RadarTargetTracker:
    """Identity-keyed relative range/closing-speed filters for radar targets."""

    def __init__(self, config: RadarTrackerConfig | None = None) -> None:
        self.config = config or RadarTrackerConfig()
        self._filters: dict[str, LinearKalmanFilter] = {}
        self._timestamps: dict[str, float] = {}
        self._last_measurement_timestamps: dict[str, float] = {}
        self._last_observed_components: dict[str, tuple[str, ...]] = {}
        self._unobserved_prior_components: dict[str, tuple[str, ...]] = {}
        self._last_update_was_measurement: dict[str, bool] = {}

    @staticmethod
    def transition(dt_s: float) -> np.ndarray:
        # d(k+1) = d(k) - closing_speed * dt; positive closing closes the gap.
        return np.array([[1.0, -dt_s], [0.0, 1.0]])

    def process_covariance(self, dt_s: float) -> np.ndarray:
        q = self.config.relative_acceleration_process_variance
        return q * np.array([
            [dt_s**3 / 3.0, -dt_s**2 / 2.0],
            [-dt_s**2 / 2.0, dt_s],
        ])

    def update(
        self,
        target_vehicle_id: str,
        timestamp_s: float,
        *,
        range_m: float | None = None,
        relative_speed_mps: float | None = None,
        range_variance: float | None = None,
        relative_speed_variance: float | None = None,
    ) -> RelativeTargetState:
        """Update one target; a target ID is required to prevent identity mixing."""

        if not target_vehicle_id:
            raise ValueError("target_vehicle_id is required for a radar track.")
        timestamp = _finite(timestamp_s, "timestamp_s")
        distance = _finite(range_m, "range_m")
        relative_speed = _finite(relative_speed_mps, "relative_speed_mps")
        previous_timestamp = self._timestamps.get(target_vehicle_id)
        if previous_timestamp is not None and timestamp < previous_timestamp - _EPSILON:
            return self._snapshot(target_vehicle_id, "invalid_timestamp", False, ())

        tracked = self._filters.get(target_vehicle_id)
        if tracked is None:
            if distance is None and relative_speed is None:
                self._timestamps[target_vehicle_id] = timestamp
                self._last_update_was_measurement[target_vehicle_id] = False
                return self._snapshot(target_vehicle_id, "uninitialized_missing_radar", False, ())
            initial_distance = distance if distance is not None else 0.0
            initial_relative_speed = relative_speed if relative_speed is not None else 0.0
            covariance = np.diag([
                _positive_variance(range_variance, self.config.range_measurement_variance, "range_variance")
                if distance is not None else self.config.initial_range_variance,
                _positive_variance(relative_speed_variance, self.config.relative_speed_measurement_variance, "relative_speed_variance")
                if relative_speed is not None else self.config.initial_relative_speed_variance,
            ])
            self._filters[target_vehicle_id] = LinearKalmanFilter(
                np.array([initial_distance, initial_relative_speed]), covariance
            )
            self._timestamps[target_vehicle_id] = timestamp
            mask = tuple(name for name, value in (("range", distance), ("relative_speed", relative_speed)) if value is not None)
            self._last_measurement_timestamps[target_vehicle_id] = timestamp
            self._last_observed_components[target_vehicle_id] = mask
            self._unobserved_prior_components[target_vehicle_id] = tuple(
                component for component in ("range", "relative_speed") if component not in mask
            )
            self._last_update_was_measurement[target_vehicle_id] = True
            if mask == ("range",):
                status = "initialized_range_only_prior_relative_speed"
            elif mask == ("relative_speed",):
                status = "initialized_relative_speed_only_prior_range"
            else:
                status = "initialized"
            return self._snapshot(target_vehicle_id, status, True, mask)

        dt_s = timestamp - previous_timestamp
        if dt_s > _EPSILON:
            tracked.predict(self.transition(dt_s), self.process_covariance(dt_s))
        self._timestamps[target_vehicle_id] = timestamp
        rows: list[np.ndarray] = []
        values: list[float] = []
        variances: list[float] = []
        mask: list[str] = []
        if distance is not None:
            rows.append(np.array([1.0, 0.0]))
            values.append(distance)
            variances.append(_positive_variance(range_variance, self.config.range_measurement_variance, "range_variance"))
            mask.append("range")
        if relative_speed is not None:
            rows.append(np.array([0.0, 1.0]))
            values.append(relative_speed)
            variances.append(_positive_variance(relative_speed_variance, self.config.relative_speed_measurement_variance, "relative_speed_variance"))
            mask.append("relative_speed")
        if rows:
            tracked.update(values, np.vstack(rows), np.diag(variances))
            self._last_measurement_timestamps[target_vehicle_id] = timestamp
            self._last_observed_components[target_vehicle_id] = tuple(mask)
            prior_components = self._unobserved_prior_components.get(target_vehicle_id, ())
            self._unobserved_prior_components[target_vehicle_id] = tuple(
                component for component in prior_components if component not in mask
            )
            self._last_update_was_measurement[target_vehicle_id] = True
        else:
            self._last_update_was_measurement[target_vehicle_id] = False
        if dt_s > self.config.long_gap_s:
            status = "degraded_long_gap_updated" if rows else "degraded_long_gap_missing"
        elif rows:
            status = "updated" if dt_s > _EPSILON else "updated_same_timestamp"
        else:
            status = "predicted_missing"
        return self._snapshot(target_vehicle_id, status, bool(rows), tuple(mask))

    def predict_missing(self, target_vehicle_id: str, timestamp_s: float) -> RelativeTargetState:
        """Explicitly represent radar target disappearance without a fake zero."""

        return self.update(target_vehicle_id, timestamp_s)

    def state(self, target_vehicle_id: str) -> RelativeTargetState | None:
        """Return the stored track without claiming it was freshly measured.

        ``last_measurement_timestamp_s`` and ``track_age_s`` identify freshness;
        callers wanting to move the state forward must explicitly call
        :meth:`predict_missing` with a timestamp.
        """

        if target_vehicle_id not in self._timestamps:
            return None
        was_measured = self._last_update_was_measurement.get(target_vehicle_id, False)
        return self._snapshot(
            target_vehicle_id,
            "latest_measurement" if was_measured else "prediction_only",
            was_measured,
            (),
        )

    def _snapshot(self, target_vehicle_id: str, status: str, radar_valid: bool, mask: tuple[str, ...]) -> RelativeTargetState:
        tracked = self._filters.get(target_vehicle_id)
        timestamp = self._timestamps.get(target_vehicle_id)
        last_measurement_timestamp = self._last_measurement_timestamps.get(target_vehicle_id)
        observed_components = self._last_observed_components.get(target_vehicle_id, ())
        unobserved_prior_components = self._unobserved_prior_components.get(target_vehicle_id, ()) if tracked is not None else ()
        track_age = None if timestamp is None or last_measurement_timestamp is None else max(0.0, timestamp - last_measurement_timestamp)
        last_update_was_measurement = self._last_update_was_measurement.get(target_vehicle_id, False)
        if tracked is None:
            return RelativeTargetState(
                target_vehicle_id, timestamp, None, None, None, None, None, radar_valid, status, mask,
                observed_components, unobserved_prior_components, last_measurement_timestamp, timestamp,
                track_age, last_update_was_measurement,
            )
        covariance = _symmetrize(tracked.covariance)
        return RelativeTargetState(
            target_vehicle_id, timestamp, float(tracked.state[0]), float(tracked.state[1]),
            _covariance_tuple(covariance), float(np.sqrt(max(0.0, covariance[0, 0]))),
            float(np.sqrt(max(0.0, covariance[1, 1]))), radar_valid, status, mask,
            observed_components, unobserved_prior_components, last_measurement_timestamp, timestamp,
            track_age, last_update_was_measurement,
        )


def propagate_predecessor_v2x(
    message: V2XMessage,
    estimator_timestamp_s: float,
    *,
    stale_after_s: float,
    speed_variance: float = 1.0,
    acceleration_variance: float = 4.0,
    acceleration_process_variance: float = 1.0,
) -> PredecessorV2XObservation:
    """Age a delivered predecessor message forward without treating it as current.

    The function intentionally does not validate, queue, or accept raw V2X
    traffic.  That remains the responsibility of ``PendingV2XChannel``.  It
    makes delay explicit and returns ``stale`` instead of a usable observation
    when the age exceeds ``stale_after_s``.
    """

    if not isinstance(message, V2XMessage):
        raise TypeError("message must be an already delivered V2XMessage.")
    now = _finite(estimator_timestamp_s, "estimator_timestamp_s")
    generation_time = _finite(message.sim_time, "message.sim_time")
    if stale_after_s <= 0.0 or not np.isfinite(stale_after_s):
        raise ValueError("stale_after_s must be finite and positive.")
    process_variance = float(acceleration_process_variance)
    if not np.isfinite(process_variance) or process_variance < 0.0:
        raise ValueError("acceleration_process_variance must be finite and non-negative.")
    age = now - generation_time
    if age < -_EPSILON:
        return PredecessorV2XObservation(
            message.sender_id, message.receiver_id, generation_time, now, age,
            None, None, None, None, "future_timestamp",
        )
    age = max(0.0, age)
    if age > stale_after_s + _EPSILON:
        return PredecessorV2XObservation(
            message.sender_id, message.receiver_id, generation_time, now, age,
            None, None, None, None, "stale",
        )
    speed = _finite(message.speed, "message.speed")
    acceleration = _finite(message.acceleration, "message.acceleration")
    speed_var = _positive_variance(speed_variance, 1.0, "speed_variance") + age**2 * process_variance
    acceleration_var = _positive_variance(acceleration_variance, 1.0, "acceleration_variance") + age * process_variance
    return PredecessorV2XObservation(
        message.sender_id, message.receiver_id, generation_time, now, age,
        speed + acceleration * age, acceleration, speed_var, acceleration_var, "fresh_propagated",
    )
