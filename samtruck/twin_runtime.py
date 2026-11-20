"""Measurement-only Phase 1 Digital Twin runtime for SAMTRUCK.

The Twin does not import TraCI or SUMOlib. It receives only SensorInterface
DTOs plus V2X messages that were actually delivered by the runner.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict
from typing import Iterable

from .digital_twin import EstimatedVehicleState, time_gap
from .estimation import (
    FilteredVehicleState,
    RadarTargetTracker,
    RelativeTargetState,
    VehicleStateEstimator,
    propagate_predecessor_v2x,
)
from .sensors import VehicleSensorInput
from .v2x import V2XMessage


class PlatoonTwin:
    """Maintain passive raw and filtered measurement-only Twin state.

    The runtime never imports TraCI/SUMOlib.  It owns independent own-state and
    radar-target filters for each truck, but those filters are observational:
    their output is never passed to the CACC controller or vehicle actuation.
    Lane-local ``OdometryMeasurement.position_m`` is intentionally excluded
    from the own-state filter because it is not a continuous route coordinate.
    """

    def __init__(self, truck_ids: tuple[str, ...] | list[str], *, history_len: int = 50,
                 v2x_stale_after_s: float = 0.5, v2x_outage_after_s: float = 2.0) -> None:
        if history_len < 1:
            raise ValueError("history_len must be at least one.")
        if v2x_stale_after_s <= 0.0:
            raise ValueError("v2x_stale_after_s must be positive.")
        if v2x_outage_after_s <= v2x_stale_after_s:
            raise ValueError("v2x_outage_after_s must exceed v2x_stale_after_s.")
        self.truck_ids = tuple(truck_ids)
        self.order = self.truck_ids
        self.history = {truck_id: deque(maxlen=history_len) for truck_id in self.truck_ids}
        self.filtered_history = {truck_id: deque(maxlen=history_len) for truck_id in self.truck_ids}
        self.v2x_stale_after_s = v2x_stale_after_s
        self.v2x_outage_after_s = v2x_outage_after_s
        self._last_message_by_sender: dict[str, V2XMessage] = {}
        self.vehicle_estimators = {truck_id: VehicleStateEstimator(truck_id) for truck_id in self.truck_ids}
        self.radar_trackers = {truck_id: RadarTargetTracker() for truck_id in self.truck_ids}
        self._radar_target_ids = {truck_id: set() for truck_id in self.truck_ids}
        self._radar_states: dict[tuple[str, str], RelativeTargetState] = {}
        self._predecessor_by_vehicle = {
            truck_id: self.order[index - 1] if index else None for index, truck_id in enumerate(self.order)
        }
        self._last_predecessor_v2x_by_receiver: dict[str, object] = {}
        # Delivery time is distinct from generation time. It is the reference
        # for receiver-side freshness: the age must advance on every Twin
        # update even when no later V2X message is delivered.
        self._last_predecessor_v2x_delivery_timestamp_s: dict[str, float] = {}

    def update(self, timestamp_s: float, sensor_inputs: Iterable[VehicleSensorInput], delivered_v2x_messages: Iterable[V2XMessage]) -> None:
        """Update from DTOs only; raw SUMO/legacy state objects are rejected."""

        inputs = tuple(sensor_inputs)
        messages = tuple(delivered_v2x_messages)
        if any(not isinstance(item, VehicleSensorInput) for item in inputs):
            raise TypeError("PlatoonTwin accepts VehicleSensorInput DTOs only.")
        if any(not isinstance(item, V2XMessage) for item in messages):
            raise TypeError("PlatoonTwin accepts delivered V2XMessage DTOs only.")
        for message in messages:
            if message.sender_id in self.history:
                self._last_message_by_sender[message.sender_id] = message
            receiver_id = message.receiver_id
            if receiver_id in self.history and self._predecessor_by_vehicle[receiver_id] == message.sender_id:
                # The runner is the only delivery authority.  This preserves a
                # delayed message as a separate observation; it is not fused
                # into the ego or radar filter in Phase 2.1B.
                self._last_predecessor_v2x_by_receiver[receiver_id] = propagate_predecessor_v2x(
                    message, timestamp_s, stale_after_s=self.v2x_stale_after_s
                )
                self._last_predecessor_v2x_delivery_timestamp_s[receiver_id] = timestamp_s
        for sensor_input in inputs:
            if sensor_input.vehicle_id not in self.history:
                continue
            self.history[sensor_input.vehicle_id].append(self._estimate(timestamp_s, sensor_input))
            self.filtered_history[sensor_input.vehicle_id].append(
                self._update_filtered_state(sensor_input)
            )

    def _estimate(self, timestamp_s: float, sensor_input: VehicleSensorInput) -> EstimatedVehicleState:
        odometry, radar = sensor_input.odometry, sensor_input.radar
        predecessor_id = sensor_input.platoon_predecessor_id
        message = self._last_message_by_sender.get(predecessor_id) if predecessor_id else None
        message_age = None if message is None else max(0.0, timestamp_s - message.sim_time)
        radar_valid = radar is not None and radar.valid
        own_speed = odometry.speed_mps
        gap = radar.range_m if radar_valid else None
        return EstimatedVehicleState(
            timestamp_s, sensor_input.vehicle_id,
            odometry.position_m if odometry.valid else None, own_speed if odometry.valid else None,
            odometry.acceleration_mps2 if odometry.valid else None, predecessor_id,
            radar.sensed_leader_id if radar_valid else None, gap,
            radar.relative_speed_mps if radar_valid else None, time_gap(gap, own_speed or 0.0),
            message_age, not radar_valid, predecessor_id is not None and message is None,
            message_age is not None and message_age > self.v2x_stale_after_s,
            (odometry.acceleration_mps2 or 0.0) < -0.5,
            (odometry.acceleration_mps2 or 0.0) > 0.5,
            "odometry+radar" if radar_valid else "odometry",
            "radar_valid" if radar_valid else "radar_missing",
        )

    def _update_filtered_state(self, sensor_input: VehicleSensorInput) -> FilteredVehicleState:
        """Update one truck's passive filters exclusively from sensor DTOs."""

        vehicle_id = sensor_input.vehicle_id
        odometry = sensor_input.odometry
        # Do not pass odometry.position_m: the current DTO carries lane-local
        # SUMO position, not an independent continuous localization reading.
        # An invalid DTO must not contribute its numerical payload; the filter
        # will perform timestamp-based prediction only when already initialized.
        speed_measurement = odometry.speed_mps if odometry.valid else None
        acceleration_measurement = odometry.acceleration_mps2 if odometry.valid else None
        filtered = self.vehicle_estimators[vehicle_id].update(
            sensor_input.timestamp_s,
            speed_mps=speed_measurement,
            acceleration_mps2=acceleration_measurement,
        )

        tracker = self.radar_trackers[vehicle_id]
        radar = sensor_input.radar
        measured_target_id = None
        if radar is not None and radar.valid and radar.sensed_leader_id:
            measured_target_id = radar.sensed_leader_id
            self._radar_target_ids[vehicle_id].add(measured_target_id)

        # Every known target advances on every sensor timestamp.  A missed
        # detection creates a prediction-only state, never a fake zero radar
        # measurement.  Tracks are retained for the run (conservative policy)
        # and remain visibly aged instead of being silently discarded.
        for target_id in tuple(self._radar_target_ids[vehicle_id]):
            if target_id == measured_target_id:
                self._radar_states[(vehicle_id, target_id)] = tracker.update(
                    target_id,
                    sensor_input.timestamp_s,
                    range_m=radar.range_m,
                    relative_speed_mps=radar.relative_speed_mps,
                )
            else:
                self._radar_states[(vehicle_id, target_id)] = tracker.predict_missing(
                    target_id, sensor_input.timestamp_s
                )
        return filtered

    def state(self, truck_id: str) -> EstimatedVehicleState | None:
        records = self.history.get(truck_id)
        return records[-1] if records else None

    def filtered_state(self, truck_id: str) -> FilteredVehicleState | None:
        """Return the latest passive own-state filter output for a truck."""

        records = self.filtered_history.get(truck_id)
        return records[-1] if records else None

    def radar_state(self, ego_vehicle_id: str, target_vehicle_id: str) -> RelativeTargetState | None:
        """Return an identity-keyed radar state without relating it to V2X."""

        return self._radar_states.get((ego_vehicle_id, target_vehicle_id))

    def gap(self, truck_idx: int) -> float | None:
        if truck_idx <= 0 or truck_idx >= len(self.order):
            return None
        state = self.state(self.order[truck_idx])
        return state.estimated_gap_m if state else None

    def relative_speed(self, truck_idx: int) -> float | None:
        """Follower minus sensed-leader speed; positive means closing."""

        if truck_idx <= 0 or truck_idx >= len(self.order):
            return None
        state = self.state(self.order[truck_idx])
        return state.relative_speed_mps if state else None

    def message_age(self, sender_id: str, timestamp_s: float) -> float | None:
        message = self._last_message_by_sender.get(sender_id)
        return None if message is None else max(0.0, timestamp_s - message.sim_time)

    def as_rows(self, vehicle_ids: Iterable[str]) -> list[dict[str, object]]:
        return [asdict(state) for truck_id in vehicle_ids if (state := self.state(truck_id)) is not None]

    def filtered_as_rows(self, vehicle_ids: Iterable[str]) -> list[dict[str, object]]:
        """Flatten filtered state for a separate CSV; no evaluator fields leak in."""

        rows: list[dict[str, object]] = []
        for truck_id in vehicle_ids:
            state = self.filtered_state(truck_id)
            if state is None:
                continue
            covariance = state.covariance
            rows.append({
                "simulation_time_s": state.timestamp_s,
                "vehicle_id": state.vehicle_id,
                "estimated_position_m": state.estimated_position_m,
                "estimated_speed_mps": state.estimated_speed_mps,
                "estimated_acceleration_mps2": state.estimated_acceleration_mps2,
                "position_variance_m2": None if covariance is None else covariance[0][0],
                "speed_variance_m2ps2": None if covariance is None else covariance[1][1],
                "acceleration_variance_m2ps4": None if covariance is None else covariance[2][2],
                "estimator_status": state.status,
                "measurement_update_mask": "|".join(state.measurement_update_mask),
                "independent_position_measurement_used": False,
                "last_independent_position_measurement_timestamp_s": state.last_independent_position_measurement_timestamp_s,
                "independent_position_measurement_age_s": state.independent_position_measurement_age_s,
                "independent_position_measurement_ever_received": state.independent_position_measurement_ever_received,
                "last_speed_measurement_timestamp_s": state.last_speed_measurement_timestamp_s,
                "speed_measurement_age_s": state.speed_measurement_age_s,
                "speed_measurement_ever_received": state.speed_measurement_ever_received,
                "last_acceleration_measurement_timestamp_s": state.last_acceleration_measurement_timestamp_s,
                "acceleration_measurement_age_s": state.acceleration_measurement_age_s,
                "acceleration_measurement_ever_received": state.acceleration_measurement_ever_received,
                "position_estimate_provenance": state.position_estimate_provenance,
                "speed_estimate_provenance": state.speed_estimate_provenance,
                "acceleration_estimate_provenance": state.acceleration_estimate_provenance,
                "predecessor_v2x_generation_time_s": self._v2x_generation_time(truck_id),
                "predecessor_v2x_age_s": self._v2x_age(truck_id, state.timestamp_s),
                "predecessor_v2x_status": self._v2x_status(truck_id, state.timestamp_s),
            })
        return rows

    def radar_as_rows(self, vehicle_ids: Iterable[str]) -> list[dict[str, object]]:
        """Flatten identity-keyed radar tracks for a separate non-ground-truth CSV."""

        rows: list[dict[str, object]] = []
        for ego_vehicle_id in vehicle_ids:
            for target_id in sorted(self._radar_target_ids.get(ego_vehicle_id, ())):
                state = self.radar_state(ego_vehicle_id, target_id)
                if state is None:
                    continue
                covariance = state.covariance
                rows.append({
                    "simulation_time_s": state.prediction_timestamp_s,
                    "ego_vehicle_id": ego_vehicle_id,
                    "sensed_target_id": state.target_vehicle_id,
                    "estimated_range_m": state.estimated_range_m,
                    "estimated_relative_speed_mps": state.estimated_relative_speed_mps,
                    "range_variance_m2": None if covariance is None else covariance[0][0],
                    "relative_speed_variance_m2ps2": None if covariance is None else covariance[1][1],
                    "track_age_s": state.track_age_s,
                    "last_measurement_timestamp_s": state.last_measurement_timestamp_s,
                    "observed_components": "|".join(state.observed_components),
                    "unobserved_prior_components": "|".join(state.unobserved_prior_components),
                    "track_status": state.status,
                    "last_update_was_measurement": state.last_update_was_measurement,
                })
        return rows

    def _v2x_generation_time(self, receiver_id: str) -> float | None:
        observation = self._last_predecessor_v2x_by_receiver.get(receiver_id)
        return getattr(observation, "generation_timestamp_s", None)

    def _v2x_age(self, receiver_id: str, current_timestamp_s: float) -> float | None:
        """Return receiver-side age since its last accepted V2X delivery."""

        delivered_at = self._last_predecessor_v2x_delivery_timestamp_s.get(receiver_id)
        return None if delivered_at is None else max(0.0, current_timestamp_s - delivered_at)

    def _v2x_status(self, receiver_id: str, current_timestamp_s: float) -> str:
        """Classify current predecessor freshness without fabricating delivery.

        No event is needed to age a known message: the elapsed Twin timestamp
        makes loss/outage visible after an absence of accepted deliveries.
        """

        if self._predecessor_by_vehicle.get(receiver_id) is None:
            return "not_applicable"
        age = self._v2x_age(receiver_id, current_timestamp_s)
        if age is None:
            return "not_available"
        if age > self.v2x_outage_after_s:
            return "outage"
        if age > self.v2x_stale_after_s:
            return "stale"
        return "fresh_propagated"
