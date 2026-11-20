"""Phase 2.1B passive estimator integration tests (no SUMO required)."""

from __future__ import annotations

import ast
import inspect
import unittest

import samtruck.twin_runtime as twin_runtime
from samtruck.sensors import OdometryMeasurement, RadarMeasurement, VehicleSensorInput
from samtruck.twin_runtime import PlatoonTwin
from samtruck.v2x import PendingV2XChannel, V2XMessage


TRUCKS = ("truck_0", "truck_1", "truck_2")


def sensor_input(
    vehicle_id: str,
    timestamp_s: float,
    *,
    lane_position_m: float = 999.0,
    speed_mps: float | None = 10.0,
    acceleration_mps2: float | None = 0.0,
    radar_target_id: str | None = None,
    radar_range_m: float | None = None,
    radar_relative_speed_mps: float | None = None,
    radar_valid: bool = True,
    odometry_valid: bool | None = None,
) -> VehicleSensorInput:
    index = TRUCKS.index(vehicle_id)
    predecessor = TRUCKS[index - 1] if index else None
    odometry = OdometryMeasurement(
        timestamp_s, vehicle_id, lane_position_m, speed_mps, acceleration_mps2,
        speed_mps is not None if odometry_valid is None else odometry_valid,
        "measured" if (speed_mps is not None if odometry_valid is None else odometry_valid) else "invalid",
    )
    radar = None if radar_target_id is None else RadarMeasurement(
        timestamp_s, vehicle_id, radar_target_id, radar_range_m, radar_relative_speed_mps,
        radar_valid, "measured" if radar_valid else "dropout",
    )
    return VehicleSensorInput(timestamp_s, vehicle_id, predecessor, odometry, radar)


class PassiveEstimatorIntegrationTests(unittest.TestCase):
    def test_each_truck_has_independent_estimator_and_update_does_not_leak(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        self.assertIsNot(twin.vehicle_estimators["truck_0"], twin.vehicle_estimators["truck_1"])
        twin.update(0.0, [sensor_input("truck_0", 0.0, speed_mps=8.0)], [])
        self.assertIsNotNone(twin.filtered_state("truck_0"))
        self.assertIsNone(twin.filtered_state("truck_1"))

    def test_sensor_timestamp_drives_prediction_interval(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        twin.update(0.0, [sensor_input("truck_0", 0.0, speed_mps=10.0)], [])
        twin.update(2.0, [sensor_input("truck_0", 2.0, speed_mps=None, acceleration_mps2=None)], [])
        state = twin.filtered_state("truck_0")
        self.assertAlmostEqual(state.estimated_position_m, 20.0, places=6)
        self.assertEqual(state.status, "degraded_long_gap_predicted")

    def test_lane_local_position_is_not_an_independent_position_measurement(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        twin.update(0.0, [sensor_input("truck_0", 0.0, lane_position_m=9999.0, speed_mps=10.0)], [])
        filtered = twin.filtered_state("truck_0")
        self.assertEqual(filtered.estimated_position_m, 0.0)
        row = twin.filtered_as_rows(["truck_0"])[0]
        self.assertFalse(row["independent_position_measurement_used"])
        self.assertNotIn("lane_position_m", row)
        self.assertIsNone(row["last_independent_position_measurement_timestamp_s"])
        self.assertIsNone(row["independent_position_measurement_age_s"])
        self.assertFalse(row["independent_position_measurement_ever_received"])
        self.assertEqual(row["position_estimate_provenance"], "not_meaningfully_initialized")

    def test_missing_speed_and_acceleration_are_prediction_only_not_crashes(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        twin.update(0.0, [sensor_input("truck_0", 0.0, speed_mps=None, acceleration_mps2=None)], [])
        self.assertEqual(twin.filtered_state("truck_0").status, "uninitialized_missing_speed")
        twin.update(1.0, [sensor_input("truck_0", 1.0, speed_mps=5.0, acceleration_mps2=None)], [])
        twin.update(1.2, [sensor_input("truck_0", 1.2, speed_mps=None, acceleration_mps2=None)], [])
        self.assertEqual(twin.filtered_state("truck_0").status, "predicted_only")

    def test_invalid_numerical_odometry_is_prediction_only_after_initialization(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        twin.update(0.0, [sensor_input("truck_0", 0.0, speed_mps=10.0, acceleration_mps2=0.0)], [])
        twin.update(1.0, [sensor_input(
            "truck_0", 1.0, speed_mps=100.0, acceleration_mps2=-9.0, odometry_valid=False
        )], [])
        state = twin.filtered_state("truck_0")
        self.assertEqual(state.status, "predicted_only")
        self.assertEqual(state.measurement_update_mask, ())
        self.assertAlmostEqual(state.estimated_position_m, 10.0, places=6)
        self.assertAlmostEqual(state.estimated_speed_mps, 10.0, places=6)
        self.assertEqual(state.last_speed_measurement_timestamp_s, 0.0)
        self.assertEqual(state.last_acceleration_measurement_timestamp_s, 0.0)
        self.assertEqual(state.speed_measurement_age_s, 1.0)
        self.assertEqual(state.acceleration_measurement_age_s, 1.0)

    def test_filtered_row_includes_component_freshness_without_changing_existing_fields(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        twin.update(0.0, [sensor_input("truck_0", 0.0, speed_mps=10.0, acceleration_mps2=1.0)], [])
        twin.update(1.0, [sensor_input("truck_0", 1.0, speed_mps=11.0, acceleration_mps2=None)], [])
        row = twin.filtered_as_rows(["truck_0"])[0]
        for field in (
            "last_speed_measurement_timestamp_s", "speed_measurement_age_s", "speed_measurement_ever_received",
            "last_acceleration_measurement_timestamp_s", "acceleration_measurement_age_s",
            "acceleration_measurement_ever_received", "speed_estimate_provenance",
            "acceleration_estimate_provenance", "predecessor_v2x_status", "measurement_update_mask",
        ):
            self.assertIn(field, row)
        self.assertEqual(row["measurement_update_mask"], "speed")
        self.assertEqual(row["last_speed_measurement_timestamp_s"], 1.0)
        self.assertEqual(row["speed_measurement_age_s"], 0.0)
        self.assertEqual(row["last_acceleration_measurement_timestamp_s"], 0.0)
        self.assertEqual(row["acceleration_measurement_age_s"], 1.0)
        self.assertEqual(row["speed_estimate_provenance"], "measurement_updated")
        self.assertEqual(row["acceleration_estimate_provenance"], "prediction_only")

    def test_invalid_initial_odometry_does_not_initialize_filter(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        twin.update(0.0, [sensor_input(
            "truck_0", 0.0, speed_mps=99.0, acceleration_mps2=8.0, odometry_valid=False
        )], [])
        state = twin.filtered_state("truck_0")
        self.assertFalse(state.initialized)
        self.assertEqual(state.status, "uninitialized_missing_speed")

    def test_valid_odometry_still_updates_filter(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        twin.update(0.0, [sensor_input("truck_0", 0.0, speed_mps=5.0, acceleration_mps2=0.0)], [])
        twin.update(1.0, [sensor_input("truck_0", 1.0, speed_mps=12.0, acceleration_mps2=2.0)], [])
        state = twin.filtered_state("truck_0")
        self.assertEqual(state.status, "updated")
        self.assertEqual(state.measurement_update_mask, ("speed", "acceleration"))
        self.assertGreater(state.estimated_speed_mps, 5.0)

    def test_radar_tracks_are_identity_keyed_and_cut_in_is_not_predecessor(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        twin.update(0.0, [sensor_input(
            "truck_1", 0.0, radar_target_id="cut_in_vehicle", radar_range_m=20.0, radar_relative_speed_mps=2.0
        )], [])
        self.assertIsNotNone(twin.radar_state("truck_1", "cut_in_vehicle"))
        self.assertIsNone(twin.radar_state("truck_1", "truck_0"))
        rows = twin.radar_as_rows(["truck_1"])
        self.assertEqual(rows[0]["sensed_target_id"], "cut_in_vehicle")

    def test_missing_radar_produces_prediction_with_age(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        twin.update(0.0, [sensor_input(
            "truck_1", 0.0, radar_target_id="truck_0", radar_range_m=30.0, radar_relative_speed_mps=1.0
        )], [])
        twin.update(1.1, [sensor_input("truck_1", 1.1)], [])
        radar = twin.radar_state("truck_1", "truck_0")
        self.assertEqual(radar.status, "degraded_long_gap_missing")
        self.assertFalse(radar.last_update_was_measurement)
        self.assertAlmostEqual(radar.last_measurement_timestamp_s, 0.0)
        self.assertAlmostEqual(radar.track_age_s, 1.1)

    def test_only_delivered_matching_predecessor_v2x_is_interpreted(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        channel = PendingV2XChannel(TRUCKS, {"truck_0": "truck_1", "truck_1": "truck_2"}, seed=4)
        message = V2XMessage("truck_0", 1.0, 10.0, 0.0, False, "truck_1", 1)
        channel.enqueue(message, latency_ms=200.0)
        early, _ = channel.deliver_ready(1.1)
        twin.update(1.1, [sensor_input("truck_1", 1.1)], early)
        self.assertEqual(twin.filtered_as_rows(["truck_1"])[0]["predecessor_v2x_status"], "not_available")
        delivered, _ = channel.deliver_ready(1.2)
        twin.update(1.2, [sensor_input("truck_1", 1.2)], delivered)
        row = twin.filtered_as_rows(["truck_1"])[0]
        self.assertEqual(row["predecessor_v2x_generation_time_s"], 1.0)
        # Filtered-row freshness is receiver-side age since the accepted
        # delivery at this update, not the original generation delay.
        self.assertAlmostEqual(row["predecessor_v2x_age_s"], 0.0)
        self.assertEqual(row["predecessor_v2x_status"], "fresh_propagated")

    def test_raw_twin_rows_remain_separate_from_filtered_rows(self) -> None:
        twin = PlatoonTwin(TRUCKS)
        twin.update(0.0, [sensor_input("truck_1", 0.0)], [])
        raw = twin.as_rows(["truck_1"])[0]
        filtered = twin.filtered_as_rows(["truck_1"])[0]
        self.assertIn("estimated_gap_m", raw)
        self.assertNotIn("position_variance_m2", raw)
        self.assertIn("position_variance_m2", filtered)
        self.assertNotIn("actual_gap_m", filtered)
        self.assertNotIn("lane_position_m", filtered)

    def test_twin_source_has_no_ground_truth_or_controller_dependency(self) -> None:
        source = inspect.getsource(twin_runtime)
        imports = []
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.Import):
                imports.extend(alias.name.lower() for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module.lower())
        for forbidden in ("traci", "sumolib", "controller", "evaluation"):
            self.assertFalse(any(name == forbidden or name.startswith(f"{forbidden}.") for name in imports))


if __name__ == "__main__":
    unittest.main()
