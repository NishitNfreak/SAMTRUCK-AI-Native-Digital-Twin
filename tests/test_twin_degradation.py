"""Phase 2.2C deterministic Twin-only degradation tests."""

from __future__ import annotations

import unittest

from run_all_scenarios import twin_v2x_state
from samtruck.sensors import SensorInterface
from samtruck.twin_degradation import TwinDegradationSchedule
from samtruck.twin_runtime import PlatoonTwin


def measured_input(timestamp_s: float = 0.0):
    return SensorInterface(seed=7).make_input(
        timestamp_s=timestamp_s,
        vehicle_id="truck_1",
        platoon_predecessor_id="truck_0",
        position_m=10.0,
        speed_mps=12.0,
        acceleration_mps2=1.0,
        sensed_leader_id="truck_0",
        radar_range_m=20.0,
        radar_relative_speed_mps=2.0,
    )


class TwinDegradationScheduleTests(unittest.TestCase):
    def test_radar_dropout_is_explicit_missing_and_windowed(self) -> None:
        schedule = TwinDegradationSchedule([{"type": "twin_radar_dropout", "at_s": 2.0, "duration_s": 1.0}])
        before = schedule.apply_sensor_inputs(1.9, [measured_input(1.9)])[0]
        during = schedule.apply_sensor_inputs(2.0, [measured_input(2.0)])[0]
        after = schedule.apply_sensor_inputs(3.0, [measured_input(3.0)])[0]
        self.assertTrue(before.radar.valid)
        self.assertFalse(during.radar.valid)
        self.assertIsNone(during.radar.range_m)
        self.assertIsNone(during.radar.relative_speed_mps)
        self.assertEqual(during.radar.status, "scheduled_dropout")
        self.assertTrue(after.radar.valid)
        self.assertEqual(schedule.statistics["radar_measurements_suppressed"], 1)

    def test_odometry_loss_preserves_timestamps_and_creates_prediction_only_twin_state(self) -> None:
        schedule = TwinDegradationSchedule([{"type": "twin_odometry_dropout", "at_s": 1.0, "duration_s": 2.0}])
        twin = PlatoonTwin(("truck_0", "truck_1", "truck_2"))
        twin.update(0.0, [measured_input(0.0)], [])
        dropped = schedule.apply_sensor_inputs(1.0, [measured_input(1.0)])[0]
        self.assertFalse(dropped.odometry.valid)
        self.assertIsNone(dropped.odometry.speed_mps)
        self.assertIsNone(dropped.odometry.acceleration_mps2)
        twin.update(1.0, [dropped], [])
        state = twin.filtered_state("truck_1")
        self.assertEqual(state.status, "predicted_only")
        self.assertEqual(state.last_speed_measurement_timestamp_s, 0.0)
        self.assertEqual(state.last_acceleration_measurement_timestamp_s, 0.0)
        self.assertEqual(state.speed_measurement_age_s, 1.0)
        self.assertEqual(state.acceleration_measurement_age_s, 1.0)
        recovered = schedule.apply_sensor_inputs(3.0, [measured_input(3.0)])[0]
        twin.update(3.0, [recovered], [])
        state = twin.filtered_state("truck_1")
        self.assertEqual(state.speed_measurement_age_s, 0.0)
        self.assertEqual(state.acceleration_measurement_age_s, 0.0)
        self.assertEqual(state.speed_estimate_provenance, "measurement_updated")

    def test_twin_v2x_schedule_isolated_from_legacy_conditions_and_recovers(self) -> None:
        schedule = TwinDegradationSchedule([
            {"type": "twin_v2x_latency", "at_s": 1.0, "duration_s": 2.0, "latency_ms": 600.0},
            {"type": "twin_v2x_packet_loss", "at_s": 1.0, "duration_s": 2.0, "loss_rate": 0.4},
            {"type": "twin_v2x_outage", "at_s": 4.0, "duration_s": 1.0},
        ])
        self.assertTrue(schedule.has_v2x_events)
        self.assertEqual(twin_v2x_state(1.5, schedule, 40.0, 0.0), (600.0, 0.4))
        self.assertEqual(twin_v2x_state(4.5, schedule, 40.0, 0.0), (None, 0.0))
        self.assertEqual(twin_v2x_state(5.0, schedule, 40.0, 0.0), (40.0, 0.0))

    def test_no_twin_v2x_events_preserves_legacy_v2x_values(self) -> None:
        schedule = TwinDegradationSchedule([{"type": "twin_radar_dropout", "at_s": 1.0, "duration_s": 1.0}])
        self.assertFalse(schedule.has_v2x_events)
        self.assertEqual(twin_v2x_state(1.5, schedule, None, 0.3), (None, 0.3))

    def test_invalid_schedule_configuration_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            TwinDegradationSchedule([{"type": "unknown", "at_s": 0.0, "duration_s": 1.0}])
        with self.assertRaises(ValueError):
            TwinDegradationSchedule([{"type": "twin_v2x_packet_loss", "loss_rate": 1.1}])


if __name__ == "__main__":
    unittest.main()
