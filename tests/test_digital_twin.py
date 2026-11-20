import unittest

from samtruck.digital_twin import VehicleState, time_gap
from samtruck.sensors import OdometryMeasurement, RadarMeasurement, VehicleSensorInput
from samtruck.twin_runtime import PlatoonTwin
from samtruck.v2x import V2XMessage


class DigitalTwinTests(unittest.TestCase):
    def test_time_gap_is_gap_divided_by_speed(self):
        self.assertAlmostEqual(time_gap(30.0, 15.0), 2.0)

    def test_time_gap_is_none_when_stopped(self):
        self.assertIsNone(time_gap(30.0, 0.0))

    def test_braking_flag_uses_negative_acceleration(self):
        state = VehicleState(
            sim_time=1.0,
            vehicle_id="truck_0",
            position=10.0,
            speed=20.0,
            acceleration=-1.0,
            lane=0,
            gap=None,
            time_gap=None,
            leader_id=None,
        )

        self.assertTrue(state.braking)

    def test_history_updates_and_is_bounded(self):
        twin = PlatoonTwin(("truck_0",), history_len=2)
        for timestamp in (0.1, 0.2, 0.3):
            twin.update(timestamp, [self._input("truck_0", timestamp)], [])
        self.assertEqual(len(twin.history["truck_0"]), 2)
        self.assertAlmostEqual(twin.state("truck_0").timestamp_s, 0.3)

    def test_relative_speed_sign_is_positive_when_follower_closes(self):
        twin = PlatoonTwin(("truck_0", "truck_1"))
        twin.update(1.0, [self._input("truck_0", 1.0), self._input("truck_1", 1.0, 25.0, 3.0)], [])
        self.assertAlmostEqual(twin.gap(1), 25.0)
        self.assertAlmostEqual(twin.relative_speed(1), 3.0)

    def test_missing_radar_and_stale_v2x_are_explicit(self):
        twin = PlatoonTwin(("truck_0", "truck_1"), v2x_stale_after_s=0.5)
        message = V2XMessage("truck_0", 1.0, 20.0, 0.0, False, "truck_1", 1)
        twin.update(1.0, [self._input("truck_0", 1.0), self._input("truck_1", 1.0, None, None)], [message])
        self.assertTrue(twin.state("truck_1").radar_missing)
        self.assertFalse(twin.state("truck_1").v2x_stale)
        twin.update(1.7, [self._input("truck_0", 1.7), self._input("truck_1", 1.7, None, None)], [])
        self.assertTrue(twin.state("truck_1").v2x_stale)

    @staticmethod
    def _input(vehicle_id, timestamp, radar_gap=20.0, relative_speed=0.0):
        radar = None if radar_gap is None else RadarMeasurement(
            timestamp, vehicle_id, "samtruck_leader", radar_gap, relative_speed, True, "measured"
        )
        predecessor = None if vehicle_id == "truck_0" else "truck_0"
        return VehicleSensorInput(
            timestamp, vehicle_id, predecessor,
            OdometryMeasurement(timestamp, vehicle_id, 10.0, 20.0, 0.0, True, "measured"), radar,
        )


if __name__ == "__main__":
    unittest.main()
