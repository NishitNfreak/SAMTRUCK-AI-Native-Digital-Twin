import unittest

from samtruck.sensors import SensorInterface


class SensorInterfaceTests(unittest.TestCase):
    def test_identical_seed_reproduces_sensor_noise(self):
        first = SensorInterface(0.5, 0.2, 0.4, seed=8)
        second = SensorInterface(0.5, 0.2, 0.4, seed=8)
        kwargs = dict(
            timestamp_s=1.0, vehicle_id="samtruck_follower_1", platoon_predecessor_id="samtruck_leader",
            position_m=12.0, speed_mps=20.0, acceleration_mps2=0.3,
            sensed_leader_id="samtruck_leader", radar_range_m=25.0, radar_relative_speed_mps=1.0,
        )
        self.assertEqual(first.make_input(**kwargs), second.make_input(**kwargs))

    def test_dropout_is_missing_not_zero(self):
        sensor = SensorInterface(radar_dropout_probability=1.0, seed=3)
        measurement = sensor.make_input(
            timestamp_s=1.0, vehicle_id="samtruck_follower_1", platoon_predecessor_id="samtruck_leader",
            position_m=12.0, speed_mps=20.0, acceleration_mps2=0.0,
            sensed_leader_id="samtruck_leader", radar_range_m=25.0, radar_relative_speed_mps=1.0,
        )
        self.assertFalse(measurement.radar.valid)
        self.assertIsNone(measurement.radar.range_m)
