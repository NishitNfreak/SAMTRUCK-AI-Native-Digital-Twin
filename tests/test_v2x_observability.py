"""Regression tests for dynamic predecessor-V2X freshness observability."""

from __future__ import annotations

import unittest

from samtruck.sensors import OdometryMeasurement, VehicleSensorInput
from samtruck.twin_runtime import PlatoonTwin
from samtruck.v2x import V2XMessage


TRUCKS = ("truck_0", "truck_1", "truck_2")


def input_for(vehicle_id: str, timestamp_s: float) -> VehicleSensorInput:
    index = TRUCKS.index(vehicle_id)
    odometry = OdometryMeasurement(
        timestamp_s, vehicle_id, 0.0, 10.0, 0.0, True, "measured"
    )
    return VehicleSensorInput(timestamp_s, vehicle_id, TRUCKS[index - 1] if index else None, odometry, None)


def delivered_to_follower(timestamp_s: float = 0.0) -> V2XMessage:
    return V2XMessage("truck_0", timestamp_s, 10.0, 0.0, False, "truck_1", 1)


class V2XObservabilityTests(unittest.TestCase):
    def _row_after(self, twin: PlatoonTwin, timestamp_s: float, messages: list[V2XMessage]) -> dict[str, object]:
        twin.update(timestamp_s, [input_for("truck_1", timestamp_s)], messages)
        return twin.filtered_as_rows(["truck_1"])[0]

    def test_age_advances_without_new_delivery(self) -> None:
        twin = PlatoonTwin(TRUCKS, v2x_stale_after_s=0.5, v2x_outage_after_s=2.0)
        self._row_after(twin, 0.0, [delivered_to_follower()])
        row = self._row_after(twin, 0.4, [])
        self.assertAlmostEqual(row["predecessor_v2x_age_s"], 0.4)
        self.assertEqual(row["predecessor_v2x_status"], "fresh_propagated")

    def test_status_progresses_from_fresh_to_stale_to_outage(self) -> None:
        twin = PlatoonTwin(TRUCKS, v2x_stale_after_s=0.5, v2x_outage_after_s=2.0)
        fresh = self._row_after(twin, 0.0, [delivered_to_follower()])
        stale = self._row_after(twin, 0.6, [])
        outage = self._row_after(twin, 2.1, [])
        self.assertEqual(fresh["predecessor_v2x_status"], "fresh_propagated")
        self.assertEqual(stale["predecessor_v2x_status"], "stale")
        self.assertAlmostEqual(stale["predecessor_v2x_age_s"], 0.6)
        self.assertEqual(outage["predecessor_v2x_status"], "outage")
        self.assertAlmostEqual(outage["predecessor_v2x_age_s"], 2.1)

    def test_never_delivered_and_leader_are_explicit(self) -> None:
        twin = PlatoonTwin(TRUCKS, v2x_stale_after_s=0.5, v2x_outage_after_s=2.0)
        twin.update(0.0, [input_for("truck_0", 0.0), input_for("truck_1", 0.0)], [])
        leader, follower = twin.filtered_as_rows(["truck_0", "truck_1"])
        self.assertEqual(leader["predecessor_v2x_status"], "not_applicable")
        self.assertIsNone(leader["predecessor_v2x_age_s"])
        self.assertEqual(follower["predecessor_v2x_status"], "not_available")
        self.assertIsNone(follower["predecessor_v2x_age_s"])


if __name__ == "__main__":
    unittest.main()
