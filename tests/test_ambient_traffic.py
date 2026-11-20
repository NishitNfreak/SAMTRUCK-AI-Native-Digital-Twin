"""Deterministic unit checks for opt-in ambient traffic demand."""
from __future__ import annotations

import unittest

from samtruck.ambient_traffic import SeededAmbientTraffic


class SeededAmbientTrafficTests(unittest.TestCase):
    def test_same_seed_produces_same_requests(self) -> None:
        pairs = [("a", "b"), ("c", "d")]
        first = SeededAmbientTraffic(pairs, seed=17, rate_vehicles_per_minute=30.0)
        second = SeededAmbientTraffic(pairs, seed=17, rate_vehicles_per_minute=30.0)
        self.assertEqual(first.due_requests(60.0), second.due_requests(60.0))

    def test_requests_have_distinct_ids_and_valid_pairs(self) -> None:
        generator = SeededAmbientTraffic([("a", "b")], seed=5, rate_vehicles_per_minute=60.0)
        requests = generator.due_requests(20.0)
        self.assertGreater(len(requests), 1)
        self.assertEqual(len({request.vehicle_id for request in requests}), len(requests))
        self.assertTrue(all((request.origin_edge, request.destination_edge) == ("a", "b") for request in requests))

    def test_invalid_configuration_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            SeededAmbientTraffic([], seed=1)
        with self.assertRaises(ValueError):
            SeededAmbientTraffic([("a", "b")], seed=1, rate_vehicles_per_minute=0.0)


if __name__ == "__main__":
    unittest.main()
