"""Seeded ambient-traffic demand for long SAMTRUCK demonstrations.

The platoon route remains fixed.  This helper produces *ambient* passenger
traffic requests only; it never supplies data to the Digital Twin, advisor, or
controller.  A caller resolves each request through SUMO before inserting it.
"""
from __future__ import annotations

from dataclasses import dataclass
import random
from typing import Iterable


@dataclass(frozen=True)
class TrafficRequest:
    """One reproducible request to add a SUMO passenger vehicle."""

    vehicle_id: str
    origin_edge: str
    destination_edge: str
    scheduled_time_s: float


class SeededAmbientTraffic:
    """Generate bounded-rate, reproducible ambient demand.

    ``rate_vehicles_per_minute`` is deliberately capped by the caller-facing
    validation to keep a demonstration responsive.  The local RNG avoids
    changing scenario, sensor, or V2X random streams.
    """

    def __init__(
        self,
        edge_pairs: Iterable[tuple[str, str]],
        *,
        seed: int,
        rate_vehicles_per_minute: float = 18.0,
    ) -> None:
        pairs = tuple((str(origin), str(destination)) for origin, destination in edge_pairs if origin != destination)
        if not pairs:
            raise ValueError("ambient traffic requires at least one distinct origin/destination pair")
        if rate_vehicles_per_minute <= 0.0:
            raise ValueError("ambient traffic rate must be positive")
        self._pairs = pairs
        self._rng = random.Random(seed)
        self._mean_interval_s = 60.0 / rate_vehicles_per_minute
        self._next_time_s = self._rng.uniform(1.0, min(5.0, self._mean_interval_s))
        self._counter = 0

    def due_requests(self, now_s: float) -> list[TrafficRequest]:
        """Return every request due by ``now_s`` and advance the schedule."""
        requests: list[TrafficRequest] = []
        while now_s + 1e-9 >= self._next_time_s:
            origin, destination = self._rng.choice(self._pairs)
            self._counter += 1
            requests.append(TrafficRequest(
                vehicle_id=f"ambient_{self._counter:06d}",
                origin_edge=origin,
                destination_edge=destination,
                scheduled_time_s=self._next_time_s,
            ))
            # Bounded jitter avoids synchronized demand while retaining a
            # predictable long-run rate and exact seed reproducibility.
            self._next_time_s += self._mean_interval_s * self._rng.uniform(0.55, 1.45)
        return requests
