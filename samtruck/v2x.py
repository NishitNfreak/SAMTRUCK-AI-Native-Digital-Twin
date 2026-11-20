from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, replace
import random


@dataclass(frozen=True)
class V2XMessage:
    sender_id: str
    sim_time: float
    speed: float
    acceleration: float
    braking: bool
    receiver_id: str | None = None
    sequence: int | None = None
    scheduled_delivery_time_s: float | None = None


@dataclass(frozen=True)
class V2XDeliveryEvent:
    """Communication outcome for the Twin-only measurement channel."""

    message: V2XMessage
    status: str
    delivery_time_s: float | None
    reason: str


class PendingV2XChannel:
    """Timestamped, validated V2X queue used only by the passive Twin branch.

    The active CACC communication behavior remains in the existing scenario
    runner. This channel has one authoritative pending queue for messages sent
    to the Digital Twin, so queued messages cannot be observed prematurely.
    """

    _EPSILON_S = 1e-9

    def __init__(
        self,
        known_vehicle_ids: tuple[str, ...] | list[str],
        expected_receivers: dict[str, str],
        stale_after_s: float = 1.0,
        seed: int = 42,
    ) -> None:
        self.known_vehicle_ids = set(known_vehicle_ids)
        self.expected_receivers = dict(expected_receivers)
        self.stale_after_s = stale_after_s
        self._rng = random.Random(seed)
        self._pending: deque[tuple[V2XMessage, float]] = deque()
        self._last_sequence: dict[tuple[str, str], int] = {}
        self.statistics = {
            "generated": 0, "delivered": 0, "dropped": 0,
            "rejected": 0, "duplicates": 0, "stale": 0,
        }

    def enqueue(self, message: V2XMessage, latency_ms: float | None, loss_probability: float = 0.0) -> list[V2XDeliveryEvent]:
        """Capture payload at generation and schedule it; outage is an immediate drop."""

        self.statistics["generated"] += 1
        if latency_ms is None:
            self.statistics["dropped"] += 1
            return [V2XDeliveryEvent(message, "dropped", None, "communication_outage")]
        if latency_ms < 0 or not 0.0 <= loss_probability <= 1.0:
            raise ValueError("Latency must be non-negative and loss probability must be in [0, 1].")
        scheduled = message.sim_time + latency_ms / 1000.0
        scheduled_message = replace(message, scheduled_delivery_time_s=scheduled)
        self._pending.append((scheduled_message, loss_probability))
        return []

    def deliver_ready(self, now_s: float) -> tuple[list[V2XMessage], list[V2XDeliveryEvent]]:
        """Deliver only messages whose scheduled simulator time has been reached."""

        delivered: list[V2XMessage] = []
        events: list[V2XDeliveryEvent] = []
        remaining: deque[tuple[V2XMessage, float]] = deque()
        while self._pending:
            message, loss_probability = self._pending.popleft()
            scheduled = message.scheduled_delivery_time_s
            if scheduled is None or scheduled > now_s + self._EPSILON_S:
                remaining.append((message, loss_probability))
                continue
            if self._rng.random() < loss_probability:
                self.statistics["dropped"] += 1
                events.append(V2XDeliveryEvent(message, "dropped", None, "packet_loss"))
                continue
            reason = self.validation_reason(message, now_s)
            if reason is not None:
                self.statistics["rejected"] += 1
                if reason == "duplicate_or_out_of_order_sequence":
                    self.statistics["duplicates"] += 1
                if reason == "stale_message":
                    self.statistics["stale"] += 1
                events.append(V2XDeliveryEvent(message, "rejected", None, reason))
                continue
            key = (message.sender_id, message.receiver_id or "")
            self._last_sequence[key] = message.sequence if message.sequence is not None else -1
            self.statistics["delivered"] += 1
            delivered.append(message)
            events.append(V2XDeliveryEvent(message, "delivered", now_s, "accepted"))
        self._pending = remaining
        return delivered, events

    def validation_reason(self, message: V2XMessage, now_s: float) -> str | None:
        """Return a rejection reason, or ``None`` when a message is acceptable."""
        if message.sender_id not in self.known_vehicle_ids:
            return "unknown_sender"
        if message.receiver_id not in self.known_vehicle_ids:
            return "unknown_receiver"
        if self.expected_receivers.get(message.sender_id) != message.receiver_id:
            return "wrong_receiver"
        if message.sim_time < 0 or message.sim_time > now_s + self._EPSILON_S:
            return "invalid_or_future_timestamp"
        if now_s - message.sim_time > self.stale_after_s + self._EPSILON_S:
            return "stale_message"
        if message.sequence is None:
            return "missing_sequence"
        key = (message.sender_id, message.receiver_id)
        if message.sequence <= self._last_sequence.get(key, -1):
            return "duplicate_or_out_of_order_sequence"
        return None


class V2XChannel:
    """Simple delayed/lossy V2V channel for explainable simulation demos."""

    def __init__(self, delay_steps: int = 0, packet_loss_every: int = 0) -> None:
        self.delay_steps = max(0, delay_steps)
        self.packet_loss_every = max(0, packet_loss_every)
        self._queues: dict[str, deque[V2XMessage]] = defaultdict(deque)
        self._sent_count = 0

    def send(self, receiver_id: str, message: V2XMessage) -> None:
        self._sent_count += 1
        if self.packet_loss_every and self._sent_count % self.packet_loss_every == 0:
            return
        queue = self._queues[receiver_id]
        queue.append(message)
        while len(queue) > self.delay_steps + 1:
            queue.popleft()

    def receive(self, receiver_id: str) -> V2XMessage | None:
        queue = self._queues[receiver_id]
        if len(queue) <= self.delay_steps:
            return None
        return queue.popleft()
