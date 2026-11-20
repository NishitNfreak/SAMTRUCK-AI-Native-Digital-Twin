import unittest

from samtruck.v2x import PendingV2XChannel, V2XChannel, V2XMessage


class V2XChannelTests(unittest.TestCase):
    def test_delay_holds_message_until_enough_steps_pass(self):
        channel = V2XChannel(delay_steps=1)
        message = V2XMessage("truck_0", 1.0, 20.0, 0.0, False)

        channel.send("truck_1", message)
        self.assertIsNone(channel.receive("truck_1"))
        channel.send("truck_1", message)
        self.assertEqual(channel.receive("truck_1"), message)

    def test_packet_loss_drops_configured_message(self):
        channel = V2XChannel(delay_steps=0, packet_loss_every=2)
        first = V2XMessage("truck_0", 1.0, 20.0, 0.0, False)
        second = V2XMessage("truck_0", 2.0, 21.0, 0.1, False)

        channel.send("truck_1", first)
        channel.send("truck_1", second)

        self.assertEqual(channel.receive("truck_1"), first)
        self.assertIsNone(channel.receive("truck_1"))


class PendingV2XChannelTests(unittest.TestCase):
    def setUp(self):
        self.channel = PendingV2XChannel(
            ("leader", "follower_1", "follower_2"),
            {"leader": "follower_1", "follower_1": "follower_2"},
            stale_after_s=1.0,
            seed=9,
        )

    @staticmethod
    def message(sender="leader", receiver="follower_1", sequence=1, time_s=1.0):
        return V2XMessage(sender, time_s, 20.0, 0.0, False, receiver, sequence)

    def test_message_waits_until_scheduled_delivery(self):
        self.channel.enqueue(self.message(), latency_ms=200.0)
        delivered, events = self.channel.deliver_ready(1.199)
        self.assertEqual(delivered, [])
        self.assertEqual(events, [])
        delivered, events = self.channel.deliver_ready(1.2)
        self.assertEqual(len(delivered), 1)
        self.assertEqual(events[0].status, "delivered")

    def test_packet_loss_prevents_delivery(self):
        self.channel.enqueue(self.message(), latency_ms=0.0, loss_probability=1.0)
        delivered, events = self.channel.deliver_ready(1.0)
        self.assertEqual(delivered, [])
        self.assertEqual(events[0].status, "dropped")
        self.assertEqual(events[0].reason, "packet_loss")

    def test_duplicate_and_older_sequences_are_rejected(self):
        self.channel.enqueue(self.message(sequence=2), latency_ms=0.0)
        self.channel.deliver_ready(1.0)
        self.channel.enqueue(self.message(sequence=2), latency_ms=0.0)
        self.channel.enqueue(self.message(sequence=1), latency_ms=0.0)
        delivered, events = self.channel.deliver_ready(1.0)
        self.assertEqual(delivered, [])
        self.assertEqual([event.reason for event in events], ["duplicate_or_out_of_order_sequence"] * 2)

    def test_wrong_receiver_future_and_stale_messages_are_rejected(self):
        wrong = self.message(receiver="follower_2")
        self.channel.enqueue(wrong, latency_ms=0.0)
        _, events = self.channel.deliver_ready(1.0)
        self.assertEqual(events[0].reason, "wrong_receiver")
        self.assertEqual(self.channel.validation_reason(self.message(time_s=2.0), 1.0), "invalid_or_future_timestamp")
        self.channel.enqueue(self.message(sequence=3), latency_ms=2000.0)
        _, events = self.channel.deliver_ready(3.0)
        self.assertEqual(events[0].reason, "stale_message")

    def test_seeded_packet_loss_is_reproducible(self):
        outcomes = []
        for _ in range(2):
            channel = PendingV2XChannel(("leader", "follower_1"), {"leader": "follower_1"}, seed=22)
            for sequence in range(1, 6):
                channel.enqueue(self.message(sequence=sequence), latency_ms=0.0, loss_probability=0.5)
            _, events = channel.deliver_ready(1.0)
            outcomes.append([(event.status, event.reason) for event in events])
        self.assertEqual(outcomes[0], outcomes[1])


if __name__ == "__main__":
    unittest.main()
