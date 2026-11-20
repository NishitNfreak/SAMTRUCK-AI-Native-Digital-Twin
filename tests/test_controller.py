import unittest

from samtruck.controller import CACCController, ResilienceState


class CACCControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = CACCController(desired_time_gap=1.2, min_gap=4.0, max_speed=30.0)

    def test_ctg_gap(self):
        self.assertAlmostEqual(self.controller.desired_gap(20.0, ResilienceState.CACC), 28.0)

    def test_blackout_uses_acc_fallback(self):
        command = self.controller.command(18.0, 18.0, 30.0, None, None, None)
        self.assertEqual(command.state, ResilienceState.ACC_FALLBACK)
        self.assertEqual(command.reason, "radar_acc_fallback")

    def test_critical_ttc_uses_aeb(self):
        command = self.controller.command(20.0, 10.0, 10.0, 10.0, -4.5, 40.0)
        self.assertEqual(command.state, ResilienceState.EMERGENCY_BRAKE)
        self.assertEqual(command.reason, "radar_aeb")

    def test_identical_inputs_produce_identical_baseline_command(self):
        inputs = (18.0, 17.0, 32.0, 17.0, -0.5, 40.0)
        first = self.controller.command(*inputs)
        second = self.controller.command(*inputs)
        self.assertEqual(first, second)
        self.assertEqual(first.state, ResilienceState.CACC)


if __name__ == "__main__":
    unittest.main()
