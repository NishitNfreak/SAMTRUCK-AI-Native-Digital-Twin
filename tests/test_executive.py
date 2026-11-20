"""Deterministic tests for the opt-in bounded active executive."""

from __future__ import annotations

import unittest

from samtruck.controller import CACCController, ResilienceState
from samtruck.executive import ActiveExecutive
from samtruck.twin_advisor import AdvisorDecision


def decision(time_s: float, vehicle_id: str, command: str, code: str = "test") -> AdvisorDecision:
    return AdvisorDecision(time_s, vehicle_id, code, "warn", [], [], "test", command, {}, "TEST")


class ActiveExecutiveTests(unittest.TestCase):
    def setUp(self) -> None:
        self.executive = ActiveExecutive(["truck_1"])

    def test_gap_target_is_slew_limited(self) -> None:
        self.executive.process_decisions([decision(0.0, "truck_1", "HOLD_BASELINE")])
        action = self.executive.process_decisions([decision(1.0, "truck_1", "INCREASE_GAP_1_5S")])[0]
        self.assertEqual(action.target_time_gap_s, 1.5)
        self.assertAlmostEqual(action.applied_time_gap_s, 1.3)
        self.assertTrue(action.v2v_enabled)

    def test_acc_fallback_and_recovery_hold(self) -> None:
        self.executive.process_decisions([decision(0.0, "truck_1", "FALLBACK_ACC_WIDE_GAP")])
        held = self.executive.process_decisions([decision(1.0, "truck_1", "HOLD_BASELINE")])[0]
        self.assertTrue(held.recovery_hold_active)
        self.assertEqual(held.command, "HOLD_BASELINE")
        self.assertEqual(held.control_mode, "ACC")
        recovered = self.executive.process_decisions([decision(2.0, "truck_1", "HOLD_BASELINE")])[0]
        self.assertFalse(recovered.recovery_hold_active)
        self.assertEqual(recovered.control_mode, "CACC")
        self.assertEqual(recovered.target_time_gap_s, 0.8)

    def test_lower_safety_recommendation_cannot_bypass_hold_time(self) -> None:
        self.executive.process_decisions([decision(0.0, "truck_1", "EMERGENCY_CAUTION")])
        guarded = self.executive.process_decisions([decision(0.5, "truck_1", "INCREASE_GAP_1_5S")])[0]
        self.assertTrue(guarded.recovery_hold_active)
        self.assertEqual(guarded.control_mode, "SPEED_OVERRIDE")
        self.assertEqual(guarded.max_speed_cap_mps, 12.0)

    def test_critical_applies_speed_cap_and_radar_only_mode(self) -> None:
        action = self.executive.process_decisions([decision(1.0, "truck_1", "EMERGENCY_CAUTION")])[0]
        controller = CACCController(desired_time_gap=0.8, min_gap=10.0, max_speed=18.0)
        self.executive.apply_to_controller("truck_1", controller)
        self.assertEqual(action.max_speed_cap_mps, 12.0)
        self.assertFalse(controller.v2v_enabled)
        self.assertEqual(controller.max_speed_cap, 12.0)
        output = controller.command(13.0, 13.0, 30.0, 18.0, 1.0, 40.0)
        self.assertEqual(output.state, ResilienceState.ACC_FALLBACK)
        self.assertLessEqual(output.target_speed, 12.0)

    def test_unknown_command_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.executive.process_decisions([decision(0.0, "truck_1", "NONE")])


class ControllerExecutiveInterfaceTests(unittest.TestCase):
    def test_controller_parameter_methods_preserve_bounds(self) -> None:
        controller = CACCController(desired_time_gap=0.8, min_gap=10.0, max_speed=18.0)
        controller.set_desired_time_gap(1.5)
        controller.set_v2v_enabled(False)
        controller.set_max_speed_cap(12.0)
        self.assertEqual(controller.desired_time_gap, 1.5)
        self.assertFalse(controller.v2v_enabled)
        self.assertEqual(controller.max_speed_cap, 12.0)
        with self.assertRaises(ValueError):
            controller.set_desired_time_gap(2.1)
