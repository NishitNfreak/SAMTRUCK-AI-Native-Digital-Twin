"""Opt-in active executive for bounded CACC parameter adaptation.

It translates already-generated advisor decisions into bounded controller
settings. It has no TraCI/Digital-Twin dependency and never calls SUMO APIs.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable

from .controller import CACCController
from .twin_advisor import AdvisorDecision


BASELINE_GAP_S = 0.8
MAX_GAP_S = 2.0
MIN_GAP_S = 0.8
RECOVERY_HOLD_S = 2.0
GAP_SLEW_RATE_S_PER_S = 0.5


@dataclass(frozen=True)
class ExecutiveAction:
    simulation_time_s: float
    vehicle_id: str
    command: str
    control_mode: str
    target_time_gap_s: float
    applied_time_gap_s: float
    v2v_enabled: bool
    max_speed_cap_mps: float | None
    recovery_hold_active: bool
    reason: str


@dataclass
class _VehicleExecutiveState:
    applied_gap_s: float = BASELINE_GAP_S
    target_gap_s: float = BASELINE_GAP_S
    v2v_enabled: bool = True
    max_speed_cap_mps: float | None = None
    control_mode: str = "CACC"
    last_timestamp_s: float | None = None
    safety_level: int = 0
    last_safety_change_timestamp_s: float | None = None


class ActiveExecutive:
    """Bounded, per-follower executive with no direct vehicle actuation."""

    _COMMANDS = {
        "HOLD_BASELINE": (BASELINE_GAP_S, True, None, "CACC"),
        "INCREASE_GAP_1_5S": (1.5, True, None, "CACC"),
        "FALLBACK_ACC": (1.2, False, None, "ACC"),
        "FALLBACK_ACC_WIDE_GAP": (1.8, False, None, "ACC"),
        "EMERGENCY_CAUTION": (2.0, False, 12.0, "SPEED_OVERRIDE"),
    }
    _SAFETY_LEVELS = {
        "HOLD_BASELINE": 0,
        "INCREASE_GAP_1_5S": 1,
        "FALLBACK_ACC": 2,
        "FALLBACK_ACC_WIDE_GAP": 2,
        "EMERGENCY_CAUTION": 3,
    }

    def __init__(self, vehicle_ids: Iterable[str], *, recovery_hold_s: float = RECOVERY_HOLD_S,
                 gap_slew_rate_s_per_s: float = GAP_SLEW_RATE_S_PER_S) -> None:
        if recovery_hold_s < 0.0 or not math.isfinite(recovery_hold_s):
            raise ValueError("recovery hold time must be finite and non-negative.")
        if gap_slew_rate_s_per_s <= 0.0 or not math.isfinite(gap_slew_rate_s_per_s):
            raise ValueError("gap slew rate must be finite and positive.")
        self.recovery_hold_s = recovery_hold_s
        self.gap_slew_rate_s_per_s = gap_slew_rate_s_per_s
        self._states = {vehicle_id: _VehicleExecutiveState() for vehicle_id in vehicle_ids}

    def process_decisions(self, decisions: Iterable[AdvisorDecision]) -> list[ExecutiveAction]:
        """Update per-vehicle bounded state and return auditable action rows."""
        actions: list[ExecutiveAction] = []
        for decision in decisions:
            state = self._states.get(decision.vehicle_id)
            if state is None:
                continue
            command = decision.command_to_executive
            if command not in self._COMMANDS:
                raise ValueError(f"Unsupported executive command: {command!r}")
            now = decision.simulation_time_s
            if not math.isfinite(now):
                raise ValueError("Executive decision timestamp must be finite.")
            target_gap, v2v_enabled, speed_cap, mode = self._COMMANDS[command]
            requested_safety_level = self._SAFETY_LEVELS[command]
            recovery_hold = False
            last_change = state.last_safety_change_timestamp_s
            if requested_safety_level < state.safety_level and last_change is not None:
                if now - last_change < self.recovery_hold_s:
                    target_gap, v2v_enabled, speed_cap, mode = (
                        state.target_gap_s, state.v2v_enabled, state.max_speed_cap_mps, state.control_mode
                    )
                    recovery_hold = True
                else:
                    state.safety_level = requested_safety_level
                    state.last_safety_change_timestamp_s = now
            elif requested_safety_level > state.safety_level:
                state.safety_level = requested_safety_level
                state.last_safety_change_timestamp_s = now

            dt = 0.0 if state.last_timestamp_s is None else max(0.0, now - state.last_timestamp_s)
            max_delta = self.gap_slew_rate_s_per_s * dt
            if target_gap > state.applied_gap_s:
                state.applied_gap_s = min(target_gap, state.applied_gap_s + max_delta)
            else:
                state.applied_gap_s = max(target_gap, state.applied_gap_s - max_delta)
            state.target_gap_s = target_gap
            state.v2v_enabled = v2v_enabled
            state.max_speed_cap_mps = speed_cap
            state.control_mode = mode
            state.last_timestamp_s = now
            actions.append(ExecutiveAction(
                now, decision.vehicle_id, command, mode, target_gap, state.applied_gap_s,
                v2v_enabled, speed_cap, recovery_hold, decision.decision_code,
            ))
        return actions

    def apply_to_controller(self, vehicle_id: str, controller: CACCController) -> None:
        """Apply state immediately before the existing controller ``command()``."""
        state = self._states[vehicle_id]
        controller.set_desired_time_gap(state.applied_gap_s)
        controller.set_v2v_enabled(state.v2v_enabled)
        controller.set_max_speed_cap(state.max_speed_cap_mps)

    def state_as_dict(self, vehicle_id: str) -> dict[str, object]:
        return asdict(self._states[vehicle_id])
