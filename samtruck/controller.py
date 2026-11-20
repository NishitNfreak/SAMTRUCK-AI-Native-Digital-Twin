from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ResilienceState(str, Enum):
    CACC = "CACC"
    DEGRADED_CACC = "DEGRADED_CACC"
    ACC_FALLBACK = "ACC_FALLBACK"
    EMERGENCY_BRAKE = "EMERGENCY_BRAKE"


@dataclass(frozen=True)
class ControlCommand:
    target_speed: float
    acceleration: float
    state: ResilienceState
    reason: str
    desired_gap: float


@dataclass
class CACCController:
    """Deterministic CTG CACC with sensor-only ACC/AEB fallback."""

    desired_time_gap: float
    min_gap: float
    max_speed: float
    kp_gap: float = 0.16
    kp_speed: float = 0.55
    leader_accel_gain: float = 0.35
    max_accel: float = 1.2
    comfortable_decel: float = -4.5
    aeb_decel: float = -7.0
    acc_min_gap: float = 5.0
    acc_time_gap: float = 1.8
    v2v_enabled: bool = True
    max_speed_cap: float | None = None

    def set_desired_time_gap(self, gap_s: float) -> None:
        """Set a bounded executive CTG target without changing control gains."""
        if not 0.8 <= gap_s <= 2.0:
            raise ValueError("desired time gap must be within [0.8, 2.0] seconds.")
        self.desired_time_gap = gap_s

    def set_v2v_enabled(self, enabled: bool) -> None:
        """Allow the executive to select CACC or radar-only ACC input use."""
        self.v2v_enabled = bool(enabled)

    def set_max_speed_cap(self, speed_mps: float | None) -> None:
        """Apply or clear an optional executive safety cap."""
        if speed_mps is not None and speed_mps <= 0.0:
            raise ValueError("max speed cap must be positive or None.")
        self.max_speed_cap = speed_mps

    def desired_gap(self, follower_speed: float, state: ResilienceState) -> float:
        speed = max(0.0, follower_speed)
        if state in (ResilienceState.ACC_FALLBACK, ResilienceState.EMERGENCY_BRAKE):
            # The frozen baseline retains its established ACC fallback gap.
            # An opt-in executive radar-only mode reuses the bounded CTG
            # setting selected through ``set_desired_time_gap``.
            if not self.v2v_enabled:
                return self.acc_min_gap + self.desired_time_gap * speed
            return self.acc_min_gap + self.acc_time_gap * speed
        return self.min_gap + self.desired_time_gap * speed

    def command(
        self,
        follower_speed: float,
        sensed_leader_speed: float,
        sensed_gap: float,
        v2v_leader_speed: float | None,
        v2v_leader_accel: float | None,
        link_latency_ms: float | None,
    ) -> ControlCommand:
        if not self.v2v_enabled:
            v2v_leader_speed = None
            v2v_leader_accel = None
            link_latency_ms = None
        if link_latency_ms is None:
            state = ResilienceState.ACC_FALLBACK
        elif link_latency_ms < 100:
            state = ResilienceState.CACC
        elif link_latency_ms < 1000:
            state = ResilienceState.DEGRADED_CACC
        else:
            state = ResilienceState.ACC_FALLBACK

        relative_speed = follower_speed - sensed_leader_speed
        ttc = sensed_gap / relative_speed if relative_speed > 0.05 else float("inf")
        if sensed_gap < self.acc_min_gap or ttc < 2.0:
            state = ResilienceState.EMERGENCY_BRAKE

        desired_gap = self.desired_gap(follower_speed, state)
        gap_error = sensed_gap - desired_gap
        leader_speed = sensed_leader_speed if v2v_leader_speed is None else v2v_leader_speed
        leader_accel = 0.0 if v2v_leader_accel is None else v2v_leader_accel
        gain = 0.55 if state is ResilienceState.DEGRADED_CACC else 1.0
        raw_accel = gain * (
            self.kp_gap * gap_error
            + self.kp_speed * (leader_speed - follower_speed)
            + self.leader_accel_gain * leader_accel
        )
        if state is ResilienceState.ACC_FALLBACK:
            raw_accel = self.kp_gap * gap_error + self.kp_speed * (sensed_leader_speed - follower_speed)
        if state is ResilienceState.EMERGENCY_BRAKE:
            raw_accel = self.aeb_decel

        accel = min(self.max_accel, max(self.aeb_decel, raw_accel))
        effective_max_speed = self.max_speed if self.max_speed_cap is None else min(self.max_speed, self.max_speed_cap)
        target_speed = min(effective_max_speed, max(0.0, follower_speed + accel * 0.1))
        reason = {
            ResilienceState.CACC: "v2v_ctg_cacc",
            ResilienceState.DEGRADED_CACC: "latency_buffered_cacc",
            ResilienceState.ACC_FALLBACK: "radar_acc_fallback",
            ResilienceState.EMERGENCY_BRAKE: "radar_aeb",
        }[state]
        return ControlCommand(target_speed, accel, state, reason, desired_gap)
