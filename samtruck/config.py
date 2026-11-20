from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = PROJECT_ROOT / "scenario"
OUTPUT_DIR = PROJECT_ROOT / "outputs"
PORTABLE_SUMO = PROJECT_ROOT / "runtime" / "sumo-1.27.1"
SUMO_HOME = PORTABLE_SUMO if PORTABLE_SUMO.exists() else Path(os.environ.get("SUMO_HOME", r"E:\SUMO"))



@dataclass(frozen=True)
class SimulationConfig:
    scenario: str = "baseline"
    step_length: float = 0.1
    duration: float = 70.0
    desired_time_gap: float = 1.2
    min_gap: float = 4.0
    max_speed: float = 30.0
    emergency_start: float = 42.0
    emergency_end: float = 46.0
    v2x_delay_steps: int = 0
    v2x_packet_loss_every: int = 0
    twin_history_len: int = 50
    twin_v2x_stale_after_s: float = 0.5
    sensor_seed: int = 42
    radar_noise_std_m: float = 0.0
    odometry_speed_noise_std_mps: float = 0.0
    radar_dropout_probability: float = 0.0

    @property
    def truck_ids(self) -> tuple[str, ...]:
        return ("truck_0", "truck_1", "truck_2")
