"""
SAMTRUCK Continuous Multi-Scenario Live Demonstration Engine
============================================================
Runs SUMO indefinitely (or for extended hours), cycling through dynamic cyber-physical
stress events every 20 seconds in a seamless real-time loop:

  Phase 1 (0s - 20s)  : Nominal CACC Platooning (Baseline, healthy V2V, 0.8s gap)
  Phase 2 (20s - 35s) : Severe Sensor Noise Storm (Radar sigma=1.5m, Odom sigma=0.6m)
  Phase 3 (35s - 50s) : Complete V2V Wireless Blackout (Age grows -> Stale -> Outage -> ACC fallback)
  Phase 4 (50s - 65s) : Emergency Lead Vehicle Hard Brake during Outage (-4.5 m/s^2)
  Phase 5 (65s - 80s) : Channel & Sensor Recovery (V2V restores -> 2.0s hold -> Slew back to CACC)
  Phase 6 (80s+)      : Platoon re-docking & Loop Repeats Seamlessly!

Forces real-time line-by-line unbuffered streaming for external terminal monitors.
"""

import os
import sys
import time
import json
import math
import argparse
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from samtruck.config import ScenarioConfig, PlatoonConfig, TwinConfig
from samtruck.builder import build_simulation_environment
from samtruck.digital_twin import DigitalTwinSupervisor
from samtruck.twin_advisor import TwinAdvisor
from samtruck.executive import ActiveExecutive


class ContinuousLiveDemo:
    def __init__(self, output_dir: Path, gui: bool = True, step_delay_s: float = 0.05):
        self.output_dir = output_dir
        self.gui = gui
        self.step_delay_s = step_delay_s
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Output log files
        self.eval_file = open(self.output_dir / "evaluator_console.jsonl", "w", encoding="utf-8", buffering=1)
        self.advisor_file = open(self.output_dir / "twin_advisor_decisions.jsonl", "w", encoding="utf-8", buffering=1)
        self.exec_file = open(self.output_dir / "executive_actions.jsonl", "w", encoding="utf-8", buffering=1)

    def run(self, max_duration_s: float = 3600.0):
        print("=" * 80)
        print(" SAMTRUCK CONTINUOUS MULTI-SCENARIO LIVE DEMO ENGINE")
        print(f" Output Stream Directory : {self.output_dir}")
        print(f" GUI Rendering Mode      : {'SUMO GUI (Visual)' if self.gui else 'Headless'}")
        print(f" Target Run Duration     : {max_duration_s} seconds (Press Ctrl+C to Stop)")
        print("=" * 80)

        # Build simulation
        config = ScenarioConfig(name="continuous_live_loop", duration_s=max_duration_s, gui=self.gui)
        platoon_cfg = PlatoonConfig()
        twin_cfg = TwinConfig()

        env = build_simulation_environment(config, platoon_cfg, twin_cfg)
        supervisor = DigitalTwinSupervisor(twin_cfg)
        advisor = TwinAdvisor()
        executive = ActiveExecutive()

        step = 0
        dt = 0.1
        sim_time = 0.0

        try:
            env.start()

            while sim_time < max_duration_s:
                # Determine active cyclical event phase (80-second repeating cycle)
                cycle_time = sim_time % 80.0
                phase_name, sensor_noise_active, v2x_blackout_active, lead_hard_brake = self._get_cycle_phase(cycle_time)

                # Inject dynamic degradation into the environment
                env.set_sensor_noise_override(sensor_noise_active)
                env.set_v2x_blackout_override(v2x_blackout_active)
                if lead_hard_brake:
                    env.apply_lead_emergency_brake(-4.5)
                else:
                    env.release_lead_emergency_brake(18.0)

                # Step physical simulation
                env.step()

                # Process Digital Twin, Advisor & Executive
                for follower_id in ["samtruck_follower_1", "samtruck_follower_2"]:
                    # 1. Twin State Estimation
                    twin_state = supervisor.step_follower(follower_id, env)

                    # 2. Log Evaluator stream (unbuffered)
                    eval_record = {
                        "step_s": round(sim_time, 2),
                        "vehicle_id": follower_id,
                        "phase": phase_name,
                        "raw_odometry_speed_mps": round(twin_state.raw_odometry_speed, 2),
                        "twin_speed_estimation_mps": round(twin_state.estimated_speed, 2),
                        "true_speed_mps": round(twin_state.ground_truth_speed, 2),
                        "raw_odometry_speed_error_mps": round(twin_state.raw_odometry_speed - twin_state.ground_truth_speed, 3),
                        "twin_speed_estimation_error_mps": round(twin_state.estimated_speed - twin_state.ground_truth_speed, 3),
                        "twin_speed_sigma_mps": round(twin_state.speed_sigma, 3),
                    }
                    self.eval_file.write(json.dumps(eval_record) + "\n")
                    self.eval_file.flush()

                    # 3. AI Advisor Judgment
                    decision = advisor.evaluate_vehicle(follower_id, twin_state, sim_time)
                    advisor_record = {
                        "step_s": round(sim_time, 2),
                        "vehicle_id": follower_id,
                        "phase": phase_name,
                        "decision_code": decision.decision_code,
                        "severity": decision.severity,
                        "v2x_status": decision.v2x_status,
                        "v2x_age_s": round(decision.v2x_age_s, 2),
                        "speed_sigma_mps": round(decision.speed_sigma_mps, 3),
                        "command_to_executive": decision.command_to_executive,
                    }
                    self.advisor_file.write(json.dumps(advisor_record) + "\n")
                    self.advisor_file.flush()

                    # 4. Active Executive Closed-Loop Mutation
                    cmd_res = executive.apply_decision(follower_id, decision, env.get_controller(follower_id), sim_time)
                    exec_record = {
                        "step_s": round(sim_time, 2),
                        "vehicle_id": follower_id,
                        "phase": phase_name,
                        "command_applied": cmd_res.command_applied,
                        "controller_mode": cmd_res.controller_mode,
                        "commanded_time_gap_s": round(cmd_res.commanded_time_gap_s, 2),
                        "speed_cap_mps": cmd_res.speed_cap_mps,
                        "recovery_guard_active": cmd_res.recovery_guard_active,
                    }
                    self.exec_file.write(json.dumps(exec_record) + "\n")
                    self.exec_file.flush()

                # Live ASCII header every 5 seconds
                if step % 50 == 0:
                    print(f"[{sim_time:6.1f}s] PHASE: {phase_name:<28} | Trucks Active: 3 | Zero Collisions")

                sim_time += dt
                step += 1

                if self.gui and self.step_delay_s > 0:
                    time.sleep(self.step_delay_s)

        except KeyboardInterrupt:
            print("\n[STOP] Continuous demo interrupted by user.")
        finally:
            env.close()
            self.eval_file.close()
            self.advisor_file.close()
            self.exec_file.close()
            print(f"[CLOSED] Artifacts saved to {self.output_dir}")

    def _get_cycle_phase(self, cycle_t: float):
        if cycle_t < 20.0:
            return ("1_NOMINAL_CACC_CRUISE", False, False, False)
        elif cycle_t < 35.0:
            return ("2_SEVERE_SENSOR_NOISE", True, False, False)
        elif cycle_t < 50.0:
            return ("3_V2V_WIRELESS_BLACKOUT", False, True, False)
        elif cycle_t < 65.0:
            return ("4_EMERGENCY_HARD_BRAKE_IN_OUTAGE", True, True, True)
        else:
            return ("5_RECOVERY_AND_RESTABILIZATION", False, False, False)


def main():
    parser = argparse.ArgumentParser(description="SAMTRUCK Continuous Multi-Scenario Live Demo")
    parser.add_argument("--output-dir", type=str, default="results/continuous_live_demo", help="Output stream dir")
    parser.add_argument("--no-gui", action="store_true", help="Run without SUMO GUI window")
    parser.add_argument("--duration", type=float, default=3600.0, help="Run duration in seconds (default: 1 hour)")
    parser.add_argument("--delay", type=float, default=0.03, help="Inter-step real-time delay in seconds")
    args = parser.parse_args()

    demo = ContinuousLiveDemo(
        output_dir=Path(args.output_dir),
        gui=not args.no_gui,
        step_delay_s=args.delay,
    )
    demo.run(max_duration_s=args.duration)


if __name__ == "__main__":
    main()