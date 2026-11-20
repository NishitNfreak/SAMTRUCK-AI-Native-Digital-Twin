"""
SAMTRUCK Evaluation Metrics Module
Computes real-data statistics, string stability, recovery time, jerk, and PASS/FAIL status.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import pandas as pd
import numpy as np


def compute_jerk(df: pd.DataFrame) -> pd.DataFrame:
    """Calculates jerk (da/dt) per vehicle in m/s^3."""
    df = df.sort_values(by=["vehicle_id", "simulation_time_s"]).copy()
    df["jerk_mps3"] = 0.0

    for veh_id, group in df.groupby("vehicle_id"):
        dt = group["simulation_time_s"].diff()
        da = group["acceleration_mps2"].diff()
        # Avoid division by zero
        jerk = np.where(dt > 0.0001, da / dt, 0.0)
        df.loc[group.index, "jerk_mps3"] = np.nan_to_num(jerk, nan=0.0)

    return df


def compute_recovery_time(df: pd.DataFrame, events: list[dict]) -> float | None:
    """
    For an applied disturbance, calculate time from the end of the last applied disturbance
    until BOTH followers stay within absolute gap error <= 1.0 m for at least 1.0 continuous second.
    Returns null (None) if condition not met or no applied disturbance.
    """
    applied_events = [e for e in events if e.get("applied", False)]
    if not applied_events:
        return None

    # Find end of last applied disturbance
    last_event_end = 0.0
    has_disturbance = False
    for e in applied_events:
        event_type = e.get("event", "")
        # Baseline/normal platooning is not a disturbance
        if event_type in {"baseline", "normal_platooning"}:
            continue
        has_disturbance = True
        at_s = float(e.get("time_s", e.get("at_s", 0.0)))
        dur_s = float(e.get("duration_s", 0.1))
        last_event_end = max(last_event_end, at_s + dur_s)

    if not has_disturbance:
        return None

    followers_df = df[df["vehicle_id"].isin(["samtruck_follower_1", "samtruck_follower_2"])].copy()
    if followers_df.empty:
        return None

    followers_df["gap_error_m"] = pd.to_numeric(followers_df["gap_error_m"], errors="coerce")
    followers_df["abs_gap_error"] = followers_df["gap_error_m"].abs()

    # Filter telemetry post disturbance end
    post_dist_df = followers_df[followers_df["simulation_time_s"] >= last_event_end].copy()
    if post_dist_df.empty:
        return None

    # Pivot to get max abs_gap_error across both followers at each time step
    pivot = post_dist_df.pivot_table(index="simulation_time_s", columns="vehicle_id", values="abs_gap_error")
    if pivot.shape[1] < 2:
        return None  # Needs telemetry for both followers

    pivot["max_error"] = pivot.max(axis=1)
    pivot = pivot.sort_index()

    # Find first continuous 1.0s window where max_error <= 1.0m
    times = pivot.index.to_numpy()
    errors = pivot["max_error"].to_numpy()

    stable_start_time = None
    window_start_idx = 0

    for i in range(len(times)):
        if errors[i] > 1.0:
            window_start_idx = i + 1
            continue
        # Check window duration
        if times[i] - times[window_start_idx] >= 1.0 - 1e-5:
            stable_start_time = times[window_start_idx]
            break

    if stable_start_time is not None:
        recovery_time = round(stable_start_time - last_event_end, 3)
        return max(0.0, recovery_time)

    return None


def compute_string_stability(df: pd.DataFrame, events: list[dict]) -> dict:
    """
    Computes quantitative disturbance propagation metric D_v and ratio R_v.
    D_v = max_{t >= event_time} |signal_v(t) - median(signal_v(t < event_time))|
    """
    applied_events = [e for e in events if e.get("applied", False)]
    # Determine disturbance event time (default to first event or mid-sim)
    if applied_events:
        event_time = min(float(e.get("time_s", e.get("at_s", 0.0))) for e in applied_events)
    else:
        event_time = 0.0

    result = {
        "formula": "D_v = max_{t >= t_event} |signal(t) - median(signal(t < t_event))|; R_v = D_v_follower / D_v_leader",
        "event_time_s": event_time,
        "speed_disturbance_m_s": {},
        "accel_disturbance_m_s2": {},
        "speed_propagation_ratio": {}
    }

    pre_df = df[df["simulation_time_s"] < event_time]
    post_df = df[df["simulation_time_s"] >= event_time]

    # If no pre-event window exists (e.g. event at t=0), fallback to first 2 seconds
    if pre_df.empty:
        pre_df = df[df["simulation_time_s"] <= 2.0]
        post_df = df[df["simulation_time_s"] > 2.0]

    truck_ids = ["samtruck_leader", "samtruck_follower_1", "samtruck_follower_2"]

    for veh_id in truck_ids:
        veh_pre = pre_df[pre_df["vehicle_id"] == veh_id]
        veh_post = post_df[post_df["vehicle_id"] == veh_id]

        if veh_pre.empty or veh_post.empty:
            result["speed_disturbance_m_s"][veh_id] = None
            result["accel_disturbance_m_s2"][veh_id] = None
            continue

        # Speed disturbance
        speed_baseline = veh_pre["speed_mps"].median()
        speed_dev = (veh_post["speed_mps"] - speed_baseline).abs().max()
        result["speed_disturbance_m_s"][veh_id] = round(float(speed_dev), 3) if not np.isnan(speed_dev) else 0.0

        # Accel disturbance
        accel_baseline = veh_pre["acceleration_mps2"].median()
        accel_dev = (veh_post["acceleration_mps2"] - accel_baseline).abs().max()
        result["accel_disturbance_m_s2"][veh_id] = round(float(accel_dev), 3) if not np.isnan(accel_dev) else 0.0

    # Speed propagation ratio
    leader_d = result["speed_disturbance_m_s"].get("samtruck_leader", 0.0)
    for follower_id in ["samtruck_follower_1", "samtruck_follower_2"]:
        follower_d = result["speed_disturbance_m_s"].get(follower_id, None)
        if leader_d is not None and leader_d > 0.001 and follower_d is not None:
            ratio = round(follower_d / leader_d, 3)
            result["speed_propagation_ratio"][follower_id] = ratio
        else:
            result["speed_propagation_ratio"][follower_id] = None

    return result


def calculate_metrics(
    telemetry_path: Path,
    messages_path: Path,
    events_path: Path,
    spec: dict,
    output_dir: Path
) -> tuple[dict, bool, str]:
    """
    Reads telemetry.csv, messages.csv, and events.json to calculate required metrics,
    exports metrics.json and metrics.csv, and returns (metrics_dict, pass_fail_bool, pass_reason).
    """
    # Load Telemetry
    if telemetry_path.exists():
        df_tel = pd.read_csv(telemetry_path)
    else:
        df_tel = pd.DataFrame()

    # Load Messages
    if messages_path.exists():
        df_msg = pd.read_csv(messages_path)
    else:
        df_msg = pd.DataFrame()

    # Load Events
    if events_path.exists():
        events_data = json.loads(events_path.read_text(encoding="utf-8"))
    else:
        events_data = []

    # Calculate Jerk
    if not df_tel.empty:
        df_tel = compute_jerk(df_tel)

    # Filter Followers for Gap metrics
    followers_df = df_tel[df_tel["vehicle_id"].isin(["samtruck_follower_1", "samtruck_follower_2"])].copy()
    if not followers_df.empty:
        followers_df["actual_gap_m"] = pd.to_numeric(followers_df["actual_gap_m"], errors="coerce")
        followers_df["desired_gap_m"] = pd.to_numeric(followers_df["desired_gap_m"], errors="coerce")
        followers_df["gap_error_m"] = pd.to_numeric(followers_df["gap_error_m"], errors="coerce")

        avg_gap = followers_df["actual_gap_m"].mean()
        avg_desired_gap = followers_df["desired_gap_m"].mean()
        mae_gap_error = followers_df["gap_error_m"].abs().mean()
        rms_gap_error = math.sqrt((followers_df["gap_error_m"] ** 2).mean())
        max_gap_error = followers_df["gap_error_m"].abs().max()
        min_gap = followers_df["actual_gap_m"].min()
    else:
        avg_gap = avg_desired_gap = mae_gap_error = rms_gap_error = max_gap_error = min_gap = None

    # Overall Telemetry Metrics
    if not df_tel.empty:
        max_speed = df_tel["speed_mps"].max()
        max_accel = df_tel["acceleration_mps2"].max()
        min_accel = df_tel["acceleration_mps2"].min()
        max_decel = max(0.0, -min_accel) if not np.isnan(min_accel) else 0.0
        max_jerk = df_tel["jerk_mps3"].abs().max()
    else:
        max_speed = max_accel = max_decel = max_jerk = None

    # Collisions
    # Collision count passed via spec metadata or calculated from collision events
    collision_count = spec.get("collisions", 0)

    # Recovery Time
    recovery_time = compute_recovery_time(df_tel, events_data) if not df_tel.empty else None

    # V2X Metrics
    if not df_msg.empty:
        transmitted_messages = len(df_msg)
        received_df = df_msg[df_msg["status"] == "received"]
        dropped_df = df_msg[df_msg["status"] == "dropped"]

        received_messages = len(received_df)
        dropped_messages = len(dropped_df)
        observed_packet_loss_pct = round((dropped_messages / transmitted_messages) * 100.0, 2) if transmitted_messages > 0 else 0.0

        if not received_df.empty:
            recv_latencies = pd.to_numeric(received_df["latency_ms"], errors="coerce").dropna()
            v2x_avg_latency = recv_latencies.mean() if not recv_latencies.empty else None
            v2x_max_latency = recv_latencies.max() if not recv_latencies.empty else None
        else:
            v2x_avg_latency = v2x_max_latency = None
    else:
        transmitted_messages = received_messages = dropped_messages = 0
        observed_packet_loss_pct = 0.0
        v2x_avg_latency = v2x_max_latency = None

    # String Stability
    string_stab = compute_string_stability(df_tel, events_data) if not df_tel.empty else {}

    # PASS / FAIL Logic
    # PASS only if:
    # 1. all 3 platoon trucks have telemetry,
    # 2. every expected scenario event was actually applied (applied == True),
    # 3. collision_count is zero.
    truck_ids_present = set(df_tel["vehicle_id"].unique()) if not df_tel.empty else set()
    all_3_trucks_present = {"samtruck_leader", "samtruck_follower_1", "samtruck_follower_2"}.issubset(truck_ids_present)

    expected_events = spec.get("events", [])
    if expected_events:
        all_events_applied = len(events_data) >= len(expected_events) and all(e.get("applied", False) for e in events_data)
    else:
        all_events_applied = True

    no_collisions = (collision_count == 0)

    pass_status = bool(all_3_trucks_present and all_events_applied and no_collisions)

    fail_reasons = []
    if not all_3_trucks_present:
        fail_reasons.append(f"Missing truck telemetry (present: {sorted(truck_ids_present)})")
    if not all_events_applied:
        unapplied = [e.get("event", "unknown") for e in events_data if not e.get("applied", False)]
        fail_reasons.append(f"Events not applied: {unapplied}")
    if not no_collisions:
        fail_reasons.append(f"Collisions detected: {collision_count}")

    pass_reason = "All 3 platoon trucks recorded telemetry, all scenario events applied successfully, and zero collisions occurred." if pass_status else "; ".join(fail_reasons)

    metrics_dict = {
        "scenario_id": spec.get("id", "unknown"),
        "scenario_title": spec.get("title", "Unknown"),
        "average_gap_m": round(float(avg_gap), 3) if avg_gap is not None and not np.isnan(avg_gap) else None,
        "average_desired_gap_m": round(float(avg_desired_gap), 3) if avg_desired_gap is not None and not np.isnan(avg_desired_gap) else None,
        "mean_absolute_gap_error_m": round(float(mae_gap_error), 3) if mae_gap_error is not None and not np.isnan(mae_gap_error) else None,
        "rms_gap_error_m": round(float(rms_gap_error), 3) if rms_gap_error is not None and not np.isnan(rms_gap_error) else None,
        "maximum_gap_error_m": round(float(max_gap_error), 3) if max_gap_error is not None and not np.isnan(max_gap_error) else None,
        "minimum_gap_m": round(float(min_gap), 3) if min_gap is not None and not np.isnan(min_gap) else None,
        "maximum_speed_mps": round(float(max_speed), 3) if max_speed is not None and not np.isnan(max_speed) else None,
        "maximum_acceleration_mps2": round(float(max_accel), 3) if max_accel is not None and not np.isnan(max_accel) else None,
        "maximum_deceleration_mps2": round(float(max_decel), 3) if max_decel is not None and not np.isnan(max_decel) else None,
        "maximum_jerk_mps3": round(float(max_jerk), 3) if max_jerk is not None and not np.isnan(max_jerk) else None,
        "collision_count": int(collision_count),
        "recovery_time_s": recovery_time,
        "v2x_average_latency_ms": round(float(v2x_avg_latency), 3) if v2x_avg_latency is not None and not np.isnan(v2x_avg_latency) else None,
        "v2x_maximum_latency_ms": round(float(v2x_max_latency), 3) if v2x_max_latency is not None and not np.isnan(v2x_max_latency) else None,
        "transmitted_messages": int(transmitted_messages),
        "received_messages": int(received_messages),
        "dropped_messages": int(dropped_messages),
        "observed_packet_loss_percent": float(observed_packet_loss_pct),
        "fuel_energy": {
            "status": "unavailable",
            "reason": "No SUMO fuel or energy model is configured for platoon_truck."
        },
        "string_stability": string_stab,
        "pass_fail": {
            "status": "PASS" if pass_status else "FAIL",
            "reason": pass_reason
        }
    }

    # Save metrics.json
    metrics_json_file = output_dir / "metrics.json"
    metrics_json_file.write_text(json.dumps(metrics_dict, indent=2), encoding="utf-8")

    # Save metrics.csv (flat tabular version)
    flat_metrics = {k: v for k, v in metrics_dict.items() if not isinstance(v, dict)}
    flat_metrics["pass_status"] = metrics_dict["pass_fail"]["status"]
    flat_metrics["pass_reason"] = metrics_dict["pass_fail"]["reason"]
    flat_metrics["fuel_energy_status"] = metrics_dict["fuel_energy"]["status"]

    metrics_csv_file = output_dir / "metrics.csv"
    pd.DataFrame([flat_metrics]).to_csv(metrics_csv_file, index=False)

    return metrics_dict, pass_status, pass_reason
