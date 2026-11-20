"""
SAMTRUCK Graph Generator Module
Generates all base and scenario-specific plots using Matplotlib Agg backend.
"""
from __future__ import annotations

from pathlib import Path
import json
import matplotlib
matplotlib.use("Agg")  # Non-interactive headless backend
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


def add_event_markers(ax, events: list[dict]):
    """Adds vertical dashed lines and labels for events where applied == True."""
    if not events:
        return
    for event in events:
        if not event.get("applied", False):
            continue
        time_s = float(event.get("time_s", event.get("at_s", 0.0)))
        event_name = event.get("event", event.get("type", "event"))
        ax.axvline(x=time_s, color="red", linestyle="--", alpha=0.75, linewidth=1.2)
        ax.text(
            time_s + 0.5,
            ax.get_ylim()[1] * 0.95 if ax.get_ylim()[1] != 0 else 1.0,
            event_name,
            color="darkred",
            fontsize=8,
            rotation=90,
            verticalalignment="top"
        )


def plot_speed_vs_time(df: pd.DataFrame, events: list[dict], output_path: Path):
    fig, ax = plt.subplots(figsize=(10, 5))
    truck_colors = {"samtruck_leader": "blue", "samtruck_follower_1": "green", "samtruck_follower_2": "orange"}

    for veh_id, group in df.groupby("vehicle_id"):
        color = truck_colors.get(veh_id, "gray")
        ax.plot(group["simulation_time_s"], group["speed_mps"], label=veh_id, color=color, linewidth=1.5)

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("Vehicle Speed vs Time")
    ax.grid(True, linestyle=":", alpha=0.6)
    add_event_markers(ax, events)
    ax.legend(loc="best")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_gap_vs_time(df: pd.DataFrame, events: list[dict], output_path: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    f1_df = df[df["vehicle_id"] == "samtruck_follower_1"]
    f2_df = df[df["vehicle_id"] == "samtruck_follower_2"]

    if not f1_df.empty:
        ax.plot(f1_df["simulation_time_s"], f1_df["actual_gap_m"], label="Actual Gap (Leader -> Truck 2)", color="green", linewidth=1.5)
        ax.plot(f1_df["simulation_time_s"], f1_df["desired_gap_m"], label="Desired Gap (Truck 2)", color="lightgreen", linestyle="--", linewidth=1.2)

    if not f2_df.empty:
        ax.plot(f2_df["simulation_time_s"], f2_df["actual_gap_m"], label="Actual Gap (Truck 2 -> Truck 3)", color="orange", linewidth=1.5)
        ax.plot(f2_df["simulation_time_s"], f2_df["desired_gap_m"], label="Desired Gap (Truck 3)", color="moccasin", linestyle="--", linewidth=1.2)

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Gap (m)")
    ax.set_title("Platoon Inter-Vehicle Gap vs Time")
    ax.grid(True, linestyle=":", alpha=0.6)
    add_event_markers(ax, events)
    ax.legend(loc="best")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_gap_error(df: pd.DataFrame, events: list[dict], output_path: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    f1_df = df[df["vehicle_id"] == "samtruck_follower_1"]
    f2_df = df[df["vehicle_id"] == "samtruck_follower_2"]

    if not f1_df.empty:
        ax.plot(f1_df["simulation_time_s"], f1_df["gap_error_m"], label="Gap Error (Truck 2)", color="green", linewidth=1.5)
    if not f2_df.empty:
        ax.plot(f2_df["simulation_time_s"], f2_df["gap_error_m"], label="Gap Error (Truck 3)", color="orange", linewidth=1.5)

    ax.axhline(y=0.0, color="black", linestyle="-", linewidth=0.8, alpha=0.7)
    ax.axhline(y=1.0, color="gray", linestyle=":", linewidth=0.8, label="±1.0m Tolerance")
    ax.axhline(y=-1.0, color="gray", linestyle=":", linewidth=0.8)

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Gap Error (m)")
    ax.set_title("Gap Error (Actual - Desired) vs Time")
    ax.grid(True, linestyle=":", alpha=0.6)
    add_event_markers(ax, events)
    ax.legend(loc="best")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_acceleration(df: pd.DataFrame, events: list[dict], output_path: Path):
    fig, ax = plt.subplots(figsize=(10, 5))
    truck_colors = {"samtruck_leader": "blue", "samtruck_follower_1": "green", "samtruck_follower_2": "orange"}

    for veh_id, group in df.groupby("vehicle_id"):
        color = truck_colors.get(veh_id, "gray")
        ax.plot(group["simulation_time_s"], group["acceleration_mps2"], label=veh_id, color=color, linewidth=1.5)

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.set_title("Vehicle Acceleration vs Time")
    ax.grid(True, linestyle=":", alpha=0.6)
    add_event_markers(ax, events)
    ax.legend(loc="best")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_jerk(df: pd.DataFrame, events: list[dict], output_path: Path):
    fig, ax = plt.subplots(figsize=(10, 5))
    truck_colors = {"samtruck_leader": "blue", "samtruck_follower_1": "green", "samtruck_follower_2": "orange"}

    if "jerk_mps3" not in df.columns:
        # compute on the fly
        from .metrics import compute_jerk
        df = compute_jerk(df)

    for veh_id, group in df.groupby("vehicle_id"):
        color = truck_colors.get(veh_id, "gray")
        ax.plot(group["simulation_time_s"], group["jerk_mps3"], label=veh_id, color=color, linewidth=1.2, alpha=0.85)

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Jerk (m/s³)")
    ax.set_title("Vehicle Jerk (da/dt) vs Time")
    ax.grid(True, linestyle=":", alpha=0.6)
    add_event_markers(ax, events)
    ax.legend(loc="best")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_position(df: pd.DataFrame, events: list[dict], output_path: Path):
    fig, ax = plt.subplots(figsize=(10, 5))
    truck_colors = {"samtruck_leader": "blue", "samtruck_follower_1": "green", "samtruck_follower_2": "orange"}

    for veh_id, group in df.groupby("vehicle_id"):
        color = truck_colors.get(veh_id, "gray")
        ax.plot(group["simulation_time_s"], group["longitudinal_position_m"], label=veh_id, color=color, linewidth=1.5)

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Longitudinal Distance (m)")
    ax.set_title("Accumulated Distance Travelled vs Time")
    ax.grid(True, linestyle=":", alpha=0.6)
    add_event_markers(ax, events)
    ax.legend(loc="best")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_v2x_latency(msg_df: pd.DataFrame, events: list[dict], output_path: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    if not msg_df.empty:
        recv_df = msg_df[msg_df["status"] == "received"].copy()
        if not recv_df.empty:
            recv_df["latency_ms"] = pd.to_numeric(recv_df["latency_ms"], errors="coerce")
            ax.plot(recv_df["generation_time_s"], recv_df["latency_ms"], "o-", color="purple", markersize=3, linewidth=1.2, label="V2X Latency (ms)")

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("V2X Communication Latency vs Time")
    ax.grid(True, linestyle=":", alpha=0.6)
    add_event_markers(ax, events)
    ax.legend(loc="best")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_packet_loss(msg_df: pd.DataFrame, events: list[dict], output_path: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    if not msg_df.empty:
        msg_df = msg_df.copy()
        msg_df["time_bin"] = msg_df["generation_time_s"].round(0)
        grouped = msg_df.groupby("time_bin")["status"].apply(lambda s: (s == "dropped").mean() * 100.0)
        ax.plot(grouped.index, grouped.values, "r-s", markersize=4, linewidth=1.5, label="Observed Packet Loss (%)")

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Packet Loss (%)")
    ax.set_ylim(-5, 105)
    ax.set_title("V2X Packet Loss Rate vs Time")
    ax.grid(True, linestyle=":", alpha=0.6)
    add_event_markers(ax, events)
    ax.legend(loc="best")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_lane_event_timeline(df: pd.DataFrame, events: list[dict], output_path: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    for veh_id, group in df.groupby("vehicle_id"):
        # Map lane_id or lane_position
        ax.plot(group["simulation_time_s"], group["lane_position_m"], label=f"{veh_id} (lane pos)", linewidth=1.2)

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Lane Position (m)")
    ax.set_title("Lane & Maneuver Event Timeline")
    ax.grid(True, linestyle=":", alpha=0.6)
    add_event_markers(ax, events)
    ax.legend(loc="best")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_membership_timeline(df: pd.DataFrame, events: list[dict], output_path: Path):
    fig, ax = plt.subplots(figsize=(10, 5))

    # Shows active platoon trucks count over time
    platoon_size = df.groupby("simulation_time_s")["vehicle_id"].nunique()
    ax.step(platoon_size.index, platoon_size.values, where="post", color="teal", linewidth=2.0, label="Platoon Size (Vehicles)")

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Platoon Member Count")
    ax.set_ylim(0, 5)
    ax.set_title("Platoon Membership Timeline (Merge / Split)")
    ax.grid(True, linestyle=":", alpha=0.6)
    add_event_markers(ax, events)
    ax.legend(loc="best")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def generate_all_plots(
    telemetry_path: Path,
    messages_path: Path,
    events_path: Path,
    spec: dict,
    output_dir: Path
):
    df_tel = pd.read_csv(telemetry_path) if telemetry_path.exists() else pd.DataFrame()
    df_msg = pd.read_csv(messages_path) if messages_path.exists() else pd.DataFrame()
    events = json.loads(events_path.read_text(encoding="utf-8")) if events_path.exists() else []

    if df_tel.empty:
        return

    # Base plots
    plot_speed_vs_time(df_tel, events, output_dir / "speed_vs_time.png")
    plot_gap_vs_time(df_tel, events, output_dir / "gap_vs_time.png")
    plot_gap_error(df_tel, events, output_dir / "gap_error_vs_time.png")
    plot_acceleration(df_tel, events, output_dir / "acceleration_vs_time.png")
    plot_jerk(df_tel, events, output_dir / "jerk_vs_time.png")
    plot_position(df_tel, events, output_dir / "position_vs_time.png")

    # Conditional V2X plots
    scenario_id = spec.get("id", "")
    event_types = {e.get("type", "") for e in spec.get("events", [])}

    is_v2x_scenario = "v2x" in scenario_id or bool(event_types.intersection({"v2x_delay", "packet_loss", "v2x_outage"}))
    if is_v2x_scenario or not df_msg.empty:
        plot_v2x_latency(df_msg, events, output_dir / "v2x_latency_vs_time.png")
        plot_packet_loss(df_msg, events, output_dir / "packet_loss_vs_time.png")

    # Conditional Maneuver plots
    is_lane_scenario = bool(event_types.intersection({"cut_in", "cut_out", "lane_change", "onramp_merge", "roadwork", "slow_vehicle"}))
    if is_lane_scenario:
        plot_lane_event_timeline(df_tel, events, output_dir / "lane_event_timeline.png")

    # Conditional Merge / Split plots
    is_merge_split_scenario = bool(event_types.intersection({"platoon_merge", "platoon_split"}))
    if is_merge_split_scenario:
        plot_membership_timeline(df_tel, events, output_dir / "membership_timeline.png")
