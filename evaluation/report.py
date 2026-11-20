"""
SAMTRUCK HTML Report Generator Module
Generates self-contained report.html for scenario evaluation.
"""
from __future__ import annotations

from pathlib import Path
import json


def generate_report(output_dir: Path, spec: dict, metrics: dict, events: list[dict]):
    """Generates a comprehensive report.html from real results only."""
    scenario_id = spec.get("id", "unknown")
    title = spec.get("title", scenario_id)
    duration = spec.get("duration_s", 0)

    pass_status = metrics.get("pass_fail", {}).get("status", "FAIL")
    pass_reason = metrics.get("pass_fail", {}).get("reason", "")
    status_color = "#28a745" if pass_status == "PASS" else "#dc3545"

    # Gather generated graph images in output_dir
    image_files = sorted(output_dir.glob("*.png"))

    # Build Events Table
    events_html = ""
    if events:
        events_html = """
        <table class="table">
            <thead>
                <tr>
                    <th>Time (s)</th>
                    <th>Event</th>
                    <th>Duration (s)</th>
                    <th>Applied</th>
                    <th>Detail</th>
                </tr>
            </thead>
            <tbody>
        """
        for e in events:
            time_s = e.get("time_s", e.get("at_s", 0.0))
            event_name = e.get("event", e.get("type", ""))
            dur_s = e.get("duration_s", "-")
            applied = e.get("applied", False)
            detail = e.get("detail", "-")
            applied_badge = '<span class="badge badge-pass">True</span>' if applied else '<span class="badge badge-fail">False</span>'

            events_html += f"""
                <tr>
                    <td>{time_s}</td>
                    <td>{event_name}</td>
                    <td>{dur_s}</td>
                    <td>{applied_badge}</td>
                    <td>{detail}</td>
                </tr>
            """
        events_html += "</tbody></table>"
    else:
        events_html = "<p>No scenario events scheduled (Baseline Platooning).</p>"

    # Build String Stability Table
    string_stab = metrics.get("string_stability", {})
    formula = string_stab.get("formula", "N/A")
    speed_dist = string_stab.get("speed_disturbance_m_s", {})
    accel_dist = string_stab.get("accel_disturbance_m_s2", {})
    prop_ratio = string_stab.get("speed_propagation_ratio", {})

    stab_html = f"""
    <p><strong>Formula:</strong> <code>{formula}</code></p>
    <table class="table">
        <thead>
            <tr>
                <th>Vehicle</th>
                <th>Speed Disturbance D_v (m/s)</th>
                <th>Accel Disturbance D_v (m/s²)</th>
                <th>Speed Propagation Ratio R_v</th>
            </tr>
        </thead>
        <tbody>
    """
    for veh in ["samtruck_leader", "samtruck_follower_1", "samtruck_follower_2"]:
        s_d = speed_dist.get(veh, "-")
        a_d = accel_dist.get(veh, "-")
        p_r = prop_ratio.get(veh, "-") if veh != "samtruck_leader" else "Baseline (1.0)"
        stab_html += f"""
            <tr>
                <td><strong>{veh}</strong></td>
                <td>{s_d}</td>
                <td>{a_d}</td>
                <td>{p_r}</td>
            </tr>
        """
    stab_html += "</tbody></table>"

    # Images Gallery
    images_html = ""
    for img_path in image_files:
        img_name = img_path.name
        img_title = img_name.replace("_", " ").replace(".png", "").title()
        images_html += f"""
        <div class="card img-card">
            <h4>{img_title}</h4>
            <img src="{img_name}" alt="{img_title}" />
        </div>
        """

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>SAMTRUCK Scenario Report: {title}</title>
    <style>
        body {{
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            line-height: 1.6;
            color: #333;
            background-color: #f8f9fa;
            margin: 0;
            padding: 20px;
        }}
        .container {{
            max-width: 1100px;
            margin: 0 auto;
            background: #fff;
            padding: 30px;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.08);
        }}
        h1, h2, h3, h4 {{
            color: #111;
        }}
        .header {{
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 2px solid #eee;
            padding-bottom: 15px;
            margin-bottom: 25px;
        }}
        .status-badge {{
            background-color: {status_color};
            color: white;
            padding: 8px 18px;
            font-size: 1.2rem;
            font-weight: bold;
            border-radius: 20px;
            text-transform: uppercase;
        }}
        .badge {{
            padding: 3px 8px;
            border-radius: 4px;
            font-size: 0.85rem;
            font-weight: bold;
        }}
        .badge-pass {{ background: #d4edda; color: #155724; }}
        .badge-fail {{ background: #f8d7da; color: #721c24; }}
        .table {{
            width: 100%;
            border-collapse: collapse;
            margin-bottom: 25px;
        }}
        .table th, .table td {{
            border: 1px solid #dee2e6;
            padding: 10px 14px;
            text-align: left;
        }}
        .table th {{
            background-color: #f1f3f5;
        }}
        .notice-box {{
            background-color: #e9ecef;
            border-left: 4px solid #6c757d;
            padding: 12px 18px;
            margin-bottom: 25px;
            border-radius: 4px;
            font-size: 0.95rem;
        }}
        .warning-box {{
            background-color: #fff3cd;
            border-left: 4px solid #ffc107;
            padding: 12px 18px;
            margin-bottom: 25px;
            border-radius: 4px;
            font-size: 0.95rem;
        }}
        .gallery {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(480px, 1fr));
            gap: 20px;
            margin-top: 20px;
        }}
        .img-card {{
            border: 1px solid #e2e8f0;
            border-radius: 6px;
            padding: 12px;
            background: #fafafa;
        }}
        .img-card img {{
            width: 100%;
            height: auto;
            border-radius: 4px;
        }}
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <div>
                <h1>SAMTRUCK Scenario Evaluation Report</h1>
                <h3>Scenario: {title} (<code>{scenario_id}</code>)</h3>
            </div>
            <div class="status-badge">{pass_status}</div>
        </div>

        <div class="notice-box">
            <strong>Execution Validation Basis:</strong> {pass_reason}<br/>
            <em>Note: PASS reflects execution and event validation only, not a safety-performance certification.</em>
        </div>

        <h2>1. Applied Scenario Events</h2>
        {events_html}

        <h2>2. Quantitative Metrics</h2>
        <table class="table">
            <thead>
                <tr>
                    <th>Metric Name</th>
                    <th>Value</th>
                    <th>Description</th>
                </tr>
            </thead>
            <tbody>
                <tr><td>Average Gap</td><td><strong>{metrics.get('average_gap_m', '-')} m</strong></td><td>Mean inter-vehicle follower gap</td></tr>
                <tr><td>Average Desired Gap</td><td><strong>{metrics.get('average_desired_gap_m', '-')} m</strong></td><td>Mean target CTG gap calculated by controller</td></tr>
                <tr><td>Mean Absolute Gap Error (MAE)</td><td><strong>{metrics.get('mean_absolute_gap_error_m', '-')} m</strong></td><td>Average absolute gap deviation</td></tr>
                <tr><td>RMS Gap Error</td><td><strong>{metrics.get('rms_gap_error_m', '-')} m</strong></td><td>Root-mean-square gap error</td></tr>
                <tr><td>Maximum Gap Error</td><td><strong>{metrics.get('maximum_gap_error_m', '-')} m</strong></td><td>Worst-case gap error peak</td></tr>
                <tr><td>Minimum Gap Observed</td><td><strong>{metrics.get('minimum_gap_m', '-')} m</strong></td><td>Closest bumper-to-bumper proximity</td></tr>
                <tr><td>Maximum Speed</td><td><strong>{metrics.get('maximum_speed_mps', '-')} m/s</strong></td><td>Peak speed across platoon</td></tr>
                <tr><td>Maximum Acceleration</td><td><strong>{metrics.get('maximum_acceleration_mps2', '-')} m/s²</strong></td><td>Peak positive acceleration</td></tr>
                <tr><td>Maximum Deceleration</td><td><strong>{metrics.get('maximum_deceleration_mps2', '-')} m/s²</strong></td><td>Peak braking deceleration magnitude</td></tr>
                <tr><td>Maximum Jerk</td><td><strong>{metrics.get('maximum_jerk_mps3', '-')} m/s³</strong></td><td>Peak derivative of acceleration (da/dt)</td></tr>
                <tr><td>Collision Count</td><td><strong>{metrics.get('collision_count', 0)}</strong></td><td>Total physical collisions detected by SUMO</td></tr>
                <tr><td>Disturbance Recovery Time</td><td><strong>{metrics.get('recovery_time_s', 'null')} s</strong></td><td>Time post-event until |gap_error| ≤ 1.0m for ≥ 1.0s</td></tr>
                <tr><td>V2X Average Latency</td><td><strong>{metrics.get('v2x_average_latency_ms', '-')} ms</strong></td><td>Mean V2V network latency</td></tr>
                <tr><td>V2X Messages Transmitted</td><td><strong>{metrics.get('transmitted_messages', 0)}</strong></td><td>Total message send attempts</td></tr>
                <tr><td>V2X Messages Dropped</td><td><strong>{metrics.get('dropped_messages', 0)}</strong></td><td>Total messages lost/dropped</td></tr>
                <tr><td>Observed Packet Loss</td><td><strong>{metrics.get('observed_packet_loss_percent', 0.0)} %</strong></td><td>Percentage packet drop rate</td></tr>
            </tbody>
        </table>

        <div class="warning-box">
            <strong>Fuel & Energy Consumption:</strong><br/>
            Status: <code>{metrics.get('fuel_energy', {}).get('status', 'unavailable')}</code><br/>
            Reason: {metrics.get('fuel_energy', {}).get('reason', 'No SUMO fuel or energy model is configured for platoon_truck.')}
        </div>

        <h2>3. String Stability Analysis</h2>
        {stab_html}

        <h2>4. Real-Data Performance Visualizations</h2>
        <div class="gallery">
            {images_html}
        </div>
    </div>
</body>
</html>
"""

    report_file = output_dir / "report.html"
    report_file.write_text(html_content, encoding="utf-8")
