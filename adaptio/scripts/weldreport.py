import json
import argparse
import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import os
import textwrap  # for robust preface wrapping
from datetime import datetime  # added for title page date/time extraction

ANGLE_OFFSET_DEG = 2  # angular window +/-
DEFAULT_STICKOUT = 30  # mm
POS_DEG_LIST = list(range(0, 360, 45))  # angles to sweep in 45-degree steps
ABW_LAYER_TARGET = 5  # target layer number for second ABW profile in blue

def compute_bead_area(weld_systems, radius_mm, angular_velocity):
    total_volume_per_s = 0.0
    for ws in weld_systems[:2]:
        d = ws.get("wireDiameter", 0)
        v = ws.get("wireSpeed", 0)
        area = np.pi * (d / 2) ** 2
        volume_per_s = area * v  # mm³/s
        total_volume_per_s += volume_per_s

    travel_speed = radius_mm * angular_velocity  # mm/s
    return total_volume_per_s / travel_speed if travel_speed else 0.0

def angle_in_range(center_deg, offset_deg):
    center_rad = np.deg2rad(center_deg)
    offset_rad = np.deg2rad(offset_deg)
    lower = (center_rad - offset_rad) % (2 * np.pi)
    upper = (center_rad + offset_rad) % (2 * np.pi)
    if lower < upper:
        return lambda angle: lower <= angle <= upper
    else:
        return lambda angle: angle >= lower or angle <= upper

def average_cluster(cluster):
    if not cluster:
        return None
    horiz = np.mean([e[0] for e in cluster])
    vert = np.mean([e[1] for e in cluster])
    bead_area = np.mean([e[2] for e in cluster])
    current = np.mean([e[3] for e in cluster])
    weld_speed = np.mean([e[4] for e in cluster])
    heat_input = np.mean([e[5] for e in cluster])
    bead_slice_area_ratio = np.mean([e[6] for e in cluster])
    groove_area_ratio = np.mean([e[7] for e in cluster])
    bead_no = cluster[0][8]
    layer_no = cluster[0][9]
    return (
        horiz, vert,
        bead_area, current, weld_speed, heat_input,
        bead_slice_area_ratio, groove_area_ratio,
        bead_no, layer_no
    )

# --- Preface helpers (minimal, self-contained) ---

def _load_preface_text():
    """
    Try to load a user-provided 'weldreport_preface.md' from the script directory or CWD.
    Falls back to a sensible default if no file is found.
    """
    candidates = [
        os.path.join(os.path.dirname(__file__), "weldreport_preface.md"),
        os.path.join(os.getcwd(), "weldreport_preface.md"),
    ]
    for p in candidates:
        if os.path.exists(p):
            try:
                with open(p, "r", encoding="utf-8") as f:
                    return f.read()
            except Exception:
                pass

    # Default preface text (kept short and neutral)
    return (
        "# Preface\n\n"
    )

def _wrap_markdown_like(text: str, width: int = 92) -> str:
    """
    Lightweight wrapper that keeps bullets and headings readable and prevents long tokens
    from overflowing. We pre-wrap the text and draw it without 'wrap=True' in Matplotlib.
    """
    out = []
    for line in text.splitlines():
        s = line.rstrip()
        if not s:
            out.append("")  # blank line between paragraphs
            continue

        # Headings: leave as-is (short lines)
        if s.lstrip().startswith("#"):
            out.append(s)
            continue

        # Bulleted lists: preserve bullet and indent wrapped lines
        stripped = s.lstrip()
        indent_len = len(s) - len(stripped)
        if stripped.startswith(("-", "*")):
            bullet = stripped.split(maxsplit=1)[0]  # "-" or "*"
            content = stripped[len(bullet):].lstrip()
            prefix = " " * indent_len + bullet + " "
            wrapped = textwrap.fill(
                content,
                width=max(20, width - len(prefix)),
                initial_indent=prefix,
                subsequent_indent=" " * len(prefix),
                break_long_words=True,
                break_on_hyphens=False,
            )
            out.append(wrapped)
            continue

        # Normal paragraph
        out.append(textwrap.fill(
            s,
            width=width,
            break_long_words=True,
            break_on_hyphens=False,
        ))
    return "\n".join(out)

def _extract_title_info(input_path):
    """
    Extract a human-friendly date and ABP start time from the first ABP entry in the log.
    - Date and time: from the first entry with 'mode' == 'abp'.
    Returns (date_str, time_str) like '23 June 2025', '10:25'. Falls back to 'Unknown' if missing.
    """
    def _parse_ts(ts):
        try:
            dt = datetime.fromisoformat(ts.replace("Z", "+00:00"))
            # Format as 'D Month YYYY' (e.g., '23 June 2025')
            return dt.strftime("%-d %B %Y"), dt.strftime("%H:%M")
        except Exception:
            return None, None

    date_str = None
    time_str = None

    with open(input_path, "r") as f:
        for line in f:
            try:
                obj = json.loads(line)

                # First ABP entry determines both date and time
                if obj.get("mode") == "abp" and "timestamp" in obj:
                    date_str, time_str = _parse_ts(obj["timestamp"])
                    break

            except Exception:
                continue

    return date_str or "Unknown date", time_str or "Unknown time"


def load_and_filter_entries(input_path, pos_deg, stickout, hide_jt=False, post_scan=False):
    angle_filter = angle_in_range(center_deg=pos_deg, offset_deg=ANGLE_OFFSET_DEG)
    entries = []
    abw_profile_points = []
    abw_profile_points_post_scan = []     # Optional: last-in-log ABW profile (green)

    # ABP clustering state
    cluster = []
    inside = False
    last_bead_no = last_layer_no = None
    abw_collected = False

    # JT clustering state: collect JT points inside window; bead/layer = None
    cluster_jt = []
    inside_jt = False

    # track layer adjustment across ABP sessions
    layer_no_adjustment = 0
    max_layer_seen = 0
    in_abp_session = False

    # Track the last candidate mcs inside the window for post-scan profile
    last_abw_candidate = None  # will hold list of (x_rel, z_rel_plus_stickout)

    with open(input_path, "r") as infile:
        for i, line in enumerate(infile):
            try:
                entry = json.loads(line)

                # Detect ABP start event
                if entry.get("annotation") == "adaptio-state-change" and entry.get("mode") == "abp":
                    # On subsequent ABP starts, adjust based on max seen so far
                    if in_abp_session:
                        layer_no_adjustment = max_layer_seen
                    in_abp_session = True
                    continue  # Skip this event for plotting

                mode = entry.get("mode")
                mcs = entry.get("mcs", [])

                # Capture first ABW profile as early as possible (independent of ABP/JT)
                # Take the first entry that is inside the angular window and has >= 7 mcs points.
                if (not abw_collected) and mcs and len(mcs) >= 7:
                    position = entry.get("weldAxis", {}).get("position")
                    if position is not None and angle_filter(position):
                        ref_x = mcs[0]["x"]
                        ref_z = mcs[0]["z"]
                        abw_profile_points = [
                            (pt["x"] - ref_x, (pt["z"] - ref_z) + stickout)
                            for pt in mcs[:7]
                        ]
                        abw_collected = True

                # Track the last-in-log ABW profile candidate (independent of ABP/JT)
                if post_scan and mcs and len(mcs) >= 7:
                    position = entry.get("weldAxis", {}).get("position")
                    if position is not None and angle_filter(position):
                        ref_x = mcs[0]["x"]
                        ref_z = mcs[0]["z"]
                        last_abw_candidate = [
                            (pt["x"] - ref_x, (pt["z"] - ref_z) + stickout)
                            for pt in mcs[:7]
                        ]

                # Only ABP "steady" contributes to ABP plots
                if entry.get("beadControl", {}).get("state") == "steady" and mode == "abp":
                    bead_no = entry.get("beadControl", {}).get("beadNo")
                    # Apply adjustment to layer number
                    layer_no = entry.get("beadControl", {}).get("layerNo", 0) + layer_no_adjustment

                    # Update max_layer_seen whenever we get a steady ABP entry
                    if layer_no > max_layer_seen:
                        max_layer_seen = layer_no

                    if not mcs:
                        continue

                    # Reference point for relative coordinates
                    ref_x = mcs[0]["x"]
                    ref_z = mcs[0]["z"]

                    slides = entry["slides"]["actual"]
                    # Store horizontal/vertical relative to mcs[0]
                    vertical = slides["vertical"] - ref_z
                    horizontal = slides["horizontal"] - ref_x

                    velocity_actual = entry["weldAxis"]["velocity"]["actual"]
                    position = entry["weldAxis"]["position"]
                    radius = entry["weldAxis"]["radius"]
                    weld_systems = entry["weldSystems"]

                    # New metrics from beadControl (default to 0.0 if missing)
                    bc = entry.get("beadControl", {})
                    bead_slice_area_ratio = float(bc.get("beadSliceAreaRatio", 0.0))
                    groove_area_ratio = float(bc.get("grooveAreaRatio", 0.0))

                    is_inside = angle_filter(position)

                    if is_inside:
                        # If this is the first entry inside the window and abw profile has not yet been collected:
                        if not abw_collected and len(mcs) >= 7:
                            # abw points relative to ref_x/ref_z, z offset with stickout
                            abw_profile_points = [
                                (pt["x"] - ref_x, (pt["z"] - ref_z) + stickout)
                                for pt in mcs[:7]
                            ]
                            abw_collected = True

                        # Clustering logic:
                        # We average over consecutive entries as long as beadNo and layerNo remain constant.
                        # If we detect a new bead/layer, we finalize the current cluster by averaging.
                        if inside and (bead_no != last_bead_no or layer_no != last_layer_no):
                            avg_entry = average_cluster(cluster)
                            if avg_entry:
                                entries.append(avg_entry)
                            cluster = []

                        # Compute per-entry values to later average over
                        bead_area = compute_bead_area(weld_systems, radius, velocity_actual)
                        current = weld_systems[1].get("current", {}).get("actual", 0)
                        travel_speed = radius * velocity_actual  # weld speed
                        heat_input = sum(ws.get("heatInput", 0) for ws in weld_systems[:2])

                        # Accumulate this entry into the current cluster
                        cluster.append(
                            (
                                horizontal, vertical,
                                bead_area, current, travel_speed, heat_input,
                                bead_slice_area_ratio, groove_area_ratio,
                                bead_no, layer_no
                            )
                        )

                        # Mark that we are currently inside the window — used for transition detection
                        inside = True
                        last_bead_no = bead_no
                        last_layer_no = layer_no

                    elif inside:
                        # We were inside, but are now outside the window — finalize the current cluster
                        avg_entry = average_cluster(cluster)
                        if avg_entry:
                            entries.append(avg_entry)
                        cluster = []
                        inside = False  # Reset state to mark we are now outside

                # Optionally include JT: require both weld systems arcing; JT does not affect ABP layering
                elif (mode == "jt") and (not hide_jt):
                    weld_systems = entry.get("weldSystems", [])
                    if len(weld_systems) >= 2:
                        ws0_arcing = weld_systems[0].get("state") == "arcing"
                        ws1_arcing = weld_systems[1].get("state") == "arcing"
                        if not (ws0_arcing and ws1_arcing):
                            continue

                        if not mcs:
                            continue

                        # Reference point for relative coordinates
                        ref_x = mcs[0]["x"]
                        ref_z = mcs[0]["z"]

                        slides = entry["slides"]["actual"]
                        vertical = slides["vertical"] - ref_z
                        horizontal = slides["horizontal"] - ref_x

                        velocity_actual = entry.get("weldAxis", {}).get("velocity", {}).get("actual", 0.0)
                        position = entry.get("weldAxis", {}).get("position")
                        radius = entry.get("weldAxis", {}).get("radius", 0.0)

                        bead_area = compute_bead_area(weld_systems, radius, velocity_actual)
                        current = weld_systems[1].get("current", {}).get("actual", 0)
                        travel_speed = radius * velocity_actual  # weld speed
                        heat_input = sum(ws.get("heatInput", 0) for ws in weld_systems[:2])

                        # Metrics from beadControl if available (fallbacks to 0.0)
                        bc = entry.get("beadControl", {})
                        bead_slice_area_ratio = float(bc.get("beadSliceAreaRatio", 0.0))
                        groove_area_ratio = float(bc.get("grooveAreaRatio", 0.0))

                        is_inside = angle_filter(position)

                        if is_inside:
                            # For JT, we store bead_no=None, layer_no=None
                            cluster_jt.append(
                                (
                                    horizontal, vertical,
                                    bead_area, current, travel_speed, heat_input,
                                    bead_slice_area_ratio, groove_area_ratio,
                                    None, None
                                )
                            )
                            inside_jt = True
                        elif inside_jt:
                            avg_entry = average_cluster(cluster_jt)
                            if avg_entry:
                                entries.append(avg_entry)
                            cluster_jt = []
                            inside_jt = False

            except Exception as e:
                print(f"Skipping entry {i} due to error: {e}")

    # Finalize any open clusters
    if cluster:
        avg_entry = average_cluster(cluster)
        if avg_entry:
            entries.append(avg_entry)
    if cluster_jt:
        avg_entry = average_cluster(cluster_jt)
        if avg_entry:
            entries.append(avg_entry)

    # Emit the last-in-log ABW profile for this angle if requested
    if post_scan and last_abw_candidate:
        abw_profile_points_post_scan = last_abw_candidate

    # --- Merge duplicate ABP labels inside the same window (wrap-around at 0° can create two runs) ---
    # We group by (layer_no, bead_no) and average, but keep JT (None labels) as-is.
    if entries:
        grouped = {}
        passthrough = []  # JT or unlabeled points

        for e in entries:
            bead_no, layer_no = e[8], e[9]
            if bead_no is None or layer_no is None:
                passthrough.append(e)
            else:
                grouped.setdefault((layer_no, bead_no), []).append(e)

        merged = [average_cluster(cluster) for cluster in grouped.values()]
        entries = merged + passthrough

    return entries, abw_profile_points, abw_profile_points_post_scan

def plot_flat(entries, abw_profile_points, abw_profile_points_post_scan, pos_deg, ranges):
    bead_areas = [e[2] for e in entries]
    currents = [e[3] for e in entries]
    weld_speeds = [e[4] for e in entries]
    heat_inputs = [e[5] for e in entries]
    bead_slice_area_ratios = [e[6] for e in entries]
    groove_area_ratios = [e[7] for e in entries]

    # 3 rows x 2 cols to match requested order
    fig, axs = plt.subplots(3, 2, figsize=(14, 15), sharey=True)
    fig.suptitle(f"Weld Data Analysis — pos_deg = {pos_deg}°", fontsize=14)

    ax1 = axs[0, 0]  # Bead area
    ax2 = axs[0, 1]  # Heat input
    ax3 = axs[1, 0]  # Current
    ax4 = axs[1, 1]  # Weld speed
    ax5 = axs[2, 0]  # Bead slice area ratio
    ax6 = axs[2, 1]  # Groove area ratio

    cmap = plt.get_cmap("viridis_r")

    def add_plot(ax, values, title, cbar_label, vmin, vmax):
        norm = Normalize(vmin=vmin, vmax=vmax)
        xs = [e[0] for e in entries]
        ys = [e[1] for e in entries]
        ax.scatter(xs, ys, c=values, cmap=cmap, norm=norm, s=50, linewidth=0.3)

        # Only label ABP points (where bead_no/layer_no are not None)
        for (x, y, *_rest, bead_no, layer_no) in entries:
            if bead_no is not None and layer_no is not None:
                ax.text(x, y, f"({layer_no},{bead_no})", fontsize=9, ha='center', va='bottom')

        # Gray ABW profile (first-in-log)
        if abw_profile_points:
            abw_xs, abw_ys = zip(*abw_profile_points)
            ax.plot(abw_xs, abw_ys, color='gray', marker='x')

        # Optional green ABW profile from last-in-log (post-scan)
        if abw_profile_points_post_scan:
            abw_xs_p, abw_ys_p = zip(*abw_profile_points_post_scan)
            ax.plot(abw_xs_p, abw_ys_p, color='green', marker='x')

        ax.set_title(title)
        ax.set_xlabel("torch x (rel abw 0, mm)")
        ax.set_ylabel("torch z (rel abw 0, mm)")
        ax.invert_xaxis()
        ax.grid(True)
        ax.set_aspect("equal")

        cbar = fig.colorbar(
            ScalarMappable(norm=norm, cmap=cmap),
            ax=ax,
            orientation="horizontal",
            pad=0.15,
            aspect=40
        )
        cbar.set_label(cbar_label)

    # Use fixed shared ranges
    add_plot(ax1, bead_areas, "Bead area (mm²)", "Bead Area (mm²)",
             ranges["bead_area"][0], ranges["bead_area"][1])
    add_plot(ax2, heat_inputs, "Heat input (kJ/mm)", "Heat input (kJ/mm)",
             ranges["heat_input"][0], ranges["heat_input"][1])
    add_plot(ax3, currents, "Current in weld system 2 (A)", "Current (A)",
             ranges["current"][0], ranges["current"][1])
    add_plot(ax4, weld_speeds, "Weld speed (mm/s)", "Weld speed (mm/s)",
             ranges["weld_speed"][0], ranges["weld_speed"][1])
    add_plot(ax5, bead_slice_area_ratios, "Bead slice area ratio", "Bead slice area ratio (–)",
             ranges["bead_slice_area_ratio"][0], ranges["bead_slice_area_ratio"][1])
    add_plot(ax6, groove_area_ratios, "Groove area ratio", "Groove area ratio (–)",
             ranges["groove_area_ratio"][0], ranges["groove_area_ratio"][1])

    plt.tight_layout(rect=(0, 0, 1, 0.95))
    return fig

def main():
    parser = argparse.ArgumentParser(
        description="Generate weld analysis report as a PDF with bead area, current, weld speed, and heat input plots."
    )
    parser.add_argument("--input", required=True, help="Path to input JSONL file")
    # Optional: hide JT (joint tracking) points from plots
    parser.add_argument("--hide-jt", action="store_true", help="Hide JT (joint tracking) points in plots")
    # Optional: add post-scan ABW profile (last-in-log) in green
    parser.add_argument("--post-scan", action="store_true", help="Add post-scan ABW profile (last in log) in green")
    args = parser.parse_args()

    # Extract info for the title page (date and ABP start time)
    report_date, abp_start_time = _extract_title_info(args.input)

    # Parse configuration from last relevant line
    config = {}
    stickout = DEFAULT_STICKOUT  # fallback
    with open(args.input, "r") as infile:
        for line in infile:
            try:
                obj = json.loads(line)
                if obj.get("type") == "modeChange" and "abpParameters" in obj:
                    config = obj["abpParameters"]
                    stickout = obj.get("verticalOffset", stickout)
            except Exception:
                continue

    # Round config values to 1 decimal
    rounded_config = {
        k: round(v, 1) if isinstance(v, (int, float)) else v
        for k, v in config.items()
    }

    # Format weldSpeed specifically with cm/min and mm/s units
    if "weldSpeed" in config:
        ws = config["weldSpeed"]
        if isinstance(ws, dict) and "min" in ws and "max" in ws:
            ws_cm_min = {k: round(v, 1) for k, v in ws.items()}
            ws_mm_s = {k: round(v * 10 / 60, 1) for k, v in ws.items()}
            rounded_config["weldSpeed(cm/min)"] = ws_cm_min
            rounded_config["weldSpeed(mm/s)"] = ws_mm_s
            del rounded_config["weldSpeed"]

    basename = os.path.splitext(os.path.basename(args.input))[0]
    output_pdf = f"weldreport_{basename}.pdf"

    # --------- First pass to gather global ranges and cache entries per pos_deg ---------
    entries_by_pos = {}
    global_bead_area = []
    global_bead_slice_area_ratio = []
    global_groove_area_ratio = []
    global_current = []
    global_weld_speed = []
    global_heat_input = []

    for pos_deg in POS_DEG_LIST:
        entries, abw_profile_points, abw_profile_points_post_scan = load_and_filter_entries(
            args.input, pos_deg, stickout, hide_jt=args.hide_jt, post_scan=args.post_scan
        )
        if entries:
            entries_by_pos[pos_deg] = (entries, abw_profile_points, abw_profile_points_post_scan)
            # Gather values for global ranges where requested
            global_bead_area.extend([e[2] for e in entries])
            global_current.extend([e[3] for e in entries])
            global_weld_speed.extend([e[4] for e in entries])
            global_heat_input.extend([e[5] for e in entries])
            global_bead_slice_area_ratio.extend([e[6] for e in entries])
            global_groove_area_ratio.extend([e[7] for e in entries])
        else:
            entries_by_pos[pos_deg] = ([], [], [])

    def _safe_minmax(vals, default=(0.0, 1.0)):
        vals = [v for v in vals if v is not None]
        if not vals:
            return default
        vmin = min(vals)
        vmax = max(vals)
        if vmin == vmax:
            # widen slightly to avoid zero range in Normalize
            eps = 1e-6 if vmin == 0 else abs(vmin) * 1e-6
            return (vmin - eps, vmax + eps)
        return (vmin, vmax)

    # Ranges from ABP params (with fallbacks to data if missing)
    # Heat input (kJ/mm) from abpParameters. Example key: config["heatInput"] = {"min":..., "max":...}
    if isinstance(config.get("heatInput"), dict):
        hi_min = config["heatInput"].get("min")
        hi_max = config["heatInput"].get("max")
        heat_input_range = _safe_minmax([hi_min, hi_max]) if (hi_min is not None and hi_max is not None) \
            else _safe_minmax(global_heat_input)
    else:
        heat_input_range = _safe_minmax(global_heat_input)

    # Current from abpParameters. Example key: weldSystem2Current {min,max}
    if isinstance(config.get("weldSystem2Current"), dict):
        c_min = config["weldSystem2Current"].get("min")
        c_max = config["weldSystem2Current"].get("max")
        current_range = _safe_minmax([c_min, c_max]) if (c_min is not None and c_max is not None) \
            else _safe_minmax(global_current)
    else:
        current_range = _safe_minmax(global_current)

    # Weld speed from abpParameters (convert cm/min to mm/s with *10/60)
    if isinstance(config.get("weldSpeed"), dict):
        ws_min = config["weldSpeed"].get("min")
        ws_max = config["weldSpeed"].get("max")
        if ws_min is not None and ws_max is not None:
            weld_speed_range = _safe_minmax([ws_min * 10.0 / 60.0, ws_max * 10.0 / 60.0])
        else:
            weld_speed_range = _safe_minmax(global_weld_speed)
    else:
        weld_speed_range = _safe_minmax(global_weld_speed)

    # Ranges computed from all applicable pos_deg
    bead_area_range = _safe_minmax(global_bead_area)
    bead_slice_area_ratio_range = _safe_minmax(global_bead_slice_area_ratio)
    groove_area_ratio_range = _safe_minmax(global_groove_area_ratio)

    shared_ranges = {
        "bead_area": bead_area_range,
        "heat_input": heat_input_range,
        "current": current_range,
        "weld_speed": weld_speed_range,
        "bead_slice_area_ratio": bead_slice_area_ratio_range,
        "groove_area_ratio": groove_area_ratio_range,
    }
    # --------- END first pass ---------

    with PdfPages(output_pdf) as pdf:
        # --- Title page ---
        fig_title = plt.figure(figsize=(8.5, 11))
        ax_title = fig_title.add_subplot(111)
        ax_title.axis("off")
        title_line = f"Weld Report - {report_date}"
        subtitle_line = f"(ABP started {abp_start_time})"
        ax_title.text(0.5, 0.60, title_line, fontsize=26, ha="center", va="center", weight="bold")
        ax_title.text(0.5, 0.52, subtitle_line, fontsize=16, ha="center", va="center")
        pdf.savefig(fig_title)
        plt.close(fig_title)

        # Preface page (reads weldreport_preface.md if present)
        preface_text = _load_preface_text()
        preface_text = _wrap_markdown_like(preface_text, width=110)

        fig_preface = plt.figure(figsize=(8.5, 11))
        fig_preface.suptitle("Preface", fontsize=16)
        axp = fig_preface.add_subplot(111)
        axp.axis("off")
        # Balanced margins and a little more room up top
        fig_preface.subplots_adjust(left=0.06, right=0.97, top=0.92)
        # Smaller font + manual wrapping (no 'wrap=True' now)
        axp.text(0.0, 0.98, preface_text, fontsize=9, va="top", ha="left")
        pdf.savefig(fig_preface)
        plt.close(fig_preface)

        # First page: weld settings
        fig_settings = plt.figure(figsize=(8.5, 6))
        fig_settings.suptitle("Weld Settings", fontsize=16)
        ax = fig_settings.add_subplot(111)
        ax.axis("off")
        lines = [f"{k}: {v}" for k, v in rounded_config.items()]
        ax.text(0, 1, "\n".join(lines), fontsize=12, verticalalignment='top')
        pdf.savefig(fig_settings)
        plt.close(fig_settings)

        # Plot per pos_deg (now using shared color ranges)
        for pos_deg in POS_DEG_LIST:
            entries, abw_profile_points, abw_profile_points_post_scan = entries_by_pos[pos_deg]
            if entries:
                fig = plot_flat(
                    entries,
                    abw_profile_points,
                    abw_profile_points_post_scan,
                    pos_deg,
                    shared_ranges
                )
                pdf.savefig(fig)
                plt.close(fig)
            else:
                print(f"No valid data for pos_deg={pos_deg}")

    print(f"Report written to {output_pdf}")

if __name__ == "__main__":
    main()
