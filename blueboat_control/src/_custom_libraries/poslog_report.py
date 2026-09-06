#!/usr/bin/env python3

r"""
Post-mission report for the robot-side position CSV.

Reads `<root>/data/Robot_data/{date}-{note}-poslog.csv` (either layout, detected
from the header), renders one PNG for quick analysis, then files the run away as

    Robot_data/<csv stem>/
        <csv stem>.csv          the poslog, MOVED here
        <date>-<note>-origin.yaml   its world-frame origin sidecar, MOVED here
        <csv stem>.png          the report

ROS-FREE. numpy is required; matplotlib is imported lazily inside the renderer
and is NOT a hard dependency - `blueboat_control/package.xml` declares neither,
and a flight node must never fail to start because a plotting library is absent.
`finalise_run` therefore always creates the folder and moves the files, and only
the picture is optional.

WRITE-ONCE (superproject CM-7 / robot_log_schema's N7). This module MOVES a
primary field record; it never rewrites one, never regenerates one, and never
touches a destination folder that already exists. If a run has already been
filed, re-running is a no-op that says so.

Run it by hand over anything already on disk:

    ros2 run blueboat_control poslog_report.py ~/ros2_ws/data/Robot_data/2026_09_04-14_18_19-poslog.csv
    ros2 run blueboat_control poslog_report.py ~/ros2_ws/data/Robot_data --all
    ... --no-archive     render the PNG beside the CSV, move nothing
"""

import argparse
import csv
import math
import os
import re
import shutil
import sys

import numpy as np

# ---------------------------------------------------------------------------
# House style. Same validated light-mode palette as
# blueboat_control/src/docs/controllers/gen_figures.py, so every figure this
# project produces reads as one system.
# ---------------------------------------------------------------------------
SURFACE = "#fcfcfb"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#8a8983"
GRID = "#e6e5e1"
ROBOT = "#2a78d6"      # categorical slot 1
TARGET = "#eb6834"     # categorical slot 2
REF = "#9a998f"
GOOD, WARN, BAD = "#1baf7a", "#d68f00", "#e34948"

# Actuation-state encoding, mirrored from robot_log_schema so this module can be
# read on a machine that has only the CSV. Order is the encoding's own.
ACT_LABELS = {
    0: ("motors disabled", MUTED),
    1: ("live", GOOD),
    2: ("not in override", WARN),
    3: ("watchdog zeroing", BAD),
}

EARTH_R = 6371000.0

# A BlueBoat tops out near 2 m/s. Anything above this in a pose-differenced
# speed is not the boat moving, it is the pose jumping - an odom re-origin, a
# GPS/EKF glitch, a relaunch under a running log. Those samples are EXCLUDED
# from the statistics and COUNTED in the summary rather than clipped away: a run
# whose pose teleports is a finding, not a rendering nuisance, and one 300 m/s
# spike otherwise sets the y-scale and the mean for the whole mission.
MAX_PLAUSIBLE_SPEED_MS = 5.0

# Column groups that differ between the two layouts. The no-pinger layout calls
# the target `target_*`; the pinger layout calls the same slot
# `corrected_pinger_*` / `pinger_*`, because a pinger target and a path target
# are not the same object.
LAYOUTS = {
    "no_pinger": {
        "target_xy": ("target_x", "target_y"),
        "target_gps": ("target_latitude", "target_longitude"),
        "target_name": "controller target",
    },
    "pinger": {
        "target_xy": ("corrected_pinger_x", "corrected_pinger_y"),
        "target_gps": ("pinger_latitude", "pinger_longitude"),
        "target_name": "USBL pinger",
    },
}

DATE_COLUMNS = ["Year", "Month", "Day", "Hour", "Minute", "Second", "MicroSecond"]


class PoslogError(Exception):
    """The file is not a poslog this module can read."""


# ---------------------------------------------------------------------------
# Reading
# ---------------------------------------------------------------------------

def sidecar_path(csv_path):
    """The `-origin.yaml` written beside a poslog, by the same rule the writer uses.

    Derived by pattern rather than by a fixed-length slice: when a same-second
    collision makes the CSV `...-poslog-2.csv`, chopping len('-poslog.csv')
    characters lands mid-name.
    """
    return re.sub(r"-poslog(-\d+)?\.csv$", r"-origin\1.yaml", str(csv_path))


def read_origin(csv_path):
    """The run's world-frame origin, or None. Three keys, one per line - parsed
    without PyYAML so this module keeps its stdlib-only floor."""
    path = sidecar_path(csv_path)
    if not os.path.isfile(path):
        return None
    origin = {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, _, value = line.partition(":")
            try:
                origin[key.strip()] = float(value.strip())
            except ValueError:
                origin[key.strip()] = value.strip()
    return origin or None


def read_poslog(csv_path):
    """Parse one poslog into float arrays keyed by column name.

    Rows are read BY NAME, never by index - the schema's own rule, and what lets
    one reader serve both layouts.
    """
    with open(csv_path, newline="") as fh:
        reader = csv.DictReader(fh)
        header = reader.fieldnames or []
        rows = list(reader)

    if "relative_x" not in header:
        raise PoslogError(
            f"{csv_path}: not a poslog (no 'relative_x' column). "
            f"Columns present: {', '.join(header) or '<empty file>'}")
    if not rows:
        raise PoslogError(f"{csv_path}: header only, no data rows.")

    layout = "pinger" if "corrected_pinger_x" in header else "no_pinger"
    spec = LAYOUTS[layout]

    data = {}
    for name in header:
        try:
            data[name] = np.array([float(r[name]) for r in rows], dtype=float)
        except (TypeError, ValueError):
            continue  # a non-numeric column is not something this report plots

    for group in (spec["target_xy"], spec["target_gps"]):
        for name in group:
            if name not in data:
                raise PoslogError(
                    f"{csv_path}: layout looks like '{layout}' but column "
                    f"'{name}' is missing.")

    run = {
        "path": str(csv_path),
        "stem": os.path.basename(str(csv_path))[:-len(".csv")],
        "layout": layout,
        "spec": spec,
        "n_rows": len(rows),
        "data": data,
        "origin": read_origin(csv_path),
    }
    run["t"] = _mission_time(data)
    return run


def _mission_time(data):
    """Seconds since the first row, from the seven wall-clock columns.

    There is no monotonic `t` column in any revision of the schema. Days are
    folded in so a run crossing midnight does not jump backwards.
    """
    for name in DATE_COLUMNS:
        if name not in data:
            raise PoslogError(f"missing time column '{name}'")
    secs = (data["Day"] * 86400.0 + data["Hour"] * 3600.0
            + data["Minute"] * 60.0 + data["Second"]
            + data["MicroSecond"] * 1e-6)
    t = secs - secs[0]
    # A month rollover shows up as one large negative step; nothing else can.
    if np.any(np.diff(t) < -1.0):
        t = np.where(t < 0, t + 30 * 86400.0, t)
    return t


# ---------------------------------------------------------------------------
# Derived quantities
# ---------------------------------------------------------------------------

def _valid_gps(lat, lon):
    """A fix of exactly (0, 0) means 'no fix' and is discarded - the same rule
    MAVROS consumers use everywhere else in this project."""
    return ~((lat == 0.0) & (lon == 0.0)) & np.isfinite(lat) & np.isfinite(lon)


def speed_series(run):
    """Absolute ground speed and the mask of implausible samples.

    The schema carries no speed column; `/blueboat/odom`'s body twist is not
    logged. Central differences over a 3 Hz log are noisy, so the result is
    smoothed over ~1 s before it is plotted or averaged.

    Returns (speed, jumps) where `jumps` marks pose discontinuities - see
    MAX_PLAUSIBLE_SPEED_MS.
    """
    t = run["t"]
    x, y = run["data"]["relative_x"], run["data"]["relative_y"]
    if len(t) < 3:
        zeros = np.zeros_like(t)
        return zeros, np.zeros_like(t, dtype=bool)
    with np.errstate(invalid="ignore", divide="ignore"):
        vx = np.gradient(x, t)
        vy = np.gradient(y, t)
    raw = np.hypot(vx, vy)
    raw[~np.isfinite(raw)] = 0.0
    jumps = raw > MAX_PLAUSIBLE_SPEED_MS
    # Smooth the plausible part only, so a spike does not smear into its
    # neighbours and inflate them too.
    clean = np.where(jumps, np.nan, raw)
    if np.all(jumps):
        return raw, jumps
    filled = np.interp(t, t[~jumps], clean[~jumps])
    return _smooth(filled, t, window_s=1.0), jumps


def _smooth(v, t, window_s):
    dt = np.median(np.diff(t)) if len(t) > 1 else 1.0
    n = max(1, int(round(window_s / dt)) | 1)     # odd, >= 1
    if n <= 1:
        return v
    kernel = np.ones(n) / n
    return np.convolve(v, kernel, mode="same")


def distance_series(run):
    """Robot-target separation in metres, and the mask of rows where a target
    actually existed.

    Computed from the WORLD-frame pair, not from lat/lon: both pairs live in the
    same local-ENU frame and subtract directly, with no projection error and no
    dependence on the GPS fix being present.
    """
    d = run["data"]
    tx, ty = run["spec"]["target_xy"]
    have = ~((d[tx] == 0.0) & (d[ty] == 0.0))
    dist = np.hypot(d[tx] - d["relative_x"], d[ty] - d["relative_y"])
    return dist, have


def compute_metrics(run):
    d = run["data"]
    t = run["t"]
    dist, have_target = distance_series(run)
    speed, jumps = speed_series(run)
    ok = ~jumps

    # Path length over plausible steps only: a teleport is not distance covered.
    dx = np.diff(d["relative_x"])
    dy = np.diff(d["relative_y"])
    step_ok = ok[1:] & ok[:-1]
    travelled = float(np.sum(np.hypot(dx, dy)[step_ok]))

    act = d.get("actuation_state")
    act_fraction = {}
    if act is not None and len(act):
        for state in sorted(ACT_LABELS):
            act_fraction[state] = float(np.mean(act == state))

    lat, lon = d["gps_latitude"], d["gps_longitude"]
    fix = _valid_gps(lat, lon)

    live = (act == 1) if act is not None else np.ones_like(t, dtype=bool)
    live_ok = live & ok

    m = {
        "duration_s": float(t[-1] - t[0]) if len(t) > 1 else 0.0,
        "rows": run["n_rows"],
        "travelled_m": travelled,
        "speed_mean": float(np.mean(speed[ok])) if np.any(ok) else float("nan"),
        "speed_max": float(np.max(speed[ok])) if np.any(ok) else float("nan"),
        "speed_mean_live": float(np.mean(speed[live_ok])) if np.any(live_ok) else float("nan"),
        "jumps": int(np.sum(jumps)),
        "dist_mean": float(np.mean(dist[have_target])) if np.any(have_target) else float("nan"),
        "dist_median": float(np.median(dist[have_target])) if np.any(have_target) else float("nan"),
        "dist_max": float(np.max(dist[have_target])) if np.any(have_target) else float("nan"),
        "dist_final": float(dist[have_target][-1]) if np.any(have_target) else float("nan"),
        "thr_right_mean": float(np.mean(d["right_thr_in"])),
        "thr_left_mean": float(np.mean(d["left_thr_in"])),
        "thr_right_abs": float(np.mean(np.abs(d["right_thr_in"]))),
        "thr_left_abs": float(np.mean(np.abs(d["left_thr_in"]))),
        "fix_rows": int(np.sum(fix)),
        "act_fraction": act_fraction,
        "target_rows": int(np.sum(have_target)),
    }
    m["thr_imbalance"] = m["thr_right_mean"] - m["thr_left_mean"]
    return m


def _fmt_duration(seconds):
    if not math.isfinite(seconds):
        return "n/a"
    minutes, secs = divmod(int(round(seconds)), 60)
    return f"{minutes:d}:{secs:02d} ({seconds:.1f} s)"


def _fmt(value, unit="", digits=2):
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "n/a"
    return f"{value:.{digits}f}{unit}"


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def _style(ax, grid="both"):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=8, length=3, color=GRID)
    ax.grid(True, axis=grid if grid != "both" else "both",
            color=GRID, linewidth=0.7)
    ax.set_axisbelow(True)


def render_report(run, out_png):
    """One figure per mission. Raises ImportError when matplotlib is absent -
    the caller decides whether that is fatal (it is not)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    plt.rcParams.update({
        "figure.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "axes.titlelocation": "left",
        "axes.titlesize": 10,
        "axes.titlecolor": INK,
        "axes.labelsize": 9,
        "axes.labelcolor": INK2,
        "font.size": 9,
        "legend.frameon": False,
        "legend.fontsize": 8,
        "figure.dpi": 130,
    })

    d, t = run["data"], run["t"]
    spec = run["spec"]
    m = compute_metrics(run)
    dist, have_target = distance_series(run)
    speed, jumps = speed_series(run)

    fig = plt.figure(figsize=(16.0, 12.5))
    gs = GridSpec(5, 4, figure=fig,
                  height_ratios=[2.3, 2.3, 1.5, 0.30, 1.55],
                  hspace=0.55, wspace=0.32,
                  left=0.055, right=0.975, top=0.925, bottom=0.045)

    ax_track = fig.add_subplot(gs[0:2, 0:2])
    ax_dist = fig.add_subplot(gs[0, 2:4])
    ax_speed = fig.add_subplot(gs[1, 2:4])
    ax_thr = fig.add_subplot(gs[2, 0:4])
    ax_state = fig.add_subplot(gs[3, 0:4], sharex=ax_thr)
    ax_table = fig.add_subplot(gs[4, 0:4])

    _plot_track(ax_track, run, spec)
    _plot_distance(ax_dist, t, dist, have_target, spec, m)
    _plot_speed(ax_speed, t, speed, jumps, m)
    _plot_thrusters(ax_thr, t, d)
    _plot_state_strip(ax_state, t, d.get("actuation_state"))
    _plot_table(ax_table, run, m)

    title = run["stem"]
    subtitle = (f"{run['n_rows']} rows · {_fmt_duration(m['duration_s'])} · "
                f"{run['layout'].replace('_', '-')} layout · "
                f"target: {spec['target_name']}")
    fig.suptitle(title, x=0.055, y=0.975, ha="left", fontsize=15,
                 color=INK, fontweight="bold")
    fig.text(0.055, 0.949, subtitle, ha="left", fontsize=9.5, color=MUTED)

    fig.savefig(out_png, bbox_inches="tight", pad_inches=0.28)
    plt.close(fig)
    return out_png


def _plot_track(ax, run, spec):
    d = run["data"]
    lat, lon = d["gps_latitude"], d["gps_longitude"]
    tlat_name, tlon_name = spec["target_gps"]
    tlat, tlon = d[tlat_name], d[tlon_name]

    fix = _valid_gps(lat, lon)
    tfix = _valid_gps(tlat, tlon)

    _style(ax)
    ax.set_title("Track (WGS84) — robot and target, no-fix rows removed")
    ax.set_xlabel("longitude (°E)")
    ax.set_ylabel("latitude (°N)")

    if not np.any(fix):
        ax.text(0.5, 0.5, "no GPS fix in this run", transform=ax.transAxes,
                ha="center", va="center", color=MUTED, fontsize=11)
        ax.set_xticks([]); ax.set_yticks([])
        return

    if np.any(tfix):
        ax.plot(tlon[tfix], tlat[tfix], "-", color=TARGET, linewidth=2.0,
                label="target", zorder=2)
    ax.plot(lon[fix], lat[fix], "-", color=ROBOT, linewidth=2.0,
            label="robot", zorder=3)

    # Start / end, ringed in the surface colour so they stay legible over the line.
    ax.plot(lon[fix][0], lat[fix][0], "o", markersize=9, color=ROBOT,
            markeredgecolor=SURFACE, markeredgewidth=2.0, zorder=4)
    ax.plot(lon[fix][-1], lat[fix][-1], "s", markersize=9, color=ROBOT,
            markeredgecolor=SURFACE, markeredgewidth=2.0, zorder=4)
    ax.annotate("start", (lon[fix][0], lat[fix][0]), textcoords="offset points",
                xytext=(9, 5), color=INK2, fontsize=8)
    ax.annotate("end", (lon[fix][-1], lat[fix][-1]), textcoords="offset points",
                xytext=(9, 5), color=INK2, fontsize=8)

    # True metric aspect: one metre east must be one metre north on the page, so
    # the shape of the track is the shape it had on the water.
    lat0 = float(np.mean(lat[fix]))
    ax.set_aspect(1.0 / max(math.cos(math.radians(lat0)), 1e-6))

    _scale_bar(ax, lat0)
    _north_arrow(ax)
    ax.legend(loc="best")
    ax.ticklabel_format(useOffset=False, style="plain")
    ax.tick_params(axis="x", labelrotation=20)


def _scale_bar(ax, lat0):
    """A labelled bar in metres, because the axes are in degrees."""
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    span_m = abs(x1 - x0) * math.radians(1.0) * EARTH_R * math.cos(math.radians(lat0))
    if span_m <= 0 or not math.isfinite(span_m):
        return
    nice = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000]
    target = span_m * 0.25
    length_m = min(nice, key=lambda v: abs(v - target))
    length_deg = length_m / (math.radians(1.0) * EARTH_R * math.cos(math.radians(lat0)))

    bx = x0 + 0.06 * (x1 - x0)
    by = y0 + 0.07 * (y1 - y0)
    ax.plot([bx, bx + length_deg], [by, by], "-", color=INK2, linewidth=2.5,
            solid_capstyle="butt", zorder=5)
    ax.text(bx + length_deg / 2, by + 0.018 * (y1 - y0), f"{length_m} m",
            ha="center", va="bottom", color=INK2, fontsize=8)


def _north_arrow(ax):
    ax.annotate("N", xy=(0.965, 0.93), xytext=(0.965, 0.80),
                xycoords="axes fraction", textcoords="axes fraction",
                ha="center", va="bottom", color=INK2, fontsize=9,
                arrowprops=dict(arrowstyle="-|>", color=INK2, linewidth=1.4))


def _plot_distance(ax, t, dist, have, spec, m):
    _style(ax)
    ax.set_title("Distance robot → target")
    ax.set_xlabel("mission time (s)")
    ax.set_ylabel("distance (m)")
    if not np.any(have):
        ax.text(0.5, 0.5, "no target in this run", transform=ax.transAxes,
                ha="center", va="center", color=MUTED)
        return
    series = np.where(have, dist, np.nan)
    ax.plot(t, series, "-", color=TARGET, linewidth=2.0)
    # Every time axis spans the whole mission, so the four panels read against
    # one another rather than each against its own window.
    ax.set_xlim(t[0], t[-1])
    if math.isfinite(m["dist_mean"]):
        ax.axhline(m["dist_mean"], color=REF, linewidth=1.2, linestyle="--")
        ax.annotate(f"mean {m['dist_mean']:.2f} m",
                    xy=(t[-1], m["dist_mean"]), textcoords="offset points",
                    xytext=(-4, 5), ha="right", color=MUTED, fontsize=8)
    ax.set_ylim(bottom=0)


def _plot_speed(ax, t, speed, jumps, m):
    _style(ax)
    ax.set_title("Ground speed (|v|, 1 s smoothed, differenced from pose)")
    ax.set_xlabel("mission time (s)")
    ax.set_ylabel("speed (m/s)")
    ax.plot(t, np.where(jumps, np.nan, speed), "-", color=ROBOT, linewidth=2.0)
    if np.any(jumps):
        # Named, not hidden: the reader must see that samples were removed. A rug
        # along the baseline rather than full-height rules - on a run with a
        # hundred jumps, rules bury the signal they are annotating.
        ax.plot(t[jumps], np.zeros(int(np.sum(jumps))), "|", color=BAD,
                markersize=9, markeredgewidth=1.2, zorder=4,
                label=f"pose jump (>{MAX_PLAUSIBLE_SPEED_MS:.0f} m/s), excluded")
        ax.legend(loc="upper left")
    ax.set_xlim(t[0], t[-1])
    if math.isfinite(m["speed_mean"]):
        ax.axhline(m["speed_mean"], color=REF, linewidth=1.2, linestyle="--")
        ax.annotate(f"mean {m['speed_mean']:.2f} m/s",
                    xy=(t[-1], m["speed_mean"]), textcoords="offset points",
                    xytext=(-4, 5), ha="right", color=MUTED, fontsize=8)
    ax.set_ylim(bottom=0)


def _plot_thrusters(ax, t, d):
    _style(ax)
    ax.set_title("Commanded thrust (as published on /thruster_input)")
    ax.set_ylabel("thrust (N)")
    ax.plot(t, d["right_thr_in"], "-", color=ROBOT, linewidth=1.8, label="right")
    ax.plot(t, d["left_thr_in"], "-", color=TARGET, linewidth=1.8, label="left")
    ax.axhline(0.0, color=GRID, linewidth=1.0)
    ax.set_xlim(t[0], t[-1])
    ax.margins(y=0.12)
    ax.legend(loc="lower right", ncol=2)
    ax.tick_params(labelbottom=False)


def _plot_state_strip(ax, t, act):
    """Whether the thrust above could actually reach the water.

    Colour alone never carries this: every band present is named in the strip's
    own legend, and the summary table gives the numeric fraction per state.
    """
    ax.set_facecolor(SURFACE)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.set_yticks([])
    ax.set_xlabel("mission time (s)")
    ax.tick_params(colors=INK2, labelsize=8, length=3, color=GRID)

    if act is None or not len(act):
        ax.text(0.5, 0.5, "no actuation_state column (pre-2026-08-31 CSV)",
                transform=ax.transAxes, ha="center", va="center",
                color=MUTED, fontsize=8)
        return

    ax.set_ylabel("actuation", rotation=0, ha="right", va="center",
                  fontsize=8, color=INK2, labelpad=8)
    seen = []
    start = 0
    for i in range(1, len(act) + 1):
        if i == len(act) or act[i] != act[start]:
            state = int(act[start])
            label, colour = ACT_LABELS.get(state, (f"state {state}", MUTED))
            ax.axvspan(t[start], t[min(i, len(t) - 1)], color=colour, alpha=0.85,
                       linewidth=0)
            if state not in seen:
                seen.append(state)
            start = i
    ax.set_ylim(0, 1)
    ax.set_xlim(t[0], t[-1])

    from matplotlib.patches import Patch
    handles = [Patch(facecolor=ACT_LABELS[s][1], label=f"{s} · {ACT_LABELS[s][0]}")
               for s in seen if s in ACT_LABELS]
    if handles:
        # Above the strip and right-aligned: the thrust panel's own lower
        # margin is empty there, and below the strip is where the x label lives.
        ax.legend(handles=handles, loc="lower right", ncol=len(handles),
                  bbox_to_anchor=(1.0, 1.45), fontsize=8)


def _plot_table(ax, run, m):
    ax.axis("off")
    act = m["act_fraction"]
    live_pct = 100.0 * act.get(1, 0.0)
    origin = run["origin"] or {}

    rows = [
        ("Mission time", _fmt_duration(m["duration_s"])),
        ("Rows logged", f"{m['rows']} (~3 Hz)"),
        ("Distance travelled", _fmt(m["travelled_m"], " m")),
        ("Mean speed |v|", _fmt(m["speed_mean"], " m/s")),
        ("Max speed |v|", _fmt(m["speed_max"], " m/s")),
        ("Pose jumps excluded",
         f"{m['jumps']} / {m['rows']} rows" if m["jumps"] else "none"),

        ("Mean distance to target", _fmt(m["dist_mean"], " m")),
        ("Median distance to target", _fmt(m["dist_median"], " m")),
        ("Max distance to target", _fmt(m["dist_max"], " m")),
        ("Final distance to target", _fmt(m["dist_final"], " m")),
        ("Rows with a target", f"{m['target_rows']} / {m['rows']}"),
        ("Rows with a GPS fix", f"{m['fix_rows']} / {m['rows']}"),

        ("Mean thrust right", _fmt(m["thr_right_mean"], " N")),
        ("Mean thrust left", _fmt(m["thr_left_mean"], " N")),
        ("Mean |thrust| right / left",
         f"{_fmt(m['thr_right_abs'])} / {_fmt(m['thr_left_abs'])} N"),
        ("Right − left imbalance", _fmt(m["thr_imbalance"], " N")),
        ("Thrust live (state 1)", f"{live_pct:.1f} % of the run"),
        ("Mean speed while live", _fmt(m["speed_mean_live"], " m/s")),
        ("World origin (lat, lon)",
         (f"{origin.get('latitude', float('nan')):.7f}, "
          f"{origin.get('longitude', float('nan')):.7f}")
         if origin else "no -origin.yaml sidecar"),
    ]

    # Three column-pairs, filled down then across, so related metrics stay together.
    per_col = 6
    cell_text = []
    for r in range(per_col):
        line = []
        for c in range(3):
            idx = c * per_col + r
            line.extend(rows[idx] if idx < len(rows) else ("", ""))
        cell_text.append(line)

    table = ax.table(cellText=cell_text, cellLoc="left", loc="upper center",
                     colWidths=[0.135, 0.1975] * 3)
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.55)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor(GRID)
        cell.set_linewidth(0.6)
        cell.set_facecolor(SURFACE)
        text = cell.get_text()
        if col % 2 == 0:
            text.set_color(MUTED)
        else:
            text.set_color(INK)
            text.set_fontweight("bold")
        if not text.get_text():
            cell.set_edgecolor(SURFACE)

    ax.set_title("Summary", loc="left", color=INK, fontsize=10, pad=10)


# ---------------------------------------------------------------------------
# Archiving
# ---------------------------------------------------------------------------

def finalise_run(csv_path, archive=True):
    """Render the report and file the run away. Returns the folder (or the PNG
    path when archiving is off).

    Ordering matters and is deliberate:
      1. create the folder            stdlib, cannot fail for a plotting reason
      2. render the PNG into it       optional; a failure here is reported, not raised
      3. move the CSV and its sidecar so the record is filed even with no picture

    The caller may be a `finally` block on a node that is being killed, so
    nothing here may raise for a cosmetic reason.
    """
    csv_path = os.path.abspath(os.path.expanduser(str(csv_path)))
    if not os.path.isfile(csv_path):
        raise PoslogError(f"{csv_path}: no such file")

    stem = os.path.basename(csv_path)[:-len(".csv")]
    parent = os.path.dirname(csv_path)

    if not archive:
        out_png = os.path.join(parent, stem + ".png")
        _render_guarded(csv_path, out_png)
        return out_png

    folder = os.path.join(parent, stem)
    if os.path.isdir(folder):
        # Write-once: a filed run is never re-filed or overwritten.
        return folder
    os.makedirs(folder, exist_ok=False)

    out_png = os.path.join(folder, stem + ".png")
    _render_guarded(csv_path, out_png)

    for source in (csv_path, sidecar_path(csv_path)):
        if not os.path.isfile(source):
            continue
        destination = os.path.join(folder, os.path.basename(source))
        if os.path.exists(destination):
            continue
        shutil.move(source, destination)

    return folder


def _render_guarded(csv_path, out_png):
    try:
        run = read_poslog(csv_path)
        render_report(run, out_png)
    except ImportError as exc:
        print(f"poslog_report: matplotlib unavailable ({exc}); "
              f"no picture rendered for {os.path.basename(csv_path)}",
              file=sys.stderr)
    except Exception as exc:                            # noqa: BLE001 - teardown
        print(f"poslog_report: could not render {csv_path}: {exc}",
              file=sys.stderr)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Render the post-mission report for a poslog CSV and file "
                    "the run into its own folder.")
    parser.add_argument("target", help="a *-poslog.csv file, or a directory")
    parser.add_argument("--all", action="store_true",
                        help="with a directory: process every *-poslog.csv in it")
    parser.add_argument("--no-archive", action="store_true",
                        help="render the picture beside the CSV; move nothing")
    args = parser.parse_args(argv)

    target = os.path.abspath(os.path.expanduser(args.target))
    if os.path.isdir(target):
        if not args.all:
            parser.error(f"{target} is a directory - pass --all to process it")
        paths = sorted(
            os.path.join(target, name) for name in os.listdir(target)
            if name.endswith(".csv") and "-poslog" in name)
        if not paths:
            print(f"No *-poslog.csv found in {target}")
            return 1
    else:
        paths = [target]

    failures = 0
    for path in paths:
        try:
            result = finalise_run(path, archive=not args.no_archive)
            print(f"{os.path.basename(path)} -> {result}")
        except (PoslogError, OSError) as exc:
            failures += 1
            print(f"{os.path.basename(path)}: {exc}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
