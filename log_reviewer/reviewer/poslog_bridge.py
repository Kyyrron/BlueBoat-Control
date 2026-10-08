#!/usr/bin/env python3

r"""
The reviewer's data layer: `poslog_report.py` REUSED, plus cropping.

WHY REUSE RATHER THAN COPY. `blueboat_control/src/_custom_libraries/poslog_report.py`
already knows how to read either CSV layout, how to derive mission time from the
seven wall-clock columns, how ground speed is differenced and smoothed, which
samples are pose jumps, and every number in the summary. If this app re-derived
any of that, the app and the archived per-run PNG would quietly start reporting
different figures for the same run. So the maths is imported, never forked; only
the PAINTING is the app's own (see figures.py), because the app needs editable
text, satellite tiles, interactivity and no actuation strip.

That module is safe to import here:
  * it is ROS-free - numpy and stdlib only at module scope;
  * `matplotlib.use("Agg")` sits INSIDE `render_report`, which this app never
    calls, so it can never hijack the Qt backend;
  * `finalise_run` - the one function that MOVES field data - is likewise never
    called from the app.

What is new here is the crop: the timeline hands a (t0, t1) window and every
figure and every number is recomputed on the rows inside it.
"""

import os
import sys

import numpy as np

# The importable half of the control stack. Two levels up from this file is
# log_reviewer/, three is the repository root.
_HERE = os.path.dirname(os.path.abspath(__file__))
_LIBS = os.path.normpath(os.path.join(
    _HERE, "..", "..", "blueboat_control", "src", "_custom_libraries"))
if _LIBS not in sys.path:
    sys.path.insert(0, _LIBS)

import poslog_report as pr                                        # noqa: E402
import robot_log_schema as rls                                    # noqa: E402

# --- re-exported unchanged, so the app cannot drift from the PNG -----------
PoslogError = pr.PoslogError
read_origin = pr.read_origin
sidecar_path = pr.sidecar_path
speed_series = pr.speed_series
distance_series = pr.distance_series
compute_metrics = pr.compute_metrics
valid_gps = pr._valid_gps
# The frame decision lives with the maths, not with either painter, so the
# archived PNG and this app can never draw the same run in different frames.
is_simulation = pr.is_simulation
track_series = pr.track_series
origin_label = pr.origin_label
fmt = pr._fmt
fmt_duration = pr._fmt_duration

SURFACE, INK, INK2, MUTED, GRID = pr.SURFACE, pr.INK, pr.INK2, pr.MUTED, pr.GRID
ROBOT, TARGET, REF = pr.ROBOT, pr.TARGET, pr.REF
GOOD, WARN, BAD = pr.GOOD, pr.WARN, pr.BAD
EARTH_R = pr.EARTH_R
MAX_PLAUSIBLE_SPEED_MS = pr.MAX_PLAUSIBLE_SPEED_MS
LAYOUTS = pr.LAYOUTS
ACT_LABELS = pr.ACT_LABELS

SOURCE_MODULE = pr.__file__


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def load(csv_path):
    """Read one poslog and tag it with what the cropper needs.

    Returns `poslog_report`'s own `run` dict with three keys added:
      `row_index`   source row numbers still present (identity at load time),
      `n_rows_full` the uncropped row count,
      `t_full`      the uncropped mission-time span (t[0], t[-1]).
    """
    run = pr.read_poslog(csv_path)
    if run["origin"] is None:
        # `poslog_report` derives the sidecar name from the `-poslog.csv`
        # suffix. An EXPORTED crop may have been renamed to anything, and its
        # sidecar is then `<stem>-origin.yaml`, so try that too - otherwise a
        # re-opened export silently loses its world origin.
        run["origin"] = pr.read_origin(str(csv_path)[:-4] + "-poslog.csv")
    # Pin the frame from the WHOLE run, so a crop inherits it (crop_run copies
    # the dict) and trimming into a no-fix stretch cannot flip the track panel
    # from degrees to metres halfway through a session.
    run["frame_mode"] = pr.track_series(run)["mode"]
    run["row_index"] = np.arange(run["n_rows"], dtype=int)
    run["n_rows_full"] = run["n_rows"]
    t = run["t"]
    run["t_full"] = (float(t[0]), float(t[-1])) if len(t) else (0.0, 0.0)
    return run


def crop_run(run, t0, t1):
    """A new `run` holding only the rows with `t0 <= t <= t1`.

    Mission time is NOT re-based. The x axis keeps the seconds it had in the
    whole run, so a cropped panel still says where in the mission you are.

    Every derived series - speed, distance, the metrics, the summary - is then
    recomputed from this dict rather than sliced out of the full-run result.
    That is deliberate: the crop IS the dataset the reader is looking at, and
    a mean over a window has to be the mean of that window.

    An empty selection cannot be rendered, so a window that catches no row
    falls back to the single nearest row rather than raising.
    """
    t = run["t"]
    mask = (t >= t0) & (t <= t1)
    if not np.any(mask):
        nearest = int(np.argmin(np.abs(t - 0.5 * (t0 + t1))))
        mask = np.zeros(len(t), dtype=bool)
        mask[nearest] = True

    out = dict(run)
    out["data"] = {name: values[mask] for name, values in run["data"].items()}
    out["t"] = t[mask]
    out["n_rows"] = int(np.count_nonzero(mask))
    out["row_index"] = run["row_index"][mask]
    out["window"] = (float(t0), float(t1))
    return out


def wall_clock(run, index):
    """The wall-clock stamp of one row of a run, as 'YYYY-MM-DD HH:MM:SS.mmm'.

    Straight off the seven date columns - there is no epoch column in any
    revision of the schema.
    """
    d = run["data"]
    try:
        return ("%04d-%02d-%02d %02d:%02d:%02d.%03d" % (
            d["Year"][index], d["Month"][index], d["Day"][index],
            d["Hour"][index], d["Minute"][index], d["Second"][index],
            d["MicroSecond"][index] / 1000.0))
    except (KeyError, IndexError, ValueError):
        return "n/a"


# ---------------------------------------------------------------------------
# Summary content - ONE source of truth for the Qt table and the exported PNG
# ---------------------------------------------------------------------------

def summary_rows(run, metrics):
    """The (label, value) pairs of the summary, lifted from
    `poslog_report._plot_table` so the screen and the picture cannot disagree.

    `actuation_state` is still read here even though the app drops the
    actuation strip: the column is what separates a run with the motor gate
    open from one with it shut, and losing the plot must not lose the number.
    """
    m = metrics
    act = m["act_fraction"]
    live_pct = 100.0 * act.get(1, 0.0)

    return [
        ("Mission time", fmt_duration(m["duration_s"])),
        ("Rows logged", "%d (~3 Hz)" % m["rows"]),
        ("Distance travelled", fmt(m["travelled_m"], " m")),
        ("Mean speed |v|", fmt(m["speed_mean"], " m/s")),
        ("Max speed |v|", fmt(m["speed_max"], " m/s")),
        ("Pose jumps excluded",
         ("%d / %d rows" % (m["jumps"], m["rows"])) if m["jumps"] else "none"),
        ("Mean speed while live", fmt(m["speed_mean_live"], " m/s")),

        ("Mean distance to target", fmt(m["dist_mean"], " m")),
        ("Median distance to target", fmt(m["dist_median"], " m")),
        ("Max distance to target", fmt(m["dist_max"], " m")),
        ("Final distance to target", fmt(m["dist_final"], " m")),
        ("Rows with a target", "%d / %d" % (m["target_rows"], m["rows"])),
        ("Rows with a GPS fix", "%d / %d" % (m["fix_rows"], m["rows"])),
        ("Target", run["spec"]["target_name"]),

        ("Mean thrust right", fmt(m["thr_right_mean"], " N")),
        ("Mean thrust left", fmt(m["thr_left_mean"], " N")),
        ("Mean |thrust| right / left",
         "%s / %s N" % (fmt(m["thr_right_abs"]), fmt(m["thr_left_abs"]))),
        ("Right - left imbalance", fmt(m["thr_imbalance"], " N")),
        ("Thrust live (state 1)", "%.1f %% of the run" % live_pct),
        ("Layout", run["layout"].replace("_", "-")),
        ("World origin (lat, lon)", origin_label(run)),
    ]


def subtitle_for(run, metrics):
    """The line under the report title, recomputed for whatever is selected."""
    of_full = ""
    if run["n_rows"] != run.get("n_rows_full", run["n_rows"]):
        of_full = " of %d" % run["n_rows_full"]
    return ("%d%s rows · %s · %s layout · target: %s%s" % (
        run["n_rows"], of_full, fmt_duration(metrics["duration_s"]),
        run["layout"].replace("_", "-"), run["spec"]["target_name"],
        " · simulation" if is_simulation(run) else ""))
