#!/usr/bin/env python3

r"""
The pinger of a run, recovered from its ros2 bag.

WHY THIS EXISTS. A run flown in path mode logs the no-pinger CSV layout, which
has no pinger columns. But `uwgps_log` runs in every launch and, in a run
session, `robot_interface` keeps tracking the pinger whatever the follow mode
is, so the run's bag (`<date>-<note>-bag/`, filed beside the poslog) holds
`/uw_gps_data` (the raw Water Linked packet) and `/blueboat/pinger_coordinates`
(the filtered fix, body frame, dead-reckoned at odom rate). This module reads
those two topics, puts the pinger on the same world frame as the track, and can
hand the export the columns of the with-pinger layout.

NO ROS, NO NEW DEPENDENCY. The reviewer must run under a bare interpreter, and
neither `mcap` nor `rosbag2_py` is installed. A run bag is an MCAP file with
UNCOMPRESSED chunks (`compression_format: ""` in its metadata.yaml), and
Float32MultiArray is a few lines of CDR, so both are decoded here directly.
Compressed chunks are read only if `zstandard` / `lz4` happen to be present;
otherwise the reader says so instead of guessing.

TIMESTAMPS - the part that goes wrong silently.
  * The poslog's time is seven NAIVE wall-clock columns, written in the boat
    computer's local zone. The bag's time is epoch nanoseconds. They are tied
    together through the first poslog row: its wall clock, read in THIS
    machine's zone, must land within `MAX_START_GAP_S` of the bag's first
    message (the bag is pre-armed and resumed at the run start, so the two
    agree to well under a second). If the zone differs, the gap is a whole
    multiple of 30 minutes and is removed; any other gap means the bag is not
    this run's, and nothing is drawn.
  * Only the bag's RECEIVE time (MCAP `log_time`) is used. The date fields
    inside /uw_gps_data are the Water Linked unit's own clock (9 s off in the
    2026-10-28 run), and mavros header stamps read 0 on the real launch.
  * Pinger samples are interpolated against the poslog's pose at the sample's
    own time, never matched to the nearest row (rows are 0.33 s apart).
"""

import os
import struct
import time

import numpy as np

from . import poslog_bridge as pb

import frame_math                                                   # noqa: E402
import mcap_lite                                                    # noqa: E402

BODY_TOPIC = "/blueboat/pinger_coordinates"
UGPS_TOPIC = "/uw_gps_data"
UGPS_COLUMNS = pb.rls.UGPS_COLUMNS     # /uw_gps_data[7:19], as robot_interface logs them

MAX_START_GAP_S = 120.0     # bag start vs first poslog row, after zone removal
ZONE_STEP_S = 1800.0        # time zones are whole multiples of 30 minutes
HOLD_S = 8.0                # a shown estimate is held this long past its sample

# The filter. Each number answers one way the estimate goes wrong:
#   max_yaw_rate  the pose between two 3 Hz rows is interpolated, and a yaw
#                 error is multiplied by the range: 10 degrees at 5 m is 0.9 m.
#                 A sample taken while the boat turns faster is not trusted.
#   max_speed     the pinger does not move; an estimate that jumps faster than
#                 this from the last accepted one is a bad fix or a bad
#                 dead-reckoning step. After `reanchor_s` without an accepted
#                 sample the next one is taken as the new anchor, so one early
#                 outlier cannot freeze the track for the rest of the run.
#   max_range     a vector longer than the pool is a bad fix.
#   consensus_s / consensus_m  a sample further than `consensus_m` from the
#                 median of every trusted sample within +/- `consensus_s` is an
#                 outlier. This is what removes the plateaus of a bad fix that
#                 dead reckoning then carries along unchanged - the speed gate
#                 cannot see those, because they do not move.
#   smooth_s      median over +/- this many seconds of accepted samples.
#   max_pose_gap_s a sample is dropped if the pose rows around it are further
#                 apart than this (a logging stall), because it would be
#                 interpolated across the stall.
FILTER = {"max_yaw_rate": 0.35, "max_speed": 1.0, "reanchor_s": 3.0,
          "smooth_s": 4.0, "max_pose_gap_s": 1.5, "max_range": 30.0,
          "consensus_s": 10.0, "consensus_m": 3.0}


# The MCAP reader and the CDR decoders live in `_custom_libraries/mcap_lite.py`
# (moved there 2026-11-24, so the pinger_filter_replay CLI and poslog_report read
# bags with the same code). These names are kept for the reviewer's callers.
PingerBagError = mcap_lite.McapError
_mcap_files = mcap_lite.mcap_files
read_topics = mcap_lite.read_topics
decode_float32_multiarray = mcap_lite.decode_float32_multiarray


# ---------------------------------------------------------------------------
# Locating and reading the bag
# ---------------------------------------------------------------------------

def find_bag(csv_path):
    """The run's bag folder beside its poslog, or None.

    Named by the writer's own rule (`poslog_report.bag_name`), so a log
    that was renamed or exported - which has no bag - finds none.
    """
    folder = os.path.dirname(os.path.abspath(str(csv_path)))
    path = os.path.join(folder, pb.pr.bag_name(csv_path))
    return path if os.path.isdir(path) else None


def read_bag(bag):
    """The pinger messages of a bag.

    Returns {"bag", "start_s", "end_s", "body_t", "body", "ugps_t", "ugps"}:
    times are EPOCH seconds of the bag's receive clock, `body` is (n, 3) and
    `ugps` is (n, 19) - the raw packet exactly as `robot_interface` logs it.
    """
    topics, (first, last) = mcap_lite.read_bag(bag, (BODY_TOPIC, UGPS_TOPIC))

    def unpack(rows, width):                    # rows already sorted by time
        times = np.array([r[0] for r in rows], dtype=float) * 1e-9
        values = np.zeros((len(rows), width))
        for i, (_t, payload) in enumerate(rows):
            v = decode_float32_multiarray(payload)
            values[i, :min(width, len(v))] = v[:width]
        return times, values

    body_t, body = unpack(topics[BODY_TOPIC], 3)
    ugps_t, ugps = unpack(topics[UGPS_TOPIC], 19)
    return {"bag": bag,
            "start_s": None if first is None else first * 1e-9,
            "end_s": None if last is None else last * 1e-9,
            "body_t": body_t, "body": body, "ugps_t": ugps_t, "ugps": ugps}


# ---------------------------------------------------------------------------
# Time
# ---------------------------------------------------------------------------

def first_row_epoch(run):
    """Epoch seconds of the poslog's first row, its naive wall clock read in
    this machine's zone."""
    d = run["data"]
    fields = [int(d[name][0]) for name in
              ("Year", "Month", "Day", "Hour", "Minute", "Second")]
    micro = float(d["MicroSecond"][0]) * 1e-6
    return time.mktime(tuple(fields) + (0, 0, -1)) + micro


def time_shift(run, bag_start_s):
    """Seconds to subtract from a bag epoch time, besides the first row's own
    epoch, to get this run's mission time; PingerBagError if the bag is not
    this run's."""
    gap = bag_start_s - first_row_epoch(run)
    zone = round(gap / ZONE_STEP_S) * ZONE_STEP_S
    if abs(gap - zone) > MAX_START_GAP_S:
        raise PingerBagError(
            "the bag starts %.0f s from the log's first row - not this run's bag"
            % gap)
    return zone


def to_mission_time(run, bag_times, shift):
    return np.asarray(bag_times) - first_row_epoch(run) - shift


# ---------------------------------------------------------------------------
# The estimate
# ---------------------------------------------------------------------------

def _pose(run):
    t = np.asarray(run["t"], dtype=float)
    d = run["data"]
    return t, d["relative_x"], d["relative_y"], np.unwrap(d["relative_psi"])


def world_samples(run, t, body, params):
    """Body-frame samples -> world frame, with the per-sample gates applied.

    The rotation is `custom_functions.transform_body_to_world`, the formula
    `robot_interface` uses for `corrected_pinger`, so this reproduces what a
    pinger-mode log of the same run would have held. Returns arrays
    (t, x, y, yaw_rate, trusted): `trusted` is False where the pose could not
    be interpolated cleanly.
    """
    pt, px, py, ppsi = _pose(run)
    inside = (t >= pt[0]) & (t <= pt[-1])
    k = np.clip(np.searchsorted(pt, t), 1, len(pt) - 1)
    gap = pt[k] - pt[k - 1]
    x = np.interp(t, pt, px)
    y = np.interp(t, pt, py)
    yaw = np.interp(t, pt, ppsi)
    # Yaw rate over a 1 s span around the sample: the rows are only 3 Hz.
    rate = np.abs(np.interp(t + 0.5, pt, ppsi) - np.interp(t - 0.5, pt, ppsi))
    c, s = np.cos(yaw), np.sin(yaw)
    wx = x + c * body[:, 0] - s * body[:, 1]
    wy = y + s * body[:, 0] + c * body[:, 1]
    trusted = inside & (gap <= params["max_pose_gap_s"])
    return wx, wy, rate, trusted


def _consensus(t, wx, wy, usable, params):
    """True where a usable sample sits within `consensus_m` of the median of the
    usable samples around it."""
    ok = np.zeros(len(t), dtype=bool)
    ut, ux, uy = t[usable], wx[usable], wy[usable]
    for i in np.nonzero(usable)[0]:
        lo = np.searchsorted(ut, t[i] - params["consensus_s"], side="left")
        hi = np.searchsorted(ut, t[i] + params["consensus_s"], side="right")
        ok[i] = np.hypot(wx[i] - np.median(ux[lo:hi]),
                         wy[i] - np.median(uy[lo:hi])) <= params["consensus_m"]
    return ok


def _filter(t, wx, wy, rate, trusted, params, body=None):
    keep = np.zeros(len(t), dtype=bool)
    last = None
    usable = trusted & (rate <= params["max_yaw_rate"])
    if body is not None:
        usable &= np.hypot(body[:, 0], body[:, 1]) <= params["max_range"]
    usable &= _consensus(t, wx, wy, usable, params)
    for i in range(len(t)):
        if not usable[i]:
            continue
        if last is None:
            keep[i], last = True, i
            continue
        dt = t[i] - t[last]
        speed = np.hypot(wx[i] - wx[last], wy[i] - wy[last]) / max(dt, 1e-3)
        if speed <= params["max_speed"] or dt > params["reanchor_s"]:
            keep[i], last = True, i
    kt, kx, ky = t[keep], wx[keep], wy[keep]
    sx, sy = np.empty_like(kx), np.empty_like(ky)
    for i, when in enumerate(kt):
        lo = np.searchsorted(kt, when - params["smooth_s"], side="left")
        hi = np.searchsorted(kt, when + params["smooth_s"], side="right")
        sx[i], sy[i] = np.median(kx[lo:hi]), np.median(ky[lo:hi])
    return keep, kt, sx, sy


def estimate(run, data, shift, params=None):
    """The pinger's world position over the run.

    `run` must be the WHOLE run (mission time from its first row), `data` the
    result of `read_bag`. Returns None when the pinger was never detected.
    The all-zero vector is the writer's "not detected yet", not a position.
    """
    params = dict(FILTER, **(params or {}))
    detected = np.any(data["body"] != 0.0, axis=1)
    if not np.any(detected):
        return None
    t = to_mission_time(run, data["body_t"][detected], shift)
    body = data["body"][detected]
    wx, wy, rate, trusted = world_samples(run, t, body, params)
    keep, kt, sx, sy = _filter(t, wx, wy, rate, trusted, params, body)
    if not len(kt):
        return None
    ut = to_mission_time(run, data["ugps_t"], shift)
    return {"t": kt, "x": sx, "y": sy,
            "raw_t": t[trusted], "raw_x": wx[trusted], "raw_y": wy[trusted],
            "n_detected": int(detected.sum()), "n_trusted": int(trusted.sum()),
            "n_accepted": int(keep.sum()),
            "ugps_t": ut, "ugps": data["ugps"],
            "first_t": float(t[0]), "last_t": float(t[-1]),
            "params": params, "bag": data["bag"], "shift_s": float(shift)}


def estimate_from_log(run, params=None):
    """The filtered pinger of a run that logged the pinger itself (with-pinger
    layout), from its own `corrected_pinger_x/y`.

    Those columns are the raw dead-reckoned estimate, already in the world
    frame and stamped with the pose of the same row, so no bag and no pose
    interpolation are involved: only the filter runs (the range gate on the
    boat-to-pinger distance). Returns None when the pinger was never detected.
    """
    params = dict(FILTER, **(params or {}))
    d = run["data"]
    wx, wy = d["corrected_pinger_x"], d["corrected_pinger_y"]
    found = (np.isfinite(wx) & np.isfinite(wy)) & ((wx != 0.0) | (wy != 0.0))
    if not np.any(found):
        return None
    t = np.asarray(run["t"], dtype=float)[found]
    wx, wy = wx[found], wy[found]
    pt, px, py, ppsi = _pose(run)
    rate = np.abs(np.interp(t + 0.5, pt, ppsi) - np.interp(t - 0.5, pt, ppsi))
    around = np.column_stack([wx - px[found], wy - py[found]])
    trusted = np.ones(len(t), dtype=bool)
    keep, kt, sx, sy = _filter(t, wx, wy, rate, trusted, params, around)
    if not len(kt):
        return None
    return {"t": kt, "x": sx, "y": sy, "raw_t": t, "raw_x": wx, "raw_y": wy,
            "n_detected": int(found.sum()), "n_trusted": int(found.sum()),
            "n_accepted": int(keep.sum()),
            "ugps_t": np.zeros(0), "ugps": np.zeros((0, 19)),
            "first_t": float(t[0]), "last_t": float(t[-1]),
            "params": params, "bag": None, "shift_s": 0.0}


def from_filter_columns(run):
    """The pinger_estimator's own filtered estimate, as the log recorded it
    (`pf_*` columns, written since 2026-11-24). None when the log has none."""
    d = run["data"]
    if "pf_x" not in d or "pf_y" not in d:
        return None
    ok = np.isfinite(d["pf_x"]) & np.isfinite(d["pf_y"])
    if not np.any(ok):
        return None
    t = np.asarray(run["t"], dtype=float)
    raw_t = raw_x = raw_y = np.zeros(0)
    if "corrected_pinger_x" in d:
        found = (d["corrected_pinger_x"] != 0.0) | (d["corrected_pinger_y"] != 0.0)
        raw_t, raw_x, raw_y = t[found], d["corrected_pinger_x"][found], d["corrected_pinger_y"][found]
    ugps = np.zeros((len(t), 19))
    for j, name in enumerate(UGPS_COLUMNS):
        if name in d:
            ugps[:, 7 + j] = d[name]
    return {"t": t[ok], "x": d["pf_x"][ok], "y": d["pf_y"][ok],
            "raw_t": raw_t, "raw_x": raw_x, "raw_y": raw_y,
            "n_detected": int(ok.sum()), "n_trusted": int(ok.sum()), "n_accepted": int(ok.sum()),
            "ugps_t": t, "ugps": ugps, "first_t": float(t[ok][0]), "last_t": float(t[ok][-1]),
            "params": {"method": "pinger_estimator, as logged (pf_* columns)"},
            "bag": None, "shift_s": 0.0, "method": "logged", "hold_s": HOLD_S}


def _from_core(core_run, to_mission, ugps_t, ugps, bag, shift):
    """Re-filter with the CURRENT pinger filter (pinger_filter_replay, the
    pinger_estimator's own core). None when the core cannot use the run (no
    pose topic, no usable fix) -- the heuristic below is then the fallback."""
    import pinger_filter_replay as pfr
    import pinger_params as pp
    if not any(k == "odom" for _, k, _v in core_run.events):
        return None
    params = pp.load_params()
    res = pfr.replay(core_run, params)
    est = [e for e in res["estimates"] if e.get("x") is not None]
    if len(est) < 2:
        return None
    t = to_mission(np.array([e["t"] for e in est], dtype=float))
    leg = np.array(res["legacy"], dtype=float) if res["legacy"] else np.zeros((0, 3))
    dec = [f["decision"] for f in res["fixes"]]
    used = sum(dec.count(k) for k in ("accepted", "downweighted", "reinit"))
    detected = len([d for d in dec if d not in ("duplicate", "invalid")])
    return {"t": t, "x": np.array([e["x"] for e in est]), "y": np.array([e["y"] for e in est]),
            "raw_t": to_mission(leg[:, 0]) if len(leg) else np.zeros(0),
            "raw_x": leg[:, 1] if len(leg) else np.zeros(0),
            "raw_y": leg[:, 2] if len(leg) else np.zeros(0),
            "n_detected": detected, "n_trusted": detected, "n_accepted": used,
            "ugps_t": ugps_t, "ugps": ugps, "first_t": float(t[0]), "last_t": float(t[-1]),
            "params": {"method": "re-filtered with the current pinger filter",
                       "preset": params.preset, "hash": params.hash},
            "bag": bag, "shift_s": float(shift), "method": "core",
            "hold_s": float(params["stale_timeout_s"])}


def attach(run):
    """Find the run's pinger and record the outcome on `run`: `run["pinger"]`
    is the estimate or None, `run["pinger_note"]` says why not (shown as the
    checkbox's tooltip). Never raises.

    In order (2026-11-24): the log's own filtered block (`pf_*`); the run's bag
    re-filtered with the current pinger filter; a pinger log's UGPS columns
    re-filtered with it; and, only when the filter cannot read the run, the
    heuristic filter below (bag, or the log's corrected_pinger columns)."""
    run["pinger"] = None
    found = from_filter_columns(run)
    if found is not None:
        run["pinger"] = found
        run["pinger_note"] = ("Show the filtered pinger the run logged (pinger_estimator, "
                              "pf_* columns): first at %.0f s" % found["first_t"])
        return
    if run["layout"] in ("pinger", "pinger_log") and find_bag(run["path"]) is None:
        found = None
        if "aco_x" in run["data"]:
            try:
                import pinger_filter_replay as pfr
                found = _from_core(pfr.load_csv(run["path"]),
                                   lambda tt: np.asarray(tt) - first_row_epoch(run),
                                   np.zeros(0), np.zeros((0, 19)), None, 0.0)
            except Exception:                           # noqa: BLE001 - fall back
                found = None
        if found is not None:
            run["pinger"] = found
            run["pinger_note"] = (
                "Show the pinger re-filtered from this log's UGPS columns with the current "
                "pinger filter (%s, rows 0.33 s apart): %d fixes, %d used"
                % (found["params"]["hash"], found["n_detected"], found["n_accepted"]))
            return
        found = estimate_from_log(run)
        if found is None:
            run["pinger_note"] = "this pinger log never detected the pinger"
            return
        run["pinger"] = found
        run["pinger_note"] = (
            "Show the filtered pinger (the log's own raw_pinger track, "
            "yaw-rate / consensus / speed filtered): %d detections, first at "
            "%.0f s, %d kept" % (found["n_detected"], found["first_t"],
                                 found["n_accepted"]))
        return
    bag = find_bag(run["path"])
    if bag is None:
        run["pinger_note"] = "no ros2 bag beside this log (%s)" % pb.pr.bag_name(run["path"])
        return
    try:
        data = read_bag(bag)
        if data["start_s"] is None:
            raise PingerBagError("the bag holds no messages")
        shift = time_shift(run, data["start_s"])
    except (PingerBagError, OSError, struct.error, ValueError) as exc:
        run["pinger_note"] = "bag unreadable: %s" % exc
        return
    try:
        import pinger_filter_replay as pfr
        found = _from_core(pfr.load_bag(bag), lambda tt: to_mission_time(run, tt, shift),
                           to_mission_time(run, data["ugps_t"], shift), data["ugps"], bag, shift)
    except Exception:                                   # noqa: BLE001 - fall back
        found = None
    if found is not None:
        run["pinger"] = found
        run["pinger_note"] = (
            "Show the pinger from the bag, re-filtered with the current pinger filter "
            "(%s): %d fixes, first at %.0f s, %d used"
            % (found["params"]["hash"], found["n_detected"], found["first_t"], found["n_accepted"]))
        return
    try:
        found = estimate(run, data, shift)
    except (PingerBagError, OSError, struct.error, ValueError) as exc:
        run["pinger_note"] = "bag unreadable: %s" % exc
        return
    if found is None:
        run["pinger_note"] = ("the bag holds no pinger detection "
                              "(%d messages, all zero)" % len(data["body"]))
        return
    run["pinger"] = found
    run["pinger_note"] = (
        "Show the pinger from the bag: %d detections, first at %.0f s, "
        "%d kept after the yaw-rate / speed filter"
        % (found["n_detected"], found["first_t"], found["n_accepted"]))


# ---------------------------------------------------------------------------
# What the panels and the export take from it
# ---------------------------------------------------------------------------

def held_at(found, when):
    """The estimate in force at each time of `when`: the last accepted sample
    at or before it, NaN before the first or more than HOLD_S after the last
    one. Returns (x, y) in the world frame."""
    when = np.asarray(when, dtype=float)
    index = np.searchsorted(found["t"], when, side="right") - 1
    ok = index >= 0
    safe = np.where(ok, index, 0)
    ok &= (when - found["t"][safe]) <= found.get("hold_s", HOLD_S)
    return (np.where(ok, found["x"][safe], np.nan),
            np.where(ok, found["y"][safe], np.nan))


def in_panel_frame(run, x, y):
    """World metres -> the track panel's own axes (degrees about the sidecar's
    origin in WGS84 mode). NaN everywhere if the frame cannot be placed."""
    if pb.track_series(run)["mode"] == "world":
        return np.asarray(x), np.asarray(y)
    origin = run.get("origin") or {}
    try:
        lat0, lon0 = float(origin["latitude"]), float(origin["longitude"])
    except (KeyError, TypeError, ValueError):
        nan = np.full(np.shape(x), np.nan)
        return nan, nan
    lat, lon = frame_math.local_en_to_latlon(x, y, lat0, lon0)
    return lon, lat


def panel_series(run):
    """The pinger on the panel's x/y, index-aligned with `run["t"]`, or None."""
    found = run.get("pinger")
    if not run.get("pinger_show") or found is None:
        return None
    wx, wy = held_at(found, run["t"])
    return in_panel_frame(run, wx, wy)


def track_points(run):
    """(x, y) of every accepted sample inside the cropped window, in the panel
    frame - what the static map draws."""
    found = run.get("pinger")
    if not run.get("pinger_show") or found is None or not len(run["t"]):
        return None
    inside = (found["t"] >= run["t"][0]) & (found["t"] <= run["t"][-1])
    if not np.any(inside):
        return None
    return in_panel_frame(run, found["x"][inside], found["y"][inside])


def export_columns(run):
    """Per data row of the run: a dict of the pinger-layout column values, for
    every row of the WHOLE run (the exporter picks the rows it keeps).

    Mirrors what `robot_interface` writes in the pinger layout: the target
    block becomes the pinger (world position and its WGS84), the twelve
    `/uw_gps_data[7:19]` values are the latest raw packet at or before the
    row, and a pinger not yet detected is 0.0 throughout.
    """
    found = run["pinger"]
    t = run["t"]
    n = len(t)
    cols = {name: np.zeros(n) for name in
            ("corrected_pinger_x", "corrected_pinger_y",
             "pinger_latitude", "pinger_longitude") + UGPS_COLUMNS}

    wx, wy = held_at(found, t)
    have = np.isfinite(wx)
    cols["corrected_pinger_x"][have] = wx[have]
    cols["corrected_pinger_y"][have] = wy[have]
    origin = run.get("origin") or {}
    if have.any() and "latitude" in origin and "longitude" in origin:
        lat, lon = frame_math.local_en_to_latlon(
            wx[have], wy[have], float(origin["latitude"]), float(origin["longitude"]))
        cols["pinger_latitude"][have] = lat
        cols["pinger_longitude"][have] = lon

    index = np.searchsorted(found["ugps_t"], t, side="right") - 1
    seen = index >= 0
    packet = found["ugps"][np.where(seen, index, 0)][:, 7:19]
    for j, name in enumerate(UGPS_COLUMNS):
        cols[name][seen] = packet[seen, j]
    return cols


def source_name(run):
    """Where the numbers came from, for export.yaml."""
    found = run["pinger"]
    if found.get("method") in ("logged", "core"):
        out = {"method": found["params"].get("method"),
               "detections": found["n_detected"], "kept": found["n_accepted"],
               "first_detection_s": round(found["first_t"], 3),
               "last_sample_s": round(found["last_t"], 3),
               "filter": found["params"], "hold_s": found.get("hold_s", HOLD_S)}
        if found["bag"] is not None:
            out.update(bag=os.path.basename(found["bag"]), bag_to_log_zone_shift_s=found["shift_s"])
        return out
    if found["bag"] is None:
        return {"source": "the log's own corrected_pinger_x/y",
                "detections": found["n_detected"], "kept": found["n_accepted"],
                "first_detection_s": round(found["first_t"], 3),
                "last_sample_s": round(found["last_t"], 3),
                "filter": found["params"], "hold_s": HOLD_S}
    return {"bag": os.path.basename(found["bag"]),
            "topics": [BODY_TOPIC, UGPS_TOPIC],
            "detections": found["n_detected"], "kept": found["n_accepted"],
            "first_detection_s": round(found["first_t"], 3),
            "last_sample_s": round(found["last_t"], 3),
            "bag_to_log_zone_shift_s": found["shift_s"],
            "filter": found["params"],
            "hold_s": HOLD_S}

