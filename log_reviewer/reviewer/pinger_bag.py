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

import mmap
import os
import struct
import time

import numpy as np

from . import poslog_bridge as pb

import frame_math                                                   # noqa: E402

BODY_TOPIC = "/blueboat/pinger_coordinates"
UGPS_TOPIC = "/uw_gps_data"
UGPS_COLUMNS = ("aco_x", "aco_y", "aco_z", "ant_x", "ant_y", "ant_z",
                "lat", "lon", "dep", "filaco_x", "filaco_y", "filaco_z")

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


class PingerBagError(Exception):
    """The bag cannot give a pinger for this run (the message says why)."""


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


def _mcap_files(bag):
    # os.listdir, not glob: a bag filed from staging keeps a hidden
    # `.run-bag-<pid>-<k>_0.mcap` name, which a `*.mcap` glob skips.
    names = sorted(n for n in os.listdir(bag) if n.endswith(".mcap"))
    return [os.path.join(bag, n) for n in names]


def _string(buf, pos):
    n = struct.unpack_from("<I", buf, pos)[0]
    return bytes(buf[pos + 4:pos + 4 + n]).decode("utf-8", "replace"), pos + 4 + n


def _decompress(name, blob):
    if name == "zstd":
        try:
            import zstandard
        except ImportError as exc:
            raise PingerBagError("the bag's chunks are zstd-compressed and "
                                 "'zstandard' is not installed") from exc
        return zstandard.ZstdDecompressor().decompress(blob)
    if name == "lz4":
        try:
            import lz4.frame
        except ImportError as exc:
            raise PingerBagError("the bag's chunks are lz4-compressed and "
                                 "'lz4' is not installed") from exc
        return lz4.frame.decompress(blob)
    raise PingerBagError("unknown MCAP chunk compression %r" % name)


def _records(buf, pos, end):
    while pos + 9 <= end:
        op = buf[pos]
        length = struct.unpack_from("<Q", buf, pos + 1)[0]
        yield op, pos + 9, pos + 9 + length
        pos += 9 + length


def read_topics(path, topics):
    """{topic: [(log_time_ns, payload bytes), ...]} and the file's (first, last)
    message time in ns. Only the wanted topics are copied out."""
    wanted = set(topics)
    out = {topic: [] for topic in topics}
    channels = {}
    span = [None, None]

    def visit(buf, start, end):
        for op, a, b in _records(buf, start, end):
            if op == 0x04:                                   # Channel
                cid = struct.unpack_from("<H", buf, a)[0]
                channels[cid] = _string(buf, a + 4)[0]
            elif op == 0x05:                                 # Message
                cid, _seq, log_time = struct.unpack_from("<HIQ", buf, a)
                topic = channels.get(cid)
                if topic in wanted:
                    out[topic].append((log_time, bytes(buf[a + 22:b])))

    with open(path, "rb") as handle:
        with mmap.mmap(handle.fileno(), 0, access=mmap.ACCESS_READ) as buf:
            if bytes(buf[:8]) != b"\x89MCAP0\r\n":
                raise PingerBagError("%s is not an MCAP file" % os.path.basename(path))
            for op, a, b in _records(buf, 8, len(buf) - 8):
                if op == 0x06:                               # Chunk
                    first, last = struct.unpack_from("<QQ", buf, a)
                    span[0] = first if span[0] is None else min(span[0], first)
                    span[1] = last if span[1] is None else max(span[1], last)
                    compression, p = _string(buf, a + 28)
                    records_len = struct.unpack_from("<Q", buf, p)[0]
                    body = bytes(buf[p + 8:p + 8 + records_len])
                    if compression:
                        body = _decompress(compression, body)
                    visit(body, 0, len(body))
                elif op == 0x0F:                             # DataEnd
                    break
                else:
                    visit(buf, a - 9, b)                     # loose records
    return out, tuple(span)


def decode_float32_multiarray(payload):
    """The `data` of a std_msgs/Float32MultiArray in little-endian CDR."""
    if len(payload) < 16 or payload[1] != 1:
        raise PingerBagError("unexpected CDR encoding in a pinger message")
    pos = 4
    dims = struct.unpack_from("<I", payload, pos)[0]
    pos += 4
    for _ in range(dims):                       # MultiArrayDimension: label, size, stride
        n = struct.unpack_from("<I", payload, pos)[0]
        pos = (pos + 4 + n + 3) & ~3
        pos += 8
    pos = (pos + 3) & ~3
    pos += 4                                    # layout.data_offset
    count = struct.unpack_from("<I", payload, pos)[0]
    pos += 4
    return np.frombuffer(payload, "<f4", count, pos).astype(float)


def read_bag(bag):
    """The pinger messages of a bag.

    Returns {"bag", "start_s", "end_s", "body_t", "body", "ugps_t", "ugps"}:
    times are EPOCH seconds of the bag's receive clock, `body` is (n, 3) and
    `ugps` is (n, 19) - the raw packet exactly as `robot_interface` logs it.
    """
    files = _mcap_files(bag)
    if not files:
        raise PingerBagError("no .mcap file inside %s" % bag)
    topics = {BODY_TOPIC: [], UGPS_TOPIC: []}
    first = last = None
    for path in files:
        part, span = read_topics(path, topics)
        for topic in topics:
            topics[topic].extend(part[topic])
        if span[0] is not None:
            first = span[0] if first is None else min(first, span[0])
            last = span[1] if last is None else max(last, span[1])

    def unpack(rows, width):
        rows.sort(key=lambda r: r[0])
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


def attach(run):
    """Find the run's pinger and record the outcome on `run`: `run["pinger"]`
    is the estimate or None, `run["pinger_note"]` says why not (shown as the
    checkbox's tooltip). Never raises.

    A with-pinger log carries the pinger itself; any other log is looked up in
    its ros2 bag."""
    run["pinger"] = None
    if run["layout"] == "pinger":
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
    ok &= (when - found["t"][safe]) <= HOLD_S
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

