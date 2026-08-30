#!/usr/bin/env python3
"""Replay a recorded run through the offline harness (N7).

`sim.py` evaluates a controller against a *simulated* plant. This replays a
*recording* -- a rosbag, a controller .npy log or a position CSV -- and emits
the trajectory-vs-target comparison in the same units and table shape
`analyze.py` already produces, so a field run can be scored, and a different
controller asked what it would have commanded, without going back on the water.

Two things come out of a recording:

  as_flown(rec)              what the boat actually did against the target it
                             was actually given -- analyze.py's own metrics.
  counterfactual(rec, ctrl)  what a chosen sim.py controller would have
                             commanded from the same logged states, beside the
                             commands that were really sent.

Reading is strictly read-only: recorded .svlog, CSV and .npy files are primary
field record (CLAUDE.md #6 / CM-7) and nothing here opens them for writing.

Sources
-------
.npy   controller log, schema ['t','x','y','psi','x_d','y_d','psi_d','u1','u2']
       with the header appended as a row of strings (CLAUDE.md #6). No twist in
       the schema, so u/v/r are derived from the pose.
.csv   position/pinger log, read by column name because the writer fills by
       name and column order is explicitly not stable. The no-pinger layout has
       target_x/target_y and no target_psi, so psi_d stays NaN.
bag    rosbag2 directory: /blueboat/odom (pose AND body-frame twist),
       /monitoring_data, /thruster_input. Needs a sourced ROS 2 workspace;
       rosbag2_py is imported lazily so the rest of the harness stays ROS-free.

Run:  python3 replay.py <recording> [--controller PID|LoS|MPC]
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np

import analyze
import sim

# Wire names read out of a bag. Not published, not subscribed -- read only.
TOPIC_ODOM = "/blueboat/odom"
TOPIC_MONITORING = "/monitoring_data"
TOPIC_THRUST = "/thruster_input"

# /monitoring_data and the .npy log carry the same nine columns (CLAUDE.md #6).
MONITORING_COLUMNS = ["t", "x", "y", "psi", "x_d", "y_d", "psi_d", "u1", "u2"]


class RecordingError(Exception):
    """A recording that cannot be read, named and explained rather than raised raw."""


# ───────────────────────────── record assembly ────────────────────────────────
def _derive_twist(t, x, y, psi):
    """Body-frame u, v, r by central difference of the logged pose.

    Only for sources whose schema has no twist (.npy, .csv). A bag carries the
    real thing on /blueboat/odom and never comes through here.
    """
    dt = np.gradient(t)
    dt[dt == 0] = np.nan
    xd, yd = np.gradient(x) / dt, np.gradient(y) / dt
    c, s = np.cos(psi), np.sin(psi)
    u = c * xd + s * yd
    v = -s * xd + c * yd
    r = np.gradient(np.unwrap(psi)) / dt
    return (np.nan_to_num(u, nan=0.0), np.nan_to_num(v, nan=0.0),
            np.nan_to_num(r, nan=0.0))


def _record(t, x, y, psi, xd, yd, psid, u1, u2, twist=None, source="", path=""):
    rec = {k: np.asarray(v, dtype=float) for k, v in
           (("t", t), ("x", x), ("y", y), ("psi", psi), ("xd", xd), ("yd", yd),
            ("psid", psid), ("u1", u1), ("u2", u2))}
    if twist is None:
        rec["u"], rec["v"], rec["r"] = _derive_twist(rec["t"], rec["x"], rec["y"], rec["psi"])
        rec["twist_source"] = "derived"
    else:
        rec["u"], rec["v"], rec["r"] = (np.asarray(a, dtype=float) for a in twist)
        rec["twist_source"] = "logged"
    rec["source"], rec["path"] = source, path
    return rec


# ───────────────────────────── readers ────────────────────────────────────────
def read_npy(path):
    """Controller .npy log. np.save coerced every row to <U32 via the string header."""
    if os.path.getsize(path) == 0:
        raise RecordingError(f"{path} is empty (0 bytes) -- an interrupted run wrote no rows")
    try:
        raw = np.load(path, allow_pickle=True)
    except (EOFError, ValueError) as exc:
        raise RecordingError(f"{path} is not a readable .npy: {exc}") from exc
    if raw.ndim != 2 or raw.shape[1] != len(MONITORING_COLUMNS):
        raise RecordingError(
            f"{path}: expected N x {len(MONITORING_COLUMNS)} ({', '.join(MONITORING_COLUMNS)}), "
            f"got shape {raw.shape}")
    rows = raw
    if str(rows[0][0]) == "t":                       # header row appended as strings
        rows = rows[1:]
    if len(rows) < 3:
        raise RecordingError(f"{path}: only {len(rows)} data rows, too short to score")
    a = rows.astype(float)
    return _record(*(a[:, i] for i in range(9)), source="npy", path=path)


def read_poslog_csv(path):
    """Position/pinger CSV, no-pinger layout. Read by column NAME (CLAUDE.md #6)."""
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise RecordingError(f"{path}: no rows")
    cols = rows[0].keys()

    def col(name, default=np.nan):
        if name not in cols:
            return np.full(len(rows), default)
        return np.array([float(r[name]) if r[name] not in ("", None) else np.nan
                         for r in rows])

    for required in ("x", "y"):
        if required not in cols:
            raise RecordingError(
                f"{path}: no '{required}' column. Columns present: {', '.join(cols)}")
    if "target_x" not in cols:
        raise RecordingError(
            f"{path}: no 'target_x' column -- this is the pinger CSV layout, which records "
            "corrected_pinger_x/y rather than the world-frame target, so there is nothing "
            "to score a path against")
    t = col("t") if "t" in cols else np.arange(len(rows), dtype=float)
    # No target_psi column exists in this layout; psi_d stays NaN rather than invented.
    return _record(t, col("x"), col("y"), col("psi"),
                   col("target_x"), col("target_y"), np.full(len(rows), np.nan),
                   col("u1"), col("u2"), source="csv", path=path)


def read_bag(path):
    """rosbag2 directory. Resampled onto /monitoring_data, master_control's own tick."""
    try:                                              # lazy: keeps the harness ROS-free
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RecordingError(
            f"reading a rosbag needs a sourced ROS 2 workspace (source /opt/ros/<distro>/"
            f"setup.bash): {exc}") from exc

    reader = rosbag2_py.SequentialReader()
    storage_id = "mcap"
    for entry in os.listdir(path):
        if entry.endswith(".db3"):
            storage_id = "sqlite3"
            break
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if TOPIC_MONITORING not in types:
        raise RecordingError(
            f"{path}: no {TOPIC_MONITORING} in the bag (topics: {', '.join(sorted(types))}). "
            "That topic carries the target, so without it there is nothing to compare against")

    mon, odom, thr = [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == TOPIC_MONITORING:
            mon.append((stamp, list(deserialize_message(data, get_message(types[topic])).data)))
        elif topic == TOPIC_ODOM:
            odom.append((stamp, deserialize_message(data, get_message(types[topic]))))
        elif topic == TOPIC_THRUST:
            thr.append((stamp, list(deserialize_message(data, get_message(types[topic])).data)))
    if len(mon) < 3:
        raise RecordingError(f"{path}: only {len(mon)} {TOPIC_MONITORING} messages")

    mon.sort(key=lambda p: p[0])
    m = np.array([p[1] for p in mon], dtype=float)
    stamps = np.array([p[0] for p in mon], dtype=float)

    twist = None
    if odom:                                          # twist is body-frame already (N3)
        odom.sort(key=lambda p: p[0])
        ost = np.array([p[0] for p in odom], dtype=float)
        ot = np.array([[o.twist.twist.linear.x, o.twist.twist.linear.y,
                        o.twist.twist.angular.z] for _, o in odom], dtype=float)
        idx = np.searchsorted(ost, stamps).clip(0, len(ost) - 1)
        twist = (ot[idx, 0], ot[idx, 1], ot[idx, 2])

    u1, u2 = m[:, 7], m[:, 8]
    if thr:                                           # prefer what was actually sent
        thr.sort(key=lambda p: p[0])
        tst = np.array([p[0] for p in thr], dtype=float)
        tv = np.array([p[1] for p in thr], dtype=float)
        idx = np.searchsorted(tst, stamps).clip(0, len(tst) - 1)
        u1, u2 = tv[idx, 0], tv[idx, 1]

    return _record(m[:, 0], m[:, 1], m[:, 2], m[:, 3], m[:, 4], m[:, 5], m[:, 6],
                   u1, u2, twist=twist, source="bag", path=path)


def load(path):
    """Dispatch on what the path actually is."""
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(path):
        raise RecordingError(f"{path}: no such file or directory")
    if os.path.isdir(path):
        return read_bag(path)
    if path.endswith(".npy"):
        return read_npy(path)
    if path.endswith(".csv"):
        return read_poslog_csv(path)
    raise RecordingError(
        f"{path}: unrecognised recording. Expected a rosbag directory, a controller .npy "
        "log or a position .csv")


# ───────────────────────────── scoring ────────────────────────────────────────
def as_flown(rec):
    """Logged pose against the logged target, in sim.run's log shape.

    e_y and e_along use the same projection as sim.Governor.errors: onto the
    target heading where one was recorded, onto the boat-to-target bearing
    where it was not (the CSV layout has no target_psi).

    tau is NOT in any recording -- the path parameter lives inside
    master_control and is never published -- so it comes back NaN and
    analyze.py's progress column reads n/a. `arc_rate` below is offered
    instead, and is explicitly a different quantity.
    """
    x, y, psi = rec["x"], rec["y"], rec["psi"]
    xd, yd, psid = rec["xd"], rec["yd"], rec["psid"]
    dx, dy = xd - x, yd - y
    heading = np.where(np.isnan(psid), np.arctan2(dy, dx), psid)
    c, s = np.cos(heading), np.sin(heading)
    e_along = c * dx + s * dy
    e_y = -(x - xd) * s + (y - yd) * c
    n = len(rec["t"])
    return {
        "t": rec["t"], "x": x, "y": y, "psi": psi,
        "u": rec["u"], "v": rec["v"], "r": rec["r"],
        "xd": xd, "yd": yd, "psid": psid,
        "e_y": e_y, "e_along": e_along,
        "tau": np.full(n, np.nan),                    # not recoverable from a recording
        "factor": np.full(n, np.nan),
        "thr_r": rec["u1"], "thr_l": rec["u2"],
        "Ud": np.full(n, np.nan),
    }


def arc_rate(rec):
    """Target-track arc length over run time. A progress proxy, NOT tau/T."""
    span = rec["t"][-1] - rec["t"][0]
    if span <= 0:
        return float("nan")
    return float(np.nansum(np.hypot(np.diff(rec["xd"]), np.diff(rec["yd"]))) / span)


def states(rec):
    """The logged states, in sim's [x, y, psi, u, v, r] order and wrapped like odom."""
    return np.column_stack([rec["x"], rec["y"], np.array([sim.wrap(a) for a in rec["psi"]]),
                            rec["u"], rec["v"], rec["r"]])


def windows(rec):
    """Reference windows rebuilt from the recording alone -- necessarily approximate.

    master_control publishes ONE target pose per tick on /monitoring_data: win[0],
    the pose at the current path parameter. The controller was handed the whole
    window, and `compute_target` reads win[1] for the reference position and the
    window's own span for the speed feedforward U_d. Neither win[1] nor U_d is on
    the wire, so a recording cannot recover them: the best available window is
    two CONSECUTIVE logged targets, which span the *governed* advance
    (tau_dot * dt) rather than the window's path_time.

    Where the governor is not throttling the two coincide. Where it is -- the
    shipped inner gains throttle to about 0.43x -- the reconstructed U_d is low
    by that factor, and a counterfactual built on it under-commands surge to
    match. check_replay.py measures that gap rather than tolerating it.
    """
    n = len(rec["t"])
    out = []
    for k in range(n):
        nxt = min(k + 1, n - 1)
        psi_k = rec["psid"][k]
        psi_n = rec["psid"][nxt]
        if np.isnan(psi_k):                           # CSV layout has no target_psi
            psi_k = math.atan2(rec["yd"][nxt] - rec["yd"][k], rec["xd"][nxt] - rec["xd"][k])
        if np.isnan(psi_n):
            psi_n = psi_k
        out.append([(rec["xd"][k], rec["yd"][k], psi_k),
                    (rec["xd"][nxt], rec["yd"][nxt], psi_n)])
    return out


def counterfactual(rec, ctrl, reference=None):
    """Step `ctrl` over the logged states; return its commands beside the logged ones.

    The controller is stateful (PIDLoS carries four integrators), so this steps
    in recorded order from a fresh instance -- which is what makes the
    cross-validation gate in check_replay.py meaningful.

    `reference` overrides the reconstructed windows with an explicit list of
    (win, dt) pairs. Only the gate uses it, to separate "does the replay step the
    controller correctly" (exact, and it does) from "can a recording recover the
    reference the controller saw" (it cannot, see `windows`).
    """
    st = states(rec)
    if reference is None:
        wins = windows(rec)
        dts = np.diff(rec["t"], append=rec["t"][-1] + 0.05)
        reference = [(w, d if d > 0 else 0.05) for w, d in zip(wins, dts)]
    out = np.zeros((len(st), 2))
    for k, (win, dt) in enumerate(reference):
        thr, _ = ctrl(win, st[k], dt)
        out[k] = np.clip(np.asarray(thr, dtype=float), -sim.THR_LIM, sim.THR_LIM)
    return out


def make_controller(name, dt=0.05):
    if name == "PID":
        return sim.PIDController(dt=dt)
    if name == "LoS":
        return sim.LoSController()
    if name == "MPC":
        return sim.MPCController()
    raise ValueError(f"unknown controller {name!r} (PID, LoS or MPC)")


# ───────────────────────────── report ─────────────────────────────────────────
def report(path, controller=None):
    rec = load(path)
    d = as_flown(rec)
    n = int(len(d["t"]) * 0.25)

    print(f"\n{os.path.basename(rec['path'])}   ({rec['source']}, {len(d['t'])} samples, "
          f"{d['t'][-1] - d['t'][0]:.0f} s, twist {rec['twist_source']})")
    print(f"  {'':<10}{'RMS e_y':>9}{'peak e_y':>10}{'settle<0.5m':>13}"
          f"{'u_mean':>9}{'tau/T':>8}{'|thrust|':>10}{'sat':>8}{'sat_tail':>10}")
    st = analyze.settle(d)
    print(f"  {'as flown':<10}{np.sqrt(np.mean(d['e_y'][-n:] ** 2)):>9.3f}"
          f"{np.max(np.abs(d['e_y'])):>10.2f}"
          f"{('never' if np.isnan(st) else f'{st:.0f} s'):>13}"
          f"{np.mean(d['u'][-n:]):>9.3f}{'n/a':>8}"
          f"{np.mean(np.abs(d['thr_r'][-n:]) + np.abs(d['thr_l'][-n:])) / 2:>10.1f}"
          f"{analyze.saturated(d) * 100:>7.1f}%{analyze.saturated(d, lo=-n) * 100:>9.1f}%")
    print(f"  tau/T is n/a: the path parameter is internal to master_control and is not "
          f"recorded.\n  Target-track arc length / run time = {arc_rate(rec):.3f} m/s "
          f"(a different quantity, not progress).")

    if controller:
        ctrl = make_controller(controller)
        cf = counterfactual(rec, ctrl)
        d_r = cf[:, 0] - d["thr_r"]
        d_l = cf[:, 1] - d["thr_l"]
        print(f"\n  counterfactual: {controller} replayed over the same logged states")
        print(f"    RMS command difference   right {np.sqrt(np.mean(d_r ** 2)):.3f} N, "
              f"left {np.sqrt(np.mean(d_l ** 2)):.3f} N")
        print(f"    mean |command|           replayed {np.mean(np.abs(cf)):.2f} N, "
              f"logged {np.mean(np.abs(np.c_[d['thr_r'], d['thr_l']])):.2f} N")
        if rec["twist_source"] == "derived":
            print("    twist was derived from the pose, so this is indicative, not exact.")
    return rec, d


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("recording", help="rosbag directory, controller .npy log, or position .csv")
    ap.add_argument("--controller", choices=("PID", "LoS", "MPC"),
                    help="also replay this controller over the logged states")
    args = ap.parse_args(argv)
    try:
        report(args.recording, args.controller)
    except RecordingError as exc:
        print(f"cannot read recording: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
