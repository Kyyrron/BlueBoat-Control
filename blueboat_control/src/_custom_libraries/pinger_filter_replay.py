#!/usr/bin/env python3
r"""
pinger_filter_replay -- run the pinger filter offline on recorded runs.

ROS-FREE (numpy + PyYAML; matplotlib only for figures, imported lazily). It
drives the SAME EstimatorCore the pinger_estimator node runs, so a bag replayed
here gives what the node would have published on that event sequence.

Inputs (auto-detected)
  * a run bag (folder or .mcap): /blueboat/odom, /mavros/imu/data, /uw_gps_data,
    /uw_gps/fix (receive times);
  * a poslog CSV or log-reviewer export with the pinger/UGPS columns (3 Hz rows:
    a fix's receive time is only known to within one row, flagged "csv-timed");
  * a per-fix log `...-pingerlog.csv` (bit-identical re-run of the live filter).

Commands
  run       one input -> fixes.csv, estimate.csv, smoothed.csv, metrics.json, figures
  grid      latency / y_sign / mount_yaw search, scored by local consistency and,
            with --outline, the fraction of fixes outside a known outline
  evaluate  A2 table over many inputs: real-fix metrics and semi-synthetic
            known-truth replays (real motion, timing and residuals)

Examples
  pinger_filter_replay.py run  <bag> --out /tmp/pf --outline docs/img/pinger_findings/makurazaki_pool.yaml
  pinger_filter_replay.py grid <bag> [<bag> ...] --grid latency_s=0:3:0.25 y_sign=-1,1 mount_yaw_deg=-20:20:5
  pinger_filter_replay.py evaluate ~/ros2_ws/data --out /tmp/a2 --set sigma_bearing_deg=12
"""

import argparse
import csv
import glob
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import frame_math as fm                                              # noqa: E402
import mcap_lite as ml                                               # noqa: E402
import pinger_log as plog                                            # noqa: E402
import pinger_input as pin                                           # noqa: E402
import pinger_params as pp                                           # noqa: E402
import robot_log_schema as rls                                       # noqa: E402
from pinger_estimator_core import EstimatorCore, legacy_world        # noqa: E402

TOPICS = ["/blueboat/odom", "/mavros/imu/data", "/uw_gps_data", "/uw_gps/fix",
          "/blueboat/pinger_coordinates"]
PINGERLOG_SUFFIX = "-pingerlog.csv"


# ---------------------------------------------------------------------------
# inputs -> events
# ---------------------------------------------------------------------------

class Run:
    """A recorded run as an event list: (t, kind, payload) sorted by time.

    kinds: odom (x, y, yaw, vx, vy, vz) | imu (wz) | packet (19 floats) |
           fixjson (str) | fixpose (Fix, pose) | legacy_rec (t, x_w, y_w) recorded
    """

    def __init__(self, name, kind, events, origin=None, path=None):
        self.name, self.kind, self.events = name, kind, events
        self.origin = origin            # (lat0, lon0) of the local ENU frame, if known
        self.path = path
        self.legacy_rec = []            # (t, x, y) legacy world as recorded (CSV inputs)


def _origin_near(path):
    folder = path if os.path.isdir(path) else os.path.dirname(path)
    for cand in sorted(glob.glob(os.path.join(folder, "*-origin.yaml")) +
                       glob.glob(os.path.join(os.path.dirname(folder), "*-origin.yaml"))):
        try:
            import yaml
            d = yaml.safe_load(open(cand)) or {}
            return float(d["latitude"]), float(d["longitude"])
        except Exception:                               # noqa: BLE001 - best effort
            continue
    return None


def load_bag(path):
    out, _ = ml.read_bag(path, TOPICS)
    ev = []
    for lt, p in out["/blueboat/odom"]:
        o = ml.decode_odometry(p)
        ev.append((lt * 1e-9, 0, "odom", (o["x"], o["y"], ml.yaw_of(o["qx"], o["qy"], o["qz"], o["qw"]),
                                          o["vx"], o["vy"], o["vz"])))
    for lt, p in out["/mavros/imu/data"]:
        ev.append((lt * 1e-9, 1, "imu", ml.decode_imu(p)["angular_velocity"][2]))
    for lt, p in out["/uw_gps_data"]:
        ev.append((lt * 1e-9, 2, "packet", [float(v) for v in ml.decode_float32_multiarray(p)]))
    for lt, p in out["/uw_gps/fix"]:
        ev.append((lt * 1e-9, 3, "fixjson", ml.decode_string(p)))
    ev.sort(key=lambda e: (e[0], e[1]))
    name = os.path.basename(os.path.normpath(path))
    run = Run(name[:-4] if name.endswith("-bag") else name, "bag",
              [(t, k, v) for t, _, k, v in ev], _origin_near(path), path)
    rec = out["/blueboat/pinger_coordinates"]
    run.recorded_body = [(lt * 1e-9, ml.decode_float32_multiarray(p)) for lt, p in rec]
    return run


def _read_csv(path):
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    hi = None
    for i, row in enumerate(rows[:3]):
        if "relative_x" in row:
            hi = i
            break
    if hi is None:
        raise ValueError(f"{path}: not a poslog (no relative_x header)")
    hdr = rows[hi]
    data = [r for r in rows[hi + 1:] if len(r) == len(hdr)]
    cols = {h: np.array([float(r[i]) if r[i] not in ("", "nan") else np.nan for r in data])
            for i, h in enumerate(hdr)}
    return cols


def load_csv(path):
    d = _read_csv(path)
    if "aco_x" not in d:
        raise ValueError(f"{path}: no UGPS columns (aco_x...): the filter cannot read this run")
    t = np.array([time.mktime((int(a), int(b), int(c), int(e), int(f), int(g), 0, 0, -1))
                  for a, b, c, e, f, g in zip(d["Year"], d["Month"], d["Day"], d["Hour"],
                                              d["Minute"], d["Second"])]) + d["MicroSecond"] * 1e-6
    ev = []
    last = None
    for i in range(len(t)):
        ev.append((t[i], "odom", (d["relative_x"][i], d["relative_y"][i], d["relative_psi"][i], 0.0, 0.0, 0.0)))
        pk = [0.0] * 19
        for k, name in enumerate(rls.UGPS_COLUMNS):         # /uw_gps_data[7:19]
            pk[7 + k] = float(d[name][i])
        key = tuple(pk[7:19])
        if key != last:
            ev.append((t[i] + 1e-6, "packet", pk))
            last = key
    run = Run(os.path.splitext(os.path.basename(path))[0], "csv", ev, _origin_near(path), path)
    if "corrected_pinger_x" in d:
        found = (d["corrected_pinger_x"] != 0) | (d["corrected_pinger_y"] != 0)
        run.legacy_rec = list(zip(t[found], d["corrected_pinger_x"][found], d["corrected_pinger_y"][found]))
    return run


# The per-fix log format lives in pinger_log.py (shared with robot_interface).
def load_pingerlog(path):
    """A per-fix log -> fixpose events (exact re-run of the live filter)."""
    rows = plog.read_rows(path)
    ev = []
    for r in rows:
        if r["decision"] == "config":
            ev.append((float(r["t_rx"]), "config", r["reason"].replace(";", ",")))
            continue
        if r["decision"] == "state":
            ev.append((float(r["t_rx"]), "state", json.loads(r["reason"].replace(";", ","))))
            continue

        def num(k):
            return float(r[k]) if r[k] not in ("", None) else None
        f = pin.Fix(num("t_rx"), (num("a_x") or 0.0, num("a_y") or 0.0, num("a_z") or 0.0), r["source"],
                    r["origin"], std=num("std"),
                    position_valid=None if r["position_valid"] == "" else bool(int(r["position_valid"])),
                    receivers_valid=None if r["receivers_valid"] == "" else int(r["receivers_valid"]),
                    invalid=r["input_invalid"] or None,
                    seq=None if r["seq"] == "" else int(r["seq"]))
        pose = None
        if r["pose_x"] != "":
            pose = (num("pose_x"), num("pose_y"), num("pose_yaw"), num("pose_yaw_rate"))
        ev.append((f.t_rx, "fixpose", (f, pose)))
    return Run(os.path.basename(path)[:-len(PINGERLOG_SUFFIX)], "pingerlog", ev, _origin_near(path), path)


def load(path):
    if os.path.isdir(path) or path.endswith(".mcap"):
        return load_bag(path)
    if path.endswith(PINGERLOG_SUFFIX):
        return load_pingerlog(path)
    return load_csv(path)


# ---------------------------------------------------------------------------
# replay
# ---------------------------------------------------------------------------

def replay(run, params, pinger_source="filter", keep_bodies=False):
    """Feed a run through EstimatorCore. Returns a dict of arrays/lists."""
    core = EstimatorCore(params, pinger_source)
    fixes, est, legacy, bodies, poses = [], [], [], [], []
    seeded = False
    for t, kind, v in run.events:
        if kind == "odom":
            o = core.on_odom(t, *v)
            poses.append((t, v[0], v[1], v[2]))
            lw = legacy_world(v[0], v[1], v[2], o["legacy_body"])
            if lw is not None:
                legacy.append((t, lw[0], lw[1]))
            if keep_bodies:
                bodies.append((t, o["body"], o["legacy_body"], o["filtered_body"]))
        elif kind == "imu":
            core.on_imu(t, v)
        elif kind in ("packet", "fixjson", "fixpose"):
            if kind == "packet":
                res = core.on_packet(t, v)
            elif kind == "fixjson":
                res = core.on_fix_json(t, v)
            else:
                res = [core.process_with_pose(*v)]
            for r in res:
                fixes.append(r)
                if r["decision"] != "duplicate":
                    e = core.estimate(t)
                    est.append(e)
        elif kind == "config":
            core.set_params(pp.load_params(preset=pp.CUSTOM, overrides=json.loads(v)))
        elif kind == "state":
            # The first snapshot seeds the filter (a per-fix log starts at one);
            # later ones are checkpoints of the same state, not re-applied.
            if not seeded:
                core.filter.restore(v)
                seeded = True
    if run.kind == "csv":
        legacy = list(run.legacy_rec)
    return {"core": core, "fixes": fixes, "estimates": est, "legacy": legacy, "bodies": bodies,
            "poses": poses, "smoothed": core.filter.rts_smooth()}


# ---------------------------------------------------------------------------
# metrics
# ---------------------------------------------------------------------------

OUTLINE_MAX_DIST_M = 20.0      # a run launched farther than this outside the outline is another site


def outline_local(outline_path, origin):
    """The outline's corners in the run's local ENU frame, or None when the run
    was recorded at another site (its launch point, the frame origin, lies more
    than OUTLINE_MAX_DIST_M outside the outline -- the Makurazaki harbour is
    only ~220 m from the pool)."""
    import yaml
    d = yaml.safe_load(open(outline_path))
    lat0, lon0 = origin
    east, north = fm.latlon_to_local_en([la for la, _ in d["corners"]],
                                        [lo for _, lo in d["corners"]], lat0, lon0)
    poly = list(zip(east.tolist(), north.tolist()))
    if float(inside_dist([0.0], [0.0], poly)[0]) < -OUTLINE_MAX_DIST_M:
        return None                     # the launch point (origin) is not at this site
    return poly


def inside_dist(px, py, poly):
    """Signed distance to a convex outline (> 0 inside)."""
    px, py = np.asarray(px, float), np.asarray(py, float)
    n = len(poly)
    area = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))
    sg = 1.0 if area > 0 else -1.0
    d = np.full(px.shape, np.inf)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        d = np.minimum(d, sg * (ex * (py - y1) - ey * (px - x1)) / math.hypot(ex, ey))
    return d


def _pct(a, q):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if len(a) else None


def track_arrays(result):
    e = [x for x in result["estimates"] if x.get("x") is not None]
    est = np.array([[x["t"], x["x"], x["y"]] for x in e]) if e else np.zeros((0, 3))
    leg = np.array(result["legacy"]) if result["legacy"] else np.zeros((0, 3))
    sm = np.array([[s["t"], s["x"], s["y"]] for s in result["smoothed"]]) if result["smoothed"] else np.zeros((0, 3))
    raw = np.array([[f["t_meas"], f["world"][0], f["world"][1]] for f in result["fixes"]
                    if f.get("world") is not None and f["decision"] != "duplicate"]) \
        if result["fixes"] else np.zeros((0, 3))
    return est, leg, sm, raw


def step_speed(tr):
    if len(tr) < 2:
        return np.zeros(0), np.zeros(0)
    d = np.hypot(np.diff(tr[:, 1]), np.diff(tr[:, 2]))
    dt = np.diff(tr[:, 0])
    ok = dt > 1e-3
    return d, d[ok] / dt[ok]


def metrics(result, run, outline=None):
    fx = result["fixes"]
    dec = {}
    for f in fx:
        dec[f["decision"]] = dec.get(f["decision"], 0) + 1
    nis = [f["nis"] for f in fx if f["decision"] in ("accepted", "downweighted") and f.get("nis") is not None]
    est, leg, sm, raw = track_arrays(result)
    acc = [f["nis"] for f in fx if f["decision"] == "accepted" and f.get("nis") is not None]
    out = {"run": run.name, "input": run.kind, "fixes": len(fx), "decisions": dec,
           "nis_mean_accepted": float(np.mean(acc)) if acc else None,
           "nis_mean_used": float(np.mean(nis)) if nis else None,
           "nis_median_used": float(np.median(nis)) if nis else None,
           "nis_frac_below_gate": float(np.mean(np.array(nis) <= 5.99)) if nis else None}
    for lab, tr in (("filtered", est), ("legacy", leg), ("smoothed", sm), ("raw", raw)):
        st, spd = step_speed(tr)
        out[lab] = {"n": int(len(tr)), "step_p50": _pct(st, 50), "step_p95": _pct(st, 95),
                    "step_max": _pct(st, 100), "speed_p50": _pct(spd, 50), "speed_p95": _pct(spd, 95),
                    "speed_p99": _pct(spd, 99), "frac_speed_gt_0p5": float(np.mean(spd > 0.5)) if len(spd) else None}
        poly = outline_local(outline, run.origin) if (outline is not None and run.origin is not None) else None
        if poly is not None and len(tr):
            d = inside_dist(tr[:, 1], tr[:, 2], poly)
            out[lab]["frac_outside_1m"] = float(np.mean(d < -1.0))
            out[lab]["outside_p95_m"] = _pct(np.maximum(-d, 0), 95)
    if result["estimates"]:
        st = [e["status"] for e in result["estimates"]]
        out["status_frac"] = {s: float(st.count(s) / len(st)) for s in set(st)}
    return out


def top_downweighted(result, n=10):
    fx = [f for f in result["fixes"] if f["decision"] in ("downweighted", "set_aside") and f.get("nis") is not None]
    return sorted(fx, key=lambda f: -f["nis"])[:n]


# ---------------------------------------------------------------------------
# outputs
# ---------------------------------------------------------------------------

def write_outputs(result, run, out_dir, outline=None, figures=True):
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "fixes.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(plog.COLUMNS)
        for f in result["fixes"]:
            w.writerow(plog.row_from_fix(f))
    with open(os.path.join(out_dir, "estimate.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "status", "x", "y", "depth", "vx", "vy", "cov_xx", "cov_xy", "cov_yy", "r95",
                    "fix_age", "fix_rate"])
        for e in result["estimates"]:
            c = e["cov"] or [None, None, None]
            w.writerow([e["t"], e["status"], e["x"], e["y"], e["depth"], e["vx"], e["vy"], c[0], c[1], c[2],
                        e["r95"], e["fix_age"], e["fix_rate"]])
    write_smoothed(os.path.join(out_dir, "smoothed.csv"), result["smoothed"])
    m = metrics(result, run, outline)
    m["top_downweighted"] = [{k: f[k] for k in ("t_rx", "decision", "nis", "reason", "world")}
                             for f in top_downweighted(result)]
    with open(os.path.join(out_dir, "metrics.json"), "w") as fh:
        json.dump(m, fh, indent=1)
    if figures:
        make_figures(result, run, out_dir, outline)
    return m


def write_smoothed(path, smoothed):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_meas", "segment", "x", "y", "vx", "vy", "cov_xx", "cov_xy", "cov_yy"])
        for s in smoothed:
            w.writerow([repr(s["t"]), s["segment"], repr(s["x"]), repr(s["y"]), repr(s["vx"]), repr(s["vy"]),
                        repr(s["cov"][0]), repr(s["cov"][1]), repr(s["cov"][2])])


def make_figures(result, run, out_dir, outline=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    est, leg, sm, raw = track_arrays(result)
    poses = np.array(result["poses"]) if result["poses"] else np.zeros((0, 4))
    fx = result["fixes"]
    colours = {"accepted": "tab:green", "downweighted": "tab:orange", "set_aside": "tab:red",
               "reinit": "tab:purple", "initialising": "tab:grey"}

    fig, ax = plt.subplots(figsize=(9, 8))
    poly = outline_local(outline, run.origin) if (outline is not None and run.origin is not None) else None
    if poly is not None:
        ax.plot([p[0] for p in poly + poly[:1]], [p[1] for p in poly + poly[:1]], "-", color="0.6", lw=1.5,
                label="outline")
    if len(poses):
        ax.plot(poses[:, 1], poses[:, 2], "-", color="k", lw=0.6, label="boat")
    for d, c in colours.items():
        pts = np.array([f["world"] for f in fx if f["decision"] == d and f.get("world") is not None])
        if len(pts):
            ax.scatter(pts[:, 0], pts[:, 1], s=6, color=c, alpha=0.5, label=f"fix: {d}")
    if len(leg):
        ax.plot(leg[:, 1], leg[:, 2], "-", color="tab:orange", lw=0.6, alpha=0.6, label="legacy")
    if len(est):
        ax.plot(est[:, 1], est[:, 2], "-", color="tab:blue", lw=1.5, label="filtered")
    if len(sm):
        ax.plot(sm[:, 1], sm[:, 2], "-", color="tab:cyan", lw=1.2, label="smoothed (RTS)")
    for i, f in enumerate(top_downweighted(result)):
        if f.get("world"):
            ax.annotate(str(i + 1), f["world"], fontsize=8, color="darkred")
    # frame the filtered/smoothed estimate and the boat (legacy and raw outliers
    # can be 100+ m off and would hide everything else)
    parts = [a[:, 1:3] for a in (est, sm) if len(a)] + ([poses[:, 1:3]] if len(poses) else [])
    if parts:
        allp = np.vstack(parts)
        lo, hi = np.percentile(allp, 0.5, axis=0) - 8, np.percentile(allp, 99.5, axis=0) + 8
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title(f"{run.name}: legacy vs filtered vs smoothed (numbers: 10 most down-weighted fixes)", fontsize=9)
    ax.legend(fontsize=7, loc="best")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "tracks.png"), dpi=90)
    plt.close(fig)

    fig, axs = plt.subplots(1, 2, figsize=(12, 4))
    for lab, tr, c in (("legacy", leg, "tab:orange"), ("filtered", est, "tab:blue"), ("smoothed", sm, "tab:cyan")):
        st, spd = step_speed(tr)
        if len(st):
            axs[0].hist(np.clip(st, 0, 10), bins=50, histtype="step", color=c, label=lab, density=True)
        if len(spd):
            axs[1].hist(np.clip(spd, 0, 10), bins=50, histtype="step", color=c, label=lab, density=True)
    axs[0].set_xlabel("step between outputs (m, clipped 10)")
    axs[1].set_xlabel("implied speed (m/s, clipped 10)")
    for a in axs:
        a.set_yscale("log")
        a.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "steps_speeds.png"), dpi=90)
    plt.close(fig)

    fig, axs = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    t0 = fx[0]["t_rx"] if fx else 0.0
    nis = [(f["t_rx"] - t0, f["nis"], f["decision"]) for f in fx if f.get("nis") is not None]
    for d, c in colours.items():
        pts = [(t, n) for t, n, dd in nis if dd == d]
        if pts:
            a = np.array(pts)
            axs[0].scatter(a[:, 0], a[:, 1], s=5, color=c, label=d)
    axs[0].axhline(5.99, color="k", lw=0.8, ls="--")
    axs[0].set_yscale("log")
    axs[0].set_ylabel("NIS")
    axs[0].legend(fontsize=7)
    names = ["accepted", "downweighted", "set_aside", "reinit", "initialising", "invalid", "duplicate"]
    for f in fx:
        axs[1].plot(f["t_rx"] - t0, names.index(f["decision"]), "|", color=colours.get(f["decision"], "k"))
    axs[1].set_yticks(range(len(names)), names)
    axs[1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "nis_decisions.png"), dpi=90)
    plt.close(fig)


# ---------------------------------------------------------------------------
# grid search
# ---------------------------------------------------------------------------

def parse_grid(specs):
    """['latency_s=0:3:0.25', 'y_sign=-1,1'] -> {name: [values]}"""
    out = {}
    for s in specs:
        name, _, rng = s.partition("=")
        if ":" in rng:
            a, b, step = (float(v) for v in rng.split(":"))
            out[name] = [round(v, 6) for v in np.arange(a, b + step / 2, step)]
        else:
            out[name] = [float(v) for v in rng.split(",")]
    return out


def _odom_and_fixes(run, source):
    """The run's odometry (t, x, y, yaw, None) and its usable, non-duplicate fixes
    (t, None, None, None, Fix) of input `source`, in event order. They do not
    depend on the geometry, so a grid search parses them once per run."""
    cache = run.__dict__.setdefault("_odom_and_fixes", {})
    if source in cache:
        return cache[source]
    out = []
    json_seen = any(k == "fixjson" for _, k, _ in run.events)
    last = None
    for t, kind, v in run.events:
        if kind == "odom":
            out.append((t, v[0], v[1], v[2], None))
        elif kind in ("packet", "fixjson", "fixpose"):
            if kind == "fixpose":
                f, _pose = v
            elif kind == "packet":
                if json_seen:
                    continue
                f = pin.fix_from_packet(t, v, source)
            else:
                msg = pin.parse_fix_message(v)
                f = pin.fixes_from_json(t, msg)[source] if msg else None
            if f is None or f.invalid or f.key == last:
                continue
            last = f.key
            out.append((t, None, None, None, f))
    cache[source] = out
    return out


def world_fixes(run, values):
    """Raw world fixes (t_meas, x, y) of a run with the given geometry, no filter."""
    from pinger_filter import PingerFilter, PoseBuffer
    pf = PingerFilter(values)
    poses = PoseBuffer(1e9)
    pts = []
    for t, x, y, yaw, f in _odom_and_fixes(run, values["source"]):
        if f is None:
            poses.add(t, x, y, yaw)
        else:
            bx, by, _d, rho = pf.map_fix(f.a)
            if rho < values["min_range_m"] or rho > values["max_range_m"]:
                continue
            pose = poses.at(f.t_rx - values["latency_s"], values["max_pose_extrap_s"])
            if pose is None:
                continue
            x, y, yaw, _ = pose
            c, s = math.cos(yaw), math.sin(yaw)
            pts.append((f.t_rx, x + c * bx - s * by, y + s * bx + c * by))
    return np.array(pts) if pts else np.zeros((0, 3))


def local_scatter(pts, half=5.0):
    if len(pts) < 5:
        return float("nan")
    t, x, y = pts[:, 0], pts[:, 1], pts[:, 2]
    out = []
    for i in range(len(t)):
        k = np.abs(t - t[i]) <= half
        k[i] = False
        if k.sum() >= 4:
            out.append(math.hypot(x[i] - np.median(x[k]), y[i] - np.median(y[k])))
    return float(np.median(out)) if out else float("nan")


def grid_search(runs, base, grid, outline=None):
    import itertools
    names = list(grid)
    rows = []
    # The outline does not depend on the geometry: once per run.
    polys = [outline_local(outline, run.origin) if (outline is not None and run.origin is not None)
             else None for run in runs]
    for combo in itertools.product(*[grid[n] for n in names]):
        vals = dict(base.values)
        for n, v in zip(names, combo):
            vals[n] = type(base.values[n])(v)
        if vals.get("y_sign") == 0:
            continue
        scat, outside = [], []
        for run, poly in zip(runs, polys):
            pts = world_fixes(run, vals)
            scat.append(local_scatter(pts))
            if poly is not None and len(pts):
                d = inside_dist(pts[:, 1], pts[:, 2], poly)
                outside.append(float(np.mean(d < -1.0)))
        rows.append(dict(zip(names, combo), local_scatter_median=float(np.nanmedian(scat)),
                         local_scatter_per_run=scat,
                         outside_mean=float(np.mean(outside)) if outside else None))
    return rows


# ---------------------------------------------------------------------------
# semi-synthetic known-truth evaluation (A2 without a still pinger)
# ---------------------------------------------------------------------------

def residual_pool(runs, values):
    """Real residuals in the line-of-sight frame: (range, along, cross) of each
    raw fix against the median of its +/-5 s neighbours."""
    out = []
    for run in runs:
        pts = world_fixes(run, values)
        if len(pts) < 10:
            continue
        poses = np.array([(t, v[0], v[1]) for t, k, v in run.events if k == "odom"])
        for i in range(len(pts)):
            k = np.abs(pts[:, 0] - pts[i, 0]) <= 5.0
            k[i] = False
            if k.sum() < 4:
                continue
            mx, my = np.median(pts[k, 1]), np.median(pts[k, 2])
            bx = np.interp(pts[i, 0], poses[:, 0], poses[:, 1])
            by = np.interp(pts[i, 0], poses[:, 0], poses[:, 2])
            lx, ly = mx - bx, my - by
            rho = math.hypot(lx, ly)
            if rho < 0.3:
                continue
            ux, uy = lx / rho, ly / rho
            ex, ey = pts[i, 1] - mx, pts[i, 2] - my
            out.append((rho, ex * ux + ey * uy, -ex * uy + ey * ux))
    return np.array(out)


def synthesize(run, truth_fn, residuals, tau, rng, extra_lag_filtered=1.0, filtered_scale=0.5):
    """Replace every acoustic value of a bag run by a synthetic one from the
    truth trajectory, keeping the real event times, duplicates and zeros.
    Raw fixes: truth seen from the pose at t_rx - tau plus a real residual of a
    similar range. 'WL filtered' (legacy's input): a model -- truth at
    t_rx - tau - extra_lag_filtered with the residual scaled by filtered_scale."""
    poses = np.array([(t, v[0], v[1], v[2]) for t, k, v in run.events if k == "odom"])
    yaw = np.unwrap(poses[:, 3])

    def pose(t):
        return (np.interp(t, poses[:, 0], poses[:, 1]), np.interp(t, poses[:, 0], poses[:, 2]),
                np.interp(t, poses[:, 0], yaw))

    def body(t, lag, scale):
        x, y, ps = pose(t - lag)
        tx, ty = truth_fn(t - lag)
        lx, ly = tx - x, ty - y
        rho = math.hypot(lx, ly)
        ux, uy = (lx / rho, ly / rho) if rho > 1e-6 else (1.0, 0.0)
        near = np.nonzero(np.abs(residuals[:, 0] - rho) <= max(1.0, 0.2 * rho))[0]
        if len(near) == 0:
            near = np.arange(len(residuals))
        _, ea, ec = residuals[rng.choice(near)]
        wx = lx + scale * (ea * ux - ec * uy)
        wy = ly + scale * (ea * uy + ec * ux)
        c, s = math.cos(ps), math.sin(ps)
        return [c * wx + s * wy, -s * wx + c * wy, 0.19]

    ev = []
    last_raw = last_fil = None
    prev_raw_key = prev_fil_key = None
    for t, kind, v in run.events:
        if kind != "packet":
            if kind != "fixjson":
                ev.append((t, kind, v))
            continue
        pk = list(v)
        raw_key, fil_key = tuple(v[7:10]), tuple(v[16:19])
        if all(a == 0.0 for a in raw_key):
            ev.append((t, kind, pk))
            continue
        if raw_key != prev_raw_key or last_raw is None:
            last_raw = body(t, tau, 1.0)
        if fil_key != prev_fil_key or last_fil is None:
            last_fil = body(t, tau + extra_lag_filtered, filtered_scale)
        prev_raw_key, prev_fil_key = raw_key, fil_key
        pk[7:10] = last_raw
        pk[16:19] = last_fil
        ev.append((t, kind, pk))
    syn = Run(run.name + "-synthetic", "bag", ev, run.origin, run.path)
    return syn


def truth_metrics(result, truth_fn, run):
    """Errors of every output against the known truth, at odom times."""
    poses = np.array(result["poses"])
    t = poses[:, 0]
    tx, ty = np.array([truth_fn(v) for v in t]).T
    out = {}
    est, leg, sm, raw = track_arrays(result)

    def held(tr):          # the value shown at each odom time (zero-order hold)
        if len(tr) == 0:
            return None
        k = np.searchsorted(tr[:, 0], t, side="right") - 1
        ok = k >= 0
        return ok, tr[np.maximum(k, 0), 1], tr[np.maximum(k, 0), 2]

    def interp(tr):
        if len(tr) < 2:
            return None
        ok = (t >= tr[0, 0]) & (t <= tr[-1, 0])
        return ok, np.interp(t, tr[:, 0], tr[:, 1]), np.interp(t, tr[:, 0], tr[:, 2])

    h = 0.5
    vx = (np.interp(t + h, t, poses[:, 1]) - np.interp(t - h, t, poses[:, 1])) / (2 * h)
    vy = (np.interp(t + h, t, poses[:, 2]) - np.interp(t - h, t, poses[:, 2])) / (2 * h)
    sp = np.hypot(vx, vy)
    est_odom = held(est)
    for lab, series in (("legacy", (lambda: (np.isin(t, leg[:, 0]),
                                             np.interp(t, leg[:, 0], leg[:, 1]),
                                             np.interp(t, leg[:, 0], leg[:, 2])) if len(leg) else None)),
                        ("filtered", lambda: est_odom),
                        ("smoothed", lambda: interp(sm))):
        s = series()
        if s is None:
            continue
        ok, x, y = s
        ok = ok & (t > t[0] + 20.0)
        if not np.any(ok):
            continue
        e = np.hypot(x[ok] - tx[ok], y[ok] - ty[ok])
        mv = sp[ok] > 0.15
        along = ((x[ok] - tx[ok]) * vx[ok] + (y[ok] - ty[ok]) * vy[ok]) / np.maximum(sp[ok], 1e-6)
        out[lab] = {"cep50": float(np.median(e)), "cep95": float(np.percentile(e, 95)),
                    "rms": float(np.sqrt(np.mean(e ** 2))),
                    "along_travel_p50": float(np.median(along[mv])) if np.any(mv) else None}
    # raw fixes vs truth at their measurement time
    if len(raw):
        rx, ry = np.array([truth_fn(v) for v in raw[:, 0]]).T
        e = np.hypot(raw[:, 1] - rx, raw[:, 2] - ry)
        out["raw"] = {"cep50": float(np.median(e)), "cep95": float(np.percentile(e, 95)),
                      "rms": float(np.sqrt(np.mean(e ** 2)))}
    # drag: displacement of the shown pinger between consecutive new fixes vs the boat's
    new_t = sorted({f["t_rx"] for f in result["fixes"] if f["decision"] not in ("duplicate", "invalid")})
    for lab, s in (("legacy", None), ("filtered", est_odom)):
        if lab == "legacy":
            if not len(leg):
                continue
            sx, sy, st = leg[:, 1], leg[:, 2], leg[:, 0]
        else:
            if s is None:
                continue
            ok, sx, sy = s
            st = t
        DL, DP = [], []
        for a, b in zip(new_t[:-1], new_t[1:]):
            k = np.nonzero((st > a + 0.02) & (st < b - 0.02))[0]
            if len(k) < 2:
                continue
            DL.append((sx[k[-1]] - sx[k[0]], sy[k[-1]] - sy[k[0]]))
            bk = np.searchsorted(t, [st[k[0]], st[k[-1]]])
            bk = np.clip(bk, 0, len(t) - 1)
            DP.append((poses[bk[1], 1] - poses[bk[0], 1], poses[bk[1], 2] - poses[bk[0], 2]))
        if DL:
            DL, DP = np.array(DL), np.array(DP)
            den = float(np.sum(DP * DP))
            out.setdefault(lab, {})["drag_slope_between_fixes"] = float(np.sum(DL * DP) / den) if den > 0 else None
            out[lab]["max_move_between_fixes_m"] = float(np.max(np.hypot(DL[:, 0], DL[:, 1])))
    return out


def find_inputs(root):
    bags = [b for b in sorted(glob.glob(os.path.join(root, "Robot_data", "**", "*-bag"), recursive=True))]
    exports = sorted(glob.glob(os.path.join(root, "Processed_Robot_data", "pinger*", "*.csv")))
    return bags, exports


def evaluate(root, params, out_dir, outline=None, seed=0, figures=False):
    """A2 over every input under a data root."""
    bags, exports = find_inputs(root)
    runs_bag = []
    for b in bags:
        r = load_bag(b)
        if any(k == "packet" and any(v[7:10]) for _, k, v in r.events):
            runs_bag.append(r)
    runs_csv = [load_csv(c) for c in exports]
    real = []
    for r in runs_bag + runs_csv:
        res = replay(r, params)
        m = write_outputs(res, r, os.path.join(out_dir, "real", r.name), outline, figures=figures)
        real.append(m)
    residuals = residual_pool(runs_bag, params.values)
    rng = np.random.default_rng(seed)
    syn = []
    for r in runs_bag:
        odom = np.array([(t, v[0], v[1]) for t, k, v in r.events if k == "odom"])
        p0 = (float(np.median(odom[:, 1])) + 6.0, float(np.median(odom[:, 2])) + 4.0)
        for kind in ("still", "crawl"):
            if kind == "still":
                def truth(t, p0=p0):
                    return p0
            else:
                t0 = odom[0, 0]

                def truth(t, p0=p0, t0=t0):
                    # back and forth over 15 m at 0.2 m/s (a pool-cleaner crawl)
                    s = (0.2 * (t - t0)) % 30.0
                    s = s if s <= 15.0 else 30.0 - s
                    return (p0[0], p0[1] - 7.5 + s)
            srun = synthesize(r, truth, residuals, params.values["latency_s"], rng)
            res = replay(srun, params)
            tm = truth_metrics(res, truth, srun)
            tm.update(run=r.name, truth=kind)
            syn.append(tm)
    report = {"params": params.as_dict(), "real": real, "semi_synthetic": syn,
              "residual_pool_size": int(len(residuals))}
    with open(os.path.join(out_dir, "a2_report.json"), "w") as fh:
        json.dump(report, fh, indent=1)
    return report


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _params_from_args(args):
    overrides = pp.parse_overrides(args.set)
    return pp.load_params(preset=args.preset, overrides=overrides, yaml_path=args.params)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("run", "grid", "evaluate"):
        s = sub.add_parser(name)
        s.add_argument("inputs", nargs="+")
        s.add_argument("--preset", default=pp.DEFAULT_PRESET)
        s.add_argument("--params", help="parameter YAML (MCS format or flat)")
        s.add_argument("--set", nargs="*", default=[], help="name=value overrides")
        s.add_argument("--outline", help="YAML with corners: [[lat, lon], ...] (weak ground truth)")
        s.add_argument("--out", default="pinger_filter_replay_out")
        if name == "run":
            s.add_argument("--source", choices=("filter", "legacy"), default="filter")
            s.add_argument("--no-figures", action="store_true")
        if name == "grid":
            s.add_argument("--grid", nargs="+", required=True, help="name=a:b:step or name=v1,v2")
        if name == "evaluate":
            s.add_argument("--seed", type=int, default=0)
            s.add_argument("--figures", action="store_true")
    args = ap.parse_args(argv)
    params = _params_from_args(args)
    if args.cmd == "run":
        for path in args.inputs:
            run = load(path)
            res = replay(run, params, args.source)
            out = args.out if len(args.inputs) == 1 else os.path.join(args.out, run.name)
            m = write_outputs(res, run, out, args.outline, figures=not args.no_figures)
            print(json.dumps({k: m[k] for k in ("run", "decisions", "nis_mean_used", "filtered", "legacy")},
                             indent=1))
    elif args.cmd == "grid":
        runs = [load(p) for p in args.inputs]
        rows = grid_search(runs, params, parse_grid(args.grid), args.outline)
        os.makedirs(args.out, exist_ok=True)
        with open(os.path.join(args.out, "grid.json"), "w") as fh:
            json.dump(rows, fh, indent=1)
        best = min(rows, key=lambda r: r["local_scatter_median"])
        print(json.dumps({k: v for k, v in best.items() if k != "local_scatter_per_run"}, indent=1))
    else:
        rep = evaluate(args.inputs[0], params, args.out, args.outline, args.seed, args.figures)
        print(f"{len(rep['real'])} real runs, {len(rep['semi_synthetic'])} semi-synthetic replays -> "
              f"{os.path.join(args.out, 'a2_report.json')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
