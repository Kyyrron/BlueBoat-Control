#!/usr/bin/env python3
r"""
Phase 0 evidence for docs/PINGER_FILTER_FINDINGS.md: every number and figure of
that document is recomputed here from the recorded field data, read-only.

    /usr/bin/python3 docs/img/pinger_findings/make_figures.py [--data ~/ros2_ws/data]

Writes the PNG figures and `phase0_metrics.json` next to this script. Nothing
under the data root is modified (bags are memory-mapped read-only).

Inputs
  * every run bag under <data>/Robot_data/** that carries /uw_gps_data with a
    non-zero acoustic fix (2026-10-27 pool 10:19-10:44, 2026-10-28 pool);
  * the pinger exports under <data>/Processed_Robot_data/pinger*/ (the only
    surviving copies of the 2026-10-27 pool-afternoon and harbour runs).

Conventions (see the findings document)
  * time = bag RECEIVE time: header stamps of /mavros/local_position/odom and
    /blueboat/odom read 0 on the real launch;
  * pose = /blueboat/odom (local ENU about the launch point, yaw absolute ENU);
  * acoustic x/y as received, rotated body->world with the pose ("as-is").
"""

import argparse
import csv
import glob
import json
import os
import struct
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "log_reviewer"))
sys.path.insert(0, os.path.join(REPO, "blueboat_control", "src", "_custom_libraries"))
from reviewer import pinger_bag as pbg                                # noqa: E402

# Pool outline, from the side-scan sonar wall fit of the GPS accuracy study
# (Desktop/randomData/report_Kagoshima/gps_accuracy_study, table "corners").
POOL = [(31.26388154, 130.29900505), (31.26405645, 130.29897899),
        (31.26399972, 130.29845774), (31.26382480, 130.29848379)]
EARTH_R = 6378137.0


# ---------------------------------------------------------------------------
# reading
# ---------------------------------------------------------------------------

def _cdr_skip_string(p, pos):
    n = struct.unpack_from("<I", p, pos)[0]
    return pos + 4 + n


def _align(pos, a):
    r = (pos - 4) % a
    return pos if r == 0 else pos + a - r


def dec_odom(p):
    pos = _cdr_skip_string(p, 12)            # header stamp (8) + frame_id
    pos = _align(_cdr_skip_string(p, pos), 8)  # child_frame_id
    pose = struct.unpack_from("<7d", p, pos)
    pos += 56 + 36 * 8
    return pose, struct.unpack_from("<6d", p, pos)


def dec_imu(p):
    pos = _align(_cdr_skip_string(p, 12), 8)
    pos += 32 + 72
    return struct.unpack_from("<3d", p, pos)


def dec_string(p):
    n = struct.unpack_from("<I", p, 4)[0]
    return bytes(p[8:8 + n - 1]).decode("utf-8", "replace")


def yaw_of(q):
    x, y, z, w = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def load_bag(bag):
    topics = ["/uw_gps_data", "/blueboat/odom", "/blueboat/pinger_coordinates",
              "/mavros/imu/data", "/mavros/local_position/odom", "/blueboat/run_state"]
    out = {t: [] for t in topics}
    for f in pbg._mcap_files(bag):
        part, _ = pbg.read_topics(f, topics)
        for t in topics:
            out[t].extend(part[t])
    for t in topics:
        out[t].sort(key=lambda r: r[0])
    r = {"name": os.path.basename(bag)[:-4] if bag.endswith("-bag") else os.path.basename(bag)}
    r["ut"] = np.array([a for a, _ in out["/uw_gps_data"]]) * 1e-9
    r["U"] = np.array([pbg.decode_float32_multiarray(p) for _, p in out["/uw_gps_data"]]).reshape(-1, 19)
    r["ot"] = np.array([a for a, _ in out["/blueboat/odom"]]) * 1e-9
    od = [dec_odom(p) for _, p in out["/blueboat/odom"]]
    r["P"] = np.array([[v[0], v[1], yaw_of(v[3:7])] for v, _ in od])
    r["bt"] = np.array([a for a, _ in out["/blueboat/pinger_coordinates"]]) * 1e-9
    r["B"] = np.array([pbg.decode_float32_multiarray(p)[:3] for _, p in out["/blueboat/pinger_coordinates"]])
    r["it"] = np.array([a for a, _ in out["/mavros/imu/data"]]) * 1e-9
    r["iw"] = np.array([dec_imu(p) for _, p in out["/mavros/imu/data"]])
    r["mt"] = np.array([a for a, _ in out["/mavros/local_position/odom"]]) * 1e-9
    mo = [dec_odom(p) for _, p in out["/mavros/local_position/odom"]]
    r["mtw"] = np.array([tw for _, tw in mo])
    r["stamp_zero"] = float(np.mean([struct.unpack_from("<iI", p, 4) == (0, 0)
                                     for _, p in out["/mavros/local_position/odom"]]))
    states = [json.loads(dec_string(p)) for _, p in out["/blueboat/run_state"]]
    r["use_pinger"] = sorted({bool(s.get("use_pinger")) for s in states})
    return r


def load_export(folder):
    path = glob.glob(os.path.join(folder, "*.csv"))[0]
    rows = list(csv.reader(open(path)))
    hdr = rows[2]
    data = np.array([[float(v) for v in row] for row in rows[3:] if len(row) == len(hdr)])
    d = {h: data[:, i] for i, h in enumerate(hdr)}
    t = np.array([time.mktime((int(a), int(b), int(c), int(e), int(f), int(g), 0, 0, -1))
                  for a, b, c, e, f, g in zip(d["Year"], d["Month"], d["Day"], d["Hour"],
                                              d["Minute"], d["Second"])]) + d["MicroSecond"] * 1e-6
    d["t"] = t - t[0]
    d["name"] = os.path.basename(folder)
    return d


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def new_mask(A):
    zero = np.all(A == 0, axis=1)
    return np.r_[True, np.any(np.diff(A, axis=0) != 0, axis=1)] & ~zero, zero


def new_fixes(r, cols):
    m, _ = new_mask(r["U"][:, cols])
    t, A = r["ut"][m], r["U"][m][:, cols]
    ok = (t > r["ot"][0] + 3) & (t < r["ot"][-1])
    return t[ok], A[ok]


def pose_at(r, t):
    yaw = np.unwrap(r["P"][:, 2])
    return (np.interp(t, r["ot"], r["P"][:, 0]), np.interp(t, r["ot"], r["P"][:, 1]),
            np.interp(t, r["ot"], yaw))


def world(r, t, A, tau=0.0, ysign=1, dyaw=0.0, rotate=True):
    x, y, ps = pose_at(r, t - tau)
    if not rotate:
        return x + A[:, 0], y + A[:, 1]
    c, s = np.cos(ps + dyaw), np.sin(ps + dyaw)
    bx, by = A[:, 0], ysign * A[:, 1]
    return x + c * bx - s * by, y + s * bx + c * by


def local_resid(t, wx, wy, half=5.0):
    out = np.full(len(t), np.nan)
    for i in range(len(t)):
        k = np.abs(t - t[i]) <= half
        k[i] = False
        if k.sum() >= 4:
            out[i] = np.hypot(wx[i] - np.median(wx[k]), wy[i] - np.median(wy[k]))
    return out


def cep_whole(wx, wy):
    return float(np.median(np.hypot(wx - np.median(wx), wy - np.median(wy))))


def origin_of(bagdir):
    for f in glob.glob(os.path.join(os.path.dirname(bagdir), "*-origin.yaml")):
        vals = {}
        for line in open(f):
            k, _, v = line.partition(":")
            vals[k.strip()] = v.strip()
        return float(vals["latitude"]), float(vals["longitude"])
    return None


def pool_poly(lat0, lon0):
    return [(np.radians(lo - lon0) * EARTH_R * np.cos(np.radians(lat0)),
             np.radians(la - lat0) * EARTH_R) for la, lo in POOL]


def inside_dist(px, py, poly):
    """Signed distance to the convex outline, > 0 inside."""
    area = sum(poly[i][0] * poly[(i + 1) % 4][1] - poly[(i + 1) % 4][0] * poly[i][1] for i in range(4))
    sg = 1 if area > 0 else -1
    d = np.full(len(px), np.inf)
    for i in range(4):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % 4]
        ex, ey = x2 - x1, y2 - y1
        d = np.minimum(d, sg * (ex * (py - y1) - ey * (px - x1)) / np.hypot(ex, ey))
    return d


def legacy_replay(r, packets=None):
    """Today's robot_interface pinger algorithm on the recorded event order
    (odom / imu / uw_gps by receive time). `packets` overrides the filaco
    vector of each /uw_gps_data packet. Returns (t, body) at each odom."""
    ev = sorted([(t, 0, i) for i, t in enumerate(r["mt"])] +
                [(t, 1, i) for i, t in enumerate(r["it"])] +
                [(t, 2, i) for i, t in enumerate(r["ut"])])
    p = np.zeros(3)
    av = None
    prev = None
    out, tt = [], []
    for t, k, i in ev:
        if k == 1:
            av = r["iw"][i]
        elif k == 2:
            p = np.array(packets[i] if packets is not None else r["U"][i, 16:19], dtype=float)
        else:
            if prev is None:
                prev = t
                continue
            dt = t - prev
            prev = t
            vel = np.array(r["mtw"][i][:3])
            if av is not None and not np.all(p == np.zeros(3)):
                p -= (vel + np.cross(np.array([0.0, 0.0, av[2]]), p)) * dt
            out.append(p.copy())
            tt.append(t)
    return np.array(tt), np.array(out)


# ---------------------------------------------------------------------------
# analyses
# ---------------------------------------------------------------------------

def analyse_bag(r, bagdir):
    m = {"run": r["name"], "use_pinger_in_run_state": r["use_pinger"]}
    dur = r["ut"][-1] - r["ut"][0]
    raw_new, raw_zero = new_mask(r["U"][:, 7:10])
    fil_new, _ = new_mask(r["U"][:, 16:19])
    tn = r["ut"][raw_new]
    m.update(duration_s=float(dur), polls=int(len(r["ut"])), poll_hz=float(len(r["ut"]) / dur),
             new_raw=int(raw_new.sum()), new_raw_hz=float(raw_new.sum() / dur),
             new_fil=int(fil_new.sum()), dup_ratio=float(1 - raw_new.sum() / max(1, (~raw_zero).sum())),
             zero_packets=int(raw_zero.sum()),
             gap_p50=float(np.median(np.diff(tn))), gap_max=float(np.max(np.diff(tn))))
    ts = np.array([time.mktime((int(a[0]), int(a[1]), int(a[2]), int(a[3]), int(a[4]), int(a[5]), 0, 0, -1))
                   + a[6] * 1e-6 for a in r["U"]])
    delay = r["ut"] - ts
    m.update(http_p50=float(np.median(delay)), http_p95=float(np.percentile(delay, 95)),
             http_max=float(delay.max()))
    m["odom_hz"] = float(len(r["ot"]) / (r["ot"][-1] - r["ot"][0]))
    m["odom_stamp_zero_frac"] = r["stamp_zero"]
    spd = np.hypot(r["mtw"][:, 0], r["mtw"][:, 1])
    m.update(twist_moving_frac=float(np.mean(spd > 0.05)), twist_max=float(spd.max()))
    t, A = new_fixes(r, slice(7, 10))
    rng = np.hypot(A[:, 0], A[:, 1])
    m["range_p50"] = float(np.median(rng))
    # frames: whole-run CEP and local residual, latency grid
    cand = {"as_is": dict(ysign=1), "y_flip": dict(ysign=-1), "no_rotation": dict(rotate=False)}
    for k, kw in cand.items():
        m["cep_" + k] = min(cep_whole(*world(r, t, A, tau, **kw)) for tau in np.arange(0, 3.01, 0.25))
    taus = np.arange(0, 3.01, 0.25)
    for lab, cols in (("raw", slice(7, 10)), ("fil", slice(16, 19))):
        tt, AA = new_fixes(r, cols)
        k = np.hypot(AA[:, 0], AA[:, 1]) < 30
        curve = [float(np.nanmedian(local_resid(tt[k], *world(r, tt[k], AA[k], tau)))) for tau in taus]
        m["latency_curve_" + lab] = curve
    yaws = np.radians(np.arange(-30, 31, 2))
    loc = [np.nanmedian(local_resid(t, *world(r, t, A, 1.5, dyaw=d))) for d in yaws]
    m["mount_yaw_best_deg"] = float(np.degrees(yaws[int(np.argmin(loc))]))
    # pool outline
    o = origin_of(bagdir)
    if o is not None:
        poly = pool_poly(*o)
        m["boat_min_inside_m"] = float(inside_dist(r["P"][:, 0], r["P"][:, 1], poly).min())
        for lab, kw in (("as_is", dict(ysign=1)), ("y_flip", dict(ysign=-1))):
            d = inside_dist(*world(r, t, A, 1.5, **kw), poly)
            m["outside_frac_" + lab] = float(np.mean(d < -1.0))
        tf, Af = new_fixes(r, slice(16, 19))
        m["outside_frac_wl_filtered"] = float(np.mean(inside_dist(*world(r, tf, Af, 2.5), poly) < -1.0))
    # legacy reproduction
    tt, body = legacy_replay(r)
    n = min(len(body), len(r["B"]))
    best = None
    for off in range(-3, 4):
        a = body[max(0, off):][:n - 3]
        b = r["B"][max(0, -off):][:n - 3]
        q = min(len(a), len(b))
        d = np.abs(a[:q, :2] - b[:q, :2]).max(axis=1)
        if best is None or np.median(d) < np.median(best):
            best = d
    m.update(legacy_repro_median_m=float(np.median(best)), legacy_repro_p99_m=float(np.percentile(best, 99)),
             legacy_repro_max_m=float(best.max()))
    # legacy drag on real data: between consecutive new filtered fixes
    P = r["P"]
    nb = min(len(P), len(r["B"]))
    B = r["B"][:nb]
    bt = r["bt"][:nb]
    c, s = np.cos(P[:nb, 2]), np.sin(P[:nb, 2])
    L = np.c_[P[:nb, 0] + c * B[:, 0] - s * B[:, 1], P[:nb, 1] + s * B[:, 0] + c * B[:, 1]]
    det = ~np.all(B == 0, axis=1)
    tf = r["ut"][fil_new]
    DL, DP = [], []
    for a_, b_ in zip(tf[:-1], tf[1:]):
        k = np.nonzero((bt > a_ + 0.02) & (bt < b_ - 0.02) & det)[0]
        if len(k) >= 2:
            DL.append(L[k[-1]] - L[k[0]])
            DP.append(P[k[-1], :2] - P[k[0], :2])
    m["_drag_DL"], m["_drag_DP"] = np.array(DL), np.array(DP)
    # semi-synthetic still pinger through the legacy algorithm (mechanism)
    m.update(legacy_mechanism(r))
    # residuals for the noise-vs-range figure
    wx, wy = world(r, t, A, 1.5)
    res = local_resid(t, wx, wy)
    m["_noise"] = np.c_[rng, res]
    m["_tracks"] = (t, wx, wy)
    return m


def legacy_mechanism(r, tau=1.5):
    """Noise-free still pinger, the run's own motion, poll timing and duplicate
    pattern, through the legacy algorithm."""
    P0 = np.array([np.median(r["P"][:, 0]) + 6.0, np.median(r["P"][:, 1]) + 4.0])
    F = r["U"][:, 16:19]
    newf = np.r_[True, np.any(np.diff(F, axis=0) != 0, axis=1)]
    syn, last = [], None
    for i, t in enumerate(r["ut"]):
        if newf[i] or last is None:
            x, y, ps = pose_at(r, t - tau)
            dx, dy = P0[0] - x, P0[1] - y
            last = [np.cos(ps) * dx + np.sin(ps) * dy, -np.sin(ps) * dx + np.cos(ps) * dy, 0.2]
        syn.append(list(last))
    tt, body = legacy_replay(r, syn)
    x, y, ps = pose_at(r, tt)
    c, s = np.cos(ps), np.sin(ps)
    L = np.c_[x + c * body[:, 0] - s * body[:, 1], y + s * body[:, 0] + c * body[:, 1]]
    ok = tt > r["ut"][0] + 0.1
    err = L - P0
    h = 0.5
    vx = (np.interp(tt + h, r["ot"], r["P"][:, 0]) - np.interp(tt - h, r["ot"], r["P"][:, 0])) / (2 * h)
    vy = (np.interp(tt + h, r["ot"], r["P"][:, 1]) - np.interp(tt - h, r["ot"], r["P"][:, 1])) / (2 * h)
    sp = np.hypot(vx, vy)
    mv = ok & (sp > 0.15)
    along = (err[:, 0] * vx + err[:, 1] * vy) / np.maximum(sp, 1e-6)
    e = np.hypot(*err[ok].T)
    # duplicate re-apply jumps
    dj, db = [], []
    for i in range(1, len(r["ut"])):
        if newf[i]:
            continue
        k0 = np.searchsorted(tt, r["ut"][i])
        kp = np.searchsorted(tt, r["ut"][i - 1])
        if 1 <= k0 < len(tt) and kp < len(tt):
            dj.append(L[k0] - L[k0 - 1])
            db.append(np.array([x[k0] - x[kp], y[k0] - y[kp]]))
    dj, db = np.array(dj), np.array(db)
    return {"mech_err_p50": float(np.median(e)), "mech_err_p95": float(np.percentile(e, 95)),
            "mech_along_p50": float(np.median(along[mv])) if mv.any() else float("nan"),
            "mech_speed_p50": float(np.median(sp[mv])) if mv.any() else float("nan"),
            "mech_dup_slope": float(np.sum(dj * db) / np.sum(db * db)) if len(dj) > 3 else float("nan"),
            "_mech_series": (tt[ok] - tt[0], np.hypot(*err[ok].T), along[ok], sp[ok])}


def analyse_export(d):
    A = np.c_[d["aco_x"], d["aco_y"], d["aco_z"]]
    nm, z = new_mask(A)
    first = int(np.argmax(~z)) if np.any(~z) else len(z)
    tn = d["t"][nm]
    gaps = np.diff(tn)
    x, y, ps = d["relative_x"], d["relative_y"], np.unwrap(d["relative_psi"])
    k = nm & (np.hypot(A[:, 0], A[:, 1]) < 40)
    out = {"run": d["name"], "duration_s": float(d["t"][-1]), "new_raw": int(nm.sum()),
           "new_raw_hz": float(nm.sum() / d["t"][-1]), "zero_rows_after_first": int(z[first:].sum()),
           "gap_p50": float(np.median(gaps)), "gap_max": float(gaps.max()),
           "depth_distinct": int(len(np.unique(np.round(A[~z, 2], 2)))),
           "depth_median": float(np.median(A[~z, 2]))}
    for lab, sg in (("as_is", 1), ("y_flip", -1)):
        c, s = np.cos(ps[k]), np.sin(ps[k])
        wx = x[k] + c * A[k, 0] - s * sg * A[k, 1]
        wy = y[k] + s * A[k, 0] + c * sg * A[k, 1]
        out["local_cep_" + lab] = float(np.nanmedian(local_resid(d["t"][k], wx, wy)))
    # follow-mode evidence: thrust differential vs the bearing the controller saw
    cx, cy = d["corrected_pinger_x"] - x, d["corrected_pinger_y"] - y
    bx = np.cos(ps) * cx + np.sin(ps) * cy
    by = -np.sin(ps) * cx + np.cos(ps) * cy
    td = d["right_thr_in"] - d["left_thr_in"]
    live = ((np.abs(bx) + np.abs(by)) > 1e-6) & (np.abs(td) > 0.5)
    out["corr_turn_vs_bearing"] = (float(np.corrcoef(td[live], np.arctan2(by, bx)[live])[0, 1])
                                   if live.sum() > 20 else float("nan"))
    return out


# ---------------------------------------------------------------------------
# figures
# ---------------------------------------------------------------------------

def figures(bags, metrics, exports):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = [m["run"] for m in metrics]
    short = [n[5:].replace("-poslog", "") for n in names]

    # 1. world tracks
    n = len(metrics)
    cols = 5
    fig, axs = plt.subplots((n + cols - 1) // cols, cols, figsize=(25, 5 * ((n + cols - 1) // cols)))
    for ax, r, m in zip(axs.flat, bags, metrics):
        t, wx, wy = m["_tracks"]
        o = origin_of(r["_dir"])
        if o is not None:
            poly = pool_poly(*o)
            ax.plot([p[0] for p in poly + poly[:1]], [p[1] for p in poly + poly[:1]], "-",
                    color="0.6", lw=1.5, label="pool outline")
        ax.plot(r["P"][:, 0], r["P"][:, 1], "k-", lw=0.7, label="boat")
        ax.scatter(wx, wy, c=t - t[0], s=5, cmap="viridis")
        ax.set_title(m["run"][5:], fontsize=9)
        ax.set_aspect("equal")
        ax.set_xlim(-55, 20)
        ax.set_ylim(-25, 25)
    for ax in list(axs.flat)[n:]:
        ax.axis("off")
    fig.suptitle("Raw UGPS fixes in the world frame (as-is mapping, latency 1.5 s), colour = time; grey = pool")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig1_world_fixes.png"), dpi=60)
    plt.close(fig)

    # 2. latency curves
    taus = np.arange(0, 3.01, 0.25)
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
    for ax, lab, title in ((axs[0], "raw", "raw acoustic fix"), (axs[1], "fil", "WL filtered fix")):
        tot = np.zeros(len(taus))
        for m in metrics:
            c = np.array(m["latency_curve_" + lab])
            ax.plot(taus, c / np.nanmin(c), color="0.75", lw=0.8)
            tot += c / np.nanmin(c)
        ax.plot(taus, tot / len(metrics), "k-", lw=2.5, label="pooled")
        ax.set_xlabel("latency assumed (s)")
        ax.set_ylabel("local scatter / run minimum")
        ax.set_title(title)
        ax.legend()
    fig.suptitle("Latency: local consistency of world fixes (median distance to the +/-5 s median)")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig2_latency.png"), dpi=80)
    plt.close(fig)

    # 3. frames
    fig, axs = plt.subplots(1, 2, figsize=(15, 5))
    xi = np.arange(len(metrics))
    for j, (k, lab) in enumerate((("cep_as_is", "as received (y left)"), ("cep_y_flip", "y flipped"),
                                  ("cep_no_rotation", "no rotation (world-aligned)"))):
        axs[0].bar(xi + (j - 1) * 0.27, [m[k] for m in metrics], 0.27, label=lab)
    axs[0].set_ylabel("whole-run CEP50 of world fixes (m), best latency")
    axs[0].set_xticks(xi, short, rotation=80, fontsize=7)
    axs[0].legend()
    have = [m for m in metrics if "outside_frac_as_is" in m]
    xj = np.arange(len(have))
    axs[1].bar(xj - 0.2, [m["outside_frac_as_is"] for m in have], 0.4, label="as received")
    axs[1].bar(xj + 0.2, [m["outside_frac_y_flip"] for m in have], 0.4, label="y flipped")
    axs[1].set_ylabel("fraction of raw fixes > 1 m outside the pool")
    axs[1].set_xticks(xj, [m["run"][5:] for m in have], rotation=80, fontsize=7)
    axs[1].legend()
    fig.suptitle("Frame mapping: the acoustic x/y as received fit the body frame; flipping y is worse")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig3_frames.png"), dpi=80)
    plt.close(fig)

    # 4. noise vs range
    N = np.vstack([m["_noise"] for m in metrics])
    N = N[np.isfinite(N[:, 1])]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.scatter(N[:, 0], N[:, 1], s=3, alpha=0.3, color="tab:blue", label="raw fix residual")
    bins = [0, 3, 6, 10, 15, 20, 30]
    mids, med, p90 = [], [], []
    for lo, hi in zip(bins[:-1], bins[1:]):
        k = (N[:, 0] >= lo) & (N[:, 0] < hi)
        if k.sum() > 10:
            mids.append((lo + hi) / 2)
            med.append(np.median(N[k, 1]))
            p90.append(np.percentile(N[k, 1], 90))
    ax.plot(mids, med, "ko-", label="median")
    ax.plot(mids, p90, "k--", label="p90")
    rr = np.linspace(0, 30, 50)
    ax.plot(rr, np.hypot(0.1 + 0.002 * rr, rr * np.radians(1.0)), "r-", label="spec (1 deg, 0.2 % range)")
    ax.set_xlim(0, 30)
    ax.set_ylim(0, 25)
    ax.set_xlabel("range (m)")
    ax.set_ylabel("distance to the +/-5 s median (m)")
    ax.set_title("Raw fix scatter vs range (pool, moving crawler; motion inflates it slightly)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig4_noise_vs_range.png"), dpi=80)
    plt.close(fig)

    # 5. HTTP delay and fix intervals
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.5))
    D = np.concatenate([r["ut"] - np.array([time.mktime((int(a[0]), int(a[1]), int(a[2]), int(a[3]), int(a[4]),
                                                        int(a[5]), 0, 0, -1)) + a[6] * 1e-6 for a in r["U"]])
                        for r in bags])
    axs[0].hist(np.clip(D, 1e-3, None), bins=np.logspace(-2, 2.1, 60))
    axs[0].set_xscale("log")
    axs[0].set_yscale("log")
    axs[0].set_xlabel("publish time - poll start (s)")
    axs[0].set_title("uwgps_log poll cycle duration (3 sequential HTTP requests)")
    G = np.concatenate([np.diff(r["ut"][new_mask(r["U"][:, 7:10])[0]]) for r in bags])
    axs[1].hist(np.clip(G, 0, 5), bins=100)
    axs[1].set_yscale("log")
    axs[1].set_xlabel("time between new raw fixes (s), clipped at 5")
    axs[1].set_title("new-fix intervals at the 2 Hz poll")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig5_transport.png"), dpi=80)
    plt.close(fig)

    # 6. legacy mechanism (semi-synthetic still pinger)
    fig, axs = plt.subplots(1, 2, figsize=(14, 4.5))
    pick = max(metrics, key=lambda m: m["mech_speed_p50"] if np.isfinite(m["mech_speed_p50"]) else 0)
    tt, e, along, sp = pick["_mech_series"]
    axs[0].plot(tt, e, "k-", lw=0.8)
    axs[0].set_xlabel("time (s)")
    axs[0].set_ylabel("legacy world error (m)")
    axs[0].set_title("legacy pinger, noise-free still target, run " + pick["run"][5:])
    for m in metrics:
        _, _, al, s_ = m["_mech_series"]
        k = s_ > 0.15
        axs[1].scatter(s_[k], al[k], s=1, alpha=0.2)
    vv = np.linspace(0, 1.2, 10)
    axs[1].plot(vv, 1.5 * vv, "r-", label="v x 1.5 s")
    axs[1].set_xlabel("boat speed (m/s)")
    axs[1].set_ylabel("legacy error along travel (m)")
    axs[1].set_ylim(-5, 8)
    axs[1].legend()
    fig.suptitle("Legacy algorithm on real motion and timing with a perfect still pinger (lag 1.5 s)")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig6_legacy_mechanism.png"), dpi=80)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--data", default=os.path.expanduser("~/ros2_ws/data"))
    args = ap.parse_args()
    bags, metrics = [], []
    for bag in sorted(glob.glob(os.path.join(args.data, "Robot_data", "**", "*-bag"), recursive=True)):
        r = load_bag(bag)
        if len(r["ut"]) < 20 or np.all(r["U"][:, 7:10] == 0):
            continue
        r["_dir"] = bag
        bags.append(r)
        metrics.append(analyse_bag(r, bag))
        print("bag", r["name"], flush=True)
    exports = [analyse_export(load_export(f))
               for f in sorted(glob.glob(os.path.join(args.data, "Processed_Robot_data", "pinger*")))]
    figures(bags, metrics, exports)
    DL = np.vstack([m["_drag_DL"] for m in metrics if len(m["_drag_DL"])])
    DP = np.vstack([m["_drag_DP"] for m in metrics if len(m["_drag_DP"])])
    taus = np.arange(0, 3.01, 0.25)
    pooled = {}
    for lab in ("raw", "fil"):
        c = np.mean([np.array(m["latency_curve_" + lab]) / np.nanmin(m["latency_curve_" + lab])
                     for m in metrics], axis=0)
        pooled[lab] = {"tau": taus.tolist(), "normalised": c.tolist(), "best_tau": float(taus[int(np.argmin(c))])}
    N = np.vstack([m["_noise"] for m in metrics])
    N = N[np.isfinite(N[:, 1])]
    noise = []
    for lo, hi in ((0, 3), (3, 6), (6, 10), (10, 15), (15, 20), (20, 30)):
        k = (N[:, 0] >= lo) & (N[:, 0] < hi)
        noise.append({"range": [lo, hi], "n": int(k.sum()), "p50": float(np.median(N[k, 1])),
                      "p90": float(np.percentile(N[k, 1], 90)), "p99": float(np.percentile(N[k, 1], 99))})
    out = {"bags": [{k: v for k, v in m.items() if not k.startswith("_")} for m in metrics],
           "exports": exports,
           "pooled_latency": pooled,
           "pooled_drag_slope_between_new_fixes": float(np.sum(DL * DP) / np.sum(DP * DP)),
           "noise_vs_range": noise}
    with open(os.path.join(HERE, "phase0_metrics.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    print(json.dumps({"pooled_latency": {k: v["best_tau"] for k, v in pooled.items()},
                      "pooled_drag_slope": out["pooled_drag_slope_between_new_fixes"]}, indent=1))


if __name__ == "__main__":
    main()
