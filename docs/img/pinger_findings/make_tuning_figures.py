#!/usr/bin/env python3
"""
The pictures of the station's "Field tuning" panel (pinger filter, 2026-11-24).

One PNG per knob of `field_tuning` in blueboat_control/config/pinger_filter.yaml,
written to blueboat_control/config/pinger_tuning/ (installed with the schema, so
the Mission Control Station finds them beside it). Each picture is produced by
running THE filter (pinger_filter_replay.replay, the estimator's own core) on a
synthetic scene at a low value, the default and a high value of the knob, with
the noise measured in the field (12 deg bearing, 20 % of range, 1.5 s latency).
Nothing is drawn by hand: what a picture shows is what the filter does.

    /usr/bin/python3 docs/img/pinger_findings/make_tuning_figures.py

Deterministic (fixed seeds). Re-run it after any change to the filter or to a
default, so the pictures keep telling the truth.
"""

import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path[:0] = [os.path.join(REPO, "test"),
                os.path.join(REPO, "blueboat_control", "src", "_custom_libraries")]

import pinger_estimator_core as pec  # noqa: E402
import pinger_params as pp  # noqa: E402
import test_pinger_filter as T  # noqa: E402  (the synthetic scenes of the A1 tests)

OUT = os.path.join(REPO, "blueboat_control", "config", "pinger_tuning")
DEFAULTS = pp.defaults()

# Field-measured noise (docs/PINGER_FILTER_FINDINGS.md): what the filter faces.
NOISE = dict(noise_deg=12.0, range_k=0.2, latency=1.5, fix_dt=0.6)

C_RAW, C_TRUE, C_EST, C_BOAT = "#b8b8b8", "#111111", "#1f6fd1", "#e8a25a"
C_PANEL = {"low": "#c0392b", "default": "#1e8449", "high": "#b9770e"}
STATUS_COLOUR = {"no_data": "#dddddd", "initialising": "#aab7c4", "tracking": "#27ae60",
                 "coasting": "#f39c12", "stale": "#c0392b"}


def run(ev, **overrides):
    return T.run_core(ev, pp.load_params(overrides=overrides))


def track(res):
    e = [(x["t"], x["x"], x["y"]) for x in res["estimates"] if x.get("x") is not None]
    return np.array(e) if e else np.zeros((0, 3))


def raw_world(res):
    w = [f["world"] for f in res["fixes"] if f.get("world") is not None]
    return np.array(w) if w else np.zeros((0, 2))


def error(res, target, t_from=15.0, t_to=math.inf):
    tr = track(res)
    tr = tr[(tr[:, 0] >= t_from) & (tr[:, 0] <= t_to)]
    if not len(tr):
        return float("nan")
    d = [math.hypot(x - target(t)[0], y - target(t)[1]) for t, x, y in tr]
    return float(np.median(d))


SEEDS = 5    # every quoted error is the median over this many random scenes


def still_metric(results, target):
    e = np.median([error(r, target) for r in results])
    return f"median error {e:.2f} m"


def drive_stop_metric(results, target):
    """The crawler drives until DRIVE_END, then stands still: both matter."""
    drive = np.median([error(r, target, 15.0, DRIVE_END) for r in results])
    stop = np.median([error(r, target, DRIVE_END + 15.0) for r in results])
    return f"error while driving {drive:.2f} m\nerror while stopped {stop:.2f} m"


def fmt(v):
    return f"{v:g}"


def three_panels(name, title, values, scene, target, boat, overrides=None, limits=None,
                 labels=("low", "default", "high"), note="", metric=still_metric):
    """Top-down map per value: boat track, the fixes as the filter places them,
    the truth, the filter's estimate (of the first scene); the errors are the
    median over SEEDS scenes. `scene(seed)` -> events."""
    evs = [scene(k) for k in range(SEEDS)]
    ev = evs[0]
    unit = pp.load_schema()["parameters"][name].get("unit") or ""
    fig, axes = plt.subplots(1, len(values), figsize=(3.3 * len(values), 3.45), dpi=100)
    ts = np.arange(0.0, max(e[0] for e in ev), 0.5)
    truth = np.array([target(t) for t in ts])
    bx = np.array([boat(t)[:2] for t in ts])
    for ax, value, label in zip(axes, values, labels):
        ov = dict(overrides or {}, **{name: value})
        results = [run(e, **ov) for e in evs]
        res = results[0]
        raw, tr = raw_world(res), track(res)
        ax.plot(bx[:, 0], bx[:, 1], color=C_BOAT, lw=1.0, alpha=0.7, label="boat")
        if len(raw):
            ax.scatter(raw[:, 0], raw[:, 1], s=4, color=C_RAW, label="USBL fixes", zorder=1)
        ax.plot(truth[:, 0], truth[:, 1], color=C_TRUE, lw=2.2, ls="--", label="true pinger", zorder=3)
        if len(np.unique(truth, axis=0)) == 1:
            ax.plot(*truth[0], marker="*", ms=13, color=C_TRUE, zorder=4)
        if len(tr):
            ax.plot(tr[:, 1], tr[:, 2], color=C_EST, lw=1.6, label="filtered", zorder=5)
        kind = ("default" if label in ("default", "right") else
                ("low" if values.index(value) == 0 else "high"))
        ax.set_title(f"{label}: {fmt(value)} {unit}".strip(), fontsize=9,
                     color=C_PANEL[kind], fontweight="bold")
        ax.text(0.02, 0.02, metric(results, target), transform=ax.transAxes, fontsize=8,
                bbox=dict(fc="white", ec="none", alpha=0.85))
        ax.set_aspect("equal")
        if limits:
            ax.set_xlim(*limits[0])
            ax.set_ylim(*limits[1])
        ax.tick_params(labelsize=7)
    axes[0].legend(fontsize=7, loc="upper left", framealpha=0.9)
    fig.suptitle(title + (f"   ({note})" if note else "")
                 + f"\nlines: one scene; errors: median of {SEEDS} scenes", fontsize=9)
    fig.tight_layout()
    path = os.path.join(OUT, f"{name}.png")
    fig.savefig(path)
    plt.close(fig)
    return path


# ------------------------------------------------------------------ scenes
def still_target():
    return lambda t: (10.0, 6.0)


def circling_boat():
    """The boat circles the pinger at 10 m, 0.8 m/s: always turning."""
    return T.circle_boat(radius=10.0, speed=0.8, centre=(10.0, 6.0))


DRIVE_END = 120.0     # the crawler drives 16 m east then 8 m north at 0.2 m/s, then stops


def crawler():
    """The brush robot's pattern: 16 m east, a right-angle turn, 8 m north
    (0.2 m/s, 120 s), then standing still (cleaning one spot)."""
    def target(t):
        s = min(t, DRIVE_END) * 0.2
        if s <= 16.0:
            return (4.0 + s, 3.0)
        return (20.0, 3.0 + (s - 16.0))
    return target


CRAWL_LIMITS = ((-2, 27), (-4, 16))


def lawnmower():
    return T.lawnmower_boat(speed=0.6, leg=24.0, gap=5.0)


def fig_latency():
    # A survey with tight turns (0.4 rad/s): a fix applied at the wrong boat
    # heading lands range x turn rate x delay off (10 m x 0.4 x 1.5 s = 6 m).
    # (Circling the pinger would hide it: the pinger stays abeam.)
    tgt = lambda t: (12.0, 6.0)                 # noqa: E731
    boat = T.lawnmower_boat(speed=0.8, leg=24.0, gap=4.0)
    def scene(k):
        return T.scenario(tgt, boat, duration=150.0, seed=3 + 100 * k, **NOISE)[0]
    return three_panels("latency_s", "Fix delay: a still pinger, a survey with tight turns",
                        [0.0, 1.5, 3.0], scene, tgt, boat, limits=((-4, 28), (-6, 18)),
                        labels=("wrong", "right", "wrong"), note="true delay 1.5 s")


def fig_mount_yaw():
    tgt, boat = still_target(), circling_boat()
    def scene(k):
        return T.scenario(tgt, boat, duration=150.0, seed=4 + 100 * k, **NOISE)[0]
    return three_panels("mount_yaw_deg", "Receiver rotation: a still pinger, the boat circling it",
                        [-25.0, 0.0, 25.0], scene, tgt, boat, limits=((-2, 22), (-6, 18)),
                        labels=("wrong", "right", "wrong"),
                        note="true rotation 0 deg")


def fig_y_sign():
    tgt = lambda t: (10.0, 9.0)                  # noqa: E731
    boat = lawnmower()
    def scene(k):
        return T.scenario(tgt, boat, duration=150.0, seed=5 + 100 * k, **NOISE)[0]
    return three_panels("y_sign", "Left / right mirror: a still pinger beside a lawnmower survey",
                        [-1, 1], scene, tgt, boat, labels=("wrong (mirrored)", "right"),
                        limits=((-6, 28), (-12, 22)), note="true sign +1")


CRAWL = "the robot drives 16 m + 8 m at 0.2 m/s, then stops"


def fig_sigma_bearing():
    tgt, boat = crawler(), lawnmower()
    def scene(k):
        return T.scenario(tgt, boat, duration=240.0, seed=6 + 100 * k, **NOISE)[0]
    return three_panels("sigma_bearing_deg", "Trust in each fix: " + CRAWL,
                        [3.0, 12.0, 40.0], scene, tgt, boat, limits=CRAWL_LIMITS,
                        metric=drive_stop_metric)


def fig_sigma_a():
    tgt, boat = crawler(), lawnmower()
    def scene(k):
        return T.scenario(tgt, boat, duration=240.0, seed=7 + 100 * k, **NOISE)[0]
    return three_panels("sigma_a", "Agility: " + CRAWL,
                        [0.002, 0.02, 0.2], scene, tgt, boat, limits=CRAWL_LIMITS,
                        metric=drive_stop_metric)


def fig_v_max():
    tgt, boat = crawler(), lawnmower()
    def scene(k):
        return T.scenario(tgt, boat, duration=240.0, seed=8 + 100 * k, outliers=0.1, **NOISE)[0]
    return three_panels("v_max", "Top speed: " + CRAWL + ", 10 % wild fixes",
                        [0.05, 0.5, 3.0], scene, tgt, boat, limits=CRAWL_LIMITS,
                        metric=drive_stop_metric)


def fig_gate_soft():
    tgt, boat = crawler(), lawnmower()
    def scene(k):
        return T.scenario(tgt, boat, duration=240.0, seed=9 + 100 * k, outliers=0.2, **NOISE)[0]
    return three_panels("gate_soft", "Outlier tolerance: " + CRAWL + ", 20 % wild fixes",
                        [1.0, 5.99, 60.0], scene, tgt, boat, limits=CRAWL_LIMITS,
                        overrides={"gate_hard": 120.0}, metric=drive_stop_metric)


def fig_reinit_n():
    """Error against time: a 3-fix burst 12 m away at 40 s, then the pinger is
    moved 15 m at 90 s. Low: the burst drags the estimate; high: slow to move."""
    boat = lawnmower()

    def tgt(t):
        return (10.0, 6.0) if t < 90.0 else (22.0, 15.0)
    ev, _ = T.scenario(tgt, boat, duration=170.0, seed=10, **NOISE)
    # The burst: replace three fixes after 40 s by agreeing ones 12 m away.
    out, k = [], 0
    for e in ev:
        if e[1] == "fixjson" and 40.0 <= e[0] and k < 3:
            x, y, yaw, _, _ = boat(e[0] - 1.5)
            wx, wy = 10.0 + 12.0 - x, 6.0 - 9.0 - y
            c, s = math.cos(yaw), math.sin(yaw)
            e = (e[0], e[1], T.make_fix_json(e[0] - 2e-4, (c * wx + s * wy, -s * wx + c * wy, 2.0),
                                             10000 + k))
            k += 1
        out.append(e)
    fig, ax = plt.subplots(figsize=(9.9, 3.45), dpi=100)
    for (n, rad), kind, ls in (((2, 2.0), "low", "-"), ((4, 2.0), "default", "-"),
                               ((4, 8.0), "high", "--")):
        res = run(out, reinit_n=n, reinit_radius_m=rad)
        tr = track(res)
        d = [math.hypot(x - tgt(t)[0], y - tgt(t)[1]) for t, x, y in tr]
        ax.plot(tr[:, 0], d, color=C_PANEL[kind], lw=1.6, ls=ls,
                label=f"{'default' if kind == 'default' else kind}: reinit_n {n}, radius {fmt(rad)} m")
    ax.axvline(40.0, color="#555555", ls=":", lw=1)
    ax.text(40.5, ax.get_ylim()[1] * 0.92, "3 agreeing wild fixes", fontsize=8)
    ax.axvline(90.0, color="#555555", ls=":", lw=1)
    ax.text(90.5, ax.get_ylim()[1] * 0.92, "pinger moved 15 m", fontsize=8)
    ax.set_xlabel("time (s)", fontsize=8)
    ax.set_ylabel("error of the filtered pinger (m)", fontsize=8)
    ax.set_ylim(bottom=0)
    ax.legend(fontsize=8, loc="center right")
    ax.tick_params(labelsize=7)
    fig.suptitle("Jump to a new place: how many far-off fixes, agreeing within what radius", fontsize=10)
    fig.tight_layout()
    path = os.path.join(OUT, "reinit_n.png")
    fig.savefig(path)
    plt.close(fig)
    return path


def fig_stale():
    """Status against time through a 12 s dropout of the USBL (40-52 s)."""
    tgt, boat = still_target(), circling_boat()
    ev, _ = T.scenario(tgt, boat, duration=80.0, seed=11, gaps=((40.0, 52.0),), **NOISE)
    settings = (((1.0, 3.0), "low"), ((2.0, 6.0), "default"), ((5.0, 15.0), "high"))
    fig, ax = plt.subplots(figsize=(9.9, 3.0), dpi=100)
    for row, ((coast, stale), kind) in enumerate(settings):
        # The live estimator's status at every odometry tick (0.1 s).
        core = pec.EstimatorCore(pp.load_params(overrides={"coast_after_s": coast,
                                                           "stale_timeout_s": stale}))
        spans = []
        for t, what, v in ev:
            if what == "odom":
                core.on_odom(t, *v)
                st = core.estimate(t)["status"]
                if spans and spans[-1][2] == st:
                    spans[-1][1] = t
                else:
                    spans.append([t, t, st])
            elif what == "imu":
                core.on_imu(t, v)
            else:
                core.on_fix_json(t, v)
        for a, b, st in spans:
            ax.barh(row, b - a + 0.1, left=a, color=STATUS_COLOUR.get(st, "#ffffff"), height=0.6, lw=0)
        ax.text(80.8, row, f"{kind}: coast {fmt(coast)} s, stale {fmt(stale)} s", va="center",
                fontsize=8, color=C_PANEL[kind])
    ax.axvspan(40.0, 52.0, color="#000000", alpha=0.06)
    ax.text(46.0, 2.55, "no USBL fix for 12 s", ha="center", fontsize=8)
    ax.set_yticks([])
    ax.set_xlim(0, 118)
    ax.set_ylim(-0.6, 2.9)
    ax.set_xlabel("time (s)", fontsize=8)
    ax.tick_params(labelsize=7)
    handles = [plt.Rectangle((0, 0), 1, 1, color=STATUS_COLOUR[s]) for s in
               ("initialising", "tracking", "coasting", "stale")]
    fig.legend(handles, ["initialising", "tracking", "coasting (still followed)",
                         "stale (follow mode holds the boat)"], fontsize=7, loc="lower center", ncol=4)
    fig.suptitle("When to give up: the status through a USBL dropout", fontsize=10)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    path = os.path.join(OUT, "stale_timeout_s.png")
    fig.savefig(path)
    plt.close(fig)
    return path


def main():
    os.makedirs(OUT, exist_ok=True)
    for make in (fig_latency, fig_mount_yaw, fig_y_sign, fig_sigma_bearing, fig_sigma_a, fig_v_max,
                 fig_gate_soft, fig_reinit_n, fig_stale):
        print("wrote", os.path.relpath(make(), REPO))


if __name__ == "__main__":
    main()
