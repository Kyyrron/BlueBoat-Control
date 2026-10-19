#!/usr/bin/env python3
"""
ROS-free tests of the live path update core (_custom_libraries/path_update.py).

House pattern (log_reviewer/smoke_test.py): one plain script, a check()
helper, exit code 1 on any failure. Also collected by pytest (test_* below).

    /usr/bin/python3 test/test_path_update.py

Covers every answer code, junction continuity (also mid-turn), the first path
from an empty start, back-to-back updates, the end of the path, the state gate
and its order, "a refusal changes nothing", and determinism.
"""

import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "blueboat_control", "src", "_custom_libraries"))

import path_stamp as ps                     # noqa: E402
import path_update as pu                    # noqa: E402
import yaml_trajectory as yt                # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    if not cond:
        print(f"FAIL  {name}  {detail}")
        if "pytest" in sys.modules:         # under pytest a failed check fails the test
            raise AssertionError(f"{name}  {detail}")


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
PIDLOS = dict(path_time=0.05, path_steps=2, dt=0.05)        # master_control PID/LoS
MPC_SIM = dict(path_time=6.0, path_steps=31, dt=0.05)       # sim MPC window
MPC_REAL = dict(path_time=2.5, path_steps=16, dt=0.05)      # real MPC window


def ctx(**kw):
    base = dict(path_source="replanner", boat=(0.0, 0.0, 0.0), **PIDLOS)
    base.update(kw)
    return pu.Context(**base)


def line(t0=0.0, t1=60.0, dt=0.5, x0=0.0, y0=0.0, heading=0.0, speed=0.5):
    t = np.arange(t0, t1 + 1e-9, dt)
    s = speed * (t - t0)
    return t, x0 + s * math.cos(heading), y0 + s * math.sin(heading), \
        np.full_like(t, heading)


def arc(t0, t1, dt, cx, cy, r, phi0, speed=0.5, ccw=True):
    """Circle arc parametrised by time; yaw = tangent."""
    t = np.arange(t0, t1 + 1e-9, dt)
    sgn = 1.0 if ccw else -1.0
    phi = phi0 + sgn * speed * (t - t0) / r
    x, y = cx + r * np.cos(phi), cy + r * np.sin(phi)
    yaw = pu.wrap(phi + sgn * math.pi / 2)
    return t, x, y, yaw


def req(rid, t, x, y, yaw, c=float("nan"), frame="world"):
    return pu.Request(id=rid, change_point=c, frame=frame, t=list(t), x=list(x),
                      y=list(y), yaw=list(yaw))


def loaded(path=None, **kw):
    """An updater in a replanner run, holding at the origin, first path loaded."""
    u = pu.PathUpdater()
    c = ctx(**kw)
    u.reset(c, "launch")
    u.set_hold((0.0, 0.0, 0.0))
    t, x, y, yaw = path if path is not None else line()
    ans, _, new_tau = u.handle(req("p0", t, x, y, yaw), c)
    assert ans["accepted"], ans
    c.tau = new_tau
    u.tick(c)
    return u, c


def snapshot(u, c):
    return (tuple(u.plan.ids), [s.start for s in u.plan.segments],
            tuple(sorted(u.used_ids)), u.awaiting, u.hold, u.end_reached,
            tuple(np.round(np.concatenate(u.window(c)[1:]), 12))
            if u.plan else None)


# ---------------------------------------------------------------------------
# 1. constants agree with their owners
# ---------------------------------------------------------------------------
def test_constants():
    check("MAX_PARAMETER == path_stamp.MAX_PLAUSIBLE_PARAMETER",
          pu.MAX_PARAMETER == ps.MAX_PLAUSIBLE_PARAMETER)
    check("every code has a category",
          all(code in pu.CATEGORY for code in (
              pu.NOT_READY, pu.NO_RUN, pu.MISSION_OWNED, pu.OPERATOR_OVERRIDE,
              pu.BAD_REQUEST, pu.DUPLICATE_ID, pu.TOO_FEW_POINTS,
              pu.TOO_MANY_POINTS, pu.NOT_FINITE, pu.TIME_NOT_INCREASING,
              pu.TIME_OUT_OF_RANGE, pu.TOO_FAST, pu.HEADING_INCOHERENT,
              pu.CHANGE_POINT_OUTSIDE, pu.LEAD_TOO_SHORT,
              pu.DISCONTINUOUS_POSITION, pu.DISCONTINUOUS_HEADING)))


# ---------------------------------------------------------------------------
# 2. yaml_trajectory.from_points is the file loader, sample for sample
# ---------------------------------------------------------------------------
def test_from_points_equals_file():
    t, x, y, yaw = arc(0.0, 40.0, 0.7, 2.0, -1.0, 4.0, 0.3)
    pts = np.column_stack([t, x, y, yaw])
    with tempfile.TemporaryDirectory() as d:
        f = os.path.join(d, "p.yaml")
        with open(f, "w") as h:
            h.write("format: blueboat_trajectory/1\nname: p\nloop: false\npoints:\n")
            for row in pts:
                h.write("  - [" + ", ".join(repr(float(v)) for v in row) + "]\n")
        a = yt.YamlTrajectory(f)
    b = yt.YamlTrajectory.from_points(pts, name="p")
    probe = np.concatenate([np.linspace(-5, 50, 997), t])
    same = all(a.pose(float(s)) == b.pose(float(s)) for s in probe)
    check("from_points == file load (bit-identical poses)", same)
    check("from_points attributes", (a.name, a.loop, a.duration) ==
          (b.name, b.loop, b.duration))


# ---------------------------------------------------------------------------
# 3. the state gate, in order
# ---------------------------------------------------------------------------
def test_state_gate():
    t, x, y, yaw = line()
    cases = [
        ("not_ready (E-STOP / readiness)", dict(ready=False), pu.NOT_READY),
        ("no_run (session idle)", dict(session_mode=True, run_active=False), pu.NO_RUN),
        ("mission_owned (fixed mission)", dict(path_source="mission"), pu.MISSION_OWNED),
        ("mission_owned (pinger)", dict(use_pinger=True), pu.MISSION_OWNED),
        ("operator_override (manual)", dict(override="manual"), pu.OPERATOR_OVERRIDE),
        ("operator_override (arrow)", dict(override="arrow"), pu.OPERATOR_OVERRIDE),
        # order: not_ready wins over everything after it
        ("order: not_ready before no_run",
         dict(ready=False, session_mode=True, run_active=False), pu.NOT_READY),
        ("order: no_run before mission_owned",
         dict(session_mode=True, run_active=False, path_source="mission"), pu.NO_RUN),
        ("order: mission_owned before override",
         dict(path_source="mission", override="manual"), pu.MISSION_OWNED),
    ]
    for name, kw, code in cases:
        u = pu.PathUpdater()
        c = ctx(**kw)
        u.reset(c, "launch")
        u.set_hold((0.0, 0.0, 0.0))
        ans, ev, nt = u.handle(req("a", t, x, y, yaw), c)
        check(f"gate {name}", (not ans["accepted"]) and ans["reason"] == code
              and ev["event"] == "request_refused" and nt is None,
              f"{ans}")
    # replanner run before odometry: hold not captured
    u = pu.PathUpdater()
    c = ctx()
    u.reset(c, "launch")
    ans, _, _ = u.handle(req("a", t, x, y, yaw), c)
    check("gate not_ready (no hold pose yet)", ans["reason"] == pu.NOT_READY)
    check("progress state awaiting_path before odometry",
          u.progress(c)["state"] == "awaiting_path")


# ---------------------------------------------------------------------------
# 4. request validity -- every request code
# ---------------------------------------------------------------------------
def test_request_codes():
    t, x, y, yaw = line()
    n = len(t)

    def answer(r, cfg=None, **kw):
        u = pu.PathUpdater(cfg)
        c = ctx(**kw)
        u.reset(c, "launch")
        u.set_hold((0.0, 0.0, 0.0))
        return u.handle(r, c)[0]

    check("bad_request: empty id", answer(req("", t, x, y, yaw))["reason"] == pu.BAD_REQUEST)
    check("bad_request: id with a comma",
          answer(req("a,b", t, x, y, yaw))["reason"] == pu.BAD_REQUEST)
    check("bad_request: id too long",
          answer(req("a" * 65, t, x, y, yaw))["reason"] == pu.BAD_REQUEST)
    check("bad_request: frame", answer(req("a", t, x, y, yaw, frame="map"))["reason"]
          == pu.BAD_REQUEST)
    check("bad_request: lengths",
          answer(req("a", t, x[:-1], y, yaw))["reason"] == pu.BAD_REQUEST)
    bad = req("a", t, x, y, yaw)
    bad.x = ["a"] * n
    check("bad_request: non-numeric", answer(bad)["reason"] == pu.BAD_REQUEST)
    check("too_few_points", answer(req("a", t[:1], x[:1], y[:1], yaw[:1]))["reason"]
          == pu.TOO_FEW_POINTS)
    check("too_many_points", answer(req("a", t, x, y, yaw),
                                    pu.Config(max_points=n - 1))["reason"]
          == pu.TOO_MANY_POINTS)
    xn = x.copy()
    xn[5] = np.nan
    check("not_finite", answer(req("a", t, xn, y, yaw))["reason"] == pu.NOT_FINITE)
    tt = t.copy()
    tt[10] = tt[9]
    check("time_not_increasing (equal)", answer(req("a", tt, x, y, yaw))["reason"]
          == pu.TIME_NOT_INCREASING)
    tt = t.copy()
    tt[10] = tt[9] + 0.0005
    check("time_not_increasing (< 1 ms)", answer(req("a", tt, x, y, yaw))["reason"]
          == pu.TIME_NOT_INCREASING)
    check("time_out_of_range (t < 0)", answer(req("a", t - 1.0, x, y, yaw))["reason"]
          == pu.TIME_OUT_OF_RANGE)
    check("time_out_of_range (t >= 1e6)",
          answer(req("a", t + 1e6, x, y, yaw))["reason"] == pu.TIME_OUT_OF_RANGE)
    tf, xf, yf, yawf = line(speed=2.5)
    check("too_fast (2.5 m/s)", answer(req("a", tf, xf, yf, yawf))["reason"] == pu.TOO_FAST)
    tm, xm, ym, yawm = line(t1=10.0, dt=0.5, speed=0.5)
    check("too_fast catches ms-for-s", answer(req("a", tm / 1000.0 + 0.0, xm, ym, yawm))
          ["reason"] in (pu.TOO_FAST, pu.TIME_NOT_INCREASING))
    # compass heading (CW from North) instead of ENU yaw: northbound line,
    # ENU yaw = pi/2, compass = 0
    tn, xn2, yn, _ = line(heading=math.pi / 2)
    check("heading_incoherent (compass instead of ENU)",
          answer(req("a", tn, xn2, yn, np.zeros_like(tn)))["reason"]
          == pu.HEADING_INCOHERENT)
    # degrees instead of radians: caught when the wrapped value misses the
    # course by > 45 deg (60 deg -> 60 rad wraps to -2.83 rad, 138 deg off).
    # NOT always: 90 (deg) wraps to 2.04 rad, 27 deg from pi/2, and passes --
    # a limitation stated in the interface doc.
    t6, x6, y6, _ = line(heading=math.radians(60))
    check("heading_incoherent (degrees instead of radians, 60 deg)",
          answer(req("a", t6, x6, y6, np.full_like(t6, 60.0)))["reason"]
          == pu.HEADING_INCOHERENT)
    check("heading coherent: ENU radians accepted",
          answer(req("a", tn, xn2, yn, np.full_like(tn, math.pi / 2)))["accepted"])
    # a stationary path (hold) may carry any yaw
    ts = np.arange(0.0, 10.0, 0.5)
    check("stationary samples: any yaw allowed",
          answer(req("a", ts, np.zeros_like(ts), np.zeros_like(ts),
                     np.full_like(ts, 2.0)))["accepted"])


# ---------------------------------------------------------------------------
# 5. first path from an empty start
# ---------------------------------------------------------------------------
def test_first_path():
    u = pu.PathUpdater()
    c = ctx()
    ev = u.reset(c, "launch")
    check("replanner reset emits 'cleared'", ev is not None and ev["event"] == "cleared")
    check("awaiting after reset", u.awaiting and u.progress(c)["state"] == "awaiting_path")
    u.set_hold((1.0, 2.0, 0.3))
    check("progress publishes the hold pose", u.progress(c)["hold"] ==
          {"x": 1.0, "y": 2.0, "yaw": 0.3})
    t, x, y, yaw = line(t0=12.0, t1=70.0, x0=1.0, y0=2.0, heading=-1.2)
    # first sample 0.3 m off the hold -> refused
    ans, _, _ = u.handle(req("far", t, x + 0.3, y, yaw), c)
    check("first path: discontinuous_position vs hold", ans["reason"] == pu.DISCONTINUOUS_POSITION)
    # c != t[0] -> refused
    ans, _, _ = u.handle(req("c", t, x, y, yaw, c=20.0), c)
    check("first path: change_point must be t[0]", ans["reason"] == pu.CHANGE_POINT_OUTSIDE)
    # heading NOT checked against the stationary hold (boat turns in place)
    ans, ev, nt = u.handle(req("first", t, x, y, yaw), c)
    check("first path accepted (heading differs from hold yaw)", ans["accepted"], f"{ans}")
    check("first path: tau := t[0]", nt == 12.0 and ans["tau"] == 12.0)
    check("first path: heading_checked False", ev["heading_checked"] is False)
    check("first path: explicit change_point == t[0] also accepted",
          _first_with_c(t, x, y, yaw, 12.0))
    c.tau = nt
    events = u.tick(c)
    check("first path activates at once", [e["event"] for e in events] == ["activated"])
    check("progress following", u.progress(c)["state"] == "following" and
          u.progress(c)["following_id"] == "first")
    # a second 'first path' is now an ordinary update and needs a change point
    ans, _, _ = u.handle(req("again", t, x, y, yaw), c)
    check("after load, NaN change point refused", ans["reason"] == pu.CHANGE_POINT_OUTSIDE)


def _first_with_c(t, x, y, yaw, c0):
    u = pu.PathUpdater()
    c = ctx()
    u.reset(c, "launch")
    u.set_hold((float(x[0]), float(y[0]), 0.0))
    return u.handle(req("first", t, x, y, yaw, c=c0), c)[0]["accepted"]


# ---------------------------------------------------------------------------
# 6. lead
# ---------------------------------------------------------------------------
def test_lead():
    u, c = loaded()                                   # 0.5 m/s line, PID/LoS window
    check("lead PID/LoS @0.5 m/s = 6.0 s", abs(u.lead_min(c) - 6.0) < 1e-9, u.lead_min(c))
    u2, c2 = loaded(**MPC_SIM)
    check("lead MPC sim @0.5 m/s = 6.1 s", abs(u2.lead_min(c2) - 6.1) < 1e-9, u2.lead_min(c2))
    u3, c3 = loaded(path=line(speed=1.0), **MPC_REAL)
    check("lead MPC real @1.0 m/s = 3.0 s", abs(u3.lead_min(c3) - 3.0) < 1e-9, u3.lead_min(c3))
    u4, c4 = loaded(path=line(speed=0.28))
    check("lead @0.28 m/s = 10.71 s", abs(u4.lead_min(c4) - 3.0 / 0.28) < 1e-6)
    u5, c5 = loaded(path=line(speed=0.5))
    u5.cfg.lead_factor = 2.0
    check("lead_factor scales the governor term", abs(u5.lead_min(c5) - 12.0) < 1e-9)
    # stationary reference (past the end): floor only
    u6, c6 = loaded(path=line(t1=10.0))
    c6.tau = 30.0
    check("lead past the end = floor", abs(u6.lead_min(c6) - (0.05 + 0.1)) < 1e-12)

    # refusal / acceptance around the boundary
    u, c = loaded()
    c.tau = 10.0
    u.tick(c)
    t, x, y, yaw = line()          # the same line: continuous everywhere
    ans, _, _ = u.handle(req("short", t, x, y, yaw, c=15.99), c)
    check("lead_too_short just below", ans["reason"] == pu.LEAD_TOO_SHORT and
          abs(ans["min_change_point"] - 16.0) < 1e-9, f"{ans}")
    ans, _, _ = u.handle(req("ok", t, x, y, yaw, c=16.0), c)
    check("accepted exactly at min_change_point", ans["accepted"], f"{ans}")


# ---------------------------------------------------------------------------
# 7. continuity, splice, update during a turn, back-to-back
# ---------------------------------------------------------------------------
def test_continuity_and_splice():
    u, c = loaded()
    c.tau = 5.0
    u.tick(c)
    cp = 20.0                                         # x = 10 on the line
    # new path turns left from the junction: 10 m along +x then an arc
    t1 = np.arange(cp, 80.0, 0.5)
    th = np.minimum((t1 - cp) * 0.05, math.pi / 2)    # smooth heading ramp
    xs = 10.0 + np.concatenate([[0.0], np.cumsum(0.25 * np.cos(th[:-1]))])
    ys = 0.0 + np.concatenate([[0.0], np.cumsum(0.25 * np.sin(th[:-1]))])
    # yaw along the polyline course (coherent)
    yaw_s = np.concatenate([np.arctan2(np.diff(ys), np.diff(xs)), [th[-1]]])
    ans, ev, _ = u.handle(req("turnleft", t1, xs, ys, yaw_s, c=cp), c)
    check("continuous update accepted", ans["accepted"], f"{ans}")
    check("junction gaps reported", ev["pos_gap"] < 1e-9 and ev["heading_checked"] is True)
    # composite continuity: positions just either side of the junction
    pa = np.array(u.plan.pose(cp - 1e-6)[:2])
    pb = np.array(u.plan.pose(cp + 1e-6)[:2])
    check("composite path continuous at the junction", np.hypot(*(pa - pb)) < 1e-5)
    check("plan = [p0 from 0, turnleft from 20]",
          u.plan.ids == ["p0", "turnleft"] and u.plan.segments[1].start == cp)
    # windows straddling the junction come from both segments
    c.tau = cp - 0.025
    _, wx, wy, _ = u.window(c)
    check("window straddling c is continuous", abs(wx[1] - wx[0] - 0.025) < 1e-6)

    # offset 0.25 m -> discontinuous_position; 15 deg -> discontinuous_heading
    u, c = loaded()
    c.tau = 5.0
    t, x, y, yaw = line(t0=20.0, x0=10.0)
    ans, _, _ = u.handle(req("off", t, x, y + 0.25, yaw, c=20.0), c)
    check("discontinuous_position (0.25 m > 0.20)", ans["reason"] == pu.DISCONTINUOUS_POSITION)
    ans, _, _ = u.handle(req("near", t, x, y + 0.15, yaw, c=20.0), c)
    check("0.15 m < 0.20 accepted", ans["accepted"])
    u, c = loaded()
    c.tau = 5.0
    tr, xr, yr, yawr = line(t0=20.0, x0=10.0, heading=math.radians(15))
    ans, ev, _ = u.handle(req("rot", tr, xr, yr, yawr, c=20.0), c)
    check("discontinuous_heading (15 deg > 10)", ans["reason"] == pu.DISCONTINUOUS_HEADING
          and abs(ev["heading_gap"] - math.radians(15)) < 1e-9)
    tr, xr, yr, yawr = line(t0=20.0, x0=10.0, heading=math.radians(8))
    ans, _, _ = u.handle(req("rot8", tr, xr, yr, yawr, c=20.0), c)
    check("8 deg < 10 accepted", ans["accepted"])


def test_update_during_turn():
    # current path: a 5 m radius circle at 0.5 m/s (CCW), boat on it
    path = arc(0.0, 120.0, 0.5, 0.0, 5.0, 5.0, -math.pi / 2)
    u, c = loaded(path=path)
    c.tau = 8.0
    u.tick(c)
    cp = 18.3                                          # mid-turn, between samples
    x0, y0, yaw0 = u.plan.pose(cp)
    # new path: leave the circle along its tangent at cp (straight line)
    t, x, y, yaw = line(t0=cp, t1=cp + 60, x0=x0, y0=y0, heading=yaw0)
    ans, ev, _ = u.handle(req("tangent", t, x, y, yaw, c=cp), c)
    check("mid-turn update along the tangent accepted", ans["accepted"], f"{ans}")
    check("mid-turn junction: interpolated pose matched",
          ev["pos_gap"] < 1e-9 and ev["heading_gap"] < 1e-9)
    # the same but sampled on the Replanner's own coarser grid starting before cp
    u, c = loaded(path=path)
    c.tau = 8.0
    t2 = np.arange(cp - 3.0, cp + 60, 1.0)
    on_arc = t2 < cp
    xa, ya, yawa = u.plan.poses(t2)
    xl = x0 + 0.5 * (t2 - cp) * math.cos(yaw0)
    yl = y0 + 0.5 * (t2 - cp) * math.sin(yaw0)
    xs = np.where(on_arc, xa, xl)
    ys = np.where(on_arc, ya, yl)
    ys_yaw = np.where(on_arc, yawa, yaw0)
    ans, ev, _ = u.handle(req("tangent2", t2, xs, ys, ys_yaw, c=cp), c)
    check("mid-turn update with samples starting before c accepted", ans["accepted"],
          f"{ans}")
    # a junction that turns the wrong way mid-turn is refused
    u, c = loaded(path=path)
    c.tau = 8.0
    t, x, y, yaw = line(t0=cp, t1=cp + 60, x0=x0, y0=y0, heading=yaw0 + math.radians(30))
    ans, _, _ = u.handle(req("kink", t, x, y, yaw, c=cp), c)
    check("mid-turn kink refused (discontinuous_heading)",
          ans["reason"] == pu.DISCONTINUOUS_HEADING)


def test_back_to_back():
    u, c = loaded()
    c.tau = 4.0
    u.tick(c)
    tA, xA, yA, yawA = line(t0=30.0, x0=15.0)          # A at 30 (same line)
    ansA, _, _ = u.handle(req("A", tA, xA, yA, yawA, c=30.0), c)
    # B after A's change point: continuity vs A
    tB, xB, yB, yawB = line(t0=40.0, x0=20.0)
    ansB, _, _ = u.handle(req("B", tB, xB, yB, yawB, c=40.0), c)
    check("back-to-back: A then B (later) accepted", ansA["accepted"] and ansB["accepted"])
    check("plan [p0, A, B]", u.plan.ids == ["p0", "A", "B"])
    # C before A's change point: replaces A and B
    tC, xC, yC, yawC = line(t0=12.0, x0=6.0)
    ansC, _, _ = u.handle(req("C", tC, xC, yC, yawC, c=12.0), c)
    check("back-to-back: C before A replaces A and B", ansC["accepted"] and
          u.plan.ids == ["p0", "C"] and ansC["active_id"] == "C")
    check("pending ids", u.progress(c)["pending_ids"] == ["C"])
    # duplicate id refused, but a REFUSED id may be reused
    ans, _, _ = u.handle(req("A", tA, xA, yA, yawA, c=30.0), c)
    check("duplicate_id refused (accepted earlier)", ans["reason"] == pu.DUPLICATE_ID)
    ans, _, _ = u.handle(req("D", tA, xA, yA + 1.0, yawA, c=30.0), c)
    check("refused D...", ans["reason"] == pu.DISCONTINUOUS_POSITION)
    ans, _, _ = u.handle(req("D", tA, xA, yA, yawA, c=30.0), c)
    check("...then D accepted with the same id", ans["accepted"])
    # activations happen in order as tau advances
    kinds = []
    for tau in np.arange(4.0, 35.0, 0.05):
        c.tau = float(tau)
        kinds += [(e["event"], e["id"]) for e in u.tick(c)]
    check("activation events in order", kinds == [("activated", "C"), ("activated", "D")],
          f"{kinds}")


# ---------------------------------------------------------------------------
# 8. end of the path, update after the end
# ---------------------------------------------------------------------------
def test_end_of_path():
    u, c = loaded(path=line(t1=20.0))                 # ends at x=10, t=20
    events = []
    for tau in np.arange(0.0, 30.0, 0.05):
        c.tau = float(tau)
        events += [e["event"] for e in u.tick(c)]
    check("end_reached emitted exactly once", events.count("end_reached") == 1)
    check("progress state end_reached", u.progress(c)["state"] == "end_reached"
          and u.progress(c)["end_reached"] is True)
    check("window past the end holds the last pose (U_d = 0)",
          np.allclose(u.window(c)[1], 10.0) and u.authored_speed(c) == 0.0)
    # append a northbound line at the last pose: heading change allowed (stationary)
    cp = c.tau + u.lead_min(c)
    t, x, y, yaw = line(t0=cp, t1=cp + 40, x0=10.0, heading=math.pi / 2)
    ans, ev, _ = u.handle(req("north", t, x, y, yaw, c=cp), c)
    check("update after the end accepted (heading not checked)", ans["accepted"] and
          ev["heading_checked"] is False, f"{ans}")
    check("end_reached cleared by the extension", u.end_reached is False and
          u.progress(c)["state"] == "following")
    # but at the very end of a MOVING approach, heading is enforced
    u, c = loaded(path=line(t1=20.0))
    c.tau = 5.0
    t, x, y, yaw = line(t0=20.0, t1=60.0, x0=10.0, heading=math.pi / 2)
    ans, _, _ = u.handle(req("north", t, x, y, yaw, c=20.0), c)
    check("junction at the end of a moving path: heading enforced",
          ans["reason"] == pu.DISCONTINUOUS_HEADING)


# ---------------------------------------------------------------------------
# 9. a refusal changes nothing; resets; determinism
# ---------------------------------------------------------------------------
def test_refusal_changes_nothing():
    u, c = loaded()
    c.tau = 5.0
    u.tick(c)
    before = snapshot(u, c)
    t, x, y, yaw = line(t0=20.0, x0=10.0)
    bad = [req("x", t, x, y + 1, yaw, c=20.0), req("x", t, x, y, yaw, c=6.0),
           req("", t, x, y, yaw, c=20.0), req("x", t, x * 100, y, yaw, c=20.0),
           req("x", t, x, y, yaw, c=500.0), req("p0", t, x, y, yaw, c=20.0)]
    for r in bad:
        ans, _, nt = u.handle(r, c)
        check(f"refused ({ans['reason']})", not ans["accepted"] and nt is None)
    for kw in (dict(ready=False), dict(override="arrow"), dict(path_source="mission")):
        c2 = ctx(tau=5.0, **kw)
        ans, _, _ = u.handle(req("y", t, x, y, yaw, c=20.0), c2)
        check(f"state refusal {ans['reason']} leaves plan", not ans["accepted"])
    check("refusals left plan/ids/hold/window untouched", snapshot(u, c) == before)
    check("refused_count counted", u.refused_count == len(bad) + 3)


def test_resets():
    u, c = loaded()
    u.handle(req("q", *line(t0=20.0, x0=10.0), c=20.0), c)
    ev = u.reset(ctx(session_mode=True, run_active=False), "run_stopped")
    check("stop clears the plan and emits 'cleared'",
          ev["event"] == "cleared" and not u.plan and not u.awaiting)
    check("idle session: state no_run", u.progress(ctx(session_mode=True,
                                                         run_active=False))["state"]
          == "no_run")
    ev = u.reset(ctx(), "run_started")
    check("new replanner run: awaiting again", u.awaiting and ev["event"] == "cleared")
    u.set_hold((0.0, 0.0, 0.0))
    ans, _, _ = u.handle(req("p0", *line()), ctx())
    check("ids are per run (p0 accepted again)", ans["accepted"])
    # a fixed-mission run with no path-update history emits nothing
    m = pu.PathUpdater()
    check("mission run reset: no event (no pathlog)",
          m.reset(ctx(path_source="mission"), "run_started") is None)
    check("mission run progress", m.progress(ctx(path_source="mission"))["state"]
          == "mission")


def test_boat_lagging():
    """tau frozen by the governor (boat >= gov_Lmax behind): the decision is
    taken against tau, never against the boat -- the boat is only reported."""
    u, c = loaded()
    c.tau = 10.0
    u.tick(c)
    t, x, y, yaw = line(t0=20.0, x0=10.0)
    far_behind = ctx(tau=10.0, boat=(-8.0, 3.0, 2.0))          # 13 m behind tau
    ans, ev, _ = u.handle(req("lag", t, x, y, yaw, c=20.0), far_behind)
    check("update while the boat lags far behind tau: accepted", ans["accepted"], f"{ans}")
    check("lagging boat pose only reported in the event",
          ev["boat"] == {"x": -8.0, "y": 3.0, "psi": 2.0})
    # tau did not move (frozen): the same lead bound applies on the next try
    t2, x2, y2, yaw2 = line(t0=10.0, x0=5.0)
    ans2, _, _ = u.handle(req("lag2", t2, x2, y2, yaw2, c=15.9), far_behind)
    check("frozen tau: lead still measured from tau", ans2["reason"] == pu.LEAD_TOO_SHORT)


def _scenario():
    u, c = loaded(path=arc(0.0, 120.0, 0.5, 0.0, 5.0, 5.0, -math.pi / 2), **MPC_SIM)
    out = []
    for k, tau in enumerate(np.arange(0.0, 60.0, 0.05)):
        c.tau = float(tau)
        out += u.tick(c)
        if k in (100, 300, 301, 700):
            cp = c.tau + u.lead_min(c) + 1.0
            x0, y0, yaw0 = u.plan.pose(cp)
            t, x, y, yaw = line(t0=cp, t1=cp + 50, x0=x0, y0=y0, heading=yaw0)
            out.append(u.handle(req(f"u{k}", t, x, y + (0.5 if k == 301 else 0.0),
                                    yaw, c=cp), c)[:2])
        out.append(tuple(np.round(np.concatenate(u.window(c)[1:]), 15)))
    out.append(u.progress(c))
    return out


def test_determinism():
    a, b = _scenario(), _scenario()
    check("same inputs -> same answers, events, windows", repr(a) == repr(b))


TESTS = [test_constants, test_from_points_equals_file, test_state_gate,
         test_request_codes, test_first_path, test_lead, test_continuity_and_splice,
         test_update_during_turn, test_back_to_back, test_end_of_path,
         test_refusal_changes_nothing, test_resets, test_boat_lagging, test_determinism]


def main():
    for test in TESTS:
        test()
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
