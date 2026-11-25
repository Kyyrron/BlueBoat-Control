#!/usr/bin/env python3
"""
ROS-free tests of the pinger filter (acceptance criteria A1) and its pieces:
pinger_params, pinger_input, legacy_pinger, pinger_filter, pinger_estimator_core,
pinger_filter_replay.

    /usr/bin/python3 test/test_pinger_filter.py

Synthetic world: a boat on a 5 m circle at 1 m/s (0.2 rad/s) or a lawnmower,
odometry at 10 Hz, Water Linked fixes polled at 4 Hz with a new fix every 0.5 s,
measured `latency` before they are received, with spec noise (1 deg bearing,
0.2 % range) and optional heavy-tailed outliers. Recorded-bag checks run only
when ~/ros2_ws/data holds bags (skipped otherwise).
"""

import csv
import glob
import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "blueboat_control", "src", "_custom_libraries"))

import numpy as np                              # noqa: E402

import legacy_pinger as lp                      # noqa: E402
import pinger_estimator_core as pec             # noqa: E402
import pinger_filter as pf                      # noqa: E402
import pinger_filter_replay as pfr              # noqa: E402
import pinger_input as pin                      # noqa: E402
import pinger_log as plog                       # noqa: E402
import pinger_params as pp                      # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    if not cond:
        print(f"FAIL  {name}  {detail}")
        if "pytest" in sys.modules:
            raise AssertionError(f"{name}  {detail}")


# ---------------------------------------------------------------------------
# synthetic world
# ---------------------------------------------------------------------------

def circle_boat(radius=5.0, speed=1.0, centre=(0.0, 0.0)):
    w = speed / radius

    def boat(t):
        a = w * t
        x = centre[0] + radius * math.cos(a)
        y = centre[1] + radius * math.sin(a)
        yaw = a + math.pi / 2
        return x, y, yaw, speed, w
    return boat


def lawnmower_boat(speed=1.0, leg=20.0, gap=4.0):
    """Straight legs joined by half circles (turn rate speed/(gap/2))."""
    r = gap / 2.0
    turn = math.pi * r
    period = 2 * (leg + turn)

    def boat(t):
        s = (speed * t) % period
        k = int((speed * t) // period)
        y0 = k * 2 * gap
        if s < leg:
            return s, y0, 0.0, speed, 0.0
        s -= leg
        if s < turn:
            a = s / r
            return leg + r * math.sin(a), y0 + r - r * math.cos(a), a, speed, speed / r
        s -= turn
        if s < leg:
            return leg - s, y0 + gap, math.pi, speed, 0.0
        s -= leg
        a = s / r
        return -r * math.sin(a), y0 + gap + r - r * math.cos(a), math.pi + a, speed, speed / r
    return boat


def make_fix_json(t, a, seq, std=None, valid=True):
    data = {"x": a[0], "y": a[1], "z": a[2], "position_valid": valid}
    if std is not None:
        data["std"] = std
    return json.dumps({"schema": pin.SCHEMA_UWGPS_FIX, "seq": seq,
                       "endpoints": {"raw": {"ok": True, "status": 200, "t_req": t, "t_resp": t, "data": data}}})


def scenario(target, boat, duration=120.0, latency=1.5, fix_dt=0.5, odom_dt=0.1, noise_deg=1.0,
             range_k=0.002, outliers=0.0, seed=1, depth=2.0, gaps=()):
    """Events (t, kind, payload) and the truth. Fixes as /uw_gps/fix JSON."""
    rng = np.random.default_rng(seed)
    ev = []
    t = 0.0
    while t <= duration:
        x, y, yaw, u, w = boat(t)
        ev.append((t, "odom", (x, y, yaw, u, 0.0, 0.0)))
        ev.append((t + 1e-4, "imu", w))
        t = round(t + odom_dt, 9)
    seq = 0
    t = 2.0
    clean = []
    while t <= duration:
        if not any(a <= t < b for a, b in gaps):
            tm = t - latency
            x, y, yaw, _, _ = boat(tm)
            px, py = target(tm)
            lx, ly = px - x, py - y
            rho = math.hypot(lx, ly)
            ux, uy = lx / rho, ly / rho
            ea = rng.normal(0, 0.1 + range_k * rho) if noise_deg > 0 else 0.0
            ec = rng.normal(0, rho * math.radians(noise_deg)) if noise_deg > 0 else 0.0
            wx = lx + ea * ux - ec * uy
            wy = ly + ea * uy + ec * ux
            if outliers and rng.random() < outliers:
                wx += rng.uniform(-20, 20)
                wy += rng.uniform(-20, 20)
            c, s = math.cos(yaw), math.sin(yaw)
            a = (c * wx + s * wy, -s * wx + c * wy, depth)
            seq += 1
            ev.append((t + 2e-4, "fixjson", make_fix_json(t, a, seq)))
            clean.append((t, tm, x + wx, y + wy))
        t = round(t + fix_dt, 9)
    ev.sort(key=lambda e: e[0])
    return ev, clean


def run_core(ev, params, source="filter"):
    run = pfr.Run("synthetic", "bag", ev)
    return pfr.replay(run, params, source, keep_bodies=True)


def P(**ov):
    return pp.load_params(overrides=ov)


# ---------------------------------------------------------------------------
# params
# ---------------------------------------------------------------------------

def test_params():
    s = pp.load_schema()
    check("schema id", s["schema"] == pp.SCHEMA_ID)
    for name, spec in s["parameters"].items():
        check(f"spec complete: {name}", all(k in spec for k in ("group", "type", "unit", "default", "description"))
              and spec["group"] in s["groups"], spec)
    for preset in pp.preset_names(s):
        p = pp.load_params(preset=preset)
        check(f"preset {preset} valid", p.preset == preset)
    # The station's Field tuning cards (MCS reads them; Control ignores them).
    here = os.path.dirname(pp.schema_path())
    for card in s.get("field_tuning") or []:
        ok = (all(n in s["parameters"] for n in card["params"])
              and os.path.isfile(os.path.join(here, card["picture"]))
              and all(card.get(k) for k in ("title", "what", "lower", "higher", "field")))
        for n, (lo, hi, scale) in (card.get("slider") or {}).items():
            spec = s["parameters"][n]
            ok = ok and n in card["params"] and lo < hi and scale in ("lin", "log") \
                and (scale == "lin" or lo > 0) and spec.get("min", lo) <= lo and hi <= spec.get("max", hi) \
                and lo <= spec["default"] <= hi
        check(f"field tuning card complete: {card.get('title')}", ok, card)
    d = pp.load_params()
    check("hash stable", d.hash == pp.load_params().hash and len(d.hash) == 16)
    check("hash changes with a value", d.hash != P(latency_s=1.2).hash)
    for bad, why in ((dict(latency_s=-1), "below the minimum"), (dict(y_sign=0), "+1 or -1"),
                     (dict(robust="magic"), "not one of"), (dict(nonsense=1), "unknown parameter"),
                     (dict(gate_hard=2.0), "gate_hard must be")):
        try:
            pp.load_params(overrides=bad)
            check(f"rejects {bad}", False)
        except pp.ParamError as exc:
            check(f"rejects {bad} with the reason", why in str(exc), str(exc))
    with tempfile.TemporaryDirectory() as tmp:
        f = os.path.join(tmp, "x.yaml")
        q = P(latency_s=1.25, robust="student_t")
        pp.write_param_yaml(f, q)
        r = pp.load_params(preset=pp.CUSTOM, yaml_path=f)
        check("YAML round trip", r.values == q.values and r.hash == q.hash)
    ov = pp.parse_overrides(["latency_s=0.8", "use_wl_std=false", "y_sign=-1"])
    check("overrides coerced", ov == {"latency_s": 0.8, "use_wl_std": False, "y_sign": -1}, ov)


# ---------------------------------------------------------------------------
# A1
# ---------------------------------------------------------------------------

def _rms_vs_truth(res, target, t_from=20.0):
    est = [(e["t"], e["x"], e["y"]) for e in res["estimates"] if e["x"] is not None and e["t"] >= t_from]
    e = np.array([math.hypot(x - target(t)[0], y - target(t)[1]) for t, x, y in est])
    return float(np.sqrt(np.mean(e ** 2))), len(e)


def test_still_target_rms():
    """A1: still target at 5-30 m, spec noise + 10 % heavy outliers: filtered RMS <= 50 % of raw."""
    for rng_m in (5.0, 12.0, 20.0, 30.0):
        def target(t, r=rng_m):
            return (r, 0.0)
        boat = circle_boat(radius=3.0, speed=0.6)
        ev, clean = scenario(target, boat, duration=180.0, outliers=0.10, seed=int(rng_m))
        raw = np.array([math.hypot(x - rng_m, y) for t, tm, x, y in clean if t >= 20.0])
        raw_rms = float(np.sqrt(np.mean(raw ** 2)))
        for preset, ov in (("slow_rov spec noise", dict(sigma_bearing_deg=1.0)),
                           ("static_target spec noise", dict(sigma_bearing_deg=1.0))):
            params = pp.load_params(preset="static_target" if "static" in preset else "slow_rov", overrides=ov)
            res = run_core(ev, params)
            f_rms, n = _rms_vs_truth(res, target)
            check(f"A1 still {rng_m:.0f} m, {preset}: RMS {f_rms:.2f} <= 0.5 x raw {raw_rms:.2f}",
                  n > 100 and f_rms <= 0.5 * raw_rms, (f_rms, raw_rms, n))
        res = run_core(ev, pp.load_params())
        f_rms, _ = _rms_vs_truth(res, target)
        check(f"A1 still {rng_m:.0f} m, defaults: RMS {f_rms:.2f} <= 0.5 x raw {raw_rms:.2f}",
              f_rms <= 0.5 * raw_rms, (f_rms, raw_rms))
        sm = res["smoothed"]
        s_e = np.array([math.hypot(s["x"] - rng_m, s["y"]) for s in sm if s["t"] >= 20.0])
        check(f"RTS smoothed RMS {np.sqrt(np.mean(s_e ** 2)):.2f} <= filtered {f_rms:.2f} ({rng_m:.0f} m)",
              np.sqrt(np.mean(s_e ** 2)) <= f_rms * 1.15)


def test_burst_rejected():
    """A1: a 3-fix burst displaced 20 m moves the estimate < 1 m."""
    def target(t):
        return (10.0, 4.0)
    ev, _ = scenario(target, circle_boat(), duration=80.0, seed=3)
    # displace three consecutive fixes after 60 s by 20 m east (world)
    out, n = [], 0
    for t, k, v in ev:
        if k == "fixjson" and t > 60.0 and n < 3:
            m = json.loads(v)
            d = m["endpoints"]["raw"]["data"]
            boat = circle_boat()
            _, _, yaw, _, _ = boat(t - 1.5)
            c, s = math.cos(yaw), math.sin(yaw)
            d["x"] += c * 20.0
            d["y"] += -s * 20.0
            v = json.dumps(m)
            n += 1
        out.append((t, k, v))
    for name, params in (("defaults", pp.load_params()), ("spec noise", P(sigma_bearing_deg=1.0))):
        res = run_core(out, params)
        before = [e for e in res["estimates"] if 59.0 <= e["t"] <= 60.0][-1]
        after = [e for e in res["estimates"] if 61.0 <= e["t"] <= 63.0][-1]
        moved = math.hypot(after["x"] - before["x"], after["y"] - before["y"])
        check(f"A1 burst of 3 at 20 m moves the estimate {moved:.2f} m < 1 m ({name})", moved < 1.0, moved)


def test_relocation_reacquired():
    """A1: a real 15 m relocation is re-acquired within reinit_n + 1 fixes."""
    def target(t):
        return (8.0, 2.0) if t < 60.0 else (8.0, 17.0)
    ev, _ = scenario(target, circle_boat(), duration=100.0, seed=4)
    for name, params in (("defaults", pp.load_params()), ("spec noise", P(sigma_bearing_deg=1.0)),
                         ("static_target", pp.load_params(preset="static_target"))):
        res = run_core(ev, params)
        after = [f for f in res["fixes"] if f["t_meas"] >= 60.0 and f["decision"] != "duplicate"]
        k = next((i for i, f in enumerate(after)
                  if f.get("posterior") and math.hypot(f["posterior"]["x"] - 8.0, f["posterior"]["y"] - 17.0) < 2.0),
                 None)
        n = params["reinit_n"]
        check(f"A1 relocation re-acquired after {None if k is None else k + 1} fixes <= {n + 1} ({name})",
              k is not None and k + 1 <= n + 1, k)
        check(f"reacquired event reported ({name})",
              any(f.get("event") == "reacquired" for f in after[:n + 1]) or
              (k is not None and after[k]["decision"] in ("accepted", "downweighted")))


def test_latency_compensation():
    """A1: still target at 20 m, 1.5 s latency, boat turning at 0.2 rad/s: the
    compensated world dispersion is < 0.5 m, uncompensated > 4 m."""
    def target(t):
        return (20.0, 0.0)

    def boat(t):                                  # turning in place at 0.2 rad/s
        return 0.0, 0.0, 0.2 * t, 0.0, 0.2
    ev, _ = scenario(target, boat, duration=100.0, latency=1.5, seed=5)
    run = pfr.Run("lat", "bag", ev)
    for lat, cmp_, lim in ((1.5, "<", 0.5), (0.0, ">", 4.0)):
        pts = pfr.world_fixes(run, P(latency_s=lat).values)
        d = np.hypot(pts[:, 1] - 20.0, pts[:, 2])
        disp = float(np.sqrt(np.mean(d ** 2)))
        ok = disp < lim if cmp_ == "<" else disp > lim
        check(f"A1 latency {lat} s: world dispersion {disp:.2f} m {cmp_} {lim}", ok, disp)


def test_duplicates_and_zeros_change_nothing():
    """A1: duplicate fixes and zero packets change nothing."""
    def target(t):
        return (12.0, -3.0)
    ev, _ = scenario(target, lawnmower_boat(), duration=90.0, outliers=0.05, seed=6)
    base = run_core(ev, pp.load_params())
    noisy = []
    for t, k, v in ev:
        noisy.append((t, k, v))
        if k == "fixjson":
            noisy.append((t + 0.1, k, v))                                     # exact duplicate
            m = json.loads(v)
            m["endpoints"]["raw"]["ok"] = False
            m["endpoints"]["raw"]["data"] = None
            m["endpoints"]["raw"]["error"] = "timeout"
            noisy.append((t + 0.2, k, json.dumps(m)))                         # failed request
    res = run_core(noisy, pp.load_params())
    a = [(f["t_rx"], f["decision"], f.get("posterior")) for f in base["fixes"]]
    b = [(f["t_rx"], f["decision"], f.get("posterior")) for f in res["fixes"]
         if f["decision"] not in ("duplicate", "invalid")]
    check("duplicates / failed requests: identical updates", a == b, (len(a), len(b)))
    xa = base["core"].filter.x
    xb = res["core"].filter.x
    check("duplicates / failed requests: identical final state", np.array_equal(xa, xb) and
          np.array_equal(base["core"].filter.P, res["core"].filter.P))
    dec = {}
    for f in res["fixes"]:
        dec[f["decision"]] = dec.get(f["decision"], 0) + 1
    check("duplicates and invalid counted", dec.get("duplicate", 0) > 100 and dec.get("invalid", 0) > 100, dec)
    # legacy packets: a zero packet is invalid, a repeated packet a duplicate
    f0 = pin.fix_from_packet(1.0, [0.0] * 19, "raw")
    check("zero packet is invalid", f0.invalid == "zero_packet")


def test_still_between_fixes():
    """A1/R2: no new fix for 4 s while the boat moves at 1 m/s and turns: the
    world estimate moves exactly 0 m, and every tick b = R(psi)^T (p_hat - p)."""
    def target(t):
        return (6.0, 9.0)
    boat = circle_boat(radius=5.0, speed=1.0)
    ev, _ = scenario(target, boat, duration=60.0, gaps=((40.0, 44.0),), seed=7)
    res = run_core(ev, pp.load_params())
    est = [e for e in res["estimates"] if e["x"] is not None and e["t"] < 40.0][-1]
    worst_move, worst_b = 0.0, 0.0
    for t, body, leg, filt in res["bodies"]:
        if 40.0 + 0.01 < t < 41.5:          # last fix received before 40.0 s (+ odom ticks after)
            x, y, yaw, _, _ = boat(t)
            dx, dy = est["x"] - x, est["y"] - y
            c, s = math.cos(yaw), math.sin(yaw)
            want = (c * dx + s * dy, -s * dx + c * dy)
            worst_b = max(worst_b, abs(filt[0] - want[0]), abs(filt[1] - want[1]))
    for e in res["estimates"]:
        if 40.0 <= e["t"] < 41.5 and e["x"] is not None:
            worst_move = max(worst_move, math.hypot(e["x"] - est["x"], e["y"] - est["y"]))
    # also: the core's world estimate is unchanged at every odom tick in the gap
    core = pec.EstimatorCore(pp.load_params())
    moves = []
    p0 = None
    for t, k, v in ev:
        if k == "odom":
            core.on_odom(t, *v)
            if 40.5 < t < 43.5:
                e = core.estimate(t)
                if p0 is None:
                    p0 = (e["x"], e["y"])
                moves.append(math.hypot(e["x"] - p0[0], e["y"] - p0[1]))
                b = core.filter.body_at(v[0], v[1], v[2])
                c, s = math.cos(v[2]), math.sin(v[2])
                dx, dy = e["x"] - v[0], e["y"] - v[1]
                worst_b = max(worst_b, abs(b[0] - (c * dx + s * dy)), abs(b[1] - (-s * dx + c * dy)))
        elif k == "imu":
            core.on_imu(t, v)
        elif k == "fixjson":
            core.on_fix_json(t, v)
    check(f"A1 no fix for 4 s: world estimate moved {max(moves):.3g} m (exactly 0)", max(moves) == 0.0, max(moves))
    check(f"A1 body = R^T(p_hat - p) at every tick (max err {worst_b:.2e} m)", worst_b < 1e-9, worst_b)
    check("estimate published during the gap does not move", worst_move == 0.0, worst_move)


def test_speed_cap():
    """A1: the output speed never exceeds v_max (fast target, too)."""
    for v_target in (0.2, 2.0):
        def target(t, v=v_target):
            return (5.0 + v * t, 3.0)
        ev, _ = scenario(target, circle_boat(radius=4.0, speed=1.0), duration=60.0, seed=8)
        for name, params in (("defaults", pp.load_params()), ("v_max 0.3", P(v_max=0.3))):
            res = run_core(ev, params)
            vmax = max((math.hypot(e["vx"], e["vy"]) for e in res["estimates"] if e["vx"] is not None),
                       default=0.0)
            check(f"A1 speed cap ({name}, target {v_target} m/s): {vmax:.3f} <= {params['v_max']}",
                  vmax <= params["v_max"] + 1e-12, vmax)


def test_online_equals_offline():
    """A1: online = offline bit for bit on the same input; and a re-run from the
    per-fix log reproduces the live estimate."""
    def target(t):
        return (9.0 + 0.1 * t, -4.0)
    ev, _ = scenario(target, lawnmower_boat(), duration=80.0, outliers=0.1, seed=9)
    params = pp.load_params()
    # "online": callbacks in arrival order, as pinger_estimator does
    core = pec.EstimatorCore(params)
    online = []
    for t, k, v in ev:
        if k == "odom":
            core.on_odom(t, *v)
        elif k == "imu":
            core.on_imu(t, v)
        elif k == "fixjson":
            for r in core.on_fix_json(t, v):
                online.append((json.dumps(r, sort_keys=True), json.dumps(core.estimate(t), sort_keys=True)))
    off = run_core(ev, params)
    offline = [(json.dumps(r, sort_keys=True), None) for r in off["fixes"]]
    check("A1 online = offline: fix records identical", [a for a, _ in online] == [a for a, _ in offline])
    # per-fix log round trip
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "x-pingerlog.csv")
        log = plog.PingerLog(path)
        log.set_config(0.0, params.values, params.hash)
        for r in off["fixes"]:
            log.write_fix(r)
        log.close()
        rerun = pfr.replay(pfr.load_pingerlog(path), pp.load_params(overrides={"latency_s": 0.3}))
    a = [(f["decision"], f.get("posterior")) for f in off["fixes"]]
    b = [(f["decision"], f.get("posterior")) for f in rerun["fixes"]]
    check("per-fix log re-run is bit-identical", a == b)
    check("RTS of the re-run identical", off["smoothed"] == rerun["smoothed"])


class _V11Reference:
    """robot_interface v1.1 (44ec6aa) pinger statements, verbatim, as the reference."""

    def __init__(self):
        self.prev_time = None
        self.pinger_coordinates = np.zeros(3)
        self.angular_velocity = None
        self.fixed_pinger = False

    def odom_callback(self, t, msg_twist):
        if self.prev_time is None:
            self.prev_time = t
            return None
        dt = t - self.prev_time
        self.prev_time = t
        x_t, y_t, z_t = msg_twist
        self.vel = np.array([x_t, y_t, z_t])
        av = self.angular_velocity
        if self.fixed_pinger:
            return None
        if av is not None and not all(self.pinger_coordinates == np.zeros(3)):
            omega = np.array([0.0, 0.0, av])
            p = self.pinger_coordinates
            self.pinger_coordinates -= (self.vel + np.cross(omega, p)) * dt
        return self.pinger_coordinates.copy()

    def uw_gps_callback(self, data):
        t_x, t_y, t_z = data[16], data[17], data[18]
        self.pinger_coordinates = np.array([t_x, t_y, t_z])


def test_legacy_bit_identical():
    """A3 (core): legacy_pinger = robot_interface v1.1 arithmetic, bit for bit."""
    rng = np.random.default_rng(11)
    worst = 0.0
    n = 0
    for trial in range(20):
        ref, new = _V11Reference(), lp.LegacyPinger()
        t = 1000.0
        for _ in range(500):
            t += float(rng.uniform(0.01, 0.4))
            k = rng.integers(0, 10)
            if k < 6:
                tw = tuple(float(v) for v in rng.normal(0, 0.7, 3))
                a = ref.odom_callback(t, tw)
                b = new.on_odom(t, tw)
                if (a is None) != (b is None) or (a is not None and not np.array_equal(a, b)):
                    worst = math.inf
                n += 1
            elif k < 8:
                w = float(rng.normal(0, 0.3))
                ref.angular_velocity = w
                new.on_imu(w)
            else:
                pk = [float(np.float32(v)) for v in rng.normal(0, 8, 19)]
                if rng.random() < 0.1:
                    pk[16:19] = [0.0, 0.0, 0.0]
                ref.uw_gps_callback(pk)
                new.on_ugps(pk)
    check(f"legacy == v1.1 robot_interface on {n} random odom events (bit for bit)", worst == 0.0)


def test_legacy_on_recorded_bags():
    """A3: legacy recomputed from recorded inputs matches the recorded vector
    (to the recorder's event-order noise). Skipped without data."""
    bags = sorted(glob.glob(os.path.expanduser("~/ros2_ws/data/Robot_data/**/*-bag"), recursive=True))
    done = 0
    for bag in bags[:40]:
        run = pfr.load_bag(bag)
        if not any(k == "packet" and any(v[16:19]) for _, k, v in run.events) or not run.recorded_body:
            continue
        res = pfr.replay(run, pp.load_params(), "legacy", keep_bodies=True)
        rec = np.array([b for _, b in run.recorded_body])
        ours = np.array([b[1] for b in res["bodies"] if b[1] is not None])
        n = min(len(rec), len(ours))
        if n < 50:
            continue
        diffs = []
        for o in range(-3, 4):
            a, b = ours[max(0, o):], rec[max(0, -o):]
            m = min(len(a), len(b)) - 4
            diffs.append(np.median(np.abs(a[:m] - b[:m])[:, :2].max(axis=1)))
        best = min(diffs)
        check(f"legacy reproduces recorded {os.path.basename(bag)[:30]}: median {best * 1000:.2f} mm",
              best < 1e-3, best)
        done += 1
    if not done:
        print("SKIP  recorded-bag legacy check (no bags with pinger data)")


def test_init_and_status():
    def target(t):
        return (7.0, 7.0)
    ev, _ = scenario(target, circle_boat(), duration=40.0, seed=12, gaps=((20.0, 41.0),))
    core = pec.EstimatorCore(pp.load_params())
    statuses = {}
    for t, k, v in ev:
        if k == "odom":
            core.on_odom(t, *v)
            statuses.setdefault(core.estimate(t)["status"], t)
        elif k == "imu":
            core.on_imu(t, v)
        else:
            core.on_fix_json(t, v)
    check("status sequence no_data -> initialising -> tracking -> coasting -> stale",
          list(statuses) == ["no_data", "initialising", "tracking", "coasting", "stale"], statuses)
    p = core.values
    check("coasting after coast_after_s", abs(statuses["coasting"] - (statuses["tracking"] + 0) - 0) >= 0)
    e = core.estimate(40.0)
    check("stale estimate keeps its position", e["x"] is not None and e["status"] == "stale")
    check("fix age reported", e["fix_age"] > p["stale_timeout_s"])


def test_mapping():
    f = pf.PingerFilter(P(y_sign=-1, mount_yaw_deg=90.0, lever_x_m=0.5, lever_y_m=0.0, z_sign=1).values)
    bx, by, d, rho = f.map_fix((3.0, 4.0, 2.0))
    # y flipped -> (3, -4); rotated +90 -> (4, 3); + lever (0.5, 0) -> (4.5, 3)
    check("mapping: y_sign, mount yaw, lever arm", abs(bx - 4.5) < 1e-12 and abs(by - 3.0) < 1e-12
          and d == 2.0 and abs(rho - 5.0) < 1e-12, (bx, by, d, rho))


def test_robust_modes_run():
    def target(t):
        return (10.0, 0.0)
    ev, _ = scenario(target, circle_boat(), duration=60.0, outliers=0.2, seed=13)
    for mode in ("huber", "student_t", "chi2", "none"):
        res = run_core(ev, P(robust=mode))
        rms, n = _rms_vs_truth(res, target)
        check(f"robust={mode} runs and tracks (RMS {rms:.2f} m)", n > 50 and np.isfinite(rms), rms)


def test_legacy_source_output():
    """pinger_source 'legacy': the body topic carries exactly the legacy vector,
    None (no publish) on the first odom like robot_interface."""
    def target(t):
        return (10.0, 5.0)
    ev, _ = scenario(target, circle_boat(), duration=20.0, seed=14)
    res = run_core(ev, pp.load_params(), "legacy")
    first = res["bodies"][0]
    check("legacy source: nothing published on the first odom", first[1] is None and first[2] is None)
    check("legacy source: body == legacy_body", all(b == lb for b, lb, _f in [(x[1], x[2], x[3]) for x in
                                                                                res["bodies"][1:]]))
    res = run_core(ev, pp.load_params(), "filter")
    check("filter source: zeros before initialisation",
          res["bodies"][0][1] == [0.0, 0.0, 0.0] and res["bodies"][0][3] is None)


def test_snapshot_midstream_log():
    """Review #1: a per-fix log that starts mid-session at a filter snapshot
    re-runs the live filter bit for bit (state rows, pinger_log)."""
    def target(t):
        return (9.0 + 0.05 * t, -4.0)
    ev, _ = scenario(target, lawnmower_boat(), duration=90.0, outliers=0.1, seed=21)
    params = pp.load_params()
    core = pec.EstimatorCore(params, keep_steps=False)
    live = []
    snap_at = 45.0
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "m-pingerlog.csv")
        log = None
        for t, k, v in ev:
            if log is None and t >= snap_at:
                log = plog.PingerLog(path)
                log.set_config(t, params.values, params.hash)
                log.write_fix(core.snapshot_record(t, "test"))
            if k == "odom":
                core.on_odom(t, *v)
            elif k == "imu":
                core.on_imu(t, v)
            else:
                for r in core.on_fix_json(t, v):
                    if log is not None:
                        log.write_fix(json.loads(json.dumps(r)))
                        live.append((r["decision"], r.get("posterior")))
        log.close()
        rerun = pfr.replay(pfr.load_pingerlog(path), pp.load_params(overrides={"latency_s": 0.2}))
    got = [(f["decision"], f.get("posterior")) for f in rerun["fixes"]]
    check(f"mid-session per-fix log re-runs bit for bit ({len(live)} fixes)", got == live and len(live) > 50)
    check("its RTS smoother runs from the snapshot", len(rerun["smoothed"]) > 20)


def test_malformed_inputs():
    """Review #7: non-finite packets, malformed endpoints, inf parameters."""
    nan = float("nan")
    f = pin.fix_from_packet(1.0, [0.0] * 7 + [nan, 1.0, 2.0] + [0.0] * 9, "raw")
    check("NaN packet -> invalid non_finite", f.invalid == "non_finite")
    core = pec.EstimatorCore(pp.load_params())
    core.on_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    core.on_odom(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    pkt = [0.0] * 16 + [nan, 1.0, 1.0]
    pkt[7:10] = [nan, 1.0, 1.0]
    res = core.on_packet(0.2, pkt)
    out = core.on_odom(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    check("a NaN packet never reaches the legacy vector",
          all(math.isfinite(v) for v in (out["legacy_body"] or [0.0])))
    check("a NaN packet is counted invalid", res and res[0]["decision"] == "invalid")
    msg = {"schema": pin.SCHEMA_UWGPS_FIX, "seq": 1, "endpoints": {"raw": "garbage"}}
    fx = pin.fixes_from_json(1.0, msg)
    check("a non-dict endpoint is an invalid fix, not an exception", fx["raw"].invalid == "malformed_endpoint")
    fx = pin.fixes_from_json(1.0, {"schema": pin.SCHEMA_UWGPS_FIX, "endpoints": None})
    check("no endpoints: nothing", fx["raw"] is None and fx["filtered"] is None)
    try:
        pp.load_params(overrides={"init_n": float("inf")})
        check("inf int parameter refused", False)
    except pp.ParamError:
        check("inf int parameter refused with a ParamError", True)


def test_publish_delay_correction():
    """Review #5: a raw fix published after a full cycle's later requests gets
    its receive time moved back by that delay (uwgps_log's own clock)."""
    msg = {"schema": pin.SCHEMA_UWGPS_FIX, "seq": 3, "t_end": 100.9,
           "endpoints": {"raw": {"ok": True, "t_req": 100.0, "t_resp": 100.05,
                                 "data": {"x": 3.0, "y": 1.0, "z": 0.5}},
                         "filtered": {"ok": True, "t_req": 100.05, "t_resp": 100.9,
                                      "data": {"x": 3.0, "y": 1.0, "z": 0.5}}}}
    fx = pin.fixes_from_json(500.0, msg)
    check(f"raw fix receive time corrected by the later requests ({500.0 - fx['raw'].t_rx:.2f} s)",
          abs(fx["raw"].t_rx - (500.0 - 0.85)) < 1e-9)
    check("filtered fix (last request) not corrected", fx["filtered"].t_rx == 500.0)
    msg["t_end"] = 1e9
    check("implausible delay ignored", pin.fixes_from_json(500.0, msg)["raw"].t_rx == 500.0)


def test_pose_buffer_guards():
    """Review suspicions: a clock stepping back restarts the buffer; no pose is
    interpolated across a long odometry gap."""
    b = pf.PoseBuffer(30.0)
    for k in range(10):
        b.add(1000.0 + 0.1 * k, k, 0.0, 0.0)
    b.add(10.0, 5.0, 5.0, 0.0)                       # system time set back by ~990 s
    b.add(10.1, 5.1, 5.0, 0.0)
    check("clock step back: the buffer restarts and keeps working",
          b.at(10.05, 0.5) is not None and b.at(10.05, 0.5)[1] == 5.0)
    g = pf.PoseBuffer(30.0)
    g.add(0.0, 0.0, 0.0, 0.0)
    g.add(5.0, 10.0, 0.0, 0.0)
    check("a fix inside a 5 s odometry gap gets no pose (max_gap 2 s)", g.at(2.5, 0.5, 2.0) is None)
    check("the same with no gap limit interpolates", g.at(2.5, 0.5)[0] == 5.0)


def test_legacy_source_status():
    """Review #3: with pinger_source legacy the estimate's source_status is the
    legacy input's freshness, independent of the filter's status."""
    core = pec.EstimatorCore(P(require_position_valid=True, min_receivers=4), "legacy")
    core.on_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    pkt = [0.0] * 7 + [5.0, 1.0, 0.5] + [0.0] * 6 + [5.0, 1.0, 0.5]
    core.on_packet(1.0, pkt)
    e = core.estimate(1.5)
    check("legacy source: source_status tracking from the legacy packets",
          e["source_status"] == "tracking" and e["filter_status"] == e["status"])
    check("legacy source: stale when the packets stop", core.estimate(30.0)["source_status"] == "stale")
    f = pec.EstimatorCore(pp.load_params(), "filter")
    check("filter source: source_status is the filter's", f.estimate(0.0)["source_status"] == "no_data")


def test_param_file_preset():
    """Review suspicion: a saved set builds on the preset it names (as MCS does)."""
    with tempfile.TemporaryDirectory() as tmp:
        f = os.path.join(tmp, "s.yaml")
        with open(f, "w") as fh:
            fh.write("schema: blueboat_pinger_filter/1\npreset: static_target\nparams:\n  latency_s: 1.1\n")
        p = pp.load_params(preset="slow_rov", yaml_path=f)
        check("partial file over its own preset (static_target)", p["model"] == "static" and p["latency_s"] == 1.1)


def test_pinger_mode_of():
    """The one pinger-mode resolver master_control and robot_interface share
    (launch parameter, configure, run_state): mode wins, else the use_pinger alias."""
    import robot_log_schema as rls
    cases = [(("follow", False), "follow"), ((" LOG ", True), "log"), (("off", True), "off"),
             (("", True), "follow"), (("", False), "off"), ((None, None), "off"),
             ((None, 1), "follow"), ((0, True), "follow")]
    check("pinger_mode_of: mode wins, else the use_pinger alias",
          all(rls.pinger_mode_of(*args) == want for args, want in cases))
    refused = []
    for bad in ("chase", 5, ["follow"]):        # a non-string mode is refused, never crashes
        try:
            rls.pinger_mode_of(bad, True)
        except ValueError:
            refused.append(bad)
    check("pinger_mode_of: an unknown (or non-string) mode is a ValueError", len(refused) == 3, refused)


TESTS = [test_params, test_mapping, test_still_target_rms, test_burst_rejected, test_relocation_reacquired,
         test_latency_compensation, test_duplicates_and_zeros_change_nothing, test_still_between_fixes,
         test_speed_cap, test_online_equals_offline, test_legacy_bit_identical, test_legacy_source_output,
         test_init_and_status, test_robust_modes_run, test_legacy_on_recorded_bags,
         test_snapshot_midstream_log, test_malformed_inputs, test_publish_delay_correction,
         test_pose_buffer_guards, test_legacy_source_status, test_param_file_preset,
         test_pinger_mode_of]


def main():
    for test in TESTS:
        test()
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
