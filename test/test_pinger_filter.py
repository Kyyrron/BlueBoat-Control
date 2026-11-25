#!/usr/bin/env python3
"""
The pinger filter, its parameters and the legacy pinger, ROS-free:

    /usr/bin/python3 test/test_pinger_filter.py

Synthetic scenes: a boat on a circle / lawnmower, a pinger still or moving,
Water Linked fixes measured `latency` s before they arrive, with bearing and
range noise and optional wild fixes. Exit 0 when every check passes.
"""

import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "blueboat_control", "src", "_custom_libraries"))

import numpy as np                              # noqa: E402

import pinger_filter as pf                      # noqa: E402
import pinger_params as pp                      # noqa: E402
import robot_log_schema as rls                  # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    if not cond:
        print(f"FAIL  {name}  {detail}")
        if "pytest" in sys.modules:
            raise AssertionError(f"{name}  {detail}")


# ---------------------------------------------------------------------------
# synthetic scenes
# ---------------------------------------------------------------------------

def circle_boat(radius=5.0, speed=1.0):
    w = speed / radius

    def boat(t):
        a = w * t
        return radius * math.cos(a), radius * math.sin(a), a + math.pi / 2, speed, w
    return boat


def lawnmower_boat(speed=1.0, leg=20.0, gap=4.0):
    """Straight legs joined by half circles."""
    r = gap / 2.0
    turn = math.pi * r
    period = 2 * (leg + turn)

    def boat(t):
        s = (speed * t) % period
        y0 = int((speed * t) // period) * 2 * gap
        if s < leg:
            return s, y0, 0.0, speed, 0.0
        s -= leg
        if s < turn:
            a = s / r
            return leg + r * math.sin(a), y0 + r - r * math.cos(a), a, speed, speed / r
        s -= turn
        if s < leg:
            return leg - s, y0 + gap, math.pi, speed, 0.0
        a = (s - leg) / r
        return -r * math.sin(a), y0 + gap + r - r * math.cos(a), math.pi + a, speed, speed / r
    return boat


def fix_json(t, a, seq):
    data = {"x": a[0], "y": a[1], "z": a[2], "position_valid": True}
    return json.dumps({"schema": pf.SCHEMA_UWGPS_FIX, "seq": seq,
                       "endpoints": {"raw": {"ok": True, "status": 200, "t_req": t, "t_resp": t,
                                             "data": data}}})


def scenario(target, boat, duration=120.0, latency=1.5, fix_dt=0.5, odom_dt=0.1, noise_deg=1.0,
             outliers=0.0, seed=1, gaps=()):
    """Events (t, 'odom' | 'fix', payload) and the clean world fixes."""
    rng = np.random.default_rng(seed)
    ev = []
    t = 0.0
    while t <= duration:
        x, y, yaw, _, _ = boat(t)
        ev.append((t, "odom", (x, y, yaw)))
        t = round(t + odom_dt, 9)
    seq, t, clean = 0, 2.0, []
    while t <= duration:
        if not any(a <= t < b for a, b in gaps):
            tm = t - latency
            x, y, yaw, _, _ = boat(tm)
            px, py = target(tm)
            lx, ly = px - x, py - y
            rho = math.hypot(lx, ly)
            ux, uy = lx / rho, ly / rho
            ea = rng.normal(0, 0.1 + 0.002 * rho)
            ec = rng.normal(0, rho * math.radians(noise_deg))
            wx, wy = lx + ea * ux - ec * uy, ly + ea * uy + ec * ux
            if outliers and rng.random() < outliers:
                wx += rng.uniform(-20, 20)
                wy += rng.uniform(-20, 20)
            c, s = math.cos(yaw), math.sin(yaw)
            seq += 1
            ev.append((t + 2e-4, "fix", fix_json(t, (c * wx + s * wy, -s * wx + c * wy, 2.0), seq)))
            clean.append((t, x + wx, y + wy))
        t = round(t + fix_dt, 9)
    ev.sort(key=lambda e: e[0])
    return ev, clean


def run(ev, params, filt=None):
    """Feed events to a PingerFilter as pinger_estimator does: (filter, fix
    records, estimates after each non-duplicate fix, body at every odom)."""
    f = filt or pf.PingerFilter(params.values)
    fixes, est, bodies = [], [], []
    for t, kind, v in ev:
        if kind == "odom":
            f.add_pose(t, *v)
            bodies.append((t, v, f.body_at(*v)))
        else:
            fix = pf.fix_from_json(t, v, params["source"])
            if fix is not None:
                r = f.add_fix(fix)
                fixes.append(r)
                if r["decision"] != "duplicate":
                    est.append(f.estimate(t))
    return f, fixes, est, bodies


def P(preset=pp.DEFAULT_PRESET, **ov):
    return pp.load_params(preset=preset, overrides=ov)


def rms_vs_truth(est, target, t_from=20.0):
    e = [math.hypot(x["x"] - target(x["t"])[0], x["y"] - target(x["t"])[1])
         for x in est if x["x"] is not None and x["t"] >= t_from]
    return float(np.sqrt(np.mean(np.square(e)))), len(e)


# ---------------------------------------------------------------------------
# parameters and the tuning file
# ---------------------------------------------------------------------------

def test_params():
    s = pp.load_schema()
    for name, spec in s["parameters"].items():
        check(f"spec complete: {name}", all(k in spec for k in ("group", "type", "unit", "default", "description"))
              and spec["group"] in s["groups"], spec)
    for preset in s["presets"]:
        check(f"preset {preset} valid", P(preset).preset == preset)
    here = os.path.dirname(pp.schema_path())
    for card in s.get("field_tuning") or []:      # the station's Field tuning cards
        ok = (all(n in s["parameters"] for n in card["params"])
              and os.path.isfile(os.path.join(here, card["picture"]))
              and all(card.get(k) for k in ("title", "what", "lower", "higher", "field")))
        for n, (lo, hi, scale) in (card.get("slider") or {}).items():
            spec = s["parameters"][n]
            ok = ok and lo < hi and scale in ("lin", "log") and (scale == "lin" or lo > 0) \
                and spec.get("min", lo) <= lo and hi <= spec.get("max", hi) and lo <= spec["default"] <= hi
        check(f"field tuning card complete: {card.get('title')}", ok, card)
    d = P()
    check("hash stable, 16 hex", d.hash == P().hash and len(d.hash) == 16)
    check("hash changes with a value", d.hash != P(latency_s=1.2).hash)
    for bad, why in ((dict(latency_s=-1), "below the minimum"), (dict(y_sign=0), "+1 or -1"),
                     (dict(model="magic"), "not one of"), (dict(nonsense=1), "unknown parameter"),
                     (dict(gate_hard=2.0), "gate_hard must be"), (dict(init_n=float("inf")), "not a int")):
        try:
            P(**bad)
            check(f"rejects {bad}", False)
        except pp.ParamError as exc:
            check(f"rejects {bad} with the reason", why in str(exc), str(exc))


def test_tuning_file():
    with tempfile.TemporaryDirectory() as tmp:
        f = os.path.join(tmp, "sub", "tuning.yaml")
        pp.write_tuning_template(f, P())
        text = open(f).read()
        check("template lists every parameter with its description",
              all(f"# {n}: " in text and spec["description"] in text
                  for n, spec in pp.load_schema()["parameters"].items()))
        check("template, all commented out: the preset's values",
              pp.load_params(yaml_path=f).values == P().values)
        open(f, "w").write(text.replace("  # latency_s: 1.5 ", "  latency_s: 0.9 "))
        check("one uncommented line changes that value only",
              pp.load_params(yaml_path=f).values == P(latency_s=0.9).values)
        try:
            pp.write_tuning_template(f, P())
            check("template never overwrites", False)
        except FileExistsError:
            check("template never overwrites", True)
        g = os.path.join(tmp, "s.yaml")
        open(g, "w").write("preset: static_target\nparams:\n  latency_s: 1.1\n")
        p = pp.load_params(preset="slow_rov", yaml_path=g)
        check("a tuning file builds on the preset it names", p["model"] == "static" and p["latency_s"] == 1.1)


# ---------------------------------------------------------------------------
# the filter
# ---------------------------------------------------------------------------

def test_still_target_rms():
    """Still target at 5-30 m, 10 % wild fixes: filtered RMS <= half the raw RMS."""
    for rng_m in (5.0, 12.0, 20.0, 30.0):
        def target(t, r=rng_m):
            return (r, 0.0)
        ev, clean = scenario(target, circle_boat(radius=3.0, speed=0.6), duration=180.0,
                             outliers=0.10, seed=int(rng_m))
        raw = np.array([math.hypot(x - rng_m, y) for t, x, y in clean if t >= 20.0])
        raw_rms = float(np.sqrt(np.mean(raw ** 2)))
        for name, params in (("defaults", P()), ("spec noise", P(sigma_bearing_deg=1.0)),
                             ("static_target", P("static_target", sigma_bearing_deg=1.0))):
            f_rms, n = rms_vs_truth(run(ev, params)[2], target)
            check(f"still {rng_m:.0f} m, {name}: RMS {f_rms:.2f} <= 0.5 x raw {raw_rms:.2f}",
                  n > 100 and f_rms <= 0.5 * raw_rms, (f_rms, raw_rms, n))


def test_burst_rejected():
    """A burst of 3 fixes displaced 20 m moves the estimate < 1 m."""
    def target(t):
        return (10.0, 4.0)
    ev, _ = scenario(target, circle_boat(), duration=80.0, seed=3)
    out, n = [], 0
    for t, k, v in ev:
        if k == "fix" and t > 60.0 and n < 3:
            m = json.loads(v)
            d = m["endpoints"]["raw"]["data"]
            yaw = circle_boat()(t - 1.5)[2]
            d["x"] += math.cos(yaw) * 20.0
            d["y"] += -math.sin(yaw) * 20.0
            v, n = json.dumps(m), n + 1
        out.append((t, k, v))
    for name, params in (("defaults", P()), ("spec noise", P(sigma_bearing_deg=1.0))):
        est = run(out, params)[2]
        before = [e for e in est if 59.0 <= e["t"] <= 60.0][-1]
        after = [e for e in est if 61.0 <= e["t"] <= 63.0][-1]
        moved = math.hypot(after["x"] - before["x"], after["y"] - before["y"])
        check(f"burst of 3 at 20 m moves the estimate {moved:.2f} m < 1 m ({name})", moved < 1.0, moved)


def test_relocation_reacquired():
    """A real 15 m move is re-acquired within reinit_n + 1 fixes."""
    def target(t):
        return (8.0, 2.0) if t < 60.0 else (8.0, 17.0)
    ev, _ = scenario(target, circle_boat(), duration=100.0, seed=4)
    for name, params in (("defaults", P()), ("spec noise", P(sigma_bearing_deg=1.0)),
                         ("static_target", P("static_target"))):
        after = [f for f in run(ev, params)[1] if f["t_meas"] >= 60.0 and f["decision"] != "duplicate"]
        k = next((i for i, f in enumerate(after) if f["posterior"]
                  and math.hypot(f["posterior"]["x"] - 8.0, f["posterior"]["y"] - 17.0) < 2.0), None)
        check(f"relocation re-acquired after {None if k is None else k + 1} fixes ({name})",
              k is not None and k + 1 <= params["reinit_n"] + 1, k)


def test_latency_compensation():
    """Still target at 20 m, boat turning on the spot, fixes 1.5 s late: the
    world fixes stay together only with the right latency."""
    def target(t):
        return (20.0, 0.0)

    def boat(t):
        return 0.0, 0.0, 0.2 * t, 0.0, 0.2
    ev, _ = scenario(target, boat, duration=100.0, latency=1.5, seed=5)
    for lat, ok in ((1.5, lambda d: d < 0.5), (0.0, lambda d: d > 4.0)):
        w = np.array([f["world"] for f in run(ev, P(latency_s=lat))[1] if f["world"]])
        disp = float(np.sqrt(np.mean(np.hypot(w[:, 0] - 20.0, w[:, 1]) ** 2)))
        check(f"latency {lat} s: world dispersion {disp:.2f} m", ok(disp), disp)


def test_duplicates_and_failed_requests_change_nothing():
    def target(t):
        return (12.0, -3.0)
    ev, _ = scenario(target, lawnmower_boat(), duration=90.0, outliers=0.05, seed=6)
    noisy = []
    for t, k, v in ev:
        noisy.append((t, k, v))
        if k == "fix":
            noisy.append((t + 0.1, k, v))                             # exact duplicate
            m = json.loads(v)
            m["endpoints"]["raw"].update(ok=False, data=None, error="timeout")
            noisy.append((t + 0.2, k, json.dumps(m)))                 # failed request
    fa, a, _, _ = run(ev, P())
    fb, b, _, _ = run(noisy, P())
    keep = [(f["t_rx"], f["decision"], f["posterior"]) for f in b if f["decision"] not in ("duplicate", "invalid")]
    check("duplicates / failed requests: identical updates",
          [(f["t_rx"], f["decision"], f["posterior"]) for f in a] == keep)
    check("... and identical final state", np.array_equal(fa.x, fb.x) and np.array_equal(fa.P, fb.P))
    dec = [f["decision"] for f in b]
    check("duplicates and failed requests counted", dec.count("duplicate") > 100 and dec.count("invalid") > 100)


def test_still_between_fixes():
    """No fix for 4 s while the boat moves and turns: the world estimate does not
    move, and the body vector is R(psi)^T (p_hat - p) at every odom."""
    def target(t):
        return (6.0, 9.0)
    ev, _ = scenario(target, circle_boat(), duration=60.0, gaps=((40.0, 44.0),), seed=7)
    _, _, est, bodies = run(ev, P())
    p_hat = [e for e in est if e["t"] < 40.0][-1]
    worst = 0.0
    for t, (x, y, yaw), b in bodies:
        if 40.5 < t < 43.5:
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = p_hat["x"] - x, p_hat["y"] - y
            worst = max(worst, abs(b[0] - (c * dx + s * dy)), abs(b[1] - (-s * dx + c * dy)))
    check(f"body = R^T(p_hat - p) on a held estimate (max err {worst:.1e} m)", worst < 1e-9, worst)
    e = run([x for x in ev if x[0] < 43.0], P())[0].estimate(43.0)
    check("the estimate itself does not move between fixes", (e["x"], e["y"]) == (p_hat["x"], p_hat["y"]))


def test_speed_cap():
    for v_target in (0.2, 2.0):
        def target(t, v=v_target):
            return (5.0 + v * t, 3.0)
        ev, _ = scenario(target, circle_boat(radius=4.0), duration=60.0, seed=8)
        for name, params in (("defaults", P()), ("v_max 0.3", P(v_max=0.3))):
            vmax = max((math.hypot(e["vx"], e["vy"]) for e in run(ev, params)[2] if e["vx"] is not None),
                       default=0.0)
            check(f"speed cap ({name}, target {v_target} m/s): {vmax:.3f}", vmax <= params["v_max"] + 1e-12)


def test_init_and_status():
    def target(t):
        return (7.0, 7.0)
    ev, _ = scenario(target, circle_boat(), duration=40.0, seed=12, gaps=((20.0, 41.0),))
    f = pf.PingerFilter(P().values)
    seen = {}
    for t, k, v in ev:
        if k == "odom":
            f.add_pose(t, *v)
            seen.setdefault(f.estimate(t)["status"], t)
        else:
            fix = pf.fix_from_json(t, v, "raw")
            if fix is not None:
                f.add_fix(fix)
    check("status no_data -> initialising -> tracking -> coasting -> stale",
          list(seen) == ["no_data", "initialising", "tracking", "coasting", "stale"], seen)
    e = f.estimate(40.0)
    check("a stale estimate keeps its position and its age", e["x"] is not None and e["fix_age"] > 6.0)


def test_live_retune():
    """set_params keeps the estimate (a live tuning change); only a new motion
    model restarts it."""
    def target(t):
        return (9.0, 4.0)
    ev, _ = scenario(target, circle_boat(), duration=40.0, seed=15)
    f = run(ev, P())[0]
    x0 = f.x.copy()
    f.set_params(P(sigma_bearing_deg=20.0, latency_s=1.2).values)
    check("a live change keeps the estimate", f.initialised and np.array_equal(f.x, x0)
          and f.p["sigma_bearing_deg"] == 20.0)
    f.set_params(P("static_target").values)
    check("a new motion model restarts it", not f.initialised and f.dim == 2)


def test_mapping():
    f = pf.PingerFilter(P(y_sign=-1, mount_yaw_deg=90.0, lever_x_m=0.5).values)
    bx, by, d, rho = f.map_fix((3.0, 4.0, 2.0))
    # y flipped -> (3, -4); rotated +90 -> (4, 3); + lever (0.5, 0) -> (4.5, 3)
    check("mapping: y_sign, mount yaw, lever arm",
          abs(bx - 4.5) < 1e-12 and abs(by - 3.0) < 1e-12 and d == 2.0 and abs(rho - 5.0) < 1e-12)


def test_inputs():
    """Malformed messages, the publish-delay correction, the pose history guards."""
    bad = {"schema": pf.SCHEMA_UWGPS_FIX, "seq": 1, "endpoints": {"raw": "garbage"}}
    check("a non-dict endpoint is an invalid fix", pf.fix_from_json(1.0, json.dumps(bad), "raw").invalid
          == "malformed_endpoint")
    check("not a /uw_gps/fix message: nothing", pf.fix_from_json(1.0, '{"schema": "x"}', "raw") is None
          and pf.fix_from_json(1.0, "not json", "raw") is None)
    msg = {"schema": pf.SCHEMA_UWGPS_FIX, "seq": 3, "t_end": 100.9,
           "endpoints": {"raw": {"ok": True, "t_resp": 100.05, "data": {"x": 3.0, "y": 1.0, "z": 0.5}},
                         "filtered": {"ok": True, "t_resp": 100.9, "data": {"x": 3.0, "y": 1.0, "z": 0.5}}}}
    check("raw fix receive time moved back by the later requests of its cycle",
          abs(pf.fix_from_json(500.0, json.dumps(msg), "raw").t_rx - 499.15) < 1e-9)
    check("filtered fix (the last request) not moved", pf.fix_from_json(500.0, json.dumps(msg), "filtered").t_rx == 500.0)
    msg["t_end"] = 1e9
    check("an implausible delay is ignored", pf.fix_from_json(500.0, json.dumps(msg), "raw").t_rx == 500.0)
    b = pf.PoseBuffer(30.0)
    for k in range(10):
        b.add(1000.0 + 0.1 * k, k, 0.0, 0.0)
    b.add(10.0, 5.0, 5.0, 0.0)                          # system time set back
    b.add(10.1, 5.1, 5.0, 0.0)
    check("clock step back: the pose history restarts", b.at(10.05, 0.5)[1] == 5.0)
    g = pf.PoseBuffer(30.0)
    g.add(0.0, 0.0, 0.0, 0.0)
    g.add(5.0, 10.0, 0.0, 0.0)
    check("no pose inside a long odometry gap", g.at(2.5, 0.5, 2.0) is None and g.at(2.5, 0.5)[0] == 5.0)


# ---------------------------------------------------------------------------
# the legacy pinger and the pinger mode
# ---------------------------------------------------------------------------

class _V11Reference:
    """robot_interface v1.1 (44ec6aa) pinger statements, verbatim."""

    def __init__(self):
        self.prev_time = None
        self.pinger_coordinates = np.zeros(3)
        self.angular_velocity = None

    def odom_callback(self, t, msg_twist):
        if self.prev_time is None:
            self.prev_time = t
            return None
        dt = t - self.prev_time
        self.prev_time = t
        x_t, y_t, z_t = msg_twist
        self.vel = np.array([x_t, y_t, z_t])
        av = self.angular_velocity
        if av is not None and not all(self.pinger_coordinates == np.zeros(3)):
            omega = np.array([0.0, 0.0, av])
            p = self.pinger_coordinates
            self.pinger_coordinates -= (self.vel + np.cross(omega, p)) * dt
        return self.pinger_coordinates.copy()

    def uw_gps_callback(self, data):
        t_x, t_y, t_z = data[16], data[17], data[18]
        self.pinger_coordinates = np.array([t_x, t_y, t_z])


def test_legacy_bit_identical():
    rng = np.random.default_rng(11)
    same, n = True, 0
    for _ in range(20):
        ref, new = _V11Reference(), pf.LegacyPinger()
        t = 1000.0
        for _ in range(500):
            t += float(rng.uniform(0.01, 0.4))
            k = rng.integers(0, 10)
            if k < 6:
                tw = tuple(float(v) for v in rng.normal(0, 0.7, 3))
                a, b = ref.odom_callback(t, tw), new.on_odom(t, tw)
                same &= (a is None) == (b is None) and (a is None or np.array_equal(a, b))
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
    check(f"legacy pinger == v1.1 robot_interface on {n} random odom events, bit for bit", same)


def test_pinger_mode_of():
    """The resolver master_control and robot_interface share: mode wins, else the use_pinger alias."""
    cases = [(("follow", False), "follow"), ((" LOG ", True), "log"), (("off", True), "off"),
             (("", True), "follow"), (("", False), "off"), ((None, None), "off"),
             ((None, 1), "follow"), ((0, True), "follow")]
    check("pinger_mode_of: mode wins, else the use_pinger alias",
          all(rls.pinger_mode_of(*args) == want for args, want in cases))
    refused = []
    for bad in ("chase", 5, ["follow"]):
        try:
            rls.pinger_mode_of(bad, True)
        except ValueError:
            refused.append(bad)
    check("pinger_mode_of: an unknown (or non-string) mode is a ValueError", len(refused) == 3, refused)


TESTS = [test_params, test_tuning_file, test_still_target_rms, test_burst_rejected,
         test_relocation_reacquired, test_latency_compensation,
         test_duplicates_and_failed_requests_change_nothing, test_still_between_fixes, test_speed_cap,
         test_init_and_status, test_live_retune, test_mapping, test_inputs, test_legacy_bit_identical,
         test_pinger_mode_of]


def main():
    for test in TESTS:
        test()
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
