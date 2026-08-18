"""Summary statistics for the head-to-head table."""
import os
import pickle

import numpy as np

CACHE = os.path.join(os.path.dirname(__file__), "cache")


def load(k):
    with open(os.path.join(CACHE, k + ".pkl"), "rb") as f:
        return pickle.load(f)


def settle(d, tol=0.5):
    """First time |e_y| stays under tol for the rest of the run."""
    bad = np.where(np.abs(d["e_y"]) > tol)[0]
    if len(bad) == 0:
        return 0.0
    if bad[-1] >= len(d["t"]) - 2:
        return float("nan")
    return float(d["t"][bad[-1] + 1])


def row(key, name, tail=0.25):
    d = load(f"{key}_{name}")
    n = int(len(d["t"]) * tail)
    return dict(
        name=name,
        rms=float(np.sqrt(np.mean(d["e_y"][-n:] ** 2))),
        peak=float(np.max(np.abs(d["e_y"]))),
        settle=settle(d),
        u=float(np.mean(d["u"][-n:])),
        tau=float(d["tau"][-1]),
        T=float(d["t"][-1]),
        prog=float(d["tau"][-1] / d["t"][-1]),
        thr=float(np.mean(np.abs(d["thr_r"][-n:]) + np.abs(d["thr_l"][-n:])) / 2),
    )


for scen, label in (("acquire", "A. Straight line, 5 m offset"),
                    ("circle", "B. 4 m circle"),
                    ("square", "C. Zig-zag, 90 deg corners"),
                    ("current", "D. Straight line + 10 N side current")):
    try:
        rows = [row(scen, n) for n in ("MPC", "PID", "LoS")]
    except FileNotFoundError:
        print(f"\n{label}: not cached yet")
        continue
    print(f"\n{label}   (run length {rows[0]['T']:.0f} s)")
    print(f"  {'ctrl':<6}{'RMS e_y':>9}{'peak e_y':>10}{'settle<0.5m':>13}"
          f"{'u_mean':>9}{'tau/T':>8}{'|thrust|':>10}")
    for r in rows:
        st = "never" if np.isnan(r["settle"]) else f"{r['settle']:.0f} s"
        print(f"  {r['name']:<6}{r['rms']:>9.3f}{r['peak']:>10.2f}{st:>13}"
              f"{r['u']:>9.3f}{r['prog']:>8.2f}{r['thr']:>10.1f}")

# MPC yaw-wrap check on the circle
try:
    d = load("circle_MPC")
    psi = d["psi"]
    jumps = np.where(np.abs(np.diff(psi)) > np.pi)[0]
    print(f"\nMPC circle: reference yaw wraps at t = "
          f"{[round(float(d['t'][i]), 1) for i in jumps[:6]]}")
    for i in jumps[:4]:
        lo, hi = max(0, i - 20), min(len(psi), i + 40)
        print(f"   around t={d['t'][i]:6.1f}s : |e_y| {np.abs(d['e_y'][lo:hi]).max():.2f} m, "
              f"thrust swing {d['thr_r'][lo:hi].min():+.1f}..{d['thr_r'][lo:hi].max():+.1f} N")
except FileNotFoundError:
    print("\ncircle_MPC not cached yet")

try:
    s = load("surge_sweep")
    print("\nsurge sweep:")
    print("  LoS ", list(zip(s["los_k"], [round(v, 3) for v in s["los_u"]])))
    print("  PID ", list(zip(s["pid_k"], [round(v, 3) for v in s["pid_u"]])))
except FileNotFoundError:
    pass

try:
    for k in ("gov_slow", "gov_ok"):
        d = load(k)
        print(f"\n{k}: tau={d['tau'][-1]:.1f} over {d['t'][-1]:.0f}s wall "
              f"(progress {d['tau'][-1]/d['t'][-1]:.2f}x), "
              f"e_along={d['e_along'][-400:].mean():+.2f} m, "
              f"factor={d['factor'][-400:].mean():.2f}, u={d['u'][-400:].mean():.3f}")
except FileNotFoundError:
    pass
