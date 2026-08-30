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


THR_LIM = 20.0          # the +/-20 N clamp in sim.run


def saturated(d, lo=None, hi=None):
    """Fraction of samples with either thruster on the +/-20 N clamp.

    Raising the inner gains buys tracking partly by living on the limiter, and
    that does not transfer to the boat. Reported over the whole run (transient,
    i.e. acquisition and corners) and over the tail window (steady state, which
    is the one that must stay at zero).
    """
    r, l = d["thr_r"][lo:hi], d["thr_l"][lo:hi]
    return float(np.mean((np.abs(r) >= THR_LIM - 1e-9) | (np.abs(l) >= THR_LIM - 1e-9)))


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
        sat=saturated(d),
        sat_tail=saturated(d, lo=-n),
    )


def main():
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
              f"{'u_mean':>9}{'tau/T':>8}{'|thrust|':>10}{'sat':>8}{'sat_tail':>10}")
        for r in rows:
            st = "never" if np.isnan(r["settle"]) else f"{r['settle']:.0f} s"
            print(f"  {r['name']:<6}{r['rms']:>9.3f}{r['peak']:>10.2f}{st:>13}"
                  f"{r['u']:>9.3f}{r['prog']:>8.2f}{r['thr']:>10.1f}"
                  f"{r['sat']*100:>7.1f}%{r['sat_tail']*100:>9.1f}%")

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

    station_keeping()


def station_keeping():
    """
    F2: what the boat does when the reference is stationary and the authored
    speed is therefore zero. Distances are from the hold point (the origin).
    'off' is the same LoS law with the hold term disabled - the behaviour
    before the fix - so the comparison is measured, not remembered.
    """
    cases = [("calm", "displaced 2.2 m, no disturbance"),
             ("cur4", "4 N lateral current"),
             ("cur10", "10 N lateral current"),
             ("past", "3 m past the hold point")]
    try:
        load("hold_LoS_calm")
    except FileNotFoundError:
        print("\nstation keeping: not cached yet")
        return

    print("\nStation keeping (F2) -- distance from the hold point")
    print(f"  {'case':<8}{'ctrl':<6}{'final':>8}{'max':>8}{'mean(20s)':>11}"
          f"{'std(20s)':>10}{'chatter':>9}{'|thrust|':>10}")
    for key, label in cases:
        for name, ctrl in (("LoS", "hold_LoS"), ("off", "hold_off"), ("PID", "hold_PID")):
            try:
                d = load(f"{ctrl}_{key}")
            except FileNotFoundError:
                continue
            r = np.hypot(d["x"], d["y"])
            tail = r[-400:]                       # last 20 s at dt = 0.05
            thr = d["thr_r"][-400:]
            chatter = int(np.sum(np.diff(np.sign(thr)) != 0))
            print(f"  {key if name == 'LoS' else '':<8}{name:<6}{r[-1]:>8.3f}{r.max():>8.3f}"
                  f"{tail.mean():>11.3f}{tail.std():>10.4f}{chatter:>9d}"
                  f"{np.abs(thr).mean():>10.2f}")
        print(f"           ({label})")


if __name__ == "__main__":
    main()
