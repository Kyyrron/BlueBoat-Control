"""Run every scenario once and cache the logs to .npz (MPC is expensive)."""
import os
import pickle
import time

import numpy as np

import sim

CACHE = os.path.join(os.path.dirname(__file__), "cache")
os.makedirs(CACHE, exist_ok=True)

MPC_ITER = 6


def cached(key, fn):
    path = os.path.join(CACHE, key + ".pkl")
    if os.path.exists(path):
        print(f"  [cache] {key}")
        with open(path, "rb") as f:
            return pickle.load(f)
    t0 = time.time()
    out = fn()
    with open(path, "wb") as f:
        pickle.dump(out, f)
    print(f"  [run  ] {key}  ({time.time()-t0:.1f}s)")
    return out


def ctrls(dt=0.05):
    return [("MPC", lambda: sim.MPCController(maxiter=MPC_ITER)),
            ("PID", lambda: sim.PIDController(dt=dt)),
            ("LoS", lambda: sim.LoSController())]


def main():
    # A -- acquisition from a 5 m cross-track offset on a straight line
    print("A: acquisition")
    for n, mk in ctrls():
        cached(f"acquire_{n}", lambda mk=mk: sim.run(mk(), "straight_line",
                                                     start=(0., -4., 0.), T=60))
    # B -- circle (continuous curvature + a full yaw wrap through +/-pi)
    print("B: circle")
    for n, mk in ctrls():
        cached(f"circle_{n}", lambda mk=mk: sim.run(mk(), "circle",
                                                    start=(0., 0., 0.), T=160))
    # C -- kin_square (sharp 90 deg corners)
    print("C: kin_square")
    for n, mk in ctrls():
        cached(f"square_{n}", lambda mk=mk: sim.run(mk(), "kin_square",
                                                    start=(0., 0., 0.), T=160))
    # D -- constant lateral current on a straight line
    print("D: current")
    for n, mk in ctrls():
        cached(f"current_{n}", lambda mk=mk: sim.run(mk(), "straight_line",
                                                     start=(0., 1., 0.), T=90,
                                                     force_world=(0., -10.)))
    # E -- lookahead sweep (LoS)
    print("E: lookahead")
    for D in (1.0, 2.5, 6.0):
        cached(f"look_{D}", lambda D=D: sim.run(sim.LoSController(lookahead=D),
                                                "straight_line", start=(0., -4., 0.), T=60))
    # F -- surge-gain sweep
    print("F: surge gains")

    def sweep():
        out = {"los_k": [4, 8, 15, 30, 60, 120, 250], "los_u": [],
               "pid_k": [0.5, 1.0, 2.0, 5.0, 10.0, 30.0, 60.0], "pid_u": []}
        for k in out["los_k"]:
            d = sim.run(sim.LoSController(ku=k), "straight_line", start=(0., 1., 0.), T=150)
            out["los_u"].append(float(d["u"][-400:].mean()))
        for k in out["pid_k"]:
            c = sim.PIDController(dt=0.05, inner={"u": (k, 0., 0.), "r": (1.5, 0., 0.)})
            d = sim.run(c, "straight_line", start=(0., 1., 0.), T=150)
            out["pid_u"].append(float(d["u"][-400:].mean()))
        return out
    cached("surge_sweep", sweep)

    # G -- point LoS (pinger / manual) from four starting poses
    print("G: point LoS")

    def pts():
        starts = [(0., 0., 0.), (0., 0., np.pi / 2), (0., 0., np.pi), (0., 0., -np.pi / 2)]
        return [sim.run_point(sim.PointLoS(), (12., 6.), start=s, T=90) for s in starts]
    cached("point_los", pts)

    # H -- governor: boat cannot make the authored speed (headwind)
    print("H: governor")
    cached("gov_slow", lambda: sim.run(sim.PIDController(dt=0.05), "straight_line",
                                       start=(0., 1., 0.), T=150, force_world=(-9., 0.)))
    cached("gov_ok", lambda: sim.run(
        sim.PIDController(dt=0.05, inner={"u": (10., 0., 0.), "r": (1.5, 0., 0.)}),
        "straight_line", start=(0., 1., 0.), T=150))
    print("done")


if __name__ == "__main__":
    main()
