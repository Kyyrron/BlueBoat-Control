#!/usr/bin/env python3
"""Cross-validation gate for replay.py.

A recording produced by the simulated plant must, when replayed, reproduce that
simulation's own numbers. If the two paths disagree on a case where they cannot,
the replay wiring is wrong and no field result from it is trustworthy.

So: run sim.run(), write the result out as a controller .npy log AND as a real
rosbag2, read both back through replay.py, and require

  1. as_flown() metrics match analyze.row() on the original simulation,
  2. counterfactual(), given the reference the controller actually saw,
     reproduces its commands EXACTLY -- this is the wiring test,
  3. counterfactual() on a reference rebuilt from the recording alone stays
     inside a stated bound -- this is not wiring, it is the recorded
     interface's own limit (see replay.windows) and it is measured, not
     tolerated.

Tolerances differ by source, on purpose:

  bag   tight. /blueboat/odom carries the body-frame twist, so the replayed
        controller sees exactly the state the simulation fed it.
  npy   loose. The schema has no twist -- u/v/r are differentiated back out of
        the pose -- and np.save coerced every value through a <U32 string. Both
        are real properties of that artifact, not slack in the check.

The bag half skips with a message when rosbag2_py is absent, so this still runs
on a machine with no ROS workspace.

Run:  python3 check_replay.py     (exit 0 pass, 1 fail)
"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile

import numpy as np

import analyze
import replay
import sim

FAILURES = []

# The bag round-trip is exact in the state it hands the controller, so the only
# slack is float64 -> float32 on the Float32MultiArray wire.
BAG_TOL = {"rms": 2e-3, "peak": 2e-3, "u": 2e-3, "thrust": 2e-2, "command": 1e-3}
# The .npy path differentiates twist out of the pose and round-trips every value
# through a <U32 string; both are real properties of that artifact, not slack.
NPY_TOL = {"rms": 2e-3, "peak": 2e-3, "u": 5e-2, "thrust": 2e-2, "command": 1e-2}

# Bound on check 3, the reference a recording cannot recover. Measured, not
# guessed: at the shipped inner gains the governor throttles to ~0.43x, so the
# rebuilt U_d is low by that factor and surge is under-commanded to match.
# Roughly (1 - factor) * authored_speed * inner_gain_u / 2 per thruster.
RECONSTRUCTION_BOUND_N = 0.25


def check(name, ok, detail=""):
    print(("  PASS  " if ok else "  FAIL  ") + name + (("\n         " + detail) if detail else ""))
    if not ok:
        FAILURES.append(name)


def close(a, b, tol):
    return bool(np.isfinite(a) and np.isfinite(b) and abs(a - b) <= tol)


# ───────────────────────────── writers (test fixtures only) ───────────────────
def write_npy(d, path):
    """Serialise a sim.run log in the controller .npy schema (CLAUDE.md #6).

    Header appended as a row of strings, which is what coerces the whole array
    to <U32 -- reproduced deliberately so the reader is tested against the real
    artifact shape, not a tidied one.
    """
    rows = [[d["t"][k], d["x"][k], d["y"][k], d["psi"][k],
             d["xd"][k], d["yd"][k], d["psid"][k], d["thr_r"][k], d["thr_l"][k]]
            for k in range(len(d["t"]))]
    # repr(float(v)), not repr(v): master_control appends Python floats, and
    # numpy >= 2 reprs a np.float64 as "np.float64(0.0)", which is not a number
    # any reader can parse. float() first keeps this faithful on both numpy lines.
    np.save(path, np.array([replay.MONITORING_COLUMNS]
                           + [[repr(float(v)) for v in r] for r in rows]))


def write_bag(d, uri):
    """Serialise a sim.run log as a rosbag2, the way master_control would have."""
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32MultiArray

    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    for i, (name, typ) in enumerate(((replay.TOPIC_ODOM, "nav_msgs/msg/Odometry"),
                                     (replay.TOPIC_MONITORING, "std_msgs/msg/Float32MultiArray"),
                                     (replay.TOPIC_THRUST, "std_msgs/msg/Float32MultiArray"))):
        w.create_topic(rosbag2_py.TopicMetadata(
            id=i, name=name, type=typ, serialization_format="cdr"))
    for k in range(len(d["t"])):
        stamp = int(d["t"][k] * 1e9)
        o = Odometry()
        o.pose.pose.position.x = float(d["x"][k])
        o.pose.pose.position.y = float(d["y"][k])
        o.twist.twist.linear.x = float(d["u"][k])     # body-frame already (N3)
        o.twist.twist.linear.y = float(d["v"][k])
        o.twist.twist.angular.z = float(d["r"][k])
        w.write(replay.TOPIC_ODOM, serialize_message(o), stamp)
        m = Float32MultiArray()
        m.data = [float(d[c][k]) for c in
                  ("t", "x", "y", "psi", "xd", "yd", "psid", "thr_r", "thr_l")]
        w.write(replay.TOPIC_MONITORING, serialize_message(m), stamp)
        thr = Float32MultiArray()
        thr.data = [float(d["thr_r"][k]), float(d["thr_l"][k])]
        w.write(replay.TOPIC_THRUST, serialize_message(thr), stamp)
    del w


# ───────────────────────────── the gate ───────────────────────────────────────
def metrics(d, tail=0.25):
    n = int(len(d["t"]) * tail)
    return dict(
        rms=float(np.sqrt(np.mean(d["e_y"][-n:] ** 2))),
        peak=float(np.max(np.abs(d["e_y"]))),
        u=float(np.mean(d["u"][-n:])),
        thrust=float(np.mean(np.abs(d["thr_r"][-n:]) + np.abs(d["thr_l"][-n:])) / 2),
    )


def compare(label, truth, got, tol):
    bad = [f"{k}: sim {truth[k]:.4f} vs replay {got[k]:.4f} (tol {tol[k]:g})"
           for k in truth if not close(truth[k], got[k], tol[k])]
    check(f"{label}: as-flown metrics reproduce the simulation", not bad,
          "; ".join(bad) if bad else
          "  ".join(f"{k}={got[k]:.4f}" for k in sorted(truth)))


def exact_reference(d, shape, ctrl):
    """The reference windows the simulation actually handed the controller.

    Rebuilt from the logged path parameter: sim.run's window is
    single_pose(linspace(tau, tau + path_time, path_steps)), and tau is logged.
    Only the gate can do this -- a recording has no tau, which is the whole
    point of check 3.

    sim.run logs tau AFTER gov.advance, so the window used at tick k was built
    from tick k-1's logged tau, and tick 0's from tau = 0.
    """
    taus = np.concatenate([[0.0], d["tau"][:-1]])
    out = []
    for tau in taus:
        ts = np.linspace(tau, tau + ctrl.path_time, int(ctrl.path_steps))
        out.append(([sim.single_pose(t, shape)[:3] for t in ts], 0.05))
    return out


def compare_commands_exact(label, d, rec, ref, mk, tol):
    """Given the reference the controller really saw, the replay must match exactly."""
    cf = replay.counterfactual(rec, mk(), reference=ref)
    dr = float(np.sqrt(np.mean((cf[:, 0] - d["thr_r"]) ** 2)))
    dl = float(np.sqrt(np.mean((cf[:, 1] - d["thr_l"]) ** 2)))
    ok = dr <= tol["command"] and dl <= tol["command"]
    check(f"{label}: replay steps the controller exactly, given the true reference", ok,
          f"RMS command difference right {dr:.3e} N, left {dl:.3e} N (tol {tol['command']:g})")


def compare_commands_reconstructed(label, d, rec, mk):
    """With the reference rebuilt from the recording, measure the unavoidable gap."""
    cf = replay.counterfactual(rec, mk())
    dr = float(np.sqrt(np.mean((cf[:, 0] - d["thr_r"]) ** 2)))
    dl = float(np.sqrt(np.mean((cf[:, 1] - d["thr_l"]) ** 2)))
    worst = max(dr, dl)
    check(f"{label}: reference rebuilt from the recording stays inside the stated bound",
          worst <= RECONSTRUCTION_BOUND_N,
          f"RMS command difference right {dr:.4f} N, left {dl:.4f} N "
          f"(bound {RECONSTRUCTION_BOUND_N} N). /monitoring_data carries win[0] only, so "
          f"win[1] and U_d are not recoverable -- see replay.windows.")


def main():
    print("replay cross-validation gate")
    print("  generating the reference simulation (PID, straight_line, 5 m offset, 60 s)")
    shape = "straight_line"
    def mk():
        return sim.PIDController(dt=0.05)
    d = sim.run(mk(), shape, start=(0., -4., 0.), T=60)
    truth = metrics(d)
    ref = exact_reference(d, shape, mk())
    print("     sim: " + "  ".join(f"{k}={truth[k]:.4f}" for k in sorted(truth))
          + f"  governor factor {np.mean(d['factor']):.2f}x")

    tmp = tempfile.mkdtemp(prefix="replay_gate_")
    try:
        # --- .npy round-trip (no ROS) -----------------------------------------
        npy = os.path.join(tmp, "run.npy")
        write_npy(d, npy)
        rec = replay.load(npy)
        check("npy: twist is reported as derived, not logged",
              rec["twist_source"] == "derived", f"twist_source = {rec['twist_source']!r}")
        compare("npy", truth, metrics(replay.as_flown(rec)), NPY_TOL)
        compare_commands_exact("npy", d, rec, ref, mk, NPY_TOL)
        compare_commands_reconstructed("npy", d, rec, mk)

        # --- rosbag2 round-trip ------------------------------------------------
        try:
            import rosbag2_py  # noqa: F401
        except ImportError:
            print("  SKIP  bag round-trip: rosbag2_py not importable "
                  "(source /opt/ros/<distro>/setup.bash to run it)")
        else:
            uri = os.path.join(tmp, "run_bag")
            write_bag(d, uri)
            rec = replay.load(uri)
            check("bag: twist comes from /blueboat/odom, not differentiated",
                  rec["twist_source"] == "logged", f"twist_source = {rec['twist_source']!r}")
            compare("bag", truth, metrics(replay.as_flown(rec)), BAG_TOL)
            compare_commands_exact("bag", d, rec, ref, mk, BAG_TOL)
            compare_commands_reconstructed("bag", d, rec, mk)

        # --- an unreadable recording must be named, not crash -----------------
        empty = os.path.join(tmp, "empty.npy")
        open(empty, "wb").close()
        try:
            replay.load(empty)
            check("a 0-byte .npy is reported, not crashed on", False, "no RecordingError raised")
        except replay.RecordingError as exc:
            check("a 0-byte .npy is reported, not crashed on", True, str(exc))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if FAILURES:
        print(f"\n{len(FAILURES)} check(s) failed: " + "; ".join(FAILURES))
        return 1
    print("\nall checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
