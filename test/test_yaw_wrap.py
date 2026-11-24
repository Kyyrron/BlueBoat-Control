#!/usr/bin/env python3
"""
Heading at +-180 deg: regression test (2026-11-13).

What was wrong. On a west-going line the measured yaw wraps between +pi and
-pi. `ur_mpc.MPCController.solve` pinned stage 0 and shifted the heading
reference onto the new branch, while the SQP_RTI warm start of stages 1..N
stayed a whole turn away; the single RTI step, linearised there, commanded
full differential thrust for one tick (+-40 N at every crossing, measured in
three Gazebo runs of 2026-11-12 and in this harness). Fixed by
`MPCController._rebranch_iterate`.

What is tested, for MPC, PID and LoS, on the deterministic harness
(`mc_harness.py`: the real master_control.py, tick by tick):

  * a due-west line entered from the side, and a weave whose heading
    oscillates about pi, so the measured yaw crosses +-180 deg in BOTH
    directions;
  * each has a twin rotated by pi that never comes near +-pi. The closed loop
    is rotation-equivariant, so the thrust and tau traces of a pair must agree
    (to rounding for PID / LoS, to solver tolerance for MPC);
  * no step of the differential thrust at a crossing tick;
  * the fault itself: the same MPC runs with the pre-fix solver
    (`rebranch_iterate = False`) must show the pulse, so the test cannot pass
    by accident.

Needs what the harness needs (ROS 2 sourced, the workspace install, and the
venv with acados for MPC):

    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    source ~/ros2_ws/.venv/bin/activate
    python3 test/test_yaw_wrap.py            # exit code 0 = pass
"""

import json
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
FIX = os.path.join(HERE, "fixtures")
HARNESS = os.path.join(HERE, "mc_harness.py")

SPEED = 0.5            # m/s along the base heading
DT = 0.25              # s between samples
DURATION = 60.0        # s
WEAVE_AMPL = 1.5       # m
WEAVE_PERIOD = 20.0    # s  -> heading swings +-43 deg about the base heading

# Agreement of a pair. PID and LoS are plain arithmetic: rounding only. MPC
# goes through acados; the two members are different floating-point problems.
THRUST_TOL = {"PID": 0.01, "LoS": 0.01, "MPC": 0.5}     # N, max |difference|
TAU_TOL = {"PID": 1e-3, "LoS": 1e-3, "MPC": 0.02}       # s
STEP_TOL = 1.0         # N: allowed excess of the differential-thrust step at a
                       # crossing tick over the twin's step at the same tick
FAULT_MIN = 15.0       # N: what the pre-fix MPC must exceed at a crossing


def _points(kind, base):
    """[t, x, y, yaw] samples: a straight line or a weave along heading `base`."""
    rows = []
    n = int(round(DURATION / DT)) + 1
    for i in range(n):
        t = i * DT
        s = SPEED * t
        if kind == "line":
            lat, dlat = 0.0, 0.0
        else:
            w = 2.0 * math.pi / WEAVE_PERIOD
            lat, dlat = WEAVE_AMPL * math.sin(w * t), WEAVE_AMPL * w * math.cos(w * t)
        x = s * math.cos(base) - lat * math.sin(base)
        y = s * math.sin(base) + lat * math.cos(base)
        yaw = base + math.atan2(dlat, SPEED)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        if kind == "line" and base != 0.0:
            yaw = 3.14159            # the literal a designer file carries for due west
        rows.append([round(t, 3), round(x, 6), round(y, 6), round(yaw, 6)])
    return rows


def write_fixtures():
    os.makedirs(FIX, exist_ok=True)
    for kind in ("line", "weave"):
        for side, base in (("west", math.pi), ("east", 0.0)):
            path = os.path.join(FIX, "yaw_wrap_%s_%s.yaml" % (side, kind))
            lines = ["format: blueboat_trajectory/1",
                     "name: yaw_wrap_%s_%s" % (side, kind),
                     "loop: false",
                     "points:"]
            lines += ["- [%s]" % ", ".join(repr(v) for v in row) for row in _points(kind, base)]
            text = "\n".join(lines) + "\n"
            if not os.path.exists(path) or open(path).read() != text:
                with open(path, "w") as handle:
                    handle.write(text)


def run(scenario, controller, out_dir, prefix=False):
    out = os.path.join(out_dir, "%s-%s-%d.json" % (scenario, controller, int(prefix)))
    env = dict(os.environ)
    env.pop("MC_HARNESS_NO_REBRANCH", None)
    if prefix:
        env["MC_HARNESS_NO_REBRANCH"] = "1"
    proc = subprocess.run([sys.executable, HARNESS, "--one", "--scenario", scenario,
                           "--controller", controller, "--out", out],
                          capture_output=True, text=True, env=env)
    if proc.returncode != 0:
        raise RuntimeError("%s/%s failed:\n%s\n%s" % (scenario, controller,
                                                    proc.stdout[-2000:], proc.stderr[-2000:]))
    with open(out) as handle:
        return json.load(handle)


def crossings(trace):
    """Ticks where the measured yaw wraps, split by direction."""
    up, down, prev = [], [], None
    for k, row in enumerate(trace["monitoring"]):
        if row is None:
            continue
        psi = row[3]
        if prev is not None and psi - prev > math.pi:
            up.append(k)            # -pi -> +pi
        if prev is not None and prev - psi > math.pi:
            down.append(k)          # +pi -> -pi
        prev = psi
    return up, down


def diff_steps(trace):
    d = [t[0] - t[1] for t in trace["thrust"]]
    return [0.0] + [abs(d[k] - d[k - 1]) for k in range(1, len(d))]


def main():
    write_fixtures()
    controllers = [c for c in (sys.argv[1].split(",") if len(sys.argv) > 1 else ["PID", "LoS", "MPC"])]
    tmp = tempfile.mkdtemp(prefix="yaw_wrap_")
    passed = failed = 0

    def check(ok, text):
        nonlocal passed, failed
        passed, failed = passed + bool(ok), failed + (not ok)
        print("%s  %s" % ("ok  " if ok else "FAIL", text), flush=True)

    for kind in ("line", "weave"):
        for controller in controllers:
            west = run("yaw_west_" + kind, controller, tmp)
            east = run("yaw_east_" + kind, controller, tmp)
            up, down = crossings(west)
            e_up, e_down = crossings(east)
            label = "%-5s %-3s" % (kind, controller)
            # the line is entered from one side: one crossing is enough there;
            # the weave must cross both ways
            crossed = (len(up) + len(down) >= 1) if kind == "line" else (len(up) >= 1 and len(down) >= 1)
            check(crossed, "%s west run crosses +-180 deg (%d up, %d down)" % (label, len(up), len(down)))
            check(not e_up and not e_down, "%s east twin never wraps" % label)
            dthr = max(abs(p - q) for a, b in zip(west["thrust"], east["thrust"]) for p, q in zip(a, b))
            check(dthr <= THRUST_TOL[controller],
                  "%s thrust equals the twin's (max |difference| %.2e N, tolerance %g)" % (
                      label, dthr, THRUST_TOL[controller]))
            dtau = max(abs(p - q) for p, q in zip(west["tau"], east["tau"]))
            check(dtau <= TAU_TOL[controller],
                  "%s tau equals the twin's (max |difference| %.2e s)" % (label, dtau))
            # the step of the differential thrust at each crossing tick, against
            # the twin's step at the SAME tick (a start-up step is in both)
            steps, twin = diff_steps(west), diff_steps(east)
            worst = max((steps[k] - twin[k] for k in up + down), default=0.0)
            check(worst <= STEP_TOL,
                  "%s no differential step at a crossing (%.2f N more than the twin at the same tick)" % (
                      label, worst))
            if controller == "MPC":
                old = run("yaw_west_" + kind, controller, tmp, prefix=True)
                o_up, o_down = crossings(old)
                o_steps = diff_steps(old)
                o_worst = max((o_steps[k] - twin[k] for k in o_up + o_down), default=0.0)
                check(o_worst >= FAULT_MIN,
                      "%s pre-fix solver shows the fault (%.1f N step at a crossing)" % (label, o_worst))

    print("\n%d passed, %d failed" % (passed, failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
