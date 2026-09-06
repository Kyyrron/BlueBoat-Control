#!/usr/bin/env python3
"""
Manual-target keep-location check.

A reached manual target is a station to HOLD, not a place to stop. Before this,
solve_LoS latched on arrival -- one second astern, then zero thrust for the rest
of the run, cleared only by a new target -- so any current carried the boat out
of the survey area with the controller commanding nothing. On the real boat the
latch never even armed (safety_distance = -1.0), so the point law hunted instead
of settling.

Four properties, and the last is the one that makes the harness admissible as
evidence for the shipped controller:

  1. The PINGER path is untouched. Not "close" -- bit-identical logs, because
     the hold is gated on the manual test solve_LoS already makes.
  2. The hold actually holds. Against a steady current the boat parks inside the
     re-acquire radius, where the law it replaced ends tens of metres downstream.
     Both baselines are RUN, not described from memory: "latch" is what
     simulation did, "pursuit" is what the real boat did.
  3. Disabling it restores the old law exactly, so the change is reversible from
     a launch argument rather than a revert.
  4. sim.PointLoS and master_control.solve_LoS / manual_keep_location are the
     same law. master_control cannot be imported without acados, so it is read
     statically -- the approach check_los_hold.py and check_pid_equivalence.py
     both take.

    python3 check_manual_hold.py      # exit 0 pass, 1 fail

Needs numpy and scipy (through sim), no ROS.
"""

import math
import os
import sys

import numpy as np

import sim

HERE = os.path.dirname(os.path.abspath(__file__))
MASTER = os.path.abspath(os.path.join(HERE, "..", "..", "master_control.py"))

TARGET = (12., 6.)
REAL = dict(k_v=0.15, k_psi=10.0)          # the gains the boat ships
DEADBAND = sim.MIN_THRUST                   # the ESC neutral band, in the plant

failures = []


def check(label, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {label}{'  -- ' + detail if detail else ''}")
    if not ok:
        failures.append(label)


def run(hold, force=0.0, start=(12., 6., 0.), T=400, deadband=DEADBAND, **kw):
    ctrl = sim.PointLoS(manual=True, hold=hold, **REAL, **kw)
    return sim.run_point(ctrl, TARGET, start=start, T=T,
                         force_world=(-force, 0.), deadband=deadband)


# ------------------------------------------------------- 1. pinger untouched
print("The pinger path is bit-identical")

for label, start in (("head-on", (0., 0., 0.)),
                     ("abeam", (0., 0., np.pi / 2)),
                     ("astern", (0., 0., np.pi)),
                     ("abeam other side", (0., 0., -np.pi / 2))):
    a = sim.run_point(sim.PointLoS(**REAL), TARGET, start=start, T=150)
    b = sim.run_point(sim.PointLoS(**REAL, hold=False), TARGET, start=start, T=150)
    same = all(np.array_equal(a[k], b[k]) for k in a)
    check(f"pinger from {label}: hold flag changes nothing", same)

pinger = sim.PointLoS(**REAL)
check("pinger law never enters the hold",
      pinger.manual is False and pinger.holding is False)

# Stronger than "the flag changes nothing": the pre-change formula is written
# out here from the original source, so this compares against what the harness
# used to compute rather than against another copy of what it computes now.
def _pre_change_point_los(k_v, k_psi):
    def law(target_world, state, dt):
        xt, yt = target_world
        xr, yr, psir = state[0], state[1], state[2]
        x = (xt - xr) * math.cos(psir) + (yt - yr) * math.sin(psir)
        y = (yt - yr) * math.cos(psir) - (xt - xr) * math.sin(psir)
        yaw_rate = k_psi * math.atan2(y, x)
        d = math.hypot(x, y)
        v = 5 * math.log(k_v * d + 1)
        return np.array([v + 0.295 * yaw_rate, v - 0.295 * yaw_rate])
    return law


now = sim.PointLoS(**REAL)
before = _pre_change_point_los(REAL["k_v"], REAL["k_psi"])
rng = np.random.default_rng(0)
worst = 0.0
for _ in range(20000):
    state = np.array([rng.uniform(-30, 30), rng.uniform(-30, 30),
                      rng.uniform(-np.pi, np.pi), 0., 0., 0.])
    target = (rng.uniform(-30, 30), rng.uniform(-30, 30))
    worst = max(worst, float(np.max(np.abs(now(target, state, 0.05)
                                          - before(target, state, 0.05)))))
check("pinger law is bit-identical to the pre-change formula, 20000 states",
      worst == 0.0, f"worst difference {worst:.3e} N")


# ------------------------------------------------------------ 2. it holds
print("\nIt holds station against a current where the old law did not")

RE_ACQ = sim.MANUAL_REACQUIRE_RADIUS
for force in (0., 2., 4., 8., 12.):
    held = run(True, force)["d"][-1]
    latched = run("latch", force)["d"][-1]
    check(f"current {force:4.0f} N: parks inside the re-acquire radius",
          held <= RE_ACQ + 1e-9, f"{held:.2f} m, limit {RE_ACQ:.2f} m")
    if force > 0:
        check(f"current {force:4.0f} N: the old latch was swept away",
              latched > 10 * held, f"latched {latched:.1f} m vs held {held:.2f} m")

# Monotone in the disturbance, and never worse than the band.
parks = [run(True, f)["d"][-1] for f in (0., 2., 4., 8., 12.)]
check("park distance grows monotonically with the current",
      all(b >= a - 1e-6 for a, b in zip(parks, parks[1:])),
      ", ".join(f"{p:.2f}" for p in parks))

# The approach case: the boat must still settle, not hunt.
approach = run(True, 0.0, start=(7., 6., 0.))
check("approaching from 5 m, the boat settles on the target",
      approach["d"][-1] <= sim.MANUAL_HOLD_RADIUS,
      f"{approach['d'][-1]:.2f} m")
check("approaching from 5 m, it does not run away",
      approach["d"].max() <= 5.01, f"max {approach['d'].max():.2f} m")

# Steady state means steady: no limit cycle inside the hold.
tail = run(True, 8.0)["d"][-2000:]
check("no limit cycle while holding against 8 N",
      float(tail.max() - tail.min()) < 0.05,
      f"peak-to-peak {tail.max() - tail.min():.3f} m")

# The arrival brake fires once, and only once.
brake = run(True, 0.0, start=(7., 6., 0.))
astern = np.asarray(brake["thr_r"]) < -0.5
edges = int(np.sum(astern[1:] & ~astern[:-1])) + int(astern[0])
check("the arrival astern pulse fires exactly once", edges == 1, f"{edges} pulses")


# --------------------------------------------------------- 3. reversible
print("\nDisabling it restores the previous law exactly")

off = run(True, 4.0, hold_radius=0.0)
raw = run(False, 4.0)
check("manual_hold_radius = 0 is bit-identical to the ungated pursuit law",
      all(np.array_equal(off[k], raw[k]) for k in off))


# ------------------------------------------------- 4. same law as the node
print("\nsim.PointLoS and master_control are the same law")

source = open(MASTER, encoding="utf-8").read()

needles = {
    "gates the hold on the manual target":
        "if manual:\n            held = self.manual_keep_location(",
    "enters the hold at manual_hold_radius":
        "if d <= self.manual_hold_radius:",
    "leaves it beyond manual_reacquire_radius":
        "elif d > self.manual_reacquire_radius:",
    "surge is proportional to the gap, capped":
        "v_hold = min(self.manual_hold_umax, self.manual_hold_kx * gap)",
    "hold rides inside the same cos shaping":
        "v_hold *= max(0.0, np.cos(bearing))",
    "hold surge carries the breakaway floor":
        "floor = self.min_thrust * max(0.0, np.cos(bearing))",
    "the differential is untouched":
        "return [v_hold + 0.295 * yaw_rate, v_hold - 0.295 * yaw_rate]",
    "the astern pulse survives":
        "return [-1., -1.]",
    "manual_hold_radius <= 0 disables it":
        "if self.manual_hold_radius <= 0.0:",
    "the pinger branch keeps its own latch":
        "if not self.stopping_sequence:",
}
for label, needle in needles.items():
    check(f"master_control {label}", needle in source)

defaults = {
    "manual_hold_radius": sim.MANUAL_HOLD_RADIUS,
    "manual_reacquire_radius": sim.MANUAL_REACQUIRE_RADIUS,
    "manual_brake_time": sim.MANUAL_BRAKE_TIME,
}
for name, value in defaults.items():
    check(f"{name} default matches the harness",
          f"dbl('{name}', {value})" in source, f"harness {value}")

# The split gains cannot be matched as one literal, so check both columns.
check("manual_hold_kx defaults match the harness",
      f"{sim.MANUAL_HOLD_KX_SIM} if self.isSimulation else {sim.MANUAL_HOLD_KX_REAL}"
      in source,
      f"harness {sim.MANUAL_HOLD_KX_SIM} sim / {sim.MANUAL_HOLD_KX_REAL} real")

# The handover must not step: the hold at the re-acquire radius has to meet the
# pursuit law it hands back to, or the command jumps as the boat crosses.
for column, k_v, factor, kx in (("real", 0.15, 10, sim.MANUAL_HOLD_KX_REAL),
                                ("sim", 2.0, 7, sim.MANUAL_HOLD_KX_SIM)):
    pursuit = factor * math.log(5 * math.log(k_v * RE_ACQ + 1) + 1)
    hold = kx * (RE_ACQ - sim.MANUAL_HOLD_RADIUS)
    check(f"{column} column: hold meets the pursuit law at the handover",
          abs(pursuit - hold) < 0.5,
          f"pursuit {pursuit:.2f} N vs hold {hold:.2f} N")


# --------------------------------------- 5. run the node's own method
# Stronger than the text matching above: master_control cannot be imported
# (acados), but manual_keep_location touches nothing but numpy and its own
# attributes, so its real source can be executed against a stub. This is the
# shipped method, not a reimplementation of it.
print("\nmaster_control.manual_keep_location, executed")

import ast
import textwrap

tree = ast.parse(source)
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
fn = next(n for n in cls.body
          if isinstance(n, ast.FunctionDef) and n.name == "manual_keep_location")
ns = {"np": np}
exec(textwrap.dedent(ast.get_source_segment(source, fn)), ns)
keep_location = ns["manual_keep_location"]


class _Stub:
    manual_hold_radius = sim.MANUAL_HOLD_RADIUS
    manual_reacquire_radius = sim.MANUAL_REACQUIRE_RADIUS
    manual_hold_kx = sim.MANUAL_HOLD_KX_REAL
    manual_hold_umax = sim.MANUAL_HOLD_KX_REAL * (sim.MANUAL_REACQUIRE_RADIUS
                                                  - sim.MANUAL_HOLD_RADIUS)
    manual_brake_time = sim.MANUAL_BRAKE_TIME
    min_thrust = sim.MIN_THRUST
    manual_hold = False
    manual_brake_t0 = None

    class _Log:
        def info(self, message):
            pass

    def get_logger(self):
        return _Stub._Log()


node = _Stub()
seq = [(3.0, 0.0), (2.5, 0.5), (1.0, 1.0), (1.0, 1.5), (1.0, 2.5),
       (1.05, 3.0), (1.4, 3.5), (2.5, 4.0)]
out = [keep_location(node, d, 0.0, 0.0, t) for d, t in seq]

check("off station it defers to the pursuit law", out[0] is None and out[1] is None)
check("arrival fires the astern pulse",
      out[2] == [-1., -1.] and out[3] == [-1., -1.])
check("after the pulse, on station, it commands nothing", out[4] == [0., 0.])
check("just outside the radius it commands the breakaway floor",
      out[5] == [sim.MIN_THRUST, sim.MIN_THRUST], f"{out[5]}")
expected = sim.MANUAL_HOLD_KX_REAL * (1.4 - sim.MANUAL_HOLD_RADIUS)
check("further out it is proportional to the gap",
      abs(out[6][0] - expected) < 1e-9 and abs(out[6][1] - expected) < 1e-9,
      f"{out[6][0]:.3f} N, expected {expected:.3f}")
check("past the re-acquire radius it hands back to the pursuit law",
      out[7] is None)
check("the differential is zero when the target is dead ahead",
      all(o is None or o[0] == o[1] for o in out))

# Disabled by its own knob, exactly as the harness copy is.
node2 = _Stub()
node2.manual_hold_radius = 0.0
check("manual_hold_radius = 0 defers to the pursuit law at every range",
      all(keep_location(node2, d, 0.0, 0.0, t) is None for d, t in seq))


print()
if failures:
    print(f"FAILED ({len(failures)}): " + "; ".join(failures))
    sys.exit(1)
print("check_manual_hold: all checks passed")
