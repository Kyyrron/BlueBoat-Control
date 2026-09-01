#!/usr/bin/env python3
"""
Trajectory library check (F1, F7, F9, and the harness copy).

The hard-coded shapes in path_generation.single_pose are the REFERENCE
CONDITIONS for existing field data. The shape is not versioned in the code, in
the position CSV or in the .npy log, so a formula that moves invalidates every
earlier run on that shape and NOTHING raises an error. This is the guard: the
expected poses below are embedded, so a shape that moves fails here rather than
silently at analysis time months later.

It also pins the three things the F1/F7/F9 rework is allowed to be:

  F1  'fsin' is read out of a cumulative table instead of re-integrated from
      t=0 per pose. The table must reproduce the original Euler loop BIT FOR
      BIT (the loop is embedded below as the oracle) and must stay pure in t --
      the same t gives the same pose whatever order poses are asked for.
  F7  'sin' and 'kin_square' hold their last pose past t = 500 instead of
      teleporting back to t = 50. Below 500 they must not have moved at all.
  F9  an unrecognised shape raises, naming the shape and the valid set,
      instead of leaving a local unbound.

Finally it checks that sim.py's copy of single_pose still agrees with the real
one. docs/controllers is only evidence for the real code while that holds, and
nothing else in the repository enforces it.

    python3 check_trajectory_library.py      # exit 0 pass, 1 fail

Needs numpy and a sourced workspace (it imports path_generation, which imports
rclpy and blueboat_interfaces). Skips cleanly, exit 0, without one.
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "_custom_libraries")))
sys.path.insert(0, HERE)

try:
    import rclpy
    import path_generation as pg
except ImportError as exc:                       # pragma: no cover
    print(f"SKIP  check_trajectory_library: {exc}")
    print("      needs a sourced ROS 2 workspace (source install/setup.bash).")
    sys.exit(0)

import sim

failures = []


def check(label, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {label}{'  -- ' + detail if detail else ''}")
    if not ok:
        failures.append(label)


# ---------------------------------------------------------------- the oracles

# The turn radius the 'fsin' branch of single_pose passes in. Mirrored here, not
# imported: the branch holds a literal, and the REFERENCE poses below pin the
# pair together -- change one without the other and this check fails.
FSIN_RADIUS = 1.5
FSIN_RADIUS_ORIGINAL = 0.1      # V/A of the pre-radius constants (A = 1, f = 0.05)


def fsin_original(t, radius=FSIN_RADIUS_ORIGINAL):
    """The pre-F1 fsin: Euler integration from t=0, verbatim. The reference.

    radius = FSIN_RADIUS_ORIGINAL reproduces the original constants exactly.
    """
    v, dt = 0.1, 0.01
    A = v / radius
    f = A / 20.0
    x = y = 0.0
    yaw = 0.0
    for i in range(int(t / dt)):
        omega = A * np.sin(2 * np.pi * f * (i * dt))
        yaw += omega * dt
        x += v * np.cos(yaw) * dt
        y += v * np.sin(yaw) * dt
    return x, y, yaw


# Poses every shape must still return: (t, x, y, quat_z, quat_w). Captured from
# the tree and cross-checked against the pre-rework code, so a formula that
# moves shows up here. If you change a shape deliberately, update this table AND
# append a row to the shape revision record in TRAJECTORY_SYSTEM.md.
REFERENCE = {
    'station_keeping': [
        (0.0, 0.0, 0.0, 0.0, 1.0),
        (60.0, 0.0, 0.0, 0.0, 1.0),
        (500.0, 0.0, 0.0, 0.0, 1.0),
    ],
    'circle': [
        (1.0, -0.012793174789522244, 0.3196587758766908, 0.7348179005608031, 0.6782644418037951),
        (12.0, -1.7059200557101732, 3.276766273203993, 0.9537273112113855, 0.3006729383391543),
        (123.456, -7.598821880151624, -1.7459899985228815, -0.5308024587213588, 0.8474955751007555),
        (500.0, -6.667752246609048, 2.9804526419173953, -0.9341073708303957, 0.35699218445272096),
    ],
    'straight_line': [
        (0.0, 0.0, 1.0, 0.0, 1.0),
        (7.3, 3.65, 1.0, 0.0, 1.0),
        (500.0, 250.0, 1.0, 0.0, 1.0),
    ],
    'sin': [
        (0.0, 0.5, 0.0, 5.357829746269671e-17, 1.0),
        (12.0, 3.8599999999999994, 3.881453632839548, 0.500811716712011, 0.8655562514383272),
        (123.456, 35.06768, 3.4822186580946846, -0.5019244140739987, 0.8649114882786985),
        (500.0, 140.5, 1.2833827891979652, 0.4505537080226237, 0.8927493243834265),
    ],
    # 2026-09-01: turn radius 0.1 m -> 1.5 m (see TRAJECTORY_SYSTEM.md's shape
    # revision record). The shape is unchanged, scaled 15x and 15x slower per
    # cycle; the authored speed is still 0.1 m/s.
    'fsin': [
        (1.0, 0.09999999512718812, 2.3268218476744032e-05, 0.0003455626793334877, 0.9999999402932156),
        (12.0, 1.1987968682151866, 0.04005688193158172, 0.049939222745582094, 0.998752258586466),
        (123.456, 2.111901093555705, 0.4017869497382728, 0.19706565415777444, -0.9803902936848),
        (500.0, 18.184613698759534, -1.3685363371190256, 0.6846505699395324, -0.7288714544290189),
    ],
    'square': [
        (0.0, 0.0, 2.0, 0.0, 1.0),
        (7.3, 3.65, -2.0, 0.0, 1.0),
        (500.0, 250.0, -2.0, 0.0, 1.0),
    ],
    'kin_square': [
        (12.0, 3.5999999999999996, 0.0, 0.0, 1.0),
        (60.0, 10.0, 2.0000000000000013, -0.7071067811865475, 0.7071067811865476),
        (300.0, 45.0, 4.999999999999994, 0.7071067811865475, 0.7071067811865476),
        (500.0, 75.0, 4.999999999999989, 0.7071067811865475, 0.7071067811865476),
    ],
    'seabed_scanning': [
        (7.3, 2.6366197723675815, 1.2866197723675814, 0.7071067811865475, 0.7071067811865476),
        (60.0, 7.967399961259224, 3.86970571409326, -0.9589242746631385, 0.28366218546322625),
        (500.0, 6.159990890683292, 3.5233711183157106, 3.6739403974420594e-16, -1.0),
    ],
}

CLAMP_AT = 500.0                    # where 'sin' and 'kin_square' hold (F7)
CLAMPED_SHAPES = ('sin', 'kin_square')

rclpy.init()
node = pg.PathGeneration()


def pose(t, shape):
    p = node.single_pose(float(t), shape)
    q = p.pose.orientation
    return p.pose.position.x, p.pose.position.y, q.z, q.w


# ----------------------------------------------- 1. every shape is unchanged
print("Every shape still returns its reference poses (field-data comparability)")
for shape, rows in REFERENCE.items():
    bad = [r[0] for r in rows if pose(r[0], shape) != tuple(r[1:])]
    check(f"{shape}: {len(rows)} reference poses bit-identical", not bad,
          f"moved at t={bad}" if bad else "")

# ------------------------------------------------------- 2. F7, the clamp
print("F7: the parameter range clamps instead of wrapping backwards")
for shape in CLAMPED_SHAPES:
    at_clamp = pose(CLAMP_AT, shape)
    beyond = [t for t in (500.001, 501.0, 600.0, 1000.0, 1e6)
              if pose(t, shape) != at_clamp]
    check(f"{shape}: holds the t={CLAMP_AT:.0f} pose past the end", not beyond,
          f"moved at t={beyond}" if beyond else "")
    # the old bug: t>500 jumped back to the t=50 pose
    check(f"{shape}: does not jump back to the t=50 pose",
          pose(501.0, shape) != pose(50.0, shape))
    # and progress never decreases across the boundary
    check(f"{shape}: x is non-decreasing across t=499 -> 501",
          pose(501.0, shape)[0] >= pose(499.0, shape)[0],
          f"{pose(499.0, shape)[0]:.3f} -> {pose(501.0, shape)[0]:.3f} m")

# --------------------------------------------- 3. F1, identity and purity
print("F1: fsin matches the original integration and stays pure in t")
ts = [0.0, 0.005, 0.5, 1.0, 7.3, 12.0, 49.99, 50.0, 123.456, 300.0, 500.0]
for radius, label in ((FSIN_RADIUS_ORIGINAL, "original 0.1 m"),
                      (FSIN_RADIUS, f"in-use {FSIN_RADIUS} m")):
    bad = [t for t in ts
           if pg._fsin_state(t, radius) != fsin_original(t, radius)]
    check(f"radius {label}: bit-identical to the Euler loop at {len(ts)} sampled t",
          not bad, f"differs at t={bad}" if bad else "")

order_ts = list(np.linspace(0.0, 1000.0, 401))


def evaluated(sequence):
    pg._fsin_yaw = np.zeros(1)
    pg._fsin_x = np.zeros(1)
    pg._fsin_y = np.zeros(1)
    return {t: pg._fsin_state(t, FSIN_RADIUS) for t in sequence}


forward = evaluated(order_ts)
reverse = evaluated(reversed(order_ts))
shuffled_ts = list(order_ts)
np.random.default_rng(0).shuffle(shuffled_ts)
shuffled = evaluated(shuffled_ts)
check("forward / reverse / shuffled evaluation give identical poses",
      all(forward[t] == reverse[t] == shuffled[t] for t in order_ts),
      f"{len(order_ts)} values of t")

# The cost F1 removed: path_publisher asks for a whole path in one call.
window = np.linspace(0.0, 1000.0, 10001)
import time
t0 = time.time()
for t in window:
    node.single_pose(float(t), 'fsin')
elapsed = time.time() - t0
check(f"the {len(window)}-pose fsin window evaluates in under a second",
      elapsed < 1.0, f"{elapsed:.3f} s")

# ------------------------------------------------ 4. F9, diagnosable typo
print("F9: an unrecognised shape is diagnosable")
try:
    node.single_pose(10.0, 'circel')
    check("single_pose raises on an unknown shape", False, "it returned a pose")
except ValueError as exc:
    text = str(exc)
    check("single_pose raises ValueError naming the shape", "'circel'" in text, text[:60])
    check("...and listing every valid shape",
          all(s in text for s in pg.SHAPES))
except Exception as exc:                          # the pre-F9 UnboundLocalError
    check("single_pose raises ValueError on an unknown shape", False,
          f"raised {type(exc).__name__} instead")

check("is_valid_shape accepts every advertised shape",
      all(pg.is_valid_shape(s) for s in pg.SHAPES))
check("is_valid_shape accepts a from_yaml path",
      pg.is_valid_shape('from_yaml:/tmp/mission.yaml'))
check("is_valid_shape rejects a typo", not pg.is_valid_shape('circel'))

# ------------------------------------- 5. the harness copy has not drifted
# Position is compared exactly: both sides compute it from the same formula and
# nothing rounds it. Yaw cannot be -- path_generation returns a quaternion, and
# recovering the angle from it costs an ULP or so (circle at t=0 is yaw = pi/2
# exactly, and round-trips to 2.2e-16 off). YAW_EPS is far below any real
# divergence: a copy that had actually drifted would be off by a formula, not by
# a rounding step.
YAW_EPS = 1e-12
print("sim.py's copy of single_pose still agrees with path_generation")
worst_xy = worst_yaw = 0.0
worst_at = None
for shape in ("station_keeping", "circle", "straight_line", "sin", "kin_square"):
    for t in np.linspace(0.0, 600.0, 1201):
        x, y, qz, qw = pose(t, shape)
        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        sx, sy, syaw = sim.single_pose(float(t), shape)
        dxy = max(abs(x - sx), abs(y - sy))
        dyaw = abs(math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw)))
        if max(dxy, dyaw) > max(worst_xy, worst_yaw):
            worst_at = (shape, round(float(t), 2))
        worst_xy = max(worst_xy, dxy)
        worst_yaw = max(worst_yaw, dyaw)
check("position identical over t in [0, 600] on all five shared shapes",
      worst_xy == 0.0, f"max |d| {worst_xy:.3e}" + (f" at {worst_at}" if worst_xy else ""))
check(f"yaw agrees to within {YAW_EPS:g} rad (quaternion round-trip)",
      worst_yaw < YAW_EPS, f"max |d| {worst_yaw:.3e}")

rclpy.shutdown()

print()
if failures:
    print(f"FAILED ({len(failures)}): " + "; ".join(failures))
    sys.exit(1)
print("All trajectory-library checks passed.")
