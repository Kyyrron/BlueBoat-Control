#!/usr/bin/env python3
"""
Zero-authored-speed hold check (F2), both controllers.

Two properties, and the second is the one that makes the harness admissible as
evidence for the shipped controller:

  1. The hold is INERT on any path that has an authored speed. Not "small" --
     bit-identical. Every trajectory in the library runs well above hold_speed,
     so the blend weight is exactly 0.0 and both laws are the same floats they
     were before the hold existed. Checked by running the real harness
     controllers with the hold on and disabled and comparing the logs exactly.

  2. sim.LoSController and master_control.los_guidance are the same law.
     sim.py is a verbatim reimplementation; if it drifts, every number in
     CONTROLLERS.md stops being evidence for what the boat runs. master_control
     cannot be imported without acados, so it is read statically -- the same
     approach check_pid_equivalence.py takes.

    python3 check_los_hold.py      # exit 0 pass, 1 fail

Needs numpy and scipy (through sim), no ROS.
"""

import ast
import os
import sys

import numpy as np

import sim

HERE = os.path.dirname(os.path.abspath(__file__))
MASTER = os.path.abspath(os.path.join(HERE, "..", "..", "master_control.py"))

failures = []


def check(label, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {label}{'  -- ' + detail if detail else ''}")
    if not ok:
        failures.append(label)


# ------------------------------------------------------------------ 1. inert
print("Inert on every path with an authored speed")

# The authored speed of each shape, read off its own parameterisation, against
# the gate. These are the paths master_control can be launched with.
authored = {"straight_line": 0.50, "circle": 4.0 * 0.08, "kin_square": 0.30,
            "sin": 0.4 * 0.7}
for shape, speed in sorted(authored.items()):
    check(f"{shape}: authored {speed:.3f} m/s is above hold_speed",
          speed > sim.HOLD_SPEED, f"gate {sim.HOLD_SPEED} m/s")

for shape, start, T in (("straight_line", (0., -4., 0.), 60),
                        ("circle", (0., 0., 0.), 60),
                        ("kin_square", (0., 0., 0.), 60)):
    for label, on_ctrl, off_ctrl in (
            ("LoS", lambda: sim.LoSController(),
             lambda: sim.LoSController(hold_kx=0.0)),
            ("PID", lambda: sim.PIDController(dt=0.05),
             lambda: sim.PIDController(dt=0.05, hold_speed=0.0))):
        on = sim.run(on_ctrl(), shape, start=start, T=T)
        off = sim.run(off_ctrl(), shape, start=start, T=T)
        same = all(np.array_equal(on[k], off[k]) for k in on)
        check(f"{shape} / {label}: identical with the hold on and disabled", same,
              f"max |dy| {np.abs(on['y'] - off['y']).max():.3e} m")

# The gate itself: at zero authored speed the term must actually engage.
hold = sim.run(sim.LoSController(), "station_keeping", start=(2., 1., 0.), T=150)
drift = sim.run(sim.LoSController(hold_kx=0.0), "station_keeping", start=(2., 1., 0.), T=150)
r_hold = float(np.hypot(hold["x"], hold["y"])[-1])
r_drift = float(np.hypot(drift["x"], drift["y"])[-1])
check("station_keeping: the hold term engages", r_hold < r_drift - 1.0,
      f"{r_drift:.3f} m without it, {r_hold:.3f} m with it")
check("station_keeping: settles inside the hold radius + a margin",
      r_hold <= sim.HOLD_RADIUS + 0.05, f"{r_hold:.3f} m vs radius {sim.HOLD_RADIUS} m")
tail_thr = hold["thr_r"][-400:]
check("station_keeping: no chatter at rest",
      int(np.sum(np.diff(np.sign(tail_thr)) != 0)) == 0,
      f"|thrust| {np.abs(tail_thr).mean():.3f} N over the last 20 s")

# PID had the same defect for a different reason: its along-track term cannot see
# a cross-track error, because the tangent it projects onto is meaningless when
# the reference does not move.
pid_hold = sim.run(sim.PIDController(dt=0.05), "station_keeping", start=(2., 1., 0.), T=150)
pid_off = sim.run(sim.PIDController(dt=0.05, hold_speed=0.0), "station_keeping",
                  start=(2., 1., 0.), T=150)
r_pid = float(np.hypot(pid_hold["x"], pid_hold["y"])[-1])
r_pid_off = float(np.hypot(pid_off["x"], pid_off["y"])[-1])
check("PID station_keeping: the hold engages", r_pid < r_pid_off - 0.5,
      f"{r_pid_off:.3f} m without it, {r_pid:.3f} m with it")
check("PID station_keeping: does not drive away while turning round",
      np.hypot(pid_hold["x"], pid_hold["y"]).max() <= 2.237,
      "slow_on_turn holds the surge down through the turn")

for force in (-4.0, -10.0):
    d = sim.run(sim.PIDController(dt=0.05), "station_keeping", start=(0., 0., 0.), T=150,
                force_world=(0., force))
    off = sim.run(sim.PIDController(dt=0.05, hold_speed=0.0), "station_keeping",
                  start=(0., 0., 0.), T=150, force_world=(0., force))
    r, r_off = np.hypot(d["x"], d["y"]), np.hypot(off["x"], off["y"])
    check(f"PID {abs(force):.0f} N current: error is bounded", r[-1] <= 2.5,
          f"{r[-1]:.3f} m held vs {r_off[-1]:.3f} m drifted")
    check(f"PID {abs(force):.0f} N current: steady", r[-400:].std() < 0.06,
          f"std {r[-400:].std():.4f} m over the last 20 s")

# Bounded, not merely smaller, under a steady disturbance.
for force, cap in ((-4.0, 1.5), (-10.0, 2.0)):
    d = sim.run(sim.LoSController(), "station_keeping", start=(0., 0., 0.), T=150,
                force_world=(0., force))
    r = np.hypot(d["x"], d["y"])
    off = sim.run(sim.LoSController(hold_kx=0.0), "station_keeping", start=(0., 0., 0.),
                  T=150, force_world=(0., force))
    r_off = np.hypot(off["x"], off["y"])
    check(f"{abs(force):.0f} N current: error is bounded", r[-1] <= cap,
          f"{r[-1]:.3f} m held vs {r_off[-1]:.3f} m drifted")
    check(f"{abs(force):.0f} N current: steady, not still moving",
          r[-400:].std() < 0.02, f"std {r[-400:].std():.4f} m over the last 20 s")

# Turns round rather than reversing when it is past the point.
past = sim.run(sim.LoSController(), "station_keeping", start=(3., 0., 0.), T=150)
r_past = np.hypot(past["x"], past["y"])
check("3 m past the hold point: comes back", r_past[-1] <= sim.HOLD_RADIUS + 0.05,
      f"{r_past[0]:.1f} m -> {r_past[-1]:.3f} m")
check("3 m past the hold point: does not overshoot outward",
      r_past.max() <= r_past[0] + 1e-9, f"max {r_past.max():.3f} m")

# ------------------------------------------------- 2. harness matches the node
print("\nsim.LoSController matches master_control.los_guidance")

with open(MASTER, "r", encoding="utf-8") as fh:
    master_src = fh.read()
fn = next((n for n in ast.walk(ast.parse(master_src))
           if isinstance(n, ast.FunctionDef) and n.name == "los_guidance"), None)
check("master_control.los_guidance exists", fn is not None)
if fn is not None:
    body = ast.unparse(fn)
    for label, needle in (
            ("blend weight is 1 - U_d / hold_speed",
             "1.0 - min(1.0, max(0.0, U_d / self.hold_speed))"),
            ("gap is the range outside hold_radius",
             "max(0.0, rng - self.hold_radius)"),
            ("steers at the hold point", "bearing = math.atan2(y_ref - y, x_ref - x)"),
            ("never commands reverse",
             "u_hold = min(self.los_hold_umax, w * self.los_hold_kx * gap)"),
            ("hold rides inside the same cos shaping",
             "u_cmd = (self.los_speed_scale * U_d + u_hold) * max(0.0, math.cos(psi_err))")):
        check(f"los_guidance: {label}", needle in body)

    for name, value in (("hold_speed", sim.HOLD_SPEED), ("hold_radius", sim.HOLD_RADIUS),
                        ("los_hold_kx", sim.HOLD_KX), ("los_hold_umax", sim.HOLD_UMAX)):
        check(f"{name} default matches the harness",
              f"'{name}', {value}" in master_src, f"harness {value}")

    pid_branch = ast.unparse(next(n for n in ast.walk(ast.parse(master_src))
                                 if isinstance(n, ast.FunctionDef)
                                 and n.name == "timer_callback"))
    for label, needle in (
            ("rotates the tangent toward the bearing",
             "psi_path = target[2] + w * g * _wrap(bearing - target[2])"),
            ("fades the rotation out inside hold_radius",
             "(rng - self.hold_radius) / self.hold_radius"),
            ("uses the class's own slow_on_turn", "slow_on_turn=slow")):
        check(f"PID branch: {label}", needle in pid_branch)

print()
if failures:
    print(f"FAILED ({len(failures)}): " + "; ".join(failures))
    sys.exit(1)
print("check_los_hold: all checks passed")
