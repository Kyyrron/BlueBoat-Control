#!/usr/bin/env python3
"""Regression guard for PID.PIDLoS -- the one real controller class the harness imports.

PIDLoS is the only controller `sim.py` does not reimplement, so the harness's
credibility rests on this class continuing to mean what its docstring says. Three
things are checked, and each fails loudly with the values it expected.

  1. The Delta = 1/los_gain re-parameterisation is an exact algebraic identity.
  2. The documented point-following defaults (u_ff = 0.0, psi_path = None ->
     error projected onto the BOAT heading, gamma_p taken from ref[2]) reproduce
     the pre-rework point law atan2(los_gain*ev, 1).
  3. master_control still constructs the class at the lookahead that identity
     was claimed for.

Provenance of the 0.4 in check 3
--------------------------------
The pre-rework `los_gain` is NOT recoverable from this repository. `PID.py`
appears only in the initial commit and already carries the reworked
`lookahead`/`los_gain` signature; `git log -S los_gain` finds no earlier form;
and no numeric LoS gain appears in CONTROLLERS.md, TRAJECTORY_SYSTEM.md or
summary_controllers.md. So 0.4 is *inferred* from master_control's
`pid_lookahead = 2.5` (Delta = 1/g), not recovered from history. Check 3
asserts it as a live coupling between two files and says so -- it does not
claim to have established what the old value was.

Run:  python3 check_pid_equivalence.py     (exit 0 pass, 1 fail; numpy only)
"""
import ast
import math
import os
import sys

import numpy as np

SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(SRC, "PID"))
import PID as PIDmod  # noqa: E402

RADIUS = 0.295
B_MAT = np.array([[1.0, 1.0], [0.0, 0.0], [RADIUS, -RADIUS]])
THR_LIM = 20.0
LIMITS = {"min": np.array([-THR_LIM] * 2), "max": np.array([THR_LIM] * 2)}

# Exactly how master_control.py:283-286 wires it (_declare_tuning_parameters).
OUTER = {"x": (3.0, 0.01, 0.0), "psi": (3.0, 0.01, 0.0)}
INNER = {"u": (1.0, 0.0, 0.0), "r": (1.5, 0.0, 0.0)}
DT = 0.05

FAILURES = []


def check(name, ok, detail=""):
    print(("  PASS  " if ok else "  FAIL  ") + name + (("\n         " + detail) if detail else ""))
    if not ok:
        FAILURES.append(name)


def make(**kw):
    return PIDmod.PIDLoS(dt=DT, B=B_MAT, outer_gains=OUTER, inner_gains=INNER,
                         thruster_limits=LIMITS, **kw)


def states_and_refs():
    """A swept set of states and refs, including sign flips and a +/-pi crossing."""
    out = []
    for psi in (-3.0, -1.2, 0.0, 0.7, 3.05):
        for ex, ey in ((5.0, 0.0), (0.0, 4.0), (-3.0, -2.5), (1.0, -6.0), (0.0, 0.0)):
            for u, r in ((0.0, 0.0), (0.4, 0.15), (-0.2, -0.3)):
                state = [1.0, -2.0, psi, u, 0.05, r]
                ref = [1.0 + ex, -2.0 + ey, psi + 0.4]
                out.append((state, ref))
    return out


# --------------------------------------------------------------------------- 1
def check_reparameterisation():
    """PIDLoS(lookahead=1/g) and PIDLoS(los_gain=g) must be bit-identical.

    Both reach self.lookahead through the same 1.0/g expression (PID.py:106-109),
    so equality here is exact, not approximate.
    """
    worst = 0.0
    for g in (0.4, 0.25, 1.0, 2.0, 0.125):
        a, b = make(lookahead=1.0 / g), make(los_gain=g)
        for state, ref in states_and_refs():
            ta, _ = a.compute(state, ref)
            tb, _ = b.compute(state, ref)
            worst = max(worst, float(np.max(np.abs(np.asarray(ta) - np.asarray(tb)))))
    check("Delta = 1/los_gain is an exact identity", worst == 0.0,
          f"max |difference| over the sweep = {worst!r} (must be exactly 0.0)")


# --------------------------------------------------------------------------- 2
def prerework_point_law(state, ref, los_gain, ctrl):
    """The pre-rework point controller, as PID.py:84-88 and :157-175 describe it.

    Error projected onto the BOAT heading, gamma_p from ref[2], no speed
    feedforward, and the steering term written the old way: atan2(g*ev, 1).
    Shares `ctrl`'s PID objects so the integrator states stay in step.
    """
    x, y, psi, u, _v, r = state
    x_ref, y_ref, psi_ref = ref
    ex_w, ey_w = x_ref - x, y_ref - y
    c, s = math.cos(psi), math.sin(psi)                 # boat heading, not path tangent
    e_along = c * ex_w + s * ey_w
    ev = -s * ex_w + c * ey_w
    psi_des = PIDmod.wrap_angle(psi_ref + math.atan2(los_gain * ev, 1.0))
    epsi = PIDmod.wrap_angle(psi_des - psi)
    u_ref = 0.0 + ctrl.pid_x.update(e_along)            # u_ff = 0.0
    r_ref = ctrl.pid_psi.update(epsi)
    X = ctrl.pid_u.update(u_ref - u)
    N = ctrl.pid_r.update(r_ref - r)
    return ctrl.allocator.allocate(np.array([X, 0.0, N]))


def check_point_following_defaults():
    """compute(state, ref) with defaults must reproduce the pre-rework law.

    atan2(ev, Delta) and atan2(g*ev, 1) are algebraically identical but not
    bit-identical in floating point, so this one carries a tolerance. It is the
    assertion that fires if the projection frame, gamma_p or the default u_ff
    ever changes.
    """
    tol = 1e-12
    worst = 0.0
    for g in (0.4, 1.0, 2.5):
        new, old = make(los_gain=g), make(los_gain=g)
        for state, ref in states_and_refs():
            # master_control.py:431 pinger branch: compute(state, target), all defaults.
            t_new, _ = new.compute(state, ref)
            t_old = prerework_point_law(state, ref, g, old)
            worst = max(worst, float(np.max(np.abs(np.asarray(t_new) - np.asarray(t_old)))))
    check("point-following defaults reproduce the pre-rework point law", worst <= tol,
          f"max |difference| = {worst:.3e} over the sweep, tolerance {tol:.0e}")


# --------------------------------------------------------------------------- 3
def master_control_pid_lookahead():
    """Read the pid_lookahead default statically out of master_control.py.

    Static on purpose: master_control imports ur_mpc, which imports
    acados_template at module scope, so it cannot be imported on a machine
    without acados -- which is most of them, and every machine this check is
    meant to run on.
    """
    path = os.path.join(SRC, "master_control.py")
    tree = ast.parse(open(path, encoding="utf-8").read())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == "dbl" and len(node.args) == 2
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == "pid_lookahead"):
            return float(ast.literal_eval(node.args[1])), path
    raise LookupError(f"no dbl('pid_lookahead', ...) call found in {path}")


def check_master_control_coupling():
    expected_gain = 0.4
    delta, path = master_control_pid_lookahead()
    actual_gain = 1.0 / delta
    ok = actual_gain == expected_gain
    check("master_control's pid_lookahead still implies los_gain = 0.4", ok,
          f"{os.path.relpath(path, SRC)} declares pid_lookahead = {delta}, "
          f"so los_gain = 1/{delta} = {actual_gain}; expected {expected_gain}."
          + ("" if ok else "\n         The documented point-following equivalence was claimed"
                           f" for los_gain = {expected_gain} (Delta = {1.0/expected_gain}). Either"
                           " restore that lookahead or restate the equivalence for the new one."))


if __name__ == "__main__":
    print("PIDLoS equivalence check")
    check_reparameterisation()
    check_point_following_defaults()
    check_master_control_coupling()
    if FAILURES:
        print(f"\n{len(FAILURES)} check(s) failed: " + "; ".join(FAILURES))
        sys.exit(1)
    print("\nall checks passed")
