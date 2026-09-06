#!/usr/bin/env python3
"""
MPC solver-failure check (C6).

The MPC drove the simulated boat in a perfect circle forever because acados was
FAILING and the failure was being published as a command. On a non-zero status
acados leaves the primal iterate untouched, so `solver.get(0, 'u')` returns the
last SUCCESSFUL solve's command; ur_mpc.solve returned it anyway and
master_control published it verbatim. A constant asymmetric thruster pair is,
physically, a constant-radius circle -- and the latch is self-sustaining, since
a circling boat never regains a small heading error.

Measured across the recorded runs in ~/ros2_ws/data/MPC_data (see
CONTROLLERS.md C6): 654 of 656 ticks carrying ONE bit-identical command for
32.7 s, against 518 "ACADOS solver failed with status 4" lines for a 517-tick
frozen run in another. Healthy runs carry one distinct command per tick.

The trigger is the QP working-set budget. FULL_CONDENSING_QPOASES is a dense
active-set method, so the condensed QP has nv = N*nu variables and 2*nv bound
constraints -- the only constraints besides the initial state. acados defaults
qp_solver_iter_max to 50, which is BELOW nv = 60 at the simulation horizon of
30, so a solve whose optimum saturates the horizon fails by construction.

Two halves:

  STATIC   stdlib + ast only, always runs. Asserts the guards are in the tree.
  CLOSED   needs acados_template + casadi. Builds the REAL MPCController with
           the shipped simulation configuration (read out of master_control.py
           by AST, so this cannot drift from what ships), drives it through the
           regime that provoked the failure, and asserts every tick solves and
           no two consecutive commands are bit-identical while the state moves.
           It also REPRODUCES the defect at qp_solver_iter_max=50 and asserts
           the guard holds there anyway.

    ~/ros2_ws/.venv/bin/python3 check_mpc_solver.py   # both halves
    /usr/bin/python3 check_mpc_solver.py              # static half, CLOSED skips
    ... check_mpc_solver.py --no-repro                # skip the reproduction

exit 0 pass, 1 fail. The closed-loop half compiles acados solvers into a
TEMPORARY directory (never the operator's cache) and takes about 2-3 minutes --
every other check in this directory returns in seconds. The reproduction
deliberately provokes a failing solver, so acados prints hundreds of its own
"SQP_RTI: QP solver returned error status" lines to stderr; that is the point
of that half, not a fault. `2>/dev/null` to read the results alone.
"""

import argparse
import ast
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
MASTER = os.path.join(SRC, "master_control.py")
UR_MPC = os.path.join(SRC, "MPC", "ur_mpc.py")
UVR_MPC = os.path.join(SRC, "MPC", "uvr_mpc.py")

failures = []


def check(label, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {label}{'  -- ' + detail if detail else ''}")
    if not ok:
        failures.append(label)


def source(path):
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def find_function(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    return None


# ======================================================================
#  Configuration, read out of master_control.py rather than retyped.
#  This is what stops the gate drifting from what actually ships; it is
#  also why this check locates code BY FILE (CLAUDE.md 2.1.1).
# ======================================================================

def _sim_branch(node):
    """Simulation value of a `X if self.isSimulation else Y` default."""
    if isinstance(node, ast.IfExp):
        return node.body
    return node


def read_master_config():
    """The shipped SIMULATION MPC configuration, extracted by AST."""
    tree = ast.parse(source(MASTER))
    fn = find_function(tree, "_declare_tuning_parameters")
    if fn is None:
        raise LookupError("_declare_tuning_parameters not found in master_control.py")

    cfg = {}
    for node in ast.walk(fn):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)):
            continue
        if node.func.id not in ("dbl", "integer", "arr") or len(node.args) != 2:
            continue
        name = node.args[0]
        if not (isinstance(name, ast.Constant) and isinstance(name.value, str)):
            continue
        try:
            cfg[name.value] = ast.literal_eval(_sim_branch(node.args[1]))
        except ValueError:
            pass

    # mpc_model: the `else:` branch of `if not self.isSimulation:` is simulation.
    model = None
    for node in ast.walk(fn):
        if not isinstance(node, ast.If):
            continue
        for stmt in node.orelse:
            for sub in ast.walk(stmt):
                if (isinstance(sub, ast.Assign)
                        and isinstance(sub.value, ast.Dict)
                        and any(isinstance(t, ast.Attribute) and t.attr == "mpc_model"
                                for t in sub.targets)):
                    model = ast.literal_eval(sub.value)
    if model is None:
        raise LookupError("simulation mpc_model dict not found in master_control.py")

    missing = [k for k in ("mpc_horizon", "mpc_time", "mpc_Q_diag", "mpc_R_diag",
                           "thrust_limit", "mpc_qp_iter_max") if k not in cfg]
    if missing:
        raise LookupError(f"defaults not found in master_control.py: {missing}")
    cfg["mpc_model"] = model
    return cfg


# ======================================================================
#  1. STATIC -- the guards are in the tree
# ======================================================================

print("[1] static: the guards are in the tree")

master_text = source(MASTER)
master_tree = ast.parse(master_text)
timer = find_function(master_tree, "timer_callback")
timer_src = ast.unparse(timer)

check("master_control: the MPC branch tests controller.last_status",
      "self.controller.last_status != 0" in timer_src)

check("master_control: a failed solve commands zero thrust",
      "u = [0.0, 0.0]" in timer_src)

check("master_control: the failure is reported through the ROS logger, not print",
      "MPC solve FAILED" in timer_src
      and "get_logger().error" in timer_src
      and "print(" not in timer_src,
      "a bare print never reaches /rosout, which is why this went unseen for days")

# The fail-safe must NOT be an early return: falling through is what keeps
# /thruster_input alive and the .npy recording. check_watchdog.py asserts the
# same invariant from the other side (returns == zero-publishes).
zero_publishes = timer_src.count("self.publish_thrust([0.0, 0.0])")
returns = sum(1 for n in ast.walk(timer) if isinstance(n, ast.Return))
check("master_control: the MPC fail-safe did not add an early return",
      zero_publishes == returns, f"{zero_publishes} zero-publishes, {returns} returns")

check("master_control: mpc_qp_iter_max is a declared parameter",
      "integer('mpc_qp_iter_max'" in master_text)

check("master_control: the budget and the logger reach the controller",
      "qp_solver_iter_max = self.mpc_qp_iter_max" in master_text
      and "logger = self.get_logger().warning" in master_text)

for label, path in (("ur_mpc", UR_MPC), ("uvr_mpc", UVR_MPC)):
    text = source(path)
    tree = ast.parse(text)
    solve_src = ast.unparse(find_function(tree, "solve"))

    check(f"{label}: the old print-and-return-the-stale-iterate is gone",
          "ACADOS solver failed with status" not in text
          and "for i in range(self.N)]" not in solve_src)

    check(f"{label}: a non-zero status returns zeros, never solver.get",
          "return np.zeros(self.model.u.size()[0])" in solve_src)

    # The only read of the iterate must sit AFTER the failure branch returned.
    fn = find_function(tree, "solve")
    guard_line = next((n.lineno for n in ast.walk(fn)
                       if isinstance(n, ast.If)
                       and "status != 0" in ast.unparse(n.test)), None)
    get_lines = [n.lineno for n in ast.walk(fn)
                 if isinstance(n, ast.Call)
                 and "self.solver.get(0, 'u')" in ast.unparse(n)]
    check(f"{label}: the iterate is read only on the success path",
          guard_line is not None and get_lines and min(get_lines) > guard_line,
          f"guard at line {guard_line}, read at {get_lines}")

    check(f"{label}: qp_solver_iter_max is set on the OCP",
          "ocp.solver_options.qp_solver_iter_max = self.qp_solver_iter_max" in text)

    # SQP_RTI takes one step per call from the previous iterate and re-seeds
    # only stage 0, so ONE bad step latches forever. Measured in Gazebo before
    # this existed: the boat was on the circle at 0.05 m and 0.10 rad, one tick
    # commanded [-20, -20], and the QP then failed 3291 consecutive times. The
    # working-set budget does not touch that -- it is a poisoned linearisation
    # point, not an iteration cap.
    check(f"{label}: a failed solve resets and retries",
          "self.solver.reset()" in solve_src
          and "self._seed_all_stages(x_current)" in solve_src
          and solve_src.count("self.solver.solve()") == 2)

    check(f"{label}: the retry re-applies the reference after the reset",
          solve_src.count("self._apply_reference(x_refs, x_current)") == 2,
          "reset() zeroes the iterate, so the yrefs have to go back in")

    check(f"{label}: the budget derivation exceeds the condensed QP size",
          "max(50, 4 * self.nu * self.N)" in text)

# The derived budget must actually clear nv = N*nu for the shipped horizon.
try:
    cfg = read_master_config()
except LookupError as exc:
    check("master_control: the shipped MPC configuration is extractable", False, str(exc))
    cfg = None

if cfg is not None:
    N, nu = int(cfg["mpc_horizon"]), 2
    derived = max(50, 4 * nu * N)
    check("budget clears the condensed QP at the shipped simulation horizon",
          derived > nu * N,
          f"N={N}, nv={nu * N} variables, budget={derived} (acados default 50)")

    check("the shipped default defers the derivation to ur_mpc",
          int(cfg["mpc_qp_iter_max"]) == 0,
          "0 means 'derive from the horizon', so the arithmetic lives in one place")


# ======================================================================
#  2. CLOSED LOOP -- needs acados + casadi
# ======================================================================

parser = argparse.ArgumentParser()
parser.add_argument("--no-repro", action="store_true",
                    help="skip the reproduction of the pre-fix configuration")
args = parser.parse_args()

print()
print("[2] closed loop: the real MPCController against the provoking regime")

sys.path.insert(0, os.path.join(SRC, "MPC"))
try:
    import numpy as np
    import casadi as ca                                            # noqa: F401
    import acados_template                                         # noqa: F401
    import ur_mpc
    HAVE_ACADOS = True
except ImportError as exc:
    HAVE_ACADOS = False
    print(f"  SKIP  closed-loop half: {exc}")
    print("        needs acados_template + casadi: run with "
          "~/ros2_ws/.venv/bin/python3 (/usr/bin/python3 has neither).")


# --- duck-typed nav_msgs/Path stubs: no rclpy, no nav_msgs -------------------

class _Q:
    def __init__(self, yaw):
        self.x = self.y = 0.0
        self.z = float(np.sin(yaw / 2.0))
        self.w = float(np.cos(yaw / 2.0))


class _Pos:
    def __init__(self, x, y):
        self.x, self.y, self.z = float(x), float(y), 0.0


class _Pose:
    def __init__(self, x, y, yaw):
        self.position = _Pos(x, y)
        self.orientation = _Q(yaw)


class _Stamped:
    def __init__(self, x, y, yaw):
        self.pose = _Pose(x, y, yaw)


class _Path:
    def __init__(self, poses):
        self.poses = poses


def make_plant(model_kwargs):
    """RK4 on the MPC's OWN model.

    This asserts nothing about plant fidelity -- it is a SOLVER gate, not a
    tracking gate. The Gazebo hull carries quadratic drag this model does not
    (TODO.md 0.2: the offline harness already models a third system).
    """
    model = ur_mpc.export_underwater_model(**model_kwargs)
    f = ca.Function("f", [model.x, model.u], [model.f_expl_expr])

    def step(x, u, dt):
        x = np.asarray(x, dtype=float)
        k1 = np.array(f(x, u)).ravel()
        k2 = np.array(f(x + 0.5 * dt * k1, u)).ravel()
        k3 = np.array(f(x + 0.5 * dt * k2, u)).ravel()
        k4 = np.array(f(x + dt * k3, u)).ravel()
        return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    return step


def score(cmds, states):
    """The exact metrics the diagnosis was measured with on the .npy logs."""
    cmds = np.asarray(cmds)
    states = np.asarray(states)
    moved = np.linalg.norm(np.diff(states, axis=0), axis=1) > 1e-9
    same = ~np.any(np.diff(cmds, axis=0) != 0, axis=1)
    longest, run = 1, 1
    for frozen, m in zip(same, moved):
        run = run + 1 if (frozen and m) else 1
        longest = max(longest, run)
    distinct = len({tuple(c) for c in cmds})
    return longest, distinct


def run_case(ctrl, plant, x0, window, ticks, dt, sliding=None):
    """Close the loop. Returns (statuses, commands, states)."""
    x = np.array(x0, dtype=float)
    statuses, cmds, states = [], [], [x.copy()]
    for k in range(ticks):
        path = _Path(window(k) if sliding else window)
        u = ctrl.solve(path=path, x_current=x)
        statuses.append(ctrl.last_status)
        cmds.append(np.asarray(u, dtype=float).copy())
        x = plant(x, np.asarray(u, dtype=float), dt)
        states.append(x.copy())
    return np.array(statuses), np.array(cmds), np.array(states)


if HAVE_ACADOS and cfg is not None:
    N = int(cfg["mpc_horizon"])
    T = float(cfg["mpc_time"])
    limit = float(cfg["thrust_limit"])
    model_kwargs = dict(cfg["mpc_model"])
    plant = make_plant(model_kwargs)
    CONTROL_DT = 0.05
    FROZEN_MAX = 20          # ticks, = 1 s at 20 Hz; the failures ran 337-654
    DISTINCT_MIN = 0.5       # of ticks; the failures had 3-4 over hundreds

    # Scenario A -- the provoking regime. advance_governor freezes tau when
    # e_along (28 m) far exceeds gov_Lmax (3.0 m), so the OCP sees a STATIC,
    # unreachable reference; the boat starts pointing the wrong way, which is
    # the measured discriminator between the frozen runs and the clean ones.
    FAR, FAR_BEARING, FAR_YAW = 28.0, 0.35, np.pi
    far_window = [_Stamped(FAR * np.cos(FAR_BEARING), FAR * np.sin(FAR_BEARING), FAR_YAW)
                  for _ in range(N + 1)]
    x0_far = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Scenario B -- normal tracking at the speed d_u's secant is exact at.
    SPEED = 0.45
    def line_window(k):
        t0 = k * CONTROL_DT * SPEED
        return [_Stamped(t0 + i * (T / N) * SPEED, 0.0, 0.0) for i in range(N + 1)]
    x0_line = [0.0, 0.0, 0.0, SPEED, 0.0, 0.0]

    build_dir = tempfile.mkdtemp(prefix="check_mpc_solver_")
    try:
        common = dict(model_kwargs,
                      horizon=N, time=T,
                      Q_weight=np.diag(cfg["mpc_Q_diag"]),
                      R_weight=np.diag(cfg["mpc_R_diag"]),
                      input_bounds={"lower": np.array([-limit, -limit]),
                                    "upper": np.array([limit, limit]),
                                    "idx": np.array([0, 1])},
                      logger=lambda *_a, **_k: None)

        print(f"  ..    building the shipped configuration "
              f"(N={N}, T={T}, budget={max(50, 4 * 2 * N)}) in {build_dir}")
        ctrl = ur_mpc.MPCController(build_dir=os.path.join(build_dir, "shipped"),
                                    qp_solver_iter_max=int(cfg["mpc_qp_iter_max"]),
                                    **common)

        for label, x0, window, ticks, sliding in (
                ("A far static reference, pi heading error", x0_far, far_window, 1200, False),
                ("B straight-line tracking at 0.45 m/s", x0_line, line_window, 600, True)):
            st, cmds, states = run_case(ctrl, plant, x0, window, ticks, CONTROL_DT,
                                        sliding=sliding)
            longest, distinct = score(cmds, states)
            bad = int(np.count_nonzero(st))

            check(f"{label}: every tick solves",
                  bad == 0, f"{bad}/{ticks} non-zero statuses")
            # Thresholds are set to separate the two REGIMES, not to demand a
            # new number every tick. A healthy run can legitimately repeat a
            # command for a few ticks while both thrusters sit on the +/-20 N
            # bound during a full-throttle transit -- measured here at 6 ticks
            # and 83 % distinct. The failure signature is nothing like that:
            # the recorded runs carried 337-654 bit-identical ticks and 3-4
            # distinct commands over hundreds. 20 ticks is 1 s at 20 Hz and
            # leaves a factor of ~17 of margin.
            check(f"{label}: no frozen command",
                  longest <= FROZEN_MAX,
                  f"longest bit-identical run {longest} tick(s), limit "
                  f"{FROZEN_MAX} (the recorded failures ran 337-654)")
            check(f"{label}: the command actually varies",
                  distinct >= DISTINCT_MIN * ticks,
                  f"{distinct} distinct of {ticks} ticks, floor "
                  f"{int(DISTINCT_MIN * ticks)} (the recorded failures had 3-4)")
            check(f"{label}: every command inside the input bounds",
                  bool(np.all(np.abs(cmds) <= limit + 1e-6)),
                  f"max |u| = {np.max(np.abs(cmds)):.3f} N of {limit:.1f}")

            # The failure that mattered in the water was not that a solve
            # failed, but that failures LATCHED. Whatever the count, no run of
            # consecutive failures may survive the reset-and-retry.
            worst = 0
            run_len = 0
            for bad_tick in (st != 0):
                run_len = run_len + 1 if bad_tick else 0
                worst = max(worst, run_len)
            check(f"{label}: no failure latches",
                  worst <= 2,
                  f"longest consecutive-failure run {worst} tick(s) "
                  f"(Gazebo before the reset-and-retry: 3291)")

        # Scenario A must also CONVERGE, not merely solve.
        st, cmds, states = run_case(ctrl, plant, x0_far, far_window, 1200, CONTROL_DT)
        final = float(np.hypot(states[-1][0] - far_window[0].pose.position.x,
                               states[-1][1] - far_window[0].pose.position.y))
        check("A far static reference: the boat closes on the target",
              final < 1.0, f"final range {final:.2f} m of {FAR:.1f} m")

        shipped_recoveries = ctrl.recoveries

        # --- reproduction: the pre-fix budget, and the guards holding anyway --
        #
        # Note what this can and cannot show. With the reset-and-retry in place
        # the budget-50 configuration no longer FAILS -- the retry rescues it,
        # so the caller sees status 0. What it still shows is the cap being hit
        # on the FIRST attempt, counted by `recoveries`. That is the honest
        # reproduction: the budget is why the first attempt fails, the retry is
        # why the boat no longer cares. Both changes are load-bearing, and this
        # separates their contributions.
        if not args.no_repro:
            print(f"  ..    building the PRE-FIX budget "
                  f"(qp_solver_iter_max=50, acados' default)")
            old = ur_mpc.MPCController(build_dir=os.path.join(build_dir, "prefix"),
                                       qp_solver_iter_max=50, **common)
            st, cmds, states = run_case(old, plant, x0_far, far_window, 400, CONTROL_DT)
            bad_idx = np.flatnonzero(st)

            rate_old = old.recoveries / 400.0
            rate_new = shipped_recoveries / 1800.0
            check("reproduction: the acados default budget hits the cap, the "
                  "shipped budget does not",
                  old.recoveries > 0 and rate_old > rate_new,
                  f"budget 50: {old.recoveries} first-attempt failures in 400 ticks "
                  f"({rate_old:.2%}); shipped budget {max(50, 4 * 2 * N)}: "
                  f"{shipped_recoveries} in 1800 ({rate_new:.2%}). Before the "
                  f"reset-and-retry existed, budget 50 failed 396/400 here -- one "
                  f"bad step poisoned every later solve")

            check("reproduction: the retry rescues what the cap loses",
                  bad_idx.size == 0,
                  f"{bad_idx.size}/400 ticks still returned a non-zero status "
                  f"after the reset-and-retry"
                  + (f" (codes {sorted(set(st[bad_idx].tolist()))})" if bad_idx.size else ""))

            if bad_idx.size:
                check("reproduction: the guard held -- every finally-failing tick "
                      "commanded zero, never the previous command",
                      bool(np.all(cmds[bad_idx] == 0.0)),
                      "this is the assertion that encodes the fail-safe")
    finally:
        shutil.rmtree(build_dir, ignore_errors=True)

elif HAVE_ACADOS:
    print("  SKIP  closed-loop half: the shipped configuration could not be read")


print()
if failures:
    print(f"FAILED ({len(failures)}): " + "; ".join(failures))
    sys.exit(1)
print("check_mpc_solver: all checks passed")
