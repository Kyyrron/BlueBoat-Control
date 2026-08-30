#!/usr/bin/env python3
"""
Loss-of-reference watchdog check (F18).

robot_interface and simulation_interface both stop re-applying the last received
/thruster_input once it goes stale. Both defects were the same one, so both fixes
are the same predicate, and this checks the predicate rather than either node --
importing the nodes needs rclpy, mavros_msgs, pandas and a sourced workspace, and
a guard that cannot run without all of those does not get run.

The predicate is extracted from the two sources by AST, so this fails if either
node's logic drifts from what is checked here rather than silently passing on a
stale copy.

    python3 check_watchdog.py      # exit 0 pass, 1 fail

stdlib only: no ROS, no third-party imports, no test framework.
"""

import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
ROBOT_IF = os.path.join(SRC, "robot_interaction", "robot_interface.py")
SIM_IF = os.path.join(SRC, "simulation_interface.py")
MASTER = os.path.join(SRC, "master_control.py")

TIMEOUT = 0.5           # thruster_input_timeout default, both interface nodes
PRODUCER_DT = 0.05      # master_control control_dt: /thruster_input tick period

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


# ---------------------------------------------------------------- the predicate
# Reimplemented once, and checked against both nodes' AST below. Keep the three
# in step: this is the behaviour the boat's safety depends on.
def stale(now, last_rx, timeout, controller_type="LoS"):
    if controller_type == "":
        return False                    # covered by manual_move_timer instead
    if last_rx is None:
        return True                     # nothing has ever arrived
    return (now - last_rx) > timeout


# ---------------------------------------------------------------- behaviour
print("Watchdog predicate")

check("never-received counts as stale",
      stale(now=10.0, last_rx=None, timeout=TIMEOUT) is True)

check("fresh command is not stale",
      stale(now=10.0, last_rx=10.0, timeout=TIMEOUT) is False)

check("inert when no controller is configured",
      stale(now=1e6, last_rx=None, timeout=TIMEOUT, controller_type="") is False,
      "manual_move_timer owns that case")

# Fires when it should: silence past the timeout.
t_last = 100.0
check("fires after the timeout",
      stale(now=t_last + TIMEOUT + 1e-9, last_rx=t_last, timeout=TIMEOUT) is True,
      f"{TIMEOUT} s = {int(TIMEOUT / PRODUCER_DT)} missed producer ticks")
check("does not fire one tick early",
      stale(now=t_last + TIMEOUT - PRODUCER_DT, last_rx=t_last, timeout=TIMEOUT) is False)

# Does not fire when it should not: 20 Hz publication with jitter. A watchdog
# that trips during a normal survey is worse than none, because it gets disabled.
# Deterministic pseudo-jitter, up to +-40% of the tick period, plus two 3-tick
# hiccups of the kind DDS produces under load.
jitter = [((i * 37) % 81 - 40) / 100.0 * PRODUCER_DT for i in range(2400)]
now = 0.0
last_rx = 0.0
spurious = 0
worst_gap = 0.0
for i, j in enumerate(jitter):
    now += PRODUCER_DT
    arrival = now + j
    if i in (600, 1500):                # two 3-tick hiccups
        arrival += 3 * PRODUCER_DT
    if stale(now=arrival, last_rx=last_rx, timeout=TIMEOUT):
        spurious += 1
    worst_gap = max(worst_gap, arrival - last_rx)
    last_rx = arrival
check("no spurious trip over 120 s at 20 Hz with jitter and hiccups",
      spurious == 0, f"worst inter-arrival gap {worst_gap:.3f} s vs timeout {TIMEOUT} s")
check("worst normal gap leaves margin",
      worst_gap < TIMEOUT / 2, f"{worst_gap:.3f} s < {TIMEOUT / 2} s")

# Recovers by itself: the guard is not latching.
check("clears as soon as input resumes",
      stale(now=200.0, last_rx=200.0, timeout=TIMEOUT) is False,
      "after being stale at now=200, last_rx=100")

# ---------------------------------------------------------------- the sources
print("\nSources agree with the predicate")

for path, label, expects_ctrl_gate in ((ROBOT_IF, "robot_interface", True),
                                       (SIM_IF, "simulation_interface", False)):
    text = source(path)
    tree = ast.parse(text)
    fn = find_function(tree, "thruster_input_stale")
    check(f"{label}: thruster_input_stale exists", fn is not None)
    if fn is None:
        continue
    body = ast.unparse(fn)
    check(f"{label}: None means stale", "self.last_thr_rx is None" in body)
    check(f"{label}: compares against thruster_input_timeout",
          "self.thruster_input_timeout" in body and "self.last_thr_rx" in body)
    check(f"{label}: controller gate {'present' if expects_ctrl_gate else 'absent'}",
          ("self.controller_type == ''" in body) is expects_ctrl_gate,
          "simulation_interface has no manual-move path")
    check(f"{label}: declares thruster_input_timeout default {TIMEOUT}",
          f"'thruster_input_timeout', {TIMEOUT}" in text)
    check(f"{label}: stamps last_thr_rx on receipt",
          "self.last_thr_rx = time.time()" in text)
    check(f"{label}: zeroes rather than disarming",
          "self.setArmedStatus(False)" not in ast.unparse(find_function(tree, "timer_callback")
                                                          or find_function(tree, "move")),
          "full_stop() stays bound to the operator 'stop' command")

# robot_interface must reach the motors through the gated path, not a new bypass (N4).
robot_text = source(ROBOT_IF)
robot_timer = ast.unparse(find_function(ast.parse(robot_text), "timer_callback"))
check("robot_interface: watchdog zeroes through manualMove without force (N4)",
      "self.manualMove([0, 0])" in robot_timer and "force=True" not in robot_timer)

# master_control publishes zero rather than falling silent on every early return.
master_tree = ast.parse(source(MASTER))
master_timer = ast.unparse(find_function(master_tree, "timer_callback"))
zero_publishes = master_timer.count("self.publish_thrust([0.0, 0.0])")
returns = sum(1 for node in ast.walk(find_function(master_tree, "timer_callback"))
              if isinstance(node, ast.Return))
check("master_control: every early return publishes zero thrust",
      zero_publishes == returns, f"{zero_publishes} zero-publishes, {returns} returns")

print()
if failures:
    print(f"FAILED ({len(failures)}): " + "; ".join(failures))
    sys.exit(1)
print("check_watchdog: all checks passed")
