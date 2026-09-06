#!/usr/bin/env python3
"""
/path_request contract check.

The bug this guards: ROS 2 does not stop a second node offering /path_request,
and when two do, every request is answered by whichever server replies first.
The response is a bare nav_msgs/Path -- no echo of the request, no server
identity -- so master_control believed whatever arrived. Measured on recorded
logs with two missions up, the controller's reference alternated tick by tick
between two entirely different trajectories, both sampled at its own single
path parameter, and the boat could follow neither.

Four properties:

  1. The response tag round-trips. path_stamp is the shared codec both ends
     use, and it must survive the float32 narrowing that RequestPath's
     Float32MultiArray request field imposes.
  2. accept_path is EXECUTED, against a stub, on the real method source: it
     takes our own answer, and rejects a wrong-length one, a foreign one, and
     an empty one. master_control cannot be imported without acados, so the
     method is read out of the file and run -- the approach
     check_manual_hold.py and check_pid_equivalence.py both take.
  3. A server built before the tag existed stamps the wall clock. That must be
     detected once and fall back to geometry rather than rejecting every
     response, which would leave the controller with no path at all.
  4. The pieces that cannot be executed here are present in the source: the
     server's singleton guard and its parameter stamping, the request timeout,
     and the governor's staleness gate.

    python3 check_path_contract.py      # exit 0 pass, 1 fail

Needs numpy only, no ROS.
"""

import ast
import os
import sys
import textwrap

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
MASTER = os.path.join(SRC, "master_control.py")
SERVER = os.path.join(SRC, "_custom_libraries", "path_generation.py")
sys.path.insert(0, os.path.join(SRC, "_custom_libraries"))

import path_stamp as ps  # noqa: E402

failures = []


def check(label, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {label}{'  -- ' + detail if detail else ''}")
    if not ok:
        failures.append(label)


# ------------------------------------------------------- 1. the tag codec
print("The response tag round-trips")

for value in (0.0, 0.05, 4.412, 123.456789, 999.999, 73754.0):
    sec, nanosec = ps.encode(value)
    check(f"parameter {value} survives the round trip",
          ps.matches(sec, nanosec, value), f"{ps.decode(sec, nanosec)!r}")

# The request field is Float32MultiArray, so the server sees a narrowed value.
worst = 0.0
for value in np.linspace(0.0, 5000.0, 4001):
    narrowed = float(np.float32(value))
    sec, nanosec = ps.encode(narrowed)
    worst = max(worst, abs(ps.decode(sec, nanosec) - narrowed))
check("the codec survives the float32 request field over 0-5000 s",
      worst <= ps.MATCH_TOLERANCE, f"worst error {worst:.3e} s")

check("a wall-clock stamp is not mistaken for a parameter",
      not ps.is_parameter(1788400000, 0))
check("a plausible parameter is not mistaken for a clock",
      ps.is_parameter(*ps.encode(73754.0)))
check("encode never produces a negative or overflowing stamp",
      all(0 <= ps.encode(v)[1] < ps.NANOSECONDS and ps.encode(v)[0] >= 0
          for v in (-5.0, 0.0, 1.9999999999, 3.999999999)))


# ------------------------------------------- 2 & 3. accept_path, executed
print("\nmaster_control.accept_path, executed against a stub")

source = open(MASTER, encoding="utf-8").read()
tree = ast.parse(source)
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
fn = next(n for n in cls.body
          if isinstance(n, ast.FunctionDef) and n.name == "accept_path")
ns = {"ps": ps, "math": __import__("math"), "np": np}
exec(textwrap.dedent(ast.get_source_segment(source, fn)), ns)
accept_path = ns["accept_path"]


class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec, self.nanosec = sec, nanosec


class _Pose:
    def __init__(self, x=0.0, y=0.0):
        self.position = type("P", (), {"x": x, "y": y, "z": 0.0})()


class _PoseStamped:
    def __init__(self, tau, x=0.0, y=0.0):
        self.header = type("H", (), {"stamp": _Stamp(*ps.encode(tau))})()
        self.pose = _Pose(x, y)


class _WallStamped(_PoseStamped):
    def __init__(self, x=0.0, y=0.0):
        super().__init__(0.0, x, y)
        self.header.stamp = _Stamp(1788400000, 0)


class _Path:
    def __init__(self, poses):
        self.poses = poses


def node(**kw):
    class _Log:
        def warning(self, *a, **k): pass
        def error(self, *a, **k): pass
        def info(self, *a, **k): pass

    class N:
        path_steps = 2
        path_time = 0.05
        path_speed_scale = 1.0
        los_hold_umax = 0.8
        tau_requested = 4.412
        path_rx_time = 10.0
        path_stamps_are_parameters = None
        foreign_path_count = 0
        controller_path = _Path([])

        def get_logger(self):
            return _Log()
    n = N()
    for k, v in kw.items():
        setattr(n, k, v)
    return n


ours = _Path([_PoseStamped(4.412, 0.16, -17.6), _PoseStamped(4.462, 0.17, -17.8)])
check("our own answer is accepted", accept_path(node(), ours) is True)

foreign = _Path([_PoseStamped(4.450, 2.18, 0.24), _PoseStamped(4.500, 2.20, 0.25)])
n = node()
check("an answer for a different parameter is rejected",
      accept_path(n, foreign) is False)
check("and it is counted as a foreign path", n.foreign_path_count == 1)

check("a wrong-length window is rejected",
      accept_path(node(), _Path([_PoseStamped(4.412)])) is False)
check("an empty path is rejected", accept_path(node(), _Path([])) is False)

check("the first answer is accepted before anything was requested",
      accept_path(node(tau_requested=None), ours) is True)

# Version skew: an old server stamps the clock.
n = node(path_stamps_are_parameters=None,
         controller_path=_Path([_PoseStamped(0.0, 0.0, 0.0)]))
first = accept_path(n, _Path([_WallStamped(0.1, 0.0), _WallStamped(0.2, 0.0)]))
check("a clock-stamping server is detected, not rejected outright", first is True)
check("and the fallback is latched for the rest of the run",
      n.path_stamps_are_parameters is False)
check("the geometric fallback then rejects an impossible jump",
      accept_path(n, _Path([_WallStamped(90.0, 40.0), _WallStamped(90.1, 40.0)])) is False)
check("and still accepts a plausible step",
      accept_path(n, _Path([_WallStamped(0.3, 0.0), _WallStamped(0.4, 0.0)])) is True)


# --------------------------------------------- 4. the pieces in the source
print("\nThe guards are in the source")

server = open(SERVER, encoding="utf-8").read()
for label, needle, text in (
        ("the server refuses to be the second one",
         "_refuse_if_server_running", server),
        ("it exits rather than continuing", "raise SystemExit(1)", server),
        ("with an escape hatch", "'allow_duplicate_server'", server),
        ("it stamps the path parameter, not the clock", "ps.encode(t)", server),
        ("it no longer stamps the clock",
         "temp_pose.header.stamp = self.get_clock()", server),
        ("the client records what it asked for", "self.tau_requested = float(", source),
        ("the client validates before accepting", "elif self.accept_path(result.path):", source),
        ("the request has a timeout", "self.path_request_timeout", source),
        ("the governor is gated on a fresh window", "if window_fresh:", source),
        ("the log is written off the control tick", "def save_monitoring(self)", source),
        ("and no longer from timer_callback", "np.save(self.title, self.monitoring)\n", source),
        ("the operator is warned about two servers",
         "MORE THAN ONE /path_request SERVER", source),
):
    present = needle in text
    # two needles are things that must be GONE
    want_absent = label in ("it no longer stamps the clock",
                            "and no longer from timer_callback")
    if want_absent:
        check(label, text.count(needle) == (1 if needle.startswith("np.save") else 0),
              f"{text.count(needle)} occurrence(s)")
    else:
        check(label, present)

print()
if failures:
    print(f"FAILED ({len(failures)}): " + "; ".join(failures))
    sys.exit(1)
print("check_path_contract: all checks passed")
