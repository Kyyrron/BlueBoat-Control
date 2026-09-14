#!/usr/bin/env python3

r"""
Thruster saturation, shared by every node that puts a force on the water.

ROS-FREE BY CONSTRUCTION -- numpy only, no rclpy, no message types. Like
frame_math.py and robot_log_schema.py it can be imported, diffed and checked
from a plain Python prompt with no sourced workspace, which is what makes the
saturation rule debuggable without a running graph. Keep it that way.

WHY ONE SCALE FACTOR AND NOT TWO CLIPS
--------------------------------------
The two thrusters do not carry independent signals. What the controller
actually commands is a wrench -- a surge force X and a yaw moment N -- which
the allocation matrix splits into a common mode and a differential:

    f_right = X/2 + N/(2*r)          r = 0.295 m, the moment arm
    f_left  = X/2 - N/(2*r)

Clipping each side on its own therefore does NOT clip the command; it
REWRITES it into a different wrench. The classic failure:

    commanded  [+45, +18]   ->  X = 63.0 N, N = 27.0 N  (a hard turn)
    per-side   [+20, +18]   ->  X = 38.0 N, N =  2.0 N  (near enough straight)

The differential collapses from 27 N to 2 N, the boat stops turning, and it
runs further from the path than it started -- the harder the controller asks,
the worse the steering it gets. Scaling both sides by one factor instead:

    uniform    [+20,  +8]   ->  X = 28.0 N, N = 12.0 N  (same turn, slower)

The right:left ratio is preserved exactly, so the direction of the wrench in
the (X, N) plane is untouched and only its magnitude shrinks: the boat holds
the commanded turn-per-metre and simply travels it more slowly. This is the
same rule PID.ThrustAllocator.allocate already applies to the branches that
go through it; this module is how the branches that do NOT go through an
allocator (solve_LoS) and the actuator boundary itself get it too.
"""

import numpy as np


def scale_to_limit(f, limit):
    """
    Uniformly scale a thrust vector so that no element exceeds +/-limit.

    Input  : f     -- sequence of per-thruster forces in Newtons.
             limit -- non-negative symmetric bound, Newtons. 0 or negative
                      disables scaling and the input is returned unchanged.
    Output : (scaled, scale) -- scaled is a NEW float ndarray, scale is the
             single factor applied (1.0 when nothing saturated, so the caller
             can log "was this tick on the limiter" for free).

    One factor for the whole vector, so the ratio between thrusters -- and
    with it the direction of the commanded wrench -- is preserved exactly.
    See the module docstring for why per-element clipping is not equivalent.

        scale_to_limit([45.0, 18.0], 20.0)  ->  ([20.0, 8.0], 0.4444...)
        scale_to_limit([3.0, -1.0],  20.0)  ->  ([3.0, -1.0], 1.0)
    """
    out = np.asarray(f, dtype=float).reshape(-1).copy()

    if limit is None or limit <= 0.0:
        return out, 1.0

    peak = float(np.max(np.abs(out))) if out.size else 0.0
    if peak <= limit:
        return out, 1.0

    scale = limit / peak
    return out * scale, scale
