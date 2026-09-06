#!/usr/bin/env python3
"""
The /path_request response tag: encode a path parameter into a ROS time stamp.

Why this exists
---------------
`RequestPath` is deliberately parameter-agnostic -- the request is an array of
path-parameter values and the response is a bare `nav_msgs/Path`, one pose per
value. That contract is what lets the reference-generation strategy change
without touching `path_generation`, and it must be preserved (N1/CM-1), so the
response cannot grow a field saying which request it answers.

It needs one anyway. ROS 2 does not stop a second node offering `/path_request`,
and when two do, every request is answered by whichever server replies first.
The client cannot tell the difference: measured on this system, `master_control`
followed two entirely different trajectories on alternating ticks, both sampled
at its own single path parameter, and could not follow either.

Each pose in the response is a `PoseStamped`, and its `header.stamp` was being
overwritten with the wall clock -- a field that told the caller nothing. Putting
the path parameter there instead costs no new field, no `.srv` change and
nothing on the wire: the caller compares the first pose's stamp against the
parameter it asked for and rejects anything else.

The parameter is non-negative (it starts at zero and is monotonic, N8), which is
what makes a `builtin_interfaces/Time` -- unsigned nanoseconds -- a lawful
carrier for it.

ROS-free by construction: plain arithmetic on two integers, no rclpy, so both
ends of the contract share one implementation and it can be checked from a
plain Python prompt with no sourced workspace. The caller builds the message.
"""

NANOSECONDS = 1_000_000_000

# A path parameter is seconds along a trajectory. Anything past this is not a
# parameter -- it is a wall clock (a 2026 epoch stamp is ~1.8e9), which is what
# a server built before 2026-09-03 puts there. The caller uses this to tell
# "the other server answered" apart from "the boat is running an old build".
MAX_PLAUSIBLE_PARAMETER = 1_000_000.0

# Tolerance for a round trip. The request field is std_msgs/Float32MultiArray,
# so the server sees the parameter already narrowed to float32; at the largest
# parameter a mission plausibly reaches, one float32 ULP is far inside this.
MATCH_TOLERANCE = 1e-3


def encode(parameter):
    """Path parameter (seconds, >= 0) -> (sec, nanosec) for a ROS time stamp."""
    value = max(0.0, float(parameter))
    sec = int(value)
    nanosec = int(round((value - sec) * NANOSECONDS))
    if nanosec >= NANOSECONDS:          # rounding carried into the next second
        sec += 1
        nanosec -= NANOSECONDS
    return sec, nanosec


def decode(sec, nanosec):
    """(sec, nanosec) -> path parameter in seconds."""
    return float(sec) + float(nanosec) / NANOSECONDS


def matches(sec, nanosec, parameter, tolerance=MATCH_TOLERANCE):
    """True when a stamp carries the parameter that was requested."""
    return abs(decode(sec, nanosec) - float(parameter)) <= tolerance


def is_parameter(sec, nanosec):
    """
    False when the stamp is a wall clock rather than a path parameter.

    Distinguishes a server that predates this contract -- which the caller must
    warn about once and then fall back on geometry for -- from a server that
    answered a different request, which is a fault to reject every time.
    """
    return decode(sec, nanosec) < MAX_PLAUSIBLE_PARAMETER
