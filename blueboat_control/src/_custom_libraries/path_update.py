#!/usr/bin/env python3
"""
Live path update (BlueBoat-Replanner entry point) -- the ROS-free core.

ROS-FREE BY CONSTRUCTION: numpy, the standard library and yaml_trajectory
(PyYAML) only -- no rclpy, no message packages. Every decision master_control
takes about a /blueboat/update_path request is taken HERE, by plain functions
of plain values, so the whole mechanism is unit-tested without a ROS graph
(blueboat_control/test/test_path_update.py). master_control only wires it:
it builds a Context from its own state, calls PathUpdater, and turns the
returned dicts into a service response and two JSON topics.

What it implements -- design and evidence in docs/PATH_UPDATE_DESIGN.md, the
contract a client codes against in docs/PATH_UPDATE_INTERFACE.md:

  * A REPLANNER-OWNED run (path_source=replanner) holds its path here, as a
    PathPlan: an ordered list of segments, each a set of [t, x, y, yaw]
    samples evaluated by yaml_trajectory.YamlTrajectory (linear in t, yaw the
    short way round, clamped) -- exactly how a designer file is followed. An
    accepted update replaces everything from its change point on (D-1).
  * Progress is tau, the governor's own path parameter, never a second notion
    of progress (D-2).
  * A request is answered in a fixed order: state gate, request validity,
    change-point range, lead, junction continuity (D-3, D-4). A refusal
    changes nothing.

Installed flat into lib/blueboat_control by CMakeLists.txt, like every other
module in this package.
"""

import math
import re
from dataclasses import dataclass, field

import numpy as np

import yaml_trajectory as yt

SCHEMA_PROGRESS = "blueboat_path_progress/1"
SCHEMA_EVENT = "blueboat_path_update_event/1"

# The only frame a path may be given in: the frame_id path_generation stamps
# on every path, i.e. /blueboat/odom's local ENU (CLAUDE.md section 7).
FRAME = "world"

ID_PATTERN = re.compile(r"^[A-Za-z0-9._:-]{1,64}$")

# path_stamp.MAX_PLAUSIBLE_PARAMETER: beyond it a stamp is read as a wall
# clock, so no path parameter may reach it (kept literal: path_stamp is the
# owner of the number, test_path_update asserts the two agree).
MAX_PARAMETER = 1_000_000.0

MIN_SAMPLE_DT = 1e-3        # s   -- "strictly increasing" with a floor
MIN_SEGMENT_LENGTH = 1e-3   # m   -- shorter steps define no direction
FIRST_PATH_T0_TOL = 1e-6    # s   -- change_point == t[0] for a first path

# ---------------------------------------------------------------------------
# Answer codes. 'ok' on acceptance; every refusal is one of these, grouped by
# what the client should do about it (docs/PATH_UPDATE_INTERFACE.md).
# ---------------------------------------------------------------------------
OK = "ok"

NOT_READY = "not_ready"
NO_RUN = "no_run"
MISSION_OWNED = "mission_owned"
OPERATOR_OVERRIDE = "operator_override"

BAD_REQUEST = "bad_request"
DUPLICATE_ID = "duplicate_id"
TOO_FEW_POINTS = "too_few_points"
TOO_MANY_POINTS = "too_many_points"
NOT_FINITE = "not_finite"
TIME_NOT_INCREASING = "time_not_increasing"
TIME_OUT_OF_RANGE = "time_out_of_range"
TOO_FAST = "too_fast"
HEADING_INCOHERENT = "heading_incoherent"
CHANGE_POINT_OUTSIDE = "change_point_outside"

LEAD_TOO_SHORT = "lead_too_short"
DISCONTINUOUS_POSITION = "discontinuous_position"
DISCONTINUOUS_HEADING = "discontinuous_heading"

CATEGORY = {
    NOT_READY: "state", NO_RUN: "state", MISSION_OWNED: "state",
    OPERATOR_OVERRIDE: "state",
    BAD_REQUEST: "request", DUPLICATE_ID: "request",
    TOO_FEW_POINTS: "request", TOO_MANY_POINTS: "request",
    NOT_FINITE: "request", TIME_NOT_INCREASING: "request",
    TIME_OUT_OF_RANGE: "request", TOO_FAST: "request",
    HEADING_INCOHERENT: "request", CHANGE_POINT_OUTSIDE: "request",
    LEAD_TOO_SHORT: "geometry", DISCONTINUOUS_POSITION: "geometry",
    DISCONTINUOUS_HEADING: "geometry",
}

PATH_SOURCES = ("mission", "replanner")

EVENT_KINDS = ("request_accepted", "request_refused", "activated",
               "end_reached", "cleared")


def wrap(angle):
    """Angle to (-pi, pi]. Works on scalars and numpy arrays."""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def _f(value):
    """float, or None for a missing / non-finite value (JSON has no NaN)."""
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


# ---------------------------------------------------------------------------
# Configuration and per-call context
# ---------------------------------------------------------------------------

@dataclass
class Config:
    """Thresholds. Each is a declared master_control parameter (D-4)."""
    gov_Lmax: float = 3.0               # master_control's gov_Lmax
    hold_speed: float = 0.05            # master_control's hold_speed
    lead_factor: float = 1.0            # path_update_lead_factor
    pos_tol: float = 0.20               # path_update_pos_tol           [m]
    heading_tol: float = math.radians(10.0)       # path_update_heading_tol [rad]
    heading_coherence: float = math.radians(45.0)  # path_update_heading_coherence [rad]
    max_speed: float = 2.0              # path_update_max_speed         [m/s]
    max_points: int = 20000             # path_update_max_points


@dataclass
class Context:
    """The node state a decision depends on, captured at the call.

    Built from flags that exist in BOTH graphs (D-3): simulation has no run
    session, so session_mode is False there and run_active always True.
    """
    ready: bool = True                  # last /blueboat/controller_ready
    session_mode: bool = False
    run_active: bool = True
    path_source: str = "mission"
    use_pinger: bool = False
    override: str = "none"              # 'none' | 'manual' | 'arrow'
    tau: float = 0.0
    path_time: float = 0.05             # the controller's reference window
    path_steps: int = 2
    dt: float = 0.05                    # control_dt
    run: int = 0
    boat: tuple = None                  # (x, y, psi) or None
    ros_time: float = None
    wall_time: float = None
    run_time: float = None

    @property
    def spacing(self):
        """tau spacing of the reference window -- the governor's dtau."""
        return self.path_time / max(1, int(self.path_steps) - 1)


@dataclass
class Request:
    """One /blueboat/update_path request, as plain values."""
    id: str = ""
    change_point: float = float("nan")
    frame: str = FRAME
    t: list = field(default_factory=list)
    x: list = field(default_factory=list)
    y: list = field(default_factory=list)
    yaw: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# The plan
# ---------------------------------------------------------------------------

class Segment:
    """One accepted update: its samples, from its change point `start` on."""
    __slots__ = ("start", "traj", "id", "t_first", "t_last", "n")

    def __init__(self, start, points, seg_id):
        self.start = float(start)
        self.traj = yt.YamlTrajectory.from_points(points, name=seg_id)
        self.id = seg_id
        self.t_first = float(points[0, 0])
        self.t_last = float(points[-1, 0])
        self.n = int(len(points))


class PathPlan:
    """A replanner-owned run's path: plan(t) = the last segment starting at
    or before t, evaluated at t (D-1). Pure in t, like every library shape."""

    def __init__(self):
        self.segments = []

    def __bool__(self):
        return bool(self.segments)

    @property
    def t_start(self):
        return self.segments[0].start if self.segments else None

    @property
    def t_end(self):
        return self.segments[-1].t_last if self.segments else None

    @property
    def ids(self):
        return [s.id for s in self.segments]

    def segment_at(self, t):
        for seg in reversed(self.segments):
            if seg.start <= t:
                return seg
        return self.segments[0] if self.segments else None

    def pose(self, t):
        """(x, y, yaw) at path parameter t."""
        x, y, _, _, _, yaw = self.segment_at(t).traj.pose(t)
        return x, y, yaw

    def poses(self, params):
        """Arrays (x, y, yaw) at every parameter in `params`."""
        out = np.empty((len(params), 3))
        for i, t in enumerate(params):
            out[i] = self.pose(float(t))
        return out[:, 0], out[:, 1], out[:, 2]

    def speed(self, t, h):
        """Authored speed over [t, t+h] -- path_progress_errors' U_d."""
        if h <= 0.0:
            return 0.0
        x0, y0, _ = self.pose(t)
        x1, y1, _ = self.pose(t + h)
        return math.hypot(x1 - x0, y1 - y0) / h

    def speed_before(self, t, h):
        """Authored speed over [t-h, t]."""
        if h <= 0.0:
            return 0.0
        return self.speed(t - h, h)

    def splice(self, segment):
        """Replace everything from segment.start on (D-1)."""
        self.segments = [s for s in self.segments
                         if s.start < segment.start] + [segment]

    def prune(self, tau):
        """Forget segments wholly behind tau (their successor started)."""
        while len(self.segments) > 1 and self.segments[1].start <= tau:
            self.segments.pop(0)

    def clear(self):
        self.segments = []


# ---------------------------------------------------------------------------
# Request validity (D-4 step 1) -- pure, no node state
# ---------------------------------------------------------------------------

def validate_samples(req, cfg):
    """Check a request's own content.

    Returns (code, message, points) -- points the (n, 4) float array when the
    code is OK, else None. The duplicate-id check needs the run's history and
    lives in PathUpdater.handle.
    """
    if not isinstance(req.id, str) or not ID_PATTERN.match(req.id):
        return BAD_REQUEST, ("id must match [A-Za-z0-9._:-]{1,64}, got "
                             f"{req.id!r}"), None
    if req.frame != FRAME:
        return BAD_REQUEST, (f"frame must be '{FRAME}' (/blueboat/odom local "
                             f"ENU), got {req.frame!r}"), None
    try:
        lengths = {len(req.t), len(req.x), len(req.y), len(req.yaw)}
    except TypeError:
        return BAD_REQUEST, "t, x, y and yaw must be arrays", None
    if len(lengths) != 1:
        return BAD_REQUEST, ("t, x, y and yaw must have the same length, got "
                             f"{len(req.t)}/{len(req.x)}/{len(req.y)}/"
                             f"{len(req.yaw)}"), None
    n = lengths.pop()
    if n < 2:
        return TOO_FEW_POINTS, f"a path needs at least 2 samples, got {n}", None
    if n > cfg.max_points:
        return TOO_MANY_POINTS, (f"{n} samples exceed path_update_max_points="
                                 f"{cfg.max_points}"), None
    try:
        pts = np.column_stack([np.asarray(req.t, dtype=float),
                               np.asarray(req.x, dtype=float),
                               np.asarray(req.y, dtype=float),
                               np.asarray(req.yaw, dtype=float)])
    except (TypeError, ValueError):
        return BAD_REQUEST, "t, x, y and yaw must be numbers", None
    if not np.all(np.isfinite(pts)):
        bad = int(np.argmax(~np.all(np.isfinite(pts), axis=1)))
        return NOT_FINITE, f"sample {bad} is not finite", None

    t, x, y, yaw = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
    dts = np.diff(t)
    if np.any(dts < MIN_SAMPLE_DT):
        k = int(np.argmax(dts < MIN_SAMPLE_DT))
        return TIME_NOT_INCREASING, (
            f"t must increase by >= {MIN_SAMPLE_DT * 1e3:.0f} ms per sample: "
            f"t[{k}]={t[k]:.6g} -> t[{k + 1}]={t[k + 1]:.6g}"), None
    if t[0] < 0.0 or t[-1] >= MAX_PARAMETER:
        return TIME_OUT_OF_RANGE, (
            f"t must stay in [0, {MAX_PARAMETER:.0f}) s, got "
            f"[{t[0]:.6g}, {t[-1]:.6g}]"), None

    dx, dy = np.diff(x), np.diff(y)
    dist = np.hypot(dx, dy)
    speed = dist / dts
    if np.any(speed > cfg.max_speed):
        k = int(np.argmax(speed))
        return TOO_FAST, (
            f"segment {k} -> {k + 1} runs at {speed[k]:.3g} m/s, above "
            f"path_update_max_speed={cfg.max_speed:.3g} m/s (check units: "
            "metres, seconds)"), None

    # Yaw is read as the path TANGENT by the governor (gamma_p), the PID
    # (psi_path) and the MPC heading reference, so it must agree with the
    # direction of motion. A sample's yaw may be the incoming or the outgoing
    # tangent at a corner, hence the min over both ends of each segment.
    moving = (dist >= MIN_SEGMENT_LENGTH) & (speed >= cfg.hold_speed)
    if np.any(moving):
        course = np.arctan2(dy, dx)
        err = np.minimum(np.abs(wrap(yaw[:-1] - course)),
                         np.abs(wrap(yaw[1:] - course)))
        bad = moving & (err > cfg.heading_coherence)
        if np.any(bad):
            k = int(np.argmax(bad))
            return HEADING_INCOHERENT, (
                f"yaw disagrees with the direction of motion by "
                f"{math.degrees(err[k]):.1f} deg on segment {k} -> {k + 1} "
                f"(limit {math.degrees(cfg.heading_coherence):.1f} deg): yaw "
                "must be ENU radians, CCW from East, along the path"), None
    return OK, "", pts


# ---------------------------------------------------------------------------
# The updater
# ---------------------------------------------------------------------------

class PathUpdater:
    """All path-update state of one master_control: the plan, the hold pose
    of a run awaiting its first path, the ids accepted in this run, counters
    and the last answer. Every method is a pure function of its arguments and
    this state -- no clock, no randomness -- so equal inputs give equal
    outputs (tested)."""

    def __init__(self, cfg=None):
        self.cfg = cfg or Config()
        self.plan = PathPlan()
        self.awaiting = False           # replanner run without a first path
        self.hold = None                # (x, y, yaw) held while awaiting
        self.used_ids = set()           # accepted ids of this run
        self.end_reached = False
        self.following_id = None       # segment tau was on at the last tick
        self.accepted_count = 0
        self.refused_count = 0
        self.last_answer = None
        self.seq = 0                    # event counter, monotonic per node

    # -- run boundaries ------------------------------------------------------
    def reset(self, ctx, reason):
        """Discard the plan (start / stop / launch). Replanner runs start
        AWAITING their first path. Returns a 'cleared' event, or None when
        nothing about path updates existed to clear (a fixed-mission run
        without any request: no event, so no pathlog -- D-3)."""
        had_state = bool(self.plan) or self.awaiting or self.used_ids
        self.plan.clear()
        self.used_ids = set()
        self.end_reached = False
        self.following_id = None
        self.hold = None
        self.accepted_count = 0
        self.refused_count = 0
        self.last_answer = None
        replanner = ctx.path_source == "replanner" and not ctx.use_pinger
        self.awaiting = replanner and ctx.run_active
        if not (replanner or had_state):
            return None
        return self._event("cleared", ctx, reason=reason,
                           message=f"path cleared ({reason})")

    def set_hold(self, pose):
        """Capture the hold pose (x, y, yaw) of a run awaiting its path."""
        self.hold = (float(pose[0]), float(pose[1]), float(pose[2]))

    # -- derived quantities ----------------------------------------------------
    def lead_min(self, ctx):
        """Minimum change-point lead [path s] (D-4 step 3).

        floor : path_time + 2*dt -- never rewrite a reference window the
                controller already holds (for MPC, its whole horizon).
        gov   : lead_factor * gov_Lmax / U_d -- the governor's catch-up
                distance converted to path seconds at the current authored
                speed; only while the reference moves (U_d >= hold_speed).
        """
        floor = ctx.path_time + 2.0 * ctx.dt
        if not self.plan:
            return floor
        u = self.plan.speed(ctx.tau, ctx.spacing)
        if u >= self.cfg.hold_speed and u > 0.0:
            return max(floor, self.cfg.lead_factor * self.cfg.gov_Lmax / u)
        return floor

    def authored_speed(self, ctx):
        if not self.plan:
            return None
        return self.plan.speed(ctx.tau, ctx.spacing)

    def gate(self, ctx):
        """(code, message) of the state refusal a request would get now, or
        ('', '') when requests are being accepted (D-3, checked in order)."""
        if not ctx.ready:
            return NOT_READY, ("controller not ready (E-STOP latched, or "
                               "readiness not received yet)")
        if ctx.session_mode and not ctx.run_active:
            return NO_RUN, "no run is active (session idle)"
        if ctx.use_pinger:
            return MISSION_OWNED, "this run follows the pinger"
        if ctx.path_source != "replanner":
            return MISSION_OWNED, ("this run follows a fixed mission "
                                   "(path_source=mission)")
        if ctx.override != "none":
            return OPERATOR_OVERRIDE, (f"operator {ctx.override} control is "
                                       "engaged; release it first")
        if self.awaiting and self.hold is None:
            return NOT_READY, "no odometry yet: the hold pose is not captured"
        return "", ""

    # -- the request -----------------------------------------------------------
    def handle(self, req, ctx):
        """Decide one request. Returns (answer, event, new_tau).

        answer  : dict(accepted, reason, message, active_id, tau,
                  min_change_point) -- the service response
        event   : dict for /blueboat/path_update_event
        new_tau : the tau the node must take (first path: t[0]) or None
        A refusal leaves every piece of state as it was, counters aside.
        """
        detail = {"points": None, "t_first": None, "t_last": None,
                  "x_first": None, "y_first": None, "yaw_first": None,
                  "change_point": _f(req.change_point), "pos_gap": None,
                  "heading_gap": None, "heading_checked": None}
        try:
            n = len(req.t)
            detail["points"] = n
            if n:
                detail["t_first"] = _f(req.t[0])
                detail["t_last"] = _f(req.t[-1])
                detail["x_first"] = _f(req.x[0]) if len(req.x) else None
                detail["y_first"] = _f(req.y[0]) if len(req.y) else None
                detail["yaw_first"] = _f(req.yaw[0]) if len(req.yaw) else None
        except (TypeError, ValueError, IndexError):
            pass

        code, message = self.gate(ctx)
        if code:
            return self._refuse(req, ctx, code, message, detail)

        code, message, pts = validate_samples(req, self.cfg)
        if code != OK:
            return self._refuse(req, ctx, code, message, detail)
        if req.id in self.used_ids:
            return self._refuse(req, ctx, DUPLICATE_ID,
                                f"id {req.id!r} was already accepted in this "
                                "run; ids must be unique per run", detail)

        t0, t_last = float(pts[0, 0]), float(pts[-1, 0])
        first = self.awaiting
        c = _f(req.change_point)
        if first:
            if c is not None and abs(c - t0) > FIRST_PATH_T0_TOL:
                return self._refuse(req, ctx, CHANGE_POINT_OUTSIDE, (
                    f"a first path starts at its first sample: change_point "
                    f"must be NaN or t[0]={t0:.6g}, got {c:.6g}"), detail)
            c = t0
        else:
            if c is None:
                return self._refuse(req, ctx, CHANGE_POINT_OUTSIDE,
                                    "a change_point is required once a path "
                                    "is loaded", detail)
            if not (t0 <= c < t_last):
                return self._refuse(req, ctx, CHANGE_POINT_OUTSIDE, (
                    f"change_point {c:.6g} must lie in the samples' range "
                    f"[{t0:.6g}, {t_last:.6g})"), detail)
            lead = self.lead_min(ctx)
            if c < ctx.tau + lead - 1e-9:
                return self._refuse(req, ctx, LEAD_TOO_SHORT, (
                    f"change_point {c:.6g} is {c - ctx.tau:.3g} s ahead of "
                    f"tau={ctx.tau:.6g}; at least {lead:.3g} s is required "
                    f"(min_change_point={ctx.tau + lead:.6g})"), detail)
        detail["change_point"] = c

        segment = Segment(c, pts, req.id)
        nx, ny, _, _, _, nyaw = segment.traj.pose(c)
        if first:
            rx, ry, ryaw = self.hold
            stationary = True
        else:
            rx, ry, ryaw = self.plan.pose(c)
            h = ctx.spacing
            stationary = (self.plan.speed_before(c, h) < self.cfg.hold_speed
                          and self.plan.speed(c, h) < self.cfg.hold_speed)
        pos_gap = math.hypot(nx - rx, ny - ry)
        heading_gap = float(abs(wrap(nyaw - ryaw)))
        detail["pos_gap"] = pos_gap
        detail["heading_gap"] = heading_gap
        detail["heading_checked"] = not stationary
        where = "the hold pose" if first else f"the current path at t={c:.6g}"
        if pos_gap > self.cfg.pos_tol:
            return self._refuse(req, ctx, DISCONTINUOUS_POSITION, (
                f"the new path starts {pos_gap:.3f} m from {where} "
                f"({rx:.3f}, {ry:.3f}); limit {self.cfg.pos_tol:.3f} m"),
                detail)
        if not stationary and heading_gap > self.cfg.heading_tol:
            return self._refuse(req, ctx, DISCONTINUOUS_HEADING, (
                f"the new path's heading differs by "
                f"{math.degrees(heading_gap):.1f} deg from {where} "
                f"(yaw {ryaw:.4f} rad); limit "
                f"{math.degrees(self.cfg.heading_tol):.1f} deg"), detail)

        # ---- accepted --------------------------------------------------------
        self.plan.splice(segment)
        self.used_ids.add(req.id)
        new_tau = None
        if first:
            self.awaiting = False
            new_tau = t0
        if self.plan.t_end is not None and (new_tau if new_tau is not None
                                            else ctx.tau) < self.plan.t_end:
            self.end_reached = False
        self.accepted_count += 1
        tau_now = new_tau if new_tau is not None else ctx.tau
        message = (("first path loaded; following from t=" f"{t0:.6g}")
                   if first else
                   (f"accepted: replaces the path from t={c:.6g} "
                    f"({c - ctx.tau:.3g} s ahead of tau)"))
        answer = self._answer(True, OK, message, req, ctx, tau_now)
        event = self._event("request_accepted", ctx, update_id=req.id,
                            reason=OK, message=message, tau=tau_now,
                            lead_min=None if first else self.lead_min(ctx),
                            **detail)
        return answer, event, new_tau

    def _refuse(self, req, ctx, code, message, detail):
        self.refused_count += 1
        answer = self._answer(False, code, message, req, ctx, ctx.tau)
        lead = (self.lead_min(ctx)
                if self.plan and CATEGORY.get(code) != "state" else None)
        rid = req.id if isinstance(req.id, str) else repr(req.id)
        event = self._event("request_refused", ctx, update_id=rid,
                            reason=code, message=message, lead_min=lead,
                            **detail)
        return answer, event, None

    def _answer(self, accepted, code, message, req, ctx, tau):
        accepting = not self.gate(ctx)[0]
        mcp = (tau + self.lead_min(ctx)
               if accepting and self.plan and not self.awaiting else None)
        self.last_answer = {"id": req.id if isinstance(req.id, str) else "",
                            "accepted": bool(accepted), "reason": code,
                            "message": message, "wall_time": ctx.wall_time}
        return {"accepted": bool(accepted), "reason": code, "message": message,
                "active_id": self.active_id or "", "tau": float(tau),
                "min_change_point": float("nan") if mcp is None else float(mcp)}

    # -- per tick --------------------------------------------------------------
    def tick(self, ctx):
        """Called on every control tick of a replanner run, AFTER the governor.
        Returns the transition events of this tick: 'activated' when tau enters
        a new segment (the along-track switching analogue), 'end_reached' once
        when tau passes the last sample."""
        events = []
        if not self.plan:
            return events
        seg = self.plan.segment_at(ctx.tau)
        if seg is not None and seg.id != self.following_id:
            self.following_id = seg.id
            events.append(self._event(
                "activated", ctx, update_id=seg.id,
                change_point=seg.start,
                message=f"now following '{seg.id}' (from t={seg.start:.6g})"))
        end = self.plan.t_end
        if ctx.tau >= end and not self.end_reached:
            self.end_reached = True
            events.append(self._event(
                "end_reached", ctx, update_id=seg.id if seg else None,
                message=f"end of the path reached (t_end={end:.6g}); "
                        "holding the last pose"))
        elif ctx.tau < end:
            self.end_reached = False
        self.plan.prune(ctx.tau)
        return events

    def window(self, ctx):
        """The reference window at tau: (params, x, y, yaw) arrays, with the
        very parameters master_control would have requested from
        /path_request (np.linspace(tau, tau + path_time, path_steps))."""
        params = np.linspace(ctx.tau, ctx.tau + ctx.path_time,
                             int(ctx.path_steps), dtype=float)
        x, y, yaw = self.plan.poses(params)
        return params, x, y, yaw

    # -- reporting ---------------------------------------------------------------
    @property
    def active_id(self):
        """The id owning the remainder of the path (last accepted)."""
        return self.plan.segments[-1].id if self.plan else None

    def state_name(self, ctx):
        code, _ = self.gate(ctx)
        if code == NOT_READY and ctx.ready:
            return "awaiting_path"          # replanner run waiting for odometry
        if code == NOT_READY:
            return "not_ready"
        if code == NO_RUN:
            return "no_run"
        if code == MISSION_OWNED:
            return "mission"
        if code == OPERATOR_OVERRIDE:
            return "override"
        if self.awaiting:
            return "awaiting_path"
        if self.end_reached:
            return "end_reached"
        return "following"

    def progress(self, ctx):
        """The /blueboat/path_progress payload (D-2)."""
        code, _ = self.gate(ctx)
        replanner = ctx.path_source == "replanner" and not ctx.use_pinger
        plan = self.plan
        lead = self.lead_min(ctx) if (replanner and plan) else None
        following = plan.segment_at(ctx.tau) if plan else None
        return {
            "schema": SCHEMA_PROGRESS,
            "ros_time": _f(ctx.ros_time),
            "wall_time": _f(ctx.wall_time),
            "run": ctx.run,
            "path_source": "replanner" if replanner else (
                "pinger" if ctx.use_pinger else "mission"),
            "state": self.state_name(ctx),
            "accepting": not code,
            "refusal_now": code,
            "tau": _f(ctx.tau),
            "U_d": _f(self.authored_speed(ctx)) if replanner else None,
            "lead_min": _f(lead),
            "min_change_point": (_f(ctx.tau + lead)
                                 if (lead is not None and not code
                                     and not self.awaiting) else None),
            "following_id": following.id if following else None,
            "active_id": self.active_id,
            "pending_ids": ([s.id for s in plan.segments if s.start > ctx.tau]
                            if plan else []),
            "t_start": _f(plan.t_start) if plan else None,
            "t_end": _f(plan.t_end) if plan else None,
            "end_reached": (bool(self.end_reached) if (replanner and plan)
                            else None),
            "hold": ({"x": self.hold[0], "y": self.hold[1],
                      "yaw": self.hold[2]}
                     if (self.awaiting and self.hold is not None) else None),
            "override": ctx.override,
            "last_answer": self.last_answer,
            "accepted_count": self.accepted_count,
            "refused_count": self.refused_count,
        }

    def _event(self, kind, ctx, update_id=None, reason="", message="",
               change_point=None, tau=None, lead_min=None, points=None,
               t_first=None, t_last=None, x_first=None, y_first=None,
               yaw_first=None, pos_gap=None, heading_gap=None,
               heading_checked=None):
        """One /blueboat/path_update_event payload. Every key is always
        present (null when not applicable), so a reader never guesses."""
        self.seq += 1
        boat = ctx.boat
        return {
            "schema": SCHEMA_EVENT,
            "seq": self.seq,
            "run": ctx.run,
            "ros_time": _f(ctx.ros_time),
            "wall_time": _f(ctx.wall_time),
            "run_time": _f(ctx.run_time),
            "event": kind,
            "id": update_id,
            "reason": reason,
            "message": message,
            "change_point": _f(change_point),
            "tau": _f(ctx.tau if tau is None else tau),
            "lead_min": _f(lead_min),
            "points": points,
            "t_first": _f(t_first),
            "t_last": _f(t_last),
            "x_first": _f(x_first),
            "y_first": _f(y_first),
            "yaw_first": _f(yaw_first),
            "pos_gap": _f(pos_gap),
            "heading_gap": _f(heading_gap),
            "heading_checked": heading_checked,
            "boat": ({"x": _f(boat[0]), "y": _f(boat[1]), "psi": _f(boat[2])}
                     if boat is not None else None),
            "active_id": self.active_id,
            "handling_ms": None,        # filled in by the node, which times it
        }
