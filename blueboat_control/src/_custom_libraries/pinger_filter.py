#!/usr/bin/env python3
r"""
The USBL pinger filter: a robust Kalman estimate of the pinger's WORLD position,
and the legacy (v1.1) pinger it replaced. Run live by the pinger_estimator node.

ROS-FREE (numpy only): every time is passed in, nothing reads a clock.

Frames
  acoustic a = (a_x, a_y, a_z)  as the Water Linked API returns it
  body       b = R(mount_yaw) * (a_x, y_sign*a_y) + lever      (x forward, y left)
  world      z = p_boat(t_rx - latency) + R(psi) * b             (/blueboat/odom local ENU)
  depth      = z_sign * a_z  (positive down)

State [pE, pN, vE, vN] (model 'cv') or [pE, pN] (model 'static'), plus a separate
1-D depth filter. Each fix is weighed by its NIS d2 = nu' S^-1 nu: up to gate_soft
it is a normal update; above it the fix still pulls but is believed less (its
noise is inflated by d2 / gate_soft); only d2 > gate_hard together with an
implied speed > v_hard sets it aside. reinit_n fixes in a row beyond gate_soft
that agree within reinit_radius_m make the estimate jump there.

Between fixes the world estimate does not move: the body vector the boat steers
to is recomputed from it at the current pose, b = R(psi)^T (p_hat - p_boat).
"""

import bisect
import json
import math
from collections import deque

import numpy as np

STATUSES = ("no_data", "initialising", "tracking", "coasting", "stale")
STATUS_CODE = {s: i for i, s in enumerate(STATUSES)}
DECISIONS = ("accepted", "downweighted", "set_aside", "invalid", "duplicate",
             "initialising", "reinit")
CHI2_2_95 = 5.991464547107979          # 95 % ellipse scale (2 dof)
CHI2_1_95 = 3.841458820694124
EVENT_HOLD_S = 2.0                     # an event is reported this long after it happened
CLOCK_STEP_BACK_S = 1.0                # a pose older than the newest by more restarts the pose history
POSE_HISTORY_S = 30.0                  # boat poses kept for the latency compensation
MAX_POSE_EXTRAP_S = 0.5                # a fix measured this much after the newest pose still uses it
MAX_POSE_GAP_S = 2.0                   # no pose is interpolated across a longer odometry gap
FIX_RATE_WINDOW_S = 10.0
STATS_WINDOW_S = 60.0                  # window of the % down-weighted / set aside statistics
MAX_PUBLISH_DELAY_S = 5.0              # a larger uwgps_log cycle delay is not trusted
SCHEMA_UWGPS_FIX = "blueboat_uwgps_fix/1"


def status_of_age(age, p):
    """'tracking' | 'coasting' | 'stale' of an input last updated `age` s ago."""
    if age <= p["coast_after_s"]:
        return "tracking"
    return "coasting" if age <= p["stale_timeout_s"] else "stale"


def _f(v):
    """JSON-safe float (None for None / non-finite)."""
    if v is None:
        return None
    v = float(v)
    return v if math.isfinite(v) else None


# ---------------------------------------------------------------------------
# input: one /uw_gps/fix message -> a Fix
# ---------------------------------------------------------------------------

class Fix:
    """One Water Linked acoustic fix: receive time t_rx (s), acoustic position
    a (m), the endpoint it came from ('raw' | 'filtered'), WL's std and validity
    fields (None when absent), `invalid` (None or the reason it cannot be used)
    and `key` (the same key twice is the same fix)."""

    __slots__ = ("t_rx", "a", "source", "std", "position_valid", "receivers_valid",
                 "invalid", "seq", "key")

    def __init__(self, t_rx, a, source, std=None, position_valid=None, receivers_valid=None,
                 invalid=None, seq=None):
        self.t_rx = float(t_rx)
        self.a = tuple(float(v) for v in a)
        self.source = source
        self.std = None if std is None else float(std)
        self.position_valid = position_valid
        self.receivers_valid = receivers_valid
        self.invalid = invalid
        self.seq = seq
        self.key = (source,) + self.a


def _num(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _publish_delay(msg, ep):
    """Seconds between this endpoint's response and the message's publication:
    the later requests of a full uwgps_log cycle delay the raw fix that much.
    Both stamps are uwgps_log's own clock. 0 when unknown or implausible."""
    try:
        d = float(msg.get("t_end")) - float(ep.get("t_resp"))
    except (TypeError, ValueError):
        return 0.0
    return d if (math.isfinite(d) and 0.0 <= d <= MAX_PUBLISH_DELAY_S) else 0.0


def fix_from_json(t_rx, text, source):
    """The `source` ('raw' | 'filtered') Fix of one /uw_gps/fix message received
    at t_rx, or None (not such a message, or that endpoint was not polled)."""
    try:
        msg = json.loads(text)
    except ValueError:
        return None
    if not isinstance(msg, dict) or msg.get("schema") != SCHEMA_UWGPS_FIX:
        return None
    eps = msg.get("endpoints")
    ep = eps.get(source) if isinstance(eps, dict) else None
    seq = msg.get("seq")
    if ep is None:
        return None
    if not isinstance(ep, dict):
        return Fix(t_rx, (0.0, 0.0, 0.0), source, invalid="malformed_endpoint", seq=seq)
    t_rx = t_rx - _publish_delay(msg, ep)
    if not ep.get("ok") or not isinstance(ep.get("data"), dict):
        why = ep.get("error") or f"HTTP {ep.get('status')}"
        return Fix(t_rx, (0.0, 0.0, 0.0), source, invalid=f"request_failed: {why}", seq=seq)
    d = ep["data"]
    x, y, z = _num(d.get("x")), _num(d.get("y")), _num(d.get("z"))
    if x is None or y is None:
        return Fix(t_rx, (0.0, 0.0, 0.0), source, invalid="no_xy", seq=seq)
    rv = d.get("receiver_valid")
    pv = d.get("position_valid")
    return Fix(t_rx, (x, y, 0.0 if z is None else z), source, std=_num(d.get("std")),
               position_valid=None if pv is None else bool(pv),
               receivers_valid=sum(1 for v in rv if v) if isinstance(rv, (list, tuple)) else None,
               seq=seq)


# ---------------------------------------------------------------------------
# boat pose history (latency compensation)
# ---------------------------------------------------------------------------

class PoseBuffer:
    """Boat poses by receive time; interpolation with the yaw unwrapped."""

    def __init__(self, keep_s):
        self.keep_s = float(keep_s)
        self.t, self.x, self.y, self.yaw = [], [], [], []

    def add(self, t, x, y, yaw):
        t = float(t)
        if self.t and t < self.t[-1] - CLOCK_STEP_BACK_S:
            # The clock stepped back (system time set): restart rather than
            # ignore every later pose for as long as the step.
            self.t, self.x, self.y, self.yaw = [], [], [], []
        if self.t:
            if t < self.t[-1]:
                return                          # out of order: ignored
            last = self.yaw[-1]
            yaw = last + math.atan2(math.sin(yaw - last), math.cos(yaw - last))
            if t == self.t[-1]:
                self.x[-1], self.y[-1], self.yaw[-1] = float(x), float(y), yaw
                return
        self.t.append(t)
        self.x.append(float(x))
        self.y.append(float(y))
        self.yaw.append(float(yaw))
        cut = bisect.bisect_left(self.t, t - self.keep_s)
        if cut > 0 and len(self.t) - cut >= 2:
            del self.t[:cut], self.x[:cut], self.y[:cut], self.yaw[:cut]

    def at(self, t, max_extrap, max_gap=float("inf")):
        """(x, y, yaw, yaw_rate) at time t, or None when t is outside the buffer
        (older than its start, newer than its end by more than max_extrap) or
        falls in an odometry gap longer than max_gap."""
        n = len(self.t)
        if n == 0 or t < self.t[0]:
            return None
        if t >= self.t[-1]:
            if t - self.t[-1] > max_extrap:
                return None
            rate = 0.0
            if n >= 2 and self.t[-1] > self.t[-2]:
                rate = (self.yaw[-1] - self.yaw[-2]) / (self.t[-1] - self.t[-2])
            return self.x[-1], self.y[-1], self.yaw[-1], rate
        i1 = bisect.bisect_right(self.t, t)
        i0 = i1 - 1
        span = self.t[i1] - self.t[i0]
        if span > max_gap:
            return None
        u = (t - self.t[i0]) / span if span > 0 else 0.0
        x = self.x[i0] + u * (self.x[i1] - self.x[i0])
        y = self.y[i0] + u * (self.y[i1] - self.y[i0])
        yaw = self.yaw[i0] + u * (self.yaw[i1] - self.yaw[i0])
        rate = (self.yaw[i1] - self.yaw[i0]) / span if span > 0 else 0.0
        return x, y, yaw, rate


# ---------------------------------------------------------------------------
# the filter
# ---------------------------------------------------------------------------

class PingerFilter:
    """Robust pinger estimate. `params`: the dict of parameter values
    (pinger_params.Params.values)."""

    def __init__(self, params):
        self.p = dict(params)
        self.poses = PoseBuffer(POSE_HISTORY_S)
        self.reset()

    def set_params(self, params):
        """Apply a new parameter set live. The estimate is kept; only a change
        of motion model (the state's size) restarts the filter."""
        model_changed = params["model"] != self.p["model"]
        self.p = dict(params)
        if model_changed:
            self.reset()

    def reset(self):
        self.cv = self.p["model"] == "cv"
        self.dim = 4 if self.cv else 2
        self.H = np.hstack([np.eye(2), np.zeros((2, self.dim - 2))])
        self.x = None
        self.P = None
        self.t_state = None             # measurement time of the last update
        self.t_rx_update = None         # receive time of the last update
        self.depth = None
        self.Pd = None
        self.init_buf = deque()
        self.setaside_buf = deque()
        self.last_key = None
        self.seen_valid = False
        self.decision_log = deque()     # (t_rx, decision)
        self.valid_log = deque()        # t_rx of valid, non-duplicate fixes
        self.event = None
        self.event_t = None

    @property
    def initialised(self):
        return self.x is not None

    def add_pose(self, t, x, y, yaw):
        """One boat pose (/blueboat/odom) at receive time t."""
        self.poses.add(t, x, y, yaw)

    # ------------------------------------------------------------ geometry
    def map_fix(self, a):
        """Acoustic (a_x, a_y, a_z) -> (body x, body y, depth, acoustic range)."""
        p = self.p
        ax, ay, az = a
        bx0, by0 = ax, p["y_sign"] * ay
        d = math.radians(p["mount_yaw_deg"])
        c, s = math.cos(d), math.sin(d)
        bx = c * bx0 - s * by0 + p["lever_x_m"]
        by = s * bx0 + c * by0 + p["lever_y_m"]
        return bx, by, p["z_sign"] * az, math.hypot(bx0, by0)

    def meas_cov(self, rho, ux, uy, yaw_rate, std):
        """2x2 world covariance of a fix at acoustic range rho along the unit
        line of sight (ux, uy): along-range and cross-range std."""
        p = self.p
        s_r = p["range_floor_m"] + p["range_k"] * rho
        ang = math.sqrt(math.radians(p["sigma_bearing_deg"]) ** 2
                        + math.radians(p["sigma_heading_deg"]) ** 2
                        + (p["sigma_time_s"] * yaw_rate) ** 2)
        s_c = rho * ang
        if p["use_wl_std"] and std is not None:
            s_r = max(s_r, std)
            s_c = max(s_c, std)
        s_r = max(s_r, 0.01)
        s_c = max(s_c, 0.01)
        u = np.array([ux, uy])
        w = np.array([-uy, ux])
        return s_r * s_r * np.outer(u, u) + s_c * s_c * np.outer(w, w)

    # ------------------------------------------------------------ motion model
    def _FQ(self, dt):
        if self.cv:
            F = np.eye(4)
            F[0, 2] = F[1, 3] = dt
            q = self.p["sigma_a"] ** 2
            Q = np.zeros((4, 4))
            Q[0, 0] = Q[1, 1] = q * dt ** 4 / 4.0
            Q[0, 2] = Q[2, 0] = Q[1, 3] = Q[3, 1] = q * dt ** 3 / 2.0
            Q[2, 2] = Q[3, 3] = q * dt ** 2
        else:
            F = np.eye(2)
            Q = np.eye(2) * (self.p["static_drift"] ** 2) * dt
        return F, Q

    def _cap_velocity(self):
        if not self.cv:
            return
        vmax = self.p["v_max"]
        v = math.hypot(self.x[2], self.x[3])
        if v > vmax:                    # v_max >= 0, so v > 0 here
            k = vmax / v
            self.x[2] *= k
            self.x[3] *= k

    # ------------------------------------------------------------------ fixes
    def add_fix(self, fix):
        """Process one Fix, with the boat pose at its measurement time. Returns
        the /blueboat/pinger/fix record (what was decided, and why)."""
        p = self.p
        t_m = fix.t_rx - p["latency_s"]
        res = {"t_rx": fix.t_rx, "t_meas": t_m, "source": fix.source, "seq": fix.seq,
               "a": list(fix.a), "std": fix.std, "position_valid": fix.position_valid,
               "receivers_valid": fix.receivers_valid,
               "decision": None, "reason": "", "body": None, "depth_meas": None,
               "pose": None, "world": None, "R": None, "nis": None, "weight": None,
               "v_implied": None, "posterior": None, "event": None}

        def done(decision, reason=""):
            res["decision"] = decision
            res["reason"] = reason
            self.decision_log.append((fix.t_rx, decision))
            res["status"] = self.status(fix.t_rx)
            if self.initialised:
                res["posterior"] = {"x": float(self.x[0]), "y": float(self.x[1]),
                                    "vx": float(self.x[2]) if self.cv else 0.0,
                                    "vy": float(self.x[3]) if self.cv else 0.0,
                                    "depth": _f(self.depth)}
            return res

        if fix.invalid:
            return done("invalid", fix.invalid)
        if fix.key == self.last_key:
            return done("duplicate", "same fix as the previous one")
        self.last_key = fix.key
        if p["require_position_valid"] and fix.position_valid is False:
            return done("invalid", "position_valid=false")
        if fix.receivers_valid is not None and fix.receivers_valid < p["min_receivers"]:
            return done("invalid", f"{fix.receivers_valid} valid receivers < {p['min_receivers']}")
        bx, by, depth, rho = self.map_fix(fix.a)
        res["body"] = [bx, by]
        res["depth_meas"] = depth
        if rho < p["min_range_m"]:
            return done("invalid", f"range {rho:.2f} m < min_range_m")
        if rho > p["max_range_m"]:
            return done("invalid", f"range {rho:.1f} m > max_range_m")
        pose = self.poses.at(t_m, MAX_POSE_EXTRAP_S, MAX_POSE_GAP_S)
        if pose is None:
            return done("invalid", "no boat pose at the measurement time")
        x, y, yaw, rate = pose
        res["pose"] = {"x": x, "y": y, "yaw": yaw, "yaw_rate": rate}
        self.seen_valid = True
        self.valid_log.append(fix.t_rx)
        c, s = math.cos(yaw), math.sin(yaw)
        wx = c * bx - s * by
        wy = s * bx + c * by
        z = np.array([x + wx, y + wy])
        nb = math.hypot(wx, wy)
        ux, uy = (wx / nb, wy / nb) if nb > 1e-9 else (1.0, 0.0)
        R = self.meas_cov(rho, ux, uy, rate, fix.std)
        res["world"] = [float(z[0]), float(z[1])]
        res["R"] = [float(R[0, 0]), float(R[0, 1]), float(R[1, 1])]
        cand = (fix.t_rx, t_m, z, R, depth)

        if not self.initialised:
            if self._cluster(self.init_buf, cand, p["init_window_s"], p["init_n"],
                             p["init_radius_m"], "acquired"):
                res["event"] = "acquired"
                return done("reinit", "initial acquisition")
            return done("initialising", f"{len(self.init_buf)}/{p['init_n']} consistent fixes")

        # predict to the measurement time (never backwards)
        dt = max(0.0, t_m - self.t_state)
        F, Q = self._FQ(dt)
        x_prior = F @ self.x
        P_prior = F @ self.P @ F.T + Q
        nu = z - self.H @ x_prior
        S = self.H @ P_prior @ self.H.T + R
        d2 = float(nu @ np.linalg.inv(S) @ nu)
        res["nis"] = d2
        v_imp = float(np.hypot(nu[0], nu[1])) / max(dt, 0.2)
        res["v_implied"] = v_imp
        R_eff, weight, decision, reason = R, 1.0, "accepted", ""
        if d2 > p["gate_soft"]:
            # Re-acquisition looks at every fix beyond the soft gate and asks
            # whether the last reinit_n of them CLUSTER: with field noise a real
            # 15 m move only gives NIS ~13-16 (inside the hard gate), so it is
            # merely down-weighted fix by fix -- but random outliers disagree.
            if self._cluster(self.setaside_buf, cand, p["reinit_window_s"], p["reinit_n"],
                             p["reinit_radius_m"], "reacquired"):
                res["event"] = "reacquired"
                return done("reinit", f"re-acquired after {p['reinit_n']} consistent fixes beyond gate_soft")
            if d2 > p["gate_hard"] and v_imp > p["v_hard"]:
                return done("set_aside", f"NIS {d2:.1f} > gate_hard at {v_imp:.1f} m/s")
            weight = p["gate_soft"] / d2
            R_eff = R / weight
            decision, reason = "downweighted", f"NIS {d2:.1f}: R x {1.0 / weight:.2f}"
        self._update(x_prior, P_prior, z, R_eff, t_m, fix.t_rx, depth)
        if d2 <= p["gate_soft"]:
            self.setaside_buf.clear()           # a normal fix ends any re-acquisition streak
        res["weight"] = weight
        return done(decision, reason)

    # --------------------------------------------------------------- helpers
    @staticmethod
    def _consistent(group, radius):
        pts = np.array([g[2] for g in group])
        med = np.median(pts, axis=0)
        return bool(np.all(np.hypot(pts[:, 0] - med[0], pts[:, 1] - med[1]) <= radius))

    def _start(self, group, t_rx, event):
        """(Re)initialise at the median of a consistent group."""
        pts = np.array([g[2] for g in group])
        med = np.median(pts, axis=0)
        Rm = sum(g[3] for g in group) / len(group)
        Ppos = Rm / len(group)
        if self.cv:
            vv = (max(self.p["v_max"], 0.05) / 2.0) ** 2
            self.x = np.array([med[0], med[1], 0.0, 0.0])
            self.P = np.zeros((4, 4))
            self.P[:2, :2] = Ppos
            self.P[2, 2] = self.P[3, 3] = vv
        else:
            self.x = np.array([med[0], med[1]])
            self.P = Ppos.copy()
        self.depth = float(np.median([g[4] for g in group]))
        self.Pd = self.p["depth_sigma_m"] ** 2 / len(group)
        self.t_state = group[-1][1]
        self.t_rx_update = t_rx
        self.init_buf.clear()
        self.setaside_buf.clear()
        self.event, self.event_t = event, t_rx

    def _cluster(self, buf, cand, window_s, n, radius, event):
        """Remember `cand` in `buf` (the starting or the doubted fixes); True,
        with the estimate (re)started at their median, when the last n of them
        within window_s agree within radius."""
        buf.append(cand)
        while buf and buf[0][0] < cand[0] - window_s:
            buf.popleft()
        if len(buf) >= n:
            group = list(buf)[-n:]
            if self._consistent(group, radius):
                self._start(group, cand[0], event)
                return True
        return False

    def _update(self, x_prior, P_prior, z, R_eff, t_m, t_rx, depth):
        H = self.H
        S = H @ P_prior @ H.T + R_eff
        K = P_prior @ H.T @ np.linalg.inv(S)
        x = x_prior + K @ (z - H @ x_prior)
        IKH = np.eye(self.dim) - K @ H
        P = IKH @ P_prior @ IKH.T + K @ R_eff @ K.T
        self.x, self.P = x, P
        self._cap_velocity()
        # depth: 1-D random walk, with its own outlier down-weighting
        dtd = max(0.0, t_m - self.t_state)
        Pd = self.Pd + self.p["depth_drift"] ** 2 * dtd
        rd = self.p["depth_sigma_m"] ** 2
        nd = depth - self.depth
        sd = Pd + rd
        d2d = nd * nd / sd
        if d2d > CHI2_1_95:
            rd *= d2d / CHI2_1_95
            sd = Pd + rd
        kd = Pd / sd
        self.depth = self.depth + kd * nd
        self.Pd = (1.0 - kd) * Pd
        self.t_state = t_m
        self.t_rx_update = t_rx

    # ---------------------------------------------------------------- outputs
    def status(self, t):
        if not self.seen_valid:
            return "no_data"
        if not self.initialised:
            return "initialising"
        return status_of_age(t - self.t_rx_update, self.p)

    def body_at(self, x, y, yaw):
        """[bx, by, depth]: the held world estimate seen from the boat pose
        (x, y, yaw) -- b = R(yaw)^T (p_hat - p_boat). None before initialisation."""
        if not self.initialised:
            return None
        dx, dy = self.x[0] - x, self.x[1] - y
        c, s = math.cos(yaw), math.sin(yaw)
        return [c * dx + s * dy, -s * dx + c * dy, self.depth]

    def position_cov_at(self, t):
        """Position covariance propagated to time t (the position itself is held)."""
        F, Q = self._FQ(max(0.0, t - (self.t_state + self.p["latency_s"])))
        P = F @ self.P @ F.T + Q
        return P[:2, :2]

    def stats(self, t):
        while self.decision_log and self.decision_log[0][0] < t - STATS_WINDOW_S:
            self.decision_log.popleft()
        counts = {d: 0 for d in DECISIONS}
        for _, d in self.decision_log:
            counts[d] += 1
        used = counts["accepted"] + counts["downweighted"] + counts["set_aside"] + counts["reinit"]
        return {"window_s": STATS_WINDOW_S,
                "pct_downweighted": _f(100.0 * counts["downweighted"] / used) if used else None,
                "pct_set_aside": _f(100.0 * counts["set_aside"] / used) if used else None}

    def fix_rate(self, t):
        while self.valid_log and self.valid_log[0] < t - FIX_RATE_WINDOW_S:
            self.valid_log.popleft()
        return len(self.valid_log) / FIX_RATE_WINDOW_S

    def estimate(self, t):
        """The filter part of the /blueboat/pinger/estimate payload at time t."""
        status = self.status(t)
        out = {"t": _f(t), "status": status, "status_code": STATUS_CODE[status],
               "event": self.event if (self.event_t is not None and t - self.event_t <= EVENT_HOLD_S) else None,
               "x": None, "y": None, "depth": None, "vx": None, "vy": None,
               "cov": None, "r95": None, "ellipse": None,
               "fix_age": _f(t - self.t_rx_update) if self.t_rx_update is not None else None,
               "fix_rate": _f(self.fix_rate(t)), "stats": self.stats(t)}
        if self.initialised:
            P = self.position_cov_at(t)
            evals, evecs = np.linalg.eigh(P)
            evals = np.maximum(evals, 0.0)
            a = math.sqrt(CHI2_2_95 * evals[1])
            b = math.sqrt(CHI2_2_95 * evals[0])
            theta = math.atan2(evecs[1, 1], evecs[0, 1])
            out.update(x=_f(self.x[0]), y=_f(self.x[1]), depth=_f(self.depth),
                       vx=_f(self.x[2]) if self.cv else 0.0, vy=_f(self.x[3]) if self.cv else 0.0,
                       cov=[_f(P[0, 0]), _f(P[0, 1]), _f(P[1, 1])],
                       r95=_f(a), ellipse={"a": _f(a), "b": _f(b), "theta": _f(theta)})
        return out


# ---------------------------------------------------------------------------
# the legacy pinger (robot_interface v1.1, 44ec6aa)
# ---------------------------------------------------------------------------

class LegacyPinger:
    """The pinger as robot_interface computed it until 2026-11-24, arithmetic for
    arithmetic: every /uw_gps_data packet overwrites the body vector with WL's
    filtered xyz; between packets it is dead-reckoned at odom rate with the body
    twist and the IMU yaw rate, p -= (v + w x p) dt. It lags the boat's motion
    ("dragged by the boat between fixes"). Kept for the `corrected_pinger_*`
    CSV columns, the station's legacy layer and `pinger_source:=legacy`."""

    def __init__(self):
        self.pinger_coordinates = np.zeros(3)
        self.prev_time = None
        self.angular_velocity_z = None          # None until the first IMU message

    def on_imu(self, angular_velocity_z):
        self.angular_velocity_z = angular_velocity_z

    def on_ugps(self, data):
        """A /uw_gps_data packet: reseed from the FILTERED acoustic position (16..18)."""
        self.pinger_coordinates = np.array([data[16], data[17], data[18]])

    def on_odom(self, t, vel_xyz):
        """The body vector after an odom received at t with the body twist
        (vx, vy, vz); None on the very first odom (it only latches the time)."""
        if self.prev_time is None:
            self.prev_time = t
            return None
        dt = t - self.prev_time
        self.prev_time = t
        vel = np.array([vel_xyz[0], vel_xyz[1], vel_xyz[2]])
        av = self.angular_velocity_z
        if av is not None and not all(self.pinger_coordinates == np.zeros(3)):
            omega = np.array([0.0, 0.0, av])
            p = self.pinger_coordinates
            self.pinger_coordinates -= (vel + np.cross(omega, p)) * dt
        return self.pinger_coordinates.copy()
