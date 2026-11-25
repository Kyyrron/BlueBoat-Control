#!/usr/bin/env python3
r"""
The pinger filter: a robust Kalman estimate of the USBL pinger's WORLD position.

ROS-FREE BY CONSTRUCTION (numpy only). Used by the pinger_estimator node (live),
the pinger_filter_replay CLI, poslog_report (run-end smoothing) and the log
reviewer. It reads no clock: every time is passed in, so the same inputs give
the same outputs bit for bit, online or offline.

Frames (docs/PINGER_FILTER_FINDINGS.md section 5)
  acoustic a = (a_x, a_y, a_z)  as the Water Linked API returns it
  body       b = R(mount_yaw) * (a_x, y_sign*a_y) + lever      (x forward, y left)
  world      z = p_boat(t_m) + R(psi(t_m)) * b                 (/blueboat/odom local ENU)
  depth      = z_sign * a_z  (positive down)
with t_m = t_rx - latency_s and the boat pose interpolated in a receive-time
pose buffer (yaw on the circle).

State: [pE, pN, vE, vN] (model 'cv', white-acceleration constant velocity) or
[pE, pN] (model 'static', random-walk drift), plus an independent 1-D depth
filter. Robust update (default Huber): NIS d2 = nu' S^-1 nu; d2 <= gate_soft is a
normal update; above it R is inflated by d2/gate_soft ("down-weighted", the fix
still pulls); only d2 > gate_hard AND an implied speed > v_hard sets a fix aside.
reinit_n consecutive fixes beyond the soft gate that agree within reinit_radius_m
re-acquire (re-initialise) the estimate there, so the filter cannot lock out. Duplicates never update (they would shrink P falsely).

Between fixes the world estimate does NOT move (no extrapolation): the body
vector a consumer steers to is recomputed from the held estimate and the
current pose, b = R(psi)^T (p_hat - p_boat). That is a frame change, not motion.
"""

import bisect
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
CLOCK_STEP_BACK_S = 1.0                # a pose older than the newest by more restarts the buffer
FIX_RATE_WINDOW_S = 10.0


def _f(v):
    """JSON-safe float (None for None / non-finite)."""
    if v is None:
        return None
    v = float(v)
    return v if math.isfinite(v) else None


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


class PingerFilter:
    """Robust pinger estimate. `params` is the dict of parameter values
    (pinger_params.Params.values)."""

    def __init__(self, params, keep_steps=True):
        self.p = dict(params)
        # The forward pass is stored for the RTS smoother (offline). The live
        # node runs for hours and never smooths: it passes keep_steps=False.
        self.keep_steps = bool(keep_steps)
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        p = self.p
        self.cv = p["model"] == "cv"
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
        self.steps = []                 # forward pass (for the RTS smoother)
        self.segment = 0
        self.decision_log = deque()     # (t_rx, decision)
        self.valid_log = deque()        # t_rx of valid, non-duplicate fixes
        self.event = None
        self.event_t = None
        self.updates = 0
        self.last_nis = None

    @property
    def initialised(self):
        return self.x is not None

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
        """2x2 world measurement covariance of a fix at acoustic range rho along
        the unit line of sight (ux, uy)."""
        p = self.p
        if p["noise_model"] == "iso":
            sig = p["iso_sigma_m"]
            if p["use_wl_std"] and std is not None:
                sig = max(sig, std)
            sig = max(sig, 0.01)
            return np.eye(2) * sig * sig
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
        if v > vmax:
            if v > 0:
                k = vmax / v
                self.x[2] *= k
                self.x[3] *= k

    # ------------------------------------------------------------------ fixes
    def process(self, fix, pose):
        """Process one Fix with the boat pose at its measurement time
        (`pose` = (x, y, yaw, yaw_rate) or None). Returns the result record
        (/blueboat/pinger/fix payload, without the transport fields)."""
        p = self.p
        t_m = fix.t_rx - p["latency_s"]
        res = {"t_rx": fix.t_rx, "t_meas": t_m, "source": fix.source, "origin": fix.origin,
               "seq": fix.seq, "a": list(fix.a), "std": fix.std, "input_invalid": fix.invalid,
               "position_valid": fix.position_valid, "receivers_valid": fix.receivers_valid,
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
            self.init_buf.append(cand)
            while self.init_buf and self.init_buf[0][0] < fix.t_rx - p["init_window_s"]:
                self.init_buf.popleft()
            if len(self.init_buf) >= p["init_n"]:
                group = list(self.init_buf)[-p["init_n"]:]
                if self._consistent(group, p["init_radius_m"]):
                    self._start(group, fix.t_rx, "acquired")
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
        Sinv = np.linalg.inv(S)
        d2 = float(nu @ Sinv @ nu)
        res["nis"] = d2
        v_imp = float(np.hypot(nu[0], nu[1])) / max(dt, 0.2)
        res["v_implied"] = v_imp
        mode = p["robust"]
        # Re-acquisition looks at EVERY fix beyond the soft gate, set aside or
        # down-weighted, and asks whether the last reinit_n of them CLUSTER: with
        # field noise (bearing ~12 deg, range ~20 %) a genuine 15 m relocation
        # gives NIS ~13-16, inside the hard gate, so it is only down-weighted
        # fix by fix -- but random outliers do not agree with each other.
        if mode != "none" and d2 > p["gate_soft"] and self._reacquire(cand):
            res["event"] = "reacquired"
            return done("reinit", f"re-acquired after {p['reinit_n']} consistent fixes beyond gate_soft")
        beyond = d2 > p["gate_hard"] and v_imp > p["v_hard"]
        R_eff, weight, decision, reason = R, 1.0, "accepted", ""
        if mode == "chi2":
            if d2 > p["gate_soft"]:
                return done("set_aside", f"NIS {d2:.1f} > gate_soft (chi2 gate)")
        elif mode == "huber":
            if d2 > p["gate_soft"]:
                if beyond:
                    return done("set_aside", f"NIS {d2:.1f} > gate_hard at {v_imp:.1f} m/s")
                weight = p["gate_soft"] / d2
                R_eff = R / weight
                decision, reason = "downweighted", f"NIS {d2:.1f}: R x {1.0 / weight:.2f}"
        elif mode == "student_t":
            if beyond:
                return done("set_aside", f"NIS {d2:.1f} > gate_hard at {v_imp:.1f} m/s")
            weight = self._student_weight(x_prior, P_prior, z, R)
            R_eff = R / weight
            if weight < 0.999:
                decision, reason = "downweighted", f"t weight {weight:.2f}"
        self._update(x_prior, P_prior, z, R_eff, F, t_m, fix.t_rx, depth)
        if d2 <= p["gate_soft"]:
            self.setaside_buf.clear()           # a normal fix ends any re-acquisition streak
        self.last_nis = d2
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
        self.segment += 1
        self._step(None, self.x, self.P)
        self.init_buf.clear()
        self.setaside_buf.clear()
        self.event, self.event_t = event, t_rx
        self.updates += 1

    def _reacquire(self, cand):
        """Remember a fix beyond the soft gate; True (and re-initialised) when the
        last reinit_n such fixes, within reinit_window_s, agree within
        reinit_radius_m of their median."""
        p = self.p
        self.setaside_buf.append(cand)
        while self.setaside_buf and self.setaside_buf[0][0] < cand[0] - p["reinit_window_s"]:
            self.setaside_buf.popleft()
        if len(self.setaside_buf) >= p["reinit_n"]:
            group = list(self.setaside_buf)[-p["reinit_n"]:]
            if self._consistent(group, p["reinit_radius_m"]):
                self._start(group, cand[0], "reacquired")
                return True
        return False

    def _student_weight(self, x_prior, P_prior, z, R):
        """Variational Student-t: the expected precision scale of R (capped at 1)."""
        dof = self.p["t_dof"]
        lam = 1.0
        Rinv = np.linalg.inv(R)
        H = self.H
        for _ in range(int(self.p["t_iters"])):
            S = H @ P_prior @ H.T + R / lam
            K = P_prior @ H.T @ np.linalg.inv(S)
            xp = x_prior + K @ (z - H @ x_prior)
            Pp = (np.eye(self.dim) - K @ H) @ P_prior
            e = z - H @ xp
            B = np.outer(e, e) + H @ Pp @ H.T
            lam = min(1.0, (dof + 2.0) / (dof + float(np.trace(Rinv @ B))))
        return lam

    def _update(self, x_prior, P_prior, z, R_eff, F, t_m, t_rx, depth):
        H = self.H
        S = H @ P_prior @ H.T + R_eff
        K = P_prior @ H.T @ np.linalg.inv(S)
        x = x_prior + K @ (z - H @ x_prior)
        IKH = np.eye(self.dim) - K @ H
        P = IKH @ P_prior @ IKH.T + K @ R_eff @ K.T
        self.x, self.P = x, P
        self._cap_velocity()
        # depth: 1-D random walk, Huber on its own
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
        self.updates += 1
        self._step(F, x_prior, P_prior)

    def _step(self, F, x_prior, P_prior):
        if self.keep_steps:
            self.steps.append({"segment": self.segment, "t": self.t_state, "F": F,
                               "x_prior": x_prior.copy(), "P_prior": P_prior.copy(),
                               "x_post": self.x.copy(), "P_post": self.P.copy()})

    # --------------------------------------------------------------- snapshot
    def snapshot(self):
        """Everything the next fix's processing depends on, JSON-able (floats
        round-trip exactly through json). restore() of it continues the filter
        bit for bit: a per-fix log that starts mid-session opens with one."""
        def cand(c):
            t_rx, t_m, z, R, depth = c
            return [t_rx, t_m, [float(z[0]), float(z[1])], [[float(v) for v in r] for r in R], depth]
        return {
            "x": None if self.x is None else [float(v) for v in self.x],
            "P": None if self.P is None else [[float(v) for v in r] for r in self.P],
            "depth": self.depth, "Pd": self.Pd, "t_state": self.t_state,
            "t_rx_update": self.t_rx_update,
            "init_buf": [cand(c) for c in self.init_buf],
            "setaside_buf": [cand(c) for c in self.setaside_buf],
            "last_key": None if self.last_key is None else list(self.last_key),
            "seen_valid": self.seen_valid, "segment": self.segment, "updates": self.updates,
            "event": self.event, "event_t": self.event_t, "last_nis": self.last_nis,
            "decision_log": [[t, d] for t, d in self.decision_log],
            "valid_log": list(self.valid_log),
        }

    def restore(self, snap):
        """Continue from a snapshot() (same parameters). The forward pass for
        the smoother starts a new segment here."""
        def cand(c):
            return (c[0], c[1], np.array(c[2], dtype=float), np.array(c[3], dtype=float), c[4])
        self.x = None if snap["x"] is None else np.array(snap["x"], dtype=float)
        self.P = None if snap["P"] is None else np.array(snap["P"], dtype=float)
        self.depth, self.Pd = snap["depth"], snap["Pd"]
        self.t_state, self.t_rx_update = snap["t_state"], snap["t_rx_update"]
        self.init_buf = deque(cand(c) for c in snap["init_buf"])
        self.setaside_buf = deque(cand(c) for c in snap["setaside_buf"])
        self.last_key = None if snap["last_key"] is None else tuple(snap["last_key"])
        self.seen_valid = bool(snap["seen_valid"])
        self.segment = int(snap["segment"]) + 1
        self.updates = int(snap["updates"])
        self.event, self.event_t, self.last_nis = snap["event"], snap["event_t"], snap["last_nis"]
        self.decision_log = deque((t, d) for t, d in snap["decision_log"])
        self.valid_log = deque(snap["valid_log"])
        self.steps = []
        if self.x is not None:
            self._step(None, self.x, self.P)

    # ---------------------------------------------------------------- outputs
    def status(self, t):
        if not self.seen_valid:
            return "no_data"
        if not self.initialised:
            return "initialising"
        age = t - self.t_rx_update
        if age <= self.p["coast_after_s"]:
            return "tracking"
        if age <= self.p["stale_timeout_s"]:
            return "coasting"
        return "stale"

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
        if not self.initialised:
            return None
        F, Q = self._FQ(max(0.0, t - (self.t_state + self.p["latency_s"])))
        P = F @ self.P @ F.T + Q
        return P[:2, :2]

    def stats(self, t):
        w = self.p["stats_window_s"]
        while self.decision_log and self.decision_log[0][0] < t - w:
            self.decision_log.popleft()
        counts = {d: 0 for d in DECISIONS}
        for _, d in self.decision_log:
            counts[d] += 1
        used = counts["accepted"] + counts["downweighted"] + counts["set_aside"] + counts["reinit"]
        return {"window_s": w, "counts": counts,
                "pct_downweighted": _f(100.0 * counts["downweighted"] / used) if used else None,
                "pct_set_aside": _f(100.0 * counts["set_aside"] / used) if used else None}

    def fix_rate(self, t):
        while self.valid_log and self.valid_log[0] < t - FIX_RATE_WINDOW_S:
            self.valid_log.popleft()
        return len(self.valid_log) / FIX_RATE_WINDOW_S

    def estimate(self, t):
        """The /blueboat/pinger/estimate payload at time t (filter part)."""
        status = self.status(t)
        out = {"t": _f(t), "status": status, "status_code": STATUS_CODE[status],
               "event": self.event if (self.event_t is not None and t - self.event_t <= EVENT_HOLD_S) else None,
               "initialised": self.initialised, "updates": self.updates,
               "x": None, "y": None, "depth": None, "vx": None, "vy": None,
               "cov": None, "vel_cov": None, "r95": None, "ellipse": None,
               "fix_age": _f(t - self.t_rx_update) if self.t_rx_update is not None else None,
               "fix_rate": _f(self.fix_rate(t)), "last_nis": _f(self.last_nis),
               "stats": self.stats(t)}
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
                       vel_cov=([_f(self.P[2, 2]), _f(self.P[2, 3]), _f(self.P[3, 3])] if self.cv else None),
                       r95=_f(a), ellipse={"a": _f(a), "b": _f(b), "theta": _f(theta)})
        return out

    # ---------------------------------------------------------------- smoother
    def rts_smooth(self):
        """Rauch-Tung-Striebel smoothing of the stored forward pass, segment by
        segment (a re-initialisation starts a new segment). Returns a list of
        dicts {t, segment, x, y, vx, vy, cov} at every update time."""
        out = []
        steps = self.steps
        i = 0
        while i < len(steps):
            j = i
            while j + 1 < len(steps) and steps[j + 1]["segment"] == steps[i]["segment"]:
                j += 1
            seg = steps[i:j + 1]
            n = len(seg)
            xs = [None] * n
            Ps = [None] * n
            xs[-1], Ps[-1] = seg[-1]["x_post"].copy(), seg[-1]["P_post"].copy()
            for k in range(n - 2, -1, -1):
                F = seg[k + 1]["F"]
                C = seg[k]["P_post"] @ F.T @ np.linalg.inv(seg[k + 1]["P_prior"])
                xs[k] = seg[k]["x_post"] + C @ (xs[k + 1] - seg[k + 1]["x_prior"])
                Ps[k] = seg[k]["P_post"] + C @ (Ps[k + 1] - seg[k + 1]["P_prior"]) @ C.T
            for k in range(n):
                out.append({"t": seg[k]["t"], "segment": seg[k]["segment"],
                            "x": float(xs[k][0]), "y": float(xs[k][1]),
                            "vx": float(xs[k][2]) if self.cv else 0.0,
                            "vy": float(xs[k][3]) if self.cv else 0.0,
                            "cov": [float(Ps[k][0, 0]), float(Ps[k][0, 1]), float(Ps[k][1, 1])]})
            i = j + 1
        return out
