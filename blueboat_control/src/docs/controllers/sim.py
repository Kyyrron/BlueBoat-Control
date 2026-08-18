"""Closed-loop simulation of every BlueBoat controller.

Plant  : the exact 3-DOF Fossen model from MPC/ur_mpc.py (same mass, added mass,
         Coriolis, damping and thrust-allocation matrix), integrated with RK4.
PID    : the REAL PID.PIDLoS class, imported from src/PID/PID.py (pure numpy).
LoS    : verbatim reimplementation of master_control.los_guidance.
PointLoS: verbatim reimplementation of master_control.solve_LoS.
MPC    : the acados OCP of ur_mpc.py re-solved with scipy SLSQP (same model,
         same Q/R, same N/T, same reference construction incl. the 15-vs-16
         padding and the dt mismatch). Not acados, but the same problem.
Reference + governor: verbatim from path_generation.single_pose,
         master_control.path_progress_errors / advance_governor and
         custom_functions.compute_target.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
from scipy.optimize import minimize

SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..",
                                   "Desktop", "Research Kyutech", "BlueBoat",
                                   "BlueBoat-SideScanSonar", "blueboat_control", "src"))
if not os.path.isdir(SRC):
    SRC = r"c:\Users\killi\Desktop\Research Kyutech\BlueBoat\BlueBoat-SideScanSonar\blueboat_control\src"
sys.path.insert(0, os.path.join(SRC, "PID"))
import PID as PIDmod  # noqa: E402  (the real controller)

# ───────────────────────── plant (ur_mpc.export_underwater_model) ──────────────
MASS, IZ = 16.01, 5.64
A_U, A_V, A_R = -26.77, -7.55, -21.77   # added mass (as passed by master_control)
D_U, D_V, D_R = -29.34, -51.54, -44.65  # viscous drag
RADIUS = 0.295
B_MAT = np.array([[1.0, 1.0], [0.0, 0.0], [RADIUS, -RADIUS]])
M = np.diag([MASS - A_U, MASS - A_V, IZ - A_R])
M_INV = np.linalg.inv(M)
D = np.diag([-D_U, -D_V, -D_R])
THR_LIM = 20.0


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def dynamics(s, thr, force_world=(0.0, 0.0)):
    """xdot for state [x, y, psi, u, v, r] under thruster forces `thr`."""
    x, y, psi, u, v, r = s
    nu = np.array([u, v, r])
    C = np.array([[0.0, -MASS * r, A_V * v],
                  [MASS * r, 0.0, -A_U * u],
                  [-A_V * v, A_U * u, 0.0]])
    tau = B_MAT @ np.asarray(thr, dtype=float)
    if force_world[0] or force_world[1]:           # environmental force -> body
        c, sn = math.cos(psi), math.sin(psi)
        fx, fy = force_world
        tau = tau + np.array([c * fx + sn * fy, -sn * fx + c * fy, 0.0])
    nu_dot = M_INV @ (tau - C @ nu - D @ nu)
    return np.array([u * math.cos(psi) - v * math.sin(psi),
                     u * math.sin(psi) + v * math.cos(psi),
                     r, nu_dot[0], nu_dot[1], nu_dot[2]])


def rk4(s, thr, h, force_world=(0.0, 0.0)):
    k1 = dynamics(s, thr, force_world)
    k2 = dynamics(s + 0.5 * h * k1, thr, force_world)
    k3 = dynamics(s + 0.5 * h * k2, thr, force_world)
    k4 = dynamics(s + h * k3, thr, force_world)
    return s + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


# ───────────────────────── reference (path_generation.single_pose) ─────────────
def single_pose(t, shape):
    x = y = yaw = 0.0
    if shape == "station_keeping":
        x = y = yaw = 0.0
    elif shape == "circle":
        radius = 4.0
        t *= 0.08
        x = -radius + radius * np.cos(t)
        y = radius * np.sin(t)
        yaw = wrap(np.arctan2(radius * np.cos(t), -radius * np.sin(t)))
    elif shape == "straight_line":
        x, y, yaw = 0.5 * t, 1.0, 0.0
    elif shape == "sin":
        if t > 500:
            t = 50
        a, f, vx = 3.5, 0.2, 0.4
        t *= 0.7
        x = 0.5 + vx * t
        y = a * (np.sin(f * t - np.pi / 2) + 1)
        yaw = np.arctan2(a * f * np.cos(f * t - np.pi / 2), vx)
    elif shape == "kin_square":
        if t > 500:
            t = 50
        seg_len, surge = 5.0, 0.3
        seg_time = seg_len / surge
        seg_i = int(t // seg_time)
        t_in = t % seg_time
        dirs = [(1, 0), (0, 1), (1, 0), (0, -1)]
        yaws = [0, math.pi / 2, 0, -math.pi / 2]
        dx, dy = dirs[seg_i % 4]
        yaw = yaws[seg_i % 4]
        x = y = 0.0
        for i in range(seg_i):
            dxi, dyi = dirs[i % 4]
            x += dxi * seg_len
            y += dyi * seg_len
        x += dx * surge * t_in
        y += dy * surge * t_in
    return x, y, yaw


# ───────────────────────── governor (master_control) ───────────────────────────
class Governor:
    def __init__(self, path_time, path_steps, speed_scale=1.0, lmin=0.5, lmax=3.0):
        self.tau = 0.0
        self.path_time, self.path_steps = path_time, path_steps
        self.scale, self.lmin, self.lmax = speed_scale, lmin, lmax

    def window(self, shape):
        ts = np.linspace(self.tau, self.tau + self.path_time, int(self.path_steps))
        return [single_pose(t, shape) for t in ts]

    def errors(self, win, state):
        x0, y0, gamma_p = win[0]
        x1, y1, _ = win[1] if len(win) > 1 else win[0]
        dtau = self.path_time / max(1, (self.path_steps - 1))
        U_d = math.hypot(x1 - x0, y1 - y0) / dtau if dtau > 0 else 0.0
        xb, yb = state[0], state[1]
        c, s = math.cos(gamma_p), math.sin(gamma_p)
        e_along = (x0 - xb) * c + (y0 - yb) * s
        e_y = -(xb - x0) * s + (yb - y0) * c
        return e_along, e_y, gamma_p, U_d

    def advance(self, e_along, dt):
        span = max(1e-6, self.lmax - self.lmin)
        factor = float(np.clip((self.lmax - e_along) / span, 0.0, 1.0))
        self.tau += self.scale * factor * dt
        return factor


def compute_target(win, dt):
    """custom_functions.compute_target on a 2-pose window."""
    x0, y0, psi0 = win[0]
    x1, y1, psi1 = win[1]
    psi = wrap(psi1)
    dx, dy = x1 - x0, y1 - y0
    u = np.hypot(dx, dy) / dt
    psi_mid = (psi + psi0) / 2.0
    v = (-np.sin(psi_mid) * dx + np.cos(psi_mid) * dy) / dt
    r = wrap(psi - psi0) / dt
    return [x1, y1, psi, u, v, r]


# ───────────────────────── controllers ─────────────────────────────────────────
class LoSController:
    """master_control.los_guidance (kinematic Fossen LoS)."""
    name = "LoS"
    path_time, path_steps = 0.05, 2

    def __init__(self, lookahead=2.5, ku=8.0, kpsi=10.0, kd=1.0, speed_scale=1.0):
        self.Delta, self.ku, self.kpsi, self.kd = lookahead, ku, kpsi, kd
        self.speed_scale = speed_scale
        self.alloc = PIDmod.ThrustAllocator(
            B_MAT, limits={"min": np.array([-THR_LIM] * 2), "max": np.array([THR_LIM] * 2)})

    def __call__(self, win, state, dt):
        t6 = compute_target(win, dt)
        x, y, psi, u, _, r = state
        x_ref, y_ref, gamma_p, U_d = t6[0], t6[1], t6[2], t6[3]
        c, s = math.cos(gamma_p), math.sin(gamma_p)
        e_y = -(x - x_ref) * s + (y - y_ref) * c
        psi_d = gamma_p + math.atan2(-e_y, self.Delta)
        psi_err = wrap(psi_d - psi)
        u_cmd = self.speed_scale * U_d * max(0.0, math.cos(psi_err))
        X = self.ku * (u_cmd - u)
        N = self.kpsi * psi_err - self.kd * r
        return self.alloc.allocate(np.array([X, 0.0, N])), t6


class PIDController:
    """The real PID.PIDLoS, wired exactly as master_control wires it."""
    name = "PID"
    path_time, path_steps = 0.05, 2

    def __init__(self, dt, lookahead=2.5,
                 outer=None, inner=None):
        outer = outer or {"x": (3., 0.01, 0.), "psi": (3.0, 0.01, 0.)}
        inner = inner or {"u": (1., 0., 0.), "r": (1.5, 0., 0.)}
        self.c = PIDmod.PIDLoS(dt=dt, B=B_MAT, outer_gains=outer, inner_gains=inner,
                               lookahead=lookahead,
                               thruster_limits={"min": np.array([-THR_LIM] * 2),
                                                "max": np.array([THR_LIM] * 2)})

    def __call__(self, win, state, dt):
        t6 = compute_target(win, dt)
        thr, _ = self.c.compute(state, t6[:3], u_ff=t6[3], psi_path=t6[2])
        return thr, t6


class MPCController:
    """ur_mpc.MPCController re-solved with SLSQP (same model/cost/horizon)."""
    name = "MPC"

    def __init__(self, horizon=15, time=2.5, path_steps=15, maxiter=12):
        self.N, self.T = horizon, time
        self.dt = time / horizon                      # 0.16667  (ur_mpc.py:156)
        self.path_time, self.path_steps = time, path_steps
        self.Q = np.diag([50., 50., 30., 1., 1., 1.])
        self.R = np.diag([0.015, 0.015])
        self.u_prev = np.zeros((self.N, 2))
        self.maxiter = maxiter

    def _refs(self, win):
        poses = list(win[:self.N + 1])
        while len(poses) < self.N + 1:                 # ur_mpc.py:217-218 padding
            poses.append(poses[-1])
        refs, psi_prev = [], None
        for i, (x, y, psi) in enumerate(poses):
            psi = wrap(psi)
            if i > 0:
                px, py, _ = poses[i - 1]
                psi = np.unwrap([psi_prev, psi])[-1]
                u = math.hypot(x - px, y - py) / self.dt
                psi_mid = (psi + psi_prev) / 2.0
                v = (-math.sin(psi_mid) * (x - px) + math.cos(psi_mid) * (y - py)) / self.dt
                r = wrap(psi - psi_prev) / self.dt
            else:
                u = v = r = 0.0
            refs.append([x, y, psi, u, v, r])
            psi_prev = psi
        return np.array(refs)

    def __call__(self, win, state, dt):
        refs = self._refs(win)

        def rollout(uflat):
            U = uflat.reshape(self.N, 2)
            s = np.array(state, dtype=float)
            cost = 0.0
            for i in range(self.N):
                e = s - refs[i]
                cost += e @ self.Q @ e + U[i] @ self.R @ U[i]
                s = rk4(s, U[i], self.dt)
            e = s - refs[self.N]
            return cost + e @ self.Q @ e

        x0 = np.vstack([self.u_prev[1:], self.u_prev[-1:]]).ravel()
        res = minimize(rollout, x0, method="SLSQP",
                       bounds=[(-THR_LIM, THR_LIM)] * (2 * self.N),
                       options={"maxiter": self.maxiter, "ftol": 1e-3})
        U = res.x.reshape(self.N, 2)
        self.u_prev = U
        return U[0], list(refs[0][:4])


class PointLoS:
    """master_control.solve_LoS -- body-frame point chase (pinger / manual)."""
    name = "Point-LoS"

    def __init__(self, k_v=2.0, k_psi=16.0, manual=False):
        self.k_v, self.k_psi, self.manual = k_v, k_psi, manual

    def __call__(self, target_world, state, dt):
        xt, yt = target_world
        xr, yr, psir = state[0], state[1], state[2]
        x = (xt - xr) * math.cos(psir) + (yt - yr) * math.sin(psir)
        y = (yt - yr) * math.cos(psir) - (xt - xr) * math.sin(psir)
        yaw_rate = self.k_psi * math.atan2(y, x)
        d = math.hypot(x, y)
        v = 5 * math.log(self.k_v * d + 1)
        if self.manual:
            v = 10 * math.log(v + 1)
        return np.array([v + 0.295 * yaw_rate, v - 0.295 * yaw_rate])


# ───────────────────────── simulation driver ───────────────────────────────────
def run(ctrl, shape="straight_line", start=(0., 0., 0.), T=80.0, dt=0.05,
        force_world=(0., 0.), speed_scale=1.0, plant_h=0.01):
    gov = Governor(ctrl.path_time, ctrl.path_steps, speed_scale=speed_scale)
    s = np.array([start[0], start[1], start[2], 0., 0., 0.])
    log = {k: [] for k in
           ("t", "x", "y", "psi", "u", "v", "r", "xd", "yd", "psid",
            "e_y", "e_along", "tau", "factor", "thr_r", "thr_l", "Ud")}
    n = int(T / dt)
    for k in range(n):
        meas = s.copy()
        meas[2] = wrap(meas[2])                      # odom delivers wrapped yaw
        win = gov.window(shape)
        e_along, e_y, gamma_p, U_d = gov.errors(win, meas)
        factor = gov.advance(e_along, dt)
        thr, t6 = ctrl(win, meas, dt)
        thr = np.clip(np.asarray(thr, dtype=float), -THR_LIM, THR_LIM)

        for key, val in (("t", k * dt), ("x", s[0]), ("y", s[1]), ("psi", wrap(s[2])),
                         ("u", s[3]), ("v", s[4]), ("r", s[5]),
                         ("xd", win[0][0]), ("yd", win[0][1]), ("psid", win[0][2]),
                         ("e_y", e_y), ("e_along", e_along), ("tau", gov.tau),
                         ("factor", factor), ("thr_r", thr[0]), ("thr_l", thr[1]),
                         ("Ud", U_d)):
            log[key].append(val)

        sub = max(1, int(round(dt / plant_h)))
        h = dt / sub
        for _ in range(sub):
            s = rk4(s, thr, h, force_world)
    return {k: np.asarray(v) for k, v in log.items()}


def run_point(ctrl, target, start=(0., 0., 0.), T=60.0, dt=0.05, plant_h=0.01):
    s = np.array([start[0], start[1], start[2], 0., 0., 0.])
    log = {k: [] for k in ("t", "x", "y", "psi", "u", "d", "thr_r", "thr_l")}
    for k in range(int(T / dt)):
        meas = s.copy()
        meas[2] = wrap(meas[2])
        thr = np.clip(ctrl(target, meas, dt), -THR_LIM, THR_LIM)
        for key, val in (("t", k * dt), ("x", s[0]), ("y", s[1]), ("psi", wrap(s[2])),
                         ("u", s[3]), ("d", math.hypot(target[0] - s[0], target[1] - s[1])),
                         ("thr_r", thr[0]), ("thr_l", thr[1])):
            log[key].append(val)
        sub = max(1, int(round(dt / plant_h)))
        for _ in range(sub):
            s = rk4(s, thr, dt / sub)
    return {k: np.asarray(v) for k, v in log.items()}


def reference_track(shape, tmax, n=1500):
    ts = np.linspace(0, tmax, n)
    pts = np.array([single_pose(t, shape)[:2] for t in ts])
    return pts[:, 0], pts[:, 1]


if __name__ == "__main__":
    import time as _t
    t0 = _t.time()
    d = run(LoSController(), "straight_line", start=(0., -4., 0.), T=20)
    print(f"LoS  20s in {_t.time()-t0:.2f}s   final e_y={d['e_y'][-1]:+.3f} m  u={d['u'][-1]:.2f} m/s")
    t0 = _t.time()
    d = run(PIDController(dt=0.05), "straight_line", start=(0., -4., 0.), T=20)
    print(f"PID  20s in {_t.time()-t0:.2f}s   final e_y={d['e_y'][-1]:+.3f} m  u={d['u'][-1]:.2f} m/s")
    t0 = _t.time()
    d = run(MPCController(), "straight_line", start=(0., -4., 0.), T=20)
    print(f"MPC  20s in {_t.time()-t0:.2f}s   final e_y={d['e_y'][-1]:+.3f} m  u={d['u'][-1]:.2f} m/s")
