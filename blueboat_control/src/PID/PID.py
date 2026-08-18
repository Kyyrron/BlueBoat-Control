import numpy as np


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        u = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        return u


class ThrustAllocator:
    """
    Allocates a desired wrench to individual thrusters with per-thruster limits.
    """

    def __init__(self, B, limits=None):
        self.set_matrix(B)
        self.limits = limits

    def set_matrix(self, B):
        self.B = B
        self.B_pinv = np.linalg.pinv(B)

    def set_limits(self, limits):
        """
        limits: dict with 'min' and 'max' arrays of size n_thrusters
        """
        self.limits = limits

    def allocate(self, tau):
        f = self.B_pinv @ tau

        if self.limits is None:
            return f

        # Apply per-thruster limits with uniform scaling to preserve direction
        scale = 1.0
        for i in range(len(f)):
            if f[i] > self.limits["max"][i]:
                scale = min(scale, self.limits["max"][i] / f[i])
            elif f[i] < self.limits["min"][i]:
                scale = min(scale, self.limits["min"][i] / f[i])

        return f * scale


class PIDLoS:
    """
    2-DOF cascaded controller (surge + yaw) with line-of-sight guidance,
    using world-frame position.

    Guidance is the canonical Fossen lookahead LoS law:

        psi_d = gamma_p + atan2(-y_e, Delta)

    where gamma_p is the path-tangent heading, y_e the signed cross-track
    error and Delta the lookahead distance. Larger Delta -> gentler,
    more damped approach to the path; smaller Delta -> more aggressive.

    Speed is handled by an optional feedforward u_ff (the path's desired
    speed at the current point) plus an along-track proportional-integral
    term, so a spatially varying speed profile is honored.

    Backward compatibility: called as compute(state, ref) with the defaults
    (lookahead = 1.0, u_ff = 0.0, psi_path = None -> gamma_p taken from
    ref[2]), this reproduces the previous behavior exactly, so the pinger /
    manual point-following paths are unchanged.
    """

    def __init__(self, dt, B, outer_gains, inner_gains,
                 lookahead=1.0, los_gain=None, thruster_limits=None):
        """
        dt : float
            Control timestep
        B : np.ndarray (3 x n) thrust allocation matrix
        outer_gains : dict {'x': (kp, ki, kd), 'psi': (kp, ki, kd)}
        inner_gains : dict {'u': (kp, ki, kd), 'r': (kp, ki, kd)}
        lookahead : float
            LoS lookahead distance Delta (metres). Replaces the old
            'los_gain' (Delta = 1 / los_gain). If los_gain is given it takes
            precedence for backward compatibility.
        thruster_limits : dict with 'min' and 'max' arrays
        """
        self.dt = dt
        if los_gain is not None and los_gain != 0.0:
            self.lookahead = 1.0 / los_gain
        else:
            self.lookahead = lookahead

        # Outer loop PIDs
        self.pid_x = PID(*outer_gains['x'], dt)     # along-track -> surge speed correction
        self.pid_psi = PID(*outer_gains['psi'], dt) # heading error -> yaw-rate reference

        # Inner loop PIDs
        self.pid_u = PID(*inner_gains['u'], dt)     # surge speed error -> force
        self.pid_r = PID(*inner_gains['r'], dt)     # yaw-rate error -> moment

        # Thruster allocation
        self.allocator = ThrustAllocator(B, limits=thruster_limits)

    def set_allocation_matrix(self, B):
        self.allocator.set_matrix(B)

    def set_thruster_limits(self, thruster_limits):
        self.allocator.set_limits(thruster_limits)

    def reset(self):
        self.pid_x.reset()
        self.pid_psi.reset()
        self.pid_u.reset()
        self.pid_r.reset()

    def compute(self, state, ref, u_ff=0.0, psi_path=None, slow_on_turn=False):
        """
        state : [x, y, psi, u, v, r]   (world-frame x, y, psi; body-frame u, v, r)
        ref   : [x_ref, y_ref, psi_ref]
        u_ff  : float, desired path speed feedforward (m/s). Default 0.0.
        psi_path : float or None, path-tangent heading gamma_p. If None,
                   ref[2] is used (identical to the previous implementation).
        slow_on_turn : if True, scale the surge command by max(0, cos(psi_err))
                   so the boat slows while turning hard onto the path. Off by
                   default to preserve prior point-following behavior.
        Returns:
            thrusts : np.ndarray (n_thrusters)
            tau : np.ndarray [X, Y=0, N]
        """
        x, y, psi, u, v, r = state
        x_ref, y_ref, psi_ref = ref

        # --- Position error in world frame ---
        ex_w = x_ref - x
        ey_w = y_ref - y

        # --- Along-track and cross-track ---
        # When psi_path is None (point following: pinger / manual) the error is
        # projected onto the BOAT heading, reproducing the previous behavior
        # exactly. When psi_path is given (path following) it is projected onto
        # the PATH tangent, which is the correct frame for LoS guidance.
        if psi_path is None:
            frame_angle = psi
        else:
            frame_angle = psi_path
        c = np.cos(frame_angle)
        s = np.sin(frame_angle)
        e_along =  c * ex_w + s * ey_w      # along-track error (target ahead > 0)
        ev      = -s * ex_w + c * ey_w      # signed cross-track term (== old 'ev')

        # Heading the LoS correction is added to: path tangent if given, else psi_ref
        gamma_p = psi_ref if psi_path is None else psi_path

        # --- LoS lookahead steering ---
        # atan2(ev, Delta) is identical to the previous atan2(los_gain*ev, 1)
        # when Delta = 1/los_gain, and is the canonical Fossen lookahead law.
        psi_los = np.arctan2(ev, self.lookahead)
        psi_des = wrap_angle(gamma_p + psi_los)
        epsi = wrap_angle(psi_des - psi)

        # --- Outer loop (position -> velocity references) ---
        u_ref = u_ff + self.pid_x.update(e_along)
        if slow_on_turn:
            u_ref *= max(0.0, np.cos(epsi))
        r_ref = self.pid_psi.update(epsi)

        # --- Inner loop (velocity -> wrench) ---
        X = self.pid_u.update(u_ref - u)
        N = self.pid_r.update(r_ref - r)
        tau = np.array([X, 0.0, N])  # No sway actuation

        # --- Allocate to thrusters with per-thruster limits ---
        thrusts = self.allocator.allocate(tau)

        return thrusts, tau
