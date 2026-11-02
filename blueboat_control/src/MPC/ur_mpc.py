"""Two-thruster MPC (acados OCP, SQP_RTI + qpOASES) for master_control.

ROS-free by construction: importable from a bare Python prompt. Failure
handling (C6) and the solver cache are documented in src/CLAUDE.md §4.
"""
import warnings
warnings.filterwarnings("ignore", category=SyntaxWarning)   # acados_template's own

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import numpy as np
import ctypes
import math
import os

def acados_build_dir(build_dir=None):
    """Absolute directory holding the generated + compiled acados solver:
    $ROS_HOME/blueboat_control/mpc by default. acados' relative defaults would
    land it in the launch's working directory (a full rebuild per directory);
    never under data_root(), which is write-once field record (CM-7)."""
    if build_dir is None:
        ros_home = os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')
        build_dir = os.path.join(ros_home, 'blueboat_control', 'mpc')
    build_dir = os.path.abspath(os.path.expanduser(build_dir))
    os.makedirs(build_dir, exist_ok=True)
    return build_dir


def preload_acados_libs():
    """dlopen libacados' dependencies by absolute path, RTLD_GLOBAL.

    libacados.so has no rpath to blasfeo / hpipm / qpOASES_e, and editing
    LD_LIBRARY_PATH at run time is too late for the loader; preloading them
    lets the later dlopen resolve. Best effort: a missing file is not an error.
    """
    try:
        from acados_template.utils import get_acados_path
        lib_dir = os.path.join(get_acados_path(), 'lib')
    except Exception:
        return
    for name in ('libblasfeo.so', 'libhpipm.so', 'libqpOASES_e.so'):
        path = os.path.join(lib_dir, name)
        if os.path.exists(path):
            try:
                ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass


def _set_export_paths(ocp, build_dir):
    """Point the OCP at build_dir, across the 0.5.4 code_gen_opts rename."""
    export_dir = os.path.join(build_dir, 'c_generated_code')
    json_path = os.path.join(build_dir, 'acados_ocp.json')
    opts = getattr(ocp, 'code_gen_opts', None)
    if opts is not None:
        opts.code_export_directory = export_dir
        opts.json_file = json_path
    else:                                       # acados < 0.5.4
        ocp.code_export_directory = export_dir
    return json_path


def build_solver(ocp, build_dir):
    """Reuse the compiled solver when possible, otherwise generate and build.

    generate=False, build=False makes acados compare the stored json with
    this OCP and regenerate only on a difference (the defaults recompile every
    launch). The except branch covers an acados without that check.

    Returns (solver, reused); `reused` = the compiled library was on disk and
    untouched by the call (acados regenerates silently).
    """
    json_path = _set_export_paths(ocp, build_dir)
    preload_acados_libs()

    export_dir = os.path.join(build_dir, 'c_generated_code')
    name = getattr(ocp.model, 'name', None)
    lib = os.path.join(export_dir, f'libacados_ocp_solver_{name}.so')
    before = os.path.getmtime(lib) if name and os.path.exists(lib) else None

    try:
        solver = AcadosOcpSolver(ocp, json_file=json_path,
                                 generate=False, build=False)
    except Exception:
        _set_export_paths(ocp, build_dir)
        solver = AcadosOcpSolver(ocp, json_file=json_path)
        return solver, False

    after = os.path.getmtime(lib) if name and os.path.exists(lib) else None
    return solver, (before is not None and before == after)

def get_yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def export_underwater_model(
    robot_mass=10.0,
    iz=5.0,
    # Added mass / inertia: M = M_rb - diag(a_*), so negative values add
    # apparent inertia (the xacro's xDotU/yDotV/nDotR convention).
    a_u   = 0.,
    a_v   = 0.,
    a_r   = 0.,
    # Linear damping: D = -diag(d_*), so negative values damp.
    d_u   = 0.,
    d_v   = 0.,
    d_r   = 0.,
):
    """3-DOF surge/sway/yaw model, two thrusters 0.295 m off the centreline."""

    model = AcadosModel()
    model.name = "ur_robot_model"

    # States
    x   = ca.SX.sym('x')
    y   = ca.SX.sym('y')
    psi = ca.SX.sym('psi')
    u   = ca.SX.sym('u')
    v   = ca.SX.sym('v')
    r   = ca.SX.sym('r')
    X   = ca.vertcat(x, y, psi, u, v, r)

    # State derivatives (model.xdot)
    x_dot_sym   = ca.SX.sym('x_dot')
    y_dot_sym   = ca.SX.sym('y_dot')
    psi_dot_sym = ca.SX.sym('psi_dot')
    u_dot_sym   = ca.SX.sym('u_dot')
    v_dot_sym   = ca.SX.sym('v_dot')
    r_dot_sym   = ca.SX.sym('r_dot')
    Xdot = ca.vertcat(x_dot_sym, y_dot_sym, psi_dot_sym, u_dot_sym, v_dot_sym, r_dot_sym)

    # Controls: [right, left] thrust
    u1 = ca.SX.sym('u1')
    u2 = ca.SX.sym('u2')
    U = ca.vertcat(u1,u2)

    # Kinematics
    x_dot   = u * ca.cos(psi) - v * ca.sin(psi)
    y_dot   = u * ca.sin(psi) + v * ca.cos(psi)
    psi_dot = r

    # Build rigid-body mass and added-mass matrices
    M_rb = ca.DM([[robot_mass, 0.0,        0.0],
                  [0.0,        robot_mass, 0.0],
                  [0.0,        0.0,        iz ]])

    M_a = -ca.DM([[a_u,  0,   0],
                 [0,    a_v, 0],
                 [0,    0,   a_r]])

    M = M_rb + M_a

    C_rb = ca.vertcat(ca.horzcat(0.0,           -robot_mass * r, 0.0),
                      ca.horzcat(robot_mass * r, 0.0,            0.0),
                      ca.horzcat(0.0,            0.0,            0.0))

    C_a = ca.vertcat(ca.horzcat(0.0 ,       0.0,        a_v * v),
                     ca.horzcat(0.0,        0.0,       -a_u * u),
                     ca.horzcat(-a_v * v,   a_u * u,    0.0))
    
    C = C_rb + C_a

    D = -ca.DM([[d_u, 0.0, 0.0],
               [0.0, d_v, 0.0],
               [0.0, 0.0, d_r ]])

    nu = ca.vertcat(u, v, r)
    Dnu = ca.mtimes(D, nu)
    Cnu = ca.mtimes(C, nu)

    r = 0.295           # thruster lever arm [m] (rebinds the yaw-rate name)

    B = ca.vertcat(ca.horzcat(1.0,       1.0),
                   ca.horzcat(0.0,       0.0),
                   ca.horzcat(r  ,      -r))

    eq_tau = ca.mtimes(B, U)

    # M * nu_dot = B*U - C*nu - D*nu
    nu_dot = ca.solve(M, eq_tau - Cnu - Dnu)

    u_ddot = nu_dot[0]
    v_ddot = nu_dot[1]
    r_ddot = nu_dot[2]

    xdot = ca.vertcat(x_dot, y_dot, psi_dot, u_ddot, v_ddot, r_ddot)

    model.x = X
    model.xdot = Xdot
    model.u = U
    model.f_expl_expr = xdot

    return model

class MPCController:
    def __init__(self, robot_mass=10, 
        iz=5, 
        a_u = 0.,
        a_v = 0.,
        a_r = 0., 
        d_u = 0.,
        d_v = 0., 
        d_r = 0., 
        horizon=20, 
        time=2.0,
        Q_weight=None, 
        R_weight=None, 
        input_bounds=None,
        build_dir=None,
        qp_solver_iter_max=None,
        logger=None):

        self.mass = robot_mass
        self.iz = iz
        self.N = horizon
        self.T = time
        self.dt = time / horizon

        self.Q = Q_weight
        self.R = R_weight
        self.input_bounds = input_bounds

        # qpOASES working-set budget (see _build_ocp); None or <= 0 derives it.
        self.nu = 2
        self.qp_solver_iter_max = self._resolve_qp_iter_max(qp_solver_iter_max)

        # Failure reports: a plain callable (master_control passes its ROS
        # logger's warning), print otherwise. This module stays ROS-free.
        self._log = logger if callable(logger) else print

        # Solver health, read by master_control to decide what to publish.
        #   last_status    - acados status of the most recent solve (0 = ok)
        #   fail_count     - consecutive failures, reset by any success
        #   total_failures - failures over the life of the node
        self.last_status = 0
        self.fail_count = 0
        self.total_failures = 0
        self.last_solve_time = 0.0
        self.recoveries = 0          # reset-and-retry attempts

        self.build_dir = acados_build_dir(build_dir)

        self.model = export_underwater_model(self.mass, self.iz, a_u, a_v, a_r, d_u, d_v, d_r)
        self.ocp = self._build_ocp()
        self.solver, self.reused_solver = build_solver(self.ocp, self.build_dir)

    def _resolve_qp_iter_max(self, requested):
        """Working-set budget for the QP. None or <= 0 derives it from N."""
        if requested is not None and int(requested) > 0:
            return int(requested)
        return max(50, 4 * self.nu * self.N)

    def _apply_reference(self, x_refs, x_current):
        """Pin stage 0 to the measured state and load the stage references."""
        self.solver.set(0, 'x', x_current)
        self.solver.set(0, 'lbx', x_current)
        self.solver.set(0, 'ubx', x_current)

        u_refs = np.zeros((self.N, self.nu))
        for i in range(self.N):
            self.solver.set(i, 'yref', np.concatenate((x_refs[i], u_refs[i])))
        self.solver.set(self.N, 'yref', np.array(x_refs[-1]))

    def _seed_all_stages(self, x_current):
        """Cold, feasible seed after a reset: hold the measured state, zero input."""
        zero_u = np.zeros(self.nu)
        for i in range(self.N + 1):
            self.solver.set(i, 'x', x_current)
        for i in range(self.N):
            self.solver.set(i, 'u', zero_u)

    def _stat(self, field):
        """solver.get_stats(field), or None: diagnostics never raise."""
        try:
            return self.solver.get_stats(field)
        except Exception:
            return None

    def _build_ocp(self):
        model = self.model
        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = self.N

        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu

        # Cost setup
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W = np.eye(ny)
        ocp.cost.W[:nx, :nx] = self.Q
        ocp.cost.W[nx:, nx:] = self.R
        ocp.cost.W_e = self.Q
        ocp.constraints.x0 = np.zeros(6)
        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(nx)

        ocp.cost.Vx = np.vstack([np.eye(nx), np.zeros((nu, nx))])
        ocp.cost.Vu = np.vstack([np.zeros((nx, nu)), np.eye(nu)])
        ocp.cost.Vx_e = np.eye(nx)

        # Input constraints
        ocp.constraints.lbu = self.input_bounds["lower"]
        ocp.constraints.ubu = self.input_bounds["upper"]
        ocp.constraints.idxbu = self.input_bounds["idx"]

        # qp_solver_iter_max is load-bearing: the dense active-set QP has
        # nv = N*nu variables (60 at the sim horizon), and acados' default
        # budget of 50 made every saturating solve fail (status 4) -- seen on
        # every Gazebo run with |psi_err| >= 2.2 rad. Finding C6.
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.qp_solver_iter_max = self.qp_solver_iter_max
        ocp.solver_options.tf = self.T

        return ocp

    def update_weights(self, Q_weight=None, R_weight=None):
        if Q_weight is not None:
            self.Q = Q_weight
        if R_weight is not None:
            self.R = R_weight

        # New weights change the OCP, so build_solver regenerates.
        self.ocp = self._build_ocp()
        self.solver, self.reused_solver = build_solver(self.ocp, self.build_dir)

    def solve(self, path, x_current):
        poses = path.poses[:self.N + 1]
        if len(poses) < self.N + 1:
            poses += [poses[-1]] * (self.N + 1 - len(poses))

        px = np.array([q.pose.position.x for q in poses])
        py = np.array([q.pose.position.y for q in poses])

        # Heading reference unwrapped CUMULATIVELY over the horizon (C2): the
        # LINEAR_LS cost does no wrapping, so a 2*pi cliff inside the window
        # pulled the solver to the far branch at full differential.
        psi_seq = np.unwrap(
            [(get_yaw_from_quaternion(q.pose.orientation) + np.pi) % (2 * np.pi) - np.pi
             for q in poses])

        # Rigid shift onto the branch nearest the MEASURED heading (differences,
        # hence the r references, unchanged).
        psi_seq = psi_seq + 2.0 * np.pi * np.round(
            (x_current[2] - psi_seq[0]) / (2.0 * np.pi))

        x_refs = []
        for i in range(self.N + 1):
            psi = psi_seq[i]
            if i > 0:
                dx = px[i] - px[i - 1]
                dy = py[i] - py[i - 1]
                u = math.hypot(dx, dy) / self.dt

                psi_prev = psi_seq[i - 1]
                dpsi = psi - psi_prev          # already continuous, no re-wrap
                r = dpsi / self.dt

                psi_mid = (psi + psi_prev) / 2.0
                dy_b = -math.sin(psi_mid) * dx + math.cos(psi_mid) * dy
                v = dy_b / self.dt
            else:
                u = 0.0
                v = 0.0
                r = 0.0

            x_refs.append([px[i], py[i], psi, u, v, r])

        x_refs = np.array(x_refs)  # shape (N+1, nx)

        self._apply_reference(x_refs, x_current)
        status = self.solver.solve()
        self.last_solve_time = self._stat('time_tot') or 0.0

        # RECOVERY (failure path only): SQP_RTI steps from the previous
        # iterate, so one bad step poisoned every later solve. Reset iterate
        # and QP memory, re-seed every stage at the measured state with zero
        # input, retry once.
        if status != 0:
            self.recoveries += 1
            self.solver.reset()
            self._seed_all_stages(x_current)
            self._apply_reference(x_refs, x_current)
            status = self.solver.solve()
            self.last_solve_time = self._stat('time_tot') or 0.0

        self.last_status = status

        # A FAILED SOLVE IS NOT A COMMAND (C6): acados leaves the stale iterate
        # in place, and replaying it drove the boat in circles. Return zeros;
        # the caller reads last_status / fail_count.
        if status != 0:
            self.fail_count += 1
            self.total_failures += 1
            self._log(
                f"acados solve failed (status {status}, "
                f"qp_iter={self._stat('qp_iter')}, qp_stat={self._stat('qp_stat')}, "
                f"{self.fail_count} consecutive, {self.total_failures} total, "
                f"{self.recoveries} resets) "
                f"- returning zero thrust, NOT the stale iterate")
            return np.zeros(self.model.u.size()[0])

        self.fail_count = 0
        return np.array(self.solver.get(0, 'u'))