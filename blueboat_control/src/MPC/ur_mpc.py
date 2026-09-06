# Remove syntax warnings from acados
import warnings
warnings.filterwarnings("ignore", category=SyntaxWarning)

# Regular imports
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import numpy as np
import ctypes
import math
import os

# ----------------------------------------------------------------------------
#  acados build location
#
#  acados defaults both its json file and its code_export_directory to RELATIVE
#  paths, so the generated C, the Makefile and the compiled solver land in
#  whatever directory the process was started from. Under `ros2 launch` that is
#  the operator's shell -- or, when the Mission Control Station starts the run,
#  the station's own repository. Worse, a new directory means a full ~60 s
#  rebuild every time. Pin both to one stable place instead.
#
#  Deliberately NOT under custom_functions.data_root(): that tree is write-once
#  field record (CM-7) and build artifacts do not belong in it.
# ----------------------------------------------------------------------------

def acados_build_dir(build_dir=None):
    """Absolute directory holding the generated + compiled acados solver."""
    if build_dir is None:
        ros_home = os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')
        build_dir = os.path.join(ros_home, 'blueboat_control', 'mpc')
    build_dir = os.path.abspath(os.path.expanduser(build_dir))
    os.makedirs(build_dir, exist_ok=True)
    return build_dir


def preload_acados_libs():
    """dlopen libacados' own dependencies by absolute path, RTLD_GLOBAL.

    libacados.so carries no rpath to libblasfeo / libhpipm / libqpOASES_e, so
    without the acados lib directory on LD_LIBRARY_PATH the ctypes load inside
    AcadosOcpSolver dies with `libqpOASES_e.so: cannot open shared object file`.
    Editing os.environ at this point cannot help -- the dynamic loader read
    LD_LIBRARY_PATH when the process started. Loading the dependencies here by
    absolute path does: once they are in the process, the later dlopen of
    libacados.so resolves against them.

    Best effort by design: a missing file is not an error, because a correctly
    exported LD_LIBRARY_PATH (or a differently built acados) needs none of this.
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

    acados short-circuits its own check_reuse_possible branch unless one of
    generate/build is False, so passing the plain defaults recompiles the
    identical model on every single launch (tens of seconds). Asking for
    generate=False, build=False instead makes acados compare the stored json
    against this OCP and regenerate only when they differ -- so the same call
    both reuses a matching solver and rebuilds a stale one. The except branch is
    a safety net for an acados that lacks that check, not the normal path.

    Returns (solver, reused), where `reused` is decided by whether a compiled
    solver was already on disk BEFORE the call -- acados does the regeneration
    silently, so the flag cannot be inferred from the constructor returning.
    """
    json_path = _set_export_paths(ocp, build_dir)
    preload_acados_libs()

    export_dir = os.path.join(build_dir, 'c_generated_code')
    name = getattr(ocp.model, 'name', None)
    before = os.path.getmtime(os.path.join(export_dir, f'libacados_ocp_solver_{name}.so')) \
        if name and os.path.exists(os.path.join(export_dir, f'libacados_ocp_solver_{name}.so')) else None

    try:
        solver = AcadosOcpSolver(ocp, json_file=json_path,
                                 generate=False, build=False)
    except Exception:
        _set_export_paths(ocp, build_dir)
        solver = AcadosOcpSolver(ocp, json_file=json_path)
        return solver, False

    lib = os.path.join(export_dir, f'libacados_ocp_solver_{name}.so')
    after = os.path.getmtime(lib) if name and os.path.exists(lib) else None
    return solver, (before is not None and before == after)

# Utility to convert quaternion to yaw
def get_yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# Model export

def export_underwater_model(
    robot_mass=10.0,
    iz=5.0,
    # Added-mass matrix entries (positive values increase apparent inertia)
    a_u   = 0.,    # added mass in surge
    a_v   = 0.,    # added mass in sway
    a_r   = 0.,    # added inertia in yaw

    # Linear damping (D). Use positive values; model subtracts D*nu.
    d_u   = 0.,    # surge damping
    d_v   = 0.,    # sway damping
    d_r   = 0.,    # yaw damping
):

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

    # State derivatives (for implicit form)
    x_dot_sym   = ca.SX.sym('x_dot')
    y_dot_sym   = ca.SX.sym('y_dot')
    psi_dot_sym = ca.SX.sym('psi_dot')
    u_dot_sym   = ca.SX.sym('u_dot')
    v_dot_sym   = ca.SX.sym('v_dot')
    r_dot_sym   = ca.SX.sym('r_dot')
    Xdot = ca.vertcat(x_dot_sym, y_dot_sym, psi_dot_sym, u_dot_sym, v_dot_sym, r_dot_sym)

    # Transient vector (tau = B*u)
    tau_u = ca.SX.sym('tau_u')
    tau_v = ca.SX.sym('tau_v')
    tau_r = ca.SX.sym('tau_r')

    # Controls
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

    # Damping matrix D
    D = -ca.DM([[d_u, 0.0, 0.0],
               [0.0, d_v, 0.0],
               [0.0, 0.0, d_r ]])

    # Velocity vector nu = [u, v, r]
    nu = ca.vertcat(u, v, r)

    # D*nu
    Dnu = ca.mtimes(D, nu)

    # C*nu
    Cnu = ca.mtimes(C, nu)

    # Tau
    tau = ca.vertcat(tau_u, tau_v, tau_r)

    r = 0.295

    B = ca.vertcat(ca.horzcat(1.0,       1.0),
                   ca.horzcat(0.0,       0.0),
                   ca.horzcat(r  ,      -r))

    eq_tau = ca.mtimes(B, U)

    # Solve for nu_dot: M * nu_dot = tau - D*nu - g  =>  nu_dot = M^{-1} * (...)
    # Use casadi inverse (for 2x2 it's fine). If you prefer numerical stability
    # for larger matrices, use ca.solve(M, rhs) instead.
    nu_dot = ca.solve(M, eq_tau - Cnu - Dnu)

    u_ddot = nu_dot[0]
    v_ddot = nu_dot[1]
    r_ddot = nu_dot[2]

    # assemble xdot
    xdot = ca.vertcat(x_dot, y_dot, psi_dot, u_ddot, v_ddot, r_ddot)

    # Pack model
    model.x = X
    model.xdot = Xdot

    model.x = X
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

        # How many working-set changes qpOASES is allowed. See the block above
        # ocp.solver_options in _build_ocp for why the acados default of 50 is
        # not enough at horizon 30. None (or <= 0) derives it from the horizon,
        # which is the only place the arithmetic should live.
        self.nu = 2
        self.qp_solver_iter_max = self._resolve_qp_iter_max(qp_solver_iter_max)

        # Where a solver failure is reported. A plain callable, NOT a ROS
        # logger: this module is ROS-free by construction (CLAUDE.md section 2.1)
        # and must stay importable from a bare Python prompt. master_control
        # hands in self.get_logger().warning; the two standalone MPC nodes hand
        # in nothing and keep printing, as they always did.
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

        # Where the generated C and the compiled solver live. None resolves to
        # $ROS_HOME/blueboat_control/mpc (see acados_build_dir); the argument is
        # a plain Python keyword, NOT a declared ROS parameter, so the node's
        # interface surface is unchanged (N1 / CM-1).
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
        """Cold, feasible seed: hold the measured state, command nothing.

        Used only after a reset. It carries no memory of the basin the solver
        was stuck in, which is the entire point.
        """
        zero_u = np.zeros(self.nu)
        for i in range(self.N + 1):
            self.solver.set(i, 'x', x_current)
        for i in range(self.N):
            self.solver.set(i, 'u', zero_u)

    def _stat(self, field):
        """solver.get_stats(field), or None. Never raises.

        Diagnostics must not be able to kill the control loop: an acados
        upgrade that renames a stats field would otherwise turn a log line
        into an exception inside timer_callback.
        """
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

        # Solver setup
        #
        # qp_solver_iter_max is NOT decoration. FULL_CONDENSING_QPOASES is a
        # dense ACTIVE-SET method, so the condensed QP has nv = N*nu variables
        # (60 at the simulation horizon of 30, 30 at the real boat's 15) and
        # 2*nv bound constraints -- the only constraints this OCP has besides
        # the initial state. Reaching a vertex where most of those bounds are
        # active costs on the order of one working-set change per active bound.
        # acados defaults the budget to 50, so at N = 30 the cap is BELOW the
        # number of variables and the solve fails by construction whenever the
        # optimum saturates the horizon -- returning status 4 (QP_FAILURE), on
        # which acados leaves the primal iterate untouched.
        #
        # Measured (CONTROLLERS.md C6): across 34 recorded Gazebo runs the
        # discriminator is the heading error. Every run reaching |psi_err| >=
        # 2.2 rad failed on 58-100 % of ticks; every run staying under 0.95 rad
        # was clean -- same trajectory, same compiled solver. A large heading
        # error drives full differential across the whole horizon, which is
        # exactly the all-bounds-active vertex. At N = 15 (30 variables against
        # 50) the cap was never binding, which is why the real boat and every
        # pre-2026-09-01 simulation run were unaffected.
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

        # Rebuild OCP and solver. New weights change the OCP, so the reuse
        # attempt inside build_solver falls through to a full regeneration.
        self.ocp = self._build_ocp()
        self.solver, self.reused_solver = build_solver(self.ocp, self.build_dir)

    def solve(self, path, x_current):
        poses = path.poses[:self.N + 1]
        if len(poses) < self.N + 1:
            poses += [poses[-1]] * (self.N + 1 - len(poses))

        px = np.array([q.pose.position.x for q in poses])
        py = np.array([q.pose.position.y for q in poses])

        # Heading reference: unwrap CUMULATIVELY across the whole horizon (C2).
        #
        # This used to be a pairwise np.unwrap([psi_prev, psi]) in which psi_prev
        # was re-read from the pose and therefore freshly wrapped every
        # iteration, so the unwrap never accumulated. A window straddling +/-pi
        # came out as e.g. 3.140, 3.143, -3.100 -- a 2*pi cliff INSIDE the
        # horizon. The LINEAR_LS cost does no wrapping of its own and the
        # terminal cost carries the same weight on psi, so the solver was pulled
        # hard toward the far branch: measured, the boat drove its yaw the wrong
        # way at full differential ([-20, +20] N) while its heading error was
        # only -0.785 rad and shrinking.
        #
        # np.unwrap over the whole array is the accumulating version.
        psi_seq = np.unwrap(
            [(get_yaw_from_quaternion(q.pose.orientation) + np.pi) % (2 * np.pi) - np.pi
             for q in poses])

        # Now put that (internally continuous) sequence on the branch nearest the
        # MEASURED heading, which arrives wrapped and independently from odometry.
        # The shift is rigid, so every difference along the horizon -- and hence
        # the r references derived below -- is untouched.
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
                dx_b =  math.cos(psi_mid) * dx + math.sin(psi_mid) * dy
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

        # RECOVERY. SQP_RTI takes ONE Newton step per call from the previous
        # iterate, and nothing here re-seeds stages 1..N -- so a single bad
        # step poisons every later solve and the controller never comes back.
        # Measured in Gazebo 2026-09-04: the boat was tracking the circle at
        # 0.05 m and 0.10 rad of error, one tick commanded [-20, -20], and from
        # the next tick on the QP failed for the remaining 93.7 % of the run.
        # Raising the working-set budget does not touch this: it is a poisoned
        # linearisation point, not an iteration cap.
        #
        # reset(reset_qp_solver_mem=1) zeroes the iterate AND the active-set
        # memory; re-seeding every stage at the measured state with zero input
        # is dynamically near-consistent (the boat coasts) and satisfies every
        # bound by construction, so the retry starts from somewhere sane rather
        # than from wherever the failure left it. This is failure handling, not
        # a control law: on the success path nothing here runs.
        if status != 0:
            self.recoveries += 1
            self.solver.reset()
            self._seed_all_stages(x_current)
            self._apply_reference(x_refs, x_current)
            status = self.solver.solve()
            self.last_solve_time = self._stat('time_tot') or 0.0

        self.last_status = status

        # A FAILED SOLVE IS NOT A COMMAND (C6).
        #
        # acados leaves the primal iterate untouched on a non-zero status, so
        # self.solver.get(0, 'u') would hand back the command from the last
        # SUCCESSFUL solve. Returning it published a constant asymmetric
        # thruster pair -- which is, physically, a constant-radius circle -- and
        # the latch was self-sustaining: once the boat is circling, its heading
        # error never gets small again, so the solve never recovers. Measured
        # 2026-09-04: 654 of 656 ticks carrying one bit-identical command for
        # 32.7 s while the state changed continuously underneath it.
        #
        # Zero is the only value that is safe without knowing the caller's
        # history. The caller decides what to do about it -- master_control
        # publishes zero thrust and logs; see last_status / fail_count.
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