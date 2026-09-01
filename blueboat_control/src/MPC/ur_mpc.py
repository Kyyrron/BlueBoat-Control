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
        build_dir=None):

        self.mass = robot_mass
        self.iz = iz
        self.N = horizon
        self.T = time
        self.dt = time / horizon

        self.Q = Q_weight
        self.R = R_weight
        self.input_bounds = input_bounds 

        # Where the generated C and the compiled solver live. None resolves to
        # $ROS_HOME/blueboat_control/mpc (see acados_build_dir); the argument is
        # a plain Python keyword, NOT a declared ROS parameter, so the node's
        # interface surface is unchanged (N1 / CM-1).
        self.build_dir = acados_build_dir(build_dir)

        self.model = export_underwater_model(self.mass, self.iz, a_u, a_v, a_r, d_u, d_v, d_r)
        self.ocp = self._build_ocp()
        self.solver, self.reused_solver = build_solver(self.ocp, self.build_dir)
    
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
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
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

        self.solver.set(0, 'x', x_current)
        self.solver.set(0, 'lbx', x_current)
        self.solver.set(0, 'ubx', x_current)

        x_refs = np.array(x_refs)  # shape (N+1, nx)

        u_refs = np.zeros((self.N, 2))  # N x nu

        for i in range(self.N):
            yref = np.concatenate((x_refs[i], u_refs[i]))
            self.solver.set(i, 'yref', yref)
        self.solver.set(self.N, 'yref', np.array(x_refs[-1]))

        status = self.solver.solve()
        if status != 0:
            print(f"ACADOS solver failed with status {status}")

        U = np.array([self.solver.get(i, 'u') for i in range(self.N)])
        return U[0]