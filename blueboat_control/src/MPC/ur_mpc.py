# Remove syntax warnings from acados
import warnings
warnings.filterwarnings("ignore", category=SyntaxWarning)

# Regular imports
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import numpy as np
import math

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
        input_bounds=None):

        self.mass = robot_mass
        self.iz = iz
        self.N = horizon
        self.T = time
        self.dt = time / horizon

        self.Q = Q_weight
        self.R = R_weight
        self.input_bounds = input_bounds 

        self.model = export_underwater_model(self.mass, self.iz, a_u, a_v, a_r, d_u, d_v, d_r)
        self.ocp = self._build_ocp()
        self.solver = AcadosOcpSolver(self.ocp, json_file='acados_ocp.json')
    
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

        # Rebuild OCP and solver
        self.ocp = self._build_ocp()
        self.solver = AcadosOcpSolver(self.ocp, json_file='acados_ocp.json')

    def solve(self, path, x_current):
        poses = path.poses[:self.N + 1]
        if len(poses) < self.N + 1:
            poses += [poses[-1]] * (self.N + 1 - len(poses))

        x_refs, u_refs = [], []
        
        for i in range(self.N + 1):
            pose = poses[i].pose
            x = pose.position.x
            y = pose.position.y
            psi = get_yaw_from_quaternion(pose.orientation)
            psi = (psi + np.pi) % (2 * np.pi) - np.pi

            if i > 0:
                prev_pose = poses[i - 1].pose
                dx = x - prev_pose.position.x
                dy = y - prev_pose.position.y
                u = math.hypot(dx, dy) / self.dt

                psi_prev = get_yaw_from_quaternion(prev_pose.orientation)
                psi = np.unwrap([psi_prev,psi])[-1]

                psi_mid = (psi + psi_prev) / 2.0
                dx_b =  math.cos(psi_mid) * dx + math.sin(psi_mid) * dy
                dy_b = -math.sin(psi_mid) * dx + math.cos(psi_mid) * dy
                v = dy_b / self.dt 

                dpsi = (psi - psi_prev + np.pi) % (2 * np.pi) - np.pi
                r = dpsi / self.dt
            else:
                u = 0.0
                v = 0.0
                r = 0.0

            x_refs.append([x, y, psi, u, v, r])
            # if i < self.N:
            #     u_refs.append([0.0, 0.0])

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