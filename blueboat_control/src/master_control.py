#!/usr/bin/env python3

# ============================================================================
# PATH-FOLLOWING REWORK.
#
# The reference used to be played on a WALL CLOCK:
#   request.path_request.data = linspace(time.time()-t0, ..., steps)
# so the desired pose advanced with real time regardless of where the boat
# actually was. Combined with a 1 Hz control loop (self.dt = 1.0), the boat
# received a target that ran away along the path and updated only once per
# second, producing smooth path-blind arcs with no resemblance to the path.
#
# This version:
#   * runs the control loop at 20 Hz (self.dt = 0.05);
#   * advances a PATH PARAMETER tau with a GOVERNOR that moves the virtual
#     target at the path's authored speed when the boat keeps up, and slows
#     or pauses tau when the boat falls behind, so the reference can never
#     outrun the boat. The authored speed can vary along the path (it is the
#     spatial rate of the parameterization), so a spatially varying speed
#     profile is followed for free. A global self.path_speed_scale scales it.
#   * uses canonical Fossen lookahead LoS for the 'LoS' controller type and
#     adds path-speed feedforward to the 'PID' controller.
#
# INTERFACES ARE UNCHANGED: same node name/namespace, same topics, same
# /path_request service (an array of parameter values in, a Path out -- so
# path_generation.py needs no change), same message types, same
# controller_type options, same monitoring format, same pinger and manual
# behavior. Only the internals of how the reference is generated and how LoS
# is computed have changed.
#
# Retains the world-frame monitoring target fix ("# --- world-frame
# monitoring target ---").
# ============================================================================

### FOR MANUAL TARGET IMPLEMENTATION IN THE VISUALISATION APP ###

# ----------------------------------------------------------------------------
# FILE MAP (class Controller) -- sections are banner-commented below.
#
#   1. WIRING                     __init__
#   2. TUNING KNOBS               _declare_tuning_parameters   <-- gains live here
#   3. THE CONTROL LOOP           timer_callback               <-- start reading here
#   4. GUIDANCE                   warn_if_multiple_path_servers, accept_path,
#                                 path_progress_errors, advance_governor,
#                                 los_guidance, solve_LoS, manual_keep_location
#   5. CALLBACKS / HELPERS        odom_, pinger_, ready_, manual_target_,
#                                 publish_thrust, save_monitoring, get_time
#
# Moved out of this file:
#   inRobotFrame()  ->  _custom_libraries/frame_math.py   (pure geometry, ROS-free)
# ----------------------------------------------------------------------------

# rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSDurabilityPolicy
import rclpy

# Common python libraries
import contextlib
import os
import sys
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from datetime import datetime

# ROS2 msg libraries
from std_msgs.msg import String, Bool, Float32, Float32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker

# Custom libraries
from urdf_parser_py import urdf
import ur_mpc
import PID
from blueboat_control import ROV
from blueboat_interfaces.srv import RequestPath
import custom_functions as cf
import frame_math as fm         # pure world<->body geometry (ROS-free)
import path_stamp as ps         # /path_request response tag (ROS-free)
import thrust_limits as tl     # uniform thrust saturation (ROS-free)


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class Controller(Node):

    # ======================================================================
    #  1. WIRING
    #  topics, service, timer. Read this to see what the node talks to.
    # ======================================================================

    def __init__(self):

        super().__init__('master_control', namespace='blueboat')


        self.declare_parameter('controller_type', 'MPC') 
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value

        self.declare_parameter('simulation', True) 
        self.isSimulation = self.get_parameter('simulation').get_parameter_value().bool_value

        self.declare_parameter('use_pinger', False) 
        self.use_pinger = self.get_parameter('use_pinger').get_parameter_value().bool_value

        # Empty means "resolve it" - see custom_functions.data_root for the order.
        self.declare_parameter('data_dir', '')
        self.data_root = cf.data_root(
            self.get_parameter('data_dir').get_parameter_value().string_value)

        # Every tuning constant, declared with today's value as its default.
        self._declare_tuning_parameters()

        self.odom_subscriber = self.create_subscription(Odometry, '/blueboat/odom', self.odom_callback, 10)
        self.pinger_subscriber = self.create_subscription(Float32MultiArray, '/blueboat/pinger_coordinates', self.pinger_callback, 10)
        self.ready_subscriber = self.create_subscription(Bool, '/blueboat/controller_ready', self.ready_callback, 10)
   
        self.manual_target_subscriber = self.create_subscription(Float32MultiArray, '/blueboat/manual_target', self.manual_target_callback, 10)

        self.data_publisher = self.create_publisher(Float32MultiArray, "/monitoring_data", 10)
        self.target_publisher = self.create_publisher(Float32MultiArray,'/controller_target', 10)
        self.thruster_input_publisher = self.create_publisher(Float32MultiArray, "/thruster_input", 10)
        self.pose_arrow_publisher = self.create_publisher(Marker, "/pose_arrow", 10)

        # Create a client for path request
        if not self.use_pinger:
            self.client = self.create_client(RequestPath, '/path_request')

            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service...")

            self.warn_if_multiple_path_servers()

        self.future = None # Used for client requests

        self.time_set = False
        self.initial_time = None
        # self.dt is set by _declare_tuning_parameters (20 Hz default, was 1.0 Hz)
        self.timer = self.create_timer(self.dt, self.timer_callback)
        # The controller log is written here, off the control tick, and a
        # final write happens on shutdown so nothing recorded is lost.
        self.log_timer = self.create_timer(2.0, self.save_monitoring)

        self.current_pose = None
        self.current_twist = None

        self.ready = False
        self.pinger_target = None
        self.manual_target = [0.0,0.0]

        # Initialize controller 
        self.controller_path = Path()

        # /path_request response validation. The response is a bare
        # nav_msgs/Path with nothing saying which request it answers, so these
        # carry the question the answer has to match.
        self.tau_requested = None       # parameter the in-flight request asked for
        self.request_sent = None        # when it went out, for the timeout
        self.path_rx_time = None        # when the held window last arrived
        self.path_stamps_are_parameters = None   # None = not yet established
        self.foreign_path_count = 0

        # ---- Path-parameter governor state ----------------------------------
        # tau is the path parameter of the virtual target (same units as the
        # path_generation time argument). It advances by the governor, NOT by
        # the wall clock.
        self.tau = 0.0
        # path_speed_scale / gov_Lmin / gov_Lmax are declared parameters, set by
        # _declare_tuning_parameters.
        # ---------------------------------------------------------------------

        # MPC Parameters
        if self.controller_type == 'MPC':
            # mpc_horizon, mpc_time, Q_weight, R_weight and input_bounds are
            # declared parameters, set by _declare_tuning_parameters.

            # Initialize MPC solver
            self.controller = None # Updated at the start of spin

            # Derived from the horizon, never declared separately: the reference
            # window and the solver's horizon must not be able to disagree.
            #
            # N + 1, not N (TODO.md F4). MPCController.solve reads N+1 poses --
            # N stages plus the terminal one -- so requesting N made it pad by
            # duplicating the last pose, giving a zero-velocity terminal
            # reference. It also desynchronised the two spacings: this window's
            # dtau was path_time/(path_steps-1) while the solver divides by
            # time/horizon, inflating every reference velocity by 7.1 %. With
            # N+1 poses the two are identically time/horizon.
            self.path_time = self.mpc_time
            self.path_steps = self.mpc_horizon + 1

        # PID Parameters
        if self.controller_type == 'PID':
            self.path_time = self.dt
            self.path_steps = 2

            # outer_gains, inner_gains, pid_lookahead and thruster_limits are
            # declared parameters, set by _declare_tuning_parameters.

            radius = 0.59/2
            self.B_matrix = B = np.array([[1.        ,1.],
                                          [0.        ,0.],
                                          [radius,-radius]])

        # LoS Parameters
        if self.controller_type ==  'LoS':
            self.path_time = self.dt
            self.path_steps = 2

            radius = 0.59/2
            self.B_matrix = np.array([[1.        ,1.],
                                      [0.        ,0.],
                                      [radius,-radius]])
            # The kinematic Fossen-LoS gains (los_lookahead, los_ku, los_kpsi,
            # los_kd, los_speed_scale) and thruster_limits are declared
            # parameters, set by _declare_tuning_parameters.
            self.los_allocator = PID.ThrustAllocator(self.B_matrix, limits=self.thruster_limits)

        # k_v, k_psi (point following) and safety_distance are declared
        # parameters, set by _declare_tuning_parameters. Their defaults still
        # differ between simulation and the real boat.
        self.stopping_sequence = False # Used as a safety to stop LoS control when it gets close to target
        self.stopping_time = None

        # Manual-target keep-location state. Deliberately SEPARATE from
        # stopping_sequence: that flag stays the pinger branch's, so the two
        # point-following modes can no longer mute one another.
        #   manual_hold        - True while holding station on a reached target
        #   manual_brake_t0    - when the arrival reverse pulse started
        #   manual_hold_target - the target the state above refers to, so a
        #                        repeat of the same target does not re-arm the
        #                        pulse while a genuinely new one does
        self.manual_hold = False
        self.manual_brake_t0 = None
        self.manual_hold_target = None

        # Initialize monitoring values
        # Monitoring rows. The header goes in as strings, which is what makes
        # np.save coerce the whole array to <U32 on write -- unchanged, because
        # every existing log and replay.py read that schema (CM-7). What did
        # change is WHEN: see save_monitoring().
        self.monitoring = []
        self.monitoring.append(['t','x','y','psi','x_d','y_d','psi_d','u1','u2'])
        self.monitoring_saved_rows = 0

        self.t_record = self.get_time()

        ctrl = self.controller_type
        date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        sim = 'simulation' if self.isSimulation else 'real'
        run_dir = cf.ensure_data_dir(self, self.data_root, 'data', f'{ctrl}_data')
        self.title = cf.reserve_run_file(run_dir, f'{date}-{ctrl}_{sim}_data', '.npy')
        self.get_logger().info(f"Controller log: {self.title}.npy")

        # Build the controller HERE, not on the first tick. The MPC branch runs
        # acados code generation and a C compile, which takes tens of seconds
        # the first time; done from timer_callback it froze the whole node --
        # the executor is single-threaded, so odom, /path_request futures and
        # publish_thrust all stopped with it, and the interface-side watchdog
        # then zeroed the thrust. Doing it before rclpy.spin means the cost is
        # paid once, visibly, at launch, and a failure kills the node where the
        # operator can see it instead of hanging the control loop.
        self._build_controller()

    # ------------------------------------------------------------------ #
    #  Controller construction                                           #
    # ------------------------------------------------------------------ #
    @staticmethod
    @contextlib.contextmanager
    def _no_stdin():
        """Run a block with stdin closed, so a prompt raises instead of blocking.

        acados_template's get_tera() asks `input("... download tera? y/N")` when
        the Tera renderer binary is missing. Under `ros2 launch` stdout is block
        buffered and stdin never delivers a line, so the question is invisible
        and the read never returns -- the node just stops, forever. With stdin
        pointed at /dev/null the same call raises EOFError immediately and we
        get to report something useful.
        """
        saved = sys.stdin
        try:
            with open(os.devnull, 'r') as devnull:
                sys.stdin = devnull
                yield
        finally:
            sys.stdin = saved

    def _check_acados_ready(self):
        """FATAL + raise if the acados environment cannot build a solver."""
        try:
            from acados_template.utils import get_tera_exec_path, get_acados_path
        except Exception as e:
            self.get_logger().fatal(
                f"controller_type 'MPC' needs acados_template, which failed to "
                f"import: {e}. Source the venv that carries it "
                f"(source ~/ros2_ws/.venv/bin/activate) before launching.")
            raise

        tera = get_tera_exec_path()
        if not (os.path.exists(tera) and os.access(tera, os.X_OK)):
            self.get_logger().fatal(
                "acados cannot generate the MPC solver: the Tera template "
                f"renderer is missing at\n    {tera}\n"
                "Without it acados stops mid-generation on an interactive "
                "prompt. Install it with:\n"
                f"    mkdir -p {os.path.dirname(tera)}\n"
                f"    curl -fL -o {tera} https://github.com/acados/tera_renderer"
                "/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64\n"
                f"    chmod +x {tera}\n"
                "and export ACADOS_SOURCE_DIR="
                f"{get_acados_path()} so this path stops being a guess.")
            raise RuntimeError(f"acados Tera renderer not found at {tera}")

    def _build_controller(self):
        """Construct the controller named by controller_type.

        PID and LoS build a plain Python object in microseconds. MPC runs acados
        code generation and a C compile, so it gets a preflight check and an
        explicit progress line.
        """
        if self.controller_type == 'MPC':
            self._check_acados_ready()
            build_dir = ur_mpc.acados_build_dir()
            self.get_logger().info(
                f"Building the acados MPC solver in {build_dir} -- the first "
                "run compiles C and can take about a minute; later runs reuse it.")
            try:
                with self._no_stdin():
                    self.controller = ur_mpc.MPCController(**self.mpc_model,
                                                horizon = self.mpc_horizon,
                                                time = self.mpc_time,
                                                Q_weight = self.Q_weight,
                                                R_weight = self.R_weight,
                                                input_bounds = self.input_bounds,
                                                build_dir = build_dir,
                                                qp_solver_iter_max = self.mpc_qp_iter_max,
                                                logger = self.get_logger().warning
                                                )
            except Exception as e:
                self.get_logger().fatal(
                    f"acados failed to build the MPC solver in {build_dir}: "
                    f"{type(e).__name__}: {e}. Delete that directory and retry; "
                    "if it persists, check ACADOS_SOURCE_DIR and that the acados "
                    "C library is built.")
                raise
            self.get_logger().info(
                "acados MPC solver "
                f"{'reused' if self.controller.reused_solver else 'generated and compiled'}.")

        if self.controller_type == 'PID':
            self.controller = PID.PIDLoS(dt = self.dt,
                                         B = self.B_matrix,
                                         outer_gains = self.outer_gains,
                                         inner_gains = self.inner_gains,
                                         lookahead = self.pid_lookahead,
                                         thruster_limits = self.thruster_limits
                                         )

        self.get_logger().info('Controller node initiated')

    # ======================================================================
    #  2. TUNING KNOBS -- every gain, all in one place
    #  Change a value here, or override it with a launch argument.
    # ======================================================================

    # ------------------------------------------------------------------ #
    #  Tuning parameters                                                 #
    # ------------------------------------------------------------------ #
    def _declare_tuning_parameters(self):
        """
        Declare every tuning constant of the control stack, with today's value
        as its default, so a gain change costs a launch argument rather than an
        edit and a rebuild.

        Declared unconditionally - independent of controller_type - so that
        `ros2 param list` shows the whole set whatever controller is running.

        ROS 2 has no dict or tuple parameter type, so the composite constants
        (gain triples, MPC weight diagonals) are declared as double arrays and
        reassembled here. Every name is new: nothing already on the wire is
        renamed or retyped (N1). Defaults live in this node rather than in a
        launch file, so simulation and the real boat get them identically (N2).

        Values are read once, at construction. Changing one takes effect at the
        next launch, not mid-run.
        """
        def dbl(name, default):
            self.declare_parameter(name, float(default))
            return self.get_parameter(name).get_parameter_value().double_value

        def integer(name, default):
            self.declare_parameter(name, int(default))
            return self.get_parameter(name).get_parameter_value().integer_value

        def arr(name, default):
            self.declare_parameter(name, [float(v) for v in default])
            value = list(self.get_parameter(name).get_parameter_value().double_array_value)
            if len(value) != len(default):
                raise ValueError(f"parameter '{name}' expects {len(default)} "
                                 f"values, got {len(value)}")
            return value

        # -- control loop --------------------------------------------------
        # Load-bearing beyond the gains: the governor rescales with it, and the
        # MPC solve has to finish inside it.
        self.dt = dbl('control_dt', 0.05)

        # -- path-parameter governor ---------------------------------------
        self.path_speed_scale = dbl('path_speed_scale', 1.0)
        self.gov_Lmin = dbl('gov_Lmin', 0.5)
        self.gov_Lmax = dbl('gov_Lmax', 3.0)
        # Cross-track half of the governor: same shape and units as the
        # along-track pair. gov_Emax = 0 disables it, which is the default.
        #
        # Off by default because the cross-track term is only safe once the
        # inner loops can actually close a lateral gap. At the shipped gains
        # they cannot, and throttling the target on an error the controller
        # cannot reduce is positive feedback: the target stalls, the boat
        # loses the forward authority it converges with, and the offset grows.
        # Raise the inner gains first, then set gov_Emax (5.0 is a reasonable
        # starting point) - see TODO.md.
        self.gov_Emin = dbl('gov_Emin', 0.5)
        self.gov_Emax = dbl('gov_Emax', 0.0)

        # -- PID ------------------------------------------------------------
        if not self.isSimulation:
            self.outer_gains = {'x':   tuple(arr('outer_gains_x',   [3.0, 0.01, 0.0])),
                                'psi': tuple(arr('outer_gains_psi', [3.0, 0.01, 0.0]))}
            self.inner_gains = {'u': tuple(arr('inner_gains_u', [1.0, 0.0, 0.0])),
                                'r': tuple(arr('inner_gains_r', [1.5, 0.0, 0.0]))}
            self.pid_lookahead = dbl('pid_lookahead', 2.5)
        else:
            self.outer_gains = {'x':   tuple(arr('outer_gains_x',   [6.0, 0.01, 0.0])),
                                'psi': tuple(arr('outer_gains_psi', [4.0, 0.01, 0.0]))}
            self.inner_gains = {'u': tuple(arr('inner_gains_u', [2.0, 0.0, 0.0])),
                                'r': tuple(arr('inner_gains_r', [2.5, 0.0, 0.0]))}
            self.pid_lookahead = dbl('pid_lookahead', 2.5)
            
        # -- kinematic Fossen-LoS -------------------------------------------
        self.los_lookahead   = dbl('los_lookahead', 2.5)
        self.los_ku          = dbl('los_ku', 20.0)
        self.los_kpsi        = dbl('los_kpsi', 10.0)
        self.los_kd          = dbl('los_kd', 1.0)
        self.los_speed_scale = dbl('los_speed_scale', 1.0 if not self.isSimulation else 2.0)
        # Zero-authored-speed hold. A stationary reference (station_keeping, a
        # clamped-out mission, the awaiting-YAML fallback) gives U_d = 0, and
        # neither controller can hold position on one: LoS commands zero surge,
        # and PID's along-track term cannot see a cross-track error because the
        # path tangent it projects onto is meaningless. Below hold_speed both
        # blend to steering at the reference point instead.
        #
        # hold_speed is the gate and hold_radius is "on station"; both are shared
        # with the PID branch, which has the same problem for the same reason.
        # The gate is what keeps this inert on a real path: every authored
        # trajectory runs at >= 0.28 m/s, so the blend weight is exactly zero and
        # both laws are unchanged. Raising it above the slowest authored speed
        # would start altering path following.
        self.hold_speed      = dbl('hold_speed', 0.05)
        self.hold_radius     = dbl('hold_radius', 0.5)
        self.los_hold_kx     = dbl('los_hold_kx', 1.0)
        self.los_hold_umax   = dbl('los_hold_umax', 0.8)

        # -- point following (manual target and pinger) ----------------------
        # Simulation and the real boat have always used different values here.
        self.k_v   = dbl('point_k_v',   2.0  if self.isSimulation else 0.15)
        self.k_psi = dbl('point_k_psi', 60.0 if self.isSimulation else 10.0)
        # Negative disables the arrival check. Since the manual-target branch
        # got its own keep-location hold below, this governs the PINGER branch
        # only, which is why its defaults are untouched.
        self.safety_distance = dbl('safety_distance', 1.0 if self.isSimulation else -1.0)

        # -- manual-target keep-location hold --------------------------------
        # A reached manual target is a station to hold, not a place to stop.
        # The old behaviour was a one-shot latch: arrive, reverse for a second,
        # then command zero for the rest of the run. In any current that is a
        # boat sitting at zero thrust being pushed out of the survey area, and
        # on the real boat the latch never even armed (safety_distance = -1.0),
        # so the point law hunted around the target instead of settling.
        #
        # This replaces the latch with a state that is re-evaluated every tick:
        #
        #   d <= manual_hold_radius      -> on station, hold
        #   d >  manual_reacquire_radius -> blown off station, resume pursuit
        #
        # and inside the hold the surge is proportional to the range outside
        # manual_hold_radius rather than zero, so the law never stops answering
        # a disturbance. It is the same shape as los_guidance's zero-authored-
        # speed hold (steer at the point, surge proportional to the gap, capped,
        # never reverse, cos-shaped) - but the gains cannot be shared with it.
        # los_hold_kx / los_hold_umax are VELOCITIES fed through los_ku and the
        # allocator; solve_LoS writes its surge straight onto the wire as if it
        # were Newtons, so the two live in different units.
        #
        # The gains are set so the hold meets the pursuit law at the handover
        # rather than stepping there. At d = manual_reacquire_radius the pursuit
        # surge is 8.38 (real) and 15.42 (simulation), so a gain of 8.0 / 15.0
        # per metre of gap arrives within half a Newton:
        #
        #     d      pursuit real   hold real   pursuit sim   hold sim
        #     1.00        5.30         0.00        13.10         0.00
        #     1.25        6.20         2.00        13.88         3.75
        #     1.50        7.00         4.00        14.50         7.50
        #     2.00        8.38         8.00        15.42        15.00
        #
        # The 2.00 N row is also the ESC breakaway: below it the propellers do
        # not turn, so the effective station-keeping box is manual_hold_radius
        # plus about 0.25 m on the real boat. That is deliberate - flooring the
        # hold surge the way the pursuit law is floored would push the boat
        # around inside its own deadband instead of letting it sit.
        # manual_hold_radius <= 0.0 disables the hold entirely and restores the
        # pre-2026-09-03 behaviour of the manual branch exactly.
        self.manual_hold_radius      = dbl('manual_hold_radius', 1.0)
        self.manual_reacquire_radius = dbl('manual_reacquire_radius', 2.0)
        self.manual_hold_kx   = dbl('manual_hold_kx', 15.0 if self.isSimulation else 8.0)
        # Derived by default rather than declared independently, so retuning a
        # radius cannot silently break the handover the gains were chosen for.
        self.manual_hold_umax = dbl(
            'manual_hold_umax',
            self.manual_hold_kx * max(0.0, self.manual_reacquire_radius
                                           - self.manual_hold_radius))
        # Seconds of reverse on each fresh arrival, to kill the way on.
        self.manual_brake_time = dbl('manual_brake_time', 1.0)

        # -- MPC -------------------------------------------------------------
        # Simulation and the real boat get different values here, for the same
        # reason the PID gains and the point-following gains above do -- except
        # that for the MPC the split covers the plant MODEL as well as the
        # weights, because an optimiser's behaviour is set by what it believes
        # about the boat, not only by what it is told to care about.
        #
        # THE MODEL. The real-boat column is unchanged: those coefficients have
        # been in the tree since the beginning and CONTROLLERS.md section 6.4 records
        # that their provenance is unknown, so nothing here claims to improve
        # them. The simulation column is FITTED TO THE GAZEBO PLANT, i.e. to
        # blueboat_description/urdf/hydrodynamics.xacro, which is what the boat
        # in simulation actually obeys. The two disagreed badly -- surge mass
        # 2.0x heavy, yaw inertia 4.8x heavy (27.41 vs 5.76 kg m^2), yaw drag
        # 2-4x over -- so the MPC planned trajectories the sim boat could not
        # fly, discovered the deficit, and spent every newton it had. Measured
        # symptom: 77.5 % of ticks on the +/-20 N bound at a 0.70 m/s mission,
        # with the effort going into yaw (|differential| 8.54 N against 5.44 N
        # of surge).
        #
        # Added mass maps one-for-one: acados builds M = M_rb - diag(a_*) and
        # the xacro states xDotU / yDotV / nDotR with the same sign convention,
        # so a_u/a_v/a_r ARE the xacro values.
        #
        # Damping does not map one-for-one. export_underwater_model carries a
        # linear D only, while the plant is linear + quadratic, and the
        # quadratic term dominates above ~0.4 m/s. Each d_* below is therefore a
        # SECANT linearisation -- the linear coefficient whose steady-state drag
        # equals the plant's at a representative operating point:
        #
        #   d_u = 25.15 + 33.800 * 0.45 m/s    -> exact at 0.45 m/s, and within
        #                                         +14 % / -17 % over 0.30-0.70
        #   d_v =  7.364 + 54.269 * 0.10 m/s   -> sway stays small in normal use
        #   d_r =  3.744 + 40.000 * 0.15 rad/s -> 0.15 is the p75 of |yaw rate|
        #                                         measured across every recorded
        #                                         sim run (median 0.000, p90 0.407)
        #
        # These are FITTED NUMBERS, not physics: refit them if hydrodynamics.xacro
        # changes, or if missions settle at a cruise speed far from 0.45 m/s.
        # docs/controllers/mpc_tuning_report.py scores a run for the symptoms.
        if not self.isSimulation:
            self.mpc_model = {'robot_mass': 16.01,   # blueboat.xacro mass
                              'iz':          5.64,   # blueboat.xacro izz
                              'a_u': -26.77, 'a_v':  -7.55, 'a_r': -21.77,
                              'd_u': -29.34, 'd_v': -51.54, 'd_r': -44.65}
        else:
            self.mpc_model = {'robot_mass': 16.01,   # blueboat.xacro mass
                              'iz':          5.64,   # blueboat.xacro izz
                              'a_u':  -5.50,         # = xDotU
                              'a_v': -12.70,         # = yDotV
                              'a_r':  -0.12,         # = nDotR
                              'd_u': -40.36,         # secant of xU + xUabsU*|u|
                              'd_v': -12.79,         # secant of yV + yVabsV*|v|
                              'd_r':  -9.74}         # secant of nR + nRabsR*|r|

        # THE HORIZON. 6.0 s / 30 steps in simulation is the one MPC change in
        # this repository with a measurement behind it (CONTROLLERS.md section 5.2):
        # the circle's steady radial offset goes -1.019 m -> -0.011 m and cruise
        # speed 26 % fast -> exact. The real boat stays at 2.5 s / 15 until the
        # solve time is measured on the companion computer -- doubling the
        # horizon is precisely the change that would break the 50 ms budget, and
        # TODO.md section 2 records that it has never been timed on target hardware.
        # Both keep dt = time/horizon at 0.167-0.200 s.
        self.mpc_horizon = integer('mpc_horizon', 30 if self.isSimulation else 15)
        self.mpc_time    = dbl('mpc_time', 6.0 if self.isSimulation else 2.5)

        self.Q_weight = np.diag(arr('mpc_Q_diag', [50.0,   # x
                                                   50.0,   # y
                                                   30.0,   # psi
                                                    1.0,   # u
                                                    1.0,   # v
                                                    1.0])) # r
        # THE EFFORT PENALTY. The stage cost is (x-x_ref)' Q (x-x_ref) + u' R u
        # with u_ref identically zero, so R is an absolute penalty in N^2 -- there
        # is no rate term anywhere. At Q_pos = 50 the break-even is where
        # saturating BOTH thrusters costs what the position error costs:
        #
        #   R = 0.015 -> 0.49 m      R = 0.10 -> 1.26 m      R = 0.25 -> 2.00 m
        #
        # At the shipped 0.015 anything beyond half a metre of error makes full
        # throttle literally the cheaper option, which is why the MPC saturates
        # where PID -- whose demand its own P-gain bounds -- does not. 0.10 in
        # simulation puts the break-even outside the governor's own 0.5 m
        # dead-band; CONTROLLERS.md section 10 advises 0.05-0.1 on general grounds.
        self.R_weight = np.diag(arr('mpc_R_diag', [0.10, 0.10] if self.isSimulation
                                                  else [0.015, 0.015]))

        # THE QP WORKING-SET BUDGET. FULL_CONDENSING_QPOASES is a dense
        # ACTIVE-SET solver, so the condensed QP carries nv = mpc_horizon * 2
        # variables -- 60 in simulation since the horizon doubled on
        # 2026-09-01, 30 on the real boat -- and reaching a vertex where most
        # of those bounds are active costs about one working-set change each.
        # acados defaults the budget to 50, i.e. BELOW nv at horizon 30, so a
        # saturating solve failed by construction and acados returned status 4
        # with the primal iterate untouched. That stale iterate was then
        # published as a command: a constant asymmetric thruster pair, which is
        # a constant-radius circle. CONTROLLERS.md C6 carries the forensics.
        #
        # 0 (the default) means "derive it from the horizon" in
        # ur_mpc.MPCController, so the arithmetic lives in exactly one place:
        # max(50, 4 * nu * N) = 240 at N = 30, 120 at N = 15. Set a positive
        # value to pin it.
        #
        # This is a solver option, so it is inside the hash acados compares for
        # code reuse: the first launch after changing it regenerates and
        # rebuilds once (about a minute) and says so in the log. No cache
        # directory needs deleting.
        self.mpc_qp_iter_max = integer('mpc_qp_iter_max', 0)

        # -- thrust limits, shared by every branch ---------------------------
        # One symmetric scalar feeds both the allocator clamp and the MPC input
        # bounds, so the two cannot drift apart.
        self.thrust_limit = dbl('thrust_limit', 20.0)
        limit = self.thrust_limit
        self.thruster_limits = {"min": np.array([-limit, -limit]),
                                "max": np.array([ limit,  limit])}
        self.input_bounds = {"lower": np.array([-limit, -limit]),
                             "upper": np.array([ limit,  limit]),
                             "idx":   np.array([0, 1])}

        # -- propeller breakaway ---------------------------------------------
        # The thrust->PWM table maps 0..2 N onto PWM 1500..1525, which sits
        # inside a T200 ESC's neutral deadband: commands in that band leave the
        # propellers stationary, so the boat holds still while the log records
        # a perfectly sensible small force. Only ONE law is measurably affected
        # -- solve_LoS following a pinger on the real boat, where the surge
        # command 5*ln(k_v*d + 1) stays under 2 N out to d = 3.28 m. Every
        # other law is inside 1.4 m and needs no floor; see CLAUDE.md section 5.
        # 0.0 disables the floor and restores the pre-2026-08-31 law exactly.
        self.min_thrust = dbl('min_thrust', 2.0)

        # -- /path_request health --------------------------------------------
        # path_request_timeout: give up on a pending request and re-issue.
        #   Without it a single lost response wedges the node permanently.
        # path_stale_timeout: beyond this the held window is not a reference any
        #   more, so the governor stops advancing tau against it. Both are
        #   generous multiples of the 20 Hz tick - this is a wedge detector, not
        #   a jitter detector.
        self.path_request_timeout = dbl('path_request_timeout', 1.0)
        self.path_stale_timeout   = dbl('path_stale_timeout', 1.0)

    # ======================================================================
    #  3. THE CONTROL LOOP
    #  Runs at 1/self.dt (20 Hz). This is the entry point.
    # ======================================================================

    def timer_callback(self):
        # Every early return below publishes zero thrust rather than falling
        # silent: a consumer that hears nothing keeps streaming the last value
        # it did hear. The interface-side watchdog is the outer guard for the
        # case this cannot cover - this node crashing or hanging.
        if not self.ready:
            self.publish_thrust([0.0, 0.0])
            return

        # The controller itself is built in __init__ (see _build_controller):
        # acados code generation is a build step, and running it from here froze
        # the single-threaded executor for the whole compile.

        if not self.time_set:
            self.initial_time = time.time()
            self.tau = 0.0
            self.time_set = True
        
        current_time = time.time() - self.initial_time

        ## Boat state (needed by the governor, so compute it up front)
        if self.current_pose is None or self.current_twist is None:
            self.publish_thrust([0.0, 0.0])
            return

        current_state = np.array([self.current_pose[0], # x
                                self.current_pose[1], # y
                                self.current_pose[5], # yaw
                                self.current_twist[0], # u (body surge)
                                self.current_twist[1], # v (body sway)
                                self.current_twist[5]]) # r
        current_state = np.array(current_state).reshape(-1)

        manual_active = (list(self.manual_target) != [0.0, 0.0])

        ## Update path (parameter-governed, NOT wall-clock)
        if not self.use_pinger:
            # Collect a completed request
            if self.future is not None and self.future.done():
                try:
                    result = self.future.result()
                    if result is None:
                        self.get_logger().error("Service returned None.")
                    elif self.accept_path(result.path):
                        self.controller_path = result.path
                        self.path_rx_time = current_time
                except Exception as e:
                    self.get_logger().error(f"Service call raised exception: {e}")
                finally:
                    self.future = None
                    self.request_sent = None

            elif (self.future is not None and self.request_sent is not None
                    and current_time - self.request_sent > self.path_request_timeout):
                # A response that never arrives must not wedge the node. Without
                # this the future stays pending forever, no further request is
                # ever issued, and the reference window is frozen for the rest
                # of the run with nothing reported. path_publisher has always
                # had this guard; this node did not.
                self.get_logger().warning(
                    f"No answer to /path_request within {self.path_request_timeout:.1f} s "
                    "- retrying.", throttle_duration_sec=5.0)
                self.future = None
                self.request_sent = None

            # Advance the governor using the boat's progress along the current
            # window (frozen while a manual target overrides path following).
            #
            # Gated on the window being FRESH. Advancing against a stale window
            # is open loop: once the boat reaches that frozen target e_along
            # falls below gov_Lmin, the along-track factor unclips to 1.0, and
            # tau integrates at full rate with no feedback at all -- exactly the
            # wall-clock reference N8 says was removed, re-entered through the
            # back door. Holding tau instead means the boat keeps station on the
            # last good target until the path server answers again.
            window_age = current_time - self.path_rx_time if self.path_rx_time is not None else None
            window_fresh = window_age is not None and window_age <= self.path_stale_timeout
            if self.controller_path.poses and not manual_active:
                if window_fresh:
                    e_along, e_y, _, _ = self.path_progress_errors(self.controller_path, current_state)
                    self.advance_governor(e_along, e_y)
                else:
                    self.get_logger().warning(
                        f"Path window stale ({window_age:.2f} s) - holding tau at "
                        f"{self.tau:.2f}.", throttle_duration_sec=5.0)

            # Issue the next request at the (governed) parameter tau
            if self.future is None:
                request = RequestPath.Request()
                request.path_request.data = np.linspace(self.tau,
                                                         self.tau + self.path_time,
                                                         int(self.path_steps), dtype=float)
                # Remember what we asked for: the response is a bare Path and
                # says nothing about which request it answers, so this is the
                # only thing accept_path() can check it against.
                self.tau_requested = float(np.float32(self.tau))
                self.request_sent = current_time
                self.future = self.client.call_async(request)
            # else: previous request still pending - keep controlling on the last path

        ## Compute thrust
        u = [0]*2

        if manual_active: # Manual target overrides: point LoS (unchanged)
            target = [*self.manual_target[:2], 0, 0, 0, 0] # yaw unused for LoS
            world_target = list(target[:3])  # --- world-frame monitoring target ---
            target = fm.inRobotFrame(current_state, target)
            u = self.solve_LoS(target, current_time)

        elif self.controller_path.poses: # Path following
            # Display the current desired pose if using gazebo
            if self.isSimulation:
                desired_pose = self.controller_path.poses[0].pose
                cf.create_pose_marker(desired_pose, self.pose_arrow_publisher) 

            if self.controller_type == 'MPC':
                u = self.controller.solve(path=self.controller_path, x_current=current_state)

                # A FAILED SOLVE IS NOT A COMMAND (C6). ur_mpc.solve already
                # returns zeros rather than the stale acados iterate; this is
                # where it is REPORTED, and it has to be reported through the
                # ROS logger. The old diagnostic was a bare print(), which never
                # reaches /rosout -- and neither Sim_launch.py nor the
                # simulator's full_mission_launch.py captures this node's
                # stdout, so the failure that drove the boat in circles for
                # 33 s left no trace anywhere an operator would look.
                #
                # Deliberately NOT an early return: falling through keeps
                # publish_thrust and the monitoring append on the path, so
                # /thruster_input stays alive (CLAUDE.md section 5) and the .npy
                # keeps recording -- which is what made this diagnosable at all.
                if self.controller.last_status != 0:
                    u = [0.0, 0.0]
                    self.get_logger().error(
                        f"MPC solve FAILED (acados status "
                        f"{self.controller.last_status}, "
                        f"{self.controller.fail_count} consecutive, "
                        f"{self.controller.total_failures} total) - commanding "
                        "ZERO thrust. The boat will drift. See CONTROLLERS.md C6.",
                        throttle_duration_sec=1.0)
                elif self.controller.last_solve_time > 0.5 * self.dt:
                    # The 20 Hz budget has never been measured on the boat's
                    # companion computer (TODO.md section 2), and doubling the
                    # horizon is precisely the change that would break it. This
                    # makes it visible from /rosout without a field harness (N7).
                    self.get_logger().warning(
                        f"MPC solve took "
                        f"{self.controller.last_solve_time * 1e3:.1f} ms of the "
                        f"{self.dt * 1e3:.0f} ms tick.", throttle_duration_sec=5.0)

                # Desired state for monitoring (first pose of the reference path)
                desired_pose = self.controller_path.poses[0].pose
                q = desired_pose.orientation
                psi_d = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
                target = [desired_pose.position.x, desired_pose.position.y, psi_d]
                world_target = list(target[:3])  # --- world-frame monitoring target ---
                
            if self.controller_type == 'PID':
                target = cf.compute_target(self.controller_path, self.dt)
                world_target = list(target[:3])  # --- world-frame monitoring target ---
                # Feed path tangent (target[2]) and authored speed (target[3])
                # so LoS steering and speed feedforward use the real path.
                psi_path, slow = target[2], False
                w = 0.0
                if self.hold_speed > 0.0:
                    w = 1.0 - min(1.0, max(0.0, target[3] / self.hold_speed))
                if w > 0.0 and self.hold_radius > 0.0:
                    # Stationary reference: the tangent is meaningless and the
                    # along-track term alone cannot see a cross-track error.
                    # Rotate the tangent handed to PIDLoS toward the bearing to
                    # the hold point, so its own along-track error becomes the
                    # range and its own LoS steering points at the point. The
                    # rotation fades out inside hold_radius, so on station the
                    # call is exactly what it was, and slow_on_turn (the class's
                    # own option) stops it driving away while it turns round.
                    rng = math.hypot(target[0] - current_state[0],
                                     target[1] - current_state[1])
                    g = min(1.0, max(0.0, (rng - self.hold_radius) / self.hold_radius))
                    if g > 0.0:
                        bearing = math.atan2(target[1] - current_state[1],
                                             target[0] - current_state[0])
                        psi_path = target[2] + w * g * _wrap(bearing - target[2])
                        slow = True
                u,_ = self.controller.compute(current_state, target[:3],
                                              u_ff=target[3], psi_path=psi_path,
                                              slow_on_turn=slow)

            if self.controller_type == 'LoS':
                target = cf.compute_target(self.controller_path, self.dt)
                world_target = list(target[:3])  # --- world-frame monitoring target ---
                u = self.los_guidance(target, current_state)
        
        elif self.use_pinger and self.pinger_target is not None: # MPC is not supported for this
            # --- world-frame monitoring target ---
            px, py = float(self.pinger_target[0]), float(self.pinger_target[1])
            c_m, s_m = np.cos(current_state[2]), np.sin(current_state[2])
            world_target = [current_state[0] + c_m*px - s_m*py,
                            current_state[1] + s_m*px + c_m*py, 0.0]
            # --------------------------------------
            if self.controller_type == 'PID':
                # Adapt the controller input to be used in robot frame
                target = [*self.pinger_target[:2], 0]
                current_state[[0,1,2]] = 0
                u,_ = self.controller.compute(current_state, target)

            if self.controller_type == 'LoS':
                target = self.pinger_target
                u = self.solve_LoS(target, current_time)

            # Publish controller target (for data recording)
            msg = Float32MultiArray()
            msg.data = [float(v) for v in target]
            self.target_publisher.publish(msg)

        else:
            self.get_logger().info('Nothing to target yet.')
            self.publish_thrust([0.0, 0.0])
            return

        target_str = ", ".join(f"{float(x):.2f}" for x in target)
        try:
            thrust_str = np.array2string(
                u,
                formatter={'float_kind': lambda x: f"{x:.2f}"}
            )
        except:
            thrust_str = ", ".join(f"{float(x):.2f}" for x in u)

        self.get_logger().info(
            f"\nTarget: [{target_str}]\n"
            f"Thrust: {thrust_str}"
        )

        # Publish thruster input. Reassigned so the monitoring row below records
        # the saturated command that was actually sent (see publish_thrust).
        u = self.publish_thrust(u)

        if self.pinger_target is not None and self.use_pinger:
            self.get_logger().info(f'\nPinger coordinates robot frame: \n{self.pinger_target}')
        if manual_active:
            target_str = ", ".join(f"{float(x):.2f}" for x in list(self.manual_target))
            self.get_logger().info(f'\nManual target coordinates: \n{target_str}')

        # Update and save monitoring metrics to be graphed later
        if self.controller_path.poses or (self.use_pinger and self.pinger_target is not None) or manual_active:
            x_m   = current_state[0]
            y_m   = current_state[1]
            psi_m = current_state[2]

            # --- world-frame monitoring target ---
            try:
                monitored = world_target
            except NameError:
                monitored = target
            x_d_m   = monitored[0]
            y_d_m   = monitored[1]
            psi_d_m = monitored[2] if len(monitored) > 2 else 0.0
            # --------------------------------------

            data_array = [current_time, x_m, y_m, psi_m,
                        x_d_m, y_d_m, psi_d_m, u[0], u[1]]

            self.monitoring.append(data_array)

            publisher_msg = Float32MultiArray()
            publisher_msg.data = [float(v) for v in data_array]
            self.data_publisher.publish(publisher_msg)

            # NOT saved from here any more -- see save_monitoring(), on its own
            # timer. Writing the whole log from the control tick cost the loop
            # its rate: measured on a 3 h mission, 20 Hz had decayed to 5-8 Hz.

    # ======================================================================
    #  4. GUIDANCE
    #  Path governor (N8) and the two steering laws.
    # ======================================================================

    # ------------------------------------------------------------------ #
    #  Path parameter governor                                           #
    # ------------------------------------------------------------------ #
    def warn_if_multiple_path_servers(self):
        """
        Say so, loudly, if more than one node offers /path_request.

        This node cannot refuse to run -- it is the controller -- but the
        operator needs to know, because with two servers every request is
        answered by whichever replies first and the reference alternates
        between two unrelated trajectories. path_generation refuses to be the
        second server; this covers the case where one was already up before
        that guard existed, or was started with allow_duplicate_server.
        """
        try:
            servers = []
            for name, namespace in self.get_node_names_and_namespaces():
                try:
                    services = self.get_service_names_and_types_by_node(name, namespace)
                except Exception:
                    continue
                if any(service == '/path_request' for service, _ in services):
                    servers.append(f"{namespace.rstrip('/')}/{name}")
        except Exception:
            return                      # graph query is a diagnostic, never fatal

        if len(servers) > 1:
            self.get_logger().error(
                "MORE THAN ONE /path_request SERVER: " + ", ".join(servers) + ". "
                "Requests will be answered by whichever replies first, so the "
                "reference will alternate between different trajectories and "
                "the boat will not follow any of them. Shut the other mission "
                "down. Responses that do not answer this node's own request "
                "are rejected, but the path will still stall while they are.")

    def accept_path(self, path):
        """
        Is this response an answer to OUR request?

        ROS 2 does not stop a second node offering /path_request, and when two
        do, every request is answered by whichever server replies first. The
        response is a bare nav_msgs/Path -- no echo of the request, no server
        identity -- so without a check the controller simply believes whatever
        arrives. Measured on this system with two missions up: the reference
        alternated tick by tick between two entirely different trajectories,
        both sampled at this node's own single tau, and the boat could follow
        neither. path_generation now refuses to be the second server; this is
        the other half, so a controller is never at the mercy of that.

        Two tests, cheapest first:

          length   the window must have as many poses as we asked for.
          tag      poses[0] must be stamped with the parameter we requested.
                   path_generation stamps each pose with the parameter it was
                   evaluated at (path_stamp.encode) precisely so this works
                   without changing RequestPath (N1/CM-1).

        A boat running a path_generation built before 2026-09-03 stamps the wall
        clock instead. Failing every response there would leave the controller
        with no path at all, which is worse than the fault being guarded, so
        that case is detected once, reported loudly, and falls back to geometry:
        reject a window whose first pose is further from the last accepted one
        than tau could possibly have moved in the meantime.

        Rejection keeps the last good window. The staleness gate in
        timer_callback is what stops the boat driving on it indefinitely.
        """
        poses = path.poses
        if not poses:
            self.get_logger().warning("Empty path from /path_request - ignored.",
                                      throttle_duration_sec=5.0)
            return False

        if len(poses) != int(self.path_steps):
            self.get_logger().warning(
                f"/path_request answered with {len(poses)} poses, asked for "
                f"{int(self.path_steps)} - ignored (another path server?).",
                throttle_duration_sec=5.0)
            self.foreign_path_count += 1
            return False

        stamp = poses[0].header.stamp

        # Establish once whether this server tags its poses at all.
        if self.path_stamps_are_parameters is None:
            self.path_stamps_are_parameters = ps.is_parameter(stamp.sec, stamp.nanosec)
            if not self.path_stamps_are_parameters:
                self.get_logger().error(
                    "The path server stamps poses with the clock, not the path "
                    "parameter: it predates the /path_request response tag. "
                    "Rebuild and reinstall blueboat_control on this machine. "
                    "Falling back to a geometric plausibility check, which "
                    "cannot reliably detect a second path server.")

        if self.path_stamps_are_parameters:
            if self.tau_requested is None:
                return True          # nothing to compare against yet
            if not ps.matches(stamp.sec, stamp.nanosec, self.tau_requested):
                self.foreign_path_count += 1
                self.get_logger().warning(
                    f"/path_request answered for parameter "
                    f"{ps.decode(stamp.sec, stamp.nanosec):.3f}, asked for "
                    f"{self.tau_requested:.3f} - ignored. Another path server "
                    f"is running ({self.foreign_path_count} so far); shut the "
                    "other mission down.", throttle_duration_sec=5.0)
                return False
            return True

        # --- fallback: geometry -------------------------------------------
        if not self.controller_path.poses or self.path_rx_time is None:
            return True              # nothing to compare against yet
        previous = self.controller_path.poses[0].pose.position
        current = poses[0].pose.position
        moved = math.hypot(current.x - previous.x, current.y - previous.y)
        # tau advances at most path_speed_scale per second, and the path itself
        # runs at some authored speed; allow a generous multiple of the window
        # before calling a jump impossible.
        allowed = max(2.0, 4.0 * self.path_time * max(1.0, self.path_speed_scale)
                      + self.los_hold_umax)
        if moved > allowed:
            self.foreign_path_count += 1
            self.get_logger().warning(
                f"/path_request window jumped {moved:.1f} m in one step (limit "
                f"{allowed:.1f} m) - ignored. Another path server is running "
                f"({self.foreign_path_count} so far).", throttle_duration_sec=5.0)
            return False
        return True

    def path_progress_errors(self, path, state):
        """
        From the current path window (poses[0] = virtual target at tau,
        poses[1] = a step further along), return:
          e_along : signed along-track gap boat->target  (target ahead > 0)
          e_y     : signed cross-track error of the boat
          gamma_p : path-tangent heading at the target
          U_d     : authored path speed at the target (m/s)
        """
        p0 = path.poses[0].pose
        p1 = path.poses[1].pose if len(path.poses) > 1 else path.poses[0].pose

        x0, y0 = p0.position.x, p0.position.y
        gamma_p = cf.quaternion_to_yaw(p0.orientation)

        dtau = self.path_time / max(1, (self.path_steps - 1))
        U_d = math.hypot(p1.position.x - x0, p1.position.y - y0) / dtau if dtau > 0 else 0.0

        xb, yb = state[0], state[1]
        c, s = math.cos(gamma_p), math.sin(gamma_p)
        e_along =  (x0 - xb) * c + (y0 - yb) * s
        e_y     = -(xb - x0) * s + (yb - y0) * c
        return e_along, e_y, gamma_p, U_d

    def advance_governor(self, e_along, e_y):
        """
        Advance the path parameter tau. When the boat is close to its virtual
        target the target moves at the authored speed (tau_dot = speed_scale);
        as the gap grows the target slows and finally pauses, so the boat can
        always catch up. Never moves backward.

        The gap is measured in both directions. The along-track factor answers
        "is the boat behind?"; the cross-track factor answers "is the boat off
        to the side?". A boat abreast of its target but far off the path is not
        keeping up with it, and only the second factor sees that.

        The cross-track factor is disabled by gov_Emax = 0 (the default), which
        makes it identically 1 and leaves the along-track behaviour untouched.

        Both factors are clipped to [0, 1] and multiplied, so their product is
        also in [0, 1]: tau is still monotonic and still bounded above by the
        path's own parameterisation rate. That upper bound is what makes an
        authored speed profile that varies along the path get followed for
        free, so nothing here may scale tau_dot by more than unity.

        Returns the combined factor (diagnostic).
        """
        span_along = max(1e-6, (self.gov_Lmax - self.gov_Lmin))
        fac_along = np.clip((self.gov_Lmax - e_along) / span_along, 0.0, 1.0)

        if self.gov_Emax > 0.0:
            span_cross = max(1e-6, (self.gov_Emax - self.gov_Emin))
            fac_cross = np.clip((self.gov_Emax - abs(e_y)) / span_cross, 0.0, 1.0)
        else:
            fac_cross = 1.0

        factor = fac_along * fac_cross
        tau_dot = self.path_speed_scale * factor
        self.tau += tau_dot * self.dt
        return factor

    # ------------------------------------------------------------------ #
    #  Perfected line-of-sight guidance (kinematic, 'LoS' controller)    #
    # ------------------------------------------------------------------ #
    def los_guidance(self, target6, state):
        """
        Canonical Fossen lookahead LoS to the path point described by
        target6 = [x_ref, y_ref, gamma_p, U_d, *_]:
            psi_d = gamma_p + atan2(-e_y, Delta)
        Surge command is the authored speed, reduced while turning hard.
        Returns differential thrust [f_right, f_left].
        """
        x, y, psi = state[0], state[1], state[2]
        u = state[3]
        r = state[5]

        x_ref, y_ref, gamma_p = target6[0], target6[1], target6[2]
        U_d = target6[3]

        c, s = math.cos(gamma_p), math.sin(gamma_p)
        e_y = -(x - x_ref) * s + (y - y_ref) * c

        psi_d = gamma_p + math.atan2(-e_y, self.los_lookahead)

        # Zero-authored-speed hold. w is exactly 0 for any path that has a
        # speed, so everything in this block collapses and u_cmd is the plain
        # feedforward law - path following is untouched.
        w = 0.0
        if self.hold_speed > 0.0:
            w = 1.0 - min(1.0, max(0.0, U_d / self.hold_speed))

        u_hold = 0.0
        if w > 0.0:
            rng = math.hypot(x_ref - x, y_ref - y)
            gap = max(0.0, rng - self.hold_radius)
            if gap > 0.0:
                # Steer at the hold point rather than along a tangent that means
                # nothing when the reference is stationary, and never command
                # reverse: a lookahead law steers the wrong way backwards, so the
                # yaw channel turns the boat round instead.
                bearing = math.atan2(y_ref - y, x_ref - x)
                psi_d = psi_d + w * _wrap(bearing - psi_d)
                u_hold = min(self.los_hold_umax, w * self.los_hold_kx * gap)

        psi_err = _wrap(psi_d - psi)

        u_cmd = (self.los_speed_scale * U_d + u_hold) * max(0.0, math.cos(psi_err))

        X = self.los_ku * (u_cmd - u)
        N = self.los_kpsi * psi_err - self.los_kd * r

        thrusts = self.los_allocator.allocate(np.array([X, 0.0, N]))
        return thrusts

    def solve_LoS(self, target, current_time):
        # POINT line-of-sight (used for pinger / manual targets, body frame).
        # Unchanged from the working version.
        x,y,z = target

        bearing = np.arctan2(y, x)
        yaw_rate = self.k_psi * bearing
        d = np.sqrt(x**2+y**2)
        v = self.k_v * d
        v = 5*np.log(v+1)

        manual = list(self.manual_target) != [0.0,0.0]
        if manual:
            v = 10*np.log(v+1) if not self.isSimulation else 7*np.log(v+1)  # If manual target, go faster. Don't need to be that precise here.

        # Propeller breakaway. v is the common-mode surge force this law puts on
        # BOTH thrusters, and 5*ln(k_v*d + 1) stays under min_thrust (2 N) out to
        # d = 3.28 m at the real boat's k_v = 0.15 -- a range at which the ESC is
        # still inside its neutral deadband, so the boat sits still while the log
        # shows it being commanded forward. Floor the surge so it actually moves.
        #
        # ONLY THE SURGE. The +/- 0.295*yaw_rate differential below is untouched,
        # so the yaw moment -- and with it which way the boat turns and how hard
        # -- is bit-identical to the unfloored law at every range and bearing.
        # The floor pushes the boat out of the deadband; it does not steer it.
        #
        # The floor is shaped by two factors, both of which only ever REDUCE it,
        # and both of which reuse a blend this file already applies elsewhere:
        #
        #   g            fades the floor in over hold_radius, exactly as the PID
        #                and LoS station-keeping holds fade theirs. Without it the
        #                floor stepped by 1.64 N as the boat crossed 0.5 m
        #                inbound; with it the commanded surge is continuous in d.
        #   max(0, cos)  kills the floor when the target is abeam or behind, the
        #                same shaping los_guidance puts on its own feedforward.
        #                Forward surge closes the range by cos(bearing) only, so
        #                past 90 degrees a floored surge would drive the boat AWAY
        #                from the target while it turned round -- the unfloored
        #                law does not, because its surge is ~0 there. The turn
        #                itself needs no help at those bearings: the differential
        #                is already 4.6 N per side at 90 degrees, well clear of
        #                the deadband.
        #
        # v is never negative here (5*ln(x+1) with x >= 0), so no sign handling is
        # needed, and the floor never lowers v. min_thrust = 0.0 disables the
        # whole block and restores the original law exactly.
        if self.min_thrust > 0.0 and self.hold_radius > 0.0:
            g = min(1.0, max(0.0, (d - self.hold_radius) / self.hold_radius))
            breakaway = self.min_thrust * g * max(0.0, np.cos(bearing))
            if 0.0 < v < breakaway:
                v = breakaway

        # A MANUAL target is a station to hold, not a place to stop: once
        # reached it is held against drift rather than abandoned at zero thrust.
        # Everything below this block therefore belongs to the PINGER branch
        # alone and is bit-identical to what it was -- safety_distance and
        # stopping_sequence are not consulted for a manual target at all.
        if manual:
            held = self.manual_keep_location(d, bearing, yaw_rate, current_time)
            if held is not None:
                return held
            # Off station: the plain pursuit law, ungated.
            return [v + 0.295 * yaw_rate, v - 0.295 * yaw_rate]

        thruster_input = [0,0]

        # Convert to differential thrust
        if not self.stopping_sequence:
            if d > self.safety_distance :
                thruster_input[0] = v + 0.295 * yaw_rate
                thruster_input[1] = v - 0.295 * yaw_rate
            else:
                self.get_logger().info("LoS target reached, initializing stopping sequence")
                self.stopping_sequence = True
                self.stopping_time = current_time

        # As a safety, if the target is close enough, briefly move back then stop
        else: 
            if current_time - self.stopping_time < 1.0:
                thruster_input = [-1.,-1.]
            else:
                thruster_input = [0.,0.]

        return thruster_input

    def manual_keep_location(self, d, bearing, yaw_rate, current_time):
        """
        Hold station on a REACHED manual target.

        Returns the thruster pair while holding, or None when the boat is off
        station and the caller should run the plain pursuit law instead.

        Why this exists. solve_LoS used to latch on arrival: one second astern,
        then zero thrust for the rest of the run, cleared only by a new target.
        With any current that is a boat commanding nothing while it is pushed
        out of the survey area -- and on the real boat the latch never armed at
        all (safety_distance = -1.0), so the point law hunted around the target
        instead of settling. Neither is station keeping.

        The state below is therefore re-evaluated every tick, never latched:

            not holding, d <= manual_hold_radius       -> hold
            holding,     d >  manual_reacquire_radius  -> resume pursuit

        The two radii are deliberately different. A single threshold with
        position noise sitting on it toggles the mode every few ticks; the gap
        between them is the hysteresis that stops that.

        The hold itself is CONTINUOUS, not an on/off deadband: surge is
        proportional to the range outside manual_hold_radius and capped, so the
        law always has an answer to a disturbance and never falls silent. It is
        the same shape as los_guidance's zero-authored-speed hold -- steer at
        the point, surge proportional to the gap, capped, never reverse, inside
        the same max(0, cos) shaping -- but its gains are its own, because
        los_hold_kx / los_hold_umax are velocities fed through los_ku and the
        allocator while this law writes its surge straight onto the wire.

        Two things it does NOT do, both on purpose:

          * The yaw channel is untouched. The differential is the caller's
            +/- 0.295*yaw_rate exactly as before, so which way the boat turns
            and how hard is the same law it always was; only the common-mode
            surge is replaced.
          * The surge is not floored to min_thrust. The pursuit law is floored
            because a command under 2 N sits in the ESC deadband and moves
            nothing while the log shows thrust; here that band IS the wanted
            behaviour -- it is what lets the boat sit still on station. The
            price is an effective hold box of manual_hold_radius plus about
            0.25 m on the real boat, where the proportional term first clears
            breakaway.

        max(0, cos(bearing)) matters for the same reason it matters in the
        floor: forward surge closes the range by cos(bearing) only, so pushing
        while the target is abeam or behind drives the boat away from the point
        it is trying to hold. The yaw channel turns it round first.
        """
        if self.manual_hold_radius <= 0.0:
            return None          # hold disabled: the pursuit law, as it was

        # --- state, re-evaluated every tick ---------------------------------
        if not self.manual_hold:
            if d <= self.manual_hold_radius:
                self.manual_hold = True
                self.manual_brake_t0 = current_time
                self.get_logger().info(
                    f"Manual target reached at {d:.2f} m - holding location "
                    f"(re-acquiring beyond {self.manual_reacquire_radius:.2f} m)")
        elif d > self.manual_reacquire_radius:
            self.manual_hold = False
            self.manual_brake_t0 = None
            self.get_logger().info(
                f"Pushed {d:.2f} m off the manual target - re-acquiring it")

        if not self.manual_hold:
            return None

        # --- arrival brake: the original one-second astern pulse ------------
        if (self.manual_brake_t0 is not None
                and current_time - self.manual_brake_t0 < self.manual_brake_time):
            return [-1., -1.]

        # --- continuous proportional hold -----------------------------------
        gap = max(0.0, d - self.manual_hold_radius)
        v_hold = min(self.manual_hold_umax, self.manual_hold_kx * gap)
        v_hold *= max(0.0, np.cos(bearing))

        # Same breakaway floor the pursuit law gets, and for the same reason:
        # the proportional term does not clear 2 N until the boat is already
        # 0.25 m outside the radius it was told to hold (0.13 m in simulation),
        # so unfloored the commanded station is one the hardware cannot reach.
        # Measured on the harness plant with an explicit 2 N per-side deadband:
        # floored the boat parks at 1.00 m, unfloored at 1.25 m.
        #
        # The floor is written out here rather than reusing the block above,
        # which is keyed to hold_radius -- a parameter shared with los_guidance
        # and the PID branch that this must not couple to. It cannot lift a zero
        # command (the guard is 0.0 < v_hold), so "no surge inside the radius"
        # survives it, and it is exactly the step that makes the station
        # attainable: the cost is that the thrusters pulse at a few hertz while
        # holding against a current. That is deliberate and is documented.
        if self.min_thrust > 0.0:
            floor = self.min_thrust * max(0.0, np.cos(bearing))
            if 0.0 < v_hold < floor:
                v_hold = floor

        return [v_hold + 0.295 * yaw_rate, v_hold - 0.295 * yaw_rate]

    # ======================================================================
    #  5. CALLBACKS AND SMALL HELPERS
    #  Inbound telemetry; nothing here computes control.
    # ======================================================================

    def odom_callback(self, msg: Odometry):
        pose, twist = cf.odometry(msg)

        self.current_pose = pose
        self.current_twist = twist

    def pinger_callback(self, msg: Float32MultiArray):
        self.pinger_target = msg.data

    def ready_callback(self, msg: Bool):
        # robot_interface now re-publishes readiness periodically (so this node can
        # never miss it); only log the transition to avoid spam
        if msg.data and not self.ready:
            self.get_logger().info(f'Controller ready')
        self.ready = msg.data

    def manual_target_callback(self, msg: Float32MultiArray):
        self.manual_target = msg.data # [x,y] in world frame
        self.stopping_sequence = False

        # Reset the keep-location state only when the target actually MOVES.
        # Re-publishing the same coordinates must not re-arm the arrival brake
        # on a boat that is already holding station on them; a genuinely new
        # target must. The [0, 0] resume sentinel differs from any real target,
        # so it clears the hold on its way past.
        target = [round(float(value), 6) for value in list(self.manual_target)[:2]]
        if target != self.manual_hold_target:
            self.manual_hold_target = target
            self.manual_hold = False
            self.manual_brake_t0 = None

    # inRobotFrame() moved to _custom_libraries/frame_math.py -- it used no
    # node state at all, so it is pure geometry and now unit-testable without
    # a ROS workspace. Called below as fm.inRobotFrame(...).

    def publish_thrust(self, u):
        """
        Publish /thruster_input. Used both for the computed command and to say
        "zero" explicitly whenever this node has nothing to command, so the
        interface nodes are never left re-applying a stale thrust.

        The command is saturated to +/- thrust_limit here, UNIFORMLY: one scale
        factor for both thrusters, so the right:left ratio -- and with it the
        direction of the commanded wrench -- survives the clamp and the boat
        keeps the turn it asked for, just slower. See thrust_limits.py for why
        clipping each side on its own does not do that.

        This is the single exit for thrust, so it is the one place the rule has
        to hold. It is a no-op for MPC, PID and LoS, whose commands are already
        bounded (the allocator scales the same way, and the MPC's input bounds
        are the same number). It binds on solve_LoS, which builds its array by
        hand, goes through no allocator, and can reach 30-40 N -- the most
        likely source of the recorded thrust above the clamp that TODO.md
        section 5 could not account for.
        """
        limited, scale = tl.scale_to_limit(u, self.thrust_limit)
        if scale < 1.0:
            self.get_logger().warn(
                f"Thrust command {np.array2string(np.asarray(u, dtype=float), precision=1)} "
                f"exceeds thrust_limit={self.thrust_limit:.1f} N - scaled by {scale:.3f} "
                f"to {np.array2string(limited, precision=1)} (direction preserved).",
                throttle_duration_sec=2.0)

        msg = Float32MultiArray()
        msg.data = [float(v) for v in limited]
        self.thruster_input_publisher.publish(msg)

        # Returned so the caller logs what went on the wire, not what it wished
        # for: /monitoring_data[7:9] and the .npy u1/u2 then agree with the CSV's
        # right_thr_in/left_thr_in, which is read straight off /thruster_input.
        return limited

    def save_monitoring(self):
        """
        Write the controller log.

        On its own timer, not in the control tick. np.save rewrites the whole
        file every call -- there is no append for .npy -- and because row 0 is
        a header of strings the entire array is re-coerced to <U32 each time.
        Done at 10 Hz from timer_callback that is O(n) work per tick against a
        list that grows every tick: measured on a 3 h run, the control loop had
        decayed from its 0.05 s tick to 0.13-0.20 s, and the decay is unbounded.

        Two changes, and neither touches the file's contents: the write happens
        on a slower timer of its own, and it is skipped entirely when no new row
        has arrived. The on-disk schema is byte-identical, so existing logs,
        replay.py and every analysis script are unaffected (CM-7).
        """
        rows = len(self.monitoring)
        if rows == self.monitoring_saved_rows:
            return                      # nothing new since the last write
        try:
            np.save(self.title, self.monitoring)
            self.monitoring_saved_rows = rows
        except Exception as exc:
            self.get_logger().error(f"Could not write {self.title}.npy: {exc}",
                                    throttle_duration_sec=10.0)

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9
        

rclpy.init()
node = Controller()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    # The log is written on a timer now, so a run cut short here could otherwise
    # lose its last couple of seconds. Flush before going away (CM-7).
    try:
        node.save_monitoring()
    except Exception:
        pass
    node.destroy_node()
    with contextlib.suppress(Exception):
        rclpy.shutdown()
