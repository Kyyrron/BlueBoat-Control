#!/usr/bin/env python3

# ============================================================================
# The controller node: MPC / PID / LoS on one control callback at 20 Hz.
#
# The reference is not played on a clock: a GOVERNOR advances the path
# parameter tau from the boat's measured progress, so the target can never
# outrun the boat (CLAUDE.md N8). 'LoS' is Fossen lookahead LoS, 'PID' the same
# law with path-speed feedforward, 'MPC' an acados OCP over a horizon window.
# /monitoring_data is world-frame in every branch; each capture site is marked
# "# --- world-frame monitoring target ---" (N9).
#
# Run sessions (session_mode:=true, station real-robot launches only): the node
# idles until 'start' on /blueboat/run_command and 'configure' may switch
# controller between runs (MPC built on a worker thread). Live path update
# (path_source:=replanner, section 7, docs/PATH_UPDATE_*.md): a replanner-owned
# run holds its own path, fed through /blueboat/update_path, instead of asking
# /path_request.
#
# FILE MAP (class Controller):
#   1. WIRING              __init__, controller construction
#   2. TUNING KNOBS        _declare_tuning_parameters   <-- gains live here
#   3. THE CONTROL LOOP    timer_callback               <-- start reading here
#   4. GUIDANCE            path governor, accept_path, los_guidance, solve_LoS,
#                          manual_keep_location
#   5. CALLBACKS/HELPERS   telemetry callbacks, publish_thrust, save_monitoring
#   6. RUN SESSION         run_command_callback ... _publish_run_state
#   7. LIVE PATH UPDATE    _init_path_update ... publish_path_progress
#                          (decisions: _custom_libraries/path_update.py)
# ============================================================================

from rclpy.node import Node, QoSProfile
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
import rclpy

import contextlib
import json
import os
import sys
import time
import math
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from scipy.spatial.transform import Rotation as R
from datetime import datetime

from std_msgs.msg import Bool, Float32MultiArray, String
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from blueboat_interfaces.srv import RequestPath, UpdatePath

import ur_mpc
import PID
import custom_functions as cf
import frame_math as fm         # world<->body geometry (ROS-free)
import path_stamp as ps         # /path_request response tag (ROS-free)
import thrust_limits as tl      # uniform thrust saturation (ROS-free)
import path_update as pu        # live path update core (ROS-free)
import robot_log_schema as rls  # pinger modes: one resolver with robot_interface (ROS-free)

# Statuses of /blueboat/pinger/estimate the follow law may steer to.
PINGER_CHASE_STATUSES = ('tracking', 'coasting')

# Column names of the controller .npy log; row 0 of the saved array (CM-7).
NPY_HEADER = ['t', 'x', 'y', 'psi', 'x_d', 'y_d', 'psi_d', 'u1', 'u2']


def _run_config_problem(ctype, mode, has_path, path_source):
    """Why a run configuration cannot run, or None. One table for a launch (a
    FATAL) and a station `configure` (its refusal)."""
    if ctype not in ('PID', 'LoS', 'MPC'):
        return f"unknown controller_type '{ctype}'"
    if ctype == 'MPC' and mode == 'follow':
        return "MPC does not support pinger following - choose PID or LoS"
    if path_source not in pu.PATH_SOURCES:
        return f"unknown path_source '{path_source}' (one of {', '.join(pu.PATH_SOURCES)})"
    if path_source == 'replanner' and mode == 'follow':
        return ("a replanner run follows paths, not the pinger - "
                "set use_pinger false or path_source 'mission'")
    if path_source == 'replanner' and not has_path:
        return ("a replanner run always has a path (the replanner's) - "
                "has_path false needs path_source 'mission'")
    return None


def _allocation_matrix():
    """B of the two-thruster allocation: [surge, sway, yaw] = B @ [right, left]."""
    radius = 0.59/2
    return np.array([[1.        ,1.],
                     [0.        ,0.],
                     [radius,-radius]])


def _reference_pose(path):
    """[x, y, psi] of the FIRST pose of a reference window, world frame.

    The one definition of "where the reference is now", shared by every branch
    so /monitoring_data[4:6] (hence the CSV target columns and the .npy x_d/y_d)
    means the same thing whatever controller ran (N9). Not what the PID and LoS
    laws are handed: cf.compute_target differences the window's second pose too.
    """
    pose = path.poses[0].pose
    q = pose.orientation
    psi = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
    return [pose.position.x, pose.position.y, psi]


class Controller(Node):

    # ======================================================================
    #  1. WIRING -- topics, services, timers, state
    # ======================================================================

    def __init__(self):

        super().__init__('master_control', namespace='blueboat')

        self.controller_type = self._param('controller_type', 'MPC').string_value
        self.isSimulation = self._param('simulation', True).bool_value
        self.use_pinger = self._param('use_pinger', False).bool_value
        # Pinger mode (2026-11-24, additive): off | follow | log; '' = from
        # use_pinger. `use_pinger` below keeps meaning FOLLOW the pinger.
        # log = "log the pinger only": the boat never steers to it. In off and
        # log mode the run follows the path (any controller), or with has_path
        # False requests no path at all and only holds a manual target /
        # arrow keys (has_path means nothing when following the pinger).
        try:
            mode = rls.pinger_mode_of(self._param('pinger_mode', '').string_value,
                                      self.use_pinger)
        except ValueError as exc:
            self.get_logger().fatal(str(exc))
            raise
        self._set_pinger_mode(mode, self._param('has_path', True).bool_value)
        # Follow mode chases only a fresh estimate (status tracking/coasting,
        # message younger than this); otherwise it holds position.
        self.pinger_stale_timeout = self._param('pinger_stale_timeout', 3.0).double_value
        self.pinger_source = 'filter'   # echoed in run_state (set by configure)
        # Run sessions (section 6): only a station real-robot launch sets it.
        self.session_mode = self._param('session_mode', False).bool_value
        # Empty = resolve it (custom_functions.data_root).
        self.data_root = cf.data_root(self._param('data_dir', '').string_value)

        self._declare_tuning_parameters()

        self.odom_subscriber = self.create_subscription(Odometry, '/blueboat/odom', self.odom_callback, 10)
        self.pinger_subscriber = self.create_subscription(Float32MultiArray, '/blueboat/pinger_coordinates', self.pinger_callback, 10)
        # The estimate's status gates the follow law (stale -> hold position).
        self.pinger_estimate_subscriber = self.create_subscription(
            String, '/blueboat/pinger/estimate', self.pinger_estimate_callback, 10)
        # What the path server serves (duration, loop) -> path_complete.
        path_info_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_info_subscriber = self.create_subscription(
            String, '/blueboat/path_info', self.path_info_callback, path_info_qos)
        self.ready_subscriber = self.create_subscription(Bool, '/blueboat/controller_ready', self.ready_callback, 10)
        self.manual_target_subscriber = self.create_subscription(Float32MultiArray, '/blueboat/manual_target', self.manual_target_callback, 10)
        # Station arrow keys: [right, left] N while driving, [] to release.
        self.arrow_subscriber = self.create_subscription(Float32MultiArray, '/blueboat/arrow_thrust', self.arrow_thrust_callback, 10)

        self.data_publisher = self.create_publisher(Float32MultiArray, "/monitoring_data", 10)
        self.target_publisher = self.create_publisher(Float32MultiArray,'/controller_target', 10)
        self.thruster_input_publisher = self.create_publisher(Float32MultiArray, "/thruster_input", 10)
        self.pose_arrow_publisher = self.create_publisher(Marker, "/pose_arrow", 10)

        if self.session_mode:
            self.run_command_subscriber = self.create_subscription(String, '/blueboat/run_command', self.run_command_callback, 10)
            self.run_state_publisher = self.create_publisher(String, '/blueboat/run_state', 10)

        # In a session path_generation always runs (the follow mode can change
        # between runs), so the client does too. Outside one: not when
        # following the pinger, nor in log mode without a path.
        if self._follows_path() or self.session_mode:
            self.client = self.create_client(RequestPath, '/path_request')

            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service...")

            self.warn_if_multiple_path_servers()

        self.future = None              # in-flight /path_request

        self.time_set = False
        self.initial_time = None
        self.timer = self.create_timer(self.dt, self.timer_callback)
        # The .npy is written off the control tick (save_monitoring).
        self.log_timer = self.create_timer(2.0, self.save_monitoring)

        self.current_pose = None
        self.current_twist = None

        self.ready = False
        self.pinger_target = None
        self.pinger_estimate = None     # latest /blueboat/pinger/estimate (dict)
        self.pinger_estimate_rx = None  # _world_clock() when it arrived
        self.pinger_hold = None         # (x, y, yaw) held while the estimate is stale
        self.path_info = None           # latest /blueboat/path_info (dict)
        self.path_complete_logged = False
        self.manual_target = [0.0,0.0]

        self.controller_path = Path()

        # /path_request response validation (accept_path): the response is a
        # bare Path, so these carry the question it has to answer.
        self.tau_requested = None       # parameter the in-flight request asked for
        self.request_sent = None        # when it went out, for the timeout
        self.path_rx_time = None        # when the held window last arrived
        self.path_stamps_are_parameters = None   # None = not yet established
        self.foreign_path_count = 0

        # Path parameter of the virtual target, advanced by the governor (N8).
        self.tau = 0.0

        self.controller = None          # built by _build_controller below
        self._apply_controller_setup(self.controller_type)

        # Pinger-branch arrival latch (solve_LoS); safety_distance arms it.
        self.stopping_sequence = False
        self.stopping_time = None

        # Manual-target keep-location state (manual_keep_location), separate
        # from the pinger latch so the two point modes cannot mute each other.
        #   manual_hold        - holding station on a reached target
        #   manual_brake_t0    - start of the arrival reverse pulse
        #   manual_hold_target - the target the state refers to, so a repeat of
        #                        the same target does not re-arm the pulse
        self.manual_hold = False
        self.manual_brake_t0 = None
        self.manual_hold_target = None

        self.monitoring = [list(NPY_HEADER)]
        self.monitoring_saved_rows = 0
        self.monitoring_chunks = []     # rows already converted for the .npy (save_monitoring)

        # Arrow control (arrow_thrust_callback): highest priority while active;
        # a stale stream commands zero, never autonomous control.
        # last_world_target is the latest world-frame target of any branch,
        # logged as the target while the keys drive.
        self.arrow_active = False
        self.arrow_cmd = [0.0, 0.0]
        self.arrow_rx = None
        self.last_world_target = None

        # Run-session state (section 6). Outside a session a run is always
        # active, which makes every run gate a no-op.
        self.run_active = not self.session_mode
        self.run_status = 'idle'        # idle | loading | running | error
        self.run_number = 0
        self.run_t0 = None              # _world_clock() at the current run's start
        self.run_note = ''
        self.run_mission = ''
        self.cmd_seq = None             # last /blueboat/run_command handled
        self.cmd_ok = True
        self.cmd_detail = ''
        self._build_pool = None         # worker thread for an MPC rebuild
        self._build_future = None
        self._build_mode = None         # (pinger_mode, has_path) with that build

        if not self.session_mode:
            self.title = self._reserve_npy(self.controller_type)
            self.get_logger().info(f"Controller log: {self.title}.npy")
        else:
            # One .npy per run, reserved at its start: nothing while idle.
            self.title = None
            self.run_state_timer = self.create_timer(1.0, self._publish_run_state)

        # Built here, not on the first tick: MPC's acados codegen + compile run
        # from timer_callback froze the single-threaded executor (odom, path
        # futures, publish_thrust), and a failure here is visible at launch.
        self._build_controller()

        # Last, so every piece of state it reads exists.
        self._init_path_update()

    # ------------------------------------------------------------------ #
    #  Parameters and files                                              #
    # ------------------------------------------------------------------ #
    def _param(self, name, default):
        """Declare one parameter and return its ParameterValue."""
        self.declare_parameter(name, default)
        return self.get_parameter(name).get_parameter_value()

    def _dbl(self, name, default):
        return self._param(name, float(default)).double_value

    def _world_clock(self):
        """Seconds on the world-timing clock (parameter ``time_source``):
        ``time.time()`` for 'wall', the node clock for 'ros'."""
        if self.time_source == 'ros':
            return self.get_clock().now().nanoseconds * 1e-9
        return time.time()

    def _int(self, name, default):
        return self._param(name, int(default)).integer_value

    def _reserve_npy(self, ctrl):
        """Claim this run's controller log name (without the .npy suffix)."""
        date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        sim = 'simulation' if self.isSimulation else 'real'
        run_dir = cf.ensure_data_dir(self, self.data_root, 'data', f'{ctrl}_data')
        return cf.reserve_run_file(run_dir, f'{date}-{ctrl}_{sim}_data', '.npy')

    # ------------------------------------------------------------------ #
    #  Controller construction                                           #
    # ------------------------------------------------------------------ #
    @staticmethod
    @contextlib.contextmanager
    def _no_stdin():
        """Run a block with stdin closed, so a prompt raises instead of blocking.

        acados_template's get_tera() asks `input(...)` when the Tera renderer is
        missing; under `ros2 launch` that prompt is invisible and never returns.
        With stdin on /dev/null it raises EOFError and we can report it.
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

    def _apply_controller_setup(self, controller_type):
        """Reference window, allocation matrix and allocator for one controller.

        Cheap and parameter-only, so it runs on the executor thread: from
        __init__, and again when a session 'configure' switches controller.
        """
        if controller_type == 'MPC':
            # Derived, never declared, so window and horizon cannot disagree.
            # N + 1 poses: solve() reads N stages plus the terminal one, and
            # this spacing (time/horizon) is exactly the solver's.
            self.path_time = self.mpc_time
            self.path_steps = self.mpc_horizon + 1

        if controller_type in ('PID', 'LoS'):
            self.path_time = self.dt
            self.path_steps = 2

        if controller_type == 'LoS':
            self.los_allocator = PID.ThrustAllocator(_allocation_matrix(), self.thrust_limit)

    def _make_controller(self, controller_type):
        """Construct and RETURN the controller object for controller_type.

        PID: a plain Python object. LoS: None (its laws are methods of this
        node). MPC: acados codegen + C compile, with a preflight check. Assigns
        nothing, because a session calls it for MPC from a worker thread and
        swaps the result in on the executor thread (_poll_controller_build).
        """
        if controller_type == 'MPC':
            self._check_acados_ready()
            build_dir = ur_mpc.acados_build_dir()
            self.get_logger().info(
                f"Building the acados MPC solver in {build_dir} -- the first "
                "run compiles C and can take about a minute; later runs reuse it.")
            try:
                with self._no_stdin():
                    controller = ur_mpc.MPCController(**self.mpc_model,
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
                f"{'reused' if controller.reused_solver else 'generated and compiled'}.")
            return controller

        if controller_type == 'PID':
            return PID.PIDLoS(dt = self.dt,
                              B = _allocation_matrix(),
                              outer_gains = self.outer_gains,
                              inner_gains = self.inner_gains,
                              lookahead = self.pid_lookahead,
                              thrust_limit = self.thrust_limit)

        return None

    def _build_controller(self):
        """Construct the controller named by controller_type (see _make_controller)."""
        controller = self._make_controller(self.controller_type)
        if controller is not None:
            self.controller = controller

        self.get_logger().info('Controller node initiated')
        self.get_logger().info(
            "Tuning defaults: %s column (gain_profile '%s', simulation %s); MPC N=%d T=%.1f s R=%.3f." % (
                'sim-tuned legacy' if self.tuning_sim else 'real-boat', self.gain_profile,
                self.isSimulation, self.mpc_horizon, self.mpc_time, float(self.R_weight[0, 0])))

    # ======================================================================
    #  2. TUNING KNOBS -- every gain, all in one place
    #  Values, real/sim splits and measurements: src/CLAUDE.md section 4 and
    #  FIELD_TUNING.md. Change a default here or override it at launch.
    # ======================================================================

    def _declare_tuning_parameters(self):
        """
        Declare every tuning constant with today's value as its default,
        unconditionally (whatever controller_type), so `ros2 param list` shows
        the whole set. Composite constants are double arrays reassembled here
        (ROS 2 has no tuple type). Read once, at construction.
        """
        dbl, integer = self._dbl, self._int

        # -- which column of the split defaults below (additive, 2026-11-13) --
        # Several defaults exist twice: the REAL BOAT's (gains, MPC model,
        # horizon and weights as flown) and a set tuned on the old Gazebo
        # plant (BlueROV2 hull coefficients). `gain_profile`:
        #   ''  or 'field'  the real boat's column. On the boat this is what
        #                   always ran. In simulation it is the field-equivalent
        #                   configuration, meant for the identified plant
        #                   (blueboat_description plant:=identified).
        #   'sim'           the sim-tuned column: simulation only, the legacy
        #                   configuration (with plant:=bluerov2_legacy it
        #                   reproduces every simulated run made before this date;
        #                   test/mc_harness.py --compare HEAD --param gain_profile:=sim).
        # Only these defaults follow it. File names, /pose_arrow and the clock
        # keep following `simulation`. An explicitly passed parameter wins over
        # either column, as before.
        self.gain_profile = self._param('gain_profile', '').string_value
        if self.gain_profile not in ('', 'field', 'sim'):
            raise ValueError(f"unknown gain_profile '{self.gain_profile}' ('' | field | sim)")
        if self.gain_profile == 'sim' and not self.isSimulation:
            raise ValueError("gain_profile 'sim' is the legacy SIMULATION tuning: "
                             "refused with simulation:=false (the real boat keeps its own column).")
        self.tuning_sim = self.gain_profile == 'sim'

        def arr(name, default):
            value = list(self._param(name, [float(v) for v in default]).double_array_value)
            if len(value) != len(default):
                raise ValueError(f"parameter '{name}' expects {len(default)} "
                                 f"values, got {len(value)}")
            return value

        # -- control loop (the governor rescales with it; MPC must solve in it)
        self.dt = dbl('control_dt', 0.05)

        # -- path-parameter governor ---------------------------------------
        self.path_speed_scale = dbl('path_speed_scale', 1.0)
        self.gov_Lmin = dbl('gov_Lmin', 0.5)
        self.gov_Lmax = dbl('gov_Lmax', 3.0)
        # Cross-track gate; gov_Emax = 0 (default) disables it. Only safe once
        # the inner loops can close a lateral gap: throttling tau on an error
        # the controller cannot reduce is positive feedback.
        self.gov_Emin = dbl('gov_Emin', 0.5)
        self.gov_Emax = dbl('gov_Emax', 0.0)

        # -- PID (split simulation/real) -------------------------------------
        if not self.tuning_sim:
            self.outer_gains = {'x':   tuple(arr('outer_gains_x',   [3.5, 0.01, 0.0])),
                                'psi': tuple(arr('outer_gains_psi', [3.3, 0.01, 0.0]))}
            self.inner_gains = {'u': tuple(arr('inner_gains_u', [1.0, 0.0, 0.0])),
                                'r': tuple(arr('inner_gains_r', [1.5, 0.0, 0.0]))}
        else:
            self.outer_gains = {'x':   tuple(arr('outer_gains_x',   [8.0, 0.01, 0.0])),
                                'psi': tuple(arr('outer_gains_psi', [4.0, 0.01, 0.0]))}
            self.inner_gains = {'u': tuple(arr('inner_gains_u', [2.5, 0.0, 0.0])),
                                'r': tuple(arr('inner_gains_r', [2.5, 0.0, 0.0]))}
        self.pid_lookahead = dbl('pid_lookahead', 2.5)

        # -- kinematic Fossen-LoS -------------------------------------------
        self.los_lookahead   = dbl('los_lookahead', 2.5)
        self.los_ku          = dbl('los_ku', 30.0)
        self.los_kpsi        = dbl('los_kpsi', 4.5)
        self.los_kd          = dbl('los_kd', 1.0)
        self.los_speed_scale = dbl('los_speed_scale', 1.0 if not self.tuning_sim else 2.0)
        # Zero-authored-speed hold, shared by PID and LoS: a stationary
        # reference (U_d = 0) blends to steering at the point below hold_speed.
        # Inert on real paths: every library shape runs >= 0.28 m/s, so the
        # blend weight is exactly zero there.
        self.hold_speed      = dbl('hold_speed', 0.05)
        self.hold_radius     = dbl('hold_radius', 0.5)
        self.los_hold_kx     = dbl('los_hold_kx', 1.0)
        self.los_hold_umax   = dbl('los_hold_umax', 0.8)

        # -- point following (manual target and pinger; split sim/real) ------
        self.k_v   = dbl('point_k_v',   2.0  if self.tuning_sim else 0.15)
        self.k_psi = dbl('point_k_psi', 60.0 if self.tuning_sim else 100.0)
        # Pinger-branch arrival check only; negative disables it.
        self.safety_distance = dbl('safety_distance', 1.0 if self.tuning_sim else -1.0)

        # -- manual-target keep-location hold (manual_keep_location) ---------
        # On station inside manual_hold_radius, pursuit again beyond
        # manual_reacquire_radius. kx is in N per metre (solve_LoS writes
        # Newtons), chosen so the hold meets the pursuit law at the handover
        # (8.00 vs 8.38 N real, 15.00 vs 15.42 N sim). manual_hold_radius <= 0
        # disables the hold and restores the old arrival latch exactly.
        self.manual_hold_radius      = dbl('manual_hold_radius', 1.0)
        self.manual_reacquire_radius = dbl('manual_reacquire_radius', 1.5)
        self.manual_hold_kx   = dbl('manual_hold_kx', 15.0 if self.tuning_sim else 8.0)
        # Derived by default, so retuning a radius cannot break the handover.
        self.manual_hold_umax = dbl(
            'manual_hold_umax',
            self.manual_hold_kx * max(0.0, self.manual_reacquire_radius
                                           - self.manual_hold_radius))
        self.manual_brake_time = dbl('manual_brake_time', 1.0)   # s of reverse on arrival

        # -- MPC (split simulation/real, plant model included) ---------------
        # Real column: in-tree since the beginning, provenance unknown. Sim
        # column: fitted to blueboat_description/urdf/hydrodynamics.xacro.
        # Added mass = the xacro's xDotU/yDotV/nDotR; damping is a SECANT
        # linearisation of its linear + quadratic drag (the acados model is
        # linear-only):  d_u = 25.15 + 33.800*0.45 m/s,
        # d_v = 7.364 + 54.269*0.10 m/s,  d_r = 3.744 + 40.000*0.15 rad/s.
        # Refit if the xacro changes or missions cruise far from 0.45 m/s.
        if not self.tuning_sim:
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

        # Horizon: 6.0 s / 30 in simulation (measured: circle offset -1.019 ->
        # -0.011 m); the real boat stays at 2.5 s / 15 until the solve time has
        # been timed in a field run (a longer horizon risks the 50 ms budget).
        self.mpc_horizon = integer('mpc_horizon', 30 if self.tuning_sim else 15)
        self.mpc_time    = dbl('mpc_time', 6.0 if self.tuning_sim else 2.5)

        self.Q_weight = np.diag(arr('mpc_Q_diag', [50.0,   # x
                                                   50.0,   # y
                                                   30.0,   # psi
                                                    1.0,   # u
                                                    1.0,   # v
                                                    1.0])) # r
        # Absolute effort penalty (u_ref = 0, no rate term). At Q_pos = 50 full
        # throttle on both thrusters costs what a position error of 0.49 m
        # (R 0.015) / 1.26 m (R 0.10) does.
        self.R_weight = np.diag(arr('mpc_R_diag', [0.10, 0.10] if self.tuning_sim
                                                  else [0.015, 0.015]))

        # qpOASES working-set budget; 0 = derive max(50, 4*nu*N) in ur_mpc.
        # acados' default 50 sits below the 60 variables of the sim horizon,
        # which made saturating solves fail (finding C6, src/CLAUDE.md).
        self.mpc_qp_iter_max = integer('mpc_qp_iter_max', 0)

        # -- thrust limit: one symmetric scalar for every exit ---------------
        self.thrust_limit = dbl('thrust_limit', 20.0)
        limit = self.thrust_limit
        self.input_bounds = {"lower": np.array([-limit, -limit]),
                             "upper": np.array([ limit,  limit]),
                             "idx":   np.array([0, 1])}

        # -- propeller breakaway floor on solve_LoS's surge (0.0 disables) ---
        # 0-2 N sits inside the T200 ESC's neutral deadband; src/CLAUDE.md §5.
        self.min_thrust = dbl('min_thrust', 2.0)

        # -- /path_request health (wedge detectors, not jitter detectors) ----
        # timeout: re-issue a request whose answer never came.
        # stale:   stop advancing tau against a window older than this.
        self.path_request_timeout = dbl('path_request_timeout', 1.0)
        self.path_stale_timeout   = dbl('path_stale_timeout', 1.0)

        # -- arrow-control dead-man: an older command is zero, arrow control
        #    stays engaged (never hands back to autonomy by itself).
        self.arrow_timeout = dbl('arrow_timeout', 0.5)

        # -- the clock of the world-timing logic (additive, 2026-11-07) ------
        # 'wall' (default, every real-robot launch): time.time(), as before.
        # 'ros': the node clock -- /clock in a simulation -- so a run slowed
        # below real time (Gazebo real-time factor < 1, for the ray-cast
        # sonar) brakes, retries and stamps /monitoring_data and the .npy
        # exactly as at real speed. The arrow dead-man stays wall in both:
        # it guards a human key stream sent at a wall 20 Hz.
        self.time_source = self._param('time_source', 'wall').string_value
        if self.time_source not in ('wall', 'ros'):
            raise ValueError(f"unknown time_source '{self.time_source}' (wall | ros)")

    # ======================================================================
    #  3. THE CONTROL LOOP -- 1/dt (20 Hz), the entry point
    #  Every early return publishes zero rather than falling silent: a
    #  consumer that hears nothing keeps the last value it heard.
    # ======================================================================

    def timer_callback(self):
        # A session's MPC rebuild is swapped in here, on the executor thread.
        self._poll_controller_build()

        if not self.ready:
            self.publish_thrust([0.0, 0.0])
            return

        if not self.time_set:
            self.initial_time = self._world_clock()
            self.tau = 0.0
            self.time_set = True

        current_time = self._world_clock() - self.initial_time

        if self.current_pose is None or self.current_twist is None:
            self.publish_thrust([0.0, 0.0])
            return

        current_state = np.array([self.current_pose[0],    # x
                                  self.current_pose[1],    # y
                                  self.current_pose[5],    # yaw
                                  self.current_twist[0],   # u (body surge)
                                  self.current_twist[1],   # v (body sway)
                                  self.current_twist[5]])  # r

        manual_active = (list(self.manual_target) != [0.0, 0.0])
        # Arrow control pauses the mission like a manual target: tau holds.
        arrow_active = self.arrow_active

        # Reference update (parameter-governed, never wall-clock). Only during
        # a run, and only when following a path.
        if self.path_source == 'replanner' and not self.use_pinger and self.run_active:
            self._replanner_reference_step(current_time, current_state,
                                           manual_active, arrow_active)
        elif self._follows_path() and self.run_active:
            self._mission_reference_step(current_time, current_state,
                                         manual_active or arrow_active)

        command = self._compute_command(current_state, current_time,
                                        manual_active, arrow_active)
        if command is None:
            self._log_nothing_to_target()
            self.publish_thrust([0.0, 0.0])
            return
        u, target, world_target = command

        # Kept for arrow control, which logs the last target registered before
        # the keys took over.
        if world_target is not None and not arrow_active:
            self.last_world_target = [float(v) for v in list(world_target)[:3]]

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

        # The monitoring row records the saturated command actually sent.
        u = self.publish_thrust(u)

        if self.pinger_target is not None and self.use_pinger:
            self.get_logger().info(f'\nPinger coordinates robot frame: \n{self.pinger_target}')
        if manual_active:
            target_str = ", ".join(f"{float(x):.2f}" for x in list(self.manual_target))
            self.get_logger().info(f'\nManual target coordinates: \n{target_str}')

        self._record_monitoring(current_time, current_state, u,
                                world_target if world_target is not None else target)

    def _mission_reference_step(self, current_time, current_state, overridden):
        """Fixed-mission reference: collect the /path_request answer, advance
        the governor on a fresh window (held while `overridden` by a manual
        target or arrow control), and issue the next request at tau."""
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
            # A lost response must not wedge the node with a frozen window.
            self.get_logger().warning(
                f"No answer to /path_request within {self.path_request_timeout:.1f} s "
                "- retrying.", throttle_duration_sec=5.0)
            self.future = None
            self.request_sent = None

        # Only against a FRESH window: advancing against a stale one is open
        # loop (the boat reaches the frozen target, the along factor unclips
        # and tau integrates at full rate -- the wall-clock reference N8
        # removed). Holding tau keeps station on the last good target.
        window_age = current_time - self.path_rx_time if self.path_rx_time is not None else None
        window_fresh = window_age is not None and window_age <= self.path_stale_timeout
        if self.controller_path.poses and not overridden:
            if window_fresh:
                e_along, e_y = self.path_progress_errors(self.controller_path, current_state)
                self.advance_governor(e_along, e_y)
            else:
                self.get_logger().warning(
                    f"Path window stale ({window_age:.2f} s) - holding tau at "
                    f"{self.tau:.2f}.", throttle_duration_sec=5.0)

        if self.future is None:
            request = RequestPath.Request()
            request.path_request.data = np.linspace(self.tau,
                                                     self.tau + self.path_time,
                                                     int(self.path_steps), dtype=float)
            # What accept_path() checks the bare response against.
            self.tau_requested = float(np.float32(self.tau))
            self.request_sent = current_time
            self.future = self.client.call_async(request)
        # else: still pending - keep controlling on the last window

    def _compute_command(self, current_state, current_time, manual_active, arrow_active):
        """(u, target, world_target) of the active branch, or None if nothing
        is to be targeted. Priority: arrows, manual target, path following,
        replanner awaiting-hold, pinger. May zero current_state[:3] (pinger PID
        runs body-frame), as the monitoring row has always recorded."""
        if (arrow_active or manual_active) and self.pinger_hold is not None:
            # The operator took over: a later hold latches where they leave the boat.
            self.pinger_hold = None
        if arrow_active:
            age = time.time() - self.arrow_rx if self.arrow_rx is not None else math.inf
            if age <= self.arrow_timeout:
                u = list(self.arrow_cmd)
            else:
                # Dead-man: zero, and STAY in arrow control.
                u = [0.0, 0.0]
                self.get_logger().warning(
                    f"Arrow control: no command for {age:.2f} s "
                    f"(arrow_timeout {self.arrow_timeout:.2f} s) - zero thrust.",
                    throttle_duration_sec=2.0)
            # --- world-frame monitoring target ---
            # The last target registered before the keys took over (the pose at
            # that moment if none yet).
            if self.last_world_target is None:
                self.last_world_target = [float(current_state[0]),
                                          float(current_state[1]),
                                          float(current_state[2])]
            world_target = list(self.last_world_target)
            return u, list(world_target), world_target

        if manual_active:
            return self._hold_at(current_state, current_time, *self.manual_target[:2], 0)

        if self.run_active and self.controller_path.poses:
            return self._path_following_command(current_state)

        if self.run_active and self._awaiting_hold_active():
            # Replanner run before its first path (section 7): hold the start pose.
            return self._hold_at(current_state, current_time, *self.path_updater.hold)

        if self.run_active and self.use_pinger:
            ok, why = self._pinger_chase_ok()
            if ok:                      # ok implies a finite, non-zero pinger_target
                if self.pinger_hold is not None:
                    self.get_logger().info("Pinger estimate tracking again - resuming pinger following.")
                    self.pinger_hold = None
                return self._pinger_command(current_state, current_time)
            return self._pinger_hold_command(current_state, current_time, why)

        return None

    def _follows_path(self):
        """Does this run follow a path (and so request one)? Not when
        following the pinger, nor without a path (has_path False, off or log)."""
        return not self.use_pinger and self.has_path

    def _set_pinger_mode(self, mode, has_path):
        """The one place the run mode is set. `use_pinger` (what the v1.0 code
        reads) is derived: True only in follow mode."""
        self.pinger_mode, self.has_path = mode, has_path
        self.use_pinger = mode == 'follow'

    def _pinger_chase_ok(self):
        """(ok, reason): may the follow law steer to /blueboat/pinger_coordinates?
        Only on a fresh estimate whose status is tracking or coasting."""
        est, rx = self.pinger_estimate, self.pinger_estimate_rx
        if est is None or rx is None:
            return False, "no estimate from pinger_estimator yet"
        age = self._world_clock() - rx
        if age > self.pinger_stale_timeout:
            return False, f"estimate {age:.1f} s old (pinger_stale_timeout {self.pinger_stale_timeout:.1f} s)"
        # The status of the source /blueboat/pinger_coordinates carries (the
        # filter's, or the legacy input's with pinger_source legacy); an
        # older estimator only has `status`.
        status = est.get('source_status') or est.get('status')
        if status not in PINGER_CHASE_STATUSES:
            return False, f"pinger estimate is '{status}'"
        tgt = self.pinger_target
        if tgt is None or len(tgt) < 2:
            return False, "no pinger vector yet"
        if not all(math.isfinite(float(v)) for v in tgt[:2]):
            return False, "pinger vector is not finite"
        if float(tgt[0]) == 0.0 and float(tgt[1]) == 0.0:
            return False, "pinger vector is zero (not detected)"
        return True, ""

    def _pinger_hold_command(self, current_state, current_time, why):
        """Follow mode with no usable estimate: hold the position where the
        estimate was lost (manual-target keep-location law, as the replanner's
        awaiting-hold), until the estimate is tracking again."""
        if self.pinger_hold is None:
            self.pinger_hold = (float(current_state[0]), float(current_state[1]),
                                float(current_state[2]))
            self._clear_manual_hold()   # no manual target here (it has priority)
            self.get_logger().warning(
                f"Pinger following paused ({why}): holding position at "
                f"({self.pinger_hold[0]:.2f}, {self.pinger_hold[1]:.2f}).")
        else:
            self.get_logger().warning(f"Pinger following paused: {why}.", throttle_duration_sec=5.0)
        return self._hold_at(current_state, current_time, *self.pinger_hold)

    def _hold_at(self, current_state, current_time, x, y, logged_yaw):
        """Hold the world point (x, y) with the manual-target keep-location law
        (a manual target, the replanner's awaiting-hold, the pinger hold).
        `logged_yaw` is the yaw the monitoring row records for it."""
        world_target = [x, y, logged_yaw]  # --- world-frame monitoring target ---
        target = fm.inRobotFrame(current_state, [x, y, 0, 0, 0, 0])
        return self.solve_LoS(target, current_time, keep_location=True), target, world_target

    def _path_following_command(self, current_state):
        """Path following on the held reference window (MPC / PID / LoS)."""
        u = [0]*2
        if self.isSimulation:
            cf.create_pose_marker(self.controller_path.poses[0].pose, self.pose_arrow_publisher)

        # --- world-frame monitoring target ---: poses[0] in every branch, NOT
        # cf.compute_target's result (the window's SECOND pose), N9.
        world_target = _reference_pose(self.controller_path)

        if self.controller_type == 'MPC':
            u = self.controller.solve(path=self.controller_path, x_current=current_state)
            # A failed solve is not a command (C6): ur_mpc already returns
            # zeros; report it through the ROS logger (stdout is not captured),
            # and fall through so /thruster_input and the .npy keep flowing.
            if self.controller.last_status != 0:
                u = [0.0, 0.0]
                self.get_logger().error(
                    f"MPC solve FAILED (acados status "
                    f"{self.controller.last_status}, "
                    f"{self.controller.fail_count} consecutive, "
                    f"{self.controller.total_failures} total) - commanding "
                    "ZERO thrust. The boat will drift.",
                    throttle_duration_sec=1.0)
            elif self.controller.last_solve_time > 0.5 * self.dt:
                # The 20 Hz budget is untimed on the real boat; this makes an
                # overrun visible from /rosout.
                self.get_logger().warning(
                    f"MPC solve took "
                    f"{self.controller.last_solve_time * 1e3:.1f} ms of the "
                    f"{self.dt * 1e3:.0f} ms tick.", throttle_duration_sec=5.0)
            return u, world_target, list(world_target)

        target = cf.compute_target(self.controller_path, self.dt)

        if self.controller_type == 'PID':
            # Path tangent (target[2]) and authored speed (target[3]) feed LoS
            # steering and speed feedforward.
            psi_path, slow = target[2], False
            w = 0.0
            if self.hold_speed > 0.0:
                w = 1.0 - min(1.0, max(0.0, target[3] / self.hold_speed))
            if w > 0.0 and self.hold_radius > 0.0:
                # Stationary reference: rotate the tangent handed to PIDLoS
                # toward the hold point, so its along-track error becomes the
                # range; fades out inside hold_radius (src/CLAUDE.md §4).
                rng = math.hypot(target[0] - current_state[0],
                                 target[1] - current_state[1])
                g = min(1.0, max(0.0, (rng - self.hold_radius) / self.hold_radius))
                if g > 0.0:
                    bearing = math.atan2(target[1] - current_state[1],
                                         target[0] - current_state[0])
                    psi_path = target[2] + w * g * PID.wrap_angle(bearing - target[2])
                    slow = True
            u = self.controller.compute(current_state, target[:3],
                                          u_ff=target[3], psi_path=psi_path,
                                          slow_on_turn=slow)

        if self.controller_type == 'LoS':
            u = self.los_guidance(target, current_state)

        return u, target, world_target

    def _pinger_command(self, current_state, current_time):
        """Pinger following (PID or LoS; MPC is not supported)."""
        u = [0]*2
        # --- world-frame monitoring target ---
        px, py = float(self.pinger_target[0]), float(self.pinger_target[1])
        c_m, s_m = np.cos(current_state[2]), np.sin(current_state[2])
        world_target = [current_state[0] + c_m*px - s_m*py,
                        current_state[1] + s_m*px + c_m*py, 0.0]
        if self.controller_type == 'PID':
            # Body-frame solve: the boat at the origin, heading zero.
            target = [*self.pinger_target[:2], 0]
            current_state[[0,1,2]] = 0
            u = self.controller.compute(current_state, target)

        if self.controller_type == 'LoS':
            target = self.pinger_target
            u = self.solve_LoS(target, current_time, keep_location=False)

        # /controller_target: body frame, pinger branch only (N9).
        msg = Float32MultiArray()
        msg.data = [float(v) for v in target]
        self.target_publisher.publish(msg)
        return u, target, world_target

    def _log_nothing_to_target(self):
        if not self.session_mode:
            self.get_logger().info('Nothing to target yet.')
        elif self.run_active:
            self.get_logger().info('Nothing to target yet.', throttle_duration_sec=5.0)
        else:
            self.get_logger().info(
                'Session idle - no run active, holding zero thrust '
                '(manual target or arrow keys to move).',
                throttle_duration_sec=30.0)

    def _record_monitoring(self, current_time, current_state, u, monitored):
        """Publish /monitoring_data and, during a run, append the .npy row."""
        # --- world-frame monitoring target ---
        psi_d = monitored[2] if len(monitored) > 2 else 0.0
        # In a session, t counts from the start of the run being recorded.
        t_m = current_time
        if self.session_mode and self.run_t0 is not None:
            t_m = self._world_clock() - self.run_t0

        data_array = [t_m, current_state[0], current_state[1], current_state[2],
                      monitored[0], monitored[1], psi_d, u[0], u[1]]
        if self.run_active:
            self.monitoring.append(data_array)

        publisher_msg = Float32MultiArray()
        publisher_msg.data = [float(v) for v in data_array]
        self.data_publisher.publish(publisher_msg)

    # ======================================================================
    #  4. GUIDANCE -- path governor (N8) and the steering laws
    # ======================================================================

    def warn_if_multiple_path_servers(self):
        """
        Say so, loudly, if more than one node offers /path_request: every
        request is then answered by whichever replies first and the reference
        alternates between trajectories. path_generation refuses to be the
        second server; this covers one started with allow_duplicate_server.
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
        Is this response an answer to OUR request? (A second /path_request
        server answers some requests; the bare Path cannot say which.)

          length   the window has as many poses as we asked for;
          tag      poses[0] is stamped with the parameter we requested
                   (path_stamp.encode in path_generation; RequestPath unchanged).

        A server predating the tag stamps the clock: detected once, reported,
        then a geometric plausibility check (the first pose cannot jump further
        than tau could have moved). Rejection keeps the last good window; the
        staleness gate stops the boat driving on it indefinitely.
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
        From the current path window (poses[0] = virtual target at tau), the
        signed along-track gap boat->target (target ahead > 0) and the signed
        cross-track error of the boat, along the path tangent at the target.
        """
        p0 = path.poses[0].pose
        x0, y0 = p0.position.x, p0.position.y
        gamma_p = cf.quaternion_to_yaw(p0.orientation)
        xb, yb = state[0], state[1]
        c, s = math.cos(gamma_p), math.sin(gamma_p)
        e_along =  (x0 - xb) * c + (y0 - yb) * s
        e_y     = -(xb - x0) * s + (yb - y0) * c
        return e_along, e_y

    def advance_governor(self, e_along, e_y):
        """
        Advance tau: at the authored speed while the boat keeps up, slowing to
        a pause as the along-track gap (and, if gov_Emax > 0, the cross-track
        error) grows. Both factors are clipped to [0, 1] and multiplied, so tau
        is monotonic and never faster than the path's own parameterisation --
        which is what makes a varying authored speed profile followed for free.
        Nothing may scale tau_dot above unity.
        """
        span_along = max(1e-6, (self.gov_Lmax - self.gov_Lmin))
        fac_along = np.clip((self.gov_Lmax - e_along) / span_along, 0.0, 1.0)

        if self.gov_Emax > 0.0:
            span_cross = max(1e-6, (self.gov_Emax - self.gov_Emin))
            fac_cross = np.clip((self.gov_Emax - abs(e_y)) / span_cross, 0.0, 1.0)
        else:
            fac_cross = 1.0

        tau_dot = self.path_speed_scale * (fac_along * fac_cross)
        self.tau += tau_dot * self.dt

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

        # Zero-authored-speed hold; w is exactly 0 on any moving path.
        w = 0.0
        if self.hold_speed > 0.0:
            w = 1.0 - min(1.0, max(0.0, U_d / self.hold_speed))

        u_hold = 0.0
        if w > 0.0:
            rng = math.hypot(x_ref - x, y_ref - y)
            gap = max(0.0, rng - self.hold_radius)
            if gap > 0.0:
                # Steer at the point; never reverse (a lookahead law steers the
                # wrong way backwards), the yaw channel turns the boat round.
                bearing = math.atan2(y_ref - y, x_ref - x)
                psi_d = psi_d + w * PID.wrap_angle(bearing - psi_d)
                u_hold = min(self.los_hold_umax, w * self.los_hold_kx * gap)

        psi_err = PID.wrap_angle(psi_d - psi)

        u_cmd = (self.los_speed_scale * U_d + u_hold) * max(0.0, math.cos(psi_err))

        X = self.los_ku * (u_cmd - u)
        N = self.los_kpsi * psi_err - self.los_kd * r

        thrusts = self.los_allocator.allocate(np.array([X, 0.0, N]))
        return thrusts

    def solve_LoS(self, target, current_time, keep_location):
        """Point line-of-sight to a body-frame target. keep_location: the
        manual-target variant (a point held: manual target, awaiting-hold,
        pinger hold); False: the pinger pursuit law."""
        x,y,z = target

        bearing = np.arctan2(y, x)
        yaw_rate = self.k_psi * bearing
        d = np.sqrt(x**2+y**2)
        v = self.k_v * d
        v = 5*np.log(v+1)

        manual = keep_location
        if manual:
            v = 10*np.log(v+1) if not self.tuning_sim else 7*np.log(v+1)  # If manual target, go faster. Don't need to be that precise here. (real / sim-tuned column, gain_profile)

        # Propeller breakaway floor on the common-mode SURGE only (the yaw
        # differential is untouched): on the real boat 5*ln(k_v*d + 1) stays
        # inside the ESC deadband out to 3.28 m. g fades it in over hold_radius
        # (continuous in d); max(0, cos) drops it for a target abeam or behind,
        # where forward surge would push the boat away. It never lowers v
        # (v >= 0); min_thrust = 0.0 restores the original law. src/CLAUDE.md §5.
        if self.min_thrust > 0.0 and self.hold_radius > 0.0:
            g = min(1.0, max(0.0, (d - self.hold_radius) / self.hold_radius))
            breakaway = self.min_thrust * g * max(0.0, np.cos(bearing))
            if 0.0 < v < breakaway:
                v = breakaway

        # A reached manual target is held (manual_keep_location); everything
        # after this block is the pinger branch's arrival latch.
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

        # Arrived: briefly reverse, then stop.
        else:
            if current_time - self.stopping_time < 1.0:
                thruster_input = [-1.,-1.]
            else:
                thruster_input = [0.,0.]

        return thruster_input

    def manual_keep_location(self, d, bearing, yaw_rate, current_time):
        """
        Hold station on a REACHED manual target: the thruster pair while
        holding, or None when off station (the caller runs plain pursuit).

        Re-evaluated every tick, never latched, with hysteresis:
            not holding, d <= manual_hold_radius       -> hold
            holding,     d >  manual_reacquire_radius  -> resume pursuit
        Inside the hold the surge is proportional to the gap outside
        manual_hold_radius, capped, max(0, cos(bearing))-shaped and floored
        to min_thrust; the yaw differential is the pursuit law's. Rationale and
        measurements: src/CLAUDE.md §5.
        """
        if self.manual_hold_radius <= 0.0:
            return None          # hold disabled: the pursuit law, as it was

        if not self.manual_hold:
            if d <= self.manual_hold_radius:
                self.manual_hold = True
                self.manual_brake_t0 = current_time
                self.get_logger().info(
                    f"Manual target reached at {d:.2f} m - holding location "
                    f"(re-acquiring beyond {self.manual_reacquire_radius:.2f} m)")
        elif d > self.manual_reacquire_radius:
            self._clear_manual_hold()
            self.get_logger().info(
                f"Pushed {d:.2f} m off the manual target - re-acquiring it")

        if not self.manual_hold:
            return None

        # Arrival brake: the original astern pulse.
        if (self.manual_brake_t0 is not None
                and current_time - self.manual_brake_t0 < self.manual_brake_time):
            return [-1., -1.]

        gap = max(0.0, d - self.manual_hold_radius)
        v_hold = min(self.manual_hold_umax, self.manual_hold_kx * gap)
        v_hold *= max(0.0, np.cos(bearing))

        # Breakaway floor, written out (not keyed to hold_radius, which is
        # shared with los_guidance/PID). It cannot lift a zero command; without
        # it the boat parks at 1.25 m instead of 1.00 m.
        if self.min_thrust > 0.0:
            floor = self.min_thrust * max(0.0, np.cos(bearing))
            if 0.0 < v_hold < floor:
                v_hold = floor

        return [v_hold + 0.295 * yaw_rate/2, v_hold - 0.295 * yaw_rate/2]

    # ======================================================================
    #  5. CALLBACKS AND SMALL HELPERS -- nothing here computes control
    # ======================================================================

    def _clear_manual_hold(self):
        """Reset the keep-location state (not the target it refers to)."""
        self.manual_hold = False
        self.manual_brake_t0 = None

    def odom_callback(self, msg: Odometry):
        self.current_pose, self.current_twist = cf.odometry(msg)

    def pinger_callback(self, msg: Float32MultiArray):
        self.pinger_target = msg.data

    def pinger_estimate_callback(self, msg: String):
        try:
            est = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(est, dict):
            self.pinger_estimate = est
            self.pinger_estimate_rx = self._world_clock()

    def path_info_callback(self, msg: String):
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(info, dict):
            self.path_info = info
            self.path_complete_logged = False

    def path_complete(self):
        """Has the run's (non-loop) path ended? The boat then holds the final
        pose until STOP RUN (yaml_trajectory clamps; the controllers hold)."""
        if not self.run_active or self.use_pinger:
            return False
        if self.path_source == 'replanner':
            plan = self.path_updater.plan
            return bool(plan) and plan.t_end is not None and self.tau >= plan.t_end
        if not self._follows_path():
            return False
        info = self.path_info
        if not info or not info.get('loaded') or info.get('loop'):
            return False
        duration = info.get('duration_s')
        return duration is not None and self.tau >= float(duration)

    def ready_callback(self, msg: Bool):
        # Readiness is re-published periodically; log only the transition.
        if msg.data and not self.ready:
            self.get_logger().info('Controller ready')
        self.ready = msg.data

    def manual_target_callback(self, msg: Float32MultiArray):
        self.manual_target = msg.data # [x,y] in world frame
        self.stopping_sequence = False

        # A newer operator intent than the arrow keys: releases arrow control
        # (the station's "hold where released" publishes the boat's position).
        if self.arrow_active:
            self.arrow_active = False
            self.get_logger().info("Arrow control released by a manual target.")

        # Reset the keep-location state only when the target MOVES, so a
        # re-published target does not re-arm the arrival brake. The [0, 0]
        # resume sentinel differs from any real target and clears it.
        target = [round(float(value), 6) for value in list(self.manual_target)[:2]]
        if target != self.manual_hold_target:
            self.manual_hold_target = target
            self._clear_manual_hold()

    def arrow_thrust_callback(self, msg: Float32MultiArray):
        """
        Station arrow keys: [right, left] N at 20 Hz while driving, [] to
        release. While engaged it overrides every branch and pauses a run; it
        goes out through publish_thrust, so this node stays the only
        /thruster_input producer. Released by [], a manual target, a run start.
        """
        data = list(msg.data)
        if not data:
            if self.arrow_active:
                self.get_logger().info("Arrow control released.")
            self.arrow_active = False
            self.arrow_cmd = [0.0, 0.0]
            return
        if len(data) != 2:
            self.get_logger().warning(
                f"/blueboat/arrow_thrust expects [right, left] or [], got "
                f"{len(data)} values - ignored.", throttle_duration_sec=5.0)
            return
        if not self.arrow_active:
            self.get_logger().info("Arrow control engaged - autonomous control paused.")
        self.arrow_active = True
        self.arrow_cmd = [float(data[0]), float(data[1])]
        self.arrow_rx = time.time()

    def publish_thrust(self, u):
        """
        Publish /thruster_input -- the single exit for thrust, also used to say
        "zero" explicitly. Saturates UNIFORMLY to +/- thrust_limit (one factor
        for both sides, so the wrench direction survives; thrust_limits.py).
        Binds in practice only on solve_LoS, which bypasses the allocator.
        Returns what went on the wire, so monitoring and the CSV agree.
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
        return limited

    def save_monitoring(self):
        """
        Write the controller log, on its own timer and only when rows were
        added. Only the new rows are converted to the file's <U32 strings
        (converting the whole list took ~0.5 s per write after an hour of run,
        on the control executor); the bytes are what np.save of the list gives.
        """
        if self.title is None:
            return                      # a session before its first run: no file
        rows = len(self.monitoring)
        if rows == self.monitoring_saved_rows:
            return                      # nothing new since the last write
        try:
            chunk = np.array(self.monitoring[self.monitoring_saved_rows:], dtype='<U32')
            # The header alone is saved as np.save always did (<U5).
            np.save(self.title, np.concatenate(self.monitoring_chunks + [chunk])
                    if rows > 1 else self.monitoring)
            self.monitoring_chunks.append(chunk)
            self.monitoring_saved_rows = rows
        except Exception as exc:
            self.get_logger().error(f"Could not write {self.title}.npy: {exc}",
                                    throttle_duration_sec=10.0)

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9

    # ======================================================================
    #  6. RUN SESSION  (session_mode only; JSON contracts: CLAUDE.md §2.3)
    #  The launch idles -- manual target and arrows only, nothing followed or
    #  recorded -- until 'start'. /blueboat/run_state answers every command
    #  and is heartbeated at 1 Hz; robot_interface opens the run's CSV from it.
    # ======================================================================

    def run_command_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            if not isinstance(cmd, dict):
                raise ValueError("not a JSON object")
        except ValueError as exc:
            self.get_logger().error(f"/blueboat/run_command: unreadable ({exc}) - ignored.")
            return

        name = cmd.get('cmd')
        handlers = {'configure': self._configure,
                    'start': self._start_run,
                    'stop': self._stop_run}
        handler = handlers.get(name)
        if handler is None:
            self._answer(cmd, False, f"unknown command '{name}'")
            return
        try:
            handler(cmd)
        except Exception as exc:                       # noqa: BLE001 - reported, never fatal
            self.get_logger().error(f"Run command '{name}' failed: {type(exc).__name__}: {exc}")
            self._answer(cmd, False, f"'{name}' failed: {exc}")

    def _answer(self, cmd, ok, detail=''):
        """Record the outcome of one command and publish it at once."""
        self.cmd_seq = cmd.get('seq')
        self.cmd_ok = bool(ok)
        self.cmd_detail = str(detail)
        if not ok:
            self.get_logger().warning(f"Run command refused: {detail}")
        self._publish_run_state()

    def _configure(self, cmd):
        """Choose controller, follow mode, log note and mission for the NEXT run.

        Refused during a run. PID/LoS are rebuilt here; MPC builds on a worker
        thread (state 'loading') and is swapped in by _poll_controller_build.
        """
        if self.run_active:
            self._answer(cmd, False, "a run is active - stop it before changing the mission")
            return
        if self._build_future is not None:
            self._answer(cmd, False, "a controller build is still in progress")
            return

        ctype = str(cmd.get('controller_type', self.controller_type))
        # Pinger mode (additive): `pinger_mode` wins; else `use_pinger`
        # (True = follow, False = off); else unchanged.
        try:
            if cmd.get('pinger_mode') not in (None, ''):
                mode = rls.pinger_mode_of(cmd.get('pinger_mode'))
            elif 'use_pinger' in cmd:
                mode = 'follow' if bool(cmd.get('use_pinger')) else 'off'
            else:
                mode = self.pinger_mode
        except ValueError as exc:
            self._answer(cmd, False, str(exc))
            return
        has_path = bool(cmd.get('has_path', True))
        # Section 7, additive: absent = unchanged.
        path_source = str(cmd.get('path_source', self.path_source))
        problem = _run_config_problem(ctype, mode, has_path, path_source)
        if problem:
            self._answer(cmd, False, problem)
            return
        self.path_source = path_source
        self.pinger_source = str(cmd.get('pinger_source', self.pinger_source))

        self.run_note = str(cmd.get('note', self.run_note))
        self.run_mission = str(cmd.get('mission', self.run_mission))

        if ctype == self.controller_type and self.run_status != 'error':
            self._set_pinger_mode(mode, has_path)
            self.run_status = 'idle'
            self._answer(cmd, True, f"{ctype} ready")
            return

        if ctype == 'MPC':
            self.run_status = 'loading'
            self._build_mode = (mode, has_path)
            if self._build_pool is None:
                self._build_pool = ThreadPoolExecutor(max_workers=1,
                                                      thread_name_prefix='mpc_build')
            self._build_future = self._build_pool.submit(self._make_controller, 'MPC')
            self.get_logger().info("Loading MPC for the next run (built off the control loop).")
            self._answer(cmd, True, "MPC loading")
            return

        # PID / LoS: immediate, on this thread.
        self._apply_controller_setup(ctype)
        self.controller = self._make_controller(ctype)
        self.controller_type = ctype
        self._set_pinger_mode(mode, has_path)
        self.run_status = 'idle'
        self.get_logger().info(f"Controller switched to {ctype} for the next run.")
        self._answer(cmd, True, f"{ctype} ready")

    def _poll_controller_build(self):
        """Swap a finished MPC build in, on the executor thread."""
        future = self._build_future
        if future is None or not future.done():
            return
        self._build_future = None
        try:
            controller = future.result()
        except Exception as exc:                       # noqa: BLE001 - reported
            # Previous controller stays loaded; 'error' refuses start until
            # the station reconfigures.
            self.run_status = 'error'
            self.cmd_ok = False
            self.cmd_detail = f"MPC build failed: {type(exc).__name__}: {exc}"
            self.get_logger().error(self.cmd_detail)
            self._publish_run_state()
            return
        self._apply_controller_setup('MPC')
        self.controller = controller
        self.controller_type = 'MPC'
        self._set_pinger_mode(*self._build_mode)    # set with every submit
        self.run_status = 'idle'
        self.cmd_ok = True
        self.cmd_detail = "MPC ready"
        self.get_logger().info("MPC loaded for the next run.")
        self._publish_run_state()

    def _start_run(self, cmd):
        """Start following the configured path or pinger, from a clean slate:
        tau 0 (the station has just written the run's path), no window, no
        controller memory, no manual target or arrow control, a fresh .npy.
        """
        if self.run_active:
            self._answer(cmd, False, "a run is already active")
            return
        if self.run_status != 'idle':
            self._answer(cmd, False, f"controller not ready ({self.run_status})")
            return

        # A run that cannot record must not start.
        ctrl = self.controller_type
        try:
            title = self._reserve_npy(ctrl)
        except OSError as exc:
            self._answer(cmd, False, f"cannot create the controller log: {exc}")
            return
        self.title = title
        self.monitoring = [list(NPY_HEADER)]
        self.monitoring_saved_rows = 0
        self.monitoring_chunks = []

        self._clear_run_reference()
        self.arrow_active = False
        self.arrow_cmd = [0.0, 0.0]
        self.stopping_time = None
        self.tau = 0.0
        self.tau_requested = None
        self.last_world_target = None
        self.path_complete_logged = False

        # Controller memory from the previous run must not leak into this one.
        if ctrl == 'PID' and self.controller is not None:
            self.controller.reset()
        if ctrl == 'MPC' and self.controller is not None:
            # As freshly built: zero iterate and QP memory.
            self.controller.solver.reset()
            self.controller.fail_count = 0

        self.run_number += 1
        self.run_t0 = self._world_clock()
        self.run_active = True
        self.run_status = 'running'
        # A new run never inherits a path (a replanner run starts holding).
        self._path_update_reset('run_started')
        logged = self.pinger_mode == 'log'
        if self.use_pinger:
            follow = 'pinger'
        elif not self.has_path:         # never a replanner run (refused at launch / configure)
            follow = 'nothing (no path, pinger logged)' if logged else 'nothing (no path)'
        else:
            follow = 'path (pinger logged)' if logged else 'path'
        self.get_logger().info(
            f"RUN {self.run_number} STARTED - {ctrl}, {follow} following"
            f"{' (' + self.run_mission + ')' if self.run_mission else ''}. "
            f"Controller log: {self.title}.npy")
        self._answer(cmd, True, f"run {self.run_number} started")

    def _clear_run_reference(self):
        """What no run outlives: the manual target and its hold, the path
        window and its pending request, the pinger hold."""
        self.manual_target = [0.0, 0.0]
        self._clear_manual_hold()
        self.manual_hold_target = None
        self.stopping_sequence = False
        self.controller_path = Path()
        self.future = None
        self.request_sent = None
        self.path_rx_time = None
        self.pinger_hold = None

    def _stop_run(self, cmd):
        """End the run: stop following, write the log, zero thrust, stay in override."""
        if not self.run_active:
            self._answer(cmd, True, "no run active")
            return
        self.run_active = False
        self.run_status = 'idle'
        self.save_monitoring()

        # Arrow control, a station toggle, is left alone.
        self._clear_run_reference()
        self._path_update_reset('run_stopped')

        self.get_logger().info(
            f"RUN {self.run_number} STOPPED - controller log {self.title}.npy written; "
            "holding zero thrust (still in override).")
        self._answer(cmd, True, f"run {self.run_number} stopped")
        self.publish_thrust([0.0, 0.0])

    def _publish_run_state(self):
        if not self.session_mode:
            return
        state = {
            'seq': self.cmd_seq,
            'ok': self.cmd_ok,
            'detail': self.cmd_detail,
            'state': self.run_status,
            'controller_type': self.controller_type,
            'use_pinger': bool(self.use_pinger),
            'note': self.run_note,
            'mission': self.run_mission,
            'run': self.run_number,
            'npy': f"{self.title}.npy" if self.title else '',
            # Additive: robot_interface skips a replanner run's path snapshot.
            'path_source': self.path_source,
            # Additive (2026-11-24): the pinger mode (robot_interface picks the
            # CSV layout from it), the pinger source the station chose, the
            # path parameter and whether a non-loop path has ended.
            'pinger_mode': self.pinger_mode,
            # What pinger_estimator applies (its estimate says), else the
            # last configure's request.
            'pinger_source': (self.pinger_estimate or {}).get('source') or self.pinger_source,
            'has_path': bool(self.has_path),
            'tau': float(self.tau),
            'path_complete': bool(self.path_complete()),
        }
        msg = String()
        msg.data = json.dumps(state)
        self.run_state_publisher.publish(msg)

    # ======================================================================
    #  7. LIVE PATH UPDATE  (BlueBoat-Replanner entry point; docs/PATH_UPDATE_*)
    #  Every decision is taken by path_update.py from a snapshot of this
    #  node's state (_path_update_context); this section only wires it.
    #  Fixed missions (path_source 'mission') refuse every request with
    #  'mission_owned'; run boundaries discard the plan; N10.
    # ======================================================================

    def _init_path_update(self):
        """Declare the feature's parameters and interfaces (all additive)."""
        self.declare_parameter('path_source', 'mission')
        self.path_source = self.get_parameter('path_source').get_parameter_value().string_value
        problem = _run_config_problem(self.controller_type, self.pinger_mode, self.has_path,
                                      self.path_source)
        if problem:
            self.get_logger().fatal(problem)
            raise ValueError(problem)

        dbl, integer = self._dbl, self._int
        # Thresholds and their derivations: docs/PATH_UPDATE_DESIGN.md D-4.
        cfg = pu.Config(
            gov_Lmax=self.gov_Lmax,
            hold_speed=self.hold_speed,
            lead_factor=dbl('path_update_lead_factor', 1.0),
            pos_tol=dbl('path_update_pos_tol', 0.20),
            heading_tol=dbl('path_update_heading_tol', math.radians(10.0)),
            heading_coherence=dbl('path_update_heading_coherence', math.radians(45.0)),
            max_speed=dbl('path_update_max_speed', 2.0),
            # 2.2 ms worst-case handling at 20 000 samples (design doc).
            max_points=integer('path_update_max_points', 20000))
        period = dbl('path_progress_period', 0.1)

        self.path_updater = pu.PathUpdater(cfg)
        self.path_progress_publisher = self.create_publisher(String, '/blueboat/path_progress', 10)
        self.path_event_publisher = self.create_publisher(String, '/blueboat/path_update_event',
                                                          QoSProfile(depth=50))
        self.update_path_service = self.create_service(UpdatePath, '/blueboat/update_path',
                                                       self.update_path_callback)
        self.path_progress_timer = self.create_timer(period, self.publish_path_progress)

        # Outside a session the launch is the run.
        self._path_update_reset('launch')
        self.get_logger().info(
            f"Live path update: path_source={self.path_source}"
            + (" - holding at the start pose until /blueboat/update_path loads "
               "the first path." if self.path_source == 'replanner' else
               " (fixed mission; /blueboat/update_path answers 'mission_owned')."))

    def _path_update_context(self):
        """A snapshot of the state a path-update decision depends on."""
        if self.arrow_active:
            override = 'arrow'
        elif list(self.manual_target) != [0.0, 0.0]:
            override = 'manual'
        else:
            override = 'none'
        boat = None
        if self.current_pose is not None:
            boat = (float(self.current_pose[0]), float(self.current_pose[1]),
                    float(self.current_pose[5]))
        now = time.time()
        clock = self._world_clock()          # == now under time_source 'wall'
        if self.session_mode:
            run_time = clock - self.run_t0 if self.run_t0 is not None else None
        else:
            run_time = clock - self.initial_time if self.initial_time is not None else None
        return pu.Context(
            # Not ready until the first ready tick has run: that tick sets
            # tau = 0 (timer_callback, time_set), which must never overwrite
            # the t[0] a first path has just set.
            ready=bool(self.ready and self.time_set), session_mode=bool(self.session_mode),
            run_active=bool(self.run_active), path_source=self.path_source,
            use_pinger=bool(self.use_pinger), override=override,
            tau=float(self.tau),
            path_time=float(getattr(self, 'path_time', self.dt)),
            path_steps=int(getattr(self, 'path_steps', 2)),
            dt=float(self.dt), run=int(self.run_number), boat=boat,
            ros_time=self.get_time(), wall_time=now, run_time=run_time)

    def _awaiting_hold_active(self):
        """A replanner run holding at its start pose, before its first path."""
        return (self.path_source == 'replanner' and not self.use_pinger
                and self.path_updater.awaiting and self.path_updater.hold is not None)

    def _capture_hold(self):
        x, y, yaw = (float(self.current_pose[0]), float(self.current_pose[1]),
                     float(self.current_pose[5]))
        self.path_updater.set_hold((x, y, yaw))
        if list(self.manual_target) == [0.0, 0.0]:
            # Shared keep-location state: start clean, unless an operator
            # target (which outranks this hold) owns it.
            self._clear_manual_hold()
        self.get_logger().info(
            f"Awaiting the first path: holding at ({x:.2f}, {y:.2f}), "
            f"yaw {yaw:.3f} rad.")
        self.publish_path_progress()

    def _path_update_reset(self, reason):
        """Discard the plan at a run boundary (launch / start / stop)."""
        event = self.path_updater.reset(self._path_update_context(), reason)
        if self.path_updater.awaiting and self.current_pose is not None:
            self._capture_hold()
        if event is not None:
            self._publish_path_event(event)
        self.publish_path_progress()

    def update_path_callback(self, request, response):
        """/blueboat/update_path: answer every request, change nothing on a refusal."""
        started = time.perf_counter()
        req = pu.Request(id=request.id, change_point=request.change_point,
                         frame=request.frame, t=np.asarray(request.t, dtype=float),
                         x=np.asarray(request.x, dtype=float),
                         y=np.asarray(request.y, dtype=float),
                         yaw=np.asarray(request.yaw, dtype=float))
        try:
            answer, event, new_tau = self.path_updater.handle(req, self._path_update_context())
        except Exception as exc:                       # noqa: BLE001 - reported, never fatal
            # A defect here must not take the controller down: refuse.
            self.get_logger().error(f"/blueboat/update_path: internal error "
                                    f"{type(exc).__name__}: {exc} - request refused.")
            response.accepted = False
            response.reason = pu.BAD_REQUEST
            response.message = f"internal error: {type(exc).__name__}: {exc}"
            response.active_id = self.path_updater.active_id or ''
            response.tau = float(self.tau)
            response.min_change_point = float('nan')
            return response

        if new_tau is not None:
            # First path: tau jumps to its first sample (the hold pose, D-5);
            # the awaiting hold ends and path following takes over next tick.
            self.tau = float(new_tau)
            if list(self.manual_target) == [0.0, 0.0]:
                self._clear_manual_hold()

        handling_ms = (time.perf_counter() - started) * 1e3
        event['handling_ms'] = handling_ms
        response.accepted = bool(answer['accepted'])
        response.reason = str(answer['reason'])
        response.message = str(answer['message'])
        response.active_id = str(answer['active_id'])
        response.tau = float(answer['tau'])
        response.min_change_point = float(answer['min_change_point'])

        if response.accepted:
            self.get_logger().info(
                f"Path update '{request.id}' ACCEPTED: {response.message} "
                f"[{handling_ms:.2f} ms]")
        else:
            self.get_logger().warning(
                f"Path update '{request.id}' refused ({response.reason}): "
                f"{response.message}")
        self._publish_path_event(event)
        self.publish_path_progress()
        return response

    def _replanner_reference_step(self, current_time, current_state, manual_active, arrow_active):
        """Replanner-run counterpart of _mission_reference_step: compose the
        window at tau from the plan, advance the governor, report transitions."""
        updater = self.path_updater
        if updater.awaiting or not updater.plan:
            if updater.awaiting and updater.hold is None and self.current_pose is not None:
                self._capture_hold()
            self.controller_path = Path()       # the awaiting-hold branch drives
            return

        ctx = self._path_update_context()
        params, xs, ys, yaws = updater.window(ctx)
        self.controller_path = self._compose_window(params, xs, ys, yaws)
        self.path_rx_time = current_time        # composed now: always fresh

        if not (manual_active or arrow_active):
            e_along, e_y = self.path_progress_errors(self.controller_path, current_state)
            self.advance_governor(e_along, e_y)

        events = updater.tick(self._path_update_context())
        for event in events:
            self._publish_path_event(event)
            self.get_logger().info(f"Path update: {event['message']}")
        if events:
            self.publish_path_progress()

    @staticmethod
    def _compose_window(params, xs, ys, yaws):
        """A nav_msgs/Path exactly as path_generation would answer it: frame
        'world', yaw-only quaternion, each pose stamped with its parameter."""
        path = Path()
        path.header.frame_id = 'world'
        quats = R.from_euler('zyx', np.column_stack(
            [yaws, np.zeros_like(yaws), np.zeros_like(yaws)])).as_quat()
        for t, x, y, q in zip(params, xs, ys, quats):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            sec, nanosec = ps.encode(t)
            pose.header.stamp.sec = sec
            pose.header.stamp.nanosec = nanosec
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = float(q[0])
            pose.pose.orientation.y = float(q[1])
            pose.pose.orientation.z = float(q[2])
            pose.pose.orientation.w = float(q[3])
            path.poses.append(pose)
        return path

    def publish_path_progress(self):
        try:
            payload = self.path_updater.progress(self._path_update_context())
        except Exception as exc:                       # noqa: BLE001 - reported
            self.get_logger().error(f"/blueboat/path_progress: {type(exc).__name__}: {exc}",
                                    throttle_duration_sec=10.0)
            return
        # Additive (2026-11-24).
        complete = self.path_complete()
        payload['path_complete'] = bool(complete)
        payload['pinger_mode'] = self.pinger_mode
        if complete and not self.path_complete_logged:
            self.path_complete_logged = True
            self.get_logger().info(f"Path complete (tau {self.tau:.1f} s) - holding the final pose "
                                   "until STOP RUN.")
        msg = String()
        msg.data = json.dumps(payload)
        self.path_progress_publisher.publish(msg)

    def _publish_path_event(self, event):
        msg = String()
        msg.data = json.dumps(event)
        self.path_event_publisher.publish(msg)


rclpy.init()
node = Controller()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    # Flush the timer-written log so a cut-short run keeps its last rows (CM-7).
    try:
        node.save_monitoring()
    except Exception:
        pass
    node.destroy_node()
    with contextlib.suppress(Exception):
        rclpy.shutdown()
