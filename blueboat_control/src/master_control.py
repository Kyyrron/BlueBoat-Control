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
#   4. GUIDANCE                   path_progress_errors, advance_governor,
#                                 los_guidance, solve_LoS
#   5. CALLBACKS / HELPERS        odom_, pinger_, ready_, manual_target_,
#                                 publish_thrust, get_time
#
# Moved out of this file:
#   inRobotFrame()  ->  _custom_libraries/frame_math.py   (pure geometry, ROS-free)
# ----------------------------------------------------------------------------

# rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSDurabilityPolicy
import rclpy

# Common python libraries
import os
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
            
        self.future = None # Used for client requests

        self.time_set = False
        self.initial_time = None
        # self.dt is set by _declare_tuning_parameters (20 Hz default, was 1.0 Hz)
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.current_pose = None
        self.current_twist = None

        self.ready = False
        self.init = False
        self.pinger_target = None
        self.manual_target = [0.0,0.0]

        # Initialize controller 
        self.controller_path = Path()

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
            self.path_time = self.mpc_time
            self.path_steps = self.mpc_horizon

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

        # Initialize monitoring values
        self.monitoring = []
        self.monitoring.append(['t','x','y','psi','x_d','y_d','psi_d','u1','u2'])

        self.t_record = self.get_time()

        ctrl = self.controller_type
        date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        sim = 'simulation' if self.isSimulation else 'real'
        run_dir = cf.ensure_data_dir(self, self.data_root, 'data', f'{ctrl}_data')
        self.title = cf.reserve_run_file(run_dir, f'{date}-{ctrl}_{sim}_data', '.npy')
        self.get_logger().info(f"Controller log: {self.title}.npy")

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
        self.outer_gains = {'x':   tuple(arr('outer_gains_x',   [3.0, 0.01, 0.0])),
                            'psi': tuple(arr('outer_gains_psi', [3.0, 0.01, 0.0]))}
        self.inner_gains = {'u': tuple(arr('inner_gains_u', [1.0, 0.0, 0.0])),
                            'r': tuple(arr('inner_gains_r', [1.5, 0.0, 0.0]))}
        self.pid_lookahead = dbl('pid_lookahead', 2.5)

        # -- kinematic Fossen-LoS -------------------------------------------
        self.los_lookahead   = dbl('los_lookahead', 2.5)
        self.los_ku          = dbl('los_ku', 20.0)
        self.los_kpsi        = dbl('los_kpsi', 10.0)
        self.los_kd          = dbl('los_kd', 1.0)
        self.los_speed_scale = dbl('los_speed_scale', 1.0)
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
        self.k_psi = dbl('point_k_psi', 16.0 if self.isSimulation else 10.0)
        # Negative disables the arrival check.
        self.safety_distance = dbl('safety_distance', -1.0)

        # -- MPC -------------------------------------------------------------
        self.mpc_horizon = integer('mpc_horizon', 15)
        self.mpc_time    = dbl('mpc_time', 2.5)
        self.Q_weight = np.diag(arr('mpc_Q_diag', [50.0,   # x
                                                   50.0,   # y
                                                   30.0,   # psi
                                                    1.0,   # u
                                                    1.0,   # v
                                                    1.0])) # r
        self.R_weight = np.diag(arr('mpc_R_diag', [0.015,  # u1
                                                   0.015])) # u2

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

        if not self.init:
            if self.controller_type == 'MPC':
                self.controller = ur_mpc.MPCController(robot_mass = 16.01,
                                                iz = 5.64,    # Yaw inertia
                                                a_u = -26.77, # added mass XdotU
                                                a_v = -7.55,  # added mass YdotV
                                                a_r = -21.77, # added mass NdotR
                                                d_u = -29.34, # viscous drag Xu
                                                d_v = -51.54, # viscous drag Yv
                                                d_r = -44.65, # viscous drag Nr
                                                horizon = self.mpc_horizon, 
                                                time = self.mpc_time, 
                                                Q_weight = self.Q_weight,
                                                R_weight = self.R_weight,
                                                input_bounds = self.input_bounds
                                                )

            if self.controller_type == 'PID':
                self.controller = PID.PIDLoS(dt = self.dt,
                                             B = self.B_matrix,
                                             outer_gains = self.outer_gains,
                                             inner_gains = self.inner_gains,
                                             lookahead = self.pid_lookahead,
                                             thruster_limits = self.thruster_limits
                                             )

            self.get_logger().info('Controller node initiated')
            self.init = True

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
                    if result is not None:
                        self.controller_path = result.path
                    else:
                        self.get_logger().error("Service returned None.")
                except Exception as e:
                    self.get_logger().error(f"Service call raised exception: {e}")
                finally:
                    self.future = None

            # Advance the governor using the boat's progress along the current
            # window (frozen while a manual target overrides path following).
            if self.controller_path.poses and not manual_active:
                e_along, e_y, _, _ = self.path_progress_errors(self.controller_path, current_state)
                self.advance_governor(e_along, e_y)

            # Issue the next request at the (governed) parameter tau
            if self.future is None:
                request = RequestPath.Request()
                request.path_request.data = np.linspace(self.tau,
                                                         self.tau + self.path_time,
                                                         int(self.path_steps), dtype=float)
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

            if (current_time - self.t_record) > 0.1: # Update the saved file at set interval
                self.t_record = current_time
                np.save(self.title, self.monitoring)

    # ======================================================================
    #  4. GUIDANCE
    #  Path governor (N8) and the two steering laws.
    # ======================================================================

    # ------------------------------------------------------------------ #
    #  Path parameter governor                                           #
    # ------------------------------------------------------------------ #
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

        yaw_rate = self.k_psi * np.arctan2(y,x)
        d = np.sqrt(x**2+y**2)
        v = self.k_v * d
        v = 5*np.log(v+1)

        if list(self.manual_target) != [0.0,0.0]:
            v = 10*np.log(v+1) # If manual target, go faster. Don't need to be that precise here.

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
            breakaway = self.min_thrust * g * max(0.0, np.cos(np.arctan2(y, x)))
            if 0.0 < v < breakaway:
                v = breakaway

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

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9
        

rclpy.init()
node = Controller()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
