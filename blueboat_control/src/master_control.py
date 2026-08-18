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


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class Controller(Node):
    def __init__(self):

        super().__init__('master_control', namespace='blueboat')


        self.declare_parameter('controller_type', 'MPC') 
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value

        self.declare_parameter('simulation', True) 
        self.isSimulation = self.get_parameter('simulation').get_parameter_value().bool_value

        self.declare_parameter('use_pinger', False) 
        self.use_pinger = self.get_parameter('use_pinger').get_parameter_value().bool_value

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
        self.dt = 0.05                     # 20 Hz control loop (was 1.0 Hz)
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
        self.path_speed_scale = 1.0   # global multiplier on the authored speed
        self.gov_Lmin = 0.5           # gap (m) below which tau runs at full authored speed
        self.gov_Lmax = 3.0           # gap (m) above which tau pauses (boat too far behind)
        # ---------------------------------------------------------------------

        # MPC Parameters
        if self.controller_type == 'MPC':
            self.mpc_horizon = 15
            self.mpc_time = 2.5
            max_linear_bound = 20.0
            min_linear_bound = -20.0
            self.input_bounds = {"lower": np.array([min_linear_bound, min_linear_bound]),
                                "upper": np.array([max_linear_bound, max_linear_bound]),
                                "idx":   np.array([0, 1])
                                }

            self.Q_weight = np.diag([50, # x
                                    50, # y 
                                    30, # psi
                                    1, # u
                                    1, # v
                                    1  # r
                                    ])
            
            self.R_weight = np.diag([0.015, # u1
                                    0.015  # u2
                                    ])

            # Initialize MPC solver
            self.controller = None # Updated at the start of spin

            self.path_time = self.mpc_time
            self.path_steps = self.mpc_horizon

        # PID Parameters
        if self.controller_type == 'PID':
            self.path_time = self.dt
            self.path_steps = 2
            
            # Real gains
            self.outer_gains = {'x': (3., 0.01, 0.),
                                'psi': (3.0, 0.01, 0.)}
                                # 'psi': (1.2, 0.01, 0.)}

            self.inner_gains = {'u': (1., 0., 0.),
                                'r': (1.5, 0., 0.)}
            
            self.thruster_limits = {"min": np.array([-20.0, -20.0]),   
                                    "max": np.array([ 20.0,  20.0])}

            radius = 0.59/2
            self.B_matrix = B = np.array([[1.        ,1.],
                                          [0.        ,0.],
                                          [radius,-radius]])

            self.pid_lookahead = 2.5   # LoS lookahead distance Delta (m) for PID guidance

        # LoS Parameters
        if self.controller_type ==  'LoS':
            self.path_time = self.dt
            self.path_steps = 2

            radius = 0.59/2
            self.B_matrix = np.array([[1.        ,1.],
                                      [0.        ,0.],
                                      [radius,-radius]])
            self.thruster_limits = {"min": np.array([-20.0, -20.0]),
                                    "max": np.array([ 20.0,  20.0])}
            # Kinematic Fossen-LoS gains
            self.los_lookahead = 2.5   # lookahead distance Delta (m)
            self.los_ku = 8.0          # surge speed error -> force
            self.los_kpsi = 10.0        # heading error -> yaw moment
            self.los_kd = 1.0          # yaw-rate damping
            self.los_speed_scale = 1.0 # multiplier on authored speed for LoS
            self.los_allocator = PID.ThrustAllocator(self.B_matrix, limits=self.thruster_limits)

        if self.isSimulation:
            self.k_v = 2.0
            self.k_psi = 16.0
        else:
            self.k_v = 0.15
            self.k_psi = 10.0

        self.safety_distance = -1.     # Brakes and stop moving if the distance to the pinger is smaller than this value, set it to negative to disable it
        self.stopping_sequence = False # Used as a safety to stop LoS control when it gets close to target
        self.stopping_time = None

        # Initialize monitoring values
        self.monitoring = []
        self.monitoring.append(['t','x','y','psi','x_d','y_d','psi_d','u1','u2'])

        self.t_record = self.get_time()

        ctrl = self.controller_type
        date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        sim = 'simulation' if self.isSimulation else 'real'
        os.makedirs(f'data/{ctrl}_data', exist_ok=True)  # avoid np.save failing silently at runtime
        self.title = f'data/{ctrl}_data/{date}-{ctrl}_{sim}_data'

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9

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

    def advance_governor(self, e_along):
        """
        Advance the path parameter tau. When the along-track gap is small the
        target moves at the authored speed (tau_dot = speed_scale); as the gap
        approaches gov_Lmax the target slows and finally pauses, so the boat
        can always catch up. Never moves backward.
        """
        span = max(1e-6, (self.gov_Lmax - self.gov_Lmin))
        factor = np.clip((self.gov_Lmax - e_along) / span, 0.0, 1.0)
        tau_dot = self.path_speed_scale * factor
        self.tau += tau_dot * self.dt

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
        psi_err = _wrap(psi_d - psi)

        u_cmd = self.los_speed_scale * U_d * max(0.0, math.cos(psi_err))

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

    def inRobotFrame(self, robot_coords, target_coords):

        def wrap_angle(angle):
            return (angle + np.pi) % (2 * np.pi) - np.pi

        x_r,y_r,psi_r,_,_,_ = robot_coords
        x_t,y_t,psi_t,_,_,_ = target_coords

        cos = np.cos
        sin = np.sin

        x = (x_t - x_r)*cos(psi_r) + (y_t - y_r)*sin(psi_r)
        y = (y_t - y_r)*cos(psi_r) - (x_t - x_r)*sin(psi_r)
        psi = wrap_angle(psi_t) - wrap_angle(psi_r)

        return x,y,psi

    def timer_callback(self):
        if not self.ready:
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
                e_along, _, _, _ = self.path_progress_errors(self.controller_path, current_state)
                self.advance_governor(e_along)

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
            target = self.inRobotFrame(current_state, target)
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
                u,_ = self.controller.compute(current_state, target[:3],
                                              u_ff=target[3], psi_path=target[2])

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

        # Publish thruster input
        msg = Float32MultiArray()
        msg.data = [float(v) for v in u]
        self.thruster_input_publisher.publish(msg)

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
        

rclpy.init()
node = Controller()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
