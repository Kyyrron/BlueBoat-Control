#!/usr/bin/env python3

# ============================================================================
# MAVROS bridge, thrust -> PWM, pose re-zeroing, pinger dead reckoning and the
# position CSV. Two things here are easy to get wrong and are commented at the
# point they happen rather than here:
#
#   * THRUST SATURATION IS UNIFORM, NOT PER-SIDE (manualMove, section 3). The
#     two thrusters carry one wrench, so clipping each independently rewrites
#     the command instead of clamping it -- [+45, +18] became [+20, +18], which
#     collapses the turn and makes the boat diverge. See thrust_limits.py.
#   * ONE CSV WRITER, BOTH LAYOUTS (section 8). The pinger layout used to be
#     written from uw_gps_callback, so a Water Linked dropout stopped recording
#     the robot too. That callback now only caches; the timer owns the file.
#
# Column layout, its revision history and the actuation_state encoding live in
# _custom_libraries/robot_log_schema.py. Rows are filled BY COLUMN NAME so the
# two can never desynchronise.
# ============================================================================

# ----------------------------------------------------------------------------
# FILE MAP (class BlueBoatController) -- sections are banner-commented below.
#
#   1. WIRING                     __init__
#   2. MAIN LOOP AND SAFETY       timer_callback, thruster_input_stale, full_stop
#   3. THRUST -> PWM              manualMove                   <-- calibration, N4 gate
#   4. OPERATOR COMMANDS          str_input_callback, move_callback,
#                                 request_param_mode
#   5. POSE / PINGER              odom_callback                <-- frame re-zeroing
#   6. INBOUND TELEMETRY          imu_, gps_, state_, uw_gps_, target_,
#                                 thr_input_, monitoring_data_, param_, mode_
#   7. MAVROS PLUMBING            set_servo, send_rc_override, setArmedStatus,
#                                 SetMode, set_motors, publish
#   8. CSV LOGGING                log_timer_callback
#
# Moved out of this file:
#   CSV column layout  ->  _custom_libraries/robot_log_schema.py  (data only,
#                          ROS-free; write-once field-data contract, CM-7)
# ----------------------------------------------------------------------------

# Common libraries import
import os
import time
from datetime import datetime
import numpy as np

# ROS2 import
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy

# msg import
from std_msgs.msg import String, Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import State, OverrideRCIn

# srv import
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.srv import CommandLong

# Custom imports
import custom_functions as cf
import robot_log_schema as rls   # CSV column layout (ROS-free, _custom_libraries/)
import thrust_limits as tl       # uniform thrust saturation (ROS-free)

# RC override channel conventions (MAVLink / mavros)
CHAN_RELEASE = 0        # give the channel back to the RC receiver
CHAN_NOCHANGE = 65535   # leave the channel untouched
PWM_NEUTRAL = 1500

class BlueBoatController(Node):

    # ======================================================================
    #  1. WIRING
    #  parameters, topics, service clients, timers.
    # ======================================================================

    def __init__(self):
        super().__init__('blueboat_controller')


        #### PINGER ####
        self.fixed_pinger = False # True -> Publish pinger coordinates in robot frame, without dead reckoning.
                                 # False -> Publish without yaw compensation and dead reckoning - default behavior in target following 

        ################## Get Parameters ##################
        self.declare_parameter('enable_motors', False)
        self.enable_motors = self.get_parameter('enable_motors').get_parameter_value().bool_value

        self.declare_parameter('use_UWgps', True)
        self.use_UWgps = self.get_parameter('use_UWgps').get_parameter_value().bool_value

        self.declare_parameter('note', '')
        self.note = self.get_parameter('note').get_parameter_value().string_value

        self.declare_parameter('controller_type', '') 
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value

        # Loss-of-reference watchdog. master_control publishes /thruster_input once
        # per 20 Hz control tick; 0.5 s is ten consecutive missed ticks, well outside
        # DDS jitter at that rate and well inside ArduPilot's own RC_OVERRIDE_TIME.
        self.declare_parameter('thruster_input_timeout', 0.5)
        self.thruster_input_timeout = self.get_parameter('thruster_input_timeout').get_parameter_value().double_value

        # Per-thruster saturation, same name and default as master_control's.
        # This used to be two hard-coded literals inside manualMove, which meant
        # master_control's thrust_limit parameter bought nothing on the real boat:
        # raising it there just moved the clamp to the one here that no parameter
        # could reach. One name, one number, both ends.
        self.declare_parameter('thrust_limit', 20.0)
        self.thrust_limit = self.get_parameter('thrust_limit').get_parameter_value().double_value

        # Empty means "resolve it" - see custom_functions.data_root for the order.
        self.declare_parameter('data_dir', '')
        self.data_root = cf.data_root(
            self.get_parameter('data_dir').get_parameter_value().string_value)

        ################## ROS2 Communication ##################
        ## Publishers
        self.param_publisher = self.create_publisher(String, '/blueboat/param_str',10)
        self.odom_publisher = self.create_publisher(Odometry, '/blueboat/odom',10)
        self.pinger_publisher = self.create_publisher(Float32MultiArray, '/blueboat/pinger_coordinates', 10)
        self.set_controller_publisher = self.create_publisher(Bool, '/blueboat/controller_ready',10)

        # Actuator stream (this is how QGC drives the boat: a fixed-rate, fire-and-forget
        # stream where the latest message wins - NOT one acknowledged RPC per actuation)
        self.rc_override_publisher = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        ## Subscribers

        self.monitoring_data = []

        # Subscriber
        self.plot_subscriber = self.create_subscription(
            Float32MultiArray,
            "/monitoring_data",
            self.monitoring_data_callback,
            10
        )

        # Node interaction
        self.str_input_subscriber = self.create_subscription(String, '/blueboat/input_str', self.str_input_callback, 10)
        self.ready_sub = self.create_subscription(Bool,'/blueboat/param_ready',self.param_callback,10)
        self.mode_sub = self.create_subscription(String, '/blueboat/param_mode',self.mode_callback,10)

        # Robot sensor
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.robot_state_sub = self.create_subscription(State,'/mavros/state',self.state_callback,10)
        self.imu_sub = self.create_subscription(Imu,'/mavros/imu/data', self.imu_callback, qos)
        self.local_odom_sub = self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_callback, qos)
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos)

        # Data logging
        self.uw_gps_sub = self.create_subscription(Float32MultiArray,'/uw_gps_data', self.uw_gps_callback,10)
        self.target_sub = self.create_subscription(Float32MultiArray,'/controller_target', self.target_callback,10)
        self.thruster_input_sub = self.create_subscription(Float32MultiArray, "/thruster_input", self.thr_input_callback,10)

        ## Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')

        ################## Initialize ##################
        self.robot_state = State()

        # Main loop initialization variables
        self.init = False
        self.mode = ''

        # Handshake retry state.
        # One-shot messages published before DDS discovery completes are silently lost,
        # which is what made the launch fail "at random". Instead of publishing once and
        # hoping, we keep a desired state and re-publish periodically until confirmed.
        self.desired_param_mode = None
        self.last_param_tx = 0.0
        self.param_retry_period = 1.0   # seconds between re-requests
        self.last_ready_tx = 0.0
        self.ready_republish_period = 1.0

        # Loss-of-reference watchdog state. last_thr_rx is None until the first
        # /thruster_input arrives, which is itself a "no reference" condition.
        self.last_thr_rx = None
        self.thr_watchdog_tripped = False

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.log_timer = self.create_timer(0.33, self.log_timer_callback) # 3 times per seconds 

        # Manual input control init
        self.stopping_sequence = False
        self.stopping_time = 0.
        self.manual_move_timer = 0.

        self.time_set = False

        ### Sensors and dead reckoning parameters
        # IMU
        self.orientation = None
        self.angular_velocity = None
        self.linear_acceleration = None

        # GPS 
        self.gps_data = [0,0] # latitude, longitude
        self.pinger_gps = [0,0]

        self.prev_time = None
        self.vel = np.zeros(3)
        self.pos = np.zeros(3)
        self.yaw0 = None # Used to start the starting yaw at 0 regardless of actual orientation

        ## Control

        self.relative_coordinates = [0,0,0]
        self.target = [0,0,0]
        self.pinger_coordinates = np.zeros(3)
        self.corrected_pinger = [0,0]
        self.thruster_input = [0,0]

        ################## Initialize PWM control ##################
        self.interpolator = cf.generate_interpolator()

        ################## Initialize data collection ##################

        # Column layout lives in robot_log_schema.py (ROS-free, in
        # _custom_libraries/). It is a WRITE-ONCE field-data contract: read the
        # module docstring before touching a name or an order. Rows are filled
        # by column NAME below, never by index, so the two cannot desynchronise.
        self.data_columns = rls.columns_for(self.use_UWgps)
        # ((world_x, world_y), (latitude, longitude)) under this layout's names,
        # so the shared leading block is filled once instead of branched twice.
        self.target_cols, self.target_gps_cols = rls.target_columns_for(self.use_UWgps)

        # Latest /uw_gps_data packet (19 raw values), CACHED here by
        # uw_gps_callback. That callback used to assemble and write a CSV row
        # itself, which tied the whole log -- robot pose included -- to the Water
        # Linked link: a UGPS dropout stopped recording the boat. Now it only
        # caches, and log_timer_callback is the single writer for both layouts.
        self.uw_gps_log = [0.0] * 19

        self.date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        log_dir = cf.ensure_data_dir(self, self.data_root, 'data', 'Robot_data')
        stem = f'{self.date}-{self.note}-poslog' if self.note else f'{self.date}-poslog'
        self.path = cf.reserve_run_file(log_dir, stem, '.csv') + '.csv'
        self.origin_path = self.path[:-len('-poslog.csv')] + '-origin.yaml'

        # Header once, then one appended-and-flushed row per tick. The previous
        # version accumulated every row in a DataFrame and rewrote the WHOLE file
        # on each write, which is O(n^2) in both time and bytes and, despite the
        # comment claiming it was "for safety in case of unexpected shutdowns",
        # is strictly less safe than this: a row appended and flushed is already
        # on disk, and a kill mid-rewrite truncates a full file rather than one
        # row. index=False drops the unnamed pandas index column the old writer
        # emitted (it read 0 on every row), and there is no all-zero seed row.
        self.log_file = open(self.path, 'w', buffering=1)
        self.log_file.write(','.join(self.data_columns) + '\n')
        self.log_file.flush()
        self.origin_written = False
        self.get_logger().info(f"Position log: {self.path}")

    # ======================================================================
    #  2. MAIN LOOP AND SAFETY
    #  20 Hz tick + loss-of-reference watchdog. Zero thrust on any doubt.
    # ======================================================================

    def timer_callback(self):
        """
        Main loop
        """

        ################## Initialize robot ##################
        if not self.init:
            # Wait until connected
            if not self.robot_state.connected:
                self.get_logger().info('Waiting for FCU connection...')
                return

            # Set mode
            if self.robot_state.mode != "MANUAL": 
                self.SetMode('MANUAL')
                return

            self.request_param_mode('override')

            self.init = True

        ################## Handshake maintenance ##################
        # Re-send the mode request until param_set confirms it. This closes the
        # discovery race that used to make the launch hang at random.
        if (self.desired_param_mode is not None
                and self.mode != self.desired_param_mode
                and time.time() - self.last_param_tx > self.param_retry_period):
            self.get_logger().info(f"Waiting for param mode '{self.desired_param_mode}' (current: '{self.mode}'), re-requesting...")
            self.last_param_tx = time.time()
            self.publish(String(), self.desired_param_mode, self.param_publisher)

        # Wait for direct control to be enabled
        if self.mode != 'override':
            return

        ################## Control loop ##################
        
        # Start recording time
        if not self.time_set:
            self.initial_time = time.time()

            # Send ready msg to controller node
            self.publish(Bool(), True, self.set_controller_publisher)
            self.last_ready_tx = time.time()

            self.time_set = True

        # Periodically re-publish readiness so a controller node that finished
        # starting late (e.g. blocked on the path service) still receives it
        if time.time() - self.last_ready_tx > self.ready_republish_period:
            self.last_ready_tx = time.time()
            self.publish(Bool(), True, self.set_controller_publisher)
        
        current_time = time.time()
        
        ## Send input to thrusters

        # If no controller is set, allow for manual input
        if self.controller_type == '' and current_time - self.initial_time >= self.manual_move_timer:
            self.manualMove([0, 0]) # If override + no controler, stop the robot after any manual move command

        # Loss-of-reference watchdog: a controller is configured but has gone quiet.
        # Zero the thrust and keep zeroing it until commands come back. Deliberately
        # NOT full_stop(): that also disarms, and these stalls are transient by design.
        # The call goes through manualMove without force, so the enable_motors gate
        # still holds (N4) - this is not a new /mavros/rc/override bypass.
        elif self.thruster_input_stale(current_time):
            if not self.thr_watchdog_tripped:
                self.thr_watchdog_tripped = True
                self.get_logger().warn(
                    f"No /thruster_input for {self.thruster_input_timeout:.2f} s "
                    f"(controller_type='{self.controller_type}') - zeroing thrust.")
            self.thruster_input = [0, 0]
            self.manualMove([0, 0])

        else:
            if self.thr_watchdog_tripped:
                self.thr_watchdog_tripped = False
                self.get_logger().info("/thruster_input resumed - releasing watchdog.")
            self.manualMove(self.thruster_input)        

    def thruster_input_stale(self, now):
        """
        Loss-of-reference watchdog predicate.

        True when a controller is configured but its /thruster_input has gone
        quiet for longer than thruster_input_timeout - a stalled, crashed or
        early-returning master_control. Without this the last received thrust
        keeps being streamed to the motors indefinitely.

        Inert when no controller is configured: that case already has its own
        stale-command guard (manual_move_timer, set by the 'move' CLI command).
        """
        if self.controller_type == '':
            return False
        if self.last_thr_rx is None:
            return True
        return (now - self.last_thr_rx) > self.thruster_input_timeout

    def full_stop(self):
        """
        Cancels any thruster input and set control parameters to False
        """
        self.thruster_input = [0,0]
        self.manualMove([0,0], force=True)
        self.setArmedStatus(False) 
        self.set_motors(False)

    # ======================================================================
    #  3. THRUST -> PWM CALIBRATION
    #  The enable_motors gate (N4) and the Newton->PWM mapping. Tunable.
    # ======================================================================

    def manualMove(self, input, force=False):
        """
        Convert a newton input to pwm and stream it to the motors through RC override
        """

        # Safety
        if not self.enable_motors and not force:
            return

        def thrust_to_pwm(T): # Thrust in Newton
            return int(self.interpolator(T))
        
        # Compensate right thruster observed weaker output
        if input[1] >= 0:
            compensation_gain = 1.2
        else:
            compensation_gain = 0.75

        compensation_gain=1.0
        # Sanitize input.
        #
        # UNIFORM saturation, not two independent clips. The two thrusters do not
        # carry independent signals: what the controller commands is a surge force
        # and a yaw moment, split into a common mode and a differential. Clipping
        # each side on its own therefore does not clamp the command, it rewrites
        # it into a different one -- [+45, +18] became [+20, +18], collapsing a
        # 27 N differential to 2 N, so the boat stopped turning and ran further
        # from the path the harder the controller asked. One scale factor for both
        # sides keeps the ratio, hence the turn, and only slows the boat down:
        # [+45, +18] -> [+20, +8]. thrust_limits.py carries the full argument.
        scaled, scale = tl.scale_to_limit(
            [input[0] * compensation_gain, input[1]], self.thrust_limit)
        if scale < 1.0:
            self.get_logger().warn(
                f"Thrust {list(input)} exceeds thrust_limit="
                f"{self.thrust_limit:.1f} N - scaled by {scale:.3f} to "
                f"[{scaled[0]:.1f}, {scaled[1]:.1f}] (direction preserved).",
                throttle_duration_sec=2.0)

        right = float(scaled[0])
        left = float(scaled[1])

        # Convert thrust to PWM (double sanitation)
        max_PWM = 1900
        min_PWM = 1100
        right_pwm = np.clip(thrust_to_pwm(right), min_PWM, max_PWM)
        left_pwm = 3000 - np.clip(thrust_to_pwm(left), min_PWM, max_PWM) # Reverses direction of thruster rotation to account for asymmetrical propeller

        # Stream PWM to thrusters (published every control tick -> ~20 Hz refresh,
        # which also keeps ArduPilot's RC_OVERRIDE_TIME watchdog fed)
        self.send_rc_override(right_pwm=right_pwm, left_pwm=left_pwm)

    # ======================================================================
    #  4. OPERATOR COMMAND SURFACE
    #  /blueboat/input_str: enable, stop, override, default, arm, disarm, move.
    # ======================================================================

    def str_input_callback(self, msg: String):
        """
        Read str_msg content and take required action
        By default, any unrecognized command will be sent to the move_callback,
        allowing for manual control through the input_str topic without needing to set the command to 'move'
        """
        input_string = msg.data.split()
        command = input_string[0]
        
        dispatch = {'enable': lambda: self.set_motors(True),
                    'stop': self.full_stop,
                    'override': lambda: self.request_param_mode('override'),
                    'default': lambda: self.request_param_mode('default'),
                    'move': lambda: self.move_callback(input_string),
                    'arm': lambda: self.setArmedStatus(True),
                    'disarm': lambda: self.setArmedStatus(False)
        }

        action = dispatch.get(command, lambda: self.move_callback(input_string))
        action()   

    def move_callback(self, in_str):
        """
        Called when input_str is 'move', the first two floats are left and right thruster inputs, 
        the last one is the length (in seconds) of the applied thrust
        """

        # Make sure the command is valid
        if len(in_str) != 4:
            self.get_logger().info(f" Incorrect move command.")
            return

        # Start measuring time and apply thrust
        self.initial_time = time.time()
        left, right, self.manual_move_timer = map(float, in_str[1:])
        self.thruster_input = [right,left]

    def request_param_mode(self, mode):
        """
        Ask param_set for a mode and remember the request so the main loop can
        re-send it until param_set confirms on /blueboat/param_mode.
        A single publish can be lost if it races DDS discovery or if param_set is
        still waiting on mavros - this was the main cause of the random launch hangs.
        """
        self.desired_param_mode = mode
        self.last_param_tx = time.time()
        self.publish(String(), mode, self.param_publisher)

    # ======================================================================
    #  5. POSE RE-ZEROING AND PINGER DEAD RECKONING
    #  Local-ENU frame: position is translated to the launch point, axes stay
    #  ENU (+x = East, +y = North) and yaw stays ABSOLUTE ENU (0 = East,
    #  CCW-positive). Yaw is deliberately NOT re-zeroed: subtracting yaw0
    #  without also rotating the position axes produced a hybrid frame that
    #  was only self-consistent when the boat launched facing East.
    #  The twist is NOT rotated either (N3): it is body-frame from MAVROS.
    # ======================================================================

    def odom_callback(self, msg: Odometry):

        # Set previous time measurement and compute dt
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            self.prev_time = t
            return
        dt = t - self.prev_time
        self.prev_time = t

        # Initialize reference on first callback
        if not hasattr(self, "origin_set") or not self.origin_set:
            self.x0 = msg.pose.pose.position.x
            self.y0 = msg.pose.pose.position.y
            self.z0 = msg.pose.pose.position.z
            self.yaw0 = cf.quaternion_to_yaw(msg.pose.pose.orientation)
            self.lat0 = self.gps_data[0]
            self.lon0 = self.gps_data[1]
            self.origin_set = True

        # Position offset (translation only -- axes stay ENU)
        x_rel = msg.pose.pose.position.x - self.x0
        y_rel = msg.pose.pose.position.y - self.y0
        z_rel = msg.pose.pose.position.z - self.z0

        # Yaw stays absolute ENU (0 = East, CCW+). yaw0 is latched above for
        # the origin sidecar only; it is no longer part of the frame.
        yaw = cf.quaternion_to_yaw(msg.pose.pose.orientation)

        self.relative_coordinates = [x_rel, y_rel, yaw]

        # Build modified odometry
        odom_out = Odometry()
        odom_out.header = msg.header
        odom_out.child_frame_id = msg.child_frame_id

        odom_out.pose.pose.position.x = x_rel
        odom_out.pose.pose.position.y = y_rel
        odom_out.pose.pose.position.z = z_rel
        odom_out.pose.pose.orientation = cf.yaw_to_quaternion(yaw)

        # Preserve velocity and covariance
        odom_out.twist = msg.twist
        odom_out.pose.covariance = msg.pose.covariance
        odom_out.twist.covariance = msg.twist.covariance

        # The pose above is translated to the launch point (axes ENU, yaw
        # absolute), but the twist is NOT rotated, and must not be: MAVROS
        # already publishes this odometry's twist in child_frame_id 'base_link'
        # (the ENU velocity goes out separately on local_position/velocity_local).
        # It is body-frame surge/sway, which is what master_control and the
        # pinger dead-reckoning below both want. See N3.

        self.odom_publisher.publish(odom_out)

        x_t = msg.twist.twist.linear.x
        y_t = msg.twist.twist.linear.y


        
        z_t = msg.twist.twist.linear.z
        self.vel = np.array([x_t,y_t,z_t])

        av = self.angular_velocity

        if self.fixed_pinger and not all(self.pinger_coordinates == np.zeros(3)): # Make sure the pinger has been detected
            # rotate pinger coordinates into the local-ENU world frame
            x_body = self.pinger_coordinates[0]
            y_body = self.pinger_coordinates[1]

            x_world, y_world = cf.transform_body_to_world(x_rel, y_rel, yaw, x_body, y_body) # now in the local-ENU world frame

            self.corrected_pinger = [x_world, y_world]
            self.publish(Float32MultiArray(), self.corrected_pinger, self.pinger_publisher)
            return

        if self.fixed_pinger:
            return

        # Apply sensor fusion to get a smoother approximation at higher frequency of pinger_coordinates
        if av is not None and not all(self.pinger_coordinates == np.zeros(3)): # Make sure the pinger has been detected
            omega = np.array([0.0, 0.0, av.z])
            p = self.pinger_coordinates

            self.pinger_coordinates -= (self.vel + np.cross(omega, p)) * dt
        
        self.publish(Float32MultiArray(), self.pinger_coordinates, self.pinger_publisher)

        if not hasattr(self, "origin_set") or not self.origin_set:
            return  

        # rotate pinger coordinates into the local-ENU world frame
        x_body = self.pinger_coordinates[0]
        y_body = self.pinger_coordinates[1]

        x_world, y_world = cf.transform_body_to_world(x_rel, y_rel, yaw, x_body, y_body) # now in the local-ENU world frame

        self.corrected_pinger = [x_world, y_world]

        # The world frame IS local ENU about (lat0, lon0), so world -> east/north
        # is the identity by construction.
        east, north = x_world, y_world

        lat, lon = cf.enu_to_gps(self.lat0, self.lon0, east, north)

        self.pinger_gps = [lat, lon]

    # ======================================================================
    #  6. INBOUND TELEMETRY
    #  Sensor and status callbacks. None of these command anything.
    # ======================================================================

    def imu_callback(self, msg: Imu):
        self.orientation = msg.orientation                  # (quaternion)
        self.angular_velocity = msg.angular_velocity        # (rad/s)
        self.linear_acceleration = msg.linear_acceleration  # (m/s^2)

    def gps_callback(self, msg : NavSatFix):
        self.gps_data = [msg.latitude, msg.longitude]

    def state_callback(self, msg):
        """
        Read the state of the robot
        """
        self.robot_state = msg

    def uw_gps_callback(self, msg):
        """
        Cache the Water Linked UGPS packet and reseed the pinger dead reckoning.

        This callback does NOT write the CSV any more. It used to assemble and
        write a whole row, which meant the pinger-mode log was driven by the UGPS
        link at 2 Hz and stopped entirely -- robot pose included -- whenever that
        link dropped. log_timer_callback is now the single writer for both
        layouts, and reads self.uw_gps_log from here.

        /uw_gps_data layout, 19 values:
            [date x7, aco xyz, ant xyz, lat, lon, dep, filaco xyz]
        """
        if not self.use_UWgps:
            return

        self.uw_gps_log = list(msg.data)

        # filaco = the FILTERED acoustic position; it seeds the dead reckoning
        # that odom_callback then propagates at odom rate.
        t_x, t_y, t_z = msg.data[16], msg.data[17], msg.data[18]
        self.pinger_coordinates = np.array([t_x, t_y, t_z])

    def target_callback(self, msg: Float32MultiArray):
        """
        Update the target, used when interacting with the controller node
        """
        self.target = msg.data

    def thr_input_callback(self, msg: Float32MultiArray):
        """
        Update the thruster inputs, used when interacting with the controller node
        """
        self.thruster_input = msg.data
        self.last_thr_rx = time.time()

    def monitoring_data_callback(self, msg: Float32MultiArray):
        """
        Callback for monitoring data.
        """
        self.monitoring_data = msg.data

    ################## ROS2 node interaction ##################

    def param_callback(self, msg: String):
        """
        Prints true if the parameter changes are successful (used with the 'default' and 'override' command)
        """
        self.get_logger().info(f" Parameters ready: {msg.data}")

    def mode_callback(self, msg: String):
        """
        Displays the mode sent to the robot to confirm the changes
        """
        previous_mode = self.mode
        self.mode = msg.data

        if previous_mode != self.mode:
            self.get_logger().info(f" Mode received: {self.mode}")

            # When leaving override, hand the RC channels back so the default
            # thruster mapping (QGC / xbox controller) works again
            if previous_mode == 'override':
                self.send_rc_override(right_pwm=PWM_NEUTRAL, left_pwm=PWM_NEUTRAL)
                self.send_rc_override(release=True)

    # ======================================================================
    #  7. MAVROS / MAVLINK PLUMBING
    #  RC override stream (N6), arming, mode. Rarely edited.
    # ======================================================================

    ################## Thruster interaction ##################

    def set_servo(self, n, pwm):
        """
        LEGACY fallback - send a single MAV_CMD_DO_SET_SERVO via the command service.
        Note: this only works when SERVOn_FUNCTION is 0 (Disabled). It must NOT be
        called at control-loop rate: every call is an acknowledged RPC, and a lost
        ACK over WiFi stalls the mavros command plugin for seconds, which was the
        source of the delayed/overrunning 'move' behavior.
        """
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(n)
        req.param2 = float(pwm)

        # explicitly set all remaining params as float
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        self.cmd_client.call_async(req)

    def send_rc_override(self, right_pwm=None, left_pwm=None, release=False):
        """
        Publish one RC override message (channel 1 = right/servo1, channel 3 = left/servo3).
        Fire-and-forget, latest value wins - the same transport class QGC uses.
        Requires 'override' mode: param_set maps SERVO1/3_FUNCTION to RCIN1/RCIN3
        passthrough and points the autopilot's GCS sysid at mavros.
        """
        msg = OverrideRCIn()
        channels = [CHAN_NOCHANGE] * 18

        if release:
            channels[0] = CHAN_RELEASE
            channels[2] = CHAN_RELEASE
        else:
            channels[0] = int(right_pwm)
            channels[2] = int(left_pwm)

        msg.channels = channels
        self.rc_override_publisher.publish(msg)

    ################## User interaction ##################
    def setArmedStatus(self,command):
        """
        Either arm or disarm the robot's thrusters. Note that the 'override' parameter completely disregards armed status
        """
        self.get_logger().info(f"{'Arming' if command else 'Disarming'} vehicle...")

        if self.arming_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = command
            self.arming_client.call_async(req)

    def SetMode(self, mode):
        """
        Set the robot's mode to the requested input.
        """
        self.get_logger().info(f"Current mode: {self.robot_state.mode}, switching to {mode}]")

        if self.mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = mode
            self.mode_client.call_async(req)

    def set_motors(self, inBool):
        """
        Set the bool value of enable_motors. 
        This is meant as a safety as no input will be set to the thrusters intil this is set to True
        """
        self.enable_motors = inBool
        self.get_logger().info(f" Enable motors: {self.enable_motors}")

    def publish(self, msg_type, in_msg, publisher):
        """
        Makes publishing within code neater
        """
        msg = msg_type
        msg.data = in_msg
        publisher.publish(msg)

    # ======================================================================
    #  8. CSV LOGGING
    #  Write-once field data. Columns in _custom_libraries/robot_log_schema.py.
    #  ONE writer for BOTH layouts, on this node's own timer. The pinger layout
    #  used to be written from uw_gps_callback instead, i.e. at the Water Linked
    #  link's rate, so a UGPS dropout stopped recording the robot as well.
    # ======================================================================

    def actuation_state(self):
        """
        One integer saying whether the logged thrust command could reach the water.

        Output : int, see robot_log_schema for the encoding --
                 0 motors disabled, 1 live (enabled and in override),
                 2 enabled but not in override, 3 watchdog forcing zero.

        Without this the CSV cannot distinguish a controller that commanded zero
        from a boat that was never listening: enable_motors False produces a
        completely normal-looking log with no PWM ever leaving the boat, and the
        loss-of-reference watchdog writes [0, 0] into the same thruster_input
        field a genuine zero command uses.
        """
        if self.thr_watchdog_tripped:
            return rls.ACT_WATCHDOG
        if not self.enable_motors:
            return rls.ACT_MOTORS_DISABLED
        if self.mode != 'override':
            return rls.ACT_NOT_OVERRIDE
        return rls.ACT_LIVE

    def write_origin_sidecar(self):
        """
        Record the world frame's own origin, once, beside the CSV.

        The world frame is local ENU: origin latched at the first odom
        callback, axes East/North, yaw absolute ENU. Every world-frame column
        in the log -- relative_x/y/psi, the target pair, corrected_pinger_x/y
        -- is expressed in it, and (lat0, lon0) is what georeferences a
        recorded run afterwards. yaw0_rad is the boat's ENU heading at the
        latch instant, kept for provenance only (it is NOT part of the frame;
        logs recorded before the local-ENU fix used yaw - yaw0 as relative_psi).
        """
        if self.origin_written or not getattr(self, 'origin_set', False):
            return
        try:
            with open(self.origin_path, 'w') as f:
                f.write('# Origin of the local-ENU world frame (translation only,\n'
                        '# axes East/North, yaw absolute ENU) used by every\n'
                        '# world-frame column of the CSV beside this file. Latched\n'
                        '# at robot_interface\'s first odom callback. yaw0_rad is\n'
                        '# the boat\'s ENU heading at that instant, provenance only.\n')
                f.write(f'latitude: {self.lat0}\n')
                f.write(f'longitude: {self.lon0}\n')
                f.write(f'yaw0_rad: {self.yaw0}\n')
                f.write(f'poslog: {os.path.basename(self.path)}\n')
            self.origin_written = True
            self.get_logger().info(f"Frame origin: {self.origin_path}")
        except OSError as exc:
            self.get_logger().warn(f"Could not write the frame origin sidecar: {exc}")

    def build_log_row(self):
        """
        Assemble one CSV row for whichever layout this run selected.

        Output : dict {column name: value} covering every column of
                 self.data_columns. Filled BY COLUMN NAME, never by index, so
                 the order in robot_log_schema can never silently desynchronise
                 from the values written into it.

        Raises AttributeError while the IMU has not reported yet; the caller
        treats that as "not ready" rather than as a failure.
        """
        now = datetime.today()
        roll, pitch, _ = cf.quaternion_to_rpy(self.orientation)

        row = {
            # Wall clock. MicroSecond is FULL microseconds in both layouts --
            # the no-pinger path used to write now.microsecond // 1000, i.e.
            # milliseconds under a column named MicroSecond.
            'Year': now.year, 'Month': now.month, 'Day': now.day,
            'Hour': now.hour, 'Minute': now.minute, 'Second': now.second,
            'MicroSecond': now.microsecond,

            # Robot pose, local-ENU world frame (origin = launch point;
            # relative_psi is ABSOLUTE ENU yaw, 0 = East, CCW+).
            'relative_x': self.relative_coordinates[0],
            'relative_y': self.relative_coordinates[1],
            'relative_psi': self.relative_coordinates[2],

            # Robot fix, immediately followed by the target's below so the two
            # can be selected and plotted together.
            'gps_latitude': self.gps_data[0],
            'gps_longitude': self.gps_data[1],

            # thruster_input is [right, left] (master_control's convention).
            'right_thr_in': self.thruster_input[0],
            'left_thr_in': self.thruster_input[1],
            'actuation_state': self.actuation_state(),

            # Attitude. Yaw is not repeated -- it is relative_psi above.
            'roll': roll,
            'pitch': pitch,

            'ang_vel_x': self.angular_velocity.x,
            'ang_vel_y': self.angular_velocity.y,
            'ang_vel_z': self.angular_velocity.z,

            'lin_acc_x': self.linear_acceleration.x,
            'lin_acc_y': self.linear_acceleration.y,
            'lin_acc_z': self.linear_acceleration.z,
        }

        # --- the target block, under whichever names this layout uses --------
        if self.use_UWgps:
            # The pinger IS the target. corrected_pinger is the dead-reckoned
            # vector rotated into the world frame; pinger_gps is that same point
            # in WGS84, both maintained by odom_callback.
            target_xy = self.corrected_pinger
            target_gps = self.pinger_gps
        else:
            # /monitoring_data = [t, x, y, psi, x_d, y_d, psi_d, u1, u2].
            # x_d/y_d are world-frame in EVERY controller branch (CM-8 / N9), so
            # no frame correction is applied here or downstream. An empty buffer
            # (no controller running yet) logs zeros rather than raising.
            if len(self.monitoring_data) >= 6:
                target_xy = [self.monitoring_data[4], self.monitoring_data[5]]
            else:
                target_xy = [0.0, 0.0]
            target_gps = self.target_to_gps(target_xy)

        row[self.target_cols[0]] = target_xy[0]
        row[self.target_cols[1]] = target_xy[1]
        row[self.target_gps_cols[0]] = target_gps[0]
        row[self.target_gps_cols[1]] = target_gps[1]

        # --- raw Water Linked packet, pinger layout only ---------------------
        if self.use_UWgps:
            # /uw_gps_data indices 7..18; 0..6 are its own date fields, which the
            # wall-clock stamp above already covers.
            ugps_names = ('aco_x', 'aco_y', 'aco_z',
                          'ant_x', 'ant_y', 'ant_z',
                          'lat', 'lon', 'dep',
                          'filaco_x', 'filaco_y', 'filaco_z')
            for name, value in zip(ugps_names, self.uw_gps_log[7:19]):
                row[name] = value

        return row

    def target_to_gps(self, target_xy):
        """
        Convert a local-ENU world-frame target into WGS84 degrees.

        Input  : target_xy -- [x, y] in the same frame as relative_x/y.
        Output : [latitude, longitude] in degrees, or [0.0, 0.0] before the
                 frame origin has been latched by the first odom callback.

        The world frame is local ENU about (lat0, lon0), so world -> east/north
        is the identity and only the equirectangular projection remains --
        identical to the pinger path.
        """
        if not getattr(self, 'origin_set', False):
            return [0.0, 0.0]

        east, north = target_xy[0], target_xy[1]
        lat, lon = cf.enu_to_gps(self.lat0, self.lon0, east, north)
        return [lat, lon]

    def log_timer_callback(self):
        """
        Append one row to the position CSV. The single writer, both layouts.
        """
        try:
            row = self.build_log_row()
        except (AttributeError, TypeError, IndexError) as exc:
            # Normal before the first IMU / odom message; self.orientation and
            # friends are None until then. Logged with the reason rather than a
            # bare "not ready", so a genuine assembly bug is not hidden by it.
            self.get_logger().warn(f" -- Not ready to log yet: {exc}",
                                   throttle_duration_sec=5.0)
            return

        try:
            self.log_file.write(
                ','.join(repr(float(row[c])) for c in self.data_columns) + '\n')
            self.log_file.flush()
        except (OSError, ValueError) as exc:
            self.get_logger().error(f"Could not append to {self.path}: {exc}")
            return

        self.write_origin_sidecar()

rclpy.init()
node = BlueBoatController()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
