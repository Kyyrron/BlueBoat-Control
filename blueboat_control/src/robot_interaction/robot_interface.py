#!/usr/bin/env python3

# ============================================================================
# PATCHED for the Mission Control Station (logging update). Marked with
# "# --- logging update ---". Changes:
#   1. CSV columns reorganised with the important ones FIRST (names kept):
#      date, robot pose, target, [pinger + GPS], thrusters, then raw sensors.
#   2. All log rows are filled BY COLUMN NAME (df.loc with the column label)
#      instead of positional iloc indices, so the order can never silently
#      de-synchronise from the data again.
#   3. Fixed swapped thruster columns: thruster_input is [right, left]
#      (master_control convention), but index 0 was written into
#      'left_thr_in' in both logging paths.
#   4. Pinger-mode CSV: removed the duplicated 'target_x/y/psi' columns
#      (they were /controller_target = the same pinger vector as
#      'corrected_pinger_x/y', just in the robot frame); the world-frame
#      corrected_pinger columns are kept.
#   5. Fixed the no-pinger target logging: /monitoring_data x_d/y_d are now
#      world-frame for every controller (patched master_control), the two
#      debug spam logs are removed, and an empty monitoring buffer logs
#      zeros without raising.
# Everything else is byte-identical to the original.
# ============================================================================

# Common libraries import
import os
import time
from datetime import datetime
import numpy as np
import pandas as pd

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

# RC override channel conventions (MAVLink / mavros)
CHAN_RELEASE = 0        # give the channel back to the RC receiver
CHAN_NOCHANGE = 65535   # leave the channel untouched
PWM_NEUTRAL = 1500

class BlueBoatController(Node):

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

        # --- logging update: important columns first (names kept) ---
        if not self.use_UWgps:
                self.data_columns = ['Year',
                                'Month',
                                'Day',
                                'Hour',
                                'Minute',
                                'Second',
                                'MicroSecond',
                                'relative_x',
                                'relative_y',
                                'relative_psi',
                                'target_x',
                                'target_y',
                                'gps_latitude',
                                'gps_longitude',
                                'right_thr_in',
                                'left_thr_in',
                                'quat_x',
                                'quat_y',
                                'quat_z',
                                'quat_w',
                                'ang_vel_x',
                                'ang_vel_y',
                                'ang_vel_z',
                                'lin_acc_x',
                                'lin_acc_y',
                                'lin_acc_z']

        else:
            self.data_columns = ['Year',
                                'Month',
                                'Day',
                                'Hour',
                                'Minute',
                                'Second',
                                'MicroSecond',
                                'relative_x',
                                'relative_y',
                                'relative_psi',
                                'corrected_pinger_x',
                                'corrected_pinger_y',
                                'gps_latitude',
                                'gps_longitude',
                                'pinger_latitude',
                                'pinger_longitude',
                                'right_thr_in',
                                'left_thr_in',
                                'aco_x',
                                'aco_y',
                                'aco_z',
                                'ant_x',
                                'ant_y',
                                'ant_z',
                                'lat',
                                'lon',
                                'dep',
                                'filaco_x',
                                'filaco_y',
                                'filaco_z',
                                'quat_x',
                                'quat_y',
                                'quat_z',
                                'quat_w',
                                'ang_vel_x',
                                'ang_vel_y',
                                'ang_vel_z',
                                'lin_acc_x',
                                'lin_acc_y',
                                'lin_acc_z']

        self.data_size = len(self.data_columns)

        self.uw_gps_log = [0]*self.data_size

        self.df_log = pd.DataFrame(np.zeros(self.data_size).reshape(1, self.data_size),
                                  columns=self.data_columns)

        self.date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        os.makedirs('../../../data/Robot_data', exist_ok=True)  # avoid every log write silently failing when the folder is missing
        self.path = f'../../../data/Robot_data/{self.date}-{self.note}-poslog.csv'

    def monitoring_data_callback(self, msg: Float32MultiArray):
        """
        Callback for monitoring data.
        """
        self.monitoring_data = msg.data

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
        # Sanitize input
        max_input = 20.
        min_input = -20.
        left = np.clip(input[1], min_input, max_input)
        right = np.clip(input[0]*compensation_gain, min_input, max_input)

        # Convert thrust to PWM (double sanitation)
        max_PWM = 1900
        min_PWM = 1100
        right_pwm = np.clip(thrust_to_pwm(right), min_PWM, max_PWM)
        left_pwm = 3000 - np.clip(thrust_to_pwm(left), min_PWM, max_PWM) # Reverses direction of thruster rotation to account for asymmetrical propeller

        # Stream PWM to thrusters (published every control tick -> ~20 Hz refresh,
        # which also keeps ArduPilot's RC_OVERRIDE_TIME watchdog fed)
        self.send_rc_override(right_pwm=right_pwm, left_pwm=left_pwm)


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

    def full_stop(self):
        """
        Cancels any thruster input and set control parameters to False
        """
        self.thruster_input = [0,0]
        self.manualMove([0,0], force=True)
        self.setArmedStatus(False) 
        self.set_motors(False)

    def publish(self, msg_type, in_msg, publisher):
        """
        Makes publishing within code neater
        """
        msg = msg_type
        msg.data = in_msg
        publisher.publish(msg)

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

    def state_callback(self, msg):
        """
        Read the state of the robot
        """
        self.robot_state = msg

    def imu_callback(self, msg: Imu):
        self.orientation = msg.orientation                  # (quaternion)
        self.angular_velocity = msg.angular_velocity        # (rad/s)
        self.linear_acceleration = msg.linear_acceleration  # (m/s^2)

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

        # Position offset
        x_rel = msg.pose.pose.position.x - self.x0
        y_rel = msg.pose.pose.position.y - self.y0
        z_rel = msg.pose.pose.position.z - self.z0

        # Yaw offset
        yaw = cf.quaternion_to_yaw(msg.pose.pose.orientation)        
        yaw_rel = cf.normalize_angle(yaw - self.yaw0)

        self.relative_coordinates = [x_rel,y_rel,yaw_rel]

        # Build modified odometry
        odom_out = Odometry()
        odom_out.header = msg.header
        odom_out.child_frame_id = msg.child_frame_id

        odom_out.pose.pose.position.x = x_rel
        odom_out.pose.pose.position.y = y_rel
        odom_out.pose.pose.position.z = z_rel
        odom_out.pose.pose.orientation = cf.yaw_to_quaternion(yaw_rel)

        # Preserve velocity and covariance
        odom_out.twist = msg.twist
        odom_out.pose.covariance = msg.pose.covariance
        odom_out.twist.covariance = msg.twist.covariance

        # --- FRAME CONSISTENCY FIX -------------------------------------------
        # The pose above is re-expressed in the boot-relative frame (position
        # offset by (x0,y0), heading rotated by -yaw0). The linear velocity from
        # MAVROS is still in the raw 'map' frame, so pose and twist lived in two
        # frames differing by a constant rotation of yaw0. Any world->body
        # transform downstream (inRobotFrame / PID) then rotated the velocity
        # feedback by yaw0 relative to the position error, producing a fixed
        # diagonal drift and mirroring heading-swept paths (e.g. sin onto -y).
        # Pinger mode was immune because it zeroes position/yaw and works purely
        # in body frame. Rotate the linear velocity by -yaw0 so the WHOLE
        # /blueboat/odom message is in one consistent frame.
        
        # c0 = np.cos(self.yaw0)
        # s0 = np.sin(self.yaw0)
        # vx_raw = msg.twist.twist.linear.x
        # vy_raw = msg.twist.twist.linear.y
        # vx_rel =  c0 * vx_raw + s0 * vy_raw   # R(-yaw0) * v_map
        # vy_rel = -s0 * vx_raw + c0 * vy_raw
        # odom_out.twist.twist.linear.x = vx_rel
        # odom_out.twist.twist.linear.y = vy_rel
        
        # ---------------------------------------------------------------------

        self.odom_publisher.publish(odom_out)

        # x_t = vx_rel
        # y_t = vy_rel
        x_t = msg.twist.twist.linear.x
        y_t = msg.twist.twist.linear.y


        
        z_t = msg.twist.twist.linear.z
        self.vel = np.array([x_t,y_t,z_t])

        av = self.angular_velocity

        if self.fixed_pinger and not all(self.pinger_coordinates == np.zeros(3)): # Make sure the pinger has been detected
            # rotate pinger coordinates into original frame
            x_body = self.pinger_coordinates[0]
            y_body = self.pinger_coordinates[1]

            x_world, y_world = cf.transform_body_to_world(x_rel, y_rel, yaw_rel, x_body, y_body) # now relative to the original frame of reference 

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

        # rotate pinger coordinates into original frame
        x_body = self.pinger_coordinates[0]
        y_body = self.pinger_coordinates[1]

        x_world, y_world = cf.transform_body_to_world(x_rel, y_rel, yaw_rel, x_body, y_body) # now relative to the original frame of reference 

        self.corrected_pinger = [x_world, y_world]

        # convert local pinger into gps coordinates
        east, north = cf.local_to_enu(x_world, y_world, self.yaw0)

        lat, lon = cf.enu_to_gps(self.lat0, self.lon0, east, north)

        self.pinger_gps = [lat, lon]

    def gps_callback(self, msg : NavSatFix):
        self.gps_data = [msg.latitude, msg.longitude]

    def uw_gps_callback(self, msg):
        """
        Read msg from the underwater_gps node, compile it with robot data and save the log
        """

        if not self.use_UWgps:
            return

        # Make sure the robot's data is available
        if self.orientation is None or self.angular_velocity is None or self.linear_acceleration is None:
            return

        ## Compile data from gps, imu, and others
        self.uw_gps_log = msg.data

        # --- logging update: fill BY COLUMN NAME (order-independent) ------
        # /uw_gps_data layout: [date(7), aco xyz, ant xyz, lat, lon, dep,
        # filaco xyz] = 19 values.
        df_tmp = pd.DataFrame(np.zeros(self.data_size).reshape(1, self.data_size), columns=self.data_columns)

        raw_names = ['Year', 'Month', 'Day', 'Hour', 'Minute', 'Second',
                     'MicroSecond', 'aco_x', 'aco_y', 'aco_z',
                     'ant_x', 'ant_y', 'ant_z', 'lat', 'lon', 'dep',
                     'filaco_x', 'filaco_y', 'filaco_z']
        for name, value in zip(raw_names, msg.data):
            df_tmp.loc[df_tmp.index[0], name] = value

        t_x, t_y, t_z = msg.data[16], msg.data[17], msg.data[18]  # filaco
        self.pinger_coordinates = np.array([t_x,t_y,t_z])

        row = df_tmp.index[0]
        df_tmp.loc[row, 'quat_x'] = self.orientation.x
        df_tmp.loc[row, 'quat_y'] = self.orientation.y
        df_tmp.loc[row, 'quat_z'] = self.orientation.z
        df_tmp.loc[row, 'quat_w'] = self.orientation.w

        df_tmp.loc[row, 'ang_vel_x'] = self.angular_velocity.x
        df_tmp.loc[row, 'ang_vel_y'] = self.angular_velocity.y
        df_tmp.loc[row, 'ang_vel_z'] = self.angular_velocity.z

        df_tmp.loc[row, 'lin_acc_x'] = self.linear_acceleration.x
        df_tmp.loc[row, 'lin_acc_y'] = self.linear_acceleration.y
        df_tmp.loc[row, 'lin_acc_z'] = self.linear_acceleration.z

        df_tmp.loc[row, 'relative_x'] = self.relative_coordinates[0]
        df_tmp.loc[row, 'relative_y'] = self.relative_coordinates[1]
        df_tmp.loc[row, 'relative_psi'] = self.relative_coordinates[2]

        # --- logging update (2): target_x/y/psi removed from the pinger
        # CSV — they were /controller_target, i.e. the SAME pinger vector as
        # corrected_pinger_x/y but in the robot frame: duplicated
        # information. corrected_pinger (world frame) is kept.
        df_tmp.loc[row, 'corrected_pinger_x'] = self.corrected_pinger[0]
        df_tmp.loc[row, 'corrected_pinger_y'] = self.corrected_pinger[1]

        df_tmp.loc[row, 'gps_latitude'] = self.gps_data[0]
        df_tmp.loc[row, 'gps_longitude'] = self.gps_data[1]

        df_tmp.loc[row, 'pinger_latitude'] = self.pinger_gps[0]
        df_tmp.loc[row, 'pinger_longitude'] = self.pinger_gps[1]

        # thruster_input is [right, left] (master_control convention); the
        # original wrote index 0 into 'left_thr_in' -> columns were swapped.
        df_tmp.loc[row, 'right_thr_in'] = self.thruster_input[0]
        df_tmp.loc[row, 'left_thr_in'] = self.thruster_input[1]
        # ------------------------------------------------------------------

        self.df_log = pd.concat([self.df_log, df_tmp])

        self.df_log.to_csv(self.path) # Rewrite the entire file every time for safety in case of unexpected shutdowns

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

    def log_timer_callback(self):

        # Log here if no uw gps callback
        if not self.use_UWgps: 

            # --- logging update -------------------------------------------
            # /monitoring_data = [t, x, y, psi, x_d, y_d, psi_d, u1, u2];
            # x_d/y_d are WORLD-frame for every controller with the patched
            # master_control, so the logged path target is now correct also
            # for LoS and manual-target sessions (it used to arrive in the
            # robot frame for those). Empty buffer -> zeros, no spam logs.
            if len(self.monitoring_data) >= 6:
                target_x = self.monitoring_data[4]
                target_y = self.monitoring_data[5]
            else:
                target_x = 0.0
                target_y = 0.0

            try:
                df_tmp = pd.DataFrame(np.zeros(self.data_size).reshape(1, self.data_size), columns=self.data_columns)
                row = df_tmp.index[0]

                now = datetime.today()

                df_tmp.loc[row, 'Year'] = now.year
                df_tmp.loc[row, 'Month'] = now.month
                df_tmp.loc[row, 'Day'] = now.day
                df_tmp.loc[row, 'Hour'] = now.hour
                df_tmp.loc[row, 'Minute'] = now.minute
                df_tmp.loc[row, 'Second'] = now.second
                df_tmp.loc[row, 'MicroSecond'] = now.microsecond // 1000

                df_tmp.loc[row, 'quat_x'] = self.orientation.x
                df_tmp.loc[row, 'quat_y'] = self.orientation.y
                df_tmp.loc[row, 'quat_z'] = self.orientation.z
                df_tmp.loc[row, 'quat_w'] = self.orientation.w

                df_tmp.loc[row, 'ang_vel_x'] = self.angular_velocity.x
                df_tmp.loc[row, 'ang_vel_y'] = self.angular_velocity.y
                df_tmp.loc[row, 'ang_vel_z'] = self.angular_velocity.z

                df_tmp.loc[row, 'lin_acc_x'] = self.linear_acceleration.x
                df_tmp.loc[row, 'lin_acc_y'] = self.linear_acceleration.y
                df_tmp.loc[row, 'lin_acc_z'] = self.linear_acceleration.z

                df_tmp.loc[row, 'relative_x'] = self.relative_coordinates[0]
                df_tmp.loc[row, 'relative_y'] = self.relative_coordinates[1]
                df_tmp.loc[row, 'relative_psi'] = self.relative_coordinates[2]

                df_tmp.loc[row, 'gps_latitude'] = self.gps_data[0]
                df_tmp.loc[row, 'gps_longitude'] = self.gps_data[1]

                # [right, left] convention -> named columns (was swapped)
                df_tmp.loc[row, 'right_thr_in'] = self.thruster_input[0]
                df_tmp.loc[row, 'left_thr_in'] = self.thruster_input[1]

                df_tmp.loc[row, 'target_x'] = target_x
                df_tmp.loc[row, 'target_y'] = target_y
                # --------------------------------------------------------------

                self.df_log = pd.concat([self.df_log, df_tmp])

                self.df_log.to_csv(self.path)
            except Exception:
                self.get_logger().warn(f" -- Not ready to log yet")


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
        else:
            self.manualMove(self.thruster_input)        
        
rclpy.init()
node = BlueBoatController()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
