#!/usr/bin/env python3

# ============================================================================
# MAVROS bridge: thrust -> PWM over RC override, pose re-zeroing to the launch
# point, pinger dead reckoning, the position CSV, and in a run session the
# per-run CSV / report / ros2bag.
#
#   * Thrust saturation is UNIFORM, not per-side (manualMove, thrust_limits.py).
#   * ONE CSV writer for both layouts (log_timer_callback); uw_gps_callback only
#     caches, so a Water Linked dropout never stops the robot's log. Column
#     layout and actuation_state encoding: robot_log_schema.py (filled by name).
#   * Run sessions (session_mode:=true): one CSV per run, opened and closed
#     from master_control's /blueboat/run_state; outside a session one CSV per
#     launch. Run bags come from a separate `ros2 bag record` process
#     (run_bag.py) that this node only starts, resumes and stops.
#
# FILE MAP (class BlueBoatController):
#   1. WIRING               __init__, open_position_log
#   2. MAIN LOOP / SAFETY   timer_callback, arm sequence, watchdog, full_stop
#   3. THRUST -> PWM        manualMove                   <-- calibration, N4 gate
#   4. OPERATOR COMMANDS    str_input_callback, move_callback, request_param_mode
#   5. POSE / PINGER        odom_callback                <-- frame re-zeroing
#   6. INBOUND TELEMETRY    imu_, gps_, state_, uw_gps_, target_, thr_input_,
#                           monitoring_data_, param_, mode_
#   7. MAVROS PLUMBING      set_servo, send_rc_override, setArmedStatus, SetMode
#   8. CSV LOGGING          log_timer_callback
#   9. SHUTDOWN             shutdown
#  10. RUN SESSION          run_state_callback, open_run_log, close_run_log,
#                           publish_run_log, path-update log
#  11. RUN BAG              bag_timer_callback, _bag_begin_run, _bag_end_run,
#                           _bag_teardown
# ============================================================================

import json
import os
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from rosbag2_interfaces.srv import Resume

import custom_functions as cf
import robot_log_schema as rls   # CSV column layout (ROS-free)
import thrust_limits as tl       # uniform thrust saturation (ROS-free)
import run_bag                   # per-run ros2bag recorder process (ROS-free)
import path_update_log as pul    # the run's -pathlog.csv (ROS-free)

# RC override channel conventions (MAVLink / mavros)
CHAN_RELEASE = 0        # give the channel back to the RC receiver
CHAN_NOCHANGE = 65535   # leave the channel untouched
PWM_NEUTRAL = 1500

# Run-file sibling of a position CSV: '...-poslog[-N].csv' -> '...-<suffix>[-N].yaml'
_POSLOG_RE = r'-poslog(-\d+)?\.csv$'

class BlueBoatController(Node):

    # ======================================================================
    #  1. WIRING -- parameters, topics, service clients, timers
    # ======================================================================

    def __init__(self):
        super().__init__('blueboat_controller')

        # True -> publish the pinger in the world frame without dead reckoning;
        # False (always) -> body frame, dead-reckoned at odom rate.
        self.fixed_pinger = False

        self.enable_motors = self._param('enable_motors', False).bool_value
        self.use_UWgps = self._param('use_UWgps', True).bool_value
        self.note = self._param('note', '').string_value
        self.controller_type = self._param('controller_type', '').string_value
        # Loss-of-reference watchdog: ten missed 20 Hz ticks, well inside
        # ArduPilot's own RC_OVERRIDE_TIME.
        self.thruster_input_timeout = self._param('thruster_input_timeout', 0.5).double_value
        # Same name and default as master_control's: one number, both ends.
        self.thrust_limit = self._param('thrust_limit', 20.0).double_value
        # Empty = resolve it (custom_functions.data_root).
        self.data_root = cf.data_root(self._param('data_dir', '').string_value)
        # Run sessions (section 10): the CSV is per run, never per launch.
        self.session_mode = self._param('session_mode', False).bool_value

        # Link guardian (session only): arm the autopilot at connection, BEFORE
        # override -- ArduPilot refuses to arm on RC passthrough, so MANUAL ->
        # 'default' -> arm -> 'override'. Our thrust path does not need it; the
        # on-boat guardian script does, to drive home if the laptop goes
        # silent. A failed arm never blocks the session ('rearm' retries).
        arm_on_connect = self._param('arm_on_connect', True).bool_value
        self.arm_on_connect = self.session_mode and arm_on_connect
        self.arm_timeout = self._param('arm_timeout', 15.0).double_value
        # Bound on param_set confirming 'default' (cold pull over WiFi).
        self.arm_default_timeout = self._param('arm_default_timeout', 25.0).double_value

        # Run bags (section 11): one ros2bag per run, sessions only.
        record_bag = self._param('record_bag', True).bool_value
        self.record_bag = self.session_mode and record_bag

        # The path server's trajectory argument, read only to snapshot a
        # from_yaml run's mission beside its CSV (`-path.yaml`).
        self.trajectory = self._param('trajectory', '').string_value

        # ---- publishers ----
        self.param_publisher = self.create_publisher(String, '/blueboat/param_str',10)
        self.odom_publisher = self.create_publisher(Odometry, '/blueboat/odom',10)
        self.pinger_publisher = self.create_publisher(Float32MultiArray, '/blueboat/pinger_coordinates', 10)
        self.set_controller_publisher = self.create_publisher(Bool, '/blueboat/controller_ready',10)
        # Actuator stream: fixed-rate, fire-and-forget, latest wins (N6).
        self.rc_override_publisher = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # ---- subscribers ----
        self.monitoring_data = []
        self.plot_subscriber = self.create_subscription(
            Float32MultiArray,
            "/monitoring_data",
            self.monitoring_data_callback,
            10
        )

        self.str_input_subscriber = self.create_subscription(String, '/blueboat/input_str', self.str_input_callback, 10)
        self.ready_sub = self.create_subscription(Bool,'/blueboat/param_ready',self.param_callback,10)
        self.mode_sub = self.create_subscription(String, '/blueboat/param_mode',self.mode_callback,10)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.robot_state_sub = self.create_subscription(State,'/mavros/state',self.state_callback,10)
        self.imu_sub = self.create_subscription(Imu,'/mavros/imu/data', self.imu_callback, qos)
        self.local_odom_sub = self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_callback, qos)
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos)

        self.uw_gps_sub = self.create_subscription(Float32MultiArray,'/uw_gps_data', self.uw_gps_callback,10)
        self.target_sub = self.create_subscription(Float32MultiArray,'/controller_target', self.target_callback,10)
        self.thruster_input_sub = self.create_subscription(Float32MultiArray, "/thruster_input", self.thr_input_callback,10)
        # Live path update events -> the run's -pathlog.csv.
        self.path_event_sub = self.create_subscription(
            String, '/blueboat/path_update_event', self.path_event_callback, QoSProfile(depth=50))

        if self.session_mode:
            self.run_state_sub = self.create_subscription(String, '/blueboat/run_state', self.run_state_callback, 10)
            self.run_log_publisher = self.create_publisher(String, '/blueboat/run_log', 10)

        # ---- service clients ----
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')

        # ---- state ----
        self.robot_state = State()
        self.init = False
        self.mode = ''

        # Handshakes are re-published until confirmed: a one-shot published
        # before DDS discovery completes is silently lost.
        self.desired_param_mode = None
        self.last_param_tx = 0.0
        self.param_retry_period = 1.0   # seconds between re-requests
        self._last_param_ready = None   # param_callback edge detection
        self.last_ready_tx = 0.0
        self.ready_republish_period = 1.0

        # Loss-of-reference watchdog (None = no /thruster_input yet).
        self.last_thr_rx = None
        self.thr_watchdog_tripped = False

        # E-STOP latch: set by full_stop(), cleared ONLY by 'enable'. Without it
        # the next manualMove(self.thruster_input) undoes a stop 50 ms later.
        self.estopped = False
        self.motors_disabled_logged = False

        # Arm-before-override sequence: None / 'default' / 'arming'.
        self.arm_seq = None
        self.arm_seq_t0 = 0.0
        self.last_arm_tx = 0.0
        self.run_running = False        # session: master_control reports a run

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.log_timer = self.create_timer(0.33, self.log_timer_callback)

        self.manual_move_timer = 0.     # 'move' duration (no controller_type)
        self.time_set = False

        # IMU
        self.orientation = None
        self.angular_velocity = None
        self.linear_acceleration = None

        # GPS
        self.gps_data = [0,0]           # latitude, longitude
        self.pinger_gps = [0,0]

        # Frame origin, latched at the first odom (odom_callback).
        self.origin_set = False
        self.prev_time = None
        self.vel = np.zeros(3)
        self.yaw0 = None

        self.relative_coordinates = [0,0,0]
        self.target = [0,0,0]           # /controller_target, stored only
        self.pinger_coordinates = np.zeros(3)
        self.corrected_pinger = [0,0]
        self.thruster_input = [0,0]

        self.interpolator = cf.generate_interpolator()   # Newton -> PWM

        # ---- position CSV (write-once field record, robot_log_schema.py) ----
        self.data_columns = rls.columns_for(self.use_UWgps)
        self.target_cols, self.target_gps_cols = rls.target_columns_for(self.use_UWgps)
        # Latest /uw_gps_data packet (19 values), cached by uw_gps_callback.
        self.uw_gps_log = [0.0] * 19

        self.log_file = None
        self.path = None
        self.origin_path = None
        self.origin_written = False
        self.log_run = None             # session: the run whose CSV is open
        self.last_csv = ''              # session: the previous run's CSV
        self.report_procs = []          # session: report subprocesses spawned
        # Path-update log: per run, created at its first request event, never
        # for the pinger layout.
        self.pathlog = None
        self.pathlog_pending = []       # session: events of a run not opened yet
        self.run_path_source = 'mission'
        if not self.session_mode:
            self.open_position_log(self.note)
            self._open_pathlog()
        else:
            self.get_logger().info(
                "Run session: no position log until a run is started "
                "(one CSV per run).")
            self.run_log_timer = self.create_timer(1.0, self.publish_run_log)

        # Run bags (section 11). bag_armed: the NEXT run's recorder, started
        # paused while the session idles; bag_run: the current run's.
        self.bag_k = 0
        self.bag_armed = None
        self.bag_armed_client = None
        self.bag_run = None
        self.bag_run_client = None
        self.bag_resume_pending = False
        self.bag_recording = False
        self.bag_last = ''              # the previous run's bag, final path
        self.bag_closers = []           # worker threads closing finished bags
        self.bag_next_arm = 0.0         # earliest time to (re-)arm a recorder
        self.bag_shutting_down = False
        if self.record_bag:
            self.get_logger().info(
                "Run bags: one ros2bag per run (all topics except the "
                "side-scan sonar), filed into the run's folder.")
            self.bag_timer = self.create_timer(0.2, self.bag_timer_callback)

    def _param(self, name, default):
        """Declare one parameter and return its ParameterValue."""
        self.declare_parameter(name, default)
        return self.get_parameter(name).get_parameter_value()

    def open_position_log(self, note):
        """
        Create a new position CSV (+ sidecar name) and write its legend and
        header: once from __init__ outside a session, per run inside one.
        """
        self.date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        log_dir = cf.ensure_data_dir(self, self.data_root, 'data', 'Robot_data')
        stem = f'{self.date}-{note}-poslog' if note else f'{self.date}-poslog'
        self.path = cf.reserve_run_file(log_dir, stem, '.csv') + '.csv'
        # By pattern, so a collision suffix ('-poslog-2.csv') carries over.
        self.origin_path = re.sub(_POSLOG_RE, r'-origin\1.yaml', self.path)

        # Legend rows + header now (a killed run keeps them), then one
        # appended-and-flushed row per tick.
        self.log_file = open(self.path, 'w', buffering=1)
        for line in rls.legend_rows(self.data_columns):
            self.log_file.write(','.join(line) + '\n')
        self.log_file.write(','.join(self.data_columns) + '\n')
        self.log_file.flush()
        self.origin_written = False
        self.get_logger().info(f"Position log: {self.path}")

    # ======================================================================
    #  2. MAIN LOOP AND SAFETY -- 20 Hz; zero thrust on any doubt
    # ======================================================================

    def timer_callback(self):
        # ---- connection: FCU, MANUAL, (session) arm, then override ----
        if not self.init:
            if not self.robot_state.connected:
                self.get_logger().info('Waiting for FCU connection...')
                return

            if self.robot_state.mode != "MANUAL":
                self.SetMode('MANUAL')
                return

            if self.arm_on_connect and not self.arm_sequence_step():
                self.maintain_param_handshake()
                return

            self.request_param_mode('override')

            self.init = True

        self.maintain_param_handshake()

        if self.mode != 'override':
            return

        # ---- E-STOP latch (N4b): re-zero every tick through the UNFORCED
        # manualMove, so the closed gate holds neutral; re-assert readiness
        # False at the republish rate (the station's acknowledgement).
        if self.estopped:
            self.thruster_input = [0, 0]
            self.manualMove([0, 0])
            if time.time() - self.last_ready_tx > self.ready_republish_period:
                self.last_ready_tx = time.time()
                self.publish(Bool(), False, self.set_controller_publisher)
            return

        # ---- ready: tell the controller, and keep re-telling it ----
        if not self.time_set:
            self.initial_time = time.time()
            self.publish(Bool(), True, self.set_controller_publisher)
            self.last_ready_tx = time.time()
            self.time_set = True

        if time.time() - self.last_ready_tx > self.ready_republish_period:
            self.last_ready_tx = time.time()
            self.publish(Bool(), True, self.set_controller_publisher)

        current_time = time.time()

        # ---- thrust ----
        # No controller: a CLI 'move' runs for its duration, then zero.
        if self.controller_type == '' and current_time - self.initial_time >= self.manual_move_timer:
            self.manualMove([0, 0])

        # Loss-of-reference watchdog: zero (unforced, behind the N4 gate) until
        # commands resume. Not full_stop(): that disarms, and stalls are transient.
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

    def maintain_param_handshake(self):
        """Re-send the mode request until param_set confirms it."""
        if (self.desired_param_mode is not None
                and self.mode != self.desired_param_mode
                and time.time() - self.last_param_tx > self.param_retry_period):
            self.get_logger().info(f"Waiting for param mode '{self.desired_param_mode}' (current: '{self.mode}'), re-requesting...")
            self.last_param_tx = time.time()
            self.publish(String(), self.desired_param_mode, self.param_publisher)

    def arm_sequence_step(self):
        """One tick of the session's arm-before-override sequence.

        MANUAL is already set: request the 'default' mapping (so Rover's pre-arm
        motor check passes), arm, and return True once armed or given up after
        a bounded wait (not fatal: the on-boat guardian arms itself if needed).
        """
        now = time.time()
        log = self.get_logger()
        if self.arm_seq is None:
            if self.robot_state.armed:
                log.info("Autopilot already armed - going to override.")
                return True
            self.arm_seq = 'default'
            self.arm_seq_t0 = now
            log.info("Link guardian: arming the autopilot before override "
                     "(normal thruster mapping first, ArduPilot refuses to arm on passthrough).")
            self.request_param_mode('default')
            return False

        if self.arm_seq == 'default':
            if self.mode == 'default':
                self.arm_seq = 'arming'
                self.arm_seq_t0 = now
                # Rover sets the 73/74 thrusters' +-1000 range only in its 1 Hz
                # loop while DISARMED; armed earlier, idle can come out at full
                # reverse. First arm request ~3.5 s from now (2 s retry).
                self.last_arm_tx = now + 1.5
            elif now - self.arm_seq_t0 > self.arm_default_timeout:
                log.error(f"param_set did not confirm 'default' within {self.arm_default_timeout:.0f} s - "
                          "continuing to override WITHOUT arming. The link guardian will arm "
                          "the boat itself if it has to take over; 'rearm' retries.")
                self.arm_seq = None
                return True
            return False

        # 'arming'
        if self.robot_state.armed:
            log.info("Autopilot ARMED (HOME = here) - going to override.")
            self.arm_seq = None
            return True
        if now - self.arm_seq_t0 > self.arm_timeout:
            log.error(f"Autopilot did not arm within {self.arm_timeout:.0f} s (pre-arm checks: GPS fix, "
                      "EKF, battery - see the PreArm messages). Continuing to override; the link "
                      "guardian arms the boat itself if it has to take over, and 'rearm' retries "
                      "between runs.")
            self.arm_seq = None
            return True
        if now - self.last_arm_tx > 2.0:
            self.last_arm_tx = now
            self.setArmedStatus(True)
        return False

    def request_rearm(self):
        """'rearm': run the arm-before-override sequence again (session only),
        after an E-STOP or a failed arm. Refused during a run and while the
        E-STOP is latched.
        """
        log = self.get_logger()
        if not self.session_mode:
            log.warn("'rearm' is a run-session command; use 'arm' outside a session.")
            return
        if self.estopped:
            log.warn("'rearm' refused: E-STOP latched - release it first.")
            return
        if self.run_running or self.log_run is not None:
            log.warn("'rearm' refused: a run is active - stop it first.")
            return
        if self.robot_state.armed:
            log.info("'rearm': autopilot already armed.")
            return
        if not self.init or self.arm_seq is not None:
            log.info("'rearm': an arm sequence is already in progress.")
            return
        log.info("'rearm': re-running the arm-before-override sequence.")
        self.init = False
        self.arm_on_connect = True

    def thruster_input_stale(self, now):
        """
        True when a controller is configured but /thruster_input has been quiet
        for longer than thruster_input_timeout. Inert without a controller
        (the 'move' CLI command has its own manual_move_timer).
        """
        if self.controller_type == '':
            return False
        if self.last_thr_rx is None:
            return True
        return (now - self.last_thr_rx) > self.thruster_input_timeout

    def full_stop(self):
        """
        EMERGENCY STOP: zero thrust, latch the motor gate off, disarm, and
        acknowledge (controller_ready False). Never changes the parameter mode:
        leaving override would TRANSFER authority to whatever else transmits.
        """
        self.estopped = True
        self.thruster_input = [0,0]
        self.manualMove([0,0], force=True)
        self.setArmedStatus(False)
        self.set_motors(False)
        self.publish(Bool(), False, self.set_controller_publisher)
        self.last_ready_tx = time.time()
        self.get_logger().warn("E-STOP: thrust zeroed, motors disabled, disarmed. "
                               "Send 'enable' to release the latch.")

    def release_estop(self):
        """Clear the E-STOP latch and re-enable the motor gate ('enable')."""
        if self.estopped:
            self.get_logger().warn("E-STOP latch released by operator 'enable'.")
        self.estopped = False
        self.motors_disabled_logged = False
        self.set_motors(True)

    # ======================================================================
    #  3. THRUST -> PWM -- the enable_motors gate (N4) and the calibration
    # ======================================================================

    def manualMove(self, input, force=False):
        """
        Convert a [right, left] Newton command to PWM and stream it through RC
        override.
        """
        # Gate (N4 / CM-16). Closed, it HOLDS NEUTRAL rather than going silent:
        # in override the ESCs follow RC channels 1/3 from any transmitter, so
        # silence let motors spin with enable_motors:=False. Only in override;
        # outside it streaming would fight mode_callback's release.
        if not self.enable_motors and not force:
            if self.mode == 'override':
                if not self.motors_disabled_logged:
                    self.motors_disabled_logged = True
                    self.get_logger().warn(
                        "enable_motors is False - holding RC channels at neutral "
                        "1500/1500. Commanded thrust is logged but never applied.")
                self.send_rc_override(right_pwm=PWM_NEUTRAL, left_pwm=PWM_NEUTRAL)
            return

        def thrust_to_pwm(T): # Thrust in Newton
            return int(self.interpolator(T))

        # Compensate right thruster observed weaker output. DEAD: overridden by
        # 1.0 below, and it keys on input[1] while scaling input[0] -- left as
        # is until someone decides what it should do (CLAUDE.md §5).
        if input[1] >= 0:
            compensation_gain = 1.2
        else:
            compensation_gain = 0.75

        compensation_gain=1.0
        # UNIFORM saturation: one factor for both sides keeps the wrench
        # direction ([+45, +18] -> [+20, +8]); thrust_limits.py.
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

        max_PWM = 1900
        min_PWM = 1100
        right_pwm = np.clip(thrust_to_pwm(right), min_PWM, max_PWM)
        # Left reversed for its asymmetric propeller (intentional).
        left_pwm = 3000 - np.clip(thrust_to_pwm(left), min_PWM, max_PWM)

        # Every tick (~20 Hz), which also feeds ArduPilot's RC_OVERRIDE_TIME.
        self.send_rc_override(right_pwm=right_pwm, left_pwm=left_pwm)

    # ======================================================================
    #  4. OPERATOR COMMAND SURFACE
    #  /blueboat/input_str: enable, stop, override, default, arm, disarm, rearm, manual, move.
    # ======================================================================

    def str_input_callback(self, msg: String):
        """
        Dispatch one /blueboat/input_str command. An unrecognised first token
        falls through to move_callback (which needs exactly four fields).
        """
        input_string = msg.data.split()
        if not input_string:
            self.get_logger().warn("Empty input_str message ignored.")
            return
        command = input_string[0]

        dispatch = {'enable': self.release_estop,
                    'disable': lambda: self.set_motors(False),
                    'stop': self.full_stop,
                    'override': lambda: self.request_param_mode('override'),
                    'default': lambda: self.request_param_mode('default'),
                    'move': lambda: self.move_callback(input_string),
                    'arm': lambda: self.setArmedStatus(True),
                    'disarm': lambda: self.setArmedStatus(False),
                    'rearm': self.request_rearm,
                    # Autopilot mode -> MANUAL (station Turn Off with the guardian
                    # OFF, so BlueOS / QGC find the boat in their mode).
                    'manual': lambda: self.SetMode('MANUAL')
        }

        action = dispatch.get(command, lambda: self.move_callback(input_string))
        action()

    def move_callback(self, in_str):
        """
        'move <left> <right> <seconds>': apply that thrust (stored [right,
        left]) for that long. Needs exactly four fields.
        """
        if len(in_str) != 4:
            self.get_logger().info(" Incorrect move command.")
            return

        self.initial_time = time.time()
        left, right, self.manual_move_timer = map(float, in_str[1:])
        self.thruster_input = [right,left]

    def request_param_mode(self, mode):
        """
        Ask param_set for a mode; the main loop re-sends it until param_set
        confirms on /blueboat/param_mode (a single publish can be lost).
        """
        self.desired_param_mode = mode
        self.last_param_tx = time.time()
        self.publish(String(), mode, self.param_publisher)

    # ======================================================================
    #  5. POSE RE-ZEROING AND PINGER DEAD RECKONING
    #  Local ENU: position translated to the launch point, axes ENU, yaw
    #  ABSOLUTE ENU (never re-zeroed: that made a hybrid frame). The twist is
    #  passed through unrotated -- MAVROS gives it body-frame (N3).
    # ======================================================================

    def odom_callback(self, msg: Odometry):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            self.prev_time = t
            return
        dt = t - self.prev_time
        self.prev_time = t

        if not self.origin_set:
            self.x0 = msg.pose.pose.position.x
            self.y0 = msg.pose.pose.position.y
            self.z0 = msg.pose.pose.position.z
            self.yaw0 = cf.quaternion_to_yaw(msg.pose.pose.orientation)
            self.lat0 = self.gps_data[0]
            self.lon0 = self.gps_data[1]
            self.origin_set = True

        x_rel = msg.pose.pose.position.x - self.x0
        y_rel = msg.pose.pose.position.y - self.y0
        z_rel = msg.pose.pose.position.z - self.z0

        # Absolute ENU; yaw0 is provenance for the sidecar only.
        yaw = cf.quaternion_to_yaw(msg.pose.pose.orientation)

        self.relative_coordinates = [x_rel, y_rel, yaw]

        odom_out = Odometry()
        odom_out.header = msg.header
        odom_out.child_frame_id = msg.child_frame_id

        odom_out.pose.pose.position.x = x_rel
        odom_out.pose.pose.position.y = y_rel
        odom_out.pose.pose.position.z = z_rel
        odom_out.pose.pose.orientation = cf.yaw_to_quaternion(yaw)

        # Twist body-frame, unrotated (N3); covariances preserved.
        odom_out.twist = msg.twist
        odom_out.pose.covariance = msg.pose.covariance
        odom_out.twist.covariance = msg.twist.covariance

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

        # Dead-reckon the body-frame pinger vector between USBL fixes.
        if av is not None and not all(self.pinger_coordinates == np.zeros(3)): # Make sure the pinger has been detected
            omega = np.array([0.0, 0.0, av.z])
            p = self.pinger_coordinates

            self.pinger_coordinates -= (self.vel + np.cross(omega, p)) * dt
        
        self.publish(Float32MultiArray(), self.pinger_coordinates, self.pinger_publisher)

        # rotate pinger coordinates into the local-ENU world frame
        x_body = self.pinger_coordinates[0]
        y_body = self.pinger_coordinates[1]

        x_world, y_world = cf.transform_body_to_world(x_rel, y_rel, yaw, x_body, y_body) # now in the local-ENU world frame

        self.corrected_pinger = [x_world, y_world]

        # The world frame IS local ENU about (lat0, lon0).
        lat, lon = cf.enu_to_gps(self.lat0, self.lon0, x_world, y_world)
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
        self.robot_state = msg

    def uw_gps_callback(self, msg):
        """
        Cache the Water Linked UGPS packet (19 values: date x7, aco xyz,
        ant xyz, lat, lon, dep, filaco xyz) and reseed the pinger dead
        reckoning. Never writes the CSV (log_timer_callback does).
        """
        # In a session a pinger run can start at any time: always track it.
        if not (self.use_UWgps or self.session_mode):
            return

        self.uw_gps_log = list(msg.data)

        # filaco = the FILTERED acoustic position.
        t_x, t_y, t_z = msg.data[16], msg.data[17], msg.data[18]
        self.pinger_coordinates = np.array([t_x, t_y, t_z])

    def target_callback(self, msg: Float32MultiArray):
        self.target = msg.data

    def thr_input_callback(self, msg: Float32MultiArray):
        self.thruster_input = msg.data
        self.last_thr_rx = time.time()

    def monitoring_data_callback(self, msg: Float32MultiArray):
        self.monitoring_data = msg.data

    def param_callback(self, msg: Bool):
        """Log param_set's readiness on change only (it is heartbeated)."""
        if self._last_param_ready == msg.data:
            return
        self._last_param_ready = msg.data
        self.get_logger().info(f" Parameters ready: {msg.data}")

    def mode_callback(self, msg: String):
        """Track param_set's mode; on leaving override, release the RC channels."""
        previous_mode = self.mode
        self.mode = msg.data

        if previous_mode != self.mode:
            self.get_logger().info(f" Mode received: {self.mode}")

            if previous_mode == 'override':
                self.send_rc_override(right_pwm=PWM_NEUTRAL, left_pwm=PWM_NEUTRAL)
                self.send_rc_override(release=True)

    # ======================================================================
    #  7. MAVROS / MAVLINK PLUMBING -- RC override stream (N6), arming, mode
    # ======================================================================

    def set_servo(self, n, pwm):
        """
        LEGACY fallback, called from nowhere (N6): one MAV_CMD_DO_SET_SERVO.
        Works only with SERVOn_FUNCTION 0; never at loop rate (acknowledged
        RPC: a lost ACK stalls the mavros command plugin for seconds).
        """
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(n)
        req.param2 = float(pwm)

        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        self.cmd_client.call_async(req)

    def send_rc_override(self, right_pwm=None, left_pwm=None, release=False):
        """
        One RC override message: channel 1 = right (servo1), 3 = left (servo3).
        Acts only in 'override' (param_set's RCIN passthrough mapping).
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

    def setArmedStatus(self,command):
        """Arm or disarm the autopilot ('override' passthrough ignores arming)."""
        self.get_logger().info(f"{'Arming' if command else 'Disarming'} vehicle...")

        if self.arming_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = command
            self.arming_client.call_async(req)

    def SetMode(self, mode):
        self.get_logger().info(f"Current mode: {self.robot_state.mode}, switching to {mode}]")

        if self.mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = mode
            self.mode_client.call_async(req)

    def set_motors(self, inBool):
        """Open or close the enable_motors gate (N4)."""
        self.enable_motors = inBool
        self.get_logger().info(f" Enable motors: {self.enable_motors}")

    def publish(self, msg_type, in_msg, publisher):
        """Set .data on a message instance and publish it."""
        msg_type.data = in_msg
        publisher.publish(msg_type)

    # ======================================================================
    #  8. CSV LOGGING -- write-once field data (robot_log_schema.py); one
    #  writer for both layouts, on this node's own timer
    # ======================================================================

    def actuation_state(self):
        """
        Could the logged thrust command reach the water? 0 motors disabled
        (channels held at neutral), 1 live, 2 enabled but not in override,
        3 watchdog forcing zero (robot_log_schema).
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
        Record the local-ENU world frame's origin (lat0, lon0), once, beside
        the CSV: it georeferences every world-frame column afterwards.
        yaw0_rad is provenance only (pre-local-ENU logs used yaw - yaw0).
        """
        if self.origin_written or not self.origin_set:
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
        One CSV row, {column name: value} for every column of
        self.data_columns. Raises AttributeError before the first IMU message
        (the caller treats it as "not ready").
        """
        now = datetime.today()
        roll, pitch, _ = cf.quaternion_to_rpy(self.orientation)

        row = {
            # Wall clock; MicroSecond is full microseconds.
            'Year': now.year, 'Month': now.month, 'Day': now.day,
            'Hour': now.hour, 'Minute': now.minute, 'Second': now.second,
            'MicroSecond': now.microsecond,

            # Robot pose, local-ENU world frame (origin = launch point;
            # relative_psi is ABSOLUTE ENU yaw, 0 = East, CCW+).
            'relative_x': self.relative_coordinates[0],
            'relative_y': self.relative_coordinates[1],
            'relative_psi': self.relative_coordinates[2],

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
            # The pinger IS the target (world frame + WGS84, odom_callback).
            target_xy = self.corrected_pinger
            target_gps = self.pinger_gps
        else:
            # /monitoring_data[4:6] = x_d, y_d, world-frame in every branch
            # (N9) -- no correction here. Zeros before the first message.
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
            # /uw_gps_data[7:19]; 0..6 are its own date fields.
            ugps_names = ('aco_x', 'aco_y', 'aco_z',
                          'ant_x', 'ant_y', 'ant_z',
                          'lat', 'lon', 'dep',
                          'filaco_x', 'filaco_y', 'filaco_z')
            for name, value in zip(ugps_names, self.uw_gps_log[7:19]):
                row[name] = value

        return row

    def target_to_gps(self, target_xy):
        """[lat, lon] of a local-ENU world-frame [x, y], or [0, 0] before the
        frame origin is latched."""
        if not self.origin_set:
            return [0.0, 0.0]
        lat, lon = cf.enu_to_gps(self.lat0, self.lon0, target_xy[0], target_xy[1])
        return [lat, lon]

    def log_timer_callback(self):
        """Append one row to the position CSV (the single writer)."""
        if self.log_file is None or self.log_file.closed:
            return              # an idle run session: nothing is recorded
        try:
            row = self.build_log_row()
        except (AttributeError, TypeError, IndexError) as exc:
            # Normal before the first IMU / odom message; the reason is logged
            # so a genuine assembly bug is not hidden.
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

    # ======================================================================
    #  9. SHUTDOWN (N5) -- best-effort, safest-first, every step guarded
    # ======================================================================

    def shutdown(self):
        """
        Ordered teardown from main()'s finally: each step is independently
        guarded so one failure skips nothing and nothing raises out.
        """
        # 1. Motors first, always.
        try:
            self.full_stop()
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(f"Shutdown: full_stop failed: {exc}")

        # 2. RC channels back: neutral, then release.
        try:
            self.send_rc_override(right_pwm=PWM_NEUTRAL, left_pwm=PWM_NEUTRAL)
            self.send_rc_override(release=True)
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(f"Shutdown: RC release failed: {exc}")

        # 3. Default servo mapping (param_set runs its own restore too).
        try:
            self.request_param_mode('default')
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(f"Shutdown: 'default' request failed: {exc}")

        # 4. Close the CSV before anything moves it (none open in an idle session).
        had_open_log = False
        try:
            if getattr(self, 'log_file', None) and not self.log_file.closed:
                had_open_log = True
                self.log_file.flush()
                self.log_file.close()
                self.get_logger().info(f"Position log closed: {self.path}")
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(f"Shutdown: closing the log failed: {exc}")
        try:
            self._close_pathlog()
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(f"Shutdown: closing the path-update log failed: {exc}")

        # 4b. Close the bags (the launch teardown already SIGINTed the recorder;
        #     this waits for it, bounded).
        run_bag_dir = None
        try:
            run_bag_dir = self._bag_teardown()
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(f"Shutdown: closing the run bag failed: {exc}")

        if not had_open_log:
            return

        # 5. Mission report, synchronously. Imported here: matplotlib must never
        #    be able to stop this flight node from starting.
        try:
            import poslog_report
            folder = poslog_report.finalise_run(self.path, bag=run_bag_dir)
            self.get_logger().info(f"Mission report: {folder}")
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(
                f"Shutdown: mission report failed ({exc}). The CSV is intact; "
                f"run `ros2 run blueboat_control poslog_report.py {self.path}` "
                "to produce it later.")

    # ======================================================================
    #  10. RUN SESSION (session_mode only) -- one CSV per run, following
    #  master_control's /blueboat/run_state, reported on /blueboat/run_log.
    #  Also the run's path-update log (master_control section 7).
    # ======================================================================
    def _open_pathlog(self, run=None):
        """Attach this run's -pathlog.csv writer (file created at the first
        event). None for the pinger layout."""
        self._close_pathlog()
        if self.use_UWgps or not self.path:
            self.pathlog = None
            self.pathlog_pending = [e for e in self.pathlog_pending if e.get('run') != run]
            return
        self.pathlog = pul.PathLog(pul.pathlog_path(self.path))
        if run is not None:
            # Events of this run that arrived before its run_state did.
            held = [e for e in self.pathlog_pending if e.get('run') == run]
            self.pathlog_pending = [e for e in self.pathlog_pending if e.get('run') != run]
            for event in held:
                self._write_path_event(event)

    def _close_pathlog(self):
        if self.pathlog is None:
            return
        log, self.pathlog = self.pathlog, None
        log.close()
        if log.rows:
            self.get_logger().info(f"Path-update log closed ({log.rows} rows): {log.path}")

    def _write_path_event(self, event):
        if self.pathlog is None:
            return
        try:
            self.pathlog.write_event(event)
        except OSError as exc:
            self.get_logger().error(f"Path-update log not written: {exc}",
                                    throttle_duration_sec=10.0)

    def path_event_callback(self, msg: String):
        try:
            event = json.loads(msg.data)
            if not isinstance(event, dict):
                raise ValueError("not a JSON object")
        except ValueError as exc:
            self.get_logger().error(f"/blueboat/path_update_event: unreadable ({exc}) - ignored.",
                                    throttle_duration_sec=10.0)
            return
        if not self.session_mode:
            self._write_path_event(event)
        elif self.log_run is not None and event.get('run') == self.log_run:
            self._write_path_event(event)
        else:
            # A run whose log is not open (yet): its run_state has not arrived.
            self.pathlog_pending.append(event)
            del self.pathlog_pending[:-100]

    def run_state_callback(self, msg: String):
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict):
                raise ValueError("not a JSON object")
        except ValueError as exc:
            self.get_logger().error(f"/blueboat/run_state: unreadable ({exc}) - ignored.",
                                    throttle_duration_sec=10.0)
            return

        running = state.get('state') == 'running'
        self.run_running = running
        run = state.get('run')
        self.run_path_source = str(state.get('path_source', 'mission'))
        if running and self.log_run != run:
            # A run not recorded yet (also a new run whose stop we missed).
            if self.log_run is not None:
                self.close_run_log()
            self.open_run_log(run, bool(state.get('use_pinger', False)),
                              str(state.get('note', '')))
        elif not running and self.log_run is not None:
            self.close_run_log()

    def open_run_log(self, run, use_pinger, note):
        """Open this run's CSV in the layout of its follow mode."""
        self.use_UWgps = use_pinger
        self.data_columns = rls.columns_for(self.use_UWgps)
        self.target_cols, self.target_gps_cols = rls.target_columns_for(self.use_UWgps)
        # /monitoring_data's last value may predate this run.
        self.monitoring_data = []
        # File-name-safe note, the station's own rule (launch_manager.sanitize_note).
        safe_note = re.sub(r'-{2,}', '-', ''.join(
            c if (c.isalnum() or c in '._') else '-' for c in note)).strip('-')
        try:
            self.open_position_log(safe_note)
        except OSError as exc:
            self.log_file = None
            self.get_logger().error(f"Run {run}: cannot open the position log: {exc}")
            return
        self.log_run = run
        self._open_pathlog(run)
        self.get_logger().info(
            f"Run {run}: logging ({'pinger' if use_pinger else 'no-pinger'} layout).")
        if self.record_bag:
            self._bag_begin_run(run)
        if not use_pinger:
            self._snapshot_trajectory(run)
        self.publish_run_log()

    def _snapshot_trajectory(self, run):
        """Copy the run's mission file beside its CSV as `-path.yaml` (the
        station rewrites one session file per start). Never fatal. Skipped for
        a replanner run, whose path came from /blueboat/update_path."""
        if self.run_path_source == 'replanner':
            return
        traj = self.trajectory
        if not traj.startswith('from_yaml:'):
            return
        src = os.path.expanduser(traj.partition(':')[2])
        dst = re.sub(_POSLOG_RE, r'-path\1.yaml', self.path)
        try:
            shutil.copyfile(src, dst)
        except OSError as exc:
            self.get_logger().warning(f"Run {run}: path not snapshotted ({exc}).")

    def close_run_log(self):
        """Close the run's CSV and file its report in a subprocess (its own
        session: a matplotlib render in-process would hold the GIL against the
        20 Hz RC stream, and a launch teardown must not kill it half-way).
        """
        run = self.log_run
        self.log_run = None
        path = self.path
        try:
            if self.log_file is not None and not self.log_file.closed:
                self.log_file.flush()
                self.log_file.close()
                self.get_logger().info(f"Run {run}: position log closed: {path}")
        except Exception as exc:                       # noqa: BLE001 - reported
            self.get_logger().error(f"Run {run}: closing the log failed: {exc}")
        self.log_file = None
        self.last_csv = path or ''
        self._close_pathlog()

        if path:
            if self.bag_run is not None:
                # The report MOVES the bag, so it starts from the worker
                # thread that closes the recorder.
                self._bag_end_run(run, path)
            else:
                self._spawn_report(run, path)
        self.publish_run_log()

    def _spawn_report(self, run, path, bag=None):
        """poslog_report.py <csv> [--bag <dir>] as its own process and session.

        Called from the executor, or from a bag-closing worker thread."""
        try:
            import poslog_report
            cmd = [sys.executable, poslog_report.__file__, path]
            if bag:
                cmd += ['--bag', bag]
            proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            self.report_procs = [p for p in self.report_procs if p.poll() is None]
            self.report_procs.append(proc)
            self.get_logger().info(
                f"Run {run}: filing the report in the background "
                f"(Robot_data/{os.path.splitext(os.path.basename(path))[0]}/).")
        except Exception as exc:                       # noqa: BLE001 - reported
            extra = f" --bag {bag}" if bag else ""
            self.get_logger().error(
                f"Run {run}: report not started ({exc}). The CSV is intact; run "
                f"`ros2 run blueboat_control poslog_report.py {path}{extra}` later.")

    def publish_run_log(self):
        if not self.session_mode:
            return
        logging = self.log_file is not None and not self.log_file.closed
        state = {
            'run': self.log_run,
            'logging': logging,
            'csv': self.path if logging else self.last_csv,
            'layout': 'pinger' if self.use_UWgps else 'no_pinger',
            # Additive: `bag` = staging dir while recording, else the previous
            # run's filed bag.
            'bag_enabled': self.record_bag,
            'bag_recording': self.bag_recording,
            'bag': (self.bag_run.output_dir if self.bag_run is not None
                    else self.bag_last),
        }
        msg = String()
        msg.data = json.dumps(state)
        self.run_log_publisher.publish(msg)

    # ======================================================================
    #  11. RUN BAG (session_mode and record_bag only)
    #  A separate `ros2 bag record` process (run_bag.py): pre-armed PAUSED
    #  while idle, resumed at run start (no discovery gap), closed at stop on a
    #  worker thread that then files it with poslog_report.
    # ======================================================================

    def bag_timer_callback(self):
        if self.bag_shutting_down:
            return
        # Resume a run's recorder as soon as its service answers.
        if self.bag_run is not None:
            if self.bag_resume_pending:
                if self.bag_run.alive():
                    self._bag_try_resume()
                else:
                    self.get_logger().error(
                        f"Run bag: recorder {self.bag_run.node_name} died before "
                        "recording; starting an unpaused one.")
                    self._bag_start_unpaused(self.log_run)
            elif not self.bag_run.alive() and self.bag_recording:
                self.bag_recording = False
                self.get_logger().error(
                    f"Run bag: recorder {self.bag_run.node_name} exited mid-run "
                    f"(code {self.bag_run.proc.returncode}); the bag so far is "
                    "kept and filed at stop.", throttle_duration_sec=10.0)
            return

        now = time.monotonic()
        if self.bag_armed is not None and not self.bag_armed.alive():
            self.get_logger().warning(
                f"Run bag: pre-armed recorder exited (code "
                f"{self.bag_armed.proc.returncode}); re-arming.")
            self._bag_drop_armed()
            self.bag_next_arm = now + 5.0
        if self.bag_armed is None and self.log_run is None and now >= self.bag_next_arm:
            try:
                self.bag_armed, self.bag_armed_client = self._bag_new(paused=True)
            except OSError as exc:
                self.bag_next_arm = now + 10.0
                self.get_logger().error(f"Run bag: cannot start the recorder ({exc}).",
                                        throttle_duration_sec=30.0)

    def _bag_new(self, paused):
        """Spawn a recorder into a fresh staging directory (+ its resume client)."""
        log_dir = cf.ensure_data_dir(self, self.data_root, 'data', 'Robot_data')
        self.bag_k += 1
        name = f'blueboat_run_bag_{os.getpid()}_{self.bag_k}'
        rec = run_bag.RunBagRecorder(run_bag.staging_dir(log_dir, self.bag_k), name)
        rec.start(paused=paused)
        client = self.create_client(Resume, rec.resume_service) if paused else None
        return rec, client

    def _bag_drop_armed(self):
        rec, client = self.bag_armed, self.bag_armed_client
        self.bag_armed, self.bag_armed_client = None, None
        if client is not None:
            self.destroy_client(client)
        if rec is not None:
            rec.stop_async(on_closed=lambda r: shutil.rmtree(r.output_dir, ignore_errors=True))

    def _bag_begin_run(self, run):
        rec, client = self.bag_armed, self.bag_armed_client
        self.bag_armed, self.bag_armed_client = None, None
        if rec is not None and rec.alive():
            self.bag_run, self.bag_run_client = rec, client
            self.bag_resume_pending = True
            self._bag_try_resume()
            return
        if rec is not None:
            if client is not None:
                self.destroy_client(client)
            rec.stop_async(on_closed=lambda r: shutil.rmtree(r.output_dir, ignore_errors=True))
        self.get_logger().warning(
            f"Run {run}: no pre-armed bag recorder; starting one now - the first "
            "one to two seconds of the run may be missing from the bag.")
        self._bag_start_unpaused(run)

    def _bag_start_unpaused(self, run):
        if self.bag_run_client is not None:
            self.destroy_client(self.bag_run_client)
        self.bag_run_client = None
        if self.bag_run is not None:
            # A recorder that never got resumed recorded nothing: drop it.
            self.bag_run.stop_async(
                on_closed=lambda r: shutil.rmtree(r.output_dir, ignore_errors=True))
            self.bag_run = None
        self.bag_resume_pending = False
        try:
            self.bag_run, _ = self._bag_new(paused=False)
            self.bag_recording = True
        except OSError as exc:
            self.bag_run = None
            self.bag_recording = False
            self.get_logger().error(f"Run {run}: no bag for this run ({exc}).")

    def _bag_try_resume(self):
        client = self.bag_run_client
        if client is None or not client.service_is_ready():
            return
        self.bag_resume_pending = False
        rec = self.bag_run
        future = client.call_async(Resume.Request())

        def done(fut, rec=rec):
            if rec is not self.bag_run:
                return
            if fut.exception() is not None:
                self.get_logger().error(f"Run bag: resume failed ({fut.exception()}); "
                                        "starting an unpaused recorder.")
                self._bag_start_unpaused(self.log_run)
                return
            self.bag_recording = True
            self.get_logger().info(f"Run {self.log_run}: bag recording -> {rec.output_dir}")
            self.publish_run_log()
        future.add_done_callback(done)

    def _bag_end_run(self, run, csv_path):
        """Close this run's recorder off-thread, then file the report with it."""
        import poslog_report
        rec, client = self.bag_run, self.bag_run_client
        self.bag_run, self.bag_run_client = None, None
        self.bag_resume_pending = False
        self.bag_recording = False
        if client is not None:
            self.destroy_client(client)
        stem = os.path.splitext(os.path.basename(csv_path))[0]
        self.bag_last = os.path.join(os.path.dirname(csv_path), stem,
                                     poslog_report.bag_name(csv_path))
        # Delay re-arming so two recorders do not subscribe at once.
        self.bag_next_arm = time.monotonic() + 1.0

        def closed(r):
            self.get_logger().info(
                f"Run {run}: bag closed (code {r.returncode}"
                f"{'' if r.has_data() else ', no metadata.yaml - ros2 bag reindex it'}).")
            self._spawn_report(run, csv_path, bag=r.output_dir)

        def failed(r, exc):
            self.get_logger().error(f"Run {run}: closing the bag failed ({exc}); "
                                    f"filing the CSV without it ({r.output_dir}).")
            self._spawn_report(run, csv_path)

        self.bag_closers = [t for t in self.bag_closers if t.is_alive()]
        self.bag_closers.append(rec.stop_async(on_closed=closed, on_error=failed))

    def _bag_teardown(self):
        """Shutdown half of section 11. Returns the open run's bag directory
        (closed, ready to be filed with its CSV), or None."""
        self.bag_shutting_down = True
        run_dir = None
        if self.bag_run is not None:
            self.bag_run.stop()
            run_dir = self.bag_run.output_dir
            self.bag_run = None
            self.bag_recording = False
        if self.bag_armed is not None:
            self.bag_armed.discard()
            self.bag_armed = None
        # Let closers of stopped runs finish and start their reports.
        for thread in self.bag_closers:
            thread.join(timeout=run_bag.SIGINT_WAIT_S + run_bag.SIGTERM_WAIT_S)
        return run_dir


def main():
    rclpy.init()
    node = BlueBoatController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


main()
