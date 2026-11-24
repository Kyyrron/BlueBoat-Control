#!/usr/bin/env python3

# ============================================================================
# robot_interface's simulation counterpart: Gazebo thrust bridge, readiness
# handshake (latched AND re-published every second), the same watchdog and
# thrust saturation, and the SAME position CSV, so a simulated and a field run
# share one reader (N2 / CM-2). Not duplicated: the pinger (no-pinger layout
# only), MAVROS / the enable_motors gate / the parameter mode (a command always
# reaches Gazebo), and the IMU (attitude and rates come from /blueboat/odom).
#
# Since 2026-11-13 it also carries the two things the real boat has between
# master_control and the water that Gazebo did not: the ACTUATOR MODEL of the
# identified plant (what a commanded Newton becomes at the propeller; read from
# the robot description, absent in the legacy plant) and the FIELD ODOMETRY
# TIMING (Gazebo's 20 Hz odometry republished on /blueboat/odom at the field
# rate and delay, as robot_interface republishes MAVROS odometry).
#
# FILE MAP (class Controller):
#   1. WIRING            __init__
#   2. MAIN LOOP         thr_input_callback, thruster_input_stale, move,
#                        description_callback, applied_thrust
#   3. STATE/TELEMETRY   raw_odom_callback, release_odom, odom_callback,
#                        gps_callback, monitoring_data_callback
#   4. CSV LOGGING       actuation_state, write_origin_sidecar, build_log_row,
#                        to_gps, log_timer_callback, path_event_callback
#   5. SHUTDOWN          shutdown
# ============================================================================

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import rclpy

import json
import math
import os
import re
import time
import xml.etree.ElementTree as ET
from collections import deque
from datetime import datetime
import numpy as np

from std_msgs.msg import Bool, Float32MultiArray, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy

from blueboat_control import ROV
import custom_functions as cf
import robot_log_schema as rls   # CSV column layout (ROS-free)
import thrust_limits as tl       # uniform thrust saturation (ROS-free)
import path_update_log as pul    # the run's -pathlog.csv (ROS-free)
import odom_emulation as oe      # field odometry timing (ROS-free)


class Controller(Node):

    # ======================================================================
    #  1. WIRING
    # ======================================================================

    def __init__(self):

        super().__init__('pid_sim', namespace='blueboat')

        self.declare_parameter('controller_type', 'MPC')
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value

        # Same names, semantics and defaults as robot_interface (CM-2).
        self.declare_parameter('thruster_input_timeout', 0.5)
        self.thruster_input_timeout = self.get_parameter('thruster_input_timeout').get_parameter_value().double_value

        self.declare_parameter('thrust_limit', 20.0)
        self.thrust_limit = self.get_parameter('thrust_limit').get_parameter_value().double_value

        # Additive (2026-11-07): the clock of the thrust watchdog and the
        # ready republish. 'wall' (default): time.time(), as before. 'ros':
        # the node clock (/clock), so a simulation slowed below real time
        # (Gazebo real-time factor < 1) does not trip the 0.5 s watchdog
        # between master_control's sim-time 20 Hz commands. Every simulation
        # launch passes 'ros'.
        self.declare_parameter('time_source', 'wall')
        self.time_source = self.get_parameter('time_source').get_parameter_value().string_value
        if self.time_source not in ('wall', 'ros'):
            raise ValueError(f"unknown time_source '{self.time_source}' (wall | ros)")

        self.declare_parameter('note', 'sim')
        self.note = self.get_parameter('note').get_parameter_value().string_value

        self.declare_parameter('data_dir', '')
        self.data_root = cf.data_root(
            self.get_parameter('data_dir').get_parameter_value().string_value)

        # ---- field-equivalent loop timing and actuator model (2026-11-13) ----
        # All additive. The values that reproduce the simulation before that
        # date are thrust_rate_hz 10, odom_profile passthrough (with the Gazebo bridge
        # on /blueboat/odom) and the legacy plant, whose description carries no
        # actuator model: Sim_launch.py sim_config:=legacy sets them together.
        #
        # Rate at which the latest /thruster_input is applied to the
        # thrusters. robot_interface streams RC override at 20 Hz.
        self.declare_parameter('thrust_rate_hz', 20.0)
        self.thrust_rate_hz = self.get_parameter('thrust_rate_hz').get_parameter_value().double_value
        if not self.thrust_rate_hz > 0.0:
            raise ValueError(f"thrust_rate_hz must be positive, got {self.thrust_rate_hz}")
        # Odometry timing applied when Gazebo's odometry arrives on the raw
        # topic /blueboat/odom_gz (upload_rov_launch.py odom_raw:=true):
        # passthrough (forwarded as is) | field_3hz | field_4hz | field_13hz
        # (odom_emulation.py: where each comes from).
        self.declare_parameter('odom_profile', 'field_13hz')
        self.odom_profile = self.get_parameter('odom_profile').get_parameter_value().string_value
        if self.odom_profile not in oe.profile_names():
            raise ValueError(f"unknown odom_profile '{self.odom_profile}' "
                             f"({' | '.join(oe.profile_names())})")
        self.declare_parameter('odom_seed', 0)
        self.declare_parameter('odom_base_delay', 0.0)
        self.odom_emulator = None
        if self.odom_profile != oe.OFF:
            self.odom_emulator = oe.OdomEmulator(
                self.odom_profile,
                self.get_parameter('odom_seed').get_parameter_value().integer_value,
                self.get_parameter('odom_base_delay').get_parameter_value().double_value)
        # Actuator model: applied = gain_side * (T - prop_speed_loss * sqrt|T| * surge).
        # Negative values (the defaults) mean "what the robot description
        # says" - the identified plant states its own, the legacy plant none
        # (gain 1, no loss: a commanded Newton is an applied Newton).
        self.declare_parameter('thrust_gain', [-1.0, -1.0])        # [right, left]
        self.declare_parameter('prop_speed_loss', -1.0)            # sqrt(N).s/m per thruster
        self.declare_parameter('esc_deadband', 0.0)                # N, |command| below it gives no thrust
        self.declare_parameter('thrust_lag_tau', 0.0)              # s, first-order spin-up lag
        self.thrust_gain_param = list(self.get_parameter('thrust_gain').get_parameter_value().double_array_value)
        if len(self.thrust_gain_param) != 2:
            raise ValueError("thrust_gain expects [right, left]")
        self.prop_speed_loss_param = self.get_parameter('prop_speed_loss').get_parameter_value().double_value
        self.esc_deadband = self.get_parameter('esc_deadband').get_parameter_value().double_value
        self.thrust_lag_tau = self.get_parameter('thrust_lag_tau').get_parameter_value().double_value
        self.actuator = None              # (gain_right, gain_left, speed_loss), set by the description
        self.actuator_identity = True     # nothing to apply: the legacy path, untouched
        self.lag_state = [0.0, 0.0]
        self.true_surge = 0.0             # body surge speed the propellers see [m/s]

        # No Water Linked UGPS in simulation: fixed, not a parameter.
        self.use_UWgps = False

        self.rov = ROV(self, thrust_visual = True)
        self.description_sub = self.create_subscription(
            String, 'robot_description', self.description_callback,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.odom_sim_subscriber = self.create_subscription(Odometry, '/blueboat/odom', self.odom_callback, 10)
        # Raw Gazebo odometry, when the bridge is retargeted to it. The
        # /blueboat/odom publisher is created at the first raw message, so a
        # launch that keeps the bridge on /blueboat/odom is untouched.
        self.raw_odom_seen = False
        self.odom_relay_refused = False
        self.odom_publisher = None
        self.odom_queue = deque()         # (release time on the node clock, message)
        self.raw_odom_sub = self.create_subscription(Odometry, '/blueboat/odom_gz', self.raw_odom_callback, 50)
        self.odom_release_timer = self.create_timer(0.005, self.release_odom)
        self.thruster_input_sub = self.create_subscription(Float32MultiArray, "/thruster_input", self.thr_input_callback,10)

        # Logging inputs. The NavSatFix (BEST_EFFORT like every consumer)
        # comes from the station's bridge or the simulator's mavros shim in a
        # GPS-anchored run; nothing publishes it under a plain Sim_launch.
        self.monitoring_sub = self.create_subscription(
            Float32MultiArray, "/monitoring_data", self.monitoring_data_callback, 10)
        gps_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, gps_qos)
        # The same -pathlog.csv robot_interface writes on the real boat.
        self.path_event_sub = self.create_subscription(
            String, '/blueboat/path_update_event', self.path_event_callback,
            QoSProfile(depth=50))

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_publisher = self.create_publisher(Bool, '/blueboat/controller_ready', latched)
        self.data_publisher = self.create_publisher(Float32MultiArray, "/monitoring_data", 10)

        self.timer = self.create_timer(1.0 / self.thrust_rate_hz, self.move)
        self.log_timer = self.create_timer(0.33, self.log_timer_callback)

        self.thr_input = [0,0]

        self.last_thr_rx = None
        self.thr_watchdog_tripped = False

        self.sent_ready = False
        # Re-published, not only latched: master_control subscribes VOLATILE
        # (CM-4), so a late match missed the one latched sample.
        self.last_ready_tx = 0.0
        self.ready_republish_period = 1.0

        # ---- logged state (section 3), all safe to log before any message ----
        self.relative_coordinates = [0.0, 0.0, 0.0]   # x, y, yaw (local ENU)
        self.roll = 0.0
        self.pitch = 0.0
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)
        self.monitoring_data = []
        self.gps_data = [0.0, 0.0]

        # /blueboat/odom is already local ENU (Gazebo world origin): nothing to
        # re-zero; yaw0 is sidecar provenance. geo_anchor = (lat, lon, x, y):
        # the first VALID fix and the world position held when it arrived.
        self.yaw0 = None
        self.geo_anchor = None
        # No row before the first odom: zero rows would read as a boat at the
        # origin, not as no data.
        self.odom_seen = False

        # Previous body-frame velocity, for the differentiated acceleration.
        self.prev_twist = None
        self.prev_twist_t = None

        # ---- position CSV (write-once field record, robot_log_schema.py) ----
        self.data_columns = rls.columns_for(self.use_UWgps)
        self.target_cols, self.target_gps_cols = rls.target_columns_for(self.use_UWgps)

        # File name on the wall clock (sorts with field runs); rows on sim time.
        self.date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        log_dir = cf.ensure_data_dir(self, self.data_root, 'data', 'Robot_data')
        stem = f'{self.date}-{self.note}-poslog' if self.note else f'{self.date}-poslog'
        self.path = cf.reserve_run_file(log_dir, stem, '.csv') + '.csv'
        self.origin_path = re.sub(r'-poslog(-\d+)?\.csv$', r'-origin\1.yaml', self.path)

        self.log_file = open(self.path, 'w', buffering=1)
        for line in rls.legend_rows(self.data_columns):
            self.log_file.write(','.join(line) + '\n')
        self.log_file.write(','.join(self.data_columns) + '\n')
        self.log_file.flush()
        self.origin_written = False
        self.get_logger().info(f"Position log: {self.path}")
        # Path-update log beside the CSV, created at the first request.
        self.pathlog = pul.PathLog(pul.pathlog_path(self.path))

    # ======================================================================
    #  2. MAIN LOOP
    # ======================================================================

    def thr_input_callback(self, msg: Float32MultiArray):
        self.thr_input = msg.data
        self.last_thr_rx = self._world_clock()

    def _world_clock(self):
        """Seconds on the watchdog clock (parameter ``time_source``)."""
        if self.time_source == 'ros':
            return self.get_clock().now().nanoseconds * 1e-9
        return time.time()

    def thruster_input_stale(self, now):
        """Loss-of-reference watchdog (robot_interface.thruster_input_stale)."""
        if self.last_thr_rx is None:
            return True
        return (now - self.last_thr_rx) > self.thruster_input_timeout

    def description_callback(self, msg: String):
        """The actuator model the robot description states for its plant.

        hydrodynamics_identified.xacro carries <thrust_gain_right>,
        <thrust_gain_left> and <prop_speed_loss> inside its Hydrodynamics
        plugin element (Gazebo ignores them); the legacy description has none.
        A non-negative parameter overrides the description.
        """
        if self.actuator is not None:
            return
        found = {}
        try:
            for plugin in ET.fromstring(msg.data).iter('plugin'):
                if 'Hydrodynamics' in (plugin.get('name') or ''):
                    for tag in ('thrust_gain_right', 'thrust_gain_left', 'prop_speed_loss', 'plant_name'):
                        node = plugin.find(tag)
                        if node is not None and node.text:
                            found[tag] = node.text.strip()
        except ET.ParseError as exc:
            self.get_logger().error(f"robot_description unreadable ({exc}): no actuator model applied.")
        gain = [float(found.get('thrust_gain_right', 1.0)), float(found.get('thrust_gain_left', 1.0))]
        loss = float(found.get('prop_speed_loss', 0.0))
        for i in range(2):
            if self.thrust_gain_param[i] >= 0.0:
                gain[i] = self.thrust_gain_param[i]
        if self.prop_speed_loss_param >= 0.0:
            loss = self.prop_speed_loss_param
        self.actuator = (gain[0], gain[1], loss)
        self.actuator_identity = (gain == [1.0, 1.0] and loss == 0.0
                                  and self.esc_deadband <= 0.0 and self.thrust_lag_tau <= 0.0)
        self.get_logger().info(
            f"Plant '{found.get('plant_name', 'bluerov2_legacy (no actuator model)')}': thrust gain "
            f"[right {gain[0]:.3f}, left {gain[1]:.3f}], propeller speed loss {loss:.2f} sqrt(N).s/m, "
            f"deadband {self.esc_deadband:.1f} N, lag {self.thrust_lag_tau:.2f} s; "
            f"thrust applied at {self.thrust_rate_hz:.0f} Hz.")

    def applied_thrust(self, r, l):
        """What the commanded [right, left] Newtons become at the propellers.

        Per thruster, in this order: ESC deadband, first-order spin-up lag,
        loss of thrust with the boat's surge speed (a propeller at a given
        speed of rotation pulls less the faster the water arrives), side gain.
        The CSV and /monitoring_data keep the COMMAND, as on the real boat.
        """
        gain_r, gain_l, loss = self.actuator
        out = []
        for i, (cmd, gain) in enumerate(((r, gain_r), (l, gain_l))):
            if self.esc_deadband > 0.0 and abs(cmd) < self.esc_deadband:
                cmd = 0.0
            if self.thrust_lag_tau > 0.0:
                a = min(1.0, (1.0 / self.thrust_rate_hz) / self.thrust_lag_tau)
                self.lag_state[i] += a * (cmd - self.lag_state[i])
                cmd = self.lag_state[i]
            out.append(gain * (cmd - loss * math.sqrt(abs(cmd)) * self.true_surge))
        return out[0], out[1]

    def move(self):
        if not self.rov.ready() or self.actuator is None:
            return

        if not self.sent_ready:
            msg = Bool()
            msg.data = True
            self.ready_publisher.publish(msg)
            self.get_logger().info(f'Ready publishing: {msg.data}')
            self.sent_ready = True
            self.last_ready_tx = self._world_clock()
        elif self._world_clock() - self.last_ready_tx > self.ready_republish_period:
            self.last_ready_tx = self._world_clock()
            self.ready_publisher.publish(Bool(data=True))

        if self.thruster_input_stale(self._world_clock()):
            if not self.thr_watchdog_tripped:
                self.thr_watchdog_tripped = True
                self.get_logger().warn(
                    f"No /thruster_input for {self.thruster_input_timeout:.2f} s "
                    "- zeroing thrust.")
            self.thr_input = [0,0]
        elif self.thr_watchdog_tripped:
            self.thr_watchdog_tripped = False
            self.get_logger().info("/thruster_input resumed - releasing watchdog.")

        # Uniform saturation as on the real boat (normally a no-op:
        # master_control already limits at its publisher).
        limited, scale = tl.scale_to_limit(self.thr_input, self.thrust_limit)
        if scale < 1.0:
            self.get_logger().warn(
                f"Thrust {list(self.thr_input)} exceeds thrust_limit="
                f"{self.thrust_limit:.1f} N - scaled by {scale:.3f}.",
                throttle_duration_sec=2.0)

        r, l = float(limited[0]), float(limited[1])
        if not self.actuator_identity:
            r, l = self.applied_thrust(r, l)
        self.rov.move([r,l])

    # ======================================================================
    #  3. STATE / TELEMETRY -- nothing here commands anything
    # ======================================================================

    def raw_odom_callback(self, msg: Odometry):
        """Gazebo's own odometry (/blueboat/odom_gz) -> /blueboat/odom.

        Forwarded as is with odom_profile 'passthrough', else thinned and delayed to
        the field pattern (odom_emulation.py). The header stamp is kept: it is
        the time the pose was true, as on the real boat.
        """
        self.true_surge = float(msg.twist.twist.linear.x)
        if self.odom_relay_refused:
            return
        if not self.raw_odom_seen:
            self.raw_odom_seen = True
            others = self.count_publishers('/blueboat/odom')
            if others > 0:
                self.odom_relay_refused = True
                self.get_logger().error(
                    f"/blueboat/odom already has {others} publisher(s) while raw odometry arrives on "
                    "/blueboat/odom_gz: the odometry is NOT relayed (two sources would alternate). "
                    "Launch with either odom_raw:=true or the bridge on /blueboat/odom, not both.")
                return
            self.odom_publisher = self.create_publisher(Odometry, '/blueboat/odom', 10)
            self.get_logger().info(
                "Relaying /blueboat/odom_gz -> /blueboat/odom, "
                + ("unchanged (odom_profile passthrough)." if self.odom_emulator is None
                   else f"at the field timing '{self.odom_profile}'."))
        if self.odom_emulator is None:
            self.odom_publisher.publish(msg)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        release = self.odom_emulator.offer(stamp)
        if release is None:
            return
        # the delay is counted from the arrival of the raw sample
        self.odom_queue.append((now + (release - stamp), msg))

    def release_odom(self):
        """Publish the queued odometry samples whose delay has elapsed."""
        if not self.odom_queue:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.odom_queue and self.odom_queue[0][0] <= now:
            self.odom_publisher.publish(self.odom_queue.popleft()[1])

    def odom_callback(self, msg: Odometry):
        pose, twist = cf.odometry(msg)

        self.rov.current_pose = pose
        self.rov.current_twist = twist
        if not self.raw_odom_seen:
            # bridge on /blueboat/odom: this IS Gazebo's odometry
            self.true_surge = float(msg.twist.twist.linear.x)

        # --- the rest is for the log only (odom is already local ENU) -------
        roll, pitch, yaw = cf.quaternion_to_rpy(msg.pose.pose.orientation)
        self.relative_coordinates = [msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     yaw]
        self.roll = roll
        self.pitch = pitch

        av = msg.twist.twist.angular
        self.angular_velocity = np.array([av.x, av.y, av.z])

        # lin_acc_*: the body-frame velocity differentiated over sim time (no
        # simulated IMU) -- NO gravity component, and 20 Hz difference noise.
        lv = msg.twist.twist.linear
        v = np.array([lv.x, lv.y, lv.z])
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_twist is not None and t > self.prev_twist_t:
            self.linear_acceleration = (v - self.prev_twist) / (t - self.prev_twist_t)
        self.prev_twist = v
        self.prev_twist_t = t

        if self.yaw0 is None:
            self.yaw0 = yaw
        self.odom_seen = True

        # Geodetic anchor on the first VALID fix (fixes may start after odom);
        # none all run (plain Sim_launch) leaves the GPS columns zero.
        if self.geo_anchor is None and not (self.gps_data[0] == 0.0
                                            and self.gps_data[1] == 0.0):
            self.geo_anchor = (self.gps_data[0], self.gps_data[1],
                               msg.pose.pose.position.x,
                               msg.pose.pose.position.y)

    def gps_callback(self, msg: NavSatFix):
        # (0, 0) = no fix, stored as-is (the anchor test ignores it).
        self.gps_data = [msg.latitude, msg.longitude]

    def monitoring_data_callback(self, msg: Float32MultiArray):
        """Cache /monitoring_data; its [4:6] target is world-frame (N9)."""
        self.monitoring_data = msg.data

    # ======================================================================
    #  4. CSV LOGGING -- same writer, columns and rate as robot_interface's
    # ======================================================================

    def actuation_state(self):
        """
        Only ACT_LIVE or ACT_WATCHDOG in simulation: no motor gate and no
        autopilot mode exist here (robot_log_schema encoding).
        """
        if self.thr_watchdog_tripped:
            return rls.ACT_WATCHDOG
        return rls.ACT_LIVE

    def write_origin_sidecar(self):
        """
        Record the world frame's origin, once, beside the CSV: written with the
        first logged row (lat/lon 0 if no GPS anchor exists yet), or at shutdown
        if no row was logged. yaw0_rad is the spawn heading, provenance only.
        """
        if self.origin_written or self.yaw0 is None:
            return
        if self.geo_anchor is None:
            lat0 = lon0 = 0.0
        else:
            # Project the anchor fix, taken at (xa, ya), back to the origin.
            lat_a, lon_a, xa, ya = self.geo_anchor
            lat0, lon0 = cf.enu_to_gps(lat_a, lon_a, -xa, -ya)
        try:
            with open(self.origin_path, 'w') as f:
                f.write('# Origin of the local-ENU world frame (Gazebo world\n'
                        '# origin; axes East/North, yaw absolute ENU) used by\n'
                        '# every world-frame column of the CSV beside this file.\n'
                        '# SIMULATED RUN. latitude/longitude are 0 when no GPS\n'
                        '# publisher was present, which is the plain Sim_launch\n'
                        '# case; yaw0_rad is the spawn heading, provenance only.\n')
                f.write(f'latitude: {lat0}\n')
                f.write(f'longitude: {lon0}\n')
                f.write(f'yaw0_rad: {self.yaw0}\n')
                f.write(f'poslog: {os.path.basename(self.path)}\n')
                f.write('source: simulation\n')
            self.origin_written = True
            self.get_logger().info(f"Frame origin: {self.origin_path}")
        except OSError as exc:
            self.get_logger().warn(f"Could not write the frame origin sidecar: {exc}")

    def build_log_row(self):
        """One no-pinger CSV row, {column name: value} (27 columns)."""
        # ROS clock = SIM TIME (dates read 1970-01-01): speeds and durations
        # must use the clock the physics ran on. Local time, like robot_interface.
        now = datetime.fromtimestamp(self.get_clock().now().nanoseconds * 1e-9)

        row = {
            'Year': now.year, 'Month': now.month, 'Day': now.day,
            'Hour': now.hour, 'Minute': now.minute, 'Second': now.second,
            'MicroSecond': now.microsecond,

            # Local-ENU world frame; relative_psi is absolute ENU yaw.
            'relative_x': self.relative_coordinates[0],
            'relative_y': self.relative_coordinates[1],
            'relative_psi': self.relative_coordinates[2],

            'gps_latitude': self.gps_data[0],
            'gps_longitude': self.gps_data[1],

            # [right, left] as commanded (the watchdog writes [0, 0] here).
            'right_thr_in': self.thr_input[0],
            'left_thr_in': self.thr_input[1],
            'actuation_state': self.actuation_state(),

            'roll': self.roll,
            'pitch': self.pitch,

            'ang_vel_x': self.angular_velocity[0],
            'ang_vel_y': self.angular_velocity[1],
            'ang_vel_z': self.angular_velocity[2],

            # Differentiated odometry, no gravity (odom_callback).
            'lin_acc_x': self.linear_acceleration[0],
            'lin_acc_y': self.linear_acceleration[1],
            'lin_acc_z': self.linear_acceleration[2],
        }

        # x_d, y_d (world frame, N9); zeros before the first message.
        if len(self.monitoring_data) >= 6:
            target_xy = [self.monitoring_data[4], self.monitoring_data[5]]
        else:
            target_xy = [0.0, 0.0]

        row[self.target_cols[0]] = target_xy[0]
        row[self.target_cols[1]] = target_xy[1]
        target_gps = self.to_gps(target_xy)
        row[self.target_gps_cols[0]] = target_gps[0]
        row[self.target_gps_cols[1]] = target_gps[1]

        return row

    def to_gps(self, xy):
        """[lat, lon] of a world-frame [x, y], or [0, 0] before a valid fix."""
        if self.geo_anchor is None:
            return [0.0, 0.0]
        lat0, lon0, xa, ya = self.geo_anchor
        lat, lon = cf.enu_to_gps(lat0, lon0, xy[0] - xa, xy[1] - ya)
        return [lat, lon]

    def log_timer_callback(self):
        """Append one row to the position CSV (the single writer)."""
        if not self.odom_seen:
            self.get_logger().warn(" -- Not ready to log yet: no /blueboat/odom",
                                   throttle_duration_sec=5.0)
            return

        try:
            row = self.build_log_row()
        except (AttributeError, TypeError, IndexError) as exc:
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

    def path_event_callback(self, msg: String):
        """One /blueboat/path_update_event -> one row of the -pathlog.csv."""
        try:
            event = json.loads(msg.data)
            if not isinstance(event, dict):
                raise ValueError("not a JSON object")
        except ValueError as exc:
            self.get_logger().error(f"/blueboat/path_update_event: unreadable ({exc}) - ignored.",
                                    throttle_duration_sec=10.0)
            return
        try:
            self.pathlog.write_event(event)
        except OSError as exc:
            self.get_logger().error(f"Path-update log not written: {exc}",
                                    throttle_duration_sec=10.0)

    # ======================================================================
    #  5. SHUTDOWN -- mirrors robot_interface's; every step guarded
    # ======================================================================

    def shutdown(self):
        # 1. Thrusters (only once the URDF is parsed: ROV.move needs its
        #    publishers).
        try:
            if self.rov.parsed():
                self.rov.move([0.0, 0.0])
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(f"Shutdown: zeroing the thrusters failed: {exc}")

        # 2. The sidecar, if no row has written it yet (no GPS all run).
        try:
            self.write_origin_sidecar()
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(f"Shutdown: origin sidecar failed: {exc}")

        # 3. Close the CSV BEFORE anything moves it.
        try:
            if getattr(self, 'log_file', None) and not self.log_file.closed:
                self.log_file.flush()
                self.log_file.close()
                self.get_logger().info(f"Position log closed: {self.path}")
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(f"Shutdown: closing the log failed: {exc}")
        try:
            if getattr(self, 'pathlog', None) is not None:
                self.pathlog.close()
                if self.pathlog.rows:
                    self.get_logger().info(
                        f"Path-update log closed ({self.pathlog.rows} rows): {self.pathlog.path}")
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(f"Shutdown: closing the path-update log failed: {exc}")

        # 4. Mission report (matplotlib imported lazily, never at module scope).
        try:
            import poslog_report
            folder = poslog_report.finalise_run(self.path)
            self.get_logger().info(f"Mission report: {folder}")
        except Exception as exc:                       # noqa: BLE001 - teardown
            self.get_logger().error(
                f"Shutdown: mission report failed ({exc}). The CSV is intact; "
                f"run `ros2 run blueboat_control poslog_report.py {self.path}` "
                "to produce it later.")


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # A launch teardown arrives as ExternalShutdownException.
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


main()
