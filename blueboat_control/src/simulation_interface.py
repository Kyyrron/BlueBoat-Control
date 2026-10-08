#!/usr/bin/env python3

# ============================================================================
# Gazebo thrust bridge, sim-side readiness handshake, and the position CSV.
#
# This node is robot_interface's simulation counterpart: same topics, same
# watchdog, same thrust saturation, and -- since this revision -- the SAME
# POSITION LOG. A simulated run and a field run now produce byte-compatible
# artifacts, so poslog_report.py, the offline notebooks and any sim-to-real
# comparison read one format rather than two (N2 / CM-2).
#
# What is deliberately NOT duplicated from robot_interface:
#   * the pinger. There is no Water Linked UGPS in simulation, so this node
#     fixes use_UWgps = False and writes COLUMNS_NO_PINGER (27 columns) only.
#   * MAVROS, the enable_motors gate and the parameter mode. Gazebo has no
#     autopilot to be in override of, so a command reaching this node always
#     reaches the water -- see actuation_state() for what that means for the
#     column of the same name.
#   * the IMU. Gazebo publishes no /mavros/imu/data under Sim_launch, so the
#     attitude and rate columns come from /blueboat/odom instead; see
#     odom_callback for the one column (lin_acc_*) where that is not a
#     like-for-like substitution.
#
# Column layout, its revision history and the actuation_state encoding live in
# _custom_libraries/robot_log_schema.py. Rows are filled BY COLUMN NAME so the
# two can never desynchronise.
# ============================================================================

# ----------------------------------------------------------------------------
# FILE MAP (class Controller)
#
#   1. WIRING                 __init__
#   2. MAIN LOOP              thr_input_callback, thruster_input_stale, move
#   3. STATE / TELEMETRY      odom_callback, gps_callback,
#                             monitoring_data_callback
#   4. CSV LOGGING            actuation_state, write_origin_sidecar,
#                             build_log_row, to_gps, log_timer_callback
#   5. SHUTDOWN               shutdown
# ----------------------------------------------------------------------------

# rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import rclpy

# Common python libraries
import os
import re
import time
from datetime import datetime
import numpy as np

# ROS2 msg libraries
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy

# Custom libraries
from blueboat_control import ROV
import custom_functions as cf
import robot_log_schema as rls   # CSV column layout (ROS-free, _custom_libraries/)
import thrust_limits as tl       # uniform thrust saturation (ROS-free)

class Controller(Node):

    # ======================================================================
    #  1. WIRING
    # ======================================================================

    def __init__(self):

        super().__init__('pid_sim', namespace='blueboat')

        self.declare_parameter('controller_type', 'MPC') 
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value

        # Loss-of-reference watchdog, same name, semantics and default as
        # robot_interface: simulation and real water must not differ on a safety
        # property. master_control publishes /thruster_input at 20 Hz, so 0.5 s is
        # ten missed producer ticks (five ticks of this node's own 10 Hz loop).
        self.declare_parameter('thruster_input_timeout', 0.5)
        self.thruster_input_timeout = self.get_parameter('thruster_input_timeout').get_parameter_value().double_value

        # Same name, semantics and default as robot_interface and master_control.
        # Simulation used to apply NO thrust limit at all -- ROV.move puts
        # whatever arrives straight onto the Gazebo thrusters -- so a command the
        # real boat would have saturated ran unbounded here and the two diverged
        # exactly where N2/CM-2 says they must not.
        self.declare_parameter('thrust_limit', 20.0)
        self.thrust_limit = self.get_parameter('thrust_limit').get_parameter_value().double_value

        # Same two names robot_interface uses, so a simulated run is filed the
        # same way a field run is. Empty data_dir means "resolve it" -- see
        # custom_functions.data_root for the order.
        self.declare_parameter('note', 'sim')
        self.note = self.get_parameter('note').get_parameter_value().string_value

        self.declare_parameter('data_dir', '')
        self.data_root = cf.data_root(
            self.get_parameter('data_dir').get_parameter_value().string_value)

        # There is no Water Linked UGPS in simulation, so the pinger layout can
        # never be filled here. Fixed rather than declared: a parameter that
        # selects a layout this node cannot populate would only produce a CSV of
        # zeros under pinger column names.
        self.use_UWgps = False

        self.rov = ROV(self, thrust_visual = True)

        self.odom_sim_subscriber = self.create_subscription(Odometry, '/blueboat/odom', self.odom_callback, 10)
        self.thruster_input_sub = self.create_subscription(Float32MultiArray, "/thruster_input", self.thr_input_callback,10)

        # Logging inputs. /monitoring_data carries the controller's world-frame
        # target in EVERY branch (CM-8 / N9), which is where target_x/y comes
        # from. The NavSatFix is BEST_EFFORT to match every other consumer of it
        # in the project (CM-4); nothing publishes it under Sim_launch, but the
        # Mission Control Station's bridge and the simulator's mavros_shim_node
        # both do in a GPS-anchored run, and the GPS columns fill in when they do.
        self.monitoring_sub = self.create_subscription(
            Float32MultiArray, "/monitoring_data", self.monitoring_data_callback, 10)
        gps_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, gps_qos)

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_publisher = self.create_publisher(Bool, '/blueboat/controller_ready', latched)
        self.data_publisher = self.create_publisher(Float32MultiArray, "/monitoring_data", 10)

        self.timer = self.create_timer(0.1, self.move)
        # Same rate as robot_interface's log timer -- three rows a second.
        self.log_timer = self.create_timer(0.33, self.log_timer_callback)

        self.thr_input = [0,0]

        # None until the first /thruster_input arrives, which is itself a
        # "no reference" condition.
        self.last_thr_rx = None
        self.thr_watchdog_tripped = False

        self.sent_ready = False

        ################## Logged state ##################
        # Filled by the callbacks in section 3; every one of them has a value
        # that is safe to log before the first message arrives, so the log timer
        # never has to guess whether the graph has come up yet.
        self.relative_coordinates = [0.0, 0.0, 0.0]   # x, y, yaw (local ENU)
        self.roll = 0.0
        self.pitch = 0.0
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)
        self.monitoring_data = []
        self.gps_data = [0.0, 0.0]

        # Frame origin. In simulation /blueboat/odom is already local ENU about
        # the Gazebo world origin, so there is nothing to re-zero -- yaw0 is
        # latched for the sidecar's provenance field only. geo_anchor is
        # (lat, lon, x, y): the first VALID fix and the world position the boat
        # held when it arrived, which is what lets the world frame be converted
        # to WGS84 even though the fix may show up long after odom does.
        self.yaw0 = None
        self.geo_anchor = None
        # No row is written before the first /blueboat/odom. Every logged value
        # has a safe default, so without this the log would open with a run of
        # all-zero rows recorded while Gazebo was still coming up -- which reads
        # downstream as a boat sitting at the origin, not as no data.
        self.odom_seen = False

        # Previous body-frame velocity sample, for the differentiated
        # acceleration in odom_callback.
        self.prev_twist = None
        self.prev_twist_t = None

        ################## Initialize data collection ##################
        # Column layout lives in robot_log_schema.py (ROS-free, in
        # _custom_libraries/). It is a WRITE-ONCE field-data contract: read the
        # module docstring before touching a name or an order. Rows are filled
        # by column NAME below, never by index.
        self.data_columns = rls.columns_for(self.use_UWgps)
        self.target_cols, self.target_gps_cols = rls.target_columns_for(self.use_UWgps)

        # The FILE NAME is stamped with the wall clock so simulated and field
        # runs sort together in one directory; the ROWS are stamped with the ROS
        # clock, which is sim time here -- see build_log_row.
        self.date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        log_dir = cf.ensure_data_dir(self, self.data_root, 'data', 'Robot_data')
        stem = f'{self.date}-{self.note}-poslog' if self.note else f'{self.date}-poslog'
        self.path = cf.reserve_run_file(log_dir, stem, '.csv') + '.csv'
        # Sidecar name derived by pattern, not by a fixed-length slice -- the
        # same rule the reader in poslog_report uses.
        self.origin_path = re.sub(r'-poslog(-\d+)?\.csv$', r'-origin\1.yaml', self.path)

        # Legend (description row, unit row) and header once, then one
        # appended-and-flushed row per tick. Same three lines as robot_interface.
        self.log_file = open(self.path, 'w', buffering=1)
        for line in rls.legend_rows(self.data_columns):
            self.log_file.write(','.join(line) + '\n')
        self.log_file.write(','.join(self.data_columns) + '\n')
        self.log_file.flush()
        self.origin_written = False
        self.get_logger().info(f"Position log: {self.path}")

    # ======================================================================
    #  2. MAIN LOOP
    # ======================================================================

    def thr_input_callback(self, msg: Float32MultiArray):
        self.thr_input = msg.data
        self.last_thr_rx = time.time()

    def thruster_input_stale(self, now):
        """
        Loss-of-reference watchdog predicate - the mirror of
        robot_interface.thruster_input_stale. True when /thruster_input has gone
        quiet for longer than thruster_input_timeout, so the last received thrust
        is not re-applied to the Gazebo thrusters indefinitely.
        """
        if self.last_thr_rx is None:
            return True
        return (now - self.last_thr_rx) > self.thruster_input_timeout

    def move(self):
        if not self.rov.ready():
            return

        if not self.sent_ready:
            msg = Bool()
            msg.data = True
            self.ready_publisher.publish(msg)
            self.get_logger().info(f'Ready publishing: {msg.data}')
            self.sent_ready = True

        if self.thruster_input_stale(time.time()):
            if not self.thr_watchdog_tripped:
                self.thr_watchdog_tripped = True
                self.get_logger().warn(
                    f"No /thruster_input for {self.thruster_input_timeout:.2f} s "
                    "- zeroing thrust.")
            self.thr_input = [0,0]
        elif self.thr_watchdog_tripped:
            self.thr_watchdog_tripped = False
            self.get_logger().info("/thruster_input resumed - releasing watchdog.")

        # Saturate the same way the real boat does: one scale factor for both
        # thrusters, so the right:left ratio and the direction of the commanded
        # wrench survive the clamp (thrust_limits.py explains why per-side
        # clipping does not). master_control already limits at its publisher, so
        # this normally does nothing -- it is here so that a command reaching
        # Gazebo from anywhere else is bounded like the hardware is.
        limited, scale = tl.scale_to_limit(self.thr_input, self.thrust_limit)
        if scale < 1.0:
            self.get_logger().warn(
                f"Thrust {list(self.thr_input)} exceeds thrust_limit="
                f"{self.thrust_limit:.1f} N - scaled by {scale:.3f}.",
                throttle_duration_sec=2.0)

        r, l = float(limited[0]), float(limited[1])
        # Apply force to thrusters
        self.rov.move([r,l])

    # ======================================================================
    #  3. STATE / TELEMETRY
    #  None of these command anything.
    # ======================================================================

    def odom_callback(self, msg: Odometry):
        pose, twist = cf.odometry(msg)

        self.rov.current_pose = pose
        self.rov.current_twist = twist

        # --- everything below is for the log only ---------------------------
        # /blueboat/odom is ALREADY local ENU here: the Gazebo bridge publishes
        # in the world frame with no re-zeroing, so unlike robot_interface there
        # is no origin to subtract. The boat spawns at (0, 0), so the two frames
        # agree in kind and in origin; only the yaw at spawn (Sim_launch's
        # spawn_yaw) varies, and that is recorded in the sidecar.
        roll, pitch, yaw = cf.quaternion_to_rpy(msg.pose.pose.orientation)
        self.relative_coordinates = [msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     yaw]
        self.roll = roll
        self.pitch = pitch

        av = msg.twist.twist.angular
        self.angular_velocity = np.array([av.x, av.y, av.z])

        # lin_acc_* is the ONE column that is not a like-for-like substitution
        # for the real boat's. There is no simulated IMU under Sim_launch, so
        # this is the body-frame linear velocity differentiated over the ROS
        # clock (sim time). Two consequences for an analyst: it carries NO
        # gravity component, where a real /mavros/imu/data acceleration does,
        # and it is as noisy as the numerical difference of a 20 Hz odometry.
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

        # Latch the geodetic anchor on the first VALID fix, together with the
        # world position held at that moment. Deferring it this way is what
        # makes the GPS columns usable in a Mission Control Station run, where
        # the synthesised fixes can start after odom does; with no fix at all
        # (a plain Sim_launch) the anchor stays None and the GPS columns stay
        # zero, which is exactly what "no fix" means everywhere else here.
        if self.geo_anchor is None and not (self.gps_data[0] == 0.0
                                            and self.gps_data[1] == 0.0):
            self.geo_anchor = (self.gps_data[0], self.gps_data[1],
                               msg.pose.pose.position.x,
                               msg.pose.pose.position.y)

    def gps_callback(self, msg: NavSatFix):
        # (0, 0) means no fix, the same rule every other consumer in the project
        # applies; it is stored as-is and discarded by the anchor test above and
        # by poslog_report's own _valid_gps.
        self.gps_data = [msg.latitude, msg.longitude]

    def monitoring_data_callback(self, msg: Float32MultiArray):
        """
        Cache /monitoring_data = [t, x, y, psi, x_d, y_d, psi_d, u1, u2].
        Only the target pair [4:6] is logged, and it is world-frame in every
        controller branch (CM-8 / N9) - no frame correction is applied here.
        """
        self.monitoring_data = msg.data

    # ======================================================================
    #  4. CSV LOGGING
    #  Same writer, same columns and same rate as robot_interface's, so a
    #  simulated run and a field run are read by one reader.
    # ======================================================================

    def actuation_state(self):
        """
        One integer saying whether the logged thrust command could reach the water.

        Output : int, see robot_log_schema for the encoding.

        Simulation has only two of the four states. There is no enable_motors
        gate and no autopilot mode here: ROV.move puts whatever this node
        commands straight onto the Gazebo thrusters, so a command always reaches
        the water (ACT_LIVE) unless the loss-of-reference watchdog has forced it
        to zero (ACT_WATCHDOG). States 0 and 2 are hardware conditions and
        cannot occur - a simulated log never shows them.
        """
        if self.thr_watchdog_tripped:
            return rls.ACT_WATCHDOG
        return rls.ACT_LIVE

    def write_origin_sidecar(self):
        """
        Record the world frame's own origin, once, beside the CSV.

        Written on the first row that has a geodetic anchor to record. Without
        a GPS publisher (a plain Sim_launch) there is none, so the sidecar is
        written once at the end of the run instead, carrying yaw0 and no fix -
        see shutdown. yaw0_rad is the boat's ENU heading at the first odom
        callback, i.e. Sim_launch's spawn_yaw; it is provenance only and is NOT
        part of the frame, which is absolute ENU.
        """
        if self.origin_written or self.yaw0 is None:
            return
        if self.geo_anchor is None:
            lat0 = lon0 = 0.0
        else:
            # The anchor fix was taken at (xa, ya), which is not necessarily the
            # frame origin; project back to (0, 0) so the sidecar means the same
            # thing it does on the real boat.
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
        """
        Assemble one CSV row of the no-pinger layout (27 columns).

        Output : dict {column name: value} covering every column of
                 self.data_columns. Filled BY COLUMN NAME, never by index.
        """
        # The ROS clock, which is SIM TIME under Sim_launch (use_sim_time=True),
        # not the wall clock robot_interface stamps with. That is deliberate:
        # every derived quantity a reader computes from these columns - speed,
        # mission duration - must be measured against the clock the physics ran
        # on, or a real-time factor other than 1 silently rescales all of them.
        # Sim time starts at 0, so the date columns read 1970-01-01; readers
        # take differences from the first row and never the absolute date.
        # Rendered in LOCAL time, like robot_interface's, so that when this node
        # runs on the wall clock instead (no /clock publisher) the two nodes'
        # stamps are directly comparable rather than a timezone apart.
        now = datetime.fromtimestamp(self.get_clock().now().nanoseconds * 1e-9)

        row = {
            'Year': now.year, 'Month': now.month, 'Day': now.day,
            'Hour': now.hour, 'Minute': now.minute, 'Second': now.second,
            'MicroSecond': now.microsecond,

            # Robot pose, local-ENU world frame. relative_psi is ABSOLUTE ENU
            # yaw (0 = East, CCW+), as it is on the real boat since 2026-08-31.
            'relative_x': self.relative_coordinates[0],
            'relative_y': self.relative_coordinates[1],
            'relative_psi': self.relative_coordinates[2],

            'gps_latitude': self.gps_data[0],
            'gps_longitude': self.gps_data[1],

            # thr_input is [right, left] (master_control's convention), logged
            # as COMMANDED on /thruster_input - the watchdog writes [0, 0] into
            # it, which is what actuation_state above disambiguates.
            'right_thr_in': self.thr_input[0],
            'left_thr_in': self.thr_input[1],
            'actuation_state': self.actuation_state(),

            # Attitude. Yaw is not repeated -- it is relative_psi above.
            'roll': self.roll,
            'pitch': self.pitch,

            'ang_vel_x': self.angular_velocity[0],
            'ang_vel_y': self.angular_velocity[1],
            'ang_vel_z': self.angular_velocity[2],

            # Differentiated from the odometry twist, NOT a simulated IMU:
            # no gravity component. See odom_callback.
            'lin_acc_x': self.linear_acceleration[0],
            'lin_acc_y': self.linear_acceleration[1],
            'lin_acc_z': self.linear_acceleration[2],
        }

        # /monitoring_data = [t, x, y, psi, x_d, y_d, psi_d, u1, u2]. x_d/y_d are
        # world-frame in EVERY controller branch (CM-8 / N9). An empty buffer
        # (no controller running yet) logs zeros rather than raising.
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
        """
        Convert a local-ENU world-frame point into WGS84 degrees.

        Input  : xy -- [x, y] in the same frame as relative_x/y.
        Output : [latitude, longitude], or [0.0, 0.0] before a valid fix has
                 anchored the frame -- (0, 0) is this project's "no fix".
        """
        if self.geo_anchor is None:
            return [0.0, 0.0]
        lat0, lon0, xa, ya = self.geo_anchor
        lat, lon = cf.enu_to_gps(lat0, lon0, xy[0] - xa, xy[1] - ya)
        return [lat, lon]

    def log_timer_callback(self):
        """
        Append one row to the position CSV. The single writer.
        """
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

    # ======================================================================
    #  5. SHUTDOWN
    #  Ordered teardown, mirroring robot_interface's: stop the thrusters, close
    #  the CSV, then file the run. Every step is independently guarded so a
    #  failure in one cannot skip the ones after it.
    # ======================================================================

    def shutdown(self):
        # 1. Thrusters first, always. rov.parsed() is False until the URDF has
        #    arrived on robot_description, and ROV.move indexes its (empty)
        #    publisher list unconditionally -- so without the test a teardown
        #    before Gazebo ever came up reports an IndexError as a failure to
        #    stop thrusters that do not exist yet.
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

        # 4. Mission report. Imported HERE, not at module scope: matplotlib is
        #    not declared in package.xml and must never be able to stop this
        #    node from starting.
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
        # A launch teardown SIGINTs the whole process group; rclpy turns that
        # into ExternalShutdownException rather than KeyboardInterrupt. Both
        # mean "the run is over", and the shutdown below must still run.
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


main()
