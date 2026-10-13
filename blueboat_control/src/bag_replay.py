#!/usr/bin/env python3

# ============================================================================
# Kinematic replay of a recorded real-robot run in Gazebo (v1.1).
#
# Input: one run's ros2bag (robot_interface + run_bag.py write it into the
# run's Robot_data/<stem>/ folder), or that run folder itself. The simulated
# boat is DRIVEN BY THE RECORDING, not by physics: every 20 ms its pose is set
# from the recorded attitude and position through Gazebo's UserCommands
# (/world/<w>/set_pose, bridged by Replay_launch.py). Replay_launch spawns the
# boat with blueboat_description's `replay:=true` description -- no
# hydrodynamics, no collision geometry, no gravity -- so no force acts on it:
# waves, wind and current cannot move it, and a set_pose is a clean teleport.
#
# What the graph sees, so every consumer runs as it would live:
#   /blueboat/odom      rebuilt from /mavros/local_position/odom, which has the
#                       FULL attitude (the recorded /blueboat/odom is yaw-only),
#                       placed in the world frame, recorded body twist.
#   /monitoring_data    x, y, x_d, y_d shifted into the world frame.
#   /set_path           the run's `-path.yaml` snapshot, world frame, every 5 s.
#   original names      every /mavros/* topic but rc/override (recorded GPS
#                       fixes, compass, IMU, state, ...), /thruster_input,
#                       /controller_target, /blueboat/pinger_coordinates,
#                       /uw_gps_data -- unchanged but for header.stamp.
#   /replay/<name>      everything else (operator commands, session plumbing,
#                       param handshake, rc/override, the raw /blueboat/odom):
#                       nothing lost, nothing that triggers a live consumer.
#   dropped             /rosout, /parameter_events, /tf, /tf_static, /clock.
#
# Every header.stamp is rewritten to this node's (sim) time: the bag's RECEIVE
# timestamps are the timing truth. On the real launch simple_launch gives
# mavros use_sim_time:=True with no /clock, so recorded stamps can read 0.
#
# Frames. The run's /blueboat/odom frame is local ENU about the boat's launch
# point (robot_interface). Empty world: world = that frame. GPS-anchored built
# world: world = ENU about the world's anchor; the offset is the median over
# the bag's GPS fixes of (fix in anchor EN) - (boat in launch frame).
#
# Controls: /replay/command (String JSON) {"cmd": "pause"|"resume"|"rate",
# "rate": x}; /replay/state (String JSON, latched, 2 Hz).
#
# FILE MAP
#   1. BAG ACCESS        resolve_bag, open_reader, read_track
#   2. TRACK MATHS       Track (frames, heave, interpolation)
#   3. BAG CLOCK         BagClock (rate, pause)
#   4. NODE              BagReplay: wiring, scheduler thread, pose timer,
#                        routing, controls, state
# ============================================================================

import glob
import json
import math
import os
import threading
import time

import numpy as np
import yaml

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

import frame_math as fm

T_MAVROS_ODOM = '/mavros/local_position/odom'
T_MAVROS_IMU = '/mavros/imu/data'
T_GPS = '/mavros/global_position/global'
T_ODOM = '/blueboat/odom'
T_MONITORING = '/monitoring_data'

# Republished unchanged under their own names (beside every /mavros/* topic).
LIVE_TOPICS = {'/thruster_input', '/controller_target',
               '/blueboat/pinger_coordinates', '/uw_gps_data'}
# Never republished: graph plumbing of the recording session.
DROPPED_TOPICS = {'/rosout', '/parameter_events', '/tf', '/tf_static', '/clock'}
# /mavros/* topics that are commands TO the boat, not telemetry from it.
MAVROS_COMMANDS = {'/mavros/rc/override'}
REPLAY_PREFIX = '/replay'

HEAVE_WINDOW_S = 10.0
HEAVE_CLIP_M = 0.5


# ============================================================================
#  1. BAG ACCESS
# ============================================================================

def resolve_bag(path):
    """A bag directory, from the bag itself or the run folder holding it."""
    path = os.path.abspath(os.path.expanduser(str(path)))
    if os.path.isfile(os.path.join(path, 'metadata.yaml')):
        return path
    candidates = sorted(d for d in glob.glob(os.path.join(path, '*'))
                        if os.path.isfile(os.path.join(d, 'metadata.yaml')))
    preferred = [d for d in candidates if os.path.basename(d).rstrip('-0123456789').endswith('-bag')]
    if preferred or candidates:
        return (preferred or candidates)[0]
    raise FileNotFoundError(f'{path}: no ros2bag (no metadata.yaml) here or one level below')


def find_path_snapshot(bag_dir):
    """The run's `-path.yaml` trajectory snapshot beside its bag, or None."""
    found = sorted(glob.glob(os.path.join(os.path.dirname(bag_dir), '*-path*.yaml')))
    return found[0] if found else None


def open_reader(bag_dir, topics=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader


def bag_span(bag_dir):
    """(first receive time ns, duration ns), read from the messages themselves."""
    reader = open_reader(bag_dir)
    first = last = None
    if reader.has_next():
        first = reader.read_next()[2]
    info = rosbag2_py.Info().read_metadata(bag_dir, '')
    end = info.starting_time.nanoseconds + info.duration.nanoseconds \
        if hasattr(info.duration, 'nanoseconds') else None
    if first is None:
        raise ValueError(f'{bag_dir}: the bag holds no message')
    if end is None:
        while reader.has_next():
            last = reader.read_next()[2]
        end = last if last is not None else first
    return int(first), max(0, int(end) - int(first))


def topic_types(bag_dir):
    reader = open_reader(bag_dir)
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def read_track(bag_dir):
    """Pass 1: the samples the pose track and the frame offsets are built from.

    Times are the bag's RECEIVE times, in ns."""
    types = topic_types(bag_dir)
    wanted = [t for t in (T_MAVROS_ODOM, T_MAVROS_IMU, T_GPS, T_ODOM) if t in types]
    out = {'odom_t': [], 'odom_p': [], 'odom_q': [], 'odom_twist': [],
           'imu_t': [], 'imu_q': [], 'bb_t': [], 'bb_p': [], 'bb_yaw': [],
           'gps_t': [], 'gps_ll': []}
    if not wanted:
        return out, types
    reader = open_reader(bag_dir, wanted)
    cls = {t: get_message(types[t]) for t in wanted}
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, cls[topic])
        if topic == T_MAVROS_ODOM:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            out['odom_t'].append(t)
            out['odom_p'].append((p.x, p.y, p.z))
            out['odom_q'].append((q.x, q.y, q.z, q.w))
            out['odom_twist'].append(msg.twist.twist)
        elif topic == T_MAVROS_IMU:
            q = msg.orientation
            if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
                continue
            out['imu_t'].append(t)
            out['imu_q'].append((q.x, q.y, q.z, q.w))
        elif topic == T_ODOM:
            p = msg.pose.pose.position
            out['bb_t'].append(t)
            out['bb_p'].append((p.x, p.y, p.z))
            out['bb_yaw'].append(_yaw(msg.pose.pose.orientation))
        elif topic == T_GPS:
            if msg.status.status < 0 or (msg.latitude == 0.0 and msg.longitude == 0.0):
                continue
            out['gps_t'].append(t)
            out['gps_ll'].append((msg.latitude, msg.longitude))
    return out, types


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


# ============================================================================
#  2. TRACK MATHS
# ============================================================================

def slerp(q0, q1, a):
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + a * (q1 - q0)
    else:
        th = math.acos(d)
        q = (math.sin((1 - a) * th) * q0 + math.sin(a * th) * q1) / math.sin(th)
    return q / np.linalg.norm(q)


def detrended(t_s, z, window_s=HEAVE_WINDOW_S, clip=HEAVE_CLIP_M):
    """z minus its centred moving mean over window_s, clipped: the EKF's
    altitude drifts by metres (GPS / baro), its short-term motion is heave."""
    z = np.asarray(z, float)
    if len(z) == 0:
        return z
    c = np.concatenate([[0.0], np.cumsum(z)])
    lo = np.searchsorted(t_s, t_s - window_s / 2.0, side='left')
    hi = np.searchsorted(t_s, t_s + window_s / 2.0, side='right')
    mean = (c[hi] - c[lo]) / np.maximum(hi - lo, 1)
    return np.clip(z - mean, -clip, clip)


class Track:
    """The recorded pose, in the replay world frame, as a function of bag time."""

    def __init__(self, samples, anchor=None, heave=True, waterline_z=-0.15):
        s = samples
        self.source = None
        self.world_offset = np.zeros(2)
        self.launch_shift = np.zeros(3)
        if s['odom_t']:
            self.source = 'mavros'
            self.t = np.asarray(s['odom_t'], np.int64)
            p = np.asarray(s['odom_p'], float)
            q = np.asarray(s['odom_q'], float)
            # mavros local frame -> the run's /blueboat/odom launch frame:
            # robot_interface subtracts the first sample's position, so the
            # shift is constant -- measured from paired samples, which also
            # covers a bag that starts mid-session (the latch predates it).
            if s['bb_t']:
                bb_t = np.asarray(s['bb_t'], np.int64)
                bb_p = np.asarray(s['bb_p'], float)
                idx = np.clip(np.searchsorted(self.t, bb_t), 0, len(self.t) - 1)
                self.launch_shift[:2] = np.median(p[idx, :2] - bb_p[:, :2], axis=0)
                self.launch_shift[2] = float(np.median(p[idx, 2] - bb_p[:, 2]))
            else:
                self.launch_shift = p[0].copy()
            self.xy = p[:, :2] - self.launch_shift[:2]
            # Attitude: the denser of IMU and odom orientation (same ENU/FLU).
            if len(s['imu_t']) > len(s['odom_t']):
                self.q_t = np.asarray(s['imu_t'], np.int64)
                self.q = np.asarray(s['imu_q'], float)
            else:
                self.q_t, self.q = self.t, q
            self.z = (detrended(self.t * 1e-9, p[:, 2]) if heave
                      else np.zeros(len(self.t)))
        elif s['bb_t']:
            self.source = 'blueboat_odom'          # yaw-only fallback
            self.t = np.asarray(s['bb_t'], np.int64)
            self.xy = np.asarray(s['bb_p'], float)[:, :2]
            self.q_t = self.t
            self.q = np.asarray([_yaw_quat(y) for y in s['bb_yaw']], float)
            self.z = np.zeros(len(self.t))
        else:
            raise ValueError('the bag has neither /mavros/local_position/odom '
                             'nor /blueboat/odom: nothing to replay a pose from')
        self.waterline_z = waterline_z

        if anchor is not None and s['gps_t']:
            g_t = np.asarray(s['gps_t'], np.int64)
            ll = np.asarray(s['gps_ll'], float)
            e, n = fm.latlon_to_local_en(ll[:, 0], ll[:, 1], anchor[0], anchor[1])
            bx = np.interp(g_t, self.t, self.xy[:, 0])
            by = np.interp(g_t, self.t, self.xy[:, 1])
            self.world_offset = np.array([np.median(e - bx), np.median(n - by)])
        elif anchor is not None:
            raise ValueError('a GPS-anchored world needs GPS fixes in the bag')

    @property
    def t0(self):
        return int(self.t[0])

    def to_world(self, x, y):
        return x + self.world_offset[0], y + self.world_offset[1]

    def mavros_to_world(self, x, y):
        """A raw mavros local position -> world frame."""
        return self.to_world(x - self.launch_shift[0], y - self.launch_shift[1])

    def pose_at(self, t_ns):
        """(x, y, z, (qx, qy, qz, qw)) in the world frame at bag time t_ns."""
        x = float(np.interp(t_ns, self.t, self.xy[:, 0]))
        y = float(np.interp(t_ns, self.t, self.xy[:, 1]))
        z = self.waterline_z + float(np.interp(t_ns, self.t, self.z))
        i = int(np.searchsorted(self.q_t, t_ns))
        if i <= 0:
            q = self.q[0]
        elif i >= len(self.q_t):
            q = self.q[-1]
        else:
            a = (t_ns - self.q_t[i - 1]) / max(1, self.q_t[i] - self.q_t[i - 1])
            q = slerp(self.q[i - 1], self.q[i], a)
        x, y = self.to_world(x, y)
        return x, y, z, tuple(float(v) for v in q)


# ============================================================================
#  3. BAG CLOCK
# ============================================================================

class BagClock:
    """Bag time (ns since the bag's first message) driven by the wall clock,
    scaled by a rate, pausable. Thread-safe."""

    def __init__(self, rate=1.0):
        self._lock = threading.Lock()
        self._rate = max(0.01, float(rate))
        self._base_bag = 0.0
        self._base_wall = time.monotonic()
        self._paused = False

    def now_ns(self):
        with self._lock:
            if self._paused:
                return int(self._base_bag)
            return int(self._base_bag + (time.monotonic() - self._base_wall) * 1e9 * self._rate)

    def _rebase(self):
        self._base_bag = self._base_bag if self._paused else (
            self._base_bag + (time.monotonic() - self._base_wall) * 1e9 * self._rate)
        self._base_wall = time.monotonic()

    def set_rate(self, rate):
        with self._lock:
            self._rebase()
            self._rate = max(0.01, float(rate))

    def set_paused(self, paused):
        with self._lock:
            self._rebase()
            self._paused = bool(paused)

    @property
    def rate(self):
        return self._rate

    @property
    def paused(self):
        return self._paused


# ============================================================================
#  4. NODE
# ============================================================================

class BagReplay(Node):

    def __init__(self):
        super().__init__('bag_replay')
        self.declare_parameter('bag', '')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('world_name', 'ocean')
        self.declare_parameter('model_name', 'blueboat')
        # NaN = no anchor (empty world, which IS the run's launch frame).
        self.declare_parameter('anchor_lat', float('nan'))
        self.declare_parameter('anchor_lon', float('nan'))
        self.declare_parameter('heave', True)
        self.declare_parameter('waterline_z', -0.15)
        self.declare_parameter('pose_rate_hz', 50.0)
        self.declare_parameter('start_paused', False)

        gp = self.get_parameter
        self.bag_dir = resolve_bag(gp('bag').get_parameter_value().string_value)
        self.world_name = gp('world_name').get_parameter_value().string_value
        self.model_name = gp('model_name').get_parameter_value().string_value
        alat = gp('anchor_lat').get_parameter_value().double_value
        alon = gp('anchor_lon').get_parameter_value().double_value
        anchor = None if (math.isnan(alat) or math.isnan(alon)) else (alat, alon)

        self.bag_start_ns, self.duration_ns = bag_span(self.bag_dir)

        samples, self.types = read_track(self.bag_dir)
        self.track = Track(samples, anchor=anchor,
                           heave=gp('heave').get_parameter_value().bool_value,
                           waterline_z=gp('waterline_z').get_parameter_value().double_value)
        self.frame = 'anchored' if anchor else 'empty'
        self.get_logger().info(
            f"Replaying {self.bag_dir}: {self.duration_ns * 1e-9:.1f} s, pose from "
            f"{self.track.source}, {self.frame} world '{self.world_name}', world offset "
            f"({self.track.world_offset[0]:.2f}, {self.track.world_offset[1]:.2f}) m.")

        # Routing table and lazy publishers.
        self.reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pubs = {}
        self.classes = {}
        self.has_header = {}
        self.odom_pub = self.create_publisher(Odometry, '/blueboat/odom', 10)
        self.monitoring_pub = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/replay/state', latched)
        self.path_pub = self.create_publisher(Path, '/set_path', 10)
        self.create_subscription(String, '/replay/command', self.command_callback, 10)

        self.pose_client = self.create_client(
            SetEntityPose, f'/world/{self.world_name}/set_pose')
        self.pose_future = None
        self.pose_calls = 0

        # Wall-clock timers: the replay's pace is the bag clock, never /clock.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        period = 1.0 / max(1.0, gp('pose_rate_hz').get_parameter_value().double_value)
        self.create_timer(period, self.pose_timer_callback, clock=steady)
        self.create_timer(0.5, self.publish_state, clock=steady)
        self.create_timer(5.0, self.publish_path, clock=steady)
        self.create_timer(0.05, self.hold_timer_callback, clock=steady)

        self.path_msg = self._load_path()
        self.last_odom = None
        self.last_gps = None
        self.last_gps_tx = 0.0
        self.finished = False
        self.stop_event = threading.Event()
        self.clock = BagClock(gp('rate').get_parameter_value().double_value)
        if gp('start_paused').get_parameter_value().bool_value:
            self.clock.set_paused(True)
        self.thread = threading.Thread(target=self.scheduler, name='bag_replay', daemon=True)
        self.thread.start()
        self.publish_state()

    # -- routing ---------------------------------------------------------

    def route(self, topic):
        """(output topic or None, mode) for a recorded topic."""
        if topic in DROPPED_TOPICS:
            return None, 'drop'
        if topic == T_ODOM:
            return REPLAY_PREFIX + topic, 'raw'
        if topic == T_MONITORING:
            return topic, 'monitoring'
        if topic.startswith('/mavros/') and topic not in MAVROS_COMMANDS:
            return topic, 'live'
        if topic in LIVE_TOPICS:
            return topic, 'live'
        return REPLAY_PREFIX + topic, 'raw'

    def publisher_for(self, out, type_name):
        pub = self.pubs.get(out)
        if pub is None:
            cls = self.class_for(type_name)
            if cls is None:
                return None
            pub = self.create_publisher(cls, out, self.reliable)
            self.pubs[out] = pub
        return pub

    def class_for(self, type_name):
        if type_name not in self.classes:
            try:
                cls = get_message(type_name)
                self.classes[type_name] = cls
                self.has_header[type_name] = 'header' in cls.get_fields_and_field_types()
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                self.get_logger().warning(f"{type_name}: not available ({exc}); "
                                          "its topics are skipped.")
                self.classes[type_name] = None
        return self.classes[type_name]

    # -- scheduler (pass 2) ------------------------------------------------

    def scheduler(self):
        try:
            reader = open_reader(self.bag_dir)
            while reader.has_next() and not self.stop_event.is_set():
                topic, data, t = reader.read_next()
                due = t - self.bag_start_ns
                while not self.stop_event.is_set():
                    ahead = due - self.clock.now_ns()
                    if ahead <= 0:
                        break
                    time.sleep(min(ahead * 1e-9 / max(self.clock.rate, 0.01), 0.02)
                               if not self.clock.paused else 0.05)
                self.dispatch(topic, data, t)
        except Exception as exc:                      # noqa: BLE001 - reported
            if self.stop_event.is_set():
                return                                 # node teardown
            self.get_logger().error(f"Replay stopped: {exc}")
        if self.stop_event.is_set():
            return
        self.finished = True
        self.get_logger().info("Replay finished: holding the last pose.")
        self.publish_state()

    def dispatch(self, topic, data, t):
        type_name = self.types.get(topic)
        out, mode = self.route(topic)
        if out is None or type_name is None:
            return
        if topic == T_MAVROS_ODOM:
            self.publish_odom(data, type_name)
        pub = self.publisher_for(out, type_name)
        if pub is None:
            return
        cls = self.classes[type_name]
        if mode in ('raw', 'live') and not self.has_header[type_name]:
            pub.publish(data)                 # serialized bytes, no rewrite needed
            return
        msg = deserialize_message(data, cls)
        if self.has_header[type_name]:
            msg.header.stamp = self.get_clock().now().to_msg()
        if mode == 'monitoring' and len(msg.data) >= 6:
            d = list(msg.data)
            d[1], d[2] = self.track.to_world(d[1], d[2])
            d[4], d[5] = self.track.to_world(d[4], d[5])
            msg.data = [float(v) for v in d]
        pub.publish(msg)
        if topic == T_GPS:
            self.last_gps = msg

    def publish_odom(self, data, type_name):
        """The replayed /blueboat/odom: full attitude, world frame, body twist."""
        src = deserialize_message(data, self.class_for(type_name))
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = f'{self.model_name}/base_link'
        x, y = self.track.mavros_to_world(src.pose.pose.position.x, src.pose.pose.position.y)
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.position.z = float(self.track.pose_at(self.clock.now_ns() + self.bag_start_ns)[2])
        msg.pose.pose.orientation = src.pose.pose.orientation
        msg.pose.covariance = src.pose.covariance
        msg.twist = src.twist
        self.odom_pub.publish(msg)
        self.last_odom = msg

    # -- Gazebo pose -------------------------------------------------------

    def pose_timer_callback(self):
        if self.pose_future is not None and not self.pose_future.done():
            return                               # latest wins: never queue
        if not self.pose_client.service_is_ready():
            return
        t = min(self.clock.now_ns(), int(self.duration_ns)) + self.bag_start_ns
        x, y, z, q = self.track.pose_at(t)
        req = SetEntityPose.Request()
        req.entity = Entity(name=self.model_name, type=Entity.MODEL)
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
        (req.pose.orientation.x, req.pose.orientation.y,
         req.pose.orientation.z, req.pose.orientation.w) = q
        self.pose_future = self.pose_client.call_async(req)
        self.pose_calls += 1

    def hold_timer_callback(self):
        """While paused or finished, keep odom and GPS alive (fresh stamps,
        zero twist) so displays and staleness checks don't drop the boat."""
        if not (self.clock.paused or self.finished) or self.last_odom is None:
            return
        msg = self.last_odom
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.twist.linear.x = msg.twist.twist.linear.y = msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = msg.twist.twist.angular.y = msg.twist.twist.angular.z = 0.0
        self.odom_pub.publish(msg)
        now = time.monotonic()
        if self.last_gps is not None and now - self.last_gps_tx >= 0.2:
            self.last_gps_tx = now
            self.last_gps.header.stamp = msg.header.stamp
            pub = self.pubs.get(T_GPS)
            if pub is not None:
                pub.publish(self.last_gps)

    # -- path --------------------------------------------------------------

    def _load_path(self):
        snap = find_path_snapshot(self.bag_dir)
        if snap is None:
            return None
        try:
            doc = yaml.safe_load(open(snap, encoding='utf-8')) or {}
            pts = doc.get('points') or []
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"Path snapshot unreadable ({snap}: {exc}).")
            return None
        if not pts:
            return None
        step = max(1, len(pts) // 2000)
        path = Path()
        path.header.frame_id = 'world'
        for row in pts[::step]:
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            x, y = self.track.to_world(float(row[1]), float(row[2]))
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            qx, qy, qz, qw = _yaw_quat(float(row[3]) if len(row) > 3 else 0.0)
            pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
            path.poses.append(pose)
        self.get_logger().info(f"Path snapshot: {snap} ({len(path.poses)} poses).")
        return path

    def publish_path(self):
        if self.path_msg is not None:
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path_msg)

    # -- controls and state ------------------------------------------------

    def command_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            name = cmd.get('cmd')
        except (ValueError, AttributeError):
            self.get_logger().warning(f"/replay/command: unreadable {msg.data!r}")
            return
        if name == 'pause':
            self.clock.set_paused(True)
        elif name == 'resume':
            self.clock.set_paused(False)
        elif name == 'rate':
            try:
                rate = float(cmd.get('rate'))
            except (TypeError, ValueError):
                self.get_logger().warning(f"/replay/command: bad rate {cmd.get('rate')!r}")
                return
            if not (0.01 <= rate <= 50.0):
                self.get_logger().warning(f"/replay/command: rate {rate} out of [0.01, 50]")
                return
            self.clock.set_rate(rate)
        else:
            self.get_logger().warning(f"/replay/command: unknown cmd {name!r}")
            return
        self.get_logger().info(
            f"Replay {'paused' if self.clock.paused else 'running'} at {self.clock.rate:g}x.")
        self.publish_state()

    def publish_state(self):
        t = min(self.clock.now_ns(), self.duration_ns) * 1e-9
        state = {
            'bag': self.bag_dir,
            't': round(t, 2),
            'duration': round(self.duration_ns * 1e-9, 2),
            'rate': self.clock.rate,
            'paused': self.clock.paused,
            'finished': self.finished,
            'frame': self.frame,
            'world': self.world_name,
            'world_offset': [round(float(v), 3) for v in self.track.world_offset],
            'pose_source': self.track.source,
            'gazebo': self.pose_client.service_is_ready(),
        }
        msg = String()
        msg.data = json.dumps(state)
        self.state_pub.publish(msg)

    def destroy_node(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        return super().destroy_node()


def main():
    rclpy.init()
    node = BagReplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A group SIGINT (the station's killpg) arrives twice: directly and
        # forwarded by ros2 launch. The second must not abort the teardown.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
