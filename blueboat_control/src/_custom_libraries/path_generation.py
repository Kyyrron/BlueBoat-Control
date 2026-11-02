#!/usr/bin/env python3

# ============================================================================
# The '/path_request' server: an array of path-parameter values in, a
# nav_msgs/Path out (one pose per value, frame 'world', each pose stamped with
# its parameter -- path_stamp). Deliberately parameter-agnostic.
#
# The hard-coded shapes are REFERENCE CONDITIONS for recorded field data
# (TRAJECTORY_SYSTEM.md carries their revision record). 'from_yaml:<path>'
# serves a station/designer file, WATCHED for (re)appearance so a GPS-anchored
# mission can be deployed mid-run; until it exists the node serves a
# station-keeping pose at the origin.
#
# FILE MAP
#   module scope    SHAPES, is_valid_shape, the 'fsin' cumulative table. The
#                   table MUST stay at module scope: _fsin_extend/_fsin_state
#                   rebind its globals with the `global` keyword.
#   class PathGeneration
#     1. WIRING               __init__, the one-server guard
#     2. SERVICE ENTRY POINT  generate_path
#     3. from_yaml RELOAD     _maybe_reload_yaml
#     4. THE SHAPE LIBRARY    single_pose
# ============================================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from blueboat_interfaces.srv import RequestPath
import math
import os
import time
import custom_functions as cf
import path_stamp as ps
import yaml_trajectory as yt

# Every `trajectory:=` selector single_pose serves; anything else is a FATAL
# at construction.
SHAPES = ('station_keeping', 'circle', 'straight_line', 'sin', 'fsin',
          'square', 'kin_square', 'seabed_scanning', 'from_yaml:<abs path>')


def _unknown_shape_message(path_shape) -> str:
    return (f"unknown trajectory '{path_shape}'. valid: "
            + ", ".join(SHAPES))


def is_valid_shape(path_shape: str) -> bool:
    """True for any selector single_pose can serve, including from_yaml."""
    return (path_shape.startswith('from_yaml')
            or path_shape in SHAPES)


# --- 'fsin' cumulative table -------------------------------------------------
# 'fsin' has no closed form (x, y integrate cos/sin of an integrated heading),
# so it is Euler-integrated once on a fixed 0.01 s grid and read by index. Each
# extension CONTINUES the accumulation from the stored last value
# (cumsum([last, *increments])), so the floats equal a single pass and
# single_pose stays pure in t whatever was asked before.
#
# The one knob is the turn radius: A = V/radius sets the yaw-rate amplitude and
# f = A/_FSIN_AF follows it, so the weave keeps its shape, scaled. A different
# radius discards the table.
_FSIN_V = 0.5           # surge [m/s] -- the authored speed
_FSIN_AF = 20.0         # yaw-rate amplitude / frequency [s] -- fixes the weave shape
_FSIN_DT = 0.01         # integration step [s]
_FSIN_MAX_STEPS = 10_000_000    # 100 000 s of path; beyond it, hold the last pose

_fsin_radius = None     # turn radius the table below was integrated for
_fsin_yaw = np.zeros(1)
_fsin_x = np.zeros(1)
_fsin_y = np.zeros(1)


def _fsin_extend(steps: int, radius: float) -> None:
    """Grow the table so index `steps` exists, continuing the same recursion."""
    global _fsin_yaw, _fsin_x, _fsin_y
    have = _fsin_yaw.size - 1
    if steps <= have:
        return
    target = min(max(steps, 2 * have, 1024), _FSIN_MAX_STEPS)
    amplitude = _FSIN_V / radius              # yaw-rate amplitude [rad/s]
    frequency = amplitude / _FSIN_AF          # yaw-rate frequency [Hz]
    i = np.arange(have, target)                  # the steps still to take
    tau = i * _FSIN_DT
    omega = amplitude * np.sin(2 * np.pi * frequency * tau)
    yaw = np.cumsum(np.concatenate(([_fsin_yaw[-1]], omega * _FSIN_DT)))
    x = np.cumsum(np.concatenate(([_fsin_x[-1]], _FSIN_V * np.cos(yaw[1:]) * _FSIN_DT)))
    y = np.cumsum(np.concatenate(([_fsin_y[-1]], _FSIN_V * np.sin(yaw[1:]) * _FSIN_DT)))
    _fsin_yaw = np.concatenate((_fsin_yaw, yaw[1:]))
    _fsin_x = np.concatenate((_fsin_x, x[1:]))
    _fsin_y = np.concatenate((_fsin_y, y[1:]))


def _fsin_state(t: float, radius: float):
    """(x, y, yaw) of the 'fsin' trajectory of turn radius `radius` at time t.

    Pure in t for a given radius. A different radius is a different shape, and
    discards the table rather than reading it at the wrong scale.
    """
    global _fsin_radius, _fsin_yaw, _fsin_x, _fsin_y
    if radius != _fsin_radius:
        _fsin_radius = radius
        _fsin_yaw = np.zeros(1)
        _fsin_x = np.zeros(1)
        _fsin_y = np.zeros(1)
    steps = int(t / _FSIN_DT)
    if steps <= 0:
        return 0.0, 0.0, 0.0
    steps = min(steps, _FSIN_MAX_STEPS)   # hold the last pose, as every shape does
    _fsin_extend(steps, radius)
    return _fsin_x[steps], _fsin_y[steps], _fsin_yaw[steps]
# -----------------------------------------------------------------------------


class PathGeneration(Node):

    # ======================================================================
    #  1. WIRING -- parameters and the /path_request service
    # ======================================================================

    def __init__(self):
        super().__init__('path_generation')

        self.declare_parameter('display_log', False)
        self.display_log = self.get_parameter('display_log').value

        self.declare_parameter('trajectory', 'station_keeping')
        self.trajectory = self.get_parameter('trajectory').get_parameter_value().string_value

        # Fail here, not inside the service handler (a callback exception
        # kills the node and the caller only sees silence).
        if not is_valid_shape(self.trajectory):
            msg = _unknown_shape_message(self.trajectory)
            self.get_logger().fatal(msg)
            raise ValueError(msg)

        # trajectory:=from_yaml:<abs path>, or trajectory:=from_yaml with the
        # optional yaml_path:=<path> (which wins; no launch file passes it).
        self.declare_parameter('yaml_path', '')
        yaml_path = self.get_parameter('yaml_path').get_parameter_value().string_value
        self.yaml_traj = None
        self._yaml_selected_path = ''
        self._yaml_mtime = None
        if self.trajectory.startswith('from_yaml'):
            self._yaml_selected_path = yaml_path or self.trajectory.partition(':')[2]
            self._maybe_reload_yaml()
            if self.yaml_traj is None:
                self.get_logger().info(
                    f"YAML trajectory '{self._yaml_selected_path}' not "
                    "available yet — holding position until it appears "
                    "(GPS-anchored missions are deployed by the station "
                    "once the georeference is established).")

        # Exactly one server: with two, every request is answered by whichever
        # replies first and the controller follows a mixture of trajectories.
        self.declare_parameter('allow_duplicate_server', False)
        self.declare_parameter('server_discovery_wait', 2.0)
        if not self.get_parameter('allow_duplicate_server').get_parameter_value().bool_value:
            self._refuse_if_server_running()

        self.path_service = self.create_service(RequestPath, '/path_request', self.generate_path)

    def _path_request_servers(self):
        """Nodes currently offering /path_request, as distinct 'ns/name'.

        No self-exclusion, deliberately: this runs BEFORE create_service, and
        the duplicate is another `path_generation` with this node's very name.
        """
        found = []
        for name, namespace in self.get_node_names_and_namespaces():
            try:
                services = self.get_service_names_and_types_by_node(name, namespace)
            except Exception:
                continue          # the node went away mid-enumeration
            if any(service == '/path_request' for service, _ in services):
                found.append(f"{namespace.rstrip('/')}/{name}")
        # Colliding names can report one node twice: distinct names only.
        return sorted(set(found))

    def _refuse_if_server_running(self):
        """Exit rather than become the second /path_request server."""
        # Give discovery a moment before believing an empty graph.
        deadline = time.monotonic() + max(
            0.0, self.get_parameter('server_discovery_wait').get_parameter_value().double_value)
        others = []
        while True:
            others = self._path_request_servers()
            if others or time.monotonic() >= deadline:
                break
            time.sleep(0.1)

        if others:
            msg = ("/path_request is already served by " + ", ".join(others) + ". "
                   "Exactly one path server may run: with two, the controller's "
                   "requests are answered by whichever replies first and it "
                   "follows a mixture of two trajectories. Shut the other "
                   "mission down first. Pass allow_duplicate_server:=true only "
                   "if you deliberately want two.")
            self.get_logger().fatal(msg)
            raise SystemExit(1)

    # ======================================================================
    #  2. SERVICE ENTRY POINT -- one pose per requested parameter value
    # ======================================================================

    def generate_path(self, request, response):
        self._maybe_reload_yaml()      # pick up a newly deployed file
        if self.display_log:
            self.get_logger().info(f"Received path_request of type: {type(request.path_request)}")

        path_msg = Path()
        path_msg.header.frame_id = 'world'

        for t in request.path_request.data:
            temp_pose = self.single_pose(t, self.trajectory)
            # Stamped with the PATH PARAMETER (not the clock), so the caller
            # can verify the answer is its own (master_control.accept_path).
            sec, nanosec = ps.encode(t)
            temp_pose.header.stamp.sec = sec
            temp_pose.header.stamp.nanosec = nanosec
            path_msg.poses.append(temp_pose)

        response.path = path_msg

        if self.display_log:
            self.get_logger().info("Returning response...")

        return response

    # ======================================================================
    #  3. from_yaml HOT RELOAD
    # ======================================================================

    def _maybe_reload_yaml(self):
        """(Re)load the YAML trajectory when the file appears or changes."""
        if not self._yaml_selected_path:
            return
        try:
            mtime = os.path.getmtime(self._yaml_selected_path)
        except OSError:
            return  # not written yet -- keep holding position
        if self.yaml_traj is not None and mtime == self._yaml_mtime:
            return
        try:
            self.yaml_traj = yt.YamlTrajectory(self._yaml_selected_path)
            self._yaml_mtime = mtime
            self.get_logger().info(
                f"Loaded YAML trajectory '{self.yaml_traj.name}' "
                f"({self.yaml_traj.duration:.1f} s) from "
                f"{self._yaml_selected_path}")
        except Exception as exc:
            self.get_logger().error(
                f"Failed to load YAML trajectory "
                f"'{self._yaml_selected_path}': {exc}")

    # ======================================================================
    #  4. THE SHAPE LIBRARY -- reference conditions for recorded field data
    # ======================================================================

    def single_pose(self, t: float, path_shape = 'station_keeping') -> PoseStamped:
        """
        The pose of `path_shape` at path parameter t.

        The hard-coded shapes are REFERENCE CONDITIONS for existing field data,
        versioned nowhere: changing a formula silently invalidates comparison
        with every earlier run on that shape. If you must, record it in
        TRAJECTORY_SYSTEM.md's shape revision record. Speed is baked into each
        formula (`x = 0.5*t` = 0.5 m/s). Raises ValueError outside SHAPES.
        """
        # Designer file: dense [t, x, y, yaw], interpolated (yaml_trajectory);
        # a station-keeping pose at the origin until the file is loaded.
        if path_shape.startswith('from_yaml'):
            if self.yaml_traj is None:
                x, y, z, roll, pitch, yaw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            else:
                x, y, z, roll, pitch, yaw = yt.read_yaml(self.yaml_traj, t)
            quat = R.from_euler('zyx', [yaw, 0.0, 0.0]).as_quat()
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            return pose

        if path_shape == 'station_keeping':
            x = 0.0
            y = 0.0
            z = 0.0
            roll = 0.0
            pitch = 0.0
            yaw = 0.0

        elif path_shape == 'circle':
            radius = 4.0 # meters
            t *= 0.08
            x = -radius + radius * np.cos(t)
            y = radius * np.sin(t)
            z = 0.0

            dx = -radius * np.sin(t)
            dy = radius * np.cos(t)
            yaw = np.arctan2(dy, dx)
            yaw = (yaw + np.pi) % (2 * np.pi) - np.pi # Normalize

        elif path_shape == 'straight_line':
            x = 0.5*t
            y = 0.0*t + 1.0
            z = 0.0
            yaw = 0.0

        elif path_shape == 'sin':
            t = min(t, 500.0)   # hold the last pose (yaml_trajectory's convention)
            a = 3.5
            f = 0.2
            vx = 0.4

            t *= 0.7

            x = 0.5 + vx*t
            y = 0. + a * (np.sin(f*t-np.pi/2) + 1)
            z = 0.0

            dx = vx
            dy = a * f * np.cos(f*t - np.pi/2)
            yaw = np.arctan2(dy, dx)

        elif path_shape == 'fsin':
            # Turn radius of the weave [m], the shape's one knob (1.5 m -> one
            # cycle per 60 s at _FSIN_V).
            radius = 1.5
            z = 0.0
            x, y, yaw = _fsin_state(t, radius)

        elif path_shape == 'square':
            heading_dt = 0.01
            t /= 2
            def get_xy(s):
                x = s
                cycles = math.floor(s / 3)
                y = 2.0 if cycles % 2 == 0 else -2.0
                return x, y

            x, y = get_xy(t)
            x = float(x)
            y = float(y)
            z = 0.0

            # Heading by forward difference.
            x_fwd, y_fwd = get_xy(t + heading_dt)
            dx = x_fwd - x
            dy = y_fwd - y
            yaw = math.atan2(dy, dx)

        elif path_shape == 'kin_square':
            t = min(t, 500.0)   # hold the last pose (yaml_trajectory's convention)
            segment_length = 5.0
            surge_speed = 0.3
            z = 0.0
            t *= 1.
            segment_time = segment_length / surge_speed
            segment_index = int(t // segment_time)
            t_in_segment = t % segment_time

            directions = [
                (1, 0),     # +X
                (0, 1),     # +Y
                (1, 0),    # +X
                (0, -1),    # -Y
            ]
            yaws = [0, math.pi/2, 0, -math.pi/2]

            dir_idx = segment_index % 4
            dx, dy = directions[dir_idx]
            yaw = yaws[dir_idx]

            # Completed segments, then along the current one.
            x, y = 0.0, 0.0
            for i in range(segment_index):
                dxi, dyi = directions[i % 4]
                x += dxi * segment_length
                y += dyi * segment_length

            x += dx * surge_speed * t_in_segment
            y += dy * surge_speed * t_in_segment

        elif path_shape == 'seabed_scanning':
            x,y,z,roll,pitch,yaw = cf.seabed_scanning(t)
            x = float(x)
            y = float(y)
            z = 0.0
            yaw = float(yaw)

        else:
            raise ValueError(_unknown_shape_message(path_shape))

        quat = R.from_euler('zyx', [yaw, 0.0, 0.0]).as_quat()

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose


def main(args=None):
    rclpy.init(args=args)
    node = PathGeneration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
