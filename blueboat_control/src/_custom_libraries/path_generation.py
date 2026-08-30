#!/usr/bin/env python3

# ============================================================================
# The '/path_request' server: an array of path-parameter values in, a
# nav_msgs/Path out. Deliberately parameter-agnostic -- the caller decides what
# the numbers mean.
#
# Mission Pattern Designer support (Mission Control Station), marked below with
# "# --- YAML trajectory support ---":
#   1. import of the yaml_trajectory helper module (same directory);
#   2. loading of a designer-generated YAML file when the 'trajectory'
#      parameter is 'from_yaml:<absolute path>' (or the optional
#      'yaml_path' parameter is set). The file is WATCHED: if it does not
#      exist yet, or its modification time changes, it is (re)loaded on the
#      next path request. This enables GPS-anchored missions: the station
#      writes the deployed file only once the run's odom<->GPS fit is
#      established, and the node holds position (station-keeping fallback
#      pose) until then;
#   3. one new branch in single_pose().
#
# The hard-coded shapes are the reference conditions for existing field data.
# Every one is byte-identical to the original except 'sin' and 'kin_square',
# which now hold their last pose past t = 500 instead of teleporting back to
# t = 50; TRAJECTORY_SYSTEM.md carries the shape revision record.
# ============================================================================

# ----------------------------------------------------------------------------
# FILE MAP
#
#   module scope    SHAPES, is_valid_shape, the 'fsin' cumulative table
#                   (_fsin_extend / _fsin_state and the _fsin_* globals).
#                   The table MUST stay at this module's scope:
#                   docs/controllers/check_trajectory_library.py resets it by
#                   assigning path_generation._fsin_x/_y/_yaw, and moving it to
#                   another module would make that reset a silent no-op.
#
#   class PathGeneration
#     1. WIRING                   __init__
#     2. SERVICE ENTRY POINT      generate_path
#     3. from_yaml HOT RELOAD     _maybe_reload_yaml
#     4. THE SHAPE LIBRARY        single_pose   <-- formulas are reference
#                                 conditions for recorded field data
# ----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from blueboat_interfaces.srv import RequestPath
import math
import custom_functions as cf

# --- YAML trajectory support ------------------------------------------------
import yaml_trajectory as yt
# -----------------------------------------------------------------------------

"""
Creates a services that handle path generation requests. Receives a an array of time values and responds with the associated path.
"""

# The complete set of `trajectory:=` selectors single_pose serves. Anything else
# is an operator typo: the node refuses to start rather than serving a path that
# silently is not the one that was asked for.
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
# 'fsin' is the one shape with no closed form: its heading is the integral of a
# sine (analytic), but x and y are integrals of the cosine and sine OF that
# heading, which are not. It was therefore re-integrated from t=0, in a Python
# loop at 0.01 s, on every single pose -- O(t) per pose, so a whole-path request
# was O(n^2) and appeared to hang the launch.
#
# The integration is now done once on the same fixed 0.01 s grid and read out by
# index. The table only ever grows, and each extension CONTINUES the accumulation
# from the stored last value -- cumsum([last, *increments]), never
# cumsum(increments) + last -- so the float sequence is identical to a single
# pass. single_pose therefore stays pure in t: the same t gives the same pose
# regardless of what was asked for before it, which is what lets a trajectory be
# swapped, replayed or hot-reloaded.
_FSIN_V = 0.1           # surge [m/s] -- the authored speed
_FSIN_A = 1             # yaw-rate amplitude
_FSIN_F = 0.05          # yaw-rate frequency [Hz]
_FSIN_DT = 0.01         # integration step [s]
_FSIN_MAX_STEPS = 10_000_000    # 100 000 s of path; beyond it, hold the last pose

_fsin_yaw = np.zeros(1)
_fsin_x = np.zeros(1)
_fsin_y = np.zeros(1)


def _fsin_extend(steps: int) -> None:
    """Grow the table so index `steps` exists, continuing the same recursion."""
    global _fsin_yaw, _fsin_x, _fsin_y
    have = _fsin_yaw.size - 1
    if steps <= have:
        return
    target = min(max(steps, 2 * have, 1024), _FSIN_MAX_STEPS)
    i = np.arange(have, target)                  # the steps still to take
    tau = i * _FSIN_DT
    omega = _FSIN_A * np.sin(2 * np.pi * _FSIN_F * tau)
    yaw = np.cumsum(np.concatenate(([_fsin_yaw[-1]], omega * _FSIN_DT)))
    x = np.cumsum(np.concatenate(([_fsin_x[-1]], _FSIN_V * np.cos(yaw[1:]) * _FSIN_DT)))
    y = np.cumsum(np.concatenate(([_fsin_y[-1]], _FSIN_V * np.sin(yaw[1:]) * _FSIN_DT)))
    _fsin_yaw = np.concatenate((_fsin_yaw, yaw[1:]))
    _fsin_x = np.concatenate((_fsin_x, x[1:]))
    _fsin_y = np.concatenate((_fsin_y, y[1:]))


def _fsin_state(t: float):
    """(x, y, yaw) of the 'fsin' trajectory at time t. Pure in t."""
    steps = int(t / _FSIN_DT)
    if steps <= 0:
        return 0.0, 0.0, 0.0
    steps = min(steps, _FSIN_MAX_STEPS)   # hold the last pose, as every shape does
    _fsin_extend(steps)
    return _fsin_x[steps], _fsin_y[steps], _fsin_yaw[steps]
# -----------------------------------------------------------------------------


class PathGeneration(Node):

    # ======================================================================
    #  1. WIRING
    #  parameters and the /path_request service.
    # ======================================================================

    def __init__(self):
        super().__init__('path_generation')

        # Declare parameters
        self.declare_parameter('display_log', False)
        self.display_log = self.get_parameter('display_log').value

        self.declare_parameter('trajectory', 'station_keeping')
        self.trajectory = self.get_parameter('trajectory').get_parameter_value().string_value

        # An unrecognised name used to surface only as an exception inside the
        # service handler, which kills the node on the first request: rclpy does
        # not marshal a callback exception back to the caller, so master_control
        # saw nothing but "Nothing to target yet." forever. Fail here instead,
        # before anything is armed, naming the shape and the valid set.
        if not is_valid_shape(self.trajectory):
            msg = _unknown_shape_message(self.trajectory)
            self.get_logger().fatal(msg)
            raise ValueError(msg)

        # --- YAML trajectory support -----------------------------------------
        # A designer-generated trajectory is selected either with
        #   trajectory:=from_yaml:<absolute path to .yaml>
        # (no launch-file change needed: the file path rides inside the
        # existing 'trajectory' argument), or with the optional dedicated
        # parameter yaml_path:=<path> combined with trajectory:=from_yaml.
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
        # -----------------------------------------------------------------------

        # Service
        self.path_service = self.create_service(RequestPath, '/path_request', self.generate_path)

    # ======================================================================
    #  2. SERVICE ENTRY POINT
    #  One pose per requested parameter value. Response frame_id is 'world'.
    # ======================================================================

    def generate_path(self, request, response):
        # --- YAML trajectory support: pick up newly deployed files ---------
        self._maybe_reload_yaml()
        # -------------------------------------------------------------------
        if self.display_log:
            self.get_logger().info(f"Received path_request of type: {type(request.path_request)}")

        path_msg = Path()
        path_msg.header.frame_id = 'world'

        for t in request.path_request.data:
            temp_pose = self.single_pose(t, self.trajectory)
            temp_pose.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses.append(temp_pose)

        response.path = path_msg

        if self.display_log:
            self.get_logger().info("Returning response...")

        return response

    # ======================================================================
    #  3. from_yaml HOT RELOAD
    #  Watches the designer file so a GPS-anchored mission can be deployed mid-run.
    # ======================================================================

    # --- YAML trajectory support ------------------------------------------
    def _maybe_reload_yaml(self):
        """(Re)load the YAML trajectory when the file appears or changes."""
        if not self._yaml_selected_path:
            return
        import os
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
    #  4. THE SHAPE LIBRARY
    #  Formulas are REFERENCE CONDITIONS for recorded field data -- read the docstring.
    # ======================================================================

    # -----------------------------------------------------------------------

    def single_pose(self, t: float, path_shape = 'station_keeping') -> PoseStamped:
        """
        Generate a path for a given time t.

        The hard-coded shapes below are the REFERENCE CONDITIONS for existing field
        data. Changing a formula silently invalidates comparison with every earlier
        run on that shape: the shape is not versioned in the code, in the position
        CSV or in the .npy log, so nothing raises an error. Field data is write-once
        and cannot be re-collected to match a changed formula. If you change one,
        record which shape moved and from what date in the shape revision record in
        TRAJECTORY_SYSTEM.md ("The built-in shapes"), and treat prior runs on that
        shape as not comparable.

        Speed is baked into each formula -- `x = 0.5*t` means 0.5 m/s.

        Raises ValueError, naming the shape and the valid set, for anything not
        in SHAPES. The node also refuses to start on a bad `trajectory:=`, so
        this is the second line of defence rather than the first.
        """
        depth_per_circle = 2.0  # meters
        num_turns = 3
        total_length = 2 * np.pi * num_turns

        # --- YAML trajectory support -----------------------------------------
        # Designer-generated trajectory: dense [t, x, y, yaw] samples,
        # linearly interpolated at time t (see yaml_trajectory.py). While
        # the file has not been written/loaded yet (GPS-anchored mission
        # awaiting deployment), a station-keeping pose at the origin is
        # returned so the controller holds position.
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
        # -----------------------------------------------------------------------

        # Station keeping
        if path_shape == 'station_keeping':
            x = 0.0
            y = 0.0
            z = 0.0
            roll = 0.0
            pitch = 0.0
            yaw = 0.0
        
        # Circle
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

        # Straight line
        elif path_shape == 'straight_line':
            x = 0.5*t
            y = 0.0*t + 1.0
            z = 0.0
            yaw = 0.0

        # Sin line
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

        # Surge sin
        elif path_shape == 'fsin':
            # Same Euler integration as ever, read out of a cumulative table
            # instead of re-run from t=0 on every pose (see _fsin_state).
            z = 0.0
            x, y, yaw = _fsin_state(t)

        # Square wave
        elif path_shape == 'square':
            period = 0.01
            amplitude = 2.0
            heading_dt = 0.01
            t /= 2
            def get_xy(s):
                x = s
                cycles = math.floor(s / 3)
                y = 2.0 if cycles % 2 == 0 else -2.0
                return x, y

            # Current position
            x, y = get_xy(t)
            x = float(x)
            y = float(y)
            z = 0.0

            # Compute heading using forward difference
            x_fwd, y_fwd = get_xy(t + heading_dt)
            dx = x_fwd - x
            dy = y_fwd - y
            yaw = math.atan2(dy, dx)

        # Kinematic square wave
        elif path_shape == 'kin_square':
            t = min(t, 500.0)   # hold the last pose (yaml_trajectory's convention)
            segment_length = 5.0
            surge_speed = 0.3
            z = 0.0
            t *= 1.
            # Time per segment
            segment_time = segment_length / surge_speed

            # Determine which segment we're in
            segment_index = int(t // segment_time)
            t_in_segment = t % segment_time
            
            directions = [
                (1, 0),     # +X
                (0, 1),     # +Y
                (1, 0),    # +X
                (0, -1),    # -Y
            ]
            yaws = [0, math.pi/2, 0, -math.pi/2]

            # Get direction and yaw
            dir_idx = segment_index % 4
            dx, dy = directions[dir_idx]
            yaw = yaws[dir_idx]

            # Total completed segments
            completed = segment_index

            # Compute cumulative position
            x, y = 0.0, 0.0
            for i in range(completed):
                dxi, dyi = directions[i % 4]
                x += dxi * segment_length
                y += dyi * segment_length

            # Move along current segment
            x += dx * surge_speed * t_in_segment
            y += dy * surge_speed * t_in_segment

        # Seabed scanning
        elif path_shape == 'seabed_scanning':
            x,y,z,roll,pitch,yaw = cf.seabed_scanning(t)
            x = float(x)
            y = float(y)
            z = 0.0
            yaw = float(yaw)

        else:
            # No fall-through: the chain above is exhaustive over SHAPES, so an
            # unrecognised name can never leave x/y/z/yaw unbound again.
            raise ValueError(_unknown_shape_message(path_shape))

        # Create and return pose
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
