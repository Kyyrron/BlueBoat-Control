#!/usr/bin/env python3

# ============================================================================
# PATCHED for the Mission Pattern Designer (Mission Control Station).
# The ONLY changes relative to the original file are marked with
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
# Every hard-coded trajectory below is byte-identical to the original and
# keeps working exactly as before. generate_path() is unchanged.
# ============================================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Float32MultiArray
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

class PathGeneration(Node):
    def __init__(self):
        super().__init__('path_generation')

        # Declare parameters
        self.declare_parameter('display_log', False)
        self.display_log = self.get_parameter('display_log').value

        self.declare_parameter('trajectory', 'station_keeping')
        self.trajectory = self.get_parameter('trajectory').get_parameter_value().string_value

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
    # -----------------------------------------------------------------------

    def single_pose(self, t: float, path_shape = 'station_keeping') -> PoseStamped:
        """
        Generate a path for a given time t.
        """
        #TODO Find a more elegant method to select path (probably a dictionnary)

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
        if path_shape == 'circle':
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
        if path_shape == 'straight_line':
            x = 0.5*t
            y = 0.0*t + 1.0
            z = 0.0
            yaw = 0.0

        # Sin line
        if path_shape == 'sin':
            if t>500:
                t = 50
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
        if path_shape == 'fsin':
            v = 0.1
            A = 1
            f = 0.05
            dt = 0.01

            x = 0.0
            y = 0.0
            z = 0.0

            yaw = 0.0

            steps = int(t / dt)

            for i in range(steps):
                tau = i * dt

                omega = A * np.sin(2 * np.pi * f * tau)
                yaw += omega * dt

                x += v * np.cos(yaw) * dt
                y += v * np.sin(yaw) * dt

        # Square wave
        if path_shape == 'square':
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
        if path_shape == 'kin_square':
            if t>500:
                t= 50
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
        if path_shape == 'seabed_scanning':
            x,y,z,roll,pitch,yaw = cf.seabed_scanning(t)
            x = float(x)
            y = float(y)
            z = 0.0
            yaw = float(yaw)

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

    def single_request(self, msg: Float32):
        time_request = msg.data
        desired_pose = self.single_pose(time_request)
        desired_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_publisher.publish(desired_pose)


def main(args=None):
    rclpy.init(args=args)
    node = PathGeneration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
