#!/usr/bin/env python3

"""
Pure frame-conversion maths for the control stack.

ROS-FREE BY CONSTRUCTION. This module imports numpy and nothing else -- no
rclpy, no message packages, no node handles. That is the point of it: every
function here can be called, diffed and unit-checked from a plain Python
prompt with no sourced workspace, which is what makes the geometry debuggable
without a running graph.

    cd blueboat_control/src/_custom_libraries && python3
    >>> import frame_math as fm
    >>> fm.inRobotFrame([0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0])
    (1.0, 0.0, 0.0)

Installed flat into lib/blueboat_control by CMakeLists.txt, like every other
script in this package, so `import frame_math as fm` resolves at runtime.
"""

import numpy as np


def inRobotFrame(robot_coords, target_coords):
    """
    Express a world-frame target in the robot's body frame.

    Inputs
    ------
    robot_coords  : sequence of EXACTLY 6 floats, (x, y, psi, *_).
                    The robot pose in the world frame. x, y in metres;
                    psi in radians, CCW-positive about +z. The last three
                    slots are unpacked and discarded -- callers pass the
                    6-element state vector [x, y, psi, u, v, r], so the
                    velocities are ignored here.
    target_coords : sequence of EXACTLY 6 floats, (x, y, psi, *_).
                    The target pose in the SAME world frame, same units.

    Both arguments must have length 6; a shorter sequence raises ValueError
    on unpacking. That is deliberate -- it is the state-vector shape used
    throughout master_control.

    Returns
    -------
    (x, y, psi) : tuple of 3 floats, the target expressed in the robot's
                  body frame. x is surge (ahead of the robot, metres),
                  y is sway (to port, metres), psi is the heading error in
                  radians.

    Relations
    ---------
    * Rotation only, by -psi_r about +z, after translating by -(x_r, y_r).
      The inverse operation is custom_functions.transform_body_to_world().
    * `psi` is the difference of two INDIVIDUALLY wrapped angles, so it lies
      in (-2*pi, 2*pi) -- it is NOT itself wrapped to (-pi, pi]. Callers that
      need a wrapped heading error must wrap the result themselves. This is
      the historical behaviour and is preserved verbatim.

    Where it is used
    ----------------
    master_control.timer_callback puts the MANUAL target through this before
    solve_LoS, and passes the PINGER vector straight through, because
    /blueboat/pinger_coordinates is already body-frame on the wire while
    /blueboat/manual_target is world-frame. That asymmetry is correct by
    design -- both are body-frame by the time they reach the solver.
    """

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


# ---------------------------------------------------------------------------
# Geodesy: lat/lon <-> local east/north metres about an anchor.
#
# The equirectangular model on a sphere of the WGS-84 equatorial radius that
# every module of the superproject uses (blueboat_sss_sim core/geo.py, MCS,
# GCS -- "byte-identical" by convention), ported here verbatim rather than
# imported: no module imports a neighbour's package (CM-3). Used by
# bag_replay.py to place a recorded run in a GPS-anchored built world.
# ---------------------------------------------------------------------------

EARTH_RADIUS_M = 6378137.0


def latlon_to_local_en(lat, lon, lat0, lon0):
    """(lat, lon) degrees -> (east, north) metres about the anchor (lat0, lon0)."""
    east = np.radians(np.asarray(lon) - lon0) * EARTH_RADIUS_M * np.cos(np.radians(lat0))
    north = np.radians(np.asarray(lat) - lat0) * EARTH_RADIUS_M
    return east, north


def local_en_to_latlon(east, north, lat0, lon0):
    """(east, north) metres about the anchor -> (lat, lon) degrees. Exact
    inverse of latlon_to_local_en."""
    lat = lat0 + np.degrees(np.asarray(north) / EARTH_RADIUS_M)
    lon = lon0 + np.degrees(np.asarray(east) / (EARTH_RADIUS_M * np.cos(np.radians(lat0))))
    return lat, lon
