#!/usr/bin/env python3
r"""
The LEGACY pinger: robot_interface's pinger algorithm as it was until 2026-11-24
(Control v1.1, `44ec6aa`), moved here arithmetic for arithmetic.

ROS-FREE BY CONSTRUCTION (numpy only). Kept for two reasons only:
  * the "legacy" comparison layer in MCS, the logs (`corrected_pinger_*` keeps
    meaning exactly what it always meant) and the offline tools;
  * an emergency fallback: `pinger_source:=legacy` puts this vector back on
    /blueboat/pinger_coordinates.

What it does (and why it is wrong, docs/PINGER_FILTER_FINDINGS.md section 7):
every /uw_gps_data packet OVERWRITES the body vector with WL's filtered xyz,
new fix or not; between packets the vector is dead-reckoned at odom rate with
the MAVROS body twist and the IMU yaw rate, `p -= (v + w x p) dt`. The vector was
measured 1.5-2.5 s earlier from where the boat was then and is applied at the
current pose, so the shown pinger lags the boat's motion; a repeated packet
re-applies the old vector at a newer pose ("dragged by the boat between fixes").

Equivalence: given the same sequence of (odom, imu, packet) events and the
same odom-to-odom dt, `on_odom` returns exactly the vector robot_interface
published (test/test_pinger_filter.py checks it against the v1.1 code).
"""

import numpy as np


class LegacyPinger:
    """robot_interface v1.1 pinger state, fed event by event."""

    def __init__(self):
        # Dead switch carried over unchanged from robot_interface (always False):
        # True would publish a WORLD vector on the body topic without dead
        # reckoning. Left alone on purpose.
        self.fixed_pinger = False
        self.pinger_coordinates = np.zeros(3)
        self.vel = np.zeros(3)
        self.prev_time = None
        self.angular_velocity_z = None          # None until the first IMU message

    def on_imu(self, angular_velocity_z):
        """imu_callback: the latest yaw rate (rad/s)."""
        self.angular_velocity_z = angular_velocity_z

    def on_ugps(self, data):
        """uw_gps_callback: reseed from the FILTERED acoustic position (16..18)."""
        t_x, t_y, t_z = data[16], data[17], data[18]
        self.pinger_coordinates = np.array([t_x, t_y, t_z])

    def on_odom(self, t, vel_xyz, x_rel=0.0, y_rel=0.0, yaw=0.0):
        """odom_callback at receive time `t` (s) with the body twist (vx, vy, vz).

        Returns the body vector robot_interface would have published on this
        odom (a copy), or None for the very first odom (it only latched time).
        With the dead `fixed_pinger` switch it would return the world vector.
        """
        if self.prev_time is None:
            self.prev_time = t
            return None
        dt = t - self.prev_time
        self.prev_time = t

        self.vel = np.array([vel_xyz[0], vel_xyz[1], vel_xyz[2]])

        if self.fixed_pinger and not all(self.pinger_coordinates == np.zeros(3)):
            c, s = np.cos(yaw), np.sin(yaw)
            x_b, y_b = self.pinger_coordinates[0], self.pinger_coordinates[1]
            return [x_rel + (c * x_b - s * y_b), y_rel + (s * x_b + c * y_b)]
        if self.fixed_pinger:
            return None

        av = self.angular_velocity_z
        if av is not None and not all(self.pinger_coordinates == np.zeros(3)):
            omega = np.array([0.0, 0.0, av])
            p = self.pinger_coordinates
            self.pinger_coordinates -= (self.vel + np.cross(omega, p)) * dt
        return self.pinger_coordinates.copy()

    def body(self):
        return self.pinger_coordinates.copy()
