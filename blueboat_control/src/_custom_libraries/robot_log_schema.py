#!/usr/bin/env python3

r"""
Column schema of the robot-side position CSV written by robot_interface.py.

ROS-FREE BY CONSTRUCTION -- this module contains data and nothing else. It
imports no numpy, no pandas, no rclpy. Read it to learn the CSV format
without opening an 858-line node.

WRITE-ONCE FIELD DATA (superproject CM-7 / this module's N7).
--------------------------------------------------------------
`<root>/data/Robot_data/{date}-{note}-poslog.csv` is a PRIMARY FIELD RECORD.
Renaming a column, reordering one, or changing what a column means silently
invalidates every CSV recorded before the change: the layout is not versioned
in the file, and no reader validates it. Recorded runs cannot be repeated to
match. If a column must change, treat every earlier log as a different format.

Rows are filled BY COLUMN NAME (`df.loc[row, 'relative_x'] = ...`), never by
positional index, precisely so that the order below can never silently
de-synchronise from the values written into it. Keep it that way.

Which layout is used
--------------------
Selected once, at construction, by the `use_UWgps` launch parameter -- which
is itself set from `use_pinger` by BlueBoat_launch.py:

    use_UWgps = False  ->  COLUMNS_NO_PINGER  (26 columns)
    use_UWgps = True   ->  COLUMNS_PINGER     (40 columns)

The two layouts are NOT a subset of one another. They share the first ten
columns and the trailing IMU block; the middle differs.

Column groups, in the order they appear
---------------------------------------
Both layouts:
  Year..MicroSecond      (7)  wall-clock stamp of the row, local time.
  relative_x/y/psi       (3)  robot pose in the BOOT-RELATIVE world frame --
                              origin and yaw are re-zeroed at robot_interface's
                              first odom callback, so (0,0,0) is wherever the
                              boat was at launch, NOT a fixed geographic frame.
                              Metres and radians.
  gps_latitude/longitude (2)  raw /mavros/global_position/global fix, degrees.
  right_thr_in           (1)  \  thrust in Newtons. NOTE THE ORDER: right
  left_thr_in            (1)  /  first, matching /thruster_input's [right, left]
                              convention. These two were historically swapped.
  quat_x/y/z/w           (4)  \
  ang_vel_x/y/z          (3)   > raw IMU, /mavros/imu/data, unprocessed.
  lin_acc_x/y/z          (3)  /

COLUMNS_NO_PINGER only:
  target_x/y             (2)  the controller's current world-frame target,
                              read from /monitoring_data[4:6]. World-frame in
                              EVERY controller branch (CM-8 / N9), so no frame
                              correction is applied here or downstream.
                              EXISTS ONLY IN THIS LAYOUT -- in pinger mode it
                              duplicated corrected_pinger_x/y and was removed.

COLUMNS_PINGER only:
  corrected_pinger_x/y   (2)  pinger position rotated into the boot-relative
                              world frame. This is the world-frame column pair;
                              the body-frame vector goes out on
                              /blueboat/pinger_coordinates instead.
  pinger_latitude/longitude (2) the same point converted to WGS84 degrees.
  aco_x/y/z              (3)  \
  ant_x/y/z              (3)   \ raw Water Linked UGPS packet, straight off
  lat, lon, dep          (3)   / /uw_gps_data (19 values; the 7 date fields
  filaco_x/y/z           (3)  /  of that message are not repeated here).
                              filaco_* is the FILTERED acoustic position and is
                              what seeds the dead reckoning.

Consumers
---------
* BlueBoat-Control/blueboat_control/src/docs/controllers/replay.py
* offline analysis notebooks
Both index by column NAME. Neither tolerates a renamed column.
"""

# 26 columns. use_UWgps = False.
COLUMNS_NO_PINGER = (
    'Year', 'Month', 'Day', 'Hour', 'Minute', 'Second', 'MicroSecond',
    'relative_x', 'relative_y', 'relative_psi',
    'target_x', 'target_y',
    'gps_latitude', 'gps_longitude',
    'right_thr_in', 'left_thr_in',
    'quat_x', 'quat_y', 'quat_z', 'quat_w',
    'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
    'lin_acc_x', 'lin_acc_y', 'lin_acc_z',
)

# 40 columns. use_UWgps = True.
COLUMNS_PINGER = (
    'Year', 'Month', 'Day', 'Hour', 'Minute', 'Second', 'MicroSecond',
    'relative_x', 'relative_y', 'relative_psi',
    'corrected_pinger_x', 'corrected_pinger_y',
    'gps_latitude', 'gps_longitude',
    'pinger_latitude', 'pinger_longitude',
    'right_thr_in', 'left_thr_in',
    'aco_x', 'aco_y', 'aco_z',
    'ant_x', 'ant_y', 'ant_z',
    'lat', 'lon', 'dep',
    'filaco_x', 'filaco_y', 'filaco_z',
    'quat_x', 'quat_y', 'quat_z', 'quat_w',
    'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
    'lin_acc_x', 'lin_acc_y', 'lin_acc_z',
)


def columns_for(use_UWgps: bool) -> list:
    """
    Return the column list for the layout `use_UWgps` selects.

    Input  : use_UWgps -- bool, the launch parameter of the same name.
    Output : a NEW list[str] (26 or 40 entries). A fresh list every call, so
             the caller may hand it to pandas and the module constants above
             can never be mutated through it.
    """
    return list(COLUMNS_PINGER if use_UWgps else COLUMNS_NO_PINGER)
