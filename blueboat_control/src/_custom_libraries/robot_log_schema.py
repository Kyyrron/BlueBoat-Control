#!/usr/bin/env python3

r"""
Column schema of the robot-side position CSV written by robot_interface.py.

ROS-FREE BY CONSTRUCTION -- this module contains data and nothing else. It
imports no numpy, no pandas, no rclpy. Read it to learn the CSV format
without opening an 850-line node.

WRITE-ONCE FIELD DATA (superproject CM-7 / this module's N7).
--------------------------------------------------------------
`<root>/data/Robot_data/{date}-{note}-poslog.csv` is a PRIMARY FIELD RECORD.
Renaming a column, reordering one, or changing what a column means silently
invalidates every CSV recorded before the change: the layout is not versioned
in the file, and no reader validates it. Recorded runs cannot be repeated to
match. If a column must change, treat every earlier log as a different format.

SCHEMA REVISION -- 2026-08-31. The layouts below are NOT the ones any earlier
document describes. Changed in this revision:
  * `target_latitude` / `target_longitude` ADDED to the no-pinger layout, so
    both layouts now carry the target's GPS position immediately after the
    robot's (the pinger layout already had `pinger_latitude/longitude`).
  * `actuation_state` ADDED to both layouts -- see the encoding below.
  * `quat_x/y/z/w` REPLACED by `roll`, `pitch`. The quaternion's yaw was
    already present as `relative_psi`, so the other two Euler angles carry
    everything it did in half the columns.
  * Column ORDER regrouped in both layouts (see below).
`data/Robot_data/` was empty when this revision landed, so no recorded field
CSV was invalidated by it. Any CSV predating 2026-08-31 is a different format.

Rows are filled BY COLUMN NAME (`df.loc[row, 'relative_x'] = ...`), never by
positional index, precisely so that the order below can never silently
de-synchronise from the values written into it. Keep it that way.

Which layout is used
--------------------
Selected once, at construction, by the `use_UWgps` launch parameter -- which
is itself set from `use_pinger` by BlueBoat_launch.py:

    use_UWgps = False  ->  COLUMNS_NO_PINGER  (27 columns)
    use_UWgps = True   ->  COLUMNS_PINGER     (39 columns)

The two layouts are NOT a subset of one another, but as of this revision they
share the SAME FIRST 19 COLUMNS STRUCTURALLY -- time, robot pose, target pose,
robot GPS, target GPS, actuation, in that order and those group sizes. Only
the two names of the target block differ (`target_*` against
`corrected_pinger_*` / `pinger_*`), because a pinger target and a path target
are not the same object. Everything after column 19 is layout-specific.

One writer, one rate
--------------------
Both layouts are written by `robot_interface.log_timer_callback` at its own
timer rate (0.33 s). This was not always true: the pinger layout used to be
written from `uw_gps_callback`, i.e. driven by the Water Linked link at 2 Hz,
so a UGPS dropout stopped logging the ROBOT as well. The UGPS callback now
only caches its packet; the timer owns the file.

Column groups, in the order they appear
---------------------------------------
Both layouts, columns 1-19:
  Year..MicroSecond      (7)  wall-clock stamp of the row, local time.
                              MicroSecond is FULL microseconds (0-999999) in
                              BOTH layouts. It used to be milliseconds in the
                              no-pinger layout and microseconds in the other.
  relative_x/y/psi       (3)  robot pose in the BOOT-RELATIVE world frame --
                              origin and yaw are re-zeroed at robot_interface's
                              first odom callback, so (0,0,0) is wherever the
                              boat was at launch, NOT a fixed geographic frame.
                              Metres and radians. The frame's own origin is
                              recorded once in the `-origin.yaml` sidecar
                              written beside the CSV; without it these columns
                              cannot be georeferenced after the fact.
  [target pose, world]   (2)  the controller's current target, SAME FRAME as
                              relative_x/y above, so the two pairs subtract
                              directly. Named `target_x/y` in the no-pinger
                              layout (read from /monitoring_data[4:6],
                              world-frame in EVERY controller branch, CM-8/N9)
                              and `corrected_pinger_x/y` in the pinger layout.
  gps_latitude/longitude (2)  raw /mavros/global_position/global fix, degrees.
  [target GPS]           (2)  the same target as two pairs above, converted to
                              WGS84 degrees through the run's origin fix.
                              Named `target_latitude/longitude` in the
                              no-pinger layout and `pinger_latitude/longitude`
                              in the pinger layout. Placed immediately after
                              the robot's fix so the two can be selected and
                              plotted together.
  right_thr_in           (1)  \  thrust in Newtons, as COMMANDED on
  left_thr_in            (1)  /  /thruster_input. NOTE THE ORDER: right first,
                              matching that topic's [right, left] convention.
                              These two were historically swapped.
  actuation_state        (1)  whether that command could reach the water:
                                0  motors disabled (enable_motors False) --
                                   no PWM left the boat at all
                                1  enabled AND param_mode == 'override' --
                                   live, thrust reaching the motors
                                2  enabled but not in override -- ArduPilot
                                   ignores the RC override stream
                                3  loss-of-reference watchdog tripped --
                                   thrust forced to zero by robot_interface
                              Without this column a run with the motor gate
                              off is byte-indistinguishable from a live one,
                              and a watchdog trip is indistinguishable from a
                              genuine zero command (the watchdog writes [0,0]
                              into the same field).

Both layouts, trailing block:
  roll, pitch            (2)  from /mavros/imu/data's quaternion. Yaw is not
                              repeated here -- it is relative_psi above.
  ang_vel_x/y/z          (3)   > raw IMU, /mavros/imu/data, unprocessed.
  lin_acc_x/y/z          (3)  /

COLUMNS_PINGER only, between the two blocks:
  aco_x/y/z              (3)  \
  ant_x/y/z              (3)   \ raw Water Linked UGPS packet, straight off
  lat, lon, dep          (3)   / /uw_gps_data (19 values; the 7 date fields
  filaco_x/y/z           (3)  /  of that message are not repeated here).
                              filaco_* is the FILTERED acoustic position and is
                              what seeds the dead reckoning. Two caveats worth
                              knowing before analysing them: ant_* is only
                              populated when uwgps_log is given its --antenna
                              flag, which no launch file passes, and `dep` is
                              set from the same value as aco_z rather than
                              from an independent depth sensor.

Consumers
---------
* BlueBoat-Control/blueboat_control/src/docs/controllers/replay.py
* offline analysis notebooks
Both index by column NAME. Neither tolerates a renamed column. NOTE that
replay.read_poslog_csv currently looks for columns named `x`, `y`, `psi`, `t`,
`u1`, `u2`, which NO revision of this schema has ever contained -- it cannot
read a CSV this system produces, and that predates this revision.
"""

# Actuation-state encoding, for readers that would rather not hard-code ints.
ACT_MOTORS_DISABLED = 0   # enable_motors False -- no PWM left the boat
ACT_LIVE = 1              # enabled and in override -- thrust reaching motors
ACT_NOT_OVERRIDE = 2      # enabled, but ArduPilot is ignoring the RC override
ACT_WATCHDOG = 3          # loss-of-reference watchdog forcing thrust to zero

# 27 columns. use_UWgps = False.
COLUMNS_NO_PINGER = (
    'Year', 'Month', 'Day', 'Hour', 'Minute', 'Second', 'MicroSecond',
    'relative_x', 'relative_y', 'relative_psi',
    'target_x', 'target_y',
    'gps_latitude', 'gps_longitude',
    'target_latitude', 'target_longitude',
    'right_thr_in', 'left_thr_in', 'actuation_state',
    'roll', 'pitch',
    'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
    'lin_acc_x', 'lin_acc_y', 'lin_acc_z',
)

# 39 columns. use_UWgps = True.
COLUMNS_PINGER = (
    'Year', 'Month', 'Day', 'Hour', 'Minute', 'Second', 'MicroSecond',
    'relative_x', 'relative_y', 'relative_psi',
    'corrected_pinger_x', 'corrected_pinger_y',
    'gps_latitude', 'gps_longitude',
    'pinger_latitude', 'pinger_longitude',
    'right_thr_in', 'left_thr_in', 'actuation_state',
    'aco_x', 'aco_y', 'aco_z',
    'ant_x', 'ant_y', 'ant_z',
    'lat', 'lon', 'dep',
    'filaco_x', 'filaco_y', 'filaco_z',
    'roll', 'pitch',
    'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
    'lin_acc_x', 'lin_acc_y', 'lin_acc_z',
)

# The two names that differ between the layouts, for code that fills the
# shared leading block once instead of branching twice. Index 0 is the
# world-frame target pair, index 1 the target's WGS84 pair.
TARGET_COLUMNS_NO_PINGER = (('target_x', 'target_y'),
                            ('target_latitude', 'target_longitude'))
TARGET_COLUMNS_PINGER = (('corrected_pinger_x', 'corrected_pinger_y'),
                         ('pinger_latitude', 'pinger_longitude'))


def columns_for(use_UWgps: bool) -> list:
    """
    Return the column list for the layout `use_UWgps` selects.

    Input  : use_UWgps -- bool, the launch parameter of the same name.
    Output : a NEW list[str] (27 or 39 entries). A fresh list every call, so
             the caller may hand it to pandas and the module constants above
             can never be mutated through it.
    """
    return list(COLUMNS_PINGER if use_UWgps else COLUMNS_NO_PINGER)


def target_columns_for(use_UWgps: bool) -> tuple:
    """
    Return the two target column-name pairs the layout uses.

    Input  : use_UWgps -- bool, the launch parameter of the same name.
    Output : ((world_x, world_y), (latitude, longitude)) -- the names under
             which the controller's target is stored in this layout. Lets one
             row-assembly path fill the target block without knowing which
             layout it is writing.
    """
    return TARGET_COLUMNS_PINGER if use_UWgps else TARGET_COLUMNS_NO_PINGER
