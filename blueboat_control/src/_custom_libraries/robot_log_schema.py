#!/usr/bin/env python3

r"""
Column schema of the position CSV written by robot_interface.py in real-robot runs
and by simulation_interface.py in Gazebo.

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

    use_UWgps = False  ->  COLUMNS_NO_PINGER  (27 columns)  <- always, in sim
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

The same file in simulation
---------------------------
`simulation_interface.log_timer_callback` writes this format too, at the same
rate and in the COLUMNS_NO_PINGER layout only -- there is no Water Linked UGPS
in Gazebo -- so a simulated run and a field run are read by one reader. Three
columns mean slightly less there, and say so at the point they are written:

  * the seven date columns carry the ROS clock, which is SIM time, so a
    simulated log reads 1970-01-01. Every reader takes differences from the
    first row; using sim time is what keeps derived speeds right when the
    real-time factor is not 1.
  * `lin_acc_*` is the odometry twist differentiated, not an IMU reading, so
    it carries NO gravity component (Gazebo publishes no /mavros/imu/data
    under Sim_launch).
  * `actuation_state` only ever takes 1 and 3. States 0 and 2 are hardware
    conditions -- an enable_motors gate and an autopilot mode -- and cannot
    occur in simulation.

The GPS pairs fill in only when something publishes
/mavros/global_position/global (the Mission Control Station's bridge, or the
simulator's mavros_shim_node); under a plain Sim_launch they stay at (0, 0),
which is this project's "no fix" everywhere else.

Column groups, in the order they appear
---------------------------------------
Both layouts, columns 1-19:
  Year..MicroSecond      (7)  wall-clock stamp of the row, local time (SIM
                              time, hence 1970, in a simulated log).
                              MicroSecond is FULL microseconds (0-999999) in
                              BOTH layouts. It used to be milliseconds in the
                              no-pinger layout and microseconds in the other.
  relative_x/y/psi       (3)  robot pose in the local-ENU world frame -- origin
                              latched at robot_interface's first odom callback,
                              so (0,0) is wherever the boat was at launch and
                              NOT a fixed geographic frame, while psi is
                              ABSOLUTE ENU yaw (0 = East, CCW+) since
                              2026-08-31. In simulation the frame is already
                              that: the Gazebo world origin, nothing re-zeroed.
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
                                   no commanded thrust reached the water. Since
                                   2026-09-04 robot_interface holds the RC
                                   channels at neutral 1500/1500 in this state
                                   rather than going silent, so that nothing
                                   else can drive them; neutral is not thrust,
                                   so the meaning of this value is unchanged and
                                   no earlier CSV is reinterpreted
                                1  enabled AND param_mode == 'override' --
                                   live, thrust reaching the motors
                                2  enabled but not in override -- ArduPilot
                                   ignores the RC override stream
                                3  loss-of-reference watchdog tripped --
                                   thrust forced to zero by the interface node
                              Without this column a run with the motor gate
                              off is byte-indistinguishable from a live one,
                              and a watchdog trip is indistinguishable from a
                              genuine zero command (the watchdog writes [0,0]
                              into the same field).

Both layouts, trailing block:
  roll, pitch            (2)  from /mavros/imu/data's quaternion, or from the
                              odometry quaternion in simulation. Yaw is not
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

Legend rows -- 2026-10-08
-------------------------
A CSV started on or after this revision opens with TWO legend rows above
the column names, written once at file creation with the header (never
added at run end -- the file is write-once, and a killed run keeps it):

    row 1   short description, 1-4 words     Robot X (odom frame), ...
    row 2   unit                             m, ...
    row 3   column names (as before)         relative_x, ...
    row 4+  data

Every CSV recorded before has no legend: column names on row 1, data from
row 2. Both are valid and NEITHER IS REWRITTEN to look like the other. A
reader tells them apart with `split_header` -- the header is the first row
holding `relative_x`, which is never a description or a unit -- and never by
date or file name. `LEGEND` below is the single source of both rows; no
description or unit may contain a comma (the nodes write rows with
`','.join`).

Consumers
---------
* poslog_report.read_poslog (the archived PNG, its CLI) and, through it, the
  log reviewer -- both formats, via split_header.
* offline analysis notebooks -- index by column NAME, do not tolerate a
  renamed column. A new-format file loads with `pd.read_csv(path, header=2)`
  (descriptions and units are then lost; read them with header=None, nrows=2).
"""

# Actuation-state encoding, for readers that would rather not hard-code ints.
ACT_MOTORS_DISABLED = 0   # enable_motors False -- no thrust; channels held neutral
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


# Rows above the column names in a CSV written since 2026-10-08.
LEGEND_ROWS = 2

# column -> (description, unit). Covers every column of both layouts.
# Descriptions are 1-4 words; units are ASCII so any spreadsheet opens them.
# Each one names what the writers ACTUALLY put in the column, in the frame
# they put it in -- not what the frame is assumed to be:
#   odom frame  the pose frame of the odometry the interface node reads:
#               /mavros/local_position/odom translated to the pose at the first
#               odom message (robot_interface), or the Gazebo world as published
#               on /blueboat/odom (simulation_interface). Origin = launch point.
#               The code never rotates it, so 'east'/'north' would be a claim
#               about MAVROS / Gazebo, not about this log.
#   (derived)   computed from odom-frame x/y through the origin fix
#               (cf.enu_to_gps), not measured by any receiver.
#   UGPS        straight from the Water Linked API, in the frame the device
#               reports it (locator relative to its topside receivers); `dep` is
#               the raw acoustic Z again, not an independent depth sensor.
#   Body        the vehicle frame: raw /mavros/imu/data on the boat (the
#               acceleration includes gravity), the odometry twist and its time
#               derivative in simulation (no gravity).
LEGEND = {
    'Year': ('Timestamp year', 'year'),
    'Month': ('Timestamp month', 'month'),
    'Day': ('Timestamp day', 'day'),
    'Hour': ('Timestamp hour', 'h'),
    'Minute': ('Timestamp minute', 'min'),
    'Second': ('Timestamp second', 's'),
    'MicroSecond': ('Timestamp microsecond', 'us'),
    'relative_x': ('Robot X (odom frame)', 'm'),
    'relative_y': ('Robot Y (odom frame)', 'm'),
    'relative_psi': ('Robot yaw (odom frame)', 'rad'),
    'target_x': ('Target X (odom frame)', 'm'),
    'target_y': ('Target Y (odom frame)', 'm'),
    'corrected_pinger_x': ('Pinger X (odom frame)', 'm'),
    'corrected_pinger_y': ('Pinger Y (odom frame)', 'm'),
    'gps_latitude': ('Robot GPS latitude', 'deg'),
    'gps_longitude': ('Robot GPS longitude', 'deg'),
    'target_latitude': ('Target latitude (derived)', 'deg'),
    'target_longitude': ('Target longitude (derived)', 'deg'),
    'pinger_latitude': ('Pinger latitude (derived)', 'deg'),
    'pinger_longitude': ('Pinger longitude (derived)', 'deg'),
    'right_thr_in': ('Right thrust command', 'N'),
    'left_thr_in': ('Left thrust command', 'N'),
    'actuation_state': ('Actuation state', 'code 0-3'),
    'aco_x': ('Locator X (UGPS raw)', 'm'),
    'aco_y': ('Locator Y (UGPS raw)', 'm'),
    'aco_z': ('Locator Z (UGPS raw)', 'm'),
    'ant_x': ('Antenna X (UGPS config)', 'm'),
    'ant_y': ('Antenna Y (UGPS config)', 'm'),
    'ant_z': ('Antenna Z (UGPS config)', 'm'),
    'lat': ('Locator latitude (UGPS)', 'deg'),
    'lon': ('Locator longitude (UGPS)', 'deg'),
    'dep': ('Locator depth (UGPS raw)', 'm'),
    'filaco_x': ('Locator X (UGPS filtered)', 'm'),
    'filaco_y': ('Locator Y (UGPS filtered)', 'm'),
    'filaco_z': ('Locator Z (UGPS filtered)', 'm'),
    'roll': ('Robot roll angle', 'rad'),
    'pitch': ('Robot pitch angle', 'rad'),
    'ang_vel_x': ('Body angular rate X', 'rad/s'),
    'ang_vel_y': ('Body angular rate Y', 'rad/s'),
    'ang_vel_z': ('Body angular rate Z', 'rad/s'),
    'lin_acc_x': ('Body acceleration X', 'm/s^2'),
    'lin_acc_y': ('Body acceleration Y', 'm/s^2'),
    'lin_acc_z': ('Body acceleration Z', 'm/s^2'),
}


def legend_rows(columns) -> tuple:
    """
    Return the two legend rows for a header.

    Input  : columns -- the column names, in file order.
    Output : (descriptions, units), two lists aligned with `columns`. A name
             this schema does not know (an older layout's `quat_x`, say) gets
             empty strings, so the rows still line up with the header.
    """
    return ([LEGEND.get(c, ('', ''))[0] for c in columns],
            [LEGEND.get(c, ('', ''))[1] for c in columns])


def split_header(rows) -> tuple:
    """
    Locate the column-name row at the top of a poslog, in either format.

    Input  : rows -- the first parsed CSV rows (lists of strings); at least
             LEGEND_ROWS + 1 of them when the file has that many.
    Output : (legend, header_index). header_index is 0 for a CSV recorded
             before 2026-10-08 and LEGEND_ROWS for one written since; legend is
             (descriptions, units) as found in the file, or None for the old
             format. header_index is None when no row among the first
             LEGEND_ROWS + 1 holds `relative_x` -- the file is not a poslog.
    """
    for index, row in enumerate(list(rows)[:LEGEND_ROWS + 1]):
        if 'relative_x' in row:
            if index == 0:
                return None, 0
            if index == LEGEND_ROWS:
                return (list(rows[0]), list(rows[1])), index
            return None, None
    return None, None


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
