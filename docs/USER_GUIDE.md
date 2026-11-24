# BlueBoat-Control — user guide

What you can do with the control stack, in detail. For installation, see the
[README](../README.md). Every command below assumes a terminal prepared with:

```bash
cd ~/ros2_ws && source env.sh
```

## 1. The three launches

| Launch | Use it for |
|---|---|
| `Sim_launch.py` | The boat in an empty Gazebo world, with a controller |
| `BlueBoat_launch.py` | The real boat, through the autopilot |
| `Replay_launch.py` | Replaying a recorded real run in Gazebo |

To run the boat inside a generated harbour world with a simulated sonar, use the simulator's
`full_mission_launch.py` (BlueBoat-SSS-Sim) instead. It starts this stack for you.

### Simulation

```bash
ros2 launch blueboat_control Sim_launch.py                                   # MPC, station keeping
ros2 launch blueboat_control Sim_launch.py controller_type:='MPC' trajectory:='kin_square'
ros2 launch blueboat_control Sim_launch.py controller_type:='LoS' trajectory:='from_yaml:/abs/path/mission.yaml'
```

### Real boat

```bash
ros2 launch blueboat_control BlueBoat_launch.py                              # connect only, no controller
ros2 launch blueboat_control BlueBoat_launch.py enable_motors:=True controller_type:='PID' note:='testing_gains'
```

In normal use the Mission Control Station (MCS) starts this launch for you, in "run session"
mode: the boat connects once, then each run is started and stopped from the station.

### Replay a recorded run in Gazebo

```bash
ros2 launch blueboat_control Replay_launch.py bag:=$HOME/ros2_ws/data/Robot_data/<run folder>
ros2 launch blueboat_control Replay_launch.py bag:=<...> world_dir:=$HOME/worlds/<path>/<world> rate:=2.0
```

The boat follows the recording exactly: position, roll, pitch and heading. No controller and no
sea act on it. Pause or change the speed while it plays:

```bash
ros2 topic pub --once /replay/command std_msgs/msg/String '{data: "{\"cmd\": \"pause\"}"}'
ros2 topic pub --once /replay/command std_msgs/msg/String '{data: "{\"cmd\": \"resume\"}"}'
ros2 topic pub --once /replay/command std_msgs/msg/String '{data: "{\"cmd\": \"rate\", \"rate\": 4}"}'
```

The replay starts no sonar. To see sonar pictures of the replayed run, start the simulated sonar
from the Ground Control Station (BlueBoat-SSS), which does it for you.

## 2. Operator commands

Send a command to the running stack:

```bash
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: <command>"
```

| Command | Effect |
|---|---|
| `enable` | Opens the motor gate. Also the only way to clear a `stop`. |
| `disable` | Closes the motor gate. |
| `stop` | Emergency stop: zero thrust, gate closed, autopilot disarmed. **Stays latched until `enable`.** |
| `override` | Sends thrust straight to the motors. The hand (Xbox) controller no longer works. |
| `default` | Restores the normal motor mapping. **Send it before closing a launch.** |
| `move <left> <right> <seconds>` | Applies these two thrusts for that time (only without a controller). |
| `arm` / `disarm` / `rearm` | Arms or disarms the autopilot; `rearm` retries the arming sequence. |
| `manual` | Puts the autopilot in MANUAL mode. |

## 3. Launch arguments

| Argument | Launch | Default | Meaning |
|---|---|---|---|
| `controller_type` | both | `MPC` (simulation), empty (real) | `MPC`, `PID` or `LoS`. Empty on the real boat starts no controller. |
| `trajectory` | both | `station_keeping` | `station_keeping`, `circle`, `straight_line`, `sin`, `fsin`, `square`, `kin_square`, `seabed_scanning`, or a designer path `from_yaml:/abs/path.yaml`. |
| `path_source` | both | `mission` | `mission` follows `trajectory`. `replanner`: the boat holds still until a program sends a path on `/blueboat/update_path`. |
| `data_dir` | both | empty | Where run logs go. Empty means `$BLUEBOAT_DATA_DIR`, else the workspace (`~/ros2_ws`). |
| `note` | both | `sim` (simulation), empty (real) | A tag added to the log file name. |
| `enable_motors` | real | `False` | **No thrust reaches the motors unless True.** |
| `fcu_url` | real | `udp://:14550@192.168.2.2:14550` | The link to the autopilot. |
| `use_pinger` | real | `False` | PID and LoS follow a Water Linked acoustic pinger instead of a path. |
| `session_mode` | real | `False` | Run session (the station passes true): connect and wait; each run is started and stopped on `/blueboat/run_command` and gets its own logs. Needs a `controller_type`. |
| `record_bag` | real | `True` | In a run session, record a ros2 bag of every topic except the sonar. |
| `spawn_yaw` | simulation | `0.0` | Start heading in radians, east-north-up convention (0 = East). |
| `sim_config` | simulation | `field_equivalent` | `field_equivalent`: the boat model identified on field logs, with the real boat's gains and position timing. `legacy`: the older model, only to reproduce old simulations. |
| `plant`, `odom_profile`, `odom_seed` | simulation | empty, empty, `0` | Override one piece of `sim_config` (see `FIELD_TUNING.md`). |
| `tuning_file` | simulation | empty | The gains flown on a given field day (`blueboat_control/config/field_runs/`). |
| `vertical_model` | simulation | `standalone` | `sea_node` only when a sea-state node runs beside it (the simulator's launch does this). |
| `robot_file` | simulation | `thrusters_ur` | Two thrusters. `thrusters_uvr` (three) does not work. |

`Replay_launch.py` takes `bag` (required: a run folder or its bag folder), `world_dir` (empty =
the stock world), `rate`, `heave`, `heave_gate`, `start_paused`, `gui` and `placement` (`gps`
or `origin`).

Every controller gain is a ROS parameter: `ros2 param list /blueboat/master_control`. What each
one does: [FIELD_TUNING.md](../FIELD_TUNING.md).

## 4. Where the logs go

Under `<data_dir>/data/` (by default `~/ros2_ws/data/`):

| File | What it is |
|---|---|
| `Robot_data/<date>-<note>-poslog.csv` | Position, target and thrust: the main log |
| `<controller>_data/<date>-....npy` | Controller internals (not needed for normal use) |

When a run ends, its log is moved into its own folder `Robot_data/<log name>/`, together with:
the world-frame origin (`-origin.yaml`), a report picture (`.png`), the path it followed
(`-path.yaml`, for a designer path) and, in a run session, the run's ros2 bag (`-bag/`).

The log has a description row and a unit row before the column names. Read it with
`pandas.read_csv(path, header=2)`. To redraw a report:

```bash
ros2 run blueboat_control poslog_report.py <csv>
```

Recorded runs are field data: never overwrite or edit them.

## 5. The log reviewer

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Control
python3 log_reviewer/run.py            # then "Open log..."
```

Open a position log, trim it on a timeline, replay the track over satellite imagery and export
the trimmed run to `~/ros2_ws/data/Processed_Robot_data/`. No ROS needed. Details:
[log_reviewer/README.md](../log_reviewer/README.md).

## 6. Known limits

- The thrust columns of the manual-target and pinger modes are not calibrated newtons. Compare
  them within one run only.
- The live path update is tested in Gazebo only. Its first water test follows
  [PATH_UPDATE_REAL_WATER_TEST.md](PATH_UPDATE_REAL_WATER_TEST.md).
- Simulated MPC turns less and simulated LoS turns more than the real boat. Simulated results are
  comparisons, not field numbers.

## 7. Folder layout

```
BlueBoat-Control/
├── blueboat_control/
│   ├── launch/            Sim_launch.py, BlueBoat_launch.py, Replay_launch.py
│   ├── config/field_runs/ gains flown on each field day (see its README)
│   └── src/               the nodes, the controllers (MPC/, PID/) and helper libraries
├── blueboat_description/  the boat model for Gazebo (URDF/xacro, meshes, world)
├── blueboat_interfaces/   the shared message and service types
├── log_reviewer/          the desktop log review app
├── controller_comparison/ the field comparison of the three controllers (PDF report)
├── docs/                  this guide, the development guide, the live path update, the parameter register
├── test/                  tests and the controller harness
├── FIELD_TUNING.md        every tuning knob
└── requirements.txt
```
