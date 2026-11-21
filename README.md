# BlueBoat-Control

v1.1 - November 20th - 2026

The control stack of a [BlueRobotics BlueBoat](https://bluerobotics.com/store/boat/blueboat/blueboat/),
a small twin-hull uncrewed surface vessel (USV). It makes the boat move: the MAVROS/ArduPilot
bridge, the thruster driver, trajectory generation and three interchangeable controllers
(**MPC**, **PID**, **LoS**). It also carries the Gazebo model of the boat, so the same ROS
interface runs in simulation and on the water, and a desktop app to review recorded runs.

**Where it fits.** It is one module of the BlueBoat side-scan sonar project. The sonar,
perception and planning modules use its topics and run unchanged in simulation and on the boat.
The Mission Control Station ([BlueBoat-MCS](https://github.com/Kyyrron/BlueBoat-MCS)) launches
and supervises it. A planner (BlueBoat-Replanner) can change the path while the boat moves,
through the live path update service `/blueboat/update_path`.

| Part | What it is |
|---|---|
| `blueboat_control` | All nodes, controllers, trajectories and launch files (ROS 2 package) |
| `blueboat_description` | URDF/xacro model, meshes, Gazebo world and spawn launch (ROS 2 package) |
| `blueboat_interfaces` | `RequestPath.srv`, `UpdatePath.srv`, `OmniscanProfile.msg`, `ProcessedSSSPing.msg` (ROS 2 package) |
| `log_reviewer/` | Desktop app to review a recorded run (not a ROS package) |

`blueboat_interfaces` is also used by the sonar modules
([BlueBoat-SSS](https://github.com/Kyyrron/BlueBoat-SSS),
[BlueBoat-SSS-Sim](https://github.com/Kyyrron/BlueBoat-SSS-Sim)), so build this repository first.

**Where the software runs.** Every node, MAVROS included, runs on the operator laptop
(`~/ros2_ws`). The laptop reaches the boat over the BlueBoat Base Station WiFi. The boat carries
only its hardware and firmware: the [BlueOS](https://bluerobotics.com/learn/blueboat-software-setup/)
/ ArduPilot autopilot at `192.168.2.2`, the sonars, GPS/compass and the Water Linked USBL.

---

## Requirements

| | Version |
|---|---|
| Ubuntu | 24.04 |
| ROS 2 | [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) |
| Gazebo | [Harmonic](https://gazebosim.org/docs/latest/ros_installation/), with `ros_gz` |
| Python packages | `requirements.txt` (acados, casadi, numpy, scipy, matplotlib, PySide6, ...) |

ROS packages from apt: `xacro`, `simple-launch`, `slider-publisher`, `mavros`, `urdfdom-py`.
From source, in the same workspace: [`pose_to_tf`](https://github.com/oKermorgant/pose_to_tf).

**acados is needed for every controller**, not only MPC: `master_control` imports it at start-up.

Optional: [QGroundControl](https://s3.amazonaws.com/downloads.bluerobotics.com/QGC/latest/QGroundControl.AppImage)
for supervision of the real boat.

---

## Installation

```bash
# 1. ROS 2 Jazzy and Gazebo Harmonic: follow the two links above, then
sudo apt install ros-jazzy-xacro ros-jazzy-simple-launch ros-jazzy-slider-publisher \
                 ros-jazzy-mavros ros-jazzy-urdfdom-py

# 2. Sources (skip the first clone if you use the BlueBoat-SideScanSonar superproject)
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Kyyrron/BlueBoat-Control.git
git clone https://github.com/oKermorgant/pose_to_tf.git

# 3. Python dependencies, in a venv
python3 -m venv ~/ros2_ws/.venv && source ~/ros2_ws/.venv/bin/activate
pip install -r ~/ros2_ws/src/BlueBoat-Control/requirements.txt
```

**4. acados C library.** `pip` installs only the Python interface. Build acados itself from
`~/ros2_ws/.venv/src/acados-template/` with
[the CMake instructions](https://docs.acados.org/installation/index.html#installation-via-cmake),
then export `ACADOS_SOURCE_DIR` to that folder. acados also needs its Tera renderer
(`<acados>/bin/t_renderer`). If it is missing, `master_control` stops with a message that gives
the exact download command.

**5. Build.** Always from the workspace root.

```bash
cd ~/ros2_ws && colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source ~/ros2_ws/.venv/bin/activate     # the controllers need acados and casadi
```

**6. Check.**

```bash
ros2 launch blueboat_control Sim_launch.py controller_type:='PID' trajectory:='circle'
ros2 launch blueboat_control Sim_launch.py controller_type:='MPC' trajectory:='circle'
```

Gazebo opens and the boat drives a 4 m radius circle. The first MPC launch compiles the solver
(about a minute) and caches it in `~/.ros/blueboat_control/mpc`. Delete that folder to force a
rebuild.

---

## Running

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

In normal use the Mission Control Station starts this launch for you (with `session_mode:=true`).

### Replay a recorded run in Gazebo

```bash
ros2 launch blueboat_control Replay_launch.py bag:=$HOME/ros2_ws/data/Robot_data/<run folder>
ros2 launch blueboat_control Replay_launch.py bag:=<...> world_dir:=$HOME/worlds/<path>/<world> rate:=2.0
```

The boat follows the recording exactly (position, roll, pitch, yaw). No controller and no sea
state act on it. Pause or change speed while it plays:

```bash
ros2 topic pub --once /replay/command std_msgs/msg/String '{data: "{\"cmd\": \"pause\"}"}'
ros2 topic pub --once /replay/command std_msgs/msg/String '{data: "{\"cmd\": \"resume\"}"}'
ros2 topic pub --once /replay/command std_msgs/msg/String '{data: "{\"cmd\": \"rate\", \"rate\": 4}"}'
```

### Operator commands

```bash
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: <command>"
```

| Command | Effect |
|---|---|
| `enable` | Opens the motor gate. Also the only way to clear a `stop`. |
| `disable` | Closes the motor gate. |
| `stop` | E-STOP: zero thrust, gate closed, disarm. **Stays latched until `enable`.** |
| `override` | Sends thrust straight to the motors. The Xbox controller no longer works. |
| `default` | Restores the normal thruster mapping. **Send it before closing a launch.** |
| `move <left> <right> <seconds>` | Applies these two thrusts for that time. |
| `arm` / `disarm` / `rearm` | Arms / disarms the autopilot; `rearm` retries the arming sequence. |
| `manual` | Puts the autopilot in MANUAL mode. |

---

## Launch arguments

| Argument | Launch | Default | Meaning |
|---|---|---|---|
| `controller_type` | both | `MPC` (sim), empty (real) | `MPC`, `PID` or `LoS`. Empty on the real boat starts no controller. |
| `trajectory` | both | `station_keeping` | `station_keeping`, `circle`, `straight_line`, `sin`, `fsin`, `square`, `kin_square`, `seabed_scanning`, or a designer mission `from_yaml:/abs/path.yaml`. |
| `path_source` | both | `mission` | `mission` follows `trajectory`. `replanner`: the boat holds until a client loads a path on `/blueboat/update_path`. |
| `data_dir` | both | empty | Root for run logs. Empty = `$BLUEBOAT_DATA_DIR`, else the workspace root (`~/ros2_ws`). |
| `note` | both | `sim` (sim), empty (real) | Tag added to the log file name. |
| `enable_motors` | real | `False` | **No thrust reaches the motors unless True.** |
| `fcu_url` | real | `udp://:14550@192.168.2.2:14550` | MAVROS to autopilot link. |
| `use_pinger` | real | `False` | PID and LoS follow a Water Linked acoustic pinger instead of a trajectory. |
| `session_mode` | real | `False` | Run session (MCS passes true): connect and wait; each run is started and stopped on `/blueboat/run_command` and gets its own logs. Needs a `controller_type`. |
| `record_bag` | real | `True` | In a run session, record a ros2 bag of every topic except the side-scan sonar. |
| `spawn_yaw` | sim | `0.0` | Spawn heading in radians, ENU (0 = East). |
| `sim_config` | sim | `field_equivalent` | `field_equivalent`: the identified BlueBoat plant with the real boat's gains and odometry timing. `legacy`: the old BlueROV2-based plant, only to reproduce old simulations. |
| `plant`, `odom_profile`, `odom_seed` | sim | empty, empty, `0` | Override one piece of `sim_config`. See `FIELD_TUNING.md`. |
| `tuning_file` | sim | empty | Gains flown on a given field day: `blueboat_control/config/field_runs/`. |
| `vertical_model` | sim | `standalone` | `sea_node` only when a sea-state node runs beside it (the simulator's launch does this). |
| `robot_file` | sim | `thrusters_ur` | 2 thrusters. `thrusters_uvr` (3) does not work. |

`Replay_launch.py` takes `bag` (required: a run folder or its `-bag` folder), `world_dir`
(empty = the stock world), `rate`, `heave`, `heave_gate`, `start_paused`, `gui` and `placement`
(`gps` or `origin`).

Every controller gain is a ROS parameter: `ros2 param list /blueboat/master_control`. What each
gain does: `FIELD_TUNING.md`.

---

## Where the logs go

Under `<data_dir>/data/` (by default `~/ros2_ws/data/`):

| File | What it is |
|---|---|
| `Robot_data/<date>-<note>-poslog.csv` | Position, target and thrust log: the main log |
| `<controller>_data/<date>-....npy` | Controller internals (not needed for normal use) |

When a run ends, its CSV is moved into its own folder `Robot_data/<csv stem>/`, together with:
`-origin.yaml` (the world-frame origin), a report `.png`, the mission it followed (`-path.yaml`,
for a `from_yaml` run) and, in a run session, the run's ros2 bag (`-bag/`, mcap).

The CSV has a description row and a unit row before the column names:
`pandas.read_csv(path, header=2)`. To redo a report: `ros2 run blueboat_control poslog_report.py <csv>`.

Recorded runs are field data: never overwrite or edit them.

---

## Log reviewer

```bash
python3 log_reviewer/run.py            # from this folder; then "Open log..."
```

Open a poslog CSV, trim it on a timeline, replay the track over satellite imagery and export the
trimmed run to `~/ros2_ws/data/Processed_Robot_data/`. No ROS needed. Details:
`log_reviewer/README.md`.

---

## Tests

```bash
python3 test/test_path_update.py                 # live path update logic, no ROS
python3 test/test_path_update_log.py             # path update logs, no ROS
python3 test/test_bag_replay_track.py            # bag replay track
python3 test/test_yaw_wrap.py                    # heading at +-180 deg (needs ROS sourced + venv)
python3 test/mc_harness.py --compare HEAD --param gain_profile:=sim   # controllers unchanged vs HEAD
QT_QPA_PLATFORM=offscreen python3 log_reviewer/smoke_test.py
```

Run them from this folder, with the workspace sourced. `mc_harness.py` needs the venv with acados.

---

## Warnings

* **`enable_motors` is False by default.** Nothing moves without it.
* **Send `default` before stopping a launch.** A boat left in `override` ignores the Xbox
  controller, and QGroundControl may not connect. MCS does this for you when it stops a mission.
* **QGroundControl and MAVROS both use UDP port 14550.** If QGC runs first, MAVROS cannot connect.
  Close QGC or change `fcu_url`.
* **Only one mission launch at a time.** A second `path_generation` refuses to start.
* **Live path update (`path_source:=replanner`) is tested in Gazebo only, not on water.** Protocol
  for the first water test: `docs/PATH_UPDATE_REAL_WATER_TEST.md`.
* **Simulation is not the real boat.** The default plant is identified on field logs, but
  simulated MPC turns less and LoS turns more than the real boat. Results from simulation are
  comparisons, not field numbers (superproject `SIM_VALIDITY_CONDITIONS.md`).
* **Thrust columns of the manual-target and pinger modes are not calibrated newtons.** Compare
  them within one run only.

---

## Directory layout

```
BlueBoat-Control/
├── blueboat_control/
│   ├── launch/            Sim_launch.py, BlueBoat_launch.py, Replay_launch.py
│   ├── config/field_runs/ gains flown on each field day (see its README)
│   └── src/               nodes: master_control, robot_interaction/, simulation_interface,
│                          bag_replay, MPC/, PID/, _custom_libraries/ (paths, logs, reports)
├── blueboat_description/  URDF/xacro, meshes, Gazebo world and spawn launch
├── blueboat_interfaces/   srv/ and msg/ definitions
├── log_reviewer/          desktop log review app
├── controller_comparison/ field comparison of the three controllers (PDF report)
├── docs/                  live path update, parameter register, references
├── test/                  tests and the controller harness
├── FIELD_TUNING.md        every tuning knob
└── requirements.txt
```

## Further reading

| Document | Content |
|---|---|
| `FIELD_TUNING.md` | Every controller parameter, real and simulated defaults, symptom to knob |
| `controller_comparison/Controller_Report_BlueBoat.pdf` | Field comparison of MPC, PID and LoS |
| `blueboat_control/src/TRAJECTORY_SYSTEM.md` | How the reference is built: trajectories and the progress governor |
| `docs/PATH_UPDATE_INTERFACE.md` | The live path update contract, for client authors |
| `docs/PATH_UPDATE_DESIGN.md` | Why the live path update works the way it does |
| `docs/PARAMETER_REGISTER.md` | Every number of the simulated plant and where it comes from |
| `test/path_update_demo.py` | A minimal live path update client (example) |

---

## Authors

BERTRAND Killian, Kyushu Institute of Technology, 2026.
Yannick NOE

Released under the MIT License — see `LICENSE`.
