# BlueBoat-Control

v1.0 - September 14th - 2026

The **platform control stack** for a [BlueRobotics BlueBoat](https://bluerobotics.com/store/boat/blueboat/blueboat/)
— a small twin-hull uncrewed surface vessel (USV). This repository is what makes the boat
move: the MAVROS/ArduPilot bridge, the thruster driver, trajectory generation, and three
interchangeable controllers (**MPC**, **PID**, **LoS**). It also carries the Gazebo model of
the boat, so the identical ROS interface runs in simulation and on the water, and a desktop
app for reading back a recorded mission.

Three ROS 2 packages plus one standalone app:

| | What it is |
|---|---|
| `blueboat_control` | Every node, controller, trajectory and launch file |
| `blueboat_description` | URDF/xacro model, meshes, Gazebo world and the spawn/bridge launch chain |
| `blueboat_interfaces` | `RequestPath.srv`, `OmniscanProfile.msg`, `ProcessedSSSPing.msg` |
| `log_reviewer/` | PySide6 desktop app for reading a recorded mission (not a ROS package) |

`blueboat_interfaces` is defined **here** and consumed by the side-scan-sonar modules, so this
repository has to be built before them:
[BlueBoat-SSS](https://github.com/Kyyrron/BlueBoat-SSS) ·
[BlueBoat-SSS-Sim](https://github.com/Kyyrron/BlueBoat-SSS-Sim) ·
[BlueBoat-MCS](https://github.com/Kyyrron/BlueBoat-MCS).

> This package started from [BlueROV2](https://github.com/CentraleNantesROV/bluerov2) and the
> hull's hydrodynamic properties are still the BlueROV2 ones.

---

## Dependencies

**System**

| | Version | Install |
|---|---|---|
| ROS 2 | Jazzy | [docs.ros.org](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) |
| Gazebo | Harmonic (LTS) | [gazebosim.org](https://gazebosim.org/docs/latest/ros_installation/) |

**ROS packages** — all `apt install ros-${ROS_DISTRO}-<name>`:
[`xacro`](https://github.com/ros/xacro/tree/ros2) ·
[`simple-launch`](https://github.com/oKermorgant/simple_launch) ·
[`slider-publisher`](https://github.com/oKermorgant/slider_publisher) ·
[`mavros`](https://github.com/mavlink/mavros) ·
[`urdfdom-py`](https://github.com/ros/urdf_parser_py)

From source, into the same workspace `src/`:
[`pose_to_tf`](https://github.com/oKermorgant/pose_to_tf) (Gazebo ground truth) ·
[`auv_control`](https://github.com/CentraleNantesROV/auv_control) (basic control laws)

**Python** — `requirements.txt`. **[acados](https://docs.acados.org/index.html)** needs two
extra pieces `pip` does not install; see step 5 below. It is only required for
`controller_type:='MPC'`, but `master_control` imports it unconditionally, so it must be
present for any controller.

**Real robot** — every node of this repository, MAVROS included, runs on the operator laptop
(`~/ros2_ws`) and reaches the boat through the BlueBoat Base Station WiFi. The boat carries only
its hardware and its own firmware: the [BlueOS](https://bluerobotics.com/learn/blueboat-software-setup/)
/ ArduPilot autopilot, which MAVROS reaches at `192.168.2.2` (`fcu_url`), the sonars, GPS/compass
and the Water Linked USBL. High-level supervision is
[QGroundControl](https://s3.amazonaws.com/downloads.bluerobotics.com/QGC/latest/QGroundControl.AppImage) (runnable through `./QGroundControl.AppImage`).

---

## Installation

From a fresh Ubuntu 24.04.

```bash
# 1. ROS 2 Jazzy and Gazebo Harmonic -- follow the two links above, then:
sudo apt install ros-jazzy-xacro ros-jazzy-simple-launch \
                 ros-jazzy-slider-publisher ros-jazzy-mavros ros-jazzy-urdfdom-py

# 2. Workspace and sources
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Kyyrron/BlueBoat-Control.git
git clone https://github.com/oKermorgant/pose_to_tf.git
git clone https://github.com/CentraleNantesROV/auv_control.git

# 3. Python dependencies (a venv is recommended -- acados, casadi and PySide6 are heavy)
python3 -m venv ~/ros2_ws/.venv && source ~/ros2_ws/.venv/bin/activate
pip install -r ~/ros2_ws/src/BlueBoat-Control/requirements.txt
```

**4. acados C library.** `pip` installs only the Python interface. Build acados itself: `cd .venv/src/acados-template/` and do [these commands](https://docs.acados.org/installation/index.html#installation-via-cmake)



**5. Build and source.**

```bash
cd ~/ros2_ws && colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**6. Check it works.**

```bash
ros2 launch blueboat_control Sim_launch.py controller_type:='PID' trajectory:='circle' # Tests PID controller
ros2 launch blueboat_control Sim_launch.py controller_type:='MPC' trajectory:='circle' # Tests MPC controller and acados installation
```

Gazebo should open with the boat driving a 4 m circle. The first MPC launch additionally
compiles the solver (about a minute) and caches it in `~/.ros/blueboat_control/mpc`; delete
that directory to force a rebuild.

---

## Features

* **Three interchangeable controllers** — MPC (acados), PID and lookahead LoS — behind one
  ROS interface and one `controller_type:=` argument.
* **Eight built-in trajectories** plus designer-authored YAML missions, all served by one
  parameter-agnostic `/path_request` service.
* **Progress-driven reference.** The virtual target walks the path only as fast as the boat
  keeps up, so it can never run away; a speed profile that varies along the path is followed
  with no extra machinery.
* **GPS-anchored deferred missions.** A `from_yaml` trajectory file is watched, so the boat
  station-keeps until the [Mission Control Station](https://github.com/Kyyrron/BlueBoat-MCS) deploys the anchored mission.
* **Gazebo simulation on the identical ROS interface** — same topics, types, node names and
  parameters, so anything downstream runs unmodified in both.
* **Safety** — a motor gate that holds neutral rather than going silent, a latching E-STOP, a
  thruster-command watchdog at both ends, and a servo-mapping restore on shutdown.
* **Write-once run logging** — a position CSV and a controller `.npy` per run, plus an
  automatic post-mission report PNG.
* **Log reviewer** — trim, zoom, replay and export a recorded mission over satellite imagery.

---

## Usage

Two launch files, sharing most arguments.

```bash
# Simulation
ros2 launch blueboat_control Sim_launch.py
ros2 launch blueboat_control Sim_launch.py controller_type:='MPC' trajectory:='kin_square'

# Real robot
ros2 launch blueboat_control BlueBoat_launch.py
ros2 launch blueboat_control BlueBoat_launch.py enable_motors:=True controller_type:='PID' note:='testing_gains'

# Publishing inputs to the real robot (see ### Terminal commands)
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: default" 
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: override" 
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: move 3 3 2"
```

### Launch arguments

| Argument | Where | Meaning |
|---|---|---|
| `controller_type` | both | `'MPC'`, `'PID'` or `'LoS'`. Empty in a real-robot run (`BlueBoat_launch.py`) starts no controller. |
| `trajectory` | both | The reference to follow. Full list in `blueboat_control/src/_custom_libraries/path_generation.py`; a designer mission is `from_yaml:/abs/path.yaml`. |
| `data_dir` | both | Root for the run artifacts. Empty resolves automatically — see below. |
| `note` | both | Tag added to the position-log file name. |
| `spawn_yaw` | sim | Boat spawn heading, radians ENU (0 = East). |
| `robot_file` | sim | `thrusters_ur` (2 thrusters, default) or `thrusters_uvr` (3, not functional). |
| `enable_motors` | real | **Defaults False.** No thrust-bearing signal reaches the motors unless this is True. |
| `use_pinger` | real | With a Water Linked underwater GPS, `PID` and `LoS` follow an acoustic pinger instead of a trajectory. |
| `fcu_url` | real | MAVROS ↔ autopilot endpoint. Defaults to `udp://:14550@192.168.2.2:14550`. |

Every controller gain is also a declared ROS parameter, so tuning costs a launch argument
rather than a rebuild — `ros2 param list /blueboat/master_control`, and see `FIELD_TUNING.md`.

### Terminal commands

```bash
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: <value>"
```

| `<value>` | Effect |
|---|---|
| `enable` | Opens the motor gate. Nothing reaches the thrusters until this is called, and it is the only thing that clears a `stop`. |
| `stop` | Zeroes thrust, closes the gate, disarms — and **latches** until `enable`. |
| `override` | Disables the default thruster mapping so input can be sent straight to the motors. Makes Xbox-controller control impossible. |
| `default` | Restores the default mapping. **Do this before closing the terminal.** MCS App does it automatically when stopping a mission, but this is the first thing to check if QGroundControl cannot connect to the robot. |
| `move <left> <right> <seconds>` | Applies the two thrusts for that duration. |
| `arm` / `disarm` | Arms / disarms the thrusters (used with the Xbox controller). |

### Where run data is written

Under `<root>/data/Robot_data/{date}-{note}-poslog.csv` for the position/target/pinger log (primary log file) and
`<root>/data/{controller}_data/{date}-…npy` for the controller log (secondary, not usefull for basic usage of the USV). 

When a run ends, the CSV, its `-origin.yaml` sidecar and a report PNG
are filed into `Robot_data/<csv stem>/`.

### Log reviewer

```bash
python3 log_reviewer/run.py # then "Open log…"
```

A standalone desktop window — no ROS, no sourced workspace. Open a poslog CSV and:

* **Timeline** — two handles over mission time; every panel and every number recomputes live
  as you drag.
* **Track** — zoom, pan, and satellite imagery from the Mission Control Station's tile cache.
* **Replay** — plays the selection back at ×1 to ×8.
* **Editable text** — every title and description is what the exported picture carries.
* **Export log** — writes the trimmed run to `~/ros2_ws/data/Processed_Robot_data/<name>/`.

Full detail in `log_reviewer/README.md`.

### Further reading

| | |
|---|---|
| `controller_comparison/Controller_Report_BlueBoat.pdf` | Field-measured comparison of the three controllers |
| `FIELD_TUNING.md` | Field-experiment references to quickly access the controllers' parameters. |
| `blueboat_control/src/TRAJECTORY_SYSTEM.md` | The trajectory library, τ and the governor, one tick end to end - usefull for AI agents |


---

## Warnings and tips

* **`enable_motors` defaults to False**
* **Publish `default` before killing a launch.** Leaving the boat in `override` disables the
  Xbox controller (QGroundControl cannot access the robot).
* **Recorded runs are primary field data.** Never overwrite or regenerate a poslog CSV, its
  sidecar or a controller `.npy`.
* **Only one `/path_request` server may run.** A second mission launch left up will corrupt
  the reference of the first; since 2026-09-03 the second `path_generation` refuses to start.
* **Simulation is not the real boat.** The hull uses the BlueROV2 drag table, which caps the
  simulated speed at about 0.78 m/s — author simulated missions at ≤ 0.45 m/s. The MPC's own
  plant model is a different fit again, so a simulated MPC result is not a
  solver-against-its-own-model result.
* **`fsin` runs at 0.5 m/s**, five times the speed it was originally authored at. Runs
  recorded either side of that change are not comparable; every other shape is untouched.
* **Manual-target and pinger thrust columns are not calibrated Newtons** — that branch writes
  a kinematic command onto the thrust topic. They are comparable within a run, not across
  laws. Path following is unaffected.

---

## Authors

BERTRAND Killian, Kyushu Institute of Technology, 2026.
Yannick NOE

Released under the MIT License — see `LICENSE`.
