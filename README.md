# BlueBoat ROS2

This repository contains the robot description and necessary launch files to describe and simulate the BlueBoat (Uncrewed Surface Vessel) with Gazebo and its hydrodynamics plugins under ROS 2.

Additionnal steps are included to make sure this can be used starting from a fresh Ubuntu install.

NOTE: This package is a modified version of the original [BlueROV2](https://github.com/CentraleNantesROV/bluerov2/tree/main), most of the physical and hydrodynamical properties are currently set on the bluerov2.

# Requirements

## ROS2
The current recommended ROS2 version is Jazzy. All the related info can be found [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## Gazebo
- The recommended gazebo version is GZ Harmonic (LTS). More info and step-by-step installation guide are found [here](https://gazebosim.org/docs/latest/ros_installation/)
- [pose_to_tf](https://github.com/oKermorgant/pose_to_tf), to get the ground truth from Gazebo if needed.

## For the description

- [Xacro](https://github.com/ros/xacro/tree/ros2) , installable through `apt install ros-${ROS_DISTRO}-xacro`
- [simple_launch](https://github.com/oKermorgant/simple_launch), installable through `apt install ros-${ROS_DISTRO}-simple-launch`

## For the control part

- [slider_publisher](https://github.com/oKermorgant/slider_publisher), installable through `apt install ros-${ROS_DISTRO}-slider-publisher`
- [auv_control](https://github.com/CentraleNantesROV/auv_control) for basic control laws
- [urdf_parser](https://github.com/ros/urdf_parser_py) intended to have the controller work with any robot description
- [acados solver](https://docs.acados.org/index.html) used for MPC computation -- see the
  prerequisites below, `pip install` alone is **not** enough

### acados prerequisites (only needed for `controller_type:='MPC'`)

`requirements.txt` installs the `acados_template` Python package from git, but that package
is only the interface. Two further pieces have to exist before an MPC solver can be built,
and neither is installed by pip:

1. **The acados C library.** Build acados itself (`libacados.so` and friends under
   `<acados>/lib`) following the [installation guide](https://docs.acados.org/installation/).
2. **The Tera template renderer.** acados generates its solver C code by rendering
   templates, and the renderer is a separate binary that must sit in `<acados>/bin`:

   ```bash
   ACADOS_DIR=~/ros2_ws/.venv/src/acados-template   # wherever acados_template resolves to
   mkdir -p "$ACADOS_DIR/bin"
   curl -fL -o "$ACADOS_DIR/bin/t_renderer" \
     https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64
   chmod +x "$ACADOS_DIR/bin/t_renderer"
   ```

Then point acados at that tree, in `~/.bashrc`:

```bash
export ACADOS_SOURCE_DIR=$HOME/ros2_ws/.venv/src/acados-template
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
```

Without `ACADOS_SOURCE_DIR` acados *guesses* the path (printing a warning) and can pick a
different acados checkout than the one the Python package came from. `LD_LIBRARY_PATH`
matters because `libacados.so` carries no rpath to `libblasfeo` / `libhpipm` /
`libqpOASES_e`; `master_control` preloads those by absolute path so a missing export is
survivable, but exporting it is still the right setup.

If `t_renderer` is missing, `master_control` refuses to start with a FATAL naming the exact
commands above. The generated and compiled solver is cached in
`$ROS_HOME/blueboat_control/mpc` (default `~/.ros/blueboat_control/mpc`), so only the first
MPC launch pays the compile; delete that directory to force a rebuild.

- [mavros](https://github.com/mavlink/mavros) 

## Real robot
This code is meant to interact with the [BlueRobotics BlueBoat](https://bluerobotics.com/store/boat/blueboat/blueboat/)

A detailled software integration tutorial can be found [here](bluerobotics.com/learn/blueboat-software-setup/)

Interaction with ROS2 uses the [blue-os ROS2 app](https://github.com/itskalvik/blueos-ros2) (can be directly installed through BlueOS app tab): 

High-level interaction is done with [QGroundControl](https://s3.amazonaws.com/downloads.bluerobotics.com/QGC/latest/QGroundControl.AppImage)

# Installation

- Clone the package and its dependencies (if from source) in your ROS 2 workspace `src` and compile with `colcon build`, make sure you are in the parent folder of `src` when compiling.

# Running 
- Make sure to source the terminal if you did not modify the bashrc file.

    `source /opt/ros/jazzy/setup.bash`

    `source install/setup.bash`

- To run a demonstration with the vehicle, you can run a Gazebo scenario, and spawn the robot with:

    `ros2 launch blueboat_control Sim_launch.py`

On a fresh install, it is likely that some python dependencies will have to be installed, proceed as such.

# Input / output

Gazebo will:

- Subscribe to /blueboat/cmd_thruster[i] and expect std_msgs/Float64 messages (thrust in Newton).
- Publish the ground truth on /blueboat/pose_gt. This pose is forwarded to /tf if pose_to_tf is used.

# High-level control
This package is meant to be easy to use, having only two launch files to either handle simulation or real robot interaction, both using the same parameters (with some exclusive to the real robot).

## Launch files

The simulation is run with:

`ros2 launch blueboat_control Sim_launch.py`

The launch file that handles every robot interaction and control can be run with:

`ros2 launch blueboat_control BlueBoat_launch.py`

## Launch parameters

- 'controller_type': choose between the available controllers. Three are available: 'MPC', 'PID', and 'LoS'
- 'trajectory': the trajectory the robot is expected to follow with a given controller. The full list of trajectories is found in blueboat_control/src/_custom_libraries/path_generation.py
- 'enable_motors' (Real robot only): both for testing and safety purposes, no signal will be sent to the motors unless this is set to True
- 'use_pinger' (Real robot only): in the case the robot is equipped with an underwater gps, the 'PID' and 'LoS' controller can be set to follow an acoustic pinger instead of a virtual one
- 'note' (Real robot only): it is possible to add a comment to the name of the log file recorded when the code is ran.
- 'fcu_url' (Real robot only): the MAVROS <-> autopilot endpoint. Defaults to `udp://:14550@192.168.2.2:14550`.
- 'data_dir': root directory for the run artifacts (the position CSV and the controller .npy). Empty by default, which resolves them automatically - see 'Where run data is written' below.

## Where run data is written

Run artifacts are written under one root, resolved at startup rather than taken from the
directory the launch was invoked in. Each node logs the file it opened, so a run is never
ambiguous about where it wrote. The root is the first of:

1. the `data_dir` launch argument, when non-empty;
2. `$BLUEBOAT_DATA_DIR`;
3. the sourced ROS workspace, i.e. the parent of the first `$COLCON_PREFIX_PATH` entry - normally `~/ros2_ws`;
4. the process working directory, as a last resort.

That gives `<root>/data/Robot_data/{date}-{note}-poslog.csv` for the position/pinger log and
`<root>/data/{controller}_data/{date}-...npy` for the controller log. An unwritable root is a
launch failure naming the path, not a silent fallback. Names are stamped to the second and are
claimed exclusively, so two runs started in the same second get `-2`, `-3`, ... rather than one
overwriting the other. Recorded runs are primary field data: never overwrite or regenerate them.

## MAVROS and QGroundControl both want port 14550

QGroundControl listens on UDP 14550 by default, and so does the default `fcu_url`. If QGC is
already running, MAVROS cannot bind and the launch fails with:

    [mavros_router]: link[1000] open failed: DeviceError:udp:bind: Address already in use

It looks intermittent because it depends on whether QGC happens to be open. Either close QGC, or
move this launch to another port:

`ros2 launch blueboat_control BlueBoat_launch.py fcu_url:='udp://:14551@192.168.2.2:14551'`

Below are example of launch commands:
`ros2 launch blueboat_control BlueBoat_launch.py enable_motors:=True controller_type:='PID' note:='testing_gains'`

`ros2 launch blueboat_control Sim_launch.py controller_type:='MPC' trajectory:='kin_square'`

## Terminal interactions
Different interactions with the real robot can be handled through a single instruction:

`ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: [value]"`

The list of available [value] is as follows:
 - 'enable': no input will be sent to the thrusters until this has been called
 - 'stop': opposite of previous command, stops the robot and disable thrusters
 - 'override': disables default thruster mapping of the robot, used to send input directly to the motors (used for various controllers and terminal control, makes control through the xbox controller impossible)
 - 'default': restores default mapping, to be used before closing the terminal
 - 'move': typical instruction follows the shape 'move [float_left] [float_right] [float_time]', it will set the float_left and float_right inputs to the thrusters and apply it for the float_time duration (seconds)
 - 'arm': arms the robot's thrusters (used when interacting with the xbox controller)
 - 'disarm': disarms the robot's thrusters

# License
blueboat package is open-sourced under the MIT License. See the LICENSE file for details.
