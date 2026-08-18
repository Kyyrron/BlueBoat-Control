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
- [acados solver](https://docs.acados.org/index.html) used for MPC computation 
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
