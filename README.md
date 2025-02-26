# Quadruped Robot Simulation and Inverse Kinematics

This repository contains two ROS 2 packages:

1. **quad_description** - URDF and Gazebo setup for a quadruped robot leg.
2. **ik** - C++ implementation of inverse kinematics using analytical equations.

## Installation

Clone the repository into your ROS 2 workspace and build it:

```sh
cd ~/ros2_ws/src
git clone https://github.com/DJIITBH/Quadruped_ROS2.git
cd ~/ros2_ws
colcon build --packages-select quad_description ik
source install/setup.bash
```

## Usage

### Quad Description

#### Spawn Quadruped Robot Leg in Gazebo
```sh
ros2 launch quad_description gazebo.launch.py
```
This command launches the Gazebo environment and spawns the quadruped robot leg.

#### Visualize in RViz and Move Joints via GUI
```sh
ros2 launch quad_description display.launch.py
```
This command opens RViz to visualize the quadruped robot leg and interactively control its joints.

### Inverse Kinematics

The **ik** package contains the inverse kinematics implementation for the quadruped leg using analytical equations.


## Dependencies
Ensure that you have the required dependencies installed:

```sh
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-rviz2
```

## Contributing
Feel free to fork this repository and contribute by submitting pull requests.

