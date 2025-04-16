# Quadruped Robot Simulation and Inverse Kinematics

This repository contains ROS 2 packages for simulating and controlling a quadruped robot, including inverse kinematics, trajectory generation, and hardware integration.

## 🧭 Repository Structure

| Branch           | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| `main`           | Simulation of a **single leg** of the quadruped robot.                      |
| `full_quad`      | Complete simulation of the **full quadruped robot**.                        |
| `quad_hardware`  | **Hardware integration** of the quadruped robot using `ros2_control` on devices like Jetson Nano or Raspberry Pi. |

## 📦 Packages

1. **quad_description** – URDF, Gazebo, and RViz setup for the robot.  
2. **ik** – C++ implementation of analytical inverse kinematics for a leg.

## 🚀 Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/DJIITBH/Quadruped_ROS2.git
cd ~/ros2_ws
colcon build --packages-select quad_description ik
source install/setup.bash
```
for main branch :

```bash
# Launch Gazebo and ros2_control for single leg
ros2 launch quadruped_description gazebo.launch.py

# Run custom IK service
ros2 run ik ik

# Run trajectory planning and execution
ros2 run quadruped_description traj_bezier

# Visualize in RViz with joint GUI
ros2 launch quadruped_description display.launch.py
```
For full_quad branch :
```bash
git checkout full_quad
colcon build
source install/setup.bash
```
```bash
ros2 launch quadruped_description gazebo.launch.py
ros2 run ik ik
ros2 run quadruped_description traj_bezier
ros2 launch quadruped_description display.launch.py
```
🤖 Hardware Integration (quad_hardware branch)
This branch deploys the robot on actual hardware (e.g., Jetson Nano, Raspberry Pi).

```bash
# Launch hardware interface (simulation time disabled)
ros2 launch quadruped_description hardware.launch.py

# Run trajectory planning and execution
ros2 run quadruped_description traj_bezier

# Launch LiDAR data publisher
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py

# Launch joystick control
ros2 launch quadruped_description joystick.launch.py
```
## 📺 Demonstration Video

Watch the robot in action:  
[YouTube: Quadruped ROS2](https://www.youtube.com/watch?v=Hp6pkhH9xcw)

## 📄 Research Paper

The analytical inverse‐kinematics derivation and controller design are detailed here:  
[Drive – Full Paper PDF](https://drive.google.com/file/d/1Jtqgm3kopphQJQr6SnIF_63w5bkKK_db/view?usp=sharing)



