# ROS 2 Control Humble + Gazebo Harmonic Demo

A minimal workspace that shows how to run a robot model under **ros2\_control** (Humble) inside **Gazebo Harmonic** using the `gz_ros2_control` integration layer.

---

## Table of Contents

1. [Features](#features)
2. [Prerequisites](#prerequisites)
3. [Quick Start](#quick-start)
4. [Step‑by‑Step Setup](#step-by-step-setup)
5. [Launching the Simulation](#launching-the-simulation)
6. [Repository Layout](#repository-layout)
7. [Troubleshooting](#troubleshooting)
8. [Resources](#resources)
9. [License](#license)

---

## Features

* ✅ **Gazebo Harmonic** physics engine out of the box.
* ✅ **ros2\_control** interfaces wired through `gz_ros2_control`.
* ✅ Example **quadruped** robot with per‑leg controllers.
* ✅ Ready‑to‑use **launch files** and `ros_gz_bridge` clock bridge.
* ✅ Works on **Ubuntu 22.04 + ROS 2 Humble**.

---

## Prerequisites

| Tool   | Version                    | Notes                                                       |
| ------ | -------------------------- | ----------------------------------------------------------- |
| Ubuntu | 22.04                      | Tested on Jammy Jellyfish                                   |
| ROS 2  | Humble Hawksbill           | [Install](https://docs.ros.org/en/humble/Installation.html) |
| Gazebo | **Harmonic** (libgz‑sim 9) | Follow instructions below                                   |
| colcon | ≥ 0.15                     | `sudo apt install python3-colcon-common-extensions`         |
| rosdep | ≥ 0.22                     | `sudo apt install python3-rosdep`                           |

> **Tip 📌** If you already have another Gazebo version installed, use containers or update the `GZ_VERSION` env var locally.

---

## Quick Start

```bash
# Clone
mkdir -p ~/gz_ros2_control_ws/src && cd ~/gz_ros2_control_ws/src

# Pull the integration layer
git clone https://github.com/ros-controls/gz_ros2_control -b humble

# Set Gazebo flavour
export GZ_VERSION=harmonic

# Install dependencies (skip the bridge & sim pkgs because we build from apt)
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y \
  --skip-keys="ros_gz_bridge ros_gz_sim"

# Build the workspace
cd ~/gz_ros2_control_ws && colcon build

# Persist environment
echo "source ~/gz_ros2_control_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Launch the demo
ros2 launch quadruped_description gazebo.launch.py
```

---

## Step‑by‑Step Setup

### 1 — Install Gazebo Harmonic & ROS Gz Packages

Follow the official guide: [https://gazebosim.org/docs/harmonic/ros\_installation/#gazebo-harmonic-with-ros-2-humble-or-rolling-use-with-caution](https://gazebosim.org/docs/harmonic/ros_installation/#gazebo-harmonic-with-ros-2-humble-or-rolling-use-with-caution)

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
#install gazebo harmonic
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
#install the ros gazebo bridge
sudo apt-get install ros-humble-ros-gzharmonic
```

### 2 — Build `gz_ros2_control` from source

See the **Quick Start** snippet or the official doc: [https://control.ros.org/humble/doc/gz\_ros2\_control](https://control.ros.org/humble/doc/gz_ros2_control)

### 3 — Update your URDF/Xacro

```xml
<!-- ros2_control block -->
<ros2_control name="GazeboSimSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
</ros2_control>

<!-- Gazebo plugin block -->
<gazebo>
  <plugin filename="gz_ros2_control-system"
          name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
    <parameters>$(find gz_ros2_control_demos)/config/cart_controller.yaml</parameters>
  </plugin>
</gazebo>
```

> Make sure the `filename` matches the library name produced in `install/lib` after building.
## The below changes have already been done, its just for understanding and learning purposes! so please dont do these again!
### 4 — Prepare the Launch File (excerpt)

```python

gz_spawn_entity = Node(
    package='ros_gz_sim', executable='create', output='screen',
    arguments=['-topic', 'robot_description', '-name', 'dog', '-allow_renaming', 'true']
)

bridge = Node(
    package='ros_gz_bridge', executable='parameter_bridge', output='screen',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
)

# … joint_broad_spawner & leg controllers omitted …

return LaunchDescription([
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': [' -r -v 1 empty.sdf']}.items()),

    RegisterEventHandler(OnProcessExit(target_action=gz_spawn_entity,
                                       on_exit=[joint_broad_spawner])),

    RegisterEventHandler(OnProcessExit(target_action=joint_broad_spawner,
        on_exit=[TimerAction(period=1.0, actions=[lb_cont_spawner, lf_cont_spawner,
                                                  rb_cont_spawner, rf_cont_spawner])])),

    bridge,
    robot_state_publisher_node,
    gz_spawn_entity,

    # Delay static TF
    TimerAction(period=2.0, actions=[static_tf_node]),

    # Optional RViz
    rviz2_node,
])
```

---

## Launching the Simulation

```bash
ros2 launch quadruped_description gazebo.launch.py
```
---

## Troubleshooting

| Symptom                                                        | Fix                                                                                                                  |
| -------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| `None of requested interfaces exist. Controller will not run.` | URDF `<joint>` names must match controller list; ensure `<ros2_control>` tags reference correct hardware interfaces. |
| `ros_gz_sim create: Service call failed`                       | Check that Gazebo has fully started; increase the delay before spawning or use `-r` flag to auto‑start simulation.   |
| Controllers never switch from *unconfigured* → *inactive*      | The **joint\_broadcaster** must be running before leg controllers; see timer chain in launch file.                   |

---

## Resources

* ros-controls **gz\_ros2\_control** docs → [https://control.ros.org/humble/doc/gz\_ros2\_control](https://control.ros.org/humble/doc/gz_ros2_control)
* Gazebo Harmonic + ROS 2 integration → [https://gazebosim.org/docs/harmonic/ros\_installation](https://gazebosim.org/docs/harmonic/ros_installation)
* Example PRs & issues → [https://github.com/ros-controls/gz\_ros2\_control/issues](https://github.com/ros-controls/gz_ros2_control/issues)

---

## License

This repository is licensed under the Apache 2.0 License. See `LICENSE` for details.
