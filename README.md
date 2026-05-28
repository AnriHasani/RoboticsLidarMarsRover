# RoboticsLidarMarsRover
Project for Elements of Robotics and Automatisation [CIS230.e] made by Anri Hasani and Dora Demiri

## Table of Contents

- [Distrobox Setup](#distrobox-setup)
- [Install ROS 2 Jazzy + Gazebo](#install-ros-2-jazzy--gazebo)
- [Auto-source ROS 2](#auto-source-ros-2)
- [Fix Gazebo Transport](#fix-gazebo-transport-required-for-distrobox)
- [Building](#building)
- [Project Structure](#project-structure)
- [Running](#running)
- [Verify](#verify)
- [Editing Code](#editing-code)

---

# ROS 2 Setup Guide

## Distrobox Setup

```bash
distrobox create --image ubuntu:24.04 --name mars-rover-ros-two
distrobox enter mars-rover-ros-two
```

## Install ROS 2 Jazzy + Gazebo

```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
```

## Auto-source ROS 2

**If your host OS uses bash**

Add this to `~/.bashrc`:

```bash
if [ -n "$DISTROBOX_ENTER_PATH" ] || [ -f /.dockerenv ]; then
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
  fi
fi
```

## Fix Gazebo Transport (Required for Distrobox)

Add this to `~/.bashrc`:

```bash
export GZ_IP=127.0.0.1
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
export GDK_BACKEND=x11
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_project
```

## Running

Three terminals, all inside the distrobox (`distrobox enter mars-rover-ros-two`, then `bash`).

**Terminal 1 — Launch Gazebo + Bridge**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_robot_project sim.launch.py
```

**Terminal 2 — Kalman Filter Node**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_project kalman_filter_node
```

**Terminal 3 — Waypoint Navigator Node**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_project navigator_node
```

*Note: Nodes automatically use simulation time, no extra flags required.*

## Verify

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /kalman_pose  # Check filtered position
ros2 topic echo /rover/cmd_vel  # Check navigation commands
```

## Editing Code

Edit `my_robot_project/kalman_filter_node.py` or `my_robot_project/waypoint_navigator.py`, then stop (`Ctrl+C`) and re-run the node. No rebuild needed for Python changes.

Rebuild if you modify `setup.py`, `package.xml`, or add new files:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_project
```
