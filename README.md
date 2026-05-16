# RoboticsLidarMarsRover
Project for Elements of Robotics and Automatisation [CIS230.e] made by Anri Hasani and Dora Demiri

## Table of Contents

- [Distrobox Setup](#distrobox-setup)
- [Install ROS 2 Jazzy + Gazebo](#install-ros-2-jazzy--gazebo)
- [Auto-source ROS 2](#auto-source-ros-2)
- [Fix Gazebo Transport](#fix-gazebo-transport-required-for-distrobox)
- [Create the Robot Model](#create-the-robot-model-sdf-file)
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

**If your host OS uses fish**

Fish lacks native ROS 2 / colcon support, so switch to bash for ROS work:

```bash
# From fish, enter bash:
bash
```

Or add this to `~/.config/fish/config.fish` to auto-switch:

```fish
if set -q DISTROBOX_ENTER_PATH; or test -f /.dockerenv
    exec bash
end
```

## Fix Gazebo Transport (Required for Distrobox)

Add this to `~/.bashrc`:

```bash
export GZ_IP=127.0.0.1
```

Gazebo runs inside a container (Docker/Podman) where its transport discovery (UDP multicast) doesn't work. Forcing `GZ_IP=127.0.0.1` makes GUI and server communicate properly via localhost.

## Create the Robot Model (SDF file)

The robot model is defined at `models/my_robot.sdf`. It describes the robot's body, wheels, joints, and the DiffDrive plugin that listens for `/cmd_vel`.

```bash
mkdir -p ~/ros2_ws/src/my_robot_project/models
```

Create `models/my_robot.sdf` with a `<model name="my_robot">` containing a body link, two wheel links with revolute joints, and a `gz-sim-diff-drive-system` plugin with `<topic>/model/my_robot/cmd_vel</topic>`.

When your robot changes: Edit `models/my_robot.sdf` (add links, change sizes, add sensors) and rebuild with `colcon build --packages-select my_robot_project`.

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_project
```

## Running

Two terminals, both inside the distrobox (`distrobox enter mars-rover-ros-two`, then `bash`).

**Terminal 1 — Launch Gazebo + Spawn Robot + Bridge**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_robot_project sim.launch.py
```

This single command:
- Starts Gazebo with an empty world (clock running via `-r`)
- Waits 8 seconds for Gazebo to fully load
- Spawns `my_robot` from the model SDF
- Starts the bridge: `/model/my_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist`

**Terminal 2 — Robot node**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_project my_node
```

In Gazebo, select `my_robot` in the Entity tree → right-click → Follow to track.

## Verify

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /model/my_robot/cmd_vel
```

## Editing Code

Edit `my_robot_project/my_first_node.py`, then stop (`Ctrl+C`) and re-run the node. No rebuild needed for Python changes.

Rebuild if you modify `setup.py`, `package.xml`, `models/my_robot.sdf`, or add new files:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_project
```
