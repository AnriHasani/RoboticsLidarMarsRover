# RoboticsLidarMarsRover
Project for Elements of Robotics and Automatisation [CIS230.e]  made by Anri Hasani and Dora Demiri

# ROS 2 Setup Guide

## Distrobox Setup

```bash
distrobox create --image ubuntu:24.04 --name mars-rover-ros-two
distrobox enter mars-rover-ros-two

# Install ROS 2 Jazzy
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions ros-jazzy-ros-gz-bridge
```

## Auto-source ROS 2

### If your host OS uses bash

Add this to `~/.bashrc`:

```bash
if [ -n "$DISTROBOX_ENTER_PATH" ] || [ -f /.dockerenv ]; then
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
  fi
fi
```

### If your host OS uses fish

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

## Building

```bash
# Switch to bash first (if using fish)
bash
cd ~/ros2_ws
colcon build --packages-select my_robot_project
```

## Running

```bash
# Switch to bash first (if using fish)
bash
cd ~/ros2_ws
source install/setup.bash
```

### Terminal 1 — Gazebo bridge

```bash
bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /model/my_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
```

### Terminal 2 — Robot node

```bash
bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_project my_node
```

## Verify

```bash
bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /model/my_robot/cmd_vel
```

