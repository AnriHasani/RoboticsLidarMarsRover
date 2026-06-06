# RoboticsLidarMarsRover
Project for Elements of Robotics and Automatisation [CIS230.e]

## Overview
This project implements a robust autonomous Mars Rover navigation system. It features an **Integrated Rover Navigator** that combines high-fidelity motion control with reactive obstacle avoidance.

## Architecture
1. **Gazebo Sim**: High-fidelity Martian environment simulation.
2. **Kalman Filter**: Fuses Wheel Odometry and IMU (via simulated noise) to provide precise pose estimation.
3. **Integrated Navigator**:
   - **Pure Pursuit**: Standard NASA/Industrial algorithm for smooth path following.
   - **Artificial Potential Fields (APF)**: Reactive avoidance where obstacles "push" the rover away while the goal "pulls" it.
   - **State Machine**: Handles mission phases (Navigating, Blocked, Recovering).

## Setup

### 1. Distrobox Setup (Optional)
```bash
distrobox create --image ubuntu:24.04 --name mars-rover
distrobox enter mars-rover
```

### 2. Install ROS 2 Jazzy
Follow the standard ROS 2 Jazzy installation instructions for Ubuntu 24.04.

### 3. Install Dependencies
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
```

### 4. Building
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_project
source install/setup.bash
```

## Running the Mission
You can launch the entire mission with a **single command**:

```bash
ros2 launch my_robot_project mars_rover.launch.py
```

### What happens:
- The simulation starts and spawns the rover.
- The Kalman Filter initializes its pose.
- After a 10s stabilization delay, the **Integrated Navigator** takes over.
- The rover will automatically traverse through 6 predefined waypoints while fluidly avoiding rocks and obstacles.

## Verification
```bash
ros2 topic echo /kalman_pose  # Check filtered position
ros2 topic echo /rover/cmd_vel  # Check navigation commands
```
