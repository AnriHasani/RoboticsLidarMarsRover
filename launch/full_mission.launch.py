import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_project')

    # 1. Start Simulation (Gazebo + Bridge + Robot State)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'sim.launch.py'))
    )

    # 2. Start Navigation Stack (EKF + SLAM + Nav2)
    # Starts automatically with internal delays
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'navigation.launch.py'))
    )

    # 3. Start Mission Control (The Commander)
    # Delay to ensure Nav2 + SLAM are ACTIVE and map frame is available
    mission_node = TimerAction(
        period=45.0,
        actions=[
            Node(
                package='my_robot_project',
                executable='mission_control',
                name='mission_control',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        sim_launch,
        nav_launch,
        mission_node
    ])
