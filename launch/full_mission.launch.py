import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_project')

    # 1. Base Simulation (Gazebo + Bridge + TF)
    # This includes the sim.launch.py which is Terminal 1 in your README
    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'sim.launch.py')
        )
    )

    # 2. Kalman Filter Node (Terminal 2 in your README)
    kalman_filter = Node(
        package='my_robot_project',
        executable='kalman_filter_node',
        name='kalman_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. PID Navigator Node (Terminal 3 in your README)
    # Added a small delay (5s) to ensure the simulation is up before it starts navigating
    pid_navigator = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='my_robot_project',
                executable='pid_navigator',
                name='pid_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        base_sim,
        kalman_filter,
        pid_navigator
    ])
