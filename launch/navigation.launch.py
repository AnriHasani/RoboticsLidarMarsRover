import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_project')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    ekf_params = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    # 1. Robot Localization (EKF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}]
    )

    # 2. Nav2 Nodes - Delayed to allow SLAM (started in sim.launch) to publish initial map
    nav_nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', 
             parameters=[nav2_params, {'use_sim_time': True}],
             remappings=[('cmd_vel', 'rover/cmd_vel')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', parameters=[nav2_params, {'use_sim_time': True}]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', parameters=[nav2_params, {'use_sim_time': True}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', parameters=[nav2_params, {'use_sim_time': True}]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', parameters=[nav2_params, {'use_sim_time': True}]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', parameters=[nav2_params, {'use_sim_time': True}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[nav2_params, {'use_sim_time': True}]
        )
    ]

    nav2_stack = TimerAction(period=20.0, actions=nav_nodes)

    return LaunchDescription([
        ekf_node,
        nav2_stack
    ])
