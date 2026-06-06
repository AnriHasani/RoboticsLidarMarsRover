import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_project')
    world = os.path.join(pkg_dir, 'worlds', 'mars_world.sdf')
    models_dir = os.path.join(pkg_dir, 'models')
    sdf_file = os.path.join(models_dir, 'rover', 'model.sdf')
    
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    env_vars = {
        'GZ_SIM_RESOURCE_PATH': models_dir + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    }

    # 1. Gazebo Simulation
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        additional_env=env_vars,
    )

    # 2. ROS-GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/rover/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/rover/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    # 3. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 4. Odom to TF
    odom_to_tf = Node(
        package='my_robot_project',
        executable='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. SLAM Toolbox (Standalone, Async)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'transform_publish_period': 0.05,
            'map_update_interval': 1.0,
            'max_laser_range': 12.0,
        }]
    )

    # 6. Kalman Filter Node
    kalman_filter = Node(
        package='my_robot_project',
        executable='kalman_filter_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 7. Integrated Navigator (Wait for SLAM pose)
    navigator = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='my_robot_project',
                executable='integrated_navigator',
                name='integrated_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=env_vars['GZ_SIM_RESOURCE_PATH']),
        gazebo,
        bridge,
        robot_state_publisher,
        odom_to_tf,
        slam_toolbox,
        kalman_filter,
        navigator
    ])
