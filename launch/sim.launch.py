import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('my_robot_project')
    world = os.path.join(pkg, 'worlds', 'mars_world.sdf')

    models_dir = os.path.join(pkg, 'models')
    env_vars = {
        'GZ_IP': os.environ.get('GZ_IP', '127.0.0.1'),
        'GZ_SIM_RESOURCE_PATH': models_dir + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    }

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        additional_env=env_vars,
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    env_actions = [
        SetEnvironmentVariable(name=name, value=value)
        for name, value in env_vars.items()
    ]

    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                     '-world', 'mars',
                     '-file', os.path.join(models_dir, 'my_robot.sdf'),
                     '-name', 'my_robot'],
                output='screen',
            )
        ]
    )

    return LaunchDescription(env_actions + [gazebo, bridge, spawn_robot])
