import os, time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('my_robot_project')
    world = os.path.join(pkg, 'worlds', 'mars_world.sdf')
    model = os.path.join(pkg, 'models', 'my_robot.sdf')

    # Let Gazebo find mars_heightmap.png via model://
    models_dir = os.path.join(pkg, 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] = \
        models_dir + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    # 1. Start Gazebo with the Mars world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen'
    )

    # 2. Spawn robot after Gazebo has loaded (8 s delay, same as your original)
    spawn = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/mars/create',
                    '--reqtype',  'gz.msgs.EntityFactory',
                    '--reptype',  'gz.msgs.Boolean',
                    '--timeout',  '5000',
                    '--req',
                    f'sdf_filename: "{model}", pose: {{position: {{x:0, y:0, z:3}}}}'
                ],
                output='screen'
            )
        ]
    )

    # 3. ROS ↔ Gazebo bridge (same topic as your existing setup)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/my_robot/cmd_vel'
            '@geometry_msgs/msg/Twist'
            ']gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([gazebo, spawn, bridge])