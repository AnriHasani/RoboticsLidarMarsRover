import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('my_robot_project')
    world = os.path.join(pkg, 'worlds', 'mars_world.sdf')

    models_dir = os.path.join(pkg, 'models')
    env_vars = {
        'GZ_IP': os.environ.get('GZ_IP', '127.0.0.1'),
        'QT_QPA_PLATFORM': os.environ.get('QT_QPA_PLATFORM', 'xcb'),
        'GDK_BACKEND': os.environ.get('GDK_BACKEND', 'x11'),
        'GZ_SIM_RESOURCE_PATH': models_dir + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        # NVIDIA GPU Offloading
        '__NV_PRIME_RENDER_OFFLOAD': '1',
        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
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
            '/rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/rover/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/rover/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    env_actions = [
        SetEnvironmentVariable(name=name, value=value)
        for name, value in env_vars.items()
    ]

    return LaunchDescription(env_actions + [gazebo, bridge])
