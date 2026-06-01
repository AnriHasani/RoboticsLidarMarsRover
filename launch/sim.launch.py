import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, ExecuteProcess, SetEnvironmentVariable, LogInfo
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle.lifecycle_node_matchers import matches_node_name
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_project')
    world = os.path.join(pkg_dir, 'worlds', 'mars_world.sdf')
    models_dir = os.path.join(pkg_dir, 'models')
    sdf_file = os.path.join(models_dir, 'rover', 'model.sdf')
    
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    env_vars = {
        'GZ_SIM_RESOURCE_PATH': models_dir + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        '__NV_PRIME_RENDER_OFFLOAD': '1',
        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
    }

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world],
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
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    odom_to_tf = Node(
        package='my_robot_project',
        executable='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    kalman_filter = Node(
        package='my_robot_project',
        executable='kalman_filter_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
            'ceres_loss_function': 'None',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'debug_logging': True,
            'throttle_scans': 1,
            'transform_publish_period': 0.02,
            'map_update_interval': 2.0,
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_time_interval': 0.25,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.,
            'stack_size_to_use': 1000000,
            'enable_interactive_mode': False,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.1,
            'minimum_travel_heading': 0.1,
            'scan_buffer_size': 10,
            'scan_buffer_maximum_scan_distance': 10.0,
            'link_match_maximum_distance': 2.0,
            'link_scan_maximum_distance': 1.5,
            'loop_search_maximum_distance': 3.0,
            'do_loop_closing': True,
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance_coarse': 3.0,
            'loop_match_minimum_response_coarse': 0.35,
            'loop_match_minimum_response_fine': 0.45,
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            'correlation_search_space_smear_deviation': 0.1,
            'loop_search_space_dimension': 8.0,
            'loop_search_space_resolution': 0.05,
            'loop_search_space_smear_deviation': 0.03,
            'distance_variance_penalty': 0.5,
            'angle_variance_penalty': 1.0,
            'fine_search_angle_offset': 0.00349,
            'coarse_search_angle_offset': 0.349,
            'coarse_angle_resolution': 0.0349,
            'minimum_angle_penalty': 0.9,
            'minimum_distance_penalty': 0.5,
            'use_response_expansion': True,
        }]
    )

    ld = LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=env_vars['GZ_SIM_RESOURCE_PATH']),
        gazebo,
        bridge,
        robot_state_publisher,
        odom_to_tf,
        kalman_filter,
        slam_toolbox,
    ])

    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=slam_toolbox,
                on_start=[
                    LogInfo(msg='>>> Configuring slam_toolbox'),
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_node_name('slam_toolbox'),
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        )
                    )
                ]
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_toolbox,
                goal_state='inactive',
                entities=[
                    LogInfo(msg='>>> Activating slam_toolbox'),
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_node_name('slam_toolbox'),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ]
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_toolbox,
                goal_state='active',
                entities=[
                    LogInfo(msg='>>> slam_toolbox is ACTIVE'),
                ]
            )
        )
    )

    return ld
