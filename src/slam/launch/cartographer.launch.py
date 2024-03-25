import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)


import launch_ros

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cartographer_prefix = get_package_share_directory('slam')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  cartographer_prefix, 'config', 'cartographer'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer.lua')
 
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    


    cartographer_config_dir = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=cartographer_config_dir,
        description='Full path to config file to load')

    configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value=configuration_basename,
        description='Name of lua file for cartographer')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    cartographer_ros = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-configuration_directory', LaunchConfiguration('cartographer_config_dir'),
                '-configuration_basename', LaunchConfiguration('configuration_basename')],
        remappings=[('/odom', '/laser/odom')])

    pose_to_odom = Node(
        package="wheel_chair",
        executable="pose_to_odom"
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_pose_to_odom_after_cartographer_ros = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=cartographer_ros,
            on_start=[pose_to_odom],
        )
    )

    pkg_share = launch_ros.substitutions.FindPackageShare(package='slam').find('slam')
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )

    return LaunchDescription([
        cartographer_config_dir,
        configuration_basename,
        use_sim_time,
        cartographer_ros,
        delay_pose_to_odom_after_cartographer_ros,
        # robot_localization_node
        # DeclareLaunchArgument(
        #     'resolution',
        #     default_value=resolution,
        #     description='Resolution of a grid cell in the published occupancy grid'),
 
        # DeclareLaunchArgument(
        #     'publish_period_sec',
        #     default_value=publish_period_sec,
        #     description='OccupancyGrid publishing period'),
 
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
        #                       'publish_period_sec': publish_period_sec}.items(),
        # ),
 
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
    ])