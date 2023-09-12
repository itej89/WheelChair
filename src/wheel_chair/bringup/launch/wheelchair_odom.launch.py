import os
import launch
from launch import launch_description_sources
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import launch_ros
from launch import actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    


    arg_sim = launch_ros.actions.SetParameter(name='use_sim_time', value=False)


    hw_description_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wheel_chair'),
                'launch',
                'wheelchair.launch.py'
            ])
        ]),
        launch_arguments={
            'rvizconfig': PathJoinSubstitution([
                FindPackageShare('navigation'),
                'rviz',
                'nav.rviz'
            ])
        }.items()
    )
    
    perception_dir = get_package_share_directory('perception')

    perception_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                perception_dir + '/launch/realsense2.launch.py'))

    
    slam_dir = get_package_share_directory('slam')

    slam_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                slam_dir + '/launch/slam_wheel_chair.launch.py'))


    #Make launch description
    ld = launch.LaunchDescription()

    #Add nodes
    ld.add_action(arg_sim)
    ld.add_action(hw_description_launch)
    ld.add_action(perception_launch)
    ld.add_action(slam_launch)

    return ld