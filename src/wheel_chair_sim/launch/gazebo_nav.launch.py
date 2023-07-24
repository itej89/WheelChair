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
    


    # Define launch file arguments
    arg_sim = launch_ros.actions.SetParameter(name='use_sim_time', value=True)

    sim_description_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wheel_chair_sim'),
                'launch',
                'display_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'rvizconfig': PathJoinSubstitution([
                FindPackageShare('navigation'),
                'rviz',
                'nav_gazebo.rviz'
            ])
        }.items()
    )
    
    slam_dir = get_package_share_directory('slam')

    slam_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                slam_dir + '/launch/slam_gazebo.launch.py'))

    

    navigation_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('navigation'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': PathJoinSubstitution([
                FindPackageShare('navigation'),
                'config',
                'nav2_gazebo_params.yaml'
            ])
        }.items()
    )
  

    #Make launch description
    ld = launch.LaunchDescription()


    #Add nodes
    ld.add_action(arg_sim)
    ld.add_action(sim_description_launch)
    ld.add_action(slam_launch)
    ld.add_action(navigation_launch)

    return ld