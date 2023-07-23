import os

from launch import LaunchDescription
from launch import launch_description_sources
from launch import actions
import launch_ros
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    
    # Define launch file arguments
    arg_sim = launch_ros.actions.SetParameter(name='use_sim_time', value=True)

    pkg_share = launch_ros.substitutions.FindPackageShare(package='slam').find('slam')

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf_gazebo.yaml')]
    )
   

    # slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # slam_toolbox_launch = actions.IncludeLaunchDescription(
    #     launch_description_sources.PythonLaunchDescriptionSource(
    #             slam_toolbox_dir + '/launch/lifelong_launch.py'))

    slam_node = launch_ros.actions.Node(
          parameters=[
            get_package_share_directory("slam") + '/config/slam_gazebo.yaml'
          ],
          package='slam_toolbox',
          executable='lifelong_slam_toolbox_node',
          name='slam_toolbox',
          output='screen'
        )


    ld.add_action(arg_sim)
    ld.add_action(robot_localization_node)
    ld.add_action(slam_node)
    return ld