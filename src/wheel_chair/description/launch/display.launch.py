import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    #Define Path varialbes
    pkg_share = launch_ros.substitutions.FindPackageShare(package='wheel_chair').find('wheel_chair')
    default_model_path = os.path.join(pkg_share, 'urdf/wheel_chair.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')



    # Define launch file arguments
    arg_sim = launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time')
    arg_gui = launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
                                        description='Flag to enable joint_state_publisher_gui')
    arg_model = launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Absolute path to robot urdf file')
    arg_rvizconfig = launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file')

    #Create nodes
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    #Make launch description
    ld = launch.LaunchDescription()

    #Add launch arguments
    ld.add_action(arg_sim)
    ld.add_action(arg_gui)
    ld.add_action(arg_model)
    ld.add_action(arg_rvizconfig)

    #Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld