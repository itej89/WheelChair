import os

from launch import LaunchDescription
from launch import launch_description_sources
from launch import actions
import launch_ros
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()


    pkg_share = launch_ros.substitutions.FindPackageShare(package='slam').find('slam')
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )

    rtabmap_odom=Node(
        namespace='rtabmap',
        package='rtabmap_odom', 
        executable='rgbd_odometry', 
        output="screen",
        parameters=[{
            "subscribe_rgbd": True,
            "queue_size": 10,
            "frame_id": "base_link",
            "publish_tf": False,
            "approx_sync": True,
            "wait_for_transform": 0.2,
            # "wait_imu_to_init": True,
            }],
        remappings=[
            ("imu", "/madgwick/imu"),
            ("rgbd_image", "/rgbd_image")]
        )


    rtabmap_slam=Node(
        namespace='rtabmap',
        package='rtabmap_slam', 
        executable='rtabmap', 
        output="screen",
        parameters=[{
            "subscribe_depth": False,
            "subscribe_rgbd": True,
            "queue_size": 10,
            "frame_id": "base_link",
            "publish_tf": True,
            "map_frame_id": "map",
            "odom_frame_id": "odom",
            "approx_sync": True
            }],
        remappings=[
            ("odom", "/odometry/filtered"),
            ("rgbd_image", "/rgbd_image"),
            ("grid_map", "/map"), 
            ("imu", "/madgwick/imu")
            ]
        )

    ld.add_action(robot_localization_node)
    ld.add_action(rtabmap_odom)
    ld.add_action(rtabmap_slam)
    return ld