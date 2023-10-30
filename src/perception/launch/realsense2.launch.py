from launch import LaunchDescription
from launch import launch_description_sources
from launch import actions
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
 
    madgwick_imu_filter = Node(
        name = "camera3",
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        remappings=[
            ("/imu/data_raw", "/camera/imu"),
            ("/imu/data", "/madgwick/imu")
        ],
        parameters=[
            {"fixed_frame": "odom"},
            {"use_mag": False},
            {"publish_tf": True}
        ]
    )

    realsense2_dir = get_package_share_directory('realsense2_camera')
    realsense2_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                realsense2_dir + '/launch/rs_launch.py'),
            launch_arguments={
                'unite_imu_method': "2",
                'align_depth.enable': 'True',
                'enable_gyro': 'True',
                'enable_accel': 'True',
                'depth_module.profile': '1280x720x30',
                'rgb_camera.profile': '1280x720x30'
                }.items()
        )



    rgbd_sync=Node(
        namespace='rtabmap',
        package='rtabmap_sync', 
        executable='rgbd_sync', 
        output="screen",
        parameters=[{
            "approx_sync": True,
            "approx_sync_max_interval": 0.0,
            "queue_size": 10,
            "qos": 1,
            "qos_camera_info": 1,
            "depth_scale": 1.0}],
        remappings=[
            ("rgb/image", "/camera/color/image_raw"),
            ("depth/image", "/camera/aligned_depth_to_color/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("rgbd_image", "/rgbd_image")]
        )

    
    tf_cam_base_link = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0.0762", "0", "0.2667", "0", "0", "0", "base_link", "camera_link"])

    ld.add_action(madgwick_imu_filter)
    ld.add_action(realsense2_launch)
    ld.add_action(rgbd_sync)
    ld.add_action(tf_cam_base_link)
    return ld