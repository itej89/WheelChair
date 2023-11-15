from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    tf_cam_base_link = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0.0762", "0", "0.2667", "0", "0", "0", "base_link", "laser_frame"])

    return LaunchDescription([
        tf_cam_base_link,
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
