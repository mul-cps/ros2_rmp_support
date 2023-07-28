import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='sllidar_ros2',
            executable='view_sllidar_launch',
            output='screen',
            parameters=[{
                'serial_port': 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_3453995662b3af4f81f4a69eba5f3f29-if00-port0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'
            }]
        )
    ])
