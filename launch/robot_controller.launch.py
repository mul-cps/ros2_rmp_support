from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    control_node = Node(
            package='segwayrmp',
            executable='SmartCar',
            name='SmartCar',
            remappings=[('cmd_vel','cmd_vel_out'), ('/odom','odom'), ('/imu','imu'), ('/bms_fb','bms_fb'), ('/joint_states','joint_states'), ('/tf','tf'), ('/tf_static','tf_static')],
            namespace = "/rmp"
         )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        control_node     
    ])