from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import LifecycleNode

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # joy_params = os.path.join(get_package_share_directory('cps_rmp220_support'),'config','joystick.yaml')
    # namespace = "/rmp"
    # joy_node = Node(
    #         package='joy',
    #         executable='joy_node',
    #         parameters=[joy_params, {'use_sim_time': use_sim_time}],
    #         namespace = namespace
    #      )

    # teleop_node = Node(
    #         package='rmp220_teleop',
    #         executable='rmp220_teleop',
    #         name='rmp220_teleop',
    #         parameters=[joy_params, {'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel', 'cmd_vel_joy')],
    #         namespace = namespace
    #      )
    
    # twist_mux_params = os.path.join(get_package_share_directory('cps_rmp220_support'),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params, {'use_sim_time': False}],
    #         #remappings=[('cmd_vel_joy','/rmp/cmd_vel_joy')],
    #         namespace = namespace
    #     )

    # Using the more 'ROS' way: joy node publishes at 50Hz defined in the config and joy-teleop
    # node publishes the joy messages only when enable button is pressed.

    joy_params = os.path.join(get_package_share_directory('cps_rmp220_support'),'config','joystick.yaml')
    namespace = "/rmp"
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            #namespace = namespace
         )
    
    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_joy')],
            #namespace = namespace
         )
    
    twist_mux_params = os.path.join(get_package_share_directory('cps_rmp220_support'),'config','twist_mux.yaml')
    twist_mux_node = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/cmd_vel_mux')]
        )
    
    rmp_teleop_node = Node(
            package='rmp220_teleop',
            executable='rmp220_teleop',
            name='rmp220_teleop',
            remappings=[('cmd_vel', 'cmd_vel_joy')],
        )   

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
        #rmp_teleop_node,
        twist_mux_node
    ])