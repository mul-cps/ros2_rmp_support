import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = False
    config_file = PathJoinSubstitution(
                [
                    FindPackageShare("explore_lite"),
                    "config",
                    "params.yaml"
                ]
            )
    namespace = "/rmp"
    mapper_node = Node(
        package="explore_lite",
        executable="explore",
        name='explore_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        #namespace = namespace,
        #remappings=[('/scan', 'scan'), ('/map', 'map')],
    )

    return LaunchDescription([
        mapper_node
    ])