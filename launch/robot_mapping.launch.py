# Copyright 2022 Factor Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("cps_rmp220_support"),
                    "description",
                    "robot.urdf.xacro"
                ]
            ),
        ]
    )

    use_sim_time = False
    slam_params_file = PathJoinSubstitution(
                [
                    FindPackageShare("cps_rmp220_support"),
                    "config",
                    "mapper_params_online_async.yaml"
                ]
            )
    mapper_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name='slam_toolbox_node',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    return LaunchDescription([
        mapper_node
    ])