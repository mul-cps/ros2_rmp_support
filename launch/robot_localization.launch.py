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
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("cps_rmp220_support"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "-c", "/controller_manager"],
    )

    joystick_spawner = Node(
        package="joy",
        executable="joy_node"
    )

    teleop_spawner = Node(
        package="rmp220_teleop",
        executable="rmp220_teleop"
    )

    cam_node = Node(
        package="ros2_cam_openCV",
        executable="cam_node"
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

    lidar_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lsx10.yaml')         
    lidar_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[lidar_dir],
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters = [ PathJoinSubstitution(
                [
                    FindPackageShare("cps_rmp220_support"),
                    "config",
                    "ekf.yaml"
                ]
            ) ,
            #{'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
    )

    return LaunchDescription([
        #control_node,
        #robot_state_pub_node,
        #joint_state_broadcaster_spawner,
        #robot_controller_spawner,
        #joystick_spawner,
        #teleop_spawner,
        #cam_node,
        #lidar_node,
        #mapper_node,
        robot_localization_node
    ])