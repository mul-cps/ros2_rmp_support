import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.actions import GroupAction

# Image transport plugins
plugins = ["image_transport/raw", "image_transport/compressed"]

# Compression settings
compr_jpeg = {"format": "jpeg", "jpeg_quality": 95}
compr_png = {"format": "png", "png_level": 1}

def generate_launch_description():
    # Declare camera-related arguments
    args = [
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='false'),
        DeclareLaunchArgument('enumerate_net_device', default_value='false'),
    ]

    # Dynamic parameter bindings
    camera_name = LaunchConfiguration('camera_name')
    enable_colored_pc = LaunchConfiguration('enable_colored_point_cloud')
    enumerate_net_device = LaunchConfiguration('enumerate_net_device')

    femto = ComposableNode(
        package="orbbec_camera",
        plugin="orbbec_camera::OBCameraNodeDriver",
        name=camera_name,
        namespace="",
        remappings=[
            ("/camera/gyro_accel/sample", "/camera/imu"),
        ],
        extra_arguments=[
            {"use_intra_process_comms": False},
        ],
        parameters=[
            {"camera_name": camera_name},
            {"use_hardware_time": True},
            {"enable_heartbeat": True},
            {"enumerate_net_device": enumerate_net_device},

            # compression settings
            {"color.image_raw.enable_pub_plugins": plugins},
            {"depth.image_raw.enable_pub_plugins": plugins},
            {"ir.image_raw.enable_pub_plugins": plugins},
            {".color.image_raw.compressed": compr_jpeg},
            {".depth.image_raw.compressed": compr_png},
            {".ir.image_raw.compressed": compr_png},

            # color
            {"enable_color": True},
            {"color_width": 1280},
            {"color_height": 720},
            {"color_fps": 30},
            {"color_format": "MJPG"},
            {"enable_color_auto_exposure": True},
            {"enable_color_auto_white_balance": True},

            # depth
            {"enable_depth": True},
            {"depth_width": 640},
            {"depth_height": 576},
            {"depth_fps": 30},
            {"depth_format": "Y16"},
            {"depth_registration": True},
            {"align_mode": "SW"},
            {"enable_frame_sync": True},
            {"enable_depth_scale": True},

            # IR
            {"enable_ir": True},
            {"ir_width": 640},
            {"ir_height": 576},
            {"ir_fps": 30},
            {"ir_format": "Y16"},
            {"enable_ir_auto_exposure": True},

            # IMU
            {"enable_gyro": True},
            {"accel_rate": "200hz"},
            {"accel_range": "4g"},
            {"enable_accel": True},
            {"gyro_rate": "200hz"},
            {"gyro_range": "1000dps"},
            {"enable_sync_output_accel_gyro": True},

            # point cloud
            {"enable_colored_point_cloud": enable_colored_pc},
            {"ordered_pc": True},
        ],
    )

    rgbd = ComposableNode(
        package="depth_image_proc",
        plugin="depth_image_proc::PointCloudXyzrgbNode",
        name="rgbd",
        namespace="",
        extra_arguments=[
            {"use_intra_process_comms": True},
        ],
        remappings=[
            ("depth_registered/image_rect", "/camera/depth/image_raw"),
            ("rgb/image_rect_color", "/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("points", "/camera/points"),
        ],
    )

    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            femto,
            rgbd,
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        args + [
            GroupAction([
                PushRosNamespace(camera_name),
                container
            ])
        ]
    )
