import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    depth_image_proc_container = ComposableNodeContainer(
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        name='depth_image_proc_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                remappings=[('image_rect', '/front_camera_sensor/depth/image_raw'),
                            ('camera_info', '/front_camera_sensor/depth/camera_info'),
                            ('points', '/front_camera_sensor/points')],
                parameters=[{'use_sim_time': use_sim_time}]
            ),
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(depth_image_proc_container)

    return ld