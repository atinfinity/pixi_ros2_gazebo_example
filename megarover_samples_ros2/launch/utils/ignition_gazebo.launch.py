# Copyright (C) 2025 atinfinity
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

from os import pathsep

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        choices=['true', 'false'],
        description='Set to "false" to run headless.')
    declare_world_fname = DeclareLaunchArgument(
        'world_fname', default_value='',
        description='gazebo world name (no extension)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world_fname = LaunchConfiguration('world_fname')

    pkg_megarover_samples_ros2 = FindPackageShare('megarover_samples_ros2')

    set_env_gazebo_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),
            pathsep,
            PathJoinSubstitution([pkg_megarover_samples_ros2, 'worlds'])]
    )

    gz_args = [world_fname, '.sdf', ' ',
               # log level : 1
               '-v 1', ' ',
               # autostart
               '-r', ' ',
               # headless
               __headless_rendering(gui)]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': gz_args
        }.items(),
    )

    
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
                '-world', 'vmegarover_sample',
                '-name', 'vmegarover',
                '-x', '0',
                '-y', '0',
                '-z', '0.3',
                '-topic', 'robot_description',
        ]
    )

    base_topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='base_topic_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            # IGN -> ROS
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",

            # ROS -> IGN
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        ],
    )

    scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='scan_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            # IGN -> ROS
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        ]
    )
    image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='image_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            # IGN -> ROS
            "/front_camera_sensor/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/front_camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        remappings=[
            ("/front_camera_sensor/depth_image", "/front_camera_sensor/depth/image_raw"),
            ("/front_camera_sensor/image", "/front_camera_sensor/image_raw"),
        ]
    )
    rgb_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='rgb_camera_info_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            # IGN -> ROS
            "/front_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
    )
    depth_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='depth_camera_info_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            # IGN -> ROS
            "/front_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/front_camera_sensor/camera_info", "/front_camera_sensor/depth/camera_info")
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        declare_world_fname,

        set_env_gazebo_resource,

        gazebo,
        spawn_entity,

        # bridges
        base_topic_bridge,
        scan_bridge,
        image_bridge,
        rgb_camera_info_bridge,
        depth_camera_info_bridge
    ])

def __headless_rendering(gui):
    cmd = ['"" if "true" == "', gui, '" else "--headless-rendering -s"']
    py_cmd = PythonExpression(cmd)
    return py_cmd