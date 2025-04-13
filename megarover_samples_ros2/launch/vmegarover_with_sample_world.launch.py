from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='false',
        choices=['true', 'false'],
        description='Set to "false" to run headless.')
    declare_world_fname = DeclareLaunchArgument(
        'world_fname', default_value='vmegarover_sample',
        description='gazebo world name (no extension)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world_fname = LaunchConfiguration('world_fname')

    launch_file_dir = PathJoinSubstitution([FindPackageShare('megarover_samples_ros2'), 'launch'])

    # setup ignition gazebo
    ignition_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, 'utils', 'ignition_gazebo.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'world_fname': world_fname
        }.items(),
    )
    # setup robot_description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, 'utils', 'robot_description.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # depth_image_proc
    depth_image_proc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, 'utils', 'depth_image_proc.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        declare_world_fname,

        ignition_gazebo_launch,

        robot_description_launch,
        depth_image_proc_launch,
    ])
