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

from pathlib import Path

from launch_ros.actions.node import ExecuteProcess, Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable


def generate_launch_description():
    mkdir_maps = ExecuteProcess(
        cmd=[
            FindExecutable(name='mkdir'),
            ' -p ',
            str(Path.home())+'/maps'
        ],
        shell=True
    )
    map_saver_cli = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver_cli',
        output='screen',
        arguments=['-f', str(Path.home())+'/maps/vmegarover_samplemap'],
        parameters=[{'save_map_timeout': 10000.0}])

    delay_map_saver_cli_after_mkdir_maps = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mkdir_maps,
            on_exit=[map_saver_cli]
        )
    )
    return LaunchDescription([
        mkdir_maps,
        delay_map_saver_cli_after_mkdir_maps
    ])
