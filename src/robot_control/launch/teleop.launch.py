# Copyright 2026 Juan José Martínez Medina
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    xterm_arg = DeclareLaunchArgument(
        'xterm',
        default_value='true',
        description='Abrir el teleop en una ventana xterm (necesita xterm)'
    )

    teleop_xterm = Node(
        package='robot_control',
        executable='teleop_wasd',
        name='teleop_wasd',
        prefix='xterm -e',
        condition=IfCondition(LaunchConfiguration('xterm'))
    )

    teleop_here = Node(
        package='robot_control',
        executable='teleop_wasd',
        name='teleop_wasd',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('xterm'))
    )

    return LaunchDescription([
        xterm_arg,
        teleop_xterm,
        teleop_here
    ])
