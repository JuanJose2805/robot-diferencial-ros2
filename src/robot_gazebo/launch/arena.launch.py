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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # 1. Localizar nuestros paquetes
    description_pkg = get_package_share_directory('robot_description')
    gazebo_pkg = get_package_share_directory('robot_gazebo')

    # 2. Localizar el archivo XACRO
    xacro_file = os.path.join(
        description_pkg,
        'urdf',
        'robot.urdf.xacro'
    )

    # 3. Procesar XACRO para obtener URDF
    robot_description = xacro.process_file(xacro_file).toxml()

    # 4. Localizar nuestro mundo de Gazebo
    world_file = os.path.join(
        gazebo_pkg,
        'worlds',
        'arena.world'
    )

    # 5. Iniciar Gazebo con nuestro mundo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    # 6. Publicar la descripcion del robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # 7. Insertar automaticamente el robot en Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot_diferencial',
            '-x', '0',
            '-y', '0',
            '-z', '0.11'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])
