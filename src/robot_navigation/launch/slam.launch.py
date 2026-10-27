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

"""
Etapa 2.4.1: mapeo del entorno con SLAM (slam_toolbox).

Levanta la simulacion completa (Gazebo + robot + teleop WASD) y encima
slam_toolbox en modo mapeo asincrono, que construye el mapa 2D a partir del
LiDAR (/scan) y la odometria (/odom) y publica la TF map -> odom. RViz2 se
abre con una configuracion propia que muestra el mapa en construccion.

Para mapear: mueve el robot con WASD por toda la arena. Para guardar el mapa
(en otra terminal):
  ros2 run nav2_map_server map_saver_cli -f \
    ~/robotics40_ws/src/robot_navigation/maps/arena

Ejemplos:
  ros2 launch robot_navigation slam.launch.py
  ros2 launch robot_navigation slam.launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('robot_bringup')
    nav_pkg = get_package_share_directory('robot_navigation')

    rviz = LaunchConfiguration('rviz')
    slam_params = LaunchConfiguration('slam_params')

    args = [
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Abrir RViz2 con el mapa en construccion'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(nav_pkg, 'config', 'slam_toolbox.yaml'),
            description='Archivo de parametros de slam_toolbox'),
    ]

    # 1. Simulacion + teleop WASD (sin su RViz ni el controlador de
    #    posicion: durante el mapeo el robot se mueve con el teclado)
    #    GroupAction aisla los argumentos: en Foxy, los launch_arguments de
    #    un include son globales, y su rviz:=false apagaria tambien el RViz
    #    de SLAM de este archivo.
    simulation = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, 'launch', 'bringup.launch.py')),
            launch_arguments={'rviz': 'false', 'controller': 'false',
                              'teleop': 'true'}.items()),
    ])

    # 2. slam_toolbox en modo mapeo asincrono
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': True}],
        output='screen')

    # 3. RViz2 con el mapa, el robot y el LiDAR en el marco map
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav_pkg, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz))

    return LaunchDescription(args + [simulation, slam_toolbox, rviz_node])
