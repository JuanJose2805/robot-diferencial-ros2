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
Etapa 2.4.2: navegacion autonoma con Nav2 sobre el mapa de la arena.

Levanta:
  1. La simulacion (Gazebo + robot + teleop WASD para cerrar todo).
  2. slam_toolbox en modo LOCALIZACION: carga el grafo de poses guardado al
     mapear (maps/arena.posegraph/.data), publica el mapa (/map) y la TF
     map -> odom. Hace el papel de map_server + amcl.
  3. Nav2 de Foxy (nav2_bringup/navigation_launch.py):
     - planner_server: ruta global (NavFn) sobre el costmap global
     - controller_server: sigue la ruta con DWB y esquiva los obstaculos que
       ve el LiDAR en el costmap local, aunque no esten en el mapa
     - recoveries_server y bt_navigator: comportamiento ante bloqueos, con
       el arbol de behavior_trees/ (tiempos de espera ampliados)
  4. RViz2 con la vista estandar de Nav2 (herramienta "Navigation2 Goal").

Ejemplos:
  ros2 launch robot_navigation navigation.launch.py
  ros2 launch robot_navigation navigation.launch.py rviz:=false
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
    nav2_pkg = get_package_share_directory('nav2_bringup')

    rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')
    loc_params = LaunchConfiguration('localization_params')

    args = [
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Abrir RViz2 con la vista de Nav2'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_pkg, 'maps', 'arena'),
            description='Grafo de poses de slam_toolbox (ruta sin extension)'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(nav_pkg, 'config', 'nav2_params.yaml'),
            description='Archivo de parametros de Nav2'),
        DeclareLaunchArgument(
            'localization_params',
            default_value=os.path.join(
                nav_pkg, 'config', 'slam_toolbox_localization.yaml'),
            description='Parametros de slam_toolbox en modo localizacion'),
    ]

    # 1. Simulacion + teleop WASD (sin su RViz ni el controlador de
    #    posicion, que tambien publicaria en /cmd_vel). GroupAction aisla
    #    los argumentos del include (en Foxy son globales).
    simulation = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, 'launch', 'bringup.launch.py')),
            launch_arguments={'rviz': 'false', 'controller': 'false',
                              'teleop': 'true'}.items()),
    ])

    # 2. Localizacion con slam_toolbox sobre el mapa guardado
    localization = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[loc_params,
                    {'use_sim_time': True, 'map_file_name': map_file}],
        output='screen')

    # 3. Nav2: planificador, controlador, recuperaciones y navegador
    nav2 = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params,
                              'use_sim_time': 'true',
                              'autostart': 'true',
                              'map_subscribe_transient_local': 'true',
                              'default_bt_xml_filename': os.path.join(
                                  nav_pkg, 'behavior_trees',
                                  'navigate_w_replanning_and_recovery.xml'),
                              }.items()),
    ])

    # 4. RViz2 con la vista estandar de Nav2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_pkg, 'rviz',
                                      'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz))

    return LaunchDescription(
        args + [simulation, localization, nav2, rviz_node])
