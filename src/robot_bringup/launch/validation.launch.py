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
Fase de validacion de hardware simulado (etapa 2.3).

Levanta la simulacion completa SIN teleop (para que nadie mueva el robot
durante las pruebas) y ejecuta el nodo hardware_validation, que prueba
sensores, actuadores y controlador de posicion e imprime un informe.

Al terminar, la simulacion queda abierta para inspeccionarla; se cierra con
Ctrl+C.

Ejemplos:
  ros2 launch robot_bringup validation.launch.py
  ros2 launch robot_bringup validation.launch.py sim:=false   # simulacion ya abierta
  ros2 launch robot_bringup validation.launch.py rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('robot_bringup')

    sim = LaunchConfiguration('sim')
    rviz = LaunchConfiguration('rviz')

    args = [
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='Levantar la simulacion (false si ya esta abierta)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Abrir RViz2 junto con la simulacion'),
    ]

    # Simulacion completa sin teleop: Gazebo + RViz2 + controlador
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'bringup.launch.py')),
        launch_arguments={'teleop': 'false', 'rviz': rviz,
                          'controller': 'true'}.items(),
        condition=IfCondition(sim))

    # Nodo de validacion: espera a que la simulacion publique y luego prueba
    validation = Node(
        package='robot_control',
        executable='hardware_validation',
        name='hardware_validation',
        output='screen',
        emulate_tty=True)

    return LaunchDescription(args + [simulation, validation])
