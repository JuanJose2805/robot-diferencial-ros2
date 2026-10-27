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
Launch unico: simulacion completa del robot diferencial.

Levanta, en este orden:
  1. Gazebo Classic con el mundo arena + robot (robot_gazebo/arena.launch.py)
  2. RViz2 con la configuracion del proyecto           (rviz:=true)
  3. Controlador de posicion en espera de metas        (controller:=true)
  4. Teleoperacion WASD en ventana xterm               (teleop:=true)

Al salir del teleop (Ctrl+C o cerrar su ventana) se apaga TODO el sistema,
incluido Gazebo.

Ejemplos:
  ros2 launch robot_bringup bringup.launch.py
  ros2 launch robot_bringup bringup.launch.py rviz:=false
  ros2 launch robot_bringup bringup.launch.py teleop:=false
"""
import os
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def stop_gazebo(context, *args, **kwargs):
    """
    Cierra gzserver/gzclient aunque hayan quedado como procesos hijos.

    En esta instalacion, gzserver y gzclient arrancan como un proceso
    envoltorio con el Gazebo real como hijo. Cuando el apagado lo inicia el
    propio launch (por ejemplo al salir del teleop), la senal solo llega al
    envoltorio y el Gazebo real queda huerfano. Aqui se le envia SIGINT
    directamente; si en 10 s no cerro, se fuerza con SIGKILL.
    """
    patterns = ('^gzserver', '^gzclient')
    for pattern in patterns:
        subprocess.run(['pkill', '-INT', '-f', pattern], check=False)
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        alive = [p for p in patterns if subprocess.run(
            ['pgrep', '-f', p], stdout=subprocess.DEVNULL).returncode == 0]
        if not alive:
            return []
        time.sleep(0.5)
    for pattern in patterns:
        subprocess.run(['pkill', '-KILL', '-f', pattern], check=False)
    return []


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('robot_gazebo')
    control_pkg = get_package_share_directory('robot_control')
    description_pkg = get_package_share_directory('robot_description')

    rviz = LaunchConfiguration('rviz')
    controller = LaunchConfiguration('controller')
    teleop = LaunchConfiguration('teleop')

    args = [
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Abrir RViz2 con modelo y LiDAR'),
        DeclareLaunchArgument(
            'controller', default_value='true',
            description='Arrancar el controlador de posicion'),
        DeclareLaunchArgument(
            'teleop', default_value='true',
            description='Abrir teleop WASD en xterm (requiere xterm)'),
    ]

    # 1. Gazebo + mundo arena + robot_state_publisher + spawn del robot
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'arena.launch.py')))

    # 2. RViz2 con la configuracion guardada del proyecto
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            description_pkg, 'rviz', 'simulation.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz))

    # 3. Controlador de posicion (solo publica /cmd_vel con meta activa)
    position_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, 'launch', 'control.launch.py')),
        condition=IfCondition(controller))

    # 4. Teleop WASD en su propia ventana xterm (necesita teclado propio).
    #    on_exit=Shutdown(): cuando el teleop termina (Ctrl+C o se cierra
    #    su ventana), el launch apaga todos los demas procesos.
    teleop_node = Node(
        package='robot_control',
        executable='teleop_wasd',
        name='teleop_wasd',
        prefix='xterm -title "Teleop WASD" -e',
        on_exit=Shutdown(reason='Teleop WASD finalizado'),
        condition=IfCondition(teleop))

    # 5. Al apagar el sistema, asegurar que Gazebo no quede abierto
    cleanup = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=stop_gazebo)]))

    return LaunchDescription(
        args + [simulation, rviz_node, position_controller, teleop_node,
                cleanup])
