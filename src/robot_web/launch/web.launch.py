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
Interfaz web del robot: simulacion + rosbridge + pagina, con un solo comando.

Segun 'mode' levanta una de las configuraciones del robot:
  sim         solo la simulacion (teleop y LiDAR desde la web)
  slam        simulacion + slam_toolbox mapeando + Nav2 para explorar
  navigation  simulacion + localizacion sobre el mapa guardado + Nav2

y encima, en el namespace /web (para no chocar con los topicos del robot):
  - rosbridge_websocket: traduce entre websockets (navegador) y ROS 2
  - web_bridge: metas y waypoints a Nav2, pose del robot por TF, guardado
    del mapa y estado de todo en /web/status
  - un servidor HTTP simple que entrega la pagina (carpeta web/)

Abre http://localhost:8000 en el navegador. En WSL2 funciona igual desde el
navegador de Windows, porque localhost se reenvia a Ubuntu.

Ejemplos:
  ros2 launch robot_web web.launch.py
  ros2 launch robot_web web.launch.py mode:=slam
  ros2 launch robot_web web.launch.py mode:=sim rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            GroupAction, IncludeLaunchDescription, LogInfo,
                            Shutdown)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg, launch_file, condition, arguments):
    """Incluye un launch dentro de un GroupAction (en Foxy aisla sus args)."""
    return GroupAction(
        condition=condition,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(pkg), 'launch', launch_file)),
            launch_arguments=arguments.items())])


def _shutdown_if_requested(event, _context):
    """Apaga el launch solo si web_bridge salio normalmente (codigo 0)."""
    if event.returncode == 0:
        return [Shutdown(reason='apagado pedido desde la interfaz web')]
    return [LogInfo(msg='web_bridge termino con error (codigo %d); el resto '
                        'del sistema sigue corriendo' % event.returncode)]


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory('robot_web'), 'web')
    world_file = os.path.join(get_package_share_directory('robot_gazebo'),
                              'worlds', 'arena.world')

    mode = LaunchConfiguration('mode')
    rviz = LaunchConfiguration('rviz')
    web_port = LaunchConfiguration('web_port')
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    maps_dir = LaunchConfiguration('maps_dir')

    args = [
        DeclareLaunchArgument(
            'mode', default_value='navigation',
            description='sim | slam | navigation'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Abrir tambien RViz2 (la web ya muestra todo)'),
        DeclareLaunchArgument(
            'web_port', default_value='8000',
            description='Puerto de la pagina web'),
        DeclareLaunchArgument(
            'rosbridge_port', default_value='9090',
            description='Puerto del websocket de rosbridge'),
        DeclareLaunchArgument(
            'maps_dir', default_value='~/maps',
            description='Carpeta donde la web guarda los mapas'),
    ]

    # 1. El robot, segun el modo elegido
    robot = [
        _include('robot_bringup', 'bringup.launch.py',
                 LaunchConfigurationEquals('mode', 'sim'),
                 {'rviz': rviz, 'controller': 'false', 'teleop': 'true'}),
        _include('robot_navigation', 'slam.launch.py',
                 LaunchConfigurationEquals('mode', 'slam'),
                 {'rviz': rviz, 'nav2': 'true'}),
        _include('robot_navigation', 'navigation.launch.py',
                 LaunchConfigurationEquals('mode', 'navigation'),
                 {'rviz': rviz}),
    ]

    # 2. rosbridge: el navegador habla con ROS 2 por este websocket
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        namespace='web',
        parameters=[{'port': rosbridge_port}],
        output='screen')

    # 3. Puente con Nav2, TF y slam_toolbox. El listener de TF de Foxy se
    #    suscribe a 'tf' y 'tf_static' con nombre relativo: dentro del
    #    namespace web quedaria en /web/tf, por eso se remapea a /tf.
    #    El plano se dibuja a partir del mismo .world que carga Gazebo; como
    #    el robot aparece en (0, 0) con rumbo 0, el marco map coincide con el
    #    del mundo.
    web_bridge = Node(
        package='robot_web',
        executable='web_bridge',
        name='web_bridge',
        namespace='web',
        parameters=[{'use_sim_time': True,
                     'mode': mode,
                     'maps_dir': maps_dir,
                     'world_file': world_file,
                     # Pose del spawn (arena.launch.py): (0, 0) mirando a +x.
                     # Es la de base_link; el boton "Volver al inicio" le
                     # suma el desfase hasta base_footprint
                     'spawn_x': 0.0, 'spawn_y': 0.0, 'spawn_yaw': 0.0}],
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        output='screen',
        # Boton "Apagar sistema" de la pagina: web_bridge termina con codigo
        # 0 y se cierra todo el launch, igual que con Ctrl+C en el teleop.
        # Si terminara por un error (codigo distinto de 0) NO se apaga nada.
        on_exit=_shutdown_if_requested)

    # 4. Servidor de la pagina. Su registro de peticiones va al archivo de
    #    log (output='log') para no llenar la terminal.
    http_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', web_port,
             '--directory', web_dir],
        name='web_server',
        output='log')

    banner = LogInfo(msg=['Interfaz web en http://localhost:', web_port,
                          ' (modo ', mode, ')'])

    return LaunchDescription(
        args + robot + [rosbridge, web_bridge, http_server, banner])
