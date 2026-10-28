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
Puente entre la interfaz web y ROS 2 (Nav2, TF y slam_toolbox).

El rosbridge de Foxy solo maneja topicos y servicios: no sabe enviar metas a
las acciones de Nav2 ni leer el arbol de TF. Este nodo hace ese trabajo y le
ofrece a la pagina una interfaz sencilla basada en topicos:

Entradas (lo que publica la web):
  goal       geometry_msgs/PoseStamped  meta unica -> NavigateToPose
  waypoints  nav_msgs/Path              ruta de puntos -> FollowWaypoints
  cancel     std_msgs/Empty             cancela la navegacion en curso
  save_map   std_msgs/String            guarda el mapa con ese nombre
  shutdown   std_msgs/Empty             apaga todo el sistema (el launch
                                        termina cuando este nodo sale)

Salidas (lo que lee la web):
  robot_pose geometry_msgs/PoseStamped  pose del robot en map (u odom)
  scan_points std_msgs/String           puntos del LiDAR ya en el marco map
                                        (JSON), para dibujarlos sobre el mapa
  status     std_msgs/String            estado en JSON (navegacion, mapa...)
  events     std_msgs/String            mensajes cortos para el registro
  world_layout std_msgs/String          plano de la arena en JSON (latched),
                                        leido del archivo .world de Gazebo

Todos los nombres son relativos: con el namespace 'web' del launch quedan
como /web/goal, /web/status, etc., separados de los topicos del robot.
"""

import json
import os
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from robot_web.web_utils import (
    compose_pose, map_path, parse_world_layout, scan_to_points,
    yaw_from_quaternion)
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import Empty, String
import tf2_ros

TF_ERRORS = (tf2_ros.LookupException, tf2_ros.ConnectivityException,
             tf2_ros.ExtrapolationException)

RESULT_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


class WebBridge(Node):
    """Traduce los pedidos de la web a acciones de Nav2 y publica el estado."""

    def __init__(self):
        super().__init__('web_bridge')

        self.mode = self._param('mode', 'sim')
        self.global_frame = self._param('global_frame', 'map')
        self.odom_frame = self._param('odom_frame', 'odom')
        self.robot_frame = self._param('robot_frame', 'base_footprint')
        self.lidar_frame = self._param('lidar_frame', 'lidar_link')
        self.maps_dir = self._param('maps_dir', '~/maps')
        self.world_file = self._param('world_file', '')
        # Pose con que aparece el modelo en Gazebo (la del spawn). Ojo: el
        # spawn ubica base_link, pero Nav2 lleva a la meta el marco
        # base_footprint (eje de las ruedas, 10 cm detras). La pose de
        # inicio se calcula sumando ese desfase, que se lee de la TF.
        self.spawn = (float(self._param('spawn_x', 0.0)),
                      float(self._param('spawn_y', 0.0)),
                      float(self._param('spawn_yaw', 0.0)))
        self.home = None
        pose_rate = float(self._param('pose_rate', 10.0))

        # TF: de aqui sale la pose del robot en el mapa
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Acciones de Nav2 (nombres absolutos: viven fuera del namespace web)
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        # En Foxy el servidor de waypoints se llama 'FollowWaypoints' (en
        # versiones posteriores paso a 'follow_waypoints')
        self.wp_client = ActionClient(
            self, FollowWaypoints, '/FollowWaypoints')
        self.serialize_client = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')

        self.create_subscription(PoseStamped, 'goal', self._on_goal, 10)
        self.create_subscription(Path, 'waypoints', self._on_waypoints, 10)
        self.create_subscription(Empty, 'cancel', self._on_cancel, 10)
        self.create_subscription(String, 'save_map', self._on_save_map, 10)

        self.pose_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.scan_pub = self.create_publisher(String, 'scan_points', 10)
        # El scan se pasa al mapa con la pose del LiDAR en el MISMO instante
        # en que se midio (no la ultima): si el robot gira o avanza, usar la
        # pose actual corre los puntos unos centimetros fuera de las paredes.
        self.pending_scan = None
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.events_pub = self.create_publisher(String, 'events', 10)
        # transient_local: quien se conecte despues (la web) recibe el plano
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.world_pub = self.create_publisher(String, 'world_layout',
                                               latched)
        self._publish_world()

        # Estado de la navegacion. 'token' identifica la meta vigente: los
        # resultados de metas viejas (reemplazadas o canceladas) se ignoran.
        self.token = 0
        self.goal_handle = None
        self.nav = {'state': 'idle', 'kind': None, 'goal': None,
                    'distance_remaining': None, 'recoveries': 0,
                    'current_waypoint': None, 'total_waypoints': 0,
                    'missed_waypoints': [], 'started': None}
        self.map_save = {'state': 'idle', 'message': '', 'path': None}
        self.pose_frame = None
        self.slam_available = False
        self.saving_lock = threading.Lock()
        self.shutdown_requested = False
        self.started_at = time.monotonic()
        self.create_subscription(Empty, 'shutdown', self._on_shutdown, 10)

        self.create_timer(1.0 / pose_rate, self._publish_poses)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            'web_bridge listo (modo %s, mapas en %s)'
            % (self.mode, os.path.expanduser(self.maps_dir)))

    def _publish_world(self):
        """Publica una vez el plano de la arena leido del .world."""
        if not self.world_file:
            return
        try:
            with open(os.path.expanduser(self.world_file)) as f:
                shapes = parse_world_layout(f.read())
        except (OSError, ValueError, SyntaxError) as err:
            self.get_logger().warning('No se pudo leer el plano %s: %s'
                                      % (self.world_file, err))
            return
        msg = String()
        msg.data = json.dumps({'frame': self.global_frame,
                               'shapes': shapes})
        self.world_pub.publish(msg)
        self.get_logger().info('Plano publicado: %d figuras' % len(shapes))

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _event(self, text, level='info'):
        """Publica un mensaje corto para el registro de la web."""
        msg = String()
        msg.data = json.dumps({'level': level, 'text': text})
        self.events_pub.publish(msg)
        # Cada nivel en su propia linea: en Foxy, rclpy no permite que una
        # misma linea de codigo registre con niveles distintos (lanza
        # "Logger severity cannot be changed between calls")
        if level == 'error':
            self.get_logger().error(text)
        elif level == 'warn':
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    # ------------------------------------------------------------------ TF
    def _lookup(self, child, stamp=None):
        """
        Pose de 'child' en map (o en odom si aun no hay mapa).

        Sin 'stamp' usa la TF mas reciente; con 'stamp', la de ese instante.
        """
        when = Time() if stamp is None else Time.from_msg(stamp)
        for frame in (self.global_frame, self.odom_frame):
            try:
                tf = self.tf_buffer.lookup_transform(frame, child, when)
            except TF_ERRORS:
                continue
            pose = PoseStamped()
            pose.header = tf.header
            pose.header.frame_id = frame
            t = tf.transform.translation
            pose.pose.position.x, pose.pose.position.y = t.x, t.y
            pose.pose.orientation = tf.transform.rotation
            return pose
        return None

    def _publish_poses(self):
        robot = self._lookup(self.robot_frame)
        self.pose_frame = robot.header.frame_id if robot else None
        if robot:
            self.pose_pub.publish(robot)
        if self.pending_scan is not None:
            self._publish_scan(self.pending_scan)

    def _on_scan(self, scan):
        self._publish_scan(scan)

    def _publish_scan(self, scan):
        """
        Publica el scan en coordenadas del mapa.

        Si la TF de ese instante todavia no llego, el scan queda pendiente y
        se reintenta en el siguiente ciclo (se guarda solo el mas nuevo).
        """
        lidar = self._lookup(scan.header.frame_id or self.lidar_frame,
                             scan.header.stamp)
        if lidar is None:
            self.pending_scan = scan
            return
        self.pending_scan = None
        q = lidar.pose.orientation
        pose = (lidar.pose.position.x, lidar.pose.position.y,
                yaw_from_quaternion(q.x, q.y, q.z, q.w))
        points = scan_to_points(pose, scan.angle_min, scan.angle_increment,
                                scan.ranges, scan.range_min, scan.range_max)
        msg = String()
        msg.data = json.dumps({'frame': lidar.header.frame_id,
                               'points': points})
        self.scan_pub.publish(msg)

    def _home(self):
        """Pose de base_footprint cuando el robot esta en el spawn."""
        if self.home is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', self.robot_frame, Time())
            except TF_ERRORS:
                return None
            q = tf.transform.rotation
            offset = (tf.transform.translation.x, tf.transform.translation.y,
                      yaw_from_quaternion(q.x, q.y, q.z, q.w))
            x, y, yaw = compose_pose(self.spawn, offset)
            self.home = {'x': round(x, 3), 'y': round(y, 3),
                         'yaw': round(yaw, 4)}
        return self.home

    # -------------------------------------------------------------- Estado
    def _publish_status(self):
        names = [n for n, _ in self.get_service_names_and_types()]
        self.slam_available = '/slam_toolbox/serialize_map' in names
        nav = dict(self.nav)
        if nav['started'] is not None and nav['state'] in ('active',):
            nav['elapsed'] = round(time.monotonic() - nav['started'], 1)
        nav.pop('started')
        status = {
            'mode': self.mode,
            'pose_frame': self.pose_frame,
            'nav_ready': self.nav_client.server_is_ready(),
            'waypoints_ready': self.wp_client.server_is_ready(),
            'slam_ready': self.slam_available,
            'maps_dir': os.path.expanduser(self.maps_dir),
            'home': self._home(),
            'nav': nav,
            'map_save': self.map_save,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    # ---------------------------------------------------------- Navegacion
    def _prepare_pose(self, pose):
        if not pose.header.frame_id:
            pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        q = pose.pose.orientation
        if q.x == q.y == q.z == q.w == 0.0:
            pose.pose.orientation.w = 1.0
        return pose

    def _describe(self, pose):
        q = pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        return {'x': round(pose.pose.position.x, 3),
                'y': round(pose.pose.position.y, 3),
                'yaw': round(yaw, 3)}

    def _start(self, kind, client, goal_msg, description, total=0):
        if not client.server_is_ready():
            self._event('Nav2 no esta disponible: lanza el modo slam o '
                        'navigation para enviar metas.', 'error')
            return
        # Si habia una navegacion del otro tipo, se cancela primero
        if self.goal_handle is not None and self.nav['kind'] != kind:
            self.goal_handle.cancel_goal_async()
        self.token += 1
        token = self.token
        self.goal_handle = None
        self.nav.update({'state': 'sending', 'kind': kind,
                         'goal': description, 'distance_remaining': None,
                         'recoveries': 0, 'current_waypoint': 0 if total
                         else None, 'total_waypoints': total,
                         'missed_waypoints': [],
                         'started': time.monotonic()})
        future = client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self._on_feedback(token, kind, fb))
        future.add_done_callback(
            lambda f: self._on_goal_response(token, f))

    def _on_goal(self, pose):
        pose = self._prepare_pose(pose)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        info = self._describe(pose)
        self._start('goal', self.nav_client, goal, info)
        self._event('Meta enviada: x=%.2f y=%.2f' % (info['x'], info['y']))

    def _on_waypoints(self, path):
        if not path.poses:
            self._event('La ruta de waypoints esta vacia.', 'warn')
            return
        goal = FollowWaypoints.Goal()
        goal.poses = [self._prepare_pose(p) for p in path.poses]
        info = [self._describe(p) for p in goal.poses]
        self._start('waypoints', self.wp_client, goal, info, len(info))
        self._event('Ruta enviada con %d waypoints.' % len(info))

    def _on_cancel(self, _msg):
        if self.goal_handle is None:
            self._event('No hay navegacion en curso.', 'warn')
            return
        self.goal_handle.cancel_goal_async()
        self._event('Cancelando la navegacion...')

    def _on_goal_response(self, token, future):
        if token != self.token:
            return
        handle = future.result()
        if not handle.accepted:
            self.nav['state'] = 'rejected'
            self._event('Nav2 rechazo la meta.', 'error')
            return
        self.goal_handle = handle
        self.nav['state'] = 'active'
        handle.get_result_async().add_done_callback(
            lambda f: self._on_result(token, f))

    def _on_feedback(self, token, kind, feedback_msg):
        if token != self.token:
            return
        fb = feedback_msg.feedback
        if kind == 'goal':
            self.nav['distance_remaining'] = round(
                float(fb.distance_remaining), 2)
            self.nav['recoveries'] = int(fb.number_of_recoveries)
        else:
            self.nav['current_waypoint'] = int(fb.current_waypoint)

    def _on_result(self, token, future):
        if token != self.token:
            return
        result = future.result()
        state = RESULT_NAMES.get(result.status, 'failed')
        self.goal_handle = None
        self.nav['state'] = state
        self.nav['started'] = None
        if self.nav['kind'] == 'waypoints' and result.result is not None:
            missed = list(result.result.missed_waypoints)
            self.nav['missed_waypoints'] = missed
            if state == 'succeeded':
                self.nav['current_waypoint'] = self.nav['total_waypoints']
            if missed:
                self._event('Waypoints no alcanzados: %s'
                            % ', '.join(str(i + 1) for i in missed), 'warn')
        texts = {'succeeded': ('Meta alcanzada.', 'info'),
                 'canceled': ('Navegacion cancelada.', 'warn'),
                 'aborted': ('Nav2 no pudo llegar a la meta.', 'error'),
                 'failed': ('La navegacion termino con error.', 'error')}
        self._event(*texts[state])

    # ------------------------------------------------------------ Apagado
    def _on_shutdown(self, _msg):
        """Pedido de la web: este nodo sale y el launch cierra todo."""
        if self.shutdown_requested:
            return
        # Un pedido que llegue justo al arrancar no puede venir de un clic
        # nuevo (seria un mensaje viejo): se ignora por seguridad
        if time.monotonic() - self.started_at < 5.0:
            self.get_logger().warning('Pedido de apagado ignorado (el nodo '
                                      'acaba de arrancar)')
            return
        self.shutdown_requested = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self._event('Apagando todo el sistema...', 'warn')

    # -------------------------------------------------------- Guardar mapa
    def _on_save_map(self, msg):
        if not self.saving_lock.acquire(blocking=False):
            self._event('Ya se esta guardando un mapa.', 'warn')
            return
        path = map_path(self.maps_dir, msg.data)
        self.map_save = {'state': 'saving', 'path': path,
                         'message': 'Guardando %s ...' % path}
        self._event('Guardando el mapa en %s' % path)
        threading.Thread(target=self._save_map, args=(path,),
                         daemon=True).start()

    def _save_map(self, path):
        """
        Guarda imagen+yaml (map_saver) y el grafo de poses (slam_toolbox).

        Corre en un hilo aparte para no frenar el resto del nodo mientras
        map_saver espera el mapa.
        """
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            done = []
            # 1) Imagen .pgm + .yaml: sirve para verlo y para el costmap
            cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                   '-f', path, '--ros-args',
                   '-p', 'save_map_timeout:=10000']
            proc = subprocess.run(cmd, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, timeout=40)
            if proc.returncode == 0 and os.path.exists(path + '.yaml'):
                done.append('.pgm/.yaml')
            else:
                tail = proc.stdout.decode(errors='replace').strip()
                self.get_logger().error('map_saver_cli: %s' % tail[-400:])

            # 2) Grafo de poses: navigation.launch.py lo usa para localizarse
            if self.serialize_client.wait_for_service(timeout_sec=2.0):
                req = SerializePoseGraph.Request()
                req.filename = path
                future = self.serialize_client.call_async(req)
                end = time.monotonic() + 30.0
                while not future.done() and time.monotonic() < end:
                    time.sleep(0.1)
                if future.done() and future.result() is not None:
                    done.append('.posegraph/.data')

            if len(done) == 2:
                self.map_save = {'state': 'saved', 'path': path,
                                 'message': 'Mapa guardado (%s)'
                                 % ', '.join(done)}
                self._event('Mapa guardado: %s' % path)
            elif done:
                self.map_save = {'state': 'partial', 'path': path,
                                 'message': 'Solo se guardo %s'
                                 % ', '.join(done)}
                self._event('Mapa guardado a medias (%s)' % ', '.join(done),
                            'warn')
            else:
                self.map_save = {'state': 'error', 'path': path,
                                 'message': 'No se pudo guardar el mapa'}
                self._event('No se pudo guardar el mapa (hay SLAM '
                            'corriendo?)', 'error')
        except (OSError, subprocess.SubprocessError) as err:
            self.map_save = {'state': 'error', 'path': path,
                             'message': str(err)}
            self._event('Error al guardar el mapa: %s' % err, 'error')
        finally:
            self.saving_lock.release()


def main(args=None):
    """
    Corre el nodo hasta que la web pida apagar el sistema.

    Un error dentro de un callback se registra y el nodo sigue: como el
    launch cierra todo cuando este nodo termina, un fallo puntual no debe
    apagar la simulacion. Solo el apagado pedido termina con codigo 0.
    """
    rclpy.init(args=args)
    node = WebBridge()
    try:
        while rclpy.ok() and not node.shutdown_requested:
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception as err:  # noqa: B902 - se registra y sigue
                node.get_logger().error('Error en un callback: %r' % err)
        if node.shutdown_requested:
            # Un momento para que el aviso y la cancelacion lleguen a destino
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Foxy: destruir los clientes de accion antes que el nodo
        node.nav_client.destroy()
        node.wp_client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
