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

"""Nodo ROS 2 del controlador de posicion para un robot diferencial."""

import math

from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from robot_control.go_to_goal import (
    ControllerParams, GoToGoal, normalize_angle, yaw_from_quaternion)
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros


class PositionController(Node):
    """
    Lleva el robot a una meta (x, y) usando solo cmd_vel y la odometria.

    Entradas:
      ~/goal     (geometry_msgs/PoseStamped)  meta con orientacion opcional
      ~/goal_xy  (geometry_msgs/Point)        meta (x, y) en el marco odom
      ~/cancel   (std_srvs/Trigger)           cancela la meta actual
    Salidas:
      cmd_vel    (geometry_msgs/Twist)        solo mientras hay una meta
      ~/status   (std_msgs/String)            idle | moving | reached | ...
    """

    def __init__(self):
        super().__init__('position_controller')

        self.cmd_vel_topic = self._param('cmd_vel_topic', '/cmd_vel')
        self.odom_topic = self._param('odom_topic', '/odom')
        self.odom_frame = self._param('odom_frame', 'odom')
        rate = float(self._param('control_rate', 20.0))
        if rate <= 0:
            raise ValueError('control_rate debe ser mayor que cero')
        self.odom_timeout = float(self._param('odom_timeout', 1.0))
        self.use_goal_yaw = bool(self._param('use_goal_yaw', False))

        defaults = ControllerParams()
        values = {}
        for name in ('k_rho', 'k_alpha', 'max_linear', 'max_angular',
                     'min_linear', 'min_angular', 'max_decel',
                     'max_angular_decel', 'goal_tolerance', 'yaw_tolerance',
                     'turn_in_place_angle', 'turn_exit_angle'):
            values[name] = float(self._param(name, getattr(defaults, name)))
        values['align_final_yaw'] = self.use_goal_yaw
        self.controller = GoToGoal(ControllerParams(**values))

        self.pose = None
        self.last_odom_time = None
        self.active = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(PoseStamped, '~/goal', self.on_goal, 10)
        self.create_subscription(Point, '~/goal_xy', self.on_goal_xy, 10)
        self.create_service(Trigger, '~/cancel', self.on_cancel)
        self.create_timer(1.0 / rate, self.on_timer)

        self.publish_status('idle')
        self.get_logger().info(
            'Controlador de posicion listo. Meta: ~/goal o ~/goal_xy')

    def _param(self, name, default):
        """Declara un parametro con valor por defecto y devuelve su valor."""
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def publish_status(self, text):
        """Publica el estado actual del controlador."""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def stop_robot(self):
        """Publica velocidad cero para detener el robot."""
        for _ in range(3):
            self.cmd_pub.publish(Twist())

    def finish(self, status):
        """Termina la meta actual, detiene el robot y publica el estado."""
        self.active = False
        self.controller.cancel()
        self.stop_robot()
        self.publish_status(status)

    def on_odom(self, msg):
        """Guarda la pose del robot en el marco de odometria."""
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (pos.x, pos.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.last_odom_time = self.get_clock().now()

    def goal_to_odom(self, goal):
        """Convierte una PoseStamped al marco de odometria: (x, y, yaw)."""
        q = goal.pose.orientation
        has_yaw = any(abs(c) > 1e-9 for c in (q.x, q.y, q.z, q.w))
        gx = goal.pose.position.x
        gy = goal.pose.position.y
        gyaw = yaw_from_quaternion(q.x, q.y, q.z, q.w) if has_yaw else None

        frame = goal.header.frame_id
        if frame in ('', self.odom_frame):
            return gx, gy, gyaw

        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as err:
            self.get_logger().warn(
                'No se puede pasar la meta de "%s" a "%s": %s'
                % (frame, self.odom_frame, err))
            return None

        t = tf.transform
        tyaw = yaw_from_quaternion(
            t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        c, s = math.cos(tyaw), math.sin(tyaw)
        ox = t.translation.x + c * gx - s * gy
        oy = t.translation.y + s * gx + c * gy
        oyaw = None if gyaw is None else normalize_angle(gyaw + tyaw)
        return ox, oy, oyaw

    def start_goal(self, x, y, yaw):
        """Activa una meta nueva (reemplaza la anterior si la hubiera)."""
        if self.pose is None:
            self.get_logger().warn('Sin odometria todavia; meta ignorada.')
            self.publish_status('error: sin odometria')
            return
        self.controller.set_goal(x, y, yaw)
        self.active = True
        self.publish_status('moving')
        self.get_logger().info('Nueva meta: x=%.2f y=%.2f' % (x, y))

    def on_goal(self, msg):
        """Recibe una meta con marco de referencia."""
        goal = self.goal_to_odom(msg)
        if goal is None:
            self.publish_status('error: marco desconocido')
            return
        self.start_goal(*goal)

    def on_goal_xy(self, msg):
        """Recibe una meta (x, y) expresada en el marco de odometria."""
        self.start_goal(msg.x, msg.y, None)

    def on_cancel(self, request, response):
        """Servicio para cancelar la meta en curso."""
        was_active = self.active
        if was_active:
            self.finish('canceled')
        response.success = was_active
        response.message = 'meta cancelada' if was_active else 'sin meta'
        return response

    def odom_is_stale(self):
        """Indica si la odometria dejo de llegar."""
        if self.last_odom_time is None:
            return True
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds
        return age * 1e-9 > self.odom_timeout

    def on_timer(self):
        """Ciclo de control: calcula y publica cmd_vel mientras hay meta."""
        if not self.active:
            return
        if self.odom_is_stale():
            self.get_logger().warn('Odometria perdida; robot detenido.')
            self.finish('error: sin odometria')
            return

        v, w, done = self.controller.step(*self.pose)
        if done:
            self.get_logger().info('Meta alcanzada.')
            self.finish('reached')
            return

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)


def main(args=None):
    """Punto de entrada del nodo."""
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.active:
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
