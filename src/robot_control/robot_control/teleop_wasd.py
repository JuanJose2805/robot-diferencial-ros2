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

"""Teleoperacion WASD del robot diferencial en una terminal interactiva."""

import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


BANNER = """
Teleoperacion del robot diferencial
-----------------------------------
  Q  W  E       W / S : avanzar / retroceder
  A     D       A / D : girar izquierda / derecha
     S          Q / E : avanzar girando izquierda / derecha

 ESPACIO o X : detener
 R / F        : aumentar / disminuir velocidad lineal
 T / G        : aumentar / disminuir velocidad angular
 CTRL+C       : detener y salir

 Seguridad: si no llegan nuevas pulsaciones de movimiento
 durante 0.5 segundos, el robot recibe velocidad cero.
"""


# Tecla -> (factor lineal, factor angular)
MOVE_BINDINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'q': (1.0, 1.0),
    'e': (1.0, -1.0),
}


# Tecla -> (incremento lineal, incremento angular)
SPEED_BINDINGS = {
    'r': (0.05, 0.0),
    'f': (-0.05, 0.0),
    't': (0.0, 0.10),
    'g': (0.0, -0.10),
}


STOP_KEYS = (' ', 'x')


class TeleopWasd(Node):
    """Controla /cmd_vel con teclas y caducidad de comando."""

    def __init__(self):
        super().__init__('teleop_wasd')

        # Parametros ROS 2
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.80)
        self.declare_parameter('max_linear', 0.50)
        self.declare_parameter('max_angular', 2.00)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('key_timeout', 0.5)

        topic = self.get_parameter('cmd_vel_topic').value

        self.max_linear = float(
            self.get_parameter('max_linear').value
        )
        self.max_angular = float(
            self.get_parameter('max_angular').value
        )
        self.linear_speed = float(
            self.get_parameter('linear_speed').value
        )
        self.angular_speed = float(
            self.get_parameter('angular_speed').value
        )

        rate = float(self.get_parameter('publish_rate').value)
        self.key_timeout = float(
            self.get_parameter('key_timeout').value
        )

        # Validacion de parametros
        if rate <= 0 or self.key_timeout <= 0:
            raise ValueError(
                'publish_rate y key_timeout deben ser positivos'
            )

        if self.max_linear < 0.05 or self.max_angular < 0.10:
            raise ValueError(
                'max_linear >= 0.05 y max_angular >= 0.10'
            )

        if not 0.05 <= self.linear_speed <= self.max_linear:
            raise ValueError('linear_speed fuera de rango')

        if not 0.10 <= self.angular_speed <= self.max_angular:
            raise ValueError('angular_speed fuera de rango')

        self.period = 1.0 / rate

        # Publicador de velocidades
        self.publisher = self.create_publisher(Twist, topic, 10)

        # Estado inicial: detenido
        self.direction = (0.0, 0.0)
        self.last_motion_key = None

    @property
    def moving(self):
        """Indica si existe una orden activa de movimiento."""
        return self.direction != (0.0, 0.0)

    def publish_twist(self):
        """Publica la velocidad correspondiente a la tecla."""
        msg = Twist()

        msg.linear.x = (
            self.direction[0] * self.linear_speed
        )

        msg.angular.z = (
            self.direction[1] * self.angular_speed
        )

        self.publisher.publish(msg)

    def stop(self):
        """Envia tres comandos de velocidad cero."""
        self.direction = (0.0, 0.0)
        self.last_motion_key = None

        for _ in range(3):
            self.publisher.publish(Twist())

    def handle_key(self, key):
        """Procesa las teclas de movimiento y velocidad."""
        if key in MOVE_BINDINGS:
            self.direction = MOVE_BINDINGS[key]

            # Actualiza el instante de la ultima pulsacion
            self.last_motion_key = time.monotonic()
            return True

        if key in STOP_KEYS:
            self.stop()
            return True

        if key in SPEED_BINDINGS:
            d_lin, d_ang = SPEED_BINDINGS[key]

            self.linear_speed = round(
                min(
                    self.max_linear,
                    max(0.05, self.linear_speed + d_lin)
                ),
                3
            )

            self.angular_speed = round(
                min(
                    self.max_angular,
                    max(0.10, self.angular_speed + d_ang)
                ),
                3
            )

            return True

        return False

    def status_line(self):
        """Muestra el estado de la teleoperacion."""
        state = 'MOVIENDO' if self.moving else 'DETENIDO'

        return (
            '\r[%s] lineal=%.2f m/s  '
            'angular=%.2f rad/s      '
        ) % (
            state,
            self.linear_speed,
            self.angular_speed
        )

    @staticmethod
    def read_key(timeout):
        """Lee una tecla sin bloquear la terminal."""
        fd = sys.stdin.fileno()

        ready, _, _ = select.select(
            [fd], [], [], timeout
        )

        if not ready:
            return ''

        char = os.read(fd, 1).decode(
            'utf-8',
            errors='ignore'
        )

        # Ignorar secuencias de las flechas
        if char == '\x1b':
            while select.select([fd], [], [], 0.0)[0]:
                os.read(fd, 1)
            return ''

        return char.lower()

    def run(self):
        """Ejecuta el control por teclado."""
        if not sys.stdin.isatty():
            self.get_logger().error(
                'Ejecuta teleop_wasd desde una terminal '
                'interactiva: ros2 run robot_control teleop_wasd'
            )
            return

        fd = sys.stdin.fileno()
        settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)

            print(BANNER)
            sys.stdout.write(self.status_line())
            sys.stdout.flush()

            while rclpy.ok():

                # Esperar una tecla durante un periodo corto
                key = self.read_key(self.period)

                if key and self.handle_key(key):
                    sys.stdout.write(self.status_line())
                    sys.stdout.flush()

                # Solo publicar cuando existe movimiento
                if self.moving:

                    elapsed = (
                        time.monotonic()
                        - self.last_motion_key
                    )

                    # Parada automatica por inactividad
                    if elapsed >= self.key_timeout:
                        self.stop()

                        sys.stdout.write(
                            self.status_line()
                        )
                        sys.stdout.flush()

                    else:
                        self.publish_twist()

        except KeyboardInterrupt:
            pass

        finally:
            # Intentar detener el robot y restaurar la terminal
            self.stop()

            termios.tcsetattr(
                fd,
                termios.TCSADRAIN,
                settings
            )

            print(
                '\nTeleoperacion finalizada. '
                'Comando de parada enviado.'
            )


def main(args=None):
    """Punto de entrada del nodo ROS 2."""
    rclpy.init(args=args)
    node = None

    try:
        node = TeleopWasd()
        node.run()

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
