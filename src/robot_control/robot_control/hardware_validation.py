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
Fase de validacion de hardware simulado.

Ejecuta tres bloques de pruebas sobre el robot en Gazebo e imprime un informe
APROBADO / FALLA por cada prueba:

  1. SENSORES     /scan, /odom y /joint_states: frecuencia y datos coherentes.
  2. ACTUADORES   W / S / A / D publicados en /cmd_vel: velocidad esperada vs
                  medida en cada rueda y movimiento real del chasis.
  3. CONTROLADOR  metas en varias direcciones al controlador de posicion y
                  prueba de cancelacion a mitad de camino.

Los tiempos se miden en tiempo de simulacion (use_sim_time), asi el resultado
no depende de la velocidad del PC.

Uso (con la simulacion y el controlador ya corriendo):
  ros2 run robot_control hardware_validation
"""

import math
import time

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

WHEEL_RADIUS = 0.05       # m, igual que el XACRO
WHEEL_SEPARATION = 0.34   # m, igual que el XACRO
WHEELS = ('left_wheel_joint', 'right_wheel_joint')

# (nombre, v [m/s], w [rad/s], duracion [s])
MOTION_PHASES = [
    ('PARADA', 0.0, 0.0, 2.0),
    ('W adelante', 0.30, 0.0, 3.0),
    ('PARADA', 0.0, 0.0, 2.0),
    ('S atras', -0.30, 0.0, 3.0),
    ('PARADA', 0.0, 0.0, 2.0),
    ('A giro izq', 0.0, 0.80, 3.0),
    ('PARADA', 0.0, 0.0, 2.0),
    ('D giro der', 0.0, -0.80, 3.0),
    ('PARADA', 0.0, 0.0, 2.0),
]

# Metas relativas a la pose inicial (m), en el marco odom
GOALS = [
    ('adelante', 1.0, 0.0),
    ('izquierda', 1.0, 1.0),
    ('atras-izq', 0.0, 1.0),
    ('origen', 0.0, 0.0),
    ('detras', -0.8, 0.0),
    ('origen', 0.0, 0.0),
]


def yaw_of(q):
    """Yaw (rad) de un cuaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    """Normaliza un angulo a [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_seconds(stamp):
    """builtin_interfaces/Time -> segundos."""
    return stamp.sec + stamp.nanosec * 1e-9


class RateCounter:
    """Frecuencia media de un topico a partir de las marcas de tiempo."""

    def __init__(self):
        self.count = 0
        self.first = None
        self.last = None

    def add(self, stamp):
        t = stamp_seconds(stamp)
        if self.first is None:
            self.first = t
        self.last = t
        self.count += 1

    def reset(self):
        self.__init__()

    @property
    def rate(self):
        if self.count < 2 or self.last <= self.first:
            return 0.0
        return (self.count - 1) / (self.last - self.first)


class HardwareValidation(Node):
    """Nodo que ejecuta la validacion y acumula los resultados."""

    def __init__(self):
        super().__init__(
            'hardware_validation',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('run_sensors', True)
        self.declare_parameter('run_actuators', True)
        self.declare_parameter('run_controller', True)
        self.declare_parameter('expected_scan_rate', 10.0)
        self.declare_parameter('min_free_space', 1.8)
        self.declare_parameter('goal_tolerance', 0.06)
        self.declare_parameter('goal_timeout', 40.0)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(
            Point, '/position_controller/goal_xy', 10)
        self.cancel_cli = self.create_client(
            Trigger, '/position_controller/cancel')
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(
            JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(
            String, '/position_controller/status', self.on_status, 10)

        self.scan = None
        self.scan_rate = RateCounter()
        self.odom_rate = RateCounter()
        self.joint_rate = RateCounter()
        self.joint_names = set()
        self.pose = None              # (x, y, yaw) en odom
        self.wheel_vel = None         # (izquierda, derecha) rad/s
        self.status = ''
        self.results = []             # (bloque, prueba, ok, detalle)

    # ------------------------------------------------------------ callbacks
    def on_scan(self, msg):
        self.scan = msg
        self.scan_rate.add(msg.header.stamp)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.odom_rate.add(msg.header.stamp)

    def on_joints(self, msg):
        self.joint_rate.add(msg.header.stamp)
        self.joint_names.update(msg.name)
        try:
            self.wheel_vel = tuple(
                msg.velocity[msg.name.index(j)] for j in WHEELS)
        except (ValueError, IndexError):
            pass

    def on_status(self, msg):
        self.status = msg.data

    # ------------------------------------------------------------ utilidades
    def now(self):
        """Tiempo de simulacion en segundos."""
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, seconds, cmd=None):
        """Procesa callbacks durante `seconds` de simulacion."""
        t0 = self.now()
        wall_limit = time.monotonic() + 5.0 * seconds + 10.0
        while (rclpy.ok() and self.now() - t0 < seconds
               and time.monotonic() < wall_limit):
            if cmd is not None:
                self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)

    def record(self, block, name, ok, detail):
        self.results.append((block, name, ok, detail))
        self.get_logger().info(
            '[%s] %s %s: %s' % (block, 'OK   ' if ok else 'FALLA', name,
                                detail))

    def stop_robot(self):
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_data(self, timeout=60.0):
        """Espera reloj de simulacion, /odom, /scan y /joint_states."""
        self.get_logger().info('Esperando simulacion (/clock, /odom, '
                               '/scan, /joint_states)...')
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.now() > 0.0 and self.pose is not None
                    and self.scan is not None
                    and self.wheel_vel is not None):
                return True
        return False

    def other_cmd_vel_publishers(self):
        """Nodos (ajenos al controlador) que publican en /cmd_vel."""
        allowed = {self.get_name(), 'position_controller'}
        infos = self.get_publishers_info_by_topic('/cmd_vel')
        return sorted({i.node_name for i in infos} - allowed)

    def free_space(self):
        """Distancia al obstaculo mas cercano segun el LiDAR."""
        s = self.scan
        vals = [r for r in s.ranges
                if math.isfinite(r) and s.range_min <= r <= s.range_max]
        return min(vals) if vals else math.inf

    # ------------------------------------------------------------ 1 sensores
    def test_sensors(self):
        block = 'SENSORES'
        for counter in (self.scan_rate, self.odom_rate, self.joint_rate):
            counter.reset()
        self.spin_for(5.0)

        exp = float(self.get_parameter('expected_scan_rate').value)
        rate = self.scan_rate.rate
        self.record(block, 'LiDAR frecuencia', abs(rate - exp) <= 0.2 * exp,
                    '%.1f Hz (esperado %.1f +-20%%)' % (rate, exp))

        s = self.scan
        n = len(s.ranges)
        span = math.degrees(s.angle_increment * n)
        self.record(block, 'LiDAR muestras y cobertura',
                    n >= 360 and span >= 355.0,
                    '%d rayos, %.0f grados, rango %.2f-%.2f m'
                    % (n, span, s.range_min, s.range_max))

        valid = [r for r in s.ranges
                 if math.isfinite(r) and s.range_min <= r <= s.range_max]
        frac = len(valid) / n if n else 0.0
        self.record(block, 'LiDAR lecturas validas', frac >= 0.9,
                    '%.1f %% con impacto dentro de rango' % (100.0 * frac))

        self_hits = sum(1 for r in valid if r < 0.30)
        self.record(block, 'LiDAR sin auto-deteccion', self_hits == 0,
                    '%d lecturas a menos de 0.30 m' % self_hits)

        orate = self.odom_rate.rate
        self.record(block, 'Odometria frecuencia', orate >= 10.0,
                    '%.1f Hz (minimo 10)' % orate)

        jrate = self.joint_rate.rate
        has_wheels = set(WHEELS) <= self.joint_names
        self.record(block, 'Encoders de ruedas (/joint_states)',
                    has_wheels and jrate >= 10.0,
                    '%.1f Hz, juntas: %s'
                    % (jrate, ', '.join(sorted(self.joint_names))))

    # ------------------------------------------------------------ 2 actuadores
    def run_phase(self, v, w, duration):
        """Publica (v, w) y mide ruedas (2a mitad) y movimiento del chasis."""
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        x0, y0, th0 = self.pose
        yaw_acc, last_yaw = 0.0, th0
        sum_l = sum_r = 0.0
        n = 0
        t0 = self.now()
        wall_limit = time.monotonic() + 5.0 * duration + 10.0
        while (rclpy.ok() and self.now() - t0 < duration
               and time.monotonic() < wall_limit):
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)
            yaw_acc += wrap(self.pose[2] - last_yaw)
            last_yaw = self.pose[2]
            if self.now() - t0 > duration / 2.0:
                sum_l += self.wheel_vel[0]
                sum_r += self.wheel_vel[1]
                n += 1
        x1, y1, _ = self.pose
        dx, dy = x1 - x0, y1 - y0
        forward = dx * math.cos(th0) + dy * math.sin(th0)
        n = max(n, 1)
        return sum_l / n, sum_r / n, forward, math.degrees(yaw_acc)

    def test_actuators(self):
        block = 'ACTUADORES'
        first_stop = True
        for name, v, w, duration in MOTION_PHASES:
            exp_l = (v - w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
            exp_r = (v + w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
            wl, wr, fwd, yaw = self.run_phase(v, w, duration)
            detail = ('ruedas esp %+.2f/%+.2f med %+.2f/%+.2f rad/s, '
                      'avance %+.3f m, giro %+.1f grados'
                      % (exp_l, exp_r, wl, wr, fwd, yaw))
            if name == 'PARADA':
                if first_stop:   # reposo real (las demas incluyen frenado)
                    first_stop = False
                    self.record(block, 'Reposo sin deriva',
                                abs(fwd) < 0.005 and abs(yaw) < 1.0, detail)
                continue
            wheels_ok = (abs(wl - exp_l) <= 0.1 * abs(exp_l) + 0.05
                         and abs(wr - exp_r) <= 0.1 * abs(exp_r) + 0.05)
            if v > 0:
                body_ok = fwd > 0.5
            elif v < 0:
                body_ok = fwd < -0.5
            elif w > 0:
                body_ok = yaw > 60.0
            else:
                body_ok = yaw < -60.0
            self.record(block, name, wheels_ok and body_ok, detail)
        self.stop_robot()

    # ------------------------------------------------------------ 3 control
    def test_controller(self):
        block = 'CONTROLADOR'
        if self.count_subscribers('/position_controller/goal_xy') == 0:
            self.record(block, 'Controlador disponible', False,
                        'position_controller no esta corriendo')
            return
        tol = float(self.get_parameter('goal_tolerance').value)
        timeout = float(self.get_parameter('goal_timeout').value)
        x0, y0, _ = self.pose
        for name, dx, dy in GOALS:
            gx, gy = x0 + dx, y0 + dy
            self.status = ''
            goal = Point()
            goal.x, goal.y = gx, gy
            self.goal_pub.publish(goal)
            t0 = self.now()
            wall_limit = time.monotonic() + 3.0 * timeout
            while (rclpy.ok() and self.now() - t0 < timeout
                   and time.monotonic() < wall_limit):
                rclpy.spin_once(self, timeout_sec=0.02)
                if self.status not in ('', 'moving'):
                    break
            elapsed = self.now() - t0
            self.spin_for(1.0)
            fx, fy, _ = self.pose
            err = math.hypot(gx - fx, gy - fy)
            status = self.status or 'TIMEOUT'
            self.record(block, 'Meta %s (%+.1f, %+.1f)' % (name, dx, dy),
                        status == 'reached' and err <= tol,
                        'error %.1f cm en %.1f s, estado %s'
                        % (100.0 * err, elapsed, status))
        self.test_cancel(block, x0, y0)

    def test_cancel(self, block, x0, y0):
        goal = Point()
        goal.x, goal.y = x0 + 1.0, y0
        self.goal_pub.publish(goal)
        self.spin_for(2.0)
        if not self.cancel_cli.wait_for_service(timeout_sec=2.0):
            self.record(block, 'Cancelacion', False,
                        'servicio /position_controller/cancel no disponible')
            return
        future = self.cancel_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        res = future.result()
        self.spin_for(1.5)
        px, py, _ = self.pose
        self.spin_for(2.0)
        qx, qy, _ = self.pose
        moved = math.hypot(qx - px, qy - py)
        ok = res is not None and res.success and moved < 0.005
        self.record(block, 'Cancelacion a mitad de camino', ok,
                    'servicio success=%s, movimiento posterior %.1f mm'
                    % (res.success if res else None, 1000.0 * moved))

    # ------------------------------------------------------------ informe
    def report(self):
        passed = sum(1 for r in self.results if r[2])
        total = len(self.results)
        lines = ['', '=' * 72,
                 ' INFORME DE VALIDACION DE HARDWARE SIMULADO',
                 '=' * 72]
        block = None
        for b, name, ok, detail in self.results:
            if b != block:
                block = b
                lines.append('')
                lines.append('[%s]' % b)
            lines.append('  %s  %-34s %s'
                         % ('OK   ' if ok else 'FALLA', name, detail))
        lines.append('')
        lines.append('-' * 72)
        verdict = 'APROBADO' if total and passed == total else 'CON FALLAS'
        lines.append(' RESULTADO: %s  (%d/%d pruebas)'
                     % (verdict, passed, total))
        lines.append('-' * 72)
        print('\n'.join(lines), flush=True)
        return total > 0 and passed == total

    # ------------------------------------------------------------ principal
    def run(self):
        if not self.wait_for_data():
            self.record('PREPARACION', 'Datos de simulacion', False,
                        'no llegan /clock, /odom, /scan o /joint_states')
            return self.report()
        others = self.other_cmd_vel_publishers()
        if others:
            self.get_logger().warn(
                'Otros nodos publican en /cmd_vel: %s. No los uses durante '
                'la validacion.' % ', '.join(others))

        if self.get_parameter('run_sensors').value:
            self.get_logger().info('--- 1/3 SENSORES ---')
            self.test_sensors()

        needs_motion = (self.get_parameter('run_actuators').value
                        or self.get_parameter('run_controller').value)
        if needs_motion:
            free = self.free_space()
            min_free = float(self.get_parameter('min_free_space').value)
            if free < min_free:
                self.record('PREPARACION', 'Espacio libre', False,
                            'obstaculo a %.2f m (< %.1f m); mueve el robot '
                            'a una zona despejada' % (free, min_free))
                return self.report()

        if self.get_parameter('run_actuators').value:
            self.get_logger().info('--- 2/3 ACTUADORES (W/S/A/D) ---')
            self.test_actuators()

        if self.get_parameter('run_controller').value:
            self.get_logger().info('--- 3/3 CONTROLADOR DE POSICION ---')
            self.test_controller()

        self.stop_robot()
        return self.report()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareValidation()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
