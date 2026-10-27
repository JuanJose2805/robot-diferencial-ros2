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

"""Ley de control go-to-goal para un robot diferencial (sin ROS)."""

from dataclasses import dataclass
import math

TURN = 'TURN'
DRIVE = 'DRIVE'
ALIGN = 'ALIGN'
DONE = 'DONE'


def normalize_angle(angle):
    """Normaliza un angulo al rango [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    """Limita value al rango [-limit, limit]."""
    return max(-limit, min(limit, value))


def yaw_from_quaternion(x, y, z, w):
    """Calcula el angulo de giro (yaw) a partir de un cuaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class ControllerParams:
    """Parametros del controlador de posicion."""

    k_rho: float = 0.8
    k_alpha: float = 2.0
    max_linear: float = 0.20
    max_angular: float = 0.8
    min_linear: float = 0.04
    min_angular: float = 0.10
    max_decel: float = 0.04
    max_angular_decel: float = 0.20
    goal_tolerance: float = 0.05
    yaw_tolerance: float = 0.05
    turn_in_place_angle: float = 0.6
    turn_exit_angle: float = 0.15
    align_final_yaw: bool = False


class GoToGoal:
    """
    Maquina de estados GIRAR -> AVANZAR -> ALINEAR -> LISTO.

    Cinematica diferencial: una velocidad lineal v y una angular w. La
    velocidad maxima se reduce al acercarse a la meta segun la deceleracion
    que el robot puede dar, para no pasarse de la meta.
    """

    def __init__(self, params=None):
        self.params = params if params is not None else ControllerParams()
        self.state = DONE
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = None

    def set_goal(self, x, y, yaw=None):
        """Define una meta nueva (yaw opcional, en el marco de odometria)."""
        self.goal_x = x
        self.goal_y = y
        self.goal_yaw = yaw
        self.state = DRIVE

    def cancel(self):
        """Cancela la meta actual."""
        self.state = DONE

    @property
    def done(self):
        """Indica si no hay una meta en curso."""
        return self.state == DONE

    def _turn_speed(self, error, minimum):
        """Velocidad angular con perfil de frenado segun el error."""
        p = self.params
        magnitude = abs(error)
        speed = min(p.k_alpha * magnitude,
                    math.sqrt(2.0 * p.max_angular_decel * magnitude),
                    p.max_angular)
        speed = max(speed, minimum)
        return math.copysign(speed, error)

    def step(self, x, y, yaw):
        """Calcula (v, w, terminado) para la pose actual del robot."""
        p = self.params
        if self.state == DONE:
            return 0.0, 0.0, True

        dx = self.goal_x - x
        dy = self.goal_y - y
        rho = math.hypot(dx, dy)

        if self.state in (TURN, DRIVE):
            if rho <= p.goal_tolerance:
                if p.align_final_yaw and self.goal_yaw is not None:
                    self.state = ALIGN
                else:
                    self.state = DONE
                    return 0.0, 0.0, True
            else:
                alpha = normalize_angle(math.atan2(dy, dx) - yaw)
                if self.state == DRIVE and abs(alpha) > p.turn_in_place_angle:
                    self.state = TURN
                elif self.state == TURN and abs(alpha) < p.turn_exit_angle:
                    self.state = DRIVE

                if self.state == TURN:
                    return 0.0, self._turn_speed(alpha, p.min_angular), False

                speed = min(p.k_rho * rho,
                            math.sqrt(2.0 * p.max_decel * rho),
                            p.max_linear)
                speed = max(speed, p.min_linear) * max(0.0, math.cos(alpha))
                w = clamp(p.k_alpha * alpha, p.max_angular)
                return speed, w, False

        # Estado ALIGN: orientarse segun el yaw de la meta
        error = normalize_angle(self.goal_yaw - yaw)
        if abs(error) <= p.yaw_tolerance:
            self.state = DONE
            return 0.0, 0.0, True
        return 0.0, self._turn_speed(error, p.min_angular), False
