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

import math

from robot_control.go_to_goal import (
    ControllerParams, GoToGoal, normalize_angle, yaw_from_quaternion)


def simulate(start, goal, goal_yaw=None, align=False,
             wheel_accel=1.0, dt=0.05):
    """Simula un robot diferencial con ruedas de aceleracion limitada."""
    radius, separation = 0.05, 0.34
    controller = GoToGoal(ControllerParams(align_final_yaw=align))
    controller.set_goal(goal[0], goal[1], goal_yaw)
    x, y, yaw = start
    wl = wr = 0.0
    went_backwards = False
    for _ in range(int(150 / dt)):
        v, w, done = controller.step(x, y, yaw)
        if done:
            break
        wl_cmd = (v - w * separation / 2) / radius
        wr_cmd = (v + w * separation / 2) / radius
        step = wheel_accel * dt
        wl += max(-step, min(step, wl_cmd - wl))
        wr += max(-step, min(step, wr_cmd - wr))
        vel = (wl + wr) * radius / 2
        went_backwards = went_backwards or vel < -1e-3
        x += vel * math.cos(yaw) * dt
        y += vel * math.sin(yaw) * dt
        yaw = normalize_angle(yaw + (wr - wl) * radius / separation * dt)
    return controller, x, y, yaw, went_backwards


def test_reaches_goal_ahead():
    controller, x, y, _, _ = simulate((0.0, 0.0, 0.0), (1.0, 0.5))
    assert controller.done
    assert math.hypot(1.0 - x, 0.5 - y) < 0.05


def test_reaches_goal_behind_without_reversing():
    controller, x, y, _, backwards = simulate((0.0, 0.0, 0.0), (-1.0, 0.8))
    assert controller.done
    assert math.hypot(-1.0 - x, 0.8 - y) < 0.05
    assert not backwards


def test_final_orientation():
    _, _, _, yaw, _ = simulate(
        (0.0, 0.0, 0.0), (1.0, 0.5), goal_yaw=math.pi / 2, align=True)
    assert abs(normalize_angle(math.pi / 2 - yaw)) < 0.1


def test_cancel_stops_output():
    controller = GoToGoal()
    controller.set_goal(1.0, 0.0)
    controller.cancel()
    assert controller.step(0.0, 0.0, 0.0) == (0.0, 0.0, True)


def test_yaw_from_quaternion():
    half = math.pi / 4
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half))
    assert abs(yaw - math.pi / 2) < 1e-9
