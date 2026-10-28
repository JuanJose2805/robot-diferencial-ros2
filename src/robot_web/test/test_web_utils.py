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

"""Pruebas de las funciones auxiliares del puente web."""

from datetime import datetime
import math
import os

from robot_web.web_utils import (
    compose_pose, map_path, parse_world_layout, quaternion_from_yaw, sanitize_map_name,
    scan_to_points, yaw_from_quaternion)

WORLD = """<sdf version="1.6"><world name="w">
  <include><uri>model://ground_plane</uri></include>
  <model name="caja"><pose>1 2 0.5 0 0 1.5708</pose><link name="l">
    <visual name="v"><geometry><box><size>2 0.5 1</size></box></geometry>
      <material><diffuse>0.9 0.5 0.1 1</diffuse></material></visual>
  </link></model>
  <model name="poste"><pose>-1 0 0 0 0 0</pose><link name="l">
    <pose>0.5 0 0 0 0 0</pose>
    <visual name="v"><geometry><cylinder><radius>0.3</radius>
      <length>1</length></cylinder></geometry></visual>
  </link></model>
</world></sdf>"""


def test_yaw_roundtrip():
    for yaw in (0.0, 0.7, -2.5, math.pi / 2):
        assert math.isclose(
            yaw_from_quaternion(*quaternion_from_yaw(yaw)), yaw, abs_tol=1e-9)


def test_sanitize_keeps_simple_names():
    assert sanitize_map_name('arena_2-b') == 'arena_2-b'


def test_sanitize_blocks_paths():
    assert sanitize_map_name('../../etc/passwd') == 'etc_passwd'
    assert '/' not in sanitize_map_name('a/b c')


def test_sanitize_empty_uses_date():
    now = datetime(2026, 10, 27, 18, 5, 9)
    assert sanitize_map_name('  ', now) == 'mapa_20261027_180509'


def test_map_path_stays_in_folder():
    path = map_path('/tmp/mapas', '../fuera')
    assert os.path.dirname(path) == '/tmp/mapas'


def test_parse_world_layout():
    caja, poste = parse_world_layout(WORLD)
    assert caja['type'] == 'box' and caja['sx'] == 2.0 and caja['sy'] == 0.5
    assert (caja['x'], caja['y']) == (1.0, 2.0)
    assert math.isclose(caja['yaw'], 1.5708)
    assert caja['color'] == [0.9, 0.5, 0.1]
    # La pose del link se suma a la del modelo
    assert poste['type'] == 'cylinder' and poste['radius'] == 0.3
    assert (poste['x'], poste['y']) == (-0.5, 0.0)


def test_scan_to_points():
    # LiDAR en (1, 2) mirando hacia +y; una lectura al frente a 2 m
    pts = scan_to_points((1.0, 2.0, math.pi / 2), 0.0, 0.1,
                         [2.0, float('inf'), 0.1], 0.26, 10.0)
    assert pts == [[1.0, 4.0]]


def test_compose_pose_home_offset():
    # base_footprint esta 0.10 m detras de base_link: con el robot en (0, 0)
    # mirando a +y, el inicio de base_footprint queda en (0, -0.10)
    x, y, yaw = compose_pose((0.0, 0.0, math.pi / 2), (-0.10, 0.0, 0.0))
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, -0.10) and math.isclose(yaw, math.pi / 2)
