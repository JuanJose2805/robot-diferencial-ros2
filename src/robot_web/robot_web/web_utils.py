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

"""Funciones auxiliares del puente web, sin dependencias de ROS."""

from datetime import datetime
import math
import os
import re
import xml.etree.ElementTree as ET

_VALID_NAME = re.compile(r'[^A-Za-z0-9_-]+')


def yaw_from_quaternion(x, y, z, w):
    """Devuelve el angulo de giro (yaw) en radianes de un cuaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    """Devuelve (x, y, z, w) de un giro puro alrededor del eje z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def sanitize_map_name(name, now=None):
    """
    Convierte el nombre que escribe el usuario en un nombre de archivo seguro.

    Solo deja letras, numeros, '_' y '-', para que no se pueda escribir fuera
    de la carpeta de mapas (por ejemplo con '../'). Si queda vacio, usa la
    fecha y hora actual.
    """
    clean = _VALID_NAME.sub('_', (name or '').strip()).strip('_')
    if not clean:
        now = now or datetime.now()
        clean = now.strftime('mapa_%Y%m%d_%H%M%S')
    return clean[:64]


def map_path(maps_dir, name):
    """Ruta absoluta (sin extension) donde se guarda un mapa."""
    return os.path.join(os.path.abspath(os.path.expanduser(maps_dir)),
                        sanitize_map_name(name))


def _pose2d(element):
    """Lee <pose>x y z roll pitch yaw</pose> y devuelve (x, y, yaw)."""
    node = element.find('pose') if element is not None else None
    if node is None or not (node.text or '').strip():
        return 0.0, 0.0, 0.0
    values = [float(v) for v in node.text.split()] + [0.0] * 6
    return values[0], values[1], values[5]


def _compose(a, b):
    """Pose b expresada en el marco a -> pose en el marco del mundo (2D)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            a[2] + b[2])


def compose_pose(a, b):
    """Pose (x, y, yaw) 'b', dada en el marco de 'a', vista desde fuera."""
    return _compose(a, b)


def _color(visual):
    for tag in ('material/diffuse', 'material/ambient'):
        node = visual.find(tag)
        if node is not None and node.text:
            return [round(float(v), 3) for v in node.text.split()[:3]]
    return [0.6, 0.6, 0.6]


def parse_world_layout(sdf_text):
    """
    Extrae del .world las figuras para dibujar el plano 2D de la arena.

    Recorre los modelos definidos en el archivo (los <include> como el sol o
    el suelo se ignoran) y devuelve una lista de diccionarios:
      {'type': 'box', 'name', 'x', 'y', 'yaw', 'sx', 'sy', 'color'}
      {'type': 'cylinder', 'name', 'x', 'y', 'radius', 'color'}
    con posiciones en metros del marco del mundo y color RGB entre 0 y 1.
    """
    root = ET.fromstring(sdf_text)
    world = root.find('world')
    shapes = []
    for model in (world.findall('model') if world is not None else []):
        model_pose = _pose2d(model)
        for link in model.findall('link'):
            link_pose = _compose(model_pose, _pose2d(link))
            for visual in link.findall('visual'):
                x, y, yaw = _compose(link_pose, _pose2d(visual))
                base = {'name': model.get('name', ''),
                        'x': round(x, 4), 'y': round(y, 4),
                        'color': _color(visual)}
                box = visual.find('geometry/box/size')
                cyl = visual.find('geometry/cylinder/radius')
                if box is not None:
                    sx, sy = [float(v) for v in box.text.split()[:2]]
                    shapes.append(dict(base, type='box', yaw=round(yaw, 4),
                                       sx=sx, sy=sy))
                elif cyl is not None:
                    shapes.append(dict(base, type='cylinder',
                                       radius=float(cyl.text)))
    return shapes


def scan_to_points(pose, angle_min, angle_increment, ranges, range_min,
                   range_max):
    """
    Pasa las lecturas de un LaserScan a puntos (x, y) del mapa.

    'pose' es (x, y, yaw) del LiDAR en el mapa en el instante del scan.
    Se descartan las lecturas fuera de rango (inf, nan o menores al minimo).
    Los puntos se redondean al centimetro para que el mensaje sea liviano.
    """
    x0, y0, yaw = pose
    points = []
    for i, r in enumerate(ranges):
        if not (range_min < r < range_max):
            continue
        a = yaw + angle_min + i * angle_increment
        points.append([round(x0 + r * math.cos(a), 2),
                       round(y0 + r * math.sin(a), 2)])
    return points
