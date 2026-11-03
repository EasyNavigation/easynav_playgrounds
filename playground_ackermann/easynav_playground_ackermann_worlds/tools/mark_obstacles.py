# Copyright 2026 Intelligent Robotics Lab
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

"""Add an "occupancy" layer to the track NavMap: the cells under the raceway's walls are lethal.

Usage: mark_obstacles.py <Raceway.obj> <in.navmap> <out.navmap>

The walls are the steep faces (normal less than 45 deg from horizontal) of the barrier materials,
sampled densely. A NavCel is lethal when one of those points lies over it, between 0.3 m below
and 2 m above it: the bridge over the track is higher, and is not an obstacle.
"""
import collections
import sys

from navmap_ros_interfaces.msg import NavMapLayer
from navmap_tools.resample import LETHAL, read_navmap, write_navmap
import numpy as np
from track_cloud import load, sample

BARRIERS = {'RumbleStrip_Barrier', 'Wall', 'TireStack', 'Railing', 'Fence', 'Concrete', 'Stands',
            'Trim', 'Bridge', 'Roof', 'Window'}
DENSITY = 50.0  # points / m2
BELOW, ABOVE = 0.3, 2.0


def wall_points(obj):
    vertices, faces = load(obj, BARRIERS)
    a, b, c = (vertices[faces[:, i]] for i in range(3))
    n = np.cross(b - a, c - a)
    steep = np.abs(n[:, 2]) < np.linalg.norm(n, axis=1) * np.cos(np.radians(45))
    return sample(vertices, faces[steep], DENSITY, np.random.default_rng(0))


def lethal_cells(msg, points):
    px, py, pz = (np.array(getattr(msg, f'positions_{k}')) for k in 'xyz')
    tri = np.stack([msg.navcels_v0, msg.navcels_v1, msg.navcels_v2], axis=1)
    buckets = collections.defaultdict(list)
    for i, (x, y) in enumerate(np.floor(points[:, :2]).astype(int)):
        buckets[(x, y)].append(i)
    lethal = np.zeros(len(tri), bool)
    for c, (i0, i1, i2) in enumerate(tri):
        xs, ys, zs = px[[i0, i1, i2]], py[[i0, i1, i2]], pz[[i0, i1, i2]]
        cand = [i for bx in range(int(np.floor(xs.min())), int(np.floor(xs.max())) + 1)
                for by in range(int(np.floor(ys.min())), int(np.floor(ys.max())) + 1)
                for i in buckets.get((bx, by), ())]
        if not cand:
            continue
        p = points[cand]
        p = p[(p[:, 2] > zs.min() - BELOW) & (p[:, 2] < zs.max() + ABOVE)]
        if not len(p):
            continue
        # Barycentric test in the horizontal plane
        d = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
        if abs(d) < 1e-12:
            continue
        l0 = ((ys[1] - ys[2]) * (p[:, 0] - xs[2]) + (xs[2] - xs[1]) * (p[:, 1] - ys[2])) / d
        l1 = ((ys[2] - ys[0]) * (p[:, 0] - xs[2]) + (xs[0] - xs[2]) * (p[:, 1] - ys[2])) / d
        lethal[c] = np.any((l0 >= 0) & (l1 >= 0) & (l0 + l1 <= 1))
    return lethal


if __name__ == '__main__':
    obj, src, dst = sys.argv[1:4]
    msg = read_navmap(src)
    lethal = lethal_cells(msg, wall_points(obj))
    msg.layers = [layer for layer in msg.layers if layer.name != 'occupancy']
    layer = NavMapLayer()
    layer.header.frame_id = msg.header.frame_id
    layer.name = 'occupancy'
    layer.type = NavMapLayer.U8
    layer.description = 'the raceway walls'
    layer.has_unknown, layer.unknown_value = True, 255.0
    layer.data_u8 = np.where(lethal, LETHAL, 0).astype(np.uint8).tobytes()
    msg.layers.append(layer)
    write_navmap(msg, dst)
    print(f'{lethal.sum()} of {len(lethal)} NavCels lethal')
