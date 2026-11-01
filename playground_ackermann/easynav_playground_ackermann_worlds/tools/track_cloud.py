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

"""Sample the drivable surface of the Sonoma Raceway mesh into a point cloud (PCD).

Usage: track_cloud.py <Raceway.obj> <out.pcd> [points_per_m2] [materials...]

Only the faces of the given materials (default: Track and Checker, the start line) are
sampled, in the world frame: the model's pose (roll pi/2, z -4.8) is applied.
"""
import sys

import numpy as np


def load(path, materials):
    vertices, faces, material = [], [], None
    with open(path) as f:
        for line in f:
            if line.startswith('v '):
                vertices.append([float(x) for x in line.split()[1:4]])
            elif line.startswith('usemtl'):
                material = line.split()[1]
            elif line.startswith('f ') and material in materials:
                idx = [int(t.split('/')[0]) - 1 for t in line.split()[1:]]
                faces += [[idx[0], idx[i], idx[i + 1]] for i in range(1, len(idx) - 1)]
    v = np.array(vertices)
    # The model's pose in model.sdf: roll pi/2 (y up -> z up), then 4.8 m down
    world = np.c_[v[:, 0], -v[:, 2], v[:, 1] - 4.8]
    return world, np.array(faces)


def sample(vertices, faces, density, rng):
    a, b, c = (vertices[faces[:, i]] for i in range(3))
    areas = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    counts = rng.poisson(areas * density)
    tri = np.repeat(np.arange(len(faces)), counts)
    r1, r2 = rng.random(len(tri)), rng.random(len(tri))
    s = np.sqrt(r1)
    return (1 - s)[:, None] * a[tri] + (s * (1 - r2))[:, None] * b[tri] + \
        (s * r2)[:, None] * c[tri]


def write_pcd(path, points):
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
        f'TYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(points.astype(np.float32).tobytes())


if __name__ == '__main__':
    density = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
    materials = set(sys.argv[4:]) or {'Track', 'Checker'}
    vertices, faces = load(sys.argv[1], materials)
    points = sample(vertices, faces, density, np.random.default_rng(0))
    write_pcd(sys.argv[2], points)
    print(f'{len(faces)} faces, {len(points)} points')
