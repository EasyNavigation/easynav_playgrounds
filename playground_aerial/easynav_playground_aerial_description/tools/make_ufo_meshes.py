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

"""Classic flying saucer, 1 m across: the hull (a lens) and the dome, as binary STL files.

Usage: make_ufo_meshes.py <meshes_dir>

Both are surfaces of revolution of a profile (radius, height) around z, centred on the hull.
"""
import math
import os
import struct
import sys

SEGMENTS = 48
# Hull: a lens with a rim, from the bottom centre to the top centre
HULL = [(0.0, -0.075), (0.16, -0.075), (0.30, -0.055), (0.42, -0.025), (0.50, 0.0),
        (0.44, 0.03), (0.32, 0.055), (0.20, 0.075), (0.0, 0.075)]
# Dome: a cap of a sphere (radius 0.2) on the hull
DOME = [(0.18 * math.cos(a), 0.075 + 0.15 * math.sin(a))
        for a in [math.radians(d) for d in range(0, 91, 10)]]


def lathe(profile):
    triangles = []
    for i in range(SEGMENTS):
        a0, a1 = 2 * math.pi * i / SEGMENTS, 2 * math.pi * (i + 1) / SEGMENTS
        for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
            p00 = (r0 * math.cos(a0), r0 * math.sin(a0), z0)
            p01 = (r0 * math.cos(a1), r0 * math.sin(a1), z0)
            p10 = (r1 * math.cos(a0), r1 * math.sin(a0), z1)
            p11 = (r1 * math.cos(a1), r1 * math.sin(a1), z1)
            for t in ((p00, p01, p11), (p00, p11, p10)):
                if len({t[0], t[1], t[2]}) == 3:  # no degenerate triangles at the axis
                    triangles.append(t)
    return triangles


def write_stl(path, triangles):
    with open(path, 'wb') as f:
        f.write(b'flying saucer'.ljust(80, b' '))
        f.write(struct.pack('<I', len(triangles)))
        for a, b, c in triangles:
            u = [b[k] - a[k] for k in range(3)]
            v = [c[k] - a[k] for k in range(3)]
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            norm = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(struct.pack('<3f', *(x / norm for x in n)))
            for p in (a, b, c):
                f.write(struct.pack('<3f', *p))
            f.write(b'\0\0')


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    for name, profile in (('ufo_hull.stl', HULL), ('ufo_dome.stl', DOME)):
        tris = lathe(profile)
        write_stl(os.path.join(out, name), tris)
        print(name, len(tris), 'triangles')
