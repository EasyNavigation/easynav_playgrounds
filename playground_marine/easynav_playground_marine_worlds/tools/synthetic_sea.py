#!/usr/bin/env python3
# Copyright 2026 José Miguel Guerrero
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

"""A synthetic sea: an offline, deterministic sea floor with water, for boats and submarines.

The world of the marine playground: nothing is downloaded and it is the same every time. West is a
beach; the floor deepens to the east (20.9 m at the origin, 48 m at the east edge); there is an islet
that stands out of the water, a shoal, a seamount, a ridge of the floor and sand ripples. z = 0 is the
sea level.

    synthetic_sea.py --out DIR [--radius 500] [--template marine.world.in]

writes in DIR:
  terrain.stl, terrain.obj, terrain.mtl, terrain.png   the floor, for Gazebo (collision and visual)
  world.sdf                                            the Gazebo world: the floor and the water
and prints DIR as its last line. It needs only numpy (and, for the default template, the package's
share directory); no other package.
"""

import argparse
import os
import struct
import sys
import zlib

import numpy as np

# where the world is on Earth is only a label of the Gazebo world (the heights are not from there):
# the Cantabrian coast
LATITUDE, LONGITUDE = 43.3848206, -3.0097979

SHORE_X = -380.0     # m: the waterline on the west edge
SHELF_SLOPE = 0.055  # the floor deepens this many metres per metre to the east
BEACH_SLOPE = 0.12   # and rises this many to the west of the shore

# (x, y, height m, sigma m): the islet, a shoal and a seamount, as Gaussian bumps of the floor
BUMPS = (
    (-140.0, -200.0, 30.0, 28.0),    # the islet: its top is 17 m over the water
    (60.0, -120.0, 20.0, 25.0),      # a shoal whose top is 4 m under the surface
    (160.0, 140.0, 22.0, 50.0),      # a wide seamount, 8 m under the surface
)
RIDGE = (250.0, 10.0, 15.0)          # a north-south ridge of the floor: x, height m, sigma_x m

# the colours of the floor by depth (m under the surface): sand, green blue, deep blue
FLOOR_DEPTHS = np.array([0.0, 3.0, 15.0, 50.0, 150.0])
FLOOR_COLOURS = np.array([[206, 190, 140], [120, 165, 150], [38, 110, 135], [16, 52, 98],
                          [8, 24, 56]], dtype=np.float64)

# the mesh of the water when it has waves (it is drawn by a vertex shader, so its vertices are what the
# wave is made of): spacing in m up to the given distance from the origin; beyond the last, WATER_FAR
WATER_STEPS = ((30.0, 1.5), (150.0, 3.0), (450.0, 6.0), (750.0, 12.0))
WATER_FAR = 24.0
# how the water looks (fragment shader): colours seen through and the opacity
WATER_ALPHA = 0.7
WATER_SHALLOW = '0.05 0.30 0.50 1.0'
WATER_DEEP = '0.02 0.12 0.25 1.0'

WATER_DENSITY = 1000.0    # kg/m3: what the gz-sim marine demos (and their AUV) are balanced for
AIR_DENSITY = 1.2
# visibility_flags of the water's visuals: the ray sensors (lidars, echosounder, sonar; their
# visibility_mask in tools/marine_sensors.py) do not see the surface, which is a drawing
WATER_VISIBILITY = 0x00000002


def height(x, y):
    """Height (m over the sea level) of the sea floor, or the land, at local metres (x east, y north)."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    z = np.where(x < SHORE_X, BEACH_SLOPE * (SHORE_X - x), -SHELF_SLOPE * (x - SHORE_X))
    for bx, by, h, s in BUMPS:
        z = z + h * np.exp(-((x - bx) ** 2 + (y - by) ** 2) / (2.0 * s * s))
    rx, rh, rs = RIDGE
    z = z + rh * np.exp(-((x - rx) ** 2) / (2.0 * rs * rs))
    return z + 0.3 * np.sin(x / 9.0 + 0.3 * y / 9.0)     # sand ripples


def colours(z):
    """RGB (uint8) of every point: the floor by depth (sand where it is shallow, deep blue where it
    is deep), the land as sand and, higher up, scrub."""
    z = np.asarray(z, dtype=np.float64)
    depth = np.maximum(-z, 0.0)
    floor = np.stack([np.interp(depth, FLOOR_DEPTHS, FLOOR_COLOURS[:, k]) for k in range(3)], axis=-1)
    sand = np.array([214.0, 200.0, 150.0])
    scrub = np.array([96.0, 120.0, 70.0])
    t = np.clip(z / 12.0, 0.0, 1.0)[..., None]
    on_land = sand * (1.0 - t) + scrub * t
    return np.clip(np.rint(np.where((z > 0.0)[..., None], on_land, floor)), 0, 255).astype(np.uint8)


def write_png(path, rgb):
    """Write an (h, w, 3) uint8 array as a PNG (row 0 on top), with no imaging library."""
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[r].tobytes() for r in range(h))

    def chunk(kind, data):
        body = kind + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body) & 0xffffffff)

    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def write_terrain(directory, radius, terrain_resolution, texture_resolution=1.0):
    """Write terrain.stl (collision), terrain.obj + .mtl + .png (visual) in `directory`; return the
    half-side of the square actually written (`radius` rounded up to a whole number of cells), in
    the map frame (x east, y north, z over the sea level)."""
    os.makedirs(directory, exist_ok=True)
    cells = int(np.ceil(radius / terrain_resolution))
    half = cells * terrain_resolution
    n = 2 * cells + 1
    coords = np.arange(n) * terrain_resolution - half
    gx, gy = np.meshgrid(coords, coords)
    z = height(gx, gy)
    verts = np.stack([gx.ravel(), gy.ravel(), z.ravel()], axis=1).astype(np.float32)

    j, i = np.mgrid[0:n - 1, 0:n - 1]
    v00 = (j * n + i).ravel()
    v10, v01, v11 = v00 + 1, v00 + n, v00 + n + 1
    tris = np.empty((v00.size * 2, 3), dtype=np.int64)
    tris[0::2] = np.stack([v00, v10, v11], axis=1)
    tris[1::2] = np.stack([v00, v11, v01], axis=1)

    # collision: binary STL
    p = verts[tris]
    normal = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]))
    rec['n'] = normal
    rec['v'] = p.reshape(-1, 9)
    with open(os.path.join(directory, 'terrain.stl'), 'wb') as f:
        f.write(b'marine playground terrain'.ljust(80, b'\0'))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())

    # visual: OBJ with UVs and smooth normals; the texture covers the square, row 0 is the north edge
    lo, hi = np.maximum(np.arange(n) - 1, 0), np.minimum(np.arange(n) + 1, n - 1)
    dzdx = (z[:, hi] - z[:, lo]) / ((hi - lo) * terrain_resolution)[None, :]
    dzdy = (z[hi, :] - z[lo, :]) / ((hi - lo) * terrain_resolution)[:, None]
    length = np.sqrt(dzdx ** 2 + dzdy ** 2 + 1.0)
    vn = np.stack([-dzdx / length, -dzdy / length, 1.0 / length], axis=-1).reshape(-1, 3)
    uv = np.stack([((gx + half) / (2 * half)).ravel(), ((gy + half) / (2 * half)).ravel()], axis=1)
    lines = ['mtllib terrain.mtl', 'usemtl terrain']
    lines += ['v %.6g %.6g %.6g' % tuple(v) for v in verts]
    lines += ['vt %.6g %.6g' % tuple(t) for t in uv]
    lines += ['vn %.6g %.6g %.6g' % tuple(v) for v in vn]
    lines += ['f %d/%d/%d %d/%d/%d %d/%d/%d' % (a, a, a, b, b, b, c, c, c) for a, b, c in tris + 1]
    with open(os.path.join(directory, 'terrain.obj'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    with open(os.path.join(directory, 'terrain.mtl'), 'w') as f:
        f.write('newmtl terrain\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\nillum 1\nmap_Kd terrain.png\n')
    texels = int(np.ceil(2 * half / texture_resolution))
    centres = (np.arange(texels) + 0.5) * (2 * half / texels) - half
    tx, ty = np.meshgrid(centres, centres)
    write_png(os.path.join(directory, 'terrain.png'), colours(height(tx, ty))[::-1])
    return half


def water_coordinates(extent):
    """Increasing coordinates, m, from -extent to extent: close together near 0 and spread out far away
    (WATER_STEPS), the last one exactly at the extent."""
    positive, current = [0.0], 0.0
    while current < extent - 1e-9:
        step = next((s for limit, s in WATER_STEPS if current < limit), WATER_FAR)
        current = min(current + step, extent)
        positive.append(current)
    positive = np.array(positive)
    return np.concatenate([-positive[:0:-1], positive])


def write_water_mesh(directory, extent):
    """Write water.obj: a flat square of side 2 x extent at z = 0, facing up, with texture coordinates
    over the whole square (the bump map of the shader). Returns the number of vertices."""
    coords = water_coordinates(extent)
    n = len(coords)
    gx, gy = np.meshgrid(coords, coords)
    j, i = np.mgrid[0:n - 1, 0:n - 1]
    v00 = (j * n + i).ravel() + 1
    v10, v01, v11 = v00 + 1, v00 + n, v00 + n + 1
    faces = np.empty((v00.size * 2, 3), dtype=np.int64)
    faces[0::2] = np.stack([v00, v10, v11], axis=1)
    faces[1::2] = np.stack([v00, v11, v01], axis=1)
    lines = ['v %.6g %.6g 0' % (x, y) for x, y in zip(gx.ravel(), gy.ravel())]
    lines += ['vt %.6g %.6g' % ((x + extent) / (2 * extent), (y + extent) / (2 * extent))
              for x, y in zip(gx.ravel(), gy.ravel())]
    lines.append('vn 0 0 1')
    lines += ['f %d/%d/1 %d/%d/1 %d/%d/1' % (a, a, b, b, c, c) for a, b, c in faces]
    with open(os.path.join(directory, 'water.obj'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return n * n


def shader_visual(shader):
    """The SDF of the visual that draws the water with waves: the mesh, and the ShaderParam plugin that
    gives the shaders their parameters (and the time of the simulation, as `t`). `shader`: the folder
    of the materials (marine_waves/materials), the parameters of the waves (name, type, value) and the
    URI of the mesh."""
    programs = os.path.join(shader['dir'], 'programs')
    textures = os.path.join(shader['dir'], 'textures')

    def param(stage, name, kind, value, arg=None):
        text = '<param><shader>%s</shader><name>%s</name>' % (stage, name)
        if kind:
            text += '<type>%s</type>' % kind
        if value is not None:
            text += '<value>%s</value>' % value
        if arg is not None:
            text += '<arg>%s</arg>' % arg          # the texture coordinate set of a cube map
        return text + '</param>'

    lines = [param('vertex', *p) for p in shader['params']]
    lines += [param('vertex', 'rescale', 'float', '0.5'),
              param('vertex', 'bumpScale', 'float_array', '37 37'),
              param('vertex', 'bumpSpeed', 'float_array', '0.01 0.01'),
              param('vertex', 'camera_position_object_space', None, None),
              param('vertex', 'worldviewproj_matrix', None, None),
              param('vertex', 't', None, 'TIME'),
              param('fragment', 'hdrMultiplier', 'float', '0.4'),
              param('fragment', 'fresnelPower', 'float', '5.0'),
              param('fragment', 'alpha', 'float', '%g' % shader.get('alpha', WATER_ALPHA)),
              param('fragment', 'shallowColor', 'float_array', WATER_SHALLOW),
              param('fragment', 'deepColor', 'float_array', WATER_DEEP),
              param('fragment', 'cubeMap', 'texture_cube', os.path.join(textures, 'skybox_lowres.dds'), 1),
              param('fragment', 'bumpMap', 'texture', os.path.join(textures, 'wave_normals.dds'))]
    body = '\n                    '.join(lines)
    return f"""
                <visual name="waves">
                    <cast_shadows>false</cast_shadows>
                    <visibility_flags>{WATER_VISIBILITY}</visibility_flags>
                    <geometry><mesh><uri>{shader['mesh']}</uri></mesh></geometry>
                    <plugin filename="gz-sim-shader-param-system" name="gz::sim::systems::ShaderParam">
                        <shader language="glsl">
                            <vertex>{os.path.join(programs, 'GerstnerWaves_vs_330.glsl')}</vertex>
                            <fragment>{os.path.join(programs, 'GerstnerWaves_fs_330.glsl')}</fragment>
                        </shader>
                    {body}
                    </plugin>
                </visual>"""


def water_sdf(half, shader=None):
    """The SDF that makes the sea water: a translucent surface at z = 0 (seen from above and from
    below) and the buoyancy system (sea water below the surface, air above). The launch file adds the
    vehicle to its <enable> list. With `shader` (see shader_visual) the surface seen from above is the
    wave field drawn by a vertex shader; the flat one is kept to be seen from below. The buoyancy stays
    that of a flat sea: the waves are applied to the vehicles as forces (tools/wave_node.py)."""
    size = 3.0 * half
    above = shader_visual(shader) if shader else f"""
                <visual name="water">
                    <cast_shadows>false</cast_shadows>
                    <visibility_flags>{WATER_VISIBILITY}</visibility_flags>
                    <geometry><plane><normal>0 0 1</normal>
                        <size>{size:.6g} {size:.6g}</size></plane></geometry>
                    <material>
                        <ambient>0.05 0.25 0.45 0.4</ambient>
                        <diffuse>0.05 0.25 0.45 0.4</diffuse>
                        <specular>0.4 0.4 0.4 0.4</specular>
                    </material>
                </visual>"""
    return f"""
        <!-- the sea: a surface, and buoyancy (generated) -->
        <model name="marine_water">
            <static>true</static>
            <pose>0 0 0 0 0 0</pose>
            <link name="surface">{above}
                <!-- the same seen from below (a plane is drawn on one side only) -->
                <visual name="water_from_below">
                    <cast_shadows>false</cast_shadows>
                    <visibility_flags>{WATER_VISIBILITY}</visibility_flags>
                    <geometry><plane><normal>0 0 -1</normal>
                        <size>{size:.6g} {size:.6g}</size></plane></geometry>
                    <material>
                        <ambient>0.04 0.2 0.4 0.85</ambient>
                        <diffuse>0.04 0.2 0.4 0.85</diffuse>
                        <specular>0.0 0.0 0.0 0.85</specular>
                    </material>
                </visual>
            </link>
        </model>
        <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
            <graded_buoyancy>
                <default_density>{WATER_DENSITY:g}</default_density>
                <density_change>
                    <above_depth>0</above_depth>
                    <density>{AIR_DENSITY:g}</density>
                </density_change>
            </graded_buoyancy>
        </plugin>
"""


def make(out, radius=500.0, terrain_resolution=4.0, template=None, shader=None):
    """Write the world of the synthetic sea in the directory `out` (see the module docstring) and
    return the half-side of the square, m. `shader`: the water with waves (see shader_visual, without
    the mesh, which is written here); without it the water is flat."""
    if template is None:
        from ament_index_python.packages import get_package_share_directory
        template = os.path.join(get_package_share_directory('easynav_playground_marine_worlds'),
                                'worlds', 'marine.world.in')
    with open(template) as f:
        text = f.read()
    if '@TERRAIN_DIR@' not in text or '@WATER@' not in text:
        raise ValueError('%s has no @TERRAIN_DIR@ / @WATER@ placeholder' % template)
    half = write_terrain(out, radius, terrain_resolution)
    if shader:
        write_water_mesh(out, 1.5 * half)
        shader = dict(shader, mesh='file://' + os.path.join(os.path.abspath(out), 'water.obj'))
    text = (text.replace('@LATITUDE@', repr(LATITUDE)).replace('@LONGITUDE@', repr(LONGITUDE))
            .replace('@ELEVATION@', '0').replace('@TERRAIN_DIR@', os.path.abspath(out))
            .replace('@WATER@', water_sdf(half, shader)))
    with open(os.path.join(out, 'world.sdf'), 'w') as f:
        f.write(text)
    return half


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', required=True, help='directory to write the world in')
    p.add_argument('--radius', type=float, default=500.0, help='half-side of the square, m (500)')
    p.add_argument('--terrain-resolution', type=float, default=4.0,
                   help="m between vertices of Gazebo's floor (4)")
    p.add_argument('--template', default=None, help='world template (default: marine.world.in)')
    a = p.parse_args(argv)
    if a.radius < 100.0:
        p.error('--radius must be at least 100 m: the beach and the islet are within 400 m')
    out = os.path.abspath(os.path.expanduser(a.out))
    half = make(out, a.radius, a.terrain_resolution, a.template)
    print('synthetic sea: %g m x %g m, floor %.1f m at the origin, islet %.1f m over the water'
          % (2 * half, 2 * half, float(height(0.0, 0.0)), float(height(-140.0, -200.0))))
    print(out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
