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

"""The synthetic sea (tools/synthetic_sea.py): its relief, and the world it writes. Needs only numpy."""

import importlib.util
import os

import struct

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def tool():
    spec = importlib.util.spec_from_file_location(
        'synthetic_sea_under_test', os.path.join(ROOT, 'tools', 'synthetic_sea.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_origin_is_in_deep_water_so_a_boat_and_a_submarine_can_start_there():
    assert -25.0 < float(tool().height(0.0, 0.0)) < -15.0


def test_west_is_a_beach_and_the_floor_deepens_to_the_east():
    h = tool().height
    assert h(-490.0, 0.0) > 5.0                       # land
    assert h(-380.0, 300.0) == pytest.approx(0.0, abs=2.0)     # the waterline
    assert h(450.0, -300.0) < h(0.0, -300.0) < h(-300.0, -300.0)


def test_the_islet_stands_out_of_the_water_and_the_shoal_does_not_reach_it():
    h = tool().height
    assert 14.0 < float(h(-140.0, -200.0)) < 20.0
    assert -6.0 < float(h(60.0, -120.0)) < -2.0       # a boat passes over it, a submarine at 3 m too


def test_it_is_deterministic():
    t = tool()
    x, y = np.meshgrid(np.linspace(-500, 500, 17), np.linspace(-500, 500, 17))
    assert np.array_equal(t.height(x, y), t.height(x, y))


def test_the_world_it_writes_has_the_floor_and_the_water(tmp_path):
    t = tool()
    half = t.make(str(tmp_path), radius=150.0, terrain_resolution=6.0,
                  template=os.path.join(ROOT, 'worlds', 'marine.world.in'))
    assert half == 150.0
    for name in ('terrain.stl', 'terrain.obj', 'terrain.mtl', 'terrain.png', 'world.sdf'):
        assert (tmp_path / name).is_file(), name
    world = (tmp_path / 'world.sdf').read_text()
    assert 'gz-sim-buoyancy-system' in world and 'marine_water' in world
    assert str(tmp_path) in world                     # the floor is the one written next to it
    assert not [w for w in ('@WATER@', '@TERRAIN_DIR@', '@LATITUDE@') if w in world]


def test_the_stl_has_the_triangles_of_its_grid_and_the_floor_heights(tmp_path):
    t = tool()
    t.write_terrain(str(tmp_path), 150.0, 6.0)
    data = (tmp_path / 'terrain.stl').read_bytes()
    count = struct.unpack('<I', data[80:84])[0]
    assert count == 2 * 50 * 50                      # 51 x 51 nodes
    rec = np.frombuffer(data[84:], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]))
    verts = rec['v'].reshape(-1, 3)
    near = verts[(np.abs(verts[:, 0]) < 1e-3) & (np.abs(verts[:, 1]) < 1e-3)]
    assert len(near) and float(near[0, 2]) == pytest.approx(float(t.height(0.0, 0.0)), abs=1e-3)


def test_the_texture_is_a_valid_png_of_the_square(tmp_path):
    t = tool()
    t.write_terrain(str(tmp_path), 150.0, 6.0, texture_resolution=2.0)
    data = (tmp_path / 'terrain.png').read_bytes()
    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    width, height = struct.unpack('>II', data[16:24])
    assert (width, height) == (150, 150)


def test_the_sea_is_made_without_navmap_or_bigmap():
    source = open(os.path.join(ROOT, 'tools', 'synthetic_sea.py')).read().lower()
    assert 'navmap' not in source and 'bigmap' not in source


def test_a_radius_that_leaves_out_the_beach_is_refused():
    with pytest.raises(SystemExit):
        tool().main(['--out', '/nonexistent/never_written', '--radius', '50'])


def test_the_water_mesh_is_fine_near_the_origin_and_covers_the_square(tmp_path):
    t = tool()
    c = t.water_coordinates(750.0)
    assert c[0] == -750.0 and c[-1] == 750.0 and np.allclose(c, -c[::-1])
    steps = np.diff(c)
    assert steps[np.abs(c[:-1]) < 30.0].max() <= 1.5 + 1e-9        # a wave of 12 m has 8 cells at least
    assert steps.max() <= 12.0 + 1e-9
    n = t.write_water_mesh(str(tmp_path), 750.0)
    text = (tmp_path / 'water.obj').read_text().splitlines()
    verts = [l for l in text if l.startswith('v ')]
    faces = [l for l in text if l.startswith('f ')]
    assert len(verts) == n == len(c) ** 2
    assert len(faces) == 2 * (len(c) - 1) ** 2
    assert max(int(i.split('/')[0]) for f in faces for i in f.split()[1:]) == n


def test_the_water_mesh_faces_up(tmp_path):
    t = tool()
    t.write_water_mesh(str(tmp_path), 100.0)
    text = (tmp_path / 'water.obj').read_text().splitlines()
    verts = np.array([[float(v) for v in l.split()[1:]] for l in text if l.startswith('v ')])
    face = [int(i.split('/')[0]) - 1 for i in [l for l in text if l.startswith('f ')][0].split()[1:]]
    a, b, c = verts[face]
    assert np.cross(b - a, c - a)[2] > 0.0                         # counter-clockwise seen from above
