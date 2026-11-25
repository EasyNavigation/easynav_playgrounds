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

"""The decisions of geo_sim.launch.py, without starting any process: the generator command of each
world, the checks of the arguments, where the robot is spawned and what is written for
applications."""

import importlib.util
import os
import struct

import pytest

np = pytest.importorskip('numpy')
pytest.importorskip('launch')

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(scope='module')
def gs():
    spec = importlib.util.spec_from_file_location(
        'geo_sim_launch', os.path.join(ROOT, 'launch', 'geo_sim.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def cfg(**kw):
    c = {'world': 'terrain', 'lat': '40.4168', 'lon': '-3.7038', 'radius': '250',
         'trees': 'true', 'walls': 'true', 'roof': 'ortho', 'ign': 'false', 'lidar': ''}
    c.update(kw)
    return c


def test_the_terrain_is_the_generator_without_buildings_walls_or_trees(gs):
    cmd = gs.generator_command('/t/geo_world.py', '/w/geo.world.in', cfg())
    assert cmd[1] == '/t/geo_world.py'
    assert {'--no-buildings', '--no-trees', '--no-walls'} <= set(cmd)
    assert '--roof' not in cmd
    assert cmd[cmd.index('--lat') + 1] == '40.4168'
    assert cmd[cmd.index('--sea') + 1] == 'off'
    assert cmd[cmd.index('--out-dir') + 1] == gs.OUT_DIR


def test_the_area_has_buildings_and_what_is_asked_for(gs):
    cmd = gs.generator_command('t', 'w', cfg(world='area'))
    assert '--no-buildings' not in cmd and '--no-trees' not in cmd and '--no-walls' not in cmd
    assert cmd[cmd.index('--roof') + 1] == 'ortho'
    cmd = gs.generator_command('t', 'w', cfg(world='area', trees='false', walls='false',
                                             roof='flat'))
    assert '--no-trees' in cmd and '--no-walls' in cmd and '--no-buildings' not in cmd
    assert cmd[cmd.index('--roof') + 1] == 'flat'


def test_by_default_the_data_is_openstreetmap_not_ign(gs):
    for world in gs.WORLDS:
        assert '--ign' not in gs.generator_command('t', 'w', cfg(world=world))


def test_ign_and_its_lidar_go_to_the_generator_in_both_worlds(gs):
    for world in gs.WORLDS:
        cmd = gs.generator_command('t', 'w', cfg(world=world, ign='true', lidar='/data/laz'))
        assert '--ign' in cmd and cmd[cmd.index('--lidar') + 1] == '/data/laz'


def test_the_lidar_without_ign_is_refused(gs):
    with pytest.raises(RuntimeError, match='ign:=true'):
        gs.generator_command('t', 'w', cfg(lidar='/data/laz'))


def test_ign_stops_the_launch_without_laspy_or_laszip(gs):
    gs.check_ign('true', [])
    gs.check_ign('false', ['laspy', 'laszip'])
    for missing in (['laspy', 'laszip'], ['laszip']):
        with pytest.raises(RuntimeError, match='python3-laspy python3-laszip'):
            gs.check_ign('true', missing)


@pytest.mark.skipif(os.environ.get('GEO_IGN_DEPS') not in ('installed', 'missing'),
                    reason='GEO_IGN_DEPS (installed or missing) is set by the CI')
def test_ign_starts_only_with_laspy_and_laszip_installed(gs):
    """The CI runs this with and without python3-laspy and python3-laszip."""
    missing = gs.ign_missing()
    if os.environ['GEO_IGN_DEPS'] == 'installed':
        assert missing == []
        gs.check_ign('true', missing)
    else:
        assert missing
        with pytest.raises(RuntimeError, match='python3-laspy python3-laszip'):
            gs.check_ign('true', missing)


def test_what_ign_misses_is_what_cannot_be_imported(gs, monkeypatch):
    import builtins
    import types
    real = builtins.__import__

    def fake(laspy):
        monkeypatch.setattr(builtins, '__import__', lambda name, *a, **k: (
            laspy if laspy is not None else (_ for _ in ()).throw(ImportError(name)))
            if name == 'laspy' else real(name, *a, **k))

    fake(None)
    assert gs.ign_missing() == ['laspy', 'laszip']

    def laszip(ok):
        return types.SimpleNamespace(LazBackend=types.SimpleNamespace(
            Laszip=types.SimpleNamespace(is_available=lambda: ok)))
    fake(laszip(False))
    assert gs.ign_missing() == ['laszip']
    fake(laszip(True))
    assert gs.ign_missing() == []


@pytest.mark.parametrize('bad', [
    {'world': 'city'}, {'lat': 'north'}, {'lat': '91'}, {'lon': '-200'}, {'radius': '0'},
    {'radius': '5000'}, {'trees': 'yes'}, {'walls': '1'}, {'roof': 'red'}, {'ign': 'yes'},
])
def test_bad_arguments_say_what_is_wrong(gs, bad):
    with pytest.raises(RuntimeError):
        gs.generator_command('t', 'w', cfg(**bad))


def write_stl(path, tris):
    with open(path, 'wb') as f:
        f.write(b'\0' * 80 + struct.pack('<I', len(tris)))
        for t in tris:
            f.write(struct.pack('<12fH', 0, 0, 1, *[c for v in t for c in v], 0))


@pytest.fixture
def world(tmp_path):
    """A world of a 20 x 20 m slope (z = 0.1 x) with a 3 m high block on [4, 6] x [4, 6]."""
    tris = [((-10, -10, -1), (10, -10, 1), (10, 10, 1)), ((-10, -10, -1), (10, 10, 1), (-10, 10, -1)),
            ((4, 4, 3), (6, 4, 3), (6, 6, 3)), ((4, 4, 3), (6, 6, 3), (4, 6, 3))]
    write_stl(str(tmp_path / 'navmap.stl'), tris)
    (tmp_path / 'world.yaml').write_text(
        '# written by geo_world.py\nkey: abc\nlatitude: 40.41680000\nlongitude: -3.70380000\n'
        'radius: 10.0\nground_elevation: 655.000\nspawn_x: 1.00\nspawn_y: 2.00\nspawn_z: 0.600\n')
    return str(tmp_path)


def test_the_world_file_is_read(gs, world):
    w = gs.read_world(world)
    assert w['dir'] == world and w['latitude'] == '40.41680000' and w['spawn_z'] == '0.600'
    assert 'key' in w and not any(k.startswith('#') for k in w)


def test_the_robot_goes_to_the_free_spot_of_the_generator_unless_told(gs, world):
    w = gs.read_world(world)
    none = {'spawn_x': '', 'spawn_y': '', 'spawn_z': ''}
    assert gs.spawn_pose(w, none) == (1.0, 2.0, 0.6)
    assert gs.spawn_pose(w, dict(none, spawn_z='5')) == (1.0, 2.0, 5.0)
    assert gs.spawn_pose(w, {'spawn_x': '3', 'spawn_y': '-2', 'spawn_z': '7'}) == (3.0, -2.0, 7.0)


def test_given_x_y_the_robot_is_put_on_the_highest_ground_there(gs, world):
    w = gs.read_world(world)
    x, y, z = gs.spawn_pose(w, {'spawn_x': '-5', 'spawn_y': '0', 'spawn_z': ''})
    assert (x, y) == (-5.0, 0.0) and z == pytest.approx(-0.5 + 0.5)    # the slope, z = 0.1 x
    x, y, z = gs.spawn_pose(w, {'spawn_x': '5', 'spawn_y': '5', 'spawn_z': ''})
    assert z == pytest.approx(3.5)                                      # on the block
    with pytest.raises(RuntimeError):
        gs.spawn_pose(w, {'spawn_x': '50', 'spawn_y': '0', 'spawn_z': ''})


def test_the_last_world_says_where_the_place_and_its_mesh_are(gs, world, tmp_path):
    out = gs.write_last_world(gs.read_world(world), 'area', path=str(tmp_path / 'x' / 'last.yaml'))
    text = open(out).read()
    assert 'world: area\n' in text and 'latitude: 40.41680000\n' in text
    assert 'ground_elevation: 655.000\n' in text
    assert 'mesh: %s\n' % os.path.join(world, 'navmap.stl') in text
