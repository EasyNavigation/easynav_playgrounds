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

"""The decisions of launch/marine_sim.launch.py, taken by calling its setup() with launch arguments,
without starting any process. Needs the workspace built and sourced (ament index)."""

import importlib.util
import os

import pytest

launch = pytest.importorskip('launch')
pytest.importorskip('ament_index_python')
from launch.actions import ExecuteProcess  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(ROOT, 'launch', 'marine_sim.launch.py')


def module():
    spec = importlib.util.spec_from_file_location('marine_sim_launch', LAUNCH_FILE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def context(**given):
    """A launch context with the declared defaults and `given` on top."""
    m = module()
    ctx = launch.LaunchContext()
    for action in m.generate_launch_description().entities:
        if isinstance(action, launch.actions.DeclareLaunchArgument):
            ctx.launch_configurations[action.name] = given.pop(action.name, None) or ''.join(
                s.perform(ctx) for s in action.default_value)
    assert not given, 'unknown launch arguments %s' % given
    return m, ctx


def command(action, ctx):
    """The command line of an ExecuteProcess, its substitutions performed."""
    return ' '.join(''.join(s.perform(ctx) for s in word) for word in action.cmd)


def test_the_robot_is_a_boat_or_a_submarine():
    m, ctx = context(robot='summit')
    with pytest.raises(RuntimeError, match='boat, submarine'):
        m.setup(ctx)


def test_a_radius_that_leaves_out_the_beach_is_refused():
    m, ctx = context(radius='50')
    with pytest.raises(RuntimeError, match='at least 100'):
        m.setup(ctx)


@pytest.mark.parametrize('robot', ['boat', 'submarine'])
def test_it_writes_the_synthetic_sea_and_starts_the_vehicle(robot, tmp_path, monkeypatch):
    m, ctx = context(robot=robot, gpu='none', radius='200')
    monkeypatch.setattr(m, 'OUT_DIR', str(tmp_path))
    actions = m.setup(ctx)
    assert (tmp_path / 'synthetic_r200' / 'world.sdf').is_file()
    world = (tmp_path / ('world_%s.sdf' % robot)).read_text()
    assert robot_model(robot) in world
    assert any(isinstance(a, Node) for a in actions)       # the bridge
    names = [command(a, ctx) for a in actions
             if isinstance(a, ExecuteProcess) and not isinstance(a, Node)]
    assert any(('boat_cmd.py' if robot == 'boat' else 'auv_cmd.py') in n for n in names)
    assert any('odom_to_tf.py' in n for n in names)


def robot_model(robot):
    return 'wamv' if robot == 'boat' else 'tethys'


def test_a_spawn_on_the_beach_is_refused(tmp_path, monkeypatch):
    m, ctx = context(robot='boat', gpu='none', spawn_x='-450')
    monkeypatch.setattr(m, 'OUT_DIR', str(tmp_path))
    with pytest.raises(RuntimeError, match='no water'):
        m.setup(ctx)


def test_the_submarine_starts_under_the_surface_and_over_the_floor():
    m = module()
    sea = m.synthetic_sea(os.path.join(ROOT, 'tools'))
    z = m.submarine_start(sea)
    assert -3.0 <= z < 0.0 and z > float(sea.height(0.0, 0.0))
    assert m.submarine_start(sea, depth=-5.0) == -5.0
    assert m.boat_start() > 0.0 and m.underwater_at(sea, 0.0, 0.0)


def test_the_launch_file_names_no_other_package_of_the_family():
    source = open(LAUNCH_FILE).read().lower()
    for name in ('bigmap', 'navmap', 'bonxai'):
        assert name not in source


def test_the_vehicle_goes_into_the_world_and_the_water_acts_on_it(tmp_path):
    m = module()
    world = tmp_path / 'w.sdf'
    world.write_text('<sdf><world name="w"><plugin filename="gz-sim-buoyancy-system" name="b">'
                     '</plugin></world></sdf>')
    models = os.path.join(os.path.dirname(ROOT), 'easynav_playground_marine_description', 'models')
    out = m.vehicle_world(str(world), str(tmp_path / 'out.sdf'),
                          os.path.join(models, 'marine_wamv', 'wamv.sdf.in'), 1.0, 2.0, 0.3, 'wamv')
    text = open(out).read()
    assert '<enable>wamv</enable>' in text and '<pose>1.0000 2.0000 0.3000' in text
    assert '@' not in text.split('<world')[1].replace('@MODELS@', '')


def test_a_world_without_water_is_refused(tmp_path):
    m = module()
    world = tmp_path / 'w.sdf'
    world.write_text('<sdf><world name="w"></world></sdf>')
    models = os.path.join(os.path.dirname(ROOT), 'easynav_playground_marine_description', 'models')
    with pytest.raises(RuntimeError, match='no water'):
        m.vehicle_world(str(world), str(tmp_path / 'o.sdf'),
                        os.path.join(models, 'marine_tethys', 'tethys.sdf.in'), 0, 0, -3, 'tethys')
