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
import math
import os
import re

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


def processes(actions, ctx):
    return [command(a, ctx) for a in actions
            if isinstance(a, ExecuteProcess) and not isinstance(a, Node)]


@pytest.mark.parametrize('robot', ['boat', 'submarine'])
def test_there_are_no_waves_unless_asked_for(robot, tmp_path, monkeypatch):
    m, ctx = context(robot=robot, gpu='none', radius='200')
    monkeypatch.setattr(m, 'OUT_DIR', str(tmp_path))
    actions = m.setup(ctx)
    assert not any('wave_node.py' in c for c in processes(actions, ctx))
    assert 'EntityWrench' not in (tmp_path / ('%s_bridge.yaml' % ('boat' if robot == 'boat' else 'tethys'))).read_text()


@pytest.mark.parametrize('robot', ['boat', 'submarine'])
def test_waves_start_the_wave_node_and_bridge_the_wrench_to_the_world(robot, tmp_path, monkeypatch):
    m, ctx = context(robot=robot, gpu='none', radius='200', waves='true', wave_height='1.2',
                     wave_period='5', wave_direction='90')
    monkeypatch.setattr(m, 'OUT_DIR', str(tmp_path))
    cmd = [c for c in processes(m.setup(ctx), ctx) if 'wave_node.py' in c]
    assert len(cmd) == 1
    assert '--vehicle %s' % robot in cmd[0] and '--height 1.2' in cmd[0]
    assert '--period 5' in cmd[0] and '--direction 90' in cmd[0] and '--sim-time' in cmd[0]
    bridge = (tmp_path / ('%s_bridge.yaml' % ('boat' if robot == 'boat' else 'tethys'))).read_text()
    assert '/world/marine_world/wrench' in bridge and 'ros_gz_interfaces/msg/EntityWrench' in bridge
    block = [b for b in bridge.split('- ros_topic_name') if 'EntityWrench' in b][0]
    assert 'direction: ROS_TO_GZ' in block and '/waves/wrench' in block
    world = (tmp_path / 'synthetic_r200' / 'world.sdf').read_text()
    assert 'gz-sim-apply-link-wrench-system' in world      # what the wrench needs in the world


@pytest.mark.parametrize('given, message', [
    ({'waves': 'yes'}, 'true or false'),
    ({'waves': 'true', 'wave_height': '5'}, 'wave_height'),
    ({'waves': 'true', 'wave_height': '-1'}, 'wave_height'),
    ({'waves': 'true', 'wave_period': '1'}, 'wave_period'),
    ({'waves': 'true', 'wave_period': '30'}, 'wave_period'),
])
def test_waves_with_values_that_make_no_sense_are_refused(given, message, tmp_path, monkeypatch):
    m, ctx = context(robot='boat', gpu='none', **given)
    monkeypatch.setattr(m, 'OUT_DIR', str(tmp_path))
    with pytest.raises(RuntimeError, match=message):
        m.setup(ctx)


def world_of(tmp_path):
    return (tmp_path / 'synthetic_r200' / 'world.sdf').read_text()


@pytest.mark.parametrize('robot', ['boat', 'submarine'])
def test_with_waves_the_water_is_drawn_by_a_shader_whose_files_exist(robot, tmp_path, monkeypatch):
    m, ctx = context(robot=robot, gpu='none', radius='200', waves='true', wave_height='0.8',
                     wave_period='5', wave_direction='130')
    monkeypatch.setattr(m, 'OUT_DIR', str(tmp_path))
    m.setup(ctx)
    world = world_of(tmp_path)
    assert 'gz-sim-shader-param-system' in world and '<name>Nwaves</name>' in world
    assert '<name>t</name><value>TIME</value>' in world
    assert (tmp_path / 'synthetic_r200' / 'water.obj').is_file()
    assert 'file://%s/water.obj' % (tmp_path / 'synthetic_r200') in world
    files = re.findall(r'<(?:vertex|fragment)>([^<]+)</', world)
    files += re.findall(r'<type>texture(?:_cube)?</type><value>([^<]+)</value>', world)
    assert len(files) == 4
    for path in files:
        assert os.path.isfile(path), path
    assert '<visual name="water_from_below">' in world          # seen from below, flat


def test_the_shader_gets_the_waves_of_the_arguments(tmp_path, monkeypatch):
    m, ctx = context(robot='boat', gpu='none', radius='200', waves='true', wave_height='0.8',
                     wave_period='5', wave_direction='90')
    monkeypatch.setattr(m, 'OUT_DIR', str(tmp_path))
    m.setup(ctx)
    world = world_of(tmp_path)
    amplitude = re.search(r'<name>amplitude</name><type>float_array</type><value>([^<]+)</value>', world)
    assert float(amplitude.group(1).split()[0]) == pytest.approx(0.4)          # half the height
    omega = re.search(r'<name>omega</name><type>float_array</type><value>([^<]+)</value>', world)
    assert float(omega.group(1).split()[0]) == pytest.approx(2 * math.pi / 5.0)
    dir0 = re.search(r'<name>dir0</name><type>float_array</type><value>([^<]+)</value>', world)
    assert [round(float(v), 6) for v in dir0.group(1).split()] == [0.0, 1.0]   # to the north


@pytest.mark.parametrize('given', [{}, {'waves': 'true', 'wave_visual': 'false'}])
def test_the_water_stays_flat_without_waves_or_without_the_visual(given, tmp_path, monkeypatch):
    m, ctx = context(robot='boat', gpu='none', radius='200', **given)
    monkeypatch.setattr(m, 'OUT_DIR', str(tmp_path))
    m.setup(ctx)
    assert 'shader-param' not in world_of(tmp_path)
    assert not (tmp_path / 'synthetic_r200' / 'water.obj').exists()


def test_wave_visual_is_true_or_false(tmp_path, monkeypatch):
    m, ctx = context(robot='boat', gpu='none', wave_visual='sometimes')
    monkeypatch.setattr(m, 'OUT_DIR', str(tmp_path))
    with pytest.raises(RuntimeError, match='true or false'):
        m.setup(ctx)


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
