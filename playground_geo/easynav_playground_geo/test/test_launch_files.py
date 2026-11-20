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

"""The launch file of the playground against the launch file of the simulation it forwards to.

No process is started: the YAML is parsed, and each argument it forwards is looked for in the
arguments that geo_sim.launch.py declares (a renamed or removed one would otherwise only show when a
launch fails).
"""

import importlib.util
import os

import pytest

yaml = pytest.importorskip('yaml')
launch = pytest.importorskip('launch')
pytest.importorskip('launch_yaml')

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLAYGROUND = os.path.join(ROOT, 'launch', 'easynav_geo.launch.yaml')


def declared_by_sim():
    """{argument: default} of geo_sim.launch.py."""
    path = os.path.join(ROOT, 'launch', 'geo_sim.launch.py')
    spec = importlib.util.spec_from_file_location('geo_sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ctx = launch.LaunchContext()
    out = {}
    for action in module.generate_launch_description().entities:
        if isinstance(action, launch.actions.DeclareLaunchArgument):
            out[action.name] = ''.join(s.perform(ctx) for s in action.default_value)
    return out


def load():
    with open(PLAYGROUND) as f:
        return yaml.safe_load(f)['launch']


def args_of():
    return {e['arg']['name']: e['arg'] for e in load() if 'arg' in e}


def include_of():
    includes = [e['include'] for e in load() if 'include' in e]
    assert len(includes) == 1
    return includes[0]


def forwarded():
    return {a['name']: str(a['value']) for a in include_of()['arg']}


def test_it_includes_the_simulation_launch_file():
    assert include_of()['file'] == '$(find-pkg-share easynav_playground_geo)/launch/geo_sim.launch.py'


def test_every_argument_is_forwarded_as_itself_and_exists_in_the_simulation():
    own, values, declared = args_of(), forwarded(), declared_by_sim()
    assert set(values) == set(own) == set(declared)
    for name, value in values.items():
        assert value == '$(var %s)' % name


def test_the_defaults_are_the_ones_of_the_simulation():
    declared = declared_by_sim()
    for name, a in args_of().items():
        assert str(a['default']) == declared[name], name


def test_the_terrain_is_the_default_world():
    assert args_of()['world']['default'] == 'terrain'


def test_the_launch_file_is_a_valid_launch_description():
    from launch.launch_description_sources import AnyLaunchDescriptionSource
    description = AnyLaunchDescriptionSource(PLAYGROUND).get_launch_description(
        launch.LaunchContext())
    names = [e.name for e in description.entities
             if isinstance(e, launch.actions.DeclareLaunchArgument)]
    assert set(names) == set(args_of())
