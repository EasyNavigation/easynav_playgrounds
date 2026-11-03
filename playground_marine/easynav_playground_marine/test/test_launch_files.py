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

"""The launch file of the playground against the launch file of the simulation it includes: every
argument it passes exists there, and the ones that matter have the same default."""

import os

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = os.path.join(os.path.dirname(ROOT), 'easynav_playground_marine_worlds', 'launch',
                   'marine_sim.launch.py')


def load():
    with open(os.path.join(ROOT, 'launch', 'easynav_marine.launch.yaml')) as f:
        return yaml.safe_load(f)['launch']


def test_it_has_a_robot_argument_that_defaults_to_the_boat():
    args = {e['arg']['name']: e['arg'] for e in load() if 'arg' in e}
    assert args['robot']['default'] == 'boat'
    assert 'submarine' in args['robot']['description']


def test_every_argument_it_passes_is_declared_by_the_simulation_launch_file():
    source = open(SIM).read()
    include = [e['include'] for e in load() if 'include' in e][0]
    for a in include['arg']:
        assert "'%s'" % a['name'] in source, a['name']


@pytest.mark.parametrize('arg', ['robot', 'radius', 'altitude', 'gui'])
def test_each_argument_of_the_playground_is_passed_on(arg):
    include = [e['include'] for e in load() if 'include' in e][0]
    assert {'name': arg, 'value': '$(var %s)' % arg} in include['arg']


def test_it_is_only_the_simulation_no_other_family_package_is_named():
    text = open(os.path.join(ROOT, 'launch', 'easynav_marine.launch.yaml')).read().lower()
    code = '\n'.join(l for l in text.splitlines() if not l.lstrip().startswith('#'))
    for name in ('bigmap', 'navmap', 'bonxai'):
        assert name not in code
