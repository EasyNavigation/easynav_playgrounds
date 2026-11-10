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

"""EasyNav with the submarine: the launch file, its parameters and its RViz configuration agree
with the simulation they drive."""

import importlib.util
import os

import pytest
import yaml

ROOT = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
WORLDS = os.path.join(os.path.dirname(ROOT), 'easynav_playground_marine_worlds')
SIM = os.path.join(WORLDS, 'launch', 'marine_sim.launch.py')


def launch():
    with open(os.path.join(ROOT, 'launch', 'easynav_submarine_mpc.launch.yaml')) as f:
        return yaml.safe_load(f)['launch']


@pytest.fixture(scope='module')
def params():
    with open(os.path.join(ROOT, 'params', 'submarine_mpc.params.yaml')) as f:
        return {node: p['ros__parameters'] for node, p in yaml.safe_load(f).items()}


def tool(name):
    spec = importlib.util.spec_from_file_location(
        name + '_for_easynav', os.path.join(WORLDS, 'tools', name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def simulation():
    """The include of the simulation, in its group."""
    groups = [e['group'] for e in launch() if 'group' in e]
    assert len(groups) == 1
    return groups[0], [e['include'] for e in groups[0] if 'include' in e][0]


def test_the_simulation_args_do_not_overwrite_the_ones_of_this_launch():
    # an include outside a scoped group sets rviz:=false here too, and RViz never starts
    assert not [e for e in launch() if 'include' in e]
    group, include = simulation()
    assert isinstance(group, list)                    # no attributes: scoped (the default)
    ours = {e['arg']['name'] for e in launch() if 'arg' in e}
    assert 'rviz' in ours and 'rviz' in {a['name'] for a in include['arg']}


def test_it_drives_the_submarine_and_leaves_the_transforms_to_the_localizer():
    _, include = simulation()
    assert include['file'].endswith('marine_sim.launch.py')
    args = {a['name']: a['value'] for a in include['arg']}
    assert args['robot'] == 'submarine' and args['odom_tf'] == 'false'
    assert args['rviz'] == 'false'                     # its own RViz, with EasyNav's displays
    source = open(SIM).read()
    for name in args:
        assert "'%s'" % name in source, name


def test_easynav_starts_with_the_parameters_and_rviz_with_its_configuration():
    entries = launch()
    timer = [e['timer'] for e in entries if 'timer' in e][0]
    node = timer['children'][0]['node']
    assert node['pkg'] == 'easynav_system' and node['exec'] == 'system_main'
    assert node['param'] == [{'from': '$(var params_file)'}]
    rviz = [e['node'] for e in entries if 'node' in e and e['node']['pkg'] == 'rviz2'][0]
    assert rviz['args'].endswith('/rviz/submarine_easynav.rviz')
    assert os.path.isfile(os.path.join(ROOT, 'rviz', 'submarine_easynav.rviz'))


def test_every_node_of_easynav_is_configured(params):
    assert {'controller_node', 'localizer_node', 'maps_manager_node', 'planner_node',
            'sensors_node', 'system_node'} <= set(params)
    assert all(p.get('use_sim_time') is True for p in params.values())


def test_the_limits_are_those_of_a_torpedo_and_valid_for_easynav(params):
    limits = params['controller_node']['robot_limits']
    assert limits['max_lateral_vel'] == 0.0            # no lateral motion
    assert limits['min_linear_vel'] == 0.0             # forward only
    for name, value in limits.items():
        if name.endswith('_acc') or name.endswith('_decel'):
            assert value > 0.0, name                   # EasyNav refuses 0 (lateral too)
    mpc = params['controller_node']['mpc']
    assert mpc['motion_model'] == 'aerial'
    assert mpc['yaw_goal_tolerance'] >= 3.14           # no turning on the spot to a heading


def test_the_map_is_the_relief_that_ships_with_the_worlds_package(params):
    bonxai = params['maps_manager_node']['bonxai']
    assert bonxai['package'] == 'easynav_playground_marine_worlds'
    assert os.path.isfile(os.path.join(WORLDS, bonxai['bonxai_path_file']))


def test_the_planner_keeps_under_the_surface_and_over_the_floor(params):
    planner = params['planner_node']['bonxai']
    assert planner['max_altitude'] < 0.0
    assert planner['fill_below'] == 'lowest'


def test_the_localizer_sits_where_the_world_is_and_where_the_vehicle_starts(params):
    ukf = params['localizer_node']['ukf']
    sea = tool('synthetic_sea')
    assert (ukf['latitude_origin'], ukf['longitude_origin']) == (sea.LATITUDE, sea.LONGITUDE)
    for name in ('local_filter', 'global_filter'):
        assert ukf[name]['initial_state'][:3] == [0.0, 0.0, -3.0]     # the default spawn
        assert ukf[name]['twist0'] == '/dvl/twist' and ukf[name]['imu0'] == '/imu/data'


def test_the_depth_goes_to_the_global_filter_only(params):
    ukf = params['localizer_node']['ukf']
    # the depth is measured in map: the local filter (odom) would learn nothing from it
    assert 'pose0' not in ukf['local_filter']
    assert ukf['global_filter']['pose0'] == '/depth'
    assert ukf['global_filter']['pose0_config'][:3] == [False, False, True]
    assert ukf['global_filter']['gps0'] == '/gps/fix'


def test_the_sensors_of_easynav_are_topics_of_the_submarine(params):
    sensors = params['sensors_node']
    ros = {e[1].lstrip('/') for e in tool('marine_sensors').bridge_entries('submarine')}
    ros |= {'sonar/points', 'gps/fix'}                 # made by the sensor and sonar nodes
    for name in sensors['sensors']:
        assert sensors[name]['topic'] in ros, name


def test_rviz_shows_the_map_the_path_and_has_the_3d_goal_tool():
    with open(os.path.join(ROOT, 'rviz', 'submarine_easynav.rviz')) as f:
        manager = yaml.safe_load(f)['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'map'
    topics = {d['Topic']['Value'] for d in manager['Displays'] if 'Topic' in d}
    assert {'/maps_manager_node/bonxai/map', '/planner_node/bonxai/path',
            '/controller_node/mpc/predicted', '/sonar/image_fan'} <= topics
    goal = [t for t in manager['Tools'] if t['Class'] == 'navmap_rviz_plugin/Goal3D']
    assert len(goal) == 1 and goal[0]['Topic']['Value'] == 'goal_pose'
