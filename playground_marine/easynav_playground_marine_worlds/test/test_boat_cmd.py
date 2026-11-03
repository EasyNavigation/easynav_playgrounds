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

"""The autopilot of the WAM-V (tools/boat_cmd.py): the thrust of each engine for a command."""

import importlib.util
import os

import pytest

TOOLS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'tools')
# the models are in the description package, next to this one in the source tree
MODELS = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                      'easynav_playground_marine_description', 'models')


def autopilot():
    spec = importlib.util.spec_from_file_location(
        'boat_cmd_under_test', os.path.join(TOOLS, 'boat_cmd.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_both_engines_push_forward_the_same_to_go_straight():
    left, right = autopilot().BoatController().step((1.4, 0.0), (1.4, 0.0))
    assert left == pytest.approx(right)
    assert 100.0 < left + right < 260.0      # measured: ~80 N on each engine holds 1.5 m/s


def test_the_right_engine_pushes_more_to_turn_left_and_the_left_one_to_turn_right():
    c = autopilot().BoatController()
    left, right = c.step((1.0, 0.3), (1.0, 0.0))
    assert right > left                         # measured: more on the right turned it left
    left, right = c.step((1.0, -0.3), (1.0, 0.0))
    assert left > right


def test_the_turn_does_not_change_the_total_push_only_how_it_is_shared():
    c = autopilot().BoatController()
    straight = sum(c.step((1.0, 0.0), (1.0, 0.0)))
    turning = sum(c.step((1.0, 0.3), (1.0, 0.0)))
    assert turning == pytest.approx(straight)


def test_it_pushes_harder_when_it_is_slower_than_asked_and_eases_when_faster():
    c = autopilot().BoatController()
    assert sum(c.step((1.5, 0.0), (0.0, 0.0))) > sum(c.step((1.5, 0.0), (1.5, 0.0)))
    assert sum(c.step((1.0, 0.0), (2.0, 0.0))) < sum(c.step((1.0, 0.0), (1.0, 0.0)))


def test_a_yaw_it_already_has_is_not_commanded_again():
    c = autopilot().BoatController()
    new = c.step((1.0, 0.3), (1.0, 0.0))
    done = c.step((1.0, 0.3), (1.0, 0.3))
    assert (done[1] - done[0]) < (new[1] - new[0])


def test_the_thrust_stays_within_the_limits_of_the_engines_in_the_model():
    a = autopilot()
    c = a.BoatController()
    for command in ((30.0, 5.0), (-30.0, -5.0), (0.0, 9.0)):
        for engine in c.step(command, (0.0, 0.0)):
            assert a.MIN_THRUST <= engine <= a.MAX_THRUST
    # the same limits the Thruster plugins of the model set
    path = os.path.join(MODELS, 'marine_wamv', 'wamv.sdf.in')
    snippet = open(path).read()
    assert snippet.count('<max_thrust_cmd>%.1f</max_thrust_cmd>' % a.MAX_THRUST) == 2
    assert snippet.count('<min_thrust_cmd>%.1f</min_thrust_cmd>' % a.MIN_THRUST) == 2


def test_the_autopilot_has_its_options_without_importing_ros():
    import subprocess
    import sys
    out = subprocess.run(
        [sys.executable, os.path.join(TOOLS, 'boat_cmd.py'), '--help'], capture_output=True,
        text=True, check=True).stdout
    for option in ('--boat', '--rate', '--timeout', '--sim-time'):
        assert option in out, option


def test_the_yaw_of_a_quaternion_and_the_wrap_of_an_angle():
    import math
    a = autopilot()

    class Q:
        def __init__(self, yaw):
            self.x = self.y = 0.0
            self.z = math.sin(yaw / 2.0)
            self.w = math.cos(yaw / 2.0)
    assert a.yaw_of(Q(0.7)) == pytest.approx(0.7)
    assert a.yaw_of(Q(-2.0)) == pytest.approx(-2.0)
    # across the +-pi cut a small turn is a small turn, not 2 pi
    assert a.wrap(math.radians(-179) - math.radians(179)) == pytest.approx(math.radians(2.0))
    assert a.wrap(math.radians(179) - math.radians(-179)) == pytest.approx(math.radians(-2.0))


def test_the_wamv_model_floats_on_boxes_and_its_engines_do_not_swing():
    import xml.etree.ElementTree as ET
    root = ET.parse(os.path.join(MODELS, 'marine_wamv', 'model.sdf'))
    floats = [c for c in root.iter('collision') if c.get('name', '').endswith('float')]
    assert len(floats) == 6  # graded buoyancy: boxes, spheres
    assert all(c.find('geometry/box') is not None for c in floats)
    joints = {j.get('name'): j.get('type') for j in root.iter('joint')}
    assert joints['left_chassis_engine_joint'] == 'fixed'
    assert joints['right_chassis_engine_joint'] == 'fixed'
    limits = [float(j.find('axis/limit/effort').text) for j in root.iter('joint')
              if j.get('name').endswith('propeller_joint')]
    assert limits and max(limits) <= 100.0  # the SDF effort


def test_the_autopilot_stays_inside_the_thrust_the_engines_can_give():
    module = autopilot()
    c = module.BoatController()
    for cmd in ((5.0, 2.0), (-5.0, -2.0), (0.0, 3.0)):
        for thrust in c.step(cmd, (0.0, 0.0)):
            assert module.MIN_THRUST <= thrust <= module.MAX_THRUST


def test_a_command_or_a_state_that_is_not_a_number_stops_the_engines_instead_of_full_thrust():
    c = autopilot().BoatController()
    nan, inf = float('nan'), float('inf')
    # (the clamps turned a NaN into MAX_THRUST on both engines: measured)
    assert c.step((nan, 0.0), (0.0, 0.0)) == (0.0, 0.0)
    assert c.step((1.0, 0.0), (nan, 0.0)) == (0.0, 0.0)
    assert c.step((0.0, inf), (0.0, 0.0)) == (0.0, 0.0)
    assert c.step((1.0, 0.1), (1.0, 0.1)) != (0.0, 0.0)
