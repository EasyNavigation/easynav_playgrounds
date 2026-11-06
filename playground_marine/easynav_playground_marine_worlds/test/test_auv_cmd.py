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

"""The autopilot of the Tethys (tools/auv_cmd.py): what it commands for what it is asked."""

import importlib.util
import math
import os

import pytest

TOOLS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'tools')


def autopilot():
    spec = importlib.util.spec_from_file_location(
        'auv_cmd_under_test', os.path.join(TOOLS, 'auv_cmd.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def at(u=0.0, r=0.0, pitch=0.0, q=0.0, vz=0.0):
    return (u, r, pitch, q, vz)


def test_the_thrust_holds_the_commanded_speed_against_the_drag():
    a = autopilot()
    c = a.AuvController()
    thrust, _, _ = c.step((2.2, 0.0, 0.0), at(u=2.2))
    assert thrust == pytest.approx(a.DRAG * 2.2 ** 2, rel=1e-6)       # the 31 N of the model
    assert thrust == pytest.approx(30.1, abs=1.0)
    assert c.step((2.0, 0.0, 0.0), at(u=0.0))[0] > c.step((2.0, 0.0, 0.0), at(u=2.0))[0]   # slow: push
    assert c.step((1.0, 0.0, 0.0), at(u=2.0))[0] < c.step((1.0, 0.0, 0.0), at(u=1.0))[0]   # fast: ease


def test_nothing_is_commanded_beyond_the_limits_of_the_model():
    a = autopilot()
    c = a.AuvController()
    for command in ((50.0, 5.0, 5.0), (-50.0, -5.0, -5.0)):
        thrust, rudder, elevator = c.step(command, at(u=0.0))
        assert abs(thrust) <= a.MAX_THRUST
        assert abs(rudder) <= a.MAX_FIN and abs(elevator) <= a.MAX_FIN
    # the limit is the model's (a mutation test showed the clamp and the check shared the number)
    assert a.MAX_FIN == pytest.approx(0.26) and a.MAX_THRUST == pytest.approx(60.0)


def test_a_positive_yaw_rate_turns_the_rudder_the_way_that_turned_the_model_left():
    c = autopilot().AuvController()
    _, rudder, _ = c.step((1.8, 0.04, 0.0), at(u=1.8))      # it turns at 0.055 rad/s at most
    assert rudder > 0.0                                  # measured: +0.17 rad gave +0.055 rad/s
    assert c.step((1.8, -0.04, 0.0), at(u=1.8))[1] < 0.0
    # already turning as asked: less rudder than when it is not turning yet
    assert c.step((1.8, 0.04, 0.0), at(u=1.8, r=0.04))[1] < rudder


def test_the_joints_get_the_commands_of_a_model_whose_x_is_the_stern():
    a = autopilot()
    # measured bow first: -30 N on the propeller drove it forward at 2.2 m/s, +0.17 rad of rudder
    # turned it left, +0.17 rad of elevator lifted the bow and it climbed
    assert a.joint_commands(30.0, 0.1, 0.1) == pytest.approx((-30.0, 0.1, -0.1))
    assert a.joint_commands(0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)
    c = a.AuvController()
    thrust, _, elevator = a.joint_commands(*c.step((2.0, 0.0, 0.3), at(u=1.5)))
    assert thrust < 0.0 and elevator > 0.0               # faster and up: push, and lift the bow


def test_a_slow_vehicle_needs_more_rudder_for_the_same_turn_because_the_fins_act_with_speed():
    c = autopilot().AuvController()
    assert c.step((1.0, 0.02, 0.0), at(u=1.0))[1] > c.step((2.0, 0.02, 0.0), at(u=2.0))[1]


def test_to_climb_it_pitches_the_nose_up_and_to_dive_down():
    # the pitch of base_link is positive bow down, and so is the controller's elevator (the joint's
    # is the opposite: see the test of joint_commands)
    c = autopilot().AuvController()
    _, _, elevator_up = c.step((2.0, 0.0, 0.3), at(u=2.0))
    _, _, elevator_down = c.step((2.0, 0.0, -0.3), at(u=2.0))
    assert elevator_up < 0.0 < elevator_down
    # with the nose already where it should be, there is little left to command
    target = -math.asin(0.3 / 2.0)
    assert abs(c.step((2.0, 0.0, 0.3), at(u=2.0, pitch=target, vz=0.3))[2]) < 0.02


def test_the_pitch_rate_is_damped():
    c = autopilot().AuvController()
    still = c.step((2.0, 0.0, 0.0), at(u=2.0))[2]
    assert c.step((2.0, 0.0, 0.0), at(u=2.0, q=0.3))[2] < still     # pitching down fast: ease off


def test_without_speed_the_fins_are_not_asked_to_dive():
    a = autopilot()
    c = a.AuvController()
    assert c.step((0.0, 0.0, 0.5), at(u=0.0))[2] == pytest.approx(0.0)
    # (and no division by a speed of zero anywhere)
    for u in (0.0, 1e-9, -1e-9):
        c.step((1.0, 0.3, 0.3), at(u=u))


def test_the_pitch_of_a_quaternion_is_positive_nose_down():
    a = autopilot()

    class Q:
        def __init__(self, pitch):
            self.x = self.z = 0.0
            self.y = math.sin(pitch / 2.0)
            self.w = math.cos(pitch / 2.0)
    assert a.pitch_of(Q(0.4)) == pytest.approx(0.4)
    assert a.pitch_of(Q(-0.2)) == pytest.approx(-0.2)
    assert a.pitch_of(Q(0.0)) == 0.0


def test_the_autopilot_has_its_options_without_importing_ros():
    import subprocess
    import sys
    out = subprocess.run(
        [sys.executable, os.path.join(TOOLS, 'auv_cmd.py'), '--help'], capture_output=True,
        text=True, check=True).stdout
    for option in ('--model', '--rate', '--timeout', '--sim-time'):
        assert option in out, option


def test_a_command_or_a_state_that_is_not_a_number_coasts_instead_of_full_thrust():
    c = autopilot().AuvController()
    nan, inf = float('nan'), float('inf')
    # (the clamps turned a NaN into MAX_THRUST: measured)
    assert c.step((nan, 0.0, 0.0), at()) == (0.0, 0.0, 0.0)
    assert c.step((1.0, 0.0, nan), at(u=1.0)) == (0.0, 0.0, 0.0)
    assert c.step((1.0, 0.0, 0.0), at(pitch=inf)) == (0.0, 0.0, 0.0)
    assert c.step((1.0, 0.0, 0.0), at(u=1.0)) != (0.0, 0.0, 0.0)


def test_a_turn_on_the_spot_is_done_on_the_move():
    a = autopilot()
    c = a.AuvController()
    thrust, rudder, _ = c.step((0.0, 0.2, 0.0), at(u=0.0))
    assert thrust > 0.0 and rudder > 0.0                       # it moves forward, turning left
    assert c.step((0.0, 0.0, -0.3), at(u=0.0))[0] > 0.0        # to dive, too


def test_without_a_turn_nor_a_dive_a_stop_is_a_stop():
    a = autopilot()
    c = a.AuvController()
    assert c.step((0.0, 0.0, 0.0), at(u=0.0))[0] == pytest.approx(0.0)
    assert c.step((0.0, 0.0, 0.0), at(u=1.0))[0] < 0.0         # it brakes


def test_faster_than_the_steerage_speed_the_command_stays():
    a = autopilot()
    c = a.AuvController()
    fast = c.step((1.5, 0.1, 0.0), at(u=1.5))[0]
    assert fast == pytest.approx(c.step((1.5, 0.0, 0.0), at(u=1.5))[0])
    slow = a.AuvController(steerage_speed=0.0)
    assert slow.step((0.0, 0.2, 0.0), at(u=0.0))[0] == pytest.approx(0.0)  # 0: off
