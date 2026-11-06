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

"""The waves (tools/wave_node.py): the wave field, and the force that the sea adds to each vehicle."""

import importlib.util
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REST = np.array([0.0, 0.0, -0.1072])        # where the boat floats in a flat sea (measured)
LEVEL = np.eye(3)


def tool():
    spec = importlib.util.spec_from_file_location(
        'wave_node_under_test', os.path.join(ROOT, 'tools', 'wave_node.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def calm(t):
    return t.Waves(0.0, 4.0, 0.0)


def test_the_waves_are_deep_water_waves():
    t = tool()
    for a, w, k, _ in t.Waves(0.6, 4.0, 0.0, tau=0.0).components:
        assert w * w == pytest.approx(t.G * k)
    main = t.Waves(0.6, 4.0, 0.0).components[0]
    assert main[0] == pytest.approx(0.3) and main[1] == pytest.approx(2 * math.pi / 4.0)


def test_the_surface_is_flat_without_waves_and_grows_from_calm():
    t = tool()
    assert calm(t).elevation(3.0, -2.0, 7.0) == 0.0
    w = t.Waves(1.0, 4.0, 0.0)
    assert w.elevation(1.0, 2.0, 0.0) == 0.0
    big = max(abs(w.elevation(0.0, 0.0, s)) for s in np.linspace(10.0, 30.0, 400))
    assert 0.4 < big <= 0.5 * (1.0 + t.SECOND_AMPLITUDE)
    assert w.growth(0.0) == 0.0 and w.growth(t.GROWTH_TAU) == pytest.approx(1.0 - math.exp(-1.0))
    assert w.growth(10 * t.GROWTH_TAU) > 0.999


def test_the_shader_params_are_the_waves_the_vehicle_feels():
    """The shader that draws the water computes z = sum a_i (1 - exp(-t/tau)) cos(k_i d_i . x - w_i t)
    with steepness 0: worked out here from the parameters it is given, it must be `elevation`."""
    t = tool()
    w = t.Waves(0.8, 5.0, 130.0)
    params = {name: (kind, value) for name, kind, value in w.shader_params()}
    floats = lambda name: [float(v) for v in params[name][1].split()]       # noqa: E731
    count, amp, k, omega = int(params['Nwaves'][1]), floats('amplitude'), floats('wavenumber'), floats('omega')
    dirs = [floats('dir0'), floats('dir1'), floats('dir2')]
    assert floats('steepness') == [0.0, 0.0, 0.0] and count == 2
    tau = float(params['tau'][1])
    for x, y, time in ((0.0, 0.0, 3.0), (40.0, -25.0, 11.7), (-120.0, 300.0, 60.0)):
        z = sum(amp[i] * (1.0 - math.exp(-time / tau)) *
                math.cos((dirs[i][0] * x + dirs[i][1] * y) * k[i] - omega[i] * time) for i in range(count))
        assert z == pytest.approx(float(w.elevation(x, y, time)), abs=1e-6)
    assert all(math.hypot(*d) == pytest.approx(1.0) for d in dirs)


def test_the_wave_goes_where_it_is_told():
    t = tool()
    w = t.Waves(1.0, 4.0, 90.0, tau=0.0)          # to the north: it does not change along x
    assert w.elevation(0.0, 0.0, 1.3) != w.elevation(0.0, 5.0, 1.3)
    only_main = t.Waves(1.0, 4.0, 90.0, tau=0.0)
    only_main.components = only_main.components[:1]
    assert only_main.elevation(0.0, 3.0, 1.3) == pytest.approx(only_main.elevation(50.0, 3.0, 1.3))


def test_a_boat_at_rest_in_a_calm_sea_gets_no_force():
    t = tool()
    force, torque = t.boat_wrench(calm(t), REST, LEVEL, 3.0)
    assert not force.any() and not torque.any()


def test_a_raised_surface_lifts_the_boat_with_the_stiffness_that_was_measured():
    t = tool()
    w = calm(t)
    w.elevation = lambda x, y, time: 0.02 + 0.0 * x     # a flat sea 2 cm higher
    force, _ = t.boat_wrench(w, REST, LEVEL, 1.0)
    assert force[0] == 0.0 and force[1] == 0.0
    assert force[2] / 0.02 == pytest.approx(28.2e3, rel=0.1)      # 600 N gave 2.13 cm


def test_a_surface_that_rises_to_one_side_rolls_the_boat_to_that_side():
    t = tool()
    w = calm(t)
    w.elevation = lambda x, y, time: 0.03 * y            # higher to the left (+y)
    _, torque = t.boat_wrench(w, REST, LEVEL, 1.0)
    assert torque[0] > 0.0                                 # positive roll: the left goes up
    w.elevation = lambda x, y, time: 0.03 * x            # higher to the bow
    _, torque = t.boat_wrench(w, REST, LEVEL, 1.0)
    assert torque[1] < 0.0                                 # negative pitch: the bow goes up


def test_the_torque_for_a_slope_is_the_stiffness_measured_in_the_simulator():
    t = tool()
    w = calm(t)
    slope = 0.02
    w.elevation = lambda x, y, time: slope * x           # a surface that rises to the bow
    _, torque = t.boat_wrench(w, REST, LEVEL, 1.0)
    assert -torque[1] / slope == pytest.approx(23.6e3, rel=0.05)      # 1000 Nm gave 2.43 deg
    w.elevation = lambda x, y, time: slope * y           # a surface that rises to the left
    _, torque = t.boat_wrench(w, REST, LEVEL, 1.0)
    assert torque[0] / slope == pytest.approx(33.2e3, rel=0.05)       # 1000 Nm gave 1.73 deg


def test_the_scaling_follows_the_axes_of_the_boat_not_the_world():
    t = tool()
    w = calm(t)
    w.elevation = lambda x, y, time: 0.02 * x
    yawed = SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.5), w=math.cos(0.5))      # 1 rad of yaw
    r = t.rotation(yawed)
    _, torque = t.boat_wrench(w, REST, r, 1.0)
    body = r.T @ torque                                  # a slope along x is a mix of pitch and roll
    assert body[1] == pytest.approx(-23.6e3 * 0.02 * math.cos(1.0), rel=0.1)
    assert body[0] == pytest.approx(33.2e3 * 0.02 * math.sin(1.0) * -1.0, rel=0.1)


def test_a_trough_below_the_floats_cannot_pull_the_boat_down_more_than_its_weight():
    t = tool()
    w = calm(t)
    w.elevation = lambda x, y, time: -1.0 + 0.0 * x       # the sea falls away from the floats
    force, _ = t.boat_wrench(w, REST, LEVEL, 1.0)
    flat_buoyancy = 28.2e3 * 0.0845                          # 2.4 kN, the draft of the boat
    assert -1.1 * flat_buoyancy < force[2] < -0.9 * flat_buoyancy
    w.elevation = lambda x, y, time: 1.0 + 0.0 * x        # a crest that covers the floats
    top, _ = t.boat_wrench(w, REST, LEVEL, 1.0)
    assert 0.0 < top[2] < 1e5


def test_the_submarine_feels_the_wave_less_the_deeper_it_is():
    t = tool()
    w = t.Waves(0.6, 4.0, 0.0, tau=0.0)
    shallow, _ = t.submarine_wrench(w, np.array([0.0, 0.0, -3.0]), LEVEL, 0.7)
    deep, _ = t.submarine_wrench(w, np.array([0.0, 0.0, -15.0]), LEVEL, 0.7)
    assert np.linalg.norm(deep) < 0.1 * np.linalg.norm(shallow)
    expected = t.TETHYS_MASS * np.array(w.orbital_acceleration(0.0, 0.0, -3.0, 0.7))
    assert np.allclose(shallow, expected)


def test_a_tilted_submarine_is_righted_towards_level():
    t = tool()
    w = calm(t)
    pos = np.array([0.0, 0.0, -3.0])
    force, torque = t.submarine_wrench(w, pos, LEVEL, 1.0)
    assert not force.any() and not torque.any()           # level in a calm sea: nothing
    roll = t.rotation(SimpleNamespace(x=math.sin(0.1), y=0.0, z=0.0, w=math.cos(0.1)))      # 0.2 rad
    _, torque = t.submarine_wrench(w, pos, roll, 1.0)
    assert torque[0] < 0.0 and torque[1] == pytest.approx(0.0, abs=1e-9)      # against the roll
    expected = t.SUBMARINE_BG * t.TETHYS_MASS * t.G * math.sin(0.2)
    assert -torque[0] == pytest.approx(expected, rel=1e-6)
    pitch = t.rotation(SimpleNamespace(x=0.0, y=math.sin(0.1), z=0.0, w=math.cos(0.1)))
    _, torque = t.submarine_wrench(w, pos, pitch, 1.0)
    assert torque[1] < 0.0 and torque[0] == pytest.approx(0.0, abs=1e-9)      # against the pitch


def test_the_submarine_rotating_is_damped_in_its_own_axes():
    t = tool()
    w = calm(t)
    pos = np.array([0.0, 0.0, -3.0])
    _, torque = t.submarine_wrench(w, pos, LEVEL, 1.0, rates=(1.0, 0.0, 0.0))
    assert np.allclose(torque, [-t.SUBMARINE_DAMPING[0], 0.0, 0.0])
    _, torque = t.submarine_wrench(w, pos, LEVEL, 1.0, rates=(0.0, 2.0, 5.0))
    assert np.allclose(torque, [0.0, -2.0 * t.SUBMARINE_DAMPING[1], 0.0])      # yaw is left free


def test_the_angular_rates_of_a_turn_are_recovered():
    t = tool()
    before = t.rotation(SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    after = t.rotation(SimpleNamespace(x=math.sin(0.005), y=0.0, z=0.0, w=math.cos(0.005)))   # 0.01 rad
    assert np.allclose(t.angular_rates(before, after, 0.02), [0.5, 0.0, 0.0], atol=1e-3)


def test_the_orbital_acceleration_at_a_crest_pulls_down_and_is_horizontal_at_the_zero_crossing():
    t = tool()
    w = t.Waves(1.0, 4.0, 0.0, tau=0.0)
    w.components = w.components[:1]
    a, omega, k, _ = w.components[0]
    ax, ay, az = w.orbital_acceleration(0.0, 0.0, 0.0, 0.0)       # phase 0: a crest
    assert az == pytest.approx(-omega ** 2 * a) and ax == pytest.approx(0.0, abs=1e-9) and ay == 0.0
    ax, _, az = w.orbital_acceleration(math.pi / (2 * k), 0.0, 0.0, 0.0)   # phase pi/2
    assert ax == pytest.approx(omega ** 2 * a) and az == pytest.approx(0.0, abs=1e-9)


def test_rotation_of_a_quaternion_matches_a_yaw():
    t = tool()
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.4), w=math.cos(0.4))      # 0.8 rad of yaw
    r = t.rotation(q)
    assert np.allclose(r @ [1.0, 0.0, 0.0], [math.cos(0.8), math.sin(0.8), 0.0])


def test_the_negative_height_and_period_are_refused():
    t = tool()
    with pytest.raises(ValueError):
        t.Waves(-1.0, 4.0, 0.0)
    with pytest.raises(ValueError):
        t.Waves(1.0, 0.0, 0.0)


def test_every_tool_the_launch_file_runs_is_executable():
    tools = os.path.join(ROOT, 'tools')
    for name in os.listdir(tools):
        if name.endswith('.py'):
            assert os.access(os.path.join(tools, name), os.X_OK), name
