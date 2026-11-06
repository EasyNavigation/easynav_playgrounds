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

"""What tools/marine_sensor_node.py adds to Gazebo's sensors: the GNSS under the water, the depth
gauge, the echosounder, the DVL's twist and altitude, the lidar over the water."""

import importlib.util
import math
import os

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEVEL = (0.0, 0.0, 0.0, 1.0)


def tool():
    spec = importlib.util.spec_from_file_location(
        'marine_sensor_node_under_test', os.path.join(ROOT, 'tools', 'marine_sensor_node.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def t():
    return tool()


def about_x(angle):
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


def about_y(angle):
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


# --- the point of a sensor in the world

def test_the_z_of_a_point_follows_the_vehicle_and_its_attitude(t):
    assert t.point_z((5.0, 2.0, -3.0), LEVEL, (0.0, 0.0, 0.25)) == pytest.approx(-2.75)
    # rolled 90 degrees: a point over the deck goes to the side, at the vehicle's height
    assert t.point_z((0.0, 0.0, -3.0), about_x(math.pi / 2), (0.0, 0.0, 0.25)) == \
        pytest.approx(-3.0, abs=1e-12)
    # upside down: under it
    assert t.point_z((0.0, 0.0, -3.0), about_x(math.pi), (0.0, 0.0, 0.25)) == pytest.approx(-3.25)


# --- GNSS

def test_without_a_pose_there_is_no_fix(t):
    assert not t.GnssGate(0.0).has_fix(None, 10.0)


def test_under_the_water_there_is_no_fix(t):
    gate = t.GnssGate(5.0)
    assert not gate.has_fix(-3.0, 0.0)
    assert not gate.has_fix(t.WET_AT - 1e-6, 100.0)


def test_at_the_surface_the_fix_comes_after_the_reacquisition_time(t):
    gate = t.GnssGate(5.0)
    assert not gate.has_fix(-3.0, 0.0)
    assert not gate.has_fix(0.3, 10.0)      # just out: the receiver searches
    assert not gate.has_fix(0.3, 14.9)
    assert gate.has_fix(0.3, 15.0)
    assert gate.has_fix(0.3, 100.0)


def test_diving_again_loses_the_fix_at_once_and_it_starts_over(t):
    gate = t.GnssGate(5.0)
    assert not gate.has_fix(0.3, 0.0) and gate.has_fix(0.3, 5.0)
    assert not gate.has_fix(-0.5, 6.0)
    assert not gate.has_fix(0.3, 7.0) and not gate.has_fix(0.3, 11.9)
    assert gate.has_fix(0.3, 12.0)


def test_a_wave_that_barely_wets_the_antenna_is_not_a_dive(t):
    gate = t.GnssGate(0.0)
    assert gate.has_fix(t.WET_AT, 0.0)                  # at the limit, still dry
    assert gate.has_fix(-0.01, 1.0)


def test_with_no_reacquisition_time_the_fix_is_immediate(t):
    gate = t.GnssGate(0.0)
    assert not gate.has_fix(-1.0, 0.0)
    assert gate.has_fix(0.5, 0.1)


# --- depth gauge

def test_the_depth_is_positive_down_and_zero_out_of_the_water(t):
    assert t.depth(-12.5) == 12.5
    assert t.depth(0.0) == 0.0
    assert t.depth(1.5) == 0.0


def test_the_noise_of_the_depth_has_the_gauge_stddev_and_never_goes_above_the_surface(t):
    rng = np.random.default_rng(3)
    d = np.array([t.depth(-10.0, rng) for _ in range(4000)])
    assert d.mean() == pytest.approx(10.0, abs=0.002)
    assert d.std() == pytest.approx(t.DEPTH_STDDEV, rel=0.1)
    shallow = [t.depth(-0.001, rng) for _ in range(500)]
    assert min(shallow) == 0.0 and max(shallow) > 0.0


def test_the_pressure_is_the_atmosphere_plus_the_water_column(t):
    assert t.pressure(0.0) == t.ATMOSPHERE
    assert t.pressure(10.0) - t.ATMOSPHERE == pytest.approx(98100.0)


# --- echosounder

def test_the_echosounder_reports_the_first_echo(t):
    assert t.first_echo([21.0, 20.4, math.inf, 20.9], 0.3, 150.0) == 20.4


@pytest.mark.parametrize('ranges', [[], [math.inf] * 25, [math.nan, math.inf], [0.1, 200.0]])
def test_no_valid_echo_is_infinity(t, ranges):
    assert t.first_echo(ranges, 0.3, 150.0) == math.inf


def test_echoes_out_of_the_range_of_the_sounder_are_ignored(t):
    assert t.first_echo([0.2, math.nan, 30.0, 151.0], 0.3, 150.0) == 30.0


# --- DVL

def beams(tilt=math.radians(30.0), sign=-1.0):
    s, c = math.sin(tilt), math.cos(tilt)
    return [(s * math.cos(a), s * math.sin(a), sign * c)
            for a in (math.radians(r) for r in (45, 135, -45, -135))]


def test_the_altitude_is_the_vertical_part_of_the_beams_ranges(t):
    slant = 20.0 / math.cos(math.radians(30.0))
    assert t.vertical_altitude([slant] * 4, beams(), 99.0) == pytest.approx(20.0)


def test_the_altitude_does_not_depend_on_the_sign_convention_of_the_beams(t):
    mixed = beams()[:2] + beams(sign=1.0)[2:]
    assert t.vertical_altitude([10.0] * 4, mixed, 99.0) == pytest.approx(
        10.0 * math.cos(math.radians(30.0)))


def test_the_altitude_uses_the_beams_that_came_back_with_three_at_least(t):
    slant = 20.0 / math.cos(math.radians(30.0))
    assert t.vertical_altitude([slant, slant, slant, math.inf], beams(), 99.0) == \
        pytest.approx(20.0)
    assert t.vertical_altitude([slant, slant, math.inf, 0.0], beams(), 99.0) == 99.0


def test_without_the_beams_directions_the_reported_altitude_stays(t):
    assert t.vertical_altitude([10.0] * 4, [(0.0, 0.0, 0.0)] * 4, 7.5) == 7.5


def test_a_dvl_with_the_bottom_gives_its_velocity_and_covariance(t):
    cov = [1e-4, 0, 0, 0, 2e-4, 0, 0, 0, 3e-4]
    v, c = t.dvl_twist((1.0, -0.1, 0.02), cov, 4, 12.0)
    assert v == [1.0, -0.1, 0.02]
    assert np.diag(c)[:3] == pytest.approx([1e-4, 2e-4, 3e-4])
    assert np.all(np.diag(c)[3:] == t.UNKNOWN)               # it does not measure rotation
    assert c.shape == (6, 6) and np.all(c[:3, 3:] == 0.0)


def test_an_unknown_covariance_is_the_noise_of_the_dvl(t):
    for cov in ([-1.0] * 9, [0.0] * 9, [math.nan] * 9):
        _, c = t.dvl_twist((1.0, 0.0, 0.0), cov, 4, 12.0)
        assert np.diag(c)[:3] == pytest.approx([t.DVL_STDDEV ** 2] * 3)


@pytest.mark.parametrize('beams_, altitude, velocity', [
    (2, 12.0, (1.0, 0.0, 0.0)),            # fewer than three beams: no bottom lock
    (4, math.inf, (1.0, 0.0, 0.0)),        # no bottom in range
    (4, 0.0, (1.0, 0.0, 0.0)),
    (4, None, (1.0, 0.0, 0.0)),
    (4, 12.0, (math.nan, 0.0, 0.0)),
])
def test_without_bottom_lock_there_is_no_twist(t, beams_, altitude, velocity):
    assert t.dvl_twist(velocity, [-1.0] * 9, beams_, altitude) is None


def test_three_beams_are_enough(t):
    assert t.dvl_twist((0.5, 0.0, 0.0), [-1.0] * 9, 3, 5.0) is not None


# --- the lidar over the water

LIDAR = (0.7, 0.0, 1.8, 0.0, 0.0, 0.0)       # where the boat carries it (marine_sensors.py)
AT_REST = (0.0, 0.0, -0.08)


def test_the_lidar_keeps_what_is_over_the_water_and_drops_the_sea_floor(t):
    points = [(50.0, 0.0, 10.0),      # a cliff
              (30.0, 0.0, -1.0),      # just over the water (the lidar is 1.72 m over it)
              (60.0, 0.0, -21.0),     # the sea floor, seen through the water
              (0.0, 0.0, -1.73)]      # just under the surface
    assert list(t.above_water(points, LIDAR, AT_REST, LEVEL)) == [True, True, False, False]


def test_the_lidar_filter_follows_the_attitude_of_the_boat(t):
    # pitched 20 degrees nose down: a point 30 m ahead at the lidar's height is under the water
    point = [(30.0, 0.0, 0.0)]
    assert t.above_water(point, LIDAR, AT_REST, LEVEL)[0]
    assert not t.above_water(point, LIDAR, AT_REST, about_y(math.radians(20.0)))[0]
    assert t.above_water(point, LIDAR, AT_REST, about_y(math.radians(-20.0)))[0]


def test_the_lidar_filter_uses_the_mount_of_the_sensor(t):
    tilted = LIDAR[:3] + (0.0, math.radians(30.0), 0.0)        # looking 30 degrees down
    assert not t.above_water([(10.0, 0.0, 0.0)], tilted, AT_REST, LEVEL)[0]


def test_an_empty_cloud_keeps_nothing(t):
    assert t.above_water(np.zeros((0, 3)), LIDAR, AT_REST, LEVEL).shape == (0,)


def test_rpy_matrix_is_a_rotation(t):
    m = t.rpy_matrix(0.3, -0.7, 2.0)
    assert m @ m.T == pytest.approx(np.eye(3))
    assert np.linalg.det(m) == pytest.approx(1.0)
    assert t.rpy_matrix(0.0, 0.0, math.pi / 2) @ [1, 0, 0] == pytest.approx([0, 1, 0])


# --- the Dvl message of Rolling and Lyrical

class Vector3:
    pass


def buggy_class():
    class Dvl:
        def __init__(self, *, altitude=None, range=None, beam_unit_vec=None):  # noqa: A002
            self.altitude = altitude
            self.range = range
            # as rosidl_generator_py writes it: `range` is the parameter, not the builtin
            self.beam_unit_vec = beam_unit_vec if beam_unit_vec is not None else [
                Vector3() for x in range(4)]
    return Dvl


def test_the_dvl_message_is_built_once_fixed(t):
    dvl = buggy_class()
    with pytest.raises(TypeError):
        dvl()
    t.fix_dvl_init(dvl, Vector3)
    m = dvl()
    assert len(m.beam_unit_vec) == 4 and all(isinstance(v, Vector3) for v in m.beam_unit_vec)
    m = dvl(altitude=3.0, range=[1.0] * 4)
    assert m.altitude == 3.0 and m.range == [1.0] * 4


def test_the_fix_keeps_given_beams_and_is_applied_once(t):
    dvl = buggy_class()
    t.fix_dvl_init(dvl, Vector3)
    first = dvl.__init__
    t.fix_dvl_init(dvl, Vector3)
    assert dvl.__init__ is first
    given = [Vector3() for _ in builtin_range(4)]
    assert dvl(beam_unit_vec=given).beam_unit_vec is given


builtin_range = range


def test_the_real_dvl_message_can_be_built_and_read_once_fixed(t):
    msgs = pytest.importorskip('marine_acoustic_msgs.msg')
    geometry = pytest.importorskip('geometry_msgs.msg')
    t.fix_dvl_init(msgs.Dvl, geometry.Vector3)
    m = msgs.Dvl(altitude=4.0)
    assert m.altitude == 4.0 and len(m.beam_unit_vec) == 4 and list(m.range) == [0.0] * 4
