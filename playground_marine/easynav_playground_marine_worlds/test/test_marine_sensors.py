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

"""The table of the vehicles' sensors (tools/marine_sensors.py): the SDF, the bridge and the static
transforms that come out of it agree with each other and with the world."""

import importlib.util
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOTS = ('boat', 'submarine')


def tool(name):
    spec = importlib.util.spec_from_file_location(
        name + '_under_test', os.path.join(ROOT, 'tools', name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def ms():
    return tool('marine_sensors')


def parsed(ms, robot):
    """The <sensor> elements of the SDF (gz: and experimental: given a namespace to parse)."""
    text = ms.sdf(robot)
    root = ET.fromstring('<root xmlns:gz="gz" xmlns:experimental="experimental">%s</root>' % text)
    params = root.find('{experimental}params')
    assert params is not None
    return params.findall('sensor')


def rotate(q, v):
    x, y, z, w = q
    m = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return m @ np.asarray(v, float)


def frames(ms, robot):
    return {f[0]: (np.array(f[1:4]), np.array(f[4:])) for f in ms.static_frames(robot)}


def test_an_unknown_vehicle_has_no_sensors(ms):
    with pytest.raises(ValueError, match='no sensors'):
        ms.sensors('drone')


@pytest.mark.parametrize('robot', ROBOTS)
def test_every_gazebo_sensor_is_in_the_sdf_with_its_pose_frame_and_topic(ms, robot):
    elements = {e.get('name'): e for e in parsed(ms, robot)}
    gazebo = [s for s in ms.sensors(robot) if s.sdf is not None]
    assert len(elements) == len(gazebo)
    for s in gazebo:
        e = next(e for e in elements.values() if e.find('gz_frame_id').text == s.frame and
                 e.find('topic').text in [b[0].replace('/points', '') for b in s.bridge])
        assert e.get('element_id') == 'base_link' and e.get('action') == 'add'
        pose = [float(v) for v in e.find('pose').text.split()]
        assert pose == pytest.approx(ms.model_pose(robot, s.pose), abs=1e-4)
        assert e.find('always_on').text == '1' and float(e.find('update_rate').text) > 0.0


@pytest.mark.parametrize('robot', ROBOTS)
def test_topics_are_not_shared(ms, robot):
    entries = ms.bridge_entries(robot)
    ros = [e[1] for e in entries]
    gz = [e[0] for e in entries]
    assert len(set(ros)) == len(ros) and len(set(gz)) == len(gz)
    assert all(e[4] == 'GZ_TO_ROS' for e in entries)
    assert all(t.startswith('/sensors/') for t in gz)


@pytest.mark.parametrize('robot', ROBOTS)
def test_the_bridge_types_are_ones_ros_gz_bridge_knows(ms, robot):
    mappings = pytest.importorskip('ros_gz_bridge.mappings')
    known = {('%s/msg/%s' % (pkg, m.ros_type), 'gz.msgs.%s' % m.gz_type)
             for pkg, ms_ in mappings.MAPPINGS.items() for m in ms_}
    for _, _, ros_type, gz_type, _ in ms.bridge_entries(robot):
        assert (ros_type, gz_type) in known, (ros_type, gz_type)


def test_the_boat_has_the_sensors_of_a_surface_vehicle(ms):
    names = {s.name for s in ms.sensors('boat')}
    assert names == {'imu', 'magnetometer', 'navsat', 'lidar', 'camera', 'echosounder'}
    ros = {e[1] for e in ms.bridge_entries('boat')}
    # the lidar goes through tools/marine_sensor_node.py, which drops what is under the water
    assert '/lidar/rays' in ros and '/lidar/points' not in ros
    assert {'/imu/data', '/imu/mag', '/gps/raw', '/camera/image_raw', '/camera/camera_info',
            '/echosounder/scan'} <= ros


def test_the_submarine_has_the_sensors_of_an_auv(ms):
    names = {s.name for s in ms.sensors('submarine')}
    assert names == {'imu', 'magnetometer', 'navsat', 'pressure', 'dvl', 'sonar'}
    assert ms.sensor('submarine', 'pressure').sdf is None      # worked out by the sensor node
    ros = {e[1] for e in ms.bridge_entries('submarine')}
    assert {'/imu/data', '/imu/mag', '/gps/raw', '/dvl/data', '/sonar/rays'} <= ros


@pytest.mark.parametrize('robot', ROBOTS)
def test_ray_sensors_do_not_see_the_water_and_the_camera_does(ms, robot):
    for e in parsed(ms, robot):
        if e.get('type') == 'gpu_lidar':
            mask = int(e.find('lidar/visibility_mask').text)
            assert mask & ms.WATER_VISIBILITY == 0
            assert mask | ms.WATER_VISIBILITY == 0xFFFFFFFF       # everything else it sees
        if e.get('type') == 'camera':
            assert e.find('camera/visibility_mask') is None


def test_the_water_is_tagged_with_the_flag_the_sensors_leave_out(ms):
    sea = tool('synthetic_sea')
    assert sea.WATER_VISIBILITY == ms.WATER_VISIBILITY
    for shader in (None, {'dir': '/m', 'mesh': 'file:///w.obj', 'params': []}):
        water = sea.water_sdf(100.0, shader)
        assert water.count('<visual') == 2
        assert water.count('<visibility_flags>%d</visibility_flags>' % ms.WATER_VISIBILITY) == 2


def test_the_gnss_noise_is_given_in_degrees(ms):
    e = next(e for e in parsed(ms, 'boat') if e.get('type') == 'navsat')
    horizontal = float(e.find('navsat/position_sensing/horizontal/noise/stddev').text)
    assert horizontal * 111320.0 == pytest.approx(1.0, rel=1e-3)          # 1 m
    assert float(e.find('navsat/position_sensing/vertical/noise/stddev').text) == 2.0


def test_the_dvl_is_a_four_beam_janus_looking_down(ms):
    e = next(e for e in parsed(ms, 'submarine') if e.get('name') == 'dvl')
    assert e.get('{gz}type') == 'dvl'
    beams = e.findall('{gz}dvl/arrangement/beam')
    assert sorted(int(b.find('rotation').text) for b in beams) == [-135, -45, 45, 135]
    assert all(b.find('tilt').text == '30' for b in beams)
    s = ms.sensor('submarine', 'dvl')
    assert s.pose[3:] == (0.0, 0.0, 0.0) and s.pose[2] < 0.0      # under the hull, level


@pytest.mark.parametrize('robot', ROBOTS)
def test_every_frame_has_one_static_transform_and_unit_quaternions(ms, robot):
    names = [f[0] for f in ms.static_frames(robot)]
    assert len(names) == len(set(names))
    assert {s.frame for s in ms.sensors(robot)} <= set(names)
    for _, (_, q) in frames(ms, robot).items():
        assert np.linalg.norm(q) == pytest.approx(1.0)


def test_the_camera_images_are_in_the_optical_frame_of_its_body(ms):
    f = frames(ms, 'boat')
    body, optical = f['camera_link'][1], f['camera_optical_link'][1]
    pitch = ms.sensor('boat', 'camera').pose[4]
    forward = np.array([math.cos(pitch), 0.0, -math.sin(pitch)])     # 10 degrees down
    assert rotate(body, [1, 0, 0]) == pytest.approx(forward)
    assert rotate(optical, [0, 0, 1]) == pytest.approx(forward)       # z: where it looks
    assert rotate(optical, [1, 0, 0]) == pytest.approx([0, -1, 0])    # x: to the right
    assert rotate(optical, [0, 1, 0]) == pytest.approx(rotate(body, [0, 0, -1]))  # y: down


def test_the_echosounder_looks_straight_down(ms):
    q = frames(ms, 'boat')['echosounder_link'][1]
    assert rotate(q, [1, 0, 0]) == pytest.approx([0, 0, -1], abs=1e-9)


def test_the_sonar_looks_where_the_submarine_goes_and_its_image_frame_is_acoustic(ms):
    f = frames(ms, 'submarine')
    rays, acoustic = f['sonar_rays_link'][1], f['sonar_link'][1]
    assert f['sonar_rays_link'][0][0] > 0.0                           # at the bow
    ahead = rotate(rays, [1, 0, 0])
    assert ahead[0] > 0.0 and ahead[2] < 0.0                          # forward and down
    # marine_acoustic_msgs: z forward, x up, y to starboard
    assert rotate(acoustic, [0, 0, 1]) == pytest.approx(ahead)
    assert rotate(acoustic, [1, 0, 0]) == pytest.approx(rotate(rays, [0, 0, 1]))
    assert rotate(acoustic, [0, 1, 0]) == pytest.approx(rotate(rays, [0, -1, 0]))
    assert f['sonar_link'][0] == pytest.approx(f['sonar_rays_link'][0])


def test_the_sonar_rays_cover_its_field_of_view_more_finely_than_its_beams(ms):
    s = ms.SONAR
    e = next(e for e in parsed(ms, 'submarine') if e.get('name') == 'sonar_rays')
    h = e.find('lidar/scan/horizontal')
    v = e.find('lidar/scan/vertical')
    assert int(h.find('samples').text) == s['rays_horizontal'] >= 2 * s['beams']
    assert int(v.find('samples').text) == s['rays_vertical']
    assert float(h.find('max_angle').text) - float(h.find('min_angle').text) == \
        pytest.approx(s['horizontal_fov'], abs=1e-5)
    assert float(v.find('max_angle').text) - float(v.find('min_angle').text) == \
        pytest.approx(s['vertical_fov'], abs=1e-5)
    assert float(e.find('lidar/range/max').text) == s['max_range']


def test_the_tethys_model_is_turned_so_that_base_link_looks_to_its_bow(ms):
    # the model has its propeller (x = 1.43) and fins at +x: its bow is at -x
    assert ms.MODEL_YAW['submarine'] == pytest.approx(math.pi)
    assert ms.MODEL_YAW['boat'] == 0.0
    sonar = ms.model_pose('submarine', ms.sensor('submarine', 'sonar').pose)
    assert sonar[0] < 0.0                                             # at the model's bow
    q = ms._quaternion(*sonar[3:])
    assert rotate(q, [1, 0, 0])[0] < 0.0 and rotate(q, [1, 0, 0])[2] < 0.0   # to the bow, down


@pytest.mark.parametrize('pose', [(0.3, -0.2, 0.1, 0.1, 0.2, 0.3), (1.0, 0.0, 0.0, 0.0, 0.35, 0.0),
                                  (0.0, 0.0, -0.13, 0.0, 0.0, 0.0)])
def test_a_pose_in_base_link_is_the_same_place_and_direction_in_the_model(ms, pose):
    m = ms.MODEL_YAW['submarine']
    model = ms.model_pose('submarine', pose)
    turn = ms._quaternion(0.0, 0.0, -m)                              # base_link in the model
    assert np.asarray(model[:3]) == pytest.approx(rotate(turn, pose[:3]), abs=1e-9)
    direction = rotate(ms._multiply(turn, ms._quaternion(*pose[3:])), [1, 0, 0])
    assert rotate(ms._quaternion(*model[3:]), [1, 0, 0]) == pytest.approx(direction, abs=1e-9)
    assert ms.model_pose('boat', pose) == pytest.approx(pose)         # the boat is not turned
