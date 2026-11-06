#!/usr/bin/env python3
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

"""The sensors of the marine playground's vehicles, in one table.

From it the launch file makes the SDF of the sensors (added to the vehicle's base_link with
<experimental:params>), the entries of the ros_gz bridge and the static transforms
base_link -> <sensor frame>, so the three always agree. Some sensors are not Gazebo's: the depth
gauge (Gazebo has no water pressure sensor) is worked out by tools/marine_sensor_node.py, and the
imaging sonar's echoes by tools/sonar_node.py from a fan of rays.

Ray sensors do not see the water's surface (its visuals carry WATER_VISIBILITY and their mask leaves
it out): the surface is a drawing, not an echo. The acoustic ones (echosounder, sonar) see the sea
floor through it, as they do; the boat's lidar would see it too, so tools/marine_sensor_node.py drops
the points under the water.
"""

import math
from collections import namedtuple

# visibility_flags of the water's visuals (tools/synthetic_sea.py has the same value), and the
# visibility_mask of the ray sensors that must not see them
WATER_VISIBILITY = 0x00000002
SEES_NO_WATER = 0xFFFFFFFF & ~WATER_VISIBILITY

# The yaw of each vehicle's Gazebo model in its ROS base_link (x to the bow, REP 103). The Tethys
# model has its propeller and fins at +x: it is turned half a turn, and tools/ground_truth.py turns its
# odometry back. The poses of the table are in base_link; sdf() puts them in the model's link.
MODEL_YAW = {'boat': 0.0, 'submarine': math.pi}

# name, frame (frame_id of its messages), pose (x, y, z, roll, pitch, yaw in base_link), sdf (the
# body of its <sensor>, without pose and frame; None: not a Gazebo sensor), bridge
# [(gz topic, ROS topic, ROS type, gz type)]
Sensor = namedtuple('Sensor', 'name frame pose sdf bridge')

GZ_TO_ROS = 'GZ_TO_ROS'


def _noise(stddev, bias_mean=None, bias_stddev=None):
    bias = ''
    if bias_mean is not None:
        bias = '<bias_mean>%g</bias_mean><bias_stddev>%g</bias_stddev>' % (bias_mean, bias_stddev)
    return '<noise type="gaussian"><mean>0</mean><stddev>%g</stddev>%s</noise>' % (stddev, bias)


def _xyz(stddev, bias_mean=None, bias_stddev=None):
    n = _noise(stddev, bias_mean, bias_stddev)
    return ''.join('<%s>%s</%s>' % (a, n, a) for a in 'xyz')


def imu(topic='/sensors/imu', rate=100):
    """A MEMS AHRS (the noise of VRX's WAM-V IMU)."""
    return ('<sensor name="imu" type="imu"><always_on>1</always_on>'
            '<update_rate>%g</update_rate><topic>%s</topic><imu>'
            '<angular_velocity>%s</angular_velocity>'
            '<linear_acceleration>%s</linear_acceleration></imu></sensor>'
            % (rate, topic, _xyz(2e-4, 7.5e-6, 8e-7), _xyz(1.7e-2, 0.1, 0.001)))


def magnetometer(topic='/sensors/magnetometer', rate=20):
    """A compass: the Earth's field (~5e-5 T) with 2e-7 T of noise."""
    return ('<sensor name="magnetometer" type="magnetometer"><always_on>1</always_on>'
            '<update_rate>%g</update_rate><topic>%s</topic>'
            '<magnetometer>%s</magnetometer></sensor>' % (rate, topic, _xyz(2e-7)))


def navsat(topic='/sensors/navsat', rate=5, horizontal_m=1.0, vertical_m=2.0):
    """A GNSS receiver. Gazebo adds the horizontal noise to the latitude and the longitude in
    degrees: horizontal_m is turned into degrees of latitude."""
    degrees = horizontal_m / 111320.0
    return ('<sensor name="navsat" type="navsat"><always_on>1</always_on>'
            '<update_rate>%g</update_rate><topic>%s</topic><navsat>'
            '<position_sensing><horizontal>%s</horizontal><vertical>%s</vertical>'
            '</position_sensing><velocity_sensing><horizontal>%s</horizontal>'
            '<vertical>%s</vertical></velocity_sensing></navsat></sensor>'
            % (rate, topic, _noise(degrees), _noise(vertical_m), _noise(0.05), _noise(0.1)))


def gpu_lidar(name, topic, rate, horizontal, vertical, ranges, stddev):
    """A ray sensor that does not see the water: horizontal and vertical are (samples, min, max)
    angles in rad, ranges (min, max) m."""
    def axis(tag, spec):
        samples, lo, hi = spec
        return ('<%s><samples>%d</samples><resolution>1</resolution><min_angle>%.6f</min_angle>'
                '<max_angle>%.6f</max_angle></%s>' % (tag, samples, lo, hi, tag))
    return ('<sensor name="%s" type="gpu_lidar"><always_on>1</always_on>'
            '<update_rate>%g</update_rate><topic>%s</topic><lidar><scan>%s%s</scan>'
            '<range><min>%g</min><max>%g</max><resolution>0.01</resolution></range>%s'
            '<visibility_mask>%d</visibility_mask></lidar></sensor>'
            % (name, rate, topic, axis('horizontal', horizontal), axis('vertical', vertical),
               ranges[0], ranges[1], _noise(stddev), SEES_NO_WATER))


def camera(topic='/sensors/camera/image', rate=15, width=1280, height=720, fov=1.3963):
    """A colour camera (it sees the water, as a camera does)."""
    return ('<sensor name="camera" type="camera"><always_on>1</always_on>'
            '<update_rate>%g</update_rate><topic>%s</topic><camera>'
            '<horizontal_fov>%g</horizontal_fov><image><width>%d</width><height>%d</height>'
            '<format>R8G8B8</format></image><clip><near>0.1</near><far>500</far></clip>'
            '%s</camera></sensor>' % (rate, topic, fov, width, height, _noise(0.007)))


def dvl(topic='/sensors/dvl', rate=5, max_range=100.0):
    """A four-beam phased-array DVL in bottom-tracking mode (Gazebo's DopplerVelocityLog, the
    Teledyne Pathfinder of the gz-sim example): beams 30 degrees off the vertical, velocity in the
    frame of the sensor."""
    beams = ''.join(
        '<beam id="%d"><aperture>2</aperture><rotation>%d</rotation><tilt>30</tilt></beam>'
        % (i + 1, r) for i, r in enumerate((45, 135, -45, -135)))
    return ('<sensor name="dvl" type="custom" gz:type="dvl"><always_on>1</always_on>'
            '<update_rate>%g</update_rate><topic>%s</topic><gz:dvl><type>phased_array</type>'
            '<arrangement degrees="true">%s</arrangement><tracking><bottom_mode><when>best</when>'
            '%s<visualize>false</visualize></bottom_mode></tracking>'
            '<resolution>0.01</resolution><maximum_range>%g</maximum_range>'
            '<minimum_range>0.1</minimum_range><reference_frame>0 0 0 0 0 0</reference_frame>'
            '</gz:dvl></sensor>' % (rate, topic, beams, _noise(0.005), max_range))


# The imaging sonar: an Oculus M750d-like multibeam (750 kHz, 130 x 20 degrees), whose echoes
# tools/sonar_node.py works out from a fan of rays that covers its field of view more finely
SONAR = {'beams': 256, 'horizontal_fov': math.radians(130.0), 'vertical_fov': math.radians(20.0),
         'min_range': 0.5, 'max_range': 50.0, 'rate': 10.0, 'frequency': 750e3,
         'rays_horizontal': 512, 'rays_vertical': 32}


def sonar_rays(topic='/sensors/sonar'):
    s = SONAR
    h, v = 0.5 * s['horizontal_fov'], 0.5 * s['vertical_fov']
    return gpu_lidar('sonar_rays', topic, s['rate'], (s['rays_horizontal'], -h, h),
                     (s['rays_vertical'], -v, v), (s['min_range'], s['max_range']), 0.02)


def echosounder(topic='/sensors/echosounder'):
    """A single-beam echosounder: a cone of 8 degrees looking down (a 5 x 5 bundle of rays; the
    shortest is the depth under the keel)."""
    a = math.radians(4.0)
    return gpu_lidar('echosounder', topic, 5, (5, -a, a), (5, -a, a), (0.3, 150.0), 0.02)


def _points(topic):
    return (topic + '/points', 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked')


DOWN = (0.0, math.pi / 2, 0.0)       # roll, pitch, yaw that turn +x straight down

SENSORS = {
    # the WAM-V: its deck (top_base) is 1.25 m over base_link, its floats' bottom 0.02 m
    'boat': (
        Sensor('imu', 'imu_link', (0.0, 0.0, 1.3, 0.0, 0.0, 0.0), imu(),
               [('/sensors/imu', '/imu/data', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU')]),
        Sensor('magnetometer', 'imu_link', (0.0, 0.0, 1.3, 0.0, 0.0, 0.0), magnetometer(),
               [('/sensors/magnetometer', '/imu/mag', 'sensor_msgs/msg/MagneticField',
                 'gz.msgs.Magnetometer')]),
        Sensor('navsat', 'gps_link', (-0.5, 0.0, 1.6, 0.0, 0.0, 0.0), navsat(),
               [('/sensors/navsat', '/gps/raw', 'sensor_msgs/msg/NavSatFix', 'gz.msgs.NavSat')]),
        Sensor('lidar', 'lidar_link', (0.7, 0.0, 1.8, 0.0, 0.0, 0.0),
               gpu_lidar('lidar', '/sensors/lidar', 10, (900, -math.pi, math.pi),
                         (16, -math.radians(15.0), math.radians(15.0)), (0.9, 100.0), 0.01),
               # tools/marine_sensor_node.py drops what is under the water: /lidar/points
               [(_points('/sensors/lidar')[0], '/lidar/rays') + _points('')[1:]]),
        # x forward and 10 degrees down; its messages are in the optical frame (z forward)
        Sensor('camera', 'camera_optical_link', (0.75, 0.0, 1.6, 0.0, math.radians(10.0), 0.0),
               camera(),
               [('/sensors/camera/image', '/camera/image_raw', 'sensor_msgs/msg/Image',
                 'gz.msgs.Image'),
                ('/sensors/camera/camera_info', '/camera/camera_info',
                 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')]),
        # under the left float, looking down
        Sensor('echosounder', 'echosounder_link', (-0.4, 1.03, 0.0) + DOWN, echosounder(),
               [('/sensors/echosounder', '/echosounder/scan', 'sensor_msgs/msg/LaserScan',
                 'gz.msgs.LaserScan')]),
    ),
    # the Tethys: a hull of 2 x 0.3 x 0.25 m about base_link
    'submarine': (
        Sensor('imu', 'imu_link', (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), imu(),
               [('/sensors/imu', '/imu/data', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU')]),
        Sensor('magnetometer', 'imu_link', (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), magnetometer(),
               [('/sensors/magnetometer', '/imu/mag', 'sensor_msgs/msg/MagneticField',
                 'gz.msgs.Magnetometer')]),
        # on a short mast: it has a fix only when the vehicle is at the surface
        Sensor('navsat', 'gps_link', (0.0, 0.0, 0.25, 0.0, 0.0, 0.0), navsat(),
               [('/sensors/navsat', '/gps/raw', 'sensor_msgs/msg/NavSatFix', 'gz.msgs.NavSat')]),
        Sensor('pressure', 'pressure_link', (0.0, 0.0, -0.1, 0.0, 0.0, 0.0), None, []),
        Sensor('dvl', 'dvl_link', (0.0, 0.0, -0.13, 0.0, 0.0, 0.0), dvl(),
               [('/sensors/dvl', '/dvl/data', 'marine_acoustic_msgs/msg/Dvl',
                 'gz.msgs.DVLVelocityTracking')]),
        # at the bow, 20 degrees down (its fan reaches the floor 18 m under the vehicle from 36 m on,
        # as an obstacle-avoidance sonar is set)
        Sensor('sonar', 'sonar_rays_link', (1.0, 0.0, 0.0, 0.0, math.radians(20.0), 0.0),
               sonar_rays(),
               [(_points('/sensors/sonar')[0], '/sonar/rays') + _points('')[1:]]),
    ),
}

# Frames that are not a sensor's own: the acoustic frame of the sonar's images (marine_acoustic_msgs:
# z forward, x up, y to starboard), and the camera's body frame (x forward)
ACOUSTIC_FROM_RAYS = (0.0, -math.pi / 2, math.pi)     # rpy of sonar_link in sonar_rays_link


def _quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


# z forward, x right, y down (REP 103 optical) from x forward, y left, z up
OPTICAL = _quaternion(-math.pi / 2, 0.0, -math.pi / 2)


def _rpy(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def model_pose(robot, pose):
    """A pose in base_link (x, y, z, roll, pitch, yaw) in the link of the vehicle's model."""
    m = MODEL_YAW[robot]
    x, y, z = pose[:3]
    c, s = math.cos(m), math.sin(m)
    # base_link in the model: turned by -m
    q = _multiply(_quaternion(0.0, 0.0, -m), _quaternion(*pose[3:]))
    return (c * x + s * y, -s * x + c * y, z) + _rpy(q)


def sensors(robot):
    if robot not in SENSORS:
        raise ValueError("no sensors for '%s'" % robot)
    return SENSORS[robot]


def sensor(robot, name):
    return next(s for s in sensors(robot) if s.name == name)


def sdf(robot):
    """The <experimental:params> that adds the Gazebo sensors to the vehicle's base_link."""
    out = []
    for s in sensors(robot):
        if s.sdf is None:
            continue
        x, y, z, roll, pitch, yaw = model_pose(robot, s.pose)
        body = s.sdf.replace(
            '<always_on>', '<pose>%.4f %.4f %.4f %.6f %.6f %.6f</pose><gz_frame_id>%s</gz_frame_id>'
            '<always_on>' % (x, y, z, roll, pitch, yaw, s.frame), 1)
        out.append(body.replace('<sensor ', '<sensor element_id="base_link" action="add" ', 1))
    return '<experimental:params>\n        %s\n      </experimental:params>' % '\n        '.join(out)


def static_frames(robot):
    """(child, x, y, z, qx, qy, qz, qw) of base_link -> each frame of the sensors, once each."""
    out, seen = [], set()

    def add(child, xyz, q):
        if child not in seen:
            seen.add(child)
            out.append((child,) + tuple(xyz) + tuple(q))

    for s in sensors(robot):
        q = _quaternion(*s.pose[3:])
        if s.frame == 'camera_optical_link':
            add('camera_link', s.pose[:3], q)
            q = _multiply(q, OPTICAL)
        add(s.frame, s.pose[:3], q)
        if s.name == 'sonar':
            add('sonar_link', s.pose[:3], _multiply(q, _quaternion(*ACOUSTIC_FROM_RAYS)))
    return out


def bridge_entries(robot):
    """(gz topic, ROS topic, ROS type, gz type, direction) of every sensor."""
    return [entry + (GZ_TO_ROS,) for s in sensors(robot) for entry in s.bridge]
