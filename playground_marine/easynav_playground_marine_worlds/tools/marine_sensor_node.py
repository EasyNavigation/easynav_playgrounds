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

"""What the marine sensors give that Gazebo does not give as such.

* GNSS (both vehicles): /gps/raw (Gazebo's NavSat, which has a fix anywhere) -> /gps/fix. With the
  antenna under water there is no fix (status NO_FIX, no position); after it comes out, the
  receiver needs `--reacquire` s to have one again.
* Depth gauge (the submarine): Gazebo has no water pressure sensor. From the vehicle's pose
  (/ground_truth/odom) and the gauge's place (tools/marine_sensors.py), the depth with noise, as
  /pressure (sensor_msgs/FluidPressure, Pa: the atmosphere plus rho g depth) and /depth
  (geometry_msgs/PoseWithCovarianceStamped in the odom frame, only z is known).
* DVL (the submarine): /dvl/data (marine_acoustic_msgs/Dvl, from Gazebo) -> /dvl/twist
  (geometry_msgs/TwistWithCovarianceStamped in the DVL's frame, only while it has the bottom; the
  same as /dvl/velocity, a TwistStamped, which RViz can draw) and /dvl/altitude (sensor_msgs/Range:
  the height over the bottom, from the beams' ranges: Gazebo's altitude is the range along them).
* Echosounder (the boat): /echosounder/scan (a cone of rays) -> /echosounder (sensor_msgs/Range,
  ULTRASOUND: the first echo, the shortest ray).
* LiDAR (the boat): /lidar/rays -> /lidar/points without the points under the water. Its rays go
  through the water's surface (a drawing, which they do not see) and would show the sea floor,
  which light does not reach back from.

The waves are left out of the pressure (a gauge deep enough barely feels them).
"""

import argparse
import builtins
import math
import os
import sys

import numpy as np

G = 9.81
RHO = 1000.0               # kg/m3, the water of the world
ATMOSPHERE = 101325.0      # Pa
DEPTH_STDDEV = 0.02        # m, a pressure gauge of ~0.01 % of its full scale
WET_AT = -0.05             # m: an antenna this far under the surface has no fix
UNKNOWN = 1e6              # variance of what a message does not measure
DVL_STDDEV = 0.005         # m/s, the noise of the DVL (tools/marine_sensors.py)


def rotation(q):
    """The matrix of a quaternion with x, y, z, w."""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0.0 else 0.0
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)]])


def point_z(position, orientation, offset):
    """World z of a point of the vehicle at `offset` (m, in base_link)."""
    return float(position[2] + (rotation(orientation) @ np.asarray(offset, float))[2])


class GnssGate:
    """No fix while the antenna is wet, and `reacquire` s after it is dry again."""

    def __init__(self, reacquire=5.0):
        self.reacquire = reacquire
        self.dry_since = None

    def has_fix(self, antenna_z, t):
        if antenna_z is None or antenna_z < WET_AT:
            self.dry_since = None
            return False
        if self.dry_since is None:
            self.dry_since = t
        return t - self.dry_since >= self.reacquire


def depth(sensor_z, rng=None, stddev=DEPTH_STDDEV):
    """Measured depth (m, positive down; 0 out of the water) of a gauge at world z."""
    d = max(0.0, -sensor_z)
    if rng is not None and stddev > 0.0:
        d = max(0.0, d + rng.normal(0.0, stddev))
    return d


def pressure(d):
    """Absolute pressure (Pa) at depth d."""
    return ATMOSPHERE + RHO * G * d


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def above_water(points, mount, position, orientation, level=0.0):
    """Which points (N, 3, in the frame of a sensor mounted at `mount`: x, y, z, roll, pitch, yaw
    in base_link) are over the water's surface at z = level, for a vehicle at position and
    orientation (quaternion x, y, z, w)."""
    points = np.asarray(points, float).reshape(-1, 3)
    body = points @ rpy_matrix(*mount[3:]).T + np.asarray(mount[:3], float)
    z = (body @ rotation(orientation).T)[:, 2] + position[2]
    return z > level


def first_echo(ranges, range_min, range_max):
    """The shortest valid range of a cone of rays, or +inf if none came back (REP 117)."""
    r = np.asarray(ranges, float)
    r = r[np.isfinite(r) & (r >= range_min) & (r <= range_max)]
    return float(r.min()) if r.size else math.inf


def vertical_altitude(ranges, unit_vectors, reported):
    """Height (m) over the bottom: the mean of each beam's range times the vertical part of its
    direction (Gazebo reports the range along the beams), with three beams at least; else
    `reported`."""
    r = np.asarray(ranges, float)
    u = np.asarray(unit_vectors, float).reshape(-1, 3)
    ok = np.isfinite(r) & (r > 0.0) & (np.abs(u[:, 2]) > 0.0)
    if ok.sum() < 3:
        return float(reported)
    return float(np.mean(r[ok] * np.abs(u[ok, 2])))


def dvl_twist(velocity, covariance, num_good_beams, altitude):
    """(linear velocity, 6x6 covariance) of a DVL reading, or None without bottom lock (fewer than
    three beams, or no altitude). Without a covariance (-1: unknown), the noise of the DVL of
    tools/marine_sensors.py."""
    if num_good_beams < 3 or not (altitude is not None and math.isfinite(altitude)
                                  and altitude > 0.0):
        return None
    v = [float(c) for c in velocity]
    if not all(math.isfinite(c) for c in v):
        return None
    cov = np.zeros((6, 6))
    given = np.asarray(covariance, float).reshape(3, 3)
    known = np.all(np.isfinite(given)) and np.all(np.diag(given) > 0.0)
    cov[:3, :3] = given if known else np.diag([DVL_STDDEV ** 2] * 3)
    cov[3:, 3:] = np.diag([UNKNOWN] * 3)          # a DVL does not measure rotation
    return v, cov


def fix_dvl_init(dvl_class, vector3_class):
    """Make marine_acoustic_msgs/Dvl usable from Python where rosidl_generator_py names the
    parameters of __init__ after the fields (Rolling, Lyrical): its field `range` hides the builtin,
    so the default of beam_unit_vec, [Vector3() for x in range(4)], fails, and so does every Dvl
    received (it is built with no arguments). The default is given from here; elsewhere it is
    harmless."""
    original = dvl_class.__init__
    if getattr(original, 'beam_unit_vec_default', False):
        return

    def init(self, *args, beam_unit_vec=None, **kwargs):
        if beam_unit_vec is None:
            beam_unit_vec = [vector3_class() for _ in builtins.range(4)]
        original(self, *args, beam_unit_vec=beam_unit_vec, **kwargs)

    init.beam_unit_vec_default = True
    dvl_class.__init__ = init


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--vehicle', choices=('boat', 'submarine'), required=True)
    parser.add_argument('--reacquire', type=float, default=5.0,
                        help='s a GNSS receiver needs to have a fix after the antenna is dry')
    parser.add_argument('--depth-rate', type=float, default=10.0, help='Hz of the depth gauge')
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--sim-time', action='store_true')
    args, ros_args = parser.parse_known_args(argv)

    import importlib.util
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped, Vector3
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from marine_acoustic_msgs.msg import Dvl
    fix_dvl_init(Dvl, Vector3)
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import FluidPressure, LaserScan, NavSatFix, NavSatStatus, PointCloud2, Range
    from sensor_msgs_py import point_cloud2

    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location(
        'marine_sensors', os.path.join(here, 'marine_sensors.py'))
    table = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(table)
    antenna = table.sensor(args.vehicle, 'navsat').pose[:3]
    gauge = (table.sensor(args.vehicle, 'pressure') if args.vehicle == 'submarine' else None)

    rclpy.init(args=ros_args)
    node = rclpy.create_node(
        'marine_sensors', parameter_overrides=[Parameter('use_sim_time', value=bool(args.sim_time))])
    rng = np.random.default_rng(args.seed)
    gate = GnssGate(args.reacquire)
    seen = {'pose': None}

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def on_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        seen['pose'] = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    node.create_subscription(Odometry, '/ground_truth/odom', on_odom, 50)

    fix_pub = node.create_publisher(NavSatFix, '/gps/fix', 10)

    def on_gps(msg):
        pose = seen['pose']
        z = point_z(pose[0], pose[1], antenna) if pose else None
        if not gate.has_fix(z, now()):
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = msg.longitude = msg.altitude = math.nan
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        fix_pub.publish(msg)

    node.create_subscription(NavSatFix, '/gps/raw', on_gps, qos_profile_sensor_data)

    if args.vehicle == 'boat':
        range_pub = node.create_publisher(Range, '/echosounder', 10)

        def on_scan(msg):
            out = Range()
            out.header = msg.header
            out.radiation_type = Range.ULTRASOUND
            out.field_of_view = msg.angle_max - msg.angle_min
            out.min_range, out.max_range = msg.range_min, msg.range_max
            out.range = first_echo(msg.ranges, msg.range_min, msg.range_max)
            range_pub.publish(out)

        node.create_subscription(LaserScan, '/echosounder/scan', on_scan, qos_profile_sensor_data)

        lidar = table.sensor('boat', 'lidar').pose
        points_pub = node.create_publisher(PointCloud2, '/lidar/points', 10)

        def on_rays(msg):
            pose = seen['pose']
            if pose is None:
                return
            cloud = point_cloud2.read_points(msg, skip_nans=True)
            if len(cloud) == 0:
                points_pub.publish(msg)
                return
            xyz = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=1).astype(float)
            kept = cloud[above_water(xyz, lidar, pose[0], pose[1])]
            out = point_cloud2.create_cloud(msg.header, msg.fields, kept)
            points_pub.publish(out)

        node.create_subscription(PointCloud2, '/lidar/rays', on_rays, qos_profile_sensor_data)
    else:
        pressure_pub = node.create_publisher(FluidPressure, '/pressure', 10)
        depth_pub = node.create_publisher(PoseWithCovarianceStamped, '/depth', 10)

        def tick():
            pose = seen['pose']
            if pose is None:
                return
            d = depth(point_z(pose[0], pose[1], gauge.pose[:3]), rng)
            stamp = node.get_clock().now().to_msg()
            p = FluidPressure()
            p.header.stamp, p.header.frame_id = stamp, gauge.frame
            p.fluid_pressure = pressure(d)
            p.variance = (RHO * G * DEPTH_STDDEV) ** 2
            pressure_pub.publish(p)
            m = PoseWithCovarianceStamped()
            m.header.stamp, m.header.frame_id = stamp, 'odom'
            m.pose.pose.position.z = -d - gauge.pose[2]      # base_link's z from the gauge's
            m.pose.pose.orientation.w = 1.0
            cov = np.diag([UNKNOWN, UNKNOWN, DEPTH_STDDEV ** 2, UNKNOWN, UNKNOWN, UNKNOWN])
            m.pose.covariance = cov.ravel().tolist()
            depth_pub.publish(m)

        node.create_timer(1.0 / args.depth_rate, tick)

        twist_pub = node.create_publisher(TwistWithCovarianceStamped, '/dvl/twist', 10)
        velocity_pub = node.create_publisher(TwistStamped, '/dvl/velocity', 10)
        altitude_pub = node.create_publisher(Range, '/dvl/altitude', 10)

        def on_dvl(msg):
            alt = Range()
            alt.header = msg.header
            alt.radiation_type = Range.ULTRASOUND
            alt.field_of_view = math.radians(60.0)       # four beams 30 degrees off the vertical
            alt.min_range, alt.max_range = 0.1, 100.0
            height = vertical_altitude(
                msg.range, [(u.x, u.y, u.z) for u in msg.beam_unit_vec], msg.altitude)
            locked = height > 0.0 and math.isfinite(height)
            alt.range = height if locked else math.inf
            altitude_pub.publish(alt)
            reading = dvl_twist((msg.velocity.x, msg.velocity.y, msg.velocity.z),
                                msg.velocity_covar, msg.num_good_beams, height)
            if reading is None:
                return
            v, cov = reading
            t = TwistWithCovarianceStamped()
            t.header = msg.header
            t.twist.twist.linear.x, t.twist.twist.linear.y, t.twist.twist.linear.z = v
            t.twist.covariance = cov.ravel().tolist()
            twist_pub.publish(t)
            velocity_pub.publish(TwistStamped(header=t.header, twist=t.twist.twist))

        node.create_subscription(Dvl, '/dvl/data', on_dvl, 10)

    node.get_logger().info('marine sensors of the %s' % args.vehicle)
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.try_shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                # Ctrl+C, or a second one while shutting down: nothing left to close
    return 0


if __name__ == '__main__':
    sys.exit(main())
