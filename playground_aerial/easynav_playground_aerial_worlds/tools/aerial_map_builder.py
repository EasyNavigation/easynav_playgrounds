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

"""Build the town's 3D map (a PCD for Bonxai) with the UFO's lidars, flying it over a grid.

Usage (with the simulation running, gazebo_sim.launch.yaml, no EasyNav):
  python3 aerial_map_builder.py <out.pcd> [--area X0 Y0 X1 Y1] [--spacing M] [--heights Z ...]

The UFO is teleported (it has no gravity) to every point of the grid at each height, turned to
4 headings, and the clouds of its top and bottom lidars are put together in the world frame with
the pose Gazebo reports, keeping one point per voxel.
"""
import argparse
import math
import subprocess
import time

import numpy as np


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def cloud_xyz(msg):
    """XYZ of a PointCloud2 with float32 x, y, z fields."""
    offsets = {f.name: f.offset for f in msg.fields}
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    pts = np.stack([data[:, offsets[k]:offsets[k] + 4].copy().view(np.float32).ravel()
                    for k in 'xyz'], axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


def write_pcd(path, points):
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
        f'TYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(points.astype(np.float32).tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('output')
    parser.add_argument('--world', default='nathan_benderson_town')
    parser.add_argument('--model', default='ufo')
    parser.add_argument('--area', nargs=4, type=float, default=(-150.0, 690.0, 260.0, 1120.0),
                        metavar=('X0', 'Y0', 'X1', 'Y1'))
    parser.add_argument('--spacing', type=float, default=25.0)
    parser.add_argument('--heights', nargs='+', type=float, default=(12.0, 35.0))
    parser.add_argument('--voxel', type=float, default=0.3)
    parser.add_argument('--settle', type=float, default=0.4)
    parser.add_argument('--robot-cut', type=float, default=0.8)
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = Node('aerial_map_builder')
    state = {'odom': None, 'top': None, 'bottom': None}
    node.create_subscription(Odometry, 'odom', lambda m: state.__setitem__('odom', m), 10)
    node.create_subscription(PointCloud2, 'top_laser/points',
                             lambda m: state.__setitem__('top', m), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, 'bottom_laser/points',
                             lambda m: state.__setitem__('bottom', m), qos_profile_sensor_data)

    def spin_for(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def teleport(x, y, z, yaw):
        req = (f'name: "{args.model}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: '
               f'{{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        subprocess.run(['gz', 'service', '-s', f'/world/{args.world}/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '2000', '--req', req], capture_output=True)

    # Lidar mountings (from the URDF): top 0.25 m up, bottom 0.11 m down and upside down
    mounts = {'top': (np.array([0, 0, 0.25]), np.eye(3)),
              'bottom': (np.array([0, 0, -0.11]), np.diag([1.0, -1.0, -1.0]))}
    voxels = {}
    x0, y0, x1, y1 = args.area
    xs = np.arange(x0, x1 + 1e-6, args.spacing)
    ys = np.arange(y0, y1 + 1e-6, args.spacing)
    total = len(xs) * len(ys) * len(args.heights)
    done = 0
    for z in args.heights:
        for i, x in enumerate(xs):
            for y in (ys if i % 2 == 0 else ys[::-1]):
                for yaw in (0.0,):  # both lidars see all around
                    teleport(x, y, z, yaw)
                    state['top'] = state['bottom'] = None
                    spin_for(args.settle)
                    t0 = time.time()
                    while (state['top'] is None or state['bottom'] is None) and \
                            time.time() - t0 < 2.0:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    odom = state['odom']
                    if odom is None or state['top'] is None or state['bottom'] is None:
                        continue
                    p = odom.pose.pose.position
                    q = odom.pose.pose.orientation
                    R = quat_to_mat(q.x, q.y, q.z, q.w)
                    t = np.array([p.x, p.y, p.z])
                    for name in ('top', 'bottom'):
                        local = cloud_xyz(state[name])
                        offset, rot = mounts[name]
                        body = local @ rot.T + offset
                        body = body[np.linalg.norm(body, axis=1) > args.robot_cut]
                        world = body @ R.T + t
                        for k in map(tuple, np.floor(world / args.voxel).astype(np.int64)):
                            voxels[k] = True
                done += 1
                print(f'{done}/{total} positions, {len(voxels)} voxels', flush=True)
    points = (np.array(list(voxels.keys()), dtype=np.float64) + 0.5) * args.voxel
    write_pcd(args.output, points)
    print(f'wrote {args.output}: {len(points)} points')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
