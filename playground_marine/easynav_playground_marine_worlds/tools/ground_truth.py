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

"""The simulator's pose of a vehicle whose model is turned in yaw from its ROS base_link.

The Tethys model has its propeller at +x; in ROS its base_link has x to the nose (REP 103). Gazebo
gives the odometry of the model's link (`model_odom`); this node turns its body frame by --yaw (the
model's yaw in base_link) and publishes it as `odom`: the same position, the orientation of
base_link, and the twist in base_link.
"""

import argparse
import math
import sys

from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter


def turn(msg, yaw):
    """`msg` (an Odometry of the model's link) with its body frame turned to base_link, in place.

    `yaw` is the model's yaw in base_link (as marine_sensors.MODEL_YAW): base_link is the model's
    link turned by -yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    # orientation: q_base = q_model * q(-yaw)
    q = msg.pose.pose.orientation
    hz, hw = math.sin(-yaw / 2), math.cos(yaw / 2)
    q.x, q.y, q.z, q.w = (q.x * hw + q.y * hz, q.y * hw - q.x * hz,
                          q.w * hz + q.z * hw, q.w * hw - q.z * hz)
    # twist: from the model's axes to base_link's, v_base = Rz(yaw) v_model
    for v in (msg.twist.twist.linear, msg.twist.twist.angular):
        v.x, v.y = c * v.x - s * v.y, s * v.x + c * v.y
    return msg


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--yaw', type=float, default=math.pi,
                        help="the model's yaw in base_link, rad (default: half a turn)")
    parser.add_argument('--sim-time', action='store_true')
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = rclpy.create_node('ground_truth')
    if args.sim_time:
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Odometry, 'odom', 50)
    node.create_subscription(Odometry, 'model_odom', lambda m: pub.publish(turn(m, args.yaw)), 50)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
