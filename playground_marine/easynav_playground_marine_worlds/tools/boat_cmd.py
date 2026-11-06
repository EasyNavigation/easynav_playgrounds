#!/usr/bin/env python3
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

"""An autopilot for the WAM-V of Gazebo: a Twist in, the thrust of its two engines out.

The WAM-V is a catamaran with an engine on each float and no rudder: it goes forward with the sum of
the two thrusts and turns with their difference. This node takes a geometry_msgs/Twist on `cmd_vel`
and, with the pose it gets on `odom` (nav_msgs/Odometry: its speed along the axis, and the yaw rate worked out from the yaw), commands the
two propellers (std_msgs/Float64 on /<boat>/left_thrust and /<boat>/right_thrust, which the
ros_gz_bridge carries to Gazebo's joint topics):
  linear.x   m/s along its axis  -> the sum of the thrusts (N): what the drag asks for plus a correction
  angular.z  rad/s of yaw        -> their difference: the right engine pushes more to turn left
A command older than a second is no command: both engines stop and the boat coasts.
Measured on the model: 100 N on each engine hold 1.4 m/s, and 100 N more on the right one than on
the left turn it at about 0.6 rad/s.
"""

import argparse
import sys

import math

MAX_THRUST = 400.0      # N, each engine: the limit set in the model's Thruster plugins
MIN_THRUST = -200.0


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_of(q):
    """Yaw (rad) of a quaternion with x, y, z, w."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    """The angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class BoatController:
    """Left and right thrust for a commanded (speed, yaw rate)."""

    def __init__(self, drag_linear=50.0, drag_quadratic=30.0, speed_gain=600.0,
                 yaw_feedforward=20.0, yaw_gain=800.0):
        self.drag_linear, self.drag_quadratic = drag_linear, drag_quadratic
        self.speed_gain = speed_gain
        self.yaw_feedforward, self.yaw_gain = yaw_feedforward, yaw_gain

    def step(self, command, state):
        """(left, right) in N, from command (u, r) and state (u, r): speed along the axis and
        yaw rate (positive to the left)."""
        u_cmd, r_cmd = command
        u, r = state
        if not all(math.isfinite(v) for v in (u_cmd, r_cmd, u, r)):
            return 0.0, 0.0      # a NaN would come out of the clamps as full thrust: stop instead
        total = (self.drag_linear * u_cmd + self.drag_quadratic * u_cmd * abs(u_cmd) +
                 self.speed_gain * (u_cmd - u))
        difference = self.yaw_feedforward * r_cmd + self.yaw_gain * (r_cmd - r)
        left = 0.5 * (total - difference)
        right = 0.5 * (total + difference)
        return clamp(left, MIN_THRUST, MAX_THRUST), clamp(right, MIN_THRUST, MAX_THRUST)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--boat', default='boat', help='the prefix of the thrust topics')
    parser.add_argument('--rate', type=float, default=20.0, help='Hz')
    parser.add_argument('--timeout', type=float, default=1.0, help='s a command is good for')
    parser.add_argument('--sim-time', action='store_true')
    args, ros_args = parser.parse_known_args(argv)

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from std_msgs.msg import Float64

    rclpy.init(args=ros_args)
    node = rclpy.create_node(
        'boat_cmd', parameter_overrides=[Parameter('use_sim_time', value=bool(args.sim_time))])
    left_pub = node.create_publisher(Float64, '/%s/left_thrust' % args.boat, 10)
    right_pub = node.create_publisher(Float64, '/%s/right_thrust' % args.boat, 10)
    controller = BoatController()
    # the gains are ROS parameters, so that they can be tuned in a running system:
    #   ros2 param set /boat_cmd yaw_gain 400.0
    for name in ('speed_gain', 'yaw_gain', 'yaw_feedforward'):
        node.declare_parameter(name, getattr(controller, name))
    seen = {'command': (0.0, 0.0), 'time': -math.inf, 'state': (0.0, 0.0), 'yaw': None,
            'yaw_time': None, 'r': 0.0}

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def on_cmd(msg):
        seen['command'] = (msg.linear.x, msg.angular.z)
        seen['time'] = now()

    def on_odom(msg):
        # the yaw rate from the yaw itself: the twist of the model's odometry did not follow it
        # (it read -0.03 rad/s while the boat turned at +0.2)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = yaw_of(msg.pose.pose.orientation)
        if seen['yaw'] is not None and t > seen['yaw_time']:
            raw = wrap(yaw - seen['yaw']) / (t - seen['yaw_time'])
            seen['r'] += 0.2 * (raw - seen['r'])
        seen['yaw'], seen['yaw_time'] = yaw, t
        seen['state'] = (msg.twist.twist.linear.x, seen['r'])

    def tick():
        for name in ('speed_gain', 'yaw_gain', 'yaw_feedforward'):
            setattr(controller, name, node.get_parameter(name).value)
        if now() - seen['time'] <= args.timeout:
            left, right = controller.step(seen['command'], seen['state'])
        else:
            left = right = 0.0
        left_pub.publish(Float64(data=float(left)))
        right_pub.publish(Float64(data=float(right)))

    node.create_subscription(Twist, 'cmd_vel', on_cmd, 10)
    node.create_subscription(Odometry, 'odom', on_odom, 50)
    node.create_timer(1.0 / args.rate, tick)
    node.get_logger().info('Twist on cmd_vel -> thrust of the engines of %s' % args.boat)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
