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

"""An autopilot for the Tethys LRAUV of Gazebo: a Twist in, thrust and fin angles out.

The Tethys is a torpedo: it has a propeller and two pairs of fins and no thrusters to the side, so
it can only steer and dive while it moves forward. This node takes a geometry_msgs/Twist on `cmd_vel`
like any other vehicle and, with the pose it gets on `odom` (nav_msgs/Odometry, twist in the
vehicle's frame), commands the three joints of the model (std_msgs/Float64 on /<model>/thrust,
/<model>/rudder and /<model>/elevator, which the ros_gz_bridge carries to Gazebo's joint topics;
a ROS topic cannot have a token that starts with a number, as Gazebo's `.../vertical_fins_joint/0/...`):
  linear.x   m/s along its axis   -> propeller thrust (N): what the drag asks for plus a correction
  angular.z  rad/s of yaw         -> rudder angle (the vertical fins)
  linear.z   m/s up               -> elevator angle (the horizontal fins), through a pitch target
A command older than a second is no command: the thrust and the fins go to zero (the vehicle coasts).
Measured on the model: 31 N holds 2.2 m/s, a rudder of 0.17 rad turns it at 0.22 rad/s at 1.8 m/s,
and an elevator of +0.17 rad pitches it 40 degrees nose down.
"""

import argparse
import math
import sys

MAX_FIN = 0.26                 # rad: the limit of the fin joints of the model
MAX_THRUST = 60.0              # N
DRAG = 6.2282                  # N / (m/s)^2: xUabsU of its Hydrodynamics plugin
MIN_SPEED = 0.4                # m/s: below it the fins do nothing, and the dive is not asked for


def clamp(value, low, high):
    return max(low, min(high, value))


class AuvController:
    """Thrust, rudder and elevator for a commanded (speed, yaw rate, vertical speed)."""

    def __init__(self, speed_gain=20.0, yaw_gain=0.8, yaw_feedforward=0.35, pitch_gain=2.0,
                 pitch_rate_gain=1.5, max_pitch=math.radians(25.0)):
        self.speed_gain, self.yaw_gain, self.yaw_feedforward = speed_gain, yaw_gain, yaw_feedforward
        self.pitch_gain, self.pitch_rate_gain, self.max_pitch = pitch_gain, pitch_rate_gain, max_pitch

    def step(self, command, state):
        """(thrust, rudder, elevator) from command (u, r, vz) and state (u, r, pitch, q, vz).

        u: speed along the axis, r: yaw rate, pitch (positive nose down, as the model's), q: pitch
        rate, vz: vertical speed, up.
        """
        u_cmd, r_cmd, vz_cmd = command
        u, r, pitch, q, vz = state
        if not all(math.isfinite(v) for v in (u_cmd, r_cmd, vz_cmd, u, r, pitch, q, vz)):
            return 0.0, 0.0, 0.0      # a NaN would come out of the clamps as full thrust: coast
        thrust = DRAG * u_cmd * abs(u_cmd) + self.speed_gain * (u_cmd - u)
        thrust = clamp(thrust, -MAX_THRUST, MAX_THRUST)
        speed = max(abs(u), MIN_SPEED)
        # the fins act with the square of the speed: the same angle turns it less when it is slow
        rudder = (self.yaw_feedforward * r_cmd * 2.0 / speed + self.yaw_gain * (r_cmd - r))
        # climbing: nose up (negative pitch) in proportion to what is asked, while it moves
        if abs(u) < MIN_SPEED:
            pitch_target = 0.0
        else:
            pitch_target = clamp(-math.asin(clamp(vz_cmd / max(abs(u), 1.0), -1.0, 1.0)),
                                 -self.max_pitch, self.max_pitch)
            # the vertical speed it has is what is left to correct
            pitch_target = clamp(pitch_target - 0.3 * (vz_cmd - vz) / max(abs(u), 1.0),
                                 -self.max_pitch, self.max_pitch)
        elevator = self.pitch_gain * (pitch_target - pitch) - self.pitch_rate_gain * q
        return thrust, clamp(rudder, -MAX_FIN, MAX_FIN), clamp(elevator, -MAX_FIN, MAX_FIN)


def pitch_of(q):
    """Pitch (rad, positive nose down) of a quaternion with x, y, z, w."""
    return math.asin(clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--model', default='tethys')
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
        'auv_cmd', parameter_overrides=[Parameter('use_sim_time', value=bool(args.sim_time))])
    thrust_pub = node.create_publisher(Float64, '/%s/thrust' % args.model, 10)
    rudder_pub = node.create_publisher(Float64, '/%s/rudder' % args.model, 10)
    elevator_pub = node.create_publisher(Float64, '/%s/elevator' % args.model, 10)
    controller = AuvController()
    seen = {'command': (0.0, 0.0, 0.0), 'time': -math.inf, 'z': None, 'z_time': None, 'vz': 0.0,
            'state': (0.0, 0.0, 0.0, 0.0, 0.0)}

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def on_cmd(msg):
        seen['command'] = (msg.linear.x, msg.angular.z, msg.linear.z)
        seen['time'] = now()

    def on_odom(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        z = msg.pose.pose.position.z
        if seen['z'] is not None and t > seen['z_time']:
            raw = (z - seen['z']) / (t - seen['z_time'])
            seen['vz'] += 0.2 * (raw - seen['vz'])      # filtered: the pose is not smooth
        seen['z'], seen['z_time'] = z, t
        tw = msg.twist.twist
        seen['state'] = (tw.linear.x, tw.angular.z, pitch_of(msg.pose.pose.orientation),
                         tw.angular.y, seen['vz'])

    def tick():
        fresh = now() - seen['time'] <= args.timeout
        command = seen['command'] if fresh else (0.0, 0.0, 0.0)
        thrust, rudder, elevator = controller.step(command, seen['state']) if fresh else (
            0.0, 0.0, 0.0)
        for pub, value in ((thrust_pub, thrust), (rudder_pub, rudder), (elevator_pub, elevator)):
            pub.publish(Float64(data=float(value)))

    node.create_subscription(Twist, 'cmd_vel', on_cmd, 10)
    node.create_subscription(Odometry, 'odom', on_odom, 50)
    node.create_timer(1.0 / args.rate, tick)
    node.get_logger().info('Twist on cmd_vel -> thrust and fins of %s' % args.model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
