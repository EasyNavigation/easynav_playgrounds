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

"""The UFO's flight controller: from the commanded twist to the velocity Gazebo applies.

Subscribes to cmd_vel (geometry_msgs/Twist, body frame: linear x, y, z and angular z; roll and
pitch never change) and publishes the UFO's velocity on output_topic at rate Hz. The velocity
follows the command as a body with inertia: a first-order response (time_constant) bounded by
the acceleration and deceleration limits of each axis, after clamping the command to the
velocity limits. Without commands for cmd_timeout, the target is zero (the UFO brakes and hovers).
"""
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

AXES = ('x', 'y', 'z', 'yaw')


class UfoFlightController(Node):

    def __init__(self):
        super().__init__('ufo_flight_controller')
        p = self.declare_parameter
        self.rate = p('rate', 50.0).value
        self.time_constant = p('time_constant', 0.3).value
        self.timeout = p('cmd_timeout', 0.5).value
        limits = {}
        for kind, defaults in (('max_vel', (3.0, 1.5, 1.5)), ('max_acc', (1.5, 1.0, 2.0)),
                               ('max_decel', (2.5, 1.5, 3.0))):
            xy, z, yaw = (p(f'{kind}_{axis}', d).value for axis, d in zip(('xy', 'z', 'yaw'),
                                                                          defaults))
            limits[kind] = {'x': xy, 'y': xy, 'z': z, 'yaw': yaw}
        self.max_vel, self.acc, self.decel = limits['max_vel'], limits['max_acc'], \
            limits['max_decel']
        self.target = dict.fromkeys(AXES, 0.0)
        self.vel = dict.fromkeys(AXES, 0.0)
        self.last_cmd = None
        self.last_step = None
        self.pub = self.create_publisher(
            Twist, p('output_topic', '/model/ufo/cmd_vel').value, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.create_timer(1.0 / self.rate, self.step)

    def cmd_cb(self, msg):
        cmd = {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z, 'yaw': msg.angular.z}
        if not all(math.isfinite(v) for v in cmd.values()):
            return
        for a in AXES:
            self.target[a] = max(-self.max_vel[a], min(self.max_vel[a], cmd[a]))
        self.last_cmd = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        if self.last_step is None:
            self.last_step = now
            return
        dt = (now - self.last_step).nanoseconds * 1e-9
        self.last_step = now
        if dt <= 0.0:
            return  # paused simulation
        if self.last_cmd is None or (now - self.last_cmd).nanoseconds * 1e-9 > self.timeout:
            self.target = dict.fromkeys(AXES, 0.0)
        for a in AXES:
            v, t = self.vel[a], self.target[a]
            dv = (t - v) * min(1.0, dt / self.time_constant)
            speeding_up = abs(v + dv) > abs(v) and v * (v + dv) >= 0.0
            limit = (self.acc if speeding_up else self.decel)[a] * dt
            self.vel[a] = v + max(-limit, min(limit, dv))
        out = Twist()
        out.linear.x, out.linear.y, out.linear.z = self.vel['x'], self.vel['y'], self.vel['z']
        out.angular.z = self.vel['yaw']
        self.pub.publish(out)


def main():
    rclpy.init()
    node = UfoFlightController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
