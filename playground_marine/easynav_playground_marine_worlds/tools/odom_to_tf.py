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

"""The pose of a simulated vehicle as the transforms EasyNav asks for.

A simulator gives the pose of a vehicle in its world as nav_msgs/Odometry (`odom`: the world is
the map frame, the pose is where the vehicle is). This node publishes it as the transform
odom -> base_footprint, and the two fixed ones EasyNav wants: map -> odom (identity) and
base_footprint -> base_link (identity). It is what a robot's own odometry and description do for
the Summit; the drone of the playground has neither.

--hover: a multicopter that has not been commanded yet does not fly (its controller starts with
the first command: until then it falls). With this flag the node publishes an empty Twist on
`cmd_vel` until somebody else (not EasyNav's controller node) publishes there, so the drone hovers where it was put. (The
controller then keeps the last command: an empty Twist makes it hover again.)
"""

import argparse
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--link-frame', default='base_link')
    parser.add_argument('--sim-time', action='store_true')
    parser.add_argument('--hover', action='store_true',
                        help='an empty cmd_vel until another node publishes on it')
    args, ros_args = parser.parse_known_args(argv)

    import rclpy
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    rclpy.init(args=ros_args)
    node = rclpy.create_node(
        'odom_to_tf', parameter_overrides=[Parameter('use_sim_time', value=bool(args.sim_time))])
    tf = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    fixed = []
    for parent, child in ((args.map_frame, args.odom_frame), (args.base_frame, args.link_frame)):
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.rotation.w = 1.0
        fixed.append(t)
    static.sendTransform(fixed)

    def on_odom(msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = args.odom_frame
        t.child_frame_id = args.base_frame
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t.transform.translation.x, t.transform.translation.y = p.x, p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q
        tf.sendTransform(t)

    node.create_subscription(Odometry, 'odom', on_odom, 50)
    if args.hover:
        hover = node.create_publisher(Twist, 'cmd_vel', 10)

        # EasyNav's controller_node publishes there too (nothing, with the dummy controller):
        # whoever else it is, commands
        ours = ('odom_to_tf', 'controller_node')

        def keep_hovering():
            others = [i for i in node.get_publishers_info_by_topic('cmd_vel')
                      if i.node_name not in ours]
            if others:
                timer.cancel()
                node.get_logger().info(
                    'cmd_vel is commanded by %s: no more hover' % others[0].node_name)
                return
            hover.publish(Twist())
        timer = node.create_timer(0.2, keep_hovering)
    node.get_logger().info('odom -> %s -> %s' % (args.base_frame, args.link_frame))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
