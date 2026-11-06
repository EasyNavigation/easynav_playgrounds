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
# See the License for the specific language governing permissions and
# limitations under the License.

"""Waves for the marine playground: the force the sea adds to a vehicle that floats in a flat sea.

The water of the world is a flat surface at z = 0 and the buoyancy plugin of Gazebo, which knows nothing
about waves. This node makes up for it from outside: it reads the pose of the vehicle (nav_msgs/Odometry
on `odom`), works out what the water would add if its surface were not flat but a wave field, and
applies that to the vehicle as a force and a torque (ros_gz_interfaces/EntityWrench on `waves/wrench`,
which the ros_gz_bridge carries to Gazebo's ApplyLinkWrench system). The buoyancy of the world stays that
of a flat sea; the surface that is drawn is the same wave, by a shader (see below).

The wave field is two plane waves of deep water (omega^2 = g k): the main one, with the height, period
and direction given, and a second one with 0.35 of its amplitude, 0.7 of its period and a direction 40
degrees away, so that the motion does not repeat with a single period. Both start at t = 0 (the time of
the simulation, not of this node) and grow as 1 - exp(-t / tau). It is the field of the shader that
draws the water (materials/GerstnerWaves_vs_330.glsl, with steepness 0 and no phase): `shader_params()`
gives its parameters, so that what is drawn and what moves the vehicle are the same wave.

* The boat (the WAM-V): its two floats are boxes. Each is cut into segments along its length, and for
  each the buoyancy that the plugin gives with a flat surface (the part of the box under z = 0) is
  compared with the one it would give under the wave (the part under the surface elevation at that
  point): the difference is the force, and its lever arm from the link gives the torque. So the boat
  heaves, rolls and pitches with the wave, and a trough that leaves a float out of the water, or a crest
  that covers it, is not a force the boat could not have. Measured on the model: 600 N lift it 2.13 cm,
  which is 28.2 kN/m; the two floats give 27.8 kN/m (the area of the waterplane, rho g A).
* The submarine (the Tethys): it is fully under water and neutrally buoyant, so the wave reaches it as
  the acceleration of the water particles (Froude-Krylov: force = mass x orbital acceleration), which
  falls with the depth as exp(-k d). A wave of 25 m (a period of 4 s) still moves it a good deal at 3 m
  and hardly at 15 m. Measured at 3 m with waves of 0.6 m: it oscillates +-0.07 m about its depth (the
  orbit of the particles is +-0.16 m: its inertia and added mass), and, which was not looked into, it
  sinks 0.3 to 0.8 m a minute (without waves it holds its depth to the millimetre) and, if it is not
  driven, turns in yaw (about -50 deg a minute). Its model has no static stability, so the node also
  gives it a righting torque and a damping of roll and pitch (see SUBMARINE_BG).

What it leaves out: the horizontal force of the wave on the boat, the waves that the vehicle itself
makes, diffraction, and the added mass of the submarine. It is meant to make the vehicles move as in a
sea, not to measure sea-keeping.

ApplyLinkWrench has two topics: `wrench/persistent` ADDS every message to the force it applies until
`wrench/clear`, and `wrench` applies a message for one physics step. This node uses the second one at
`--rate` Hz, with each message scaled by (time since the last one) / (physics step), so that its
impulse is the one of the force it stands for.
"""

import argparse
import math
import sys

import numpy as np

G = 9.81
RHO = 1000.0             # kg/m3, the water of the world (graded buoyancy of gz-sim)

# the WAM-V of marine_wamv/model.sdf: two floats, boxes of 4 x 0.3545 x 0.3545 m centred at
# (-0.4, +-1.03, 0.2) in the frame of base_link, which is the frame of its pose
FLOAT_X = (-2.4, 1.6)    # m, from the stern to the bow of a float
FLOAT_Y = (-1.03, 1.03)  # m, where the two floats are
FLOAT_WIDTH = 0.3545
FLOAT_HEIGHT = 0.3545
FLOAT_BOTTOM = 0.2 - 0.5 * FLOAT_HEIGHT    # m over base_link
SEGMENTS = 8             # per float
# The torque that the segments give is that of a hydrostatic model of the floats; the simulator's boat
# resists a tilt with another stiffness (the cause was not looked into). Measured with a pure torque of
# 1000 Nm on a boat at rest: 2.43 deg of pitch (23.6 kNm/rad, the model has 41.0) and 1.73 deg of roll
# (33.2 kNm/rad, the model has 29.5). The torque is scaled by the ratio, in the axes of the boat, so
# that a slope of the surface tilts it by that slope, as it would a boat in a sea.
PITCH_SCALE = 23.6 / 41.0
ROLL_SCALE = 33.2 / 29.5
BOAT_LINK = 'wamv::base_link'

TETHYS_MASS = 147.8671   # kg: with the buoyancy of its body, the mass of water it displaces
# The model of the Tethys has the centre of its buoyancy at its centre of mass: no static stability, so
# the first disturbance (a wave) makes it lose its attitude slowly (unaided, with the waves of 0.6 m:
# pitch -42 deg, roll 40 deg and yaw -87 deg in 60 s). The node adds the torque that a centre of
# buoyancy BG metres above the centre of mass would give, and a damping of the roll and pitch rates
# (Nm s/rad, about the axes of the body): without it the righting torque, applied with the delay of
# the loop, made the roll (inertia 3 kg m2, almost no damping in the model) grow at the wave's period
# up to 23 deg. The numbers are choices: a damping ratio of 0.7 for the roll and 0.5 for the pitch.
# Yaw is left free, as it is on a real vehicle: the autopilot steers it when it is driven.
SUBMARINE_BG = 0.03
SUBMARINE_DAMPING = (16.0, 60.0)
SUBMARINE_LINK = 'tethys::base_link'

SECOND_AMPLITUDE, SECOND_PERIOD, SECOND_TURN = 0.35, 0.7, math.radians(40.0)
GROWTH_TAU = 2.0         # s: the waves grow as 1 - exp(-t / tau), as the shader's do


class Waves:
    """The wave field: two plane waves, calm at t = 0 and fully grown after a few `tau`."""

    def __init__(self, height, period, direction_deg, tau=GROWTH_TAU):
        if height < 0.0 or period <= 0.0:
            raise ValueError('the height must not be negative and the period must be positive')
        a = 0.5 * height
        omega = 2.0 * math.pi / period
        turn = math.radians(direction_deg)
        omega2 = omega / SECOND_PERIOD
        # (amplitude m, angular frequency rad/s, wavenumber rad/m, direction rad)
        self.components = (
            (a, omega, omega * omega / G, turn),
            (SECOND_AMPLITUDE * a, omega2, omega2 * omega2 / G, turn + SECOND_TURN))
        self.tau = tau

    def growth(self, t):
        return 1.0 if self.tau <= 0.0 else 1.0 - math.exp(-max(t, 0.0) / self.tau)

    def elevation(self, x, y, t):
        """Height of the surface (m over the sea level) at (x, y); x and y may be arrays."""
        eta = 0.0
        for a, w, k, d in self.components:
            eta = eta + a * np.cos(k * (x * math.cos(d) + y * math.sin(d)) - w * t)
        return self.growth(t) * eta

    def orbital_acceleration(self, x, y, z, t):
        """Acceleration (m/s2, world frame) of the water particles at (x, y, z), z under the surface
        being negative: horizontal along each wave, vertical, and less with the depth."""
        ax = ay = az = 0.0
        for a, w, k, d in self.components:
            phase = k * (x * math.cos(d) + y * math.sin(d)) - w * t
            decay = math.exp(k * min(z, 0.0))
            horizontal = w * w * a * decay * math.sin(phase)
            ax += horizontal * math.cos(d)
            ay += horizontal * math.sin(d)
            az += -w * w * a * decay * math.cos(phase)
        g = self.growth(t)
        return g * ax, g * ay, g * az

    def shader_params(self):
        """The parameters of the vertex shader that draws this field (GerstnerWaves_vs_330.glsl), as
        (name, type, value) with the value as the text of the SDF: it takes three waves; the third is
        left at zero. A steepness of 0 makes them sinusoids that only move up and down, which is what
        `elevation` gives."""
        unused = (0.0, 0.0, 1.0, 0.0)
        a, w, k, d = zip(*(tuple(self.components) + (unused,)))
        number = lambda values: ' '.join('%.9g' % v for v in values)           # noqa: E731
        direction = lambda angle: '%.9g %.9g' % (math.cos(angle), math.sin(angle))   # noqa: E731
        return [('Nwaves', 'int', str(len(self.components))),
                ('amplitude', 'float_array', number(a)),
                ('wavenumber', 'float_array', number(k)),
                ('omega', 'float_array', number(w)),
                ('steepness', 'float_array', '0 0 0'),
                ('dir0', 'float_array', direction(d[0])),
                ('dir1', 'float_array', direction(d[1])),
                ('dir2', 'float_array', direction(d[2])),
                ('tau', 'float', '%.9g' % self.tau)]


def rotation(q):
    """The matrix of a quaternion with x, y, z, w (any object with those attributes)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0.0 else 0.0
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)]])


def float_points():
    """(n, 3) centres of the bottoms of the segments of the floats in the frame of the boat, and the
    area of the waterplane of one of them."""
    length = (FLOAT_X[1] - FLOAT_X[0]) / SEGMENTS
    xs = FLOAT_X[0] + (np.arange(SEGMENTS) + 0.5) * length
    points = [(x, y, FLOAT_BOTTOM) for y in FLOAT_Y for x in xs]
    return np.array(points), length * FLOAT_WIDTH


def boat_wrench(waves, position, rot, t):
    """(force, torque) in the world frame, about the origin of the link, that the sea adds to the boat
    in `waves` to what it already gives in a flat sea. `rot` is the rotation matrix of the boat."""
    points, area = float_points()
    arms = points @ rot.T                                  # from the link to each segment, world frame
    bottoms = position[2] + arms[:, 2]
    eta = waves.elevation(position[0] + arms[:, 0], position[1] + arms[:, 1], t)
    flat = np.clip(0.0 - bottoms, 0.0, FLOAT_HEIGHT)       # what the plugin counts as submerged
    wave = np.clip(eta - bottoms, 0.0, FLOAT_HEIGHT)       # what the wave would have under it
    lift = RHO * G * area * (wave - flat)                  # N upwards, each segment
    force = np.array([0.0, 0.0, lift.sum()])
    torque = np.array([(arms[:, 1] * lift).sum(), -(arms[:, 0] * lift).sum(), 0.0])
    body = rot.T @ torque
    body[0] *= ROLL_SCALE
    body[1] *= PITCH_SCALE
    return force, rot @ body


def submarine_wrench(waves, position, rot, t, rates=(0.0, 0.0, 0.0), mass=TETHYS_MASS,
                     bg=SUBMARINE_BG, damping=SUBMARINE_DAMPING):
    """(force, torque) on the submarine: the water particles' acceleration (mass x acceleration); the
    torque of a centre of buoyancy `bg` m over the centre of mass, r x F with r = R (0, 0, bg) and
    F = (0, 0, mass g), which is (bg m g) (R[1][2], -R[0][2], 0); and a damping of the roll and pitch
    `rates` (rad/s, in the axes of the body)."""
    a = waves.orbital_acceleration(position[0], position[1], position[2], t)
    righting = bg * mass * G * np.array([rot[1, 2], -rot[0, 2], 0.0])
    damp = rot @ np.array([-damping[0] * rates[0], -damping[1] * rates[1], 0.0])
    return mass * np.array(a), righting + damp


def angular_rates(before, after, elapsed):
    """Angular velocity (rad/s, in the axes of the body) that takes the rotation `before` to `after`
    in `elapsed` s, to first order."""
    step = before.T @ after
    return np.array([step[2, 1] - step[1, 2], step[0, 2] - step[2, 0],
                     step[1, 0] - step[0, 1]]) / (2.0 * elapsed)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--vehicle', choices=('boat', 'submarine'), required=True)
    parser.add_argument('--height', type=float, default=0.4, help='m, crest to trough of the main wave')
    parser.add_argument('--period', type=float, default=4.0, help='s of the main wave')
    parser.add_argument('--direction', type=float, default=180.0,
                        help='degrees, where the main wave goes (0: to +x, east; 90: to +y, north)')
    parser.add_argument('--tau', type=float, default=GROWTH_TAU,
                        help='s: the waves grow as 1 - exp(-t / tau) from the start of the simulation')
    parser.add_argument('--rate', type=float, default=50.0, help='Hz of the wrenches')
    parser.add_argument('--physics-step', type=float, default=0.005,
                        help="s, max_step_size of the world's physics")
    parser.add_argument('--sim-time', action='store_true')
    args, ros_args = parser.parse_known_args(argv)

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from ros_gz_interfaces.msg import Entity, EntityWrench

    waves = Waves(args.height, args.period, args.direction, args.tau)
    link = BOAT_LINK if args.vehicle == 'boat' else SUBMARINE_LINK
    rclpy.init(args=ros_args)
    node = rclpy.create_node(
        'waves', parameter_overrides=[Parameter('use_sim_time', value=bool(args.sim_time))])
    publisher = node.create_publisher(EntityWrench, 'waves/wrench', 10)
    seen = {'pose': None, 'start': None, 'last': None, 'rot': None, 'rates': np.zeros(3)}

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def on_odom(msg):
        seen['pose'] = msg.pose.pose

    def tick():
        pose = seen['pose']
        t = now()
        if pose is None:
            return
        if seen['start'] is None:
            seen['start'], seen['last'] = t, t
            node.get_logger().info('the first wave is applied at t = %.3f s' % t)
            return
        elapsed, seen['last'] = t - seen['last'], t
        if not 0.0 < elapsed < 1.0:
            return                              # a pause or a jump of the clock: no impulse to make up
        rot = rotation(pose.orientation)
        if seen['rot'] is not None:             # the rates of the pose, smoothed: odometry has none
            seen['rates'] += 0.5 * (angular_rates(seen['rot'], rot, elapsed) - seen['rates'])
        seen['rot'] = rot
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        if not np.all(np.isfinite(position)):
            return
        if args.vehicle == 'boat':
            force, torque = boat_wrench(waves, position, rot, t)
        else:
            force, torque = submarine_wrench(waves, position, rot, t, seen['rates'])
        scale = elapsed / args.physics_step     # one step of this force stands for `elapsed` of it
        msg = EntityWrench()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.entity.name = link
        msg.entity.type = Entity.LINK
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = (float(v) for v in force * scale)
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = (float(v) for v in torque * scale)
        publisher.publish(msg)

    node.create_subscription(Odometry, 'odom', on_odom, 50)
    node.create_timer(1.0 / args.rate, tick)
    node.get_logger().info(
        'waves for the %s: %.2f m high, %.1f s, going to %.0f deg (a wave of %.0f m)' % (
            args.vehicle, args.height, args.period, args.direction,
            2.0 * math.pi / waves.components[0][2]))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
