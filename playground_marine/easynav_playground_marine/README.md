<!--
Copyright 2026 José Miguel Guerrero
SPDX-License-Identifier: Apache-2.0
-->

# easynav_playground_marine

Playground for **boats and submarines**: one launch starts Gazebo with water and a vehicle in it. It is
only the simulation: it depends on Gazebo and ros_gz, and on no other package of the EasyNav family.

```bash
ros2 launch easynav_playground_marine easynav_marine.launch.yaml robot:=boat
ros2 launch easynav_playground_marine easynav_marine.launch.yaml robot:=submarine
```

| `robot` | Vehicle | Where |
| --- | --- | --- |
| `boat` (default) | WAM-V, the catamaran of VRX, steered by the difference of its two engines | on the water |
| `submarine` | MBARI Tethys LRAUV, a torpedo with a propeller and two pairs of fins | under it, `altitude:=` m deep |

## The world

A **synthetic sea**: offline, the same every time (`tools/synthetic_sea.py` of the worlds package). A beach
to the west, a floor that deepens to the east (20.9 m at the origin, 48 m at the edge), an islet 17 m over
the water, a shoal 4 m under the surface, a seamount and a ridge. 1 km x 1 km; z = 0 is the sea level.

## Arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `robot` | `boat` | `boat` or `submarine` |
| `radius` | `''` | half-side of the square, m (500; at least 100) |
| `altitude` | `''` | `robot:=submarine`: depth under the surface, m (half way to the floor, at most 3) |
| `gui` | `true` | Gazebo GUI |
| `waves` | `false` | `true`: the vehicle moves as in a sea and the water is drawn with the waves (see Waves) |
| `wave_visual` | `true` | `waves:=true`: draw the waves on the water. `false`: the water that is drawn stays flat and only the vehicle moves |
| `wave_height` | `0.4` | `waves:=true`: height of the main wave, m, crest to trough (0 to 2) |
| `wave_period` | `4.0` | `waves:=true`: period of the main wave, s (2 to 15) |
| `wave_direction` | `180` | `waves:=true`: where the main wave goes, degrees (0 east, 90 north, 180 to the beach) |

The worlds package's launch also takes `spawn_x`, `spawn_y` and `gpu`.

## Waves

```bash
ros2 launch easynav_playground_marine easynav_marine.launch.yaml robot:=boat waves:=true
ros2 launch easynav_playground_marine easynav_marine.launch.yaml robot:=submarine waves:=true wave_height:=0.6 wave_direction:=90
```

With `waves:=true` the vehicle moves as in a sea **and the water is drawn with the waves**. The two are the
same wave field (two plane waves of deep water: the main one and a second, smaller and faster one, so that
the motion does not repeat with one period):

* The **vehicle** is moved by a force that the `waves` node (`tools/wave_node.py` of the worlds package)
  applies to it, worked out from its pose. The buoyancy of Gazebo stays that of a flat sea.
* The **water** is a dense mesh drawn by a vertex shader (Gerstner waves with steepness 0, i.e. sinusoids;
  `marine_waves` of the description package, from the Waves model of Gazebo Fuel) that gets its parameters
  from the same `Waves` object and the time of the simulation. `wave_visual:=false` leaves the water flat.

* **Boat:** the floats are cut in segments and, for each, the buoyancy under the wave is compared with the
  one that Gazebo gives under a flat sea; the difference is the force and torque. It heaves, rolls and
  pitches, and a trough that leaves a float out of the water is not a force it could not have.
* **Submarine:** the particles of the water push it with their acceleration (mass x acceleration), which
  falls with the depth as exp(-k d): at 3 m with a wave of 25 m it is about half of what the surface
  has; at 15 m, almost nothing.

Measured (60 s, headless, `wave_direction:=180`):

| Vehicle | Waves | Heave | Pitch | Roll |
| --- | --- | --- | --- | --- |
| Boat, not driven | 0.4 m | +-0.27 m (the surface's) | +-7 deg | +-2 deg |
| Boat at 1.5 m/s | 0.4 m | +-0.26 m | +-6 deg | +-2.5 deg |
| Submarine at 3 m, 1.2 m/s | 0.6 m | +-0.07 m (the particles' orbit is +-0.16 m) | +-1 deg | < 1 deg |

**Does what is drawn match what moves the vehicle?** Measured: a graduated pole in the water, the simulation
paused at 8 instants (the exact simulation time is read) and the waterline on the pole read in a screenshot,
against the wave field at that instant. Wave of 1 m, error between -0.05 and +0.045 m (RMS 0.034 m, about
1.5 pixels; the wave amplitude is 0.7 m); the shift in time that fits best is 0.00 s.

Limits:

* Only the vertical force of the wave on the boat acts (no horizontal drift, no waves made by the vehicle).
* At 0.6 m the boat heaves +-0.37 m and pitches +-10 deg, and bigger waves are rougher still: its floats have
  a draught of 8 cm, so a trough leaves them out of the water.
* In waves the **submarine sinks** 0.3 to 0.8 m a minute (without waves it holds its depth exactly) and, if
  it is not commanded, **turns in yaw** (about -50 deg a minute). The cause was not investigated; the Tethys
  model has no yaw stability. With `cmd_vel` its autopilot steers the yaw rate, not the heading, so it
  wanders by +-20 deg. Its roll and pitch are held by a righting torque and a damping that the node adds
  (the model's centre of buoyancy is at its centre of mass).
* The shader needs the Gazebo GUI (it runs on the GPU; headless there is nothing to draw). The surface seen
  from **below** stays flat, so the submarine sees a flat ceiling. The water is more opaque than before
  (`alpha` 0.7, in `synthetic_sea.py`): the sea floor is seen through it only in the shallows.
* The torques on the boat are scaled to the stiffness that was measured in Gazebo (see the comments of
  `wave_node.py`), and the boat's roll and pitch damping was raised in `wamv.sdf.in` for this.

## Driving it

Command it with a `Twist` on `/cmd_vel`; the pose is `/odom` (and `odom -> base_link`, `map -> odom` on `/tf`):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.5}, angular: {z: 0.2}}"                 # boat
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.5, z: -0.2}, angular: {z: 0.1}}"        # submarine
```

`linear.x` is m/s forward, `angular.z` rad/s of yaw and, for the submarine, `linear.z` m/s up. A command
older than a second stops the engines.

## Limits

* No navigation system comes with it: the vehicles carry no sensors and none of the EasyNav systems of the
  family targets water, so this package has no `params/` or `maps/` of its own, unlike the Kobuki, Summit
  and TIAGo playgrounds. What a navigation system needs from the simulation is there (`/odom`, `/tf`,
  `/clock`, a world with a solid floor).
* The submarine is a torpedo: it steers and dives only while it moves forward. Near the coast, or over
  the shoal, it can run aground: the floor is solid.
* The WAM-V and the Tethys come from Gazebo Fuel the first time (internet).

## Packages

| Package | Contents | Depends on |
| --- | --- | --- |
| `easynav_playground_marine_description` | the WAM-V and Tethys models | nothing |
| `easynav_playground_marine_worlds` | the sea, the vehicle in it, the bridge and the autopilots | Gazebo, `ros_gz`, ROS 2 (`rclpy`, `tf2_ros`), numpy |
| `easynav_playground_marine` (this) | the launch | the other two |

## Licenses

The launchers, the tools and the synthetic sea are Apache-2.0. The vehicles are by third parties and come
from Gazebo Fuel, both under CC-BY-4.0: the WAM-V by Carlos Aguero and Cole Biesemeyer (Open Robotics; a
modified copy of its SDF is in the description package) and the MBARI Tethys LRAUV by Mabel Zhang
(Accurrent; downloaded, not copied). See the README of `easynav_playground_marine_description`.
