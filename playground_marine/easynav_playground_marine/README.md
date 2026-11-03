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

The worlds package's launch also takes `spawn_x`, `spawn_y` and `gpu`.

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
