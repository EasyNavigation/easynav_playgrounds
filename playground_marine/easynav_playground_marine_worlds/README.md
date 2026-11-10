<!--
Copyright 2026 José Miguel Guerrero
SPDX-License-Identifier: Apache-2.0
-->

# easynav_playground_marine_worlds

The marine simulation: a Gazebo world with water, a boat or a submarine in it with its sensors, the bridge
to ROS, their autopilots and RViz. It needs Gazebo Harmonic or newer (`ros_gz_sim`, `ros_gz_bridge`), RViz
and numpy, nothing else.

```bash
ros2 launch easynav_playground_marine_worlds marine_sim.launch.py robot:=boat
ros2 launch easynav_playground_marine_worlds marine_sim.launch.py robot:=submarine altitude:=5
```

Arguments: `robot` (`boat`, `submarine`), `radius`, `altitude` (the submarine's depth), `spawn_x`, `spawn_y`,
`gui`, `rviz`, `odom_tf` (false: a localizer publishes `map -> odom -> base_footprint`), `gpu`, and `waves`, `wave_visual`, `wave_height`, `wave_period`, `wave_direction` (see the README of
`easynav_playground_marine`).

| Part | What it does |
| --- | --- |
| `tools/synthetic_sea.py` | The synthetic sea: relief (`height()`), colours by depth, the floor for Gazebo (STL for the collision, OBJ with a PNG texture for the visual) and the water (with waves: a mesh, finer near the origin, and the visual with the `ShaderParam` plugin). It also runs on its own: `synthetic_sea.py --out DIR [--radius 500]`, and `--pcd FILE` writes the relief as a point cloud |
| `maps/synthetic_sea.pcd` | The relief as Bonxai's map (`synthetic_sea.py --pcd maps/synthetic_sea.pcd`, a point every 2 m; a test checks it is up to date) |
| `worlds/marine.world.in` | The world template: `@TERRAIN_DIR@` (the floor), `@WATER@` (surface and buoyancy) |
| `launch/marine_sim.launch.py` | Writes the sea, puts the vehicle in the world, starts Gazebo, the bridge and the autopilot |
| `launch/world.launch.yaml` | Gazebo server (and client) with a world file |
| `tools/boat_cmd.py`, `tools/auv_cmd.py` | The autopilots: a `Twist` on `/cmd_vel` in, thrust (and fins) out |
| `tools/wave_node.py` | `waves:=true`: the force and torque that a wave field adds to the vehicle (`/waves/wrench`, bridged to Gazebo's `ApplyLinkWrench`) and the parameters of the shader that draws the same waves (`Waves.shader_params()`) |
| `tools/ground_truth.py` | The submarine's true pose: Gazebo's odometry of the Tethys model (whose +x is its stern) turned to that of `base_link` (x to the bow), on `/ground_truth/odom` |
| `tools/odom_to_tf.py` | The true pose (`/ground_truth/odom`) to `odom -> base_footprint -> base_link`, `map -> odom`, and `base_link -> <sensor frame>` (`--frame`) |
| `tools/marine_sensors.py` | The table of the sensors of each vehicle: from it the SDF (`sdf()`), the bridge (`bridge_entries()`) and the static transforms (`static_frames()`) |
| `tools/marine_sensor_node.py` | What Gazebo does not give as such: the GNSS without a fix under the water, the depth gauge, the echosounder's `Range`, the DVL's twist and altitude, the lidar without what is under the water |
| `tools/sonar_node.py` | The imaging sonar: echoes from a fan of rays (`ProjectedSonarImage`, a fan image and the strong echoes as points) |
| `rviz/boat.rviz`, `rviz/submarine.rviz` | RViz with every sensor of the vehicle |

What it writes (`~/.ros/easynav_playground_marine/`): `synthetic_r<radius>/` (floor and world),
`world_<robot>.sdf` (the world with the vehicle) and the bridge files.

## Tests

`colcon test --packages-select easynav_playground_marine_worlds`, or `pytest test/` with the workspace
sourced: the autopilots, the synthetic sea, the sensors (the table, the sensor node and the sonar) and the
decisions of the launch file (no process starts).
