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
| `rviz` | `true` | RViz with the sensors of the vehicle (see Sensors) |
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
  (`alpha` 0.7, in `synthetic_sea.py`): the sea floor is seen through it only in the shallows, and the
  submarine at 3 m is seen from above, but dim (checked with the GUI: the waves are drawn as with the boat).
* The torques on the boat are scaled to the stiffness that was measured in Gazebo (see the comments of
  `wave_node.py`), and the boat's roll and pitch damping was raised in `wamv.sdf.in` for this.

## Sensors

Each vehicle carries the sensors of its kind. Their place, rate and noise are in one table,
`tools/marine_sensors.py` of the worlds package, from which the launch makes the SDF, the bridge and the
static transforms `base_link -> <frame>`. `rviz:=true` (default) opens RViz with all of them, following the
vehicle.

**Boat (WAM-V)**

| Sensor | Topic | Type | Frame | Notes |
| --- | --- | --- | --- | --- |
| IMU | `/imu/data` | `sensor_msgs/Imu` | `imu_link` | 100 Hz, the noise of VRX's WAM-V |
| Magnetometer | `/imu/mag` | `sensor_msgs/MagneticField` | `imu_link` | 20 Hz, tesla, ENU |
| GNSS | `/gps/fix` | `sensor_msgs/NavSatFix` | `gps_link` | 5 Hz, 1 m (horizontal) and 2 m (vertical) of noise; `/gps/raw` is Gazebo's |
| 3D lidar | `/lidar/points` | `sensor_msgs/PointCloud2` | `lidar_link` | VLP-16-like on a mast: 16 x 900 rays, 30 deg vertical, 100 m, 10 Hz; only what is over the water |
| Camera | `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` | `camera_optical_link` | 1280 x 720, 80 deg, 15 Hz, 10 deg down |
| Echosounder | `/echosounder` | `sensor_msgs/Range` (ULTRASOUND) | `echosounder_link` | under the left float, a cone of 8 deg, 150 m, 5 Hz: the depth under the keel |

**Submarine (Tethys)**

| Sensor | Topic | Type | Frame | Notes |
| --- | --- | --- | --- | --- |
| IMU | `/imu/data` | `sensor_msgs/Imu` | `imu_link` | 100 Hz |
| Magnetometer | `/imu/mag` | `sensor_msgs/MagneticField` | `imu_link` | 20 Hz, tesla, ENU |
| Depth gauge | `/pressure`, `/depth` | `sensor_msgs/FluidPressure` (Pa), `geometry_msgs/PoseWithCovarianceStamped` (z of `base_link` in `map`) | `pressure_link` | 10 Hz, 2 cm of noise |
| DVL | `/dvl/data`, `/dvl/twist`, `/dvl/velocity`, `/dvl/altitude` | `marine_acoustic_msgs/Dvl`, `TwistWithCovarianceStamped`, `TwistStamped`, `Range` | `dvl_link` | four beams 30 deg off the vertical, bottom tracking up to 100 m, 5 Hz; the twist only with the bottom in range |
| GNSS | `/gps/fix` | `sensor_msgs/NavSatFix` | `gps_link` | on a short mast: no fix under the water (`STATUS_NO_FIX`), and 5 s after the antenna comes out |
| Imaging sonar | `/sonar/image`, `/sonar/image_fan`, `/sonar/points` | `marine_acoustic_msgs/ProjectedSonarImage`, `Image` (mono8), `PointCloud2` | `sonar_link` (z forward, x up), `sonar_rays_link` | forward-looking multibeam like an Oculus M750d: 256 beams over 130 x 20 deg, 0.5-50 m in 0.1 m bins, 10 Hz, 20 deg down |

The simulator's own pose of the vehicle is `/ground_truth/odom` (not a sensor): the autopilots, the waves
and the transforms `odom -> base_footprint` are made from it.

How what Gazebo does not have is made:

* **Imaging sonar.** Gazebo has no acoustic sensor. A fan of rays (`gpu_lidar`, 512 x 32 over the sonar's
  field of view) finds what the sound would hit, and `tools/sonar_node.py` turns the hits into echoes as a
  multibeam forms its image: Lambert backscatter (cos^2 of the incidence, from the neighbouring hits), the
  vertical beam pattern, the two-way absorption of the water, no elevation (every hit of a beam at the same
  range adds to one cell), shadows behind what is hit, the spread of the pulse and of the beam, speckle and
  a noise floor, log-compressed to 8 bits. It is an image with the look and the geometry of a sonar, not a
  physical simulation of the sound (multipath, the surface's reverberation and the water column are left
  out).
* **Depth gauge.** Gazebo has no water pressure sensor: `tools/marine_sensor_node.py` works the depth out
  from the true pose, with noise (the waves are left out of it).
* **GNSS under the water, echosounder, lidar.** The same node takes the fix away under the water, gives the
  echosounder's first echo as a `Range`, and drops the lidar's points under the surface: the ray sensors do
  not see the water's surface (`visibility_flags`, it is a drawing) and would show the sea floor through it.

## EasyNav with the submarine

```bash
ros2 launch easynav_playground_marine easynav_submarine_mpc.launch.yaml
```

EasyNav drives the Tethys (`params/submarine_mpc.params.yaml`, `rviz/submarine_easynav.rviz`):

* **Map**: the relief of the synthetic sea as a point cloud for Bonxai
  (`easynav_playground_marine_worlds/maps/synthetic_sea.pcd`, a point every 2 m, made by
  `synthetic_sea.py --pcd`).
* **Planner**: A* in 3D (`BonxaiPlanner`, 2 m cells) under the surface (`max_altitude: -1`) and over the
  floor (`fill_below: lowest`), 2 m from the relief at least.
* **Localization**: dead reckoning, as an AUV navigates (`FusionLocalizer`). The local filter
  (`odom -> base_footprint`) fuses the IMU and the DVL; the global one (`map -> odom`) adds the depth gauge
  (in `map`: the depth is measured from the surface) and the GNSS when the vehicle is at the surface. Both
  start where the launch puts the vehicle by default, (0, 0) at 3 m. The simulation runs with
  `odom_tf:=false`: the true pose makes no transform.
* **Controller**: MPC with the aerial model, no lateral motion and forward only. The Tethys turns and dives
  only while it moves: a turn asked for at a standstill (the MPC's, to face a goal behind it) is done on
  the move by `auv_cmd.py`, at its steerage speed (0.6 m/s).

Goals are under the water: put them with the **3D Goal Pose** tool of RViz (shortcut `h`), or

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 60.0, y: 0.0, z: -6.0}, orientation: {w: 1.0}}}"
```

Measured headless: a goal 60 m ahead and 3 m deeper is reached in 48 s, and one behind it, (-40, 30, -5),
in 105 s, turning on the move; the estimate stays within 10 cm of the true pose. With no thrust at the
goal the vehicle coasts on (some 30 m from 1.5 m/s): the goal is reached as it goes through.

## Driving it

Command it with a `Twist` on `/cmd_vel`; its true pose is `/ground_truth/odom` (and `odom -> base_link`,
`map -> odom` on `/tf`):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.5}, angular: {z: 0.2}}"                 # boat
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.5, z: -0.2}, angular: {z: 0.1}}"        # submarine
```

`linear.x` is m/s forward, `angular.z` rad/s of yaw and, for the submarine, `linear.z` m/s up. A command
older than a second stops the engines.

## Limits

* EasyNav drives the submarine only (see above). The boat has its sensors, but no EasyNav configuration
  yet.
* `marine_acoustic_msgs/Dvl` cannot be built from Python as generated on Rolling and Lyrical (its field
  `range` hides the builtin in `__init__`); `tools/marine_sensor_node.py` patches its `__init__`
  (`fix_dvl_init`) before using it. A Python node of your own that reads `/dvl/data` needs the same.
* The submarine is a torpedo: it steers and dives only while it moves forward. Near the coast, or over
  the shoal, it can run aground: the floor is solid.
* The WAM-V and the Tethys come from Gazebo Fuel the first time (internet).

## Packages

| Package | Contents | Depends on |
| --- | --- | --- |
| `easynav_playground_marine_description` | the WAM-V and Tethys models | nothing |
| `easynav_playground_marine_worlds` | the sea, the vehicle and its sensors in it, the bridge, the autopilots and RViz | Gazebo, `ros_gz`, ROS 2 (`rclpy`, `tf2_ros`, `sensor_msgs_py`, `marine_acoustic_msgs`), RViz (`rviz_imu_plugin`), numpy |
| `easynav_playground_marine` (this) | the launches, EasyNav's parameters and RViz configuration for the submarine | the other two, EasyNav (`easynav_system`, the Bonxai maps manager and planner, the fusion localizer, the MPC controller), `navmap_rviz_plugin` |

## Licenses

The launchers, the tools and the synthetic sea are Apache-2.0. The vehicles are by third parties and come
from Gazebo Fuel, both under CC-BY-4.0: the WAM-V by Carlos Aguero and Cole Biesemeyer (Open Robotics; a
modified copy of its SDF is in the description package) and the MBARI Tethys LRAUV by Mabel Zhang
(Accurrent; downloaded, not copied). See the README of `easynav_playground_marine_description`.
