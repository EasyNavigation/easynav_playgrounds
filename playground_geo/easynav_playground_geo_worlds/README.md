<!--
Copyright 2026 José Miguel Guerrero
SPDX-License-Identifier: Apache-2.0
-->

# easynav_playground_geo_worlds

The geo simulation: the Gazebo world of any real place, made from its coordinates and a radius from
open data, with the Robotnik Summit XL in it. It needs Gazebo Harmonic or newer (`ros_gz_sim`,
`ros_gz_bridge`), the Summit XL of `playground_summit`, and outside Spain `navmap_tools`; not EasyNav.

How to use it, its arguments, the data sources and their licenses are in the README of the
playground, [`easynav_playground_geo`](../easynav_playground_geo/README.md). This one is about what
is inside.

```bash
ros2 launch easynav_playground_geo_worlds geo_sim.launch.py lat:=46.0207 lon:=7.7491 radius:=300
ros2 launch easynav_playground_geo_worlds geo_sim.launch.py lat:=40.4168 lon:=-3.7038 radius:=250 world:=area
```

(the same as `easynav_geo.launch.yaml` of the playground, which only forwards its arguments here).

| Part | What it does |
| --- | --- |
| `tools/geo_world.py` | The world generator: downloads the terrain, the imagery and, for `world:=area`, the buildings, walls and trees of the place, and writes the world, its meshes and `world.yaml`. It also runs alone (below) |
| `tools/tree_models.py` | Plants the trees: the oak and the pine of `models/trees/`, scaled to each crown, merged into one mesh with one collision mesh of the trunks |
| `worlds/geo.world.in` | The world template: physics, the sensor systems (NavSat among them), `<spherical_coordinates>` of the place (`@LATITUDE@`, `@LONGITUDE@`, `@ELEVATION@`) and the terrain (`@TERRAIN_DIR@`) |
| `launch/geo_sim.launch.py` | Checks the arguments, runs `geo_world.py` (or reuses its world), writes `~/.ros/easynav_geo/last_world.yaml`, starts Gazebo (`easynav_playground_summit_worlds`' `world.launch.yaml`) and spawns the Summit (its `summit.launch.yaml`) on the ground |
| `models/trees/` | Open Robotics' oak and pine from Gazebo Fuel (CC0) |

## What is generated, and where

Each world goes to its own directory under `~/.ros/easynav_geo/worlds/`, named
`terrain_<lat>_<lon>_<radius>` or `area_<lat>_<lon>_<radius>`:

| File | What it is |
|---|---|
| `world.sdf` | The Gazebo world |
| `terrain.obj`, `terrain.mtl`, `ortho.jpg` | The terrain as it is drawn, with the imagery |
| `terrain.stl` | The terrain as it collides (a coarser mesh, so that Gazebo stays fast) |
| `buildings.*`, `walls.*`, `trees.*` | `world:=area`: what stands on the terrain, drawn and colliding |
| `navmap.stl` | Terrain + buildings + walls + tree trunks, without the crowns: the surface a ground robot moves on and the obstacles on it, for making a map (see `last_world.yaml` below) |
| `navmap_ground.stl`, `navmap_holes.stl` | The same split in two: the ground alone, and what stands on it |
| `world.yaml` | The place, the elevation of the origin, the spawn spot, and how many buildings, walls and trees there are |

Other caches: the OpenStreetMap answers in `~/.cache/easynav_geo/overpass/`, and outside Spain the
terrain and imagery tiles in `navmap_tools`' own cache. To make a world again from scratch, remove
its directory.


## Using the generator alone

`geo_world.py` makes a world without starting Gazebo, and has more options than the launch
(`--help`):

```bash
G=$(ros2 pkg prefix easynav_playground_geo_worlds)/lib/easynav_playground_geo_worlds/geo_world.py
python3 $G --lat 48.8562 --lon 2.2976 --radius 400                 # Paris, with everything
python3 $G --lat 46.0207 --lon 7.7491 --radius 300 --no-buildings --no-trees --no-walls
```

Among them: `--terrain-res` and `--navmap-res` (mesh spacing), `--ortho-ppm` (imagery pixels per
metre), `--max-trees`, `--force` (make it again), and `--survey-model`, `--survey-stl` and
`--survey-elevation`, which put a survey mesh of the real site (photogrammetry, for instance) into
the terrain. It prints its progress, and as its last line the directory of the world.

## last_world.yaml

`geo_sim.launch.py` writes `~/.ros/easynav_geo/last_world.yaml` at every launch: the kind of world,
its directory, `latitude`, `longitude`, `radius`, `ground_elevation` (m above sea level of z = 0) and
`mesh` (`navmap.stl` of the world). It is how an application finds the place the robot is in; see
"Making a map of the same place" in the playground's README.

## Tests

```bash
colcon test --packages-select easynav_playground_geo_worlds && colcon test-result --verbose
```

- `test/test_geo_sim_launch.py`: the decisions of `geo_sim.launch.py` without starting anything: the
  generator command of each world, the refused arguments, the spawn pose (the free spot of the
  generator, or the ground under the given x, y), `last_world.yaml`.
- `test/test_geo_world.py`: the parts of the generator that work offline: OpenStreetMap buildings
  from a canned answer, the regions and their projections, the building geometry, the cache key of
  each world.

The downloads are not tested here (they need the network and the public services).

## License

Apache-2.0 (`LICENSE`); the tree models are CC0 (`models/trees/README.md`). The data in a
generated world is under the terms of its providers: see the playground's README.
