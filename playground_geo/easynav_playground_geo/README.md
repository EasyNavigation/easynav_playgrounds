<!--
Copyright 2026 José Miguel Guerrero
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Geo Playground

The world of **any real place** in Gazebo, from its coordinates and a radius, with the Robotnik
Summit XL in it. The terrain comes from public elevation models and the ground is covered with
aerial imagery; on request, the buildings, walls and trees of the place are added too. Everything is
made from open data (OpenStreetMap and others, anywhere; in Spain, optionally, the IGN's: see
[`ign`](#optional-the-ign-data-of-spain-ign-true)), once per place, and kept for the next launch.

```bash
# the terrain of a place and its aerial imagery (fast): Zermatt, in the Alps
ros2 launch easynav_playground_geo easynav_geo.launch.yaml lat:=46.0207 lon:=7.7491 radius:=300

# the same plus buildings, walls and trees: the centre of Madrid
ros2 launch easynav_playground_geo easynav_geo.launch.yaml lat:=40.4168 lon:=-3.7038 radius:=250 world:=area

# the same with the IGN data of Spain (needs sudo apt install python3-laspy python3-laszip)
ros2 launch easynav_playground_geo easynav_geo.launch.yaml lat:=40.4168 lon:=-3.7038 radius:=250 world:=area ign:=true
```

It is **only the simulation** for now: the world and the robot, with no EasyNav configuration yet.
Navigation (EasyNav, or anything else) is another launch, started after this one; see
[Making a map of the same place](#making-a-map-of-the-same-place).

Like the other playgrounds it is split by what each part needs, but it has no robot description of
its own: the robot is the Summit XL of `playground_summit`.

| Package | Contents | Depends on |
| --- | --- | --- |
| [`easynav_playground_geo_worlds`](../easynav_playground_geo_worlds) | The simulation: the world generator, the world template, the tree models, and the launcher that makes the world, starts Gazebo and spawns the Summit | Gazebo, ros_gz, the Summit XL of `playground_summit`, `navmap_tools`; not EasyNav. Optional: laspy and laszip for `ign:=true` |
| `easynav_playground_geo` (this one) | The launcher above, `easynav_geo.launch.yaml`; the EasyNav configurations will go here | `easynav_playground_geo_worlds` |
| (none) | The robot: `easynav_playground_summit_description` | |

## Contents

- [Build](#build)
- [The two worlds](#the-two-worlds)
- [Arguments](#arguments)
- [Where the data comes from](#where-the-data-comes-from)
- [Making a map of the same place](#making-a-map-of-the-same-place)
- [Optional: the IGN data of Spain (`ign:=true`)](#optional-the-ign-data-of-spain-ign-true)
- [The robot](#the-robot)
- [GPU](#gpu)
- [Limits](#limits)
- [Tests](#tests)
- [Licenses and attribution](#licenses-and-attribution)

## Build

ROS 2 Rolling, with Gazebo (`gz_sim_vendor`) and the Summit XL of
[`playground_summit`](../../playground_summit/easynav_playground_summit) in the workspace:

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to easynav_playground_geo
source install/setup.bash
```

rosdep installs everything the default (OpenStreetMap) worlds need: the Python packages of the
world generator (numpy, scipy, OpenCV, requests, Pillow, pyproj and `mapbox_earcut`, which
triangulates the roofs) and `navmap_tools` (the NavMap repository), for the Copernicus terrain and
the Esri imagery. The first launch of a place needs a network connection; the next ones do not.

`ign:=true` (Spain only) needs two more packages, which rosdep does not install; see
[below](#optional-the-ign-data-of-spain-ign-true).

## The two worlds

| `world:=` | What is in it | First launch of a place |
|---|---|---|
| `terrain` (default) | The terrain with the aerial imagery draped on it | Downloads the elevation model and the imagery: seconds to a minute |
| `area` | The terrain, plus buildings, walls, fences and hedges, and trees | Also queries OpenStreetMap (and the cadastre with `ign:=true`), builds every building and plants the trees: minutes |

Both are the same generator (`tools/geo_world.py` of `easynav_playground_geo_worlds`): `terrain` is
`area` without buildings, walls and trees. Each world is made once and kept in
`~/.ros/easynav_geo/worlds/` (what goes there: the
[worlds package](../easynav_playground_geo_worlds/README.md#what-is-generated-and-where)); launching
the same place with the same options again starts Gazebo at once. Changing any option (the radius,
`trees`, `ign`, the LiDAR files...) makes another world, which is generated the first time.

The world is a square of side `2 x radius` centred on `lat`, `lon`. That point is the origin of the
world: x points east, y north, and z = 0 is the ground there. The world declares those coordinates
in its `<spherical_coordinates>`, so Gazebo's NavSat (GPS) sensor gives the real latitude,
longitude and altitude of the place.

## Arguments

| Argument | Default | What it is |
|---|---|---|
| `world` | `terrain` | `terrain` or `area` (see above) |
| `lat`, `lon` | `40.284796`, `-3.821407` | Centre of the world, WGS84 degrees. The default is the URJC campus at Fuenlabrada (Madrid) |
| `radius` | `300` | Half side of the square, m (above 0, at most 2000). `radius:=300` is a 600 x 600 m world |
| `trees` | `true` | `world:=area`: plant trees where the imagery shows crowns |
| `walls` | `true` | `world:=area`: walls, fences and hedges from OpenStreetMap |
| `roof` | `ortho` | `world:=area`: `ortho`, the roofs show the aerial imagery; `flat`, one colour per roof |
| `ign` | `false` | `false`: OpenStreetMap buildings and walls, Copernicus terrain, Esri imagery, anywhere. `true` (Spain only): IGN terrain, PNOA orthophoto, Catastro buildings and PNOA-LiDAR; needs laspy and laszip, and stops saying so without them (see [below](#optional-the-ign-data-of-spain-ign-true)) |
| `lidar` | `''` | `ign:=true` only: a PNOA-LiDAR `.laz`/`.las` file or a directory of them; `''` uses `~/.ros/easynav_geo/lidar/` if it has any |
| `spawn_x`, `spawn_y` | `''` | Where the Summit starts, in the world frame (m). `''`: the free spot nearest to the origin that the generator found (not inside or next to a building, wall or tree) |
| `spawn_z` | `''` | Height of the Summit at the start. `''`: 0.5 m above the ground there (the highest surface, so on a roof if `spawn_x`/`spawn_y` fall on a building) |
| `gui` | `true` | Gazebo GUI; `false` runs headless |
| `gpu` | `auto` | `auto`, `nvidia` or `none` (see [GPU](#gpu)) |

A wrong value (an unknown `world`, a latitude out of range, a radius of 0, `lidar` without
`ign:=true`, `ign:=true` outside Spain...) stops the launch with a message saying which argument is
wrong, before anything is downloaded.

## Where the data comes from

`ign` chooses the sources:

| | `ign:=false` (default, anywhere) | `ign:=true` (Spain only) |
|---|---|---|
| Terrain | Copernicus GLO-30, 30 m grid: smooth, streets come out flat | IGN MDT, 5 m grid (PNOA-LiDAR ground where you have the files) |
| Imagery | Esri World Imagery | PNOA orthophoto (IGN) |
| Buildings | OpenStreetMap: footprint x `height`, else `building:levels` x 3.2 m, else a guess by type | Catastro (cadastre): footprint x floors above ground x 3.2 m |
| Walls, fences, hedges | OpenStreetMap | OpenStreetMap |
| Trees | Crowns found in the imagery | Crowns found in the orthophoto (or in the LiDAR) |

With `ign:=true`, the Basque Country and Navarra have their own cadastres, which the generator does
not read: there the buildings come from OpenStreetMap too.

OpenStreetMap buildings are only as good as their tags: where they lack `height` and
`building:levels`, the height is a guess. Buildings are plain prisms with a facade texture and,
with `roof:=ortho`, the imagery on the roof. Open structures (a tower on legs, a canopy) come out
as a solid block over their footprint. A building taller than 60 floors (192 m) is cut to that
height. The public OpenStreetMap servers (Overpass) are often busy: the generator tries three and
waits, so a first `world:=area` can take minutes.

Trees use two Gazebo Fuel models, an oak and a pine (CC0, `models/trees/` of the worlds package), scaled to each crown
and merged into one mesh. Only the trunks collide: the robot drives under the crowns. So that its
sensors agree (Gazebo's lidar sees the drawn mesh), every crown starts 2.5 m over the ground or
higher, as on a pruned street tree, and below that a tree is drawn as its trunk alone, no wider than
what collides (the models' roots and low branches are drawn into it). Before
planting, every tree is checked: none inside a building, wall or structure, none within 3 m of the
robot's start, no two trunks closer than 1.5 m.

## Making a map of the same place

Every launch writes `~/.ros/easynav_geo/last_world.yaml` with what it started:

```yaml
world: area
directory: /home/<user>/.ros/easynav_geo/worlds/area_40.41680_-3.70380_250
latitude: 40.41680000
longitude: -3.70380000
radius: 250.0
ground_elevation: 648.015      # metres above sea level of z = 0
mesh: /home/<user>/.ros/easynav_geo/worlds/area_40.41680_-3.70380_250/navmap.stl
```

An application that follows the simulation reads it to make its map of the very place the robot
is in, in the same frame: `mesh` is the ground and the obstacles (`navmap.stl`, in the world frame),
and `latitude`, `longitude` and `ground_elevation` tie that frame to the GPS. For example, a NavMap
of the place can be made from `navmap.stl` with `navmap_tools`. Nothing in this playground depends
on which application does it.

## Optional: the IGN data of Spain (`ign:=true`)

In Spain, `ign:=true` makes the world from the data of the Instituto Geográfico Nacional and the
Catastro instead (see [Where the data comes from](#where-the-data-comes-from)): a finer terrain
(5 m instead of 30 m), the PNOA orthophoto and the cadastre's buildings, and, if you have them, the
PNOA-LiDAR point clouds of the place.

It needs **laspy** (reads the LiDAR's `.las`/`.laz` files) and **laszip** (its backend that
decompresses `.laz`). They are not in rosdep, so `rosdep install` does not install them; from apt:

```bash
sudo apt install python3-laspy python3-laszip
```

`ign:=true` checks them before anything is made: if either is missing, the launch stops and says
which one and the command above. `ign:=false` (the default) never uses them.

### PNOA-LiDAR

The IGN terrain is a 5 m grid and the cadastre gives floors, not heights. With the classified
PNOA-LiDAR point cloud of the area (`ORT-CLA` `.laz` sheets of 2 x 2 km), the generator uses:

- the LiDAR ground for the terrain wherever it has data (1 m, drawn with a 2.5 m mesh);
- the real roof height of each cadastre building;
- trees at their real height and crown, from the vegetation height;
- anything else that stands out over the ground (walls, sheds, tall fences) as a block.

The CNIG [download centre](https://centrodedescargas.cnig.es/CentroDescargas/lidar-ign-pnoa) does
not allow scripted downloads: download the sheets of your area by hand and put them in
`~/.ros/easynav_geo/lidar/` (or give `lidar:=<file or directory>`). The generator lists the sheets
a place needs (and marks those already there):

```bash
python3 $(ros2 pkg prefix easynav_playground_geo_worlds)/lib/easynav_playground_geo_worlds/geo_world.py \
  --lat 40.4168 --lon -3.7038 --radius 250 --ign --lidar-tiles
```

With `ign:=true`, the launch prints the same list when it finds no LiDAR for the place, and makes
the world without it.

## The robot

The Summit XL of [`playground_summit`](../../playground_summit/easynav_playground_summit), with its
controllers, sensors and ROS–Gazebo bridge, started by that playground's `summit.launch.yaml`. It
appears at the spawn spot, 0.5 m above the ground, and falls onto it. Its topics are those of
`playground_summit`.

## GPU

On a laptop with an NVIDIA GPU and hybrid graphics, Gazebo may fall back to software rendering
(`libEGL warning: egl: failed to create dri2 screen`) and run many times slower than real time. With
`gpu:=auto` (the default) the launch renders on the NVIDIA GPU when
`/usr/share/glvnd/egl_vendor.d/10_nvidia.json` exists, by setting `__NV_PRIME_RENDER_OFFLOAD`,
`__GLX_VENDOR_LIBRARY_NAME` and `__EGL_VENDOR_LIBRARY_FILENAMES` for every process it starts (unless
they are already set). `gpu:=nvidia` forces it, `gpu:=none` turns it off. Check the speed with
`gz topic -e -t /stats -n 1` (`real_time_factor`).

## Limits

- **Network.** The first launch of a place downloads its data, from public services that change and
  are sometimes down or slow. If a service fails, the generator says which; a `world:=area` whose
  walls could not be fetched is marked incomplete and made again on the next launch.
- **Detail.** Buildings are prisms of their footprint (no balconies or roof shapes), and their height
  is a number of floors x 3.2 m unless there is LiDAR or an OpenStreetMap `height`. Without LiDAR,
  trees come from the imagery: tight rows are missed and some dark lawns become trees.
- **Size.** The radius is capped at 2000 m. Large `world:=area` worlds are heavy for Gazebo (a city
  centre has thousands of buildings and trees); start with a radius of a few hundred metres.
- **Land only.** The sea is not simulated here; see [`playground_marine`](../../playground_marine).
- **`ign:=true` and LiDAR** are Spain only (the LiDAR untested in the Canary Islands, which use
  their own CRS).

## Tests

```bash
colcon test --packages-select easynav_playground_geo easynav_playground_geo_worlds
colcon test-result --verbose
```

Here, `test/test_launch_files.py`: `easynav_geo.launch.yaml` against `geo_sim.launch.py` of the
worlds package: every argument is forwarded, exists there and has the same default (nothing is
started). The tests of the simulation and the generator are in the
[worlds package](../easynav_playground_geo_worlds/README.md#tests).

## Licenses and attribution

The code of both packages is Apache-2.0 (`LICENSE`); the tree models are CC0
(`easynav_playground_geo_worlds/models/trees/README.md`).

A generated world contains data of its providers, under their terms. If you publish images, videos
or the worlds themselves, credit them as their terms ask:

| Data | Provider | Terms |
|---|---|---|
| Buildings, walls, fences, hedges | © OpenStreetMap contributors | ODbL 1.0 |
| Terrain (`ign:=false`) | Copernicus DEM GLO-30, © DLR e.V. and © Airbus Defence and Space GmbH, provided under COPERNICUS by the European Union and ESA | Copernicus DEM licence |
| Imagery (`ign:=false`) | Esri World Imagery (Esri, Maxar, Earthstar Geographics and the GIS User Community) | Esri's terms of use |
| MDT, PNOA orthophoto, PNOA-LiDAR (`ign:=true`) | © Instituto Geográfico Nacional | CC BY 4.0 |
| Buildings (`ign:=true`) | Dirección General del Catastro | The cadastre's reuse conditions, with attribution |
