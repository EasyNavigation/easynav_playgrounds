<!--
Copyright 2026 José Miguel Guerrero
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Geo Playground

The world of **any real place** in Gazebo, from its coordinates and a radius, with the Robotnik
Summit XL in it. The terrain comes from public elevation models and the ground is covered with
aerial imagery; on request, the buildings, walls and trees of the place are added too. Everything is
made from open data, once per place, and kept for the next launch.

```bash
# the terrain of a place and its aerial imagery (fast): Zermatt, in the Alps
ros2 launch easynav_playground_geo easynav_geo.launch.yaml lat:=46.0207 lon:=7.7491 radius:=300

# the same plus buildings, walls and trees: the centre of Madrid
ros2 launch easynav_playground_geo easynav_geo.launch.yaml lat:=40.4168 lon:=-3.7038 radius:=250 world:=area
```

Like the other playgrounds, this package is **only the simulation**: the world and the robot.
Navigation (EasyNav, or anything else) is another launch, started after this one; see
[Making a map of the same place](#making-a-map-of-the-same-place).

## Contents

- [Build](#build)
- [The two worlds](#the-two-worlds)
- [Arguments](#arguments)
- [Where the data comes from](#where-the-data-comes-from)
- [What is generated, and where](#what-is-generated-and-where)
- [Making a map of the same place](#making-a-map-of-the-same-place)
- [More precision in Spain: PNOA-LiDAR](#more-precision-in-spain-pnoa-lidar)
- [The robot](#the-robot)
- [GPU](#gpu)
- [Using the generator alone](#using-the-generator-alone)
- [Limits](#limits)
- [Tests](#tests)
- [Licenses and attribution](#licenses-and-attribution)

## Build

ROS 2 Rolling, with Gazebo (`gz_sim_vendor`) and the Summit XL of
[`playground_summit`](../../playground_summit/easynav_playground_summit) in the workspace:

```bash
colcon build --packages-up-to easynav_playground_geo
source install/setup.bash
```

The world generator needs, besides the Python packages rosdep installs (numpy, scipy, OpenCV,
requests, Pillow, pyproj), two that have no rosdep key:

```bash
pip install --user --break-system-packages mapbox_earcut trimesh "numpy<2"
```

Outside Spain it also uses `navmap_tools` (the NavMap repository) for the Copernicus terrain and
the Esri imagery. The first launch of a place needs a network connection; the next ones do not.

## The two worlds

| `world:=` | What is in it | First launch of a place |
|---|---|---|
| `terrain` (default) | The terrain with the aerial imagery draped on it | Downloads the elevation model and the imagery: seconds to a minute |
| `area` | The terrain, plus buildings, walls, fences and hedges, and trees | Also queries the cadastre / OpenStreetMap, builds every building and plants the trees: minutes |

Both are the same generator (`tools/geo_world.py`): `terrain` is `area` without buildings, walls and
trees. Each world is made once and kept; launching the same place with the same options again
starts Gazebo at once. Changing any option (the radius, `trees`, the LiDAR files...) makes another
world, which is generated the first time.

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
| `lidar` | `''` | Spain only: a PNOA-LiDAR `.laz`/`.las` file or a directory of them; `''` uses `~/.ros/easynav_geo/lidar/` if it has any (see [below](#more-precision-in-spain-pnoa-lidar)) |
| `spawn_x`, `spawn_y` | `''` | Where the Summit starts, in the world frame (m). `''`: the free spot nearest to the origin that the generator found (not inside or next to a building, wall or tree) |
| `spawn_z` | `''` | Height of the Summit at the start. `''`: 0.5 m above the ground there (the highest surface, so on a roof if `spawn_x`/`spawn_y` fall on a building) |
| `gui` | `true` | Gazebo GUI; `false` runs headless |
| `gpu` | `auto` | `auto`, `nvidia` or `none` (see [GPU](#gpu)) |

A wrong value (an unknown `world`, a latitude out of range, a radius of 0...) stops the launch with
a message saying which argument is wrong, before anything is downloaded.

## Where the data comes from

The generator chooses the sources by where the place is:

| | Spain | Anywhere else |
|---|---|---|
| Terrain | IGN MDT, 5 m grid (PNOA-LiDAR ground where you have the files) | Copernicus GLO-30, 30 m grid: smooth, streets come out flat |
| Imagery | PNOA orthophoto (IGN) | Esri World Imagery |
| Buildings | Catastro (cadastre): footprint x floors above ground x 3.2 m | OpenStreetMap: footprint x `height`, else `building:levels` x 3.2 m, else a guess by type |
| Walls, fences, hedges | OpenStreetMap | OpenStreetMap |
| Trees | Crowns found in the orthophoto (or in the LiDAR) | Crowns found in the imagery |

The Basque Country and Navarra have their own cadastres, which the generator does not read: there
the buildings also come from OpenStreetMap.

Outside Spain the buildings are only as good as OpenStreetMap's tags: where they lack `height` and
`building:levels`, the height is a guess. Buildings are plain prisms with a facade texture and,
with `roof:=ortho`, the imagery on the roof. Open structures (a tower on legs, a canopy) come out
as a solid block over their footprint. A building taller than 60 floors (192 m) is cut to that
height. The public OpenStreetMap servers (Overpass) are often busy: the generator tries three and
waits, so a first `world:=area` can take minutes.

Trees use two Gazebo Fuel models, an oak and a pine (CC0, `models/trees/`), scaled to each crown
and merged into one mesh. Only the trunks collide: the robot drives under the crowns. Before
planting, every tree is checked: none inside a building, wall or structure, none within 3 m of the
robot's start, no two trunks closer than 1.5 m.

## What is generated, and where

Each world goes to its own directory under `~/.ros/easynav_geo/worlds/`, named
`terrain_<lat>_<lon>_<radius>` or `area_<lat>_<lon>_<radius>`:

| File | What it is |
|---|---|
| `world.sdf` | The Gazebo world |
| `terrain.obj`, `terrain.mtl`, `ortho.jpg` | The terrain as it is drawn, with the imagery |
| `terrain.stl` | The terrain as it collides (a coarser mesh, so that Gazebo stays fast) |
| `buildings.*`, `walls.*`, `trees.*` | `world:=area`: what stands on the terrain, drawn and colliding |
| `navmap.stl` | Terrain + buildings + walls + tree trunks, without the crowns: the surface a ground robot moves on and the obstacles on it, for making a map (see below) |
| `navmap_ground.stl`, `navmap_holes.stl` | The same split in two: the ground alone, and what stands on it |
| `world.yaml` | The place, the elevation of the origin, the spawn spot, and how many buildings, walls and trees there are |

Other caches: the OpenStreetMap answers in `~/.cache/easynav_geo/overpass/`, and outside Spain the
terrain and imagery tiles in `navmap_tools`' own cache. To make a world again from scratch, remove
its directory.

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
of the place can be made from `navmap.stl` with `navmap_tools`. Nothing in this package depends on
which application does it.

## More precision in Spain: PNOA-LiDAR

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
python3 $(ros2 pkg prefix easynav_playground_geo)/lib/easynav_playground_geo/geo_world.py \
  --lat 40.4168 --lon -3.7038 --radius 250 --lidar-tiles
```

The launch prints the same list when it finds no LiDAR for a place in Spain. Reading `.laz` needs
`pip install --user --break-system-packages laspy lazrs`.

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

## Using the generator alone

`geo_world.py` makes a world without starting Gazebo, and has more options than the launch
(`--help`):

```bash
G=$(ros2 pkg prefix easynav_playground_geo)/lib/easynav_playground_geo/geo_world.py
python3 $G --lat 48.8562 --lon 2.2976 --radius 400                 # Paris, with everything
python3 $G --lat 46.0207 --lon 7.7491 --radius 300 --no-buildings --no-trees --no-walls
```

Among them: `--terrain-res` and `--navmap-res` (mesh spacing), `--ortho-ppm` (imagery pixels per
metre), `--max-trees`, `--force` (make it again), and `--survey-model`, `--survey-stl` and
`--survey-elevation`, which put a survey mesh of the real site (photogrammetry, for instance) into
the terrain. It prints its progress, and as its last line the directory of the world.

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
- **LiDAR** is Spain only (and untested in the Canary Islands, which use their own CRS).

## Tests

```bash
colcon test --packages-select easynav_playground_geo && colcon test-result --verbose
```

- `test/test_launch_files.py`: `easynav_geo.launch.yaml` against `geo_sim.launch.py`: every argument
  is forwarded, exists there and has the same default (nothing is started).
- `test/test_geo_sim_launch.py`: the decisions of the launch without starting anything: the generator
  command of each world, the refused arguments, the spawn pose, `last_world.yaml`.
- `test/test_geo_world.py`: the parts of the generator that work offline (OpenStreetMap buildings
  from a canned answer, the regions, the geometry, the cache key of each world).

## Licenses and attribution

The code is Apache-2.0 (`LICENSE`); the tree models are CC0 (`models/trees/README.md`).

A generated world contains data of its providers, under their terms. If you publish images, videos
or the worlds themselves, credit them as their terms ask:

| Data | Provider | Terms |
|---|---|---|
| MDT, PNOA orthophoto, PNOA-LiDAR (Spain) | © Instituto Geográfico Nacional | CC BY 4.0 |
| Buildings (Spain) | Dirección General del Catastro | The cadastre's reuse conditions, with attribution |
| Buildings, walls, fences, hedges | © OpenStreetMap contributors | ODbL 1.0 |
| Terrain outside Spain | Copernicus DEM GLO-30, © DLR e.V. and © Airbus Defence and Space GmbH, provided under COPERNICUS by the European Union and ESA | Copernicus DEM licence |
| Imagery outside Spain | Esri World Imagery (Esri, Maxar, Earthstar Geographics and the GIS User Community) | Esri's terms of use |
