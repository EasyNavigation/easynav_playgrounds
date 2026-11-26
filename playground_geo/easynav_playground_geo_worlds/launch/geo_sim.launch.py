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

"""The world of a real place in Gazebo, with the Summit XL in it.

geo_world.py makes the world of lat, lon, radius from open data (once: it is kept in
~/.ros/easynav_geo/worlds/), then Gazebo starts it and the Summit is spawned on the ground.

* world:=terrain: the terrain and its aerial imagery.
* world:=area: the same, plus buildings, walls and trees.

By default the data is open data of anywhere: Copernicus terrain, Esri imagery and OpenStreetMap
buildings and walls. In Spain, ign:=true takes the IGN terrain, the PNOA orthophoto, the Catastro
buildings and the PNOA-LiDAR instead: it needs laspy and laszip (sudo apt install python3-laspy
python3-laszip), and stops saying so when they are not installed.

What was launched is written to ~/.ros/easynav_geo/last_world.yaml (the place and the mesh of the
world), so that an application can make its map of the same place.
"""

import math
import os
import struct
import subprocess
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORLDS = ('terrain', 'area')
OUT_DIR = '~/.ros/easynav_geo/worlds'
LAST_WORLD = '~/.ros/easynav_geo/last_world.yaml'
DEFAULT_PLACE = ('40.284796', '-3.821407', '300')   # the URJC campus at Fuenlabrada (Madrid)
MAX_RADIUS = 2000.0
EGL_NVIDIA = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
IGN_DEPS = 'sudo apt install python3-laspy python3-laszip'


def ign_missing():
    """What ign:=true needs to read the PNOA-LiDAR and is not installed: [] when laspy reads .laz
    with its LASzip backend."""
    try:
        import laspy
    except ImportError:
        return ['laspy', 'laszip']
    try:
        return [] if laspy.LazBackend.Laszip.is_available() else ['laszip']
    except AttributeError:      # a laspy without LazBackend (1.x)
        return ['laszip']


def check_ign(ign, missing):
    """Raise RuntimeError if ign:=true and `missing` (from ign_missing()) is not empty."""
    if ign == 'true' and missing:
        raise RuntimeError('ign:=true needs %s, which %s not installed: %s (or launch with '
                           'ign:=false, OpenStreetMap data)' % (
                               ' and '.join(missing), 'is' if len(missing) == 1 else 'are',
                               IGN_DEPS))


def check_place(lat, lon, radius):
    """Return lat, lon, radius as floats, or raise RuntimeError saying what is wrong."""
    try:
        la, lo, r = float(lat), float(lon), float(radius)
    except ValueError:
        raise RuntimeError('lat:=, lon:= and radius:= must be numbers (got %r, %r, %r)' % (
            lat, lon, radius))
    if not -85.0 <= la <= 85.0 or not -180.0 <= lo <= 180.0:
        raise RuntimeError('lat:=%s lon:=%s is not a place on Earth (lat -85..85, lon -180..180)' % (
            lat, lon))
    if not 0.0 < r <= MAX_RADIUS:
        raise RuntimeError('radius:=%s must be above 0 and at most %.0f m' % (radius, MAX_RADIUS))
    return la, lo, r


def generator_command(tool, template, cfg):
    """The geo_world.py command line for the launch arguments `cfg` (strings, as given)."""
    world = cfg['world']
    if world not in WORLDS:
        raise RuntimeError("world:=%s must be one of: %s" % (world, ', '.join(WORLDS)))
    for k in ('trees', 'walls', 'ign'):
        if cfg[k] not in ('true', 'false'):
            raise RuntimeError('%s:=%s must be true or false' % (k, cfg[k]))
    if cfg['roof'] not in ('ortho', 'flat'):
        raise RuntimeError('roof:=%s must be ortho or flat' % cfg['roof'])
    lat, lon, radius = check_place(cfg['lat'], cfg['lon'], cfg['radius'])
    cmd = [sys.executable, tool, '--lat', repr(lat), '--lon', repr(lon), '--radius', repr(radius),
           '--template', template, '--out-dir', OUT_DIR, '--sea', 'off']
    if world == 'terrain':
        cmd += ['--no-buildings', '--no-trees', '--no-walls']
    else:
        cmd += ['--roof', cfg['roof']]
        if cfg['trees'] == 'false':
            cmd.append('--no-trees')
        if cfg['walls'] == 'false':
            cmd.append('--no-walls')
    if cfg['ign'] == 'true':
        cmd.append('--ign')
        if cfg['lidar']:
            cmd += ['--lidar', os.path.expanduser(cfg['lidar'])]
    elif cfg['lidar']:
        raise RuntimeError('lidar:=%s needs ign:=true (the PNOA-LiDAR goes with the IGN data)' %
                           cfg['lidar'])
    return cmd


def read_world(world_dir):
    """world.yaml of a world directory as a dict of strings (plus 'dir')."""
    out = {'dir': world_dir}
    with open(os.path.join(world_dir, 'world.yaml')) as f:
        for line in f:
            if ':' in line and not line.lstrip().startswith('#'):
                k, v = line.split(':', 1)
                out[k.strip()] = v.strip()
    return out


def read_stl(path):
    """The triangles (n, 3, 3) of a binary STL file."""
    import numpy as np
    with open(path, 'rb') as f:
        data = f.read()
    (n,) = struct.unpack_from('<I', data, 80)
    rec = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]),
                        count=n, offset=84)
    return rec['v'].reshape(n, 3, 3).astype(np.float64)


def ground_height(tris, x, y):
    """The highest surface of the triangles (n, 3, 3) over (x, y), or None if none is."""
    import numpy as np
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    det = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    ok = np.abs(det) > 1e-12
    det = np.where(ok, det, 1.0)
    l0 = ((b[:, 1] - c[:, 1]) * (x - c[:, 0]) + (c[:, 0] - b[:, 0]) * (y - c[:, 1])) / det
    l1 = ((c[:, 1] - a[:, 1]) * (x - c[:, 0]) + (a[:, 0] - c[:, 0]) * (y - c[:, 1])) / det
    l2 = 1.0 - l0 - l1
    inside = ok & (l0 >= -1e-9) & (l1 >= -1e-9) & (l2 >= -1e-9)
    if not inside.any():
        return None
    z = l0 * a[:, 2] + l1 * b[:, 2] + l2 * c[:, 2]
    return float(z[inside].max())


def spawn_pose(world, given):
    """Where the Summit is spawned: the given x, y, z, else the free spot geo_world.py chose. Given
    only x, y: on the ground there (the highest surface of navmap.stl), 0.5 m above it."""
    if not (given['spawn_x'] or given['spawn_y']):
        x, y, z = (float(world[k]) for k in ('spawn_x', 'spawn_y', 'spawn_z'))
        return x, y, float(given['spawn_z']) if given['spawn_z'] else z
    x, y = float(given['spawn_x'] or 0.0), float(given['spawn_y'] or 0.0)
    if given['spawn_z']:
        return x, y, float(given['spawn_z'])
    ground = ground_height(read_stl(os.path.join(world['dir'], 'navmap.stl')), x, y)
    if ground is None:
        raise RuntimeError('spawn %.1f, %.1f is outside the world (radius %s m)' % (
            x, y, world.get('radius', '?')))
    return x, y, ground + 0.5


def write_last_world(world, kind, path=LAST_WORLD):
    """What was launched, for an application that makes its map of the same place."""
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    lines = [
        '# written by easynav_playground_geo_worlds: the world of the last launch',
        'world: %s' % kind,
        'directory: %s' % world['dir'],
        'latitude: %s' % world['latitude'],
        'longitude: %s' % world['longitude'],
        'radius: %s' % world['radius'],
        '# metres above sea level of z = 0 (the ground at the origin)',
        'ground_elevation: %s' % world['ground_elevation'],
        '# what a map of the place is made from: terrain + buildings + walls + trunks, in the',
        "# world's frame (x east, y north, z up from the ground at the origin)",
        'mesh: %s' % os.path.join(world['dir'], 'navmap.stl'),
    ]
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='terrain',
                              description='terrain: the terrain and its imagery. area: plus '
                                          'buildings, walls and trees'),
        DeclareLaunchArgument('lat', default_value=DEFAULT_PLACE[0],
                              description='Latitude of the centre (WGS84 degrees): the origin of '
                                          'the world'),
        DeclareLaunchArgument('lon', default_value=DEFAULT_PLACE[1],
                              description='Longitude of the centre (WGS84 degrees)'),
        DeclareLaunchArgument('radius', default_value=DEFAULT_PLACE[2],
                              description='Half side of the square world, m (up to %.0f)' %
                                          MAX_RADIUS),
        DeclareLaunchArgument('trees', default_value='true',
                              description='world:=area: trees (crowns found in the imagery)'),
        DeclareLaunchArgument('walls', default_value='true',
                              description='world:=area: walls, fences and hedges from '
                                          'OpenStreetMap'),
        DeclareLaunchArgument('roof', default_value='ortho',
                              description='world:=area: ortho, the roofs show the aerial '
                                          'imagery; flat, one colour'),
        DeclareLaunchArgument('ign', default_value='false',
                              description='false: OpenStreetMap buildings and walls, Copernicus '
                                          'terrain, Esri imagery (anywhere). true (Spain only): '
                                          'IGN terrain, PNOA orthophoto, Catastro buildings and '
                                          'PNOA-LiDAR; needs %s' % IGN_DEPS),
        DeclareLaunchArgument('lidar', default_value='',
                              description="ign:=true: PNOA-LiDAR .laz/.las file or directory "
                                          "('' = ~/.ros/easynav_geo/lidar if it has sheets): finer "
                                          'terrain, real heights of buildings and trees'),
        DeclareLaunchArgument('spawn_x', default_value='',
                              description="x of the Summit ('' = a free spot near the origin)"),
        DeclareLaunchArgument('spawn_y', default_value='', description='y of the Summit'),
        DeclareLaunchArgument('spawn_z', default_value='',
                              description="z of the Summit ('' = 0.5 m above the ground there)"),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('gpu', default_value='auto',
                              description='auto, nvidia or none: on a laptop with an NVIDIA GPU '
                                          'Gazebo may fall back to software rendering; nvidia (or '
                                          'auto, when the NVIDIA EGL vendor file exists) renders '
                                          'on the GPU'),
        OpaqueFunction(function=setup),
    ])


def setup(context):
    here = get_package_share_directory('easynav_playground_geo_worlds')
    summit_worlds = get_package_share_directory('easynav_playground_summit_worlds')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    cfg = {k: arg(k) for k in ('world', 'lat', 'lon', 'radius', 'trees', 'walls', 'roof',
                               'ign', 'lidar')}
    given = {k: arg(k) for k in ('spawn_x', 'spawn_y', 'spawn_z')}
    gpu = arg('gpu')
    if gpu not in ('auto', 'nvidia', 'none'):
        raise RuntimeError('gpu:=%s must be auto, nvidia or none' % gpu)

    tool = os.path.join(get_package_prefix('easynav_playground_geo_worlds'), 'lib',
                        'easynav_playground_geo_worlds', 'geo_world.py')
    cmd = generator_command(tool, os.path.join(here, 'worlds', 'geo.world.in'), cfg)
    check_ign(cfg['ign'], ign_missing() if cfg['ign'] == 'true' else [])
    print('geo_sim.launch.py: %s world of %s, %s, radius %s m (made once, then reused; the first '
          'time takes from seconds to minutes)' % (cfg['world'], cfg['lat'], cfg['lon'],
                                                   cfg['radius']), flush=True)
    # its progress is shown as it goes; the last line it prints is the world's directory
    last = ''
    with subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1) as proc:
        for line in proc.stdout:
            print(line.rstrip(), flush=True)
            if line.strip():
                last = line.strip()
    if proc.returncode != 0:
        raise RuntimeError('geo_world.py failed (see above)')
    world = read_world(last)
    x, y, z = spawn_pose(world, given)
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise RuntimeError('no spawn pose (%s, %s, %s)' % (x, y, z))
    write_last_world(world, cfg['world'])

    def setenv(name, value):
        # the processes of the launch get a copy of the environment taken when it started
        if name not in context.environment:
            os.environ[name] = value
            context.environment[name] = value

    if gpu == 'nvidia' or (gpu == 'auto' and os.path.isfile(EGL_NVIDIA)):
        setenv('__NV_PRIME_RENDER_OFFLOAD', '1')
        setenv('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
        setenv('__EGL_VENDOR_LIBRARY_FILENAMES', EGL_NVIDIA)

    return [
        LogInfo(msg='world %s in %s; the Summit at (%.1f, %.1f, %.2f)' % (
            cfg['world'], world['dir'], x, y, z)),
        # The playground's world launch starts the GUI from libexec/gz/sim/, which gz_sim_vendor
        # does not have (it is libexec/gz/sim<version>/): it would never open. It is started here.
        ExecuteProcess(condition=IfCondition(LaunchConfiguration('gui')), cmd=['gz', 'sim', '-g'],
                       name='gz_gui', output='screen'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(summit_worlds, 'launch', 'world.launch.yaml')),
            launch_arguments={'world': os.path.join(world['dir'], 'world.sdf'),
                              'gui': 'false'}.items()),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(summit_worlds, 'launch',
                                                    'summit.launch.yaml')),
            launch_arguments={'x': str(x), 'y': str(y), 'z': str(z)}.items()),
    ]
