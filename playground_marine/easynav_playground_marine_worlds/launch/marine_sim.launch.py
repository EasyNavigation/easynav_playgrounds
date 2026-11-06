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

"""The marine simulation: a Gazebo world with water and a boat or a submarine in it.

    ros2 launch easynav_playground_marine_worlds marine_sim.launch.py robot:=boat
    ros2 launch easynav_playground_marine_worlds marine_sim.launch.py robot:=submarine

* robot:=boat (the WAM-V, on the water) or robot:=submarine (the Tethys LRAUV, under it).
* The world is a synthetic sea (tools/synthetic_sea.py): offline, always the same, written to
  ~/.ros/easynav_playground_marine/ each time.
* waves:=true makes the vehicle move as in a sea (tools/wave_node.py): wave_height:= (m, crest to
  trough), wave_period:= (s), wave_direction:= (degrees, where the wave goes). The water is drawn with
  those waves, by a shader, unless wave_visual:=false (then it stays flat and only the vehicle moves).
* The vehicle carries the sensors of its kind (tools/marine_sensors.py); the simulator's own pose of
  it is /ground_truth/odom. rviz:=true (default) opens RViz with all of them.

It needs Gazebo, ros_gz, numpy and RViz; nothing else.
"""

import importlib.util
import os
import re

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOTS = ('boat', 'submarine')
OUT_DIR = os.path.expanduser('~/.ros/easynav_playground_marine')   # what this launch generates
SYNTHETIC_RADIUS = 500.0                                            # m, half-side of the sea
WORLD_NAME = 'marine_world'                                         # worlds/marine.world.in
MAX_WAVE_HEIGHT = 2.0                                               # m: the floats are 0.35 m high
GROUND_TRUTH = '/ground_truth/odom'          # the simulator's pose of the vehicle (not a sensor)

BOAT_HELP = (
    'robot:=boat: the WAM-V starts at (%.1f, %.1f) on the water (z = %.2f). Command it with a Twist on '
    '/cmd_vel (linear.x m/s forward, angular.z rad/s of yaw); its true pose is /ground_truth/odom:\n'
    "  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 1.5}, "
    "angular: {z: 0.2}}\"\n"
    'A command older than a second stops the engines.\n'
    'Sensors: /imu/data /imu/mag /gps/fix /lidar/points /camera/image_raw /echosounder')

SUBMARINE_HELP = (
    'robot:=submarine: the Tethys LRAUV starts at (%.1f, %.1f, %.2f). It is a torpedo: it steers and '
    'dives only while it moves forward. Command it with a Twist on /cmd_vel (linear.x m/s forward, '
    'angular.z rad/s of yaw, linear.z m/s up); its true pose is /ground_truth/odom:\n'
    "  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 1.5, z: -0.2}, "
    "angular: {z: 0.1}}\"\n"
    'A command older than a second stops the propeller and centres the fins.\n'
    'Sensors: /imu/data /imu/mag /gps/fix (at the surface) /pressure /depth /dvl/data /dvl/twist '
    '/dvl/altitude /sonar/image /sonar/image_fan /sonar/points')


def load_tool(tools, name):
    """The module tools/<name>.py (they are installed as programs, not as a Python package)."""
    spec = importlib.util.spec_from_file_location(name, os.path.join(tools, name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def synthetic_sea(tools):
    return load_tool(tools, 'synthetic_sea')


def underwater_at(sea, x=0.0, y=0.0):
    """True if (x, y) is under the sea surface (z = 0) by more than 1 m."""
    return float(sea.height(x, y)) < -1.0


def boat_start():
    """The z of the base of a boat put on the water: the sea surface, a little above it (it settles by
    itself; the WAM-V floats with its base 0.1 m under the surface)."""
    return 0.3


def submarine_start(sea, x=0.0, y=0.0, depth=None):
    """The z at which the submarine starts: `depth` m under the surface, by default half way to the
    floor but never deeper than 3 m."""
    available = max(0.0, -float(sea.height(x, y)))
    return -(min(3.0, available / 2.0) if depth is None else abs(float(depth)))


def vehicle_world(world_in, world_out, snippet, x, y, z, buoyant, sensors=''):
    """Write `world_in` with a vehicle added and return world_out.

    `snippet` is the SDF of the vehicle (@X@ @Y@ @Z@ @YAW@ filled in, @MODELS@ the directory of the
    models, @SENSORS@ `sensors`: the SDF of its sensors), put before the last </world>.
    `buoyant`: the name of the model that the world's Buoyancy plugin (the water) must act on: added
    to its <enable> list; a world without water is refused.
    """
    with open(world_in) as f:
        world = f.read()
    with open(snippet) as f:
        vehicle = f.read()
    for key, value in (('@X@', x), ('@Y@', y), ('@Z@', z), ('@YAW@', 0.0)):
        vehicle = vehicle.replace(key, '%.4f' % value)
    vehicle = vehicle.replace('@MODELS@', os.path.dirname(os.path.dirname(os.path.abspath(snippet))))
    vehicle = vehicle.replace('@SENSORS@', sensors)
    left = sorted(set(re.findall(r'@[A-Z_]+@', vehicle.split('-->', 1)[-1])))
    if left:
        raise RuntimeError('%s has placeholders this launch does not fill: %s' % (snippet, left))
    plugin = world.find('gz-sim-buoyancy-system')
    end_plugin = world.find('</plugin>', plugin) if plugin >= 0 else -1
    if end_plugin < 0:
        raise RuntimeError('the world has no water (no buoyancy plugin): this place has no sea')
    world = world[:end_plugin] + '    <enable>%s</enable>\n        ' % buoyant + world[end_plugin:]
    end = world.rfind('</world>')
    if end < 0:
        raise RuntimeError('%s has no </world>' % world_in)
    os.makedirs(os.path.dirname(os.path.abspath(world_out)), exist_ok=True)
    with open(world_out, 'w') as f:
        f.write(world[:end] + '\n        ' + vehicle + '\n    ' + world[end:])
    return world_out


def bridge_config(path, entries):
    """Write a ros_gz_bridge parameter_bridge config file and return its path.

    `entries`: (gz topic, ros topic, ros type, gz type, direction) with direction GZ_TO_ROS or
    ROS_TO_GZ. A command-line remap cannot do this for Gazebo's joint topics (`.../0/cmd_pos`): the
    `0` token is not a valid name for rcl, which kills the bridge.
    """
    lines = []
    for gz_topic, ros_topic, ros_type, gz_type, direction in entries:
        lines += ['- ros_topic_name: "%s"' % ros_topic, '  gz_topic_name: "%s"' % gz_topic,
                  '  ros_type_name: "%s"' % ros_type, '  gz_type_name: "%s"' % gz_type,
                  '  direction: %s' % direction]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        f.write('# generated by marine_sim.launch.py\n' + '\n'.join(lines) + '\n')
    return path


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'robot', default_value='boat',
            description='boat (the WAM-V, on the water) or submarine (the Tethys LRAUV, under it)'),
        DeclareLaunchArgument(
            'radius', default_value='',
            description='Half-side of the square sea, m (default 500, at least 100)'),
        DeclareLaunchArgument(
            'altitude', default_value='',
            description='robot:=submarine: depth under the surface, m (default half way to the '
                        'floor, at most 3); robot:=boat: ignored'),
        DeclareLaunchArgument('spawn_x', default_value='0', description='x where the vehicle starts'),
        DeclareLaunchArgument('spawn_y', default_value='0', description='y where it starts'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='RViz with the sensors of the vehicle'),
        DeclareLaunchArgument(
            'waves', default_value='false',
            description='true: the vehicle moves as in a sea, and the water is drawn with the waves'),
        DeclareLaunchArgument(
            'wave_visual', default_value='true',
            description='waves:=true: draw the waves on the water (a shader). false: the water that '
                        'is drawn stays flat and only the vehicle moves'),
        DeclareLaunchArgument(
            'wave_height', default_value='0.4',
            description='waves:=true: height of the main wave, crest to trough, m (0 to %g)'
                        % MAX_WAVE_HEIGHT),
        DeclareLaunchArgument('wave_period', default_value='4.0',
                              description='waves:=true: period of the main wave, s (2 to 15)'),
        DeclareLaunchArgument(
            'wave_direction', default_value='180',
            description='waves:=true: where the main wave goes, degrees (0: east, 90: north; '
                        '180: to the beach)'),
        DeclareLaunchArgument(
            'gpu', default_value='auto',
            description='auto/nvidia/none: with an NVIDIA GPU on a hybrid laptop Gazebo falls back '
                        'to software rendering (EGL dri2 error) and runs ~30x slower; nvidia (or '
                        'auto, when the NVIDIA EGL vendor file exists) renders on the GPU'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=setup)])


def setup(context):
    here = get_package_share_directory('easynav_playground_marine_worlds')
    models = os.path.join(get_package_share_directory('easynav_playground_marine_description'),
                          'models')
    tools = os.path.join(get_package_prefix('easynav_playground_marine_worlds'), 'lib',
                         'easynav_playground_marine_worlds')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    robot = arg('robot')
    if robot not in ROBOTS:
        raise RuntimeError("robot must be one of %s, not '%s'" % (', '.join(ROBOTS), robot))
    if arg('waves') not in ('true', 'false'):
        raise RuntimeError("waves must be true or false, not '%s'" % arg('waves'))
    waves_on = arg('waves') == 'true'
    if arg('wave_visual') not in ('true', 'false'):
        raise RuntimeError("wave_visual must be true or false, not '%s'" % arg('wave_visual'))
    if waves_on:
        height, period = float(arg('wave_height')), float(arg('wave_period'))
        float(arg('wave_direction'))
        if not 0.0 <= height <= MAX_WAVE_HEIGHT:
            raise RuntimeError('wave_height must be between 0 and %g m, not %g' % (MAX_WAVE_HEIGHT, height))
        if not 2.0 <= period <= 15.0:
            raise RuntimeError('wave_period must be between 2 and 15 s, not %g' % period)

    if arg('rviz') not in ('true', 'false'):
        raise RuntimeError("rviz must be true or false, not '%s'" % arg('rviz'))

    gpu = arg('gpu')
    egl_nvidia = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
    if gpu == 'nvidia' or (gpu == 'auto' and os.path.isfile(egl_nvidia)):
        # every process of this launch (Gazebo's sensors above all) renders on the NVIDIA GPU
        for name, value in (('__NV_PRIME_RENDER_OFFLOAD', '1'), ('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
                            ('__EGL_VENDOR_LIBRARY_FILENAMES', egl_nvidia)):
            if name not in context.environment:
                os.environ[name] = value
                context.environment[name] = value

    # the sea: the floor and the water, written now (it takes a second)
    radius = float(arg('radius') or SYNTHETIC_RADIUS)
    if radius < 100.0:
        raise RuntimeError('radius must be at least 100 m: the beach and the islet are within 400 m')
    sea = synthetic_sea(tools)
    out = os.path.join(OUT_DIR, 'synthetic_r%g' % radius)
    shader = None
    if waves_on and arg('wave_visual') == 'true':
        # the water is drawn with the very field that moves the vehicle (the same module)
        field = load_tool(tools, 'wave_node').Waves(height, period, float(arg('wave_direction')))
        shader = {'dir': os.path.join(models, 'marine_waves', 'materials'), 'params': field.shader_params()}
    sea.make(out, radius, template=os.path.join(here, 'worlds', 'marine.world.in'), shader=shader)
    world_file = os.path.join(out, 'world.sdf')

    # the world with the vehicle and its sensors put in it, the bridge that carries the commands, the
    # true pose, the sensors and the clock, and the autopilot of the robot
    table = load_tool(tools, 'marine_sensors')
    truth = ['--ros-args', '-r', 'odom:=%s' % GROUND_TRUTH]
    wave_entries = [('/world/%s/wrench' % WORLD_NAME, '/waves/wrench',
                     'ros_gz_interfaces/msg/EntityWrench', 'gz.msgs.EntityWrench',
                     'ROS_TO_GZ')] if waves_on else []
    x, y = float(arg('spawn_x')), float(arg('spawn_y'))
    if not underwater_at(sea, x, y):
        raise RuntimeError('there is no water at (%.0f, %.0f): the %s needs more than 1 m of it '
                           '(spawn_x:= spawn_y:= to start elsewhere)' % (x, y, robot))
    if robot == 'boat':
        z = boat_start()
        snippet, buoyant, help_text = (
            os.path.join(models, 'marine_wamv', 'wamv.sdf.in'), 'wamv', BOAT_HELP % (x, y, z))
        joints = '/model/wamv/joint/'
        bridged = bridge_config(
            os.path.join(OUT_DIR, 'boat_bridge.yaml'), [
                ('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', 'GZ_TO_ROS'),
                ('/model/wamv/odometry', GROUND_TRUTH, 'nav_msgs/msg/Odometry',
                 'gz.msgs.Odometry', 'GZ_TO_ROS'),
                (joints + 'left_engine_propeller_joint/cmd_thrust', '/boat/left_thrust',
                 'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'),
                (joints + 'right_engine_propeller_joint/cmd_thrust', '/boat/right_thrust',
                 'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ')] + wave_entries +
            table.bridge_entries(robot))
        autopilot = ExecuteProcess(cmd=[os.path.join(tools, 'boat_cmd.py'), '--sim-time'] + truth,
                                   name='boat_cmd', output='screen')
    else:
        altitude = arg('altitude')
        z = submarine_start(sea, x, y, float(altitude) if altitude else None)
        snippet, buoyant, help_text = (
            os.path.join(models, 'marine_tethys', 'tethys.sdf.in'), 'tethys',
            SUBMARINE_HELP % (x, y, z))
        joints = '/model/tethys/joint/'
        # Gazebo's joint topics have a `0` token that a ROS name cannot have: a config file
        bridged = bridge_config(
            os.path.join(OUT_DIR, 'tethys_bridge.yaml'), [
                ('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', 'GZ_TO_ROS'),
                ('/model/tethys/odometry', GROUND_TRUTH, 'nav_msgs/msg/Odometry',
                 'gz.msgs.Odometry', 'GZ_TO_ROS'),
                (joints + 'propeller_joint/cmd_thrust', '/tethys/thrust',
                 'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'),
                (joints + 'vertical_fins_joint/0/cmd_pos', '/tethys/rudder',
                 'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'),
                (joints + 'horizontal_fins_joint/0/cmd_pos', '/tethys/elevator',
                 'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ')] + wave_entries +
            table.bridge_entries(robot))
        autopilot = ExecuteProcess(cmd=[os.path.join(tools, 'auv_cmd.py'), '--sim-time'] + truth,
                                   name='auv_cmd', output='screen')
    world = vehicle_world(world_file, os.path.join(OUT_DIR, 'world_%s.sdf' % robot),
                          snippet, x, y, z, buoyant, table.sdf(robot))
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name='%s_bridge' % robot, output='screen', parameters=[{'config_file': bridged}])
    frames = ['--frame=%s:%s' % (f[0], ','.join('%.6f' % v for v in f[1:]))
              for f in table.static_frames(robot)]
    odom_to_tf = ExecuteProcess(cmd=[os.path.join(tools, 'odom_to_tf.py'), '--sim-time'] + frames +
                                truth, name='odom_to_tf', output='screen')
    sensors = [ExecuteProcess(
        cmd=[os.path.join(tools, 'marine_sensor_node.py'), '--vehicle', robot, '--sim-time'],
        name='marine_sensors', output='screen')]
    if robot == 'submarine':
        sensors.append(ExecuteProcess(cmd=[os.path.join(tools, 'sonar_node.py'), '--sim-time'],
                                      name='sonar', output='screen'))
    if arg('rviz') == 'true':
        sensors.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(here, 'rviz', '%s.rviz' % robot)],
            parameters=[{'use_sim_time': True}]))
    gazebo = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(here, 'launch', 'world.launch.yaml')),
        launch_arguments={'world': world, 'gui': arg('gui')}.items())
    actions = [LogInfo(msg='marine simulation: %s, synthetic sea (%s)' % (robot, out)),
               LogInfo(msg=help_text), gazebo, bridge, odom_to_tf, autopilot] + sensors
    if waves_on:
        actions.append(ExecuteProcess(
            cmd=[os.path.join(tools, 'wave_node.py'), '--vehicle', robot, '--height', arg('wave_height'),
                 '--period', arg('wave_period'), '--direction', arg('wave_direction'), '--sim-time'] +
            truth,
            name='waves', output='screen'))
    return actions
