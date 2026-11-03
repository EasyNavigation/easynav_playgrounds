# Copyright 2026 Intelligent Robotics Lab
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

"""Build the town world from disaster_world: only the ground, the roads and the buildings.

Usage: prepare_world.py <disaster_world> <fuel_dir> <park_dir> <out_worlds_pkg>

  disaster_world  a clone of https://github.com/juanscelyg/disaster_world
  fuel_dir        the Fuel building models it includes by URL, unzipped (one folder each,
                  named as in Fuel, e.g. "Collapsed House")
  park_dir        the nathan_benderson_park files it references (meshes/, materials/textures/)

Vehicles, dumpsters, victims, trees, the ground station and the heaviest buildings are left
out, and the animated water becomes a flat plane. Every model is local (no download at startup), and the park keeps
only its albedo textures, at 1024 px: rendering is what slows Gazebo down here.
"""
import os
import re
import shutil
import sys

from PIL import Image

WORLD = 'worlds/disaster_nathan_benderson_park_world.sdf'
DROP = {'coast_water', 'nathan_benderson_island_trees', 'ground_station', 'ambulance', 'bus',
        'dumpster', 'fire_truck', 'hatchback', 'hatchback_blue', 'hatchback_red', 'pickup',
        'suv', 'truck_box', 'rescue_randy', 'rescue_randy_sitting', 'survivor_male',
        'survivor_female', 'standing_person'}
# The heaviest buildings (4,500-7,400 triangles each, or 4 MB), and the pavement under the gas
# stations and the parking garage
COMPLEX = {'collapsed_industrial_building', 'apartment', 'parking_garage', 'school',
           'gas_station', 'power_plant', 'pavement_plane'}
DROP |= COMPLEX
# disaster_world models that only include a Fuel model by URL: name -> Fuel folder
FUEL = {'apartment': 'Apartment', 'collapsed_fire_station': 'Collapsed Fire Station',
        'collapsed_house': 'Collapsed House',
        'collapsed_industrial_building': 'Collapsed industrial building',
        'fire_station': 'Fire Station', 'grocery_store': 'Grocery store',
        'law_office': 'Law office', 'osrf_first_office': 'OSRF First Office',
        'parking_garage': 'Parking garage', 'post_office': 'Post office', 'salon': 'Salon',
        'school': 'School', 'thrift_shop': 'Thrift shop'}
TEXTURE_SIZE = 1024
PARK_URL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/nathan_benderson_park/5/files/'


def fuel_model(src, name, dst):
    shutil.copytree(src, dst, ignore=shutil.ignore_patterns('thumbnails', 'metadata.pbtxt'))
    sdf = open(os.path.join(dst, 'model.sdf')).read()
    sdf = re.sub(r'model://[^/<]+/', '', sdf)  # relative to the model
    sdf = re.sub(r'<model name="[^"]*"', f'<model name="{name}"', sdf, count=1)
    open(os.path.join(dst, 'model.sdf'), 'w').write(sdf)
    # The meshes name their textures bare: point them to materials/textures/
    textures = os.path.join(dst, 'materials', 'textures')
    for mesh in os.listdir(os.path.join(dst, 'meshes')):
        path = os.path.join(dst, 'meshes', mesh)
        dae = open(path).read()
        dae = re.sub(r'<init_from>([^<>/]+)</init_from>',
                     lambda m: f'<init_from>../materials/textures/{m.group(1)}</init_from>'
                     if os.path.exists(os.path.join(textures, m.group(1))) else m.group(0), dae)
        open(path, 'w').write(dae)
    config = open(os.path.join(dst, 'model.config')).read()
    note = (f'Local copy of the Fuel model "{os.path.basename(src)}" '
            f'(https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
            f'{os.path.basename(src).replace(" ", "%20")}), as disaster_world includes it. '
            'Licensed under Creative Commons Zero v1.0 Universal.')
    config = re.sub(r'<description>.*?</description>', f'<description>\n    {note}\n  </description>',
                    config, flags=re.S)
    config = re.sub(r'<name>[^<]*</name>', f'<name>{name}</name>', config, count=1)
    open(os.path.join(dst, 'model.config'), 'w').write(config)


def park_model(src_model, park_dir, dst):
    os.makedirs(os.path.join(dst, 'meshes'))
    os.makedirs(os.path.join(dst, 'materials', 'textures'))
    sdf = open(os.path.join(src_model, 'model.sdf')).read()
    # No trees
    sdf = re.sub(r'\s*<visual name="(PalmTree|Trees)_visual">.*?</visual>', '', sdf, flags=re.S)
    # Albedo only: no normal, roughness, metalness or environment maps
    sdf = re.sub(r'\s*<(normal_map|roughness_map|metalness_map|environment_map)>[^<]*</\1>', '',
                 sdf)
    used = sorted(set(re.findall(re.escape(PARK_URL) + r'([^<]+)', sdf)))
    for rel in used:
        src = os.path.join(park_dir, rel)
        out = rel
        if rel.startswith('materials/textures/') and 'Heightmap' not in rel and \
                'flat_normal' not in rel:
            out = os.path.splitext(rel)[0] + '.jpg'
            img = Image.open(src).convert('RGB')
            img.thumbnail((TEXTURE_SIZE, TEXTURE_SIZE))
            img.save(os.path.join(dst, out), quality=85)
        else:
            shutil.copy(src, os.path.join(dst, rel))
        sdf = sdf.replace(PARK_URL + rel, out)
    open(os.path.join(dst, 'model.sdf'), 'w').write(sdf)
    shutil.copy(os.path.join(src_model, 'model.config'), dst)


def lake(dst):
    os.makedirs(dst)
    open(os.path.join(dst, 'model.config'), 'w').write('''<?xml version="1.0"?>
<model>
  <name>lake_water</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Intelligent Robotics Lab</name></author>
  <description>Flat water surface at z = 0 (no waves, no collision).</description>
</model>
''')
    open(os.path.join(dst, 'model.sdf'), 'w').write('''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="lake_water">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane><normal>0 0 1</normal><size>1800 3500</size></plane>
        </geometry>
        <material>
          <ambient>0.1 0.25 0.35 1</ambient>
          <diffuse>0.1 0.3 0.45 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
''')


def world(src, dst):
    sdf = open(src).read()

    def keep(m):
        uri = re.search(r'<uri>\s*model://([^<\s]+)\s*</uri>', m.group(0)).group(1)
        return '' if uri in DROP else m.group(0)
    sdf = re.sub(r'\n?[ \t]*<include>.*?</include>', keep, sdf, flags=re.S)
    sdf = sdf.replace('''    <!-- Water surface at z = 0 -->''', '''    <!-- Water surface at z = 0 -->
    <include>
      <uri>model://lake_water</uri>
      <name>lake_water</name>
      <pose>0 0 0 0 0 0</pose>
    </include>''')
    # Comments of what is gone
    sdf = re.sub(r'\n[ \t]*<!-- (Ground station|Fire trucks|Trees on|Power plant|Pavement)'
                 r'[^\n]*-->', '', sdf)
    sdf = sdf.replace('disaster_nathan_benderson_park_world', 'nathan_benderson_town')
    open(dst, 'w').write(sdf)
    return sorted(set(re.findall(r'<uri>\s*model://([^<\s]+)\s*</uri>', sdf)))


if __name__ == '__main__':
    disaster, fuel_dir, park_dir, out = sys.argv[1:5]
    models = os.path.join(out, 'models')
    os.makedirs(os.path.join(out, 'worlds'), exist_ok=True)
    shutil.rmtree(models, ignore_errors=True)
    os.makedirs(models)
    used = world(os.path.join(disaster, WORLD), os.path.join(out, 'worlds',
                                                              'nathan_benderson_town.sdf'))
    for name in used:
        dst = os.path.join(models, name)
        if name == 'lake_water':
            lake(dst)
        elif name == 'nathan_benderson_park':
            park_model(os.path.join(disaster, 'models', name), park_dir, dst)
        elif name in FUEL:
            fuel_model(os.path.join(fuel_dir, FUEL[name]), name, dst)
        else:
            shutil.copytree(os.path.join(disaster, 'models', name), dst)
    print('models:', ' '.join(used))
