#!/usr/bin/env python3
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

"""A Gazebo world of any place: terrain, buildings and trees, from open data.

    geo_world.py --lat 40.28317 --lon -3.81923 --radius 500        (Spain)
    geo_world.py --lat 48.8562 --lon 2.2976 --radius 400           (anywhere else)

Spain (--region spain, the default there):
  terrain     IGN MDT (WCS, 5 m)             -> terrain.obj / terrain.stl, with the PNOA orthophoto
  buildings   Catastro INSPIRE (WFS)         -> prisms: footprint x (floors above ground x 3.2 m),
                                                textured facades, roofs from the orthophoto
Anywhere else (--region world): no LiDAR, and
  terrain     Copernicus GLO-30 (30 m)       -> smooth: streets are flat, with Esri imagery
  buildings   OpenStreetMap (Overpass)       -> prisms: footprint x height (the `height` tag, else
                                                `building:levels` x 3.2 m, else a guess by type)
Both:
  trees       crowns found in the orthophoto -> trunk + crown (or --dsm: heights from a surface model)

--no-buildings --no-trees --no-walls: the terrain alone (world:=terrain of the playground).

Everything is in the local East-North-Up frame of --lat/--lon (x east, y north, z = height above the
terrain at the origin), the frame of the Gazebo world and of any map made from it. Writes, in
<out-dir>/<name>/: world.sdf, navmap.stl (terrain + buildings + walls + trunks: what a navigation map
of the place is made from; navmap_ground.stl and navmap_holes.stl: the same split into the ground and
what stands on it), terrain/buildings/trees .obj + .stl, ortho.jpg, world.yaml (what the launch
reads). The last line printed is that directory.
"""

import argparse
import hashlib
import io
import json
import math
import os
import re
import sys
import time
import xml.etree.ElementTree as ET

try:
    import cv2
    import mapbox_earcut  # noqa: F401
    import trimesh  # noqa: F401  (tree_models writes the forest with it)
    import numpy as np
    import requests
    from PIL import Image
    from pyproj import CRS, Transformer
    from scipy import ndimage
except ImportError as e:
    sys.exit('geo_world: %s. Install: pip install --user --break-system-packages mapbox_earcut '
             'trimesh pyproj requests scipy opencv-python-headless pillow "numpy<2" (and laspy '
             'lazrs for --lidar). Outside Spain it also needs navmap_tools (a sourced workspace)' % e)

import tree_models  # noqa: E402  (next to this file)

FLOOR_HEIGHT = 3.2        # m per floor above ground (Catastro gives the number of floors only)
PARAPET = 0.5             # m of roof edge above the last floor
WMS_TILE = 2000           # px per orthophoto request
UA = {'User-Agent': 'easynav-playground-geo/0.1'}

URL_MDT = 'https://servicios.idee.es/wcs-inspire/mdt'
URL_BU = 'https://ovc.catastro.meh.es/INSPIRE/wfsBU.aspx'
URL_ORTHO = 'https://www.ign.es/wms-inspire/pnoa-ma'

NS = {
    'gml': 'http://www.opengis.net/gml/3.2',
    'bu': 'http://inspire.jrc.ec.europa.eu/schemas/bu-ext2d/2.0',
    'bc': 'http://inspire.jrc.ec.europa.eu/schemas/bu-core2d/2.0',
}


def log(msg):
    print('geo_world: ' + msg, flush=True)


def get(url, params, what, timeout=120, tries=4):
    last = None
    for k in range(tries):
        try:
            r = requests.get(url, params=params, headers=UA, timeout=timeout)
            if r.status_code == 200:
                return r
            last = 'HTTP %d' % r.status_code
        except requests.RequestException as e:
            last = str(e)
        time.sleep(2 * (k + 1))
    raise RuntimeError('%s: %s' % (what, last))


# ------------------------------------------------------------------------------------------- frames
class Frame:
    """ENU metres around (lat, lon) <-> the projected CRS the Spanish services are asked in."""

    def __init__(self, lat, lon, epsg=None):
        self.lat, self.lon = lat, lon
        # REGCAN95 UTM 28N / ETRS89 UTM 30N, or the UTM zone of the place (outside Spain)
        self.epsg = epsg or (4083 if lat < 30.0 else 25830)
        aeqd = CRS.from_proj4('+proj=aeqd +lat_0=%.9f +lon_0=%.9f +datum=WGS84 +units=m' % (lat, lon))
        proj = CRS.from_epsg(self.epsg)
        self._to_enu = Transformer.from_crs(proj, aeqd, always_xy=True)
        self._to_proj = Transformer.from_crs(aeqd, proj, always_xy=True)

    def enu(self, e, n):
        return self._to_enu.transform(e, n)

    def proj(self, x, y):
        return self._to_proj.transform(x, y)


# --------------------------------------------------------------------------------------------- data
def parse_asc(body):
    """(heights [rows from the north], x of the west edge, y of the north edge, cell) of an ArcGrid
    body (a WCS GetCoverage answer, possibly multipart). No-data cells are NaN."""
    tokens = body[body.index('ncols'):].split()
    head, pos = {}, 0
    while tokens[pos].lower() in ('ncols', 'nrows', 'xllcorner', 'xllcenter', 'yllcorner',
                                  'yllcenter', 'cellsize', 'nodata_value'):
        head[tokens[pos].lower()] = float(tokens[pos + 1])
        pos += 2
    nc, nr, cell = int(head['ncols']), int(head['nrows']), head['cellsize']
    x0 = head['xllcorner'] if 'xllcorner' in head else head['xllcenter'] - cell / 2
    y0 = head['yllcorner'] if 'yllcorner' in head else head['yllcenter'] - cell / 2
    z = np.array(tokens[pos:pos + nc * nr], dtype=np.float64).reshape(nr, nc)
    z[np.abs(z - head.get('nodata_value', -9999.0)) < 0.5] = np.nan
    return z.astype(np.float32), x0, y0 + nr * cell, cell


def fetch_mdt(frame, bbox, block=2000.0):
    """(array [rows, cols], x0, y0 of the top-left corner, pixel size) of the IGN terrain model.

    Asked as ArcGrid (millimetres): the service's GeoTIFF is int32, whole metres, and on a 5 m grid
    those 1 m steps are terraces and bumps on ground that is flat. Requested in blocks of `block` m
    on the 5 m grid (one GetCoverage each) and put together."""
    cov = 'Elevacion%d_5' % frame.epsg
    cell = 5.0
    x0 = math.floor(bbox[0] / cell) * cell
    y0 = math.floor(bbox[1] / cell) * cell
    x1 = math.ceil(bbox[2] / cell) * cell
    y1 = math.ceil(bbox[3] / cell) * cell
    nc, nr = int(round((x1 - x0) / cell)), int(round((y1 - y0) / cell))
    z = np.full((nr, nc), np.nan, np.float32)
    by = y0
    while by < y1:
        bx = x0
        while bx < x1:
            ex, ey = min(bx + block, x1), min(by + block, y1)
            r = get(URL_MDT, dict(service='WCS', version='2.0.1', request='GetCoverage',
                                  coverageId=cov, format='application/asc',
                                  subset=['x(%f,%f)' % (bx, ex), 'y(%f,%f)' % (by, ey)]), 'IGN MDT')
            bz, west, north, bcell = parse_asc(r.content.decode(errors='replace'))
            # its cells into ours (the service snaps the block to its own 5 m grid)
            c0 = int(round((west - x0) / cell))
            r0 = int(round((y1 - north) / cell))
            rr0, cc0 = max(r0, 0), max(c0, 0)
            rr1, cc1 = min(r0 + bz.shape[0], nr), min(c0 + bz.shape[1], nc)
            if rr1 > rr0 and cc1 > cc0:
                part = bz[rr0 - r0:rr1 - r0, cc0 - c0:cc1 - c0]
                dst = z[rr0:rr1, cc0:cc1]
                dst[~np.isnan(part)] = part[~np.isnan(part)]
            bx = ex
        by = ey
    z[z < -1000] = np.nan
    if np.isnan(z).all():
        raise RuntimeError('IGN MDT: no data in the area')
    if np.isnan(z).any():
        idx = ndimage.distance_transform_edt(np.isnan(z), return_distances=False, return_indices=True)
        z = z[tuple(idx)]
    return z, x0, y1, cell, cell


def make_dem_sampler(dem):
    z, x0, y0, sx, sy = dem

    def sample(x, y):
        c = (np.asarray(x) - x0) / sx - 0.5
        r = (y0 - np.asarray(y)) / sy - 0.5
        return ndimage.map_coordinates(z, [r, c], order=3, mode='nearest')
    return sample


def fetch_ortho(bbox, ppm, path, epsg=25830):
    """The PNOA orthophoto of bbox (projected metres) at `ppm` pixels per metre, as one JPEG."""
    w = int(math.ceil((bbox[2] - bbox[0]) * ppm))
    h = int(math.ceil((bbox[3] - bbox[1]) * ppm))
    img = np.zeros((h, w, 3), np.uint8)
    nx, ny = math.ceil(w / WMS_TILE), math.ceil(h / WMS_TILE)
    for j in range(ny):
        for i in range(nx):
            c0, c1 = i * WMS_TILE, min(w, (i + 1) * WMS_TILE)
            r0, r1 = j * WMS_TILE, min(h, (j + 1) * WMS_TILE)
            tb = (bbox[0] + c0 / ppm, bbox[3] - r1 / ppm, bbox[0] + c1 / ppm, bbox[3] - r0 / ppm)
            log('orthophoto tile %d/%d' % (j * nx + i + 1, nx * ny))
            r = get(URL_ORTHO, dict(service='WMS', version='1.3.0', request='GetMap',
                                    layers='OI.OrthoimageCoverage', styles='', crs='EPSG:%d' % epsg,
                                    bbox=','.join('%.3f' % v for v in tb), width=c1 - c0,
                                    height=r1 - r0, format='image/jpeg'), 'PNOA orthophoto')
            t = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
            if t is None:
                raise RuntimeError('PNOA orthophoto: the service did not return an image')
            img[r0:r1, c0:c1] = cv2.resize(t, (c1 - c0, r1 - r0))
    cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return img


# ---------------------------------------------------------------------------- outside Spain
def in_spain(lat, lon):
    """Whether (lat, lon) is where the Spanish services (IGN terrain, Catastro) answer: the
    peninsula (Galicia reaches further west than Portugal's border to the south), the Balearic and
    the Canary Islands, Ceuta and Melilla, by generous boxes."""
    return ((35.0 <= lat <= 44.0 and -7.5 <= lon <= 5.0) or
            (41.7 <= lat <= 44.0 and -9.4 <= lon <= -6.0) or
            (27.0 <= lat <= 29.5 and -19.0 <= lon <= -13.0))


def utm_epsg(lat, lon):
    """WGS84 / UTM zone of a place."""
    return (32600 if lat >= 0 else 32700) + int((lon + 180.0) // 6.0) + 1


def lonlat_box(frame, bbox, pad=0.0005):
    """(west, south, east, north) in degrees of a box in the frame's projected CRS."""
    to_ll = Transformer.from_crs(CRS.from_epsg(frame.epsg), CRS.from_epsg(4326), always_xy=True)
    lon, lat = to_ll.transform([bbox[0], bbox[2], bbox[0], bbox[2]], [bbox[1], bbox[1], bbox[3], bbox[3]])
    return min(lon) - pad, min(lat) - pad, max(lon) + pad, max(lat) + pad


def fetch_copernicus(frame, bbox):
    """sample(x, y) in projected metres -> metres above sea level, from Copernicus GLO-30 (30 m)."""
    from navmap_tools.geo.cache import default_cache_dir, DiskCache
    from navmap_tools.geo.dem import load_dem_grid
    from navmap_tools.geo.projection import BBox
    try:
        dem = load_dem_grid(BBox(*lonlat_box(frame, bbox)), DiskCache(default_cache_dir()))
    except RuntimeError as e:
        if 'no DEM coverage' not in str(e):
            raise
        # Copernicus has no tile over the open sea: the land is flat 0 m and the sea floor
        # (EMODnet, add_sea) is what the area has
        log('terrain: no Copernicus tile here (open sea): flat 0 m, the sea floor comes from EMODnet')

        def flat(x, y):
            return np.zeros(np.shape(np.asarray(x, dtype=np.float64)))
        return flat
    to_ll = Transformer.from_crs(CRS.from_epsg(frame.epsg), CRS.from_epsg(4326), always_xy=True)

    def sample(x, y):
        lon, lat = to_ll.transform(np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64))
        return dem.sample_array(lon, lat)
    return sample


def add_sea(dem_at, frame, bbox, mode):
    """(sample(x, y), has_sea): the land sampler with the floor of EMODnet under the sea."""
    from navmap_tools.geo.cache import default_cache_dir, DiskCache
    from navmap_tools.geo.projection import BBox
    try:
        from navmap_tools.geo.sea import with_sea
    except ImportError:
        log('sea: this navmap_tools has no sea floor (navmap_tools.geo.sea): the sea stays flat')
        return dem_at, False
    to_ll = Transformer.from_crs(CRS.from_epsg(frame.epsg), CRS.from_epsg(4326), always_xy=True)
    to_proj = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(frame.epsg), always_xy=True)

    class Land:
        """The land sampler as navmap_tools' sea.py wants it: lon, lat -> metres."""
        pixel_size_lat = 5.0 / 111320.0

        def sample_array(self, lon, lat, missing=None):
            x, y = to_proj.transform(np.asarray(lon, dtype=np.float64), np.asarray(lat, dtype=np.float64))
            return np.asarray(dem_at(x, y), dtype=np.float64)

    sea, has_sea = with_sea(Land(), BBox(*lonlat_box(frame, bbox)), DiskCache(default_cache_dir()), mode,
                            log=lambda m: log(m.replace('[navmap_gis_tool] ', '')))
    if not has_sea:
        return dem_at, False

    def sample(x, y):
        lon, lat = to_ll.transform(np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64))
        return sea.sample_array(lon, lat)
    return sample, True


DEEP_FLOOR = 150.0       # m: the depth where navmap_tools' colours stop changing
DEEP_LIGHT = np.array([52.0, 120.0, 168.0])      # the shallowest part of a deep area
DEEP_DARK = np.array([10.0, 30.0, 70.0])         # the deepest


def shade_seabed(img, rel, sea_z, bbox, ppm, to_enu, path):
    """
    The orthophoto (BGR) with the sea floor coloured by depth, written to `path` again.

    The imagery of the sea is a flat blue that hides the relief: sand where it is shallow, deep
    blue where it is deep, as navmap_tools' seabed_colours says. `rel(x, y)` is the terrain height
    in the area's frame (ENU metres), `to_enu(ex, ny)` takes the projected metres of the image.
    """
    from navmap_tools.geo.sea import seabed_colours
    H, W = img.shape[:2]
    step = 4                                          # the depth is smooth: every 4th pixel
    cols = np.arange(0, W + step, step).clip(0, W - 1)
    rows = np.arange(0, H + step, step).clip(0, H - 1)
    ex = bbox[0] + (cols + 0.5) / ppm
    ny = bbox[3] - (rows + 0.5) / ppm
    gx, gy = to_enu(*np.meshgrid(ex, ny))
    depth = np.maximum(sea_z - np.asarray(rel(gx, gy), dtype=np.float64), 0.0)
    rgb = seabed_colours(depth)
    if depth.min() >= DEEP_FLOOR:
        # beyond the last depth of the ramp every pixel would be the same dark blue: the relief of
        # the area itself (shallower to deeper) is what is left to show
        t = ((depth - depth.min()) / max(float(depth.max() - depth.min()), 1.0))[..., None]
        rgb = DEEP_LIGHT * (1.0 - t) + DEEP_DARK * t
    colour = cv2.resize(rgb[..., ::-1].astype(np.float32), (W, H),
                        interpolation=cv2.INTER_LINEAR)           # RGB -> BGR
    under = cv2.resize((depth > 0.3).astype(np.float32), (W, H),
                       interpolation=cv2.INTER_LINEAR)[..., None]
    out = img.astype(np.float32) * (1.0 - under) + colour * under
    out = np.clip(np.rint(out), 0, 255).astype(np.uint8)
    cv2.imwrite(path, out, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return out


def fetch_ortho_world(frame, bbox, ppm, path):
    """The Esri World Imagery of bbox (projected metres) at `ppm` pixels per metre, as one JPEG."""
    from navmap_tools.geo.cache import default_cache_dir, DiskCache
    from navmap_tools.geo.imagery import load_imagery_mosaic
    from navmap_tools.geo.projection import BBox
    mosaic = load_imagery_mosaic(
        BBox(*lonlat_box(frame, bbox)), DiskCache(default_cache_dir()), zoom=19)
    log('orthophoto: Esri World Imagery, zoom %d' % mosaic.zoom)
    w = int(math.ceil((bbox[2] - bbox[0]) * ppm))
    h = int(math.ceil((bbox[3] - bbox[1]) * ppm))
    to_ll = Transformer.from_crs(CRS.from_epsg(frame.epsg), CRS.from_epsg(4326), always_xy=True)
    xs = bbox[0] + (np.arange(w) + 0.5) / ppm
    img = np.zeros((h, w, 3), np.uint8)
    for r0 in range(0, h, 200):
        r1 = min(h, r0 + 200)
        ys = bbox[3] - (np.arange(r0, r1) + 0.5) / ppm
        gx, gy = np.meshgrid(xs, ys)
        lon, lat = to_ll.transform(gx, gy)
        rgb = mosaic.sample_array(lon, lat)
        img[r0:r1] = np.clip(np.rint(rgb), 0, 255).astype(np.uint8)[..., ::-1]   # BGR
    cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return img


INCOMPLETE = []        # what a source failed to give: the area is not cached as done

OVERPASS_CACHE = '~/.cache/easynav_geo/overpass'


def overpass(query, what, tries=6, timeout=120):
    """The parsed JSON an Overpass server answers to `query`, trying the servers in turn. An answer
    is kept on disk: the public servers are often busy, and a rerun must not ask again for what it
    already has."""
    cache = os.path.join(os.path.expanduser(OVERPASS_CACHE),
                         hashlib.md5(query.encode()).hexdigest() + '.json')
    if os.path.isfile(cache):
        try:
            with open(cache) as f:
                return json.load(f)
        except ValueError:   # cut short (a crash while it was written): fetched again
            os.remove(cache)
    last = None
    for k in range(tries):
        try:
            r = requests.post(OVERPASS[k % len(OVERPASS)], data={'data': query}, headers=UA,
                              timeout=timeout)
            if r.status_code == 200:
                data = r.json()
                os.makedirs(os.path.dirname(cache), exist_ok=True)
                with open(cache + '.tmp', 'w') as f:   # whole or absent: never a cut file
                    json.dump(data, f)
                os.replace(cache + '.tmp', cache)
                return data
            last = 'HTTP %d' % r.status_code
        except (requests.RequestException, ValueError) as ex:
            last = str(ex)
        log('%s: Overpass server %d/%d failed (%s)' % (what, k + 1, tries, last))
        if k + 1 < tries:
            time.sleep(3 * (k + 1))
    raise RuntimeError('%s: %s' % (what, last))


# building=* worth a number of floors when OpenStreetMap gives neither height nor levels
LEVELS_BY_TYPE = {'house': 2, 'detached': 2, 'semidetached_house': 2, 'terrace': 2, 'bungalow': 1,
                  'garage': 1, 'garages': 1, 'carport': 1, 'shed': 1, 'hut': 1, 'cabin': 1,
                  'roof': 1, 'kiosk': 1, 'greenhouse': 1, 'barn': 1, 'service': 1,
                  'apartments': 5, 'cathedral': 4, 'church': 3}
DEFAULT_LEVELS = 3


MAX_FLOORS = 60       # the tallest prism (192 m): a taller building keeps its footprint, not its height


def osm_floors(tags):
    """Floors of an OpenStreetMap building: from `height`, else `building:levels`, else by type,
    at most MAX_FLOORS (a taller one is cut: what matters to a robot is the footprint)."""
    m = re.match(r'\s*([0-9]+(?:\.[0-9]+)?)', tags.get('height', ''))
    if m and float(m.group(1)) >= 2.0:
        floors = max(1, int(round((float(m.group(1)) - PARAPET) / FLOOR_HEIGHT)))
        return min(floors, MAX_FLOORS)
    m = re.match(r'\s*([0-9]+(?:\.[0-9]+)?)', tags.get('building:levels', ''))
    if m and float(m.group(1)) >= 1.0:
        return min(max(1, int(round(float(m.group(1))))), MAX_FLOORS)
    return LEVELS_BY_TYPE.get(tags.get('building', ''), DEFAULT_LEVELS)


def join_rings(ways):
    """Closed rings (lists of (lon, lat)) from the pieces of a multipolygon, joined by their ends."""
    pieces = [list(w) for w in ways if len(w) >= 2]
    rings = []
    while pieces:
        ring = pieces.pop()
        while ring[0] != ring[-1]:
            for k, q in enumerate(pieces):
                if q[0] == ring[-1]:
                    ring += q[1:]
                elif q[-1] == ring[-1]:
                    ring += q[::-1][1:]
                else:
                    continue
                pieces.pop(k)
                break
            else:
                break            # an open piece: not a ring
        if ring[0] == ring[-1] and len(ring) >= 4:
            rings.append(ring[:-1])
    return rings


def fetch_buildings_osm(frame, bbox):
    """[(building id, floors, exterior ring, [hole rings])] in projected metres, from OpenStreetMap
    (the format of fetch_buildings)."""
    w_, s_, e_, n_ = lonlat_box(frame, bbox, pad=0.0)
    query = ('[out:json][timeout:90];(way["building"]["building"!="no"](%f,%f,%f,%f);'
             'relation["building"]["building"!="no"]["type"="multipolygon"](%f,%f,%f,%f););'
             'out geom tags;' % ((s_, w_, n_, e_) * 2))
    elements = overpass(query, 'OpenStreetMap buildings').get('elements', [])
    to_proj = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(frame.epsg), always_xy=True)

    def project(ring):
        x, y = to_proj.transform([p[0] for p in ring], [p[1] for p in ring])
        return np.c_[x, y]

    out = []
    for el in elements:
        tags = el.get('tags', {})
        bid = 'osm%s%d' % (el['type'][0], el['id'])
        floors = osm_floors(tags)
        if el['type'] == 'way':
            g = [(p['lon'], p['lat']) for p in el.get('geometry') or []]
            if len(g) >= 4 and g[0] == g[-1]:
                out.append((bid, floors, project(g[:-1]), []))
        else:
            def members(role):
                return [[(p['lon'], p['lat']) for p in m.get('geometry') or []]
                        for m in el.get('members', []) if m['type'] == 'way' and m.get('role') == role]
            outer, inner = join_rings(members('outer')), join_rings(members('inner'))
            for ring in outer:
                pr = project(ring)
                mine = [project(h) for h in inner
                        if len(h) and _inside(pr, project([h[0]])[0])]
                out.append((bid, floors, pr, mine))
    return out


def _inside(ring, p):
    """Whether point p is inside the polygon `ring` (ray casting)."""
    x, y = p
    xs, ys = ring[:, 0], ring[:, 1]
    x2, y2 = np.roll(xs, -1), np.roll(ys, -1)
    cross = ((ys > y) != (y2 > y)) & (x < (x2 - xs) * (y - ys) / np.where(y2 == ys, 1e-12, y2 - ys) + xs)
    return bool(cross.sum() % 2)


def buildings_of(region, frame, bbox):
    """The buildings of the area: the Catastro in Spain, OpenStreetMap elsewhere and wherever the
    Catastro has none (the Basque Country and Navarra keep their own cadastre, which it does not
    serve)."""
    if region != 'world':
        log('buildings: Catastro')
        parts = fetch_buildings(bbox, epsg=frame.epsg)
        if parts:
            return parts
        log('buildings: the Catastro has none here (the Basque Country and Navarra are not in it): '
            'OpenStreetMap')
    else:
        log('buildings: OpenStreetMap')
    return fetch_buildings_osm(frame, bbox)


def fetch_buildings(bbox, tile=800.0, epsg=25830):
    """[(building id, floors, exterior ring, [hole rings])] in projected metres. The Catastro
    service answers nothing for areas over about 1 km^2: the box is asked in tiles."""
    out, seen = [], set()
    nx = max(1, math.ceil((bbox[2] - bbox[0]) / tile))
    ny = max(1, math.ceil((bbox[3] - bbox[1]) / tile))
    for j in range(ny):
        for i in range(nx):
            tb = (bbox[0] + (bbox[2] - bbox[0]) * i / nx, bbox[1] + (bbox[3] - bbox[1]) * j / ny,
                  bbox[0] + (bbox[2] - bbox[0]) * (i + 1) / nx, bbox[1] + (bbox[3] - bbox[1]) * (j + 1) / ny)
            r = get(URL_BU, dict(service='wfs', version='2.0.0', request='GetFeature',
                                 typenames='BU:BuildingPart', srsname='EPSG::%d' % epsg,
                                 bbox='%f,%f,%f,%f,urn:ogc:def:crs:EPSG::%d' % (tb + (epsg,))),
                    'Catastro buildings')
            try:
                root = ET.fromstring(r.content)
            except ET.ParseError as e:    # an error page answered with status 200
                raise RuntimeError('Catastro buildings: not XML (%s): %r' % (e, r.text[:200]))
            if root.tag.endswith('ExceptionReport') and 'No records' in r.text:
                continue    # nothing in this tile (rural, or a territory the Catastro does not hold)
            if not root.tag.endswith('FeatureCollection'):
                raise RuntimeError('Catastro buildings: %s' % r.text[:300])
            for part in root.iter('{%s}BuildingPart' % NS['bu']):
                pid = part.get('{%s}id' % NS['gml'], '')
                if pid in seen:
                    continue
                seen.add(pid)
                fl = part.find('.//bu:numberOfFloorsAboveGround', NS)
                floors = int(fl.text) if fl is not None and fl.text and fl.text.strip().isdigit() else 0
                bid = re.sub(r'^ES\.SDGC\.BU\.', '', pid).split('_part')[0]
                for patch in part.iter('{%s}PolygonPatch' % NS['gml']):
                    def ring(el):
                        pl = el.find('.//gml:posList', NS)
                        v = np.array(pl.text.split(), dtype=np.float64).reshape(-1, 2)
                        return v[:-1] if len(v) > 1 and np.allclose(v[0], v[-1]) else v
                    ext = patch.find('gml:exterior', NS)
                    if ext is None:
                        continue
                    try:
                        out.append((bid, floors, ring(ext),
                                    [ring(h) for h in patch.findall('gml:interior', NS)]))
                    except (ValueError, AttributeError):   # a posList that is not pairs of numbers
                        log('Catastro building %s skipped: malformed polygon' % bid)
    return out


def read_text(path):
    with open(path) as f:
        return f.read()


# ----------------------------------------------------------------------------------------- survey
def read_stl(path):
    """Triangles [n, 3, 3] of a binary STL."""
    with open(path, 'rb') as f:
        raw = f.read()
    n = int(np.frombuffer(raw[80:84], '<u4')[0])
    rec = np.frombuffer(raw[84:84 + 50 * n], dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return rec['v'].astype(np.float64)


class Survey:
    """A survey mesh (photogrammetry of a pit, say) in the world's East-North frame, shifted by dz in
    height: its top surface as a grid, to lower the terrain below it and to put things on it."""

    def __init__(self, tris, cell=0.5):
        self.tris = tris
        self.cell = cell
        self.x0, self.y0 = tris[..., 0].min() - 1, tris[..., 1].min() - 1
        w = int((tris[..., 0].max() + 1 - self.x0) / cell) + 1
        h = int((tris[..., 1].max() + 1 - self.y0) / cell) + 1
        self.hf = np.full((h, w), -1e9, np.float32)
        px = ((tris[..., 0] - self.x0) / cell).astype(np.int32)
        py = ((tris[..., 1] - self.y0) / cell).astype(np.int32)
        for i in np.argsort(tris[..., 2].mean(1)):     # highest last: it wins
            cv2.fillConvexPoly(self.hf, np.c_[px[i], py[i]], float(tris[i, :, 2].max()))

    def z(self, x, y):
        """Top surface at (x, y), NaN where the survey has nothing."""
        c = np.round((np.asarray(x) - self.x0) / self.cell).astype(int)
        r = np.round((np.asarray(y) - self.y0) / self.cell).astype(int)
        ok = (c >= 0) & (c < self.hf.shape[1]) & (r >= 0) & (r < self.hf.shape[0])
        out = np.full(np.shape(x), np.nan)
        v = self.hf[np.clip(r, 0, self.hf.shape[0] - 1), np.clip(c, 0, self.hf.shape[1] - 1)]
        out[ok & (v > -1e8)] = v[ok & (v > -1e8)]
        return out


# ------------------------------------------------------------------------------------------ meshes
class Mesh:
    """Triangles with per-vertex UVs, split in named materials."""

    def __init__(self):
        self.parts = {}

    def add(self, material, v, uv, f):
        self.parts.setdefault(material, []).append((np.asarray(v, np.float64),
                                                    np.asarray(uv, np.float64),
                                                    np.asarray(f, np.int64)))

    def triangles(self):
        tris = []
        for items in self.parts.values():
            for v, _, f in items:
                tris.append(v[f])
        return np.concatenate(tris) if tris else np.zeros((0, 3, 3))

    def write_obj(self, path, mtl_name):
        with open(path, 'w') as o:
            o.write('mtllib %s\n' % mtl_name)
            base = 1
            for mat, items in self.parts.items():
                o.write('usemtl %s\n' % mat)
                for v, uv, f in items:
                    np.savetxt(o, v, fmt='v %.3f %.3f %.3f')
                    np.savetxt(o, uv, fmt='vt %.5f %.5f')
                    np.savetxt(o, vertex_normals(v, f), fmt='vn %.3f %.3f %.3f')
                    g = f + base
                    np.savetxt(o, np.c_[g[:, 0], g[:, 0], g[:, 0], g[:, 1], g[:, 1], g[:, 1],
                                        g[:, 2], g[:, 2], g[:, 2]], fmt='f %d/%d/%d %d/%d/%d %d/%d/%d')
                    base += len(v)


def vertex_normals(v, f):
    """Per-vertex normals: the mean of the faces around each vertex (up for a roof, the wall's own
    for a wall, smooth for the terrain). The renderer lights a mesh without normals white."""
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    out = np.zeros_like(v)
    for k in range(3):
        np.add.at(out, f[:, k], n)
    out /= np.maximum(np.linalg.norm(out, axis=1, keepdims=True), 1e-12)
    return out


def write_stl(path, tris):
    tris = np.asarray(tris, np.float32).reshape(-1, 3, 3)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'], rec['v'] = n, tris
    with open(path, 'wb') as f:
        f.write(b'geo_world'.ljust(80, b' '))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())


def write_mtl(path, entries):
    with open(path, 'w') as m:
        for name, (kd, tex) in entries.items():
            m.write('newmtl %s\nKa 1 1 1\nKd %.3f %.3f %.3f\nKs 0 0 0\nd 1\nillum 1\n' % ((name,) + tuple(kd)))
            if tex:
                m.write('map_Kd %s\n' % tex)


def terrain_mesh(half, res, height, uv_of):
    n = int(round(2 * half / res)) + 1
    xs = np.linspace(-half, half, n)
    X, Y = np.meshgrid(xs, xs)
    Z = height(X, Y)
    v = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    idx = np.arange(n * n).reshape(n, n)
    a, b, c, d = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, 1:].ravel(), idx[1:, :-1].ravel()
    f = np.concatenate([np.stack([a, b, c], 1), np.stack([a, c, d], 1)])
    return v, uv_of(v[:, 0], v[:, 1]), f


def signed_area(r):
    x, y = r[:, 0], r[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def extrude(ext, holes, z0, z1, uv_roof, facade_w=6.0, floors=1):
    """Roof (earcut) and walls of one footprint. Returns (roof v, uv, f), (wall v, uv, f)."""
    import mapbox_earcut as earcut
    if signed_area(ext) < 0:
        ext = ext[::-1]
    holes = [h[::-1] if signed_area(h) > 0 else h for h in holes]
    rings = [ext] + holes
    pts = np.concatenate(rings)
    ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
    tri = earcut.triangulate_float64(pts, ends).reshape(-1, 3).astype(np.int64)
    if len(tri) == 0:
        return None
    # roofs face up whatever the order earcut returned
    cr = np.cross(np.c_[pts[tri[:, 1]] - pts[tri[:, 0]], np.zeros(len(tri))],
                  np.c_[pts[tri[:, 2]] - pts[tri[:, 0]], np.zeros(len(tri))])[:, 2]
    tri[cr < 0] = tri[cr < 0][:, ::-1]
    rv = np.c_[pts, np.full(len(pts), z1)]
    roof = (rv, uv_roof(pts[:, 0], pts[:, 1]), tri)
    wv, wuv, wf = [], [], []
    for r in rings:
        m = len(r)
        for i in range(m):
            p, q = r[i], r[(i + 1) % m]
            L = float(np.hypot(*(q - p)))
            if L < 0.05:
                continue
            k = len(wv)
            wv += [(p[0], p[1], z0), (q[0], q[1], z0), (q[0], q[1], z1), (p[0], p[1], z1)]
            wuv += [(0, 0), (L / facade_w, 0), (L / facade_w, floors), (0, floors)]
            wf += [(k, k + 1, k + 2), (k, k + 2, k + 3)]
    return roof, (np.array(wv), np.array(wuv), np.array(wf))


def facade_texture(path, wall, trim, window):
    """One floor of facade, 6 m x 3.2 m: wall, two windows with frame, a trim line at the bottom."""
    h, w = 160, 300
    rng = np.random.default_rng(7)
    img = np.full((h, w, 3), wall, np.float32) + rng.normal(0, 4, (h, w, 1))
    for cx in (w * 0.25, w * 0.75):
        x0, x1, y0, y1 = int(cx - 42), int(cx + 42), 28, 112
        cv2.rectangle(img, (x0 - 5, y0 - 5), (x1 + 5, y1 + 5), trim, -1)
        cv2.rectangle(img, (x0, y0), (x1, y1), window, -1)
        cv2.line(img, ((x0 + x1) // 2, y0), ((x0 + x1) // 2, y1), trim, 3)
        cv2.line(img, (x0, (y0 + y1) // 2), (x1, (y0 + y1) // 2), trim, 3)
    cv2.rectangle(img, (0, h - 6), (w, h), trim, -1)
    cv2.imwrite(path, np.clip(img[..., ::-1], 0, 255).astype(np.uint8))


FACADES = [  # RGB: wall, trim, window
    ('facade_beige', (214, 198, 168), (170, 155, 130), (60, 72, 84)),
    ('facade_white', (226, 224, 218), (150, 150, 150), (50, 62, 76)),
    ('facade_brick', (170, 92, 70), (205, 190, 170), (58, 62, 70)),
    ('facade_grey', (150, 156, 160), (110, 115, 120), (40, 52, 66)),
]


# ---------------------------------------------------------------------------------------- trees
def detect_trees(img, ppm, mask_blocked):
    """(col, row, radius m) of the tree crowns in the orthophoto: dark green round blobs. Lawns are
    as green as crowns, so the colour alone is not enough: a crown is darker than its surroundings
    at the scale of a crown (difference of Gaussians), and green."""
    f = img.astype(np.float32)
    b, g, r = f[..., 0], f[..., 1], f[..., 2]
    v = f.max(2)
    exg = cv2.GaussianBlur(2 * g - r - b, (0, 0), 3)
    s1 = 1.5 * ppm
    dark = cv2.GaussianBlur(v, (0, 0), 2.2 * s1) - cv2.GaussianBlur(v, (0, 0), s1)
    cand = (dark > 6) & (exg > 5) & (cv2.GaussianBlur(v, (0, 0), s1) < 100) & ~mask_blocked
    k = max(3, int(round(1.75 * ppm))) | 1
    cand = cv2.morphologyEx(cand.astype(np.uint8), cv2.MORPH_OPEN, np.ones((k, k), np.uint8))
    n, _, st, cen = cv2.connectedComponentsWithStats(cand)
    out = []
    for i in range(1, n):
        area = st[i, cv2.CC_STAT_AREA] / ppm ** 2
        if 3.0 < area < 150.0:
            out.append((cen[i][0], cen[i][1], min(5.0, max(1.3, math.sqrt(area / math.pi)))))
    return out


# ------------------------------------------------------------------------------------------ lidar
LIDAR_DIR = '~/.ros/easynav_geo/lidar'
GROUND_SMOOTH = 2.0     # cells (m) of Gaussian blur of the LiDAR ground
CNIG_LIDAR = 'https://centrodedescargas.cnig.es/CentroDescargas/lidar-ign-pnoa'


def find_lidar(paths, bbox):
    """The .laz/.las files (paths: files or directories) whose extent meets bbox (projected metres)."""
    import laspy
    files = []
    for p in paths:
        p = os.path.expanduser(p)
        if os.path.isdir(p):
            files += [os.path.join(p, f) for f in sorted(os.listdir(p)) if f.lower().endswith(('.laz', '.las'))]
        elif os.path.isfile(p):
            files.append(p)
    keep = []
    for f in files:
        with laspy.open(f) as r:
            h = r.header
            if h.mins[0] <= bbox[2] and h.maxs[0] >= bbox[0] and h.mins[1] <= bbox[3] and h.maxs[1] >= bbox[1]:
                keep.append(f)
    return keep


def find_lidar_names(paths):
    names = []
    for p in paths:
        p = os.path.expanduser(p)
        if os.path.isdir(p):
            names += [os.path.join(p, f) for f in os.listdir(p)]
        elif os.path.isfile(p):
            names.append(p)
    return names


def lidar_tiles(bbox, have=()):
    """The PNOA-LiDAR sheets that cover bbox, as (x, y, present): X-Y are the UTM km of the sheet's
    upper-left corner (west edge, north edge), as in the file names. Sheets are 1 x 1 km (Madrid, 2016 and
    2024; some regions use 2 x 2 km). `present`: a file with that X-Y in its name is among `have`."""
    x0, x1 = math.floor(bbox[0] / 1000.0), math.ceil(bbox[2] / 1000.0)
    y0, y1 = math.floor(bbox[1] / 1000.0) + 1, math.ceil(bbox[3] / 1000.0)
    return [(x, y, any('%d-%d' % (x, y) in os.path.basename(f) for f in have))
            for x in range(x0, x1) for y in range(y0, y1 + 1)]


def lidar_hint(bbox, have=()):
    t = lidar_tiles(bbox, have)
    return (('LiDAR sheets of this area' if all(ok for _, _, ok in t) else
             'no (or not all the) LiDAR for this area. Download by hand (the CNIG blocks scripted downloads) the '
             'classified sheets that cover it') + ' in %s: X-Y = %s\n'
            '  3rd coverage 2022-2025 (10 pts/m2): PNOA-<year>-<region>-X-Y-H<zone>-NPC02.LAZ at '
            'https://centrodedescargas.cnig.es/CentroDescargas/lidar-tercera-cobertura\n'
            '  2nd coverage 2015-2021 (1.5 pts/m2): PNOA-<year>-<region>-X-Y-ORT-CLA-RGB.LAZ at '
            'https://centrodedescargas.cnig.es/CentroDescargas/lidar-segunda-cobertura\n'
            '  one coverage is enough: the newest that exists' % (
                LIDAR_DIR, ', '.join('%d-%d%s' % (x, y, ' (have it)' if ok else '') for x, y, ok in t)))


def lidar_year(path):
    m = re.search(r'PNOA[-_](\d{4})', os.path.basename(path))
    return int(m.group(1)) if m else 0


def read_lidar(files, bbox):
    """x, y (projected), z, classification of every point inside bbox. Where flights of several years
    overlap, only the newest one is used."""
    import laspy
    xs, ys, zs, cs = [], [], [], []
    files = sorted(files, key=lidar_year, reverse=True)
    rects = []     # (year, x0, y0, x1, y1) of the sheets already read
    for f in files:
        log('lidar: reading %s' % os.path.basename(f))
        yr = lidar_year(f)
        with laspy.open(f) as r:
            older = [q for q in rects if q[0] > yr]
            rects.append((yr, r.header.mins[0], r.header.mins[1], r.header.maxs[0], r.header.maxs[1]))
            for pts in r.chunk_iterator(2_000_000):
                x, y = np.asarray(pts.x), np.asarray(pts.y)
                m = (x >= bbox[0]) & (x <= bbox[2]) & (y >= bbox[1]) & (y <= bbox[3])
                for _, qx0, qy0, qx1, qy1 in older:
                    m &= ~((x >= qx0) & (x <= qx1) & (y >= qy0) & (y <= qy1))
                if m.any():
                    xs.append(x[m])
                    ys.append(y[m])
                    zs.append(np.asarray(pts.z)[m])
                    cs.append(np.asarray(pts.classification)[m])
    if not xs:
        return None
    return (np.concatenate(xs), np.concatenate(ys), np.concatenate(zs), np.concatenate(cs))


class Lidar:
    """A classified point cloud in the local ENU frame as 1 m grids (row = north, column = east):
    the ground (DTM), the highest point of structures and of vegetation, and where there is data."""

    GROUND, VEG, NOISE = (2,), (3, 4, 5), (0, 7, 12, 18)

    def __init__(self, x, y, z, cls, extent, ground_asl, cell=1.0):
        self.cell, self.ext, self.ground = cell, extent, ground_asl
        n = self.n = int(math.ceil(2 * extent / cell))
        ix = np.floor((x + extent) / cell).astype(np.int64)
        iy = np.floor((y + extent) / cell).astype(np.int64)
        ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
        flat = (iy * n + ix)[ok]
        z, cls = z[ok] - ground_asl, cls[ok]
        self.cover = np.bincount(flat, minlength=n * n).reshape(n, n) > 0
        g = np.isin(cls, self.GROUND)
        if g.sum() < 0.02 * len(cls):
            raise ValueError('the point cloud has no ground class (2): use the classified files '
                             '(ORT-CLA)')
        cnt = np.bincount(flat[g], minlength=n * n).reshape(n, n).astype(np.float64)
        sm = np.bincount(flat[g], weights=z[g], minlength=n * n).reshape(n, n)
        # normalised convolution: smooth where there is ground, filling the holes (under buildings)
        w = (cnt > 0).astype(np.float64)
        num = cv2.GaussianBlur(np.where(cnt > 0, sm / np.maximum(cnt, 1), 0.0), (0, 0), 2.0)
        den = cv2.GaussianBlur(w, (0, 0), 2.0)
        dtm = np.where(den > 0.05, num / np.maximum(den, 1e-9), np.nan)
        hole = np.isnan(dtm)
        if hole.any() and (~hole).any():
            d, idx = ndimage.distance_transform_edt(hole, return_indices=True)
            far = d > 40
            dtm = dtm[tuple(idx)]
            dtm[far & hole] = np.nan
        # The ground classification is noisy at 1 m (a few cm to a few dm: curbs, grass, misclassified
        # points) and the NavMap sees every triangle's slope: median filter and blur, or flat plazas fill
        # with 'steeper than 25 degrees' triangles.
        nanm = np.isnan(dtm)
        if (~nanm).any():
            if nanm.any():
                dtm = dtm[tuple(ndimage.distance_transform_edt(nanm, return_distances=False,
                                                              return_indices=True))]
            dtm = cv2.GaussianBlur(cv2.medianBlur(dtm.astype(np.float32), 5), (0, 0), GROUND_SMOOTH)
            dtm = np.where(nanm, np.nan, dtm)
        self.dtm = dtm.astype(np.float32)
        self.valid = ~np.isnan(self.dtm) & ndimage.binary_dilation(self.cover, iterations=3)
        self.struct = self._top(flat, z, ~g & ~np.isin(cls, self.VEG) & ~np.isin(cls, self.NOISE) & (cls < 32))
        self.veg = self._top(flat, z, np.isin(cls, self.VEG))

    def _top(self, flat, z, m):
        n = self.n
        top = np.full(n * n, -np.inf)
        np.maximum.at(top, flat[m], z[m])
        top = top.reshape(n, n)
        return np.where(np.isfinite(top), top, np.nan).astype(np.float32)

    def _rc(self, x, y):
        return (np.asarray(y) + self.ext) / self.cell - 0.5, (np.asarray(x) + self.ext) / self.cell - 0.5

    def ground_at(self, x, y):
        """DTM height (above the origin's ground) and whether the lidar has it."""
        r, c = self._rc(x, y)
        d = np.nan_to_num(self.dtm, nan=0.0)
        z = ndimage.map_coordinates(d, [r, c], order=1, mode='nearest')
        ok = ndimage.map_coordinates(self.valid.astype(np.float32), [r, c], order=1, mode='nearest') > 0.99
        return z, ok

    def ndsm(self, grid):
        return np.where(np.isnan(grid), 0.0, grid - np.nan_to_num(self.dtm, nan=0.0)).astype(np.float32)

    def values(self, grid, ring):
        """The cells of `grid` inside the polygon `ring` (ENU), as a 1-D array."""
        pts = np.c_[(ring[:, 0] + self.ext) / self.cell, (ring[:, 1] + self.ext) / self.cell]
        c0, r0 = np.maximum(np.floor(pts.min(0)).astype(int) - 1, 0)
        c1, r1 = np.minimum(np.ceil(pts.max(0)).astype(int) + 2, self.n)
        if c1 <= c0 or r1 <= r0:
            return np.zeros(0)
        m = np.zeros((r1 - r0, c1 - c0), np.uint8)
        cv2.fillPoly(m, [np.round(pts - [c0, r0]).astype(np.int32)], 1)
        return grid[r0:r1, c0:c1][m > 0]

    def paint(self, mask, ring):
        pts = np.c_[(ring[:, 0] + self.ext) / self.cell, (ring[:, 1] + self.ext) / self.cell]
        cv2.fillPoly(mask, [np.round(pts).astype(np.int32)], 1)

    def xy(self, col, row):
        return col * self.cell - self.ext, row * self.cell - self.ext


def lidar_trees(L, keep_out):
    """(x, y, crown radius, height) of the trees: peaks of the vegetation height model, the radius
    where it falls under half the peak."""
    chm = cv2.GaussianBlur(L.ndsm(L.veg), (0, 0), 1.0)
    pk = (ndimage.maximum_filter(chm, size=5) == chm) & (chm > 2.5) & ~keep_out
    out = []
    for r, c in zip(*np.nonzero(pk)):
        H = float(chm[r, c])
        rad = []
        for th in np.linspace(0, 2 * np.pi, 8, endpoint=False):
            k = 1
            while k < 9:
                rr, cc = int(round(r + k * math.sin(th))), int(round(c + k * math.cos(th)))
                if not (0 <= rr < L.n and 0 <= cc < L.n) or chm[rr, cc] < 0.5 * H:
                    break
                k += 1
            rad.append(k)
        x, y = L.xy(c + 0.5, r + 0.5)
        out.append((x, y, float(np.clip(np.mean(rad), 1.2, 7.0)), H))
    return out


def lidar_structures(L, keep_out, f=4):
    """Footprints [(ring, holes, height)] of what stands over the ground and is neither a mapped
    building nor vegetation: walls, sheds, tall fences. Cars and bushes are dropped. The grid is
    contoured at f times its resolution, so a wall one cell wide gets a polygon, not a line."""
    nd = L.ndsm(L.struct)
    m = ((nd >= 0.8) & (nd <= 40) & ~keep_out).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    mu = cv2.resize(m, None, fx=f, fy=f, interpolation=cv2.INTER_NEAREST)
    ndu = cv2.resize(nd, None, fx=f, fy=f, interpolation=cv2.INTER_NEAREST)
    cnts, hier = cv2.findContours(mu, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    if hier is None:
        return out

    def world(poly):
        poly = poly.reshape(-1, 2).astype(np.float64)
        return np.c_[L.xy((poly[:, 0] + 0.5) / f, (poly[:, 1] + 0.5) / f)]
    for i, c in enumerate(cnts):
        if hier[0][i][3] != -1:
            continue
        mask = np.zeros_like(mu)
        cv2.drawContours(mask, [c], -1, 1, -1)
        area = np.count_nonzero(mask) / f ** 2 * L.cell ** 2
        (_, _), (w, h), _ = cv2.minAreaRect(c)
        long_, short = max(w, h) / f * L.cell, max(min(w, h) / f * L.cell, 1.0)
        height = float(np.percentile(ndu[mask > 0], 75))
        elongated = long_ >= 4.0 and long_ >= 3.0 * short
        if area < 2.0 or not (height >= 2.2 or elongated):
            continue
        poly = cv2.approxPolyDP(c, 0.5 * f, True)
        if len(poly) < 3:
            continue
        holes = [world(cv2.approxPolyDP(hc, 0.5 * f, True)) for j, hc in enumerate(cnts)
                 if hier[0][j][3] == i and cv2.contourArea(hc) > 4 * f * f]
        out.append((world(poly), holes, height))
    return out


# --------------------------------------------------------------------------------------- walls (OSM)
BARRIERS = {   # barrier=*: width m, default height m, material
    'wall': (0.35, 1.8, 'concrete'), 'retaining_wall': (0.5, 1.5, 'concrete'),
    'city_wall': (1.2, 5.0, 'concrete'), 'fence': (0.15, 1.5, 'fence'),
    'hedge': (0.8, 1.2, 'hedge'), 'guard_rail': (0.25, 0.8, 'fence'),
}
OVERPASS = ('https://overpass-api.de/api/interpreter', 'https://overpass.private.coffee/api/interpreter',
            'https://overpass.kumi.systems/api/interpreter')


def fetch_barriers(lat_lon_bbox, to_enu):
    """[(polyline in ENU, width, height, material)] of the walls, fences and hedges of OSM."""
    s, w, n, e = lat_lon_bbox
    q = '[out:json][timeout:90];way["barrier"~"^(%s)$"](%f,%f,%f,%f);out geom tags;' % (
        '|'.join(BARRIERS), s, w, n, e)
    try:
        data = overpass(q, 'OpenStreetMap walls', tries=3, timeout=45)
    except RuntimeError as ex:
        log('walls: %s: skipped (the next run asks again)' % ex)
        INCOMPLETE.append('walls')
        return []
    out = []
    for el in data.get('elements', []):
        tags = el.get('tags', {})
        wd, h, mat = BARRIERS[tags['barrier']]
        m = re.match(r'\s*([0-9.]+)', tags.get('height', ''))
        if m:
            h = max(0.3, float(m.group(1)))
        g = el.get('geometry') or []
        if len(g) < 2:
            continue
        x, y = to_enu.transform([p['lon'] for p in g], [p['lat'] for p in g])
        out.append((np.c_[x, y], wd, h, mat))
    return out


def wall_boxes(line, width, height, ground):
    """A closed box per segment of a polyline, following the ground: (vertices, faces); every face has
    its own vertices (flat normals)."""
    vs, fs = [], []
    for p, q in zip(line[:-1], line[1:]):
        d = q - p
        L = float(np.hypot(*d))
        if L < 0.05:
            continue
        nrm = np.array([-d[1], d[0]]) / L * width / 2
        gp, gq = float(ground(np.array([p[0]]), np.array([p[1]]))[0]), float(
            ground(np.array([q[0]]), np.array([q[1]]))[0])
        c = [(p + nrm, gp), (p - nrm, gp), (q - nrm, gq), (q + nrm, gq)]
        b = [(*pt, g - 0.3) for pt, g in c]
        t = [(*pt, g + height) for pt, g in c]
        for quad in ([b[0], b[1], b[2], b[3]][::-1], t, [b[0], t[0], t[3], b[3]],
                     [b[2], b[3], t[3], t[2]], [b[1], b[2], t[2], t[1]], [b[0], b[1], t[1], t[0]]):
            k = len(vs)
            vs += quad
            fs += [(k, k + 1, k + 2), (k, k + 2, k + 3)]
    return np.array(vs), np.array(fs)


# -------------------------------------------------------------------------------------------- main
def sdf_world(template, lat, lon, elev, d, survey_model='', survey_dz=0.0, water=''):
    with open(template) as f:
        t = f.read()
    t = t.replace('@LATITUDE@', '%.8f' % lat).replace('@LONGITUDE@', '%.8f' % lon)
    t = t.replace('@ELEVATION@', '%.3f' % elev)
    head = t.split('<!-- Terrain generated')[0].rstrip()   # (the template's own @WATER@ is after it)
    surface = """<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>"""

    def model(name, visual, collision, extra=''):
        return """
        <model name="%s">
            <static>true</static>
            <link name="link">
                <collision name="collision"><geometry><mesh><uri>file://%s/%s</uri></mesh></geometry>%s</collision>
                <visual name="visual"><geometry><mesh><uri>file://%s/%s</uri></mesh></geometry>%s</visual>
            </link>
        </model>""" % (name, d, collision, surface, d, visual, extra)
    inc = ('\n        <include><uri>%s</uri><pose>0 0 %.3f 0 0 0</pose></include>' % (
        survey_model, survey_dz)) if survey_model else ''
    def has_triangles(stl):    # a binary STL without triangles (80 + 4 bytes) crashes the physics
        path = os.path.join(d, stl)
        return os.path.isfile(path) and os.path.getsize(path) > 84

    return head + '\n' + inc + model('area_terrain', 'terrain.obj', 'terrain.stl') + \
        (model('area_buildings', 'buildings.obj', 'buildings.stl') if has_triangles('buildings.stl') else '') + \
        (model('area_walls', 'walls.obj', 'walls.stl') if has_triangles('walls.stl') else '') + \
        (model('area_trees', 'trees.glb', 'trees.stl') if has_triangles('trees.stl') else '') + \
        water + '\n    </world>\n</sdf>\n'


def cache_key(a, region, lidar_id):
    """What the world is made of: a world on disk with another key is made again."""
    return hashlib.md5(json.dumps([a.lat, a.lon, a.radius, a.terrain_res, a.navmap_res, a.ortho_ppm,
                                  a.no_trees, a.no_walls, a.no_buildings, a.survey_model, a.survey_stl,
                                  a.survey_elevation, a.roof, lidar_id, a.max_trees, region, a.sea,
                                  a.sea_model, 11]).encode()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--lat', type=float, required=True)
    ap.add_argument('--lon', type=float, required=True)
    ap.add_argument('--radius', type=float, default=500.0, help='half-side of the square area, m')
    ap.add_argument('--region', choices=('auto', 'spain', 'world'), default='auto',
                    help='spain: IGN terrain, Catastro buildings, LiDAR; world: Copernicus terrain and '
                    'OpenStreetMap buildings (auto: by the coordinates)')
    ap.add_argument('--name', default='', help='directory name (default area_<lat>_<lon>_<radius>)')
    ap.add_argument('--out-dir', default='~/.ros/easynav_geo/worlds')
    ap.add_argument('--terrain-res', type=float, default=0.0,
                    help='visual terrain mesh spacing, m (0: 4, or 2.5 with LiDAR); the collision mesh is 4')
    ap.add_argument('--navmap-res', type=float, default=2.0, help='terrain spacing of the NavMap mesh, m')
    ap.add_argument('--lidar', nargs='*', default=None,
                    help='PNOA-LiDAR .laz/.las files or directories (default: %s)' % LIDAR_DIR)
    ap.add_argument('--no-lidar', action='store_true')
    ap.add_argument('--lidar-tiles', action='store_true',
                    help='only list the PNOA-LiDAR sheets to download for this area, and exit')
    ap.add_argument('--no-walls', action='store_true', help='no walls/fences/hedges from OpenStreetMap')
    ap.add_argument('--ortho-ppm', type=float, default=3.0, help='orthophoto pixels per metre')
    ap.add_argument('--no-trees', action='store_true')
    ap.add_argument('--no-buildings', action='store_true',
                    help='no buildings (with --no-trees and --no-walls: the terrain alone)')
    ap.add_argument('--sea', choices=('auto', 'on', 'off'), default='auto',
                    help='the sea floor from EMODnet (European seas) where the area has sea, and a world '
                         'with water: auto (when at least 3 %% of the area is sea), on, off')
    ap.add_argument('--sea-model', action='append', default=[], metavar='URI',
                    help='a model to put in the water (e.g. file:///.../my_model/model.sdf)')
    ap.add_argument('--max-trees', type=int, default=4000,
                    help='most trees planted (the biggest): the simulation draws them all')
    ap.add_argument('--tree-models', default='', help='directory with the oak/ and pine/ models (default: models/trees)')
    ap.add_argument('--roof', choices=('ortho', 'flat'), default='ortho',
                    help='ortho: roofs show the orthophoto; flat: a flat colour (its mean)')
    ap.add_argument('--template', default='', help='world template (worlds/geo.world.in)')
    ap.add_argument('--survey-model', default='', help='model URI of a survey (a mesh of the real site, e.g. model://my_site) '
                    'placed over the terrain (the terrain is lowered below it)')
    ap.add_argument('--survey-stl', default='', help="the survey's collision STL, in the world's frame")
    ap.add_argument('--survey-elevation', type=float, default=0.0,
                    help='m ASL of the survey\'s z = 0')
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()

    region = a.region if a.region != 'auto' else ('spain' if in_spain(a.lat, a.lon) else 'world')
    name = a.name or ('terrain' if a.no_buildings and a.no_trees and a.no_walls else 'area') + \
        '_%.5f_%.5f_%d' % (a.lat, a.lon, a.radius)
    out = os.path.join(os.path.expanduser(a.out_dir), name)
    lidar_paths = [] if (a.no_lidar or region == 'world') else (
        a.lidar if a.lidar is not None else [LIDAR_DIR])
    if region == 'world' and a.lidar:
        log('--lidar ignored: the PNOA-LiDAR covers Spain only')
    lidar_id = []
    for p_ in lidar_paths:
        p_ = os.path.expanduser(p_)
        for f_ in ([os.path.join(p_, x) for x in sorted(os.listdir(p_))] if os.path.isdir(p_) else [p_]):
            if os.path.isfile(f_) and f_.lower().endswith(('.laz', '.las')):
                lidar_id.append((f_, os.path.getsize(f_)))
    key = cache_key(a, region, lidar_id)
    yaml_path = os.path.join(out, 'world.yaml')
    if not a.force and not a.lidar_tiles and os.path.isfile(yaml_path) and ('key: ' + key) in read_text(yaml_path):
        log('%s is up to date' % out)
        print(out)
        return 0
    os.makedirs(out, exist_ok=True)

    here = os.path.dirname(os.path.abspath(__file__))
    template = a.template
    # --template, then the source tree (tools/../worlds), then the install: lib/easynav_playground_geo_worlds/
    # -> share/easynav_playground_geo_worlds in the default isolated layout, ../../../share in a merged one
    for c in (template, os.path.join(here, '..', 'worlds', 'geo.world.in'),
              os.path.join(here, '..', '..', 'share', 'easynav_playground_geo_worlds', 'worlds', 'geo.world.in'),
              os.path.join(here, '..', '..', '..', 'share', 'easynav_playground_geo_worlds', 'worlds',
                           'geo.world.in')):
        if c and os.path.isfile(c):
            template = c
            break
    else:
        raise SystemExit('geo_world: world template not found (--template)')

    frame = Frame(a.lat, a.lon, utm_epsg(a.lat, a.lon) if region == 'world' else None)
    rng = np.random.default_rng(3)
    half = a.radius
    margin = 40.0   # terrain, buildings and imagery a bit beyond the square
    ang = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * (half + margin)
    to_proj = Transformer.from_crs(
        CRS.from_proj4('+proj=aeqd +lat_0=%.9f +lon_0=%.9f +datum=WGS84 +units=m' % (a.lat, a.lon)),
        CRS.from_epsg(frame.epsg), always_xy=True)
    px, py = to_proj.transform(ang[:, 0], ang[:, 1])
    bbox = (min(px), min(py), max(px), max(py))
    aeqd = CRS.from_proj4('+proj=aeqd +lat_0=%.9f +lon_0=%.9f +datum=WGS84 +units=m' % (a.lat, a.lon))
    ll_to_enu = Transformer.from_crs(CRS.from_epsg(4326), aeqd, always_xy=True)
    enu_to_ll = Transformer.from_crs(aeqd, CRS.from_epsg(4326), always_xy=True)
    ll_back = Transformer.from_crs(CRS.from_epsg(frame.epsg), aeqd, always_xy=True)   # projected -> ENU
    log('area %.5f, %.5f, half-side %.0f m, EPSG:%d, %s' % (
        a.lat, a.lon, half, frame.epsg, 'Spain' if region == 'spain' else 'outside Spain'))
    if a.lidar_tiles:
        sq = [to_proj.transform(sx_ * half, sy_ * half) for sx_, sy_ in ((-1, -1), (1, 1))]
        have = find_lidar_names(lidar_paths)
        print(lidar_hint((min(q[0] for q in sq), min(q[1] for q in sq),
                          max(q[0] for q in sq), max(q[1] for q in sq)), have))
        return 0

    # --- terrain
    if region == 'world':
        log('terrain: Copernicus GLO-30 (30 m)')
        dem_at = fetch_copernicus(frame, bbox)
    else:
        log('terrain: IGN MDT')
        dem_at = make_dem_sampler(fetch_mdt(frame, bbox))

    has_sea = False
    if a.sea != 'off':
        dem_at, has_sea = add_sea(dem_at, frame, bbox, a.sea)

    def asl(x, y):   # ENU -> terrain height above sea level
        ex, ny = to_proj.transform(x, y)
        return dem_at(ex, ny)
    ground = float(asl(np.array([0.0]), np.array([0.0]))[0])
    if has_sea and ground < 0.0:
        ground = 0.0      # the origin is in the sea: z = 0 is the sea level, the floor is below it
    ground = round(ground, 3)
    sea_z = 0.0 - ground  # z of the sea surface (where the area has sea)
    rel = lambda x, y: asl(x, y) - ground    # noqa: E731

    survey = None
    survey_dz = 0.0
    if a.survey_stl:
        survey_dz = a.survey_elevation - ground          # the survey's z = 0 is this high above ours
        tris = read_stl(a.survey_stl)
        tris[..., 2] += survey_dz
        survey = Survey(tris)
        log('survey %s: %d triangles' % (os.path.basename(a.survey_stl), len(tris)))

    def surface(x, y):    # what the robot stands on: the survey where there is one, else the terrain
        z = rel(x, y)
        if survey is not None:
            sz = survey.z(x, y)
            z = np.where(np.isnan(sz), z, sz)
        return z

    # --- LiDAR (optional): finer terrain, real heights of buildings, trees and walls
    L = None
    files = find_lidar(lidar_paths, bbox) if lidar_paths else []
    if files:
        pts = read_lidar(files, bbox)
        if pts is not None:
            xe, yn = frame.enu(pts[0], pts[1])
            try:
                L = Lidar(xe, yn, pts[2], pts[3], half + margin, ground)
                log('lidar: %d points, %.0f%% of the area covered' % (len(pts[0]), 100 * L.cover.mean()))
            except ValueError as e:
                log('lidar ignored: %s' % e)
    if L is not None:
        mdt_rel = rel

        def rel(x, y):       # noqa: F811  the LiDAR ground where it has data, the MDT elsewhere
            z, ok = L.ground_at(x, y)
            return np.where(ok, z, mdt_rel(x, y))
    else:
        sq = [to_proj.transform(sx_ * half, sy_ * half) for sx_, sy_ in ((-1, -1), (1, 1))]
        if lidar_paths:
            log('lidar: ' + lidar_hint((min(q[0] for q in sq), min(q[1] for q in sq),
                                        max(q[0] for q in sq), max(q[1] for q in sq)),
                                       find_lidar_names(lidar_paths)).replace('\n', '\ngeo_world: '))

    # --- orthophoto
    ortho_path = os.path.join(out, 'ortho.jpg')
    if region == 'world':
        img = fetch_ortho_world(frame, bbox, a.ortho_ppm, ortho_path)
    else:
        img = fetch_ortho(bbox, a.ortho_ppm, ortho_path, frame.epsg)   # in the area's own CRS
    H, W = img.shape[:2]
    img_land = img      # the trees are looked for in the imagery as it is: the shaded sea is no forest
    if has_sea:
        img = shade_seabed(img, rel, sea_z, bbox, a.ortho_ppm, ll_back.transform, ortho_path)

    def pix_of(ex, ny):   # projected metres -> (col, row)
        return (ex - bbox[0]) * a.ortho_ppm, (bbox[3] - ny) * a.ortho_ppm

    def uv_of_enu(x, y):
        ex, ny = to_proj.transform(x, y)
        c, r = pix_of(ex, ny)
        return np.c_[c / W, 1.0 - r / H]

    def make_terrain(res):
        v, uv, f = terrain_mesh(half + margin * 0.5, res, rel, uv_of_enu)
        if survey is not None:   # the terrain goes under the survey, which is drawn over it
            sz = survey.z(v[:, 0], v[:, 1])
            ins = ~np.isnan(sz)
            v[ins, 2] = np.minimum(v[ins, 2], sz[ins] - 0.5)
        return v, uv, f

    vis_res = a.terrain_res or (2.5 if L is not None else 4.0)
    tv, tuv, tf = make_terrain(vis_res)
    tm = Mesh()
    tm.add('terrain', tv, tuv, tf)
    tm.write_obj(os.path.join(out, 'terrain.obj'), 'terrain.mtl')
    write_mtl(os.path.join(out, 'terrain.mtl'), {'terrain': ((1, 1, 1), 'ortho.jpg')})
    if vis_res == 4.0:
        coll = tm.triangles()
    else:       # collision: coarser than what is drawn, or Gazebo has too many triangles
        cv_, cuv, cf = make_terrain(4.0)
        coll = cv_[cf]
    write_stl(os.path.join(out, 'terrain.stl'), coll)
    nv, nuv, nf = make_terrain(a.navmap_res)
    nav_terrain = nv[nf]
    log('terrain: %d triangles (%.1f m), ground at the origin %.2f m ASL' % (len(tf), vis_res, ground))

    # --- buildings
    parts = [] if a.no_buildings else buildings_of(region, frame, bbox)
    if a.no_buildings:
        log('buildings: none (--no-buildings)')
    by_building = {}
    for bid, floors, ext, holes in parts:
        by_building.setdefault(bid, []).append((floors, ext, holes))
    bm = Mesh()
    blocked = np.zeros((H, W), np.uint8)
    footprints = []
    for i, (nm, wall, trim, win) in enumerate(FACADES):
        facade_texture(os.path.join(out, nm + '.png'), wall, trim, win)
    n_b = 0
    lidar_heights = 0
    for bid, plist in by_building.items():
        plist = [p for p in plist if p[0] > 0]
        if not plist:
            continue
        allx = np.concatenate([p[1] for p in plist])
        ex, ny = frame.enu(allx[:, 0], allx[:, 1])
        lim = half + margin
        if ex.max() < -lim or ex.min() > lim or ny.max() < -lim or ny.min() > lim:
            continue
        zb = float(np.min(rel(ex, ny))) - 0.3
        fac = FACADES[int(hashlib.md5(bid.encode()).hexdigest(), 16) % len(FACADES)][0]
        for floors, ext, holes in plist:
            e_, n_ = frame.enu(ext[:, 0], ext[:, 1])
            ring = np.c_[e_, n_]
            if abs(signed_area(ring)) < 4.0:
                continue
            if survey is not None and not np.isnan(
                    survey.z(np.array([ring[:, 0].mean()]), np.array([ring[:, 1].mean()]))[0]):
                continue   # the survey already has it
            hr = []
            for h in holes:
                he, hn = frame.enu(h[:, 0], h[:, 1])
                hr.append(np.c_[he, hn])
            z1 = zb + 0.3 + floors * FLOOR_HEIGHT + PARAPET
            if L is not None:     # the real roof height, where the LiDAR saw it
                vals = L.values(L.struct, ring)
                vals = vals[~np.isnan(vals)]
                if len(vals) >= max(3, 0.25 * abs(signed_area(ring))) and np.percentile(vals, 90) - zb > 2.0:
                    z1 = float(np.percentile(vals, 90)) + 0.05
                    lidar_heights += 1
            res = extrude(ring, hr, zb, z1, uv_of_enu, floors=floors)
            if res is None:
                continue
            (rv, ruv, rf), (wv, wuv, wf) = res
            if a.roof == 'flat':
                c, r = pix_of(*to_proj.transform(rv[:, 0], rv[:, 1]))
                m = np.zeros((H, W), np.uint8)
                cv2.fillPoly(m, [np.c_[c, r].astype(np.int32)], 1)
                col = cv2.mean(img, mask=m)[:3][::-1] if m.any() else (150, 150, 150)
                mat = 'roof_%02x%02x%02x' % tuple(int(v) & 0xFC for v in col)
                bm.add(mat, rv, ruv, rf)
                bm.roof_colors = getattr(bm, 'roof_colors', {})
                bm.roof_colors[mat] = tuple(v / 255.0 for v in col)
            else:
                bm.add('roof', rv, ruv, rf)
            bm.add(fac, wv, wuv, wf)
            for rr in [ring] + hr:
                c, r = pix_of(*to_proj.transform(rr[:, 0], rr[:, 1]))
                cv2.fillPoly(blocked, [np.c_[c, r].astype(np.int32)], 1)
            footprints.append((ring, hr))
        n_b += 1
    mtl = {nm: ((1, 1, 1), nm + '.png') for nm, *_ in FACADES}
    mtl['roof'] = ((1, 1, 1), 'ortho.jpg')
    for mat, col in getattr(bm, 'roof_colors', {}).items():
        mtl[mat] = (col, None)
    bm.write_obj(os.path.join(out, 'buildings.obj'), 'buildings.mtl')
    write_mtl(os.path.join(out, 'buildings.mtl'), mtl)
    btri = bm.triangles()
    write_stl(os.path.join(out, 'buildings.stl'), btri)
    log('buildings: %d (%d triangles, %d with LiDAR roof height)' % (n_b, len(btri), lidar_heights))

    # --- what the LiDAR grid must not turn into walls or trees: mapped buildings and the survey
    keep_out = None
    if L is not None:
        keep_out = np.zeros((L.n, L.n), np.uint8)
        for ring, hr in footprints:
            L.paint(keep_out, ring)
        keep_out = cv2.dilate(keep_out, np.ones((3, 3), np.uint8), iterations=2) > 0
        if survey is not None:
            gx, gy = np.meshgrid((np.arange(L.n) + 0.5) * L.cell - L.ext, (np.arange(L.n) + 0.5) * L.cell - L.ext)
            keep_out |= ~np.isnan(survey.z(gx, gy))

    # --- walls, fences, hedges (OpenStreetMap) and tall structures the LiDAR sees (walls, sheds)
    wall_mesh, wall_nav, n_walls = Mesh(), [], 0
    lim = half + margin

    def add_wall(v, f, mat, nav_v=None, nav_f=None):
        wall_mesh.add(mat, v, np.zeros((len(v), 2)), f)
        wall_nav.append((nav_v if nav_v is not None else v)[nav_f if nav_f is not None else f])

    if not a.no_walls:
        log('walls: OpenStreetMap')
        corners = enu_to_ll.transform([-lim, lim, lim, -lim], [-lim, -lim, lim, lim])
        for line, wd, ht, mat in fetch_barriers(
                (min(corners[1]), min(corners[0]), max(corners[1]), max(corners[0])), ll_to_enu):
            if (np.abs(line).max(axis=1) > lim).all():
                continue
            if survey is not None and not np.isnan(survey.z(line[:, 0], line[:, 1])).all():
                continue
            v, f = wall_boxes(line, wd, ht, surface)
            if len(v) == 0:
                continue
            nv_, nf_ = wall_boxes(line, max(wd, 1.0), ht, surface)   # the NavMap grid is 1 m
            add_wall(v, f, mat, nv_, nf_)
            n_walls += 1
            for q, r_ in zip(line[:-1], line[1:]):
                c0, r0 = pix_of(*to_proj.transform(q[0], q[1]))
                c1, r1 = pix_of(*to_proj.transform(r_[0], r_[1]))
                cv2.line(blocked, (int(c0), int(r0)), (int(c1), int(r1)), 1, int(2 * a.ortho_ppm))
    n_osm = n_walls
    if L is not None:
        for ring, hr, ht in lidar_structures(L, keep_out):
            zb = float(np.min(rel(ring[:, 0], ring[:, 1]))) - 0.3
            res = extrude(ring, hr, zb, zb + 0.3 + ht, lambda x, y: np.zeros((len(x), 2)))
            if res is None:
                continue
            (rv, ruv, rf), (wv, wuv, wf) = res
            add_wall(rv, rf, 'concrete')
            add_wall(wv, wf, 'concrete')
            n_walls += 1
            L.paint(keep_out.view(np.uint8), ring)
            c_, r_ = pix_of(*to_proj.transform(ring[:, 0], ring[:, 1]))
            cv2.fillPoly(blocked, [np.c_[c_, r_].astype(np.int32)], 1)
    if n_walls:
        wall_mesh.write_obj(os.path.join(out, 'walls.obj'), 'walls.mtl')
        write_mtl(os.path.join(out, 'walls.mtl'), {'concrete': ((0.62, 0.6, 0.57), None),
                                                  'fence': ((0.35, 0.37, 0.38), None),
                                                  'hedge': ((0.16, 0.34, 0.12), None)})
        write_stl(os.path.join(out, 'walls.stl'), wall_mesh.triangles())
    elif os.path.isfile(os.path.join(out, 'walls.stl')):
        os.remove(os.path.join(out, 'walls.stl'))
    log('walls and structures: %d (%d from OpenStreetMap, %d from the LiDAR)' % (n_walls, n_osm, n_walls - n_osm))

    # --- spawn: the nearest point to the origin that is not inside / next to a building or a wall
    sx, sy = 0.0, 0.0
    free = cv2.dilate(blocked, np.ones((int(3 * a.ortho_ppm) | 1,) * 2, np.uint8))
    for rr in np.arange(0, 60, 1.0):
        done = False
        for th in np.linspace(0, 2 * np.pi, max(1, int(rr * 8)), endpoint=False):
            x, y = rr * math.cos(th), rr * math.sin(th)
            c, r = pix_of(*to_proj.transform(x, y))
            if free[int(r), int(c)] == 0:
                sx, sy, done = x, y, True
                break
        if done:
            break
    spawn_z = float(surface(np.array([sx]), np.array([sy]))[0]) + 0.5

    # --- trees: crowns from the LiDAR where there is some, from the orthophoto elsewhere; planted
    # with the Gazebo Fuel oak and pine as ONE mesh (trees.glb) with ONE collision mesh (the trunks)
    trees = []     # (x, y, crown radius, height or None)
    trunk_blocks = np.zeros((0, 3, 3))
    if not a.no_trees:
        if L is not None:
            trees += lidar_trees(L, keep_out)
            log('trees: %d from the LiDAR' % len(trees))
        log('trees: crowns in the orthophoto')
        mask = cv2.dilate(blocked, np.ones((int(2 * a.ortho_ppm) | 1,) * 2, np.uint8)) > 0
        for c, r, rad in detect_trees(img_land, a.ortho_ppm, mask):
            x, y = ll_back.transform(bbox[0] + c / a.ortho_ppm, bbox[3] - r / a.ortho_ppm)
            if abs(x) > half or abs(y) > half:
                continue
            if survey is not None and not np.isnan(survey.z(np.array([x]), np.array([y]))[0]):
                continue
            if L is not None:
                rr_, cc_ = L._rc(x, y)
                if 0 <= int(rr_) < L.n and 0 <= int(cc_) < L.n and L.cover[int(rr_), int(cc_)]:
                    continue
            trees.append((x, y, rad, None))
        trees = [t for t in trees if abs(t[0]) <= half and abs(t[1]) <= half]
        if has_sea:    # the orthophoto's dark sea is not a forest
            trees = [t for t in trees if float(rel(np.array([t[0]]), np.array([t[1]]))[0]) >= sea_z - 0.3]
        wide = cv2.dilate(blocked, np.ones((int(1.5 * a.ortho_ppm) | 1,) * 2, np.uint8))

        def blocked_at(x, y):          # a trunk would touch a building, a wall or a structure
            c, r = pix_of(*to_proj.transform(x, y))
            if 0 <= int(r) < H and 0 <= int(c) < W and wide[int(r), int(c)]:
                return True
            return survey is not None and not np.isnan(survey.z(np.array([x]), np.array([y]))[0])
        n0 = len(trees)
        trees, why = tree_models.resolve(trees, blocked_at, [(sx, sy, 3.0)], rng)
        log('trees: %d found, %d kept (dropped: %s)' % (n0, len(trees), ', '.join(
            '%d %s' % (v, k) for k, v in why.items() if v) or 'none'))
        if len(trees) > a.max_trees:     # keep the biggest: Gazebo draws them all
            trees = sorted(trees, key=lambda t: -t[2])[:a.max_trees]
            log('trees: capped to the %d biggest (--max-trees)' % a.max_trees)
    for stale in ('trees.obj', 'trees.mtl', 'trees.glb', 'trees.stl'):
        if os.path.isfile(os.path.join(out, stale)):
            os.remove(os.path.join(out, stale))
    if trees:
        trunk_tris, trunk_blocks, ntri, info = tree_models.plant(
            trees, surface, tree_models.models_dir(here) if not a.tree_models else a.tree_models,
            os.path.join(out, 'trees.glb'), rng)
        write_stl(os.path.join(out, 'trees.stl'), trunk_tris)
        problems = tree_models.validate(info, blocked_at)
        if problems:
            raise SystemExit('geo_world: tree collisions: ' + '; '.join(problems))
        log('trees: %d planted as one mesh (%d triangles, %.0f MB) and one trunk collision mesh; '
            'no trunk overlaps another or a building/wall' % (
                len(trees), ntri, os.path.getsize(os.path.join(out, 'trees.glb')) / 1e6))

    # --- NavMap source (terrain + buildings + walls + trunks, no crowns: the robot drives under them)
    write_stl(os.path.join(out, 'navmap.stl'), np.concatenate(
        [nav_terrain, btri, trunk_blocks] + wall_nav + ([survey.tris] if survey is not None else [])))
    # the same split for a map with holes where things stand: the ground alone, and what
    # stands on it (navmap_gis_tool --dem-holes-mesh: the cells it touches have no ground)
    write_stl(os.path.join(out, 'navmap_ground.stl'), np.concatenate(
        [nav_terrain] + ([survey.tris] if survey is not None else [])))
    write_stl(os.path.join(out, 'navmap_holes.stl'), np.concatenate([btri, trunk_blocks] + wall_nav))

    water = ''
    underwater = False
    if has_sea:
        from navmap_tools.geo.dem_file import LocalDem
        from navmap_tools.geo.terrain_export import water_sdf
        step = 2.0
        n_w = int(round(2 * half / step)) + 1
        gx, gy = np.meshgrid((np.arange(n_w) - (n_w - 1) // 2) * step, (np.arange(n_w) - (n_w - 1) // 2) * step)
        floor = LocalDem(side=n_w, resolution=step, data_resolution=step, radius=half,
                         origin_lat=a.lat, origin_lon=a.lon, reference_altitude=ground,
                         requested_reference=ground, source=6, z=rel(gx, gy).astype(np.float32))
        water = water_sdf(floor, a.sea_model)
        underwater = bool(float(rel(np.array([0.0]), np.array([0.0]))[0]) < sea_z - 1.0)
        log('sea: the world has water at z = %.2f m%s' % (
            sea_z, ' (the origin is under it)' if underwater else ''))
    world = sdf_world(template, a.lat, a.lon, ground, out, a.survey_model, survey_dz, water)
    with open(os.path.join(out, 'world.sdf'), 'w') as f:
        f.write(world)
    with open(yaml_path, 'w') as f:
        f.write('# written by geo_world.py\nkey: %s\nlatitude: %.8f\nlongitude: %.8f\nradius: %.1f\n'
                'ground_elevation: %.3f\nspawn_x: %.2f\nspawn_y: %.2f\nspawn_z: %.3f\n'
                'buildings: %d\ntrees: %d\nwalls: %d\nlidar: %s\nsea: %s\nunderwater: %s\n' % (
                    ('incomplete-' + key) if INCOMPLETE else key, a.lat, a.lon, half, ground, sx, sy,
                    spawn_z, n_b, len(trees), n_walls, 'yes' if L is not None else 'no',
                    'true' if has_sea else 'false', 'true' if underwater else 'false'))
    if INCOMPLETE:
        log('WARNING: no %s (the source did not answer): the world is made without them and the '
            'next run asks again for what is missing' % ' or '.join(INCOMPLETE))
    log('ready: %s (spawn %.1f, %.1f)' % (out, sx, sy))
    print(out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
