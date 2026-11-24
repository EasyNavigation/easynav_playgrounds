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

"""The part of tools/geo_world.py that works outside Spain (OpenStreetMap buildings, UTM frame,
the choice of region), without network: Overpass is replaced by a canned answer."""

import importlib.util
import os
import sys
import types

import pytest

np = pytest.importorskip('numpy')
for _module in ('cv2', 'scipy', 'pyproj', 'requests', 'PIL'):
    pytest.importorskip(_module)

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.path.dirname(HERE), 'tools')


def needs_sea_floor():
    """Skip what needs the floor of the sea when navmap_tools has none (the NavMap of a PR
    without it)."""
    try:
        found = importlib.util.find_spec('navmap_tools.geo.sea') is not None
    except ModuleNotFoundError:
        found = False
    if not found:
        pytest.skip('this navmap_tools has no sea floor (navmap_tools.geo.sea)')


@pytest.fixture(scope='module')
def aw():
    sys.path.insert(0, TOOLS)
    spec = importlib.util.spec_from_file_location('geo_world_under_test', os.path.join(TOOLS, 'geo_world.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(autouse=True)
def private_overpass_cache(aw, monkeypatch, tmp_path):
    """No test reads or fills the user's cache of Overpass answers."""
    monkeypatch.setattr(aw, 'OVERPASS_CACHE', str(tmp_path / 'overpass'))


@pytest.mark.parametrize('lat, lon, expected', [
    (48.8562, 2.2976, 32631),       # Paris, zone 31 N
    (46.0207, 7.7491, 32632),       # Zermatt, zone 32 N
    (-33.4489, -70.6693, 32719),    # Santiago, zone 19 S
    (40.7128, -74.0060, 32618),     # New York, zone 18 N
])
def test_utm_zone_of_a_place(aw, lat, lon, expected):
    assert aw.utm_epsg(lat, lon) == expected


@pytest.mark.parametrize('lat, lon, spain', [
    (40.4168, -3.7038, True), (28.1235, -15.4363, True), (39.5696, 2.6502, True),
    (42.2406, -8.7207, True),                                  # Vigo
    (48.8562, 2.2976, False), (46.0207, 7.7491, False), (38.7223, -9.1393, False),
    (41.1579, -8.6291, False),                                 # Porto
])
def test_which_places_are_spain(aw, lat, lon, spain):
    assert aw.in_spain(lat, lon) is spain


@pytest.mark.parametrize('tags, floors', [
    ({'building': 'yes', 'height': '20'}, 6),                  # (20 - 0.5 parapet) / 3.2
    ({'building': 'yes', 'height': '20 m', 'building:levels': '2'}, 6),   # height wins
    ({'building': 'yes', 'building:levels': '4'}, 4),
    ({'building': 'yes', 'building:levels': '2.5'}, 2),
    ({'building': 'house'}, 2),
    ({'building': 'garage'}, 1),
    ({'building': 'yes'}, 3),                                  # a guess
    ({'building': 'yes', 'height': 'tall'}, 3),                # not a number
    ({'building': 'yes', 'height': '900'}, 60),                # capped
    ({'building': 'tower', 'tower:construction': 'lattice', 'height': '330'}, 60),  # cut, not left out
])
def test_floors_from_the_tags(aw, tags, floors):
    assert aw.osm_floors(tags) == floors


def test_pieces_of_a_multipolygon_are_joined_into_rings(aw):
    a, b, c, d = (0, 0), (1, 0), (1, 1), (0, 1)
    rings = aw.join_rings([[a, b, c], [c, d, a]])
    assert len(rings) == 1 and len(rings[0]) == 4
    # a piece given backwards joins too, and an open piece is no ring
    assert len(aw.join_rings([[a, b, c], [a, d, c]])) == 1
    assert aw.join_rings([[a, b, c]]) == []


def _square(lon, lat, size=0.0002):
    return [{'lon': lon, 'lat': lat}, {'lon': lon + size, 'lat': lat},
            {'lon': lon + size, 'lat': lat + size}, {'lon': lon, 'lat': lat + size},
            {'lon': lon, 'lat': lat}]


def test_buildings_from_openstreetmap_in_the_format_of_the_catastro(aw, monkeypatch):
    lon, lat = 2.2976, 48.8562
    answer = {'elements': [
        {'type': 'way', 'id': 1, 'tags': {'building': 'yes', 'building:levels': '5'},
         'geometry': _square(lon + 0.0001, lat + 0.0001)},
        {'type': 'way', 'id': 2, 'tags': {'building': 'tower', 'height': '330'},
         'geometry': _square(lon - 0.0003, lat)},       # very tall: kept, cut to 60 floors
        {'type': 'way', 'id': 3, 'tags': {'building': 'yes'},
         'geometry': _square(lon, lat)[:3]},            # not closed: skipped
        {'type': 'relation', 'id': 4, 'tags': {'building': 'yes', 'height': '16'}, 'members': [
            {'type': 'way', 'role': 'outer', 'geometry': _square(lon - 0.001, lat - 0.001, 0.0006)},
            {'type': 'way', 'role': 'inner', 'geometry': _square(lon - 0.0008, lat - 0.0008, 0.0002)},
            {'type': 'way', 'role': 'inner', 'geometry': _square(lon + 0.01, lat + 0.01, 0.0002)},
        ]},
    ]}
    asked = []
    monkeypatch.setattr(aw, 'overpass', lambda q, what, tries=6: asked.append(q) or answer)
    frame = aw.Frame(lat, lon, aw.utm_epsg(lat, lon))
    bbox = (452000.0, 5411000.0, 453000.0, 5412000.0)
    parts = aw.fetch_buildings_osm(frame, bbox)
    assert 'way["building"]' in asked[0] and 'relation["building"]' in asked[0]
    assert [p[0] for p in parts] == ['osmw1', 'osmw2', 'osmr4']
    assert [p[1] for p in parts] == [5, 60, 5]        # 5 levels; 330 m cut; 16 m -> (16 - 0.5) / 3.2
    way, rel = parts[0], parts[2]
    assert way[2].shape == (4, 2) and way[3] == []
    assert 13 < np.hypot(*(way[2][1] - way[2][0])) < 16           # a 0.0002 degree side, ~14.7 m
    assert 440000 < way[2][0][0] < 460000 and 5400000 < way[2][0][1] < 5420000   # UTM metres
    assert len(rel[3]) == 1                                       # only the hole inside the outer ring


def test_a_building_taller_than_the_prism_cap_is_still_a_prism(aw):
    assert aw.osm_floors({'building': 'office', 'height': '120'}) == 37


def test_overpass_tries_the_next_server_when_one_fails(aw, monkeypatch):
    calls = []

    class Reply:
        def __init__(self, code, body=None):
            self.status_code, self._body = code, body

        def json(self):
            return self._body

    def post(url, data=None, headers=None, timeout=None):
        calls.append(url)
        return Reply(502) if len(calls) < 3 else Reply(200, {'elements': []})

    monkeypatch.setattr(aw.requests, 'post', post)
    monkeypatch.setattr(aw.time, 'sleep', lambda s: None)
    assert aw.overpass('q', 'test') == {'elements': []}
    assert len(set(calls)) == 3 and calls[0] != calls[1]


def test_overpass_gives_up_with_the_reason(aw, monkeypatch):
    class Reply:
        status_code = 429

    monkeypatch.setattr(aw.requests, 'post', lambda *a, **k: Reply())
    monkeypatch.setattr(aw.time, 'sleep', lambda s: None)
    with pytest.raises(RuntimeError, match='buildings: HTTP 429'):
        aw.overpass('q', 'buildings', tries=2)


def test_an_overpass_answer_is_kept_and_not_asked_twice(aw, monkeypatch, tmp_path):
    monkeypatch.setattr(aw, 'OVERPASS_CACHE', str(tmp_path))
    calls = []

    class Reply:
        status_code = 200

        def json(self):
            return {'elements': [{'id': 1}]}

    monkeypatch.setattr(aw.requests, 'post', lambda *a, **k: calls.append(1) or Reply())
    assert aw.overpass('same query', 'x') == {'elements': [{'id': 1}]}
    assert aw.overpass('same query', 'x') == {'elements': [{'id': 1}]}
    assert len(calls) == 1
    aw.overpass('another query', 'x')
    assert len(calls) == 2


def test_walls_that_cannot_be_fetched_mark_the_area_incomplete(aw, monkeypatch, tmp_path):
    monkeypatch.setattr(aw, 'OVERPASS_CACHE', str(tmp_path))
    monkeypatch.setattr(aw, 'INCOMPLETE', [])

    def down(*args, **kwargs):
        raise aw.requests.ConnectionError('down')

    monkeypatch.setattr(aw.requests, 'post', down)
    monkeypatch.setattr(aw.time, 'sleep', lambda s: None)
    assert aw.fetch_barriers((48.85, 2.29, 48.86, 2.30), None) == []
    assert aw.INCOMPLETE == ['walls']


GML = ('<wfs:FeatureCollection xmlns:wfs="http://www.opengis.net/wfs/2.0" '
       'xmlns:gml="http://www.opengis.net/gml/3.2" '
       'xmlns:bu="http://inspire.jrc.ec.europa.eu/schemas/bu-ext2d/2.0">%s</wfs:FeatureCollection>')
PART = ('<bu:BuildingPart gml:id="ES.SDGC.BU.%s_part1"><bu:numberOfFloorsAboveGround>3'
        '</bu:numberOfFloorsAboveGround><gml:PolygonPatch><gml:exterior><gml:posList>%s'
        '</gml:posList></gml:exterior></gml:PolygonPatch></bu:BuildingPart>')
SQUARE = '0 0 10 0 10 10 0 10 0 0'


def _catastro(aw, monkeypatch, content):
    class Reply:
        status_code = 200
        text = content.decode('utf-8', 'replace') if isinstance(content, bytes) else content
    Reply.content = content if isinstance(content, bytes) else content.encode()
    monkeypatch.setattr(aw, 'get', lambda *a, **k: Reply())


def test_the_catastro_buildings_are_read(aw, monkeypatch):
    _catastro(aw, monkeypatch, GML % (PART % ('A1', SQUARE)))
    out = aw.fetch_buildings((0, 0, 100, 100))
    assert [(b[0], b[1], len(b[2])) for b in out] == [('A1', 3, 4)]


def test_an_error_page_with_status_200_is_a_clear_error_not_a_parser_traceback(aw, monkeypatch):
    _catastro(aw, monkeypatch, '<html><body>Servicio no disponible</body></html>')
    with pytest.raises(RuntimeError, match='Catastro buildings'):
        aw.fetch_buildings((0, 0, 100, 100))
    _catastro(aw, monkeypatch, 'not xml at all <<<')
    with pytest.raises(RuntimeError, match='Catastro buildings'):
        aw.fetch_buildings((0, 0, 100, 100))


def test_one_malformed_building_is_skipped_and_the_others_are_kept(aw, monkeypatch):
    _catastro(aw, monkeypatch, GML % (PART % ('BAD', '0 0 10') + PART % ('GOOD', SQUARE)))
    out = aw.fetch_buildings((0, 0, 100, 100))
    assert [b[0] for b in out] == ['GOOD']


def _tri_area(v, f):
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    return 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)


def _random_star_polygon(rng, n):
    """A simple (non self-intersecting) polygon: radii at sorted random angles."""
    ang = np.sort(rng.uniform(0, 2 * np.pi, n))
    ang += np.linspace(0, 1e-3, n)   # no two angles equal
    r = rng.uniform(3.0, 12.0, n)
    return np.c_[r * np.cos(ang), r * np.sin(ang)]


def test_extrude_roof_area_and_wall_area_match_the_footprint_for_random_polygons(aw):
    """Property: for any simple footprint the roof triangles cover its area and face up, and the walls
    are perimeter x height (seeded: a failure is reproducible)."""
    rng = np.random.default_rng(2024)
    for _ in range(60):
        ring = _random_star_polygon(rng, int(rng.integers(5, 25)))
        for flip in (False, True):          # either winding
            ext = ring[::-1] if flip else ring
            out = aw.extrude(ext, [], 0.0, 9.0, lambda x, y: np.c_[x, y])
            assert out is not None
            (rv, _, rf), (wv, _, wf) = out
            x, y = ring[:, 0], ring[:, 1]
            area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
            assert _tri_area(rv, rf).sum() == pytest.approx(area, rel=1e-6)
            normals = np.cross(rv[rf[:, 1]] - rv[rf[:, 0]], rv[rf[:, 2]] - rv[rf[:, 0]])
            assert (normals[:, 2] > 0).all()
            perimeter = np.hypot(*(np.roll(ring, -1, axis=0) - ring).T).sum()
            assert _tri_area(wv, wf).sum() == pytest.approx(perimeter * 9.0, rel=1e-6)


def test_extrude_subtracts_a_courtyard(aw):
    outer = np.array([[0, 0], [20, 0], [20, 20], [0, 20]], float)
    hole = np.array([[5, 5], [15, 5], [15, 15], [5, 15]], float)
    (rv, _, rf), (wv, _, wf) = aw.extrude(outer, [hole], 0.0, 3.0, lambda x, y: np.c_[x, y])
    assert _tri_area(rv, rf).sum() == pytest.approx(400 - 100)
    assert _tri_area(wv, wf).sum() == pytest.approx((80 + 40) * 3.0)   # outside and courtyard walls


# Two buildings of the Catastro (Madrid, metres about their centre) whose courtyard touches the
# outline at a vertex
TOUCHING = [
    ([[-19.05, -15.33], [-25.58, -14.86], [-22.88, 1.67], [-16.28, 0.56], [-15.71, 3.96],
      [-10.79, 3.14], [-9.51, 10.15], [2.97, 11.12], [2.72, 1.5], [13.08, 0.78], [12.11, 12.01],
      [12.08, 12.34], [21.06, 13.42], [25.79, 14.0], [25.8, 13.74], [28.23, -18.74], [2.77, -16.9],
      [3.62, -5.17], [0.38, -4.93], [-6.06, -4.29], [-6.9, -9.77], [-17.94, -8.34]],
     [[-6.06, -4.29], [-5.31, 1.09], [-11.06, 1.53], [-11.96, -3.84]]),
    ([[-10.05, -10.51], [-9.16, -4.63], [-20.45, -2.93], [-20.91, -2.07], [-17.48, 4.96],
      [-12.63, 2.55], [-13.28, 1.22], [-14.07, -0.4], [-9.37, -1.14], [-9.19, -0.78], [-6.51, 4.56],
      [1.91, 0.44], [7.93, 12.72], [10.5, 11.47], [10.37, 11.18], [14.27, 9.34], [16.91, 8.08],
      [16.87, 7.96], [15.41, 3.36], [15.36, 3.2], [13.32, 4.2], [12.03, 4.83], [11.33, 3.41],
      [9.59, -0.13], [13.66, -2.15], [11.87, -7.8], [-2.17, -5.69], [-3.08, -11.71], [-3.35, -11.67],
      [-2.44, -5.76], [-8.19, -4.88], [-9.09, -10.67], [-9.98, -10.52]],
     [[1.91, 0.44], [0.25, -2.94], [6.23, -3.71], [7.11, -2.01], [9.52, 2.97], [5.24, 5.07],
      [2.84, -0.0]]),
]


def _roof_covers_exactly(aw, outer, holes):
    """The roof of extrude(outer, holes) has the footprint's area, faces up, and every triangle is
    inside the outline and outside the courtyards (no overlap, no gap)."""
    (rv, _, rf), _ = aw.extrude(outer, holes, 0.0, 3.0, lambda x, y: np.c_[x, y])
    area = abs(aw.signed_area(outer)) - sum(abs(aw.signed_area(h)) for h in holes)
    assert _tri_area(rv, rf).sum() == pytest.approx(area, rel=1e-9)
    normals = np.cross(rv[rf[:, 1]] - rv[rf[:, 0]], rv[rf[:, 2]] - rv[rf[:, 0]])
    assert (normals[:, 2] > 0).all()
    for c in rv[rf].mean(axis=1)[:, :2]:
        assert aw._inside(outer, c) and not any(aw._inside(h, c) for h in holes)


@pytest.mark.parametrize('outer, hole', TOUCHING)
def test_extrude_of_a_courtyard_touching_the_outline(aw, outer, hole):
    _roof_covers_exactly(aw, np.array(outer), [np.array(hole)])


def test_extrude_with_several_courtyards(aw):
    """A block with a 3 x 3 grid of courtyards, one of them touching the outline at a corner."""
    outer = np.array([[0, 0], [40, 0], [40, 40], [0, 40]], float)
    holes = [np.array([[x, y], [x + 6, y], [x + 6, y + 6], [x, y + 6]], float)
             for x in (4, 17, 30) for y in (4, 17, 30)]
    holes[-1] = np.array([[34, 34], [40, 40], [30, 38]], float)   # its corner on the outline's
    _roof_covers_exactly(aw, outer, holes)


def test_extrude_of_a_degenerate_footprint_is_none_not_an_error(aw):
    line = np.array([[0, 0], [5, 0], [10, 0]], float)
    assert aw.extrude(line, [], 0.0, 3.0, lambda x, y: np.c_[x, y]) is None


def test_signed_area_is_positive_counter_clockwise(aw):
    sq = np.array([[0, 0], [2, 0], [2, 2], [0, 2]], float)
    assert aw.signed_area(sq) == pytest.approx(4.0)
    assert aw.signed_area(sq[::-1]) == pytest.approx(-4.0)


def test_point_in_polygon_agrees_with_opencv_for_random_points(aw):
    cv2 = pytest.importorskip('cv2')
    rng = np.random.default_rng(7)
    ring = _random_star_polygon(rng, 17)
    contour = (ring * 1000).astype(np.int32).reshape(-1, 1, 2)
    for _ in range(300):
        p = rng.uniform(-14, 14, 2)
        if abs(cv2.pointPolygonTest(contour, (float(p[0] * 1000), float(p[1] * 1000)), True)) < 5:
            continue   # on the edge: the two may differ by rounding
        assert aw._inside(ring, p) == (cv2.pointPolygonTest(
            contour, (float(p[0] * 1000), float(p[1] * 1000)), False) > 0)


NO_RECORDS = ('<?xml version="1.0"?><ExceptionReport xmlns="http://www.opengis.net/ows/1.1" '
              'version="2.0.0"><Exception exceptionCode="OperationProcessingFailed"><ExceptionText>'
              'No records founded for BBOX and SRS provided</ExceptionText></Exception>'
              '</ExceptionReport>')


def test_a_tile_without_records_is_empty_not_an_error(aw, monkeypatch):
    _catastro(aw, monkeypatch, NO_RECORDS)
    assert aw.fetch_buildings((0, 0, 100, 100)) == []


def test_where_the_catastro_has_nothing_the_buildings_come_from_openstreetmap(aw, monkeypatch):
    _catastro(aw, monkeypatch, NO_RECORDS)
    osm = [('osmw1', 3, np.array([[0, 0], [5, 0], [5, 5]], float), [])]
    monkeypatch.setattr(aw, 'fetch_buildings_osm', lambda frame, bbox: osm)
    assert aw.buildings_of('spain', types.SimpleNamespace(epsg=25830), (0, 0, 100, 100)) == osm


def test_where_the_catastro_has_buildings_openstreetmap_is_not_asked(aw, monkeypatch):
    _catastro(aw, monkeypatch, GML % (PART % ('A1', SQUARE)))

    def never(frame, bbox):
        raise AssertionError('OpenStreetMap asked')
    monkeypatch.setattr(aw, 'fetch_buildings_osm', never)
    assert [b[0] for b in aw.buildings_of('spain', types.SimpleNamespace(epsg=25830), (0, 0, 100, 100))] == ['A1']


def _flat_emodnet(depth):
    from navmap_tools.geo.dem import DemGrid
    px = 1.0 / 960.0
    return DemGrid(elevation=np.full((120, 120), depth, np.float32), west=-3.07, north=43.44,
                   pixel_size_lon=px, pixel_size_lat=px)


def _sea_frame_and_box(aw):
    frame = aw.Frame(43.385, -3.01)
    cx, cy = frame.proj(0.0, 0.0)
    return frame, (cx - 400, cy - 400, cx + 400, cy + 400)


def test_the_sea_floor_replaces_the_flat_zero_of_the_land_source(aw, monkeypatch):
    needs_sea_floor()
    pytest.importorskip('navmap_tools')
    from navmap_tools.geo import sea
    monkeypatch.setattr(sea, 'load_emodnet', lambda bbox, cache, force=False: _flat_emodnet(-9.0))
    frame, box = _sea_frame_and_box(aw)
    dem_at, has_sea = aw.add_sea(lambda x, y: np.zeros(np.shape(x)), frame, box, 'auto')
    assert has_sea
    cx, cy = frame.proj(0.0, 0.0)
    assert dem_at(np.array([cx]), np.array([cy]))[0] == pytest.approx(-9.0)


def test_the_land_is_kept_where_the_land_source_has_height(aw, monkeypatch):
    needs_sea_floor()
    pytest.importorskip('navmap_tools')
    from navmap_tools.geo import sea
    monkeypatch.setattr(sea, 'load_emodnet', lambda bbox, cache, force=False: _flat_emodnet(-9.0))
    frame, box = _sea_frame_and_box(aw)
    cx, cy = frame.proj(0.0, 0.0)

    def half_sea(x, y):       # land (30 m) east of the origin, sea west of it
        return np.where(np.asarray(x) > cx, 30.0, 0.0)
    dem_at, has_sea = aw.add_sea(half_sea, frame, box, 'auto')
    assert has_sea
    assert dem_at(np.array([cx + 100]), np.array([cy]))[0] == pytest.approx(30.0)
    assert dem_at(np.array([cx - 100]), np.array([cy]))[0] == pytest.approx(-9.0)


def test_open_sea_without_a_copernicus_tile_is_flat_land_not_an_error(aw, monkeypatch):
    pytest.importorskip('navmap_tools')
    import navmap_tools.geo.dem as dem_mod

    def no_coverage(bbox, cache, force=False):
        raise RuntimeError('no DEM coverage found for bbox')
    monkeypatch.setattr(dem_mod, 'load_dem_grid', no_coverage)
    frame = aw.Frame(43.4, -3.0, aw.utm_epsg(43.4, -3.0))
    sample = aw.fetch_copernicus(frame, (-100.0, -100.0, 100.0, 100.0))
    assert np.all(sample(np.array([0.0, 50.0]), np.array([0.0, -50.0])) == 0.0)

    def other_error(bbox, cache, force=False):
        raise RuntimeError('boom')
    monkeypatch.setattr(dem_mod, 'load_dem_grid', other_error)
    with pytest.raises(RuntimeError, match='boom'):
        aw.fetch_copernicus(frame, (-100.0, -100.0, 100.0, 100.0))


def test_a_deep_sea_floor_shows_its_own_relief_instead_of_one_flat_colour(aw, tmp_path):
    needs_sea_floor()
    pytest.importorskip('navmap_tools')
    img = np.full((100, 100, 3), 120, np.uint8)

    def rel(x, y):       # 3 650 m down, 30 m of slope from west (shallower) to east (deeper)
        return -3650.0 - 30.0 * (np.asarray(x, dtype=np.float64) + 50.0) / 100.0
    out = aw.shade_seabed(img, rel, 0.0, (-50.0, -50.0, 50.0, 50.0), 1.0, lambda x, y: (x, y),
                          str(tmp_path / 'ortho.jpg'))
    west, east = out[50, 5].astype(int), out[50, 95].astype(int)
    assert west.sum() > east.sum() + 60       # the shallower side is lighter, not the same blue
    assert east[0] > east[2] and west[0] > west[2]                # both are blue


def test_without_sea_or_with_it_off_the_sampler_is_the_land_one(aw, monkeypatch):
    needs_sea_floor()
    pytest.importorskip('navmap_tools')
    from navmap_tools.geo import sea
    frame, box = _sea_frame_and_box(aw)
    land = lambda x, y: np.full(np.shape(x), 12.0)    # noqa: E731
    monkeypatch.setattr(sea, 'load_emodnet', lambda bbox, cache, force=False: _flat_emodnet(5.0))
    assert aw.add_sea(land, frame, box, 'auto') == (land, False)       # EMODnet: all land
    assert aw.add_sea(land, frame, box, 'off') == (land, False)

    def down(bbox, cache, force=False):
        raise sea.NoCoverage('not reachable')
    monkeypatch.setattr(sea, 'load_emodnet', down)
    assert aw.add_sea(land, frame, box, 'auto') == (land, False)       # not Europe: flat sea


def test_the_water_goes_in_the_world_before_it_closes(aw, tmp_path):
    template = tmp_path / 't.in'
    template.write_text('<sdf><world>@LATITUDE@ @LONGITUDE@ @ELEVATION@\n<!-- Terrain generated -->'
                        '\n  @WATER@\n</world></sdf>')
    dry = aw.sdf_world(str(template), 43.0, -3.0, 0.0, str(tmp_path))
    wet = aw.sdf_world(str(template), 43.0, -3.0, 0.0, str(tmp_path), water='<model name="w"/>')
    assert '<model name="w"/>' not in dry and '@' not in dry
    assert wet.index('<model name="w"/>') < wet.index('</world>') and '@' not in wet


def test_a_mesh_without_triangles_is_left_out_of_the_world(aw, tmp_path):
    template = tmp_path / 't.in'
    template.write_text('<sdf><world>@LATITUDE@ @LONGITUDE@ @ELEVATION@\n<!-- Terrain generated -->'
                        '\n  @WATER@\n</world></sdf>')
    (tmp_path / 'buildings.stl').write_bytes(bytes(84))          # no buildings: header + count 0
    (tmp_path / 'trees.stl').write_bytes(bytes(84 + 50))         # one triangle
    world = aw.sdf_world(str(template), 43.0, -3.0, 0.0, str(tmp_path))
    assert 'area_buildings' not in world and 'area_trees' in world and 'area_terrain' in world


def test_the_sea_floor_of_the_orthophoto_is_coloured_by_depth(aw, tmp_path):
    needs_sea_floor()
    pytest.importorskip('navmap_tools')
    cv2 = pytest.importorskip('cv2')
    img = np.full((100, 100, 3), 120, np.uint8)             # a flat grey "orthophoto"

    def rel(x, y):       # the west half is 2 m under the surface, the east 40 m, land to the north
        z = np.where(np.asarray(x) < 0, -2.0, -40.0)
        return np.where(np.asarray(y) > 20, 5.0, z)
    path = str(tmp_path / 'ortho.jpg')
    out = aw.shade_seabed(img, rel, 0.0, (-50.0, -50.0, 50.0, 50.0), 1.0, lambda x, y: (x, y), path)
    assert out.shape == img.shape and os.path.isfile(path)
    shallow = out[80, 20].astype(int)           # BGR: south-west, 2 m deep
    deep = out[80, 80].astype(int)              # south-east, 40 m deep
    assert shallow[2] > shallow[0] and shallow[2] > deep[2]       # sand is redder than the deep blue
    assert deep[0] > deep[2]                                      # and the deep blue is blue
    assert (out[5, 50] == 120).all()                              # the land is untouched
    assert cv2.imread(path) is not None


def _asc_answer(west, south, rows, cell=5.0):
    """A WCS GetCoverage answer in ArcGrid, multipart as the IGN sends it."""
    body = 'ncols %d\nnrows %d\nxllcorner %.3f\nyllcorner %.3f\ncellsize %.1f\nNODATA_value -9999\n' % (
        len(rows[0]), len(rows), west, south, cell)
    body += '\n'.join(' '.join('%.3f' % v for v in r) for r in rows)
    return types.SimpleNamespace(content=(
        '--wcs\nContent-Type: application/asc\nContent-ID: coverage/out.asc\n\n' + body +
        '\n--wcs--\n').encode(), status_code=200)


def test_the_ign_terrain_is_read_with_its_decimals_not_rounded_to_whole_metres(aw):
    # the GeoTIFF of the IGN WCS is int32 (whole metres): on a 5 m grid those steps were 1 m
    # terraces on flat streets. The ArcGrid answer keeps the millimetres
    z, west, north, cell = aw.parse_asc(_asc_answer(
        1000.0, 2000.0, [[647.675, 647.988], [648.107, -9999.0]]).content.decode())
    assert cell == 5.0 and west == 1000.0 and north == 2010.0
    assert z[0, 0] == pytest.approx(647.675, abs=1e-3) and z[1, 0] == pytest.approx(648.107, abs=1e-3)
    assert np.isnan(z[1, 1])


def test_the_ign_terrain_is_asked_in_blocks_and_put_together(aw, monkeypatch):
    # a 30 x 20 m area in blocks of 10 m: six answers, each of the 5 m cells of its block, with a
    # height that tells where it comes from (x + y / 100)
    asked = []

    def fake_get(url, params, what, **kw):
        sx = [float(v) for v in params['subset'][0][2:-1].split(',')]
        sy = [float(v) for v in params['subset'][1][2:-1].split(',')]
        asked.append((sx, sy))
        assert params['format'] == 'application/asc'
        xs = np.arange(sx[0], sx[1], 5.0) + 2.5
        ys = np.arange(sy[1], sy[0], -5.0) - 2.5
        return _asc_answer(sx[0], sy[0], [[x + y / 100.0 for x in xs] for y in ys])
    monkeypatch.setattr(aw, 'get', fake_get)
    frame = types.SimpleNamespace(epsg=25830)
    z, x0, y0, sx, sy = aw.fetch_mdt(frame, (0.0, 0.0, 30.0, 20.0), block=10.0)
    assert len(asked) == 6
    assert (x0, y0, sx, sy) == (0.0, 20.0, 5.0, 5.0) and z.shape == (4, 6)
    for r in range(4):
        for c in range(6):
            assert z[r, c] == pytest.approx((c * 5 + 2.5) + (20 - r * 5 - 2.5) / 100.0, abs=1e-3)
    # and the sampler reads it at the cell centres
    sample = aw.make_dem_sampler((z, x0, y0, sx, sy))
    assert float(sample(np.array([12.5]), np.array([7.5]))[0]) == pytest.approx(12.5 + 0.075, abs=1e-3)


def key_args(**kw):
    a = types.SimpleNamespace(
        lat=40.4168, lon=-3.7038, radius=250.0, terrain_res=0.0, navmap_res=2.0, ortho_ppm=3.0,
        no_trees=False, no_walls=False, no_buildings=False, survey_model='', survey_stl='',
        survey_elevation=0.0, roof='ortho', max_trees=4000, sea='off', sea_model=[])
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def test_a_world_with_or_without_buildings_is_another_world(aw):
    """The terrain alone (world:=terrain) and the area of the same place are not the same world on
    disk: one is never taken for the other."""
    area = aw.cache_key(key_args(), 'spain', [])
    assert area == aw.cache_key(key_args(), 'spain', [])
    assert aw.cache_key(key_args(no_buildings=True), 'spain', []) != area
    assert aw.cache_key(key_args(no_trees=True), 'spain', []) != area
    assert aw.cache_key(key_args(radius=300.0), 'spain', []) != area
