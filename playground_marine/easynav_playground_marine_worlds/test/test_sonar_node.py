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

"""The imaging sonar of tools/sonar_node.py: its geometry, the echoes it makes of what the rays hit
(ranges, beams, incidence, absorption, beam pattern, shadows, noise) and its images."""

import importlib.util
import math
import os

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROWS, COLS = 32, 512          # the rays of marine_sensors.SONAR


def tool(name='sonar_node'):
    spec = importlib.util.spec_from_file_location(
        name + '_under_test', os.path.join(ROOT, 'tools', name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def t():
    return tool()


def sonar(t, **kw):
    kw.setdefault('seed', 1)
    return t.Sonar(**kw)


def rays(s, rows=ROWS, cols=COLS):
    """Unit directions (rows, cols, 3) of the rays as the gpu_lidar casts them."""
    el = np.linspace(-s.half_v, s.half_v, rows)
    az = np.linspace(-s.half_h, s.half_h, cols)
    e, a = np.meshgrid(el, az, indexing='ij')
    return np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], axis=2)


def wall(s, distance, tilt=0.0, rows=ROWS, cols=COLS):
    """The hits of the rays on a plane at `distance` m ahead, its normal turned `tilt` rad about the
    vertical (0: facing the sonar)."""
    n = np.array([-math.cos(tilt), -math.sin(tilt), 0.0])
    d = rays(s, rows, cols)
    t_ = (n @ np.array([distance, 0.0, 0.0])) / (d @ n)       # the plane through (distance, 0, 0)
    t_ = np.where(t_ > 0, t_, np.nan)
    return (d * t_[..., None]).reshape(-1, 3)


def central(points, degrees=20.0):
    """The hits within `degrees` of straight ahead (all of them within range)."""
    return points[np.abs(np.arctan2(points[:, 1], points[:, 0])) < math.radians(degrees)]


def clean(t, **kw):
    """A sonar without speckle nor noise: the echoes as the model makes them."""
    return sonar(t, noise_floor=0.0, speckle=False, **kw)


def echo(s, points):
    return s.intensity(points, ROWS, COLS)


# --- geometry

def test_a_sonar_needs_beams_and_a_range(t):
    for kw in ({'beams': 1}, {'min_range': 5.0, 'max_range': 5.0}, {'min_range': 0.0},
               {'range_resolution': 0.0}):
        with pytest.raises(ValueError):
            t.Sonar(**kw)


def test_the_range_bins_cover_min_to_max(t):
    s = sonar(t, min_range=0.5, max_range=50.0, range_resolution=0.1)
    assert s.bins == 495 and len(s.ranges) == 495
    assert s.ranges[0] == pytest.approx(0.55) and s.ranges[-1] == pytest.approx(49.95)
    assert np.diff(s.ranges) == pytest.approx(np.full(494, 0.1))


def test_beam_zero_is_at_port_and_the_last_at_starboard(t):
    s = sonar(t)
    assert s.azimuths[0] == pytest.approx(s.half_h) and s.azimuths[-1] == pytest.approx(-s.half_h)
    assert list(s.beam_of(s.azimuths)) == list(range(s.beams))
    assert s.beam_of(np.array([0.0]))[0] in (127, 128)


def test_the_beam_directions_follow_marine_acoustic_msgs(t):
    s = sonar(t)
    d = s.beam_directions()
    assert np.linalg.norm(d, axis=1) == pytest.approx(np.ones(s.beams))
    assert np.all(d[:, 0] == 0.0)                       # x up: the fan is horizontal
    assert d[0, 1] < 0.0 and d[-1, 1] > 0.0             # y to starboard: beam 0 at port
    assert d[s.beams // 2, 2] == pytest.approx(1.0, abs=1e-4)    # z forward


def test_hits_go_to_the_cell_of_the_ray_that_made_them(t):
    s = sonar(t)
    p = rays(s)
    grid = s.grid((p * 20.0).reshape(-1, 3), ROWS, COLS)
    assert np.allclose(grid, p * 20.0)
    one = s.grid(np.array([[10.0, 0.0, 0.0]]), ROWS, COLS)
    assert np.sum(np.all(np.isfinite(one), axis=2)) == 1


def test_hits_out_of_the_field_of_view_are_left_out(t):
    s = sonar(t)
    grid = s.grid(np.array([[0.0, 10.0, 0.0], [-10.0, 0.0, 0.0], [5.0, 0.0, 10.0]]), ROWS, COLS)
    assert not np.any(np.isfinite(grid))


# --- echoes

def test_a_wall_ahead_shows_at_its_range_with_water_before_it_and_shadow_behind(t):
    s = clean(t)
    image = echo(s, wall(s, 20.0))
    profile = image[:, s.beams // 2 - 5: s.beams // 2 + 5].sum(axis=1)
    peak = s.ranges[np.argmax(profile)]
    assert peak == pytest.approx(20.0, abs=0.3)
    near = s.ranges < 19.5
    beyond = s.ranges > 21.5
    assert profile[near].max() < 1e-3 * profile.max() and profile[beyond].max() < 1e-3 * profile.max()


def test_an_object_shows_in_the_beams_of_its_bearing(t):
    s = clean(t)
    a = math.radians(30.0)                                          # to port
    box = [(15.0 * math.cos(a), 15.0 * math.sin(a), z) for z in np.linspace(-2.0, 2.0, 30)]
    image = echo(s, np.array(box))
    beam = int(np.argmax(image.sum(axis=0)))
    assert s.azimuths[beam] == pytest.approx(a, abs=math.radians(1.5))
    assert s.ranges[int(np.argmax(image.sum(axis=1)))] == pytest.approx(15.0, abs=0.3)


def test_a_surface_facing_the_sonar_echoes_more_than_an_oblique_one(t):
    s = clean(t)
    facing = echo(s, central(wall(s, 20.0))).sum()
    oblique = echo(s, central(wall(s, 20.0, tilt=math.radians(60.0)))).sum()
    # incidence 40 to 80 degrees over the central rays: a mean cos^2 of about 0.27 (the border of
    # the patch, without neighbours to tell its normal, counts as facing)
    assert 0.2 < oblique / facing < 0.5


def test_far_echoes_are_weaker_by_the_absorption(t):
    s = clean(t, absorption_db_per_m=0.2)
    near = echo(s, central(wall(s, 10.0))).sum()
    far = echo(s, central(wall(s, 40.0))).sum()
    ratio_db = 10.0 * math.log10(near / far)
    assert ratio_db == pytest.approx(2 * 0.2 * 30.0, abs=1.5)       # two ways, 30 m more


def test_the_vertical_beam_pattern_favours_the_centre(t):
    s = clean(t)
    centre = echo(s, np.array([[20.0, 0.0, 0.0]])).sum()
    edge = echo(s, np.array([[20.0 * math.cos(s.half_v), 0.0, 20.0 * math.sin(s.half_v)]])).sum()
    assert edge / centre == pytest.approx(0.5, rel=0.02)            # -3 dB at the edge


def test_with_nothing_in_view_there_is_only_the_noise_floor(t):
    s = sonar(t, noise_floor=2e-4)
    image = echo(s, np.zeros((0, 3)))
    assert image.shape == (s.bins, s.beams)
    assert image.mean() == pytest.approx(2e-4, rel=0.05)
    assert np.mean(s.to_uint8(image)) < 30


def test_invalid_hits_are_ignored(t):
    good = echo(clean(t), np.array([[20.0, 0.0, 0.0]]))
    dirty = echo(clean(t), np.array([[20.0, 0.0, 0.0], [np.nan, 0, 0], [np.inf, 1, 1]]))
    assert np.allclose(good, dirty)


def test_hits_out_of_range_make_no_echo(t):
    s = clean(t, min_range=0.5, max_range=50.0)
    assert echo(s, np.array([[0.2, 0.0, 0.0], [80.0, 0.0, 0.0]])).max() == 0.0


def test_the_same_seed_gives_the_same_image(t):
    a = echo(sonar(t, seed=7), wall(sonar(t), 20.0))
    b = echo(sonar(t, seed=7), wall(sonar(t), 20.0))
    c = echo(sonar(t, seed=8), wall(sonar(t), 20.0))
    assert np.array_equal(a, b) and not np.array_equal(a, c)


def test_speckle_keeps_the_mean_echo(t):
    s0 = clean(t)
    image = echo(s0, wall(s0, 20.0))
    s = sonar(t, noise_floor=0.0, seed=0)
    means = [echo(s, wall(s, 20.0)).sum() for _ in range(20)]
    assert np.mean(means) == pytest.approx(image.sum(), rel=0.1)


# --- images

def test_log_compression_maps_the_dynamic_range_to_eight_bits(t):
    s = sonar(t, dynamic_range_db=40.0)
    out = s.to_uint8(np.array([[1.0, 10.0, 1e-2, 1e-4, 1e-6, 0.0]]))
    assert out.dtype == np.uint8
    assert list(out[0]) == [255, 255, 127, 0, 0, 0]
    ramp = s.to_uint8(np.logspace(-5, 1, 50)[None, :])[0]
    assert np.all(np.diff(ramp.astype(int)) >= 0)


def test_the_fan_puts_the_sonar_at_the_bottom_centre_and_port_on_the_left(t):
    s = sonar(t)
    rows, cols = s.fan_map(640)
    h, w = rows.shape
    assert w == 640
    assert rows[h - 1, w // 2] == -1                           # at the apex: inside min_range
    ahead = rows[:, w // 2]
    top = np.nonzero(ahead >= 0)[0][0]                         # the farthest pixel straight ahead
    assert top <= 2 and ahead[top] >= s.bins - 2
    assert abs(cols[top, w // 2] - s.beams // 2) <= 1
    assert rows[h - 1, 0] == -1 and rows[h - 1, w - 1] == -1   # beside the apex: out of the fan
    middle = h - 1 - h // 4
    inside = np.nonzero(rows[middle] >= 0)[0]
    assert cols[middle, inside[0]] < cols[middle, inside[-1]]  # left (port) has the low beams


def test_the_fan_image_shows_the_cells_of_the_fan(t):
    s = sonar(t)
    fan = s.fan_map(320)
    image8 = np.zeros((s.bins, s.beams), np.uint8)
    image8[:, :s.beams // 2] = 200                               # the port half
    picture = t.fan_image(image8, fan)
    assert picture.shape == fan[0].shape and picture.dtype == np.uint8
    h, w = picture.shape
    assert picture[h // 2, : w // 2].max() == 200 and picture[h // 2, w // 2 + 5:].max() == 0
    assert np.all(picture[fan[0] < 0] == 0)


def test_detections_are_the_strong_cells_at_zero_elevation(t):
    s = sonar(t)
    image8 = np.zeros((s.bins, s.beams), np.uint8)
    image8[100, 10] = 250
    image8[200, s.beams - 1] = 100
    p = s.detections(image8, 170)
    assert p.shape == (1, 3)
    r, a = s.ranges[100], s.azimuths[10]
    assert p[0] == pytest.approx([r * math.cos(a), r * math.sin(a), 0.0])
    assert p[0, 1] > 0.0                                        # port is +y in the rays' frame
    assert s.detections(image8, 255).shape == (0, 3)


def test_the_sonar_of_the_playground_works_with_its_rays(t):
    table = tool('marine_sensors').SONAR
    s = t.Sonar(table['beams'], table['horizontal_fov'], table['vertical_fov'],
                table['min_range'], table['max_range'], seed=2)
    image = s.intensity(wall(s, 25.0, rows=table['rays_vertical'],
                             cols=table['rays_horizontal']),
                        table['rays_vertical'], table['rays_horizontal'])
    image8 = s.to_uint8(image)
    assert image8.shape == (s.bins, table['beams'])
    ahead = image8[:, np.abs(s.azimuths) < math.radians(5.0)]
    assert ahead[np.abs(s.ranges - 25.0) < 0.3].mean() > ahead[s.ranges < 20.0].mean() + 100


def test_blur_keeps_the_energy_and_the_size(t):
    image = np.zeros((50, 40))
    image[25, 20] = 1.0
    out = t.blur(image, 2.0, axis=1)
    assert out.shape == image.shape and out.sum() == pytest.approx(1.0)
    assert out[25, 20] < 1.0 and out[25, 18] > 0.0
    assert t.blur(image, 0.0, axis=0) is image
