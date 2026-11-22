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

"""The trees of tools/tree_models.py: below CROWN_CLEARANCE a tree is its trunk alone, as wide as
what collides, so the robot's lidar (which sees the drawn mesh) sees no obstacle its body never
touches."""

import importlib.util
import itertools
import os
import sys

import pytest

np = pytest.importorskip('numpy')
pytest.importorskip('scipy')

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.path.dirname(HERE), 'tools')


@pytest.fixture(scope='module')
def tm():
    sys.path.insert(0, TOOLS)
    spec = importlib.util.spec_from_file_location('tree_models_under_test',
                                                  os.path.join(TOOLS, 'tree_models.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def parts(tm):
    d = tm.models_dir(TOOLS)
    return {sp: tm.load_dae(os.path.join(d, s['dae'])) for sp, s in tm.SPECIES.items()}


def trees(tm):
    """(species, sxy, sz) of trees from small to big crowns and heights, as plant() chooses them."""
    for r, h, j in itertools.product((0.5, 1.0, 2.0, 3.5, 6.0), (None, 2.0, 4.0, 8.0, 20.0), (0.9, 1.1)):
        sp, sxy, sz = tm.choose(r, tm.tree_height(r, h))
        yield sp, sxy * j, sz * j


def test_the_models_as_they_are_hang_their_crowns_low(tm, parts):
    """What the lift is for: the oak's leaves start 1.2 m up and the pine's 0.6 m, and the oak's roots
    spread metres around the trunk."""
    assert tm.crown_base(parts['oak']) < 1.5
    assert tm.crown_base(parts['pine']) < 1.0
    P = parts['oak']['Bark'][0]
    assert np.hypot(P[:, 0] - tm.SPECIES['oak']['cx'], P[:, 1]).max() > 2.0


def test_no_leaf_or_branch_below_the_clearance(tm, parts):
    for sp, sxy, sz in trees(tm):
        zc = tm.crown_base(parts[sp])
        rt = tm.trunk_radius(sp, sxy)
        for part, (P, N, U, F) in parts[sp].items():
            Q = tm.shape(P, sp, part, sxy, sz, zc)
            z = Q[:, 2] - tm.SINK          # over the ground
            low = z < tm.CROWN_CLEARANCE - 1e-9
            if part == 'Branch':
                assert not low.any(), (sp, sxy, sz)
            else:
                assert np.hypot(Q[low, 0], Q[low, 1]).max(initial=0.0) <= rt + 1e-9, (sp, sxy, sz)


def test_a_tree_keeps_its_height_and_width_when_it_is_tall_enough(tm, parts):
    for sp in ('oak', 'pine'):
        s = tm.SPECIES[sp]
        zc = tm.crown_base(parts[sp])
        sz = 1.5
        sxy = 1.2
        P = parts[sp]['Branch'][0]
        Q = tm.shape(P, sp, 'Branch', sxy, sz, zc)
        assert Q[:, 2].max() == pytest.approx(sz * s['height'], abs=0.01)   # height: 2 decimals
        width = np.ptp(P[:, :2], axis=0) * sxy
        assert np.ptp(Q[:, :2], axis=0) == pytest.approx(width, abs=1e-6)


def test_a_big_tree_is_only_scaled(tm, parts):
    """Where the crown already starts above the clearance, the tree is the model scaled as before."""
    sp = 'oak'
    zc = tm.crown_base(parts[sp])
    sz = 3.0
    assert zc * sz > tm.CROWN_CLEARANCE + tm.SINK
    P = parts[sp]['Branch'][0]
    Q = tm.shape(P, sp, 'Branch', 1.0, sz, zc)
    assert Q[:, 2] == pytest.approx(P[:, 2] * sz, abs=1e-6)
