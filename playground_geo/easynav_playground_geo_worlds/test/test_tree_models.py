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


def _read_glb(path):
    """(the JSON, the binary chunk) of a .glb file, checking its header."""
    import json
    import struct
    b = open(path, 'rb').read()
    magic, version, length = struct.unpack('<4sII', b[:12])
    assert (magic, version, length) == (b'glTF', 2, len(b))
    jl, jt = struct.unpack('<I4s', b[12:20])
    assert jt == b'JSON' and jl % 4 == 0
    bl, bt = struct.unpack('<I4s', b[20 + jl:28 + jl])
    assert bt == b'BIN\0' and bl % 4 == 0 and 28 + jl + bl == len(b)
    return json.loads(b[20:20 + jl]), b[28 + jl:]


def _accessor(doc, blob, i):
    a = doc['accessors'][i]
    v = doc['bufferViews'][a['bufferView']]
    dtype = {5126: np.float32, 5125: np.uint32}[a['componentType']]
    width = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3}[a['type']]
    assert v['byteOffset'] % 4 == 0
    return np.frombuffer(blob, dtype, a['count'] * width, v['byteOffset']).reshape(-1, width)


def test_write_glb_round_trip(tm, tmp_path):
    """What write_glb writes is what it was given: the geometry, the texture coordinates turned to
    glTF's (origin at the top left), the image, and leaves cut by their alpha."""
    from PIL import Image
    v = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 1]], float)
    n = np.tile([0.0, 0.0, 1.0], (4, 1))
    uv = np.array([[0, 0], [1, 0], [1, 1], [0, 0.25]])
    f = np.array([[0, 1, 2], [0, 2, 3]])
    img = Image.new('RGBA', (3, 2), (10, 20, 30, 0))
    img.putpixel((2, 1), (200, 100, 50, 255))
    path = str(tmp_path / 'm.glb')
    tm.write_glb(path, [dict(name='leaves', v=v, n=n, uv=uv, f=f, image=img, mask=True),
                        dict(name='bark', v=v + 5, n=n, uv=uv, f=f[:1], image=img, mask=False)])
    doc, blob = _read_glb(path)
    assert [nd['name'] for nd in doc['nodes']] == ['leaves', 'bark']
    for k, (dv, df) in enumerate(((v, f), (v + 5, f[:1]))):
        prim = doc['meshes'][doc['nodes'][k]['mesh']]['primitives'][0]
        at = prim['attributes']
        assert _accessor(doc, blob, at['POSITION']) == pytest.approx(dv)
        assert doc['accessors'][at['POSITION']]['min'] == pytest.approx(dv.min(0))
        assert _accessor(doc, blob, at['NORMAL']) == pytest.approx(n)
        assert _accessor(doc, blob, at['TEXCOORD_0']) == pytest.approx(np.c_[uv[:, 0], 1 - uv[:, 1]])
        assert (_accessor(doc, blob, prim['indices']).reshape(-1, 3) == df).all()
        mat = doc['materials'][prim['material']]
        assert mat['alphaMode'] == ('MASK' if k == 0 else 'OPAQUE') and mat['doubleSided']
        tex = doc['textures'][mat['pbrMetallicRoughness']['baseColorTexture']['index']]
        view = doc['bufferViews'][doc['images'][tex['source']]['bufferView']]
        import io
        png = Image.open(io.BytesIO(blob[view['byteOffset']:view['byteOffset'] + view['byteLength']]))
        assert np.array_equal(np.asarray(png), np.asarray(img))


def test_plant_writes_a_forest_glb(tm, tmp_path):
    """plant() writes the four submeshes (oak and pine, leaves and bark) of the trees it plants."""
    trees = [(0.0, 0.0, 3.0, None), (10.0, 0.0, 1.0, 6.0), (0.0, 10.0, 2.0, 12.0)]
    path = str(tmp_path / 'trees.glb')
    trunks, blocks, ntri, info = tm.plant(trees, lambda x, y: np.zeros(len(x)),
                                          tm.models_dir(TOOLS), path, np.random.default_rng(0))
    doc, blob = _read_glb(path)
    names = {nd['name'] for nd in doc['nodes']}
    assert names <= {'oak_branch', 'oak_bark', 'pine_branch', 'pine_bark'} and names
    count = sum(doc['accessors'][m['primitives'][0]['indices']]['count'] for m in doc['meshes'])
    assert count == 3 * ntri
    assert len(info) == len(trees)
