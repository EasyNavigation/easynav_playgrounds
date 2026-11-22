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

"""Trees for geo_world.py: the Gazebo Fuel oak and pine (CC0, models/trees/), planted by the
thousand into ONE mesh (a .glb with 4 submeshes: oak/pine x leaves/bark) and ONE collision mesh (the
trunks), so Gazebo draws and collides the whole forest as a single static object. The crowns start
CROWN_CLEARANCE over the ground: below it a tree is the trunk that collides, and nothing else."""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

C = '{http://www.collada.org/2005/11/COLLADASchema}'

# What each Fuel model measures (m), the crown radius and the trunk radius at the base, and its files.
SPECIES = {
    'oak': dict(dae='oak/oak_tree.dae', leaves='oak/branch_diffuse.png', bark='oak/bark_diffuse.png',
                height=6.53, crown=5.0, cx=-0.6, trunk=0.32),
    'pine': dict(dae='pine/pine_tree.dae', leaves='pine/branch_diffuse.png', bark='pine/bark_diffuse.png',
                 height=5.08, crown=1.59, cx=0.0, trunk=0.15),
}
TRUNK_COLLISION_HEIGHT = 2.0      # m: the part of the trunk the robot can touch
MIN_TRUNK_GAP = 1.5               # m between trunks: their collisions never overlap
# m over the ground under the trunk: no leaf or branch is lower. Only the trunks collide, so below
# this height a tree must show its trunk alone: the lidar sees the visual mesh, and a crown hanging
# lower would be an obstacle for the robot's sensors that its body never touches. Street trees are
# pruned like that.
CROWN_CLEARANCE = 2.5
SINK = 0.1                        # m: the base of a model goes this far under the ground


def models_dir(here):
    """models/trees of the source tree or of the installed package."""
    for c in (os.path.join(here, '..', 'models', 'trees'),
              os.path.join(here, '..', '..', 'share', 'easynav_playground_geo_worlds', 'models', 'trees')):
        if os.path.isdir(c):
            return os.path.abspath(c)
    raise FileNotFoundError('models/trees of easynav_playground_geo_worlds not found (--tree-models)')


def load_dae(path):
    """{node name: (vertices [n, 3], normals, uvs, faces [m, 3])} of a COLLADA file, the node matrix
    applied and the vertices shared between faces."""
    root = ET.parse(path).getroot()
    geoms = {}
    for g in root.iter(C + 'geometry'):
        mesh = g.find(C + 'mesh')
        src = {}
        for s in mesh.findall(C + 'source'):
            acc = s.find(C + 'technique_common').find(C + 'accessor')
            src[s.get('id')] = np.array(s.find(C + 'float_array').text.split(), float).reshape(
                -1, int(acc.get('stride')))
        vert = {v.get('id'): v for v in mesh.findall(C + 'vertices')}
        cols = []
        for tri in mesh.findall(C + 'triangles'):
            ins = {i.get('semantic'): i for i in tri.findall(C + 'input')}
            stride = max(int(i.get('offset')) for i in ins.values()) + 1
            idx = np.array(tri.find(C + 'p').text.split(), int).reshape(-1, stride)
            v = ins['VERTEX']
            pos = src[vert[v.get('source')[1:]].find(C + 'input').get('source')[1:]][idx[:, int(v.get('offset'))]]
            n = ins.get('NORMAL')
            nrm = src[n.get('source')[1:]][idx[:, int(n.get('offset'))]] if n is not None else np.zeros_like(pos)
            t = ins.get('TEXCOORD')
            uv = src[t.get('source')[1:]][idx[:, int(t.get('offset'))]][:, :2] if t is not None else np.zeros((len(pos), 2))
            cols.append(np.c_[pos, nrm, uv])
        geoms['#' + g.get('id')] = np.concatenate(cols)
    out = {}
    for node in root.iter(C + 'node'):
        ig = node.find(C + 'instance_geometry')
        if ig is None:
            continue
        m = node.find(C + 'matrix')
        M = np.array(m.text.split(), float).reshape(4, 4) if m is not None else np.eye(4)
        a = geoms[ig.get('url')]
        pos = (np.c_[a[:, :3], np.ones(len(a))] @ M.T)[:, :3]
        nrm = a[:, 3:6] @ np.linalg.inv(M[:3, :3]).T
        nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-9)
        rows = np.round(np.c_[pos, nrm, a[:, 6:8]], 5)
        uniq, inv = np.unique(rows, axis=0, return_inverse=True)
        out[node.get('name')] = (uniq[:, :3], uniq[:, 3:6], uniq[:, 6:8], inv.reshape(-1, 3))
    return out


def choose(r, h):
    """(species, horizontal scale, vertical scale) of a tree of crown radius r and height h: tall and
    narrow crowns are pines, broad ones oaks. The proportions of the model are kept within reason."""
    sp = 'pine' if h / (2.0 * r) >= 1.4 else 'oak'
    s = SPECIES[sp]
    sz = float(np.clip(h / s['height'], 0.3, 3.0))
    sxy = float(np.clip(r / s['crown'], 0.5 * sz, 1.6 * sz)) if sp == 'pine' else float(
        np.clip(r / s['crown'], 0.25, 2.2))
    return sp, sxy, sz


def crown_base(parts):
    """Height (m, in the model) where the crown of a model starts: its lowest leaf."""
    return float(min(P[:, 2].min() for name, (P, N, U, F) in parts.items() if name == 'Branch'))


def trunk_radius(sp, sxy):
    """Radius (m) of the trunk of a tree: what collides, and the widest the trunk is drawn below the
    crown."""
    return float(np.clip(SPECIES[sp]['trunk'] * sxy, 0.15, 0.6))


def shape(P, sp, part, sxy, sz, zc):
    """The vertices P [n, 3] of a part ('Branch': leaves, else bark) of the model of species sp,
    scaled (sxy, sz) for one tree and with the crown lifted: the crown (above zc, the crown base of
    the model) starts CROWN_CLEARANCE over the ground or higher, keeps its width and fills the height
    of the tree (sz x the model's) as far as it can; below it the trunk is stretched and drawn no
    wider than its collision (the roots and the low branches of the model are drawn into it). Base of
    the trunk at z = 0, SINK under the ground."""
    s = SPECIES[sp]
    Q = np.array(P, float)
    Q[:, 0] -= s['cx']
    base = max(CROWN_CLEARANCE + SINK, zc * sz)
    kc = max((sz * s['height'] - base) / (s['height'] - zc), 0.35 * sz)
    low = Q[:, 2] < zc
    if part != 'Branch':
        cap = trunk_radius(sp, sxy) / sxy
        r = np.hypot(Q[:, 0], Q[:, 1])
        f = np.where(low & (r > cap), cap / np.maximum(r, 1e-9), 1.0)
        Q[:, :2] *= f[:, None]
    Q[:, :2] *= sxy
    Q[:, 2] = np.where(low, Q[:, 2] * base / zc, base + (Q[:, 2] - zc) * kc)
    return Q


def tree_height(r, h):
    return h if h is not None else 3.5 + 1.3 * r


def resolve(trees, blocked, keep_clear, rng):
    """Drop the trees whose trunk would collide: with a building, wall or structure (`blocked(x, y)`
    is True there), with the spawn point or the like (`keep_clear` [(x, y, radius)]), or with another
    trunk (closer than MIN_TRUNK_GAP: the bigger tree stays). Returns (kept, {reason: count})."""
    why = {'in a building/wall': 0, 'near the robot start': 0, 'too close to another trunk': 0}
    ok = []
    for t in trees:
        x, y = t[0], t[1]
        if blocked(x, y):
            why['in a building/wall'] += 1
        elif any(math.hypot(x - cx, y - cy) < cr for cx, cy, cr in keep_clear):
            why['near the robot start'] += 1
        else:
            ok.append(t)
    order = sorted(range(len(ok)), key=lambda i: -ok[i][2])      # big crowns first
    kept = []
    pts = np.array([[ok[i][0], ok[i][1]] for i in order]) if ok else np.zeros((0, 2))
    alive = np.ones(len(order), bool)
    if len(pts):
        kd = cKDTree(pts)
        for k in range(len(order)):
            if not alive[k]:
                continue
            for j in kd.query_ball_point(pts[k], MIN_TRUNK_GAP):
                if j > k and alive[j]:
                    alive[j] = False
                    why['too close to another trunk'] += 1
    kept = [ok[order[k]] for k in range(len(order)) if alive[k]]
    return kept, why


def plant(trees, ground, models, out_glb, rng):
    """Write the forest. `trees`: [(x, y, crown radius, height or None)]; `ground(x, y)`: terrain z.
    Returns (trunk triangles [n, 3, 3] for the collision mesh, the same as 1 m square blocks for the
    NavMap, number of triangles drawn)."""
    import trimesh
    from PIL import Image
    from trimesh.visual import TextureVisuals
    from trimesh.visual.material import PBRMaterial
    parts = {k: load_dae(os.path.join(models, s['dae'])) for k, s in SPECIES.items()}
    zc = {k: crown_base(p) for k, p in parts.items()}
    acc = {}
    trunks, blocks, info = [], [], []
    for x, y, r, h in trees:
        h = tree_height(r, h)
        sp, sxy, sz = choose(r, h)
        j = rng.uniform(0.9, 1.1)
        sxy, sz = sxy * j, sz * j
        yaw = rng.uniform(0, 2 * math.pi)
        c, s_ = math.cos(yaw), math.sin(yaw)
        z = float(ground(np.array([x]), np.array([y]))[0])
        for part, (P, N, U, F) in parts[sp].items():
            Q = shape(P, sp, part, sxy, sz, zc[sp])
            Q = np.c_[c * Q[:, 0] - s_ * Q[:, 1], s_ * Q[:, 0] + c * Q[:, 1], Q[:, 2]] + np.array([x, y, z - SINK])
            Nn = np.c_[c * N[:, 0] - s_ * N[:, 1], s_ * N[:, 0] + c * N[:, 1], N[:, 2]]
            a = acc.setdefault((sp, part), dict(v=[], n=[], u=[], f=[], base=0))
            a['v'].append(Q)
            a['n'].append(Nn)
            a['u'].append(U)
            a['f'].append(F + a['base'])
            a['base'] += len(Q)
        rt = trunk_radius(sp, sxy)
        info.append((x, y, rt))
        trunks.append(prism(x, y, z - 0.2, z + TRUNK_COLLISION_HEIGHT, rt, 8))
        blocks.append(prism(x, y, z - 0.2, z + TRUNK_COLLISION_HEIGHT, 0.5, 4, square=True))
    scene = trimesh.Scene()
    ntri = 0
    for (sp, part), a in acc.items():
        V, N, U, F = (np.concatenate(a[k]) for k in ('v', 'n', 'u', 'f'))
        img = Image.open(os.path.join(models, SPECIES[sp]['leaves' if part == 'Branch' else 'bark'])).convert('RGBA')
        mat = PBRMaterial(baseColorTexture=img, alphaMode='MASK' if part == 'Branch' else 'OPAQUE',
                          alphaCutoff=0.5, doubleSided=True, metallicFactor=0.0, roughnessFactor=1.0)
        m = trimesh.Trimesh(vertices=V, faces=F, vertex_normals=N, process=False,
                            visual=TextureVisuals(uv=U, material=mat))
        scene.add_geometry(m, node_name='%s_%s' % (sp, part.lower()), geom_name='%s_%s' % (sp, part.lower()))
        ntri += len(F)
    scene.export(out_glb)
    cat = lambda L: np.concatenate(L) if L else np.zeros((0, 3, 3))   # noqa: E731
    return cat(trunks), cat(blocks), ntri, info


def validate(info, blocked):
    """Problems (empty = none) in the trunks [(x, y, radius)]: two overlapping, or one in a building,
    wall or structure."""
    out = []
    if len(info) > 1:
        pts = np.array([(x, y) for x, y, _ in info])
        rad = np.array([r for *_, r in info])
        for i, j in cKDTree(pts).query_pairs(2 * rad.max() + 0.3):   # the gap tested below
            if math.hypot(*(pts[i] - pts[j])) < rad[i] + rad[j] + 0.3:
                out.append('trunks %d and %d overlap' % (i, j))
                break
    bad = [i for i, (x, y, _) in enumerate(info) if blocked(x, y)]
    if bad:
        out.append('%d trunks inside a building/wall' % len(bad))
    return out


def prism(x, y, z0, z1, r, n, square=False):
    """Triangles [m, 3, 3] of a closed prism (a regular n-gon of radius r, or a square of half-side r)."""
    if square:
        ring = np.array([(-r, -r), (r, -r), (r, r), (-r, r)])
        n = 4
    else:
        a = np.linspace(0, 2 * np.pi, n, endpoint=False)
        ring = np.c_[r * np.cos(a), r * np.sin(a)]
    b = np.c_[ring[:, 0] + x, ring[:, 1] + y, np.full(n, z0)]
    t = np.c_[ring[:, 0] + x, ring[:, 1] + y, np.full(n, z1)]
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(b[i], b[j], t[j]), (b[i], t[j], t[i])]
    for i in range(1, n - 1):
        tris += [(t[0], t[i], t[i + 1]), (b[0], b[i + 1], b[i])]
    return np.array(tris)
