#!/usr/bin/env python3
"""Stitches the tiles of a tiled terrain mesh (the URJC excavation, collision and visual meshes).

The terrain is a set of tiles that share no vertices; each one ends in a vertical skirt that goes
down under the ground. A tile's surface ends at the skirt's rim, and the rims of neighbouring tiles
are at slightly different heights: each seam is a step a robot may not climb. Here the rim vertices
of different tiles closer than `reach` horizontally meet halfway, in height and across (when they
differ by less than `max_step`; more is a wall). The skirts' bottoms do not move.

Usage: stitch_rims.py in.stl in.dae out.stl out.dae [--reach 0.35] [--max-step 0.5]
"""
import argparse
import struct
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np


def load_stl(path):
    b = open(path, 'rb').read()
    n = struct.unpack('<I', b[80:84])[0]
    dt = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
    return np.frombuffer(b, dtype=dt, count=n, offset=84)['v'].reshape(-1, 3, 3).astype(np.float64)


def save_stl(path, tris):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['n'] = n
    rec['v'] = tris.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(b'stitched rims'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        f.write(rec.tobytes())


def components(nv, faces):
    """Tile of each vertex: connected components through the faces."""
    p = np.arange(nv)

    def find(a):
        while p[a] != a:
            p[a] = p[p[a]]
            a = p[a]
        return a
    for a, b, c in faces:
        ra = find(a)
        p[find(b)] = ra
        p[find(c)] = ra
    return np.array([find(i) for i in range(nv)])


def stitch(V, faces, reach, max_step, skirt_deg=60.0, ground_deg=45.0):
    """New positions of the vertices: the rims of neighbouring tiles met halfway."""
    tile = components(len(V), faces)
    t = V[faces]
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    slope = np.degrees(np.arccos(np.clip(np.abs(n[:, 2]) / np.maximum(
        np.linalg.norm(n, axis=1), 1e-12), 0, 1)))
    on_skirt = np.zeros(len(V), bool)
    on_ground = np.zeros(len(V), bool)
    on_skirt[faces[slope > skirt_deg].ravel()] = True
    on_ground[faces[slope < ground_deg].ravel()] = True
    rim = np.flatnonzero(on_skirt & on_ground)
    # Each rim vertex meets the neighbouring tile's rim halfway, in height and across (closing the
    # crack between them): that rim interpolated where it passes closest, between its two
    # vertices nearest to this one. Both sides do the same: no chains along a seam.
    g = defaultdict(list)
    for v in rim:
        g[tuple(np.floor(V[v, :2] / reach).astype(int))].append(v)
    P = V.copy()
    steps = []
    for v in rim:
        k = np.floor(V[v, :2] / reach).astype(int)
        near = [w for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                for w in g.get((k[0] + dx, k[1] + dy), ()) if tile[w] != tile[v]]
        if not near:
            continue
        near = np.array(near)
        h = np.hypot(V[near, 0] - V[v, 0], V[near, 1] - V[v, 1])
        best = near[np.argmin(h)]
        if h.min() >= reach:
            continue
        same = near[tile[near] == tile[best]]
        hs = np.hypot(V[same, 0] - V[v, 0], V[same, 1] - V[v, 1])
        order = same[np.argsort(hs)]
        a_, b_ = V[order[0]], V[order[1]] if len(order) > 1 else V[order[0]]
        ab = b_[:2] - a_[:2]
        t = np.clip(np.dot(V[v, :2] - a_[:2], ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
        q = a_ + t * (b_ - a_)  # the other rim, where it passes closest
        if abs(q[2] - V[v, 2]) >= max_step:
            continue
        steps.append(abs(q[2] - V[v, 2]))
        P[v] = (V[v] + q) / 2.0
    Z = P
    return Z, len(rim), np.array(steps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('stl')
    ap.add_argument('dae')
    ap.add_argument('out_stl')
    ap.add_argument('out_dae')
    ap.add_argument('--reach', type=float, default=0.35)
    ap.add_argument('--max-step', type=float, default=0.5)
    a = ap.parse_args()

    tris = load_stl(a.stl)
    V, inv = np.unique(np.round(tris.reshape(-1, 3), 4), axis=0, return_inverse=True)
    faces = inv.reshape(-1, 3)
    Z, nrim, steps = stitch(V, faces, a.reach, a.max_step)
    print(f'collision: {nrim} rim vertices, {len(steps)} at seams; steps closed: median '
          f'{np.median(steps):.3f} p95 {np.percentile(steps, 95):.3f} max {steps.max():.3f} m')
    V = Z
    save_stl(a.out_stl, V[faces])

    text = open(a.dae).read()
    root = ET.fromstring(text)
    ns = {'c': root.tag.split('}')[0].strip('{')}
    node = root.find('.//c:visual_scene//c:node', ns)
    m = np.array([float(x) for x in node.find('c:matrix', ns).text.split()]).reshape(4, 4)
    geo = root.find('.//c:library_geometries/c:geometry/c:mesh', ns)
    pos_src = geo.find('c:vertices/c:input[@semantic="POSITION"]', ns).get('source')[1:]
    fa = geo.find(f"c:source[@id='{pos_src}']/c:float_array", ns)
    P = np.array([float(x) for x in fa.text.split()]).reshape(-1, 3)
    W = P @ m[:3, :3].T + m[:3, 3]
    vf = []
    for tri in geo.findall('c:triangles', ns):
        stride = len(tri.findall('c:input', ns))
        off = int(tri.find("c:input[@semantic='VERTEX']", ns).get('offset'))
        idx = np.array([int(x) for x in tri.find('c:p', ns).text.split()]).reshape(-1, stride)
        vf.append(idx[:, off].reshape(-1, 3))
    vf = np.vstack(vf)
    # Vertices repeated in the file (same position) are one for the tiles
    Wu, winv = np.unique(np.round(W, 4), axis=0, return_inverse=True)
    Zu, nrim, steps = stitch(Wu, winv[vf], a.reach, a.max_step)
    print(f'visual: {nrim} rim vertices, {len(steps)} at seams; steps closed: median '
          f'{np.median(steps):.3f} p95 {np.percentile(steps, 95):.3f} max {steps.max():.3f} m')
    # the node's matrix is a translation (and scale): the same move in its coordinates
    P += (Zu[winv] - Wu[winv]) / np.diag(m)[:3]
    new = ' '.join(f'{x:.6g}' for x in P.ravel())
    start = text.index('>', text.index(f'id="{fa.get("id")}"')) + 1
    end = text.index('</float_array>', start)
    open(a.out_dae, 'w').write(text[:start] + new + text[end:])


if __name__ == '__main__':
    main()
