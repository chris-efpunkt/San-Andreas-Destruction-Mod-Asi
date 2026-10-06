#!/usr/bin/env python3
"""Synthetische Testmeshes (grobe Originalgeometrie, Naehte, Huegel) im GM02-Format."""
import sys, struct, numpy as np
from pathlib import Path
def write(path, pos, tri, pts, seam=False, nocol=False):
    pos = np.asarray(pos, 'f4'); tri = np.asarray(tri, 'u2')
    if seam:  # every triangle gets its own vertices: all edges are seams
        pos = pos[tri.reshape(-1)]; tri = np.arange(len(pos), dtype='u2').reshape(-1, 3)
    nv, nt = len(pos), len(tri); uv = (pos[:, :2] / 8).astype('f4'); pre = np.full((nv, 4), 200, 'u1'); night = np.full((nv, 4), 90, 'u1')
    q = np.round(pos * 128).astype('<i2'); ct = np.zeros((nt, 8), 'u1'); ct[:, :6] = np.ascontiguousarray(tri[:, [0, 2, 1]]).astype('<u2').view('u1').reshape(nt, 6); ct[:, 6] = 9
    bmin = pos.min(0); bmax = pos.max(0); cen = (bmin + bmax) / 2; rad = float(np.linalg.norm(pos - cen, axis=1).max())
    with open(path, 'wb') as g:
        if nocol: q = q[:0]; ct = ct[:0]
        g.write(b'GM02'); g.write(struct.pack('<6I', nv, nt, len(q), len(ct), 1, len(pts)))
        g.write(struct.pack('<6f', *bmin, *bmax)); g.write(struct.pack('<4f', *cen, rad))
        g.write(pos.tobytes()); g.write(uv.tobytes()); g.write(pre.tobytes()); g.write(night.tobytes())
        g.write(tri.astype('<u2').tobytes()); g.write(np.zeros(nt, '<u2').tobytes()); g.write(q.tobytes()); g.write(ct.tobytes()); g.write(np.asarray(pts, '<f4').tobytes())
def grid(n, size, zf):
    xs = np.linspace(-size / 2, size / 2, n + 1); pos = np.array([[x, y, zf(x, y)] for y in xs for x in xs]); tri = []
    for j in range(n):
        for i in range(n):
            a = j * (n + 1) + i; b = a + 1; c = a + n + 1; d = c + 1; tri += [[a, b, d], [a, d, c]]
    return pos, tri
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True); rng = np.random.default_rng(3)
flat = lambda x, y: 10.0
hill = lambda x, y: 10 + 6 * np.sin(x / 9) * np.cos(y / 7)
steep = lambda x, y: 10 + 0.9 * x
for name, n, size, zf in [('quad200', 1, 200, flat), ('grid40', 5, 200, flat), ('hill', 12, 120, hill), ('slope42', 4, 120, steep), ('fine', 60, 60, hill)]:
    pos, tri = grid(n, size, zf)
    pts = [[x, y, zf(x, y)] for x, y in rng.uniform(-size * 0.45, size * 0.45, (8, 2))]
    pts += [[0, 0, zf(0, 0)], [size / (2 * n) if n > 1 else 0.01, 0, zf(size / (2 * n), 0)], [-size / 2, -size / 2, zf(-size / 2, -size / 2)], [size / 2, 3.0, zf(size / 2, 3.0)]]   # centre, on an edge, corner, border
    write(out / (name + '.bin'), pos, tri, pts); write(out / (name + '_seams.bin'), pos, tri, pts, seam=True)
# tall thin column (palm trunk): 0.8 x 0.8 x 12 m, closed box
pos = np.array([[x, y, z] for z in (0, 12) for y in (-.4, .4) for x in (-.4, .4)], 'f4')
tri = [[0,1,5],[0,5,4],[1,3,7],[1,7,5],[3,2,6],[3,6,7],[2,0,4],[2,4,6],[4,5,7],[4,7,6],[0,2,3],[0,3,1]]
write(out / 'palm.bin', pos, tri, [[0.4, 0.1, 0.8], [-0.4, 0.0, 1.5], [0.1, 0.4, 0.5]])
# tall palm with a wide crown, like the Las Venturas palms: trunk 0.8 m, crown 7 m wide, 32 m high
def box(x0, y0, z0, x1, y1, z1, o):
    p = [[x, y, z] for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    t = [[0,1,5],[0,5,4],[1,3,7],[1,7,5],[3,2,6],[3,6,7],[2,0,4],[2,4,6],[4,5,7],[4,7,6],[0,2,3],[0,3,1]]
    return p, [[a + o, b + o, c + o] for a, b, c in t]
p1, t1 = box(-.4, -.4, 0, .4, .4, 30, 0); p2, t2 = box(-3.5, -3.5, 30, 3.5, 3.5, 32, 8)
write(out / 'palm_crown.bin', np.array(p1 + p2, 'f4'), t1 + t2, [[0.4, 0.1, 0.8], [-0.4, 0.0, 1.5], [0.1, 0.4, 0.5]])
# the Las Venturas palm as the game has it: no collision triangles at all (the game's file has one box for the trunk)
write(out / 'palm_lv.bin', np.array(p1 + p2, 'f4'), t1 + t2, [[0.4, 0.1, 0.8], [-0.4, 0.0, 1.5], [0.1, 0.4, 0.5]], nocol=True)
# ---- things to break through (v3.6) ----
def qbox(x0, y0, z0, x1, y1, z1, inward=False, alt=False, skip=()):
    """closed box of six quads with their own vertices; inward: faces look into the box; alt: the other diagonal; skip: faces left out (0 -y, 1 +x, 2 +y, 3 -x, 4 top, 5 bottom)"""
    c = [[x, y, z] for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    quads = [[0, 1, 5, 4], [1, 3, 7, 5], [3, 2, 6, 7], [2, 0, 4, 6], [4, 5, 7, 6], [0, 2, 3, 1]]
    P, T = [], []
    for f, q in enumerate(quads):
        if f in skip: continue
        o = len(P); P += [c[i] for i in q]
        t = [[1, 2, 3], [1, 3, 0]] if alt else [[0, 1, 2], [0, 2, 3]]
        T += [[o + a, o + (c_ if inward else b), o + (b if inward else c_)] for a, b, c_ in t]
    return P, T
def join(*parts):
    P, T = [], []
    for p, t in parts:
        o = len(P); P += p; T += [[a + o, b + o, c + o] for a, b, c in t]
    return np.array(P, 'f4'), T
# hangar: walls and roof 0.3 m thick, a room inside with a floor
pos, tri = join(qbox(-10, -10, 0, 10, 10, 8), qbox(-9.7, -9.7, 0.3, 9.7, 9.7, 7.7, inward=True))
write(out / 'hangar.bin', pos, tri, [[0.7, 0.3, 8.0], [10.0, 1.0, 4.0], [5.0, -3.0, 8.0]])
# the same of thin metal: inside faces lie on the outside faces, sharing no vertex and cut along the other diagonal
pos, tri = join(qbox(-10, -10, 0, 10, 10, 8, skip=(5,)), qbox(-10, -10, 0, 10, 10, 8, inward=True, alt=True))
write(out / 'hangar_thin.bin', pos, tri, [[0.7, 0.3, 8.0], [10.0, 1.0, 4.0], [5.0, -3.0, 8.0]])
# a building that is only a skin, under a roof slab 0.3 m thick: nothing is modelled inside
pos, tri = join(qbox(-10, -10, 0, 10, 10, 8, skip=(4, 5)), qbox(-10.5, -10.5, 8, 10.5, 10.5, 8.3))
write(out / 'skinroof.bin', pos, tri, [[0.7, 0.3, 8.3], [3.0, -4.0, 8.3], [-5.0, 2.0, 8.3]])
# a building that is only a skin (four walls, a roof)
pos, tri = join(qbox(-10, -10, 0, 10, 10, 8, skip=(5,)))
write(out / 'skin.bin', pos, tri, [[0.7, 0.3, 8.0], [10.0, 1.0, 4.0], [5.0, -3.0, 8.0]])
# bridge deck, 1.6 m thick, nothing under it in this model
pos, tri = join(qbox(-20, -6, 10, 20, 6, 11.6))
write(out / 'bridge.bin', pos, tri, [[0.7, 0.3, 11.6], [-8.0, 2.0, 11.6], [5.0, -1.0, 11.6]])
# thin deck (0.4 m): through with one blast
pos, tri = join(qbox(-20, -6, 10, 20, 6, 10.4))
write(out / 'deck.bin', pos, tri, [[0.7, 0.3, 10.4], [-8.0, 2.0, 10.4], [5.0, -1.0, 10.4]])
# free-standing wall, 0.4 m thick
pos, tri = join(qbox(-10, -0.2, 0, 10, 0.2, 5))
write(out / 'wall.bin', pos, tri, [[2.0, -0.2, 2.5], [-4.0, 0.2, 1.5], [6.0, -0.2, 3.5]])
# big billboard on two legs (too wide to be knocked over as a whole)
pos, tri = join(qbox(-5.4, -0.4, 0, -4.6, 0.4, 8), qbox(4.6, -0.4, 0, 5.4, 0.4, 8), qbox(-7, -0.3, 8, 7, 0.3, 13))
write(out / 'billboard.bin', pos, tri, [[-5.0, -0.4, 3.0], [5.0, 0.4, 4.0], [1.0, -0.3, 10.0]])
print('ok')
