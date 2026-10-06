#!/usr/bin/env python3
"""Exportiert Modelle (Render + Kollision komplett) fuer core_test / host_test.
usage: export_models.py <terrain_dir> <out_dir> [points_per_model] [limit]"""
import sys, os, struct
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))
import validate_terrain as V

def read_render(b):
    root, _ = V.chunks(b); cl, _ = V.chunks(b, root[0][2], root[0][3])
    gl = [c for c in cl if c[0] == 0x1A][0]; g, _ = V.chunks(b, gl[2], gl[3]); geo = [c for c in g if c[0] == 0x0F]
    if len(geo) != 1: raise ValueError('multi geometry')
    gc, _ = V.chunks(b, geo[0][2], geo[0][3]); st = [c for c in gc if c[0] == 1][0]; hd = b[st[2]:st[3]]
    flags, nt, nv, nm = struct.unpack_from('<4I', hd, 0); p = 16
    uvs = (flags >> 16) & 255
    if uvs == 0: uvs = 2 if flags & 0x80 else (1 if flags & 4 else 0)
    if flags & 0x01000000 or uvs < 1 or not flags & 8: raise ValueError('layout')
    pre = np.frombuffer(hd, 'u1', nv * 4, p).reshape(nv, 4); p += nv * 4
    uv = np.frombuffer(hd, '<f4', nv * 2, p).reshape(nv, 2); p += nv * 8 * uvs
    tri = np.frombuffer(hd, '<u2', nt * 4, p).reshape(nt, 4); p += nt * 8
    p += 16; hv, hn = struct.unpack_from('<II', hd, p); p += 8
    pos = np.frombuffer(hd, '<f4', nv * 3, p).reshape(nv, 3)
    ext = [c for c in gc if c[0] == 3][0]; ec, _ = V.chunks(b, ext[2], ext[3])
    nc = [c for c in ec if c[0] == 0x253F2F9]; night = None
    if nc and struct.unpack_from('<I', b, nc[0][2])[0]:
        night = np.frombuffer(b, 'u1', nv * 4, nc[0][2] + 4).reshape(nv, 4)
    return pos, uv, pre, night, tri[:, [1, 0, 3]], tri[:, 2]

def main(src, outdir, ppm=6, limit=None):
    V.D = Path(src); out = Path(outdir); out.mkdir(parents=True, exist_ok=True)
    cols = {}
    for f in sorted(V.D.glob('*.col')):
        try:
            for r in V.parse_col(f.read_bytes()):
                n, o = V.parse_col_rec(r)
                if o and o['v'] is not None: cols[n] = o
        except Exception as e: print('skip', f.name, e)
    rng = np.random.default_rng(11); done = 0
    for f in sorted(V.D.glob('*.dff')):
        n = f.stem.lower()
        if n not in cols: continue
        try: pos, uv, pre, night, tri, mat = read_render(f.read_bytes())
        except Exception as e: continue
        c = cols[n]; cv = c['v']; fc = c['f']
        a, b, cc = cv[fc[:, 0]], cv[fc[:, 1]], cv[fc[:, 2]]
        L = np.linalg.norm(np.cross(b - a, cc - a), axis=1)
        if L.sum() <= 0: continue
        pick = rng.choice(len(fc), size=ppm, p=L / L.sum()); w = rng.dirichlet([1, 1, 1], size=ppm)
        pts = (a[pick] * w[:, [0]] + b[pick] * w[:, [1]] + cc[pick] * w[:, [2]]).astype('<f4')
        with open(out / (n + '.bin'), 'wb') as g:
            g.write(b'GM02'); g.write(struct.pack('<6I', len(pos), len(tri), c['nv'], c['ntri'], 1 if night is not None else 0, len(pts)))
            g.write(struct.pack('<6f', *c['bmin'], *c['bmax'])); g.write(struct.pack('<4f', *c['sphere']))
            g.write(pos.astype('<f4').tobytes()); g.write(uv.astype('<f4').tobytes()); g.write(pre.tobytes())
            if night is not None: g.write(night.tobytes())
            g.write(np.ascontiguousarray(tri).astype('<u2').tobytes()); g.write(np.ascontiguousarray(mat).astype('<u2').tobytes())
            g.write((cv * 128).round().astype('<i2').tobytes()); g.write(c['raw_faces']); g.write(pts.tobytes())
        done += 1
        if limit and done >= limit: break
    print('exported', done, 'models from', src)

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 6, int(sys.argv[4]) if len(sys.argv) > 4 else None)
