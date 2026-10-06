/* SA Destruction v3.0 - crater geometry core (pure C, no platform or engine dependencies).
 *
 * Carves a spherical crater into an arbitrary triangle mesh WITHOUT needing a pre-refined
 * mesh:
 *   1. gm_refine : triangles touching the sphere are subdivided locally until no edge that
 *                  passes through the sphere is longer than h. Long edges are first cut where
 *                  they cross a slightly larger "guard" sphere, so the fine mesh stays confined
 *                  to the crater and repeated craters do not multiply triangles. Every decision
 *                  depends only on the edge itself (positions in canonical order), so both
 *                  triangles sharing an edge - and duplicated seam vertices - always split
 *                  identically. No cracks.
 *   2. gm_cut    : edges crossing the sphere surface are split exactly on the surface.
 *                  This gives a crisp rim.
 *   3. gm_project: every vertex inside the ground-side half of the sphere is projected from
 *                  the pole Q (the point of the sphere on the air side) onto the far side of
 *                  the sphere. Rim vertices are fixed points of this map, so the bowl stays
 *                  attached to the rim. Geometry in the air-side half (poles, fences, the top of
 *                  a rock) is left alone: projecting it would smear it across the whole bowl.
 * Thin geometry (fences, sheets, slabs thinner than the crater is deep) is different: there is
 * nothing behind it that a bowl could be carved into. Such triangles are flagged by gm_mark_thin
 * and simply removed where they lie inside the sphere: a hole.
 * For everything else no triangle is removed; the mesh keeps its topology and stays watertight.
 */
#ifndef GM_CORE_INCLUDED
#define GM_CORE_INCLUDED

#define GM_MAXV 65535u
#define GM_MAXT 65535u
#define GM_MAX_FATTR 3u
#define GM_MAX_BATTR 3u
#define GM_EPS 0.10f          /* vertices within this distance of the sphere surface count as ON it:
                                  they are neither moved nor is an edge cut next to them (no slivers) */
#define GM_MAX_PASSES 48u
#define GM_HASH_SIZE 32768u    /* power of two */

typedef struct {
    U32 nv, nt;
    Vec3* pos;                 /* [GM_MAXV] */
    U16* tri;                  /* [GM_MAXT * 3] */
    U16* aux;                  /* [GM_MAXT] per-triangle payload (material index / surface+light) */
    U8* flag;                  /* [GM_MAXT] optional: 1 = thin geometry, gets a hole instead of a bowl */
    U32 nf; float* fa[GM_MAX_FATTR]; U32 fdim[GM_MAX_FATTR];   /* float attributes, lerped */
    U32 nb; U8* ba[GM_MAX_BATTR];                              /* RGBA attributes, lerped */
    int normalAttr;            /* index into fa[] holding vertex normals, or -1 */
} GmMesh;

typedef struct { U32 splits, feet, cuts, moved, bowlTris, holeTris, kept, shell, debris; } GmStats;

#ifdef GM_TRACE
static void gm_trace_vertex(U32 newIndex, U32 a, U32 b, U32 c, int kind);  /* kind: 0 mid, 1 foot, 2 cut, 3 guard split, 4 copy */
static void gm_trace_tri(U32 newTri, U32 parentTri);
#define GM_TRACE_V(n, a, b, c, k) gm_trace_vertex(n, a, b, c, k)
#define GM_TRACE_T(n, p) gm_trace_tri(n, p)
#else
#define GM_TRACE_V(n, a, b, c, k) ((void)0)
#define GM_TRACE_T(n, p) ((void)0)
#endif

static float gm_sqrt(float a) {
    union { float f; U32 i; } u;
    float x;
    if (!(a > 0.f)) return 0.f;
    u.f = a; u.i = 0x1FBD1DF5u + (u.i >> 1); x = u.f;
    x = 0.5f * (x + a / x); x = 0.5f * (x + a / x); x = 0.5f * (x + a / x);
    return x;
}
static Vec3 gm_sub(Vec3 a, Vec3 b) { Vec3 r = { a.x - b.x, a.y - b.y, a.z - b.z }; return r; }
static float gm_dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static Vec3 gm_cross(Vec3 a, Vec3 b) { Vec3 r = { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; return r; }
static float gm_dist2(Vec3 a, Vec3 b) { Vec3 d = gm_sub(a, b); return gm_dot(d, d); }

/* canonical order of two points: makes every per-edge computation independent of the
 * direction in which a triangle happens to traverse the edge */
static int gm_less(Vec3 a, Vec3 b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
}
/* squared distance from p to segment ab (a, b in canonical order) */
static float gm_seg_dist2(Vec3 p, Vec3 a, Vec3 b) {
    Vec3 ab, ap, q;
    float t, l2;
    if (gm_less(b, a)) { Vec3 s = a; a = b; b = s; }
    ab = gm_sub(b, a); ap = gm_sub(p, a); l2 = gm_dot(ab, ab);
    t = l2 > 0.f ? gm_dot(ap, ab) / l2 : 0.f;
    if (t < 0.f) t = 0.f; else if (t > 1.f) t = 1.f;
    q.x = a.x + ab.x * t; q.y = a.y + ab.y * t; q.z = a.z + ab.z * t;
    return gm_dist2(p, q);
}
/* closest point on triangle abc to p (Ericson); barycentric weights in w[3] */
static Vec3 gm_closest_on_tri(Vec3 p, Vec3 a, Vec3 b, Vec3 c, float* w) {
    Vec3 ab = gm_sub(b, a), ac = gm_sub(c, a), ap = gm_sub(p, a), bp, cp, r;
    float d1 = gm_dot(ab, ap), d2 = gm_dot(ac, ap), d3, d4, d5, d6, va, vb, vc, v, u, den;
    if (d1 <= 0.f && d2 <= 0.f) { w[0] = 1; w[1] = 0; w[2] = 0; return a; }
    bp = gm_sub(p, b); d3 = gm_dot(ab, bp); d4 = gm_dot(ac, bp);
    if (d3 >= 0.f && d4 <= d3) { w[0] = 0; w[1] = 1; w[2] = 0; return b; }
    vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
        v = d1 / (d1 - d3); w[0] = 1 - v; w[1] = v; w[2] = 0;
        r.x = a.x + v * ab.x; r.y = a.y + v * ab.y; r.z = a.z + v * ab.z; return r;
    }
    cp = gm_sub(p, c); d5 = gm_dot(ab, cp); d6 = gm_dot(ac, cp);
    if (d6 >= 0.f && d5 <= d6) { w[0] = 0; w[1] = 0; w[2] = 1; return c; }
    vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
        u = d2 / (d2 - d6); w[0] = 1 - u; w[1] = 0; w[2] = u;
        r.x = a.x + u * ac.x; r.y = a.y + u * ac.y; r.z = a.z + u * ac.z; return r;
    }
    va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
        u = (d4 - d3) / ((d4 - d3) + (d5 - d6)); w[0] = 0; w[1] = 1 - u; w[2] = u;
        r.x = b.x + u * (c.x - b.x); r.y = b.y + u * (c.y - b.y); r.z = b.z + u * (c.z - b.z); return r;
    }
    den = 1.f / (va + vb + vc); v = vb * den; u = vc * den;
    w[0] = 1 - v - u; w[1] = v; w[2] = u;
    r.x = a.x + ab.x * v + ac.x * u; r.y = a.y + ab.y * v + ac.y * u; r.z = a.z + ab.z * v + ac.z * u;
    return r;
}

/* ---- edge -> new vertex map (per pass) ------------------------------------------- */
static U8 gm_orphan[GM_MAXV];             /* 2 = vertex only belonged to triangles removed by a hole */
static float gm_clamp[GM_MAXV];           /* how far the bowl projection may move a vertex: see gm_break. Static storage starts at 0 = nothing moves; gm_crater sets it. */

static U32 gm_hkey[GM_HASH_SIZE];
static U16 gm_hval[GM_HASH_SIZE];
static U32 gm_hcount;
static void gm_hash_clear(void) { memset(gm_hkey, 0xFF, sizeof(gm_hkey)); gm_hcount = 0; }
/* returns slot; *found tells whether the key was present */
static U32 gm_hash_slot(U32 a, U32 b, int* found) {
    U32 key = a < b ? (a << 16) | b : (b << 16) | a;
    U32 i = (key * 2654435761u) >> 17;      /* 15 bits */
    for (;;) {
        if (gm_hkey[i] == key) { *found = 1; return i; }
        if (gm_hkey[i] == 0xFFFFFFFFu) { *found = 0; gm_hkey[i] = key; gm_hcount++; return i; }
        i = (i + 1) & (GM_HASH_SIZE - 1);
    }
}

/* new vertex = sum of w[k] * vertex i[k]; position is given explicitly */
static int gm_add_vertex(GmMesh* m, Vec3 p, const U32* idx, const float* w, U32 n) {
    U32 v = m->nv, k, d, j;
    if (v >= GM_MAXV) return -1;
    m->pos[v] = p;
    for (k = 0; k < m->nf; k++) {
        U32 dim = m->fdim[k]; float* a = m->fa[k];
        for (d = 0; d < dim; d++) {
            float s = 0.f;
            for (j = 0; j < n; j++) s += w[j] * a[idx[j] * dim + d];
            a[v * dim + d] = s;
        }
        if ((int)k == m->normalAttr) {
            float* q = a + v * 3; float l = gm_sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
            if (l > 1e-6f) { q[0] /= l; q[1] /= l; q[2] /= l; }
        }
    }
    for (k = 0; k < m->nb; k++) {
        U8* a = m->ba[k];
        for (d = 0; d < 4; d++) {
            float s = 0.5f;
            for (j = 0; j < n; j++) s += w[j] * (float)a[idx[j] * 4 + d];
            a[v * 4 + d] = (U8)(s < 0.f ? 0.f : (s > 255.f ? 255.f : s));
        }
    }
    m->nv = v + 1;
    return (int)v;
}
/* vertex on edge (a,b) at canonical parameter t (measured from the canonically smaller end) */
static int gm_edge_vertex(GmMesh* m, U32 a, U32 b, float t, int kind) {
    int found; U32 slot, idx[2]; float w[2]; Vec3 pa, pb, p; int v;
    if (gm_hcount > GM_HASH_SIZE / 2) return -1;
    slot = gm_hash_slot(a, b, &found);
    if (found) return gm_hval[slot];
    if (gm_less(m->pos[b], m->pos[a]) || (!gm_less(m->pos[a], m->pos[b]) && b < a)) { U32 s = a; a = b; b = s; }
    pa = m->pos[a]; pb = m->pos[b];
    if (kind == 0) { p.x = 0.5f * (pa.x + pb.x); p.y = 0.5f * (pa.y + pb.y); p.z = 0.5f * (pa.z + pb.z); }
    else { p.x = pa.x + (pb.x - pa.x) * t; p.y = pa.y + (pb.y - pa.y) * t; p.z = pa.z + (pb.z - pa.z) * t; }
    idx[0] = a; idx[1] = b; w[0] = 1.f - t; w[1] = t;
    v = gm_add_vertex(m, p, idx, w, 2);
    if (v < 0) return -1;
    gm_hval[slot] = (U16)v;
    GM_TRACE_V((U32)v, a, b, a, kind);
    return v;
}
static int gm_add_tri(GmMesh* m, U32 a, U32 b, U32 c, U16 aux, U32 parent) {
    U32 t = m->nt;
    if (t >= GM_MAXT) return -1;
    m->tri[t * 3] = (U16)a; m->tri[t * 3 + 1] = (U16)b; m->tri[t * 3 + 2] = (U16)c; m->aux[t] = aux;
    if (m->flag) m->flag[t] = m->flag[parent];
    m->nt = t + 1;
    GM_TRACE_T(t, parent); (void)parent;
    return (int)t;
}
static void gm_set_tri(GmMesh* m, U32 t, U32 a, U32 b, U32 c) {
    m->tri[t * 3] = (U16)a; m->tri[t * 3 + 1] = (U16)b; m->tri[t * 3 + 2] = (U16)c;
}
/* quick reject: triangle entirely outside the sphere's bounding box */
static int gm_far(const GmMesh* m, U32 t, Vec3 C, float R) {
    Vec3 a = m->pos[m->tri[t * 3]], b = m->pos[m->tri[t * 3 + 1]], c = m->pos[m->tri[t * 3 + 2]];
    if (a.x > C.x + R && b.x > C.x + R && c.x > C.x + R) return 1;
    if (a.x < C.x - R && b.x < C.x - R && c.x < C.x - R) return 1;
    if (a.y > C.y + R && b.y > C.y + R && c.y > C.y + R) return 1;
    if (a.y < C.y - R && b.y < C.y - R && c.y < C.y - R) return 1;
    if (a.z > C.z + R && b.z > C.z + R && c.z > C.z + R) return 1;
    if (a.z < C.z - R && b.z < C.z - R && c.z < C.z - R) return 1;
    return 0;
}

/* Where to split an edge that passes through the crater sphere. Edges reaching far outside
 * are cut where they cross the guard sphere (radius Rg > R); only what lies inside the guard
 * sphere is bisected. Parameter is measured in canonical direction. */
#define GM_GUARD_BAND 0.05f
static int gm_refine_vertex(GmMesh* m, U32 a, U32 b, Vec3 C, float Rg) {
    Vec3 pa, pb, d, f; float A, B, Cc, disc, t0, t1, t = 0.5f, lim = Rg + GM_GUARD_BAND; int ain, bin;
    if (gm_less(m->pos[b], m->pos[a]) || (!gm_less(m->pos[a], m->pos[b]) && b < a)) { U32 s = a; a = b; b = s; }
    pa = m->pos[a]; pb = m->pos[b];
    ain = gm_dist2(pa, C) <= lim * lim; bin = gm_dist2(pb, C) <= lim * lim;
    if (!(ain && bin)) {
        d = gm_sub(pb, pa); f = gm_sub(pa, C);
        A = gm_dot(d, d); B = 2.f * gm_dot(f, d); Cc = gm_dot(f, f) - Rg * Rg;
        disc = B * B - 4.f * A * Cc;
        if (A > 0.f && disc > 0.f) {
            disc = gm_sqrt(disc); t0 = (-B - disc) / (2.f * A); t1 = (-B + disc) / (2.f * A);
            t = ain ? t1 : t0;                     /* leaving: exit point; otherwise: entry point */
            if (!(t > 0.02f && t < 0.98f)) t = 0.5f;
        }
    }
    return gm_edge_vertex(m, a, b, t, t == 0.5f ? 0 : 3);
}

/* 1. local refinement. Returns 0 ok, -1 out of capacity. */
static int gm_refine(GmMesh* m, Vec3 C, float R, float h, GmStats* st) {
    float R2 = R * R, h2 = h * h, Rg = R + 0.75f * h;
    U32 pass;
    for (pass = 0; pass < GM_MAX_PASSES; pass++) {
        U32 nt0 = m->nt, t, changed = 0;
        /* a) sphere footprint strictly inside a triangle: insert the foot point */
        for (t = 0; t < nt0; t++) {
            U32 i0, i1, i2, idx[3]; Vec3 a, b, c, F; float w[3]; int v; U16 aux;
            if (gm_far(m, t, C, R)) continue;
            i0 = m->tri[t * 3]; i1 = m->tri[t * 3 + 1]; i2 = m->tri[t * 3 + 2];
            a = m->pos[i0]; b = m->pos[i1]; c = m->pos[i2];
            if (gm_seg_dist2(C, a, b) < R2 || gm_seg_dist2(C, b, c) < R2 || gm_seg_dist2(C, c, a) < R2) continue;
            /* canonical vertex order, so coincident (double-sided) faces get the identical foot point */
            idx[0] = i0; idx[1] = i1; idx[2] = i2;
            if (gm_less(m->pos[idx[1]], m->pos[idx[0]])) { U32 s = idx[0]; idx[0] = idx[1]; idx[1] = s; }
            if (gm_less(m->pos[idx[2]], m->pos[idx[1]])) { U32 s = idx[1]; idx[1] = idx[2]; idx[2] = s; }
            if (gm_less(m->pos[idx[1]], m->pos[idx[0]])) { U32 s = idx[0]; idx[0] = idx[1]; idx[1] = s; }
            F = gm_closest_on_tri(C, m->pos[idx[0]], m->pos[idx[1]], m->pos[idx[2]], w);
            if (gm_dist2(C, F) >= R2) continue;
            if (w[0] <= 0.f || w[1] <= 0.f || w[2] <= 0.f) continue;   /* foot on the border: edges handle it */
            v = gm_add_vertex(m, F, idx, w, 3);
            if (v < 0) return -1;
            GM_TRACE_V((U32)v, i0, i1, i2, 1);
            aux = m->aux[t];
            gm_set_tri(m, t, i0, i1, (U32)v);
            if (gm_add_tri(m, i1, i2, (U32)v, aux, t) < 0 || gm_add_tri(m, i2, i0, (U32)v, aux, t) < 0) return -1;
            st->feet++; changed = 1;
        }
        /* b) split every edge that is longer than h and passes through the sphere */
        gm_hash_clear();
        nt0 = m->nt;
        for (t = 0; t < nt0; t++) {
            U32 v[3], mid[3] = { 0, 0, 0 }, mask = 0, e; U16 aux;
            if (gm_far(m, t, C, R)) continue;
            v[0] = m->tri[t * 3]; v[1] = m->tri[t * 3 + 1]; v[2] = m->tri[t * 3 + 2];
            if (v[0] == v[1] || v[1] == v[2] || v[2] == v[0]) continue;
            for (e = 0; e < 3; e++) {
                Vec3 a = m->pos[v[e]], b = m->pos[v[(e + 1) % 3]];
                if (gm_dist2(a, b) > h2 && gm_seg_dist2(C, a, b) < R2) mask |= 1u << e;
            }
            if (!mask) continue;
            for (e = 0; e < 3; e++) if (mask & (1u << e)) {
                int nv = gm_refine_vertex(m, v[e], v[(e + 1) % 3], C, Rg);
                if (nv < 0) return -1;
                mid[e] = (U32)nv;
            }
            aux = m->aux[t];
#define GM_T(a, b, c) do { if (gm_add_tri(m, a, b, c, aux, t) < 0) return -1; } while (0)
            if (mask == 1 || mask == 2 || mask == 4) {
                e = mask == 1 ? 0u : (mask == 2 ? 1u : 2u);
                gm_set_tri(m, t, v[e], mid[e], v[(e + 2) % 3]); GM_T(mid[e], v[(e + 1) % 3], v[(e + 2) % 3]);
            } else if (mask == 7) {
                gm_set_tri(m, t, v[0], mid[0], mid[2]); GM_T(mid[0], v[1], mid[1]); GM_T(mid[2], mid[1], v[2]); GM_T(mid[0], mid[1], mid[2]);
            } else {
                /* two edges split: corner triangle at the shared vertex plus a quad. The quad's
                 * diagonal is chosen from the positions only, so coincident (double-sided) faces
                 * are triangulated identically. n = the edge that is NOT split. */
                U32 n = mask == 3 ? 2u : (mask == 6 ? 0u : 1u);
                U32 p0 = v[n], p1 = v[(n + 1) % 3], apex = v[(n + 2) % 3], mA = mid[(n + 1) % 3], mB = mid[(n + 2) % 3];
                gm_set_tri(m, t, mA, apex, mB);
                if (gm_less(m->pos[p0], m->pos[p1])) { GM_T(p0, p1, mA); GM_T(p0, mA, mB); }
                else { GM_T(p0, p1, mB); GM_T(p1, mA, mB); }
            }
            st->splits++; changed = 1;
        }
        if (!changed) return 0;
    }
    return 0;
}

/* 2. exact cut along the sphere surface. Returns 0 ok, -1 out of capacity.
 * Vertex classes: 1 inside, 0 on the surface (within GM_EPS), -1 outside.
 * Only edges that run from class 1 to class -1 are cut. */
static int gm_class(Vec3 p, Vec3 C, float R) {
    float d2 = gm_dist2(p, C), lo = R - GM_EPS, hi = R + GM_EPS;
    return d2 > hi * hi ? -1 : (d2 < lo * lo ? 1 : 0);
}
/* vertices that the bowl projection moves: inside the sphere AND in its ground-side half */
static int gm_inside(Vec3 p, Vec3 C, float R, Vec3 Q) {
    return gm_class(p, C, R) > 0 && gm_dot(gm_sub(p, C), gm_sub(Q, C)) < 0.f;
}
static int gm_cut_vertex(GmMesh* m, U32 in, U32 out, Vec3 C, float R) {
    /* parameter along the canonical direction where the edge meets the sphere */
    U32 a = in, b = out; Vec3 pa, pb, d, f; float A, B, Cc, disc, t0, t1, t;
    if (gm_less(m->pos[b], m->pos[a]) || (!gm_less(m->pos[a], m->pos[b]) && b < a)) { a = out; b = in; }
    pa = m->pos[a]; pb = m->pos[b]; d = gm_sub(pb, pa); f = gm_sub(pa, C);
    A = gm_dot(d, d); B = 2.f * gm_dot(f, d); Cc = gm_dot(f, f) - R * R;
    disc = B * B - 4.f * A * Cc;
    if (A <= 0.f) return -1;
    disc = disc > 0.f ? gm_sqrt(disc) : 0.f;
    t0 = (-B - disc) / (2.f * A); t1 = (-B + disc) / (2.f * A);
    t = (a == in) ? t1 : t0;       /* leaving the sphere: far root, entering: near root */
    if (t < 0.001f) t = 0.001f; else if (t > 0.999f) t = 0.999f;
    return gm_edge_vertex(m, a, b, t, 2);
}
static int gm_cut(GmMesh* m, Vec3 C, float R, GmStats* st) {
    U32 nt0 = m->nt, t;
    gm_hash_clear();
    for (t = 0; t < nt0; t++) {
        U32 v[3], k, ncut = 0, e0 = 0, e1 = 0, o; int cl[3], x[3] = { 0, 0, 0 }; U16 aux;
        if (gm_far(m, t, C, R + GM_EPS)) continue;
        for (k = 0; k < 3; k++) { v[k] = m->tri[t * 3 + k]; cl[k] = gm_class(m->pos[v[k]], C, R); }
        for (k = 0; k < 3; k++) {
            U32 j = (k + 1) % 3;
            if (cl[k] * cl[j] >= 0) continue;                 /* not an inside-outside edge */
            x[k] = cl[k] > 0 ? gm_cut_vertex(m, v[k], v[j], C, R) : gm_cut_vertex(m, v[j], v[k], C, R);
            if (x[k] < 0) return -1;
            if (ncut++ == 0) e0 = k; else e1 = k;
        }
        if (!ncut) continue;
        aux = m->aux[t];
        if (ncut == 1) {
            /* inside - on surface - outside triangle: split through the on-surface vertex */
            U32 w = v[(e0 + 2) % 3];
            gm_set_tri(m, t, v[e0], (U32)x[e0], w);
            if (gm_add_tri(m, (U32)x[e0], v[(e0 + 1) % 3], w, aux, t) < 0) return -1;
        } else {
            /* the two cut edges share vertex o */
            U32 xa, xb;
            if (e0 == 0 && e1 == 1) o = 1; else if (e0 == 1 && e1 == 2) o = 2; else o = 0;
            xa = (U32)x[o]; xb = (U32)x[(o + 2) % 3];          /* on edges (o,o+1) and (o+2,o) */
            gm_set_tri(m, t, v[o], xa, xb);
            if (gm_less(m->pos[v[(o + 1) % 3]], m->pos[v[(o + 2) % 3]])) {   /* canonical diagonal, see gm_refine */
                if (gm_add_tri(m, xa, v[(o + 1) % 3], xb, aux, t) < 0 ||
                    gm_add_tri(m, v[(o + 1) % 3], v[(o + 2) % 3], xb, aux, t) < 0) return -1;
            } else {
                if (gm_add_tri(m, xa, v[(o + 1) % 3], v[(o + 2) % 3], aux, t) < 0 ||
                    gm_add_tri(m, xa, v[(o + 2) % 3], xb, aux, t) < 0) return -1;
            }
        }
        st->cuts++;
    }
    return 0;
}

/* 3. projection of the inside vertices from the pole Q onto the sphere.
 * shade: colour multiplier at the crater centre (1 = unchanged). */
static void gm_project(GmMesh* m, Vec3 C, float R, Vec3 Q, float shade, U8* moved, GmStats* st) {
    Vec3 qc = gm_sub(Q, C);
    U32 i, k;
    for (i = 0; i < m->nv; i++) {
        Vec3 p = m->pos[i], u; float uu, t, s, f;
        if (gm_orphan[i] == 2 || gm_clamp[i] == 0.f || !gm_inside(p, C, R, Q)) continue;   /* removed by a hole / stays where it is */
        u = gm_sub(p, Q); uu = gm_dot(u, u);
        if (uu < 1e-8f) continue;
        t = -2.f * gm_dot(qc, u) / uu;
        if (!(t > 0.f)) continue;
        if (gm_clamp[i] > 0.f) {                       /* the material ends before the sphere: stop there */
            float tc = 1.f + gm_clamp[i] / gm_sqrt(uu);
            if (tc < t) t = tc;
        }
        s = gm_sqrt(gm_dist2(p, C)) / R;
        m->pos[i].x = Q.x + u.x * t; m->pos[i].y = Q.y + u.y * t; m->pos[i].z = Q.z + u.z * t;
        f = shade + (1.f - shade) * s * s;
        for (k = 0; k < m->nb; k++) {
            U8* c = m->ba[k] + i * 4;
            c[0] = (U8)((float)c[0] * f); c[1] = (U8)((float)c[1] * f); c[2] = (U8)((float)c[2] * f);
        }
        if (m->normalAttr >= 0) {
            float* n = m->fa[m->normalAttr] + i * 3; Vec3 d = gm_sub(C, m->pos[i]); float l = gm_sqrt(gm_dot(d, d));
            if (l > 1e-6f) { n[0] = d.x / l; n[1] = d.y / l; n[2] = d.z / l; }
        }
        if (moved) moved[i] = 1;
        st->moved++;
    }
}

/* 4. the bowl gets its own material. Every triangle that has a moved vertex becomes `bowlAux`
 * and is texture-mapped by position: uv = (M[0..2].p + M[3], M[4..6].p + M[7]). Vertices it shares
 * with untouched triangles are duplicated, so the surrounding surface keeps its own mapping. */
static U16 gm_dup[GM_MAXV];
static int gm_bowl_material(GmMesh* m, const U8* moved, U16 bowlAux, int uvAttr, const float* M, GmStats* st) {
    U32 t, k, nt = m->nt, nv0 = m->nv;
    memset(gm_dup, 0xFF, sizeof(gm_dup));
    for (t = 0; t < nt; t++) {
        U32 v[3];
        v[0] = m->tri[t * 3]; v[1] = m->tri[t * 3 + 1]; v[2] = m->tri[t * 3 + 2];
        if (!((v[0] < nv0 && moved[v[0]]) || (v[1] < nv0 && moved[v[1]]) || (v[2] < nv0 && moved[v[2]]))) continue;
        m->aux[t] = bowlAux; st->bowlTris++;
        for (k = 0; k < 3; k++) {
            U32 x = v[k]; Vec3 p;
            if (!moved[x]) {
                if (gm_dup[x] == 0xFFFFu) {
                    U32 idx[1]; float w[1]; int nvx;
                    idx[0] = x; w[0] = 1.f;
                    nvx = gm_add_vertex(m, m->pos[x], idx, w, 1);
                    if (nvx < 0) return -1;
                    GM_TRACE_V((U32)nvx, x, x, x, 4);
                    gm_dup[x] = (U16)nvx;
                }
                x = gm_dup[x]; m->tri[t * 3 + k] = (U16)x;
            }
            if (uvAttr >= 0) {
                float* uv = m->fa[uvAttr] + x * m->fdim[uvAttr];
                p = m->pos[x];
                uv[0] = M[0] * p.x + M[1] * p.y + M[2] * p.z + M[3];
                uv[1] = M[4] * p.x + M[5] * p.y + M[6] * p.z + M[7];
            }
        }
    }
    return 0;
}

/* 2b. holes: flagged (thin) triangles that lie inside the sphere are removed.
 * gm_orphan[v] = 2 afterwards for vertices that only belonged to removed triangles. */
static void gm_delete_holes(GmMesh* m, Vec3 C, float R, GmStats* st) {
    U32 t, n = 0, i;
    memset(gm_orphan, 0, m->nv);
    if (!m->flag) return;
    for (t = 0; t < m->nt; t++) {
        int kill = 0;
        if (m->flag[t] && !gm_far(m, t, C, R)) {
            int c0 = gm_class(m->pos[m->tri[t * 3]], C, R), c1 = gm_class(m->pos[m->tri[t * 3 + 1]], C, R), c2 = gm_class(m->pos[m->tri[t * 3 + 2]], C, R);
            kill = c0 >= 0 && c1 >= 0 && c2 >= 0 && (c0 + c1 + c2) > 0;      /* nothing outside, something inside */
        }
        if (kill) { st->holeTris++; gm_orphan[m->tri[t * 3]] = gm_orphan[m->tri[t * 3 + 1]] = gm_orphan[m->tri[t * 3 + 2]] = 2; continue; }
        if (n != t) {
            m->tri[n * 3] = m->tri[t * 3]; m->tri[n * 3 + 1] = m->tri[t * 3 + 1]; m->tri[n * 3 + 2] = m->tri[t * 3 + 2];
            m->aux[n] = m->aux[t]; m->flag[n] = m->flag[t];
        }
        n++;
    }
    m->nt = n;
    if (st->holeTris) for (i = 0; i < m->nt * 3u; i++) gm_orphan[m->tri[i]] = 1;
}

/* ray (o, unit d) against triangle abc: distance along the ray, or -1 */
static float gm_ray_tri(Vec3 o, Vec3 d, Vec3 a, Vec3 b, Vec3 c) {
    Vec3 e1 = gm_sub(b, a), e2 = gm_sub(c, a), pv = gm_cross(d, e2), tv, qv; float det = gm_dot(e1, pv), inv, u, v;
    if (det > -1e-9f && det < 1e-9f) return -1.f;
    inv = 1.f / det; tv = gm_sub(o, a); u = gm_dot(tv, pv) * inv;
    if (u < -0.001f || u > 1.001f) return -1.f;
    qv = gm_cross(tv, e1); v = gm_dot(d, qv) * inv;
    if (v < -0.001f || u + v > 1.002f) return -1.f;
    return gm_dot(e2, qv) * inv;
}
/* Flag sheets near the sphere - they get a hole instead of a bowl (call before gm_crater, m->flag
 * must be set):
 *   - triangles whose surface type is a sheet (sheetSurf[aux & 255], may be NULL): wire fences,
 *     railings, hedges - what bullets fly through
 *   - with thickness > 0: triangles with a back face of the same mesh less than `thickness` behind
 *     them. Meant for double-sided faces and boards a few centimetres thick. NOT for walls and
 *     slabs: a hole through a wall of a GTA building shows the void inside the hollow shell.
 * flip = 1 for the GTA collision winding. Returns the number of flagged triangles. */
#define GM_MAX_NEAR 4000u
static U16 gm_near[GM_MAX_NEAR];
static Vec3 gm_tri_unit_normal(const GmMesh* m, U32 t, int flip) {
    Vec3 a = m->pos[m->tri[t * 3]], b = m->pos[m->tri[t * 3 + 1]], c = m->pos[m->tri[t * 3 + 2]];
    Vec3 n = flip ? gm_cross(gm_sub(c, a), gm_sub(b, a)) : gm_cross(gm_sub(b, a), gm_sub(c, a)); float l = gm_sqrt(gm_dot(n, n));
    if (l > 1e-12f) { n.x /= l; n.y /= l; n.z /= l; } else { n.x = n.y = n.z = 0.f; }
    return n;
}
static U32 gm_mark_thin(GmMesh* m, Vec3 C, float R, float thickness, int flip, const U8* sheetSurf) {
    U32 t, i, j, k, nn = 0, marked = 0;
    memset(m->flag, 0, m->nt);
    for (t = 0; t < m->nt && nn < GM_MAX_NEAR; t++) if (!gm_far(m, t, C, R + thickness)) gm_near[nn++] = (U16)t;
    for (i = 0; i < nn; i++) {
        Vec3 a, b, c, n, cen, foot, smp[2]; float w[3]; int thin = 0;
        t = gm_near[i];
        if (gm_far(m, t, C, R)) continue;
        if (sheetSurf && sheetSurf[m->aux[t] & 0xFFu]) { m->flag[t] = 1; marked++; continue; }
        if (!(thickness > 0.f)) continue;
        a = m->pos[m->tri[t * 3]]; b = m->pos[m->tri[t * 3 + 1]]; c = m->pos[m->tri[t * 3 + 2]];
        n = gm_tri_unit_normal(m, t, flip);
        if (n.x == 0.f && n.y == 0.f && n.z == 0.f) continue;
        cen.x = (a.x + b.x + c.x) / 3.f; cen.y = (a.y + b.y + c.y) / 3.f; cen.z = (a.z + b.z + c.z) / 3.f;
        foot = gm_closest_on_tri(C, a, b, c, w);
        /* look behind the triangle from the point nearest the blast (nudged inward) and from its centre */
        smp[0].x = foot.x * 0.9f + cen.x * 0.1f; smp[0].y = foot.y * 0.9f + cen.y * 0.1f; smp[0].z = foot.z * 0.9f + cen.z * 0.1f;
        smp[1] = cen;
        for (k = 0; k < 2 && !thin; k++) {
            Vec3 o, d;
            o.x = smp[k].x + n.x * 0.03f; o.y = smp[k].y + n.y * 0.03f; o.z = smp[k].z + n.z * 0.03f;
            d.x = -n.x; d.y = -n.y; d.z = -n.z;
            for (j = 0; j < nn && !thin; j++) {
                U32 u = gm_near[j]; float dist; Vec3 nu;
                if (u == t) continue;
                dist = gm_ray_tri(o, d, m->pos[m->tri[u * 3]], m->pos[m->tri[u * 3 + 1]], m->pos[m->tri[u * 3 + 2]]);
                if (dist < 0.f || dist > thickness + 0.03f) continue;
                nu = gm_tri_unit_normal(m, u, flip);
                if (gm_dot(nu, n) < -0.5f) thin = 1;          /* a back face: this is a slab */
            }
        }
        if (thin) { m->flag[t] = 1; marked++; }
    }
    return marked;
}

/* Surface probe: closest point of the mesh to E, and which way the surface around the blast faces.
 * nsum receives the weighted normal sum of the triangles within `radius` of E.
 *
 *  - A triangle the blast cannot see does not count: the hidden bottom and far side of the big
 *    boxes buildings are made of, the back of a wall, ground beyond the crest of a hill.
 *    (The hidden bottom face of a casino block used to count and turned craters into domes.)
 *  - A triangle counts with the side that faces the blast, whatever its winding. GTA's collision
 *    does not care about winding and many models have all or some triangles reversed; trusting
 *    it opens the crater into the ground. Only for a blast sitting in the triangle's plane the
 *    winding decides (flip = 1 for the collision convention: (C-A)x(B-A) faces outward); the
 *    weight changes over gradually within GM_SIDE_EPS behind the plane, so nothing jumps.
 *  - A triangle counts with at most radius^2 of area: of a 90 m wall only the part near the blast
 *    matters. It counts by how squarely the blast faces it - the ground under the blast fully,
 *    a kerb top seen edge-on hardly - and by how close it is: a grenade lying on the ground digs
 *    into the ground, whatever hangs 2 m above it.
 *  - viewer (may be NULL): for rockets and shells, which explode right AT the surface they hit -
 *    in front of it or a little inside, there is no telling. The face that was hit is the one
 *    the shot could see, so within GM_VIEW_BAND of a triangle's plane the viewer's side counts. */
#define GM_VIEW_BAND 0.30f
#define GM_SIDE_EPS 0.03f
#define GM_MAX_PROBE 65535u
static U16 gm_pt[GM_MAX_PROBE];
static float gm_probe_weight;        /* running sum of |weight|: how much of nsum cancelled out (diagnostics) */
static float gm_pb[GM_MAX_PROBE * 6u];
static U32 gm_probe_collect(const GmMesh* m, Vec3 E, float radius) {
    U32 t, nn = 0;
    for (t = 0; t < m->nt && nn < GM_MAX_PROBE; t++) {
        Vec3 a, b, c; float* bb;
        if (gm_far(m, t, E, radius)) continue;
        a = m->pos[m->tri[t * 3]]; b = m->pos[m->tri[t * 3 + 1]]; c = m->pos[m->tri[t * 3 + 2]];
        bb = gm_pb + nn * 6u;
        bb[0] = a.x < b.x ? (a.x < c.x ? a.x : c.x) : (b.x < c.x ? b.x : c.x); bb[3] = a.x > b.x ? (a.x > c.x ? a.x : c.x) : (b.x > c.x ? b.x : c.x);
        bb[1] = a.y < b.y ? (a.y < c.y ? a.y : c.y) : (b.y < c.y ? b.y : c.y); bb[4] = a.y > b.y ? (a.y > c.y ? a.y : c.y) : (b.y > c.y ? b.y : c.y);
        bb[2] = a.z < b.z ? (a.z < c.z ? a.z : c.z) : (b.z < c.z ? b.z : c.z); bb[5] = a.z > b.z ? (a.z > c.z ? a.z : c.z) : (b.z > c.z ? b.z : c.z);
        gm_pt[nn++] = (U16)t;
    }
    return nn;
}
/* is the straight way from E to v blocked by one of the collected triangles (other than number `self`)? */
static int gm_probe_blocked(const GmMesh* m, U32 nn, U32 self, Vec3 E, Vec3 v) {
    Vec3 d = gm_sub(v, E), lo, hi; float L = gm_sqrt(gm_dot(d, d)); U32 j;
    if (L < 0.06f) return 0;
    d.x /= L; d.y /= L; d.z /= L;
    lo.x = (E.x < v.x ? E.x : v.x) - 0.01f; lo.y = (E.y < v.y ? E.y : v.y) - 0.01f; lo.z = (E.z < v.z ? E.z : v.z) - 0.01f;
    hi.x = (E.x > v.x ? E.x : v.x) + 0.01f; hi.y = (E.y > v.y ? E.y : v.y) + 0.01f; hi.z = (E.z > v.z ? E.z : v.z) + 0.01f;
    for (j = 0; j < nn; j++) {
        const float* bb = gm_pb + j * 6u; U32 u = gm_pt[j]; float tt;
        if (j == self || bb[0] > hi.x || bb[3] < lo.x || bb[1] > hi.y || bb[4] < lo.y || bb[2] > hi.z || bb[5] < lo.z) continue;
        tt = gm_ray_tri(E, d, m->pos[m->tri[u * 3]], m->pos[m->tri[u * 3 + 1]], m->pos[m->tri[u * 3 + 2]]);
        if (tt > 0.02f && tt < L - 0.03f) return 1;
    }
    return 0;
}
static void gm_probe(const GmMesh* m, Vec3 E, float radius, int flip, const Vec3* viewer, float* bestD2, Vec3* best, Vec3* nsum) {
    U32 t, i, k, nn = gm_probe_collect(m, E, radius + 0.35f); float r2 = radius * radius;
    for (i = 0; i < nn; i++) {
        Vec3 P[3], p, n, v; float w[3], d2, dd, f, len, sd; U32 nz = 0, kz = 0;
        t = gm_pt[i];
        if (gm_far(m, t, E, radius)) continue;
        P[0] = m->pos[m->tri[t * 3]]; P[1] = m->pos[m->tri[t * 3 + 1]]; P[2] = m->pos[m->tri[t * 3 + 2]];
        p = gm_closest_on_tri(E, P[0], P[1], P[2], w); d2 = gm_dist2(E, p);
        if (d2 >= r2) continue;
        n = flip ? gm_cross(gm_sub(P[2], P[0]), gm_sub(P[1], P[0])) : gm_cross(gm_sub(P[1], P[0]), gm_sub(P[2], P[0]));
        len = gm_sqrt(gm_dot(n, n));                                 /* twice the area */
        if (len < 1e-9f) continue;
        if (d2 < *bestD2) { *bestD2 = d2; *best = p; }
        dd = gm_sqrt(d2);
        f = 1.f - dd / radius; f *= f;                 /* fades out towards the probe radius: no jumps */
        f *= 0.09f / ((dd + 0.3f) * (dd + 0.3f));      /* what is close to the blast decides */
        if (len > 2.f * r2) f *= 2.f * r2 / len;
        /* Can the blast see the triangle? Look at its nearest point; if that is on the border,
         * at a point a little way inside, where a neighbouring face cannot be in the way. */
        v = p;
        for (k = 0; k < 3; k++) if (w[k] < 1e-4f) { nz++; kz = k; }
        if (nz == 1u) {
            Vec3 e = gm_sub(P[(kz + 2u) % 3u], P[(kz + 1u) % 3u]), in = gm_sub(P[kz], p); float el2 = gm_dot(e, e), h, s, q;
            if (el2 > 1e-12f) { q = gm_dot(in, e) / el2; in.x -= e.x * q; in.y -= e.y * q; in.z -= e.z * q; }
            h = gm_sqrt(gm_dot(in, in));
            if (h > 1e-6f) { s = (0.4f * h < 0.15f ? 0.4f * h : 0.15f) / h; v.x += in.x * s; v.y += in.y * s; v.z += in.z * s; }
        } else if (nz >= 2u) {
            Vec3 g; float gl, s;
            g.x = (P[0].x + P[1].x + P[2].x) / 3.f - p.x; g.y = (P[0].y + P[1].y + P[2].y) / 3.f - p.y; g.z = (P[0].z + P[1].z + P[2].z) / 3.f - p.z;
            gl = gm_sqrt(gm_dot(g, g));
            if (gl > 1e-6f) { s = (0.4f * gl < 0.15f ? 0.4f * gl : 0.15f) / gl; v.x += g.x * s; v.y += g.y * s; v.z += g.z * s; }
        }
        if (gm_probe_blocked(m, nn, i, E, v)) continue;
        sd = gm_dot(n, gm_sub(E, P[0])) / len;         /* height of the blast above the triangle's plane */
        /* seen edge-on it counts little, seen face-on fully (cosine of the angle of incidence;
         * the 5 cm keep a blast lying right on the surface at full weight) */
        f *= ((sd < 0.f ? -sd : sd) + 0.05f) / (dd + 0.05f);
        if (viewer && sd > -GM_VIEW_BAND && sd < GM_VIEW_BAND) {
            /* impact right at the surface: the side the shot came from counts */
            float sv = gm_dot(n, gm_sub(*viewer, P[0])) / len;
            if (sv < -0.25f) f = -f; else if (sv <= 0.25f && sd < 0.f) f *= sd > -2.f * GM_SIDE_EPS ? 1.f + sd / GM_SIDE_EPS : -1.f;
        } else
        if (sd < 0.f) f *= sd > -2.f * GM_SIDE_EPS ? 1.f + sd / GM_SIDE_EPS : -1.f;   /* blast behind it: the other side counts */
        nsum->x += n.x * f; nsum->y += n.y * f; nsum->z += n.z * f;
        gm_probe_weight += (f < 0.f ? -f : f) * len;
    }
}

/* ---- Solid or not: how far a crater goes -----------------------------------------------------
 * The bowl projection pushes every vertex inside the sphere away from the pole Q until it lies on
 * the sphere. That is right as long as there is solid material all the way. Where the material
 * ENDS before the sphere is reached - a roof, a bridge deck, a wall, the corner of a block, a pole -
 * the vertex must stop there: beyond is air. (Pushed on regardless, it drags a rock "tunnel"
 * through the air under the bridge.)
 *
 * gm_break follows, for every vertex inside the sphere, the ray it would travel on:
 *   - a face looking towards the ray starts solid material, a face looking away ends it. Only
 *     UNTOUCHED faces count; crater rock from earlier blasts is folded and says nothing reliable.
 *     Rock itself is always solid at its own surface (and can be broken through: blast a bridge
 *     twice).
 *   - gm_clamp[v] = -1: solid up to the sphere, the vertex goes there (the bowl)
 *                 >= 0: the material ends after this many metres, the vertex stops there.
 *                       0 = it does not move at all (far side of a wall, a thin sheet).
 * Triangles whose vertices all stop short lie flat on the far surface: they are the opening and
 * are flagged for removal, and so is the far side behind them. Triangles with some vertices on
 * the sphere and some stopped form the wall of the hole.
 *
 * Needs trustworthy winding (visible meshes drawn with back-face culling). flip as in gm_probe.
 * m->flag must be set. Returns the number of triangles flagged (flag value 2); gm_shell_p /
 * gm_shell_d then hold a point 25 cm behind the opening nearest the crater's axis and the
 * direction there, for the caller's "is there really a space behind it" checks. */
#define GM_SHELL_MAX 2.5f
#define GM_SHELL_HITS 48u
#define GM_AT_MAX 96u
#define GM_AT_EPS 0.004f        /* a face this close to a vertex is "at" it */
#define GM_EDGE_ON 0.002f       /* |normal . ray| below this: the face is seen exactly edge-on */
static Vec3 gm_shell_p, gm_shell_d; static float gm_shell_far;     /* ... and how thick the material is there */
static int gm_break_failed;                             /* gm_break ran out of capacity: the mesh is unusable */
static U32 gm_solid_aux = 0xFFFFFFFFu;                 /* material index of crater rock, or none */
#define GM_POS_HASH 65536u
static U16 gm_ph[GM_POS_HASH];                         /* position -> first vertex with that position */
static U16 gm_rep[GM_MAXV];
static U32 gm_ic[GM_MAXV + 2u];                        /* per position: where its triangle list starts */
static U16 gm_il[GM_MAX_PROBE * 3u];                   /* ... the list: indices into gm_pt */
static Vec3 gm_pnrm[GM_MAX_PROBE];                     /* unit normals of the collected triangles */
static U8 gm_prock[GM_MAX_PROBE];
static U32 gm_pos_slot(const GmMesh* m, U32 i) {
    const U32* w = (const U32*)(const void*)&m->pos[i]; Vec3 p = m->pos[i]; U32 k;
    U32 x = p.x == 0.f ? 0u : w[0], y = p.y == 0.f ? 0u : w[1], z = p.z == 0.f ? 0u : w[2];      /* -0 == +0 */
    U32 hsh = (x * 0x9E3779B1u) ^ (y * 0x85EBCA77u) ^ (z * 0xC2B2AE3Du); hsh ^= hsh >> 15;
    for (k = hsh & (GM_POS_HASH - 1u); ; k = (k + 1u) & (GM_POS_HASH - 1u)) {
        U32 j = gm_ph[k];
        if (j == 0xFFFFu) { gm_ph[k] = (U16)i; return i; }
        if (m->pos[j].x == p.x && m->pos[j].y == p.y && m->pos[j].z == p.z) return j;
    }
}
/* A grid for the walks. Every ray comes from the pole Q, so seen from Q a ray is a point: with
 * u,v = the direction from Q projected onto a plane across the crater's axis, a ray can only meet
 * triangles whose own u,v extent contains it. The collected triangles are sorted into the cells of
 * a GM_GRID x GM_GRID grid over u,v; triangles too close to Q, or covering too many cells, go into
 * a list that is always looked at. */
#define GM_GRID 24u
#define GM_GRID_POOL 400000u
static U16 gm_gl[GM_GRID_POOL]; static U32 gm_gs[GM_GRID * GM_GRID + 2u];     /* cell lists: indices into gm_pt */
static U16 gm_gov[GM_MAX_PROBE]; static U32 gm_gov_n;                           /* always looked at */
static U16 gm_gall[GM_MAX_PROBE];                                               /* 0, 1, 2, ...: used when there is no grid */
static int gm_grid_ok; static Vec3 gm_gq, gm_ga, gm_ge1, gm_ge2;
static int gm_grid_uv(Vec3 p, float* u, float* v) {
    Vec3 w = gm_sub(p, gm_gq); float dz = gm_dot(w, gm_ga);
    if (dz < 0.2f) return 0;
    *u = gm_dot(w, gm_ge1) / dz; *v = gm_dot(w, gm_ge2) / dz;
    return 1;
}
static U32 gm_grid_idx(float u) {
    float f = (u + 1.2f) * ((float)GM_GRID / 2.4f);
    return f <= 0.f ? 0u : (f >= (float)(GM_GRID - 1u) ? GM_GRID - 1u : (U32)f);
}
/* the triangles a ray through p can meet (and those within about 13 cm of p): *n of them, not counting gm_gov */
static const U16* gm_grid_at(Vec3 p, U32 nn, U32* n) {
    float u, v; U32 c;
    if (!gm_grid_ok || !gm_grid_uv(p, &u, &v)) { *n = nn; return gm_gall; }
    c = gm_grid_idx(v) * GM_GRID + gm_grid_idx(u);
    *n = gm_gs[c + 1u] - gm_gs[c];
    return gm_gl + gm_gs[c];
}
static void gm_grid_build(const GmMesh* ref, U32 nn, Vec3 C, Vec3 Q) {
    static U8 lo[GM_MAX_PROBE][2], hi[GM_MAX_PROBE][2]; U32 i, k, total = 0; float l;
    gm_grid_ok = 0; gm_gov_n = 0;
    gm_gq = Q; gm_ga = gm_sub(C, Q); l = gm_sqrt(gm_dot(gm_ga, gm_ga));
    if (l < 1e-6f) return;
    gm_ga.x /= l; gm_ga.y /= l; gm_ga.z /= l;
    gm_ge1.x = gm_ga.y; gm_ge1.y = -gm_ga.x; gm_ge1.z = 0.f; l = gm_sqrt(gm_dot(gm_ge1, gm_ge1));
    if (l < 0.1f) { gm_ge1.x = 1.f; gm_ge1.y = 0.f; gm_ge1.z = 0.f; l = gm_sqrt(1.f - gm_ga.x * gm_ga.x); gm_ge1.x -= gm_ga.x * gm_ga.x; gm_ge1.y -= gm_ga.x * gm_ga.y; gm_ge1.z -= gm_ga.x * gm_ga.z; }
    gm_ge1.x /= l; gm_ge1.y /= l; gm_ge1.z /= l; gm_ge2 = gm_cross(gm_ga, gm_ge1);
    memset(gm_gs, 0, sizeof(gm_gs));
    for (i = 0; i < nn; i++) {
        U32 t = gm_pt[i]; float u0 = 1e9f, u1 = -1e9f, v0 = 1e9f, v1 = -1e9f, dzmin = 1e9f, mg; int ok = 1; U32 a, b, c, d;
        for (k = 0; k < 3u && ok; k++) {
            Vec3 w = gm_sub(ref->pos[ref->tri[t * 3 + k]], Q); float dz = gm_dot(w, gm_ga), u, v;
            if (dz < 0.3f) { ok = 0; break; }
            u = gm_dot(w, gm_ge1) / dz; v = gm_dot(w, gm_ge2) / dz;
            if (u < u0) u0 = u; if (u > u1) u1 = u; if (v < v0) v0 = v; if (v > v1) v1 = v; if (dz < dzmin) dzmin = dz;
        }
        lo[i][0] = 255;
        if (ok) {
            mg = 0.16f / dzmin + 0.01f;
            a = gm_grid_idx(u0 - mg); b = gm_grid_idx(u1 + mg); c = gm_grid_idx(v0 - mg); d = gm_grid_idx(v1 + mg);
            if ((b - a + 1u) * (d - c + 1u) <= 64u) { lo[i][0] = (U8)a; hi[i][0] = (U8)b; lo[i][1] = (U8)c; hi[i][1] = (U8)d; total += (b - a + 1u) * (d - c + 1u); }
        }
        if (lo[i][0] == 255) gm_gov[gm_gov_n++] = (U16)i;
        else { U32 x, y; for (y = lo[i][1]; y <= hi[i][1]; y++) for (x = lo[i][0]; x <= hi[i][0]; x++) gm_gs[y * GM_GRID + x + 2u]++; }
    }
    if (total > GM_GRID_POOL) { gm_gov_n = 0; return; }
    for (i = 0; i < GM_GRID * GM_GRID; i++) gm_gs[i + 2u] += gm_gs[i + 1u];
    for (i = 0; i < nn; i++) if (lo[i][0] != 255) { U32 x, y; for (y = lo[i][1]; y <= hi[i][1]; y++) for (x = lo[i][0]; x <= hi[i][0]; x++) gm_gl[gm_gs[y * GM_GRID + x + 1u]++] = (U16)i; }
    gm_grid_ok = 1;
}
/* collect the triangles near the sphere of mesh `ref`, with normals, and sort them into the grid
 * for rays from Q; returns how many */
static U32 gm_break_collect(const GmMesh* ref, Vec3 C, float R, Vec3 Q, int flip) {
    U32 nn = gm_probe_collect(ref, C, R + 0.05f), i;
    for (i = 0; i < nn; i++) {
        U32 t = gm_pt[i]; Vec3 a = ref->pos[ref->tri[t * 3]], b = ref->pos[ref->tri[t * 3 + 1]], c = ref->pos[ref->tri[t * 3 + 2]];
        Vec3 n = flip ? gm_cross(gm_sub(c, a), gm_sub(b, a)) : gm_cross(gm_sub(b, a), gm_sub(c, a)); float l = gm_sqrt(gm_dot(n, n));
        if (l > 1e-12f) { n.x /= l; n.y /= l; n.z /= l; } else { n.x = n.y = n.z = 0.f; }
        gm_pnrm[i] = n; gm_prock[i] = (U8)((U32)ref->aux[t] == gm_solid_aux);
        gm_gall[i] = (U16)i;
    }
    gm_grid_build(ref, nn, C, Q);
    return nn;
}
/* One vertex at p: how far does it go? inc[0..ninc) = the collected triangles of `ref` that touch p.
 * Returns -1 (to the sphere) or the distance after which the material ends. */
/* Sheet mode, for models drawn without back-face culling: a face seen from both sides is a sheet.
 * Where the walk finds no end to the material behind an untouched face that lies across the crater's
 * axis (|normal . axis| > 0.5), the material ends at once. Set by the caller once it knows that
 * there is real space behind (gm_sheet_axis = the crater's axis). */
static int gm_sheet_mode; static Vec3 gm_sheet_axis;
/* Rock-aware mode. In open ground a crater's rock always has solid ground behind it, whichever way
 * it faces: overlapping craters are simply pushed out to the new sphere, the surface stays one
 * sheet. In a thing with a far side (a slab, a wall: untouched faces that look away from the blast
 * inside the sphere) rock is the wall of a hole: a ray can leave the material through it, and what
 * lies behind an old hole is air. gm_break decides per blast. */
static int gm_rock_aware;
static int gm_far_side_seen;        /* after gm_break: this blast found (or was told of) a far side */
static int gm_rock_aware_known;      /* set by the caller: this object has shown a far side before (holes may have eaten it since) */
#define GM_THIN 0.25f
#define GM_SKIN 0.08f
#define GM_GHOST 3.0f
#define GM_GAP 1.0f             /* air up to this length between two pieces of material is bridged (a kerb on the ground) */
static int gm_walk_sealed;                             /* set by gm_break_walk when it returns a distance */
static float gm_walk_next;                             /* ... and then: how far to the next thing after the material ended (-1: nothing before the sphere) */
static int gm_walk_rock;                               /* ... and whether crater rock meets there (bit 1 of gm_farv) */
static int gm_walk_front;                              /* ... and whether the vertex was on a surface that looks at the ray (0: on a far side) */
static float gm_next[GM_MAXV]; static U8 gm_farv[GM_MAXV];
#define GM_KEEP(v, c) (gm_clamp[v] = (c), gm_seal[v] = (U8)((c) >= 0.f && gm_walk_sealed), gm_next[v] = gm_walk_next, gm_farv[v] = (U8)((gm_walk_front ? 0 : 1) | (gm_walk_rock ? 2 : 0)))
static U8 gm_seal[GM_MAXV];                            /* vertex stops on a face that stays: it closes the bowl, it is not part of an opening */
static U8 gm_bnd[GM_MAXV];                             /* vertex on the outline of an opening */
static float gm_break_walk(const GmMesh* ref, U32 nn, const U16* inc, U32 ninc, Vec3 p, Vec3 C, Vec3 Q) {
    Vec3 qc = gm_sub(Q, C), u = gm_sub(p, Q), d, o = p, e, lo, hi; float uu = gm_dot(u, u), tp, L, ht[GM_SHELL_HITS], lastExit = 0.f; signed char hs[GM_SHELL_HITS];
    U32 j, k, nh = 0, jj, gcn; const U16* gcl; int depth, hasFront = 0, hasBack = 0, exitHere = 0, edgeOn = -1, sheetAt = 0;
    gm_walk_sealed = 0; gm_walk_next = -1.f; gm_walk_front = 1; gm_walk_rock = 0;
    if (uu < 1e-8f) return -1.f;
    tp = -2.f * gm_dot(qc, u) / uu;
    if (!(tp > 1.f)) return -1.f;
    L = (tp - 1.f) * gm_sqrt(uu);
    d.x = u.x / gm_sqrt(uu); d.y = u.y / gm_sqrt(uu); d.z = u.z / gm_sqrt(uu);
    /* the faces at the vertex itself */
    for (j = 0; j < ninc; j++) {
        float s = gm_dot(gm_pnrm[inc[j]], d);
        if (gm_prock[inc[j]]) { sheetAt = -1000; if (!gm_rock_aware) { hasFront = 1; continue; } }   /* rock: solid behind it, whichever way it is folded - in open ground */
        if (gm_sheet_mode) { float a = gm_dot(gm_pnrm[inc[j]], gm_sheet_axis); if (a > 0.5f || a < -0.5f) sheetAt++; }
        if (s < -GM_EDGE_ON) hasFront = 1; else if (s > GM_EDGE_ON) hasBack = 1; else if (edgeOn < 0) edgeOn = (int)j;
    }
    if (hasBack) {
        /* a face here that looks away: the ray leaves through it - unless a face that looks towards
         * the ray reaches out beyond it (the back of a kerb standing on the road: the road goes on
         * behind it, the ray leaves the kerb and enters the ground at the same place) */
        for (j = 0; j < ninc && !exitHere; j++) {
            Vec3 nb = gm_pnrm[inc[j]]; int beyond = 0;
            if ((gm_prock[inc[j]] && !gm_rock_aware) || gm_dot(nb, d) <= GM_EDGE_ON) continue;
            for (k = 0; k < ninc && !beyond; k++) {
                U32 tf = gm_pt[inc[k]], q;
                if (k == j || ((!gm_prock[inc[k]] || gm_rock_aware) && gm_dot(gm_pnrm[inc[k]], d) >= -GM_EDGE_ON)) continue;
                if (gm_dot(gm_pnrm[inc[k]], nb) < -0.9f) continue;       /* the two lie back to back: a skin with nothing in it, not material going on */
                for (q = 0; q < 3u; q++) if (gm_dot(nb, gm_sub(ref->pos[ref->tri[tf * 3 + q]], p)) > 1e-3f) beyond = 1;
            }
            if (!beyond) exitHere = 1;
        }
    }
    if (gm_rock_aware && hasFront && !exitHere) {
        /* a skin: rock that looks the other way within a few centimetres before or behind this place
         * (two craters pushed against each other from both sides) - there is nothing between them */
        Vec3 ob; ob.x = p.x - d.x * GM_SKIN; ob.y = p.y - d.y * GM_SKIN; ob.z = p.z - d.z * GM_SKIN;
        gcl = gm_grid_at(p, nn, &gcn);
        for (jj = 0; jj < gcn + (gcl == gm_gall ? 0u : gm_gov_n) && !exitHere; jj++) {
            U32 w; float tt;
            j = jj < gcn ? gcl[jj] : gm_gov[jj - gcn]; w = gm_pt[j];
            if (!gm_prock[j] || gm_dot(gm_pnrm[j], d) < 0.5f) continue;
            tt = gm_ray_tri(ob, d, ref->pos[ref->tri[w * 3]], ref->pos[ref->tri[w * 3 + 1]], ref->pos[ref->tri[w * 3 + 2]]);
            if (tt >= 0.f && tt <= 2.f * GM_SKIN) exitHere = 1;
        }
    }
    gm_walk_rock = sheetAt < 0;
    if (hasFront) depth = exitHere ? 0 : 1;
    else if (hasBack) { depth = 0; gm_walk_front = 0; }
    else if (edgeOn >= 0) { Vec3 n = gm_pnrm[inc[edgeOn]]; o.x = p.x - n.x * 0.02f; o.y = p.y - n.y * 0.02f; o.z = p.z - n.z * 0.02f; depth = 1; }   /* seen edge-on: start just inside */
    else depth = 1;                                                       /* touches nothing we know: assume solid */
    /* what the ray meets on its way */
    e.x = o.x + d.x * L; e.y = o.y + d.y * L; e.z = o.z + d.z * L;
    lo.x = (o.x < e.x ? o.x : e.x) - 0.05f; lo.y = (o.y < e.y ? o.y : e.y) - 0.05f; lo.z = (o.z < e.z ? o.z : e.z) - 0.05f;
    hi.x = (o.x > e.x ? o.x : e.x) + 0.05f; hi.y = (o.y > e.y ? o.y : e.y) + 0.05f; hi.z = (o.z > e.z ? o.z : e.z) + 0.05f;
    gcl = gm_grid_at(p, nn, &gcn);
    for (jj = 0; jj < gcn + (gcl == gm_gall ? 0u : gm_gov_n); jj++) {
        const float* bb; U32 w; float tt, s; int own = 0;
        j = jj < gcn ? gcl[jj] : gm_gov[jj - gcn]; bb = gm_pb + j * 6u; w = gm_pt[j];
        if ((gm_prock[j] && !gm_rock_aware) || bb[0] > hi.x || bb[3] < lo.x || bb[1] > hi.y || bb[4] < lo.y || bb[2] > hi.z || bb[5] < lo.z) continue;
        for (k = 0; k < ninc && !own; k++) if (inc[k] == j) own = 1;
        if (own) continue;
        s = gm_dot(gm_pnrm[j], d);
        if (s > -1e-4f && s < 1e-4f) continue;
        tt = gm_ray_tri(o, d, ref->pos[ref->tri[w * 3]], ref->pos[ref->tri[w * 3 + 1]], ref->pos[ref->tri[w * 3 + 2]]);
        if (tt < 0.004f || tt > L + 0.02f) continue;
        if (nh >= GM_SHELL_HITS) return -1.f;                             /* too tangled to tell: solid */
        ht[nh] = tt; hs[nh] = (signed char)(s < 0.f ? (gm_prock[j] ? 2 : 1) : -1); nh++;       /* 2: into rock */
    }
    for (j = 1; j < nh; j++) { float tv = ht[j]; signed char sv = hs[j]; k = j;      /* in order; at one place, what starts solid first */
        while (k > 0 && (ht[k - 1] > tv + 1e-3f || (ht[k - 1] > tv - 1e-3f && (hs[k - 1] > 0) < (sv > 0)))) { ht[k] = ht[k - 1]; hs[k] = hs[k - 1]; k--; }
        ht[k] = tv; hs[k] = sv; }
    for (j = 0; j < nh; j++) {
        if (j > 0 && (hs[j] > 0) == (hs[j - 1] > 0) && ht[j] - ht[j - 1] < 1e-3f) continue;       /* the same face, met on a shared edge */
        if (hs[j] > 0) {
            if (depth == 0 && ht[j] - lastExit > GM_GAP) break;           /* open air behind the material: it ended there, whatever comes later */
            if (hs[j] == 2 && depth > 0) continue;                        /* rock inside material: an old crater's skin, not a second body */
            depth++;
        } else if (depth > 0) { depth--; if (depth == 0) lastExit = ht[j]; }
    }
    if (depth > 0 && gm_rock_aware && sheetAt <= 0) {
        /* No end found - but in a thing with a far side the material cannot go on beyond that far
         * side, even where holes have eaten it: the planes of the untouched faces that look away
         * from the ray, up to GM_GHOST to the side of what is left of them, still bound it. */
        float best = 1e30f;
        for (j = 0; j < nn; j++) {
            U32 w = gm_pt[j]; Vec3 a, x, q; float s = gm_dot(gm_pnrm[j], d), tt, wq[3];
            if (gm_prock[j] || s < 0.3f) continue;
            a = ref->pos[ref->tri[w * 3]];
            tt = gm_dot(gm_pnrm[j], gm_sub(a, o)) / s;
            if (tt < -0.01f || tt > L || tt >= best) continue;
            x.x = o.x + d.x * tt; x.y = o.y + d.y * tt; x.z = o.z + d.z * tt;
            q = gm_closest_on_tri(x, a, ref->pos[ref->tri[w * 3 + 1]], ref->pos[ref->tri[w * 3 + 2]], wq);
            if (gm_dist2(x, q) < GM_GHOST * GM_GHOST) best = tt;
        }
        if (best < 1e29f) { lastExit = best > 0.f ? best : 0.f; depth = 0; }
    }
    if (depth > 0) {
        if (sheetAt <= 0) return -1.f;
        lastExit = 0.f;                   /* a sheet drawn from both sides, with nothing found behind it: it ends where it begins */
    }
    /* the material ends at lastExit. If that place is in the skin of the sphere (within GM_EPS of
     * it), the face there is not cut and stays: the vertex stops on it and seals the bowl. */
    { Vec3 x; x.x = o.x + d.x * lastExit; x.y = o.y + d.y * lastExit; x.z = o.z + d.z * lastExit;
      gm_walk_sealed = gm_class(x, C, gm_sqrt(gm_dot(qc, qc))) <= 0; }
    for (j = 0; j < nh; j++) if (ht[j] > lastExit + 0.02f) { gm_walk_next = ht[j]; break; }
    return lastExit;
}
/* The same question for any point p (not just a vertex): the faces "at" it are the collected
 * triangles of `ref` within eps of it. */
static float gm_break_point(const GmMesh* ref, U32 nn, Vec3 p, float eps, Vec3 C, Vec3 Q) {
    static U16 at[GM_AT_MAX]; U32 na = 0, i, jj, gcn; float w[3]; const U16* gcl = gm_grid_at(p, nn, &gcn);
    for (jj = 0; jj < gcn + (gcl == gm_gall ? 0u : gm_gov_n) && na < GM_AT_MAX; jj++) {
        const float* bb; U32 t; Vec3 q;
        i = jj < gcn ? gcl[jj] : gm_gov[jj - gcn]; bb = gm_pb + i * 6u; t = gm_pt[i];
        if (p.x < bb[0] - eps || p.x > bb[3] + eps || p.y < bb[1] - eps || p.y > bb[4] + eps || p.z < bb[2] - eps || p.z > bb[5] + eps) continue;
        q = gm_closest_on_tri(p, ref->pos[ref->tri[t * 3]], ref->pos[ref->tri[t * 3 + 1]], ref->pos[ref->tri[t * 3 + 2]], w);
        if (gm_dist2(p, q) < eps * eps) at[na++] = (U16)i;
    }
    if (!na) return -1.f;                             /* nothing of `ref` here: no telling, solid */
    return gm_break_walk(ref, nn, at, na, p, C, Q);
}
/* The edge of the opening. Where a triangle of m has a vertex that goes to the sphere and one that
 * stops short, the edge between them is divided at the place where the one becomes the other
 * (found by bisection, asking `ref`), so that the opening gets its true outline instead of
 * following the triangles. The new vertices go to the sphere and are marked in gm_bnd: they
 * belong to the wall of the hole and to the opening alike.
 * Returns the number of triangles divided, -1 out of capacity. */
#define GM_SPLIT_MIN 0.12f
#define GM_OPEN(v) (gm_clamp[v] >= 0.f && !gm_seal[v])       /* the vertex stops where the material ends, and nothing closes the bowl there */
/* an edge that runs from an open vertex to one that is not: closed, or not moved at all (on the
 * sphere, in its skin) */
static int gm_break_mixed(const GmMesh* m, U32 a, U32 b, Vec3 C, float R, Vec3 Q) {
    if (gm_bnd[a] || gm_bnd[b] || gm_dist2(m->pos[a], m->pos[b]) < GM_SPLIT_MIN * GM_SPLIT_MIN) return 0;      /* (short edges are left alone: the outline is exact enough there) */
    return (GM_OPEN(a) && gm_inside(m->pos[a], C, R, Q)) != (GM_OPEN(b) && gm_inside(m->pos[b], C, R, Q));
}
static int gm_break_edge(GmMesh* m, const GmMesh* ref, U32 nn, float eps, U32 a, U32 b, Vec3 C, float R, Vec3 Q, int mayAdd) {
    float lo = 0.f, hi = 1.f, t; Vec3 pa, pb; int sa, v, found; U32 k;
    if (gm_hcount > GM_HASH_SIZE / 2) return -1;
    k = gm_hash_slot(a, b, &found);
    if (found) return gm_hval[k];
    gm_hkey[k] = 0xFFFFFFFFu; gm_hcount--;            /* only looked: gm_edge_vertex enters it */
    if (!mayAdd) return -2;
    if (gm_less(m->pos[b], m->pos[a]) || (!gm_less(m->pos[a], m->pos[b]) && b < a)) { U32 x = a; a = b; b = x; }
    pa = m->pos[a]; pb = m->pos[b]; sa = GM_OPEN(a) && gm_inside(pa, C, R, Q);
    for (k = 0; k < 10u; k++) {
        Vec3 x; int op = 0; t = 0.5f * (lo + hi);
        x.x = pa.x + (pb.x - pa.x) * t; x.y = pa.y + (pb.y - pa.y) * t; x.z = pa.z + (pb.z - pa.z) * t;
        if (gm_inside(x, C, R, Q)) { float c = gm_break_point(ref, nn, x, eps, C, Q); op = c >= 0.f && !gm_walk_sealed; }
        if (op == sa) lo = t; else hi = t;
    }
    /* the new vertex stands on the side that is not open, and does what the surface does there */
    t = sa ? hi : lo;
    if (t < 0.001f) t = 0.001f; else if (t > 0.999f) t = 0.999f;
    v = gm_edge_vertex(m, a, b, t, 2);
    if (v >= 0) {
        float c = -1.f; int sl = 0;
        if (gm_inside(m->pos[v], C, R, Q)) {
            c = gm_break_point(ref, nn, m->pos[v], eps, C, Q); sl = c >= 0.f;
            if (c >= 0.f && !gm_walk_sealed) c = 0.f;                               /* (cannot tell here: it stays where it is) */
        }
        gm_clamp[v] = c; gm_seal[v] = (U8)sl; gm_bnd[v] = 1; gm_next[v] = gm_walk_next; gm_farv[v] = (U8)((gm_walk_front ? 0 : 1) | (gm_walk_rock ? 2 : 0));
    }
    return v;
}
#define GM_MAX_MID 150u          /* per blast: at most this many triangles get a vertex in the middle ... */
#define GM_MAX_OUTLINE 600u      /* ... and this many edges are divided for the outline (what is left over stays as it is: a little coarser) */
static int gm_break_split(GmMesh* m, const GmMesh* ref, U32* pnn, float eps, Vec3 C, float R, Vec3 Q, int flip) {
    static U16 mt[GM_MAX_MID]; static float mc[GM_MAX_MID], mn[GM_MAX_MID]; static U8 ms[GM_MAX_MID], mf[GM_MAX_MID];
    U32 nt0 = m->nt, t, pass, nm = 0, nn = *pnn; int done = 0;
    /* a triangle whose corners all stop short but whose middle does not (a post thinner than the
     * triangles on it: all corners on its edges) gets a vertex in the middle */
    for (t = 0; t < nt0 && nm < GM_MAX_MID; t++) {
        U32 k, nin = 0, nopen = 0; Vec3 cen; float c; int op;
        if ((m->flag && m->flag[t]) || gm_far(m, t, C, R + GM_EPS)) continue;
        cen.x = cen.y = cen.z = 0.f;
        for (k = 0; k < 3u; k++) { U32 v = m->tri[t * 3 + k]; Vec3 p = m->pos[v]; cen.x += p.x / 3.f; cen.y += p.y / 3.f; cen.z += p.z / 3.f;
            if (gm_bnd[v] || !gm_inside(p, C, R, Q)) continue;
            nin++; if (GM_OPEN(v)) nopen++; }
        if (!nin || nopen != nin || !gm_inside(cen, C, R, Q)) continue;
        { Vec3 a = m->pos[m->tri[t * 3]], b = m->pos[m->tri[t * 3 + 1]], c2 = m->pos[m->tri[t * 3 + 2]];
          if (gm_dist2(a, b) < 0.09f && gm_dist2(b, c2) < 0.09f && gm_dist2(c2, a) < 0.09f) continue; }       /* small: not worth it */      /* (only this way round: an opening smaller than a triangle is simply not made) */
        c = gm_break_point(ref, nn, cen, eps, C, Q); op = c >= 0.f && !gm_walk_sealed;
        if (op) continue;
        mt[nm] = (U16)t; mc[nm] = c; ms[nm] = (U8)(c >= 0.f && gm_walk_sealed); mn[nm] = gm_walk_next; mf[nm] = (U8)((gm_walk_front ? 0 : 1) | (gm_walk_rock ? 2 : 0)); nm++;
    }
    for (pass = 0; pass < nm; pass++) {
        U32 idx[3]; static const float w3[3] = { 1.f / 3.f, 1.f / 3.f, 1.f / 3.f }; Vec3 cen; int v; U16 aux;
        t = mt[pass]; idx[0] = m->tri[t * 3]; idx[1] = m->tri[t * 3 + 1]; idx[2] = m->tri[t * 3 + 2]; aux = m->aux[t];
        cen.x = (m->pos[idx[0]].x + m->pos[idx[1]].x + m->pos[idx[2]].x) / 3.f; cen.y = (m->pos[idx[0]].y + m->pos[idx[1]].y + m->pos[idx[2]].y) / 3.f; cen.z = (m->pos[idx[0]].z + m->pos[idx[1]].z + m->pos[idx[2]].z) / 3.f;
        v = gm_add_vertex(m, cen, idx, w3, 3);
        if (v < 0) return -1;
        gm_clamp[v] = mc[pass]; gm_seal[v] = ms[pass]; gm_bnd[v] = 0; gm_next[v] = mn[pass]; gm_farv[v] = mf[pass];
        gm_set_tri(m, t, idx[0], idx[1], (U32)v);
        if (gm_add_tri(m, idx[1], idx[2], (U32)v, aux, t) < 0 || gm_add_tri(m, idx[2], idx[0], (U32)v, aux, t) < 0) return -1;
        done++;
    }
    if (nm && m == ref) { nn = gm_break_collect(m, C, R, Q, flip); if (!nn) { *pnn = 0; return done; } }
    nt0 = m->nt;
    gm_hash_clear();
    for (pass = 0; pass < 2u; pass++)                 /* first all the new vertices (the mesh `ref` looks at must not change meanwhile), then the triangles */
    for (t = 0; t < nt0; t++) {
        U32 v[3], k, ncut = 0, e0 = 0, e1 = 0, o; int x[3] = { 0, 0, 0 }; U16 aux;
        if (gm_far(m, t, C, R + GM_EPS)) continue;
        for (k = 0; k < 3u; k++) v[k] = m->tri[t * 3 + k];
        for (k = 0; k < 3u; k++) {
            U32 j = (k + 1u) % 3u;
            if (!gm_break_mixed(m, v[k], v[j], C, R, Q)) continue;
            x[k] = gm_break_edge(m, ref, nn, eps, v[k], v[j], C, R, Q, !pass && gm_hcount < GM_MAX_OUTLINE);
            if (x[k] == -2) continue;                 /* over the budget: this edge stays whole */
            if (x[k] < 0) return -1;
            if (ncut++ == 0) e0 = k; else e1 = k;
        }
        if (!ncut || !pass) continue;
        aux = m->aux[t];
        if (ncut == 1) {
            U32 w = v[(e0 + 2) % 3];
            gm_set_tri(m, t, v[e0], (U32)x[e0], w);
            if (gm_add_tri(m, (U32)x[e0], v[(e0 + 1) % 3], w, aux, t) < 0) return -1;
        } else {
            U32 xa, xb;
            if (e0 == 0 && e1 == 1) o = 1; else if (e0 == 1 && e1 == 2) o = 2; else o = 0;
            xa = (U32)x[o]; xb = (U32)x[(o + 2) % 3];
            gm_set_tri(m, t, v[o], xa, xb);
            if (gm_less(m->pos[v[(o + 1) % 3]], m->pos[v[(o + 2) % 3]])) {
                if (gm_add_tri(m, xa, v[(o + 1) % 3], xb, aux, t) < 0 ||
                    gm_add_tri(m, v[(o + 1) % 3], v[(o + 2) % 3], xb, aux, t) < 0) return -1;
            } else {
                if (gm_add_tri(m, xa, v[(o + 1) % 3], v[(o + 2) % 3], aux, t) < 0 ||
                    gm_add_tri(m, xa, v[(o + 2) % 3], xb, aux, t) < 0) return -1;
            }
        }
        done++;
    }
    if (done && m == ref) nn = gm_break_collect(m, C, R, Q, flip);
    *pnn = nn;
    return done;
}
/* Open ground, but is there another cavity straight ahead - a tunnel dug earlier, about to be
 * met by this one? Five rays from the pole around the crater's axis: each meets the surface, goes
 * through the ground and - if there is a cavity - comes out of rock from behind, inside the sphere,
 * with at least GM_GAP of air after it. One such ray: the blast joins the two (rock-aware mode).
 * Craters that merely overlap side by side do not have this straight ahead; they stay one sheet. */
static int gm_cavity_ahead(const GmMesh* ref, U32 nn, Vec3 C, float R, Vec3 Q) {
    Vec3 a = gm_sub(C, Q), e1, e2; float l = gm_sqrt(gm_dot(a, a)); U32 r, j, k, found = 0;
    if (l < 1e-6f) return 0;
    a.x /= l; a.y /= l; a.z /= l;
    e1.x = a.y; e1.y = -a.x; e1.z = 0.f; l = gm_sqrt(gm_dot(e1, e1));
    if (l < 0.1f) { e1.x = 1.f; e1.y = 0.f; e1.z = 0.f; l = 1.f; }
    e1.x /= l; e1.y /= l; e1.z /= l; e2 = gm_cross(a, e1);
    for (r = 0; r < 5u; r++) {
        static const float su[5] = { 0.f, 0.15f, -0.15f, 0.f, 0.f }, sv[5] = { 0.f, 0.f, 0.f, 0.15f, -0.15f };
        Vec3 d; float ht[32], tv; signed char hs[32], sv2; U32 nh = 0;
        d.x = a.x + e1.x * su[r] + e2.x * sv[r]; d.y = a.y + e1.y * su[r] + e2.y * sv[r]; d.z = a.z + e1.z * su[r] + e2.z * sv[r];
        l = gm_sqrt(gm_dot(d, d)); d.x /= l; d.y /= l; d.z /= l;
        for (j = 0; j < nn && nh < 32u; j++) {
            U32 w = gm_pt[j]; float s = gm_dot(gm_pnrm[j], d), tt;
            if (s > -1e-4f && s < 1e-4f) continue;
            tt = gm_ray_tri(Q, d, ref->pos[ref->tri[w * 3]], ref->pos[ref->tri[w * 3 + 1]], ref->pos[ref->tri[w * 3 + 2]]);
            if (tt < 0.05f) continue;
            ht[nh] = tt; hs[nh] = (signed char)(s < 0.f ? 1 : (gm_prock[j] ? -2 : -1)); nh++;
        }
        for (j = 1; j < nh; j++) { tv = ht[j]; sv2 = hs[j]; k = j; while (k > 0 && ht[k - 1] > tv) { ht[k] = ht[k - 1]; hs[k] = hs[k - 1]; k--; } ht[k] = tv; hs[k] = sv2; }
        /* the surface (looking at us), then out of rock from behind, then air */
        for (j = 0; j < nh && hs[j] > 0 && j + 1u < nh && hs[j + 1u] > 0 && ht[j + 1u] - ht[j] < 0.05f; j++) {}      /* (doubled faces at the surface) */
        if (j < nh && hs[j] == -2) j--;                    /* (or the first thing met is rock from behind: what is left of the wall between) */
        else if (j + 1u >= nh || hs[j] <= 0 || hs[j + 1u] != -2) continue;
        { Vec3 x; x.x = Q.x + d.x * ht[j + 1u]; x.y = Q.y + d.y * ht[j + 1u]; x.z = Q.z + d.z * ht[j + 1u];
          if (gm_class(x, C, R) <= 0) continue; }
        if (j + 2u < nh && ht[j + 2u] - ht[j + 1u] < GM_GAP) continue;
        found++;
    }
    return found >= 1u;
}
static U32 gm_break(GmMesh* m, Vec3 C, float R, Vec3 Q, int flip) {
    U32 nn, i, k, v, marked = 0, used = 0; Vec3 qc = gm_sub(Q, C); float ql = gm_sqrt(gm_dot(qc, qc)), bestAxis = 1e30f;
    for (v = 0; v < m->nv; v++) { gm_clamp[v] = -1.f; gm_bnd[v] = 0; gm_seal[v] = 0; gm_next[v] = -1.f; gm_farv[v] = 0; }
    if (!m->flag || ql < 1e-6f) return 0;
    nn = gm_break_collect(m, C, R, Q, flip);
    if (!nn) return 0;
    gm_far_side_seen = gm_rock_aware_known;
    for (i = 0; i < nn && !gm_far_side_seen; i++) {
        U32 t = gm_pt[i];
        if (gm_prock[i] || gm_dot(gm_pnrm[i], qc) > -0.3f * ql) continue;         /* untouched, looking away from the blast ... */
        if (gm_class(m->pos[m->tri[t * 3]], C, R) > 0 || gm_class(m->pos[m->tri[t * 3 + 1]], C, R) > 0 || gm_class(m->pos[m->tri[t * 3 + 2]], C, R) > 0) gm_far_side_seen = 1;   /* ... inside the sphere */
    }
    gm_rock_aware = gm_far_side_seen || gm_cavity_ahead(m, nn, C, R, Q);
    /* vertices by position, and for each position the triangles that touch it */
    memset(gm_ph, 0xFF, sizeof(gm_ph));
    for (v = 0; v < m->nv; v++) gm_rep[v] = 0xFFFFu;
    for (i = 0; i < nn; i++) for (k = 0; k < 3u; k++) {
        v = m->tri[gm_pt[i] * 3 + k];
        if (gm_rep[v] == 0xFFFFu) { if (++used > GM_POS_HASH / 2u) return 0; gm_rep[v] = (U16)gm_pos_slot(m, v); }
    }
    memset(gm_ic, 0, (m->nv + 2u) * sizeof(U32));
    for (i = 0; i < nn; i++) for (k = 0; k < 3u; k++) gm_ic[gm_rep[m->tri[gm_pt[i] * 3 + k]] + 2u]++;
    for (v = 0; v < m->nv; v++) gm_ic[v + 2u] += gm_ic[v + 1u];
    for (i = 0; i < nn; i++) for (k = 0; k < 3u; k++) { U32 r = gm_rep[m->tri[gm_pt[i] * 3 + k]]; gm_il[gm_ic[r + 1u]++] = (U16)i; }
    /* gm_ic[r] .. gm_ic[r + 1] is now the list of position r */
    for (v = 0; v < m->nv; v++) {
        U32 r = gm_rep[v];
        if (r != v || !gm_inside(m->pos[v], C, R, Q)) continue;
        /* the faces at this place: those that use it, and those that merely pass through it (a second
         * layer lying on the first without sharing its vertices - thin metal, a wall with two faces) */
        { static U16 at[GM_AT_MAX]; U32 na = gm_ic[v + 1u] - gm_ic[v], j; Vec3 p = m->pos[v]; float w[3];
          if (na > GM_AT_MAX) na = GM_AT_MAX;
          for (j = 0; j < na; j++) at[j] = gm_il[gm_ic[v] + j];
          { U32 jj, gcn; const U16* gcl = gm_grid_at(p, nn, &gcn);
          for (jj = 0; jj < gcn + (gcl == gm_gall ? 0u : gm_gov_n) && na < GM_AT_MAX; jj++) {
              const float* bb; U32 t; Vec3 q; int own = 0;
              i = jj < gcn ? gcl[jj] : gm_gov[jj - gcn]; bb = gm_pb + i * 6u; t = gm_pt[i];
              if (p.x < bb[0] - GM_AT_EPS || p.x > bb[3] + GM_AT_EPS || p.y < bb[1] - GM_AT_EPS || p.y > bb[4] + GM_AT_EPS || p.z < bb[2] - GM_AT_EPS || p.z > bb[5] + GM_AT_EPS) continue;
              if (gm_rep[m->tri[t * 3]] == v || gm_rep[m->tri[t * 3 + 1]] == v || gm_rep[m->tri[t * 3 + 2]] == v) continue;
              q = gm_closest_on_tri(p, m->pos[m->tri[t * 3]], m->pos[m->tri[t * 3 + 1]], m->pos[m->tri[t * 3 + 2]], w);
              if (gm_dist2(p, q) >= GM_AT_EPS * GM_AT_EPS) continue;
              for (j = 0; j < na && !own; j++) if (at[j] == i) own = 1;
              if (!own) at[na++] = (U16)i;
          } }
          { float c = gm_break_walk(m, nn, at, na, p, C, Q); GM_KEEP(v, c); } }
    }
    for (v = 0; v < m->nv; v++) if (gm_rep[v] != 0xFFFFu && gm_rep[v] != v) { int in = gm_inside(m->pos[v], C, R, Q); U32 r = gm_rep[v]; gm_clamp[v] = in ? gm_clamp[r] : -1.f; gm_seal[v] = (U8)(in ? gm_seal[r] : 0); gm_next[v] = in ? gm_next[r] : -1.f; gm_farv[v] = (U8)(in ? gm_farv[r] : 0); }
    /* give the opening its true outline */
    if (gm_break_split(m, m, &nn, GM_AT_EPS, C, R, Q, flip) < 0) { gm_break_failed = 1; return 0; }
    if (!nn) return 0;
    /* the opening: triangles facing the blast (or seen edge-on) whose vertices all stop short */
    for (i = 0; i < nn; i++) {
        U32 t = gm_pt[i], stopped = 0, free_ = 0; Vec3 cen, u, d; float uu, far_ = 0.f; int cls[3], sum = 0;
        if (m->flag[t]) continue;
        cen.x = cen.y = cen.z = 0.f;
        for (k = 0; k < 3u; k++) { Vec3 pp = m->pos[m->tri[t * 3 + k]]; cls[k] = gm_class(pp, C, R); sum += cls[k]; cen.x += pp.x / 3.f; cen.y += pp.y / 3.f; cen.z += pp.z / 3.f; }
        if (cls[0] < 0 || cls[1] < 0 || cls[2] < 0 || sum == 0) continue;            /* not inside the sphere */
        u = gm_sub(cen, Q); uu = gm_dot(u, u);
        if (uu < 1e-8f) continue;
        d.x = u.x / gm_sqrt(uu); d.y = u.y / gm_sqrt(uu); d.z = u.z / gm_sqrt(uu);
        if (!gm_sheet_mode && (!gm_prock[i] || gm_rock_aware) && gm_dot(gm_pnrm[i], d) > 0.15f) continue;   /* far sides: next pass (a sheet has no far side) */
        for (k = 0; k < 3u; k++) {
            v = m->tri[t * 3 + k];
            if (!gm_inside(m->pos[v], C, R, Q)) continue;                            /* on the rim or in the air-side half: does not move anyway */
            if (gm_bnd[v]) continue;                                                 /* on the outline: wall and opening alike */
            if (!GM_OPEN(v)) free_++; else { stopped++; if (gm_clamp[v] > far_) far_ = gm_clamp[v]; }
        }
        if (free_ || !stopped) continue;
        m->flag[t] = 2; marked++;
        { Vec3 ax; float along = gm_dot(gm_sub(cen, C), qc) / (ql * ql), dist2;      /* nearest the axis: where the caller looks behind */
          ax.x = cen.x - (C.x + qc.x * along); ax.y = cen.y - (C.y + qc.y * along); ax.z = cen.z - (C.z + qc.z * along); dist2 = gm_dot(ax, ax);
          if (dist2 < bestAxis) { float fc = gm_break_point(m, nn, cen, GM_AT_EPS, C, Q);      /* where the material ends behind this very place */
              if (fc >= 0.f) far_ = fc;
              bestAxis = dist2; gm_shell_far = far_; gm_shell_p.x = cen.x + d.x * (far_ + 0.25f); gm_shell_p.y = cen.y + d.y * (far_ + 0.25f); gm_shell_p.z = cen.z + d.z * (far_ + 0.25f); gm_shell_d = d; } }
    }
    if (!marked) return 0;
    /* the far side goes where it lies behind the opening (seen from its centre). The mesh is cut along
     * the sphere, and the opening has its true outline: what is inside the sphere on the far side
     * is exactly what lies behind the opening, give or take the straight edges of both outlines. */
    for (i = 0; i < nn; i++) {
        U32 t = gm_pt[i], q, nopen = 0; Vec3 P[3], cen, smp[4]; int cls[3], sum = 0, ok = 1;
        if (m->flag[t] || (gm_prock[i] && !gm_rock_aware)) continue;
        for (k = 0; k < 3u; k++) { v = m->tri[t * 3 + k]; P[k] = m->pos[v]; cls[k] = gm_class(P[k], C, R); sum += cls[k];
            if (!gm_inside(P[k], C, R, Q) || gm_bnd[v]) continue;
            if (GM_OPEN(v)) nopen++; else ok = 0; }                                       /* a corner that moves: this is no far side */
        if (!ok || !nopen) continue;
        if (cls[0] < 0 || cls[1] < 0 || cls[2] < 0 || sum == 0) continue;
        cen.x = (P[0].x + P[1].x + P[2].x) / 3.f; cen.y = (P[0].y + P[1].y + P[2].y) / 3.f; cen.z = (P[0].z + P[1].z + P[2].z) / 3.f;
        smp[0] = cen; for (k = 0; k < 3u; k++) { smp[k + 1u].x = P[k].x * 0.9f + cen.x * 0.1f; smp[k + 1u].y = P[k].y * 0.9f + cen.y * 0.1f; smp[k + 1u].z = P[k].z * 0.9f + cen.z * 0.1f; }
        /* Rock seen from behind, none of it moving: the ground it was the skin of is being taken away
         * (or already gone), whatever stands before it - it goes. Untouched faces go only behind the opening. */
        for (q = 0; q < 1u && ok && !gm_prock[i]; q++) {
            Vec3 u = gm_sub(smp[q], Q), rd; float uu = gm_dot(u, u), best = GM_SHELL_MAX; U32 j; int hit = -1;
            if (uu < 1e-8f) { ok = 0; break; }
            rd.x = -u.x / gm_sqrt(uu); rd.y = -u.y / gm_sqrt(uu); rd.z = -u.z / gm_sqrt(uu);
            if (q == 0u && gm_dot(gm_pnrm[i], rd) > -0.15f) { ok = 0; break; }       /* not a far side at all */
            { U32 jj, gcn; const U16* gcl = gm_grid_at(smp[q], nn, &gcn);
            for (jj = 0; jj < gcn + (gcl == gm_gall ? 0u : gm_gov_n); jj++) {
                U32 w; float tt;
                j = jj < gcn ? gcl[jj] : gm_gov[jj - gcn]; w = gm_pt[j];
                if (j == i) continue;
                tt = gm_ray_tri(smp[q], rd, m->pos[m->tri[w * 3]], m->pos[m->tri[w * 3 + 1]], m->pos[m->tri[w * 3 + 2]]);
                if (tt < -0.004f || tt >= best) continue;
                best = tt; hit = (int)j;
            } }
            if (hit < 0 || m->flag[gm_pt[hit]] != 2) ok = 0;      /* what stands before it is not part of the opening */
        }
        if (ok) { m->flag[t] = 3; marked++; }
    }
    for (i = 0; i < nn; i++) if (m->flag[gm_pt[i]] == 3) m->flag[gm_pt[i]] = 2;
    return marked;
}
/* No hole after all (the caller found no real space behind the opening): nothing is removed.
 * What becomes of the vertices that stop where the material ends:
 *   mode 0  they stop there: the bowl lies on the far side (a thing with a far side, blast inside it)
 *   mode 1  where the material is thinner than GM_THIN they go on to the next thing they meet, or
 *           to the sphere: the bowl is pushed into the hollow behind, where nobody sees it (a building
 *           that is only a skin; without this a thin skin would show no crater at all). Where it is
 *           thicker they stop: the bowl lies on the far side.
 *   mode 2  they go to the sphere (nothing behind at all: the underside of the map)
 * Far sides themselves never move, and in mode 1 crater rock does not either. */
static void gm_unbreak(GmMesh* m, int mode) {
    U32 t, v;
    for (t = 0; t < m->nt; t++) if (m->flag[t] == 2) m->flag[t] = 0;
    for (v = 0; v < m->nv; v++) {
        if (gm_clamp[v] < 0.f) continue;
        if (mode == 0 || (gm_farv[v] & 1) || (mode == 1 && (gm_clamp[v] > GM_THIN || (gm_farv[v] & 2)))) { gm_seal[v] = 1; continue; }
        gm_clamp[v] = mode == 1 ? gm_next[v] : -1.f; gm_seal[v] = (U8)(gm_clamp[v] >= 0.f);
    }
}

/* Is what stops the vertices a thin skin (most of them stop within GM_THIN) or a thick part? */
static int gm_thin_skin(const GmMesh* m) {
    U32 v, thin = 0, all = 0;
    for (v = 0; v < m->nv; v++) if (gm_clamp[v] >= 0.f && !gm_farv[v] && !gm_bnd[v]) { all++; if (gm_clamp[v] <= GM_THIN) thin++; }       /* (untouched surface only: crater rock is never a skin) */
    return all > 0u && thin * 10u >= all * 7u;
}

/* Is the point P inside a hollow shell - a building that is only a skin, nothing modelled inside?
 * Rays go out from P: forward (d), in a cone around it and sideways. Inside a hollow shell they
 * meet the skin from behind; in a real room they meet walls and floor from the front, in the
 * open nothing. Returns 1 if more rays meet backs than fronts (and at least three do).
 * rockSolid: crater rock counts as seen from the front whichever way it faces (it has ground behind it). */
static U32 gm_enc_nb, gm_enc_nf;            /* what the last gm_enclosed counted: rays that met backs, fronts */
static int gm_enclosed(const GmMesh* m, Vec3 P, Vec3 d, int flip, int rockSolid) {
    Vec3 e1, e2, dir[13]; U32 r, t, nb = 0, nf = 0; float l;
    e1.x = d.y; e1.y = -d.x; e1.z = 0.f; l = gm_sqrt(gm_dot(e1, e1));
    if (l < 0.1f) { e1.x = 1.f; e1.y = 0.f; e1.z = 0.f; l = 1.f; }
    e1.x /= l; e1.y /= l; e1.z /= l; e2 = gm_cross(d, e1);
    dir[0] = d;
    for (r = 0; r < 6u; r++) {
        static const float cs[6] = { 1.f, 0.5f, -0.5f, -1.f, -0.5f, 0.5f }, sn[6] = { 0.f, 0.866f, 0.866f, 0.f, -0.866f, -0.866f };
        Vec3 side; side.x = e1.x * cs[r] + e2.x * sn[r]; side.y = e1.y * cs[r] + e2.y * sn[r]; side.z = e1.z * cs[r] + e2.z * sn[r];
        dir[1 + r] = side;
        dir[7 + r].x = 0.5f * d.x + 0.866f * side.x; dir[7 + r].y = 0.5f * d.y + 0.866f * side.y; dir[7 + r].z = 0.5f * d.z + 0.866f * side.z;
    }
    for (r = 0; r < 13u; r++) {
        float best = 150.f; int kind = 0;
        for (t = 0; t < m->nt; t++) {
            Vec3 a = m->pos[m->tri[t * 3]], b = m->pos[m->tri[t * 3 + 1]], c = m->pos[m->tri[t * 3 + 2]], n; float tt = gm_ray_tri(P, dir[r], a, b, c);
            int k2;
            if (tt < 0.003f || tt >= best + 0.005f) continue;
            n = flip ? gm_cross(gm_sub(c, a), gm_sub(b, a)) : gm_cross(gm_sub(b, a), gm_sub(c, a));
            k2 = ((rockSolid && (U32)m->aux[t] == gm_solid_aux) || gm_dot(n, dir[r]) < 0.f) ? 1 : 2;
            if (tt > best - 0.005f) { if (k2 == 1) kind = 1; if (tt < best) best = tt; continue; }   /* two faces at one place (a wall with an inside face on it): the one looking at us counts */
            best = tt; kind = k2;
        }
        if (kind == 1) nf++; else if (kind == 2) nb++;
    }
    gm_enc_nb = nb; gm_enc_nf = nf;
    return nb >= 3u && nb > nf;
}

/* Which side of the visible surface at S does the direction n look from? The layer nearest to S
 * counts (within reach): +1 its front is seen from n, -1 its back, 2 both (two faces back to
 * back, the rim of a crater), 0 nothing there. Faces seen edge-on say nothing.
 * A back at S with its other face further out on the side of n (looking the opposite way: a wall
 * whose collision surface lies on the far face) is a front too: +1. */
static int gm_side_at(const GmMesh* m, Vec3 S, Vec3 n, float reach) {
    U32 t, pass, nf = 0, nb = 0, ahead = 0; float dmin = reach * reach, lim = 0.f; Vec3 bn = { 0.f, 0.f, 0.f };
    for (pass = 0; pass < 3u; pass++) {
        for (t = 0; t < m->nt; t++) {
            Vec3 a = m->pos[m->tri[t * 3]], b = m->pos[m->tri[t * 3 + 1]], c = m->pos[m->tri[t * 3 + 2]], nn, q; float w[3], d2, l, cs;
            if (a.x - S.x > reach && b.x - S.x > reach && c.x - S.x > reach) continue;
            if (S.x - a.x > reach && S.x - b.x > reach && S.x - c.x > reach) continue;
            if (a.y - S.y > reach && b.y - S.y > reach && c.y - S.y > reach) continue;
            if (S.y - a.y > reach && S.y - b.y > reach && S.y - c.y > reach) continue;
            if (a.z - S.z > reach && b.z - S.z > reach && c.z - S.z > reach) continue;
            if (S.z - a.z > reach && S.z - b.z > reach && S.z - c.z > reach) continue;
            nn = gm_cross(gm_sub(b, a), gm_sub(c, a)); l = gm_sqrt(gm_dot(nn, nn));
            if (l < 2e-4f) continue;                                   /* a sliver */
            cs = gm_dot(nn, n) / l;
            if (cs > -0.3f && cs < 0.3f) continue;
            q = gm_closest_on_tri(S, a, b, c, w); d2 = gm_dist2(S, q);
            if (pass == 0u) { if (d2 < dmin) dmin = d2; continue; }
            if (pass == 1u) {
                if (d2 > lim) continue;
                if (cs > 0.f) nf++; else { nb++; bn.x += nn.x / l; bn.y += nn.y / l; bn.z += nn.z / l; }
            } else if (d2 > lim && d2 < reach * reach && cs > 0.f && gm_dot(gm_sub(q, S), n) > 0.05f && gm_dot(nn, bn) < -0.7f * l) ahead++;
        }
        if (pass == 0u) { float d; if (dmin >= reach * reach) return 0; d = gm_sqrt(dmin) + 0.1f; lim = d * d; }
        if (pass == 1u) { float bl = gm_sqrt(gm_dot(bn, bn)); if (nf || !nb || bl < 1e-6f) break; bn.x /= bl; bn.y /= bl; bn.z /= bl; }
    }
    return nf && !nb ? 1 : nb && !nf ? (ahead ? 1 : -1) : nf ? 2 : 0;
}

/* After a break-through: bits that hang in the air. Whatever lies wholly inside the sphere and is
 * joined to nothing outside it (a scrap of the far side, a sheet of rock) is removed.
 * Returns the number of triangles removed. */
static U16 gm_uf[GM_MAXV]; static U8 gm_anch[GM_MAXV];
static U32 gm_uf_find(U32 v) { while (gm_uf[v] != v) { gm_uf[v] = gm_uf[gm_uf[v]]; v = gm_uf[v]; } return v; }
static U32 gm_drop_debris(GmMesh* m, Vec3 C, float R) {
    U32 t, k, v, used = 0, n = 0, dropped = 0; float lim = (R + GM_EPS + 0.05f) * (R + GM_EPS + 0.05f);
    memset(gm_ph, 0xFF, sizeof(gm_ph));
    for (v = 0; v < m->nv; v++) gm_rep[v] = 0xFFFFu;
    for (t = 0; t < m->nt; t++) {
        if (gm_far(m, t, C, R + 0.3f)) continue;
        for (k = 0; k < 3u; k++) {
            v = m->tri[t * 3 + k];
            if (gm_rep[v] == 0xFFFFu) { if (++used > GM_POS_HASH / 2u) return 0; gm_rep[v] = (U16)gm_pos_slot(m, v); gm_uf[v] = (U16)v; }
        }
    }
    /* gm_uf: components; gm_anch: 1 = the component reaches outside the sphere */
    for (v = 0; v < m->nv; v++) if (gm_rep[v] != 0xFFFFu) gm_anch[v] = 0;
    for (t = 0; t < m->nt; t++) {
        U32 r0, r1, r2;
        if (gm_far(m, t, C, R + 0.3f)) continue;
        r0 = gm_uf_find(gm_rep[m->tri[t * 3]]); r1 = gm_uf_find(gm_rep[m->tri[t * 3 + 1]]); r2 = gm_uf_find(gm_rep[m->tri[t * 3 + 2]]);
        gm_uf[r1] = (U16)r0; gm_uf[gm_uf_find(r2)] = (U16)r0;
    }
    for (t = 0; t < m->nt; t++) {
        if (gm_far(m, t, C, R + 0.3f)) continue;
        for (k = 0; k < 3u; k++) { v = m->tri[t * 3 + k]; if (gm_dist2(m->pos[v], C) > lim) gm_anch[gm_uf_find(gm_rep[v])] = 1; }
    }
    for (t = 0; t < m->nt; t++) {
        if (!gm_far(m, t, C, R + 0.3f) && !gm_anch[gm_uf_find(gm_rep[m->tri[t * 3]])]) { dropped++; continue; }
        if (n != t) {
            m->tri[n * 3] = m->tri[t * 3]; m->tri[n * 3 + 1] = m->tri[t * 3 + 1]; m->tri[n * 3 + 2] = m->tri[t * 3 + 2];
            m->aux[n] = m->aux[t]; if (m->flag) m->flag[n] = m->flag[t];
        }
        n++;
    }
    m->nt = n;
    return dropped;
}

/* Vertices no triangle uses any more are taken out (holes and debris leave them behind).
 * Returns the number removed. */
static U32 gm_compact(GmMesh* m) {
    U32 v, i, k, d, n = 0;
    for (v = 0; v < m->nv; v++) gm_uf[v] = 0xFFFFu;
    for (i = 0; i < m->nt * 3u; i++) gm_uf[m->tri[i]] = 0;
    for (v = 0; v < m->nv; v++) {
        if (gm_uf[v] == 0xFFFFu) continue;
        if (n != v) {
            m->pos[n] = m->pos[v];
            for (k = 0; k < m->nf; k++) { U32 dim = m->fdim[k]; float* a = m->fa[k]; for (d = 0; d < dim; d++) a[n * dim + d] = a[v * dim + d]; }
            for (k = 0; k < m->nb; k++) { U8* a = m->ba[k]; for (d = 0; d < 4u; d++) a[n * 4u + d] = a[v * 4u + d]; }
        }
        gm_uf[v] = (U16)n; n++;
    }
    for (i = 0; i < m->nt * 3u; i++) m->tri[i] = gm_uf[m->tri[i]];
    v = m->nv - n; m->nv = n;
    return v;
}

/* Crater rock is cut again by every blast that touches it, and ends up as a heap of slivers. Where
 * a rock triangle has an edge shorter than eps, one end of the edge is moved onto the other (all
 * vertices at that place move together); the triangles on that edge collapse and are removed by
 * gm_drop_degenerate. Only places where nothing but rock meets are moved, and never so that a
 * triangle would turn over. Returns the number of edges collapsed. */
static U8 gm_wfix[GM_MAXV];
static U16 gm_wt[GM_MAXT];                 /* the triangles looked at */
static U16 gm_wl[GM_MAXT * 3u];            /* per place: indices into gm_wt */
static U32 gm_weld_short(GmMesh* m, Vec3 C, float R, U32 rockAux, float eps) {
    U32 t, k, v, used = 0, nn = 0, done = 0; float lim = R + 0.3f;
    memset(gm_ph, 0xFF, sizeof(gm_ph));
    for (v = 0; v < m->nv; v++) gm_rep[v] = 0xFFFFu;
    for (t = 0; t < m->nt; t++) {
        if (gm_far(m, t, C, lim)) continue;
        gm_wt[nn++] = (U16)t;
        for (k = 0; k < 3u; k++) {
            v = m->tri[t * 3 + k];
            if (gm_rep[v] == 0xFFFFu) { if (++used > GM_POS_HASH / 2u) return 0; gm_rep[v] = (U16)gm_pos_slot(m, v); gm_uf[v] = (U16)v; gm_wfix[v] = 0; }
        }
    }
    /* a place may move if only rock meets there and it lies well inside the region looked at */
    for (k = 0; k < nn; k++) {
        t = gm_wt[k];
        for (v = 0; v < 3u; v++) { U32 r = gm_rep[m->tri[t * 3 + v]]; Vec3 p = m->pos[r];
            if ((U32)m->aux[t] != rockAux || p.x < C.x - lim || p.x > C.x + lim || p.y < C.y - lim || p.y > C.y + lim || p.z < C.z - lim || p.z > C.z + lim) gm_wfix[r] = 1; }
    }
    /* per place: the triangles that meet there */
    memset(gm_ic, 0, (m->nv + 2u) * sizeof(U32));
    for (k = 0; k < nn; k++) for (v = 0; v < 3u; v++) gm_ic[gm_rep[m->tri[gm_wt[k] * 3 + v]] + 2u]++;
    for (v = 0; v < m->nv; v++) gm_ic[v + 2u] += gm_ic[v + 1u];
    for (k = 0; k < nn; k++) for (v = 0; v < 3u; v++) { U32 r = gm_rep[m->tri[gm_wt[k] * 3 + v]]; gm_wl[gm_ic[r + 1u]++] = (U16)k; }
    for (k = 0; k < nn; k++) {
        U32 e;
        t = gm_wt[k];
        if ((U32)m->aux[t] != rockAux) continue;
        for (e = 0; e < 3u; e++) {
            U32 a = gm_uf_find(gm_rep[m->tri[t * 3 + e]]), b = gm_uf_find(gm_rep[m->tri[t * 3 + (e + 1u) % 3u]]), j; int ok = 1; Vec3 pb;
            if (a == b || gm_dist2(m->pos[a], m->pos[b]) >= eps * eps) continue;
            if (gm_wfix[a]) { U32 x = a; a = b; b = x; }
            if (gm_wfix[a] || a != gm_rep[a]) continue;                      /* neither end may move (a place that has taken another in stays put) */
            pb = m->pos[b];
            for (j = gm_ic[a]; j < gm_ic[a + 1u] && ok; j++) {               /* would a triangle turn over? */
                U32 w = gm_wt[gm_wl[j]], q; Vec3 P[3], N[3], n0, n1; int hasB = 0;
                for (q = 0; q < 3u; q++) { U32 r = gm_uf_find(gm_rep[m->tri[w * 3 + q]]); P[q] = m->pos[r]; N[q] = r == a ? pb : P[q]; if (r == b) hasB = 1; }
                if (hasB) continue;                                           /* collapses: fine */
                n0 = gm_cross(gm_sub(P[1], P[0]), gm_sub(P[2], P[0])); n1 = gm_cross(gm_sub(N[1], N[0]), gm_sub(N[2], N[0]));
                if (gm_dot(n0, n1) <= 0.f) ok = 0;
            }
            if (!ok) continue;
            gm_uf[a] = (U16)b; gm_wfix[b] = 1; done++;
            break;
        }
    }
    if (done) for (v = 0; v < m->nv; v++) if (gm_rep[v] != 0xFFFFu) { U32 r = gm_uf_find(gm_rep[v]); if (r != gm_rep[v]) m->pos[v] = m->pos[r]; }
    return done;
}

/* Full crater. Returns 1 changed, 0 nothing inside the sphere, -1 out of capacity
 * (mesh contents are then undefined - the caller works on a scratch copy).
 * breakThrough: 1 = run gm_break (render winding) so that thin things are broken through.
 * moved: optional GM_MAXV flags, set for every vertex that was moved.
 * gm_after_cut: optional hook, called when the mesh is refined and cut along the sphere, before
 * anything is removed or moved - the place to flag triangles (m->flag) and set gm_clamp.
 * gm_stop_after_cut: return right after that (the mesh stays refined and cut, nothing moved). */
static void (*gm_after_cut)(GmMesh* m, Vec3 C, float R, Vec3 Q, GmStats* st);
static int gm_stop_after_cut;
static int gm_crater(GmMesh* m, Vec3 C, float R, Vec3 Q, float h, float shade, int breakThrough, U8* moved, GmStats* st) {
    U32 v;
    memset(st, 0, sizeof(*st));
    if (moved) memset(moved, 0, GM_MAXV);
    if (gm_refine(m, C, R, h, st) < 0) return -1;
    if (gm_cut(m, C, R, st) < 0) return -1;
    for (v = 0; v < m->nv; v++) gm_clamp[v] = -1.f;
    gm_break_failed = 0;
    if (breakThrough) st->shell = gm_break(m, C, R, Q, 0);
    if (gm_after_cut) gm_after_cut(m, C, R, Q, st);
    if (gm_break_failed) return -1;
    if (gm_stop_after_cut) return 0;
    gm_delete_holes(m, C, R, st);
    gm_project(m, C, R, Q, shade, moved, st);
    if (st->shell) st->debris = gm_drop_debris(m, C, R);
    return (st->moved || st->holeTris) ? 1 : 0;
}

/* Remove triangles with exactly zero area (call after quantisation). Returns how many were removed. */
static U32 gm_drop_degenerate(GmMesh* m) {
    U32 t, n = 0;
    for (t = 0; t < m->nt; t++) {
        Vec3 a = m->pos[m->tri[t * 3]], b = m->pos[m->tri[t * 3 + 1]], c = m->pos[m->tri[t * 3 + 2]];
        Vec3 x = gm_cross(gm_sub(b, a), gm_sub(c, a));
        if (x.x == 0.f && x.y == 0.f && x.z == 0.f) continue;
        if (n != t) {
            m->tri[n * 3] = m->tri[t * 3]; m->tri[n * 3 + 1] = m->tri[t * 3 + 1]; m->tri[n * 3 + 2] = m->tri[t * 3 + 2];
            m->aux[n] = m->aux[t];
            if (m->flag) m->flag[n] = m->flag[t];
        }
        n++;
    }
    t = m->nt - n; m->nt = n;
    return t;
}
#endif
