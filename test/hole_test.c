/* Offline test of hole cutting in thin geometry (fences, slabs, double-sided sheets). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
typedef unsigned int U32; typedef unsigned short U16; typedef unsigned char U8; typedef short S16;
typedef struct { float x, y, z; } Vec3;
#include "../src/crater_core.c"
static int fails, checks; static const char* name = "";
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; if (fails < 40) { printf("  FAIL %s:%d: ", name, __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static GmMesh M;
static void reset(void) {
    if (!M.pos) { M.pos = malloc(GM_MAXV * 12); M.tri = malloc(GM_MAXT * 6); M.aux = malloc(GM_MAXT * 2); M.flag = malloc(GM_MAXT); }
    M.nv = M.nt = 0; M.nf = M.nb = 0; M.normalAttr = -1;
}
/* rectangle patch: origin o, edges u (nu cells) and v (nv cells); normal = u x v (render winding) or flipped */
static void add_patch(Vec3 o, Vec3 u, Vec3 v, int nu, int nvv, int reverse, U16 aux) {
    U32 base = M.nv; int i, j;
    for (j = 0; j <= nvv; j++) for (i = 0; i <= nu; i++) {
        Vec3 p = { o.x + u.x * i / nu + v.x * j / nvv, o.y + u.y * i / nu + v.y * j / nvv, o.z + u.z * i / nu + v.z * j / nvv };
        M.pos[M.nv++] = p;
    }
    for (j = 0; j < nvv; j++) for (i = 0; i < nu; i++) {
        U32 a = base + j * (nu + 1) + i, b = a + 1, c = a + nu + 1, d = c + 1;
        U16* t = M.tri + M.nt * 3;
        if (!reverse) { t[0] = a; t[1] = b; t[2] = d; t[3] = a; t[4] = d; t[5] = c; } else { t[0] = a; t[1] = d; t[2] = b; t[3] = a; t[4] = c; t[5] = d; }
        M.aux[M.nt] = aux; M.aux[M.nt + 1] = aux; M.nt += 2;
    }
}
/* is point p (on the patch plane) covered by a triangle of the mesh that lies in that plane? */
static int covered(Vec3 p, Vec3 n) {
    U32 t;
    for (t = 0; t < M.nt; t++) {
        Vec3 a = M.pos[M.tri[t * 3]], b = M.pos[M.tri[t * 3 + 1]], c = M.pos[M.tri[t * 3 + 2]], q; float w[3];
        if (fabsf(gm_dot(gm_sub(a, p), n)) > 0.01f || fabsf(gm_dot(gm_sub(b, p), n)) > 0.01f || fabsf(gm_dot(gm_sub(c, p), n)) > 0.01f) continue;
        q = gm_closest_on_tri(p, a, b, c, w);
        if (gm_dist2(p, q) < 1e-6f) return 1;
    }
    return 0;
}
/* sample the patch: inside the sphere it must be gone (hole=1) or still there (hole=0); outside it must be intact */
static void check_patch(const char* what, Vec3 o, Vec3 u, Vec3 v, Vec3 C, float R, int hole) {
    Vec3 n = gm_cross(u, v); float l = sqrtf(gm_dot(n, n)), lu = sqrtf(gm_dot(u, u)), lv = sqrtf(gm_dot(v, v)), s, t; int badIn = 0, badOut = 0, nIn = 0;
    n.x /= l; n.y /= l; n.z /= l;
    for (s = 0.07f; s < lu; s += 0.19f) for (t = 0.07f; t < lv; t += 0.19f) {
        Vec3 p = { o.x + u.x * s / lu + v.x * t / lv, o.y + u.y * s / lu + v.y * t / lv, o.z + u.z * s / lu + v.z * t / lv };
        float d = sqrtf(gm_dist2(p, C)); int cov;
        if (fabsf(d - R) < 0.25f) continue;                 /* rim zone: tolerance band + chord error */
        cov = covered(p, n);
        if (d < R) { nIn++; if (hole ? cov : 0) badIn++; } else if (!cov) badOut++;
    }
    CHECK(badOut == 0, "%s: %d sample points outside the sphere lost their surface", what, badOut);
    if (hole) CHECK(nIn > 20 && badIn == 0, "%s: %d of %d sample points inside the sphere still covered (no hole)", what, badIn, nIn);
}
static void check_mesh(const char* what) {
    U32 i; for (i = 0; i < M.nt * 3; i++) if (M.tri[i] >= M.nv) { CHECK(0, "%s: index out of range", what); return; }
}
/* every edge that became a border must lie on the sphere (the rim of the hole) */
static void check_rim(const char* what, Vec3 C, float R, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax) {
    static unsigned long long e[GM_MAXT * 3]; U32 t, n = 0, i, bad = 0;
    for (t = 0; t < M.nt; t++) { int k; for (k = 0; k < 3; k++) { U32 a = M.tri[t * 3 + k], b = M.tri[t * 3 + (k + 1) % 3]; if (a > b) { U32 x = a; a = b; b = x; } e[n++] = ((unsigned long long)a << 32) | b; } }
    for (i = 0; i < n; i++) { U32 j, cnt = 0; for (j = 0; j < n; j++) if (e[j] == e[i]) cnt++;
        if (cnt == 1) { Vec3 a = M.pos[e[i] >> 32], b = M.pos[e[i] & 0xFFFFFFFFu]; Vec3 mid = { (a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2 };
            int outer = mid.x <= xmin + 1e-3f || mid.x >= xmax - 1e-3f || mid.y <= ymin + 1e-3f || mid.y >= ymax - 1e-3f || mid.z <= zmin + 1e-3f || mid.z >= zmax - 1e-3f;
            if (!outer && (gm_class(a, C, R) != 0 || gm_class(b, C, R) != 0)) bad++; } }
    CHECK(bad == 0, "%s: %u open edges that are not on the crater rim (crack)", what, bad);
}
/* tears: an open edge near the crater whose two end points are not also the end points of another
 * open edge (the border of the neighbouring patch). Counts them. */
static U32 count_tears(Vec3 C, float R) {
    static U32 ea[GM_MAXT * 3], eb[GM_MAXT * 3]; static U8 open[GM_MAXT * 3]; U32 n = 0, t, i, j, tears = 0;
    for (t = 0; t < M.nt; t++) { int k; for (k = 0; k < 3; k++) { ea[n] = M.tri[t * 3 + k]; eb[n] = M.tri[t * 3 + (k + 1) % 3]; n++; } }
    for (i = 0; i < n; i++) { open[i] = 1; for (j = 0; j < n; j++) if (j != i && ((ea[j] == eb[i] && eb[j] == ea[i]) || (ea[j] == ea[i] && eb[j] == eb[i]))) { open[i] = 0; break; } }
    for (i = 0; i < n; i++) if (open[i]) {
        Vec3 a = M.pos[ea[i]], b = M.pos[eb[i]], mid = { (a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2 }; int twin = 0;
        if (gm_dist2(mid, C) > (R + 0.3f) * (R + 0.3f)) continue;
        for (j = 0; j < n && !twin; j++) if (open[j] && j != i) {
            Vec3 c = M.pos[ea[j]], d = M.pos[eb[j]];
            if ((gm_dist2(a, c) < 1e-10f && gm_dist2(b, d) < 1e-10f) || (gm_dist2(a, d) < 1e-10f && gm_dist2(b, c) < 1e-10f)) twin = 1;
        }
        if (!twin) tears++;
    }
    return tears;
}
int main(void) {
    GmStats st; U8 sheet[256]; Vec3 C, Q; float R; U32 marked; int FS = 0;
    memset(sheet, 0, sizeof(sheet)); sheet[55] = 1;
    /* T1: wire fence, single layer, collision winding, see-through surface. Blast 1 m in front of it. */
    name = "T1 fence"; reset();
    { Vec3 o = { 0, -10, 0 }, u = { 0, 20, 0 }, v = { 0, 0, 6 }; add_patch(o, u, v, 4, 2, 1, 55 | (3 << 8));
      C.x = 1.f; C.y = 0.4f; C.z = 2.5f; R = 3.f; Q.x = C.x + R; Q.y = C.y; Q.z = C.z;
      marked = gm_mark_thin(&M, C, R, 0.f, 1, sheet);
      CHECK(marked > 0, "fence not recognised as a sheet");
      CHECK(gm_crater(&M, C, R, Q, 1.0f, 1.f, FS, NULL, &st) == 1 && st.holeTris > 0 && st.moved == 0, "fence: holeTris=%u moved=%u", st.holeTris, st.moved);
      check_mesh(name); check_patch(name, o, u, v, C, R, 1); check_rim(name, C, R, -1, 1, -10, 10, 0, 6); }
    /* T1b: the same fence with a normal (solid) surface type and nothing behind it: stays a solid, gets a bowl */
    name = "T1b solid sheet"; reset();
    { Vec3 o = { 0, -10, 0 }, u = { 0, 20, 0 }, v = { 0, 0, 6 }; add_patch(o, u, v, 4, 2, 1, 4);
      marked = gm_mark_thin(&M, C, R, 0.f, 1, sheet);
      CHECK(marked == 0, "plain single sheet wrongly flagged");
      Q.x = C.x + R; CHECK(gm_crater(&M, C, R, Q, 1.0f, 1.f, FS, NULL, &st) == 1 && st.holeTris == 0 && st.moved > 0, "solid sheet: holeTris=%u moved=%u", st.holeTris, st.moved); check_mesh(name); }
    /* T2: slab 0.3 m thick (balcony, canopy): top faces up, bottom faces down, air below. Grenade on top,
     *     crater 0.9 m deep: it goes THROUGH. An opening in both faces, a rock wall between them, and
     *     nothing pushed out below the slab (that was the "tunnel in the air"). */
    name = "T2 thin slab"; reset(); FS = 1; gm_solid_aux = 0xFFFFFFFFu;
    { Vec3 o = { -10, -10, 0 }, u = { 20, 0, 0 }, v = { 0, 20, 0 }, ob = { -10, -10, -0.3f }; float r = 2.2f, d = 0.9f; U32 i, below = 0, wall = 0; Vec3 up = { 0, 0, 1 }, pc;
      add_patch(o, u, v, 4, 4, 0, 1); add_patch(ob, u, v, 3, 3, 1, 9);
      R = (r * r + d * d) / (2 * d); C.x = 0.7f; C.y = 0.3f; C.z = R - d; Q = C; Q.z += R;
      CHECK(gm_crater(&M, C, R, Q, 1.0f, 1.f, FS, NULL, &st) == 1 && st.holeTris > 0 && st.shell > 0, "slab: holeTris=%u shell=%u moved=%u", st.holeTris, st.shell, st.moved);
      check_mesh(name);
      for (i = 0; i < M.nt * 3; i++) { Vec3 p = M.pos[M.tri[i]]; if (p.z < -0.301f) below++; if (p.z < -0.05f && p.z > -0.25f) wall++; }
      CHECK(below == 0, "slab: %u vertex uses below the slab (pushed out of its far side)", below);
      pc.x = C.x; pc.y = C.y; pc.z = 0.f; CHECK(!covered(pc, up), "slab: top face still closed at the centre of the crater");
      pc.z = -0.3f; CHECK(!covered(pc, up), "slab: bottom face still closed under the centre of the crater");
      pc.x = C.x + 6.f; pc.z = 0.f; CHECK(covered(pc, up), "slab: top face damaged outside the crater");
      pc.z = -0.3f; CHECK(covered(pc, up), "slab: bottom face damaged outside the crater");
      CHECK(st.moved > 0, "slab: no wall of the hole (nothing moved)"); (void)wall; }
    /* T2b: the same with the break-through logic off (collision meshes without help): the bottom is pushed out. */
    name = "T2b slab, logic off"; reset(); FS = 0;
    { Vec3 o = { -10, -10, 0 }, u = { 20, 0, 0 }, v = { 0, 20, 0 }, ob = { -10, -10, -0.3f }; U32 t, pushed = 0;
      add_patch(o, u, v, 4, 4, 0, 1); add_patch(ob, u, v, 3, 3, 1, 9);
      CHECK(gm_crater(&M, C, R, Q, 1.0f, 1.f, FS, NULL, &st) == 1 && st.shell == 0, "slab without the logic: shell=%u", st.shell);
      for (t = 0; t < M.nt; t++) if (M.aux[t] == 9 && M.pos[M.tri[t * 3]].z < -0.35f) pushed++;
      CHECK(pushed > 0, "expected the far side to move when the logic is off"); }
    /* T2c: kerb on the ground, 0.4 m wide and 0.3 m high, blast beside it. Its far side has the ground
     *      behind it: solid. The whole kerb collapses into the bowl, nothing is left standing, no hole. */
    name = "T2c kerb"; reset(); FS = 1;
    { Vec3 g1 = { -30, -30, 0 }, g1u = { 30.6f, 0, 0 }, gv = { 0, 60, 0 }, g2 = { 1.0f, -30, 0 }, g2u = { 29.f, 0, 0 };
      Vec3 kt = { 0.6f, -30, 0.3f }, ktu = { 0.4f, 0, 0 }, kn = { 0.6f, -30, 0 }, kz = { 0, 0, 0.3f }, kf = { 1.0f, -30, 0 }; float r = 2.2f, d = 0.9f; U32 i, left = 0;
      add_patch(g1, g1u, gv, 2, 2, 0, 1); add_patch(g2, g2u, gv, 2, 2, 0, 1);       /* ground on both sides */
      add_patch(kt, ktu, gv, 1, 2, 0, 1);                                            /* kerb top */
      add_patch(kn, gv, kz, 2, 1, 1, 1);                                             /* near side, faces -x */
      add_patch(kf, gv, kz, 2, 1, 0, 1);                                             /* far side, faces +x */
      R = (r * r + d * d) / (2 * d); C.x = 0.f; C.y = 0.3f; C.z = R - d; Q = C; Q.z += R;
      CHECK(gm_crater(&M, C, R, Q, 1.0f, 1.f, FS, NULL, &st) == 1 && st.holeTris == 0 && st.shell == 0 && st.moved > 0, "kerb: holeTris=%u shell=%u moved=%u", st.holeTris, st.shell, st.moved);
      check_mesh(name);
      for (i = 0; i < M.nv; i++) if (gm_dist2(M.pos[i], C) < (R - GM_EPS - 0.01f) * (R - GM_EPS - 0.01f) && M.pos[i].z < C.z) left++;
      CHECK(left == 0, "kerb: %u vertices left standing inside the crater", left); }
    /* T2d: top of a 4 m retaining wall, blast on the upper ground 1 m from the edge. The crater bites the
     *      corner off: nothing may be pushed out beyond the wall face (x > 1), and the bite must be
     *      closed by rock - looking at the wall from outside there is no way to see inside. */
    name = "T2d wall edge"; reset(); FS = 1;
    { Vec3 g = { -30, -30, 0 }, gu = { 31.f, 0, 0 }, gv = { 0, 60, 0 }, w = { 1.f, -30, -4.f }, wz = { 0, 0, 4.f }; float r = 2.2f, d = 0.9f; U32 i, out = 0, t, open = 0;
      add_patch(g, gu, gv, 2, 2, 0, 1); add_patch(w, gv, wz, 2, 2, 0, 9);           /* wall at x = 1, faces +x */
      R = (r * r + d * d) / (2 * d); C.x = 0.f; C.y = 0.3f; C.z = R - d; Q = C; Q.z += R;
      CHECK(gm_crater(&M, C, R, Q, 1.0f, 1.f, FS, NULL, &st) == 1 && st.moved > 0, "wall edge: moved=%u", st.moved);
      check_mesh(name);
      for (i = 0; i < M.nt * 3; i++) if (M.pos[M.tri[i]].x > 1.02f) out++;
      CHECK(out == 0, "wall edge: %u vertex uses pushed out beyond the wall face", out);
      /* rays from outside (x = 6) towards the bite: each must hit something before it reaches x = -3 */
      { float yy, zz; for (yy = -1.2f; yy <= 1.8f; yy += 0.3f) for (zz = -0.7f; zz <= -0.05f; zz += 0.13f) {
            Vec3 ro = { 6.f, yy, zz }, rd = { -1.f, 0.f, 0.f }; float best = 1e9f;
            for (t = 0; t < M.nt; t++) { float tt = gm_ray_tri(ro, rd, M.pos[M.tri[t * 3]], M.pos[M.tri[t * 3 + 1]], M.pos[M.tri[t * 3 + 2]]); if (tt > 0.f && tt < best) best = tt; }
            if (best > 9.f) open++; } }
      CHECK(open == 0, "wall edge: %u of 66 sight lines go through the bite into the inside", open);
      /* and with the logic off the face bulges (so the checks above mean something) */
      reset(); add_patch(g, gu, gv, 2, 2, 0, 1); add_patch(w, gv, wz, 2, 2, 0, 9); out = 0;
      gm_crater(&M, C, R, Q, 1.0f, 1.f, 0, NULL, &st);
      for (i = 0; i < M.nt * 3; i++) if (M.pos[M.tri[i]].x > 1.02f) out++;
      CHECK(out > 0, "wall edge: expected a bulge with the logic off"); }
    FS = 0;
    /* T3: slab 3 m thick: thicker than the crater is deep -> bowl in the top face, bottom untouched */
    name = "T3 thick slab"; reset();
    { Vec3 o = { -10, -10, 0 }, u = { 20, 0, 0 }, v = { 0, 20, 0 }, ob = { -10, -10, -3.f };
      add_patch(o, u, v, 4, 4, 0, 1); add_patch(ob, u, v, 3, 3, 1, 1);
      marked = gm_mark_thin(&M, C, R, 0.12f, 0, NULL);
      CHECK(marked == 0, "thick slab wrongly flagged (%u)", marked);
      CHECK(gm_crater(&M, C, R, Q, 1.0f, 1.f, FS, NULL, &st) == 1 && st.holeTris == 0 && st.moved > 0, "thick slab: holeTris=%u moved=%u", st.holeTris, st.moved);
      check_mesh(name); check_patch("T3 bottom", ob, u, v, C, R, 0); }
    /* T4: plain ground */
    name = "T4 ground"; reset();
    { Vec3 o = { -30, -30, 0 }, u = { 60, 0, 0 }, v = { 0, 60, 0 }; add_patch(o, u, v, 2, 2, 0, 1);
      CHECK(gm_mark_thin(&M, C, R, 0.12f, 0, NULL) == 0, "ground wrongly flagged");
      CHECK(gm_crater(&M, C, R, Q, 1.0f, 1.f, FS, NULL, &st) == 1 && st.holeTris == 0 && st.moved > 0, "ground: no bowl"); check_mesh(name); }
    /* T5: double-sided sheet (two coincident layers, opposite winding), no surface information */
    name = "T5 double-sided"; reset();
    { Vec3 o = { 0, -10, 0 }, u = { 0, 20, 0 }, v = { 0, 0, 6 }; add_patch(o, u, v, 3, 2, 0, 2); add_patch(o, u, v, 3, 2, 1, 2);
      C.x = 1.f; C.y = 0.4f; C.z = 2.5f; R = 3.f; Q = C; Q.x += R;
      marked = gm_mark_thin(&M, C, R, 0.12f, 0, NULL);
      CHECK(marked >= 4, "double-sided sheet not recognised (%u)", marked);
      CHECK(gm_crater(&M, C, R, Q, 1.0f, 1.f, FS, NULL, &st) == 1 && st.holeTris > 0, "double-sided: no hole");
      check_mesh(name); check_patch(name, o, u, v, C, R, 1); }
    /* T6: fence standing on ground in ONE mesh: hole in the fence, bowl in the ground */
    name = "T6 fence on ground"; reset();
    { Vec3 o = { 0, -10, 0 }, u = { 0, 20, 0 }, v = { 0, 0, 6 }, g = { -30, -30, 0 }, gu = { 60, 0, 0 }, gv = { 0, 60, 0 }; float r = 2.2f, d = 0.9f; U32 i, below = 0;
      add_patch(g, gu, gv, 3, 3, 1, 4); add_patch(o, u, v, 4, 2, 1, 55);          /* collision winding */
      R = (r * r + d * d) / (2 * d); C.x = 0.8f; C.y = 0.2f; C.z = R - d; Q = C; Q.z += R;
      marked = gm_mark_thin(&M, C, R, 0.f, 1, sheet);
      CHECK(gm_crater(&M, C, R, Q, 1.0f, 1.f, FS, NULL, &st) == 1 && st.holeTris > 0 && st.moved > 0, "fence on ground: holeTris=%u moved=%u", st.holeTris, st.moved);
      check_mesh(name); check_patch("T6 fence", o, u, v, C, R, 1);
      for (i = 0; i < M.nv; i++) if (M.pos[i].z < -0.3f) below++;
      CHECK(below > 5, "ground bowl missing (%u vertices below ground)", below); }
    /* T7: a roof made of two layers back to back, over a real room (floor and four walls that look
     *     inwards): the blast opens it. Over a building that is only a skin (walls look outwards,
     *     nothing inside) gm_enclosed says so - the caller then keeps it solid. */
    name = "T7 two-layer roof"; FS = 1; gm_solid_aux = 0xFFFFFFFFu;
    { int hollow; for (hollow = 0; hollow < 2; hollow++) {
      Vec3 o = { -10, -10, 8 }, u = { 20, 0, 0 }, v = { 0, 20, 0 }, fl = { -10, -10, 0 }, up = { 0, 0, 1 }, pc; float r = 2.2f, d = 0.9f; U32 i, low = 0, n;
      Vec3 wz = { 0, 0, 8 }, w1 = { -10, -10, 0 }, w2 = { 10, -10, 0 }, w3 = { -10, 10, 0 }, wx = { 20, 0, 0 }, wy = { 0, 20, 0 };
      reset();
      add_patch(o, u, v, 4, 4, 0, 1); add_patch(o, u, v, 4, 4, 1, 9);               /* roof: top layer, bottom layer */
      if (!hollow) add_patch(fl, u, v, 2, 2, 0, 1);                                  /* floor */
      add_patch(w1, wy, wz, 2, 1, hollow ? 1 : 0, 1);                                /* wall x = -10: normal +x (inwards) / -x */
      add_patch(w2, wy, wz, 2, 1, hollow ? 0 : 1, 1);                                /* wall x = +10 */
      add_patch(w1, wx, wz, 2, 1, hollow ? 0 : 1, 1);                                /* wall y = -10: normal +y inwards needs reverse */
      add_patch(w3, wx, wz, 2, 1, hollow ? 1 : 0, 1);                                /* wall y = +10 */
      R = (r * r + d * d) / (2 * d); C.x = 0.7f; C.y = 0.3f; C.z = 8.f + R - d; Q = C; Q.z += R;
      memset(M.flag, 0, GM_MAXT);
      CHECK(gm_refine(&M, C, R, 1.3f, &st) == 0 && gm_cut(&M, C, R, &st) == 0, "refine/cut failed");
      n = gm_break(&M, C, R, Q, 0);
      CHECK(n > 0, "two-layer roof (%s): no opening found", hollow ? "skin" : "room");
      CHECK(gm_enclosed(&M, gm_shell_p, gm_shell_d, 0, 1) == hollow, "two-layer roof: %s wrongly judged", hollow ? "a building that is only a skin" : "a real room");
      if (hollow) continue;
      gm_delete_holes(&M, C, R, &st); gm_project(&M, C, R, Q, 1.f, NULL, &st);
      check_mesh(name);
      for (i = 0; i < M.nt * 3; i++) if (M.pos[M.tri[i]].z < 7.99f && M.pos[M.tri[i]].z > 7.0f) low++;
      CHECK(low == 0, "two-layer roof: %u vertex uses hang below the roof", low);
      pc.x = C.x; pc.y = C.y; pc.z = 8.f; CHECK(!covered(pc, up), "two-layer roof: still closed at the centre"); } }
    /* T7b: the same roof, but its two layers do not share a single vertex (4x4 against 3x3
     *      quads) and lie on top of each other or a little apart: thin metal as the game has it. */
    name = "T7b roof layers apart"; FS = 1; gm_solid_aux = 0xFFFFFFFFu;
    { int gi; static const float gap[5] = { 0.f, 0.002f, 0.02f, 0.1f, 0.4f };
      for (gi = 0; gi < 5; gi++) {
      Vec3 o = { -10, -10, 8 }, o2 = { -10, -10, 8 }, u = { 20, 0, 0 }, v = { 0, 20, 0 }, fl = { -10, -10, 0 }, up = { 0, 0, 1 }, pc; float r = 2.2f, d = 0.9f; U32 i, low = 0, n;
      reset(); o2.z -= gap[gi];
      add_patch(o, u, v, 4, 4, 0, 1); add_patch(o2, u, v, 3, 3, 1, 9); add_patch(fl, u, v, 2, 2, 0, 1);
      R = (r * r + d * d) / (2 * d); C.x = 0.7f; C.y = 0.3f; C.z = 8.f + R - d; Q = C; Q.z += R;
      memset(M.flag, 0, GM_MAXT);
      CHECK(gm_refine(&M, C, R, 1.3f, &st) == 0 && gm_cut(&M, C, R, &st) == 0, "refine/cut failed");
      n = gm_break(&M, C, R, Q, 0);
      CHECK(n > 0, "roof layers %.3f apart: no opening found", gap[gi]);
      gm_delete_holes(&M, C, R, &st); gm_project(&M, C, R, Q, 1.f, NULL, &st);
      check_mesh(name);
      for (i = 0; i < M.nt * 3; i++) if (M.pos[M.tri[i]].z < 8.f - gap[gi] - 0.01f && M.pos[M.tri[i]].z > 7.0f) low++;
      CHECK(low == 0, "roof layers %.3f apart: %u vertex uses hang below the roof", gap[gi], low);
      pc.x = C.x; pc.y = C.y; pc.z = 8.f; CHECK(!covered(pc, up), "roof layers %.3f apart: still closed at the centre", gap[gi]);
      pc.x = C.x + 1.2f; pc.y = C.y - 0.8f; CHECK(!covered(pc, up), "roof layers %.3f apart: still closed off centre", gap[gi]); } }
    /* T8: a pole, 0.6 x 0.6 m and 8 m high, standing on the ground; rocket from the side at 3 m height.
     *     The pole is cut through: nothing of it is left inside the crater, and the two ends are closed. */
    name = "T8 pole"; reset(); FS = 1;
    { Vec3 g = { -30, -30, 0 }, gu = { 60, 0, 0 }, gv = { 0, 60, 0 }, z8 = { 0, 0, 8 }, ex = { 0.6f, 0, 0 }, ey = { 0, 0.6f, 0 };
      Vec3 a0 = { -0.3f, -0.3f, 0 }, a1 = { 0.3f, -0.3f, 0 }, a3 = { -0.3f, 0.3f, 0 }; float r = 2.8f, d = 1.2f; U32 i, left = 0, t, open = 0;
      add_patch(g, gu, gv, 2, 2, 0, 1);
      add_patch(a0, ex, z8, 1, 4, 0, 9);        /* y = -0.3: normal ex x ez = -y */
      add_patch(a1, ey, z8, 1, 4, 0, 9);        /* x = +0.3: normal ey x ez = +x */
      add_patch(a3, ex, z8, 1, 4, 1, 9);        /* y = +0.3: normal +y */
      add_patch(a0, ey, z8, 1, 4, 1, 9);        /* x = -0.3: normal -x */
      R = (r * r + d * d) / (2 * d); C.x = -0.3f - (R - d); C.y = 0.05f; C.z = 3.f; Q = C; Q.x -= R;      /* blast from -x */
      CHECK(gm_crater(&M, C, R, Q, 1.2f, 1.f, FS, NULL, &st) == 1 && st.shell > 0, "pole: shell=%u holeTris=%u moved=%u", st.shell, st.holeTris, st.moved);
      check_mesh(name);
      /* cut through: between 2.3 m and 3.7 m nothing of the pole is left (the ends above and below are capped) */
      for (t = 0; t < M.nt; t++) if (M.aux[t] == 9) for (i = 0; i < 3; i++) { float z = M.pos[M.tri[t * 3 + i]].z; if (z > 2.3f && z < 3.7f) left++; }
      CHECK(left == 0, "pole: %u vertex uses of the pole left in the middle of the crater - not cut through", left);
      { U32 lower = 0, upper = 0; for (t = 0; t < M.nt; t++) if (M.aux[t] == 9) { if (M.pos[M.tri[t * 3]].z < 2.3f) lower++; else upper++; }
        CHECK(lower > 4 && upper > 4, "pole: stump (%u triangles) or top (%u) missing", lower, upper); }
      /* the stump is closed on top: looking down the pole's axis from the cut, one does not see into it */
      { Vec3 ro = { 0.05f, 0.03f, 2.9f }, rd = { 0, 0, -1 }; float best = 1e9f; for (t = 0; t < M.nt; t++) if (M.aux[t] == 9) { float tt = gm_ray_tri(ro, rd, M.pos[M.tri[t * 3]], M.pos[M.tri[t * 3 + 1]], M.pos[M.tri[t * 3 + 2]]); if (tt > 0.f && tt < best) best = tt; }
        CHECK(best < 2.5f, "pole: the stump is open on top"); }
      for (i = 0; i < M.nt * 3; i++) if (M.pos[M.tri[i]].x > 0.31f && M.pos[M.tri[i]].z > 0.5f) open++;
      CHECK(open == 0, "pole: %u vertex uses pushed out behind the pole", open); }
    /* T10: a bridge deck 1.6 m thick, road below. One rocket leaves a bowl (the deck is thicker than the
     *      crater is deep). The second, into that bowl, goes through: an opening in the underside,
     *      rock walls around it, and nothing hanging below the deck. */
    name = "T10 bridge"; reset(); FS = 1;
    { Vec3 o = { -15, -15, 0 }, u = { 30, 0, 0 }, v = { 0, 30, 0 }, ob = { -15, -15, -1.6f }, up = { 0, 0, 1 }, pc; float r = 2.8f, d = 1.2f, Mx[8] = { 0.3f, 0, 0, 0, 0, 0.3f, 0, 0 }; static U8 mv[GM_MAXV]; static float uvbuf[GM_MAXV * 2]; U32 i, below = 0, shell1;
      add_patch(o, u, v, 4, 4, 0, 1); add_patch(ob, u, v, 3, 3, 1, 9);
      M.nf = 1; M.fa[0] = uvbuf; M.fdim[0] = 2; memset(uvbuf, 0, sizeof(uvbuf)); gm_solid_aux = 77u;
      R = (r * r + d * d) / (2 * d); C.x = 0.4f; C.y = 0.2f; C.z = R - d; Q = C; Q.z += R;
      CHECK(gm_crater(&M, C, R, Q, 1.6f, 1.f, FS, mv, &st) == 1 && st.moved > 0, "bridge: first blast moved nothing"); shell1 = st.shell;
      CHECK(shell1 == 0 && st.holeTris == 0, "bridge: first blast already went through (shell=%u)", shell1);
      CHECK(gm_bowl_material(&M, mv, 77, 0, Mx, &st) == 0, "bowl material failed");
      pc.x = C.x; pc.y = C.y; pc.z = -1.6f; CHECK(covered(pc, up), "bridge: underside damaged by the first blast");
      C.z = R - d - 1.1f; Q = C; Q.z += R;                                         /* second blast, lying in the bowl */
      CHECK(gm_crater(&M, C, R, Q, 1.6f, 1.f, FS, mv, &st) == 1 && st.shell > 0 && st.holeTris > 0, "bridge: second blast did not go through (shell=%u holes=%u)", st.shell, st.holeTris);
      CHECK(gm_bowl_material(&M, mv, 77, 0, Mx, &st) == 0, "bowl material failed");
      check_mesh(name);
      for (i = 0; i < M.nt * 3; i++) if (M.pos[M.tri[i]].z < -1.601f) below++;
      CHECK(below == 0, "bridge: %u vertex uses hang below the deck", below);
      pc.z = -1.6f; CHECK(!covered(pc, up), "bridge: underside still closed under the crater");
      pc.x = C.x + 7.f; CHECK(covered(pc, up), "bridge: underside damaged outside the crater");
      /* from below, looking up through the opening: sky, not rock */
      { Vec3 ro = { C.x, C.y, -5.f }, rd = { 0, 0, 1 }; float best = 1e9f; U32 t; for (t = 0; t < M.nt; t++) { float tt = gm_ray_tri(ro, rd, M.pos[M.tri[t * 3]], M.pos[M.tri[t * 3 + 1]], M.pos[M.tri[t * 3 + 2]]); if (tt > 0.f && tt < best) best = tt; }
        CHECK(best > 1e8f, "bridge: the hole is blocked %.2f m above the road", best); }
      gm_solid_aux = 0xFFFFFFFFu; M.nf = 0; }
    /* T9: a tunnel - eight rockets one after the other down a slope into flat ground. Crater rock is
     *     solid: no part of an older bowl (the tunnel's ceiling!) is left standing as a "far side",
     *     and nothing tears. And the rock folds where craters overlap: judged by shape it looks
     *     "thin" - v3.4 cut holes into it there (the slits seen in game). No shape rule any more. */
    name = "T9 tunnel"; reset(); FS = 1;
    { Vec3 g = { -30, -30, 0 }, gu = { 60, 0, 0 }, gv = { 0, 60, 0 }, E = { 0, 0, 0.2f }, dir = { 0.8f, 0, -0.6f }; float r = 2.5f, d = 1.07f, Mx[8] = { 0.3f, 0, 0, 0, 0, 0.3f, 0, 0 }; static U8 mv[GM_MAXV];
      static Vec3 save[GM_MAXV]; static U16 saveT[GM_MAXT * 3], saveA[GM_MAXT]; static float uvbuf[GM_MAXV * 2]; U32 snv, snt, k, t, keptOff = 0, keptOn = 0, tears = 0, oldRule = 0;
      add_patch(g, gu, gv, 2, 2, 0, 1); M.nf = 1; M.fa[0] = uvbuf; M.fdim[0] = 2; memset(uvbuf, 0, sizeof(uvbuf));
      R = (r * r + d * d) / (2 * d);
      for (k = 0; k < 8; k++) {
          Vec3 S = { 0, 0, 0 }, ns = { 0, 0, 0 }, o = { -6, 0, 5 }; float best = 1e30f, len, tb = 1e30f;
          gm_probe(&M, E, 3.5f, 0, 0, &best, &S, &ns); len = sqrtf(gm_dot(ns, ns));
          CHECK(best < 1e20f && len > 1e-6f, "tunnel: no surface at step %u", k); if (!(len > 1e-6f)) break;
          ns.x /= len; ns.y /= len; ns.z /= len;
          C.x = S.x + ns.x * (R - d); C.y = S.y + ns.y * (R - d); C.z = S.z + ns.z * (R - d); Q.x = C.x + ns.x * R; Q.y = C.y + ns.y * R; Q.z = C.z + ns.z * R;
          oldRule += gm_mark_thin(&M, C, R, 0.12f, 0, NULL); memset(M.flag, 0, M.nt);      /* what v3.4 would have holed */
          snv = M.nv; snt = M.nt; memcpy(save, M.pos, snv * 12); memcpy(saveT, M.tri, snt * 6); memcpy(saveA, M.aux, snt * 2);
          gm_solid_aux = 0xFFFFFFFFu; gm_crater(&M, C, R, Q, 1.5f, 1.f, FS, mv, &st); keptOff += st.shell;
          M.nv = snv; M.nt = snt; memcpy(M.pos, save, snv * 12); memcpy(M.tri, saveT, snt * 6); memcpy(M.aux, saveA, snt * 2);
          gm_solid_aux = 77u;
          CHECK(gm_crater(&M, C, R, Q, 1.5f, 1.f, FS, mv, &st) == 1 && st.moved > 0, "tunnel: step %u moved nothing", k); keptOn += st.shell + st.holeTris;
          CHECK(gm_bowl_material(&M, mv, 77, 0, Mx, &st) == 0, "bowl material failed");
          gm_drop_degenerate(&M);
          if (k == 7) tears = count_tears(C, R);
          for (t = 0; t < M.nt; t++) { float tt = gm_ray_tri(o, dir, M.pos[M.tri[t * 3]], M.pos[M.tri[t * 3 + 1]], M.pos[M.tri[t * 3 + 2]]); if (tt > 0.01f && tt < tb) tb = tt; }
          E.x = o.x + dir.x * (tb - 0.2f); E.y = 0.3f * sinf((float)k * 1.3f); E.z = o.z + dir.z * (tb - 0.2f);
      }
      CHECK(keptOn == 0, "tunnel: solid ground was opened %u times", keptOn);
      (void)keptOff;
      CHECK(oldRule > 0, "tunnel: the scene does not reproduce the slits (the old thin-rule flags nothing)");
      CHECK(tears == 0, "tunnel: %u torn edges", tears);
      gm_solid_aux = 0xFFFFFFFFu; M.nf = 0; }
    FS = 0;
    printf("hole_test checks=%d fails=%d\n", checks, fails);
    return fails ? 1 : 0;
}
