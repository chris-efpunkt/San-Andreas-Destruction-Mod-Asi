/* Offline test of the crater geometry core on real GTA SA meshes (render + collision).
 * usage: core_test model.bin [-v]
 * Every stage (refine, cut, project) is checked separately:
 *   - topology unchanged (Euler characteristic of the index complex and of the seam-welded complex)
 *   - refine/cut do not change the surface (area, orientation of every piece vs. its parent)
 *   - seam duplicates stay bit-identical
 *   - after projection nothing is left inside the sphere, vertices outside are untouched
 */
#define GM_TRACE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
typedef unsigned int U32; typedef unsigned short U16; typedef unsigned char U8; typedef short S16;
typedef struct { float x, y, z; } Vec3;
#include "../src/crater_core.c"

typedef struct { U32 a, b, c; U32 cls; U8 kind, used; } Key;
#define KEYS (1u << 19)
typedef struct {
    U32 cls[GM_MAXV], ncls;
    U32 orig[GM_MAXT];
    Key* keys;
} Trace;
static Trace* T;
static U32 key_class(U32 a, U32 b, U32 c, int kind) {
    U32 h = (a * 73856093u ^ b * 19349663u ^ c * 83492791u ^ (U32)kind * 2654435761u) & (KEYS - 1);
    for (;;) {
        Key* k = &T->keys[h];
        if (!k->used) { k->used = 1; k->a = a; k->b = b; k->c = c; k->kind = (U8)kind; k->cls = T->ncls++; return k->cls; }
        if (k->a == a && k->b == b && k->c == c && k->kind == kind) return k->cls;
        h = (h + 1) & (KEYS - 1);
    }
}
static void gm_trace_vertex(U32 n, U32 a, U32 b, U32 c, int kind) {
    if (kind == 1) {                                            /* foot point: one class per (welded) triangle */
        U32 x = T->cls[a], y = T->cls[b], z = T->cls[c], s;
        if (x > y) { s = x; x = y; y = s; } if (y > z) { s = y; y = z; z = s; } if (x > y) { s = x; x = y; y = s; }
        T->cls[n] = key_class(x, y, z, 1); return;
    }
    if (kind == 4) { T->cls[n] = T->cls[a]; return; }           /* copy of a vertex: same place */
    { U32 ca = T->cls[a], cb = T->cls[b]; if (ca > cb) { U32 s = ca; ca = cb; cb = s; } T->cls[n] = key_class(ca, cb, 0, kind); }
    (void)c;
}
static void gm_trace_tri(U32 n, U32 parent) { T->orig[n] = T->orig[parent]; }

static const char* name; static int fails, checks, verbose;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; if (fails < 30) { printf("  FAIL %s:%d: ", name, __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

/* ---- helpers ---- */
static int cmp_u64(const void* a, const void* b) { unsigned long long x = *(const unsigned long long*)a, y = *(const unsigned long long*)b; return x < y ? -1 : x > y; }
static unsigned long long* ebuf;
typedef struct { U32 a, b, c; } Face;
static Face* fbuf;
static int cmp_face(const void* x, const void* y) { return memcmp(x, y, sizeof(Face)); }
/* Euler characteristic V - E + F over ids[]; returns also boundary edge count and degenerate faces */
static long g_dupFaces;
static long euler(const GmMesh* m, const U32* ids, long* boundary, long* degenerate) {
    U32 t, ne = 0, i; long V = 0, E = 0, B = 0, D = 0, F = 0;
    static U8* seen; if (!seen) seen = calloc(1u << 21, 1);
    memset(seen, 0, 1u << 21);
    for (t = 0; t < m->nt; t++) {
        U32 v[3]; int k;
        for (k = 0; k < 3; k++) v[k] = ids ? ids[m->tri[t * 3 + k]] : m->tri[t * 3 + k];
        if (v[0] == v[1] || v[1] == v[2] || v[2] == v[0]) { D++; continue; }
        { U32 x = v[0], y = v[1], z = v[2], s2;
          if (x > y) { s2 = x; x = y; y = s2; } if (y > z) { s2 = y; y = z; z = s2; } if (x > y) { s2 = x; x = y; y = s2; }
          fbuf[F].a = x; fbuf[F].b = y; fbuf[F].c = z; F++; }
        for (k = 0; k < 3; k++) {
            U32 a = v[k], b = v[(k + 1) % 3]; if (a > b) { U32 s = a; a = b; b = s; }
            ebuf[ne++] = ((unsigned long long)a << 32) | b;
            if (!seen[a]) { seen[a] = 1; V++; } if (!seen[b]) { seen[b] = 1; V++; }
        }
    }
    qsort(ebuf, ne, sizeof(*ebuf), cmp_u64);
    for (i = 0; i < ne;) { U32 j = i; while (j < ne && ebuf[j] == ebuf[i]) j++; E++; if (j - i == 1) B++; i = j; }
    /* double-sided faces (same three vertices twice) count once */
    qsort(fbuf, (size_t)F, sizeof(Face), cmp_face);
    { long u = 0, k; for (k = 0; k < F; k++) if (!k || memcmp(&fbuf[k], &fbuf[k - 1], sizeof(Face))) u++; g_dupFaces = F - u; F = u; }
    if (boundary) *boundary = B; if (degenerate) *degenerate = D;
    return V - E + F;
}
static Vec3 tri_normal(const GmMesh* m, U32 t) {
    Vec3 a = m->pos[m->tri[t * 3]], b = m->pos[m->tri[t * 3 + 1]], c = m->pos[m->tri[t * 3 + 2]];
    return gm_cross(gm_sub(b, a), gm_sub(c, a));
}
static double total_area(const GmMesh* m) { double s = 0; U32 t; for (t = 0; t < m->nt; t++) { Vec3 n = tri_normal(m, t); s += 0.5 * sqrt((double)gm_dot(n, n)); } return s; }

typedef struct {
    GmMesh m; Trace tr;
    Vec3* pos0; Vec3* onorm; U32 nv0, nt0; U8* attr0[GM_MAX_BATTR];
    long chiI, chiC, degC, dupI; int isCol;
} TestMesh;

static void init_classes(TestMesh* tm) {
    /* weld by exact position */
    U32 i, n = tm->m.nv; U32* order = malloc(n * 4);
    for (i = 0; i < n; i++) order[i] = i;
    /* simple hash weld */
    { U32 size = 1u << 18; int* tab = malloc(size * sizeof(int)); memset(tab, 0xFF, size * sizeof(int));
      tm->tr.ncls = 0;
      for (i = 0; i < n; i++) {
          U32 b[3]; memcpy(b, &tm->m.pos[i], 12);
          U32 h = (b[0] * 73856093u ^ b[1] * 19349663u ^ b[2] * 83492791u) & (size - 1);
          for (;;) {
              if (tab[h] < 0) { tab[h] = (int)i; tm->tr.cls[i] = tm->tr.ncls++; break; }
              if (!memcmp(&tm->m.pos[tab[h]], &tm->m.pos[i], 12)) { tm->tr.cls[i] = tm->tr.cls[tab[h]]; break; }
              h = (h + 1) & (size - 1);
          }
      }
      free(tab); }
    free(order);
    memset(tm->tr.keys, 0, KEYS * sizeof(Key));
}
static void begin(TestMesh* tm) {
    U32 t; long b;
    T = &tm->tr;
    tm->nv0 = tm->m.nv; tm->nt0 = tm->m.nt;
    memcpy(tm->pos0, tm->m.pos, tm->m.nv * 12);
    for (t = 0; t < tm->m.nt; t++) { tm->tr.orig[t] = t; tm->onorm[t] = tri_normal(&tm->m, t); }
    tm->chiI = euler(&tm->m, NULL, &b, NULL); tm->dupI = g_dupFaces;
    tm->chiC = euler(&tm->m, tm->tr.cls, &b, &tm->degC);
}
static void check_topology(TestMesh* tm, const char* stage) {
    long b, d, chi; U32 i;
    chi = euler(&tm->m, NULL, &b, &d);
    if (tm->dupI == 0 && strcmp(stage, "bowl material")) CHECK(chi == tm->chiI, "%s: index-complex Euler %ld -> %ld (crack/T-junction)", stage, tm->chiI, chi);
    if (tm->degC == 0) {
        chi = euler(&tm->m, tm->tr.cls, &b, &d);
        CHECK(chi == tm->chiC && d == 0, "%s: welded Euler %ld -> %ld deg=%ld (seam mismatch)", stage, tm->chiC, chi, d);
    }
    /* seam duplicates bit-identical: all vertices of a class share one position */
    { static Vec3* cp; static U8* have; U32 bad = 0;
      if (!cp) { cp = malloc(sizeof(Vec3) * (1u << 21)); have = malloc(1u << 21); }
      memset(have, 0, tm->tr.ncls < (1u << 21) ? tm->tr.ncls : (1u << 21));
      for (i = 0; i < tm->m.nv; i++) { U32 c = tm->tr.cls[i];
          if (!have[c]) { have[c] = 1; cp[c] = tm->m.pos[i]; } else if (memcmp(&cp[c], &tm->m.pos[i], 12)) bad++; }
      CHECK(bad == 0, "%s: %u seam vertices differ from their duplicates", stage, bad); }
    for (i = 0; i < tm->m.nt * 3; i++) if (tm->m.tri[i] >= tm->m.nv) { CHECK(0, "%s: index out of range", stage); break; }
}
static int key_exists(U32 a, U32 b, U32 c, int kind) {
    U32 h = (a * 73856093u ^ b * 19349663u ^ c * 83492791u ^ (U32)kind * 2654435761u) & (KEYS - 1);
    for (;;) { Key* k = &T->keys[h]; if (!k->used) return 0; if (k->a == a && k->b == b && k->c == c && k->kind == kind) return 1; h = (h + 1) & (KEYS - 1); }
}
static void debug_tjunction(TestMesh* tm, Vec3 C, float R, float h) {
    U32 t; int shown = 0;
    for (t = 0; t < tm->m.nt && shown < 4; t++) { int k; for (k = 0; k < 3; k++) {
        U32 ia = tm->m.tri[t * 3 + k], ib = tm->m.tri[t * 3 + (k + 1) % 3]; U32 ca = tm->tr.cls[ia], cb = tm->tr.cls[ib];
        if (ca > cb) { U32 x = ca; ca = cb; cb = x; }
        if (ca != cb && (key_exists(ca, cb, 0, 0) || key_exists(ca, cb, 0, 3))) {
            Vec3 a = tm->m.pos[ia], b = tm->m.pos[ib];
            printf("    T-junction: tri %u (orig %u) edge idx %u-%u cls %u-%u len %.4f segdist %.4f R %.4f h %.3f far=%d idx=(%u,%u,%u)\n", t, tm->tr.orig[t], ia, ib, ca, cb,
                   sqrt(gm_dist2(a, b)), sqrt(gm_seg_dist2(C, a, b)), R, h, gm_far(&tm->m, t, C, R), tm->m.tri[t*3], tm->m.tri[t*3+1], tm->m.tri[t*3+2]);
            shown++;
        } } }
}
static void check_surface_same(TestMesh* tm, double area0, const char* stage) {
    double a = total_area(&tm->m); U32 t, flipped = 0;
    CHECK(fabs(a - area0) <= 1e-4 * area0 + 1e-3, "%s: area %f -> %f", stage, area0, a);
    for (t = 0; t < tm->m.nt; t++) {
        Vec3 n = tri_normal(&tm->m, t), o = tm->onorm[tm->tr.orig[t]];
        double lo = sqrt((double)gm_dot(o, o)), ln = sqrt((double)gm_dot(n, n));
        if (lo < 1e-6 || ln < 2e-5) continue;      /* slivers below 0.1 cm2 have no reliable normal */
        if (gm_dot(n, o) < -1e-3 * lo * ln) { flipped++; if (verbose) { U32 x=tm->m.tri[t*3],y=tm->m.tri[t*3+1],z=tm->m.tri[t*3+2]; printf("    flipped piece %u: area %.3g parent area %.3g cos %.3f  (%.5f %.5f %.5f)(%.5f %.5f %.5f)(%.5f %.5f %.5f)\n", t, 0.5*ln, 0.5*lo, gm_dot(n,o)/(lo*ln), tm->m.pos[x].x,tm->m.pos[x].y,tm->m.pos[x].z,tm->m.pos[y].x,tm->m.pos[y].y,tm->m.pos[y].z,tm->m.pos[z].x,tm->m.pos[z].y,tm->m.pos[z].z); } }
    }
    CHECK(flipped == 0, "%s: %u pieces flipped against their parent triangle", stage, flipped);
    CHECK(!memcmp(tm->pos0, tm->m.pos, tm->nv0 * 12), "%s: original vertices moved", stage);
}

static double g_maxPen, g_bowlArea, g_flipArea; static long g_bowl, g_bowlFlipped, g_moved, g_tris, g_dropped;
static int crater_checked(TestMesh* tm, Vec3 C, float R, Vec3 Q, float h, float depth) {
    GmStats st; double area0; U32 i, t; int r;
    memset(&st, 0, sizeof(st));
    begin(tm); area0 = total_area(&tm->m);
    r = gm_refine(&tm->m, C, R, h, &st);
    if (r < 0) return -1;
    { int f0 = fails; check_topology(tm, "refine"); if (fails != f0 && verbose) debug_tjunction(tm, C, R, h); } check_surface_same(tm, area0, "refine");
    r = gm_cut(&tm->m, C, R, &st);
    if (r < 0) return -1;
    check_topology(tm, "cut"); check_surface_same(tm, area0, "cut");
    /* no edge inside the sphere longer than h may remain */
    for (t = 0; t < tm->m.nt; t++) { int k; for (k = 0; k < 3; k++) {
        Vec3 a = tm->m.pos[tm->m.tri[t * 3 + k]], b = tm->m.pos[tm->m.tri[t * 3 + (k + 1) % 3]];
        float lo = R - GM_EPS;
        if (gm_dist2(a, C) < lo * lo && gm_dist2(b, C) < lo * lo) CHECK(gm_dist2(a, b) <= h * h * 1.0001f, "edge inside sphere longer than h: %f", sqrt(gm_dist2(a, b)));
    } }
    { Vec3* before = malloc(tm->m.nv * 12); Vec3 nq = gm_sub(Q, C); memcpy(before, tm->m.pos, tm->m.nv * 12);
      static U8 movedFlags[GM_MAXV]; memset(movedFlags, 0, sizeof(movedFlags)); memset(gm_orphan, 0, sizeof(gm_orphan));
      { U32 q; for (q = 0; q < tm->m.nv; q++) gm_clamp[q] = -1.f; }      /* everything solid: plain bowl */
      gm_project(&tm->m, C, R, Q, 1.0f, movedFlags, &st);
      for (i = 0; i < tm->m.nv; i++) {
          double d = sqrt((double)gm_dist2(tm->m.pos[i], C));
          int was = gm_inside(before[i], C, R, Q);
          int upper = gm_dot(gm_sub(tm->m.pos[i], C), nq) >= 0.f;
          if (!upper) CHECK(d >= R - GM_EPS - 2e-3, "vertex %u left inside ground-side half: %f < %f", i, d, R);
          if (!was) CHECK(!memcmp(&before[i], &tm->m.pos[i], 12), "unaffected vertex %u moved", i);
          else { CHECK(fabs(d - R) < 2e-3 * R + 1e-3, "projected vertex %u not on sphere: %f vs %f", i, d, R);
                 CHECK(gm_dot(gm_sub(tm->m.pos[i], C), nq) < 1e-3f * R * R, "projected vertex %u landed on the air side", i); }
      }
      /* per triangle: penetration of the interior into the sphere, orientation as seen from Q */
      for (t = 0; t < tm->m.nt; t++) {
          U32 id[3] = { tm->m.tri[t * 3], tm->m.tri[t * 3 + 1], tm->m.tri[t * 3 + 2] }; int k, moved = 0, air = 0;
          Vec3 a = tm->m.pos[id[0]], b = tm->m.pos[id[1]], c = tm->m.pos[id[2]];
          static const float W[4][3] = { {1/3.f,1/3.f,1/3.f}, {.5f,.5f,0}, {0,.5f,.5f}, {.5f,0,.5f} };
          if (gm_far(&tm->m, t, C, R)) continue;
          for (k = 0; k < 3; k++) {
              float lo = R - GM_EPS; Vec3 p = tm->m.pos[id[k]];
              moved += gm_inside(before[id[k]], C, R, Q);
              if (gm_dist2(p, C) < lo * lo) air++;          /* still inside: air-side geometry, left alone on purpose */
          }
          if (!air) for (k = 0; k < 4; k++) {
              Vec3 p = { a.x*W[k][0]+b.x*W[k][1]+c.x*W[k][2], a.y*W[k][0]+b.y*W[k][1]+c.y*W[k][2], a.z*W[k][0]+b.z*W[k][1]+c.z*W[k][2] };
              double pen = R - sqrt((double)gm_dist2(p, C));
              if (pen > g_maxPen) g_maxPen = pen;
              CHECK(pen < 0.5 * depth, "surface left inside sphere: %.3f m deep (crater depth %.2f)", pen, depth);
          }
          if (moved == 3) {
              /* the projection keeps the orientation as seen from Q */
              Vec3 a0 = before[id[0]], b0 = before[id[1]], c0 = before[id[2]];
              Vec3 n0 = gm_cross(gm_sub(b0, a0), gm_sub(c0, a0)), n1 = tri_normal(&tm->m, t);
              Vec3 cen0 = { (a0.x+b0.x+c0.x)/3, (a0.y+b0.y+c0.y)/3, (a0.z+b0.z+c0.z)/3 }, cen1 = { (a.x+b.x+c.x)/3, (a.y+b.y+c.y)/3, (a.z+b.z+c.z)/3 };
              Vec3 v0 = gm_sub(Q, cen0), v1 = gm_sub(Q, cen1);
              double s0 = gm_dot(n0, v0) / (sqrt((double)gm_dot(n0, n0) * gm_dot(v0, v0)) + 1e-30);
              double s1 = gm_dot(n1, v1) / (sqrt((double)gm_dot(n1, n1) * gm_dot(v1, v1)) + 1e-30);
              double ar = 0.5 * sqrt((double)gm_dot(n1, n1));
              g_bowl++; g_bowlArea += ar;
              if (fabs(s0) > 0.05 && fabs(s1) > 0.02 && (s0 > 0) != (s1 > 0)) { g_bowlFlipped++; g_flipArea += ar; }
          }
      }
      check_topology(tm, "project");
      if (!tm->isCol) {
          /* bowl material: rock index 77, planar mapping */
          static const float M[8] = { 0.33f, 0, 0, 1.5f, 0, 0.33f, 0, -2.5f };
          U32 nvB = tm->m.nv; float* uv0 = malloc(nvB * 8); U16* aux0 = malloc(tm->m.nt * 2); U32 bowl = 0;
          memcpy(uv0, tm->m.fa[0], nvB * 8); memcpy(aux0, tm->m.aux, tm->m.nt * 2);
          CHECK(gm_bowl_material(&tm->m, movedFlags, 77, 0, M, &st) == 0, "bowl material: out of capacity");
          for (t = 0; t < tm->m.nt; t++) {
              int k, mv = 0, isBowl = tm->m.aux[t] == 77 && aux0[t] != 77 ? 1 : (tm->m.aux[t] == 77);
              for (k = 0; k < 3; k++) { U32 x = tm->m.tri[t * 3 + k]; if (x < nvB && movedFlags[x]) mv = 1; }
              if (mv) CHECK(tm->m.aux[t] == 77, "triangle with a moved vertex is not bowl material");
              if (tm->m.aux[t] != 77) {
                  CHECK(tm->m.aux[t] == aux0[t], "material of an untouched triangle changed");
                  for (k = 0; k < 3; k++) { U32 x = tm->m.tri[t * 3 + k]; CHECK(x < nvB && !memcmp(uv0 + x * 2, tm->m.fa[0] + x * 2, 8), "UV of a vertex outside the bowl changed"); }
              } else if (isBowl && (mv || aux0[t] != 77)) {
                  bowl++;
                  for (k = 0; k < 3; k++) { U32 x = tm->m.tri[t * 3 + k]; Vec3 pp = tm->m.pos[x]; float* uv = tm->m.fa[0] + x * 2;
                      CHECK(fabsf(uv[0] - (0.33f * pp.x + 1.5f)) < 1e-4f && fabsf(uv[1] - (0.33f * pp.y - 2.5f)) < 1e-4f, "bowl vertex has no rock mapping"); }
              }
          }
          CHECK(bowl == st.bowlTris, "bowl triangle count %u vs %u", bowl, st.bowlTris);
          for (i = nvB; i < tm->m.nv; i++) CHECK(!memcmp(&tm->m.pos[i], &tm->m.pos[0], 0) , "x");
          check_topology(tm, "bowl material");
          free(uv0); free(aux0);
      }
      free(before); }
    g_moved += st.moved; g_tris += (long)(tm->m.nt - tm->nt0);
    return st.moved ? 1 : 0;
}

/* collision vertices live as int16/128 in the game: quantise like the runtime will */
static void quantise(GmMesh* m) {
    U32 i; for (i = 0; i < m->nv; i++) {
        m->pos[i].x = (float)lroundf(m->pos[i].x * 128.f) / 128.f;
        m->pos[i].y = (float)lroundf(m->pos[i].y * 128.f) / 128.f;
        m->pos[i].z = (float)lroundf(m->pos[i].z * 128.f) / 128.f;
    }
}
static void alloc_mesh(TestMesh* tm, int isCol) {
    memset(tm, 0, sizeof(*tm)); tm->isCol = isCol;
    tm->m.pos = malloc(GM_MAXV * 12); tm->m.tri = malloc(GM_MAXT * 6); tm->m.aux = malloc(GM_MAXT * 2);
    tm->pos0 = malloc(GM_MAXV * 12); tm->onorm = malloc(GM_MAXT * 12); tm->tr.keys = malloc(KEYS * sizeof(Key));
    tm->m.normalAttr = -1;
}

static void dump_obj(const GmMesh* m, const char* path) {
    FILE* f = fopen(path, "w"); U32 i; if (!f) return;
    for (i = 0; i < m->nv; i++) fprintf(f, "v %f %f %f\n", m->pos[i].x, m->pos[i].y, m->pos[i].z);
    for (i = 0; i < m->nt; i++) fprintf(f, "f %u %u %u\n", m->tri[i * 3] + 1, m->tri[i * 3 + 1] + 1, m->tri[i * 3 + 2] + 1);
    fclose(f);
}
static TestMesh RM, CM;
int main(int argc, char** argv) {
    FILE* f; U32 hdr[6], i, npts, hasNight; float bb[10]; float* pts; char magic[4];
    S16* cv; U8* ct; int applied = 0, none = 0, overflow = 0;
    static const float RR[3] = { 2.2f, 2.8f, 4.0f }, DD[3] = { 0.9f, 1.2f, 1.5f };
    if (argc < 2) return 2;
    name = argv[1]; verbose = argc > 2;
    ebuf = malloc(sizeof(*ebuf) * GM_MAXT * 3); fbuf = malloc(sizeof(Face) * GM_MAXT);
    f = fopen(argv[1], "rb"); if (!f) { perror(argv[1]); return 2; }
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "GM02", 4) || fread(hdr, 4, 6, f) != 6 || fread(bb, 4, 10, f) != 10) return 2;
    alloc_mesh(&RM, 0); alloc_mesh(&CM, 1);
    RM.m.nv = hdr[0]; RM.m.nt = hdr[1]; CM.m.nv = hdr[2]; CM.m.nt = hdr[3]; hasNight = hdr[4]; npts = hdr[5];
    RM.m.nf = 1; RM.m.fdim[0] = 2; RM.m.fa[0] = malloc(GM_MAXV * 8);
    RM.m.nb = hasNight ? 2 : 1; RM.m.ba[0] = malloc(GM_MAXV * 4); RM.m.ba[1] = malloc(GM_MAXV * 4);
    if (fread(RM.m.pos, 12, RM.m.nv, f) != RM.m.nv || fread(RM.m.fa[0], 8, RM.m.nv, f) != RM.m.nv || fread(RM.m.ba[0], 4, RM.m.nv, f) != RM.m.nv) return 2;
    if (hasNight && fread(RM.m.ba[1], 4, RM.m.nv, f) != RM.m.nv) return 2;
    if (fread(RM.m.tri, 6, RM.m.nt, f) != RM.m.nt || fread(RM.m.aux, 2, RM.m.nt, f) != RM.m.nt) return 2;
    cv = malloc(CM.m.nv * 6); ct = malloc(CM.m.nt * 8); pts = malloc(npts * 12);
    if (fread(cv, 6, CM.m.nv, f) != CM.m.nv || fread(ct, 8, CM.m.nt, f) != CM.m.nt || fread(pts, 12, npts, f) != npts) return 2;
    fclose(f);
    for (i = 0; i < CM.m.nv; i++) { CM.m.pos[i].x = cv[i * 3] / 128.f; CM.m.pos[i].y = cv[i * 3 + 1] / 128.f; CM.m.pos[i].z = cv[i * 3 + 2] / 128.f; }
    for (i = 0; i < CM.m.nt; i++) { memcpy(&CM.m.tri[i * 3], ct + i * 8, 6); CM.m.aux[i] = (U16)(ct[i * 8 + 6] | (ct[i * 8 + 7] << 8)); }
    init_classes(&RM); init_classes(&CM);
    { const char* nm = "sqrt"; const char* keep = name; float a; name = nm;
      for (a = 1e-6f; a < 1e7f; a *= 1.37f) CHECK(fabsf(gm_sqrt(a) - sqrtf(a)) <= 2e-6f * sqrtf(a), "gm_sqrt(%g)=%g", a, gm_sqrt(a));
      name = keep; }
    /* each exported point, plus a cluster of overlapping craters around the first one */
    for (i = 0; i < npts + 4; i++) {
        static Vec3 lastS; int sameSpot = argc > 2 && !strcmp(argv[2], "-same");
        Vec3 E, S = { 0, 0, 0 }, n = { 0, 0, 0 }, C, Q; float best = 1e30f, len, r, d, R, h; int k = (int)(i % 3), rc, rr;
        if (sameSpot && i > 0) { E = lastS; E.z += 0.1f; }
        else if (i < npts) { E.x = pts[i * 3]; E.y = pts[i * 3 + 1]; E.z = pts[i * 3 + 2]; }
        else { float ang = (float)(i - npts) * 1.7f; E.x = pts[0] + 1.4f * cosf(ang); E.y = pts[1] + 1.4f * sinf(ang); E.z = pts[2] + 0.3f; }
        r = RR[k]; d = DD[k]; R = (r * r + d * d) / (2 * d); h = 0.45f * r; if (h < 0.6f) h = 0.6f;
        gm_probe(&CM.m, E, 2.5f, 1, 0, &best, &S, &n);
        len = sqrtf(gm_dot(n, n));
        if (best > 1e20f || len < 1e-6f) { none++; continue; }
        n.x /= len; n.y /= len; n.z /= len; lastS = S;
        if (sameSpot) k = 0, r = RR[0], d = DD[0], R = (r * r + d * d) / (2 * d), h = 0.6f > 0.45f * r ? 0.6f : 0.45f * r;
        C.x = S.x + n.x * (R - d); C.y = S.y + n.y * (R - d); C.z = S.z + n.z * (R - d);
        Q.x = C.x + n.x * R; Q.y = C.y + n.y * R; Q.z = C.z + n.z * R;
        rc = crater_checked(&CM, C, R, Q, h, d);
        rr = crater_checked(&RM, C, R, Q, h, d);
        if (rc < 0 || rr < 0) { overflow++; break; }
        quantise(&CM.m);
        for (U32 v = 0; v < CM.m.nv; v++) if (gm_dot(gm_sub(CM.m.pos[v], C), n) < -0.02f) CHECK(sqrt((double)gm_dist2(CM.m.pos[v], C)) >= R - GM_EPS - 0.016, "quantised col vertex inside ground-side half");
        /* what the runtime does before writing: drop zero-area triangles. Must not open the surface:
         * a dropped triangle has zero area, so only the bookkeeping changes. */
        g_dropped += (long)gm_drop_degenerate(&CM.m) + (long)gm_drop_degenerate(&RM.m);
        init_classes(&CM); init_classes(&RM);
        applied += (rc > 0) + (rr > 0);
        if (verbose || sameSpot) printf("  crater %u r=%.1f d=%.1f R=%.2f: col %u v %u t, render %u v %u t\n", i, r, d, R, CM.m.nv, CM.m.nt, RM.m.nv, RM.m.nt);
    }
    if (argc > 3) { char pth[512]; snprintf(pth, 512, "%s_render.obj", argv[3]); dump_obj(&RM.m, pth); snprintf(pth, 512, "%s_col.obj", argv[3]); dump_obj(&CM.m, pth); }
    printf("%s applied=%d nosurface=%d overflow=%d rv=%u->%u rt=%u->%u cv=%u->%u ct=%u->%u bowl=%ld flipped=%ld fliparea=%.4f%% dropped=%ld maxpen=%.3f checks=%d fails=%d\n",
           argv[1], applied, none, overflow, hdr[0], RM.m.nv, hdr[1], RM.m.nt, hdr[2], CM.m.nv, hdr[3], CM.m.nt, g_bowl, g_bowlFlipped, g_bowlArea > 0 ? 100.0 * g_flipArea / g_bowlArea : 0.0, g_dropped, g_maxPen, checks, fails);
    return fails ? 1 : 0;
}
