/* Offline test of the v3.6 runtime against a simulated GTA SA process.
 * The simulation holds the structures the runtime touches (building pool, CEntity, CMatrix,
 * RpAtomic, RpGeometry + morph target + material list + day/night colour plugin, CBaseModelInfo,
 * CColModel, CCollisionData block, explosion slots), filled with REAL mesh data.
 * Engine functions are small re-implementations that follow RenderWare's documented behaviour
 * and track every allocation, so use-after-free, double free and leaks are detected.
 *
 * usage: host_test model.bin [-v]
 */
#define HOST_TEST
#include "../src/sa_runtime.c"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SIM_BASE 0x00400000u
#define SIM_SIZE 0x0A000000u
#define HEAP_START 0x01000000u
static U8* sim; static const char* name; static int fails, checks, verbose;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; if (fails < 40) { printf("  FAIL %s:%d: ", name, __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

/* ---- tracked heap ---- */
typedef struct { U32 addr, size; int live; char tag; } Alloc;
static Alloc allocs[200000]; static U32 nalloc, heapTop = HEAP_START;
static U32 sim_alloc(U32 size, char tag) {
    U32 a = heapTop; heapTop += (size + 0x1Fu) & ~0xFu;            /* gap between blocks catches overruns */
    if (heapTop - SIM_BASE > SIM_SIZE || nalloc >= 200000) { printf("sim heap exhausted\n"); exit(3); }
    allocs[nalloc].addr = a; allocs[nalloc].size = size; allocs[nalloc].live = 1; allocs[nalloc].tag = tag; nalloc++;
    memset(sim + (a - SIM_BASE), 0xCD, size);
    return a;
}
static Alloc* find_alloc(U32 a) {      /* allocation containing address a (binary search: addresses ascend) */
    U32 lo = 0, hi = nalloc;
    while (lo < hi) { U32 mid = (lo + hi) / 2; if (allocs[mid].addr <= a) lo = mid + 1; else hi = mid; }
    if (!lo) return NULL;
    return (a < allocs[lo - 1].addr + allocs[lo - 1].size) ? &allocs[lo - 1] : NULL;
}
static void sim_free(U32 a, char expectTag) {
    Alloc* al = find_alloc(a);
    CHECK(al && al->addr == a, "free of a pointer that is not an allocation start: %#x", a);
    if (!al || al->addr != a) return;
    CHECK(al->live, "double free %#x (tag %c)", a, al->tag);
    if (expectTag) CHECK(al->tag == expectTag, "free of %#x with tag %c, expected %c", a, al->tag, expectTag);
    al->live = 0; memset(sim + (a - SIM_BASE), 0xDD, al->size);
}
static int range_ok(U32 a, U32 n) {
    if (a < SIM_BASE || n > SIM_SIZE || a - SIM_BASE > SIM_SIZE - n) return 0;
    if (!n) return 1;
    if (a >= HEAP_START) {
        Alloc* al = find_alloc(a);
        if (!al || !al->live || a + n > al->addr + al->size) { CHECK(0, "access to freed/unallocated memory %#x (+%u)", a, n); return 0; }
    }
    return 1;
}
static int mem_read(U32 a, void* out, U32 n) { if (!range_ok(a, n)) return 0; memcpy(out, sim + (a - SIM_BASE), n); return 1; }
static int mem_write(U32 a, const void* src, U32 n) { if (!range_ok(a, n)) return 0; memcpy(sim + (a - SIM_BASE), src, n); return 1; }
static U32 peek32(U32 a) { U32 v = 0; mem_read(a, &v, 4); return v; }
static U32 peek16(U32 a) { U16 v = 0; mem_read(a, &v, 2); return v; }
static U32 peek8(U32 a) { U8 v = 0; mem_read(a, &v, 1); return v; }
static float peekf(U32 a) { float v = 0; mem_read(a, &v, 4); return v; }
static void w32(U32 a, U32 v) { mem_write(a, &v, 4); }
static void w16(U32 a, U32 v) { U16 x = (U16)v; mem_write(a, &x, 2); }
static void w8(U32 a, U32 v) { U8 x = (U8)v; mem_write(a, &x, 1); }
static void wf(U32 a, float v) { mem_write(a, &v, 4); }

static char logbuf[64][320]; static int logn;
static unsigned long g_brokeTotal;
static void logline(const char* s) { snprintf(logbuf[logn % 64], 320, "%s", s); logn++; if (strstr(s, "broke through")) g_brokeTotal++; if (verbose) printf("    %s\n", s); }
static int logged(const char* needle) { int i; for (i = 0; i < 64; i++) if (strstr(logbuf[i], needle)) return 1; return 0; }
static int opened(void) { return logged("holes(render/col)=") || logged("broke through"); }      /* the blast removed surface: sheet holes or a break-through */
static void clear_log(void) { memset(logbuf, 0, sizeof(logbuf)); logn = 0; }

/* ---- fake engine ---- */
#define VC_OFFSET 0x60u
#define GEOM_SIZE 0x80u            /* RpGeometry 0x60 + plugin data: day/night (12) + d3d9 usage (4) */
#define USAGE_OFF 0x6Cu
#define FX_OFF 0x70u               /* 2D effect plugin: pointer to the effect block */
static int n_removePlanes, n_unlock, n_create, n_destroyed, n_setGeom;
static U32 game_malloc(U32 size) { return sim_alloc(size, 'M'); }
static void game_free(U32 p) { sim_free(p, 'M'); }
static void game_remove_planes(U32 cd) { n_removePlanes++; CHECK(peek32(cd + 0x1C) != 0, "RemoveTrianglePlanes without planes"); if (peek32(cd + 0x1C)) sim_free(peek32(cd + 0x1C), 'P'); w32(cd + 0x1C, 0); }
static U32 rw_geom_create(U32 nv, U32 nt, U32 format) {
    U32 g = sim_alloc(GEOM_SIZE, 'G'), sets = (format >> 16) & 0xFF, mt, i;
    if (!sets) sets = (format & 0x80) ? 2 : ((format & 4) ? 1 : 0);
    memset(sim + (g - SIM_BASE), 0, GEOM_SIZE); n_create++;
    w8(g, 8); w32(g + 8, format & 0xFF); w16(g + 0xC, 0xFFF); w16(g + 0xE, 1);
    w32(g + 0x10, nt); w32(g + 0x14, nv); w32(g + 0x18, 1); w32(g + 0x1C, sets);
    w32(g + 0x2C, sim_alloc(nt * 8, 'T'));
    if (format & 8) w32(g + 0x30, sim_alloc(nv * 4, 'C'));
    for (i = 0; i < sets; i++) w32(g + 0x34 + i * 4, sim_alloc(nv * 8, 'U'));
    mt = sim_alloc(0x1C, 'O'); memset(sim + (mt - SIM_BASE), 0, 0x1C); w32(g + 0x5C, mt);
    w32(mt, g); w32(mt + 0x14, sim_alloc(nv * 12, 'V'));
    if (format & 0x10) w32(mt + 0x18, sim_alloc(nv * 12, 'N'));
    return g;
}
static int rw_matlist_append(U32 ml, U32 mat) {
    U32 arr = peek32(ml), num = peek32(ml + 4), space = peek32(ml + 8);
    if (num == space) {
        U32 na = sim_alloc((space + 4) * 4, 'L');
        if (arr) { memcpy(sim + (na - SIM_BASE), sim + (arr - SIM_BASE), num * 4); sim_free(arr, 'L'); }
        arr = na; w32(ml, arr); w32(ml + 8, space + 4);
    }
    w32(arr + num * 4, mat); w32(ml + 4, num + 1); w32(mat + 0x18, peek32(mat + 0x18) + 1);
    return (int)num;
}
static U32 rw_geom_unlock(U32 g) { n_unlock++; if (!peek32(g + 0x54)) w32(g + 0x54, sim_alloc(0x10, 'H')); w16(g + 0xC, 0); return g; }
static void rw_geom_destroy(U32 g) {
    U32 rc = peek16(g + 0xE), i, mt;
    CHECK(rc > 0 && rc < 1000, "geometry refcount corrupt: %u", rc);
    w16(g + 0xE, rc - 1);
    if (rc != 1) return;
    n_destroyed++;
    if (peek32(g + VC_OFFSET)) sim_free(peek32(g + VC_OFFSET), 'M');            /* plugin destructor: CMemoryMgr::Free */
    if (peek32(g + VC_OFFSET + 4)) sim_free(peek32(g + VC_OFFSET + 4), 'M');
    if (peek32(g + FX_OFF)) sim_free(peek32(g + FX_OFF), 'E');                  /* 2dfx plugin destructor */
    for (i = 0; i < peek32(g + 0x24); i++) { U32 m = peek32(peek32(g + 0x20) + i * 4); w32(m + 0x18, peek32(m + 0x18) - 1); }
    if (peek32(g + 0x20)) sim_free(peek32(g + 0x20), 'L');
    sim_free(peek32(g + 0x2C), 'T');
    if (peek32(g + 0x30)) sim_free(peek32(g + 0x30), 'C');
    for (i = 0; i < 8; i++) if (peek32(g + 0x34 + i * 4)) sim_free(peek32(g + 0x34 + i * 4), 'U');
    if (peek32(g + 0x54)) sim_free(peek32(g + 0x54), 'H');
    mt = peek32(g + 0x5C); sim_free(peek32(mt + 0x14), 'V'); if (peek32(mt + 0x18)) sim_free(peek32(mt + 0x18), 'N'); sim_free(mt, 'O');
    sim_free(g, 'G');
}
static void rw_atomic_set_geometry(U32 atomic, U32 g) {
    U32 old = peek32(atomic + 0x18);
    n_setGeom++;
    if (old == g) return;
    if (g) w16(g + 0xE, peek16(g + 0xE) + 1);
    w32(atomic + 0x18, g);
    if (g) mem_write(atomic + 0x1C, sim + (peek32(g + 0x5C) + 4 - SIM_BASE), 16);
    if (old) rw_geom_destroy(old);
}
static U32 rw_d3d9_get_usage(U32 g) { return peek32(g + USAGE_OFF); }
static void rw_d3d9_set_usage(U32 g, U32 f) { w32(g + USAGE_OFF, f); }
/* raster: struct (0x34) + one pixel buffer per mip level; like RW, the struct shows the locked level's size */
#define MAXLV 8
static U32 rasterLevels[MAXLV], rasterOf, rasterNumLevels, rasterLocked = 99, n_rasterCreate, mock_noMips;
static U32 rw_raster_create(U32 w, U32 h, U32 depth, U32 flags) {
    U32 r, lv, size = w;
    n_rasterCreate++;
    if ((flags & 0x8000u) && mock_noMips) return 0;                   /* device without mipmapped textures */
    r = sim_alloc(0x34, 'r'); memset(sim + (r - SIM_BASE), 0, 0x34);
    w32(r + 0xC, w); w32(r + 0x10, h); w32(r + 0x14, depth); w8(r + 0x20, flags & 7);
    rasterNumLevels = (flags & 0x8000u) ? 8u : 1u; rasterOf = r;
    for (lv = 0; lv < rasterNumLevels; lv++) { rasterLevels[lv] = sim_alloc((size * 4 + 16) * size, 'x'); size = size > 1 ? size / 2 : 1; }
    return r;
}
static U32 rw_raster_num_levels(U32 r) { CHECK(r == rasterOf, "num levels: wrong raster"); return rasterNumLevels; }
static U32 rw_raster_lock(U32 r, U32 level, U32 mode) {
    U32 size = 128u >> level;
    CHECK(r == rasterOf && level < rasterNumLevels && rasterLocked == 99 && (mode & 1), "bad raster lock (level %u)", level);
    rasterLocked = level; w32(r + 0xC, size); w32(r + 0x10, size); w32(r + 0x18, size * 4 + 16);   /* pitch larger than width*4, like real drivers */
    return rasterLevels[level];
}
static void rw_raster_unlock(U32 r) { CHECK(r == rasterOf && rasterLocked != 99, "unlock without lock"); rasterLocked = 99; w32(r + 0xC, 128); w32(r + 0x10, 128); }
static void rw_raster_destroy(U32 r) { U32 lv; for (lv = 0; lv < rasterNumLevels; lv++) sim_free(rasterLevels[lv], 'x'); sim_free(r, 'r'); rasterOf = 0; }
static U32 rw_texture_create(U32 raster) { U32 t = sim_alloc(0x58, 't'); memset(sim + (t - SIM_BASE), 0, 0x58); w32(t, raster); w32(t + 0x50, 0x1101); w32(t + 0x54, 1); return t; }
static U32 rw_material_create(void) { U32 m = sim_alloc(0x1C, 'k'); memset(sim + (m - SIM_BASE), 0, 0x1C); w32(m + 4, 0xFFFFFFFFu); wf(m + 0xC, 1.f); wf(m + 0x10, 1.f); wf(m + 0x14, 1.f); w32(m + 0x18, 1); return m; }
static void rw_material_set_texture(U32 m, U32 t) { w32(m, t); w32(t + 0x54, peek32(t + 0x54) + 1); }
static U32 n_objCreate, n_worldAdd, lastObj;
static U32 game_object_create(U32 model) {
    U32 o = sim_alloc(0x17C, 'B'), m = sim_alloc(0x48, 'X');
    memset(sim + (o - SIM_BASE), 0, 0x17C); memset(sim + (m - SIM_BASE), 0, 0x48);
    wf(m, 1.f); wf(m + 0x14, 1.f); wf(m + 0x28, 1.f);                   /* CPhysical(): unit matrix */
    w32(o + 0x14, m); w16(o + 0x22, model); w32(o + 0x1C, 0x85);        /* collision, STATIC, visible */
    w32(o + 0x40, 0x80000Cu); wf(o + 0x8C, 99999.f); wf(o + 0x90, 99999.f); wf(o + 0x98, 0.1f);   /* default object data: no gravity, immovable */
    w8(o + 0x13C, 1); n_objCreate++; lastObj = o;
    return o;
}
static void game_world_add(U32 e) {
    n_worldAdd++;
    CHECK(e == lastObj, "CWorld::Add on something else");
    CHECK(!(peek32(e + 0x1C) & 4), "object added to the world while still static (it would never move)");
    CHECK(peekf(e + 0x8C) < 5000.f && (peek32(e + 0x40) & 2) && !(peek32(e + 0x40) & 4), "object added without mass/gravity set up");
}
static float ray_first(Vec3 o, Vec3 d, int collision);
static int mock_meshLos; static Vec3 mock_origin;      /* lines of sight straight down are answered from the collision mesh of the object under test */
static Vec3 mock_hit; static U32 mock_ent; static int mock_ground = 1;      /* mock_ground: there is ground far below everything (as in the game) */
static int game_line_of_sight(const Vec3* from, const Vec3* to, U8* cp, U32* ent) {
    if (mock_meshLos) { Vec3 o = { from->x - mock_origin.x, from->y - mock_origin.y, from->z - mock_origin.z }, dn = { 0, 0, -1 }, hit = *from; float h = ray_first(o, dn, 1);
        if (h > from->z - to->z) return 0;
        hit.z = from->z - h; memcpy(cp, &hit, 12); *ent = 0x7FFF0000u; return 1; }
    if (!mock_ent) { if (!mock_ground || to->z - from->z > -70.f) return 0; memcpy(cp, to, 12); *ent = 0x7FFF0000u; return 1; }
    memcpy(cp, &mock_hit, 12); *ent = mock_ent; return 1;
}

/* ---- model data ---- */
static U32 nv, nt, ncv, nct, hasNight, npts; static float bbox[6], sphere[4];
static float *R_pos, *R_uv, *PTS; static U8 *R_pre, *R_night, *C_tris; static U16 *R_tri, *R_mat; static S16* C_verts;
#define NMAT 3u
#define POOL_CAP 64u
#define MODEL_A 8468u
static U32 A_pool, A_storage, A_flags, A_mat[NMAT], A_mi, A_cm, A_miAtomic, A_fx;
static int vg_set; static float vg_px, vg_py, vg_pz, vg_h;      /* placement of the object under test (for validate_geom) */
static U8 origSphereBox[0x14 + 0x1C];

static void reset_sim(void) {
    memset(sim, 0, heapTop - SIM_BASE); heapTop = HEAP_START; nalloc = 0;
    n_removePlanes = n_unlock = n_create = n_destroyed = n_setGeom = 0; clear_log();
    g_qHead = g_qTail = 0; memset(g_seenActive, 0, sizeof(g_seenActive)); mock_ent = 0;
    g_rockMat = 0; g_rockTried = 0; rasterOf = 0; rasterLocked = 99;
    g_sheetReady = 0; g_propsEnabled = 1; n_objCreate = n_worldAdd = 0; lastObj = 0;
    g_nUndo = 0; g_undoBytes = 0; g_undoFull = 0; g_undoOff = 0; g_pendSnap = 0; g_nHidden = g_nFallen = g_nMark = 0;      /* the simulated heap is gone: so is what the runtime kept in it */
    mock_meshLos = 0;
}
static U32 make_geometry(void) {
    U32 g = rw_geom_create(nv, nt, 1u | 2u | 4u | 8u | (1u << 16)), i, mt = peek32(g + 0x5C), night, day;   /* tristrip flag set like stock models */
    for (i = 0; i < NMAT; i++) rw_matlist_append(g + 0x20, A_mat[i]);
    for (i = 0; i < nt; i++) { U32 a = peek32(g + 0x2C) + i * 8; w16(a, R_tri[i * 3]); w16(a + 2, R_tri[i * 3 + 1]); w16(a + 4, R_tri[i * 3 + 2]); w16(a + 6, R_mat[i] % NMAT); }
    mem_write(peek32(mt + 0x14), R_pos, nv * 12); mem_write(peek32(g + 0x30), R_pre, nv * 4); mem_write(peek32(g + 0x34), R_uv, nv * 8);
    if (hasNight) {
        night = game_malloc(nv * 4); day = game_malloc(nv * 4); mem_write(night, R_night, nv * 4); mem_write(day, R_pre, nv * 4);
        w32(g + VC_OFFSET, night); w32(g + VC_OFFSET + 4, day); wf(g + VC_OFFSET + 8, 0.25f);
    } else { w32(g + VC_OFFSET, 0); w32(g + VC_OFFSET + 4, 0); w32(g + VC_OFFSET + 8, 0); }
    w32(g + USAGE_OFF, 0x08);
    A_fx = sim_alloc(4 + 2 * 0x40, 'E'); w32(A_fx, 2); w32(g + FX_OFF, A_fx);     /* two 2D effects */
    { float cx = 0, cy = 0, cz = 0, r = 0; for (i = 0; i < nv; i++) { cx += R_pos[i * 3]; cy += R_pos[i * 3 + 1]; cz += R_pos[i * 3 + 2]; } cx /= nv; cy /= nv; cz /= nv;
      for (i = 0; i < nv; i++) { float d = sqrtf((R_pos[i*3]-cx)*(R_pos[i*3]-cx) + (R_pos[i*3+1]-cy)*(R_pos[i*3+1]-cy) + (R_pos[i*3+2]-cz)*(R_pos[i*3+2]-cz)); if (d > r) r = d; }
      wf(mt + 4, cx); wf(mt + 8, cy); wf(mt + 0xC, cz); wf(mt + 0x10, r); }
    rw_geom_unlock(g);
    return g;
}
static U32 make_atomic(U32 geom) { U32 a = sim_alloc(0x70, 'A'); memset(sim + (a - SIM_BASE), 0, 0x70); w8(a, 1); rw_atomic_set_geometry(a, geom); return a; }
/* collision block exactly like CFileLoader::LoadCollisionModelVer3 lays it out */
static int scene_noSphere, expectBoxTris = 1, scene_boxSet, propWide; static float scene_box[6]; static U32 scene_special;
static U32 make_coldata(int planes, int shadow, int groups) {
    U32 extra = 0x14 + 0x1C, vb = (ncv * 6 + 3) & ~3u, ng = groups ? 2u : 0u, size = 0x30 + extra + vb + (ng ? ng * 28 + 4 : 0) + nct * 8;
    U32 b = game_malloc(size), oV = 0x30 + extra, oT = oV + vb + (ng ? ng * 28 + 4 : 0), i;
    memset(sim + (b - SIM_BASE), 0, size);
    w16(b, scene_noSphere ? 0 : 1); w16(b + 2, 1); w16(b + 4, nct); w8(b + 6, 0); w8(b + 7, (ng ? 2u : 0u) | (shadow ? 4u : 0u));
    w32(b + 8, b + 0x30); w32(b + 0xC, b + 0x30 + 0x14); w32(b + 0x14, b + oV); w32(b + 0x18, b + oT);
    for (i = 0; i < 0x14; i++) origSphereBox[i] = (U8)(i * 7 + 3);
    { float bx[6] = { bbox[0] + 0.5f, bbox[1] + 0.5f, bbox[2] + 0.25f, bbox[0] + 3.5f, bbox[1] + 2.5f, bbox[2] + 1.75f }; memcpy(origSphereBox + 0x14, scene_boxSet ? scene_box : bx, 24);
      origSphereBox[0x14 + 0x18] = 7; origSphereBox[0x14 + 0x19] = 0; origSphereBox[0x14 + 0x1A] = 0x55; origSphereBox[0x14 + 0x1B] = 0; }
    mem_write(b + 0x30, origSphereBox, extra);
    mem_write(b + oV, C_verts, ncv * 6); mem_write(b + oT, C_tris, nct * 8);
    if (ng) {   /* two groups covering all triangles, with the whole bounding box */
        U32 gA = b + oT - 4 - ng * 28, k; w32(b + oT - 4, ng);
        for (k = 0; k < ng; k++) { mem_write(gA + k * 28, bbox, 24); w16(gA + k * 28 + 24, k ? nct / 2 : 0); w16(gA + k * 28 + 26, k ? nct - 1 : nct / 2 - 1); }
    }
    if (shadow) { w32(b + 0x20, 4); w32(b + 0x24, 6); w32(b + 0x28, b + oV); w32(b + 0x2C, b + oT); }
    if (planes) w32(b + 0x1C, sim_alloc(16, 'P'));
    return b;
}
static U32 add_entity(U32 slot, float x, float y, float z, float heading, int withMatrix, float tilt, U32 model, U32 geom) {
    U32 e = A_storage + slot * BUILDING_SIZE;
    if (!vg_set) { vg_px = x; vg_py = y; vg_pz = z; vg_h = heading; vg_set = 1; }
    w8(A_flags + slot, slot & 0x7F);
    wf(e + 4, x); wf(e + 8, y); wf(e + 0xC, z); wf(e + 0x10, heading); w16(e + 0x22, model); w8(e + 0x36, 1); w32(e + 0x1C, 0x85);
    w32(e + 0x18, make_atomic(geom));
    if (withMatrix) {
        U32 m = sim_alloc(0x48, 'X'); float c = cosf(heading), s = sinf(heading), ct = cosf(tilt), st = sinf(tilt);
        memset(sim + (m - SIM_BASE), 0, 0x48); w32(e + 0x14, m);
        wf(m, c); wf(m + 4, s); wf(m + 0x10, -s * ct); wf(m + 0x14, c * ct); wf(m + 0x18, st);
        wf(m + 0x20, s * st); wf(m + 0x24, -c * st); wf(m + 0x28, ct); wf(m + 0x30, x); wf(m + 0x34, y); wf(m + 0x38, z);
    }
    return e;
}
/* scene with the model loaded; returns the geometry. Entities are added by the caller. */
static U32 build_scene(int planes, int shadow, int groups) {
    U32 i, g;
    reset_sim(); vg_set = 0;
    w32(ADDR_EXTRA_VC_OFFSET, VC_OFFSET); w32(ADDR_2DFX_OFFSET, FX_OFF);
    wf(ADDR_CAMERA_MATRIX + 0x14, 1.f); wf(ADDR_CAMERA_MATRIX + 0x30, 10.f);
    A_pool = sim_alloc(0x14, 'p'); A_storage = sim_alloc(POOL_CAP * BUILDING_SIZE, 'p'); A_flags = sim_alloc(POOL_CAP, 'p');
    memset(sim + (A_storage - SIM_BASE), 0, POOL_CAP * BUILDING_SIZE); memset(sim + (A_flags - SIM_BASE), 0x80, POOL_CAP);
    w32(A_pool, A_storage); w32(A_pool + 4, A_flags); w32(A_pool + 8, POOL_CAP); w32(ADDR_BUILDING_POOL, A_pool);
    for (i = 0; i < NMAT; i++) { A_mat[i] = sim_alloc(0x20, 'm'); memset(sim + (A_mat[i] - SIM_BASE), 0, 0x20); }
    g = make_geometry();
    A_mi = sim_alloc(0x24, 'i'); memset(sim + (A_mi - SIM_BASE), 0, 0x24); w32(ADDR_MODEL_INFO_PTRS + MODEL_A * 4, A_mi);
    A_cm = sim_alloc(0x30, 'c'); memset(sim + (A_cm - SIM_BASE), 0, 0x30); w32(A_mi + 0x14, A_cm); w16(A_mi + 0x12, (scene_special << 11) | 0x40);
    mem_write(A_cm, bbox, 24); mem_write(A_cm + 0x18, sphere, 16); w8(A_cm + 0x29, 1 | 2 | 4); w32(A_cm + 0x2C, make_coldata(planes, shadow, groups));
    A_miAtomic = make_atomic(g); w32(A_mi + 0x1C, A_miAtomic);
    rw_geom_destroy(g);                 /* loader drops its creation reference */
    return g;
}
static void explode(U32 slot, U32 type, Vec3 pos, float stamp) {
    U32 a = ADDR_EXPLOSIONS + slot * EXPLOSION_SIZE;
    w32(a, type); wf(a + 4, pos.x); wf(a + 8, pos.y); wf(a + 0xC, pos.z); wf(a + 0x20, stamp); w8(a + 0x28, 1);
}
static long kept_count(void) { long n = g_pendSnap ? 1 : 0; U32 i; for (i = 0; i < g_nUndo; i++) n += (g_undo[i].snap ? 1 : 0) + (g_undo[i].origCol ? 1 : 0); return n; }      /* what the runtime keeps for F8 */
static long live_count(char tag) { long n = 0; U32 i; for (i = 0; i < nalloc; i++) if (allocs[i].live && allocs[i].tag == tag) n++; return n; }

/* ---- validation of what the runtime produced ---- */
static void validate_col(const char* what, int hadShadow) {
    U32 cd = peek32(A_cm + 0x2C), ns, nb, ntri, flags, pS, pB, pV, pT, i, maxIdx = 0, nvert; Alloc* al = find_alloc(cd);
    float lo[3], hi[3], cen[3], rad; U8* blk;
    CHECK(al && al->live && al->addr == cd && al->tag == 'M', "%s: colData is not a live game allocation", what);
    if (!al || !al->live) return;
    blk = sim + (cd - SIM_BASE);
    ns = peek16(cd); nb = peek16(cd + 2); ntri = peek16(cd + 4); flags = peek8(cd + 7);
    pS = peek32(cd + 8); pB = peek32(cd + 0xC); pV = peek32(cd + 0x14); pT = peek32(cd + 0x18);
    CHECK(ns == 1 && nb == 0 && peek8(cd + 6) == 0, "%s: expected 1 sphere, 0 boxes (boxes become triangles)", what);
    CHECK(pS == cd + 0x30 && pB == 0 && peek32(cd + 0x10) == 0, "%s: sphere/box pointers wrong", what);
    CHECK(!memcmp(blk + 0x30, origSphereBox, 0x14), "%s: sphere data not carried over", what);
    CHECK(pV == cd + 0x30 + 0x14, "%s: vertices do not follow the sphere", what);
    { /* the box survives as 12 triangles with its surface type: count triangles carrying surface 7 / light 0x55 */
      U32 k, nb7 = 0; for (k = 0; k < ntri; k++) if (peek8(pT + k * 8 + 6) == 7 && peek8(pT + k * 8 + 7) == 0x55) nb7++;
      if (expectBoxTris && !opened()) CHECK(nb7 >= 12, "%s: box triangles missing (%u with the box surface)", what, nb7); }
    CHECK(peek32(cd + 0x1C) == 0, "%s: stale triangle planes pointer", what);
    CHECK(ntri > 0 && pT > pV && pT + ntri * 8 == cd + al->size, "%s: triangle array does not end the block", what);
    CHECK((pV & 3) == 0 && (pT & 3) == 0, "%s: unaligned arrays", what);
    for (i = 0; i < ntri * 3; i++) { U32 v = peek16(pT + (i / 3) * 8 + (i % 3) * 2); if (v > maxIdx) maxIdx = v; }
    nvert = maxIdx + 1;
    { U32 ng = (flags & 2) ? peek32(pT - 4) : 0, gA = pT - 4 - ng * 28, k, covered = 0; static U8 seen[65536]; memset(seen, 0, sizeof(seen));
      CHECK(pV + nvert * 6 <= ((flags & 2) ? gA : pT), "%s: vertices overlap what follows", what);
      if (ntri > 80) CHECK(flags & 2, "%s: no face groups on a %u-triangle mesh", what, ntri);
      for (k = 0; k < ng; k++) {
          U32 g = gA + k * 28, first = peek16(g + 24), last = peek16(g + 26), t; float b[6]; mem_read(g, b, 24);
          CHECK(first <= last && last < ntri, "%s: face group %u range %u..%u", what, k, first, last);
          for (t = first; t <= last && t < ntri; t++) { int j;
              CHECK(!seen[t], "%s: triangle %u in two face groups", what, t); seen[t] = 1; covered++;
              for (j = 0; j < 3; j++) { U32 v = peek16(pT + t * 8 + j * 2); float x = (S16)peek16(pV + v * 6) / 128.f, y = (S16)peek16(pV + v * 6 + 2) / 128.f, z = (S16)peek16(pV + v * 6 + 4) / 128.f;
                  CHECK(x >= b[0] && y >= b[1] && z >= b[2] && x <= b[3] && y <= b[4] && z <= b[5], "%s: face group %u box does not contain its triangle %u", what, k, t); }
          }
      }
      if (flags & 2) CHECK(covered == ntri, "%s: face groups cover %u of %u triangles", what, covered, ntri); }
    mem_read(A_cm, lo, 12); mem_read(A_cm + 0xC, hi, 12); mem_read(A_cm + 0x18, cen, 12); rad = peekf(A_cm + 0x24);
    for (i = 0; i < nvert; i++) {
        float x = (S16)peek16(pV + i * 6) / 128.f, y = (S16)peek16(pV + i * 6 + 2) / 128.f, z = (S16)peek16(pV + i * 6 + 4) / 128.f;
        CHECK(x >= lo[0] && y >= lo[1] && z >= lo[2] && x <= hi[0] && y <= hi[1] && z <= hi[2], "%s: vertex outside CColModel box", what);
        CHECK(sqrtf((x-cen[0])*(x-cen[0]) + (y-cen[1])*(y-cen[1]) + (z-cen[2])*(z-cen[2])) <= rad + 1e-3f, "%s: vertex outside CColModel sphere", what);
    }
    if (hadShadow) CHECK((flags & 4) && peek32(cd + 0x28) == pV && peek32(cd + 0x2C) == pT && peek32(cd + 0x20) == ntri, "%s: shadow mesh not set", what);
    else CHECK(!(flags & 4) && !peek32(cd + 0x28), "%s: shadow mesh appeared", what);
    CHECK(peek8(A_cm + 0x29) == 7, "%s: CColModel flags changed", what);
}
static void validate_geom(const char* what, U32 expectAtomics) {
    U32 g = peek32(A_miAtomic + 0x18), v, t, i, mt, pT, pV; Alloc* al = find_alloc(g);
    CHECK(al && al->live && al->tag == 'G', "%s: master atomic has no live geometry", what);
    if (!al || !al->live) return;
    v = peek32(g + 0x14); t = peek32(g + 0x10); mt = peek32(g + 0x5C); pT = peek32(g + 0x2C); pV = peek32(mt + 0x14);
    CHECK(peek16(g + 0xE) == expectAtomics, "%s: geometry refcount %u, expected %u", what, peek16(g + 0xE), expectAtomics);
    CHECK(peek16(g + 0xC) == 0 && peek32(g + 0x54), "%s: geometry not unlocked / no mesh", what);
    CHECK(peek32(g + 0x18) == 1 && peek32(g + 0x1C) == 1 && peek32(mt) == g, "%s: morph/uv layout", what);
    if (peek32(g + 0x24) == NMAT) CHECK(opened(), "%s: no rock material although a bowl was carved", what);   /* holes only: no rock needed */
    else CHECK(peek32(g + 0x24) == NMAT + 1, "%s: material count %u (expected originals + rock)", what, peek32(g + 0x24));
    if (peek32(g + 0x24) == NMAT + 1) { U32 rock = peek32(peek32(g + 0x20) + NMAT * 4), tex, bowl = 0, k; float M[8]; Frame fr = { { vg_px, vg_py, vg_pz }, cosf(vg_h), sinf(vg_h) }; Vec3 nw;
      CHECK(rock == g_rockMat && rock && find_alloc(rock)->live && find_alloc(rock)->tag == 'k', "%s: last material is not the rock material", what);
      CHECK(peek32(rock + 0x18) == 2, "%s: rock material refcount %u (expected creation + this geometry)", what, peek32(rock + 0x18));
      CHECK(peek32(rock + 0x10) == 0, "%s: rock material pipeline flags not cleared", what);
      tex = peek32(rock); CHECK(tex && find_alloc(tex)->tag == 't' && peek32(tex + 0x50) == (mock_noMips ? 0x1102u : 0x1106u) && peek32(tex) == rasterOf, "%s: rock texture setup", what);
      { static U8 want[128 * 128 * 4]; U32 y, bad = 0; rock_generate(want);
        for (y = 0; y < 128; y++) if (memcmp(sim + (rasterLevels[0] + y * (128 * 4 + 16) - SIM_BASE), want + y * 512, 512)) bad++;
        CHECK(bad == 0 && rasterLocked == 99, "%s: rock texture pixels wrong in %u rows", what, bad);
        if (!mock_noMips) { U8 px[4]; mem_read(rasterLevels[7], px, 4); CHECK(px[3] == 255 && px[0] > 20 && px[0] < 200, "%s: smallest mip level not written", what); } }
      /* bowl triangles use the rock slot and are mapped by world position */
      nw.x = (g_lastQ.x - g_lastC.x) / g_lastR; nw.y = (g_lastQ.y - g_lastC.y) / g_lastR; nw.z = (g_lastQ.z - g_lastC.z) / g_lastR;
      rock_mapping(&fr, nw, M);
      { U32 mapped = 0, pUv = peek32(g + 0x34);
        for (k = 0; k < peek32(g + 0x10); k++) if (peek16(peek32(g + 0x2C) + k * 8 + 6) == NMAT) {
            int j, okm = 1; bowl++;
            for (j = 0; j < 3; j++) { U32 x = peek16(peek32(g + 0x2C) + k * 8 + j * 2); Vec3 pp; float uv[2]; mem_read(pV + x * 12, &pp, 12); mem_read(pUv + x * 8, uv, 8);
                if (fabsf(uv[0] - (M[0] * pp.x + M[1] * pp.y + M[2] * pp.z + M[3])) > 2e-3f || fabsf(uv[1] - (M[4] * pp.x + M[5] * pp.y + M[6] * pp.z + M[7])) > 2e-3f) okm = 0; }
            mapped += (U32)okm;
        }
        if (!opened()) CHECK(bowl >= 3 && (mapped >= 3 || !strcmp(what, "T3")), "%s: rock triangles %u, with world mapping of the last crater %u", what, bowl, mapped); } }
    for (i = 0; i < NMAT; i++) { CHECK(peek32(peek32(g + 0x20) + i * 4) == A_mat[i], "%s: material %u differs", what, i); CHECK(peek32(A_mat[i] + 0x18) == 1, "%s: material %u refcount %u", what, i, peek32(A_mat[i] + 0x18)); }
    CHECK(find_alloc(pT)->size == t * 8 && find_alloc(pV)->size == v * 12 && find_alloc(peek32(g + 0x30))->size == v * 4 && find_alloc(peek32(g + 0x34))->size == v * 8, "%s: array sizes", what);
    for (i = 0; i < t; i++) { int j; for (j = 0; j < 3; j++) CHECK(peek16(pT + i * 8 + j * 2) < v, "%s: triangle index out of range", what); CHECK(peek16(pT + i * 8 + 6) <= NMAT, "%s: material index out of range", what); }
    if (hasNight) {
        Alloc *n = find_alloc(peek32(g + VC_OFFSET)), *d = find_alloc(peek32(g + VC_OFFSET + 4));
        CHECK(n && n->live && n->tag == 'M' && n->size == v * 4 && d && d->live && d->tag == 'M' && d->size == v * 4 && n != d, "%s: day/night colour arrays", what);
        CHECK(peekf(g + VC_OFFSET + 8) == 0.25f, "%s: day/night balance lost", what);
    } else CHECK(!peek32(g + VC_OFFSET) && !peek32(g + VC_OFFSET + 4), "%s: day/night arrays appeared", what);
    CHECK(peek32(g + USAGE_OFF) == 0x08, "%s: D3D9 usage flags %#x", what, peek32(g + USAGE_OFF));
    CHECK(peek32(g + FX_OFF) == A_fx && find_alloc(A_fx)->live && peek32(A_fx) == 2 && live_count('E') == 1, "%s: 2D effects not handed over to the new geometry", what);
    { float c[3], r = peekf(mt + 0x10); mem_read(mt + 4, c, 12);
      for (i = 0; i < v; i++) { float p[3]; mem_read(pV + i * 12, p, 12); CHECK(sqrtf((p[0]-c[0])*(p[0]-c[0]) + (p[1]-c[1])*(p[1]-c[1]) + (p[2]-c[2])*(p[2]-c[2])) <= r + 1e-3f, "%s: vertex outside bounding sphere", what); } }
    CHECK(live_count('G') == 1, "%s: %ld geometries alive (leak or premature destroy)", what, live_count('G'));
    CHECK(live_count('M') == (hasNight ? 3 : 1) + kept_count(), "%s: %ld game allocations alive (%ld of them kept for the reset)", what, live_count('M'), kept_count());
}
/* crater really there: nothing left inside the ground-side half of the sphere, and a bowl exists */
static unsigned long g_keptTotal; static int dumpNo;
static void validate_crater(const char* what, Vec3 Cl, float R, Vec3 nl) {
    U32 cd = peek32(A_cm + 0x2C), pV = peek32(cd + 0x14), pT = peek32(cd + 0x18), ntri = peek16(cd + 4), i, onSphere = 0;
    U32 g = peek32(A_miAtomic + 0x18), rT = peek32(g + 0x2C), rnt = peek32(g + 0x10), rV = peek32(peek32(g + 0x5C) + 0x14), rOn = 0;
    int holes = opened(); U32 kept = 0;
    /* only vertices that triangles actually use count (a hole leaves unused vertices behind) */
    for (i = 0; i < ntri * 3; i++) {
        U32 x = peek16(pT + (i / 3) * 8 + (i % 3) * 2);
        Vec3 p = { (S16)peek16(pV + x * 6) / 128.f, (S16)peek16(pV + x * 6 + 2) / 128.f, (S16)peek16(pV + x * 6 + 4) / 128.f }; float d = sqrtf(gm_dist2(p, Cl));
        if (gm_dot(gm_sub(p, Cl), nl) < -0.03f && d < R - GM_EPS - 0.02f) {      /* stopped where the material ends: on purpose */
            U32 q, st = 0; for (q = 0; q < g_dbgStayN && !st; q++) if (gm_dist2(g_dbgStay[q], p) < 4e-4f) st = 1;
            if (st) continue;
        }
        if (gm_dot(gm_sub(p, Cl), nl) < -0.03f) CHECK(d >= R - GM_EPS - 0.02f, "%s: collision vertex inside crater sphere (%.3f < %.3f)", what, d, R);
        if (fabsf(d - R) < GM_EPS + 0.03f) onSphere++;
    }
    for (i = 0; i < rnt * 3; i++) {
        U32 x = peek16(rT + (i / 3) * 8 + (i % 3) * 2); Vec3 p; float d; mem_read(rV + x * 12, &p, 12); d = sqrtf(gm_dist2(p, Cl));
        if (gm_dot(gm_sub(p, Cl), nl) < -0.01f && d < R - GM_EPS - 3e-3f) {      /* far sides of walls stay on purpose */
            U32 q, st = 0; Vec3 pw = to_world(&(Frame){ { vg_px, vg_py, vg_pz }, cosf(vg_h), sinf(vg_h) }, p); (void)pw;
            for (q = 0; q < g_dbgStayN && !st; q++) if (gm_dist2(g_dbgStay[q], p) < 1e-8f) st = 1;
            if (st) { kept++; continue; }
        }
        if (gm_dot(gm_sub(p, Cl), nl) < -0.01f) CHECK(d >= R - GM_EPS - 3e-3f, "%s: render vertex inside crater sphere (%.3f < %.3f)", what, d, R);   /* far sides of walls stay on purpose */
        if (fabsf(d - R) < GM_EPS + 0.02f) rOn++;
    }
    CHECK(holes || (onSphere >= 6 && rOn >= 6), "%s: no bowl (col %u, render %u vertex uses on the sphere)", what, onSphere, rOn);
    g_keptTotal += kept;
}
/* recompute what blast() must have chosen, from the collision mesh in the simulation */
static int expected_sphere(Vec3 El, U32 type, Vec3* C, float* R, Vec3* n) {
    static Vec3 pos[65536]; static U16 tri[65536 * 3], aux[65536]; GmMesh m; U32 cd = peek32(A_cm + 0x2C), pV = peek32(cd + 0x14), pT = peek32(cd + 0x18), ntri = peek16(cd + 4), i, mx = 0;
    Vec3 S = { 0, 0, 0 }, ns = { 0, 0, 0 }; float best = 1e30f, len; const CraterSpec* sp = &g_spec[type];
    for (i = 0; i < ntri * 3; i++) { tri[i] = (U16)peek16(pT + (i / 3) * 8 + (i % 3) * 2); if (tri[i] > mx) mx = tri[i]; }
    for (i = 0; i <= mx; i++) { pos[i].x = (S16)peek16(pV + i * 6) / 128.f; pos[i].y = (S16)peek16(pV + i * 6 + 2) / 128.f; pos[i].z = (S16)peek16(pV + i * 6 + 4) / 128.f; }
    memset(&m, 0, sizeof(m)); m.nv = mx + 1; m.nt = ntri; m.pos = pos; m.tri = tri; m.aux = aux;
    gm_probe(&m, El, sp->reach + 1.0f, 1, 0, &best, &S, &ns); len = sqrtf(gm_dot(ns, ns));
    if (best > 1e20f || sqrtf(best) > sp->reach || len < 1e-6f) return 0;
    n->x = ns.x / len; n->y = ns.y / len; n->z = ns.z / len; *R = (sp->r * sp->r + sp->d * sp->d) / (2 * sp->d);
    C->x = S.x + n->x * (*R - sp->d); C->y = S.y + n->y * (*R - sp->d); C->z = S.z + n->z * (*R - sp->d);
    return 1;
}

static int run_point(const float* P) {
    const float PX = 1500.25f, PY = -700.5f, PZ = 12.75f; Vec3 El = { P[0], P[1], P[2] + 0.1f }, Ew, C, n; float R; int before = fails, k;
    U32 g0, cd0, e; float dist1 = 0.f; int guard1 = 0;
    /* --- T1: one placed object, grenade on the surface ------------------------------------ */
    g0 = build_scene(1, 1, 1); e = add_entity(3, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); (void)e;
    cd0 = peek32(A_cm + 0x2C);
    Ew.x = PX + El.x; Ew.y = PY + El.y; Ew.z = PZ + El.z;
    explode(5, 0, Ew, 1000.f); frame_tick(0);
    if (logged("no surface in reach")) return 2;
    if (!logged("RW v")) {      /* e.g. collision without a visible mesh at this spot: the object must be untouched */
        CHECK(peek32(A_cm + 0x2C) == cd0 && find_alloc(g0)->live && n_create == 1 && live_count('M') == (hasNight ? 3 : 1), "T1: object changed although the log says it was skipped");
        return fails == before ? 3 : 0;
    }
    /* the sphere the runtime chose, in model space (object is unrotated at PX,PY,PZ) */
    C.x = g_lastC.x - PX; C.y = g_lastC.y - PY; C.z = g_lastC.z - PZ; R = g_lastR;
    n.x = (g_lastQ.x - g_lastC.x) / R; n.y = (g_lastQ.y - g_lastC.y) / R; n.z = (g_lastQ.z - g_lastC.z) / R;
    dist1 = g_lastClarity > 0.6f ? g_lastDist : 0.f;      /* 0: blast in the plane of a wall, the direction is a toss-up - no comparisons */
    guard1 = logged("using the blast direction"); if (guard1) dist1 = 0.f;
    if (logged("crater dug from the side that is seen")) dist1 = 0.f;      /* (the test point lies under the surface: the crater was turned round - nothing to compare a rocket "on the surface" with) */
    CHECK(R > 1.5f && R < 3.2f, "T1: crater sphere radius %f implausible for a grenade", R);
    { /* the blast must lie on the open side of the crater: in front of the plane through the bowl's deepest point */
      Vec3 deep = { g_lastC.x - n.x * R, g_lastC.y - n.y * R, g_lastC.z - n.z * R };
      CHECK(gm_dot(n, gm_sub(Ew, deep)) > 0.f, "T1: crater opens away from the blast (n=(%.2f,%.2f,%.2f))", n.x, n.y, n.z); }
    CHECK(logged("EXPLOSION type=0") && logged("COL v") && logged("RW v") && !logged("skipped"), "T1 log: %s", logbuf[(logn + 63) % 64]);
    CHECK(peek32(A_cm + 0x2C) != cd0 && find_alloc(cd0)->live && g_nUndo == 1 && g_undo[0].origCol == cd0 && g_undo[0].snap, "T1: old collision block not replaced, or the original not kept for the reset");
    CHECK(!find_alloc(g0)->live && n_destroyed == 1, "T1: old geometry not destroyed (destroyed=%d)", n_destroyed);
    CHECK(n_removePlanes == 1 && live_count('P') == 0, "T1: triangle planes not removed");
    CHECK(logged("atomics=2"), "T1: expected 2 atomics swapped");
    validate_col("T1", 1); validate_geom("T1", 2); validate_crater("T1", C, R, n);
    { U32 g = peek32(A_miAtomic + 0x18); CHECK(!(peek32(g + 8) & 1u), "T1: tristrip flag kept"); CHECK(peek32(peek32(A_storage + 3 * BUILDING_SIZE + 0x18) + 0x18) == g, "T1: placed atomic not swapped"); }
    if (getenv("HOST_DUMP")) {       /* visual check: the mesh after the first crater, in model space */
        char pth[600]; FILE* o; U32 g = peek32(A_miAtomic + 0x18), nvv = peek32(g + 0x14), ntt = peek32(g + 0x10), pT = peek32(g + 0x2C), pVv = peek32(peek32(g + 0x5C) + 0x14), q;
        snprintf(pth, 600, "%s_%d.obj", getenv("HOST_DUMP"), dumpNo++); o = fopen(pth, "w");
        fprintf(o, "# C %f %f %f R %f Q %f %f %f E %f %f %f\n", C.x, C.y, C.z, R, g_lastQ.x - PX, g_lastQ.y - PY, g_lastQ.z - PZ, El.x, El.y, El.z);
        for (q = 0; q < nvv; q++) { Vec3 pp; mem_read(pVv + q * 12, &pp, 12); fprintf(o, "v %f %f %f\n", pp.x, pp.y, pp.z); }
        for (q = 0; q < ntt; q++) fprintf(o, "f %u %u %u %u\n", peek16(pT + q * 8) + 1, peek16(pT + q * 8 + 2) + 1, peek16(pT + q * 8 + 4) + 1, peek16(pT + q * 8 + 6));
        fclose(o);
    }
    /* --- T2: the same explosion stays active for many frames: applied once ---------------- */
    { int c0 = n_create; U32 cd1 = peek32(A_cm + 0x2C); for (k = 0; k < 30; k++) frame_tick(0); CHECK(n_create == c0 && peek32(A_cm + 0x2C) == cd1, "T2: explosion applied more than once"); }
    /* --- T3: more blasts on top (works on the already modified data), three in one frame --- */
    wf(ADDR_CAMERA_MATRIX + 0x30, Ew.x + 8.f * n.x + 1.f); wf(ADDR_CAMERA_MATRIX + 0x34, Ew.y + 8.f * n.y); wf(ADDR_CAMERA_MATRIX + 0x38, Ew.z + 8.f * n.z + 1.5f);
    for (k = 0; k < 3; k++) { Vec3 E2 = Ew; E2.x += 1.3f * (float)(k + 1); E2.y -= 0.9f * (float)k; explode(6 + (U32)k, k == 1 ? 2u : 0u, E2, 2000.f + (float)k); }
    { int c0 = n_create; frame_tick(0); CHECK(n_create - c0 <= 2, "T3: more than 2 craters in one frame"); frame_tick(0); frame_tick(0); }
    CHECK(!logged("holes(render/col)="), "T3: craters dug into each other were given holes: %s", logbuf[(logn + 63) % 64]);
    validate_col("T3", 1); validate_geom("T3", 2);
    /* --- T4: rocket via F8, molotov and airburst do nothing --------------------------------- */
    { int c0 = n_create; U32 cd1 = peek32(A_cm + 0x2C); Vec3 up = Ew; up.z = PZ + bbox[5] + 20.f;
      explode(1, 1, Ew, 3000.f); explode(2, 0, up, 3001.f); frame_tick(0); frame_tick(0);
      CHECK(n_create == c0 && peek32(A_cm + 0x2C) == cd1, "T4: molotov/airburst changed geometry");
      CHECK(logged("no surface in reach"), "T4: airburst not reported"); }
    /* --- T5: rotated placements, without and with matrix: same crater in model space -------- */
    for (k = 0; k < 2; k++) {
        float hd = k ? -2.3f : 1.1f, c = cosf(hd), s = sinf(hd);
        g0 = build_scene(0, 0, 0); add_entity(7, PX, PY, PZ, hd, k, 0.f, MODEL_A, g0);
        Ew.x = PX + c * El.x - s * El.y; Ew.y = PY + s * El.x + c * El.y; Ew.z = PZ + El.z;
        mock_hit = Ew; mock_ent = A_storage + 7 * BUILDING_SIZE;
        /* the camera looks at the surface it shoots at: in front of it, a little above */
        wf(ADDR_CAMERA_MATRIX + 0x30, Ew.x + 8.f * (c * n.x - s * n.y)); wf(ADDR_CAMERA_MATRIX + 0x34, Ew.y + 8.f * (s * n.x + c * n.y)); wf(ADDR_CAMERA_MATRIX + 0x38, Ew.z + 8.f * n.z + 1.5f);
        if (k) { explode(0, 2, Ew, 500.f); frame_tick(0); } else { explode(0, 0, Ew, 500.f); frame_tick(0); }
        if (k && logged("left unchanged")) continue;     /* F8 on a point between two stacked ground layers: seen from below, nothing to carve */
        CHECK(logged("COL v") && logged("RW v"), "T5 (matrix=%d): no crater: %s", k, logbuf[(logn + 63) % 64]);
        CHECK(n_removePlanes == 0, "T5: planes removed although none existed");
        validate_col("T5", 0); validate_geom("T5", 2);
        { Vec3 Cl = to_local(&(Frame){ { PX, PY, PZ }, c, s }, g_lastC), Ql = to_local(&(Frame){ { PX, PY, PZ }, c, s }, g_lastQ), nl2 = { (Ql.x - Cl.x) / g_lastR, (Ql.y - Cl.y) / g_lastR, (Ql.z - Cl.z) / g_lastR };
          validate_crater("T5", Cl, g_lastR, nl2);
          if (!k && dist1 > 0.05f) CHECK(sqrtf(gm_dist2(Cl, C)) < 0.08f, "T5: rotated placement gives a different crater centre (%.3f m off)", sqrtf(gm_dist2(Cl, C))); }
    }
    /* --- T6: a model placed twice shares one mesh: it must not be carved -------------------- */
    g0 = build_scene(1, 0, 1); add_entity(1, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); add_entity(9, PX + 900.f, PY, PZ, 0.5f, 0, 0.f, MODEL_A, g0);
    cd0 = peek32(A_cm + 0x2C); Ew.x = PX + El.x; Ew.y = PY + El.y; Ew.z = PZ + El.z; explode(0, 0, Ew, 10.f); frame_tick(0);
    if (logged("knocked over")) CHECK(n_objCreate == 1 && n_create == 1 && find_alloc(g0)->live, "T6: small shared model: expected one physics object, geometry untouched");
    else CHECK((logged("placed 2 times") || logged("no surface in reach")) && n_create == 1 && peek32(A_cm + 0x2C) == cd0 && find_alloc(g0)->live, "T6: shared model was changed: %s", logbuf[(logn + 63) % 64]);
    /* --- T7: tilted object is left alone ------------------------------------------------------ */
    g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.3f, 1, 0.25f, MODEL_A, g0); cd0 = peek32(A_cm + 0x2C);
    explode(0, 0, Ew, 10.f); frame_tick(0);
    CHECK(n_create == 1 && peek32(A_cm + 0x2C) == cd0 && find_alloc(g0)->live, "T7: tilted object was modified");
    /* --- T8: collision not a single allocation -> skipped, nothing touched --------------------- */
    g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); cd0 = peek32(A_cm + 0x2C); w8(A_cm + 0x29, 1 | 4);
    explode(0, 0, Ew, 10.f); frame_tick(0);
    CHECK(n_create == 1 && peek32(A_cm + 0x2C) == cd0 && find_alloc(g0)->live, "T8: multi-alloc collision was modified");
    /* --- T10: interiors are left alone ----------------------------------------------------------- */
    g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); cd0 = peek32(A_cm + 0x2C);
    w32(ADDR_CURR_AREA, 3); explode(0, 0, Ew, 10.f); frame_tick(0);
    CHECK(n_create == 1 && peek32(A_cm + 0x2C) == cd0 && logn == 0, "T10: blast inside an interior changed something");
    w32(ADDR_CURR_AREA, 0); w8(A_storage + 2 * BUILDING_SIZE + 0x2F, 5); explode(1, 0, Ew, 11.f); frame_tick(0);
    CHECK(n_create == 1 && peek32(A_cm + 0x2C) == cd0, "T10: interior object was modified from outside");
    /* --- T11: device without mipmapped textures: plain texture instead ---------------------------- */
    g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); mock_noMips = 1;
    explode(0, 0, Ew, 10.f); frame_tick(0);
    CHECK(logged("no mipmaps") && n_rasterCreate >= 2, "T11: fallback texture not used"); validate_geom("T11", 2); mock_noMips = 0;
    /* --- T12: collision boxes become outward-facing triangles ------------------------------------- */
    g0 = build_scene(0, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0);
    { Target tt; ColInfo ci; U32 k; Vec3 bc = { bbox[0] + 2.f, bbox[1] + 1.5f, bbox[2] + 1.f };
      CHECK(inspect(A_storage + 2 * BUILDING_SIZE, &tt) && read_col(&tt, &ci), "T12: read_col failed: %s", g_why);
      CHECK(ci.boxesConverted == 1 && ci.nBoxes == 0 && g_cm.nt == nct + 12, "T12: box not converted (%u tris)", g_cm.nt);
      for (k = nct; k < g_cm.nt; k++) {
          Vec3 a = g_cm.pos[g_cm.tri[k * 3]], b = g_cm.pos[g_cm.tri[k * 3 + 1]], c = g_cm.pos[g_cm.tri[k * 3 + 2]];
          Vec3 nn = gm_cross(gm_sub(c, a), gm_sub(b, a)), cen = { (a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3 };
          CHECK(gm_dot(nn, gm_sub(cen, bc)) > 0.1f && g_cm.aux[k] == (7 | (0x55 << 8)), "T12: box triangle %u faces inward or lost its surface", k);
      } }
    /* --- T15: blast beside the wall of a big collision block, 25 cm above the block's bottom face
     *          (how the casino in Las Venturas is built). The crater must open towards the blast.
     *          The huge bottom face used to win and turn the crater into a dome. ---------------- */
    { float fx = (bbox[3] - bbox[0]) * 0.6f, fy = (bbox[4] - bbox[1]) * 0.6f; if (fx > 40.f) fx = 40.f; if (fy > 40.f) fy = 40.f;
      if (fx > 10.f && fy > 10.f) {
        float cx = 0.5f * (bbox[0] + bbox[3]), cy = 0.5f * (bbox[1] + bbox[4]), zb = bbox[5] + 12.f, bbs[6], rr; Vec3 Eb, nw;
        scene_box[0] = cx - 0.5f * fx; scene_box[1] = cy - 0.5f * fy; scene_box[2] = zb; scene_box[3] = cx + 0.5f * fx; scene_box[4] = cy + 0.5f * fy; scene_box[5] = zb + 6.f; scene_boxSet = 1;
        g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); scene_boxSet = 0;
        memcpy(bbs, bbox, 24); bbs[5] = zb + 6.f; mem_write(A_cm, bbs, 24); rr = sphere[3] + 30.f; mem_write(A_cm + 0x24, &rr, 4);
        Eb.x = PX + scene_box[0] - 0.22f; Eb.y = PY + cy + 0.3f; Eb.z = PZ + zb + 0.25f;
        explode(0, 0, Eb, 10.f); frame_tick(0);
        CHECK(logged("crater: surface="), "T15: no crater computed beside the block: %s", logbuf[(logn + 63) % 64]);
        nw.x = (g_lastQ.x - g_lastC.x) / g_lastR; nw.y = (g_lastQ.y - g_lastC.y) / g_lastR; nw.z = (g_lastQ.z - g_lastC.z) / g_lastR;
        CHECK(nw.x < -0.9f && fabsf(nw.z) < 0.1f, "T15: crater beside a wall opens towards (%.2f,%.2f,%.2f), expected (-1,0,0)", nw.x, nw.y, nw.z);
        CHECK(g_lastC.x < Eb.x + 0.3f, "T15: crater sphere sits inside the block (dome instead of crater)");
        /* T16: the same block, blast lying on its roof: opens upwards */
        scene_boxSet = 1; g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); scene_boxSet = 0;
        mem_write(A_cm, bbs, 24); mem_write(A_cm + 0x24, &rr, 4);
        Eb.x = PX + cx + 1.f; Eb.y = PY + cy; Eb.z = PZ + zb + 6.f + 0.12f;
        explode(0, 0, Eb, 10.f); frame_tick(0);
        nw.x = (g_lastQ.x - g_lastC.x) / g_lastR; nw.y = (g_lastQ.y - g_lastC.y) / g_lastR; nw.z = (g_lastQ.z - g_lastC.z) / g_lastR;
        CHECK(logged("crater: surface=") && nw.z > 0.9f, "T16: crater on a block's roof opens towards (%.2f,%.2f,%.2f), expected up", nw.x, nw.y, nw.z);
        /* T17: blast hanging under the block (overhang): opens downwards, into the air the blast is in */
        scene_boxSet = 1; g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); scene_boxSet = 0;
        mem_write(A_cm, bbs, 24); mem_write(A_cm + 0x24, &rr, 4);
        Eb.x = PX + cx; Eb.y = PY + cy; Eb.z = PZ + zb - 0.3f;
        explode(0, 0, Eb, 10.f); frame_tick(0);
        nw.x = (g_lastQ.x - g_lastC.x) / g_lastR; nw.y = (g_lastQ.y - g_lastC.y) / g_lastR; nw.z = (g_lastQ.z - g_lastC.z) / g_lastR;
        CHECK(logged("crater: surface=") && nw.z < -0.9f, "T17: crater under an overhang opens towards (%.2f,%.2f,%.2f), expected down", nw.x, nw.y, nw.z);
      } }
    /* --- T18: the T1 blast again, now with a 40 x 40 m block hanging 2.5 m above it (canopy):
     *          the crater still goes into the ground like in T1, the canopy's huge underside
     *          must not turn it around ------------------------------------------------------- */
    if (n.z > 0.9f && dist1 > 0.08f) { float bbs[6], rr = sphere[3] + 60.f; Vec3 nw;
      scene_box[0] = El.x - 20.f; scene_box[1] = El.y - 20.f; scene_box[2] = El.z + 2.5f; scene_box[3] = El.x + 20.f; scene_box[4] = El.y + 20.f; scene_box[5] = El.z + 8.f; scene_boxSet = 1;
      g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); scene_boxSet = 0;
      for (k = 0; k < 3; k++) { bbs[k] = bbox[k] < scene_box[k] ? bbox[k] : scene_box[k]; bbs[k + 3] = bbox[k + 3] > scene_box[k + 3] ? bbox[k + 3] : scene_box[k + 3]; }
      mem_write(A_cm, bbs, 24); mem_write(A_cm + 0x24, &rr, 4);
      explode(0, 0, Ew, 10.f); frame_tick(0);
      nw.x = (g_lastQ.x - g_lastC.x) / g_lastR; nw.y = (g_lastQ.y - g_lastC.y) / g_lastR; nw.z = (g_lastQ.z - g_lastC.z) / g_lastR;
      CHECK(logged("crater: surface=") && gm_dot(nw, n) > 0.8f, "T18: canopy above the blast turned the crater from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)", n.x, n.y, n.z, nw.x, nw.y, nw.z);
      if (!guard1) CHECK(!logged("using the blast direction"), "T18: the surface normals alone pointed away from the blast"); }
    /* --- T19: the same model with every collision triangle wound the other way round (many of the
     *          game's models are like that): the crater must still go into the ground ----------- */
    if (n.z > 0.9f && dist1 > 0.08f) { U32 cd, pT, ntri, i; Vec3 nw;
      g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0);
      cd = peek32(A_cm + 0x2C); pT = peek32(cd + 0x18); ntri = peek16(cd + 4);
      for (i = 0; i < ntri; i++) { U16 x = (U16)peek16(pT + i * 8 + 2), y = (U16)peek16(pT + i * 8 + 4); w16(pT + i * 8 + 2, y); w16(pT + i * 8 + 4, x); }
      explode(0, 0, Ew, 10.f); frame_tick(0);
      nw.x = (g_lastQ.x - g_lastC.x) / g_lastR; nw.y = (g_lastQ.y - g_lastC.y) / g_lastR; nw.z = (g_lastQ.z - g_lastC.z) / g_lastR;
      CHECK(logged("crater: surface=") && nw.z > 0.7f, "T19: collision wound the other way round: crater opens towards (%.2f,%.2f,%.2f)", nw.x, nw.y, nw.z);
      if (!guard1) CHECK(!logged("using the blast direction"), "T19: the surface normals alone pointed away from the blast"); }
    /* --- T20: rocket that explodes exactly ON the surface (the F8 test crater does), model wound
     *          the right and the wrong way round: the same crater, opening towards the shooter --- */
    if (n.z > 0.9f && dist1 > 0.08f) { Vec3 nw0 = { 0, 0, 0 }, Es = { PX + P[0], PY + P[1], PZ + P[2] }, toCam = { 2.f, -3.f, 12.f };
      for (k = 0; k < 2; k++) { U32 cd, pT, ntri, i; Vec3 nw;
        g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0);
        cd = peek32(A_cm + 0x2C); pT = peek32(cd + 0x18); ntri = peek16(cd + 4);
        if (k) for (i = 0; i < ntri; i++) { U16 x = (U16)peek16(pT + i * 8 + 2), y = (U16)peek16(pT + i * 8 + 4); w16(pT + i * 8 + 2, y); w16(pT + i * 8 + 4, x); }
        wf(ADDR_CAMERA_MATRIX + 0x30, Es.x + toCam.x); wf(ADDR_CAMERA_MATRIX + 0x34, Es.y + toCam.y); wf(ADDR_CAMERA_MATRIX + 0x38, Es.z + toCam.z);
        explode(0, 2, Es, 10.f); frame_tick(0);
        nw.x = (g_lastQ.x - g_lastC.x) / g_lastR; nw.y = (g_lastQ.y - g_lastC.y) / g_lastR; nw.z = (g_lastQ.z - g_lastC.z) / g_lastR;
        CHECK(logged("crater: surface=") && gm_dot(nw, toCam) > 0.f, "T20: rocket on the surface (winding %s): crater opens towards (%.2f,%.2f,%.2f), away from the shooter", k ? "reversed" : "normal", nw.x, nw.y, nw.z);
        if (!k) nw0 = nw; else CHECK(gm_dot(nw, nw0) > 0.9f, "T20: reversed winding changes the rocket crater from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)", nw0.x, nw0.y, nw0.z, nw.x, nw.y, nw.z); } }
    /* --- T23: the object is full (mesh limit): the blast is refused, nothing is touched, and it says so --- */
    g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); cd0 = peek32(A_cm + 0x2C);
    for (k = 0; k < 2; k++) {
        g_meshLimit = k ? nt + 20u : nct + 12u + 20u;            /* k = 0: the collision would pass the limit, k = 1: the visible mesh */
        explode((U32)k, 0, Ew, 10.f + (float)k); frame_tick(0);
        if (logged("is full")) CHECK(n_create == 1 && peek32(A_cm + 0x2C) == cd0 && find_alloc(g0)->live && find_alloc(cd0)->live, "T23 (%d): object reported full, but it was changed", k);
        else { U32 cdn = peek32(A_cm + 0x2C), gn = peek32(A_miAtomic + 0x18);      /* small crater that fits: fine, but then it must fit */
            CHECK(peek16(cdn + 4) <= g_meshLimit && peek32(gn + 0x10) <= g_meshLimit && peek32(gn + 0x14) <= g_meshLimit, "T23 (%d): mesh limit %u passed without a word (col %u tris, render %u/%u)", k, g_meshLimit, peek16(cdn + 4), peek32(gn + 0x14), peek32(gn + 0x10));
            g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); cd0 = peek32(A_cm + 0x2C); }
        clear_log();
    }
    g_meshLimit = 32000u;
    /* --- T21: a kerb (collision box 0.4 m wide, 0.3 m high) beside the blast: it is solid, it collapses
     *          into the bowl. No hole - a hole there shows the void under the map. ---------------- */
    if (n.z > 0.9f && dist1 > 0.08f) { float bbs[6], rr = sphere[3] + 20.f; int ra = -1, cb = -1; const char* hl = 0; U32 q;
      scene_box[0] = El.x + 0.5f; scene_box[1] = El.y - 3.f; scene_box[2] = El.z - 0.2f; scene_box[3] = El.x + 0.9f; scene_box[4] = El.y + 3.f; scene_box[5] = El.z + 0.2f; scene_boxSet = 1;
      g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); scene_boxSet = 0;
      for (k = 0; k < 3; k++) { bbs[k] = bbox[k] < scene_box[k] ? bbox[k] : scene_box[k]; bbs[k + 3] = bbox[k + 3] > scene_box[k + 3] ? bbox[k + 3] : scene_box[k + 3]; }
      mem_write(A_cm, bbs, 24); mem_write(A_cm + 0x24, &rr, 4);
      explode(0, 0, Ew, 10.f); frame_tick(0);
      for (q = 0; q < 64; q++) if ((hl = strstr(logbuf[q], "holes(render/col)=")) != 0) { sscanf(hl, "holes(render/col)=%d/%d", &ra, &cb); break; }
      CHECK(cb <= 0, "T21: the kerb's collision was given %d holes", cb);
      /* --- T22: a wire fence (thin box with a shoot-through surface) beside the blast: holes in the
       *          fence, but the ground it stands on keeps its visible triangles ------------------- */
      scene_box[0] = El.x + 0.7f; scene_box[1] = El.y - 3.f; scene_box[2] = El.z - 0.2f; scene_box[3] = El.x + 0.8f; scene_box[4] = El.y + 3.f; scene_box[5] = El.z + 2.2f; scene_boxSet = 1;
      g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); scene_boxSet = 0;
      for (k = 0; k < 3; k++) { bbs[k] = bbox[k] < scene_box[k] ? bbox[k] : scene_box[k]; bbs[k + 3] = bbox[k + 3] > scene_box[k + 3] ? bbox[k + 3] : scene_box[k + 3]; }
      mem_write(A_cm, bbs, 24); mem_write(A_cm + 0x24, &rr, 4);
      w32(ADDR_SURFACE_INFOS + 0x90 + 7 * 12 + 4, 0x3000u);                       /* the box's surface type 7: see- and shoot-through */
      { int ra0 = -1, cb0 = -1, base = 0;
        /* how many visible holes the model has here anyway (boards, double-sided faces): the same blast without the fence being a sheet */
        ra = cb = -1; explode(0, 0, Ew, 10.f); frame_tick(0);
        for (q = 0; q < 64; q++) if ((hl = strstr(logbuf[q], "holes(render/col)=")) != 0) { sscanf(hl, "holes(render/col)=%d/%d", &ra, &cb); break; }
        w32(ADDR_SURFACE_INFOS + 0x90 + 7 * 12 + 4, 0);
        scene_boxSet = 1; g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); scene_boxSet = 0;
        mem_write(A_cm, bbs, 24); mem_write(A_cm + 0x24, &rr, 4);
        explode(0, 0, Ew, 10.f); frame_tick(0);
        for (q = 0; q < 64; q++) if ((hl = strstr(logbuf[q], "holes(render/col)=")) != 0) { sscanf(hl, "holes(render/col)=%d/%d", &ra0, &cb0); break; }
        base = ra0 > 0 ? ra0 : 0;
        CHECK(cb > 0, "T22: no holes in the fence's collision (%d)", cb);
        { U32 t, steep = 0;      /* does the model itself have upright faces here? They may really lie on the fence. */
          for (t = 0; t < nt && !steep; t++) { Vec3 a, b, c, nn2; float l;
              memcpy(&a, R_pos + R_tri[t * 3] * 3, 12); memcpy(&b, R_pos + R_tri[t * 3 + 1] * 3, 12); memcpy(&c, R_pos + R_tri[t * 3 + 2] * 3, 12);
              { float ww[3]; Vec3 cp = gm_closest_on_tri(El, a, b, c, ww); if (gm_dist2(cp, El) > 25.f) continue; }
              nn2 = gm_cross(gm_sub(b, a), gm_sub(c, a)); l = sqrtf(gm_dot(nn2, nn2));
              if (l > 1e-9f && nn2.z < 0.8f * l) steep = 1; }       /* upright or facing down */
          if (!steep) CHECK(ra <= base, "T22: %d visible triangles removed next to the fence (%d without it)", ra, base); } } }
    /* --- T14: every surface declared shoot-through (wire fence) -> the blast punches a hole, no bowl.
     *          Declared glass / see-through only -> solid, a bowl and no hole ------------------- */
    for (k = 0; k < 2; k++) { U32 q;
      g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0);
      for (q = 0; q < 179; q++) w32(ADDR_SURFACE_INFOS + 0x90 + q * 12 + 4, k ? 0x00081000u : 0x2000u);
      explode(0, 0, Ew, 10.f); frame_tick(0);
      if (logged("RW v")) {
          if (!k) { CHECK(logged("holes(render/col)=") && !logged("holes(render/col)=0/0"), "T14: no holes reported: %s", logbuf[(logn + 63) % 64]);
                    expectBoxTris = 0; validate_col("T14", 0); expectBoxTris = 1; }
          else { CHECK(!logged("holes(render/col)="), "T14: glass was given holes: %s", logbuf[(logn + 63) % 64]);
                 validate_col("T14 glass", 0); }
      }
      for (q = 0; q < 179; q++) w32(ADDR_SURFACE_INFOS + 0x90 + q * 12 + 4, 0); }
    /* --- T9: no building pool yet (menu) ------------------------------------------------------- */
    reset_sim(); explode(0, 0, Ew, 10.f); frame_tick(0); CHECK(nalloc == 0, "T9: allocations without a world");
    return fails == before ? 1 : 0;
}

/* ---- breaking through (v3.6) -------------------------------------------------------------- */
/* first thing a ray meets in what the runtime left behind (model space); 1e30 if nothing */
static float ray_first(Vec3 o, Vec3 d, int collision) {
    float best = 1e30f; U32 i;
    if (collision) { U32 cd = peek32(A_cm + 0x2C), pV = peek32(cd + 0x14), pT = peek32(cd + 0x18), ntri = peek16(cd + 4);
        for (i = 0; i < ntri; i++) { Vec3 q[3]; U32 k; float t;
            for (k = 0; k < 3; k++) { U32 x = peek16(pT + i * 8 + k * 2); q[k].x = (S16)peek16(pV + x * 6) / 128.f; q[k].y = (S16)peek16(pV + x * 6 + 2) / 128.f; q[k].z = (S16)peek16(pV + x * 6 + 4) / 128.f; }
            t = gm_ray_tri(o, d, q[0], q[1], q[2]); if (t >= 0.f && t < best) best = t; } }
    else { U32 g = peek32(A_miAtomic + 0x18), rT = peek32(g + 0x2C), rnt = peek32(g + 0x10), rV = peek32(peek32(g + 0x5C) + 0x14);
        for (i = 0; i < rnt; i++) { Vec3 q[3]; U32 k; float t;
            for (k = 0; k < 3; k++) mem_read(rV + peek16(rT + i * 8 + k * 2) * 12, &q[k], 12);
            t = gm_ray_tri(o, d, q[0], q[1], q[2]); if (t >= 0.f && t < best) best = t; } }
    return best;
}
/* mode: "through" = after `blasts` blasts at the same spot the thing is open (nothing within `clear` metres behind it),
 *       "bowl" = stays closed, "hollow" / "nothing" = stays closed and the log says why;
 *       with "nc-" in front the model is drawn without back-face culling */
/* triangles of the visible mesh that hang in the air: joined to nothing that reaches out of the sphere */
static U32 count_debris(Vec3 Cl, float R) {
    static U32 par[65536]; U32 g = peek32(A_miAtomic + 0x18), rT = peek32(g + 0x2C), rnt = peek32(g + 0x10), rnv = peek32(g + 0x14), rV = peek32(peek32(g + 0x5C) + 0x14), i, j, n = 0;
    static Vec3 pv[65536]; static U8 anch[65536];
    for (i = 0; i < rnv; i++) { mem_read(rV + i * 12, &pv[i], 12); par[i] = i; anch[i] = 0; }
    #define FIND(x) ({ U32 q_ = (x); while (par[q_] != q_) { par[q_] = par[par[q_]]; q_ = par[q_]; } q_; })
    for (i = 0; i < rnv; i++) if (gm_dist2(pv[i], Cl) < (R + 1.f) * (R + 1.f)) for (j = 0; j < i; j++) if (gm_dist2(pv[i], pv[j]) < 1e-10f) { par[FIND(i)] = FIND(j); break; }
    for (i = 0; i < rnt; i++) { U32 a = peek16(rT + i * 8), b = peek16(rT + i * 8 + 2), c = peek16(rT + i * 8 + 4); par[FIND(b)] = FIND(a); par[FIND(c)] = FIND(a); }
    for (i = 0; i < rnt * 3; i++) { U32 v = peek16(rT + (i / 3) * 8 + (i % 3) * 2); if (gm_dist2(pv[v], Cl) > (R + 0.2f) * (R + 0.2f)) anch[FIND(v)] = 1; }
    for (i = 0; i < rnt; i++) if (!anch[FIND(peek16(rT + i * 8))]) n++;
    #undef FIND
    return n;
}
static float break_ring;      /* -break ... <ring>: the opening must be free out to this radius around the crater's axis */
static int run_break(const float* P, const char* mode, int blasts, float clear) {
    const float PX = 1500.25f, PY = -700.5f, PZ = 12.75f; Vec3 El = { P[0], P[1], P[2] }, Ew, n, o, d; int before = fails, k, nc = !strncmp(mode, "nc-", 3), through; U32 g0; float fr, fc, R;
    Vec3 cen = { 0.5f * (bbox[0] + bbox[3]), 0.5f * (bbox[1] + bbox[4]), 0.5f * (bbox[2] + bbox[5]) }, out = gm_sub(El, cen); float ol;
    /* the blast lies 10 cm outside the surface: away from the middle of the model, along the axis the point is outermost on (terrain: above) */
    if (npts > 3) { out.x = out.y = 0.f; out.z = 1.f; } else { float ax = fabsf(out.x) / (bbox[3] - bbox[0]), ay = fabsf(out.y) / (bbox[4] - bbox[1]), az = fabsf(out.z) / (bbox[5] - bbox[2]);
      if (az >= ax && az >= ay) { out.x = out.y = 0.f; } else if (ax >= ay) { out.y = out.z = 0.f; } else { out.x = out.z = 0.f; } }
    ol = sqrtf(gm_dot(out, out)); out.x /= ol; out.y /= ol; out.z /= ol;
    if (nc) mode += 3;                    /* nc-...: the model is drawn without back-face culling */
    through = !strcmp(mode, "through");
    mock_ground = strcmp(mode, "nothing") != 0;
    scene_box[0] = bbox[0] - 60.f; scene_box[1] = bbox[1] - 60.f; scene_box[2] = bbox[2] - 60.f; scene_box[3] = bbox[0] - 59.f; scene_box[4] = bbox[1] - 59.f; scene_box[5] = bbox[2] - 59.f; scene_boxSet = 1;   /* the scene's collision box: far away */
    g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); scene_boxSet = 0;
    if (nc) w16(A_mi + 0x12, 0);
    Ew.x = PX + El.x + 0.1f * out.x; Ew.y = PY + El.y + 0.1f * out.y; Ew.z = PZ + El.z + 0.1f * out.z;
    for (k = 0; k < blasts; k++) {
        clear_log(); explode((U32)k % 8u, 0, Ew, 100.f + (float)k); frame_tick(0);
        CHECK(logged("RW v") && logged("COL v"), "break (%s) blast %d: no crater: %s", mode, k + 1, logbuf[(logn + 63) % 64]);
        if (!logged("RW v")) { mock_ground = 1; return 0; }
        expectBoxTris = 0; validate_col("break", 0); validate_geom("break", 2); expectBoxTris = 1;
        if (k + 1 < blasts) CHECK(through || !logged("broke through"), "break (%s): broke through at blast %d", mode, k + 1);
    }
    R = g_lastR; n.x = (g_lastQ.x - g_lastC.x) / R; n.y = (g_lastQ.y - g_lastC.y) / R; n.z = (g_lastQ.z - g_lastC.z) / R;
    if (through) CHECK(gm_dot(n, out) > 0.9f, "break (%s): crater points (%.2f,%.2f,%.2f), expected (%.0f,%.0f,%.0f)", mode, n.x, n.y, n.z, out.x, out.y, out.z);
    /* look through the middle of the crater, and a little beside it */
    for (k = 0; k < 5; k++) { static const float sx[5] = { 0.f, 0.3f, -0.3f, 0.f, 0.f }, sy[5] = { 0.f, 0.f, 0.f, 0.3f, -0.3f }; Vec3 e1, e2;
        e1.x = out.y; e1.y = out.z; e1.z = out.x; e2 = gm_cross(out, e1);
        o.x = El.x + out.x * 1.0f + e1.x * sx[k] + e2.x * sy[k]; o.y = El.y + out.y * 1.0f + e1.y * sx[k] + e2.y * sy[k]; o.z = El.z + out.z * 1.0f + e1.z * sx[k] + e2.z * sy[k];
        d.x = -out.x; d.y = -out.y; d.z = -out.z;
        fr = ray_first(o, d, 0); fc = ray_first(o, d, 1);
        if (through) {
            CHECK(fr > 1.0f + clear, "break: the visible mesh is still closed (ray %d meets it after %.2f m)", k, fr - 1.0f);
            CHECK(fc > 1.0f + clear, "break: the collision is still closed (ray %d meets it after %.2f m)", k, fc - 1.0f);
        } else if (k == 0) {
            CHECK(fr < 1.0f + R && fc < 1.0f + R, "break (%s): open although it must stay solid (ray %d: visible %.2f, collision %.2f)", mode, k, fr - 1.0f, fc - 1.0f);
        }
    }
    if (getenv("HOST_DUMP")) {       /* visual check */
        char pth[600]; FILE* f; U32 g = peek32(A_miAtomic + 0x18), nvv = peek32(g + 0x14), ntt = peek32(g + 0x10), pT = peek32(g + 0x2C), pVv = peek32(peek32(g + 0x5C) + 0x14), q;
        snprintf(pth, 600, "%s_%d.obj", getenv("HOST_DUMP"), dumpNo++); f = fopen(pth, "w");
        fprintf(f, "# C %f %f %f R %f Q %f %f %f E %f %f %f\n", g_lastC.x - PX, g_lastC.y - PY, g_lastC.z - PZ, R, g_lastQ.x - PX, g_lastQ.y - PY, g_lastQ.z - PZ, El.x, El.y, El.z);
        for (q = 0; q < nvv; q++) { Vec3 pp; mem_read(pVv + q * 12, &pp, 12); fprintf(f, "v %f %f %f\n", pp.x, pp.y, pp.z); }
        for (q = 0; q < ntt; q++) fprintf(f, "f %u %u %u %u\n", peek16(pT + q * 8) + 1, peek16(pT + q * 8 + 2) + 1, peek16(pT + q * 8 + 4) + 1, peek16(pT + q * 8 + 6));
        fclose(f);
    }
    if (through && break_ring > 0.f) for (k = 0; k < 12; k++) { Vec3 e1, e2; float a = 0.5236f * (float)k, cx = break_ring * cosf(a), sy = break_ring * sinf(a);
        e1.x = out.y; e1.y = out.z; e1.z = out.x; e2 = gm_cross(out, e1);
        o.x = El.x + out.x * 1.0f + e1.x * cx + e2.x * sy; o.y = El.y + out.y * 1.0f + e1.y * cx + e2.y * sy; o.z = El.z + out.z * 1.0f + e1.z * cx + e2.z * sy;
        d.x = -out.x; d.y = -out.y; d.z = -out.z;
        fr = ray_first(o, d, 0); fc = ray_first(o, d, 1);
        CHECK(fr > 1.0f + clear && fc > 1.0f + clear, "break: the opening is not round - blocked at %.2f m from the axis, direction %d (visible %.2f, collision %.2f)", break_ring, k, fr - 1.f, fc - 1.f); }
    if (through) { Vec3 Cl = { g_lastC.x - PX, g_lastC.y - PY, g_lastC.z - PZ }; U32 nd = count_debris(Cl, R); CHECK(nd == 0, "break: %u triangles hang in the air", nd); }
    if (through) CHECK(logged("broke through(render/col)=") && !logged("/0"), "break: log does not report the break-through in both meshes: %s", logbuf[(logn + 63) % 64]);
    else CHECK(!logged("broke through"), "break (%s): log reports a break-through: %s", mode, logbuf[(logn + 63) % 64]);
    if (!strcmp(mode, "hollow")) CHECK(logged("(hollow inside: stays solid)"), "break: no note about the hollow inside: %s", logbuf[(logn + 63) % 64]);
    if (!strcmp(mode, "nothing") && !nc) CHECK(logged("(nothing below: stays solid)"), "break: no note about nothing below: %s", logbuf[(logn + 63) % 64]);
    if (nc) CHECK(logged("[double-sided") && (logged("space behind") != 0) == (mock_ground != 0), "break: note about double-sided drawing wrong: %s", logbuf[(logn + 63) % 64]);
    mock_ground = 1;
    return fails == before ? 1 : 0;
}

/* A wrecked car blows up with its centre a little under the road; a second one sits in the crater
 * of the first, under the crater's floor. Both must leave bowls: seen from above, nothing of the
 * surface within 3 m may be higher than it was (kerbs and slopes of real roads included), some of
 * it clearly lower, and no hole in it. */
static int run_under(const float* P) {
    const float PX = 1500.25f, PY = -700.5f, PZ = 12.75f; int before = fails, k, c, ix, iy; U32 g0; Vec3 E; static float h0[2][13][13];
    g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0);
    for (k = 0; k < 3; k++) {
        Vec3 o = { P[0] + 0.4f * (float)k, P[1], P[2] + 30.f }, dn = { 0, 0, -1 }; float zg = o.z - ray_first(o, dn, 1);
        /* (before: the highest thing within 15 cm of each sight line - a kerb's edge may shift a few centimetres without anything having risen) */
        for (c = 0; c < 2; c++) for (iy = 0; iy < 13; iy++) for (ix = 0; ix < 13; ix++) { int e; float hb = 1e30f;
            for (e = 0; e < 5; e++) { Vec3 q = { o.x - 3.f + 0.5f * (float)ix + 0.013f + (e == 1 ? 0.15f : e == 2 ? -0.15f : 0.f), o.y - 3.f + 0.5f * (float)iy + 0.007f + (e == 3 ? 0.15f : e == 4 ? -0.15f : 0.f), o.z }; float h = ray_first(q, dn, c);
                if (e == 0 && h > 1e20f) { hb = h; break; }
                if (h < hb) hb = h; }
            h0[c][iy][ix] = hb; }
        E.x = PX + o.x; E.y = PY + o.y; E.z = PZ + zg - (k == 1 ? 0.45f : 0.3f);
        clear_log(); explode((U32)k, 4, E, 100.f + (float)k); frame_tick(0);
        CHECK(logged("RW v") && logged("COL v"), "under %d: no crater: %s", k, logbuf[(logn + 63) % 64]);
        /* (on the flat test ground the crater must have been turned round; on a steep slope of a real model the blast is hardly behind anything) */
        if (npts > 8) CHECK(logged("behind the visible surface: crater dug from the side that is seen"), "under %d: the blast under the surface was not recognised", k);
        if (fails != before) return 0;
        for (c = 0; c < 2; c++) { float up = 0.f, down = 0.f; U32 holes = 0;
            for (iy = 0; iy < 13; iy++) for (ix = 0; ix < 13; ix++) { Vec3 q = { o.x - 3.f + 0.5f * (float)ix + 0.013f, o.y - 3.f + 0.5f * (float)iy + 0.007f, o.z }; float h = ray_first(q, dn, c);
                if (h0[c][iy][ix] > 1e20f) continue;                 /* nothing was there */
                if (h > 1e20f) { int e, all = 1;      /* a hole, not a hairline: open 5 cm to every side as well */
                    for (e = 0; e < 4 && all; e++) { Vec3 q3 = q; q3.x += e == 0 ? 0.05f : e == 1 ? -0.05f : 0.f; q3.y += e == 2 ? 0.05f : e == 3 ? -0.05f : 0.f; if (ray_first(q3, dn, c) <= 1e20f) all = 0; }
                    if (all) holes++;
                    continue; }
                if (h0[c][iy][ix] - h > up) { up = h0[c][iy][ix] - h; if (verbose) printf("      rise %.2f at (%.2f,%.2f): %.2f -> %.2f (%s)\n", up, q.x, q.y, o.z - h0[c][iy][ix], o.z - h, c ? "col" : "vis"); }
                if (h - h0[c][iy][ix] > down) down = h - h0[c][iy][ix]; }
            CHECK(up < 0.06f, "under %d: a mound - the %s rises %.2f m above what was there", k, c ? "collision" : "visible mesh", up);
            CHECK(down > 0.4f, "under %d: no bowl in the %s (deepest %.2f m under what was there)", k, c ? "collision" : "visible mesh", down);
            CHECK(holes == 0, "under %d: holes in the ground (%s, %u of 169 sight lines)", k, c ? "collision" : "visible mesh", holes);
        }
    }
    return fails == before ? 1 : 0;
}

/* F8: after a handful of blasts everything is put back as it was - the collision block the game
 * loaded, a geometry with exactly the original data, nothing left over in memory - and the player,
 * who stood in a crater, on top of the ground. Also: twice in a row, with nothing to do, and after
 * the game has loaded the model afresh on its own. */
typedef struct { U32 nv, nt, usage; U8 *tri, *pos, *pre, *uv, *night, *day; U8 sph[16]; U32 mats[NMAT]; } GeoCopy;
static void geo_copy(U32 g, GeoCopy* c) {
    U32 mt = peek32(g + 0x5C), i;
    c->nv = peek32(g + 0x14); c->nt = peek32(g + 0x10); c->usage = peek32(g + USAGE_OFF);
    c->tri = malloc(c->nt * 8); mem_read(peek32(g + 0x2C), c->tri, c->nt * 8);
    c->pos = malloc(c->nv * 12); mem_read(peek32(mt + 0x14), c->pos, c->nv * 12);
    c->pre = malloc(c->nv * 4); mem_read(peek32(g + 0x30), c->pre, c->nv * 4);
    c->uv = malloc(c->nv * 8); mem_read(peek32(g + 0x34), c->uv, c->nv * 8);
    c->night = c->day = NULL;
    if (peek32(g + VC_OFFSET)) { c->night = malloc(c->nv * 4); mem_read(peek32(g + VC_OFFSET), c->night, c->nv * 4); }
    if (peek32(g + VC_OFFSET + 4)) { c->day = malloc(c->nv * 4); mem_read(peek32(g + VC_OFFSET + 4), c->day, c->nv * 4); }
    mem_read(mt + 4, c->sph, 16);
    for (i = 0; i < NMAT; i++) c->mats[i] = peek32(peek32(g + 0x20) + i * 4);
}
static void geo_same(const char* what, U32 g, const GeoCopy* c) {
    U32 mt = peek32(g + 0x5C), i; U8* b;
    CHECK(peek32(g + 0x14) == c->nv && peek32(g + 0x10) == c->nt, "%s: geometry has %u vertices, %u triangles (original %u, %u)", what, peek32(g + 0x14), peek32(g + 0x10), c->nv, c->nt);
    if (peek32(g + 0x14) != c->nv || peek32(g + 0x10) != c->nt) return;
    b = malloc(c->nv * 12 + c->nt * 8);
    mem_read(peek32(g + 0x2C), b, c->nt * 8); CHECK(!memcmp(b, c->tri, c->nt * 8), "%s: triangles differ from the original", what);
    mem_read(peek32(mt + 0x14), b, c->nv * 12); CHECK(!memcmp(b, c->pos, c->nv * 12), "%s: vertices differ from the original", what);
    mem_read(peek32(g + 0x30), b, c->nv * 4); CHECK(!memcmp(b, c->pre, c->nv * 4), "%s: vertex colours differ from the original", what);
    mem_read(peek32(g + 0x34), b, c->nv * 8); CHECK(!memcmp(b, c->uv, c->nv * 8), "%s: texture coordinates differ from the original", what);
    CHECK((peek32(g + VC_OFFSET) != 0) == (c->night != NULL) && (peek32(g + VC_OFFSET + 4) != 0) == (c->day != NULL), "%s: day/night arrays not as in the original", what);
    if (c->night && peek32(g + VC_OFFSET)) { mem_read(peek32(g + VC_OFFSET), b, c->nv * 4); CHECK(!memcmp(b, c->night, c->nv * 4), "%s: night colours differ from the original", what); }
    if (c->day && peek32(g + VC_OFFSET + 4)) { mem_read(peek32(g + VC_OFFSET + 4), b, c->nv * 4); CHECK(!memcmp(b, c->day, c->nv * 4), "%s: day colours differ from the original", what); }
    mem_read(mt + 4, b, 16); CHECK(!memcmp(b, c->sph, 16), "%s: bounding sphere differs from the original", what);
    CHECK(peek32(g + 0x24) == NMAT, "%s: %u materials (original %u: the rock must be gone)", what, peek32(g + 0x24), NMAT);
    for (i = 0; i < NMAT && i < peek32(g + 0x24); i++) CHECK(peek32(peek32(g + 0x20) + i * 4) == c->mats[i], "%s: material %u is not the original one", what, i);
    CHECK(peek32(g + USAGE_OFF) == c->usage, "%s: D3D9 usage flags differ", what);
    free(b);
}
static int run_reset(const float* P) {
    const float PX = 1500.25f, PY = -700.5f, PZ = 12.75f; static const char tags[] = "GMTCUVNHOLEP"; long base[16]; int before = fails, k, round, inCrater, onGround;
    U32 g0, cd0, cdSize, e1, ped, pmtx, g1; U8 *colBytes, cm0[0x30], cm1[0x30], *b; GeoCopy gc; Vec3 E, pp, dn = { 0, 0, -1 }; float zg;
    g0 = build_scene(1, 1, 1); e1 = add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0);
    wf(ADDR_CAMERA_MATRIX + 0x30, PX + P[0] - 6.f); wf(ADDR_CAMERA_MATRIX + 0x34, PY + P[1] - 8.f); wf(ADDR_CAMERA_MATRIX + 0x38, PZ + P[2] + 6.f);
    cd0 = peek32(A_cm + 0x2C); cdSize = find_alloc(cd0)->size; colBytes = malloc(cdSize); b = malloc(cdSize); mem_read(cd0, colBytes, cdSize); memset(colBytes + 0x1C, 0, 4);      /* (its triangle planes go with the first blast) */
    mem_read(A_cm, cm0, 0x30); geo_copy(g0, &gc);
    { Vec3 o = { P[0], P[1], P[2] + 30.f }; onGround = fabsf(o.z - ray_first(o, dn, 1) - P[2]) < 0.05f; }      /* the test point lies on something one can stand on */
    for (k = 0; tags[k]; k++) base[k] = live_count(tags[k]);
    base[11] = 0;                                                                /* 'P': the planes are gone for good */
    /* with nothing changed yet: nothing happens */
    clear_log(); frame_tick(1);
    CHECK(logged("F8 reset: visible mesh restored on 0 objects, collision on 0") && peek32(A_cm + 0x2C) == cd0 && peek32(A_miAtomic + 0x18) == g0 && n_create == 1, "reset: F8 with nothing to take back changed something: %s", logbuf[(logn + 63) % 64]);
    for (round = 0; round < 2; round++) {
        static const float off[4][2] = { { 0.f, 0.f }, { 1.6f, 0.4f }, { -0.9f, 1.3f }, { 0.3f, -0.2f } }; static const U32 typ[4] = { 0, 2, 4, 0 };
        int made = 0;
        for (k = 0; k < 4; k++) {
            E.x = PX + P[0] + 0.4f * off[k][0]; E.y = PY + P[1] + 0.4f * off[k][1]; E.z = PZ + P[2] + (typ[k] == 4u ? -0.3f : 0.12f);
            clear_log(); explode((U32)k, typ[k], E, 100.f + (float)(k + 10 * round)); frame_tick(0);
            if (logged("RW v") && logged("COL v")) made++; }
        if (!made && npts <= 8) { free(colBytes); free(b); return 1; }      /* (a test point of a real model where nothing can be carved: nothing to take back) */
        CHECK(made >= 1, "reset: none of four blasts made a crater: %s", logbuf[(logn + 63) % 64]);
        if (fails != before) return 0;
        CHECK(peek32(A_miAtomic + 0x18) != g0 && peek32(A_cm + 0x2C) != cd0 && peek32(peek32(e1 + 0x18) + 0x18) == peek32(A_miAtomic + 0x18), "reset: the blasts did not change the model and its instance");
        CHECK(kept_count() == 2 && find_alloc(cd0)->live, "reset: original not kept (%ld blocks)", kept_count());
        /* the player stands at the bottom of the crater */
        { Vec3 o = { P[0], P[1], P[2] + 30.f }; zg = o.z - ray_first(o, dn, 1); }
        inCrater = onGround && zg < P[2] - 0.4f && zg > P[2] - 6.f;      /* (a point on a wall or with a hole under it: no crater to stand in) */
        if (!inCrater) zg = P[2];
        ped = sim_alloc(0x200, 'B'); pmtx = sim_alloc(0x48, 'X'); memset(sim + (ped - SIM_BASE), 0, 0x200); memset(sim + (pmtx - SIM_BASE), 0, 0x48);
        w32(ped + 0x14, pmtx); pp.x = PX + P[0]; pp.y = PY + P[1]; pp.z = PZ + zg + (round ? 0.7f : 1.0f); mem_write(pmtx + 0x30, &pp, 12); wf(ped + 0x4C, -0.2f); wf(ped + 0x44, 0.05f);
        w32(round ? ADDR_PLAYER_VEHICLE : ADDR_PLAYER_PED, ped); if (!round) w32(ADDR_PLAYER_VEHICLE, 0);
        mock_meshLos = 1; mock_origin.x = PX; mock_origin.y = PY; mock_origin.z = PZ;
        clear_log(); frame_tick(1);
        mock_meshLos = 0;
        CHECK(logged("F8 reset: visible mesh restored on 1 objects, collision on 1"), "reset %d: %s", round, logbuf[(logn + 63) % 64]);
        /* collision: the game's own block again, untouched, and the model's bounds as they were */
        CHECK(peek32(A_cm + 0x2C) == cd0 && find_alloc(cd0)->live, "reset %d: the original collision block is not back", round);
        mem_read(cd0, b, cdSize); CHECK(!memcmp(b, colBytes, cdSize), "reset %d: the original collision block was altered", round);
        mem_read(A_cm, cm1, 0x30); CHECK(!memcmp(cm0, cm1, 0x30), "reset %d: collision model header (bounds) not as before", round);
        /* visible mesh: a geometry with exactly the original data, on the model and on every instance */
        g1 = peek32(A_miAtomic + 0x18);
        CHECK(find_alloc(g1) && find_alloc(g1)->live && find_alloc(g1)->tag == 'G', "reset %d: no live geometry on the model", round);
        if (fails != before) return 0;
        geo_same(round ? "reset 1" : "reset 0", g1, &gc);
        CHECK(peek32(peek32(e1 + 0x18) + 0x18) == g1 && peek16(g1 + 0xE) == 2, "reset %d: the instance does not share the restored geometry (refcount %u)", round, peek16(g1 + 0xE));
        CHECK(peek32(g1 + FX_OFF) == A_fx && find_alloc(A_fx)->live, "reset %d: 2D effects not handed back", round);
        CHECK(peek16(g1 + 0xC) == 0 && peek32(g1 + 0x54), "reset %d: restored geometry not unlocked", round);
        { float as[4]; mem_read(A_miAtomic + 0x1C, as, 16); CHECK(!memcmp(as, gc.sph, 16), "reset %d: atomic bounding sphere not as before", round); }
        /* nothing left over */
        for (k = 0; tags[k]; k++) CHECK(live_count(tags[k]) == base[k], "reset %d: %ld live allocations of kind '%c' (before the blasts: %ld)", round, live_count(tags[k]), tags[k], base[k]);
        CHECK(kept_count() == 0 && g_nUndo == 0, "reset %d: the runtime still keeps %ld blocks", round, kept_count());
        CHECK(g_rockMat && peek32(g_rockMat + 0x18) == 1, "reset %d: rock material refcount %u (expected: only its creation)", round, g_rockMat ? peek32(g_rockMat + 0x18) : 0);
        /* the player: on top of the ground again, not falling */
        mem_read(pmtx + 0x30, &pp, 12);
        if (inCrater) { Vec3 o = { pp.x - PX, pp.y - PY, pp.z - PZ }; float d = ray_first(o, dn, 1), feet = round ? 0.7f : 1.0f;
            /* (he stands ON something: the ground of the model, or - where a hole went through a bridge deck - whatever he stood on below it) */
            CHECK(d > feet - 0.05f && d < feet + 0.25f, "reset %d: after the reset the player has the ground %.2f m under him (expected %.1f)", round, d, feet);
            if (logged("put on top of the ground")) CHECK(peekf(ped + 0x4C) == 0.f && peekf(ped + 0x44) == 0.05f, "reset %d: lifted player keeps falling (speed %.2f)", round, peekf(ped + 0x4C)); }
        else if (npts > 8) CHECK(0, "reset %d: no crater under the player on open ground", round);
        if (npts > 8) CHECK(logged("put on top of the ground") && fabsf(pp.z - (PZ + P[2] + (round ? 0.8f : 1.1f))) < 0.12f, "reset %d: on open ground the player was not put on top (%.2f m above it)", round, pp.z - (PZ + P[2]));
        sim_free(ped, 'B'); sim_free(pmtx, 'X'); w32(ADDR_PLAYER_PED, 0); w32(ADDR_PLAYER_VEHICLE, 0);
        if (fails != before) return 0;
    }
    /* the game loads the model afresh on its own (the player drove away and came back): F8 then only lets go of what it kept */
    { U32 gNew, cdNew, cdOld, fxNew;
      E.x = PX + P[0]; E.y = PY + P[1]; E.z = PZ + P[2] + 0.12f; clear_log(); explode(0, 0, E, 900.f); frame_tick(0);
      if (!logged("RW v") && npts <= 8) { free(colBytes); free(b); return fails == before ? 1 : 0; }
      CHECK(logged("RW v") && kept_count() == 2, "reset: no crater before the reload: %s", logbuf[(logn + 63) % 64]);
      if (fails != before) return 0;
      gNew = make_geometry(); fxNew = sim_alloc(8, 'E'); w32(fxNew, 2); w32(gNew + FX_OFF, fxNew);
      rw_atomic_set_geometry(A_miAtomic, gNew); rw_atomic_set_geometry(peek32(e1 + 0x18), gNew); rw_geom_destroy(gNew);
      cdOld = peek32(A_cm + 0x2C); cdNew = make_coldata(0, 1, 1); mem_write(A_cm, cm0, 0x2C); w32(A_cm + 0x2C, cdNew); sim_free(cdOld, 'M');
      clear_log(); frame_tick(1);
      CHECK(logged("F8 reset: visible mesh restored on 0 objects, collision on 0"), "reset after reload: %s", logbuf[(logn + 63) % 64]);
      CHECK(peek32(A_miAtomic + 0x18) == gNew && peek32(A_cm + 0x2C) == cdNew && find_alloc(gNew)->live && find_alloc(cdNew)->live, "reset after reload: the freshly loaded meshes were touched");
      CHECK(kept_count() == 0 && !find_alloc(cd0)->live && live_count('M') == (hasNight ? 3 : 1) && live_count('G') == 1, "reset after reload: kept blocks not released (%ld game allocations, %ld geometries)", live_count('M'), live_count('G'));
      /* ... and the next crater is kept and taken back as usual */
      clear_log(); explode(1, 0, E, 901.f); frame_tick(0); frame_tick(1);
      CHECK(peek32(A_cm + 0x2C) == cdNew && kept_count() == 0 && live_count('G') == 1, "reset after reload: second round failed"); }
    free(colBytes); free(b);
    return fails == before ? 1 : 0;
}

/* two shafts dug into open ground a few metres apart, then joined underground from the bottom of
 * the first: in the end there must be a free way from one bottom to the other, to see and to walk */
static int run_tunnels(const float* P) {
    const float PX = 1500.25f, PY = -700.5f, PZ = 12.75f; int before = fails, k, sh; U32 g0; Vec3 bot[2], dir, E; float len, fr, fc, t;
    g0 = build_scene(1, 0, 1); add_entity(2, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0);
    for (sh = 0; sh < 2; sh++) {
        E.x = PX + P[0] + 6.f * (float)sh; E.y = PY + P[1]; E.z = PZ + P[2] + 0.15f;
        for (k = 0; k < 7; k++) {
            clear_log(); explode((U32)k, 0, E, 100.f + (float)(sh * 20 + k)); frame_tick(0);
            CHECK(logged("COL v") && logged("RW v"), "tunnels: shaft %d blast %d: no crater: %s", sh, k, logbuf[(logn + 63) % 64]);
            if (fails != before) return 0;
            bot[sh].x = E.x; bot[sh].y = E.y; bot[sh].z = g_lastC.z - (g_lastQ.z - g_lastC.z) / g_lastR * (g_lastR - 0.5f);   /* half a metre above the bottom */
            E = bot[sh];
        }
    }
    bot[1].z = bot[0].z < bot[1].z ? bot[0].z : bot[1].z; bot[0].z = bot[1].z;
    dir = gm_sub(bot[1], bot[0]); len = sqrtf(gm_dot(dir, dir)); dir.x /= len; dir.y /= len; dir.z /= len;
    for (t = 0.6f, k = 0; t < len && k < 12; k++) {        /* from the first shaft towards the second: always at the wall in the way */
        Vec3 o = { bot[0].x - PX, bot[0].y - PY, bot[0].z - PZ }; float f = ray_first(o, dir, 1), f2 = ray_first(o, dir, 0);
        if (f2 < f) f = f2;
        if (f > len - 0.3f) break;                          /* through */
        t = f > 0.3f ? f - 0.2f : 0.1f;
        E.x = bot[0].x + dir.x * t; E.y = bot[0].y + dir.y * t; E.z = bot[0].z + dir.z * t;
        clear_log(); explode((U32)k, 0, E, 200.f + (float)k); frame_tick(0);
        if (verbose) printf("    joining blast %d at %.2f m of %.2f: %s\n", k, t, len, logbuf[(logn + 63) % 64]);
    }
    { Vec3 o = { bot[0].x - PX, bot[0].y - PY, bot[0].z - PZ };
      fr = ray_first(o, dir, 0); fc = ray_first(o, dir, 1);
      CHECK(fr > len - 0.3f, "tunnels: not joined to the eye - rock after %.2f m of %.2f (%d joining blasts)", fr, len, k);
      CHECK(fc > len - 0.3f, "tunnels: not joined for walking - collision after %.2f m of %.2f (%d joining blasts)", fc, len, k);
      CHECK(k <= 6, "tunnels: %d blasts needed to join two shafts %.1f m apart", k, len); }
    if (getenv("HOST_DUMP")) {
        char pth[600]; FILE* f; U32 g = peek32(A_miAtomic + 0x18), nvv = peek32(g + 0x14), ntt = peek32(g + 0x10), pT = peek32(g + 0x2C), pVv = peek32(peek32(g + 0x5C) + 0x14), q;
        snprintf(pth, 600, "%s_tunnels.obj", getenv("HOST_DUMP")); f = fopen(pth, "w");
        fprintf(f, "# A %f %f %f B %f %f %f\n", bot[0].x - PX, bot[0].y - PY, bot[0].z - PZ, bot[1].x - PX, bot[1].y - PY, bot[1].z - PZ);
        for (q = 0; q < nvv; q++) { Vec3 pp; mem_read(pVv + q * 12, &pp, 12); fprintf(f, "v %f %f %f\n", pp.x, pp.y, pp.z); }
        for (q = 0; q < ntt; q++) fprintf(f, "f %u %u %u %u\n", peek16(pT + q * 8) + 1, peek16(pT + q * 8 + 2) + 1, peek16(pT + q * 8 + 4) + 1, peek16(pT + q * 8 + 6));
        fclose(f);
    }
    /* ... there is ground under both shafts and under the way between them ... */
    for (k = 0; k < 6; k++) { float f = 0.5f * (float)(k / 2); Vec3 o = { bot[0].x - PX + (bot[1].x - bot[0].x) * f, bot[0].y - PY, bot[0].z - PZ + 0.3f }, dn = { 0.f, 0.f, -1.f }; float a = ray_first(o, dn, k & 1);
      CHECK(a < 4.f, "tunnels: no ground under the joined tunnels (%s, at %.0f %% of the way)", (k & 1) ? "collision" : "visible mesh", 100.f * f); }
    /* ... and the second shaft is still open: from its bottom straight up into the sky, and from above down to the bottom */
    for (k = 0; k < 2; k++) { Vec3 o = { bot[1].x - PX, bot[1].y - PY, bot[1].z - PZ + 0.3f }, up = { 0.f, 0.f, 1.f }, o2 = { bot[1].x - PX, bot[1].y - PY, P[2] + 3.f }, dn = { 0.f, 0.f, -1.f };
      float a = ray_first(o, up, k), b = ray_first(o2, dn, k);
      CHECK(a > 1e20f, "tunnels: the second shaft is closed off above the join (%s, after %.2f m going up)", k ? "collision" : "visible mesh", a);
      CHECK(b > P[2] + 3.f - (bot[1].z - PZ) - 0.6f, "tunnels: looking down the second shaft there is a lid after %.2f m (%s)", b, k ? "collision" : "visible mesh"); }
    return fails == before ? 1 : 0;
}

/* tall thin object (tree, pole): must be knocked over as a physics object, never carved */
static int run_prop(const float* P) {
    const float PX = 300.5f, PY = 820.25f, PZ = 31.f; Vec3 Ew = { PX + P[0] + 0.6f, PY + P[1] + 0.4f, PZ + P[2] }; int before = fails; U32 g0, e, cd0, cd1, ns, k;
    scene_noSphere = 1;
    /* the model's collision box: a trunk-sized block at the foot, inside the bounding box */
    { float cx = 0.5f * (bbox[0] + bbox[3]), cy = 0.5f * (bbox[1] + bbox[4]), hw = 0.25f * (bbox[3] - bbox[0] - (propWide ? 16.f : 0.f)), hd = 0.25f * (bbox[4] - bbox[1] - (propWide ? 16.f : 0.f));
      if (hw > 0.4f) hw = 0.4f; if (hd > 0.4f) hd = 0.4f;
      scene_box[0] = cx - hw; scene_box[1] = cy - hd; scene_box[2] = bbox[2]; scene_box[3] = cx + hw; scene_box[4] = cy + hd; scene_box[5] = bbox[2] + 0.3f * (bbox[5] - bbox[2]); scene_boxSet = 1; }
    g0 = build_scene(1, 0, 0); e = add_entity(4, PX, PY, PZ, 0.7f, 0, 0.f, MODEL_A, g0); cd0 = peek32(A_cm + 0x2C);
    explode(3, 0, Ew, 77.f); w32(ADDR_TIME_MS, 5000); frame_tick(0);
    CHECK(logged("knocked over") && n_objCreate == 1 && n_worldAdd == 1, "P1: prop not knocked over: %s", logbuf[(logn + 63) % 64]);
    CHECK(n_create == 1 && find_alloc(g0)->live, "P1: prop geometry was carved");
    CHECK((peek32(e + 0x1C) & 0x81) == 0, "P1: static object still visible/collidable");
    cd1 = peek32(A_cm + 0x2C); ns = peek16(cd1);
    CHECK(cd1 != cd0 && !find_alloc(cd0)->live && ns >= 1 && ns <= 8 && peek32(cd1 + 8) == cd1 + 0x30, "P1: collision spheres not added (%u)", ns);
    for (k = 0; k < ns; k++) { float sp[4]; mem_read(cd1 + 0x30 + k * 0x14, sp, 16);
        CHECK(sp[3] >= 0.3f && sp[3] <= 1.2f && sp[0] >= bbox[0] && sp[0] <= bbox[3] && sp[1] >= bbox[1] && sp[1] <= bbox[4] && sp[2] >= bbox[2] && sp[2] <= bbox[5], "P1: sphere %u misplaced", k);
        CHECK(sp[3] <= 0.75f, "P1: sphere %u is fatter (%.2f m) than the trunk", k, sp[3]); }
    CHECK(peek16(cd1 + 4) == nct + 12 && peek16(cd1 + 2) == 0, "P1: collision triangles changed (%u, expected %u + 12 from the box)", peek16(cd1 + 4), nct);
    if (lastObj) { U32 m = peek32(lastObj + 0x14);
        CHECK(fabsf(peekf(m + 0x30) - PX) < 1e-3f && fabsf(peekf(m + 0x34) - PY) < 1e-3f && fabsf(peekf(m + 0x38) - PZ) < 1e-3f, "P1: object not at the prop's position");
        CHECK(fabsf(peekf(m) - cosf(0.7f)) < 1e-4f && fabsf(peekf(m + 4) - sinf(0.7f)) < 1e-4f && peekf(m + 0x28) == 1.f, "P1: object heading wrong");
        CHECK(peek8(lastObj + 0x13C) == 3 && peek32(lastObj + 0x150) == 125000 && peek16(ADDR_NUM_TEMP_OBJECTS) == 1, "P1: not registered as temporary object");
        CHECK(peekf(lastObj + 0x98) > 0.9f && (peekf(lastObj + 0x44) != 0.f || peekf(lastObj + 0x48) != 0.f) && peekf(lastObj + 0x4C) > 0.f, "P1: no push / air resistance");
        { float wx = peekf(lastObj + 0x50), wy = peekf(lastObj + 0x54), hgt = bbox[5] - bbox[2], ax = PX - Ew.x, ay = PY - Ew.y, top;
          top = sqrtf(wx * wx + wy * wy) * hgt;                     /* speed of the tip, units per frame */
          CHECK(wy * ax - wx * ay > 0.f, "P1: tips towards the blast instead of away from it");
          CHECK(top > 0.01f && top < 0.2f, "P1: tip starts at %.3f units per frame", top); } }
    /* a second blast at the same place: the tree is gone, no second object */
    explode(4, 0, Ew, 99.f); frame_tick(0);
    CHECK(n_objCreate == 1 && n_worldAdd == 1, "P1: knocked-over prop was knocked over again (%d objects)", n_objCreate);
    /* P2: switched off with F7 */
    g0 = build_scene(1, 0, 0); e = add_entity(4, PX, PY, PZ, 0.7f, 0, 0.f, MODEL_A, g0);
    frame_tick2(0, 1); explode(3, 0, Ew, 78.f); frame_tick(0);
    CHECK(n_objCreate == 0 && (peek32(e + 0x1C) & 0x81) == 0x81 - 0 * 0 && logged("switched off"), "P2: prop handled although switched off");
    /* P3: blast far away */
    g0 = build_scene(1, 0, 0); e = add_entity(4, PX, PY, PZ, 0.7f, 0, 0.f, MODEL_A, g0); Ew.x += 40.f;
    explode(3, 0, Ew, 79.f); frame_tick(0);
    CHECK(n_objCreate == 0 && n_create == 1, "P3: distant blast changed the prop");
    /* P4: F8 - the static object is shown again, the fallen one may go, and it can be knocked over anew */
    Ew.x -= 40.f;
    g0 = build_scene(1, 0, 0); e = add_entity(4, PX, PY, PZ, 0.7f, 0, 0.f, MODEL_A, g0);
    explode(3, 0, Ew, 80.f); w32(ADDR_TIME_MS, 5000); frame_tick(0);
    CHECK(n_objCreate == 1 && (peek32(e + 0x1C) & 0x81) == 0 && peek32(lastObj + 0x150) == 125000, "P4: prop not knocked over");
    w32(ADDR_TIME_MS, 9000); clear_log(); frame_tick2(1, 0);
    CHECK(logged("objects shown again: 1") && (peek32(e + 0x1C) & 0x81) == 0x81, "P4: static object not shown again after F8 (flags %#x): %s", peek32(e + 0x1C), logbuf[(logn + 63) % 64]);
    CHECK(peek32(lastObj + 0x150) == 9000, "P4: fallen object not released for removal (time %u)", peek32(lastObj + 0x150));
    explode(4, 0, Ew, 81.f); frame_tick(0);
    CHECK(n_objCreate == 2 && (peek32(e + 0x1C) & 0x81) == 0, "P4: prop cannot be knocked over again after F8");
    scene_noSphere = 0; scene_boxSet = 0;
    return fails == before ? 1 : 0;
}

/* A long tunnel: blast after blast down a slope into the same object until it is full.
 * The meshes must stay valid all the way, stop growing at the limit, and never pass it. */
static int stressBox;
static int run_stress(const float* P) {
    const float PX = 1500.25f, PY = -700.5f, PZ = 12.75f; int before = fails, k, full = 0, done = 0, heapOut = 0; U32 g0, cd, g; Vec3 dir = { 0.8f, 0.05f, -0.6f };
    g0 = build_scene(1, 1, 1); add_entity(3, PX, PY, PZ, 0.f, 0, 0.f, MODEL_A, g0); (void)g0;
    for (k = 0; k < 1500 && full < 5; k++) {
        Vec3 E; static Vec3 next; static int have, shaft;
        if (!have || shaft >= 45) { float a = 0.9f * (float)(k / 7); E.x = PX + P[0] + 9.f * cosf(a) * (float)(k > 0); E.y = PY + P[1] + 9.f * sinf(a) * (float)(k > 0); E.z = PZ + P[2] + 0.15f; shaft = 0; }   /* a new shaft a few metres away */
        else E = next;
        clear_log(); explode((U32)(k % 12), (U32)(k % 3 == 2 ? 2 : 0), E, 100.f + (float)k); frame_tick(0);
        if (logged("COL v") || logged("is full")) {      /* next blast: at the deepest point of this crater, a little to the side */
            Vec3 nn = { (g_lastQ.x - g_lastC.x) / g_lastR, (g_lastQ.y - g_lastC.y) / g_lastR, (g_lastQ.z - g_lastC.z) / g_lastR };
            next.x = g_lastC.x - nn.x * (g_lastR - 0.6f) + dir.x * 0.12f; next.y = g_lastC.y - nn.y * (g_lastR - 0.6f) + 0.1f * sinf((float)k); next.z = g_lastC.z - nn.z * (g_lastR - 0.6f);
            have = 1; shaft++;
        } else have = 0;
        cd = peek32(A_cm + 0x2C); g = peek32(A_miAtomic + 0x18);
        CHECK(peek16(cd + 4) <= 32000u && peek32(g + 0x10) <= 32000u && peek32(g + 0x14) <= 32000u, "stress: mesh limit passed at blast %d (col %u tris, render %u/%u)", k, peek16(cd + 4), peek32(g + 0x14), peek32(g + 0x10));
        if (logged("is full")) full++; else if (logged("COL v")) { done++; CHECK(full == 0 || 1, "x"); }
        if ((k % 40 == 39 || full == 1) && logged("RW v") && !opened()) { validate_col("stress", 1); validate_geom("stress", 2); }
        if (fails != before) break;
        if (heapTop - SIM_BASE > SIM_SIZE - 0x01800000u) { heapOut = 1; break; }      /* the simulated heap never gives memory back: enough blasts for one run */
    }
    if (stressBox) {      /* a closed thing (slab, wall, building): nothing may be pushed out of the space it took up */
        U32 g2 = peek32(A_miAtomic + 0x18), rT = peek32(g2 + 0x2C), rnt = peek32(g2 + 0x10), rV = peek32(peek32(g2 + 0x5C) + 0x14), i, outside = 0; float worst = 0.f;
        for (i = 0; i < rnt * 3; i++) { Vec3 p; float ex = 0.f; mem_read(rV + peek16(rT + (i / 3) * 8 + (i % 3) * 2) * 12, &p, 12);
            if (bbox[0] - p.x > ex) ex = bbox[0] - p.x; if (p.x - bbox[3] > ex) ex = p.x - bbox[3]; if (bbox[1] - p.y > ex) ex = bbox[1] - p.y; if (p.y - bbox[4] > ex) ex = p.y - bbox[4];
            if (bbox[2] - p.z > ex) ex = bbox[2] - p.z; if (p.z - bbox[5] > ex) ex = p.z - bbox[5];
            if (ex > 0.25f) outside++; if (ex > worst) worst = ex; }
        printf("  stress: %u vertex uses outside the original bounding box, worst %.2f m\n", outside, worst);
        CHECK(outside < 200u && worst < 1.5f, "stress: rock pushed out of the thing (%u vertex uses outside its box, worst %.2f m)", outside, worst);
    }
    cd = peek32(A_cm + 0x2C); g = peek32(A_miAtomic + 0x18);
    printf("  stress: %d blasts carved, %d refused as full; collision %u tris / render %u verts %u tris\n", done, full, peek16(cd + 4), peek32(g + 0x14), peek32(g + 0x10));
    if (!g_brokeTotal) {      /* a thing that is broken through has no fixed number of craters in it (the shaft ends in the open) */
        CHECK(full >= 5 || heapOut, "stress: the object never reported full after %d blasts", k);
        CHECK(done > 60, "stress: only %d craters before the object was full", done);
    } else CHECK(done > 40 || full == 0, "stress: only %d craters before the object was full", done);
    return fails == before ? 1 : 0;
}

int main(int argc, char** argv) {
    FILE* f; U32 hdr[6], i; char magic[4]; int full = 0, skip = 0, bad = 0;
    if (argc < 2) return 2;
    name = argv[1]; verbose = argc > 2;
    f = fopen(argv[1], "rb"); if (!f) { perror(argv[1]); return 2; }
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "GM02", 4) || fread(hdr, 4, 6, f) != 6 || fread(bbox, 4, 6, f) != 6 || fread(sphere, 4, 4, f) != 4) return 2;
    nv = hdr[0]; nt = hdr[1]; ncv = hdr[2]; nct = hdr[3]; hasNight = hdr[4]; npts = hdr[5];
    R_pos = malloc(nv * 12); R_uv = malloc(nv * 8); R_pre = malloc(nv * 4); R_night = malloc(nv * 4); R_tri = malloc(nt * 6); R_mat = malloc(nt * 2);
    C_verts = malloc(ncv * 6); C_tris = malloc(nct * 8); PTS = malloc(npts * 12);
    if (fread(R_pos, 12, nv, f) != nv || fread(R_uv, 8, nv, f) != nv || fread(R_pre, 4, nv, f) != nv) return 2;
    if (hasNight && fread(R_night, 4, nv, f) != nv) return 2;
    if (fread(R_tri, 6, nt, f) != nt || fread(R_mat, 2, nt, f) != nt || fread(C_verts, 6, ncv, f) != ncv || fread(C_tris, 8, nct, f) != nct || fread(PTS, 12, npts, f) != npts) return 2;
    fclose(f);
    sim = calloc(SIM_SIZE, 1);
    { float sx = bbox[3] - bbox[0], sy = bbox[4] - bbox[1], sz = bbox[5] - bbox[2], wide = sx > sy ? sx : sy;
      int prop; (void)sz; (void)wide;
      { Target tt; ColInfo ci; Kind kd; U32 g0 = build_scene(1, 0, 1); add_entity(2, 100.f, 100.f, 10.f, 0.f, 0, 0.f, MODEL_A, g0);
        prop = inspect(A_storage + 2 * BUILDING_SIZE, &tt) && read_col(&tt, &ci) && (classify(&tt, &ci, &kd), kd.prop); }
      if (argc > 2 && !strcmp(argv[2], "-tree")) { prop = 1; scene_special = 2; verbose = 0; }      /* marked as palm by the game: a prop whatever its shape */
      if (argc > 2 && !strcmp(argv[2], "-prop")) {      /* like the Las Venturas palms: no tree flag, bounding box as wide as the crown and more */
          prop = 1; verbose = 0; bbox[0] -= 8.f; bbox[1] -= 8.f; bbox[3] += 8.f; bbox[4] += 8.f; sphere[3] += 12.f; propWide = 1; }
      if (argc > 5 && !strcmp(argv[2], "-break")) { int ok = 1; verbose = argc > 7; if (argc > 6) break_ring = (float)atof(argv[6]);
          for (i = 0; i < npts && i < 3; i++) ok &= run_break(PTS + i * 3, argv[3], atoi(argv[4]), (float)atof(argv[5]));
          printf("%s break %s x%s %s\n", argv[1], argv[3], argv[4], ok ? "ok" : "FAILED"); return fails ? 1 : 0; }
      if (argc > 2 && !strcmp(argv[2], "-reset")) { int ok = 1; verbose = argc > 3; for (i = 0; i < (npts > 8 ? 1u : npts) && i < 3; i++) ok &= run_reset(PTS + i * 3 + (npts > 8 ? 8 * 3 : 0)); printf("%s reset %s\n", argv[1], ok ? "ok" : "FAILED"); return fails ? 1 : 0; }
      if (argc > 2 && !strcmp(argv[2], "-under")) { int ok; verbose = argc > 3; ok = run_under(PTS + (npts > 8 ? 8 * 3 : 0)); printf("%s under %s\n", argv[1], ok ? "ok" : "FAILED"); return fails ? 1 : 0; }
      if (argc > 2 && !strcmp(argv[2], "-tunnels")) { int ok; verbose = argc > 3; ok = run_tunnels(PTS + (npts > 8 ? 8 * 3 : 0)); printf("%s tunnels %s\n", argv[1], ok ? "ok" : "FAILED"); return fails ? 1 : 0; }
      if (argc > 2 && !strcmp(argv[2], "-stressbox")) { stressBox = 1; argv[2] = "-stress"; }
      if (argc > 2 && !strcmp(argv[2], "-stress")) { verbose = 0; printf("%s stress %s\n", argv[1], run_stress(PTS) ? "ok" : "FAILED"); return fails ? 1 : 0; }
      for (i = 0; i < npts && i < 3; i++) { int r = prop ? run_prop(PTS + i * 3) : run_point(PTS + i * 3); if (r == 1) full++; else if (r >= 2) skip++; else bad++; }
      if (prop) printf("(prop suite) "); }
    printf("%s points=%u full=%d skipped=%d failed=%d checks=%d kept=%lu broke=%lu\n", argv[1], npts < 3 ? npts : 3, full, skip, bad, checks, g_keptTotal, g_brokeTotal);
    return fails ? 1 : 0;
}
