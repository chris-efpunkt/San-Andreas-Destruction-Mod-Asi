/* SA Destruction v3.6.2 - explosion craters by runtime retopology. GTA San Andreas 1.0 US (x86).
 *
 * Every explosion in the game (grenade, rocket, satchel, vehicle, ...) carves a spherical crater
 * into the ORIGINAL world geometry. No modified game files are needed.
 *
 * Per explosion, on the game thread:
 *   1. find static world objects (building pool) whose collision bounds touch the blast
 *   2. find the closest collision surface point S and the local surface normal n
 *   3. crater sphere: centre C = S + n*(R-d), pole Q = C + n*R   (rim radius r, depth d)
 *   4. for each object: rebuild the collision mesh (new CCollisionData block) and the render
 *      mesh (new RpGeometry) with the crater carved in  -> see crater_core.c
 * The runtime is stateless: it always works on whatever geometry the game currently has loaded,
 * so there are no cached pointers that could go stale when the game streams models in and out.
 *
 * Keys: F8 = reset: every crater and hole is taken back, knocked-over props stand again.
 *       F7 = knocking over props on/off.
 * Build with -DHOST_TEST to run the logic against a simulated game memory (test/host_test.c).
 */
typedef unsigned int   U32;
typedef unsigned short U16;
typedef unsigned char  U8;
typedef short          S16;
typedef struct { float x, y, z; } Vec3;

/* ---- GTA SA 1.0 US addresses (gta-reversed / plugin-sdk) ----------------------- */
#define ADDR_CALL_GAME_PROCESS 0x53E981u /* "call CGame::Process" inside Idle()        */
#define ADDR_GAME_PROCESS      0x53BEE0u
#define ADDR_LINE_OF_SIGHT     0x56BA00u /* CWorld::ProcessLineOfSight                 */
#define ADDR_REMOVE_TRI_PLANES 0x416400u /* CCollision::RemoveTrianglePlanes(data)     */
#define ADDR_MEM_MALLOC        0x72F420u /* CMemoryMgr::Malloc                         */
#define ADDR_MEM_FREE          0x72F430u /* CMemoryMgr::Free                           */
#define ADDR_GEOM_CREATE       0x74CA90u /* RpGeometryCreate(numVerts, numTris, format)*/
#define ADDR_GEOM_DESTROY      0x74CCC0u /* RpGeometryDestroy                          */
#define ADDR_GEOM_UNLOCK       0x74C800u /* RpGeometryUnlock                           */
#define ADDR_MATLIST_APPEND    0x74E350u /* _rpMaterialListAppendMaterial              */
#define ADDR_ATOMIC_SET_GEOM   0x749D40u /* RpAtomicSetGeometry(atomic, geometry, flags)*/
#define ADDR_D3D9_SET_USAGE    0x7588B0u /* RpD3D9GeometrySetUsageFlags                */
#define ADDR_D3D9_GET_USAGE    0x7588D0u /* RpD3D9GeometryGetUsageFlags                */
#define ADDR_MODEL_INFO_PTRS   0xA9B0C8u /* CModelInfo::ms_modelInfoPtrs               */
#define ADDR_BUILDING_POOL     0xB74498u /* CPools::ms_pBuildingPool                   */
#define ADDR_EXPLOSIONS        0xC88950u /* CExplosion::aExplosions[16], 0x7C each     */
#define ADDR_EXTRA_VC_OFFSET   0x8D12BCu /* CCustomBuildingDNPipeline::ms_extraVertColourPluginOffset */
#define ADDR_2DFX_OFFSET       0xC3A1E0u /* C2dEffect::g2dEffectPluginOffset (geometry plugin: lights, particles) */
#define ADDR_SURFACE_INFOS     0xB79538u /* g_surfaceInfos: 0x90 bytes adhesion table, then 179 x 12-byte surface entries */
#define ADDR_OBJECT_CREATE     0x5A1F60u /* CObject::Create(modelId, bool)                 */
#define ADDR_WORLD_ADD         0x563220u /* CWorld::Add(entity)                            */
#define ADDR_NUM_TEMP_OBJECTS  0xBB4A70u /* CObject::nNoTempObjects (uint16)               */
#define ADDR_TIME_MS           0xB7CB84u /* CTimer::m_snTimeInMilliseconds                 */
#define ADDR_CURR_AREA         0xB72914u /* CGame::currArea: 0 = outside world, else interior */
#define ADDR_RASTER_CREATE     0x7FB230u /* RwRasterCreate(width, height, depth, flags)    */
#define ADDR_RASTER_LOCK       0x7FB2D0u /* RwRasterLock(raster, level, lockMode)          */
#define ADDR_RASTER_UNLOCK     0x7FAEC0u /* RwRasterUnlock                                 */
#define ADDR_RASTER_DESTROY    0x7FB020u /* RwRasterDestroy                                */
#define ADDR_RASTER_NUM_LEVELS 0x7FB160u /* RwRasterGetNumLevels                           */
#define ADDR_TEXTURE_CREATE    0x7F37C0u /* RwTextureCreate(raster)                        */
#define ADDR_MATERIAL_CREATE   0x74D990u /* RpMaterialCreate()                             */
#define ADDR_MATERIAL_SET_TEX  0x74DBC0u /* RpMaterialSetTexture(material, texture)        */
#define ADDR_CAMERA_MATRIX     (0xB6F028u + 0x974u)
#define ADDR_PLAYER_PED        0xB6F5F0u /* pointer to the player's CPed                    */
#define ADDR_PLAYER_VEHICLE    0xBA18FCu /* pointer to the vehicle the player sits in, or 0 */
#define ADDR_EXE_SIGNATURE     0x401000u
#define SIG_10US_COMPACT       0x53EC8B55u
#define SIG_10US_HOODLUM       0x16197BE9u

#define VK_F7 0x76
#define VK_F8 0x77
#define NUM_EXPLOSIONS 16u
#define EXPLOSION_SIZE 0x7Cu
#define BUILDING_SIZE  0x38u
#define MAX_CAND       16u
#define MAX_SRC_V      60000u     /* largest source mesh we touch */
#define MAX_SRC_T      60000u
/* An object stops taking craters when one of its meshes reaches this many vertices or triangles.
 * The game crashed right after a collision mesh went from 32722 to 32922 triangles - past 32767,
 * the end of a signed 16-bit number. Stay clear of it, for both meshes. */
static U32 g_meshLimit = 32000u;
#define FULL_TEXT "this object is full: it cannot take any more craters (mesh limit reached)"
#define MAX_EXTRA      65536u     /* bytes of collision spheres/boxes/lines we carry over */
#define MAX_GROUPS     6000u
#define MAX_HOLE_TRIS  1500u      /* thin collision triangles remembered for the render mesh */
#define PROP_MAX_WIDTH 10.0f      /* objects placed several times with a footprint below this are knocked over */
#define PROP_THIN_WIDTH 5.0f      /* tall objects narrower than this are knocked over */
#define GROUP_CELL     8.0f       /* metres, spatial bucket for collision face groups */
#define GROUP_MAX_TRIS 48u
#define CRATERS_PER_FRAME 2u
#define ROCK_TILE_M 4.0f          /* metres covered by one repeat of the rock texture */

/* ---- platform layer ------------------------------------------------------------ */
static int  mem_read(U32 addr, void* out, U32 n);
static int  mem_write(U32 addr, const void* src, U32 n);
static void logline(const char* s);
static int  game_line_of_sight(const Vec3* from, const Vec3* to, U8* colPoint, U32* entity);
static void game_remove_planes(U32 colData);
static U32  game_malloc(U32 size);
static void game_free(U32 ptr);
static U32  rw_geom_create(U32 numVerts, U32 numTris, U32 format);
static void rw_geom_destroy(U32 geom);
static U32  rw_geom_unlock(U32 geom);
static int  rw_matlist_append(U32 matList, U32 material);
static void rw_atomic_set_geometry(U32 atomic, U32 geom);
static U32  rw_d3d9_get_usage(U32 geom);
static void rw_d3d9_set_usage(U32 geom, U32 flags);
static U32  rw_raster_create(U32 w, U32 h, U32 depth, U32 flags);
static U32  rw_raster_lock(U32 raster, U32 level, U32 mode);      /* -> pixel address */
static void rw_raster_unlock(U32 raster);
static void rw_raster_destroy(U32 raster);
static U32  rw_raster_num_levels(U32 raster);
static U32  rw_texture_create(U32 raster);
static U32  rw_material_create(void);
static void rw_material_set_texture(U32 material, U32 texture);
static U32  game_object_create(U32 model);
static void game_world_add(U32 entity);
/* trusted reads of live engine structures (pool storage, model info table, explosion array) */
static U32   peek32(U32 addr);
static U32   peek16(U32 addr);
static U32   peek8(U32 addr);
static float peekf(U32 addr);

#ifdef HOST_TEST
#include <string.h>
#else
#define WINAPI __stdcall
#define DLLIMPORT __declspec(dllimport)
#define MAX_PATH 260
DLLIMPORT U32   WINAPI GetModuleFileNameA(void*, char*, U32);
DLLIMPORT int   WINAPI DisableThreadLibraryCalls(void*);
DLLIMPORT int   WINAPI CloseHandle(void*);
DLLIMPORT void* WINAPI CreateFileA(const char*, U32, U32, void*, U32, U32, void*);
DLLIMPORT int   WINAPI WriteFile(void*, const void*, U32, U32*, void*);
DLLIMPORT U32   WINAPI SetFilePointer(void*, long, long*, U32);
DLLIMPORT void* WINAPI GetCurrentProcess(void);
DLLIMPORT U32   WINAPI GetCurrentProcessId(void);
DLLIMPORT int   WINAPI ReadProcessMemory(void*, const void*, void*, U32, U32*);
DLLIMPORT int   WINAPI WriteProcessMemory(void*, void*, const void*, U32, U32*);
DLLIMPORT int   WINAPI VirtualProtect(void*, U32, U32, U32*);
DLLIMPORT int   WINAPI FlushInstructionCache(void*, const void*, U32);
DLLIMPORT void* WINAPI GetForegroundWindow(void);
DLLIMPORT U32   WINAPI GetWindowThreadProcessId(void*, U32*);
DLLIMPORT short WINAPI GetAsyncKeyState(int);
int _fltused = 0;
void* __cdecl memset(void*, int, unsigned int);
void* __cdecl memcpy(void*, const void*, unsigned int);
#endif

#include "crater_core.c"
#include "rock_texture.c"

/* ---- small helpers --------------------------------------------------------------- */
static U32 slen(const char* s) { U32 n = 0; while (s[n] && n < 950) n++; return n; }
static int cat(char* s, int p, int cap, const char* t) { while (*t && p < cap - 1) s[p++] = *t++; s[p] = 0; return p; }
static int dec(char* s, int p, int cap, U32 n) {
    char b[12]; int j = 0;
    do { b[j++] = (char)('0' + n % 10); n /= 10; } while (n && j < 11);
    while (j && p < cap - 1) s[p++] = b[--j];
    s[p] = 0; return p;
}
static int hex(char* s, int p, int cap, U32 n) {
    static const char d[] = "0123456789ABCDEF"; int i;
    p = cat(s, p, cap, "0x");
    for (i = 28; i >= 0 && p < cap - 1; i -= 4) s[p++] = d[(n >> i) & 15];
    s[p] = 0; return p;
}
static U32 u32(const U8* b) { return (U32)b[0] | ((U32)b[1] << 8) | ((U32)b[2] << 16) | ((U32)b[3] << 24); }
static U32 u16(const U8* b) { return (U32)b[0] | ((U32)b[1] << 8); }
static float f32(const U8* b) { union { U32 d; float f; } u; u.d = u32(b); return u.f; }
static void put32(U8* b, U32 v) { b[0] = (U8)v; b[1] = (U8)(v >> 8); b[2] = (U8)(v >> 16); b[3] = (U8)(v >> 24); }
static void put16(U8* b, U32 v) { b[0] = (U8)v; b[1] = (U8)(v >> 8); }
static void putf(U8* b, float f) { union { U32 d; float f; } u; u.f = f; put32(b, u.d); }
static float ab(float x) { return x < 0.f ? -x : x; }
static int number_f(char* s, int p, int cap, float f) {
    U32 a, frac;
    if (f != f || ab(f) > 1000000.f) return cat(s, p, cap, "INVALID");
    if (f < 0) { p = cat(s, p, cap, "-"); f = -f; }
    a = (U32)f; frac = (U32)((f - (float)a) * 100.f + 0.5f);
    if (frac >= 100) { a++; frac = 0; }
    p = dec(s, p, cap, a); p = cat(s, p, cap, ".");
    if (frac < 10) p = cat(s, p, cap, "0");
    return dec(s, p, cap, frac);
}
static int vprint(char* s, int p, int cap, const Vec3* v) {
    p = cat(s, p, cap, "("); p = number_f(s, p, cap, v->x);
    p = cat(s, p, cap, ","); p = number_f(s, p, cap, v->y);
    p = cat(s, p, cap, ","); p = number_f(s, p, cap, v->z);
    return cat(s, p, cap, ")");
}
static int position_ok(const Vec3* v) {
    return v->x == v->x && v->y == v->y && v->z == v->z &&
           ab(v->x) < 12000.f && ab(v->y) < 12000.f && ab(v->z) < 10000.f;
}
static int close(float a, float b, float tolerance) { return ab(a - b) < tolerance; }
static void sincos_f(float a, float* s, float* c) {
    const float PI = 3.14159265f, TWO_PI = 6.28318531f;
    float sign = 1.f, x2;
    while (a > PI) a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    if (a > PI * 0.5f) { a = PI - a; sign = -1.f; }
    else if (a < -PI * 0.5f) { a = -PI - a; sign = -1.f; }
    x2 = a * a;
    *s = a * (1.f + x2 * (-1.f / 6.f + x2 * (1.f / 120.f + x2 * (-1.f / 5040.f + x2 * (1.f / 362880.f)))));
    *c = sign * (1.f + x2 * (-0.5f + x2 * (1.f / 24.f + x2 * (-1.f / 720.f + x2 * (1.f / 40320.f + x2 * (-1.f / 3628800.f))))));
}
static const char* g_why = "";
static void stage(const char* what) { char m[120]; int p = cat(m, 0, 120, "[v3.6.2]     stage: "); cat(m, p, 120, what); logline(m); }

/* ---- object placement ------------------------------------------------------------ */
typedef struct { Vec3 pos; float c, s; } Frame;       /* world = pos + Rz(c,s) * local */
static int entity_frame(U32 ent, Frame* f) {
    U8 e[0x38], mat[0x40];
    U32 m;
    if (!mem_read(ent, e, sizeof(e))) { g_why = "entity not readable"; return 0; }
    m = u32(e + 0x14);
    if (m) {
        float rx, ry, rz, fx, fy, fz, ux, uy, uz;
        if (!mem_read(m, mat, sizeof(mat))) { g_why = "entity matrix not readable"; return 0; }
        rx = f32(mat); ry = f32(mat + 4); rz = f32(mat + 8);
        fx = f32(mat + 0x10); fy = f32(mat + 0x14); fz = f32(mat + 0x18);
        ux = f32(mat + 0x20); uy = f32(mat + 0x24); uz = f32(mat + 0x28);
        if (!close(uz, 1.f, .02f) || ab(ux) > .02f || ab(uy) > .02f || ab(rz) > .02f || ab(fz) > .02f) { g_why = "object is tilted"; return 0; }
        if (!close(rx, fy, .02f) || !close(ry, -fx, .02f) || !close(rx * rx + ry * ry, 1.f, .04f)) { g_why = "object is scaled"; return 0; }
        f->c = rx; f->s = ry;
        f->pos.x = f32(mat + 0x30); f->pos.y = f32(mat + 0x34); f->pos.z = f32(mat + 0x38);
    } else {
        float heading = f32(e + 0x10);
        if (heading != heading || ab(heading) > 100.f) { g_why = "invalid heading"; return 0; }
        sincos_f(heading, &f->s, &f->c);
        f->pos.x = f32(e + 4); f->pos.y = f32(e + 8); f->pos.z = f32(e + 0xC);
    }
    if (!position_ok(&f->pos)) { g_why = "invalid position"; return 0; }
    return 1;
}
static Vec3 to_local(const Frame* f, Vec3 w) {
    float dx = w.x - f->pos.x, dy = w.y - f->pos.y; Vec3 l;
    l.x = dx * f->c + dy * f->s; l.y = -dx * f->s + dy * f->c; l.z = w.z - f->pos.z;
    return l;
}
static Vec3 to_world(const Frame* f, Vec3 l) {
    Vec3 w;
    w.x = f->pos.x + l.x * f->c - l.y * f->s; w.y = f->pos.y + l.x * f->s + l.y * f->c; w.z = f->pos.z + l.z;
    return w;
}
static Vec3 dir_local(const Frame* f, Vec3 d) { Vec3 l; l.x = d.x * f->c + d.y * f->s; l.y = -d.x * f->s + d.y * f->c; l.z = d.z; return l; }
static Vec3 dir_world(const Frame* f, Vec3 d) { Vec3 w; w.x = d.x * f->c - d.y * f->s; w.y = d.x * f->s + d.y * f->c; w.z = d.z; return w; }

/* ---- what we need to know about one world object ------------------------------------ */
typedef struct {
    U32 entity, model, atomic, geom, modelInfo, colModel, colData;
    Frame frame;
} Target;
static int inspect(U32 ent, Target* t) {
    U8 e[0x38], atom[0x20], cm[0x30];
    U32 mi = 0;
    if (!entity_frame(ent, &t->frame)) return 0;
    if (!mem_read(ent, e, sizeof(e))) { g_why = "entity not readable"; return 0; }
    t->entity = ent; t->model = u16(e + 0x22);
    if (t->model < 1u || t->model > 19999u) { g_why = "model id out of range"; return 0; }
    t->atomic = u32(e + 0x18);
    if (!t->atomic || !mem_read(t->atomic, atom, sizeof(atom))) { g_why = "render object not loaded"; return 0; }
    if (atom[0] != 1) { g_why = "render object is not a single atomic"; return 0; }
    t->geom = u32(atom + 0x18);
    if (!t->geom) { g_why = "no geometry"; return 0; }
    if (!mem_read(ADDR_MODEL_INFO_PTRS + t->model * 4u, &mi, 4) || !mi) { g_why = "no model info"; return 0; }
    t->modelInfo = mi;
    if (!mem_read(mi + 0x14, &t->colModel, 4) || !t->colModel || !mem_read(t->colModel, cm, sizeof(cm))) { g_why = "no collision model"; return 0; }
    t->colData = u32(cm + 0x2c);
    if (!t->colData) { g_why = "collision data not loaded"; return 0; }
    if (!(cm[0x29] & 2u)) { g_why = "collision data is not a single allocation"; return 0; }
    return 1;
}

/* ---- scratch meshes -------------------------------------------------------------------- */
static Vec3 g_cPos[GM_MAXV]; static U16 g_cTri[GM_MAXT * 3]; static U16 g_cAux[GM_MAXT];
static Vec3 g_rPos[GM_MAXV]; static U16 g_rTri[GM_MAXT * 3]; static U16 g_rAux[GM_MAXT];
static float g_rUv0[GM_MAXV * 2], g_rUv1[GM_MAXV * 2], g_rNrm[GM_MAXV * 3];
static U8 g_rPre[GM_MAXV * 4], g_rNight[GM_MAXV * 4], g_rDay[GM_MAXV * 4];
static U8 g_io[GM_MAXT * 8];                     /* raw triangle / vertex transfer buffer */
static U8 g_extra[MAX_EXTRA];
static U8 g_groups[MAX_GROUPS * 28];
static U16 g_sortKey[GM_MAXT]; static U16 g_sortTri[GM_MAXT * 3]; static U16 g_sortAux[GM_MAXT];
static U32 g_bucket[4097];
static U8 g_cFlag[GM_MAXT], g_rFlag[GM_MAXT];
static Vec3 g_holeTri[MAX_HOLE_TRIS * 3]; static U32 g_nHoleTri;
static U8 g_sheet[256]; static int g_sheetReady;
static GmMesh g_cm, g_rm;
/* surface types that are sheets by nature: see-through, shoot-through or glass (fences, grilles, windows) */
static void load_sheet_surfaces(void) {
    U32 i;
    if (g_sheetReady) return;
    g_sheetReady = 1;
    /* a sheet is what bullets fly through (wire fence, railing, hedge). Glass is not: the glass
     * fronts of buildings have nothing behind them, a hole would show the void inside the shell. */
    for (i = 0; i < 179u; i++) g_sheet[i] = (peek32(ADDR_SURFACE_INFOS + 0x90u + i * 12u + 4u) & 0x00002000u) ? 1 : 0;
}

typedef struct {
    U32 nSpheres, nBoxes, nLines, flags, extraBytes, planes, boxesConverted;
    U32 pSpheres, pBoxes, pLines, pVerts, pTris;
} ColInfo;

/* collision mesh of the object -> g_cm (positions in metres, aux = surface | light << 8).
 * Collision BOXES are turned into triangles here (12 per box, facing outward): buildings are
 * largely made of boxes, and only triangles can be carved. The rebuilt collision block then has
 * no boxes any more. Spheres and lines are carried over unchanged. */
static int read_col(const Target* t, ColInfo* ci) {
    static const U8 quad[6][4] = { {1,3,7,5}, {0,4,6,2}, {2,6,7,3}, {0,1,5,4}, {4,5,7,6}, {0,2,3,1} };   /* counter-clockwise seen from outside */
    U8 h[0x30]; U32 nt, nv = 0, i, k, maxIndex = 0, nBoxes;
    if (!mem_read(t->colData, h, sizeof(h))) { g_why = "collision header not readable"; return 0; }
    ci->nSpheres = u16(h); nBoxes = u16(h + 2); nt = u16(h + 4); ci->nLines = h[6]; ci->flags = h[7];
    ci->pSpheres = u32(h + 8); ci->pBoxes = u32(h + 0xC); ci->pLines = u32(h + 0x10);
    ci->pVerts = u32(h + 0x14); ci->pTris = u32(h + 0x18); ci->planes = u32(h + 0x1C);
    ci->nBoxes = 0;
    if (ci->flags & 1u) { g_why = "collision uses disks"; return 0; }
    if (nt > MAX_SRC_T || (nt && (!ci->pVerts || !ci->pTris)) || (nBoxes && !ci->pBoxes)) { g_why = "collision mesh unusable"; return 0; }
    if (!nt && !nBoxes && !ci->nSpheres) { g_why = "collision has no shapes"; return 0; }
    ci->extraBytes = ci->nSpheres * 0x14u + ci->nLines * 0x20u;
    if (ci->extraBytes > MAX_EXTRA || nBoxes * 0x1Cu > MAX_EXTRA) { g_why = "too many collision primitives"; return 0; }
    if ((ci->nSpheres && !ci->pSpheres) || (ci->nLines && !ci->pLines)) { g_why = "collision primitive pointers missing"; return 0; }
    if (nt) {
        if (!mem_read(ci->pTris, g_io, nt * 8u)) { g_why = "collision triangles not readable"; return 0; }
        for (i = 0; i < nt; i++) {
            U32 a = u16(g_io + i * 8), b = u16(g_io + i * 8 + 2), c = u16(g_io + i * 8 + 4);
            g_cTri[i * 3] = (U16)a; g_cTri[i * 3 + 1] = (U16)b; g_cTri[i * 3 + 2] = (U16)c;
            g_cAux[i] = (U16)u16(g_io + i * 8 + 6);
            if (a > maxIndex) maxIndex = a; if (b > maxIndex) maxIndex = b; if (c > maxIndex) maxIndex = c;
        }
        nv = maxIndex + 1;
        if (nv > MAX_SRC_V) { g_why = "collision mesh too large"; return 0; }
        if (!mem_read(ci->pVerts, g_io, nv * 6u)) { g_why = "collision vertices not readable"; return 0; }
        for (i = 0; i < nv; i++) {
            g_cPos[i].x = (float)(S16)u16(g_io + i * 6) / 128.f;
            g_cPos[i].y = (float)(S16)u16(g_io + i * 6 + 2) / 128.f;
            g_cPos[i].z = (float)(S16)u16(g_io + i * 6 + 4) / 128.f;
        }
    }
    if (nBoxes) {
        if (nv + nBoxes * 8u > MAX_SRC_V || nt + nBoxes * 12u > MAX_SRC_T) { g_why = "collision mesh too large"; return 0; }
        if (!mem_read(ci->pBoxes, g_io, nBoxes * 0x1Cu)) { g_why = "collision boxes not readable"; return 0; }
        for (i = 0; i < nBoxes; i++) {
            const U8* bx = g_io + i * 0x1Cu; U16 aux = (U16)(bx[0x18] | (bx[0x1A] << 8));
            float lo[3], hi[3];
            lo[0] = f32(bx); lo[1] = f32(bx + 4); lo[2] = f32(bx + 8); hi[0] = f32(bx + 12); hi[1] = f32(bx + 16); hi[2] = f32(bx + 20);
            for (k = 0; k < 8; k++) {
                g_cPos[nv + k].x = (k & 1u) ? hi[0] : lo[0]; g_cPos[nv + k].y = (k & 2u) ? hi[1] : lo[1]; g_cPos[nv + k].z = (k & 4u) ? hi[2] : lo[2];
                if (!position_ok(&g_cPos[nv + k]) || ab(g_cPos[nv + k].x) > 255.f || ab(g_cPos[nv + k].y) > 255.f || ab(g_cPos[nv + k].z) > 255.f) { g_why = "collision box out of range"; return 0; }
            }
            for (k = 0; k < 6; k++) {
                U32 a = nv + quad[k][0], b = nv + quad[k][1], c = nv + quad[k][2], d = nv + quad[k][3];
                /* collision winding: (C-A)x(B-A) points outward, i.e. clockwise seen from outside */
                g_cTri[nt * 3] = (U16)a; g_cTri[nt * 3 + 1] = (U16)c; g_cTri[nt * 3 + 2] = (U16)b; g_cAux[nt++] = aux;
                g_cTri[nt * 3] = (U16)a; g_cTri[nt * 3 + 1] = (U16)d; g_cTri[nt * 3 + 2] = (U16)c; g_cAux[nt++] = aux;
            }
            nv += 8;
        }
    }
    ci->boxesConverted = nBoxes;
    memset(&g_cm, 0, sizeof(g_cm));
    g_cm.nv = nv; g_cm.nt = nt; g_cm.pos = g_cPos; g_cm.tri = g_cTri; g_cm.aux = g_cAux; g_cm.normalAttr = -1;
    g_cm.flag = g_cFlag; memset(g_cFlag, 0, nt);
    return 1;
}
static S16 quant(float v) {
    float q = v * 128.f; int i = (int)(q < 0.f ? q - 0.5f : q + 0.5f);
    if (i > 32767) i = 32767; if (i < -32767) i = -32767;
    return (S16)i;
}

/* Collision face groups: triangles sorted into an 8 m grid, max 48 per group, each with its
 * bounding box. The game uses them to skip triangles that cannot touch a sphere. */
static U32 build_groups(GmMesh* m) {
    Vec3 lo = m->pos[m->tri[0]], hi = lo; U32 t, k, n = 0, start;
    float cell;
    for (t = 0; t < m->nt * 3; t++) {
        Vec3 p = m->pos[m->tri[t]];
        if (p.x < lo.x) lo.x = p.x; if (p.y < lo.y) lo.y = p.y; if (p.x > hi.x) hi.x = p.x; if (p.y > hi.y) hi.y = p.y;
    }
    cell = GROUP_CELL;
    while ((hi.x - lo.x) / cell >= 64.f || (hi.y - lo.y) / cell >= 64.f) cell *= 2.f;
    memset(g_bucket, 0, sizeof(g_bucket));
    for (t = 0; t < m->nt; t++) {
        Vec3 a = m->pos[m->tri[t * 3]], b = m->pos[m->tri[t * 3 + 1]], c = m->pos[m->tri[t * 3 + 2]];
        U32 cx = (U32)(((a.x + b.x + c.x) / 3.f - lo.x) / cell), cy = (U32)(((a.y + b.y + c.y) / 3.f - lo.y) / cell);
        if (cx > 63) cx = 63; if (cy > 63) cy = 63;
        g_sortKey[t] = (U16)(cy * 64 + cx); g_bucket[g_sortKey[t] + 1]++;
    }
    for (k = 0; k < 4096; k++) g_bucket[k + 1] += g_bucket[k];
    for (t = 0; t < m->nt; t++) {
        U32 d = g_bucket[g_sortKey[t]]++;
        g_sortTri[d * 3] = m->tri[t * 3]; g_sortTri[d * 3 + 1] = m->tri[t * 3 + 1]; g_sortTri[d * 3 + 2] = m->tri[t * 3 + 2];
        g_sortAux[d] = m->aux[t];
    }
    /* g_bucket[k] is now the END of bucket k */
    memcpy(m->tri, g_sortTri, m->nt * 6u); memcpy(m->aux, g_sortAux, m->nt * 2u);
    start = 0;
    for (k = 0; k < 4096 && start < m->nt; k++) {
        U32 end = g_bucket[k];
        while (start < end) {
            U32 last = start + GROUP_MAX_TRIS - 1, i; Vec3 bl, bh; U8* g;
            if (last >= end) last = end - 1;
            if (n >= MAX_GROUPS) return 0;
            bl = m->pos[m->tri[start * 3]]; bh = bl;
            for (i = start * 3; i < (last + 1) * 3; i++) {
                Vec3 p = m->pos[m->tri[i]];
                if (p.x < bl.x) bl.x = p.x; if (p.y < bl.y) bl.y = p.y; if (p.z < bl.z) bl.z = p.z;
                if (p.x > bh.x) bh.x = p.x; if (p.y > bh.y) bh.y = p.y; if (p.z > bh.z) bh.z = p.z;
            }
            g = g_groups + n * 28;
            putf(g, bl.x - 0.02f); putf(g + 4, bl.y - 0.02f); putf(g + 8, bl.z - 0.02f);
            putf(g + 12, bh.x + 0.02f); putf(g + 16, bh.y + 0.02f); putf(g + 20, bh.z + 0.02f);
            put16(g + 24, start); put16(g + 26, last);
            n++; start = last + 1;
        }
    }
    return n;
}

/* Collision, step 1: carve the crater in the scratch mesh. Nothing in the game is touched.
 * Returns 1 ready to commit, 0 object not inside the crater, -1 failed. */
static U32 g_statCv0, g_statCv1, g_statCt0, g_statCt1, g_statGroups;
static ColInfo g_ci; static U32 g_nGroups;
static U32 g_statHoleC, g_statHoleR;
static U32 g_statShell, g_statShellC;        /* triangles opened by breaking through (visible / collision) */
#ifdef HOST_TEST
static Vec3 g_dbgStay[16000]; static U32 g_dbgStayN;     /* where the last crater stopped vertices short of the sphere (collision and visible mesh) */
#endif
/* ---- F8: everything back as it was ---------------------------------------------------------
 * The first time a model is changed its ORIGINAL meshes are kept: of the visible geometry a plain
 * copy of its data (no engine object is held on to - the game may unload the model and its
 * textures whenever it likes), of the collision the game's own block, which is simply not freed.
 * A reset builds the geometry anew from the copy and puts the block back - where the model still
 * carries what we made of it. Where the game has loaded the model afresh in the meantime (it does
 * when one drives away and comes back), it is whole again anyway; what was kept is only released. */
#define MAX_UNDO 512u
#define UNDO_MAX_BYTES (96u * 1024u * 1024u)
typedef struct { U32 model, snap, snapBytes, curGeom, origCol, curCol; U8 bounds[0x28]; } Undo;
static Undo g_undo[MAX_UNDO]; static U32 g_nUndo, g_undoBytes; static int g_undoOff, g_undoFull;
static U32 g_pendSnap, g_pendBytes, g_pendGeom;       /* copy of the geometry about to be changed, not yet filed */
static Undo* undo_find(U32 model) { U32 i; for (i = 0; i < g_nUndo; i++) if (g_undo[i].model == model) return &g_undo[i]; return 0; }
static Undo* undo_for(U32 model) {
    Undo* u = undo_find(model);
    if (u) return u;
    if (g_nUndo >= MAX_UNDO) {
        if (!g_undoFull) { g_undoFull = 1; logline("[v3.6.2] very many objects changed: the ones changed from now on cannot be reset with F8 (they become whole again when the game loads them afresh)."); }
        return 0;
    }
    u = &g_undo[g_nUndo++]; memset(u, 0, sizeof(*u)); u->model = model;
    return u;
}
static void col_after_cut(GmMesh* m, Vec3 C, float R, Vec3 Q, GmStats* st);
static int prepare_col(const Target* t, Vec3 C, float R, Vec3 Q, float h, float depth, int sheetsOnly) {
    GmStats st; U32 i, off; int r;
    if (!read_col(t, &g_ci)) return -1;
    g_statCv0 = g_cm.nv; g_statCt0 = g_cm.nt;
    /* sheets (fences, railings) get a hole instead of a bowl; everything else is solid */
    g_nHoleTri = 0; (void)depth;
    if (gm_mark_thin(&g_cm, C, R, 0.f, 1, g_sheet))
        for (i = 0; i < g_cm.nt && g_nHoleTri < MAX_HOLE_TRIS; i++) if (g_cFlag[i]) {
            g_holeTri[g_nHoleTri * 3] = g_cPos[g_cTri[i * 3]]; g_holeTri[g_nHoleTri * 3 + 1] = g_cPos[g_cTri[i * 3 + 1]]; g_holeTri[g_nHoleTri * 3 + 2] = g_cPos[g_cTri[i * 3 + 2]];
            g_nHoleTri++;
        }
    if (sheetsOnly) return 1;
    gm_after_cut = col_after_cut;
    r = gm_crater(&g_cm, C, R, Q, h, 1.f, 0, 0, &st);
    gm_after_cut = 0;
    if (r < 0) { g_why = FULL_TEXT; return -1; }
#ifdef HOST_TEST
    for (i = 0; i < g_cm.nv && g_dbgStayN < 16000u; i++) if (gm_clamp[i] >= 0.f) g_dbgStay[g_dbgStayN++] = g_cPos[i];
#endif
    if (r == 0) return 0;
    g_statShellC = st.shell < st.holeTris ? st.shell : st.holeTris; g_statHoleC = st.holeTris - g_statShellC;
    for (i = 0; i < g_cm.nv; i++) {             /* the game stores collision vertices as int16/128 */
        g_cPos[i].x = (float)quant(g_cPos[i].x) / 128.f; g_cPos[i].y = (float)quant(g_cPos[i].y) / 128.f; g_cPos[i].z = (float)quant(g_cPos[i].z) / 128.f;
    }
    gm_drop_degenerate(&g_cm);
    gm_compact(&g_cm);
    if (g_cm.nv > g_meshLimit || g_cm.nt > g_meshLimit) { g_why = FULL_TEXT; return -1; }
    if (g_cm.nt < 1) return 2;                  /* nothing left of it */
    g_nGroups = g_cm.nt > 80 ? build_groups(&g_cm) : 0;
    /* spheres and lines are carried over unchanged (boxes have become triangles) */
    off = 0;
    if (g_ci.nSpheres) { if (!mem_read(g_ci.pSpheres, g_extra + off, g_ci.nSpheres * 0x14u)) { g_why = "collision spheres not readable"; return -1; } off += g_ci.nSpheres * 0x14u; }
    if (g_ci.nLines) { if (!mem_read(g_ci.pLines, g_extra + off, g_ci.nLines * 0x20u)) { g_why = "collision lines not readable"; return -1; } }
    return 1;
}
/* Collision, step 2: build the new CCollisionData block and swap it in. Returns 1 done, -1 failed. */
static int commit_col(const Target* t) {
    const ColInfo ci = g_ci; U8 hdr[0x30], cm[0x30], cm0[0x28]; U32 i, nGroups = g_nGroups, size, off, base, oVerts, oGroups, oTris, vbytes;
    Vec3 lo, hi, cen; float radius, r2;
    int r;
    if (!mem_read(t->colModel, cm, sizeof(cm)) || u32(cm + 0x2c) != t->colData) { g_why = "collision model changed meanwhile"; return -1; }
    memcpy(cm0, cm, sizeof(cm0));
    /* one block, laid out like the game's own loader does it:
     * CCollisionData | spheres | boxes | lines | vertices | face groups | group count | triangles */
    vbytes = (g_cm.nv * 6u + 3u) & ~3u;
    oVerts = 0x30u + ci.extraBytes; oGroups = oVerts + vbytes; oTris = oGroups + (nGroups ? nGroups * 28u + 4u : 0u);
    size = oTris + g_cm.nt * 8u;
    base = game_malloc(size);
    if (!base) { g_why = "game allocator returned no memory"; return -1; }
    memset(hdr, 0, sizeof(hdr));
    put16(hdr, ci.nSpheres); put16(hdr + 2, ci.nBoxes); put16(hdr + 4, g_cm.nt); hdr[6] = (U8)ci.nLines;
    hdr[7] = (U8)((nGroups ? 2u : 0u) | (ci.flags & 4u));
    off = base + 0x30u;
    if (ci.nSpheres) { put32(hdr + 8, off); off += ci.nSpheres * 0x14u; }
    if (ci.nBoxes) { put32(hdr + 0xC, off); off += ci.nBoxes * 0x1Cu; }
    if (ci.nLines) { put32(hdr + 0x10, off); }
    put32(hdr + 0x14, base + oVerts); put32(hdr + 0x18, base + oTris);   /* +0x1C triangle planes: built lazily by the game */
    if (ci.flags & 4u) {                                                   /* shadow mesh = the collision mesh itself */
        put32(hdr + 0x20, g_cm.nt); put32(hdr + 0x24, g_cm.nv); put32(hdr + 0x28, base + oVerts); put32(hdr + 0x2C, base + oTris);
    }
    lo.x = f32(cm); lo.y = f32(cm + 4); lo.z = f32(cm + 8); hi.x = f32(cm + 0xC); hi.y = f32(cm + 0x10); hi.z = f32(cm + 0x14);
    cen.x = f32(cm + 0x18); cen.y = f32(cm + 0x1C); cen.z = f32(cm + 0x20); radius = f32(cm + 0x24); r2 = radius * radius;
    memset(g_io, 0, vbytes);
    for (i = 0; i < g_cm.nv; i++) {
        Vec3 p = g_cPos[i]; float d2 = gm_dist2(p, cen);
        put16(g_io + i * 6, (U32)(U16)quant(p.x)); put16(g_io + i * 6 + 2, (U32)(U16)quant(p.y)); put16(g_io + i * 6 + 4, (U32)(U16)quant(p.z));
        if (p.x - 0.05f < lo.x) lo.x = p.x - 0.05f; if (p.y - 0.05f < lo.y) lo.y = p.y - 0.05f; if (p.z - 0.05f < lo.z) lo.z = p.z - 0.05f;
        if (p.x + 0.05f > hi.x) hi.x = p.x + 0.05f; if (p.y + 0.05f > hi.y) hi.y = p.y + 0.05f; if (p.z + 0.05f > hi.z) hi.z = p.z + 0.05f;
        if (d2 > r2) r2 = d2;
    }
    if (r2 > radius * radius) radius = gm_sqrt(r2) + 0.05f;
    r = mem_write(base, hdr, 0x30u) && (!ci.extraBytes || mem_write(base + 0x30u, g_extra, ci.extraBytes)) && mem_write(base + oVerts, g_io, vbytes);
    if (r && nGroups) { U8 n4[4]; put32(n4, nGroups); r = mem_write(base + oGroups, g_groups, nGroups * 28u) && mem_write(base + oTris - 4u, n4, 4); }
    if (r) {
        for (i = 0; i < g_cm.nt; i++) {
            put16(g_io + i * 8, g_cTri[i * 3]); put16(g_io + i * 8 + 2, g_cTri[i * 3 + 1]); put16(g_io + i * 8 + 4, g_cTri[i * 3 + 2]);
            put16(g_io + i * 8 + 6, g_cAux[i]);
        }
        r = mem_write(base + oTris, g_io, g_cm.nt * 8u);
    }
    if (!r) { game_free(base); g_why = "new collision block not writable"; return -1; }
    /* swap: bounds first (bigger is always safe), then the data pointer, then free the old block */
    putf(cm, lo.x); putf(cm + 4, lo.y); putf(cm + 8, lo.z); putf(cm + 0xC, hi.x); putf(cm + 0x10, hi.y); putf(cm + 0x14, hi.z); putf(cm + 0x24, radius);
    if (ci.planes) game_remove_planes(t->colData);
    put32(cm + 0x2c, base);
    if (!mem_write(t->colModel, cm, sizeof(cm))) { game_free(base); g_why = "collision model not writable"; return -1; }
    { /* the block that goes: ours from an earlier blast - or the game's original, which is kept for F8 */
      Undo* u = undo_find(t->model); int keep = 0;
      if (!u && !g_undoOff) u = undo_for(t->model);
      if (u) {
          if (u->origCol && u->curCol != t->colData) { game_free(u->origCol); u->origCol = 0; }      /* loaded afresh since: what was kept is nobody's any more */
          if (!u->origCol && !g_undoOff) { u->origCol = t->colData; memcpy(u->bounds, cm0, sizeof(cm0)); keep = 1; }
          u->curCol = base;
      }
      if (!keep) game_free(t->colData); }
    g_statCv1 = g_cm.nv; g_statCt1 = g_cm.nt; g_statGroups = nGroups;
    return 1;
}

/* render mesh of the object -> g_rm */
typedef struct {
    U32 flags, nSets, nMat, pMat, pTri, pPre, pUv[2], pVerts, pNormals, morph, vcOffset, pNight, pDay, dnBalance;
    Vec3 sphC; float sphR;
} GeoInfo;
static int read_render(U32 geom, GeoInfo* gi) {
    U8 g[0x60], mt[0x1C], vc[12]; U32 nt, nv, i, k, off = 0;
    if (!mem_read(geom, g, sizeof(g))) { g_why = "geometry not readable"; return 0; }
    gi->flags = u32(g + 8); nt = u32(g + 0x10); nv = u32(g + 0x14); gi->nSets = u32(g + 0x1C);
    gi->pMat = u32(g + 0x20); gi->nMat = u32(g + 0x24); gi->pTri = u32(g + 0x2C); gi->pPre = u32(g + 0x30);
    gi->pUv[0] = u32(g + 0x34); gi->pUv[1] = u32(g + 0x38); gi->morph = u32(g + 0x5C);
    if (gi->flags & 0x01000000u) { g_why = "geometry is pre-instanced (no vertex data in memory)"; return 0; }
    if (u32(g + 0x18) != 1u) { g_why = "geometry has several morph targets"; return 0; }
    if (!nt || nt > MAX_SRC_T || nv < 3 || nv > MAX_SRC_V || !gi->pTri || !gi->morph) { g_why = "render mesh size outside limits"; return 0; }
    if (gi->nSets > 2) { g_why = "more than two UV sets"; return 0; }
    if (!gi->nMat || gi->nMat > 255 || !gi->pMat) { g_why = "material list unusable"; return 0; }
    if (!mem_read(gi->morph, mt, sizeof(mt)) || u32(mt) != geom) { g_why = "morph target invalid"; return 0; }
    gi->sphC.x = f32(mt + 4); gi->sphC.y = f32(mt + 8); gi->sphC.z = f32(mt + 0xC); gi->sphR = f32(mt + 0x10);
    gi->pVerts = u32(mt + 0x14); gi->pNormals = (gi->flags & 0x10u) ? u32(mt + 0x18) : 0;
    if (!gi->pVerts || !mem_read(gi->pVerts, g_rPos, nv * 12u)) { g_why = "render vertices not readable"; return 0; }
    if (!mem_read(gi->pTri, g_io, nt * 8u)) { g_why = "render triangles not readable"; return 0; }
    for (i = 0; i < nt; i++) {
        for (k = 0; k < 3; k++) { U32 v = u16(g_io + i * 8 + k * 2); if (v >= nv) { g_why = "render triangle index out of range"; return 0; } g_rTri[i * 3 + k] = (U16)v; }
        g_rAux[i] = (U16)u16(g_io + i * 8 + 6);
        if (g_rAux[i] >= gi->nMat) { g_why = "render material index out of range"; return 0; }
    }
    memset(&g_rm, 0, sizeof(g_rm));
    g_rm.nv = nv; g_rm.nt = nt; g_rm.pos = g_rPos; g_rm.tri = g_rTri; g_rm.aux = g_rAux; g_rm.normalAttr = -1;
    g_rm.flag = g_rFlag; memset(g_rFlag, 0, nt);
    for (k = 0; k < gi->nSets; k++) {
        float* dst = k ? g_rUv1 : g_rUv0;
        if (!gi->pUv[k] || !mem_read(gi->pUv[k], dst, nv * 8u)) { g_why = "texture coordinates not readable"; return 0; }
        g_rm.fa[g_rm.nf] = dst; g_rm.fdim[g_rm.nf] = 2; g_rm.nf++;
    }
    if (gi->pNormals) {
        if (!mem_read(gi->pNormals, g_rNrm, nv * 12u)) { g_why = "normals not readable"; return 0; }
        g_rm.normalAttr = (int)g_rm.nf; g_rm.fa[g_rm.nf] = g_rNrm; g_rm.fdim[g_rm.nf] = 3; g_rm.nf++;
    }
    if ((gi->flags & 8u) && gi->pPre) {
        if (!mem_read(gi->pPre, g_rPre, nv * 4u)) { g_why = "vertex colours not readable"; return 0; }
        g_rm.ba[g_rm.nb++] = g_rPre;
    } else gi->pPre = 0;
    /* day/night vertex colours (geometry plugin of the building pipeline) */
    gi->vcOffset = 0; gi->pNight = gi->pDay = 0; gi->dnBalance = 0;
    if (mem_read(ADDR_EXTRA_VC_OFFSET, &off, 4) && off >= 0x60u && off < 0x400u && mem_read(geom + off, vc, 12)) {
        gi->vcOffset = off; gi->pNight = u32(vc); gi->pDay = u32(vc + 4); gi->dnBalance = u32(vc + 8);
        if (gi->pNight) { if (!mem_read(gi->pNight, g_rNight, nv * 4u)) { g_why = "night colours not readable"; return 0; } g_rm.ba[g_rm.nb++] = g_rNight; }
        if (gi->pDay) { if (!mem_read(gi->pDay, g_rDay, nv * 4u)) { g_why = "day colours not readable"; return 0; } g_rm.ba[g_rm.nb++] = g_rDay; }
    }
    return 1;
}

/* ---- rock material for crater interiors ---------------------------------------------------
 * One RpMaterial with a generated 128x128 texture, created on first use and kept for the session.
 * Returns 0 if the engine refuses; craters then keep the surrounding texture. */
static U32 g_rockMat; static int g_rockTried;
static U8 g_rockPix[ROCK_SIZE * ROCK_SIZE * 4u], g_rockMipA[ROCK_SIZE * ROCK_SIZE], g_rockMipB[ROCK_SIZE * ROCK_SIZE];
static U32 rock_raster(int withMips) {
    U8 r[0x34]; U32 raster, levels, lv, size = ROCK_SIZE, y; const U8* cur = g_rockPix; U8* next = g_rockMipA;
    raster = rw_raster_create(ROCK_SIZE, ROCK_SIZE, 32u, 0x04u | 0x0600u | (withMips ? 0x8000u : 0u));   /* texture, 888, mipmapped */
    if (!raster) return 0;
    if (!mem_read(raster, r, sizeof(r)) || u32(r + 0x14) != 32u) { rw_raster_destroy(raster); return 0; }
    levels = withMips ? rw_raster_num_levels(raster) : 1u;
    if (levels < 1u || levels > 8u) { rw_raster_destroy(raster); return 0; }
    for (lv = 0; lv < levels; lv++) {
        U32 px = rw_raster_lock(raster, lv, 1u | 4u), stride, ok = 1;        /* write, no fetch */
        if (!px) { rw_raster_destroy(raster); return 0; }
        if (!mem_read(raster, r, sizeof(r))) ok = 0;
        stride = u32(r + 0x18);
        if (ok && (u32(r + 0xC) != size || stride < size * 4u)) ok = 0;      /* RW reports the locked level's size */
        for (y = 0; ok && y < size; y++) ok = (U32)mem_write(px + y * stride, cur + y * size * 4u, size * 4u);
        rw_raster_unlock(raster);
        if (!ok) { rw_raster_destroy(raster); return 0; }
        if (size > 1u) { size = rock_halve(cur, size, next); cur = next; next = (next == g_rockMipA) ? g_rockMipB : g_rockMipA; }
    }
    return raster;
}
static U32 rock_material(void) {
    U32 raster, tex, mat, fa, zero = 0; int mips = 1;
    if (g_rockTried) return g_rockMat;
    g_rockTried = 1;
    stage("rock texture: generate");
    rock_generate(g_rockPix);
    stage("rock texture: raster");
    raster = rock_raster(1);
    if (!raster) { mips = 0; raster = rock_raster(0); }
    if (!raster) { logline("[v3.6.2] rock texture could not be created; craters keep the surrounding texture."); return 0; }
    stage("rock texture: texture + material");
    tex = rw_texture_create(raster);
    if (!tex) { rw_raster_destroy(raster); logline("[v3.6.2] RwTextureCreate failed; craters keep the surrounding texture."); return 0; }
    fa = (mips ? 6u : 2u) | 0x1100u;                 /* trilinear (or linear), wrap in u and v */
    mem_write(tex + 0x50u, &fa, 4);
    mat = rw_material_create();
    if (!mat) { logline("[v3.6.2] RpMaterialCreate failed; craters keep the surrounding texture."); return 0; }
    rw_material_set_texture(mat, tex);
    mem_write(mat + 0x10u, &zero, 4);                /* building pipeline keeps its per-material flags here: none */
    g_rockMat = mat;
    logline(mips ? "[v3.6.2] rock material ready (128x128, mipmapped)." : "[v3.6.2] rock material ready (128x128, no mipmaps).");
    return mat;
}
/* texture mapping of the bowl from world position, so it lines up across neighbouring objects.
 * M maps a model-space point to (u, v): planar projection along the dominant axis of the normal. */
static void rock_mapping(const Frame* f, Vec3 n, float* M) {
    Vec3 aU = { 1, 0, 0 }, aV = { 0, 1, 0 }; float k = 1.f / ROCK_TILE_M, o;
    if (!(ab(n.z) >= ab(n.x) && ab(n.z) >= ab(n.y))) {
        aV.y = 0; aV.z = 1;
        if (ab(n.x) >= ab(n.y)) { aU.x = 0; aU.y = 1; }
    }
    M[0] = k * (aU.x * f->c + aU.y * f->s); M[1] = k * (-aU.x * f->s + aU.y * f->c); M[2] = k * aU.z;
    o = k * gm_dot(aU, f->pos); M[3] = o - (float)(int)o;
    M[4] = k * (aV.x * f->c + aV.y * f->s); M[5] = k * (-aV.x * f->s + aV.y * f->c); M[6] = k * aV.z;
    o = k * gm_dot(aV, f->pos); M[7] = o - (float)(int)o;
}

/* Carve the crater into the render mesh: build a new RpGeometry and hand it to every atomic
 * that showed the old one. Returns 1 done, 0 untouched, -1 failed. */
static U32 g_statRv0, g_statRv1, g_statRt0, g_statRt1, g_statAtomics, g_statWeld;
#define WELD_EPS 0.08f            /* rock edges shorter than this are collapsed */
static U8 g_moved[GM_MAXV];
/* ---- breaking through thin things ---------------------------------------------------------
 * Found on the VISIBLE mesh (its winding can be trusted), then carried over to the collision. */
#define MAX_SHELL_TRIS 6000u
#define COL_AT_EPS 0.12f          /* the visible faces "at" a collision vertex: those within 12 cm */
static Vec3 g_shellTri[MAX_SHELL_TRIS * 3]; static float g_shellBox[MAX_SHELL_TRIS * 6]; static U32 g_nShellTri;
static const Target* g_hookT; static int g_shellNote;      /* note: 1 hollow inside, 2 nothing below, 3 blast behind the surface */
static float g_hookDepth;
static int g_skipObj;                                      /* the blast went off inside the material of a thing with a far side: leave it alone */
static int g_veto;                                         /* -1: none; else no hole after all, and what gm_unbreak mode the collision has to follow */
static U8 g_slab[20000]; static int g_hookRocky;           /* per model: has shown a far side; g_hookRocky: the mesh at hand already carries crater rock */                                  /* depth of the crater being made */
static int g_scanOk, g_anyClamp; static Vec3 g_scanC; static float g_scanR;    /* g_rm holds the visible mesh, refined and cut, for the collision to look at */
static void render_after_cut(GmMesh* m, Vec3 C, float R, Vec3 Q, GmStats* st) {
    U32 n, t; Vec3 dw; int nocull = 0;
    g_nShellTri = 0;
    g_anyClamp = 0; gm_sheet_mode = 0; g_veto = -1;
    { U8 point[0x2c]; U32 obj = 0; Vec3 ax, pb, pf, from, to; float rl = gm_sqrt(gm_dist2(Q, C)), sd = R - g_hookDepth;
      ax.x = (Q.x - C.x) / rl; ax.y = (Q.y - C.y) / rl; ax.z = (Q.z - C.z) / rl;
      pf.x = C.x - ax.x * (sd - 0.25f); pf.y = C.y - ax.y * (sd - 0.25f); pf.z = C.z - ax.z * (sd - 0.25f);      /* 25 cm in front of the surface */
      pb.x = C.x - ax.x * (sd + 0.3f); pb.y = C.y - ax.y * (sd + 0.3f); pb.z = C.z - ax.z * (sd + 0.3f);         /* 30 cm behind it */
      if (peek16(g_hookT->modelInfo + 0x12u) & 0x40u) {
          /* The blast itself must be in the open. If the surface shows it its back (the blast went off
           * under the road, inside a step), there is nothing to break through to: solid. */
          Vec3 back; back.x = -ax.x; back.y = -ax.y; back.z = -ax.z;
          if (gm_enclosed(m, pf, back, 0, 0)) {      /* (rock too: under a crater's floor the blast is in the ground) */
              g_shellNote = 3;
              /* in a thing with a far side (a slab riddled with holes) that means: inside its material. Nothing sensible can be carved from there. */
              if (g_hookRocky && g_slab[g_hookT->model]) g_skipObj = 1;
              return;
          }
      } else {
          /* Drawn without back-face culling: its faces are seen from both sides, so a single face is a
           * sheet - if there is real space on both sides of it. The test: is there ground somewhere
           * below? (Under the map there is none: terrain stays solid.) */
          int ok = 1, side;
          nocull = 1;
          for (side = 0; side < 2 && ok; side++) {
              from = to_world(&g_hookT->frame, side ? pf : pb); to = from; to.z -= 150.f;
              memset(point, 0, sizeof(point)); obj = 0;
              ok = game_line_of_sight(&from, &to, point, &obj);
          }
          if (ok) { gm_sheet_mode = 1; gm_sheet_axis = ax; }
      } }
    /* a thing with a far side stays one for as long as its changed mesh lives, even when holes have eaten the far side here */
    gm_rock_aware_known = g_hookRocky && g_slab[g_hookT->model];
    n = gm_break(m, C, R, Q, 0);
    g_slab[g_hookT->model] = (U8)gm_far_side_seen;
    for (t = 0; t < m->nv; t++) if (gm_clamp[t] >= 0.f) { g_anyClamp = 1; break; }
    if (!n) return;
    /* a hole only where there is a real space behind: not into a building that is just a skin ... */
    if (!nocull && gm_enclosed(m, gm_shell_p, gm_shell_d, 0, 1)) { g_veto = gm_thin_skin(m) ? 1 : 0; gm_unbreak(m, g_veto); g_shellNote = 1; return; }      /* a thin skin: pushed in; a thick part: the bowl stops on its far side */
    /* ... and not downwards into nothing (the underside of the map) */
    dw = dir_world(&g_hookT->frame, gm_shell_d);
    if (dw.z < -0.5f) {
        U8 point[0x2c]; U32 obj = 0; Vec3 from = to_world(&g_hookT->frame, gm_shell_p), to;
        to.x = from.x + dw.x * 150.f; to.y = from.y + dw.y * 150.f; to.z = from.z + dw.z * 150.f;
        memset(point, 0, sizeof(point));
        if (!game_line_of_sight(&from, &to, point, &obj)) { gm_unbreak(m, 2); g_veto = 2; g_shellNote = 2; return; }
    }
    st->shell = n;
    for (t = 0; t < m->nt && g_nShellTri < MAX_SHELL_TRIS; t++) if (m->flag[t] == 2) {
        Vec3 a = m->pos[m->tri[t * 3]], b = m->pos[m->tri[t * 3 + 1]], c = m->pos[m->tri[t * 3 + 2]]; float* bb = g_shellBox + g_nShellTri * 6u;
        g_shellTri[g_nShellTri * 3] = a; g_shellTri[g_nShellTri * 3 + 1] = b; g_shellTri[g_nShellTri * 3 + 2] = c;
        bb[0] = (a.x < b.x ? (a.x < c.x ? a.x : c.x) : (b.x < c.x ? b.x : c.x)) - 0.3f; bb[3] = (a.x > b.x ? (a.x > c.x ? a.x : c.x) : (b.x > c.x ? b.x : c.x)) + 0.3f;
        bb[1] = (a.y < b.y ? (a.y < c.y ? a.y : c.y) : (b.y < c.y ? b.y : c.y)) - 0.3f; bb[4] = (a.y > b.y ? (a.y > c.y ? a.y : c.y) : (b.y > c.y ? b.y : c.y)) + 0.3f;
        bb[2] = (a.z < b.z ? (a.z < c.z ? a.z : c.z) : (b.z < c.z ? b.z : c.z)) - 0.3f; bb[5] = (a.z > b.z ? (a.z > c.z ? a.z : c.z) : (b.z > c.z ? b.z : c.z)) + 0.3f;
        g_nShellTri++;
    }
}
/* collision: what lies on a broken-through part of the visible mesh goes too */
static void col_after_cut(GmMesh* m, Vec3 C, float R, Vec3 Q, GmStats* st) {
    U32 t, k;
    if (!g_scanOk || (!g_nShellTri && !g_anyClamp)) return;
    /* how far each collision vertex goes: ask the visible mesh. The faces "at" a collision vertex
     * are the visible triangles within 12 cm of it. */
    { U32 nn = gm_break_collect(&g_rm, C, R, Q, 0), v;
      for (v = 0; v < m->nv; v++) { gm_bnd[v] = 0; gm_seal[v] = 0; gm_next[v] = -1.f; gm_farv[v] = 0; if (gm_inside(m->pos[v], C, R, Q)) { float c = gm_break_point(&g_rm, nn, m->pos[v], COL_AT_EPS, C, Q); GM_KEEP(v, c); } }
      if (g_veto >= 0) { gm_unbreak(m, g_veto); return; }                 /* no hole in the visible mesh: none here, the vertices do what they do there */
      if (gm_break_split(m, &g_rm, &nn, COL_AT_EPS, C, R, Q, 0) < 0) { gm_break_failed = 1; return; } }
    for (t = 0; t < m->nt; t++) {
        Vec3 a, b, c, cen, n; float nl, w[3]; int c0, c1, c2;
        if (m->flag[t] || gm_far(m, t, C, R)) continue;
        a = m->pos[m->tri[t * 3]]; b = m->pos[m->tri[t * 3 + 1]]; c = m->pos[m->tri[t * 3 + 2]];
        c0 = gm_class(a, C, R); c1 = gm_class(b, C, R); c2 = gm_class(c, C, R);
        if (c0 < 0 || c1 < 0 || c2 < 0 || c0 + c1 + c2 == 0) continue;
        cen.x = (a.x + b.x + c.x) / 3.f; cen.y = (a.y + b.y + c.y) / 3.f; cen.z = (a.z + b.z + c.z) / 3.f;
        n = gm_cross(gm_sub(b, a), gm_sub(c, a)); nl = gm_sqrt(gm_dot(n, n));
        if (nl < 1e-12f) continue;
        for (k = 0; k < g_nShellTri; k++) {
            const float* bb = g_shellBox + k * 6u; Vec3 ha, hb, hc, hn, q; float hl, dp;
            if (cen.x < bb[0] || cen.x > bb[3] || cen.y < bb[1] || cen.y > bb[4] || cen.z < bb[2] || cen.z > bb[5]) continue;
            ha = g_shellTri[k * 3]; hb = g_shellTri[k * 3 + 1]; hc = g_shellTri[k * 3 + 2];
            hn = gm_cross(gm_sub(hb, ha), gm_sub(hc, ha)); hl = gm_sqrt(gm_dot(hn, hn));
            if (hl < 1e-12f) continue;
            dp = gm_dot(n, hn) / (nl * hl);
            if (dp > -0.7f && dp < 0.7f) continue;                                  /* the floor at the foot of a wall is not the wall */
            q = gm_closest_on_tri(cen, ha, hb, hc, w);
            if (gm_dist2(cen, q) < 0.3f * 0.3f) { m->flag[t] = 2; st->shell++; break; }
        }
    }
}

/* the visible mesh, read and with its sheets flagged: ready for gm_crater */
static int render_prepare(const Target* t, Vec3 C, float R, GeoInfo* pgi, U32* pRock, U32* pRockIdx) {
    GeoInfo gi; U32 old = t->geom, i, rock, rockIdx = 0xFFFFu;
    if (!read_render(old, &gi)) return 0;
    g_statRv0 = g_rm.nv; g_statRt0 = g_rm.nt;
    /* holes in the visible mesh: only what lies ON a collision sheet (the wire of a fence), not what
     * merely stands next to one. Nothing is judged thin by its shape: roofs are often two layers
     * back to back, and craters dug into each other fold the rock - both got holed that way. */
    gm_mark_thin(&g_rm, C, R, 0.f, 0, 0);
    /* crater rock already in this mesh? (the far-side rule must know it: rock is solid) */
    rock = gi.nSets >= 1u ? rock_material() : 0u;
    if (rock) {
        for (i = 0; i < gi.nMat; i++) { U32 mat = 0; if (mem_read(gi.pMat + i * 4u, &mat, 4) && mat == rock) rockIdx = i; }
    }
    gm_solid_aux = rockIdx != 0xFFFFu ? rockIdx : 0xFFFFFFFFu;
    g_hookRocky = rockIdx != 0xFFFFu;
    if (g_nHoleTri) {
        for (i = 0; i < g_rm.nt; i++) {
            Vec3 pnt, a, b, c, n; float w[3], nl; U32 k;
            if (g_rFlag[i] || gm_far(&g_rm, i, C, R)) continue;
            a = g_rPos[g_rTri[i * 3]]; b = g_rPos[g_rTri[i * 3 + 1]]; c = g_rPos[g_rTri[i * 3 + 2]];
            pnt = gm_closest_on_tri(C, a, b, c, w);
            if (gm_dist2(pnt, C) >= R * R) continue;
            n = gm_cross(gm_sub(b, a), gm_sub(c, a)); nl = gm_sqrt(gm_dot(n, n));
            if (nl < 1e-12f || n.z > 0.7f * nl) continue;                           /* what one can stand on is ground, not fence */
            for (k = 0; k < g_nHoleTri; k++) {
                Vec3 ha = g_holeTri[k * 3], hb = g_holeTri[k * 3 + 1], hc = g_holeTri[k * 3 + 2], hn = gm_cross(gm_sub(hb, ha), gm_sub(hc, ha)), q; float hl = gm_sqrt(gm_dot(hn, hn)), dp;
                if (hl < 1e-12f) continue;
                dp = gm_dot(n, hn) / (nl * hl);
                if (dp > -0.8f && dp < 0.8f) continue;                              /* not parallel to the sheet */
                if (ab(gm_dot(hn, gm_sub(a, ha))) > 0.2f * hl || ab(gm_dot(hn, gm_sub(b, ha))) > 0.2f * hl || ab(gm_dot(hn, gm_sub(c, ha))) > 0.2f * hl) continue;   /* not in its plane */
                q = gm_closest_on_tri(pnt, ha, hb, hc, w);
                if (gm_dist2(pnt, q) < 0.35f * 0.35f) break;
            }
            if (k == g_nHoleTri) continue;
            /* ... and all of it: each corner on the sheet too. A big ground triangle that merely
             * touches the foot of a fence is not part of the fence. */
            { Vec3 cor[3]; U32 j, on = 0; cor[0] = a; cor[1] = b; cor[2] = c;
              for (j = 0; j < 3u; j++) {
                  for (k = 0; k < g_nHoleTri; k++) {
                      Vec3 q = gm_closest_on_tri(cor[j], g_holeTri[k * 3], g_holeTri[k * 3 + 1], g_holeTri[k * 3 + 2], w);
                      if (gm_dist2(cor[j], q) < 0.35f * 0.35f) { on++; break; }
                  }
                  if (on != j + 1u) break;
              }
              if (on == 3u) g_rFlag[i] = 1; }
        }
    }
    *pgi = gi; *pRock = rock; *pRockIdx = rockIdx;
    return 1;
}
/* dry run on the visible mesh: where does this blast break through? Fills g_shellTri. */
static void shell_scan(const Target* t, Vec3 C, float R, Vec3 Q, float h) {
    GeoInfo gi; GmStats st; U32 rock, rockIdx;
    g_nShellTri = 0; g_shellNote = 0; g_scanOk = 0; g_anyClamp = 0;
    if (!render_prepare(t, C, R, &gi, &rock, &rockIdx)) return;
    g_hookT = t; gm_after_cut = render_after_cut; gm_stop_after_cut = 1;
    g_scanOk = gm_crater(&g_rm, C, R, Q, h, 1.f, 0, 0, &st) == 0;
    gm_after_cut = 0; gm_stop_after_cut = 0;
    g_scanC = C; g_scanR = R;
}

/* A plain copy of the geometry data that read_render has just put into the render buffers:
 * header (vertices, triangles, materials, which arrays, bounding sphere), then the arrays. */
#define SNAP_HDR 32u
static U32 snap_bits(const GeoInfo* gi) { return (gi->pPre ? 1u : 0u) | (gi->nSets << 1) | (gi->pNormals ? 8u : 0u) | (gi->pNight ? 16u : 0u) | (gi->pDay ? 32u : 0u); }
static U32 snap_size(U32 nv, U32 nt, U32 bits) {
    return SNAP_HDR + nv * 12u + nt * 8u + ((bits & 1u) ? nv * 4u : 0u) + ((bits >> 1) & 3u) * nv * 8u + ((bits & 8u) ? nv * 12u : 0u) + ((bits & 16u) ? nv * 4u : 0u) + ((bits & 32u) ? nv * 4u : 0u);
}
static void snapshot_take(U32 geom, const GeoInfo* gi) {
    U32 nv = g_rm.nv, nt = g_rm.nt, bits = snap_bits(gi), size = snap_size(nv, nt, bits), b, off, i; U8 hdr[SNAP_HDR]; int r;
    if (g_undoBytes + size > UNDO_MAX_BYTES) { if (!g_undoFull) { g_undoFull = 1; logline("[v3.6.2] very many objects changed: the ones changed from now on cannot be reset with F8 (they become whole again when the game loads them afresh)."); } return; }
    b = game_malloc(size);
    if (!b) return;
    put32(hdr, nv); put32(hdr + 4, nt); put32(hdr + 8, gi->nMat); put32(hdr + 12, bits);
    putf(hdr + 16, gi->sphC.x); putf(hdr + 20, gi->sphC.y); putf(hdr + 24, gi->sphC.z); putf(hdr + 28, gi->sphR);
    for (i = 0; i < nt; i++) { put16(g_io + i * 8, g_rTri[i * 3]); put16(g_io + i * 8 + 2, g_rTri[i * 3 + 1]); put16(g_io + i * 8 + 4, g_rTri[i * 3 + 2]); put16(g_io + i * 8 + 6, g_rAux[i]); }
    off = b + SNAP_HDR;
    r = mem_write(b, hdr, SNAP_HDR) && mem_write(off, g_rPos, nv * 12u); off += nv * 12u;
    r = r && mem_write(off, g_io, nt * 8u); off += nt * 8u;
    if (bits & 1u) { r = r && mem_write(off, g_rPre, nv * 4u); off += nv * 4u; }
    if (gi->nSets > 0) { r = r && mem_write(off, g_rUv0, nv * 8u); off += nv * 8u; }
    if (gi->nSets > 1) { r = r && mem_write(off, g_rUv1, nv * 8u); off += nv * 8u; }
    if (bits & 8u) { r = r && mem_write(off, g_rNrm, nv * 12u); off += nv * 12u; }
    if (bits & 16u) { r = r && mem_write(off, g_rNight, nv * 4u); off += nv * 4u; }
    if (bits & 32u) { r = r && mem_write(off, g_rDay, nv * 4u); off += nv * 4u; }
    if (!r) { game_free(b); return; }
    g_pendSnap = b; g_pendBytes = size; g_pendGeom = geom;
}
/* ... and back into the render buffers, over the geometry `gi` describes (the changed one, read by
 * read_render just before). Returns the number of original materials, 0 if the copy does not fit. */
static U32 snapshot_load(U32 b, GeoInfo* gi) {
    U8 hdr[SNAP_HDR]; U32 nv, nt, nMat, bits, off, i; int r;
    if (!mem_read(b, hdr, SNAP_HDR)) return 0;
    nv = u32(hdr); nt = u32(hdr + 4); nMat = u32(hdr + 8); bits = u32(hdr + 12);
    if (bits != snap_bits(gi) || !nMat || nMat > gi->nMat || nv < 3u || nv > MAX_SRC_V || !nt || nt > MAX_SRC_T) return 0;
    off = b + SNAP_HDR;
    r = mem_read(off, g_rPos, nv * 12u); off += nv * 12u;
    r = r && mem_read(off, g_io, nt * 8u); off += nt * 8u;
    if (bits & 1u) { r = r && mem_read(off, g_rPre, nv * 4u); off += nv * 4u; }
    if (gi->nSets > 0) { r = r && mem_read(off, g_rUv0, nv * 8u); off += nv * 8u; }
    if (gi->nSets > 1) { r = r && mem_read(off, g_rUv1, nv * 8u); off += nv * 8u; }
    if (bits & 8u) { r = r && mem_read(off, g_rNrm, nv * 12u); off += nv * 12u; }
    if (bits & 16u) { r = r && mem_read(off, g_rNight, nv * 4u); off += nv * 4u; }
    if (bits & 32u) { r = r && mem_read(off, g_rDay, nv * 4u); off += nv * 4u; }
    if (!r) return 0;
    for (i = 0; i < nt; i++) {
        U32 k; for (k = 0; k < 3u; k++) { U32 v = u16(g_io + i * 8 + k * 2); if (v >= nv) return 0; g_rTri[i * 3 + k] = (U16)v; }
        g_rAux[i] = (U16)u16(g_io + i * 8 + 6);
        if (g_rAux[i] >= nMat) return 0;
    }
    g_rm.nv = nv; g_rm.nt = nt;
    gi->sphC.x = f32(hdr + 16); gi->sphC.y = f32(hdr + 20); gi->sphC.z = f32(hdr + 24); gi->sphR = f32(hdr + 28);
    return nMat;
}
static int write_geometry(const Target* t, const GeoInfo* pgi, U32 nMat, U32 rock, int restoring);
static int apply_render(const Target* t, Vec3 C, float R, Vec3 Q, float h, float depth, const float* uvM) {
    GeoInfo gi; GmStats st; U32 i, rock = 0, rockIdx = 0xFFFFu, appendRock = 0; int r;
    (void)depth; (void)i;
    if (g_pendSnap) { game_free(g_pendSnap); g_pendSnap = 0; }
    if (!render_prepare(t, C, R, &gi, &rock, &rockIdx)) return -1;
    { Undo* u = undo_find(t->model);       /* first change of this geometry: copy it while the buffers still hold the original */
      if (u ? (!u->snap || u->curGeom != t->geom) : g_nUndo < MAX_UNDO) snapshot_take(t->geom, &gi); }
    g_hookT = t; gm_after_cut = render_after_cut;
    r = gm_crater(&g_rm, C, R, Q, h, 1.f, 0, g_moved, &st);
    gm_after_cut = 0;
#ifdef HOST_TEST
    if (r > 0) for (i = 0; i < g_rm.nv && g_dbgStayN < 16000u; i++) if (gm_clamp[i] >= 0.f) g_dbgStay[g_dbgStayN++] = g_rPos[i];
#endif
    if (r < 0) { g_why = FULL_TEXT; return -1; }
    if (r == 0) return 0;
    g_statShell = st.shell < st.holeTris ? st.shell : st.holeTris; g_statHoleR = st.holeTris - g_statShell;
    /* crater interior: rock material (reuse its slot if this geometry already has one) */
    if (rock) {
        if (rockIdx == 0xFFFFu && gi.nMat < 255u) { rockIdx = gi.nMat; appendRock = 1; }
        if (rockIdx != 0xFFFFu && gm_bowl_material(&g_rm, g_moved, (U16)rockIdx, 0, uvM, &st) < 0) { g_why = FULL_TEXT; return -1; }
    }
    if (!st.bowlTris) appendRock = 0;           /* only holes: no rock surface needed */
    if (rockIdx != 0xFFFFu) g_statWeld = gm_weld_short(&g_rm, C, R, rockIdx, WELD_EPS);
    gm_drop_degenerate(&g_rm);
    gm_compact(&g_rm);
    if (g_rm.nv > g_meshLimit || g_rm.nt > g_meshLimit) { g_why = FULL_TEXT; return -1; }
    if (g_rm.nt < 1) return 2;                  /* nothing left of it */
    return write_geometry(t, &gi, gi.nMat, appendRock ? rock : 0u, 0);
}
/* The mesh in the render buffers becomes a new geometry in place of t->geom: same vertex format, the
 * first nMat materials of the old one (plus rock), swapped in on the model and all its instances.
 * restoring: the buffers hold the original again (F8). Returns 1 done, -1 failed. */
static int write_geometry(const Target* t, const GeoInfo* pgi, U32 nMat, U32 rock, int restoring) {
    const GeoInfo gi = *pgi; U8 g[0x60], mt[0x1C]; U32 old = t->geom, ng, i, format, morph, pVerts, pool, cap, storage, flagsArr, swapped = 0, miAtomic = 0, night = 0, day = 0;
    float r2; int r; const int appendRock = rock != 0u;
    /* same vertex format, plain triangle list (no strips) */
    format = (gi.flags & 0xFEu) | (gi.nSets << 16);
    ng = rw_geom_create(g_rm.nv, g_rm.nt, format);
    if (!ng || !mem_read(ng, g, sizeof(g))) { g_why = "RpGeometryCreate failed"; return -1; }
    morph = u32(g + 0x5C);
    if (u32(g + 0x14) != g_rm.nv || u32(g + 0x10) != g_rm.nt || u32(g + 0x18) != 1u || !u32(g + 0x2C) || !morph ||
        !mem_read(morph, mt, sizeof(mt)) || !(pVerts = u32(mt + 0x14)) ||
        (gi.pPre && !u32(g + 0x30)) || (gi.nSets > 0 && !u32(g + 0x34)) || (gi.nSets > 1 && !u32(g + 0x38)) || (gi.pNormals && !u32(mt + 0x18))) {
        rw_geom_destroy(ng); g_why = "new geometry does not have the expected layout"; return -1;
    }
    /* materials in the same order -> triangle material indices stay valid */
    for (i = 0; i < nMat; i++) {
        U32 mat = 0;
        if (!mem_read(gi.pMat + i * 4u, &mat, 4) || !mat || rw_matlist_append(ng + 0x20u, mat) != (int)i) { rw_geom_destroy(ng); g_why = "material list could not be copied"; return -1; }
    }
    if (appendRock && rw_matlist_append(ng + 0x20u, rock) != (int)nMat) { rw_geom_destroy(ng); g_why = "rock material could not be added"; return -1; }
    for (i = 0; i < g_rm.nt; i++) {
        put16(g_io + i * 8, g_rTri[i * 3]); put16(g_io + i * 8 + 2, g_rTri[i * 3 + 1]); put16(g_io + i * 8 + 4, g_rTri[i * 3 + 2]);
        put16(g_io + i * 8 + 6, g_rAux[i]);
    }
    r = mem_write(u32(g + 0x2C), g_io, g_rm.nt * 8u) && mem_write(pVerts, g_rPos, g_rm.nv * 12u);
    if (r && gi.pPre) r = mem_write(u32(g + 0x30), g_rPre, g_rm.nv * 4u);
    if (r && gi.nSets > 0) r = mem_write(u32(g + 0x34), g_rUv0, g_rm.nv * 8u);
    if (r && gi.nSets > 1) r = mem_write(u32(g + 0x38), g_rUv1, g_rm.nv * 8u);
    if (r && gi.pNormals) r = mem_write(u32(mt + 0x18), g_rNrm, g_rm.nv * 12u);
    if (r && gi.vcOffset && (gi.pNight || gi.pDay)) {
        /* the plugin's destructor frees these with CMemoryMgr::Free, so they must come from CMemoryMgr::Malloc */
        U8 vc[12];
        if (gi.pNight) { night = game_malloc(g_rm.nv * 4u); r = night && mem_write(night, g_rNight, g_rm.nv * 4u); }
        if (r && gi.pDay) { day = game_malloc(g_rm.nv * 4u); r = day && mem_write(day, g_rDay, g_rm.nv * 4u); }
        put32(vc, night); put32(vc + 4, day); put32(vc + 8, gi.dnBalance);
        if (!r || !mem_write(ng + gi.vcOffset, vc, 12)) {
            /* not attached: free them ourselves and make sure the destructor sees nothing */
            U8 zero[12]; memset(zero, 0, 12); mem_write(ng + gi.vcOffset, zero, 12);
            if (night) game_free(night); if (day) game_free(day); r = 0;
        }
    }
    if (r) {    /* bounding sphere: old centre, radius grown to contain the bowl */
        r2 = gi.sphR * gi.sphR;
        for (i = 0; i < g_rm.nv && !restoring; i++) { float d2 = gm_dist2(g_rPos[i], gi.sphC); if (d2 > r2) r2 = d2; }      /* (restoring: the sphere the model came with, as it was) */
        putf(mt + 4, gi.sphC.x); putf(mt + 8, gi.sphC.y); putf(mt + 0xC, gi.sphC.z); putf(mt + 0x10, r2 > gi.sphR * gi.sphR ? gm_sqrt(r2) + 0.05f : gi.sphR);
        r = mem_write(morph + 4u, mt + 4, 16);
    }
    if (!r) { rw_geom_destroy(ng); g_why = "new geometry not writable"; return -1; }
    rw_d3d9_set_usage(ng, rw_d3d9_get_usage(old));
    if (rw_geom_unlock(ng) != ng || !mem_read(ng + 0x54u, &i, 4) || !i) { rw_geom_destroy(ng); g_why = "RpGeometryUnlock built no mesh"; return -1; }
    /* 2D effects (lights, particle emitters) hang on the old geometry and are counted in the model
     * info. Hand the block over to the new geometry so nothing is lost and the count stays right. */
    {
        U32 off2 = 0, fx = 0, cur = 0, zero = 0;
        if (mem_read(ADDR_2DFX_OFFSET, &off2, 4) && off2 >= 0x60u && off2 < 0x400u && mem_read(old + off2, &fx, 4) && fx &&
            mem_read(ng + off2, &cur, 4) && !cur && mem_write(ng + off2, &fx, 4)) mem_write(old + off2, &zero, 4);
    }
    if (!restoring) {      /* the copy of the original, taken before the first change, is filed for F8 */
        Undo* u = undo_for(t->model);
        if (u) {
            if (u->snap && u->curGeom != old) { game_free(u->snap); g_undoBytes -= u->snapBytes; u->snap = 0; }      /* loaded afresh since */
            if (!u->snap && g_pendSnap && g_pendGeom == old) { u->snap = g_pendSnap; u->snapBytes = g_pendBytes; g_undoBytes += g_pendBytes; g_pendSnap = 0; }
            u->curGeom = ng;
        }
    }
    /* swap: the model's master atomic (new instances are cloned from it) and every placed instance */
    miAtomic = peek32(t->modelInfo + 0x1Cu);
    if (miAtomic && peek8(miAtomic) == 1u && peek32(miAtomic + 0x18u) == old) { rw_atomic_set_geometry(miAtomic, ng); swapped++; }
    pool = peek32(ADDR_BUILDING_POOL);
    if (pool) {
        storage = peek32(pool); flagsArr = peek32(pool + 4u); cap = peek32(pool + 8u);
        for (i = 0; storage && flagsArr && i < cap && i < 100000u; i++) {
            U32 ent = storage + i * BUILDING_SIZE, rw;
            if (peek8(flagsArr + i) & 0x80u) continue;
            rw = peek32(ent + 0x18u);
            if (rw && rw != miAtomic && peek8(rw) == 1u && peek32(rw + 0x18u) == old) { rw_atomic_set_geometry(rw, ng); swapped++; }
        }
    }
    rw_geom_destroy(ng);        /* drop our creation reference; the atomics keep it alive */
    g_statRv1 = g_rm.nv; g_statRt1 = g_rm.nt; g_statAtomics = swapped;
    return 1;
}

/* ---- crater parameters per explosion type ---------------------------------------------- */
typedef struct { float r, d, reach; } CraterSpec;      /* rim radius, depth, max distance to a surface */
static const CraterSpec g_spec[13] = {
    { 2.2f, 0.9f, 2.5f },   /*  0 grenade / satchel */
    { 0.0f, 0.0f, 0.0f },   /*  1 molotov: fire only */
    { 2.8f, 1.2f, 2.5f },   /*  2 rocket            */
    { 2.0f, 0.8f, 2.0f },   /*  3 weak rocket       */
    { 3.0f, 1.0f, 3.0f },   /*  4 car               */
    { 3.0f, 1.0f, 3.0f },   /*  5 car (quick)       */
    { 3.0f, 1.0f, 3.0f },   /*  6 boat              */
    { 4.0f, 1.5f, 3.5f },   /*  7 aircraft          */
    { 2.5f, 1.0f, 2.5f },   /*  8 mine              */
    { 2.2f, 0.8f, 2.5f },   /*  9 object (barrel)   */
    { 2.8f, 1.2f, 2.5f },   /* 10 tank shell        */
    { 1.2f, 0.4f, 1.5f },   /* 11 small             */
    { 1.5f, 0.5f, 1.5f }    /* 12 RC vehicle        */
};

/* static world objects of the outside world whose collision bounds reach the blast */
static U32 find_candidates(Vec3 E, float range, U32* out) {
    U32 pool = peek32(ADDR_BUILDING_POOL), storage, flagsArr, cap, i, n = 0;
    if (!pool) return 0;
    storage = peek32(pool); flagsArr = peek32(pool + 4u); cap = peek32(pool + 8u);
    if (!storage || !flagsArr || cap > 100000u) return 0;
    for (i = 0; i < cap && n < MAX_CAND; i++) {
        U32 ent = storage + i * BUILDING_SIZE, model, mi, cm, mat, area; Vec3 p, bc, l; float br, dx, dy, c, s;
        if (peek8(flagsArr + i) & 0x80u) continue;                      /* empty slot */
        if (!peek32(ent + 0x18u)) continue;                             /* nothing rendered: not streamed in */
        if ((peek32(ent + 0x1Cu) & 0x81u) != 0x81u) continue;           /* hidden or without collision (e.g. a tree we already knocked over) */
        area = peek8(ent + 0x2Fu);
        if (area != 0u && area != 13u) continue;                        /* belongs to an interior */
        model = peek16(ent + 0x22u);
        if (model < 1u || model > 19999u) continue;
        mi = peek32(ADDR_MODEL_INFO_PTRS + model * 4u);
        if (!mi) continue;
        cm = peek32(mi + 0x14u);
        if (!cm || !peek32(cm + 0x2Cu)) continue;                       /* no collision loaded */
        mat = peek32(ent + 0x14u);
        if (mat) { p.x = peekf(mat + 0x30u); p.y = peekf(mat + 0x34u); p.z = peekf(mat + 0x38u); c = peekf(mat); s = peekf(mat + 4u); }
        else { p.x = peekf(ent + 4u); p.y = peekf(ent + 8u); p.z = peekf(ent + 0xCu); sincos_f(peekf(ent + 0x10u), &s, &c); }
        /* blast position in the object's own space (rotation around Z; tilted objects are refused later) */
        dx = E.x - p.x; dy = E.y - p.y;
        l.x = dx * c + dy * s; l.y = -dx * s + dy * c; l.z = E.z - p.z;
        bc.x = peekf(cm + 0x18u); bc.y = peekf(cm + 0x1Cu); bc.z = peekf(cm + 0x20u); br = peekf(cm + 0x24u) + range;
        if (!(gm_dist2(l, bc) < br * br)) continue;                     /* outside the bounding sphere */
        if (l.x < peekf(cm) - range || l.y < peekf(cm + 4u) - range || l.z < peekf(cm + 8u) - range ||
            l.x > peekf(cm + 0xCu) + range || l.y > peekf(cm + 0x10u) + range || l.z > peekf(cm + 0x14u) + range) continue;   /* outside the box */
        out[n++] = ent;
    }
    return n;
}

static U32 g_craters;
static Vec3 g_lastC, g_lastQ; static float g_lastR;      /* last crater sphere, world space (diagnostics / tests) */
static float g_lastDist, g_lastClarity;   /* clarity: 1 = all surfaces around the blast agree on the direction, 0 = they cancel */
static int g_propsEnabled = 1;                           /* F7: knock over trees/poles as physics objects */
static void log_skip(U32 ent, const char* part) {
    char m[200]; int p = cat(m, 0, 200, "[v3.6.2]   model="); p = dec(m, p, 200, peek16(ent + 0x22u));
    p = cat(m, p, 200, " skipped ("); p = cat(m, p, 200, part); p = cat(m, p, 200, "): "); cat(m, p, 200, g_why); logline(m);
}

/* ---- what kind of object is it? ------------------------------------------------------------
 * Every placed copy of a model shares one geometry and one collision. Carving a model that is
 * placed several times would change all copies at once, so such models are never carved.
 * Small repeated or tall-thin objects (trees, poles) count as props: a blast knocks them over. */
typedef struct { U32 instances, special; int prop, shared; float wide, tall, base; } Kind;
static U32 count_instances(U32 model) {
    U32 pool = peek32(ADDR_BUILDING_POOL), storage, flagsArr, cap, i, n = 0;
    if (!pool) return 0;
    storage = peek32(pool); flagsArr = peek32(pool + 4u); cap = peek32(pool + 8u);
    for (i = 0; storage && flagsArr && i < cap && i < 100000u; i++)
        if (!(peek8(flagsArr + i) & 0x80u) && peek16(storage + i * BUILDING_SIZE + 0x22u) == model && peek32(storage + i * BUILDING_SIZE + 0x18u)) n++;
    return n;
}
/* What is knocked over instead of carved:
 *   - everything the game itself marks as tree or palm (model flag "sways in the wind")
 *   - what is tall and thin where it stands (poles, masts, and trees: their collision is the trunk)
 *   - small things that stand around in several copies (they share one mesh, carving one would
 *     deform them all)
 * Larger models placed several times are left alone for the same reason.
 * Measured on the collision SHAPES (read_col must have run for this object), not on the model's
 * bounding box: the box of a palm includes its crown, 16 m wide. */
static void classify(const Target* t, const ColInfo* ci, Kind* k) {
    Vec3 lo = { 1e30f, 1e30f, 1e30f }, hi = { -1e30f, -1e30f, -1e30f }; float bx0 = 1e30f, bx1 = -1e30f, by0 = 1e30f, by1 = -1e30f, zcut; U32 i, pass; int tallThin, veg;
    for (pass = 0; pass < 2u; pass++) {
        zcut = lo.z + 0.4f * (hi.z - lo.z);
        for (i = 0; i < g_cm.nv; i++) {
            Vec3 p = g_cm.pos[i];
            if (!pass) { if (p.x < lo.x) lo.x = p.x; if (p.y < lo.y) lo.y = p.y; if (p.z < lo.z) lo.z = p.z; if (p.x > hi.x) hi.x = p.x; if (p.y > hi.y) hi.y = p.y; if (p.z > hi.z) hi.z = p.z; }
            else if (p.z <= zcut) { if (p.x < bx0) bx0 = p.x; if (p.x > bx1) bx1 = p.x; if (p.y < by0) by0 = p.y; if (p.y > by1) by1 = p.y; }
        }
        for (i = 0; i < ci->nSpheres && i < 64u; i++) {
            U32 a = ci->pSpheres + i * 0x14u; float x = peekf(a), y = peekf(a + 4u), z = peekf(a + 8u), r = peekf(a + 12u);
            if (!(r >= 0.f && r < 200.f)) continue;
            if (!pass) { if (x - r < lo.x) lo.x = x - r; if (y - r < lo.y) lo.y = y - r; if (z - r < lo.z) lo.z = z - r; if (x + r > hi.x) hi.x = x + r; if (y + r > hi.y) hi.y = y + r; if (z + r > hi.z) hi.z = z + r; }
            else if (z - r <= zcut) { if (x - r < bx0) bx0 = x - r; if (x + r > bx1) bx1 = x + r; if (y - r < by0) by0 = y - r; if (y + r > by1) by1 = y + r; }
        }
        if (!pass && !(hi.z >= lo.z)) {        /* no shapes at all: fall back to the bounding box */
            U32 cm = t->colModel; lo.x = peekf(cm); lo.y = peekf(cm + 4u); lo.z = peekf(cm + 8u); hi.x = peekf(cm + 0xCu); hi.y = peekf(cm + 0x10u); hi.z = peekf(cm + 0x14u);
            bx0 = lo.x; bx1 = hi.x; by0 = lo.y; by1 = hi.y; break;
        }
    }
    k->wide = (hi.x - lo.x) > (hi.y - lo.y) ? (hi.x - lo.x) : (hi.y - lo.y);
    k->base = (bx1 - bx0) > (by1 - by0) ? (bx1 - bx0) : (by1 - by0);
    k->tall = hi.z - lo.z;
    k->special = (peek16(t->modelInfo + 0x12u) >> 11) & 0xFu;      /* CBaseModelInfo::nSpecialType: 1 tree, 2 palm */
    veg = k->special == 1u || k->special == 2u;
    /* thin where it stands AND not wide anywhere: a piece of land with one deep corner is not a pole */
    tallThin = k->tall > 2.f && k->base < PROP_THIN_WIDTH && k->tall > 1.5f * k->base && k->wide < PROP_MAX_WIDTH + 2.f;
    if (peek16(t->modelInfo + 0x12u) & 0x100u) { tallThin = 0; veg = 0; }      /* the game calls it a road */
    k->instances = count_instances(t->model);
    k->prop = veg || tallThin || (k->instances >= 2u && k->wide < PROP_MAX_WIDTH && !(peek16(t->modelInfo + 0x12u) & 0x100u));
    k->shared = !k->prop && k->instances >= 2u;
}
/* make a static world object disappear: no collision, not drawn; its low-detail stand-in too */
#define MAX_HIDDEN 256u
typedef struct { U32 ent, model, flags, lodFlags; } Hidden;
static Hidden g_hidden[MAX_HIDDEN]; static U32 g_nHidden;
#define MAX_FALLEN 128u
typedef struct { U32 obj, model, removeAt; } Fallen;
static Fallen g_fallen[MAX_FALLEN]; static U32 g_nFallen;
static void hide_entity(U32 ent) {
    U32 f = 0, lod = 0;
    if (g_nHidden < MAX_HIDDEN && mem_read(ent + 0x1Cu, &f, 4)) {      /* remembered for F8 */
        Hidden* h = &g_hidden[g_nHidden++]; U32 lf = 0;
        h->ent = ent; h->model = peek16(ent + 0x22u); h->flags = f;
        h->lodFlags = mem_read(ent + 0x30u, &lod, 4) && lod && mem_read(lod + 0x1Cu, &lf, 4) ? lf : 0u;
        lod = 0;
    }
    if (mem_read(ent + 0x1Cu, &f, 4)) { f &= ~0x81u; mem_write(ent + 0x1Cu, &f, 4); }
    if (mem_read(ent + 0x30u, &lod, 4) && lod && mem_read(lod + 0x1Cu, &f, 4)) { f &= ~0x80u; mem_write(lod + 0x1Cu, &f, 4); }
}
/* Knock a prop over: the static object is hidden and a physics object of the same model takes
 * its place. A moving object only collides with the world through collision SPHERES, so the
 * model's collision gets a column of spheres first if it has none. Returns 1 done, -1 failed. */
static int topple(const Target* t, Vec3 E) {
    U8 cm[0x30], mtx[0x40]; Vec3 lo, hi, away, com, v; float rad, len, c = t->frame.c, s = t->frame.s, f; U32 obj, mat, n = 0, i, u, added = 0; U16 cnt = 0;
    char m[200]; int p;
    if (!mem_read(t->colModel, cm, sizeof(cm))) { g_why = "collision model not readable"; return -1; }
    lo.x = f32(cm); lo.y = f32(cm + 4); lo.z = f32(cm + 8); hi.x = f32(cm + 0xC); hi.y = f32(cm + 0x10); hi.z = f32(cm + 0x14);
    if (!read_col(t, &g_ci)) return -1;
    if (g_ci.nSpheres == 0u) {
        /* the trunk: where the lower half of the collision mesh stands, and how thick it is there.
         * (All copies of the model share these spheres - they must not be fatter than the trunk.) */
        float ax = 0.f, ay = 0.f, zmid = 0.5f * (lo.z + hi.z); U32 cntLow = 0;
        for (i = 0; i < g_cm.nv; i++) if (g_cm.pos[i].z <= zmid) { ax += g_cm.pos[i].x; ay += g_cm.pos[i].y; cntLow++; }
        rad = 0.f;
        if (cntLow) {
            ax /= (float)cntLow; ay /= (float)cntLow;
            for (i = 0; i < g_cm.nv; i++) if (g_cm.pos[i].z <= zmid) {
                float dx = g_cm.pos[i].x - ax, dy = g_cm.pos[i].y - ay, d = gm_sqrt(dx * dx + dy * dy);
                if (d > rad) rad = d;
            }
        } else { ax = 0.5f * (lo.x + hi.x); ay = 0.5f * (lo.y + hi.y); rad = 0.5f * ((hi.x - lo.x) < (hi.y - lo.y) ? (hi.x - lo.x) : (hi.y - lo.y)); }
        if (rad < 0.3f) rad = 0.3f; if (rad > 1.2f) rad = 1.2f;
        n = (U32)((hi.z - lo.z) / (1.6f * rad)) + 1u; if (n > 8u) n = 8u; if (n < 1u) n = 1u;
        for (i = 0; i < n; i++) {
            U8* sp = g_extra + i * 0x14u; float z = n > 1u ? lo.z + rad + (hi.z - lo.z - 2.f * rad) * (float)i / (float)(n - 1u) : 0.5f * (lo.z + hi.z);
            putf(sp, ax); putf(sp + 4, ay); putf(sp + 8, z); putf(sp + 12, rad);
            sp[16] = (U8)(g_cAux[0] & 0xFFu); sp[17] = 0; sp[18] = (U8)(g_cAux[0] >> 8); sp[19] = 0;
        }
        if (g_ci.nLines && !mem_read(g_ci.pLines, g_extra + n * 0x14u, g_ci.nLines * 0x20u)) { g_why = "collision lines not readable"; return -1; }
        g_ci.nSpheres = n; g_ci.extraBytes = n * 0x14u + g_ci.nLines * 0x20u;
        gm_drop_degenerate(&g_cm);
        if (g_cm.nt < 1u) { g_why = "collision mesh empty"; return -1; }
        g_nGroups = g_cm.nt > 80u ? build_groups(&g_cm) : 0u;
        g_undoOff = 1;             /* (spheres added to a prop's collision are not a crater: nothing to take back) */
        if (commit_col(t) < 0) { g_undoOff = 0; return -1; }
        g_undoOff = 0;
        added = n;
    }
    stage("prop: create object");
    obj = game_object_create(t->model);
    if (!obj || !mem_read(obj + 0x14u, &mat, 4) || !mat) { g_why = "CObject::Create gave no usable object"; return -1; }
    /* same place and heading as the static object */
    memset(mtx, 0, sizeof(mtx));
    putf(mtx, c); putf(mtx + 4, s); putf(mtx + 0x10, -s); putf(mtx + 0x14, c); putf(mtx + 0x28, 1.f);
    putf(mtx + 0x30, t->frame.pos.x); putf(mtx + 0x34, t->frame.pos.y); putf(mtx + 0x38, t->frame.pos.z);
    if (!mem_write(mat, mtx, 0x3Cu)) { g_why = "object matrix not writable"; return -1; }
    /* a light, free object like the game's own flying car parts */
    f = 400.f; mem_write(obj + 0x8Cu, &f, 4);
    f = 400.f * ((hi.z - lo.z) * (hi.z - lo.z) / 12.f + 1.f); mem_write(obj + 0x90u, &f, 4);        /* turn mass ~ m*L^2/12 */
    f = 0.97f; mem_write(obj + 0x98u, &f, 4); f = 0.1f; mem_write(obj + 0x9Cu, &f, 4);
    com.x = 0.5f * (lo.x + hi.x); com.y = 0.5f * (lo.y + hi.y); com.z = lo.z + 0.45f * (hi.z - lo.z); mem_write(obj + 0xA4u, &com, 12);
    u = peek32(obj + 0x40u); u |= 0x2u | 0x8u; u &= ~(0x4u | 0x40u | 0x800000u); mem_write(obj + 0x40u, &u, 4);   /* gravity, collidable; forces allowed */
    u = peek32(obj + 0x1Cu); u &= ~0x4u; mem_write(obj + 0x1Cu, &u, 4);                                         /* not static */
    { U8 type = 3; mem_write(obj + 0x13Cu, &type, 1); }                                                          /* temporary: the game removes it later */
    u = peek32(ADDR_TIME_MS) + 120000u; mem_write(obj + 0x150u, &u, 4);
    if (g_nFallen < MAX_FALLEN) { g_fallen[g_nFallen].obj = obj; g_fallen[g_nFallen].model = t->model; g_fallen[g_nFallen].removeAt = u; g_nFallen++; }
    cnt = (U16)(peek16(ADDR_NUM_TEMP_OBJECTS) + 1u); mem_write(ADDR_NUM_TEMP_OBJECTS, &cnt, 2);
    stage("prop: add to world");
    game_world_add(obj);
    /* push it away from the blast and start it tipping over */
    away.x = t->frame.pos.x - E.x; away.y = t->frame.pos.y - E.y; away.z = 0.f; len = gm_sqrt(away.x * away.x + away.y * away.y);
    if (len < 0.05f) { away.x = 1.f; away.y = 0.f; } else { away.x /= len; away.y /= len; }
    v.x = away.x * 0.04f; v.y = away.y * 0.04f; v.z = 0.02f; mem_write(obj + 0x44u, &v, 12);
    f = hi.z - lo.z > 4.f ? 0.12f / (hi.z - lo.z) : 0.03f;       /* tall things start tipping slowly: the top of a 30 m palm must not whip round */
    v.x = -away.y * f; v.y = away.x * f; v.z = 0.f; mem_write(obj + 0x50u, &v, 12);
    hide_entity(t->entity);
    p = cat(m, 0, 200, "[v3.6.2]   model="); p = dec(m, p, 200, t->model); p = cat(m, p, 200, " knocked over as physics object");
    if (added) { p = cat(m, p, 200, ", collision spheres added="); p = dec(m, p, 200, added); }
    logline(m);
    return 1;
}

/* One blast at world position E. */
/* Is the crater about to be dug from the wrong side? The wreck of a car blows up with its centre a
 * little under the road, a grenade sinks into a kerb: the blast is IN the ground, the visible surface
 * at S shows it its back, and a crater dug from there comes out as a mound. Three things must hold:
 *   - the visible surface at S shows its back to the side n (the side of the blast);
 *   - the blast sits in the material: towards the surface it meets backs, away from it nothing that
 *     looks at it (in a hole dug earlier - a shaft through a bridge - it would meet the hole's walls);
 *   - on the other side of the surface is the open, not the inside of a building that is only a skin. */
static int blast_behind(const Target* t, Vec3 S, Vec3 n, Vec3 E, float R, float depth) {
    GeoInfo gi; U32 rock, rockIdx; Vec3 Sl = to_local(&t->frame, S), El = to_local(&t->frame, E), nl, q, back; float l; int side;
    nl = to_local(&t->frame, (Vec3){ S.x + n.x, S.y + n.y, S.z + n.z }); nl = gm_sub(nl, Sl); l = gm_sqrt(gm_dot(nl, nl));
    if (l < 1e-6f) return 0;
    nl.x /= l; nl.y /= l; nl.z /= l;
    g_why = "";
    if (!render_prepare(t, Sl, R, &gi, &rock, &rockIdx)) return 0;
    side = gm_side_at(&g_rm, Sl, nl, 0.4f);
    if (side == 0 || side == 1) return 0;
    back.x = -nl.x; back.y = -nl.y; back.z = -nl.z;
    if (!gm_enclosed(&g_rm, El, back, 0, 0)) return 0;
    gm_enclosed(&g_rm, El, nl, 0, 0);
    if (gm_enc_nf) return 0;
    l = 2.f * R - depth;
    q.x = Sl.x - nl.x * l; q.y = Sl.y - nl.y * l; q.z = Sl.z - nl.z * l;
    return !gm_enclosed(&g_rm, q, nl, 0, 0);
}
#define MAX_MARKS 256u
typedef struct { Vec3 c; float r; } Mark;
static Mark g_mark[MAX_MARKS]; static U32 g_nMark;      /* where craters were dug (for F8: is the player standing in one?) */
static void blast(Vec3 E, U32 type) {
    const CraterSpec* sp; U32 ents[MAX_CAND], n, i, done = 0; Target tg[MAX_CAND]; int ok[MAX_CAND]; Kind kind[MAX_CAND];
    Vec3 S = { 0, 0, 0 }, nsum = { 0, 0, 0 }, nrm, C, Q, cam = { 0, 0, 0 }, camL; float best = 1e30f, len, R, h, dist, scale, rim, depth; char m[300]; int p, haveCam = 0; U32 iBest = 0;
    if (type > 12u) return;
    sp = &g_spec[type];
    if (sp->r <= 0.f || !position_ok(&E)) return;
    if (peek32(ADDR_CURR_AREA) != 0u) return;              /* interiors stay intact */
    load_sheet_surfaces();
    n = find_candidates(E, sp->r + sp->reach, ents); gm_probe_weight = 0.f;
    /* rockets and tank shells explode right at the surface they hit: the camera tells which side was hit */
    if (type == 2u || type == 3u || type == 10u) {
        U8 cm[0x48];
        if (mem_read(ADDR_CAMERA_MATRIX, cm, sizeof(cm))) { cam.x = f32(cm + 0x30); cam.y = f32(cm + 0x34); cam.z = f32(cm + 0x38); haveCam = position_ok(&cam) && gm_dist2(cam, E) > 1.f; }
    }
    p = cat(m, 0, 300, "[v3.6.2] EXPLOSION type="); p = dec(m, p, 300, type); p = cat(m, p, 300, " pos="); p = vprint(m, p, 300, &E);
    p = cat(m, p, 300, " objects="); p = dec(m, p, 300, n); logline(m);
    /* sort the objects: props are knocked over, shared models are left alone, the rest is carved */
    for (i = 0; i < n; i++) {
        ColInfo ci; Vec3 El, Sl = { 0, 0, 0 }, nl = { 0, 0, 0 }; float b = 1e30f;
        g_why = ""; kind[i].prop = kind[i].shared = 0;
        ok[i] = inspect(ents[i], &tg[i]);
        if (!ok[i]) { log_skip(ents[i], "inspect"); continue; }
        if (!read_col(&tg[i], &ci)) { ok[i] = 0; log_skip(ents[i], "collision"); continue; }
        classify(&tg[i], &ci, &kind[i]);
        if (kind[i].shared) {
            ok[i] = 0;
            p = cat(m, 0, 300, "[v3.6.2]   model="); p = dec(m, p, 300, tg[i].model); p = cat(m, p, 300, " not changed: placed ");
            p = dec(m, p, 300, kind[i].instances); p = cat(m, p, 300, " times here, all copies share one mesh (collision ");
            p = number_f(m, p, 300, kind[i].wide); p = cat(m, p, 300, " m wide, "); p = number_f(m, p, 300, kind[i].base); p = cat(m, p, 300, " m at the foot, "); p = number_f(m, p, 300, kind[i].tall);
            p = cat(m, p, 300, " m high, type "); p = dec(m, p, 300, kind[i].special); cat(m, p, 300, ")"); logline(m);
            continue;
        }
        El = to_local(&tg[i].frame, E);
        if (kind[i].prop) {
            ok[i] = 0;
            { U32 q; float reachP = sp->r * 1.2f;         /* how close the blast is to its collision shapes */
              gm_probe(&g_cm, El, reachP, 1, 0, &b, &Sl, &nl);
              for (q = 0; q < ci.nSpheres && q < 64u; q++) {
                  U32 a = ci.pSpheres + q * 0x14u; Vec3 c; float d;
                  c.x = peekf(a); c.y = peekf(a + 4u); c.z = peekf(a + 8u);
                  d = gm_sqrt(gm_dist2(El, c)) - peekf(a + 12u);
                  if (d < reachP && d * d < b) b = d > 0.f ? d * d : 0.f;
              } }
            if (b > 1e20f) continue;                        /* too far from the blast */
            if (!g_propsEnabled) { p = cat(m, 0, 300, "[v3.6.2]   model="); p = dec(m, p, 300, tg[i].model); cat(m, p, 300, " is a prop; knocking over is switched off (F7)"); logline(m); continue; }
            if (topple(&tg[i], E) < 0) log_skip(ents[i], "prop");
            else done++;
            continue;
        }
        b = best;
        if (haveCam) { camL = to_local(&tg[i].frame, cam); gm_probe(&g_cm, El, sp->reach + 1.0f, 1, &camL, &b, &Sl, &nl); }
        else gm_probe(&g_cm, El, sp->reach + 1.0f, 1, 0, &b, &Sl, &nl);
        if (b < best) { best = b; S = to_world(&tg[i].frame, Sl); iBest = i; }
        nl = dir_world(&tg[i].frame, nl); nsum.x += nl.x; nsum.y += nl.y; nsum.z += nl.z;
    }
    len = gm_sqrt(gm_dot(nsum, nsum)); dist = gm_sqrt(best);
    if (best > 1e20f || dist > sp->reach || len < 1e-6f) { logline(done ? "[v3.6.2]   no surface in reach, no crater." : "[v3.6.2]   no surface in reach, nothing changed."); return; }
    nrm.x = nsum.x / len; nrm.y = nsum.y / len; nrm.z = nsum.z / len;
    /* the crater always opens towards the side the blast is on */
    if (dist > 0.05f) {
        Vec3 toE; toE.x = (E.x - S.x) / dist; toE.y = (E.y - S.y) / dist; toE.z = (E.z - S.z) / dist;
        if (gm_dot(nrm, toE) < -0.2f) { nrm = toE; logline("[v3.6.2]   surface normals point away from the blast: using the blast direction"); }
    }
    /* a blast further from the surface leaves a smaller crater */
    scale = 1.f - 0.5f * dist / sp->reach; rim = sp->r * scale; depth = sp->d * scale;
    R = (rim * rim + depth * depth) / (2.f * depth);
    /* The side the blast is on must be one that can be seen. A wrecked car's explosion sits a little
     * under the road, a grenade sinks into a kerb: the visible surface then shows the blast its back,
     * and a crater dug from there comes out as a mound. Dig it from the visible side instead. */
    /* (only where the faces are drawn from one side: there the winding is right. Not for rockets: they go off ON the
     * surface, and which side that is the shooter's view decides, as before) */
    if (!haveCam && dist > 0.05f && (peek16(tg[iBest].modelInfo + 0x12u) & 0x40u) && blast_behind(&tg[iBest], S, nrm, E, R, depth)) {
        nrm.x = -nrm.x; nrm.y = -nrm.y; nrm.z = -nrm.z;
        logline("[v3.6.2]   the blast went off behind the visible surface: crater dug from the side that is seen");
    }
    C.x = S.x + nrm.x * (R - depth); C.y = S.y + nrm.y * (R - depth); C.z = S.z + nrm.z * (R - depth);
    Q.x = C.x + nrm.x * R; Q.y = C.y + nrm.y * R; Q.z = C.z + nrm.z * R;
    h = 0.6f * rim; if (h < 0.6f) h = 0.6f;         /* edge length in the bowl: a third fewer triangles per crater than 0.45 */
    g_mark[g_nMark & (MAX_MARKS - 1u)].c = C; g_mark[g_nMark & (MAX_MARKS - 1u)].r = R; g_nMark++;
    g_lastC = C; g_lastQ = Q; g_lastR = R; g_lastDist = dist; g_lastClarity = gm_probe_weight > 0.f ? len / gm_probe_weight : 0.f;
    p = cat(m, 0, 300, "[v3.6.2]   crater: surface="); p = vprint(m, p, 300, &S); p = cat(m, p, 300, " dist="); p = number_f(m, p, 300, dist);
    p = cat(m, p, 300, " normal="); p = vprint(m, p, 300, &nrm); p = cat(m, p, 300, " rim="); p = number_f(m, p, 300, rim);
    p = cat(m, p, 300, " depth="); p = number_f(m, p, 300, depth); logline(m);
    for (i = 0; i < n; i++) {
        Vec3 Cl, Ql; float uvM[8]; int rc, rr, note; U32 boxes;
        if (!ok[i]) continue;
        /* the object may have changed while we worked on the previous one: look again */
        g_why = "";
        if (!inspect(ents[i], &tg[i])) { log_skip(ents[i], "inspect"); continue; }
        Cl = to_local(&tg[i].frame, C); Ql = to_local(&tg[i].frame, Q);
        g_statHoleC = g_statHoleR = g_statShell = g_statShellC = 0;
        /* 1. which collision triangles are sheets (fence)  2. where the visible mesh is broken through
         * 3. the collision crater, with both  4. the visible crater */
#ifdef HOST_TEST
        g_dbgStayN = 0;
#endif
        if (prepare_col(&tg[i], Cl, R, Ql, h, depth, 1) < 0) { log_skip(ents[i], "collision"); continue; }
        gm_sheet_mode = 0; g_hookDepth = depth; g_skipObj = 0;
        shell_scan(&tg[i], Cl, R, Ql, h);
        if (g_skipObj) { log_skip(ents[i], "the blast went off inside its material"); continue; }
        note = g_shellNote;
        rc = prepare_col(&tg[i], Cl, R, Ql, h, depth, 0);
        if (rc < 0) { log_skip(ents[i], "collision"); continue; }
        if (rc == 0) continue;                              /* not inside the crater */
        boxes = g_ci.boxesConverted;
        /* render first: an object whose visible mesh cannot be changed keeps its collision too */
        rock_mapping(&tg[i].frame, nrm, uvM);
        rr = apply_render(&tg[i], Cl, R, Ql, h, depth, uvM);
        if (g_pendSnap) { game_free(g_pendSnap); g_pendSnap = 0; }      /* the mesh was left as it is: no copy of it is needed */
        if (rr < 0) { log_skip(ents[i], "render"); continue; }
        p = cat(m, 0, 300, "[v3.6.2]   model="); p = dec(m, p, 300, tg[i].model);
        if (rc == 2 || rr == 2) {                           /* blown away completely */
            hide_entity(ents[i]); cat(m, p, 300, " destroyed completely (nothing left inside the blast)"); logline(m); done++; continue;
        }
        if (rr == 0) { cat(m, p, 300, ": visible mesh not inside the crater, left unchanged"); logline(m); continue; }
        p = cat(m, p, 300, " RW v "); p = dec(m, p, 300, g_statRv0); p = cat(m, p, 300, "->"); p = dec(m, p, 300, g_statRv1);
        p = cat(m, p, 300, " t "); p = dec(m, p, 300, g_statRt0); p = cat(m, p, 300, "->"); p = dec(m, p, 300, g_statRt1);
        p = cat(m, p, 300, " atomics="); p = dec(m, p, 300, g_statAtomics);
        if (commit_col(&tg[i]) < 0) { p = cat(m, p, 300, " | COLLISION NOT UPDATED: "); cat(m, p, 300, g_why); logline(m); }
        else {
            p = cat(m, p, 300, " | COL v "); p = dec(m, p, 300, g_statCv0); p = cat(m, p, 300, "->"); p = dec(m, p, 300, g_statCv1);
            p = cat(m, p, 300, " t "); p = dec(m, p, 300, g_statCt0); p = cat(m, p, 300, "->"); p = dec(m, p, 300, g_statCt1);
            p = cat(m, p, 300, " groups="); p = dec(m, p, 300, g_statGroups);
            if (boxes) { p = cat(m, p, 300, " boxes->tris="); p = dec(m, p, 300, boxes); }
            if (g_statHoleC || g_statHoleR) { p = cat(m, p, 300, " holes(render/col)="); p = dec(m, p, 300, g_statHoleR); p = cat(m, p, 300, "/"); p = dec(m, p, 300, g_statHoleC); }
            if (g_statShell || g_statShellC) { p = cat(m, p, 300, " broke through(render/col)="); p = dec(m, p, 300, g_statShell); p = cat(m, p, 300, "/"); p = dec(m, p, 300, g_statShellC); }
            if (note == 1) p = cat(m, p, 300, " (hollow inside: stays solid)"); else if (note == 2) p = cat(m, p, 300, " (nothing below: stays solid)");
            else if (note == 3) p = cat(m, p, 300, " (blast behind the surface: stays solid)");
            if (!(peek16(tg[i].modelInfo + 0x12u) & 0x40u)) p = cat(m, p, 300, gm_sheet_mode ? " [double-sided, space behind]" : " [double-sided]");
            logline(m);
        }
        done++;
    }
    if (done) g_craters++;
    if (!done) logline("[v3.6.2]   nothing changed.");
}

/* ---- explosion detection: watch the game's explosion slots --------------------------------- */
typedef struct { Vec3 pos; U32 type; } Pending;
static Pending g_queue[32]; static U32 g_qHead, g_qTail;
static U8 g_seenActive[NUM_EXPLOSIONS]; static U32 g_seenStamp[NUM_EXPLOSIONS]; static Vec3 g_seenPos[NUM_EXPLOSIONS];
static void enqueue(Vec3 pos, U32 type) {
    U32 next = (g_qTail + 1u) & 31u;
    if (next == g_qHead) return;                 /* full: drop */
    g_queue[g_qTail].pos = pos; g_queue[g_qTail].type = type; g_qTail = next;
}
static void poll_explosions(void) {
    U32 i;
    for (i = 0; i < NUM_EXPLOSIONS; i++) {
        U32 a = ADDR_EXPLOSIONS + i * EXPLOSION_SIZE, active = peek8(a + 0x28u), stamp; Vec3 pos;
        if (!active) { g_seenActive[i] = 0; continue; }
        stamp = peek32(a + 0x20u);                /* expire time: differs for every new explosion in a slot */
        pos.x = peekf(a + 4u); pos.y = peekf(a + 8u); pos.z = peekf(a + 0xCu);
        if (g_seenActive[i] && g_seenStamp[i] == stamp && g_seenPos[i].x == pos.x && g_seenPos[i].y == pos.y && g_seenPos[i].z == pos.z) continue;
        g_seenActive[i] = 1; g_seenStamp[i] = stamp; g_seenPos[i] = pos;
        enqueue(pos, peek32(a));
    }
}
static void process_queue(void) {
    U32 k;
    for (k = 0; k < CRATERS_PER_FRAME && g_qHead != g_qTail; k++) {
        Pending e = g_queue[g_qHead]; g_qHead = (g_qHead + 1u) & 31u;
        blast(e.pos, e.type);
    }
}
/* F8, last part: the player (or the vehicle he sits in) stood in a crater or a tunnel - where the ground
 * is whole again now. He is put on top of the lowest surface that lies above his feet. */
/* The player at the moment of the reset: what he stands on is measured BEFORE the ground comes back, so that afterwards it is
 * known whether the ground under him is still the same (then he is left alone - under a roof, next to a crater, in the air). */
static U32 g_plMtx, g_plEnt; static Vec3 g_plPos; static float g_plBelow, g_plFeet; static int g_plNear;
static float ground_below(Vec3 p, float reach) {
    U8 point[0x2c]; U32 obj = 0; Vec3 to = p; to.z -= reach;
    memset(point, 0, sizeof(point));
    return game_line_of_sight(&p, &to, point, &obj) ? p.z - f32(point + 8) : -1.f;
}
static void player_before(void) {
    U32 i, n = g_nMark < MAX_MARKS ? g_nMark : MAX_MARKS;
    g_plNear = 0; g_plEnt = 0; g_plMtx = 0; g_plFeet = 0.7f;
    if (!mem_read(ADDR_PLAYER_VEHICLE, &g_plEnt, 4) || !g_plEnt) { g_plFeet = 1.0f; if (!mem_read(ADDR_PLAYER_PED, &g_plEnt, 4) || !g_plEnt) return; }
    if (!mem_read(g_plEnt + 0x14u, &g_plMtx, 4) || !g_plMtx || !mem_read(g_plMtx + 0x30u, &g_plPos, 12) || !position_ok(&g_plPos)) return;
    for (i = 0; i < n && !g_plNear; i++) { float d = g_mark[i].r + 3.f; if (gm_dist2(g_plPos, g_mark[i].c) < d * d) g_plNear = 1; }
    if (g_plNear) g_plBelow = ground_below(g_plPos, 40.f);
}
static void lift_player(void) {
    U8 point[0x2c]; U32 obj = 0, k, n = 0; Vec3 p = g_plPos, v, from, to; float top, lowest, feet, now, floorZ, h[12]; int standing;
    if (!g_plNear) return;
    now = ground_below(p, 40.f);
    if (now < 0.f ? g_plBelow < 0.f : g_plBelow >= 0.f && now > g_plBelow - 0.05f && now < g_plBelow + 0.05f) return;      /* the same ground as before under him */
    standing = g_plBelow >= 0.f && g_plBelow < 2.f;
    if (!standing && now >= 0.f && (g_plBelow < 0.f || now < g_plBelow)) return;      /* he is in the air, the ground came back BELOW him */
    feet = standing ? g_plBelow : g_plFeet;
    floorZ = p.z - feet + 0.05f;                              /* just above where his feet are */
    top = p.z + 40.f;
    for (k = 0; k < 12u && top > floorZ; k++) {               /* the surfaces above his feet, from the top down */
        from = p; from.z = top; to = p; to.z = floorZ;
        memset(point, 0, sizeof(point)); obj = 0;
        if (!game_line_of_sight(&from, &to, point, &obj)) break;
        h[n] = f32(point + 8);
        if (!(h[n] < top - 0.01f)) break;
        top = h[n++] - 0.3f;
    }
    if (!n) return;                                           /* nothing above him: whatever he stood on is gone, he drops onto what is there */
    /* the ground that came back: the lowest of them with room for him above it (a layer hidden closely under a road is none) */
    for (k = n - 1u; k > 0u && h[k - 1u] - h[k] < 2.f * feet + 0.2f; k--) { }
    lowest = h[k];
    p.z = lowest + feet + 0.1f;
    if (mem_write(g_plMtx + 0x30u, &p, 12)) {
        if (mem_read(g_plEnt + 0x44u, &v, 12)) { v.z = 0.f; mem_write(g_plEnt + 0x44u, &v, 12); }
        logline("[v3.6.2] F8: the player stood in a crater - put on top of the ground.");
    }
}
/* F8: every crater and hole is taken back, hidden objects are shown again. */
static void reset_all(void) {
    U32 i, nRender = 0, nCol = 0, nShown = 0, pool = 0, storage = 0, flagsArr = 0, cap = 0; char m[200]; int p;
    if (mem_read(ADDR_BUILDING_POOL, &pool, 4) && pool) { mem_read(pool, &storage, 4); mem_read(pool + 4u, &flagsArr, 4); mem_read(pool + 8u, &cap, 4); }
    if (cap > 100000u) cap = 100000u;
    player_before();
    for (i = 0; i < g_nUndo; i++) {
        Undo* u = &g_undo[i]; U32 mi = 0, miAtomic = 0, colModel = 0, g = 0; U8 cm[0x30], atom[0x20];
        mem_read(ADDR_MODEL_INFO_PTRS + u->model * 4u, &mi, 4);
        if (u->snap) {
            if (mi && mem_read(mi + 0x1Cu, &miAtomic, 4) && miAtomic && mem_read(miAtomic, atom, sizeof(atom)) && atom[0] == 1u && u32(atom + 0x18) == u->curGeom) {
                Target t; GeoInfo gi; U32 nMat;
                memset(&t, 0, sizeof(t)); t.model = u->model; t.modelInfo = mi; t.geom = u->curGeom;
                g_why = "";
                if (read_render(u->curGeom, &gi) && (nMat = snapshot_load(u->snap, &gi)) != 0u && write_geometry(&t, &gi, nMat, 0u, 1) > 0) nRender++;
                else { p = cat(m, 0, 200, "[v3.6.2] F8: model "); p = dec(m, p, 200, u->model); p = cat(m, p, 200, " could not be restored: "); cat(m, p, 200, g_why); logline(m); }
            }
            game_free(u->snap);
        }
        if (u->origCol) {
            if (mi && mem_read(mi + 0x14u, &colModel, 4) && colModel && mem_read(colModel, cm, sizeof(cm)) && u32(cm + 0x2c) == u->curCol && (cm[0x29] & 2u)) {
                if (mem_read(u->curCol + 0x1Cu, &g, 4) && g) game_remove_planes(u->curCol);
                memcpy(cm, u->bounds, sizeof(u->bounds)); put32(cm + 0x2c, u->origCol);
                if (mem_write(colModel, cm, sizeof(cm))) { game_free(u->curCol); nCol++; }
                else game_free(u->origCol);
            } else game_free(u->origCol);              /* loaded afresh since (or not loaded at all): whole again anyway */
        }
    }
    g_nUndo = 0; g_undoBytes = 0; g_undoFull = 0;
    if (g_pendSnap) { game_free(g_pendSnap); g_pendSnap = 0; }
    /* knocked-over and blown-away objects: the static ones are shown again, the fallen ones may go */
    for (i = 0; i < g_nHidden; i++) {
        Hidden* h = &g_hidden[i]; U32 f = 0, lod = 0, idx;
        if (!storage || !flagsArr || h->ent < storage || (h->ent - storage) % BUILDING_SIZE) continue;
        idx = (h->ent - storage) / BUILDING_SIZE;
        if (idx >= cap || (peek8(flagsArr + idx) & 0x80u) || peek16(h->ent + 0x22u) != h->model) continue;
        if (mem_read(h->ent + 0x1Cu, &f, 4)) { f |= h->flags & 0x81u; mem_write(h->ent + 0x1Cu, &f, 4); nShown++; }
        if (mem_read(h->ent + 0x30u, &lod, 4) && lod && mem_read(lod + 0x1Cu, &f, 4)) { f |= h->lodFlags & 0x80u; mem_write(lod + 0x1Cu, &f, 4); }
    }
    g_nHidden = 0;
    for (i = 0; i < g_nFallen; i++) {
        Fallen* fl = &g_fallen[i]; U16 model = 0; U8 type = 0; U32 at = 0, now = 0;
        if (mem_read(fl->obj + 0x22u, &model, 2) && model == fl->model && mem_read(fl->obj + 0x13Cu, &type, 1) && type == 3u &&
            mem_read(fl->obj + 0x150u, &at, 4) && at == fl->removeAt && mem_read(ADDR_TIME_MS, &now, 4)) mem_write(fl->obj + 0x150u, &now, 4);      /* still ours: the game may remove it now */
    }
    g_nFallen = 0;
    memset(g_slab, 0, sizeof(g_slab));
    g_qHead = g_qTail = 0;
    p = cat(m, 0, 200, "[v3.6.2] F8 reset: visible mesh restored on "); p = dec(m, p, 200, nRender); p = cat(m, p, 200, " objects, collision on "); p = dec(m, p, 200, nCol);
    p = cat(m, p, 200, ", objects shown again: "); p = dec(m, p, 200, nShown); cat(m, p, 200, "."); logline(m);
    lift_player();
    g_nMark = 0;
}
static void frame_tick2(int f8Pressed, int f7Pressed) {
    if (f7Pressed) { g_propsEnabled = !g_propsEnabled; logline(g_propsEnabled ? "[v3.6.2] F7: knocking over props ON." : "[v3.6.2] F7: knocking over props OFF."); }
    poll_explosions();
    if (f8Pressed) reset_all();
    process_queue();
}
static void frame_tick(int f8Pressed) { frame_tick2(f8Pressed, 0); }

/* ---- Windows: game-thread hook, input, logging ------------------------------------------- */
#ifndef HOST_TEST
static char g_logpath[MAX_PATH];
static U32 g_chain;
static int g_hooked;
typedef U8 (__cdecl *LineOfSightFn)(const Vec3*, const Vec3*, void*, void**, U8, U8, U8, U8, U8, U8, U8, U8);
typedef void (__cdecl *VoidFn)(void);
typedef U32 (__cdecl *Fn1)(U32);
typedef U32 (__cdecl *Fn2)(U32, U32);
typedef U32 (__cdecl *Fn3)(U32, U32, U32);

static int addr_ok(U32 a, U32 n) { return a >= 0x10000u && n && a < 0xFFFF0000u - n; }
static int mem_read(U32 a, void* out, U32 n) {
    U32 got = 0;
    return addr_ok(a, n) && ReadProcessMemory(GetCurrentProcess(), (const void*)a, out, n, &got) && got == n;
}
static int mem_write(U32 a, const void* src, U32 n) {
    U32 put = 0;
    return addr_ok(a, n) && WriteProcessMemory(GetCurrentProcess(), (void*)a, src, n, &put) && put == n;
}
static U32 peek32(U32 a) { return *(const U32*)a; }
static U32 peek16(U32 a) { return *(const U16*)a; }
static U32 peek8(U32 a) { return *(const U8*)a; }
static float peekf(U32 a) { return *(const float*)a; }
static void logline(const char* s) {
    U32 n = 0;
    void* h = CreateFileA(g_logpath, 0x40000000u, 1u, 0, 4u, 0x80u, 0);
    if (h == (void*)-1) return;
    SetFilePointer(h, 0, 0, 2u);
    WriteFile(h, s, slen(s), &n, 0); WriteFile(h, "\r\n", 2, &n, 0); CloseHandle(h);
}
static int game_line_of_sight(const Vec3* from, const Vec3* to, U8* colPoint, U32* entity) {
    return ((LineOfSightFn)ADDR_LINE_OF_SIGHT)(from, to, colPoint, (void**)entity, 1, 0, 0, 0, 0, 0, 0, 0) != 0;   /* buildings only */
}
static void game_remove_planes(U32 colData) { ((Fn1)ADDR_REMOVE_TRI_PLANES)(colData); }
static U32 game_malloc(U32 size) { return ((Fn1)ADDR_MEM_MALLOC)(size); }
static void game_free(U32 ptr) { ((Fn1)ADDR_MEM_FREE)(ptr); }
static U32 rw_geom_create(U32 nv, U32 nt, U32 format) { return ((Fn3)ADDR_GEOM_CREATE)(nv, nt, format); }
static void rw_geom_destroy(U32 geom) { ((Fn1)ADDR_GEOM_DESTROY)(geom); }
static U32 rw_geom_unlock(U32 geom) { return ((Fn1)ADDR_GEOM_UNLOCK)(geom); }
static int rw_matlist_append(U32 matList, U32 material) { return (int)((Fn2)ADDR_MATLIST_APPEND)(matList, material); }
static void rw_atomic_set_geometry(U32 atomic, U32 geom) { ((Fn3)ADDR_ATOMIC_SET_GEOM)(atomic, geom, 0); }
static U32 rw_raster_create(U32 w, U32 h, U32 depth, U32 flags) { return ((U32 (__cdecl*)(U32, U32, U32, U32))ADDR_RASTER_CREATE)(w, h, depth, flags); }
static U32 rw_raster_lock(U32 raster, U32 level, U32 mode) { return ((Fn3)ADDR_RASTER_LOCK)(raster, level, mode); }
static void rw_raster_unlock(U32 raster) { ((Fn1)ADDR_RASTER_UNLOCK)(raster); }
static void rw_raster_destroy(U32 raster) { ((Fn1)ADDR_RASTER_DESTROY)(raster); }
static U32 rw_raster_num_levels(U32 raster) { return ((Fn1)ADDR_RASTER_NUM_LEVELS)(raster); }
static U32 rw_texture_create(U32 raster) { return ((Fn1)ADDR_TEXTURE_CREATE)(raster); }
static U32 rw_material_create(void) { return ((U32 (__cdecl*)(void))ADDR_MATERIAL_CREATE)(); }
static void rw_material_set_texture(U32 material, U32 texture) { ((Fn2)ADDR_MATERIAL_SET_TEX)(material, texture); }
static U32 game_object_create(U32 model) { return ((Fn2)ADDR_OBJECT_CREATE)(model, 0); }
static void game_world_add(U32 entity) { ((Fn1)ADDR_WORLD_ADD)(entity); }
static U32 rw_d3d9_get_usage(U32 geom) { return ((Fn1)ADDR_D3D9_GET_USAGE)(geom); }
static void rw_d3d9_set_usage(U32 geom, U32 flags) { ((Fn2)ADDR_D3D9_SET_USAGE)(geom, flags); }

static void tick(void) {
    static int old8, old7;
    int k8 = (GetAsyncKeyState(VK_F8) & (short)0x8000) != 0, k7 = (GetAsyncKeyState(VK_F7) & (short)0x8000) != 0;
    if (k8 | k7) {
        U32 pid = 0; void* f = GetForegroundWindow();
        if (f) GetWindowThreadProcessId(f, &pid);
        if (pid != GetCurrentProcessId()) k8 = k7 = 0;
    }
    frame_tick2(k8 && !old8, k7 && !old7);
    old8 = k8; old7 = k7;
}
/* Replaces the target of "call CGame::Process" in Idle(): once per game frame on the game
 * thread, after the world update and before rendering. */
static void __cdecl hook_game_process(void) {
    ((VoidFn)g_chain)();
    tick();
}
static int patch_call(U32 target) {
    U32 rel = target - (ADDR_CALL_GAME_PROCESS + 5u), old = 0, tmp = 0;
    if (!VirtualProtect((void*)(ADDR_CALL_GAME_PROCESS + 1u), 4, 0x40u, &old)) return 0;
    memcpy((void*)(ADDR_CALL_GAME_PROCESS + 1u), &rel, 4);
    VirtualProtect((void*)(ADDR_CALL_GAME_PROCESS + 1u), 4, old, &tmp);
    FlushInstructionCache(GetCurrentProcess(), (const void*)ADDR_CALL_GAME_PROCESS, 5);
    return 1;
}
static void install_hook(void) {
    U8 b[5]; U32 sig = 0; char msg[200]; int p;
    if (!mem_read(ADDR_CALL_GAME_PROCESS, b, 5) || b[0] != 0xE8) {
        logline("[v3.6.2] HOOK FAILED: no call instruction at 0x53E981. This is not gta_sa.exe 1.0 US. Mod inactive."); return;
    }
    g_chain = ADDR_CALL_GAME_PROCESS + 5u + u32(b + 1);
    mem_read(ADDR_EXE_SIGNATURE, &sig, 4);
    if (g_chain != ADDR_GAME_PROCESS && sig != SIG_10US_COMPACT && sig != SIG_10US_HOODLUM) {
        p = cat(msg, 0, 200, "[v3.6.2] HOOK FAILED: unknown exe (signature "); p = hex(msg, p, 200, sig);
        p = cat(msg, p, 200, ", call target "); p = hex(msg, p, 200, g_chain);
        cat(msg, p, 200, "). Mod inactive."); logline(msg); return;
    }
    if (!patch_call((U32)&hook_game_process)) { logline("[v3.6.2] HOOK FAILED: code page could not be made writable. Mod inactive."); return; }
    g_hooked = 1;
    p = cat(msg, 0, 200, "[v3.6.2] HOOK OK: game-thread hook at 0x53E981, previous target "); p = hex(msg, p, 200, g_chain);
    p = cat(msg, p, 200, g_chain == ADDR_GAME_PROCESS ? " (CGame::Process)" : " (another mod, chained)");
    p = cat(msg, p, 200, ", exe signature "); hex(msg, p, 200, sig); logline(msg);
}
static void remove_hook(void) {
    U8 b[5];
    if (!g_hooked || !mem_read(ADDR_CALL_GAME_PROCESS, b, 5) || b[0] != 0xE8) return;
    if (ADDR_CALL_GAME_PROCESS + 5u + u32(b + 1) == (U32)&hook_game_process) patch_call(g_chain);
    g_hooked = 0;
}
int WINAPI DllMain(void* mod, U32 reason, void* reserved) {
    if (reason == 1) {
        U32 len, start = 0, dot, i;
        DisableThreadLibraryCalls(mod);
        len = GetModuleFileNameA(mod, g_logpath, MAX_PATH); dot = len;
        if (!len || len >= MAX_PATH - 5) return 1;
        for (i = 0; i < len; i++) { if (g_logpath[i] == '\\' || g_logpath[i] == '/') start = i + 1; if (g_logpath[i] == '.') dot = i; }
        if (dot < start) dot = len;
        g_logpath[dot++] = '.'; g_logpath[dot++] = 'l'; g_logpath[dot++] = 'o'; g_logpath[dot++] = 'g'; g_logpath[dot] = 0;
        logline("[v3.6.2] SA Destruction: explosions carve craters, break through roofs, walls and bridge decks, join tunnels and knock over trees (outside world only). F8 = reset (all craters and holes taken back), F7 = props on/off.");
        install_hook();
    } else if (reason == 0 && !reserved) {
        remove_hook();
    }
    return 1;
}
void* __cdecl memcpy(void* dest, const void* src, unsigned int count) {
    U8* d = (U8*)dest; const U8* s = (const U8*)src; unsigned int i;
    for (i = 0; i < count; i++) d[i] = s[i];
    return dest;
}
void* __cdecl memset(void* dest, int ch, unsigned int count) {
    U8* d = (U8*)dest; unsigned int i;
    for (i = 0; i < count; i++) d[i] = (U8)ch;
    return dest;
}
#endif
