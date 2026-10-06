/* SA Destruction - procedural rock texture (no files, no CRT).
 * Tileable 128x128 value noise: large mottling + medium grain + dark crack lines,
 * grey-brown like blasted rock. Output: BGRA bytes (D3D X8R8G8B8 order). */
#ifndef ROCK_TEXTURE_INCLUDED
#define ROCK_TEXTURE_INCLUDED
#define ROCK_SIZE 128u
static U32 rock_hash(U32 x, U32 y, U32 seed) {
    U32 h = x * 374761393u + y * 668265263u + seed * 2246822519u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}
/* smooth value noise in [0,1], lattice of `cells` cells across the texture (wraps) */
static float rock_noise(U32 px, U32 py, U32 cells, U32 seed) {
    U32 step = ROCK_SIZE / cells, x0 = px / step, y0 = py / step, x1 = (x0 + 1u) % cells, y1 = (y0 + 1u) % cells;
    float fx = (float)(px % step) / (float)step, fy = (float)(py % step) / (float)step, a, b, c, d;
    fx = fx * fx * (3.f - 2.f * fx); fy = fy * fy * (3.f - 2.f * fy);
    a = (float)(rock_hash(x0, y0, seed) & 0xFFFFu) / 65535.f; b = (float)(rock_hash(x1, y0, seed) & 0xFFFFu) / 65535.f;
    c = (float)(rock_hash(x0, y1, seed) & 0xFFFFu) / 65535.f; d = (float)(rock_hash(x1, y1, seed) & 0xFFFFu) / 65535.f;
    return (a + (b - a) * fx) + ((c + (d - c) * fx) - (a + (b - a) * fx)) * fy;
}
static void rock_generate(U8* bgra) {
    U32 x, y;
    for (y = 0; y < ROCK_SIZE; y++) for (x = 0; x < ROCK_SIZE; x++) {
        float big = rock_noise(x, y, 4, 11) * 0.6f + rock_noise(x, y, 8, 23) * 0.4f;                   /* large mottling */
        float grain = rock_noise(x, y, 16, 37) * 0.30f + rock_noise(x, y, 32, 41) * 0.40f + rock_noise(x, y, 64, 53) * 0.30f;
        float ridge = rock_noise(x, y, 4, 67) * 0.55f + rock_noise(x, y, 8, 71) * 0.30f + rock_noise(x, y, 16, 79) * 0.15f;
        float grit = (float)(rock_hash(x, y, 97) & 0xFFu) / 255.f, crack, v, r, g, b;
        U8* p = bgra + (y * ROCK_SIZE + x) * 4u;
        ridge = ridge - 0.5f; if (ridge < 0.f) ridge = -ridge;              /* 0 along the crack lines */
        crack = ridge * 9.f; if (crack > 1.f) crack = 1.f;                  /* soft dark seams */
        v = 0.27f + 0.26f * big + 0.52f * (grain - 0.5f) + 0.07f * (grit - 0.5f);
        v *= 0.72f + 0.28f * crack;
        if (v < 0.05f) v = 0.05f; if (v > 0.80f) v = 0.80f;
        r = v * 1.05f; g = v * 0.99f; b = v * 0.91f;                        /* slightly warm grey */
        if (r > 1.f) r = 1.f;
        p[0] = (U8)(b * 255.f); p[1] = (U8)(g * 255.f); p[2] = (U8)(r * 255.f); p[3] = 255;
    }
}
/* next mip level by 2x2 averaging; returns the new size */
static U32 rock_halve(const U8* src, U32 size, U8* dst) {
    U32 n = size / 2u, x, y, k;
    for (y = 0; y < n; y++) for (x = 0; x < n; x++) for (k = 0; k < 4; k++)
        dst[(y * n + x) * 4u + k] = (U8)(((U32)src[((2u * y) * size + 2u * x) * 4u + k] + src[((2u * y) * size + 2u * x + 1u) * 4u + k] +
                                         src[((2u * y + 1u) * size + 2u * x) * 4u + k] + src[((2u * y + 1u) * size + 2u * x + 1u) * 4u + k] + 2u) / 4u);
    return n;
}
#endif
