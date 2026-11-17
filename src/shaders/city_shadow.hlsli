// The baked city shadows, read by the lit draws (city_shadow.hlsl bakes them; fly/city_shadow.h drives it). A point on a roof or the
// ground reads the sunlit spans of the texels round it (bilinear, only those at its own height, so a roof's edge does not take the
// street's shadow) and compares the hour; a wall, a tree or anything else above the height map marches the height map toward the sun
// instead (a few dozen loads, only for those pixels). Direct3D 12 only for now: the other backends leave it off.
#ifndef TBD_CITY_SHADOW_HLSLI
#define TBD_CITY_SHADOW_HLSLI

#ifndef VULKAN
cbuffer CityShadowView : register(b4) {
    float4 gCityA; // x, y: the region's corner in scene space (x, z); z: the floating origin's height (world = scene + z); w: the hour
    float4 gCityB; // x, y: the ring texel of the region's corner; z: on; w: floor = the live map's town reach (m), frac x 100 = march steps
};
Texture2D<uint2> gCityTimes : register(t7);
Texture2D<float> gCityHeights : register(t8);

static const int kCityN = 2560;      // renderer.h kCityShadowSize
static const float kCityCell = 2.0;  // kCityShadowCell
static const float kCityTop = 1500.0, kCityFar = 2000.0;

int2 CityRing(int2 region) { return (region + int2(gCityB.xy)) % kCityN; }
float CityHeight(int2 region) {
    if (any(region < 0) || any(region >= kCityN)) return -1e4;
    return kCityTop - gCityHeights.Load(int3(CityRing(region), 0)) * kCityFar;
}
// How lit a texel's spans say the point is now: each span fades in and out over the minute or two the sun's disc takes to cross an
// edge, which also gives the right penumbra (a far caster's edge sweeps the ground faster).
float CitySpansLit(uint2 t) {
    const float q = (gCityA.w - 6.0) / 12.0 * 1023.0, w = 1.6;
    const uint s0 = t.x & 1023u, e0 = (t.x >> 10) & 1023u, s1 = (t.x >> 20) & 1023u;
    const uint e1 = t.y & 1023u, s2 = (t.y >> 10) & 1023u, e2 = (t.y >> 20) & 1023u;
    float lit = 0.0;
    if (e0 > s0) lit += smoothstep(s0 - w, s0 + w, q) * (1.0 - smoothstep(e0 - w, e0 + w, q));
    if (e1 > s1) lit += smoothstep(s1 - w, s1 + w, q) * (1.0 - smoothstep(e1 - w, e1 + w, q));
    if (e2 > s2) lit += smoothstep(s2 - w, s2 + w, q) * (1.0 - smoothstep(e2 - w, e2 + w, q));
    return saturate(lit);
}
// For points off the height map's surface: the height map marched toward the sun now, from the point itself. `steps` loads at most,
// each step ~30 % longer than the last (they only run past the live map's town, where a wall is a few pixels: 40 fine steps cost
// ~5 ms a frame over a city's walls and trees for nothing visible).
float CityMarch(float2 r, float y, float3 n, float jitter, int steps) {
    const float3 l = gLight.sunDir.xyz;
    if (l.y < 0.02 || dot(n, l) <= 0.0) return 1.0; // the lighting's own n.l darkens it
    const float flat = length(l.xz), tanEl = l.y / max(flat, 1e-4);
    const float2 dir = l.xz / max(flat, 1e-4) / kCityCell;
    const float2 start = r + n.xz * (0.9 / max(length(n.xz), 0.3)); // out of its own building's texels
    float s = kCityCell * (0.5 + jitter);
    [loop] for (int k = 0; k < steps && s < 700.0; ++k) {
        const float h = CityHeight(int2(floor(start + dir * s)));
        if (h > y + 0.3 + s * tanEl) return 0.0;
        if (y + s * tanEl > 650.0) break; // above anything a city holds
        s += max(kCityCell, s * 0.3);
    }
    return 1.0;
}
// 1 = sunlit, 0 = in a baked city shadow. `wp` scene position, `n` unit normal, `shadowPos` the point in the live sun map (clip).
// Where the live map still draws the town (game.cpp: tiles within ~350 m of the eye, and the point inside the map) it already has
// these shadows, crisper: nothing to pay there, a 40 m blend at the edge. Beyond, roofs and ground read the bake (8 loads), walls,
// trees and the rest march briefly. FLY_FEATURES bit 4096 turns it off.
float CityShadow(float3 wp, float3 n, float jitter, float3 shadowPos) {
    if (gCityB.z < 0.5 || FEATURE_OFF(4096)) return 1.0;
    const float liveNear = floor(gCityB.w);
    const int steps = (int)round(frac(gCityB.w) * 100.0);
    const bool inMap = all(abs(shadowPos.xy) < 0.97) && shadowPos.z < 1.0;
    const float dist = length(wp.xz - gLight.eye.xz);
    const float k = inMap ? smoothstep(liveNear - 40.0, liveNear, dist) : 1.0;
    if (k <= 0.0) return 1.0;
    // a facade a few pixels wide a kilometre off needs no fine march: the full count to 700 m, 4 steps past ~1.8 km (A/B at 420 m:
    // the march was 2 of the lookup's 3 ms)
    const int marchN = (int)round(lerp((float)steps, min(4.0, (float)steps), smoothstep(700.0, 1800.0, dist)));
    const float2 r = (wp.xz - gCityA.xy) / kCityCell; // region texels
    if (any(r < 1.0) || any(r > kCityN - 2.0)) return 1.0;
    const float y = wp.y + gCityA.z;
    if (n.y < 0.5) return lerp(1.0, CityMarch(r, y, n, jitter, marchN), k); // walls, leaves: never at the height map's surface
    const float2 f = r - 0.5, fr = frac(f);
    const int2 i0 = int2(floor(f));
    if (dist > 1000.0) { // past ~1 km a 2 m texel is under a pixel: the nearest one alone (2 loads, not 8)
        const int2 ring = CityRing(int2(floor(r)));
        const uint2 t = gCityTimes.Load(int3(ring, 0));
        const float h = kCityTop - gCityHeights.Load(int3(ring, 0)) * kCityFar;
        if (abs(h - y) < 2.5) return (t.x >> 31) == 0u ? 1.0 : lerp(1.0, CitySpansLit(t), k);
        return lerp(1.0, CityMarch(r, y, n, jitter, marchN), k);
    }
    float sum = 0.0, wsum = 0.0;
    bool unknown = false;
    [unroll] for (int q = 0; q < 4; ++q) {
        const int2 c = i0 + int2(q & 1, q >> 1);
        const int2 ring = CityRing(c);
        const uint2 t = gCityTimes.Load(int3(ring, 0));
        const float h = kCityTop - gCityHeights.Load(int3(ring, 0)) * kCityFar;
        const float w = ((q & 1) ? fr.x : 1.0 - fr.x) * ((q >> 1) ? fr.y : 1.0 - fr.y) * (abs(h - y) < 2.5 ? 1.0 : 0.0);
        if ((t.x >> 31) == 0u) { unknown = unknown || w > 0.0; continue; }
        sum += w * CitySpansLit(t);
        wsum += w;
    }
    if (wsum > 0.02) return lerp(1.0, sum / wsum, k);
    if (unknown) return 1.0; // its block isn't baked yet
    return lerp(1.0, CityMarch(r, y, n, jitter, marchN), k);
}
#else
float CityShadow(float3 wp, float3 n, float jitter, float3 shadowPos) { return 1.0; }
#endif

#endif
