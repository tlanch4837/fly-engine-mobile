// Water, on both backends: the ocean at a beach (src/fly/ocean.h). Compute evolves a wind-sea spectrum and runs its FFTs; the
// surface is drawn after the sky and haze are composed, so it refracts the finished picture and reads the depth buffer for the bed.
// Public, long-standing methods only: Tessendorf's FFT ocean ("Simulating Ocean Water", 2001) with a JONSWAP spectrum; linear
// shoaling and Snell refraction of swell, McCowan's depth-limited breaking, ballistic swash run-up; Gerstner (trochoidal) crests;
// Beer-Lambert absorption with single scattering (Henyey-Greenstein); Beckmann glitter whose slope variance grows with the waves
// a pixel can no longer resolve (Bruneton, Neyret and Holzschuch, "Real-time Realistic Ocean Lighting", 2010); caustics from the
// height's Hessian (a refracted ray bundle's Jacobian).
// Entry points: CSWaves (spectrum at time t, FFT along rows), CSRipple (FFT along columns, fields, foam), CSSky (the sky table),
// WaterVS / WaterPS. Compiled by fxc (cs/vs/ps_5_1) and by dxc (-DVULKAN).
#ifdef VULKAN
#define RO(n) [[vk::binding(n, 0)]]
#define RW(n) [[vk::binding(4 + n, 0)]]
#define GX(n) [[vk::binding(n, 0)]]
#else
#define RO(n)
#define RW(n)
#define GX(n)
#endif

#ifndef OCEAN_N
#define OCEAN_N 256                  // FFT size: texels per side of a cascade (water_m.hlsl: 128, water_s.hlsl: 64)
#endif
static const uint kN = OCEAN_N;
static const uint kLogN = OCEAN_N == 256 ? 8u : (OCEAN_N == 128 ? 7u : 6u);
static const uint kCascades = 3;
static const uint kSkyW = 64, kSkyH = 32;
static const uint kFoamSize = 512;
static const float kG = 9.81;
static const float kTau = 6.2831853;

float3 SkyBaseFn(float3 d, float3 eyePos, float3 sun);

// ------------------------------------------------------------------ compute
#ifdef WATER_COMPUTE
struct ComputeConstants {
    float4 a;    // CSWaves: time, cascade metres x3. CSRipple: foam decay, fold threshold, foam gain. CSSky: eye
    float4 b;    // CSWaves: choppiness, frequency quantum, the dispatch's first cascade. CSRipple: -, -, its first cascade. CSSky: sun
    float4 c[6]; // CSWaves, CSRipple: c[0] each cascade's first texel (ocean.h: the cascades may have grids of their own size)
};
#ifdef VULKAN
[[vk::push_constant]] ComputeConstants pc;
#else
ConstantBuffer<ComputeConstants> pc : register(b0);
#endif
RO(0) StructuredBuffer<float4> gSpectrum : register(t0);   // h0(k), conj(h0(-k)) per texel
RW(0) RWStructuredBuffer<float4> gInter : register(u0);    // after the row pass: 6 complex fields per texel
RW(1) RWStructuredBuffer<float4> gOut : register(u1);      // the cascades, 3 float4 per texel (see CSRipple)
RW(2) RWStructuredBuffer<float4> gSkyOut : register(u2);

groupshared float2 gsA[kN];
groupshared float2 gsB[kN];

float2 CMul(float2 a, float2 b) { return float2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x); }
float2 TimesI(float2 a) { return float2(-a.y, a.x); }

// The inverse DFT of the group's 256 values (thread i holds entry i), radix 2, decimation in time from a bit-reversed load.
float2 GroupIfft(float2 v, uint i) {
    GroupMemoryBarrierWithGroupSync(); // the last call's readers are done
    gsA[reversebits(i) >> (32u - kLogN)] = v;
    GroupMemoryBarrierWithGroupSync();
    [unroll] for (uint s = 0; s < kLogN; ++s) {
        const uint hs = 1u << s, span = hs << 1, start = i & ~(span - 1u), t = i & (span - 1u), j = t & (hs - 1u);
        float sn, cs;
        sincos(3.14159265 * float(j) / float(hs), sn, cs);
        float2 a, b;
        if ((s & 1u) == 0u) { a = gsA[start + j]; b = gsA[start + j + hs]; } else { a = gsB[start + j]; b = gsB[start + j + hs]; }
        const float2 wb = CMul(float2(cs, sn), b);
        const float2 o = t < hs ? a + wb : a - wb;
        if ((s & 1u) == 0u) gsB[i] = o; else gsA[i] = o;
        GroupMemoryBarrierWithGroupSync();
    }
    return (kLogN & 1u) != 0u ? gsB[i] : gsA[i]; // an even number of stages ends back in A
}

// One row of one cascade: the spectrum at this time, its derived fields packed two real fields to a complex one, FFT along x.
uint CascadeBase(uint cascade) { return (uint)(cascade == 0u ? pc.c[0].x : (cascade == 1u ? pc.c[0].y : pc.c[0].z)); }
[numthreads(OCEAN_N, 1, 1)]
void CSWaves(uint3 gid : SV_GroupID, uint3 tid : SV_GroupThreadID) {
    const uint row = gid.x, cascade = gid.y + (uint)pc.b.z, col = tid.x;
    const float L = cascade == 0u ? pc.a.y : (cascade == 1u ? pc.a.z : pc.a.w);
    const float time = pc.a.x, chop = pc.b.x, w0 = pc.b.y;
    const float2 k = (float2(col, row) - float(kN) * 0.5) * (kTau / L);
    const float kl = length(k);
    const float4 h0 = gSpectrum[CascadeBase(cascade) + row * kN + col];
    float2 h = 0;
    float2 kn = 0;
    if (kl > 1e-6) {
        const float omega = max(floor(sqrt(kG * kl) / w0 + 0.5), 1.0) * w0; // whole turns in the loop, so time can wrap
        float sn, cs;
        sincos(omega * time, sn, cs);
        h = CMul(h0.xy, float2(cs, sn)) + CMul(h0.zw, float2(cs, -sn));
        kn = k / kl;
    }
    const float2 ih = TimesI(h);
    const float2 dx = -ih * chop * kn.x, dz = -ih * chop * kn.y;          // choppy displacement
    const float2 sx = ih * k.x, sz = ih * k.y;                            // slopes
    const float2 dxx = h * chop * k.x * kn.x, dzz = h * chop * k.y * kn.y, dxz = h * chop * k.x * kn.y; // the displacement's derivatives
    const float2 hxx = -h * k.x * k.x, hzz = -h * k.y * k.y, hxz = -h * k.x * k.y;                       // the height's Hessian
    const float2 r0 = GroupIfft(h + TimesI(dx), col);
    const float2 r1 = GroupIfft(dz + TimesI(sx), col);
    const float2 r2 = GroupIfft(sz + TimesI(dxx), col);
    const float2 r3 = GroupIfft(dzz + TimesI(dxz), col);
    const float2 r4 = GroupIfft(hxx + TimesI(hzz), col);
    const float2 r5 = GroupIfft(hxz, col);
    const uint o = (CascadeBase(cascade) + row * kN + col) * 3u;
    gInter[o] = float4(r0, r1);
    gInter[o + 1u] = float4(r2, r3);
    gInter[o + 2u] = float4(r4, r5);
}

// One column: FFT along z, then the fields. Out per texel: (dx, h, dz, foam), (slope x, slope z, dDx/dx, dDz/dz), (hxx, hzz, hxz, dDx/dz).
// Foam: where the choppy surface folds (Jacobian low) it builds, and it fades where it does not.
[numthreads(OCEAN_N, 1, 1)]
void CSRipple(uint3 gid : SV_GroupID, uint3 tid : SV_GroupThreadID) {
    const uint col = gid.x, cascade = gid.y + (uint)pc.b.z, row = tid.x;
    const uint o = (CascadeBase(cascade) + row * kN + col) * 3u;
    const float4 a = gInter[o], b = gInter[o + 1u], c = gInter[o + 2u];
    const float sign = ((row + col) & 1u) != 0u ? -1.0 : 1.0; // the spectrum is centred on the tile: a checkerboard undoes the shift
    const float2 r0 = GroupIfft(a.xy, row) * sign, r1 = GroupIfft(a.zw, row) * sign, r2 = GroupIfft(b.xy, row) * sign;
    const float2 r3 = GroupIfft(b.zw, row) * sign, r4 = GroupIfft(c.xy, row) * sign, r5 = GroupIfft(c.zw, row) * sign;
    const float h = r0.x, dx = r0.y, dz = r1.x, sx = r1.y, sz = r2.x, dxx = r2.y, dzz = r3.x, dxz = r3.y;
    const float jacobian = (1.0 + dxx) * (1.0 + dzz) - dxz * dxz;
    // planar: per cascade three planes of n x n (the renderer copies each plane into a texture slice)
    const uint q = CascadeBase(cascade) * 3u + row * kN + col, plane = kN * kN;
    if (pc.b.w > 0.5) { // half precision, packed two texels to a float4 (8 bytes each): the renderer copies it into 16-bit float textures
        const uint e = q / 2u, hp = plane / 2u;
        const float4 pe = gOut[e];
        const float prevH = f16tof32(asuint((col & 1u) != 0u ? pe.w : pe.y) >> 16);
        const float foamH = saturate(max(prevH * pc.a.x, (pc.a.y - jacobian) * pc.a.z));
        const float4 v0 = float4(dx, h, dz, foamH), v1 = float4(sx, sz, dxx, dzz), v2 = float4(r4.x, r4.y, r5.x, dxz);
        const float2 p0 = asfloat(uint2(f32tof16(v0.x) | (f32tof16(v0.y) << 16), f32tof16(v0.z) | (f32tof16(v0.w) << 16)));
        const float2 p1 = asfloat(uint2(f32tof16(v1.x) | (f32tof16(v1.y) << 16), f32tof16(v1.z) | (f32tof16(v1.w) << 16)));
        const float2 p2 = asfloat(uint2(f32tof16(v2.x) | (f32tof16(v2.y) << 16), f32tof16(v2.z) | (f32tof16(v2.w) << 16)));
        if ((col & 1u) == 0u) { gOut[e].xy = p0; gOut[e + hp].xy = p1; gOut[e + 2u * hp].xy = p2; }
        else { gOut[e].zw = p0; gOut[e + hp].zw = p1; gOut[e + 2u * hp].zw = p2; }
        return;
    }
    const float prev = gOut[q].w;
    const float foam = saturate(max(prev * pc.a.x, (pc.a.y - jacobian) * pc.a.z));
    gOut[q] = float4(dx, h, dz, foam);
    gOut[q + plane] = float4(sx, sz, dxx, dzz);
    gOut[q + 2u * plane] = float4(r4.x, r4.y, r5.x, dxz);
}

// The sky table: the same single scattering as post.hlsl's SkyRadiance, a lat-long grid once per frame; a reflection costs one lookup.
[numthreads(8, 8, 1)]
void CSSky(uint3 id : SV_DispatchThreadID) {
    const float2 uv = (float2(id.xy) + 0.5) / float2(kSkyW, kSkyH);
    const float y = uv.y * 2.0 - 1.0;
    const float az = uv.x * kTau, sy = sqrt(saturate(1.0 - y * y));
    const float4 c = float4(SkyBaseFn(float3(cos(az) * sy, y, sin(az) * sy), pc.a.xyz, pc.b.xyz), 1.0);
    if (pc.b.w > 0.5) { // half precision, two texels to a float4 (see CSRipple)
        const uint i = id.y * kSkyW + id.x;
        const float2 p = asfloat(uint2(f32tof16(c.x) | (f32tof16(c.y) << 16), f32tof16(c.z) | (f32tof16(c.w) << 16)));
        if ((i & 1u) == 0u) gSkyOut[i / 2u].xy = p; else gSkyOut[i / 2u].zw = p;
        return;
    }
    gSkyOut[id.y * kSkyW + id.x] = c;
}
#endif

// A copy of the single-scattering sky in post.hlsl with its own constants.
static const float kEarth = 6360e3, kTop = 6420e3;
static const float3 kBetaR = float3(5.8e-6, 13.5e-6, 33.1e-6);
static const float kBetaM = 21e-6, kSunI = 12.0, kPI = 3.14159265;
float SphereFar(float3 o, float3 d, float r) {
    const float b = dot(o, d), c = dot(o, o) - r * r, h = b * b - c;
    return h < 0.0 ? -1.0 : -b + sqrt(h);
}
float2 OpticalToSun(float3 p, float3 l) {
    const float b0 = dot(p, l);
    if (b0 < 0.0 && dot(p, p) - b0 * b0 < kEarth * kEarth) return float2(1e9, 1e9);
    const float step = SphereFar(p, l, kTop) / 4.0;
    float2 od = 0;
    for (int i = 0; i < 4; ++i) od += exp(-max(length(p + l * (i + 0.5) * step) - kEarth, 0.0) / float2(8000.0, 1200.0)) * step;
    return od;
}
float3 Transmit(float2 od) { return exp(-(kBetaR * od.x + kBetaM * 1.1 * od.y)); }
float3 SkyBaseFn(float3 dIn, float3 eyePos, float3 sun) {
    const float3 o = float3(eyePos.x, max(eyePos.y, 1.0) + kEarth, eyePos.z);
    const float3 d = normalize(float3(dIn.x, max(dIn.y, 0.002), dIn.z));
    const float mu = dot(d, sun), g = 0.76;
    const float pr = 3.0 / (16.0 * kPI) * (1.0 + mu * mu);
    const float pm = 3.0 / (8.0 * kPI) * (1.0 - g * g) * (1.0 + mu * mu) / ((2.0 + g * g) * pow(1.0 + g * g - 2.0 * g * mu, 1.5));
    const float step = min(SphereFar(o, d, kTop), 150000.0) / 12.0;
    float2 od = 0;
    float3 sumR = 0, sumM = 0;
    for (int i = 0; i < 12; ++i) {
        const float3 p = o + d * (i + 0.5) * step;
        const float2 rho = exp(-max(length(p) - kEarth, 0.0) / float2(8000.0, 1200.0)) * step;
        od += rho;
        const float3 t = Transmit(od + OpticalToSun(p, sun));
        sumR += t * rho.x;
        sumM += t * rho.y;
    }
    return kSunI * (sumR * kBetaR * pr + sumM * kBetaM * pm) + float3(0.002, 0.003, 0.006);
}

// ------------------------------------------------------------------ the surface
#ifndef WATER_COMPUTE
GX(0) cbuffer PostParams : register(b0) { // the post chain's own constants, as ResolveScene filled them
    float4x4 invViewProj;
    float4x4 prevViewProj;
    float4 eye;
    float4 sunDir;
    float4 res;
    float4 misc;
};
struct LightingData {
    float4 sunDirL, sunColor, skyColor, groundColor;
    float4x4 shadowVP;
    float4 shadowParams, eyeL;
    float4x4 tightVP;
    float4 tightParams;
    float4 skySh[4];
    float4 skyGrad[3];
    float4 probeOrigin, probeGrid, probeUpdate, probePrev, lightDebug, cloudMap;
};
GX(1) ConstantBuffer<LightingData> gLight : register(b1);
struct WaterConstants {
    float4x4 viewProj; // jittered, as the scene was drawn
    float4 grid;       // the shore grid: world x, z of its corner; cell metres; cells along x
    float4 place;      // the clipmap's finest cell; the shore grid's cells along z; rings; time (s, wrapped)
    float4 tiles;      // metres per cascade (w is the renderer's)
    float4 extra;      // jitter x, y (clip space); a ring's half width in cells; debug view
};
#ifdef VULKAN
[[vk::push_constant]] WaterConstants wc;
#else
ConstantBuffer<WaterConstants> wc : register(b2);
#endif
GX(2) Texture2D<float> gDepth : register(t0);
GX(3) Texture2D<float4> gRefract : register(t1);
GX(4) Texture2D<float> gShadow : register(t2);
GX(5) SamplerState gClamp : register(s0);
GX(6) SamplerComparisonState gCmp : register(s1);
GX(7) StructuredBuffer<float4> gWaves : register(t5);
GX(8) StructuredBuffer<float4> gShore : register(t6); // 8 header float4, the grid (2 per cell), the foam tile
GX(9) StructuredBuffer<float4> gSky : register(t7);
// The same data as textures, for the hardware's filtering (the flight tier; the renderer copies the buffers in each frame)
GX(10) Texture2D<float4> gFoamTex : register(t8);
GX(11) Texture2DArray<float4> gWaveTex0 : register(t9);
GX(12) Texture2DArray<float4> gWaveTex1 : register(t10);
GX(13) Texture2DArray<float4> gWaveTex2 : register(t11);
GX(14) Texture2D<float4> gSkyTex : register(t12);
GX(15) SamplerState gWrap : register(s2);
GX(10) Texture2D<float4> gProbes : register(t3);
GX(11) Texture2D<float4> gCloudShadow : register(t4);

#define PROBE_ORIGIN gLight.probeOrigin
#define PROBE_GRID gLight.probeGrid
#include "probe.hlsli"
#include "city_air.hlsli"

// ---- the cascades: buffers, so bilinear by hand
uint WaveN(uint cascade) { const float4 h = gShore[7]; return (uint)(cascade == 0u ? h.y : (cascade == 1u ? h.z : h.w)); } // each cascade's grid (header H7)
bool MobileTier() { return gShore[7].x > 0.5; }
// The open sea in a floating-origin world (ocean.h InitDeep): the grid's corner holds the render origin's world position (every
// lookup is world-locked), the ring count's slot the sea level in render space.
bool DeepSea() { return gShore[4].w > 0.5; }
bool FlightTier() { return gShore[7].x > 1.5; } // tier 2: the sea from the air on a standalone headset
float4 WaveTexel(uint base, uint n, int2 t, uint field) { // planar: three planes of n x n per cascade, from 3 x its first texel
    const uint x = (uint)(t.x & int(n - 1u)), y = (uint)(t.y & int(n - 1u));
    return gWaves[base * 3u + field * n * n + y * n + x];
}
bool OceanTextures() { return gShore[7].x > 1.5; } // the flight tier reads the textures
float4 WaveTex(uint cascade, float2 world, uint field) { // one filtered fetch; FFT sample j sits at j / n, a texel centre at (j + 0.5) / n
    const float4 h = gShore[7];
    if (cascade == 0u) return gWaveTex0.SampleLevel(gWrap, float3(world / wc.tiles.x + 0.5 / h.y, field), 0.0);
    if (cascade == 1u) return gWaveTex1.SampleLevel(gWrap, float3(world / wc.tiles.y + 0.5 / h.z, field), 0.0);
    return gWaveTex2.SampleLevel(gWrap, float3(world / wc.tiles.z + 0.5 / h.w, field), 0.0);
}
float4 WaveNearest(uint cascade, float2 world, uint field) { // one read: the copy-free flight pipeline's ripples (TAA's jitter filters them)
    if (OceanTextures()) return WaveTex(cascade, world, field); // a filtered fetch costs the same
    const float4 h = gShore[7];
    const uint n0 = (uint)h.y, n1 = (uint)h.z, n = (uint)(cascade == 0u ? h.y : (cascade == 1u ? h.z : h.w));
    const uint base = cascade == 0u ? 0u : (cascade == 1u ? n0 * n0 : n0 * n0 + n1 * n1);
    const float2 p = world / (cascade == 0u ? wc.tiles.x : (cascade == 1u ? wc.tiles.y : wc.tiles.z)) * float(n);
    return WaveTexel(base, n, int2(floor(p + 0.5)), field);
}
float TileMetres(uint cascade) { return cascade == 0u ? wc.tiles.x : (cascade == 1u ? wc.tiles.y : wc.tiles.z); } // a branch: fxc misread tiles[cascade]
float4 WaveSample(uint cascade, float2 world, uint field) {
    if (OceanTextures()) return WaveTex(cascade, world, field);
    const float4 h = gShore[7]; // the grids, once per sample
    const uint n0 = (uint)h.y, n1 = (uint)h.z, n = (uint)(cascade == 0u ? h.y : (cascade == 1u ? h.z : h.w));
    const uint base = cascade == 0u ? 0u : (cascade == 1u ? n0 * n0 : n0 * n0 + n1 * n1);
    const float2 p = world / TileMetres(cascade) * float(n); // the FFT's sample j sits at j L / n: no half-texel shift, so every grid size shows the same sea in the same place
    const float2 fl = floor(p), f = p - fl;
    const int2 c = int2(fl);
    return lerp(lerp(WaveTexel(base, n, c, field), WaveTexel(base, n, c + int2(1, 0), field), f.x),
                lerp(WaveTexel(base, n, c + int2(0, 1), field), WaveTexel(base, n, c + int2(1, 1), field), f.x), f.y);
}

// ---- the shore grid
struct Shore {
    float4 a;   // bed height, the shallowest depth on the way in, train 1 phase, its set phase
    float4 b;   // train 2 phase, its set phase, train 1's phase and set phase at this column's shoreline
    float4 c;   // what the depth alone decides: train 1's wavenumber and shoaling, train 2's
    float2 grad; // the bed's slope
};
static const uint kCellF4 = 3u; // float4s per shore cell (ocean.h)
Shore ShoreAt(float2 p) {
    const float cell = wc.grid.z;
    const int nx = (int)wc.grid.w, nz = (int)wc.place.y;
    float2 g = (p - wc.grid.xy) / cell - 0.5;
    const float beyond = max(g.y - float(nz - 1), 0.0) * cell; // metres past the offshore edge: open water, the phases run on
    g = clamp(g, 0.0, float2(nx - 1, nz - 1) - 0.001);
    const int2 c = int2(floor(g));
    const float2 f = g - float2(c);
    const uint i00 = 8u + (uint)(c.y * nx + c.x) * kCellF4, i01 = i00 + (uint)nx * kCellF4;
    const float4 a00 = gShore[i00], b00 = gShore[i00 + 1u], c00 = gShore[i00 + 2u], a10 = gShore[i00 + 3u], b10 = gShore[i00 + 4u], c10 = gShore[i00 + 5u];
    const float4 a01 = gShore[i01], b01 = gShore[i01 + 1u], c01 = gShore[i01 + 2u], a11 = gShore[i01 + 3u], b11 = gShore[i01 + 4u], c11 = gShore[i01 + 5u];
    Shore s;
    s.a = lerp(lerp(a00, a10, f.x), lerp(a01, a11, f.x), f.y);
    s.b = lerp(lerp(b00, b10, f.x), lerp(b01, b11, f.x), f.y);
    s.c = lerp(lerp(c00, c10, f.x), lerp(c01, c11, f.x), f.y);
    s.grad = float2(lerp(a10.x - a00.x, a11.x - a01.x, f.y), lerp(a01.x - a00.x, a11.x - a10.x, f.x)) / cell;
    if (beyond > 0.0) {
        const float4 h1 = gShore[1], h3 = gShore[3];
        s.a.z += h1.z * beyond; s.a.w += h1.w * beyond;
        s.b.x += h3.z * beyond; s.b.y += h3.w * beyond;
        s.a.x -= beyond * 0.02;
        s.a.y = max(s.a.y, -s.a.x);
    }
    return s;
}

// ---- foam: the tile in the shore buffer
float4 FoamTexel(int2 t) {
    const uint base = 8u + (uint)wc.grid.w * (uint)wc.place.y * kCellF4;
    return gShore[base + (uint)(t.y & int(kFoamSize - 1)) * kFoamSize + (uint)(t.x & int(kFoamSize - 1))];
}
float4 FoamNearest(float2 uv) { // one read: the flight tier, TAA does the filtering (with the texture: filtered, the same cost)
    if (gShore[7].x > 1.5) return gFoamTex.SampleLevel(gWrap, uv, 0.0);
    return FoamTexel(int2(floor(uv * float(kFoamSize))));
}
float4 FoamSample(float2 uv) {
    if (gShore[7].x > 1.5) return gFoamTex.SampleLevel(gWrap, uv, 0.0);
    const float2 p = uv * float(kFoamSize) - 0.5, fl = floor(p), f = p - fl;
    const int2 c = int2(fl);
    return lerp(lerp(FoamTexel(c), FoamTexel(c + int2(1, 0)), f.x), lerp(FoamTexel(c + int2(0, 1)), FoamTexel(c + int2(1, 1)), f.x), f.y);
}
// White water is a churning pile, not paint: its lumps, a few tens of centimetres across, roll along with the bore.
float FoamLump(float2 p, float2 drift) {
    return FoamSample(p / 2.7 + drift * 0.37).z * 0.65 + FoamSample(p / 0.95 + drift * 1.05 + 0.5).x * 0.35;
}
float KDepth(float w, float d) { // Eckart's closed form of w^2 = g k tanh(k d)
    const float k0 = w * w / kG;
    return k0 / sqrt(tanh(k0 * max(d, 0.01)));
}
float Shoaling(float w, float k, float d) { // sqrt(Cg deep / Cg here): what is left of a wave's energy flux, kept
    const float kd = k * max(d, 0.01);
    const float n = kd > 8.0 ? 0.5 : 0.5 * (1.0 + 2.0 * kd / sinh(2.0 * kd));
    return sqrt((0.5 * kG / w) / (n * w / k));
}
float Envelope(float4 h0, float4 h1, float t, float setPhase, float x) { // a wave group: the sets
    const float e = 0.5 + 0.5 * cos(h0.w * t + setPhase + h1.x * x);
    return 1.0 - h1.y + h1.y * e * sqrt(e);
}
float Wrap(float a) { return a - kTau * floor(a / kTau + 0.5); }

struct Surf {
    float3 d;        // displacement at the rest point: x, z sideways, y the height above the still level
    float2 grad;     // the height's slope over the rest coordinates
    float4 m;        // the sideways displacement's derivatives: dDx/dx, dDx/dz, dDz/dx, dDz/dz
    float breaking;  // how much the depth is cutting the waves here, 0..1
    float roller;    // white water on the front of a breaking crest
    float trail;     // foam the last crest left
    float crest;     // 0 trough .. 1 crest, of the biggest train
    float lift;      // the crest's height over its amplitude, for light through it
    float bed, depth;
    float film;      // the swash sheet's thickness on the sand, metres
    float wet, gloss; // the sand: darkened by water, and still shining with it
    float swashFoam, swashEdge;
    float swashMargin; // how far the covering sheet's front stands above this point (m; < 0 short of it): the mobile tier's wet edge
    float sheetA;      // that sheet's thickness scale, and its age as exp(-tau / 3.5), negative once it drains (no bubbly front)
    float sheetAge;
    float reach;       // how far the last waves' highest run-up stands above this point: wet sand below it
    float2 dir;      // the biggest train's travel
    float zone;      // the surf zone
    float phase;     // train 1's phase, wrapped to a turn
    float curl;      // a plunging breaker's lip here: 0 none .. 1 the overhang at its fullest
    float lip;       // the thin sheet of the lip itself (light goes through it), 0..1
    float lipFoam;   // the lip's edge, torn into spray
    float lipW;      // how much of the surface here the lip has moved
    float tube;      // the tube's back wall, in the roof's shade
    float ratio;     // the waves' height over what the depth allows
    float stage;     // how far the plunge has got: 0 the crest just pitching .. 1 the lip landing
};

void TrainAt(float4 h0, float4 h1, float phase, float setPhase, float x, float t, float2 kShoal, out float amp, out float theta, out float2 g, out float k) {
    theta = h0.y * t + phase + h0.z * x;
    k = kShoal.x; // KDepth and Shoaling at this depth, from the shore grid
    g = float2(h0.z, sqrt(max(k * k - h0.z * h0.z, 1e-6))); // the phase's gradient: Snell's alongshore part and what is left across
    amp = h0.x * Envelope(h0, h1, t, setPhase, x) * kShoal.y;
}
// Swell is never even along its crests: peaks and lulls a few tens of metres long, drifting, so waves break in sections.
float CrestVariation(float x, float t, float seed) {
    return 1.0 + 0.24 * sin(x * 0.083 + t * 0.07 + seed) * sin(x * 0.031 - t * 0.045 + 1.3 * seed) + 0.1 * sin(x * 0.21 + t * 0.13 + 2.0 * seed);
}

// Gerstner: the sideways motion pulls points toward each crest, so crests sharpen and troughs flatten (a trochoid).
void Gerstner(float a, float q, float theta, float2 g, float k, inout Surf s) {
    float sn, cs;
    sincos(theta, sn, cs);
    const float2 dir = -g / k;
    const float side = q / k;
    s.d.y += a * cs;
    s.d.xz += dir * side * sn;
    s.grad += -a * sn * g;
    const float c2 = side * cs;
    s.m += float4(dir.x * g.x, dir.x * g.y, dir.y * g.x, dir.y * g.y) * c2;
}

// A plunging breaker, in the crest's own frame (f forward toward the shore, y up, both from the crest's rest point; th train 1's phase,
// negative on the face). The crest is wrapped round a tube: the face's upper part stands up as the tube's back wall, the water over the
// crest becomes its roof, the lip; the tip pitches forward and down as the stage runs on. Points after the tip come back along the lip's
// top, thicker, and blend into the wave's back. Returns the change to the plain wave's position and how much of it applies.
float2 BreakerBase(float th, float a, float k, float q, float lean, out float2 dBase) { // train 1 on its own: Gerstner and the forward lean, as SurfAt has them; and its slope in th
    float sn, cs, sh, ch;
    sincos(th, sn, cs);
    sincos(th + 0.45, sh, ch);
    const float hump = 0.5 + 0.5 * ch;
    dBase = float2(-1.0 / k + q / k * cs - 1.5 * lean * hump * hump * sh, -a * sn);
    return float2(-th / k + q / k * sn + lean * hump * hump * hump, a * cs);
}
float SmoothstepD(float e0, float e1, float x) { const float u = saturate((x - e0) / (e1 - e0)); return 6.0 * u * (1.0 - u) / (e1 - e0); }
// The warp's change to the plain wave's position (xy: forward, up), already weighted, and its derivative in th (for the slope).
float2 LipWarp(float th, float a, float k, float q, float lean, float stage, out float2 dWarp, out float wgt, out float under, out float p, out float pq) {
    const float H = 2.0 * a;
    const float R = H * (0.3 + 0.15 * stage);                 // the tube
    const float yc = a + 0.1 * H - R;
    const float t0 = -acos(clamp(yc / a, -1.0, 1.0)), tt = 0.3, te = tt + 1.0;
    float2 unused;
    const float fc = BreakerBase(t0, a, k, q, lean, unused).x + R; // its back wall stands where the face reaches the tube's centre height
    const float atip = radians(100.0 + 120.0 * stage);         // the lip: level at first, then pitching down
    p = saturate((th - t0) / (tt - t0));
    pq = saturate((th - tt) / (te - tt));
    under = th < tt ? 1.0 : 0.0;
    float al, dal, r, dr;
    if (th < tt) {
        al = atip * p; dal = (p > 0.0 && p < 1.0) ? atip / (tt - t0) : 0.0;
        r = R; dr = 0.0;
    } else {
        const float pc = max(pq, 1e-3), a35 = radians(35.0), thick = 0.35 * R * (1.0 - 0.5 * stage);
        al = atip + (a35 - atip) * pow(pc, 0.8); dal = pq < 1.0 ? (a35 - atip) * 0.8 * pow(pc, -0.2) / (te - tt) : 0.0;
        r = R + thick * sqrt(pc); dr = pq < 1.0 ? thick * 0.5 / sqrt(pc) / (te - tt) : 0.0;
    }
    float sa, ca;
    sincos(al, sa, ca);
    const float2 target = float2(fc - r * ca, yc + r * sa);
    const float2 dTarget = float2(-dr * ca + r * sa * dal, dr * sa + r * ca * dal);
    float2 dBase;
    const float2 warp = target - BreakerBase(th, a, k, q, lean, dBase);
    const float s1 = smoothstep(t0 - 0.8, t0, th), s2 = smoothstep(te, te + 1.0, th);
    wgt = s1 * (1.0 - s2);
    const float dW = SmoothstepD(t0 - 0.8, t0, th) * (1.0 - s2) - s1 * SmoothstepD(te, te + 1.0, th);
    dWarp = (dTarget - dBase) * wgt + warp * dW;
    return warp * wgt;
}

Surf SurfAt(float2 p, float t, float cellFade, float lipFade, float grainAmt = 1.0) {
    Surf s = (Surf)0;
    s.swashMargin = -1.0;
    s.reach = -1.0;
    const Shore sh = ShoreAt(p);
    const float4 H0 = gShore[0], H1 = gShore[1], H2 = gShore[2], H3 = gShore[3], H4 = gShore[4];
    s.bed = sh.a.x;
    s.depth = -sh.a.x;
    const float dw = max(s.depth, 0.04);
    float a1, a2, th1, th2, k1, k2;
    float2 g1, g2;
    TrainAt(H0, H1, sh.a.z, sh.a.w, p.x, t, sh.c.xy, a1, th1, g1, k1);
    TrainAt(H2, H3, sh.b.x, sh.b.y, p.x, t, sh.c.zw, a2, th2, g2, k2);
    a1 *= CrestVariation(p.x, t, 0.0);
    a2 *= CrestVariation(p.x * 0.7, t, 2.1);
    // McCowan: no wave higher than gamma times the depth; the shallowest depth on the way in keeps broken waves small after the bar
    const float dmin = sh.a.y;
    const float dEff = min(dw, dmin * 1.08 + 0.28 * max(dw - dmin, 0.0));
    const float lim = 0.5 * H4.x * dEff + 0.035;
    const float E = a1 + a2;
    const float ratio = E / lim;
    const float cut = pow(1.0 + pow(ratio, 6.0), -1.0 / 6.0); // a soft minimum
    s.breaking = saturate((ratio - 0.92) * 1.3);
    s.zone = saturate(1.0 - dmin / 2.4) * saturate(s.depth * 4.0);
    const float sea = smoothstep(-0.12, 0.35, s.depth); // up the beach the swash takes over from the waves
    a1 *= cut * sea * cellFade;
    a2 *= cut * sea * cellFade;
    const float q1 = min(0.92, k1 * a1 * (1.05 + 1.4 * s.breaking));
    const float q2 = min(0.8, k2 * a2 * 1.05);
    Gerstner(a1, q1, th1, g1, k1, s);
    Gerstner(a2, q2, th2, g2, k2, s);
    // a breaking crest pitches forward: its front steepens past what a trochoid does
    const float2 dir1 = -g1 / k1;
    float sn, cs;
    sincos(th1 + 0.45, sn, cs);
    const float lean = 0.55 * a1 * s.breaking;
    const float hump = 0.5 + 0.5 * cs;
    s.d.xz += dir1 * lean * hump * hump * hump;
    s.m += float4(dir1.x * g1.x, dir1.x * g1.y, dir1.y * g1.x, dir1.y * g1.y) * (lean * 3.0 * hump * hump * (-0.5 * sn));
    s.dir = dir1;
    const float w1 = Wrap(th1);
    // ---- the plunge: from the break point the lip pitches out over the face and falls; a few metres on it has landed and the wave
    // goes on as a bore (the roller below). The stage runs on how far past breaking the depth has cut this wave.
    s.ratio = ratio;
    s.stage = saturate((ratio - 1.0) / 1.5); // past the break point the ratio climbs to 2.5 across the outer surf zone
    s.curl = smoothstep(0.0, 0.12, s.stage) * (1.0 - smoothstep(0.72, 1.0, s.stage)) * sea * saturate(a1 * 5.0 - 0.5);
    if (s.curl > 0.0 && abs(w1) < 2.6) {
        float under, pu, pq, W;
        float2 dWarp;
        const float2 warp = LipWarp(w1, a1, k1, q1, lean, s.stage, dWarp, W, under, pu, pq);
        const float wgt = W * s.curl;
        s.d.xz += dir1 * warp.x * s.curl * lipFade; // the fold drawn only where the cells can carry it; shaded everywhere
        s.d.y += warp.y * s.curl * lipFade;
        s.lipW = wgt;
        // its slope: the phase runs along the travel at -k1 per metre, so the warp's derivative goes into the sideways map and the
        // height's gradient like a Gerstner term, and the sheet's fold shows as det < 0
        const float2 dW = dWarp * s.curl * lipFade;
        s.m += float4(dir1.x * g1.x, dir1.x * g1.y, dir1.y * g1.x, dir1.y * g1.y) * dW.x;
        s.grad += g1 * dW.y;
        s.lip = wgt * (under > 0.5 ? smoothstep(0.3, 0.65, pu) : 1.0 - smoothstep(0.0, 0.35, pq)); // the roof is a thin sheet; its top only near the tip
        s.tube = wgt * under * smoothstep(0.0, 0.25, pu) * (1.0 - smoothstep(0.55, 0.9, pu)); // the back wall, under the roof
        s.lipFoam = wgt * smoothstep(0.3, 0.9, s.stage) * (under > 0.5 ? smoothstep(0.94, 1.0, pu) : 1.0 - smoothstep(0.0, 0.1, pq));
    }
    s.phase = frac(th1 / kTau);
    s.crest = 0.5 + 0.5 * cos(w1);
    s.lift = saturate(s.d.y / max(a1 + a2, 0.05));
    // white water: down the face in front of a breaking crest, then a trail that thins behind it
    const float front = smoothstep(-1.6, -0.25, w1) * (1.0 - smoothstep(0.05, 0.7, w1));
    const float since = w1 >= 0.0 ? w1 : w1 + kTau;
    s.roller = s.breaking * front * sea * (1.0 - 0.9 * s.curl); // while the lip is in the air the face is glassy
    s.trail = s.breaking * exp(-since / 1.9) * sea * (1.0 - s.curl); // a wave still curling has not broken yet: no trail behind it

    // ---- the swash: each wave that reaches the shoreline runs up the beach face and drains back down it
    // nothing higher ever gets wet: the biggest run-up tops out at z0 + runup. The mobile tier's vertices (no sand noise) look a little
    // past both ends, so what they hand on interpolates right up to where the pixels decide
    const float bandOut = grainAmt < 0.5 ? 0.3 : 0.0;
    if (s.bed > -0.35 - bandOut && s.bed < H4.y + 0.05 + bandOut) {
        const float4 grain = FoamSample(p / 6.1 + 0.13); // borrowed noise: the sand's centimetres of relief, so the wet line wanders
        const float noiseZ = (grain.z - 0.5) * 0.06 + (grain.w - 0.5) * 0.015;
        const float zp = s.bed + noiseZ * grainAmt;
        const float zpW = s.bed + noiseZ; // the wet sand left behind always takes the noise: its patches are tens of centimetres, vertices hold them
        const float T1 = kTau / H0.y;
        const float ths = H0.y * t + sh.b.z + H0.z * p.x;
        const float tau0 = frac(ths / kTau) * T1;
        const float sb = clamp(length(sh.grad), 0.035, 0.3);
        const float z0 = -0.1;
        [unroll] for (int w = 0; w < 3; ++w) {
            const float tau = tau0 + float(w) * T1;
            const float env = Envelope(H0, H1, t - tau, sh.b.w, p.x);
            const float R = H4.y * env * env;              // this wave's run-up, metres up from z0
            const float U0 = sqrt(2.0 * kG * R), tm = U0 / (kG * sb);
            const float back = 0.25 * kG * sb * sb;       // the backwash drains at half the uprush's deceleration
            const float zf = tau < tm ? z0 + U0 * sb * tau - 0.5 * kG * sb * sb * tau * tau : z0 + R - back * (tau - tm) * (tau - tm);
            const float tEnd = tm + sqrt(R / back);
            if (tau < tEnd && zf - zp > s.swashMargin) {
                s.swashMargin = zf - zp;
                s.sheetA = 0.012 + 0.07 * R * saturate(1.0 - tau / tEnd);
                s.sheetAge = exp(-tau / 3.5) * (tau < tm * 1.1 ? 1.0 : -1.0);
            }
            s.reach = max(s.reach, z0 + R - zp);
            if (zp < zf && tau < tEnd) {
                const float sheet = (0.012 + 0.07 * R * saturate(1.0 - tau / tEnd)) * pow(saturate((zf - zp) / 0.2), 0.55);
                s.film = max(s.film, sheet);
                s.wet = 1.0; s.gloss = 1.0;
                const float edge = saturate(1.0 - (zf - zp) / 0.045);
                if (tau < tm * 1.1) s.swashEdge = max(s.swashEdge, edge); // the bubbly front of the uprush
                s.swashFoam = max(s.swashFoam, exp(-tau / 3.5) * (0.35 + 0.65 * edge));
            } else if (zpW < z0 + R) {
                const float tu = tm + sqrt(max(z0 + R - zpW, 0.0) / back); // when the draining front passed this point
                if (tau > tu) {
                    const float dry = tau - tu;
                    s.gloss = max(s.gloss, exp(-dry / H4.z));
                    s.wet = max(s.wet, exp(-dry / 30.0));
                }
            }
        }
        s.swashFoam *= saturate(1.0 - s.depth * 3.0);
    }
    return s;
}

float3 FftDepthFade(float depth) { return saturate(float3(depth / 7.0, depth / 0.9, depth / 0.22)); } // short waves live in shallower water

// ---- dynamic relief (steep parallax occlusion) for the waves the mesh is too coarse to carry
// The vertex shader fades each cascade out of the mesh as its cells grow (CascadeLod); what it left out is still in the pixel shader's
// normals, but not in where things are: the backs of small waves would show at a grazing view, their fronts would not stand up. So the
// pixel marches its view ray through that leftover height, from its top bound down (Tatarchuk, "Dynamic Parallax Occlusion Mapping",
// 2006; steps follow how many pixels the relief spans, then one secant step), and shades the point the ray really meets.
// The mobile tier: how much of a cascade's slope the vertices hand on (their spacing against the cascade's texels, as the pixel shader's
// texels); the pixel shader adds the rest where its footprint can show it.
// A pixel's footprint on the water (the geometric mean of its two axes, as WaterColor takes it from derivatives), from the camera alone,
// so the vertex shader can know it too: the distance over the projection's scale, stretched by the grazing angle.
float FootprintAt(float3 world) {
    const float3 d = world - eye.xyz;
    const float dist = max(length(d), 1e-3);
    const float pxAngle = 2.0 / (length(wc.viewProj[1].xyz) * res.y); // row 1 of P V is P's y scale along the view's up
    return dist * pxAngle / sqrt(max(abs(d.y) / dist, 0.03));
}
// vertices no further apart than a texel (no aliasing), and only what a pixel there can show (the rest is roughness, as on the desktop)
float VertexCarries(uint c, float cell, float3 world) {
    const float texel = TileMetres(c) / float(WaveN(c));
    return saturate(2.0 - cell / texel) * saturate(2.5 - FootprintAt(world) / (TileMetres(c) / 256.0 * 1.5));
}
float CascadeLod(uint c, float cell) { return saturate(TileMetres(c) / (12.0 * cell) - 1.0); } // as much of a cascade as cells this size carry
float2 WaterRelief(float2 rest, float3 v, float cell, float footprint) {
    const float3 deep = FftDepthFade(-ShoreAt(rest).a.x);
    const float4 rms = gShore[6];
    const float r1 = deep.y * (1.0 - CascadeLod(1u, cell)), r2 = deep.z * (1.0 - CascadeLod(2u, cell));
    const float A = 2.5 * (r1 * rms.y + r2 * rms.z);  // the leftover relief's bound above and below the mesh
    const float vy = max(v.y, 0.06);
    const float span = 2.0 * A / vy;                  // how far the ray runs sideways while it crosses the relief
    if (span < footprint * 1.5 || A < 0.002) return rest; // under a pixel and a half: nothing to see
    const int steps = clamp(int(span / (footprint * 2.0)), 3, 10);
    const float dt = span / float(steps) / 1.0;
    float tPrev = A / vy, gPrev = A; // the ray's height over the mesh minus the relief's, at the top bound
    [loop] for (int k = 1; k <= steps; ++k) {
        const float tk = A / vy - dt * float(k);
        const float2 p = rest + v.xz * tk;
        float h = 0.0;
        if (r1 > 0.0) h += WaveSample(1u, p, 0).y * r1;
        if (r2 > 0.0) h += WaveSample(2u, p, 0).y * r2;
        const float gap = v.y * tk - h;
        if (gap <= 0.0) { // between the last two steps, by similar triangles
            const float tHit = tPrev - (tPrev - tk) * gPrev / max(gPrev - gap, 1e-5);
            return rest + v.xz * tHit;
        }
        tPrev = tk; gPrev = gap;
    }
    return rest + v.xz * (-A / vy);
}

float LipFade(float cell) { return saturate(1.6 - cell / 0.5); } // a lip only where the cells can draw it

struct VsIn {
    float3 pos : POSITION;  // the clipmap: x, z the vertex in its ring's cells, y the ring
    float3 normal : NORMAL;
    float4 color : COLOR;
    float2 flow : TEXCOORD1;
};
struct VsOut {
    float4 pos : SV_Position;
    float3 world : TEXCOORD0;
    float2 rest : TEXCOORD1;  // the undisplaced point: every lookup in the pixel shader uses it, so it stays on the surface it moved
    float cell : TEXCOORD2;
    float3 hazeLight : TEXCOORD3; // the air between the camera and this point, as the composite computes it, per vertex
    float3 hazeKeep : TEXCOORD4;
    // the mobile tier: the surf as the vertex worked it out, for the pixel shader to take as it is (zero on the desktop tier)
    float4 p0 : TEXCOORD5;  // the slope over the rest coordinates (swell, lip, the cascades the cells carry); the sideways map's dDx/dx, dDz/dz
    float4 p1 : TEXCOORD6;  // dDx/dz, dDz/dx; the height; folding (whitecaps)
    float4 p2 : TEXCOORD7;  // roller, trail, breaking, surf zone
    float4 p3 : TEXCOORD8;  // lip, tube, lip foam, lip weight
    float4 p4 : TEXCOORD9;  // travel direction; bed; lift
    float4 p5 : TEXCOORD10; // swash sheet, wet, gloss, swash foam
    float4 p6 : TEXCOORD11; // swash margin, curl, depth, reach
    float4 p7 : TEXCOORD12; // the flight tier: what the water passes of the bed behind it (rgb)
    float4 p8 : TEXCOORD13; // and the light it scatters toward the eye (rgb)
};

// The composite's aerial scattering, then its ground dust without probes in it (a mean colour and the sun).
void AirTo(float3 world, out float3 light, out float3 keep) {
    light = 0;
    keep = 1;
    if (CityAirOn(misc.z)) {
        const float k = CityAirKeep(eye.xyz, world);
        light = CityHazeOf(normalize(world - eye.xyz), gLight.skyGrad[1].rgb, gLight.sunColor.rgb, sunDir.xyz) * (1.0 - k);
        keep = k;
        return;
    }
    if (misc.z < 0.5) return;
    const float3 away = world - eye.xyz;
    const float dist = length(away);
    const float3 d = away / max(dist, 1e-3);
    const float3 o = float3(eye.x, max(eye.y, 1.0) + kEarth, eye.z);
    const float mu = dot(d, sunDir.xyz), g = 0.76;
    const float pr = 3.0 / (16.0 * kPI) * (1.0 + mu * mu);
    const float pm = 3.0 / (8.0 * kPI) * (1.0 - g * g) * (1.0 + mu * mu) / ((2.0 + g * g) * pow(1.0 + g * g - 2.0 * g * mu, 1.5));
    const int steps = FlightTier() ? 2 : 4; // the flight tier: two steps (the haze over a few kilometres of sea air is smooth)
    const float step = min(dist * 2.5, 150000.0) / float(steps);
    float2 od = 0;
    float3 sumR = 0, sumM = 0;
    for (int i = 0; i < steps; ++i) {
        const float3 p = o + d * (i + 0.5) * step;
        const float2 rho = exp(-max(length(p) - kEarth, 0.0) / float2(8000.0, 1200.0)) * step;
        od += rho;
        const float3 t = Transmit(od + OpticalToSun(p, sunDir.xyz));
        sumR += t * rho.x;
        sumM += t * rho.y;
    }
    const float3 trans = Transmit(od);
    light = kSunI * (sumR * kBetaR * pr + sumM * kBetaM * pm);
    keep = trans;
    const float kDensity = 0.0005, kFalloff = 1.0 / 45.0;
    const float groundY = gLight.probeOrigin.y * gLight.probeGrid.w + 3.0 * gLight.probeGrid.w - 3.0;
    const float h0 = max(eye.y - groundY, 0.0), dy = d.y * kFalloff * dist;
    const float T = exp(-kDensity * exp(-h0 * kFalloff) * dist * (abs(dy) > 1e-3 ? (1.0 - exp(-dy)) / dy : 1.0));
    const float gg = 0.6;
    const float hg = (1.0 - gg * gg) / (4.0 * kPI * pow(1.0 + gg * gg - 2.0 * gg * mu, 1.5));
    light = light * T + (gLight.skySh[0].rgb + gLight.sunColor.rgb * 3.2 * kPI * hg) * (1.0 - T);
    keep *= T;
}

float3 SkyAmbient(float3 n); // in the light section below
VsOut WaterVS(VsIn i) {
    VsOut o;
    const float level = i.pos.y, n = wc.extra.z;
    const float cell = wc.place.x * exp2(level);
    const float2 local = i.pos.xz * cell;
    // each ring sits on its own world grid; its outer quarter morphs onto the next ring's, which its outer edge already is
    const float cheb = max(abs(i.pos.x), abs(i.pos.z)) / n;
    const float morph = saturate((cheb - 0.74) / 0.25);
    const float2 at = eye.xz + local;
    const float2 p0 = round(at / cell) * cell, p1 = round(at / (2.0 * cell)) * (2.0 * cell);
    const float2 restR = lerp(p0, p1, morph);              // where the vertex is, in render space
    const bool deepSea = DeepSea();
    const float2 rest = deepSea ? restR + wc.grid.xy : restR; // where the waves are looked up: the world's place
    const float seaY = deepSea ? wc.place.z : 0.0;
    const float cellHere = cell * (1.0 + morph);
    const float t = wc.place.w;
    const bool mobile = MobileTier();
    // swell too long for a coarse ring's cells is left flat; the mobile tier's swash takes the sand's noise per pixel instead
    const Surf s = SurfAt(rest, t, saturate(70.0 / cellHere - 1.0), LipFade(cellHere), mobile ? 0.0 : 1.0);
    float3 d = s.d;
    const float3 deep = FftDepthFade(s.depth);
    float2 grad = s.grad;
    float4 m = s.m;
    float height = s.d.y, folds = 0.0;
    const float3 film3 = s.film > 0.003 ? float3(0.0, 0.25, 0.5) : float3(0.0, 0.0, 0.0);
    [unroll] for (uint c = 0; c < kCascades; ++c) {
        const float lod = CascadeLod(c, cellHere); // a cascade only where the ring's cells can carry it
        const float dfade = c == 0u ? deep.x : (c == 1u ? deep.y : deep.z);
        const float fade = dfade * lod;
        if (mobile) {
            // the mobile tier's pixel shader takes the height, the folding and the slopes of what the cells can carry from here
            if (dfade > 0.0) {
                const float4 f0 = WaveSample(c, rest, 0);
                d += f0.xyz * fade;
                height += f0.y * dfade;
                folds = max(folds, f0.w * dfade);
            }
            const float carried = VertexCarries(c, cellHere, float3(restR.x + d.x, s.d.y + seaY, restR.y + d.z)) * max(dfade, c == 0u ? film3.x : (c == 1u ? film3.y : film3.z));
            if (carried > 0.0) {
                const float4 f1 = WaveSample(c, rest, 1);
                grad += f1.xy * carried;
                m += float4(f1.z, 0.0, 0.0, f1.w) * carried;
            }
        } else if (fade > 0.0) d += WaveSample(c, rest, 0).xyz * fade;
    }
    const float lumpH = (s.roller * 0.24 + s.trail * 0.05) * saturate(0.6 / cellHere); // only where the cells can carry it
    if (lumpH > 0.001) d.y += lumpH * (FoamLump(rest, s.dir * t * 0.35) - 0.35);
    float y = d.y;
    // the beach: the swash sheet, and sand still wet from it, ride just above the bed (the depth test hides what is under the sand)
    // Everything the run-up can reach rides there, wet or not: the pixel shader draws the sheet and the wet sand, and lets the dry sand be,
    // so the wet line is drawn per pixel and not along the mesh's edges.
    if (y < s.bed + 0.004 && s.bed < gShore[4].y + 0.08) y = s.bed + max(s.film, 0.0) + 0.03;
    const float3 p = float3(restR.x + d.x, y + seaY, restR.y + d.z);
    o.pos = mul(wc.viewProj, float4(p, 1.0));
    o.world = p;
    o.rest = rest;
    o.cell = cellHere;
    AirTo(p, o.hazeLight, o.hazeKeep);
    o.p0 = o.p1 = o.p2 = o.p3 = o.p4 = o.p5 = o.p6 = o.p7 = o.p8 = float4(0, 0, 0, 0);
    if (FlightTier()) {
        // The flight tier's water body, per vertex: absorption and single scattering for a flat surface over the shore grid's depth.
        // It changes over metres; the vertices are a few pixels apart from the air. The pixel adds the bed's picture, the waves' light.
        const float3 v = normalize(eye.xyz - p);
        const float3 sunRgb = gLight.sunColor.rgb * 3.2;
        const float shadowV = 1.0; // the shadow tables are the pixel shader's: it scales the sunlit share by its own lookup
        const float3 rd = refract(-v, float3(0, 1, 0), 0.75);
        const float rdy = max(-rd.y, 0.05);
        const float waterDepth = max(s.depth + height, 0.0);
        const float path = min(waterDepth / rdy, 400.0);
        const float stir = saturate(s.zone * (0.25 + 0.6 * s.breaking) * saturate(s.depth / 0.9) + s.roller * 0.6) * (1.0 - 0.85 * s.lip);
        const float3 absorb = float3(0.42, 0.064, 0.038) + float3(0.02, 0.025, 0.04) * stir;
        const float3 scatter = float3(0.018, 0.026, 0.03) + float3(0.22, 0.2, 0.15) * stir;
        const float3 ext = absorb + scatter;
        const float3 lw = refract(-gLight.sunDirL.xyz, float3(0, 1, 0), 0.75);
        const float mus = max(-lw.y, 0.2);
        const float sunShare = 0.72 * shadowV;
        const float3 down = sunShare * exp(-ext * waterDepth / mus) + (1.0 - sunShare) * exp(-ext * waterDepth * 1.15);
        const float3 kSun = ext * (1.0 + rdy / mus), kAmb = ext * (1.0 + rdy * 1.15);
        const float cosT = dot(rd, lw), g = 0.82;
        const float hg = (1.0 - g * g) / (4.0 * kPI * pow(max(1.0 + g * g - 2.0 * g * cosT, 1e-4), 1.5));
        o.p7 = float4(down * exp(-ext * path), 0.0);
        o.p8 = float4(scatter * sunRgb * shadowV * hg * (1.0 - exp(-kSun * path)) / kSun + scatter * SkyAmbient(float3(0, 1, 0)) * 0.6 * (1.0 - exp(-kAmb * path)) / kAmb, waterDepth);
    }
    if (mobile) {
        o.p0 = float4(grad, m.x, m.w);
        o.p1 = float4(m.y, m.z, height, folds);
        o.p2 = float4(s.roller, s.trail, s.breaking, s.zone);
        o.p3 = float4(s.lip, s.tube, s.lipFoam, s.lipW);
        o.p4 = float4(s.dir, s.bed, s.lift);
        o.p5 = float4(s.sheetA, s.wet, s.gloss, s.sheetAge);
        o.p6 = float4(s.swashMargin, s.curl, s.depth, s.reach);
    }
    return o;
}

// ---- light
float FastAtan2(float y, float x) {
    const float ax = abs(x), ay = abs(y), a = min(ax, ay) / max(max(ax, ay), 1e-8), s2 = a * a;
    float r = ((-0.0464964749 * s2 + 0.15931422) * s2 - 0.327622764) * s2 * a + a;
    if (ay > ax) r = 1.57079637 - r;
    if (x < 0.0) r = 3.14159274 - r;
    return y < 0.0 ? -r : r;
}
float3 SkyLookup(float3 d) {
    if (gShore[7].x > 1.5) { // the texture: wraps around in azimuth, clamped at the poles
        const float azF = FastAtan2(d.z, d.x);
        return gSkyTex.SampleLevel(gWrap, float2(azF / kTau + 1.0, clamp(d.y * 0.5 + 0.5, 0.5 / float(kSkyH), 1.0 - 0.5 / float(kSkyH))), 0.0).rgb;
    }
    const float az = atan2(d.z, d.x);
    if (false)
        return gSkyTex.SampleLevel(gWrap, float2(az / kTau + 1.0, clamp(d.y * 0.5 + 0.5, 0.5 / float(kSkyH), 1.0 - 0.5 / float(kSkyH))), 0.0).rgb;
    const float2 p = float2(az / kTau + 1.0, saturate(d.y * 0.5 + 0.5)) * float2(kSkyW, kSkyH) - 0.5;
    const float2 fl = floor(p);
    const float2 f = p - fl;
    const int x0 = ((int)fl.x + (int)kSkyW) % (int)kSkyW, x1 = (x0 + 1) % (int)kSkyW;
    const int y0 = clamp((int)fl.y, 0, (int)kSkyH - 1), y1 = min(y0 + 1, (int)kSkyH - 1);
    return lerp(lerp(gSky[y0 * kSkyW + x0].rgb, gSky[y0 * kSkyW + x1].rgb, f.x), lerp(gSky[y1 * kSkyW + x0].rgb, gSky[y1 * kSkyW + x1].rgb, f.x), f.y);
}
// The sky without its table: the lighting's own reflection gradient (zenith, horizon, ground), and the sun's glow around it. The flight
// tier's reflections: no lookups.
float3 SkyGradient(float3 d) {
    const float3 sky = lerp(gLight.skyGrad[1].rgb, gLight.skyGrad[0].rgb, sqrt(saturate(d.y)));
    const float sunGlow = pow(saturate(dot(d, gLight.sunDirL.xyz)), 12.0);
    return lerp(gLight.skyGrad[2].rgb, sky, smoothstep(-0.05, 0.05, d.y)) + gLight.sunColor.rgb * sunGlow * 0.35;
}
float3 WorldAt(float2 uv, float depth) {
    const float4 w = mul(invViewProj, float4(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, depth, 1.0));
    return w.xyz / w.w;
}
float Hash2(float2 p) { return frac(sin(dot(p, float2(127.1, 311.7))) * 43758.5453); }
float SunShadow(float3 world) {
    if (gLight.shadowParams.w < 0.5) return 1.0;
    const float3 s = mul(gLight.shadowVP, float4(world, 1.0)).xyz;
    const float2 uv = float2(s.x * 0.5 + 0.5, 0.5 - s.y * 0.5);
    const float edge = saturate(min(min(uv.x, 1.0 - uv.x), min(uv.y, 1.0 - uv.y)) * 20.0);
    if (edge <= 0.0 || s.z >= 1.0) return 1.0;
    return lerp(1.0, gShadow.SampleCmpLevelZero(gCmp, uv, s.z), edge);
}
// The same, soft: 3 x 3 taps 1.5 texels apart, so the shadow of a tree line on a river is no row of shadow-map texels (stair steps
// in the glint). Rivers only: nine taps a pixel on a few percent of the screen.
float SunShadowSoft(float3 world) {
    if (gLight.shadowParams.w < 0.5) return 1.0;
    const float3 s = mul(gLight.shadowVP, float4(world, 1.0)).xyz;
    const float2 uv = float2(s.x * 0.5 + 0.5, 0.5 - s.y * 0.5);
    const float edge = saturate(min(min(uv.x, 1.0 - uv.x), min(uv.y, 1.0 - uv.y)) * 20.0);
    if (edge <= 0.0 || s.z >= 1.0) return 1.0;
    float2 dim;
    gShadow.GetDimensions(dim.x, dim.y);
    const float2 step = 1.5 / dim;
    float lit = 0.0;
    [unroll] for (int y = -1; y <= 1; ++y)
        [unroll] for (int x = -1; x <= 1; ++x) lit += gShadow.SampleCmpLevelZero(gCmp, uv + float2(x, y) * step, s.z);
    return lerp(1.0, lit / 9.0, edge);
}
float CloudSun(float3 world) { // lighting.hlsli's, on our own constants
    if (gLight.cloudMap.w < 0.5) return 1.0;
    const float2 uv = (world.xz - gLight.cloudMap.xy) / (2.0 * gLight.cloudMap.z) + 0.5;
    float2 dim;
    gCloudShadow.GetDimensions(dim.x, dim.y);
    const float2 p = uv * dim - 0.5, b = floor(p), f = p - b;
    const int3 c = int3(clamp(int2(b), int2(0, 0), int2(dim) - 2), 0);
    const float t = lerp(lerp(gCloudShadow.Load(c).r, gCloudShadow.Load(c + int3(1, 0, 0)).r, f.x), lerp(gCloudShadow.Load(c + int3(0, 1, 0)).r, gCloudShadow.Load(c + int3(1, 1, 0)).r, f.x), f.y);
    return lerp(1.0, t, saturate(min(min(uv.x, 1.0 - uv.x), min(uv.y, 1.0 - uv.y)) * 20.0));
}
float3 SkyAmbient(float3 n) {
    const float3 mean = gLight.skySh[0].rgb;
    return max(mean + (gLight.skySh[1].rgb * n.x + gLight.skySh[2].rgb * n.y + gLight.skySh[3].rgb * n.z) * (2.0 / 3.0), mean * 0.1);
}
float Schlick(float c) { return 0.02 + 0.98 * pow(1.0 - saturate(c), 5.0); } // water, n = 1.333

// What the water reflects in a direction: the sky the screen holds there (clouds, haze, the horizon as composed) where it is on
// screen, the sky table where it is not; scene objects by a short march through the depth buffer.
float3 Reflected(float3 world, float3 r, float dist, float rough, bool march, bool skyOnly = false) {
    float3 col = SkyLookup(r);
    if (skyOnly) { // the flight tier: the sky table, spread up the sky by roughness as below; seen from above, reflections point off screen anyway
        if (rough > 0.02) col = col * 0.55 + SkyLookup(normalize(r + float3(0, min(rough * 1.5, 0.8), 0))) * 0.45;
        return col;
    }
    if (rough > 0.02) { // a spread of facets sends some of the reflection higher up the sky, where it is deeper blue
        const float sp = min(rough * 0.9, 0.5);
        col = col * 0.4 + SkyLookup(normalize(r + float3(0, sp, 0))) * 0.35 + SkyLookup(normalize(r + float3(0, sp * 2.2, 0))) * 0.25;
    }
    const float4 c = mul(wc.viewProj, float4(r, 0.0)); // the direction at infinity
    if (c.w > 1e-4) {
        const float2 suv = float2(c.x / c.w * 0.5 + 0.5, 0.5 - c.y / c.w * 0.5);
        const float2 fade = saturate(min(suv, 1.0 - suv) * 10.0);
        if (fade.x * fade.y > 0.0 && gDepth.SampleLevel(gClamp, suv, 0) <= 0.0 && dot(r, gLight.sunDirL.xyz) < 0.9992)
            col = lerp(col, min(gRefract.SampleLevel(gClamp, suv, 0).rgb, 60.0), fade.x * fade.y * saturate(1.0 - rough * 2.5));
    }
    if (march && dist < 300.0) {
        const float reach = clamp(dist * 0.8, 8.0, 160.0);
        [loop] for (int k = 1; k <= 10; ++k) {
            const float tt = (float(k) - Hash2(world.xz * 37.0 + misc.y) * 0.9) / 10.0;
            const float3 q = world + r * tt * tt * reach;
            const float4 cq = mul(wc.viewProj, float4(q, 1.0));
            if (cq.w <= 0.0) break;
            const float2 suv = float2(cq.x / cq.w * 0.5 + 0.5, 0.5 - cq.y / cq.w * 0.5);
            if (any(suv < 0.0) || any(suv > 1.0)) break;
            const float d = gDepth.SampleLevel(gClamp, suv, 0);
            if (d <= 0.0) continue;
            const float3 hit = WorldAt(suv, d);
            const float gap = length(q - eye.xyz) - length(hit - eye.xyz);
            if (gap > 0.0 && gap < reach * 0.3 * tt + 0.3 && hit.y > world.y - 0.05) {
                const float2 fade = saturate(min(suv, 1.0 - suv) * 12.0);
                col = lerp(col, gRefract.SampleLevel(gClamp, suv, 0).rgb, fade.x * fade.y * (1.0 - tt * tt));
                break;
            }
        }
    }
    return col;
}

// Beckmann microfacets: the sun's glitter, as wide as the slopes the pixel holds but cannot show.
float Glitter(float3 n, float3 v, float3 l, float var) {
    const float3 h = normalize(v + l);
    const float nh = saturate(dot(n, h)), nv = max(dot(n, v), 1e-3), nl = dot(n, l);
    if (nl <= 0.0) return 0.0;
    const float c2 = nh * nh, t2 = (1.0 - c2) / max(c2, 1e-4);
    const float D = exp(-t2 / var) / (kPI * var * c2 * c2);
    const float G = min(1.0, min(2.0 * nh * nv / max(dot(v, h), 1e-3), 2.0 * nh * nl / max(dot(v, h), 1e-3)));
    return D * G / (4.0 * nv);
}

// Coverage 0..1 into the pattern real foam makes: dense foam with pinholes, patches with holes, then a lace of filaments between
// big holes as it thins. Two scales of holes, each blended toward its edge value once its texels are smaller than the pixel, and a
// large blotch that moves the threshold, so foam keeps its patchiness at any distance. Blotches stretch along `dir`, the travel.
float FoamPattern(float2 p, float coverage, float footprint, float2 dir, float2 drift, out float bump, bool nearest = false) {
    const float2 side = float2(-dir.y, dir.x);
    const float2 qa = p + drift;
    const float2 q = float2(dot(qa, side), dot(qa, dir) * 0.6);
    const float ba = saturate(footprint * float(kFoamSize) / 3.0 * 0.5 - 0.5), bb = saturate(footprint * float(kFoamSize) / 10.3 * 0.5 - 0.5);
    const float bc = saturate(footprint * float(kFoamSize) / 33.0 * 0.5 - 0.5);
    // a scale whose texels are all under the pixel is blended fully to its edge value below: its lookup is skipped, exactly
    const float4 kMean = float4(0.62, 0.3, 0.5, 0.62);
    const float2 ua = qa / 3.0, ub = float2(qa.x * 0.8 - qa.y * 0.6, qa.x * 0.6 + qa.y * 0.8) / 10.3 + 0.37;
    const float2 uc = float2(q.x * 0.6 + q.y * 0.8, -q.x * 0.8 + q.y * 0.6) / 33.0 + 0.71;
    const float4 a = ba < 1.0 ? (nearest ? FoamNearest(ua) : FoamSample(ua)) : kMean;
    const float4 b = bb < 1.0 ? (nearest ? FoamNearest(ub) : FoamSample(ub)) : kMean;
    const float4 c = bc < 1.0 ? (nearest ? FoamNearest(uc) : FoamSample(uc)) : kMean;
    const float big = lerp(c.w, 0.62, bc);
    const float holes = min(min(lerp(a.x, 0.62, ba), lerp(b.w, 0.62, bb) * 0.9 + 0.08), big * 0.8 + 0.2);
    const float blot = lerp(b.z, 0.5, bb) * 0.4 + lerp(c.z, 0.5, bc) * 0.6;
    const float thr = lerp(0.95, 0.08, coverage) + (blot - 0.5) * 0.45;
    const float w = 0.03 + 0.25 * max(ba, bb * 0.5);
    float foam = smoothstep(thr - w, thr + w, holes);
    foam *= 0.55 + 0.45 * smoothstep(thr, thr + 0.18, holes); // thin at its edges, thick inside
    const float bub = lerp(a.y, 0.3, ba);
    bump = holes;
    return saturate(foam * (0.9 + 0.12 * bub));
}

struct PsOut {
    float4 color : SV_Target0; // alpha 0.5: TaaPS keeps less history here
    float2 velocity : SV_Target1;
};
float4 WaterColor(VsOut i);
float4 WaterColorFlight(VsOut i);
// The water's own motion on screen: only the camera's, taken at the surface and not at the bed the depth buffer holds. Never exactly zero,
// so TaaPS trusts it over the depth.
// Early depth: the water writes depth (a breaker's lip folds over its face) and discards dry sand, which would push the test after
// the shader; tested first, a wave hidden behind a nearer one is never shaded. A discarded pixel keeps a depth 3 cm over the sand.
[earlydepthstencil]
PsOut WaterPS(VsOut i) {
    PsOut o;
    [branch] if (FlightTier()) o.color = WaterColorFlight(i); // an if, not ?: (HLSL evaluates both sides of a ?:)
    else o.color = WaterColor(i);
    const float4 c = mul(prevViewProj, float4(i.world, 1.0));
    const float2 uv = i.pos.xy * res.zw;
    const float2 prevUv = float2(c.x / c.w + wc.extra.x, -(c.y / c.w + wc.extra.y)) * 0.5 + 0.5;
    o.velocity = uv - prevUv + float2(1e-5, 0.0);
    return o;
}

float a1Debug(Surf s) { return saturate(log2(max(s.ratio, 1e-3)) * 0.25 + 0.5); }
float4 WaterColor(VsOut i) {
    const bool reliefOff = wc.extra.w > 7.5; // FLY_OCEAN_DEBUG=8: the dynamic relief off, for comparison
    const float gDbg = reliefOff ? 0.0 : wc.extra.w;
    const float2 uv = i.pos.xy * res.zw;
    const float footprint = sqrt(length(ddx(i.rest)) * length(ddy(i.rest))); // the geometric mean: at grazing angles the long axis would throw away what the short one still shows
    const float t = wc.place.w;
    const bool mobile = MobileTier(), lite = FlightTier();
    const float2 rest = (reliefOff || mobile) ? i.rest : WaterRelief(i.rest, normalize(eye.xyz - i.world), i.cell, footprint);
    Surf s;
    if (mobile) {
        // the mobile tier: the surf as the vertices worked it out; per pixel only the swash's front, against the sand's own noise
        s = (Surf)0;
        s.grad = i.p0.xy; s.m = float4(i.p0.z, i.p1.x, i.p1.y, i.p0.w);
        s.d.y = i.p1.z;
        s.roller = i.p2.x; s.trail = i.p2.y; s.breaking = i.p2.z; s.zone = i.p2.w;
        s.lip = i.p3.x; s.tube = i.p3.y; s.lipFoam = i.p3.z; s.lipW = i.p3.w;
        s.dir = i.p4.xy / max(length(i.p4.xy), 1e-4); s.bed = i.p4.z; s.lift = i.p4.w;
        s.wet = i.p5.y; s.gloss = i.p5.z;
        s.swashMargin = i.p6.x; s.curl = i.p6.y; s.depth = i.p6.z; s.reach = i.p6.w;
        if (s.bed > -0.4 && s.bed < gShore[4].y + 0.1) {
            // SurfAt's own sheet, edge and foam for the covering wave, at this pixel's height (the sand's noise as there)
            const float4 grain = FoamSample(rest / 6.1 + 0.13);
            const float noise = (grain.z - 0.5) * 0.06 + (grain.w - 0.5) * 0.015;
            const float margin = s.swashMargin - noise;
            if (margin > 0.0) {
                s.film = i.p5.x * pow(saturate(margin / 0.2), 0.55);
                s.wet = 1.0; s.gloss = 1.0;
                const float edge = saturate(1.0 - margin / 0.045);
                if (i.p5.w > 0.0) s.swashEdge = edge;
                s.swashFoam = abs(i.p5.w) * (0.35 + 0.65 * edge) * saturate(1.0 - s.depth * 3.0);
            } else if (s.reach - noise < 0.0) { s.wet = 0.0; s.gloss = 0.0; } // above the last run-ups: dry
        }
    } else s = SurfAt(rest, t, 1.0, 1.0); // the lip's shading even where the cells cannot fold it: its shadowed underside still reads
    const float3 toEye = eye.xyz - i.world;
    const float dist = length(toEye);
    const float3 v = toEye / dist;
    const float3 l = gLight.sunDirL.xyz;
    const float3 sunRgb = gLight.sunColor.rgb * 3.2;
    const float shadow = SunShadow(i.world) * CloudSun(i.world);

    // ---- the surface: the swell's analytic slope, plus each cascade the pixel can still resolve; what it cannot goes into roughness
    float2 grad = s.grad;
    float4 m = s.m;
    float height = s.d.y, folds = mobile ? i.p1.w : 0.0, caustFade = 1.0;
    float var = 0.0004; // the sun's own width and the finest capillaries
    const float3 deep = FftDepthFade(s.depth);
    const float3 film3 = s.film > 0.003 ? float3(0.0, 0.25, 0.5) : float3(0.0, 0.0, 0.0); // the sheet keeps a little texture of its own
    const float4 H5 = gShore[5];
    var += gShore[6].w * 0.5 * deep.z * deep.z; // waves a coarser cascade grid dropped (mobile tier): always under the pixel, always roughness
    [unroll] for (uint c = 0; c < kCascades; ++c) {
        const float texel = TileMetres(c) / 256.0; // the full grid's texel on every tier: what a pixel resolves is the same sea's, whatever grid draws it
        const float sharp = saturate(2.5 - footprint / (texel * 1.5));
        const float fade = max(c == 0u ? deep.x : (c == 1u ? deep.y : deep.z), c == 0u ? film3.x : (c == 1u ? film3.y : film3.z));
        const float cvar = c == 0u ? H5.x : (c == 1u ? H5.y : H5.z);
        var += cvar * 0.5 * fade * fade * (1.0 - sharp * sharp);
        if (mobile) { // the height, the folding and what the vertices carried are in already: only the slope they could not carry
            const float w = sharp * fade * (1.0 - VertexCarries(c, i.cell, i.world));
            if (w > 0.02) {
                const float4 f1 = WaveSample(c, rest, 1);
                grad += f1.xy * w;
                m += float4(f1.z, 0.0, 0.0, f1.w) * w;
            }
        } else if (fade > 0.0) {
            const float4 f0 = WaveSample(c, rest, 0);
            height += f0.y * fade;
            folds = max(folds, f0.w * fade);
            if (sharp > 0.0) {
                const float4 f1 = WaveSample(c, rest, 1);
                const float w = sharp * fade;
                grad += f1.xy * w;
                m += float4(f1.z, 0.0, 0.0, f1.w) * w; // the cross term is small: left out
            }
        }
    }
    // the slope over the displaced surface, not over the rest coordinates: the inverse transpose of the sideways map
    const float2x2 J = float2x2(1.0 + m.x, m.y, m.z, 1.0 + m.w);
    const float det = J._11 * J._22 - J._12 * J._21;
    const float2 adjGrad = mul(float2x2(J._22, -J._21, -J._12, J._11), grad);
    // under a breaker's lip the sheet has folded over (det < 0): its normal there faces down, the underside of the overhang
    const float detUse = s.lipW > 0.02 ? (det < 0.0 ? min(det, -0.04) : max(det, 0.04)) : max(det, 0.15);
    const float2 slope = adjGrad / detUse;
    if (gDbg > 6.5) return float4(s.bed > 0.8 ? 1.0 : 0.0, s.bed > 0.52 ? 1.0 : 0.0, gShore[4].y > 0.7 ? 1.0 : 0.0, 1.0); // FLY_OCEAN_DEBUG=7: bed bands, run-up sanity
    if (gDbg > 4.5 && gDbg < 5.5) return float4(s.curl > 0.05 ? 1.0 : 0.0, s.lipW > 0.05 ? 1.0 : 0.0, s.tube > 0.05 ? 1.0 : (frac(rest.y / 10.0) < 0.04 ? 0.5 : 0.0), 1.0); // FLY_OCEAN_DEBUG=5: curl zone, lip, tube wall; 10 m z lines
    if (gDbg > 3.5) return float4(frac(rest.x / 10.0), frac(rest.y / 10.0), s.bed > 0.0 ? 1.0 : 0.0, 1.0); // FLY_OCEAN_DEBUG=4: a 10 m grid, land in blue
    if (gDbg > 2.5) return float4(s.film > 0.003 ? 1.0 : 0.0, s.wet > 0.02 ? 1.0 : 0.0, (s.bed > -0.6 && s.bed < 2.6) ? 1.0 : 0.0, 1.0); // FLY_OCEAN_DEBUG=3: sheet, wet, bed
    if (gDbg > 0.5 && gDbg < 1.5) return float4(saturate(slope.x * 2.0 + 0.5), saturate(slope.y * 2.0 + 0.5), s.breaking, 1.0);
    float3 n = normalize(float3(-adjGrad.x, detUse, -adjGrad.y));
    if (dot(n, v) < 0.0) n = normalize(n - v * (dot(n, v) * 1.02)); // a fold seen edge-on: never show the far side
    const float rough = saturate(sqrt(var) * 2.0);
    // where a breaker stands steep or hangs over, a view ray goes through the wave's body and out into the air, not down to the bed
    const float throughWave = saturate(s.lipW * 3.0) * saturate((0.8 - n.y) * 2.0);

    // ---- what the depth buffer holds under this pixel
    const float rawDepth = gDepth.SampleLevel(gClamp, uv, 0);
    const float3 below = rawDepth > 0.0 ? WorldAt(uv, rawDepth) : i.world - v * 400.0;
    const float column = max(max(i.world.y - below.y, 0.0), throughWave * 1.5); // a lip has sky or the face behind it, never a shallow edge
    const float3 sceneHere = gRefract.SampleLevel(gClamp, uv, 0).rgb;

    // ---- foam coverage from every source
    const float2 drift = s.dir * t * 0.35;
    float bump = 0.0;
    const float surfFoam = saturate(s.roller * 1.25 + s.trail * 0.75 + s.zone * s.breaking * 0.12 * (1.0 - s.curl) + s.lipFoam * 1.4);
    const float swashPatch = s.swashFoam > 0.003 ? smoothstep(0.32, 0.72, FoamSample(rest / 4.3 + 0.21).z) : 0.0; // backwash foam gathers in streaks and patches
    const float coverage = saturate(max(max(surfFoam, folds * 0.8), s.swashFoam * (0.3 + 0.7 * swashPatch)));
    float foam = coverage > 0.003 ? FoamPattern(rest, coverage, footprint, s.dir, drift * (1.0 - s.swashFoam), bump) : 0.0;
    if (s.swashEdge > 0.0) foam = max(foam, s.swashEdge * smoothstep(0.35, 0.75, FoamSample(rest / 1.7).x * 0.6 + s.swashEdge * 0.55) * 0.9); // the uprush's bubbly front

    // ---- sand above the water: either under the swash sheet or wet from it; else this pixel is not water at all
    const bool onSand = height < s.bed + 0.004;
    if (onSand && s.film <= 0.003) {
        if (s.wet < 0.02) discard;
        // Wet sand: darker (water fills the gaps between grains, less light scatters back out), and while a film is left, a mirror
        const float3 nSand = normalize(float3(-s.grad.x, 1.0, -s.grad.y) + float3(Hash2(floor(rest * 90.0)) - 0.5, 0.0, Hash2(floor(rest * 90.0) + 7.0) - 0.5) * 0.05 * (1.0 - s.gloss));
        const float lum = dot(sceneHere, float3(0.3, 0.55, 0.15));
        float3 col = lerp(lum.xxx, sceneHere, 1.0 + 0.3 * s.wet) * lerp(1.0, 0.52, s.wet);
        const float nv = saturate(dot(nSand, v));
        const float3 r = reflect(-v, nSand);
        const float Fs = Schlick(nv) * s.gloss;
        col = lerp(col, Reflected(i.world, r, dist, 0.15, !mobile, lite), Fs) + sunRgb * shadow * Glitter(nSand, v, l, lerp(0.05, 0.004, s.gloss)) * Fs * 0.6;
        col = col * i.hazeKeep + i.hazeLight;
        return float4(min(col, 2000.0), 0.5);
    }

    // ---- the swash sheet and the open water
    const float thin = onSand ? s.film : column;
    if (thin > 0.0 && !onSand) n = normalize(lerp(float3(0, 1, 0), n, saturate(column / 0.04 + 0.2))); // the very edge lies flat
    const float nv = saturate(dot(n, v));
    const float F = Schlick(nv) * lerp(1.0, 0.72, saturate(rough * 3.0) * saturate(1.0 - v.y * 8.0));
    const float3 r0 = reflect(-v, n);
    const float3 r = float3(r0.x, abs(r0.y), r0.z);
    float3 reflection = Reflected(i.world, r, dist, rough, F > 0.03 && !mobile, lite) * (1.0 - 0.4 * s.tube); // mobile: the sky, no march through the depth; // inside the tube the wall mirrors the lip's underside, not the sky

    // refraction: the bend into the water, and what it lands on
    const float3 rd = refract(-v, n, 0.75);
    float2 refrUv = uv + n.xz * 0.05 * saturate(min(column, thin + 0.001) * 0.6) / max(dist * 0.06, 1.0);
    const float refrRaw = gDepth.SampleLevel(gClamp, refrUv, 0);
    float3 bed = refrRaw > 0.0 ? WorldAt(refrUv, refrRaw) : i.world - v * 400.0;
    if (bed.y > i.world.y + 0.02) { refrUv = uv; bed = below; } // the bend found something above the water: keep the straight view
    float3 scene = gRefract.SampleLevel(gClamp, refrUv, 0).rgb;
    float path = min(length(bed - i.world), 400.0);
    // A breaker's face and lip: the ray goes through the wave's body and out of its back into the air, not down to the bed. The body is
    // thick low on the face and thin at the top, thinnest through the lip; behind it, the sky.
    if (throughWave > 0.0) {
        const float body = min(lerp(3.5, 0.45, saturate(s.lift)), lerp(3.5, 0.18, s.lip));
        path = lerp(path, body, throughWave);
        scene = lerp(scene, SkyLookup(normalize(float3(rd.x, abs(rd.y) + 0.1, rd.z))), throughWave);
    }
    const float bedDepth = max(i.world.y - bed.y, 0.0);

    // the water: clear coastal sea, and in the surf, sand stirred up into it (it scatters, beige)
    const float stir = saturate(s.zone * (0.25 + 0.6 * s.breaking) * saturate(s.depth / 0.9) + s.roller * 0.6) * (1.0 - 0.85 * saturate(throughWave * 1.5 + s.lip)); // bores stir sand up; the thin swash and a pitching lip (water off the wave's back) run clear
    const float3 absorb = float3(0.42, 0.064, 0.038) + float3(0.02, 0.025, 0.04) * stir;
    const float3 scatter = float3(0.018, 0.026, 0.03) + float3(0.22, 0.2, 0.15) * stir;
    const float3 ext = absorb + scatter;
    // the sun's way down into the water, and its fall-off with depth
    const float3 lw = refract(-l, float3(0, 1, 0), 0.75);
    const float mus = max(-lw.y, 0.2);
    // the bed: the sun reaches it through the depth above (focused into caustics by the waves), the sky dimmer still
    float caustic = 1.0;
    const float causticSharp = saturate(1.5 - footprint * 40.0) * saturate(1.0 - stir * 0.7); // stirred sand and distance blur them
    if (rawDepth > 0.0 && bedDepth > 0.02 && bedDepth < 12.0 && causticSharp > 0.0 && !lite) { // flight tier: none (a flicker the eye loses at speed)
        const float2 top = bed.xz - lw.xz / mus * bedDepth;
        float hxx = 0.0, hzz = 0.0, hxz = 0.0;
        [unroll] for (uint c = 1; c < kCascades; ++c) {
            const float4 hh = WaveSample(c, top, 2);
            const float fade = c == 1u ? deep.y : deep.z;
            hxx += hh.x * fade; hzz += hh.y * fade; hxz += hh.z * fade;
        }
        const float a = 0.25 * bedDepth / mus * 1.7;
        const float jac = (1.0 - a * hxx) * (1.0 - a * hzz) - a * a * hxz * hxz;
        caustic = lerp(1.0, clamp(1.0 / max(abs(jac), 0.2), 0.35, 2.6), causticSharp * saturate(bedDepth * 2.5) * 0.75); // strongest from half a metre down
    }
    if (gDbg > 5.5) return float4(caustic * 0.5, caustic * 0.5, bedDepth, 1.0); // FLY_OCEAN_DEBUG=6: caustics, bed depth
    const float sunShare = 0.72 * shadow;
    const float3 down = lerp(sunShare * caustic * exp(-ext * bedDepth / mus) + (1.0 - sunShare) * exp(-ext * bedDepth * 1.15), 1.0, throughWave); // through a breaker: the sky behind it, unlit by any bed
    const float3 view = exp(-ext * path);
    const float3 ambient = SkyAmbient(float3(0, 1, 0));
    // single scattering along the view ray, in closed form for a flat surface: the sun's light falls off with depth as it goes
    const float rdy = max(-rd.y, 0.05);
    const float3 kSun = ext * (1.0 + rdy / mus), kAmb = ext * (1.0 + rdy * 1.15);
    const float cosT = dot(rd, lw), g = 0.82;
    const float hg = (1.0 - g * g) / (4.0 * kPI * pow(max(1.0 + g * g - 2.0 * g * cosT, 1e-4), 1.5));
    const float3 inSun = scatter * sunRgb * shadow * hg * (1.0 - exp(-kSun * path)) / kSun;
    const float3 inAmb = scatter * ambient * 0.6 * (1.0 - exp(-kAmb * path)) / kAmb;
    float3 water = scene * down * view + inSun + inAmb;
    // light through a crest: when the sun is behind a wave, its thin top glows green
    const float back = pow(saturate(dot(-v, l) * 0.5 + 0.5 + 0.15), 4.0);
    const float thinCrest = max(s.lift * s.lift * (0.4 + 0.6 * s.breaking) * saturate(1.0 - nv * 1.3 + 0.35), s.lip * 0.6);
    water += sunRgb * shadow * float3(0.05, 0.32, 0.26) * back * thinCrest * 0.55;
    // the lip and the face under it glow with what the sun puts through the lip, whichever side the sun is on
    water += (sunRgb * shadow * 0.05 + ambient * 0.25) * float3(0.06, 0.3, 0.26) * max(s.lip, s.tube * 0.25) * (0.5 + 0.5 * back);

    // on a lip's underside the mirror looks down, at the wave's own face and trough: the same sea, not the sky or the beach on screen
    reflection = lerp(reflection, water * 0.85, saturate(0.5 - r0.y * 5.0) * saturate(s.lipW * 4.0) * (n.y < 0.15 ? 1.0 : 0.0));
    float3 color = lerp(water, reflection, F);
    color += sunRgb * shadow * Glitter(n, v, l, var + 0.0006) * F * (1.0 - foam);

    // ---- foam on top: thick, matte, bright; lit by its own lumps, a little shadow in its hollows
    if (foam > 0.002) { // only where there is foam: its lumps take six lookups
        const float lumpAmp = s.roller * 0.24 + s.trail * 0.05 + 0.015;
        const float lumpFade = mobile ? 0.0 : saturate(1.0 - footprint * 20.0); // lumps smaller than the pixel: their mean, no slope (mobile: none)
        const float l0 = mobile ? 0.5 : FoamLump(rest, drift);
        float2 dl = float2(0.0, 0.0);
        if (lumpFade > 0.0) dl = float2(FoamLump(rest + float2(0.04, 0.0), drift), FoamLump(rest + float2(0.0, 0.04), drift)) - l0;
        const float3 nUp = float3(n.x, abs(n.y), n.z); // foam is a pile of bubbles lit through, whichever way its sheet faces
        const float3 nf = normalize(nUp + float3(-dl.x, 0.0, -dl.y) / 0.04 * lumpAmp * 0.45 * lumpFade);
        const float3 foamLit = float3(0.86, 0.88, 0.87) * (SkyAmbient(nf) * 1.15 + sunRgb * shadow * (saturate(dot(nf, l) * 0.6 + 0.4) * 0.8 + 0.12)) * (0.72 + 0.28 * smoothstep(0.3, 0.75, bump)) * (0.8 + 0.2 * l0);
        color = lerp(color, foamLit, saturate(foam) * (1.0 - smoothstep(600.0, 3000.0, dist) * 0.7));
    }

    // ---- where the water thins to nothing at the edge of the sheet or the shore
    const float edge = saturate(thin / 0.012);
    color = lerp(sceneHere, color, edge * edge * (3.0 - 2.0 * edge));
    if (gDbg > 1.5 && gDbg < 2.5) color = float3(foam, s.breaking, saturate(s.lip + s.tube)); // FLY_OCEAN_DEBUG=2: foam, breaking, a breaker's lip and tube

    color = color * i.hazeKeep + i.hazeLight;
    return float4(min(color, 2000.0), 0.5);
}

// ---- the flight tier (tier 2): the sea from the air on a standalone headset. What a pilot at a hundred feet and 150 mph reads: the
// colour of the water over sand and deep sea, the swell and the wind sea's glitter, white water, the swash line. So: the surf from the
// vertices as on the mobile tier; the depth from the shore grid, not the depth buffer; the sand under the water from one look at the
// picture behind (no bend); one sky lookup; the finest cascade per pixel; foam by its exact level of detail; no caustics, no lumps,
// no screen-space reflections.
float4 WaterColorFlight(VsOut i) {
    const float2 uv = i.pos.xy * res.zw;
    const float2 rest = i.rest;
    // shaded once per 2 x 2 pixels (variable rate shading), the derivatives span two pixels: the detail tests keep the real pixel's size
    // (TAA's jitter fills in what one shade per block misses), or the glint's ripples would fall to roughness and smear
    const float rate = max(abs(ddx(i.pos.x)), 1.0);
    const float footprint = sqrt(length(ddx(rest)) * length(ddy(rest))) / rate;
    const float t = wc.place.w;
    const float3 toEye = eye.xyz - i.world;
    const float dist = length(toEye);
    const float3 v = toEye / dist;
    const float3 l = gLight.sunDirL.xyz;
    const float3 sunRgb = gLight.sunColor.rgb * 3.2;
    const float shadow = SunShadow(i.world);
    const float4 H5 = gShore[5];

    // the surf, as the vertices worked it out
    float2 grad = i.p0.xy;
    float4 m = float4(i.p0.z, i.p1.x, i.p1.y, i.p0.w);
    const float height = i.p1.z, folds = i.p1.w;
    const float roller = i.p2.x, trail = i.p2.y, breaking = i.p2.z, zone = i.p2.w;
    const float2 dir = i.p4.xy / max(length(i.p4.xy), 1e-4);
    const float bedH = i.p4.z, lift = i.p4.w, depthG = max(i.p6.z, 0.0);
    float film = 0.0, wet = i.p5.y, gloss = i.p5.z, swashFoam = 0.0, swashEdge = 0.0;
    if (bedH > -0.4 && bedH < gShore[4].y + 0.1) { // the swash's front per pixel, as on the mobile tier
        const float4 grain = FoamNearest(rest / 6.1 + 0.13);
        const float noise = (grain.z - 0.5) * 0.06 + (grain.w - 0.5) * 0.015;
        const float margin = i.p6.x - noise;
        if (margin > 0.0) {
            film = i.p5.x * pow(saturate(margin / 0.2), 0.55);
            wet = 1.0; gloss = 1.0;
            const float edge = saturate(1.0 - margin / 0.045);
            if (i.p5.w > 0.0) swashEdge = edge;
            swashFoam = abs(i.p5.w) * (0.35 + 0.65 * edge) * saturate(1.0 - depthG * 3.0);
        } else if (i.p6.w - noise < 0.0) { wet = 0.0; gloss = 0.0; }
    }

    // the slope: what the vertices carried, the cascades the pixel still shows; the rest is roughness
    const float3 deep = FftDepthFade(depthG);
    float var = 0.0004 + gShore[6].w * 0.5 * deep.z * deep.z;
    [unroll] for (uint c = 0; c < kCascades; ++c) {
        const float texel = TileMetres(c) / 256.0;
        const float sharp = saturate(2.5 - footprint / (texel * 1.5));
        const float fade = c == 0u ? deep.x : (c == 1u ? deep.y : deep.z);
        const float cvar = c == 0u ? H5.x : (c == 1u ? H5.y : H5.z);
        var += cvar * 0.5 * fade * fade * (1.0 - sharp * sharp);
        const float w = sharp * fade * (1.0 - VertexCarries(c, i.cell, i.world)); // the ripples in the glint: worth their four reads
        if (w > 0.05) {
            const float4 f1 = WaveSample(c, rest, 1);
            grad += f1.xy * w;
            m += float4(f1.z, 0.0, 0.0, f1.w) * w;
        }
    }
    const float2x2 J = float2x2(1.0 + m.x, m.y, m.z, 1.0 + m.w);
    const float det = max(J._11 * J._22 - J._12 * J._21, 0.15);
    const float2 adj = mul(float2x2(J._22, -J._21, -J._12, J._11), grad);
    float3 n = normalize(float3(-adj.x, det, -adj.y));
    if (dot(n, v) < 0.0) n = normalize(n - v * (dot(n, v) * 1.02));
    const float rough = saturate(sqrt(var) * 2.0);

    // foam
    const float2 drift = dir * t * 0.35;
    const float coverage = saturate(max(max(saturate(roller * 1.25 + trail * 0.75 + zone * breaking * 0.12 + i.p3.z * 1.4), folds * 0.8), swashFoam * 0.65));
    float bump = 0.0;
    float foam = coverage > 0.003 ? FoamPattern(rest, coverage, footprint, dir, drift * (1.0 - swashFoam), bump, true) : 0.0;
    if (swashEdge > 0.0) foam = max(foam, swashEdge * 0.75);

    // the picture behind, straight through: the sand under shallow water, and on the beach the sand itself
    const float3 sceneHere = gRefract.SampleLevel(gClamp, uv, 0).rgb;
    const bool onSand = height < bedH + 0.004;
    const float nv = saturate(dot(n, v));
    if (onSand && film <= 0.003) {
        if (wet < 0.02) discard;
        const float lum = dot(sceneHere, float3(0.3, 0.55, 0.15));
        float3 col = lerp(lum.xxx, sceneHere, 1.0 + 0.3 * wet) * lerp(1.0, 0.52, wet);
        const float3 nS = float3(0, 1, 0);
        const float Fs = Schlick(saturate(v.y)) * gloss;
        col = lerp(col, SkyLookup(reflect(-v, nS)), Fs) + sunRgb * shadow * Glitter(nS, v, l, lerp(0.05, 0.004, gloss)) * Fs * 0.6;
        return float4(min(col * i.hazeKeep + i.hazeLight, 2000.0), 0.5);
    }

    const float F = Schlick(nv) * lerp(1.0, 0.72, saturate(rough * 3.0) * saturate(1.0 - v.y * 8.0));
    const float3 r0 = reflect(-v, n);
    const float3 rr = float3(r0.x, abs(r0.y), r0.z);
    const float3 reflection = SkyLookup(normalize(rr + float3(0, min(rough * 0.7, 0.4), 0))); // one tap of the sky table, lifted as a rough surface spreads it

    // the water body as the vertices worked it out (flat surface), over the bed's picture straight behind
    const float waterDepth = onSand ? film : i.p8.w;
    float3 water = sceneHere * i.p7.rgb * (0.28 + 0.72 * shadow) + i.p8.rgb * (0.3 + 0.7 * shadow); // the vertices assumed full sun
    const float back = pow(saturate(dot(-v, l) * 0.5 + 0.65), 4.0);
    water += sunRgb * shadow * float3(0.05, 0.32, 0.26) * back * lift * lift * (0.4 + 0.6 * breaking) * saturate(1.35 - nv * 1.3) * 0.55;

    float3 color = lerp(water, reflection, F);
    color += sunRgb * shadow * Glitter(n, v, l, var + 0.0006) * F * (1.0 - foam);
    if (foam > 0.002) {
        const float3 nUp = float3(n.x, abs(n.y), n.z);
        const float3 foamLit = float3(0.86, 0.88, 0.87) * (SkyAmbient(nUp) * 1.15 + sunRgb * shadow * (saturate(dot(nUp, l) * 0.6 + 0.4) * 0.8 + 0.12)) * (0.72 + 0.28 * smoothstep(0.3, 0.75, bump));
        color = lerp(color, foamLit, saturate(foam) * (1.0 - smoothstep(600.0, 3000.0, dist) * 0.7));
    }
    const float thin = onSand ? film : waterDepth;
    const float edge = saturate(thin / 0.012);
    color = lerp(sceneHere, color, edge * edge * (3.0 - 2.0 * edge));
    return float4(min(color * i.hazeKeep + i.hazeLight, 2000.0), 0.5);
}

// ---- the copy-free flight pipeline (WaterDraw::blend): WaterVSFlight + WaterPSBlend, dual-source blending over the composed picture.
// The flight tier's shading is linear in the picture behind the water: out = picture x keep + light. So the water never reads the
// picture (no copy of it, and none of the depth: the depth comes from the shore grid), and writes one target, no velocity.
typedef min16float hf; typedef min16float2 hf2; typedef min16float3 hf3; typedef min16float4 hf4; // half precision where it is enough
struct VsOutFlight { // seven interpolants: what a pixel reads is a large part of its cost on a phone-class GPU
    float4 pos : SV_Position;
    float4 r0 : TEXCOORD0;   // rest point, cell size, the haze's transmittance (one number: it is near grey over sea air)
    float4 r1 : TEXCOORD1;   // the haze's own light, depth (the shore grid's)
    float4 a : TEXCOORD2;    // slope over the rest coordinates; the sideways map's dDx/dx, dDz/dz
    float4 b : TEXCOORD3;    // height, bed, foam coverage, travel direction x
    float4 c : TEXCOORD4;    // travel direction z, wet, gloss, the covering sheet's thickness scale
    float4 d : TEXCOORD5;    // the sheet's age, its margin, the run-ups' reach, -
    float4 tint : TEXCOORD6; // what the water passes of the picture behind (full sun); w: the light it scatters, its luminance share
    float4 add : TEXCOORD7;  // the light it scatters toward the eye (full sun), crest glow folded in; w unused
};
VsOutFlight WaterVSFlight(VsIn vin) {
    const VsOut o = WaterVS(vin); // the flight tier's vertex: everything below is already in its payload
    VsOutFlight f;
    f.pos = o.pos;
    f.r0 = float4(o.rest, o.cell, dot(o.hazeKeep, float3(0.3, 0.55, 0.15)));
    f.r1 = float4(o.hazeLight, o.p6.z);
    f.a = o.p0;
    const float coverage = saturate(max(saturate(o.p2.x * 1.25 + o.p2.y * 0.75 + o.p2.w * o.p2.z * 0.12 + o.p3.z * 1.4), o.p1.w * 0.8));
    const float2 dir = o.p4.xy / max(length(o.p4.xy), 1e-4);
    f.b = float4(o.p1.z, o.p4.z, coverage, dir.x);
    f.c = float4(dir.y, o.p5.y, o.p5.z, o.p5.x);
    f.d = float4(o.p5.w, o.p6.x, o.p6.w, 0.0);
    // the crest's glow (light through a thin crest with the sun behind it), per vertex with the flat surface's view
    const float3 v = normalize(eye.xyz - o.world), l = gLight.sunDirL.xyz;
    const float back = pow(saturate(dot(-v, l) * 0.5 + 0.65), 4.0);
    const float glow = o.p4.w * o.p4.w * (0.4 + 0.6 * o.p2.z) * saturate(1.35 - saturate(v.y) * 1.3) * back * 0.55;
    f.tint = float4(o.p7.rgb, 0.0);
    f.add = float4(o.p8.rgb + gLight.sunColor.rgb * 3.2 * float3(0.05, 0.32, 0.26) * glow, 0.0);
    return f;
}

struct PsBlendOut {
    float4 light : SV_Target0; // added
    float4 keep : SV_Target1;  // the picture behind is multiplied by this
};
[earlydepthstencil]
PsBlendOut WaterPSBlend(VsOutFlight i) {
    PsBlendOut o;
    const float2 rest = i.r0.xy;
    const float cellHere = i.r0.z;
    const float3 world = WorldAt(i.pos.xy * res.zw, i.pos.z); // from the depth this pixel writes: no interpolant for it
    const float rate = max(abs(ddx(i.pos.x)), 1.0); // see WaterColorFlight
    const float footprint = sqrt(length(ddx(rest)) * length(ddy(rest))) / rate;
    const float t = wc.place.w;
    const float3 toEye = eye.xyz - world;
    const float dist = length(toEye);
    const hf3 v = (hf3)(toEye / dist);
    const hf3 l = (hf3)gLight.sunDirL.xyz;
    const hf3 sunRgb = (hf3)(gLight.sunColor.rgb * 3.2);
    const hf shadow = (hf)SunShadow(world);
    const float4 H5 = gShore[5];
    const float height = i.b.x, bedH = i.b.y, depthG = max(i.r1.w, 0.0);

    // the swash's front per pixel
    hf film = 0.0, wet = (hf)i.c.y, gloss = (hf)i.c.z, swashFoam = 0.0, swashEdge = 0.0;
    if (bedH > -0.4 && bedH < gShore[4].y + 0.1) {
        const float4 grain = FoamNearest(rest / 6.1 + 0.13);
        const float noise = (grain.z - 0.5) * 0.06 + (grain.w - 0.5) * 0.015;
        const float margin = i.d.y - noise;
        if (margin > 0.0) {
            film = i.c.w * pow(saturate(margin / 0.2), 0.55);
            wet = 1.0; gloss = 1.0;
            const float edge = saturate(1.0 - margin / 0.045);
            if (i.d.x > 0.0) swashEdge = edge;
            swashFoam = abs(i.d.x) * (0.35 + 0.65 * edge) * saturate(1.0 - depthG * 3.0);
        } else if (i.d.z - noise < 0.0) { wet = 0.0; gloss = 0.0; }
    }

    // the slope
    hf2 grad = (hf2)i.a.xy;
    hf4 m = hf4(i.a.z, 0.0, 0.0, i.a.w); // the cross terms are small from the air: not carried
    const float3 deep = FftDepthFade(depthG);
    float var = 0.0004 + gShore[6].w * 0.5 * deep.z * deep.z;
    [unroll] for (uint c = 0; c < kCascades; ++c) {
        const float texel = TileMetres(c) / 256.0;
        const float sharp = saturate(2.5 - footprint / (texel * 1.5));
        const float fade = c == 0u ? deep.x : (c == 1u ? deep.y : deep.z);
        const float cvar = c == 0u ? H5.x : (c == 1u ? H5.y : H5.z);
        var += cvar * 0.5 * fade * fade * (1.0 - sharp * sharp);
        const float w = sharp * fade * (1.0 - VertexCarries(c, cellHere, world));
        if (w > 0.05) {
            const float4 f1 = c == 2u ? WaveNearest(c, rest, 1) : WaveSample(c, rest, 1); // the finest, centimetre texels: one read; the long cascades filtered (their texels are tens of centimetres)
            grad += (hf2)(f1.xy * w);
            m += (hf4)(float4(f1.z, 0.0, 0.0, f1.w) * w);
        }
    }
    const hf j11 = 1.0 + m.x, j22 = 1.0 + m.w;
    const hf det = max(j11 * j22 - m.y * m.z, (hf)0.15);
    const hf2 adj = hf2(j22 * grad.x - m.z * grad.y, -m.y * grad.x + j11 * grad.y);
    hf3 n = normalize(hf3(-adj.x, det, -adj.y));
    if (dot(n, v) < 0.0) n = normalize(n - v * (dot(n, v) * (hf)1.02));
    const hf rough = (hf)saturate(sqrt(var) * 2.0);

    // foam
    const float2 dir = float2(i.b.w, i.c.x);
    const float coverage = max(i.b.z, (float)swashFoam * 0.65);
    float bump = 0.0;
    hf foam = (hf) coverage > 0.03 ? FoamPattern(rest, coverage, footprint, dir, dir * t * 0.35 * (1.0 - swashFoam), bump, true) : 0.0; // under 3% the pattern is specks the air hides
    if (swashEdge > 0.0) foam = max(foam, swashEdge * (hf)0.75);

    const bool onSand = height < bedH + 0.004;
    hf3 keep, light;
    if (onSand && film <= 0.003) {
        if (wet < 0.02) { o.light = float4(0, 0, 0, 0); o.keep = float4(1, 1, 1, 1); return o; } // a no-op blend, not discard (see WaterPSQuest)
        // wet sand: darker, and a mirror while the film lasts
        const float3 nS = float3(0, 1, 0);
        const hf Fs = (hf)Schlick(saturate(v.y)) * gloss;
        keep = lerp((hf)1.0, (hf)0.52, wet) * ((hf)1.0 - Fs) * hf3(1, 1, 1);
        light = (hf3)SkyLookup(reflect(-(float3)v, nS)) * Fs + sunRgb * shadow * (hf)Glitter(nS, (float3)v, (float3)l, lerp(0.05, 0.004, (float)gloss)) * Fs * (hf)0.6;
    } else {
        const hf nv = saturate(dot(n, v));
        const hf one = 1.0, x5 = one - nv, x2 = x5 * x5;
        const hf F = ((hf)0.02 + (hf)0.98 * x2 * x2 * x5) * lerp(one, (hf)0.72, saturate(rough * (hf)3.0) * saturate(one - v.y * (hf)8.0)); // Schlick
        const hf3 r0 = reflect(-v, n);
        const hf3 reflection = (hf3)SkyLookup(normalize(float3(r0.x, abs(r0.y), r0.z) + float3(0, min((float)rough * 0.7, 0.4), 0)));
        const hf3 tint = (hf3)i.tint.rgb * ((hf)0.28 + (hf)0.72 * shadow);
        const hf3 add = (hf3)i.add.rgb * ((hf)0.3 + (hf)0.7 * shadow);
        const hf3 glint = sunRgb * shadow * (hf)Glitter((float3)n, (float3)v, (float3)l, var + 0.0006) * F * (one - foam);
        hf3 foamLit = 0;
        if (foam > 0.002) {
            const hf3 nUp = hf3(n.x, abs(n.y), n.z);
            foamLit = hf3(0.86, 0.88, 0.87) * ((hf3)SkyAmbient((float3)nUp) * (hf)1.15 + sunRgb * shadow * (saturate(dot(nUp, l) * (hf)0.6 + (hf)0.4) * (hf)0.8 + (hf)0.12)) * ((hf)0.72 + (hf)0.28 * (hf)smoothstep(0.3, 0.75, bump));
        }
        const hf fa = saturate(foam) * (one - (hf)smoothstep(600.0, 3000.0, dist) * (hf)0.7);
        const float thin = onSand ? (float)film : max(depthG + height, 0.0);
        const hf ed = (hf)saturate(thin / 0.012), edge = ed * ed * ((hf)3.0 - (hf)2.0 * ed);
        // out = picture x keep + light, from: water = picture x tint + add; lerp(water, reflection, F) + glint; foam over; the thin edge
        keep = (1.0 - edge) + edge * (1.0 - fa) * (1.0 - F) * tint;
        light = edge * ((1.0 - fa) * ((1.0 - F) * add + F * reflection + glint) + fa * foamLit);
    }
    o.light = float4(min((float3)light * i.r0.w + i.r1.rgb, 2000.0), 0.5); // alpha 0.5 as the usual water writes (TaaPS)
    o.keep = float4((float3)keep * i.r0.w, 0.0);
    return o;
}

// ---- the Quest look (tier 3, WaterDraw::blend 2): the flight tier pared to what a standalone headset can afford, about 2 ms.
// Kept: the swell and surf shapes (vertices), the glint's ripples (one fetch), the sky's reflection (one fetch), the water's colour
// over sand and deep sea (vertices), foam (one fetch), the swash line (vertices). Pared: the glint is a cheaper lobe, the swash's
// edge has no per-pixel noise, no sun-shadow lookup (open sea has nothing to cast one), six interpolants.
struct VsOutQuest {
    float4 pos : SV_Position;
    float4 r0 : TEXCOORD0;  // rest point, cell size, the haze's transmittance
    float4 r1 : TEXCOORD1;  // the haze's light, wet sand (0..1, with its gloss folded in: 2 = glossy)
    float4 a : TEXCOORD2;   // slope over the rest coordinates; the sideways map's dDx/dx, dDz/dz
    float4 b : TEXCOORD3;   // height, bed, foam coverage, the swash sheet's thickness
    float4 tint : TEXCOORD4; // what the water passes of the picture behind; w: the edge's water depth
    float4 add : TEXCOORD5;  // the light it scatters toward the eye; w: drift direction angle
};
VsOutQuest WaterVSQuest(VsIn vin) {
    const VsOutFlight f = WaterVSFlight(vin);
    VsOutQuest q;
    q.pos = f.pos;
    q.r0 = f.r0;
    // the swash from the vertices alone: the sheet where the newest front stands over the sand, wet sand behind it
    const float margin = f.d.y;
    const float sheet = margin > 0.0 ? f.c.w * pow(saturate(margin / 0.2), 0.55) : 0.0;
    const float wet = margin > 0.0 ? 1.0 : (f.d.z < 0.0 ? 0.0 : f.c.y);
    const float gloss = margin > 0.0 ? 1.0 : (f.d.z < 0.0 ? 0.0 : f.c.z);
    q.r1 = float4(f.r1.rgb, wet + gloss);
    q.a = f.a;
    q.b = float4(f.b.x, f.b.y, max(f.b.z, margin > 0.0 ? abs(f.d.x) * 0.5 : 0.0), sheet);
    q.tint = float4(f.tint.rgb, max(f.r1.w + f.b.x, 0.0));
    q.add = float4(f.add.rgb, atan2(f.c.x, f.b.w));
    return q;
}

[earlydepthstencil]
PsBlendOut WaterPSQuest(VsOutQuest i) {
    PsBlendOut o;
    const float2 rest = i.r0.xy;
    const float3 world = WorldAt(i.pos.xy * res.zw, i.pos.z);
    const float rate = max(abs(ddx(i.pos.x)), 1.0);
    const float footprint = sqrt(length(ddx(rest)) * length(ddy(rest))) / rate;
    const float3 toEye = eye.xyz - world;
    const float dist = length(toEye);
    const hf3 v = (hf3)(toEye / dist);
    const hf3 l = (hf3)gLight.sunDirL.xyz;
    const hf3 sunRgb = (hf3)(gLight.sunColor.rgb * 3.2);
    const float height = i.b.x, bedH = i.b.y, film = i.b.w;
    const float wetGloss = i.r1.w, wet = saturate(wetGloss), gloss = saturate(wetGloss - 1.0);
    const bool onSand = height < bedH + 0.004;
    hf3 keep, light;
    if (onSand && film <= 0.003) {
        // dry sand: a no-op blend (picture x 1 + 0) instead of discard: discard turns off depth compression and the early-depth fast
        // paths for the whole draw on phone-class and integrated GPUs
        if (wet < 0.02) { o.light = float4(0, 0, 0, 0); o.keep = float4(1, 1, 1, 1); return o; }
        const hf Fs = (hf)(Schlick(saturate((float)v.y)) * gloss);
        keep = (hf3)(lerp(1.0, 0.52, wet)) * ((hf)1.0 - Fs);
        light = (hf3)gLight.skyGrad[0].rgb * Fs;
    } else {
        // the slope: the vertices' swell and surf, the finest cascade's ripples in one fetch; the rest is roughness
        hf2 grad = (hf2)i.a.xy;
        const float depthG = max(i.tint.w - height, 0.0);
        const float4 H5 = gShore[5];
        // the middle cascade (the glint's pattern, metres across) where the vertices did not carry it, and the finest (its sparkle)
        const float sharp1 = saturate(2.5 - footprint / (TileMetres(1u) / 256.0 * 1.5)), fade1 = saturate(depthG / 0.9);
        const float w1 = sharp1 * fade1 * (1.0 - saturate(2.0 - i.r0.z / (TileMetres(1u) / float(WaveN(1u)))) * sharp1);
        if (w1 > 0.05) grad += (hf2)(WaveTex(1u, rest, 1).xy * w1);
        const float fine = saturate(depthG / 0.22) * saturate(2.5 - footprint / (TileMetres(2u) / 256.0 * 1.5));
        if (fine > 0.05) grad += (hf2)(WaveTex(2u, rest, 1).xy * fine);
        const float sharp0 = saturate(2.5 - footprint / (TileMetres(0u) / 256.0 * 1.5));
        const float var = 0.0004 + gShore[6].w * 0.5 + H5.x * 0.5 * (1.0 - sharp0 * sharp0) + H5.y * 0.5 * fade1 * fade1 * (1.0 - sharp1 * sharp1)
                        + H5.z * 0.5 * (1.0 - fine * fine);
        const hf j11 = (hf)1.0 + (hf)i.a.z, j22 = (hf)1.0 + (hf)i.a.w;
        hf3 n = normalize(hf3(-j22 * grad.x, max(j11 * j22, (hf)0.15), -j11 * grad.y));
        if (dot(n, v) < 0.0) n = normalize(n - v * (dot(n, v) * (hf)1.02));
        const hf rough = (hf)saturate(sqrt(var) * 2.0);
        const hf nv = saturate(dot(n, v)), one = 1.0, x5 = one - nv, x2 = x5 * x5;
        const hf F = ((hf)0.02 + (hf)0.98 * x2 * x2 * x5) * lerp(one, (hf)0.72, saturate(rough * (hf)3.0) * saturate(one - v.y * (hf)8.0));
        const hf3 r0 = reflect(-v, n);
        const hf3 reflection = (hf3)SkyLookup(normalize(float3(r0.x, abs(r0.y), r0.z) + float3(0, min((float)rough * 0.7, 0.4), 0)));
        // the glint: a normalised Blinn-Phong lobe as wide as the slopes the pixel holds (one pow, where Beckmann takes an exp)
        const hf3 h = normalize(v + l);
        const float nh = saturate(dot((float3)n, (float3)h));
        const float e = 2.0 / var - 2.0; // the Blinn-Phong lobe that matches Beckmann's width (Walter et al. 2007), its normalisation
        const float spec = (e + 2.0) / (2.0 * kPI) * pow(nh, e) / (4.0 * max((float)nv, 0.1)) * step(0.0, dot((float3)n, (float3)l));
        const hf3 glint = sunRgb * (hf)min(spec, 2000.0) * F;
        // foam: the coverage the vertices worked out, broken by the foam tile's 10 m scale (one fetch); lit flat
        const float coverage = i.b.z;
        hf foam = 0.0;
        if (coverage > 0.03) {
            const float ang = i.add.w;
            const float2 drift = float2(cos(ang), sin(ang)) * wc.place.w * 0.35;
            const float4 fs = gFoamTex.SampleLevel(gWrap, (rest + drift) / 10.3 + 0.37, saturate(footprint * 512.0 / 10.3 * 0.5) * 3.0);
            const float thr = lerp(0.95, 0.08, coverage) + (fs.z - 0.5) * 0.45;
            const float holes = fs.w * 0.9 + 0.08;
            foam = (hf)max(smoothstep(thr - 0.22, thr + 0.22, holes) * (0.55 + 0.45 * smoothstep(thr, thr + 0.3, holes)) * saturate(coverage * 1.6),
                           smoothstep(0.55, 0.95, coverage) * 0.9); // soft: thin at its edges, dense inside; solid where the bore is
        }
        const hf3 foamLit = hf3(0.86, 0.88, 0.87) * ((hf3)SkyAmbient(float3(0, 1, 0)) * (hf)1.15 + sunRgb * (saturate(l.y * (hf)0.6 + (hf)0.4) * (hf)0.8 + (hf)0.12));
        const hf fa = foam * (one - (hf)smoothstep(600.0, 3000.0, dist) * (hf)0.7);
        const float thin = onSand ? film : i.tint.w;
        const hf ed = (hf)saturate(thin / 0.012), edge = ed * ed * ((hf)3.0 - (hf)2.0 * ed);
        keep = (one - edge) + edge * (one - fa) * (one - F) * (hf3)i.tint.rgb;
        light = edge * ((one - fa) * ((one - F) * (hf3)i.add.rgb + F * reflection + glint * (one - foam)) + fa * foamLit);
    }
    o.light = float4(min((float3)light * i.r0.w + i.r1.rgb, 2000.0), 0.5);
    o.keep = float4((float3)keep * i.r0.w, 0.0);
    return o;
}

// ---- rivers and lakes (WaterDraw::rivers): flat water at its own level, from the land cover (fly/flight_water.h). Drawn with the
// copy-free blend into the lit scene before the composite: out = scene x keep + light. The tile's vertices are relative to its corner, which rides in place.xyz (render
// space); place.w is the time, the grid's corner the render origin's world position (the ripples are world-locked). Vertex colour
// alpha: how much of the place is water (the banks fade), TEXCOORD1: the current (m/s, along the river).
struct VsOutRiver {
    float4 pos : SV_Position;
    float4 world : TEXCOORD0;     // render space; w: water (the banks fade)
    float4 flow : TEXCOORD1;      // the current, m/s; zw unused
    float4 bank : TEXCOORD2;      // metres to the nearest bank, the way to it (x, z); metres across to the far bank (flight_water.h)
};
VsOutRiver WaterVSRiver(VsIn i) {
    VsOutRiver o;
    const float3 p = i.pos + wc.place.xyz;
    // drawn a little toward the eye (0.3 % of the distance, at most 15 m): the distant terrain's coarser mesh, on its own grid, would
    // poke through the sheet near the banks in triangles; the banks fade to nothing, so the land the water now covers shows anyway
    const float3 toEye = eye.xyz - p;
    const float d = length(toEye);
    o.pos = mul(wc.viewProj, float4(p + toEye * (min(d * 0.003, 15.0) / max(d, 1.0)), 1.0));
    o.world = float4(p, i.color.a);
    o.flow = float4(i.flow, 0.0, 0.0);
    o.bank = float4(i.color.r, i.color.g, i.color.b, i.normal.x);
    return o; // no haze here: drawn into the lit scene before the composite, which hazes it as it hazes the land
}
// Smooth value noise, 0..1: the wind's patches on a river (cat's paws: rougher, duller water beside calm, glassy streaks).
float RiverNoise(float2 p) {
    const float2 c = floor(p), f = p - c, u = f * f * (3.0 - 2.0 * f);
    return lerp(lerp(Hash2(c), Hash2(c + float2(1, 0)), u.x), lerp(Hash2(c + float2(0, 1)), Hash2(c + float2(1, 1)), u.x), u.y);
}
[earlydepthstencil]
PsBlendOut WaterPSRiver(VsOutRiver i) {
    PsBlendOut o;
    if (wc.extra.w > 8.5 && wc.extra.w < 9.5) { o.light = float4(i.world.w, 0.0, 1.0 - i.world.w, 0.0); o.keep = float4(0, 0, 0, 1); return o; } // FLY_OCEAN_DEBUG=9: river coverage (red water, blue bank)
    const float3 world = i.world.xyz;
    const float2 look = world.xz + wc.grid.xy; // the world's place: the ripples do not move with the render origin
    const float t = wc.place.w;
    const float3 toEye = eye.xyz - world;
    const float dist = length(toEye);
    const float3 v = toEye / dist;
    const float3 l = gLight.sunDirL.xyz;
    const float3 sunRgb = gLight.sunColor.rgb * 3.2;
    const float footprint = sqrt(length(ddx(look)) * length(ddy(look)));
    // the wind: patches a few hundred metres across where it ruffles the water more (duller, the glint spread wider) beside calm,
    // glassier stretches, drifting slowly downwind
    const float gust = RiverNoise(look / 420.0 + t * float2(0.0021, 0.0013)) * 0.65 + RiverNoise(look / 130.0 - t * float2(0.003, 0.004)) * 0.35;
    const float wind = lerp(0.55, 1.45, gust);
    // the current carries the ripples downstream: two phases half a cycle apart, each restarting while the other carries the look
    // (a flow map, Vlachos 2010), so they never stretch out
    const float2 flow = i.flow.xy;
    const float cycle = 4.0, ph0 = frac(t / cycle), ph1 = frac(t / cycle + 0.5), w0 = 1.0 - abs(ph0 * 2.0 - 1.0);
    float2 grad = 0;
    float var = 0.0025 * wind; // a river's own chop (wind against the current): the sun's glint spreads and dims
    const float4 H5 = gShore[5];
    [unroll] for (uint c = 1; c < kCascades; ++c) { // the middle and finest cascades: river ripples, scaled down (short fetch, calm)
        const float sharp = saturate(2.5 - footprint / (TileMetres(c) / 256.0 * 1.5));
        const float calm = (c == 1u ? 0.25 : 0.45) * sqrt(wind);
        var += (c == 1u ? H5.y : H5.z) * 0.5 * calm * calm * (1.0 - sharp * sharp);
        if (sharp > 0.0) {
            const float2 a = WaveSample(c, look - flow * ph0 * cycle, 1).xy, b = WaveSample(c, look - flow * ph1 * cycle + 0.37 * TileMetres(c), 1).xy;
            grad += (a * w0 + b * (1.0 - w0)) * calm * sharp;
        }
    }
    const float3 n = normalize(float3(-grad.x, 1.0, -grad.y));
    const float rough = saturate(sqrt(var) * 2.0);
    // look knobs (FLY_OCEAN_DEBUG = 1000 + 100 a + 10 b + c tunes them for captures; the ocean takes any debug over 7.5 as relief off)
    float3 knob = float3(9.0, 6.0, 9.0);
    if (wc.extra.w > 999.5) { const float k = wc.extra.w - 1000.0; knob = float3(floor(k / 100.0), floor(fmod(k, 100.0) / 10.0), fmod(k, 10.0)); }
    const float bodyGain = 0.6 + knob.x * 0.2, glance = 1.0 - knob.y * 0.05, lowSky = knob.z / 9.0;
    const float nv = saturate(dot(n, v)) * glance; // a rippled sheet's facets: on the whole a little more of a glance than its mean
    // Fresnel at the ripples' mean slope: a rippled sheet seen at a glance reflects a little less than a mirror (facets turned away)
    const float F = Schlick(nv) * lerp(1.0, 0.86, saturate(rough * 3.0) * saturate(1.0 - v.y * 8.0));
    float3 r0 = reflect(-v, n);
    r0.y = abs(r0.y);
    // what it mirrors: from the air, the low sky by the horizon, pale with haze; ripples smear it a little up the sky
    const float3 rd = normalize(r0 + float3(0, min(rough * 0.3, 0.15), 0));
    const float3 sky = lerp(SkyLookup(rd), SkyLookup(normalize(float3(rd.x, rd.y * 0.3, rd.z))), lowSky);
    // the banks in the mirror: the embankments, trees and houses on the bank ahead (along the reflected ray) stand in it while the ray
    // is still below their tops (~24 m: the embankment, its trees and the houses behind); a ray that leaves the water at a glance climbs slowly, so the band of bank under them is wide
    const float2 bdir = i.bank.yz / max(length(i.bank.yz), 1e-3);
    const float hl = length(rd.xz);
    const float2 u = rd.xz / max(hl, 1e-4);
    const float rise = rd.y / max(hl, 1e-3);
    const float ca = dot(u, bdir);
    const float across = ca > 0.0 ? i.bank.x / max(ca, 0.05) : i.bank.w / max(-ca, 0.05);
    const float under = 24.0 / max(rise, 0.02);
    const float bankMir = (1.0 - smoothstep(under * 0.4, under * 1.5, across)) * saturate(i.world.w * 2.0);
    const float shade = SunShadowSoft(world + l * 6.0); // looked up 6 m toward the sun: the banks and the trees still shade it, the ground under the water does not
    const float3 ambient = SkyAmbient(float3(0, 1, 0));
    const float3 bankCol = (ambient * 0.9 + sunRgb * saturate(l.y) * 0.45) * float3(0.085, 0.095, 0.07); // trees, brick and stone in the sun's light: olive, dark
    const float3 reflection = lerp(sky, bankCol, bankMir * 0.85);
    const float3 glint = sunRgb * shade * Glitter(n, v, l, var + 0.0009) * F * (1.0 - bankMir);
    // a soft cap (a few times a sunlit white): many times the screen's white, the glint would clip even where the bank has faded the
    // water to a fifth, and show the tile's cells as stair steps
    const float3 glintC = glint / (1.0 + glint / (sunRgb * 0.5 + 1e-3));
    // the water itself: a silty tidal river (the Thames) is all but opaque, the silt scattering the light back up: grey-olive, tan;
    // darker in the banks' shade
    const float3 body = (ambient * 0.6 + sunRgb * saturate(l.y) * 0.4 * lerp(0.35, 1.0, shade)) * float3(0.15, 0.145, 0.115) * bodyGain;
    // the bank: the vertex alpha as it is, linear (sharpened, the far tiles' 40 m cells would show as scallops along the bank); the
    // glint, many times brighter than the land, fades out sooner
    const float edge = i.world.w;
    const float3 keep = 1.0 - edge; // opaque: silt hides the bed, and under it the terrain's own water mark (world.h) shines, cell by cell
    float3 light = edge * ((1.0 - F) * body + F * reflection + glintC * edge);
    if (wc.extra.w > 9.5 && wc.extra.w < 10.5) light = edge * F * reflection;           // FLY_OCEAN_DEBUG=10: the mirror alone
    else if (wc.extra.w > 10.5 && wc.extra.w < 11.5) light = edge * (1.0 - F) * body;   // 11: the body alone
    else if (wc.extra.w > 11.5 && wc.extra.w < 12.5) light = float3(bankMir, gust, F);  // 12: banks in the mirror, wind, Fresnel
    o.light = float4(min(light, 2000.0), 0.0);
    o.keep = float4(keep, 1.0); // the scene's alpha (its surface code) stays the land's
    return o;
}
#endif
