// The city's sunlit times of day, baked on the GPU (fly/city_shadow.h; Renderer::BakeCityShadow). The town's height map, seen from
// straight above (a ring buffer round the camera, kCityShadowSize texels of kCityShadowCell metres), is marched toward the sun at 96
// times between 06:00 and 18:00 (the sun's path is fixed: sun.h Sun::Direction), the moments a point goes into or out of shadow are
// refined by bisection, and the three longest sunlit spans are kept: six 10 bit times in a uint2. The lit draws then read one texel
// and compare the hour (city_shadow.hlsli). tools/shadow_horizon.py measured three spans wrong at 0.3 % of points over a day.
// Layout of the result: x = s0 | e0 << 10 | s1 << 20 | 1 << 31 (baked), y = e1 | s2 << 10 | e2 << 20; t = (hour - 6) / 12 * 1023.

static const int kN = 2560;          // renderer.h kCityShadowSize
static const float kCell = 2.0;      // kCityShadowCell
static const float kTop = 1500.0, kFar = 2000.0; // kCityShadowTop, kCityShadowFar: height = kTop - depth * kFar

struct BakeConstants {
    float4 a; // x, y: the ring texel of the region's corner (region texel 0, 0); z: the sun's tilt (degrees); w: march reach (metres)
    float4 b; // x: the highest height in the region (world metres), yzw spare
};
ConstantBuffer<BakeConstants> gBake : register(b0);
Texture2D<float> gHeights : register(t0);

struct VSOut { float4 pos : SV_Position; };
VSOut BakeVS(uint id : SV_VertexID) {
    VSOut o;
    const float2 uv = float2((id << 1) & 2, id & 2);
    o.pos = float4(uv * float2(2, -2) + float2(-1, 1), 0, 1);
    return o;
}

int2 RingOf(int2 region) { return (region + int2(gBake.a.xy)) % kN; }
// World height at a region texel; nothing there (cleared) and outside the region read as far below everything.
float HeightAt(int2 region) {
    if (any(region < 0) || any(region >= kN)) return -1e4;
    return kTop - gHeights.Load(int3(RingOf(region), 0)) * kFar;
}
float3 SunAt(float hour) { // sun.h Sun::Direction
    const float h = radians((hour - 12.0) * 15.0), t = radians(gBake.a.z);
    return normalize(float3(sin(h), cos(h) * cos(t), cos(h) * sin(t)));
}
// Whether the point at region texel p (centre), height h0, sees the sun at `hour`.
bool Lit(int2 p, float h0, float hour) {
    const float3 d = SunAt(hour);
    if (d.y < 0.02) return false;
    const float flat = length(d.xz), tanEl = d.y / max(flat, 1e-4);
    const float2 dir = d.xz / max(flat, 1e-4);
    const float rise = gBake.b.x - h0; // past this distance nothing can be tall enough
    const float reach = min(gBake.a.w, rise / tanEl);
    float s = kCell;
    [loop] for (int k = 0; k < 96 && s <= reach; ++k) {
        const int2 q = p + int2(round(dir * (s / kCell)));
        if (HeightAt(q) > h0 + 0.6 + s * tanEl) return false;
        s += max(kCell, s * 0.05);
    }
    return true;
}

uint2 BakePS(VSOut i) : SV_Target {
    const int2 ring = int2(i.pos.xy);
    const int2 p = (ring - int2(gBake.a.xy) + kN) % kN; // region texel
    const float h0 = HeightAt(p);
    if (h0 < -400.0) return uint2(1023u << 10 | 1u << 31, 0u); // nothing drawn here: lit all day
    // the spans, at 96 times and refined to ~0.5 minute
    const int kSamples = 96;
    const float h0Hour = 6.0, span = 12.0;
    float best[3][2] = {{0, 0}, {0, 0}, {0, 0}}; // the three longest, as hours
    bool prev = false;
    float start = h0Hour;
    [loop] for (int k = 1; k <= kSamples; ++k) {
        const float t1 = h0Hour + span * k / kSamples;
        const bool cur = k == kSamples ? false : Lit(p, h0, t1);
        if (cur != prev) {
            float lo = t1 - span / kSamples, hi = t1; // the change lies between: lo has `prev`, hi has `cur`
            [loop] for (int r = 0; r < 4; ++r) {
                const float mid = 0.5 * (lo + hi);
                if (Lit(p, h0, mid) == prev) lo = mid; else hi = mid;
            }
            const float at = 0.5 * (lo + hi);
            if (cur) start = at;
            else { // a span closed: keep it if it is among the three longest
                const float len = at - start;
                if (len > best[2][1] - best[2][0]) {
                    best[2][0] = start; best[2][1] = at;
                    if (best[2][1] - best[2][0] > best[1][1] - best[1][0]) {
                        const float a0 = best[1][0], a1 = best[1][1]; best[1][0] = best[2][0]; best[1][1] = best[2][1]; best[2][0] = a0; best[2][1] = a1;
                        if (best[1][1] - best[1][0] > best[0][1] - best[0][0]) {
                            const float c0 = best[0][0], c1 = best[0][1]; best[0][0] = best[1][0]; best[0][1] = best[1][1]; best[1][0] = c0; best[1][1] = c1;
                        }
                    }
                }
            }
            prev = cur;
        }
    }
    uint q[6];
    [unroll] for (int s = 0; s < 3; ++s) {
        const bool has = best[s][1] > best[s][0];
        q[s * 2] = has ? (uint)clamp(round((best[s][0] - h0Hour) / span * 1023.0), 0.0, 1023.0) : 0u;
        q[s * 2 + 1] = has ?(uint)clamp(round((best[s][1] - h0Hour) / span * 1023.0), 0.0, 1023.0) : 0u;
    }
    return uint2(q[0] | q[1] << 10 | q[2] << 20 | 1u << 31, q[3] | q[4] << 10 | q[5] << 20);
}
