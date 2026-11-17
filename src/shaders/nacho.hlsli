// "Nacho" impostor cards (render/nacho.h, Renderer::DrawNachoCards), included at the end of mesh.hlsl.
// After Thomas Murphy's "Imposter cards - nacho time" (BCON26). Each asset is baked from six sides (+X -X +Z -Z +Y -Y) into one
// shared atlas pair (gAlbedo: premultiplied display-space colour + coverage; gNormalMap: object-space normal * 0.5 + 0.5, a = the
// surface's offset along the face's direction / half size * 0.5 + 0.5). Every instance is one quad facing the eye (in the shadow pass:
// the sun). Per pixel the ray, in the asset's own space, meets three slice planes through its centre (one per axis, skipped when seen
// nearly edge-on: |rd.axis| < 0.4); the nearest opaque hit wins (at most three atlas reads), slides to the baked surface depth and
// writes it. The quad stands pulled toward the viewer, so every surface lies behind it: the depth written is only ever farther
// (conservative depth keeps early rejection, which tile GPUs need).
// Instance rows (InstanceIn, 12 floats; nacho.h NachoInstance packs them):
//   row0  position xyz (world), scale            row1  yaw (radians), tint (r + g*256 + b*65536, 8 bits each), centre x, centre z
//   row2  centre y, half size h (asset units), cell origin (x/16 + y/16*1024, atlas texels), cell size (texels)
// pc.mvp: the view-projection (the shadow pass: the light's). pc.model[0]: fade near, fade far (metres; far <= near = no fade), gain, debug.
struct NachoVSOut {
    noperspective centroid float4 pos : SV_Position; // fxc: a conservative depth output wants this on the position
    float3 world : TEXCOORD0;                   // on the quad
    float4 clip : TEXCOORD1;                    // the same point in clip space: depth along the ray is linear in it
    nointerpolation float4 rayClip : TEXCOORD2; // perspective: the eye in clip space; the shadow pass: the ray's direction in clip space
    nointerpolation float4 inst : TEXCOORD3;    // position, scale
    nointerpolation float4 asset : TEXCOORD4;   // centre (asset space), h
    nointerpolation float4 cell : TEXCOORD5;    // cell origin xy (texels), cell size, yaw
    nointerpolation float4 tint : TEXCOORD6;    // rgb tint * gain, a the fade (0 hidden .. 1 shown) + 10 * the debug view
};
float3 NachoYaw(float3 v, float c, float s) { return float3(c * v.x + s * v.z, v.y, -s * v.x + c * v.z); }
// ---- box buildings (fly/city_nacho.h, instance scale -2): the building's own box, a unit cube mesh scaled to it, drawn whole; the
// pixel shader keeps the (at most three) faces turned to the viewer and colours each from that side's baked view. One texture
// read a pixel, the box's own depth: cheap on tile GPUs (Quest). row1.x carries the box's half extents as fractions of h, 8 bits each.
float3 NachoBoxExtents(float packed, float h) {
    const float z = floor(packed / 65536.0), y = floor((packed - z * 65536.0) / 256.0), x = packed - z * 65536.0 - y * 256.0;
    return max(float3(x, y, z) / 255.0, 0.004) * h;
}
NachoVSOut NachoBoxVertex(VSIn v, InstanceIn inst, bool shadow) {
    NachoVSOut o;
    const float3 ip = inst.row0.xyz;
    const float h = inst.row2.y;
    const float3 C = float3(inst.row1.z, inst.row2.x, inst.row1.w);
    const float3 E = NachoBoxExtents(inst.row1.x, h);
    o.world = ip + C + v.pos.xyz * E;
    o.pos = mul(pc.mvp, float4(o.world, 1.0));
    o.clip = o.pos;
    o.rayClip = shadow ? mul(pc.mvp, float4(-gLight.sunDir.xyz, 0.0)) : mul(pc.mvp, float4(gLight.eye.xyz, 1.0));
    o.inst = float4(ip, 1.0);
    o.asset = float4(C, h);
    const float packed = inst.row2.z;
    const float cy = floor(packed / 1024.0);
    o.cell = float4((packed - cy * 1024.0) * 16.0, cy * 16.0, inst.row2.w, inst.row1.x); // w: the packed extents (yaw is 0)
    const float t = inst.row1.y;
    const float b = floor(t / 65536.0), g = floor((t - b * 65536.0) / 256.0), r = t - b * 65536.0 - g * 256.0;
    const float4 prm = pc.model[0];
    o.tint = float4(float3(r, g, b) / 255.0 * (prm.z > 0.0 ? prm.z : 1.0), 1.0 + 40.0); // fade 1, debug 4: a box
    return o;
}
NachoVSOut NachoVertex(VSIn v, InstanceIn inst, bool shadow) {
    if (inst.row0.w < -1.5) return NachoBoxVertex(v, inst, shadow);
    NachoVSOut o;
    const float3 ip = inst.row0.xyz;
    const bool solid = inst.row0.w < 0.0; // a building (fly/city_nacho.h): negative scale
    const bool murphy = inst.row0.w < -2.5; // -3: Thomas Murphy's card (yaw slot = the box's packed extents, row2.w = texels a metre)
    const float sc = murphy ? 1.0 : abs(inst.row0.w), yaw = murphy ? 0.0 : inst.row1.x, h = inst.row2.y;
    const float3 C = float3(inst.row1.z, inst.row2.x, inst.row1.w);
    const float3 cw = ip + NachoYaw(C, cos(yaw), sin(yaw)) * sc;
    // the slice box (half size h) from any side. A tree's shadow card only has to hold its crown, which is round and fits within h of
    // the centre, not the box's corners (1.75 h): ~2.5x fewer shadow-map pixels, each running the trace and writing depth, for the
    // tens of thousands of trees that cast into the sharp sun map (that was ~90 ms a frame low over a town).
    const float rs = h * sc * (shadow && !solid ? 1.1 : 1.75);
    const float3 toEye = gLight.eye.xyz - cw;
    const float d = max(length(toEye), 1e-3);
    const float3 vd = shadow ? gLight.sunDir.xyz : toEye / d;
    // pulled toward the viewer by the box's radius (so nothing is in front of it), and grown to still cover the box in perspective
    const float off = shadow ? rs : min(rs, d * 0.8);
    const float R = shadow ? rs : min((d - off) * rs / sqrt(max(d * d - rs * rs, 0.04 * rs * rs)), rs * 8.0);
    const float3 up = abs(vd.y) > 0.98 ? float3(0, 0, -1) : float3(0, 1, 0);
    const float3 rt = normalize(cross(up, vd)), u2 = cross(vd, rt);
    o.world = cw + vd * off + (rt * v.pos.x + u2 * v.pos.y) * R;
    o.pos = mul(pc.mvp, float4(o.world, 1.0));
    o.clip = o.pos;
    o.rayClip = shadow ? mul(pc.mvp, float4(-gLight.sunDir.xyz, 0.0)) : mul(pc.mvp, float4(gLight.eye.xyz, 1.0));
    o.inst = float4(inst.row0.xyz, sc);
    o.asset = float4(C, h);
    const float packed = inst.row2.z;
    const float cy = floor(packed / 1024.0); // exact: integers under 2^24, divided by powers of two
    o.cell = float4((packed - cy * 1024.0) * 16.0, cy * 16.0, inst.row2.w, murphy ? inst.row1.x : yaw);
    const float t = inst.row1.y;
    const float b = floor(t / 65536.0), g = floor((t - b * 65536.0) / 256.0), r = t - b * 65536.0 - g * 256.0;
    const float4 prm = pc.model[0]; // fade near, fade far, gain, debug
    const float fade = prm.y > prm.x ? smoothstep(prm.x, prm.y, d) : 1.0;
    o.tint = float4(float3(r, g, b) / 255.0 * (prm.z > 0.0 ? prm.z : 1.0), (shadow ? 1.0 : fade) + 10.0 * (murphy ? 5.0 : solid ? 3.0 : round(prm.w))); // + 10 * debug (3: a building): the pixel shader cannot read pc
    return o;
}
NachoVSOut NachoVS(VSIn v, InstanceIn inst) { return NachoVertex(v, inst, false); }
NachoVSOut NachoShadowVS(VSIn v, InstanceIn inst) { return NachoVertex(v, inst, true); }

struct NachoHitInfo { float4 albedo, nd; int face; float along; }; // along: metres from the quad to the surface along the ray
float2 NachoCellUv(NachoVSOut i, int face, float2 uv, float2 atlas) {
    const float2 q = clamp(uv, 0.002, 0.998);
    return (i.cell.xy + float2((face + q.x) * i.cell.z, (1.0 - q.y) * i.cell.z)) / atlas; // rows run top down in the bake
}
bool NachoTrace(NachoVSOut i, bool shadow, out NachoHitInfo hit) {
    hit.albedo = float4(0, 0, 0, 0); hit.nd = float4(0.5, 0.5, 1, 0.5); hit.face = -1; hit.along = 0.0;
    float2 atlas;
    gAlbedo.GetDimensions(atlas.x, atlas.y);
    const float sc = i.inst.w, yaw = i.cell.w, h = i.asset.w;
    const float3 C = i.asset.xyz;
    const float c = cos(-yaw), s = sin(-yaw);
    // the mip: this pixel's footprint on the quad against a texel's size (the quad is continuous, the slices are not)
    const float foot = max(length(ddx(i.world)), length(ddy(i.world)));
    const float lod = clamp(log2(max(foot * i.cell.z / (2.0 * h * sc), 1e-6)), 0.0, 4.0); // past mip 4 a cell bleeds into its neighbours
    const float3 rdw = shadow ? -gLight.sunDir.xyz : normalize(i.world - gLight.eye.xyz);
    const float3 rd = NachoYaw(rdw, c, s);
    const float L = 4.0 * h; // start the ray outside the box, ahead of the quad
    const float3 ro = NachoYaw(i.world - i.inst.xyz, c, s) / sc - rd * L;
    float t[3] = {1e9, 1e9, 1e9};
    int f[3] = {0, 0, 0};
    float2 uv[3] = {float2(0, 0), float2(0, 0), float2(0, 0)};
    const float inv2h = 0.5 / h;
    if (abs(rd.x) > 0.4) {
        const float tt = (C.x - ro.x) / rd.x;
        const float3 q = ro + rd * tt - C;
        if (tt > 0.0 && abs(q.y) < h && abs(q.z) < h) { const bool pos = ro.x > C.x; t[0] = tt; f[0] = pos ? 0 : 1; uv[0] = float2((pos ? -q.z : q.z) * inv2h + 0.5, q.y * inv2h + 0.5); }
    }
    if (abs(rd.z) > 0.4) {
        const float tt = (C.z - ro.z) / rd.z;
        const float3 q = ro + rd * tt - C;
        if (tt > 0.0 && abs(q.x) < h && abs(q.y) < h) { const bool pos = ro.z > C.z; t[1] = tt; f[1] = pos ? 2 : 3; uv[1] = float2((pos ? q.x : -q.x) * inv2h + 0.5, q.y * inv2h + 0.5); }
    }
    if (abs(rd.y) > 0.4) {
        const float tt = (C.y - ro.y) / rd.y;
        const float3 q = ro + rd * tt - C;
        if (tt > 0.0 && abs(q.x) < h && abs(q.z) < h) { const bool pos = ro.y > C.y; t[2] = tt; f[2] = pos ? 4 : 5; uv[2] = float2(q.x * inv2h + 0.5, (pos ? -q.z : q.z) * inv2h + 0.5); }
    }
    float ht = 0.0;
    float2 fuv = float2(0, 0);
    // a crown is sparse leaves: its mips average the coverage down, and a fixed 0.5 cut eats a far crown to a few specks (small, thin
    // trees from the air). The cut falls with the mip so a far crown keeps its full, closed silhouette.
    const float cut = 0.5 - 0.075 * lod;
    [unroll] for (int k = 0; k < 3; ++k) { // nearest first; the first opaque one wins
        int bi = 0;
        float bt = t[0];
        if (t[1] < bt) { bt = t[1]; bi = 1; }
        if (t[2] < bt) { bt = t[2]; bi = 2; }
        if (bt >= 1e8) break;
        const int fc = bi == 0 ? f[0] : bi == 1 ? f[1] : f[2];
        const float2 u = bi == 0 ? uv[0] : bi == 1 ? uv[1] : uv[2];
        const float4 a = gAlbedo.SampleLevel(gTexSampler, NachoCellUv(i, fc, u, atlas), lod);
        if (a.a > cut) { hit.albedo = a; hit.face = fc; fuv = u; ht = bt; break; }
        if (bi == 0) t[0] = 1e9; else if (bi == 1) t[1] = 1e9; else t[2] = 1e9;
    }
    if (hit.face < 0) return false;
    hit.nd = gNormalMap.SampleLevel(gTexSampler, NachoCellUv(i, hit.face, fuv, atlas), lod);
    // the real surface: slide along the ray from the slice to the baked offset (one parallax step)
    const int fc = hit.face;
    const float3 D = fc == 0 ? float3(1, 0, 0) : fc == 1 ? float3(-1, 0, 0) : fc == 2 ? float3(0, 0, 1) : fc == 3 ? float3(0, 0, -1) : fc == 4 ? float3(0, 1, 0) : float3(0, -1, 0);
    const float offs = (hit.nd.a * 2.0 - 1.0) * h, dn = dot(rd, D);
    float ts = abs(dn) > 0.05 ? (offs - dot(ro - C, D)) / dn : ht;
    ts = clamp(ts, ht - h * 1.8, ht + h * 1.8);
    hit.along = max(ts - L, 0.0) * sc;
    return true;
}
float NachoDepth(NachoVSOut i, float along, bool shadow) {
    float4 c;
    if (shadow) c = i.clip + i.rayClip * along;
    else c = i.clip + (i.clip - i.rayClip) * (along / max(length(i.world - gLight.eye.xyz), 1e-4));
    return saturate(c.z / c.w);
}
struct NachoOut {
    float4 color : SV_Target0;
    float depth : SV_DepthLessEqual; // reversed z: farther is less
};
// ---- buildings (fly/city_nacho.h): instances with a negative scale. A building is solid, so instead of the first opaque slice
// (a crown's rule, which shows a box as crossed cardboard) the ray marches the DEPTH HULL: a point is inside when, for each axis, it
// lies in that axis's baked silhouette and between the +side's and the -side's baked surfaces (below the top only on y: a mesh
// without a floor shows its roof's underside to -Y). The hit is coloured by the camera-facing side whose surface it sits on.
float NachoOffset(NachoVSOut i, int face, float2 uv, float2 atlas, float lod) { return gNormalMap.SampleLevel(gTexSampler, NachoCellUv(i, face, uv, atlas), lod).a * 2.0 - 1.0; }
float2 NachoFaceUv(int f, float3 q, float inv2h) {
    const float2 a = f == 0 ? float2(-q.z, q.y) : f == 1 ? float2(q.z, q.y) : f == 2 ? float2(q.x, q.y) : f == 3 ? float2(-q.x, q.y) : f == 4 ? float2(q.x, -q.z) : float2(q.x, q.z);
    return a * inv2h + 0.5;
}
float3 NachoFaceDir(int f) { return f == 0 ? float3(1, 0, 0) : f == 1 ? float3(-1, 0, 0) : f == 2 ? float3(0, 0, 1) : f == 3 ? float3(0, 0, -1) : f == 4 ? float3(0, 1, 0) : float3(0, -1, 0); }
bool NachoInside(NachoVSOut i, float3 q, float h, float inv2h, float2 atlas, float lod) {
    const float eps = h * 0.012;
    [unroll] for (int a = 0; a < 3; ++a) {
        const int fp = a * 2;
        const float2 u = NachoFaceUv(fp, q, inv2h);
        if (any(u < 0.0) || any(u > 1.0)) return false;
        if (gAlbedo.SampleLevel(gTexSampler, NachoCellUv(i, fp, u, atlas), lod).a < 0.5) return false;
        const float c = dot(q, NachoFaceDir(fp));
        if (c > NachoOffset(i, fp, u, atlas, lod) * h + eps) return false;
        if (a < 2 && -c > NachoOffset(i, fp + 1, NachoFaceUv(fp + 1, q, inv2h), atlas, lod) * h + eps) return false;
    }
    return true;
}
bool NachoSolidTrace(NachoVSOut i, bool shadow, out NachoHitInfo hit) {
    hit.albedo = float4(0, 0, 0, 0); hit.nd = float4(0.5, 0.5, 1, 0.5); hit.face = -1; hit.along = 0.0;
    float2 atlas;
    gAlbedo.GetDimensions(atlas.x, atlas.y);
    const float sc = i.inst.w, yaw = i.cell.w, h = i.asset.w, inv2h = 0.5 / h;
    const float3 C = i.asset.xyz;
    const float c = cos(-yaw), s = sin(-yaw);
    const float foot = max(length(ddx(i.world)), length(ddy(i.world)));
    const float lod = shadow ? 0.0 : clamp(log2(max(foot * i.cell.z / (2.0 * h * sc), 1e-6)) - 0.5, 0.0, 3.0);
    const float3 rdw = shadow ? -gLight.sunDir.xyz : normalize(i.world - gLight.eye.xyz);
    const float3 rd = NachoYaw(rdw, c, s);
    const float L = 4.0 * h;
    const float3 ro = NachoYaw(i.world - i.inst.xyz, c, s) / sc - rd * L;
    const float3 rdi = 1.0 / (rd + (step(0.0, rd) * 2.0 - 1.0) * 1e-6);
    const float3 t0 = (C - h - ro) * rdi, t1 = (C + h - ro) * rdi, tn = min(t0, t1), tf = max(t0, t1);
    const float ta = max(max(tn.x, tn.y), max(tn.z, 0.0)), tb = min(min(tf.x, tf.y), tf.z);
    if (ta >= tb) return false;
    const int kSteps = 32;
    const float dt = (tb - ta) / kSteps;
    float lo = ta, hi = -1.0;
    [loop] for (int k = 0; k <= kSteps; ++k) {
        const float tt = ta + dt * k;
        if (NachoInside(i, ro + rd * tt - C, h, inv2h, atlas, lod)) { hi = tt; break; }
        lo = tt;
    }
    if (hi < 0.0) return false;
    [unroll] for (int b = 0; b < 6; ++b) { const float m = 0.5 * (lo + hi); if (NachoInside(i, ro + rd * m - C, h, inv2h, atlas, lod)) hi = m; else lo = m; }
    const float3 q = ro + rd * hi - C;
    // the side it sits on: of the three facing the ray, the one whose baked surface is nearest the hit (fully covered texels only:
    // a wall's points lie on the silhouette edge of the views across it)
    float best = 1e9;
    int bf = 0;
    [unroll] for (int a = 0; a < 3; ++a) {
        const int fs = a * 2 + (dot(rd, NachoFaceDir(a * 2)) < 0.0 ? 0 : 1);
        const float2 u = NachoFaceUv(fs, q, inv2h);
        const float cov = gAlbedo.SampleLevel(gTexSampler, NachoCellUv(i, fs, u, atlas), lod).a;
        const float e = abs(dot(q, NachoFaceDir(fs)) - NachoOffset(i, fs, u, atlas, lod) * h) + (1.0 - cov) * h * 2.0;
        if (e < best) { best = e; bf = fs; }
    }
    const float2 fu = NachoFaceUv(bf, q, inv2h);
    hit.face = bf;
    hit.albedo = gAlbedo.SampleLevel(gTexSampler, NachoCellUv(i, bf, fu, atlas), lod);
    hit.albedo.a = max(hit.albedo.a, 0.5);
    hit.nd = gNormalMap.SampleLevel(gTexSampler, NachoCellUv(i, bf, fu, atlas), lod);
    hit.along = max(hi - L, 0.0) * sc;
    return true;
}
NachoOut NachoShadeBuilding(NachoVSOut i, NachoHitInfo hit);
NachoOut NachoBuildingPS(NachoVSOut i) {
    NachoHitInfo hit;
    if (!NachoSolidTrace(i, false, hit)) discard;
    return NachoShadeBuilding(i, hit);
}
// The box's face under this pixel: the axis on whose face the point lies; false for the bottom.
bool NachoBoxFace(NachoVSOut i, out int face, out float3 q, out float3 N) {
    const float h = i.asset.w;
    const float3 E = NachoBoxExtents(i.cell.w, h);
    q = i.world - i.inst.xyz - i.asset.xyz;
    const float3 r = abs(q) / E;
    if (r.x >= r.y && r.x >= r.z) { face = q.x > 0.0 ? 0 : 1; }
    else if (r.z >= r.y) { face = q.z > 0.0 ? 2 : 3; }
    else { face = q.y > 0.0 ? 4 : 5; }
    N = NachoFaceDir(face);
    return face != 5;
}
NachoOut NachoBoxPS(NachoVSOut i) {
    int face; float3 q, N;
    if (!NachoBoxFace(i, face, q, N)) discard;
    const float3 rd = normalize(i.world - gLight.eye.xyz);
    if (dot(rd, N) >= 0.0) discard; // a back face: the three facing the viewer are the box
    float2 atlas;
    gAlbedo.GetDimensions(atlas.x, atlas.y);
    const float h = i.asset.w;
    const float foot = max(length(ddx(i.world)), length(ddy(i.world)));
    const float lod = clamp(log2(max(foot * i.cell.z / (2.0 * h), 1e-6)), 0.0, 4.0);
    const float2 uv = NachoFaceUv(face, q, 0.5 / h);
    NachoHitInfo hit;
    hit.face = face;
    hit.albedo = gAlbedo.SampleLevel(gTexSampler, NachoCellUv(i, face, uv, atlas), lod);
    if (hit.albedo.a < 0.5) discard; // nothing of the building on this side here (a gable's corners, an L's notch)
    hit.nd = gNormalMap.SampleLevel(gTexSampler, NachoCellUv(i, face, uv, atlas), lod);
    // the real surface lies behind the face by the baked offset: along the ray to it (for depth and lighting)
    const float offs = (hit.nd.a * 2.0 - 1.0) * h;
    hit.along = clamp((offs - dot(q, N)) / dot(rd, N), 0.0, 2.0 * h);
    return NachoShadeBuilding(i, hit);
}
// ---- Thomas Murphy's card (fly/city_nacho.h mode 2): the view ray meets three planes, one per axis, each at the building's box face on
// the ray's side (his per-axis plane depth); the one the ray enters the box through is in front. Its view is read ONCE from a tight
// atlas rectangle with that view's own proportions (city_nacho_tight.h; FaceRect there must match NachoTightTexel here).
float2 NachoTightTexel(int f, float2 uv, float3 E, float k, float2 origin) {
    const float3 W = floor(2.0 * E * k) + 1.0;
    const float G = 2.0;
    float ox, w, hg;
    if (f == 0) { ox = 0.0; w = W.z; hg = W.y; }
    else if (f == 1) { ox = W.z + G; w = W.z; hg = W.y; }
    else if (f == 2) { ox = 2.0 * W.z + 2.0 * G; w = W.x; hg = W.y; }
    else if (f == 3) { ox = 2.0 * W.z + W.x + 3.0 * G; w = W.x; hg = W.y; }
    else { ox = 2.0 * W.z + 2.0 * W.x + 4.0 * G; w = W.x; hg = W.z; }
    return origin + float2(ox + clamp(uv.x * w, 0.5, w - 0.5), clamp((1.0 - uv.y) * hg, 0.5, hg - 0.5));
}
float2 NachoTightUv(int f, float3 q, float3 E) {
    const float2 a = f == 0 ? float2(-q.z / E.z, q.y / E.y) : f == 1 ? float2(q.z / E.z, q.y / E.y) : f == 2 ? float2(q.x / E.x, q.y / E.y)
                   : f == 3 ? float2(-q.x / E.x, q.y / E.y) : float2(q.x / E.x, -q.z / E.z);
    return a * 0.5 + 0.5;
}
bool NachoMurphyTrace(NachoVSOut i, bool shadow, out NachoHitInfo hit) {
    hit.albedo = float4(0, 0, 0, 0); hit.nd = float4(0.5, 0.5, 1, 1); hit.face = -1; hit.along = 0.0;
    float2 atlas;
    gAlbedo.GetDimensions(atlas.x, atlas.y);
    const float h = i.asset.w, k = i.cell.z;
    const float3 C = i.asset.xyz, E = NachoBoxExtents(i.cell.w, h);
    const float3 rd = shadow ? -gLight.sunDir.xyz : normalize(i.world - gLight.eye.xyz);
    const float L = 4.0 * h;
    const float3 q0 = (i.world - i.inst.xyz) - rd * L - C; // the ray's start, outside the box, relative to its centre
    float best = -1.0;
    int face = -1;
    float3 q = float3(0, 0, 0);
    [unroll] for (int a = 0; a < 3; ++a) {
        const float s = q0[a] > 0.0 ? 1.0 : -1.0; // the plane on the ray's side
        if (a == 1 && s < 0.0) continue;        // no bottom
        if (abs(rd[a]) < 1e-5) continue;
        const float t = (s * E[a] - q0[a]) / rd[a];
        const float3 p = q0 + rd * t;
        const int b = (a + 1) % 3, c = (a + 2) % 3;
        if (t > 0.0 && abs(p[b]) <= E[b] * 1.001 && abs(p[c]) <= E[c] * 1.001 && t > best) {
            best = t; q = p;
            face = a == 0 ? (s > 0.0 ? 0 : 1) : a == 2 ? (s > 0.0 ? 2 : 3) : 4;
        }
    }
    if (face < 0) return false;
    const float foot = max(length(ddx(i.world)), length(ddy(i.world)));
    const float lod = shadow ? 0.0 : clamp(log2(max(foot * k, 1e-6)), 0.0, 1.0); // the 2-texel gutters hold one mip
    const float2 tc = NachoTightTexel(face, NachoTightUv(face, q, E), E, k, i.cell.xy) / atlas;
    hit.albedo = gAlbedo.SampleLevel(gTexSampler, tc, lod);
    if (hit.albedo.a < 0.5) return false; // nothing of the building on that side there
    hit.nd = gNormalMap.SampleLevel(gTexSampler, tc, lod);
    hit.face = face;
    // the written depth: slid from the box face to the baked surface (the colour stays the face's, as his card does)
    const float3 N = NachoFaceDir(face);
    const int ax = face < 2 ? 0 : face < 4 ? 2 : 1;
    const float offs = (hit.nd.a * 2.0 - 1.0) * E[ax];
    const float dn = dot(rd, N);
    const float slide = dn < -0.05 ? clamp((offs - dot(q, N)) / dn, 0.0, 2.0 * h) : 0.0;
    hit.along = max(best + slide - L, 0.0);
    return true;
}
NachoOut NachoMurphyPS(NachoVSOut i) {
    NachoHitInfo hit;
    if (!NachoMurphyTrace(i, false, hit)) discard;
    return NachoShadeBuilding(i, hit);
}
NachoOut NachoShadeBuilding(NachoVSOut i, NachoHitInfo hit) {
    NachoOut o;
    o.depth = min(NachoDepth(i, hit.along, false), i.pos.z);
    const float sc = i.inst.w, yaw = i.tint.a > 35.0 ? 0.0 : i.cell.w; // a box's cell.w holds its extents (it never turns)
    const float3 sw = i.world + normalize(i.world - gLight.eye.xyz) * hit.along;
    const float3 n = normalize(NachoYaw(hit.nd.xyz * 2.0 - 1.0, cos(yaw), sin(yaw)) + float3(0, 1e-4, 0));
    float3 albedo = pow(saturate(hit.albedo.rgb / max(hit.albedo.a, 1e-3)), 2.2) * i.tint.rgb;
    const float eyeDist = length(gLight.eye.xyz - sw);
    if (abs(n.y) < 0.4) { // walls past resolving their floors pull toward a shared facade brightness (AtlasBuildingPS)
        const float fpw = max(length(ddx(i.world)), length(ddy(i.world)));
        const float k = smoothstep(0.35, 1.4, fpw) * 0.5;
        const float Lm = max(dot(albedo, float3(0.2126, 0.7152, 0.0722)), 1e-3);
        albedo *= exp(log(0.16 / Lm) * k);
        albedo = lerp(albedo, dot(albedo, float3(0.2126, 0.7152, 0.0722)), k * 0.6);
    }
    if (n.y > 0.5) { // roofs: the aerial photograph's colour from afar (AtlasBuildingPS / RoofPS)
        float w;
        const float3 img = GroundImagery(sw, eyeDist, w);
        w *= smoothstep(60.0, 250.0, eyeDist);
        if (w > 0.0) {
            const float la = max(dot(albedo, float3(0.2126, 0.7152, 0.0722)), 1e-4), li = dot(img, float3(0.2126, 0.7152, 0.0722));
            albedo = lerp(albedo, lerp(albedo * (li / la), img, saturate(gLight.portals[69].x + 0.2)), w);
        }
    }
    const float3 lo = sw + gLight.sunDir.xyz * (gLight.shadowParams.z * 2.0 + 0.012 * i.asset.w * sc);
    gShadowPixel = i.pos.xy;
    gTightSoft = 1.0;
    const float3 loShadow = mul(gLight.shadowVP, float4(lo, 1.0)).xyz;
    gPixelShadow = ShadowFactor(loShadow, mul(gLight.tightVP, float4(lo, 1.0)).xyz);
    gPixelShadow = min(gPixelShadow, CityShadow(sw, n, ShadowNoise(), loShadow));
    gMatAO = 1.0;
    const float roughness = 0.85, metal = 0.0;
    const float cloudSun = CloudSun(sw);
    float share;
    const float3 lit = LitPbr(albedo, n, sw, gPixelShadow * cloudSun, roughness, metal, cloudSun, share);
    o.color = float4(lit, 0.76 + 0.23 * share);
    return o;
}
NachoOut NachoPS(NachoVSOut i) {
    const float debug = floor(i.tint.a / 10.0 + 1e-3), fade = i.tint.a - debug * 10.0; // the card dithers in over [near, far] with the exact complement of the mesh's SetDrawFade(-fade)
    if (fade <= 0.001 || (fade < 0.999 && FadeNoise(i.pos.xy) >= fade)) discard;
    if (debug > 4.5) return NachoMurphyPS(i);
    if (debug > 3.5) return NachoBoxPS(i);
    if (debug > 2.5) return NachoBuildingPS(i);
    NachoHitInfo hit;
    if (!NachoTrace(i, false, hit)) discard;
    NachoOut o;
    o.depth = min(NachoDepth(i, hit.along, false), i.pos.z);
    const float sc = i.inst.w, yaw = i.tint.a > 35.0 ? 0.0 : i.cell.w; // a box's cell.w holds its extents (it never turns)
    const float3 sw = i.world + normalize(i.world - gLight.eye.xyz) * hit.along;
    float3 n = normalize(NachoYaw(hit.nd.xyz * 2.0 - 1.0, cos(yaw), sin(yaw)) + float3(0, 1e-4, 0));
    const float depthOff = hit.nd.a * 2.0 - 1.0;
    float ao = lerp(0.2, 1.0, smoothstep(-0.7, 0.8, depthOff)); // deep in the canopy (a large baked offset behind the slice): occluded
    // The crown as one volume (what makes a city's trees read from the air, as photogrammetry shows them: a sunlit cap, a dark
    // underside, the side away from the sun in the crown's own shade). Where the hit lies in the crown: the slice box's centre, the
    // crown's a little above it (the bole below). The leaf normal bends toward the crown's outward direction, more with distance
    // (far off, a leaf's own normal is only noise); the lower crown and the side turned from the sun lose sky and sun.
    const float R = i.asset.w * sc;
    const float3 cw = i.inst.xyz + NachoYaw(i.asset.xyz, cos(yaw), sin(yaw)) * sc;
    const float3 rel = (sw - cw) / R - float3(0.0, 0.15, 0.0);
    const float3 ns = normalize(rel + float3(0.0, 0.25, 0.0));
    const float eyeD = length(sw - gLight.eye.xyz);
    n = normalize(lerp(n, ns, lerp(0.5, 0.8, saturate(eyeD / 1200.0))));
    const float sunSide = dot(ns, gLight.sunDir.xyz);
    // far off (past ~1 km) a crown is a few pixels: its own shading would only darken the whole canopy into noise, so it eases
    // back to the plain card's light and the haze takes over
    const float nearF = 1.0 - smoothstep(900.0, 2600.0, eyeD);
    ao *= lerp(1.0, lerp(0.35, 1.0, smoothstep(-0.75, 0.45, rel.y)) * lerp(0.7, 1.0, smoothstep(-0.5, 0.3, sunSide)), nearF);
    const float crownShade = lerp(1.0, lerp(0.25, 1.0, smoothstep(-0.35, 0.35, sunSide)), nearF); // the crown shades its own far side
    // sunlit leaf at the crown's cap glows a warmer olive (light through and off the outer leaves); the inside stays a cool deep green
    const float cap = saturate(sunSide) * smoothstep(-0.1, 0.6, rel.y);
    if (debug > 0.5) { // debug: 1 the face that coloured the pixel, 2 the albedo unlit
        const float3 kFace[6] = {float3(1, 0, 0), float3(0.4, 0, 0), float3(0, 0, 1), float3(0, 0, 0.4), float3(0, 1, 0), float3(0, 0.4, 0)};
        o.color = float4(debug > 1.5 ? pow(saturate(hit.albedo.rgb / hit.albedo.a), 2.2) * 3.0 : kFace[hit.face] * 3.0, 0.9);
        return o;
    }
    // real shadows: the card's own surface is in the sun's map (NachoShadowPS), so this is self-shadowing too
    const float3 lo = sw + gLight.sunDir.xyz * (gLight.shadowParams.z * 2.0 + 0.012 * i.asset.w * sc);
    gShadowPixel = i.pos.xy;
    gTightSoft = 1.0;
    const float3 loShadow = mul(gLight.shadowVP, float4(lo, 1.0)).xyz;
    gPixelShadow = ShadowFactor(loShadow, mul(gLight.tightVP, float4(lo, 1.0)).xyz) * crownShade;
    gPixelShadow = lerp(gPixelShadow, max(gPixelShadow, 0.6), 1.0 - nearF); // far off the map's coarse cascade buries whole stands in each other's shade
    gPixelShadow *= lerp(1.0, CityShadow(sw, normalize(float3(gLight.sunDir.x, 0.0, gLight.sunDir.z) + 1e-4), ShadowNoise(), loShadow), 0.85); // the buildings' baked shade (city_shadow.hlsli)
    gMatAO = ao;
    gSpecular = 0.3; // a crown's sheen is lost among its own leaves' shade: full sky specular frosted the dark leaf grey-blue
    float roughness, metal;
    PbrFromAlpha(1.0, roughness, metal);
    const float3 albedo = pow(saturate(hit.albedo.rgb / max(hit.albedo.a, 1e-3)), 2.2) * i.tint.rgb * lerp(1.0, float3(1.25, 1.08, 0.7), cap)
                         * (1.0 + 0.3 * (1.0 - nearF)); // a far crown is its sunlit outer leaves (its dark heart is hidden behind them)
    const float cloudSun = CloudSun(sw);
    float share;
    const float3 lit = LitPbr(albedo, n, sw, gPixelShadow * cloudSun, roughness, metal, cloudSun, share);
    o.color = float4(lit, 0.76 + 0.23 * share);
    return o;
}
// The shadow caster: the same trace from the sun, depth only. The map is not reversed (LESS): farther is greater. Shader-written
// depth gets no rasterizer bias, so the bias is added here.
float NachoShadowPS(NachoVSOut i) : SV_DepthGreaterEqual {
    if (i.tint.a > 45.0) { // Murphy's card (tint.a 51): the same three planes, from the sun
        NachoHitInfo mh;
        if (!NachoMurphyTrace(i, true, mh)) discard;
        return max(NachoDepth(i, mh.along, true), i.pos.z) + 0.00025;
    }
    if (i.tint.a > 35.0) { // a box (tint.a 41): its faces toward the sun, cut where the side's view is empty
        int face; float3 q, N;
        if (!NachoBoxFace(i, face, q, N) || dot(N, gLight.sunDir.xyz) <= 0.0) discard;
        float2 atlas;
        gAlbedo.GetDimensions(atlas.x, atlas.y);
        if (gAlbedo.SampleLevel(gTexSampler, NachoCellUv(i, face, NachoFaceUv(face, q, 0.5 / i.asset.w), atlas), 1.0).a < 0.5) discard;
        return i.pos.z + 0.00025;
    }
    NachoHitInfo hit;
    if (!(i.tint.a > 25.0 ? NachoSolidTrace(i, true, hit) : NachoTrace(i, true, hit))) discard; // tint.a 31: a building
    return max(NachoDepth(i, hit.along, true), i.pos.z) + 0.00025;
}
