// "Nacho" impostor cards, after Thomas Murphy's "Imposter cards - nacho time" (BCON26); the three.js reference this follows is
// fly-optima engine/lib/trees/nacho.js. Portable: no OS headers, no graphics API (the drawing is Renderer::DrawNachoCards and
// shaders/nacho.hlsli).
//   Bake      each asset (a vertex list, as procmesh.h / scene.h build them) is rasterized on the CPU from six sides (+X -X +Z -Z +Y -Y),
//             orthographic, into ONE shared atlas pair. Cells are shelf-packed and sized to the asset (big assets big cells), at most
//             maxCell texels a face. Albedo: premultiplied display-space colour + coverage. Normal: object-space normal * 0.5 + 0.5, and
//             in alpha the surface's offset along the face direction (+1 toward the viewer, 0 the slice through the centre) * 0.5 + 0.5.
//             Being CPU work it needs no backend support and could be cached to disk; ~0.3 s for a 13k-triangle tree at 512 px.
//   Instance  12 floats (NachoInstance), so every backend's existing instance stream carries it: position, scale, yaw, tint, the asset's
//             centre and half size, and its atlas cell. All assets share the atlas, so a whole forest is one instanced draw.
//   Fade      FadeAt gives a card's dither keep at a distance; the real mesh draws with SetDrawFade(-keep) (the exact complement), and
//             not at all once keep reaches 1.
#pragma once
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "renderer.h"

namespace tbd::nacho {

struct Asset {
    float cx = 0, cy = 0, cz = 0, h = 1; // the slice box: centre and half size, in the asset's own units
    uint32_t cellX = 0, cellY = 0, cell = 0; // its strip of six cell x cell faces, top-left texel (multiples of 16)
};

struct Atlas {
    int width = 0, height = 0, maxCell = 0;
    std::vector<uint8_t> albedo, normal; // width * height * 4, top row first (CreateTexture's layout)
    std::vector<Asset> assets;
    TextureId albedoTex = kNoTexture, normalTex = kNoTexture;
};

namespace detail {
struct Face { int axis; float sign; };
// The six views in the shader's order: the face's direction D (toward its camera), and how a point q (relative to the centre) maps to
// the face's (u, v) in 0..1 and its offset along D. Must match NachoTrace's slice uv in nacho.hlsli.
inline void FaceUV(int f, float qx, float qy, float qz, float inv2h, float& u, float& v, float& d) {
    switch (f) {
    case 0: u = -qz * inv2h + 0.5f; v = qy * inv2h + 0.5f; d = qx; break;  // +X
    case 1: u = qz * inv2h + 0.5f; v = qy * inv2h + 0.5f; d = -qx; break;  // -X
    case 2: u = qx * inv2h + 0.5f; v = qy * inv2h + 0.5f; d = qz; break;   // +Z
    case 3: u = -qx * inv2h + 0.5f; v = qy * inv2h + 0.5f; d = -qz; break; // -Z
    case 4: u = qx * inv2h + 0.5f; v = -qz * inv2h + 0.5f; d = qy; break;  // +Y
    default: u = qx * inv2h + 0.5f; v = qz * inv2h + 0.5f; d = -qy; break; // -Y
    }
}
inline Vec3 FaceDir(int f) {
    static const Vec3 k[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}};
    return k[f];
}

// The shelf packer ("hungry packer"): strips of six square cells, cell size proportional to the asset's size, the biggest first.
// Shrinks the largest cell until the atlas fits in maxTexture. Cell sizes and positions are multiples of 16 texels (the instance packs
// the origin in 16-texel units, and the first four mips stay inside a cell).
inline bool Pack(const std::vector<float>& hs, int maxCell, int maxTexture, Atlas& out) {
    if (hs.empty()) return false;
    const float hmax = *std::max_element(hs.begin(), hs.end());
    for (int mc = maxCell; mc >= 32; mc = (int)(mc * 0.85f) / 16 * 16) {
        const int cols = std::max(1, (int)std::ceil(std::sqrt(hs.size() / 2.0)));
        const int W = std::min(maxTexture / 96 * 96, mc * 6 * cols);
        std::vector<int> order(hs.size());
        for (size_t i = 0; i < order.size(); ++i) order[i] = (int)i;
        std::sort(order.begin(), order.end(), [&](int a, int b) { return hs[a] > hs[b]; });
        int x = 0, y = 0, shelf = 0;
        bool fits = true;
        out.assets.assign(hs.size(), Asset{});
        for (int i : order) {
            const int cs = std::max(32, (int)std::lround(mc * hs[i] / hmax / 16.0f) * 16);
            if (cs * 6 > W) { fits = false; break; }
            if (x + cs * 6 > W) { y += shelf; x = 0; shelf = 0; }
            if (!shelf) shelf = cs;
            out.assets[i].cellX = (uint32_t)x;
            out.assets[i].cellY = (uint32_t)y;
            out.assets[i].cell = (uint32_t)cs;
            x += cs * 6;
        }
        if (fits && y + shelf <= maxTexture) {
            out.width = W;
            out.height = (y + shelf + 15) / 16 * 16;
            out.maxCell = mc;
            return true;
        }
    }
    return false;
}

// One face of one asset, supersampled `ss` times, into the atlas at its cell.
inline void BakeFace(const std::vector<Vertex>& mesh, const Asset& a, int f, Atlas& at) {
    const int ss = 2, cs = (int)a.cell, n = cs * ss;
    std::vector<float> zbuf((size_t)n * n, -1e30f);
    std::vector<float> col((size_t)n * n * 3, 0.0f), nrm((size_t)n * n * 3, 0.0f);
    const float inv2h = 0.5f / a.h;
    const Vec3 D = FaceDir(f);
    struct P { float x, y, d; };
    for (size_t t = 0; t + 2 < mesh.size(); t += 3) {
        const Vertex* v[3] = {&mesh[t], &mesh[t + 1], &mesh[t + 2]};
        P p[3];
        for (int k = 0; k < 3; ++k) {
            float u, vv, d;
            FaceUV(f, v[k]->px - a.cx, v[k]->py - a.cy, v[k]->pz - a.cz, inv2h, u, vv, d);
            p[k] = {u * n, (1.0f - vv) * n, d / a.h};
        }
        const float area = (p[1].x - p[0].x) * (p[2].y - p[0].y) - (p[2].x - p[0].x) * (p[1].y - p[0].y);
        if (std::fabs(area) < 1e-9f) continue;
        // double sided: the normal faces this view's camera (as a back face's flipped normal would)
        const Vec3 e1{v[1]->px - v[0]->px, v[1]->py - v[0]->py, v[1]->pz - v[0]->pz}, e2{v[2]->px - v[0]->px, v[2]->py - v[0]->py, v[2]->pz - v[0]->pz};
        Vec3 vn0{v[0]->nx, v[0]->ny, v[0]->nz};
        const Vec3 geo = Cross(e1, e2);
        float flip = Dot(geo, D) < 0.0f ? -1.0f : 1.0f;
        if (Dot(geo, vn0) < 0.0f) flip = -flip; // the triangle's winding disagrees with its normals: trust the normals' side
        (void)vn0;
        const int x0 = std::max(0, (int)std::floor(std::min({p[0].x, p[1].x, p[2].x}))), x1 = std::min(n - 1, (int)std::ceil(std::max({p[0].x, p[1].x, p[2].x})));
        const int y0 = std::max(0, (int)std::floor(std::min({p[0].y, p[1].y, p[2].y}))), y1 = std::min(n - 1, (int)std::ceil(std::max({p[0].y, p[1].y, p[2].y})));
        const float ia = 1.0f / area;
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                const float px = x + 0.5f, py = y + 0.5f;
                const float w0 = ((p[1].x - px) * (p[2].y - py) - (p[2].x - px) * (p[1].y - py)) * ia;
                const float w1 = ((p[2].x - px) * (p[0].y - py) - (p[0].x - px) * (p[2].y - py)) * ia;
                const float w2 = 1.0f - w0 - w1;
                if (w0 < 0 || w1 < 0 || w2 < 0) continue;
                const float d = w0 * p[0].d + w1 * p[1].d + w2 * p[2].d;
                const size_t i = (size_t)y * n + x;
                if (d <= zbuf[i]) continue; // nearest to this face's camera wins
                zbuf[i] = d;
                for (int c = 0; c < 3; ++c) {
                    const float* a0 = &v[0]->r; const float* a1 = &v[1]->r; const float* a2 = &v[2]->r;
                    col[i * 3 + c] = w0 * a0[c] + w1 * a1[c] + w2 * a2[c];
                    const float* n0 = &v[0]->nx; const float* n1 = &v[1]->nx; const float* n2 = &v[2]->nx;
                    nrm[i * 3 + c] = (w0 * n0[c] + w1 * n1[c] + w2 * n2[c]) * flip;
                }
            }
    }
    // down to the cell: coverage-weighted colour (stored premultiplied), normal and offset
    const int ox = (int)a.cellX + f * cs, oy = (int)a.cellY;
    std::vector<float> cov((size_t)cs * cs), nn((size_t)cs * cs * 4);
    for (int y = 0; y < cs; ++y)
        for (int x = 0; x < cs; ++x) {
            float c[3] = {}, nv[3] = {}, d = 0, w = 0;
            for (int sy = 0; sy < ss; ++sy)
                for (int sx = 0; sx < ss; ++sx) {
                    const size_t i = (size_t)(y * ss + sy) * n + (x * ss + sx);
                    if (zbuf[i] < -1e29f) continue;
                    w += 1;
                    d += zbuf[i];
                    for (int k = 0; k < 3; ++k) { c[k] += col[i * 3 + k]; nv[k] += nrm[i * 3 + k]; }
                }
            const float alpha = w / (ss * ss);
            const size_t o = ((size_t)(oy + y) * at.width + ox + x) * 4, l = (size_t)y * cs + x;
            for (int k = 0; k < 3; ++k) at.albedo[o + k] = (uint8_t)std::lround(std::clamp(w > 0 ? c[k] / w * alpha : 0.0f, 0.0f, 1.0f) * 255.0f);
            at.albedo[o + 3] = (uint8_t)std::lround(alpha * 255.0f);
            cov[l] = alpha;
            if (w > 0) {
                Vec3 nm = Normalize(Vec3{nv[0], nv[1], nv[2]});
                nn[l * 4 + 0] = nm.x; nn[l * 4 + 1] = nm.y; nn[l * 4 + 2] = nm.z; nn[l * 4 + 3] = std::clamp(d / w, -1.0f, 1.0f);
            }
        }
    // dilate the normal and offset into the empty texels around the silhouette, so filtering at the edge does not pull in garbage
    for (int pass = 0; pass < 6; ++pass) {
        std::vector<float> next = cov;
        for (int y = 0; y < cs; ++y)
            for (int x = 0; x < cs; ++x) {
                const size_t l = (size_t)y * cs + x;
                if (cov[l] > 0) continue;
                float s[4] = {}, w = 0;
                for (int k = 0; k < 4; ++k) {
                    const int xx = x + (k == 0) - (k == 1), yy = y + (k == 2) - (k == 3);
                    if (xx < 0 || yy < 0 || xx >= cs || yy >= cs) continue;
                    const size_t m = (size_t)yy * cs + xx;
                    if (cov[m] <= 0) continue;
                    for (int c = 0; c < 4; ++c) s[c] += nn[m * 4 + c];
                    w += 1;
                }
                if (w > 0) { for (int c = 0; c < 4; ++c) nn[l * 4 + c] = s[c] / w; next[l] = 1e-6f; }
            }
        cov.swap(next);
    }
    for (int y = 0; y < cs; ++y)
        for (int x = 0; x < cs; ++x) {
            const size_t o = ((size_t)(oy + y) * at.width + ox + x) * 4, l = (size_t)y * cs + x;
            Vec3 nm{nn[l * 4], nn[l * 4 + 1], nn[l * 4 + 2]};
            float d = nn[l * 4 + 3];
            if (cov[l] <= 0) { nm = D; d = 0.0f; }
            nm = Normalize(nm);
            at.normal[o + 0] = (uint8_t)std::lround(std::clamp(nm.x * 0.5f + 0.5f, 0.0f, 1.0f) * 255.0f);
            at.normal[o + 1] = (uint8_t)std::lround(std::clamp(nm.y * 0.5f + 0.5f, 0.0f, 1.0f) * 255.0f);
            at.normal[o + 2] = (uint8_t)std::lround(std::clamp(nm.z * 0.5f + 0.5f, 0.0f, 1.0f) * 255.0f);
            at.normal[o + 3] = (uint8_t)std::lround(std::clamp(d * 0.5f + 0.5f, 0.0f, 1.0f) * 255.0f);
        }
}
} // namespace detail

// Bakes every mesh (triangle lists in their own units, any origin) into one atlas. maxCell: the largest asset's face size in texels
// (the reference allows up to 2048; a phone or a Quest wants 256-512: an atlas of 1 tree at 512 is 3072 x 512, 12 MB with mips for the
// pair). maxTexture: the GPU's limit. False when nothing fits.
inline bool Bake(const std::vector<const std::vector<Vertex>*>& meshes, Atlas& out, int maxCell = 512, int maxTexture = 16384) {
    maxCell = std::clamp(maxCell / 16 * 16, 32, 2048);
    while (maxCell > 32 && maxCell * 6 > maxTexture) maxCell /= 2;
    std::vector<float> hs;
    std::vector<Asset> boxes;
    for (const std::vector<Vertex>* m : meshes) {
        Vec3 lo{1e30f, 1e30f, 1e30f}, hi{-1e30f, -1e30f, -1e30f};
        for (const Vertex& v : *m) {
            lo = {std::min(lo.x, v.px), std::min(lo.y, v.py), std::min(lo.z, v.pz)};
            hi = {std::max(hi.x, v.px), std::max(hi.y, v.py), std::max(hi.z, v.pz)};
        }
        if (m->empty()) lo = hi = {0, 0, 0};
        Asset a;
        a.cx = (lo.x + hi.x) * 0.5f; a.cy = (lo.y + hi.y) * 0.5f; a.cz = (lo.z + hi.z) * 0.5f;
        a.h = std::max(std::max(hi.x - lo.x, hi.y - lo.y), std::max(hi.z - lo.z, 1e-4f)) * 0.5f * 1.02f;
        boxes.push_back(a);
        hs.push_back(a.h);
    }
    if (!detail::Pack(hs, maxCell, maxTexture, out)) return false;
    for (size_t i = 0; i < boxes.size(); ++i) {
        out.assets[i].cx = boxes[i].cx; out.assets[i].cy = boxes[i].cy; out.assets[i].cz = boxes[i].cz; out.assets[i].h = boxes[i].h;
    }
    out.albedo.assign((size_t)out.width * out.height * 4, 0);
    out.normal.assign((size_t)out.width * out.height * 4, 0);
    // every face of every asset is its own job (they write disjoint cells); a few threads, as each holds ~28 MB of
    // supersampled buffers at a 512 px cell
    const int jobs = (int)meshes.size() * 6;
    const int threads = std::clamp((int)std::thread::hardware_concurrency(), 1, std::min(6, jobs));
    std::atomic<int> next{0};
    auto work = [&] { for (int j; (j = next.fetch_add(1)) < jobs;) detail::BakeFace(*meshes[j / 6], out.assets[j / 6], j % 6, out); };
    std::vector<std::thread> pool;
    for (int t = 1; t < threads; ++t) pool.emplace_back(work);
    work();
    for (std::thread& t : pool) t.join();
    return true;
}

// The baked atlas (CPU copies and boxes) to a file and back, so the bake runs once per machine. The caller names the file by
// what went into the bake (it changes when the meshes do). False when the file is missing, short or not an atlas.
inline bool Save(const Atlas& at, const std::string& file) {
    if (at.albedo.empty() || file.empty()) return false;
    FILE* f = std::fopen(file.c_str(), "wb");
    if (!f) return false;
    const uint32_t head[5] = {0x4F48434Eu /*NCHO*/, (uint32_t)at.width, (uint32_t)at.height, (uint32_t)at.maxCell, (uint32_t)at.assets.size()};
    bool ok = std::fwrite(head, sizeof head, 1, f) == 1 && std::fwrite(at.assets.data(), sizeof(Asset), at.assets.size(), f) == at.assets.size() &&
              std::fwrite(at.albedo.data(), 1, at.albedo.size(), f) == at.albedo.size() && std::fwrite(at.normal.data(), 1, at.normal.size(), f) == at.normal.size();
    ok = std::fclose(f) == 0 && ok;
    if (!ok) std::remove(file.c_str());
    return ok;
}
inline bool Load(Atlas& at, const std::string& file) {
    FILE* f = file.empty() ? nullptr : std::fopen(file.c_str(), "rb");
    if (!f) return false;
    uint32_t head[5] = {};
    bool ok = std::fread(head, sizeof head, 1, f) == 1 && head[0] == 0x4F48434Eu && head[1] && head[2] && head[1] <= 16384 && head[2] <= 16384 && head[4] && head[4] < 4096;
    if (ok) {
        at.width = (int)head[1]; at.height = (int)head[2]; at.maxCell = (int)head[3];
        at.assets.resize(head[4]);
        at.albedo.resize((size_t)at.width * at.height * 4);
        at.normal.resize(at.albedo.size());
        ok = std::fread(at.assets.data(), sizeof(Asset), at.assets.size(), f) == at.assets.size() &&
             std::fread(at.albedo.data(), 1, at.albedo.size(), f) == at.albedo.size() && std::fread(at.normal.data(), 1, at.normal.size(), f) == at.normal.size();
    }
    std::fclose(f);
    if (!ok) at = Atlas{};
    return ok;
}

// The atlas pair as textures (mips generated). Outside BeginFrame..EndFrame. Keeps the CPU copies unless `freeCpu`.
inline bool Upload(Renderer& r, Atlas& at, bool freeCpu = true) {
    if (at.width <= 0) return false;
    at.albedoTex = r.CreateTexture(at.albedo.data(), at.width, at.height);
    at.normalTex = r.CreateTexture(at.normal.data(), at.width, at.height);
    if (freeCpu) { std::vector<uint8_t>().swap(at.albedo); std::vector<uint8_t>().swap(at.normal); }
    return at.albedoTex != kNoTexture && at.normalTex != kNoTexture;
}

// The card's quad: four corners at (+-1, +-1), two triangles. The vertex shader turns it to face the viewer.
inline MeshId CreateQuad(Renderer& r) {
    const Vertex q[6] = {{-1, -1, 0, 0, 0, 1, 1, 1, 1, 1}, {1, -1, 0, 0, 0, 1, 1, 1, 1, 1}, {1, 1, 0, 0, 0, 1, 1, 1, 1, 1},
                         {-1, -1, 0, 0, 0, 1, 1, 1, 1, 1}, {1, 1, 0, 0, 0, 1, 1, 1, 1, 1}, {-1, 1, 0, 0, 0, 1, 1, 1, 1, 1}};
    return r.CreateMesh(q, 6, Topology::Triangles);
}

// One instance: `pos` is where the asset's origin goes (world, or floating-origin local), turned by `yaw` about +y and scaled.
// The yaw turns x toward -z (x' = cos*x + sin*z, z' = -sin*x + cos*z), as YawScaleMatrix does for the real mesh.
struct NachoInstance { float rows[12]; };
inline NachoInstance Pack(const Atlas& at, uint32_t asset, Vec3 pos, float scale, float yaw, Vec3 tint = {1, 1, 1}) {
    const Asset& a = at.assets[asset];
    const auto b8 = [](float v) { return (float)std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f); };
    NachoInstance o;
    o.rows[0] = pos.x; o.rows[1] = pos.y; o.rows[2] = pos.z; o.rows[3] = scale;
    o.rows[4] = yaw; o.rows[5] = b8(tint.x) + b8(tint.y) * 256.0f + b8(tint.z) * 65536.0f; o.rows[6] = a.cx; o.rows[7] = a.cz;
    o.rows[8] = a.cy; o.rows[9] = a.h; o.rows[10] = (float)(a.cellX / 16 + (a.cellY / 16) * 1024); o.rows[11] = (float)a.cell;
    return o;
}
// The real mesh placed like a card: translate * yaw * scale.
inline Mat4 YawScaleMatrix(Vec3 pos, float scale, float yaw) {
    Mat4 m = Mat4::Identity();
    const float c = std::cos(yaw), s = std::sin(yaw);
    m.At(0, 0) = c * scale; m.At(0, 2) = s * scale;
    m.At(1, 1) = scale;
    m.At(2, 0) = -s * scale; m.At(2, 2) = c * scale;
    m.At(0, 3) = pos.x; m.At(1, 3) = pos.y; m.At(2, 3) = pos.z;
    return m;
}
// Where an instance's slice box is centred (the card's fade measures from here, as NachoVertex does).
inline Vec3 Center(const Asset& a, Vec3 pos, float scale, float yaw) {
    const float c = std::cos(yaw), s = std::sin(yaw);
    return pos + Vec3{c * a.cx + s * a.cz, a.cy, -s * a.cx + c * a.cz} * scale;
}
// The card's dither keep at distance d for fade [nearM, farM]: 0 = only the mesh, 1 = only the card. The mesh draws with
// SetDrawFade(-keep) while keep < 1 (SetDrawFade(-0) = 0 = no fade).
inline float FadeAt(float d, float nearM, float farM) {
    if (farM <= nearM) return 1.0f;
    const float t = std::clamp((d - nearM) / (farM - nearM), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace tbd::nacho
