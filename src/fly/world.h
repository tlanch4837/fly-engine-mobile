// Fly-engine world: double-precision world positions, a floating origin, and procedural terrain streamed in chunks around the
// aircraft out to tens of kilometres, anywhere on Earth (earth_gen.h). Everything is generated on the client (Optima-Earth rule:
// nothing baked). Chunks are built on worker threads and become meshes on the main thread.
// Axes match Optima: x east, y up, z south; metres, around the current anchor (WorldGen::anchor).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "../floating_origin.h"
#include "../render/renderer.h"
#include "earth_gen.h"
#include "workers.h"

namespace fly {

// A vertex list's box (view culling): lo / hi, both zero when empty.
inline void MeshBox(const std::vector<tbd::Vertex>& v, float lo[3], float hi[3]) {
    lo[0] = lo[1] = lo[2] = 1e30f; hi[0] = hi[1] = hi[2] = -1e30f;
    for (const tbd::Vertex& q : v) {
        lo[0] = std::min(lo[0], q.px); lo[1] = std::min(lo[1], q.py); lo[2] = std::min(lo[2], q.pz);
        hi[0] = std::max(hi[0], q.px); hi[1] = std::max(hi[1], q.py); hi[2] = std::max(hi[2], q.pz);
    }
    if (v.empty()) lo[0] = lo[1] = lo[2] = hi[0] = hi[1] = hi[2] = 0.0f;
}

using tbd::WorldPosition;

// What the flight model and the game ask of the ground. A thin handle on the current WorldGen.
struct HeightField {
    std::shared_ptr<const WorldGen> gen;
    double townHeight = 40;
    double Height(double x, double z) const { return gen ? gen->Height(x, z) : 0.0; }
};

// ---- terrain chunks: a quadtree-free ring LOD. Level L chunks are kBase * 2^L metres wide with the same vertex count,
// level L covers the ring out to kRings chunks around the camera. Skirts hide the cracks between levels.
class Terrain {
public:
    static constexpr int kLevels = 9;     // 500 m .. 128 km chunks: the outer rings reach ~320 km, the horizon from cruise height (7 levels stopped at ~80 km and left the sea plane as a grey band under the horizon)
    static constexpr double kBase = 500;  // level-0 chunk width, metres
    static constexpr int kGrid = 40;      // quads per chunk side
    static constexpr int kRings = 2;      // chunks each side of the camera per level

    HeightField field;

    struct Chunk {
        int level; int64_t cx, cz; tbd::MeshId mesh = tbd::kInvalidMesh; WorldPosition origin; bool used = false;
        float lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0}; // the mesh's box, relative to origin (view culling)
    };

    // A new world: every chunk goes, jobs for the old one are ignored when they land.
    void Reset(tbd::Renderer& r, std::shared_ptr<const WorldGen> gen) {
        for (auto& kv : chunks_) r.RetireMesh(kv.second.mesh);
        chunks_.clear();
        pending_.clear();
        { std::lock_guard<std::mutex> l(m_); done_.clear(); }
        ++generation_;
        field.gen = std::move(gen);
        field.townHeight = field.gen ? field.gen->townHeight : 40;
    }

    // Wants the rings around the eye, queues the missing chunks on the workers (nearest and finest first), turns finished ones
    // into meshes (at most `uploads` a frame) and retires chunks nobody wants once nothing is missing (no holes meanwhile).
    void Update(tbd::Renderer& r, const WorldPosition& eye, Workers& workers, int uploads = 8) {
        if (!field.gen) return;
        // finished jobs -> meshes
        {
            std::vector<Done> ready;
            {
                std::lock_guard<std::mutex> l(m_);
                const size_t n = std::min<size_t>(done_.size(), (size_t)std::max(1, uploads));
                ready.assign(std::make_move_iterator(done_.begin()), std::make_move_iterator(done_.begin() + n));
                done_.erase(done_.begin(), done_.begin() + n);
            }
            for (Done& d : ready) {
                if (d.generation != generation_) continue;
                pending_.erase(d.key);
                Chunk c{d.key.L, d.key.x, d.key.z};
                c.origin = d.origin;
                if (!d.verts.empty()) c.mesh = r.CreateMesh(d.verts.data(), (uint32_t)d.verts.size(), tbd::Topology::Triangles);
                MeshBox(d.verts, c.lo, c.hi);
                if (auto it = chunks_.find(d.key); it != chunks_.end()) r.RetireMesh(it->second.mesh);
                chunks_[d.key] = c;
                if (onChunk && c.mesh != tbd::kInvalidMesh) onChunk(c); // city_shadow.h: the ground under the baked shadows
            }
        }
        for (auto& kv : chunks_) kv.second.used = false;
        missing_ = 0;
        const double aboveGround = std::max(0.0, eye.y - std::max(0.0, field.Height(eye.x, eye.z)));
        // high up, the finest levels are pointless: start the rings at a level whose chunks are wider than the height
        const int first = std::clamp((int)std::floor(std::log2(std::max(1.0, aboveGround / (kBase * 0.75)))), 0, kLevels - 3);
        for (int L = first; L < kLevels; ++L) {
            const double w = kBase * std::pow(2.0, L);
            const int64_t ex = (int64_t)std::floor(eye.x / w), ez = (int64_t)std::floor(eye.z / w);
            for (int64_t dz = -kRings; dz <= kRings; ++dz)
                for (int64_t dx = -kRings; dx <= kRings; ++dx) {
                    const int64_t cx = ex + dx, cz = ez + dz;
                    if (L > first) { // inside the next-finer level's ring: that level draws it
                        const double wf = w / 2;
                        const int64_t fx = (int64_t)std::floor(eye.x / wf), fz = (int64_t)std::floor(eye.z / wf);
                        const bool covered = cx * 2 >= fx - kRings && cx * 2 + 1 <= fx + kRings && cz * 2 >= fz - kRings && cz * 2 + 1 <= fz + kRings;
                        if (covered) continue;
                    }
                    const Key k{L, cx, cz};
                    auto it = chunks_.find(k);
                    if (it != chunks_.end()) { it->second.used = true; continue; }
                    ++missing_;
                    if (pending_.count(k)) continue;
                    pending_.insert(k);
                    const double dist = std::hypot((cx + 0.5) * w - eye.x, (cz + 0.5) * w - eye.z);
                    auto gen = field.gen;
                    const uint64_t generation = generation_;
                    workers.Submit([this, gen, k, generation] {
                        Done d{k, generation, {}, {}};
                        Build(*gen, k, d.verts, d.origin);
                        std::lock_guard<std::mutex> l(m_);
                        done_.push_back(std::move(d));
                    }, L * 1e5 + dist * 0.5);
                }
        }
        // until everything wanted exists, keep drawing what we have (the old coarse or fine chunks fill the gaps)
        if (missing_ == 0)
            for (auto it = chunks_.begin(); it != chunks_.end();) {
                if (!it->second.used) { r.RetireMesh(it->second.mesh); it = chunks_.erase(it); }
                else ++it;
            }
        else
            for (auto& kv : chunks_) kv.second.used = true;
    }
    template <class F> void ForEach(F&& f) const { for (const auto& kv : chunks_) if (kv.second.used && kv.second.mesh != tbd::kInvalidMesh) f(kv.second); }
    std::function<void(const Chunk&)> onChunk; // each chunk turned into a mesh (city_shadow.h)
    size_t Count() const { return chunks_.size(); }
    size_t Missing() const { return missing_; }

private:
    struct Key {
        int L; int64_t x, z;
        bool operator<(const Key& o) const { return L != o.L ? L < o.L : x != o.x ? x < o.x : z < o.z; }
    };
    struct Done { Key key; uint64_t generation; std::vector<tbd::Vertex> verts; WorldPosition origin; };
    std::map<Key, Chunk> chunks_;
    std::set<Key> pending_;
    std::mutex m_;
    std::vector<Done> done_;
    uint64_t generation_ = 0;
    size_t missing_ = 0;

    static void Build(const WorldGen& g, const Key& k, std::vector<tbd::Vertex>& v, WorldPosition& origin) {
        const double w = kBase * std::pow(2.0, k.L), step = w / kGrid;
        origin = {k.x * w, 0, k.z * w};
        const int N = kGrid + 1;
        std::vector<GroundSample> S((N + 2) * (N + 2));
        auto at = [&](int i, int j) -> GroundSample& { return S[(j + 1) * (N + 2) + (i + 1)]; };
        for (int j = -1; j <= N; ++j)
            for (int i = -1; i <= N; ++i) at(i, j) = g.Sample(origin.x + i * step, origin.z + j * step);
        auto vert = [&](int i, int j, double yOff) {
            const GroundSample& s = at(i, j);
            const double h = s.height;
            const double nx = at(i - 1, j).height - at(i + 1, j).height, nz = at(i, j - 1).height - at(i, j + 1).height, ny = 2 * step;
            const double l = std::sqrt(nx * nx + ny * ny + nz * nz);
            float col[3];
            WorldGen::Colour(s, 1 - ny / l, col);
            const float farm = (float)WorldGen::FarmWeight(s, 1 - ny / l);
            // mesh.hlsl MappedGroundPS: 0.08 land, up to 0.10 farmland (FarmField), 0.06 open water (rivers, lakes: the land cover's water)
            const float alpha = s.cover == 10 ? 0.06f : 0.08f + 0.02f * farm;
            return tbd::Vertex{(float)(i * step), (float)(h + yOff), (float)(j * step), (float)(nx / l), (float)(ny / l), (float)(nz / l),
                               col[0], col[1], col[2], alpha, (float)(i * step / 50), (float)(j * step / 50)};
        };
        v.reserve(kGrid * kGrid * 6 + kGrid * 48);
        for (int j = 0; j < kGrid; ++j)
            for (int i = 0; i < kGrid; ++i) {
                const tbd::Vertex a = vert(i, j, 0), b = vert(i + 1, j, 0), cc = vert(i + 1, j + 1, 0), d = vert(i, j + 1, 0);
                v.insert(v.end(), {a, d, cc, a, cc, b}); // counter-clockwise seen from above (+y)
            }
        // skirts: a strip hanging down along each edge, hides cracks against coarser neighbours
        const double drop = step * 1.5;
        auto skirt = [&](int i0, int j0, int i1, int j1) {
            const tbd::Vertex a = vert(i0, j0, 0), b = vert(i1, j1, 0), a2 = vert(i0, j0, -drop), b2 = vert(i1, j1, -drop);
            v.insert(v.end(), {a, a2, b2, a, b2, b, a, b2, a2, a, b, b2}); // both windings
        };
        for (int q = 0; q < kGrid; ++q) { skirt(q, 0, q + 1, 0); skirt(q, kGrid, q + 1, kGrid); skirt(0, q, 0, q + 1); skirt(kGrid, q, kGrid, q + 1); }
    }
};

// Model matrix for something stored relative to `local` (double), drawn with the floating origin at `origin`.
inline tbd::Mat4 Relative(const WorldPosition& local, const WorldPosition& origin) {
    tbd::Mat4 m = tbd::Mat4::Identity();
    m.At(0, 3) = (float)(local.x - origin.x);
    m.At(1, 3) = (float)(local.y - origin.y);
    m.At(2, 3) = (float)(local.z - origin.z);
    return m;
}

} // namespace fly
