// Optima-Earth geometry builder: C++ port of the GB class in fly-optima engine/index.html.
// Collects triangles (position, normal, uv, colour) and emits the engine's Vertex list. Materials are flat colours for now
// (the JS version used textured three.js materials); each builder carries the colour of its material.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "../render/vertex.h"

namespace optima {

using V3 = std::array<double, 3>;
using V2 = std::array<double, 2>;
using RGB = std::array<float, 3>;

inline V3 Sub(const V3& a, const V3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline V3 Cross(const V3& a, const V3& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
inline double Dot(const V3& a, const V3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline V3 Nrm(const V3& a) { const double l = std::sqrt(Dot(a, a)); return l > 0 ? V3{a[0] / l, a[1] / l, a[2] / l} : V3{0, 1, 0}; }

// '#rrggbb' -> linear-ish 0..1 floats (three.js Color(hex) converts sRGB to linear; we do the same so colours match).
inline RGB Hex(const std::string& h) {
    if (h.size() < 7 || h[0] != '#') return {0.8f, 0.8f, 0.8f};
    auto c = [&](int i) {
        const double s = std::stoi(h.substr(i, 2), nullptr, 16) / 255.0;
        return (float)(s <= 0.04045 ? s / 12.92 : std::pow((s + 0.055) / 1.055, 2.4));
    };
    return {c(1), c(3), c(5)};
}

struct GB {
    RGB color{0.8f, 0.8f, 0.8f}; // material colour; per-triangle colours multiply it (as three.js vertexColors would)
    std::vector<tbd::Vertex> v;

    explicit GB(RGB c = {0.8f, 0.8f, 0.8f}) : color(c) {}

    void Tri(V3 a, V3 b, V3 c, const V3& n, V2 ua = {0, 0}, V2 ub = {1, 0}, V2 uc = {0, 1}, const RGB* col = nullptr) {
        if (Dot(Cross(Sub(b, a), Sub(c, a)), n) < 0) { std::swap(b, c); std::swap(ub, uc); }
        const RGB k = col ? RGB{(*col)[0], (*col)[1], (*col)[2]} : color;
        // The engine winds counter-clockwise seen from the front, the same as three.js.
        for (const auto* p : {&a, &b, &c}) {
            const V2& uv = p == &a ? ua : p == &b ? ub : uc;
            v.push_back({(float)(*p)[0], (float)(*p)[1], (float)(*p)[2], (float)n[0], (float)n[1], (float)n[2], k[0], k[1], k[2], 1.0f,
                         (float)uv[0], (float)uv[1]});
        }
    }
    void Quad(const V3& a, const V3& b, const V3& c, const V3& d, const V3& n,
              const std::array<V2, 4>& uv = {V2{0, 0}, V2{1, 0}, V2{1, 1}, V2{0, 1}}, const RGB* col = nullptr) {
        Tri(a, b, c, n, uv[0], uv[1], uv[2], col);
        Tri(a, c, d, n, uv[0], uv[2], uv[3], col);
    }
    void UpTri(const V3& a, const V3& b, const V3& c, V2 ua, V2 ub, V2 uc) {
        V3 n = Nrm(Cross(Sub(b, a), Sub(c, a)));
        if (n[1] < 0) n = {-n[0], -n[1], -n[2]};
        Tri(a, b, c, n, ua, ub, uc);
    }
    // Two-sided: the JS materials were DoubleSide (roof undersides, siding seen from inside). The engine culls back faces,
    // so the back copy is emitted explicitly where it matters.
    void AppendTo(std::vector<tbd::Vertex>& out, bool doubleSided = false) const {
        out.insert(out.end(), v.begin(), v.end());
        if (!doubleSided) return;
        for (size_t i = 0; i + 2 < v.size(); i += 3) {
            tbd::Vertex a = v[i], b = v[i + 2], c = v[i + 1];
            for (tbd::Vertex* q : {&a, &b, &c}) { q->nx = -q->nx; q->ny = -q->ny; q->nz = -q->nz; }
            out.push_back(a); out.push_back(b); out.push_back(c);
        }
    }
};

// mulberry32, bit for bit the rng() in index.html: same seed, same building on every client.
struct Rng {
    uint32_t s;
    explicit Rng(int64_t seed) : s((uint32_t)(int32_t)seed) {}
    double operator()() {
        s += 0x6D2B79F5u;
        uint32_t t = s;
        t = (t ^ (t >> 15)) * (1u | t);
        t = (t + ((t ^ (t >> 7)) * (61u | t))) ^ t;
        return (double)(t ^ (t >> 14)) / 4294967296.0;
    }
};

} // namespace optima
