// C++ port of fly-optima engine/lib/gen/residential/kit.js (house kit: frame, rect cover, roofs, walls, openings).
// Formulas are kept 1:1 with the JS so a footprint gives the same house in both engines.
#pragma once
#include <algorithm>
#include <functional>
#include <optional>
#include <string>

#include "gb.h"

namespace optima {

using Pts = std::vector<V2>; // footprint, local metres: [0] x east, [1] z south

// The facts one structure carries in the tiles (see lib/gen/README.md).
struct Structure {
    Pts pts;
    std::string cat, cls, name, style, roof, wall, brickCol;
    double height = 0, floors = 0, area = 0, base = 0;
    int64_t seed = 1;
    bool brick = false;
    std::optional<V2> front;
};

using HeightFn = std::function<double(double x, double z)>;

struct Frame {
    double cx, cz, ux, uz, vx, vz, A, L, W;
    V2 ToW(double u, double v) const { return {cx + u * ux + v * vx, cz + u * uz + v * vz}; }
    V2 ToL(double x, double z) const { return {(x - cx) * ux + (z - cz) * uz, (x - cx) * vx + (z - cz) * vz}; }
};

// minimum-area oriented box: centre, unit axes u (long) and v
inline Frame MakeFrame(const Pts& pts) {
    struct Best { double A, ux, uz, u0, u1, v0, v1; };
    std::optional<Best> best;
    for (size_t i = 0; i < pts.size(); ++i) {
        const V2 &a = pts[i], &b = pts[(i + 1) % pts.size()];
        const double L = std::hypot(b[0] - a[0], b[1] - a[1]);
        if (L < .5) continue;
        const double ux = (b[0] - a[0]) / L, uz = (b[1] - a[1]) / L;
        double u0 = 1e9, u1 = -1e9, v0 = 1e9, v1 = -1e9;
        for (const V2& p : pts) {
            const double u = p[0] * ux + p[1] * uz, v = -p[0] * uz + p[1] * ux;
            u0 = std::min(u0, u); u1 = std::max(u1, u); v0 = std::min(v0, v); v1 = std::max(v1, v);
        }
        const double A = (u1 - u0) * (v1 - v0);
        if (!best || A < best->A) best = Best{A, ux, uz, u0, u1, v0, v1};
    }
    if (!best) best = Best{0, 1, 0, 0, 0, 0, 0};
    double ux = best->ux, uz = best->uz, u0 = best->u0, u1 = best->u1, v0 = best->v0, v1 = best->v1;
    if (v1 - v0 > u1 - u0) { // u = long axis
        const double nux = -uz, nuz = ux; ux = nux; uz = nuz;
        const double a = v0, b = v1, c = -u1, d = -u0; u0 = a; u1 = b; v0 = c; v1 = d;
    }
    const double cu = (u0 + u1) / 2, cv = (v0 + v1) / 2;
    return {cu * ux - cv * uz, cu * uz + cv * ux, ux, uz, -uz, ux, best->A, u1 - u0, v1 - v0};
}

inline bool InPoly(double x, double z, const Pts& P) {
    bool c = false;
    for (size_t i = 0, j = P.size() - 1; i < P.size(); j = i++) {
        const V2 &a = P[i], &b = P[j];
        if ((a[1] > z) != (b[1] > z) && x < (b[0] - a[0]) * (z - a[1]) / (b[1] - a[1]) + a[0]) c = !c;
    }
    return c;
}
inline double PolyArea(const Pts& P) {
    double a = 0;
    for (size_t i = 0; i < P.size(); ++i) { const V2 &p = P[i], &q = P[(i + 1) % P.size()]; a += p[0] * q[1] - q[0] * p[1]; }
    return a / 2;
}

struct Rect {
    double u0, u1, v0, v1;
    bool hasRidge = false;
    V3 ridge[2]{};
    double rise = 0;
    bool alongU = true;
};

// cover the footprint (in frame coords) with up to `max` disjoint axis-aligned rectangles, largest first
inline std::vector<Rect> Rects(const Pts& pts, const Frame& F, int max = 4) {
    Pts L;
    for (const V2& p : pts) L.push_back(F.ToL(p[0], p[1]));
    auto cl = [](std::vector<double> vs) {
        std::sort(vs.begin(), vs.end());
        std::vector<double> o;
        for (double v : vs) if (o.empty() || v - o.back() > .9) o.push_back(v);
        return o;
    };
    std::vector<double> xu, xv;
    for (const V2& p : L) { xu.push_back(p[0]); xv.push_back(p[1]); }
    const std::vector<double> us = cl(xu), vs = cl(xv);
    const int nu = (int)us.size() - 1, nv = (int)vs.size() - 1;
    std::vector<Rect> out;
    if (nu < 1 || nv < 1) return out;
    std::vector<std::vector<int>> cell(nu, std::vector<int>(nv));
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j) cell[i][j] = InPoly((us[i] + us[i + 1]) / 2, (vs[j] + vs[j + 1]) / 2, L) ? 1 : 0;
    for (int k = 0; k < max; ++k) {
        struct B { double a; int i0, i1, j0, j1; };
        std::optional<B> best;
        for (int i0 = 0; i0 < nu; ++i0)
            for (int j0 = 0; j0 < nv; ++j0) {
                if (cell[i0][j0] != 1) continue;
                for (int i1 = i0; i1 < nu && cell[i1][j0] == 1; ++i1)
                    for (int j1 = j0; j1 < nv; ++j1) {
                        bool ok = true;
                        for (int i = i0; i <= i1 && ok; ++i) if (cell[i][j1] != 1) ok = false;
                        if (!ok) break;
                        const double w = us[i1 + 1] - us[i0], h = vs[j1 + 1] - vs[j0], a = w * h;
                        if (std::min(w, h) >= 2.6 && (!best || a > best->a)) best = B{a, i0, i1, j0, j1};
                    }
            }
        if (!best || (!out.empty() && best->a < 12)) break;
        for (int i = best->i0; i <= best->i1; ++i)
            for (int j = best->j0; j <= best->j1; ++j) cell[i][j] = 2;
        out.push_back({us[best->i0], us[best->i1 + 1], vs[best->j0], vs[best->j1 + 1]});
    }
    return out;
}

struct RoofOpts { double eave, pitch; bool hip; double over = .35; GB* roofGB; GB* wallGB; };

// roofs over the rects; returns ridge top height
inline double Roofs(std::vector<Rect>& R, const Frame& F, const RoofOpts& o) {
    const double eave = o.eave, pitch = o.pitch, over = o.over;
    double top = eave;
    auto inR = [](double u, double v, const Rect& r) { return u > r.u0 - .05 && u < r.u1 + .05 && v > r.v0 - .05 && v < r.v1 + .05; };
    auto P = [&](double u, double v, double y) { const V2 w = F.ToW(u, v); return V3{w[0], y, w[1]}; };
    double maxRise = 1e9;
    for (size_t k = 0; k < R.size(); ++k) {
        Rect& r0 = R[k];
        const bool alongU = (r0.u1 - r0.u0) >= (r0.v1 - r0.v0);
        const double a0 = alongU ? r0.u0 : r0.v0, a1 = alongU ? r0.u1 : r0.v1, b0 = alongU ? r0.v0 : r0.u0, b1 = alongU ? r0.v1 : r0.u1;
        auto Q = [&](double a, double b, double y) { return alongU ? P(a, b, y) : P(b, a, y); };
        const double w = (b1 - b0) / 2, bm = (b0 + b1) / 2;
        const double rise = std::min(w * pitch, maxRise), pt = rise / w;
        if (k == 0) maxRise = rise * .97;
        auto touch = [&](double a) {
            for (size_t q = 0; q < k; ++q) if (alongU ? inR(a, bm, R[q]) : inR(bm, a, R[q])) return true;
            return false;
        };
        const bool e0 = touch(a0 - .4), e1 = touch(a1 + .4);
        const double A0 = e0 ? a0 - w : a0 - over, A1 = e1 ? a1 + w : a1 + over;
        const double ye = eave - over * pt, yr = eave + rise, B0 = b0 - over, B1 = b1 + over;
        const bool hip0 = o.hip && !e0, hip1 = o.hip && !e1;
        double r0a = hip0 ? a0 + w : A0, r1a = hip1 ? a1 - w : A1;
        if (r0a > r1a) r0a = r1a = (a0 + a1) / 2;
        GB& g = *o.roofGB;
        const double s = 1.0 / 3, sl = std::hypot(w + over, rise + over * pt) * s;
        const double ea0 = hip0 ? a0 - over : A0, ea1 = hip1 ? a1 + over : A1;
        g.UpTri(Q(ea0, B0, ye), Q(ea1, B0, ye), Q(r1a, bm, yr), {ea0 * s, 0}, {ea1 * s, 0}, {r1a * s, sl});
        g.UpTri(Q(ea0, B0, ye), Q(r1a, bm, yr), Q(r0a, bm, yr), {ea0 * s, 0}, {r1a * s, sl}, {r0a * s, sl});
        g.UpTri(Q(ea1, B1, ye), Q(ea0, B1, ye), Q(r0a, bm, yr), {-ea1 * s, 0}, {-ea0 * s, 0}, {-r0a * s, sl});
        g.UpTri(Q(ea1, B1, ye), Q(r0a, bm, yr), Q(r1a, bm, yr), {-ea1 * s, 0}, {-r0a * s, sl}, {-r1a * s, sl});
        if (hip0) g.UpTri(Q(ea0, B1, ye), Q(ea0, B0, ye), Q(r0a, bm, yr), {B1 * s, 0}, {B0 * s, 0}, {bm * s, sl});
        if (hip1) g.UpTri(Q(ea1, B0, ye), Q(ea1, B1, ye), Q(r1a, bm, yr), {B0 * s, 0}, {B1 * s, 0}, {bm * s, sl});
        // gable-end wall triangles
        const struct { bool is; double a; double sg; } ends[2] = {{!hip0 && !e0, a0, -1}, {!hip1 && !e1, a1, 1}};
        for (const auto& en : ends) {
            if (!en.is) continue;
            const V3 p1 = Q(en.a, b0, eave), p2 = Q(en.a, b1, eave), p3 = Q(en.a, bm, yr);
            const V3 n = alongU ? V3{F.ux * en.sg, 0, F.uz * en.sg} : V3{F.vx * en.sg, 0, F.vz * en.sg};
            o.wallGB->Tri(p1, p2, p3, n, {b0 * s, eave * s}, {b1 * s, eave * s}, {bm * s, yr * s});
        }
        r0.ridge[0] = Q(r0a, bm, yr); r0.ridge[1] = Q(r1a, bm, yr); r0.hasRidge = true; r0.rise = rise; r0.alongU = alongU;
        top = std::max(top, yr);
    }
    return top;
}

// Ear-clipping triangulation of a simple polygon (stands in for THREE.ShapeGeometry / earcut).
inline std::vector<int> Triangulate(const Pts& P) {
    std::vector<int> idx(P.size()), out;
    for (size_t i = 0; i < P.size(); ++i) idx[i] = (int)i;
    if (PolyArea(P) < 0) std::reverse(idx.begin(), idx.end());
    auto cross = [&](int a, int b, int c) { return (P[b][0] - P[a][0]) * (P[c][1] - P[a][1]) - (P[b][1] - P[a][1]) * (P[c][0] - P[a][0]); };
    auto inside = [&](int p, int a, int b, int c) {
        return cross(a, b, p) >= 0 && cross(b, c, p) >= 0 && cross(c, a, p) >= 0;
    };
    size_t guard = 0;
    while (idx.size() > 3 && guard++ < P.size() * P.size()) {
        bool clipped = false;
        for (size_t i = 0; i < idx.size(); ++i) {
            const int a = idx[(i + idx.size() - 1) % idx.size()], b = idx[i], c = idx[(i + 1) % idx.size()];
            if (cross(a, b, c) <= 0) continue;
            bool ear = true;
            for (int q : idx) if (q != a && q != b && q != c && inside(q, a, b, c)) { ear = false; break; }
            if (!ear) continue;
            out.insert(out.end(), {a, b, c});
            idx.erase(idx.begin() + i);
            clipped = true;
            break;
        }
        if (!clipped) break;
    }
    if (idx.size() == 3) out.insert(out.end(), {idx[0], idx[1], idx[2]});
    return out;
}

// flat cap at height y over the footprint, facing up
inline void Cap(const Pts& pts, double y, GB& g) {
    const std::vector<int> t = Triangulate(pts);
    for (size_t i = 0; i + 2 < t.size(); i += 3) {
        const V2 &a = pts[t[i]], &b = pts[t[i + 1]], &c = pts[t[i + 2]];
        g.Tri({a[0], y, a[1]}, {b[0], y, b[1]}, {c[0], y, c[1]}, {0, 1, 0}, {a[0] / 3, a[1] / 3}, {b[0] / 3, b[1] / 3}, {c[0] / 3, c[1] / 3});
    }
}

struct Edge { size_t i; V2 a, b; double L; V2 d, n; };

// edges of the footprint with outward normals
inline std::vector<Edge> Edges(const Pts& pts) {
    std::vector<Edge> E;
    double cx = 0, cz = 0;
    for (const V2& p : pts) { cx += p[0] / pts.size(); cz += p[1] / pts.size(); }
    double sg = PolyArea(pts) > 0 ? 1 : -1;
    {
        double acc = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            const V2 &a = pts[i], &b = pts[(i + 1) % pts.size()];
            double L = std::hypot(b[0] - a[0], b[1] - a[1]); if (L == 0) L = 1;
            acc += ((b[1] - a[1]) / L * -sg * ((a[0] + b[0]) / 2 - cx) - (b[0] - a[0]) / L * -sg * ((a[1] + b[1]) / 2 - cz)) * L;
        }
        if (acc < 0) sg = -sg;
    }
    for (size_t i = 0; i < pts.size(); ++i) {
        const V2 &a = pts[i], &b = pts[(i + 1) % pts.size()];
        const double L = std::hypot(b[0] - a[0], b[1] - a[1]);
        if (L < .05) continue;
        E.push_back({i, a, b, L, {(b[0] - a[0]) / L, (b[1] - a[1]) / L}, {(b[1] - a[1]) / L * -sg, -(b[0] - a[0]) / L * -sg}});
    }
    return E;
}

// a rectangle on wall edge e: centred at t along the edge, bottom y, size w x h, pushed out by off; optional colour
inline void OnWall(GB& g, const Edge& e, double t, double y, double w, double h, double off, const RGB* col = nullptr) {
    auto p = [&](double s) { return V2{e.a[0] + e.d[0] * s + e.n[0] * off, e.a[1] + e.d[1] * s + e.n[1] * off}; };
    const V2 A = p(t - w / 2), B = p(t + w / 2);
    g.Quad({A[0], y, A[1]}, {B[0], y, B[1]}, {B[0], y + h, B[1]}, {A[0], y + h, A[1]}, {e.n[0], 0, e.n[1]},
           {V2{0, 0}, V2{1, 0}, V2{1, 1}, V2{0, 1}}, col);
}

inline void WallStrip(GB& g, const Edge& e, double y0, double y1, double off = 0) {
    const V3 n{e.n[0], 0, e.n[1]};
    const V2 a{e.a[0] + e.n[0] * off, e.a[1] + e.n[1] * off}, b{e.b[0] + e.n[0] * off, e.b[1] + e.n[1] * off};
    g.Quad({a[0], y0, a[1]}, {b[0], y0, b[1]}, {b[0], y1, b[1]}, {a[0], y1, a[1]}, n,
           {V2{0, y0 / 3}, V2{e.L / 3, y0 / 3}, V2{e.L / 3, y1 / 3}, V2{0, y1 / 3}});
}

inline void Box(GB& g, double cx, double cz, double ux, double uz, double hl, double hw, double y0, double y1, bool top = true) {
    const double vx = -uz, vz = ux;
    auto c = [&](double a, double b) { return V2{cx + ux * a + vx * b, cz + uz * a + vz * b}; };
    const V2 C[4] = {c(-hl, -hw), c(hl, -hw), c(hl, hw), c(-hl, hw)};
    const V2 nn[4] = {{-vx, -vz}, {ux, uz}, {vx, vz}, {-ux, -uz}};
    for (int i = 0; i < 4; ++i) {
        const V2 &a = C[i], &b = C[(i + 1) % 4];
        const double L = std::hypot(b[0] - a[0], b[1] - a[1]);
        g.Quad({a[0], y0, a[1]}, {b[0], y0, b[1]}, {b[0], y1, b[1]}, {a[0], y1, a[1]}, {nn[i][0], 0, nn[i][1]},
               {V2{0, 0}, V2{L / 3, 0}, V2{L / 3, (y1 - y0) / 3}, V2{0, (y1 - y0) / 3}});
    }
    if (top) g.Quad({C[0][0], y1, C[0][1]}, {C[1][0], y1, C[1][1]}, {C[2][0], y1, C[2][1]}, {C[3][0], y1, C[3][1]}, {0, 1, 0});
}

template <size_t N> const char* Pick(Rng& r, const char* const (&a)[N]) { return a[(size_t)std::floor(r() * N)]; }

inline double MinGround(const Pts& pts, const HeightFn& hAt) {
    double m = 1e9;
    for (const V2& p : pts) m = std::min(m, hAt(p[0], p[1]));
    return m;
}

} // namespace optima
