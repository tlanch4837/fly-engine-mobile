// C++ port of fly-optima engine/lib/gen/public.js: schools, libraries, churches, civic, medical, fire / police stations.
// Same rng(s.seed) call order as the JS, so a structure with the same facts gives the same building in both engines.
// Data-driven: footprint (walls follow it exactly), height/floors, name (signs), roof colour (s.roof), kind (cls / name).
// Procedural: brick colour from PAL, window rhythm, entrance edges, canopies, rooftop units, gym mass, steeple, flagpole.
// Differences from the JS (see README of the port in the report): no text on signs (flat panels with a lettering bar),
// no ctx.W (entrances are scored by footprint shape, optional s.front hint), kind picked from the seed when cls/name are empty,
// ribbon windows when the punched-window count would blow the triangle budget, fire-station entrance moved beside the bays.
#pragma once
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "kit.h"

namespace optima {

std::vector<tbd::Vertex> BuildFallback(const Structure& s, const HeightFn& hAt); // inline in generators.h

namespace public_detail {

// flat stand-ins for the public.js materials (M(ctx)) and the ctx.mats it borrows
namespace col {
inline const RGB coping = Hex("#b9b6ad"), frame = Hex("#3b3f44"), unit = Hex("#c9cbc8"), dark = Hex("#2c2f33"),
                 canopy = Hex("#dcdad3"), glass = Hex("#2a3440"), garage = Hex("#e8e6e0"), white = Hex("#f4f4f2"),
                 walk = Hex("#a9a59c"), signBg = Hex("#e9e6dc"), signFg = Hex("#1d3557"), monBg = Hex("#f2efe6"),
                 monFg = Hex("#22324a"), flagRed = Hex("#b22234"), flagWhite = Hex("#f5f5f5"), flagBlue = Hex("#3c3b6e");
}
// data/palettes.json
inline const char* const BRICK_LIB[] = {"#b59a7b", "#9a6650"};             // Tan, Blend
inline const char* const BRICK_CHURCH[] = {"#8e4b3a", "#b59a7b", "#7a5546"}; // Red, Tan, Brown
inline const char* const BRICK_DEF[] = {"#8e4b3a", "#7a5546", "#9a6650"};    // Red, Brown, Blend
inline const char* const ROOF_FLAT[] = {"#d8d8d4", "#9a9b98", "#8e877c"};    // White TPO, Gray Membrane, Gravel BUR
inline const char* const SHINGLE[] = {"#3c3d3f", "#6d6457", "#5d6063"};      // Charcoal, Weathered Wood, Pewter Gray

enum class Kind { Civic, School, Church, Library, Medical, Fire, Police };

inline std::string Lower(std::string t) {
    for (char& c : t) c = (char)std::tolower((unsigned char)c);
    return t;
}

inline Kind KindOf(const Structure& s, double area) {
    const std::string t = Lower(s.cls + " " + s.name);
    static const struct { Kind k; const char* w[7]; } KW[] = {
        {Kind::Fire, {"fire"}},
        {Kind::Police, {"police", "sheriff"}},
        {Kind::Library, {"library"}},
        {Kind::Church, {"church", "worship", "chapel", "temple", "mosque", "synagogue", "parish"}},
        {Kind::Medical, {"hospital", "clinic", "medical", "health"}},
        {Kind::School, {"school", "academy", "education", "college", "kindergarten"}},
    };
    for (const auto& kw : KW)
        for (const char* w : kw.w)
            if (w && t.find(w) != std::string::npos) return kw.k;
    if (t.find_first_not_of(' ') != std::string::npos) return Kind::Civic; // JS: unknown class -> civic
    // synthetic structure: deterministic pick from the seed, on its own stream so the main rng sequence matches the JS
    Rng kr((int64_t)(int32_t)((uint32_t)s.seed ^ 0x9E3779B9u));
    const double x = kr();
    struct W { Kind k; double p; };
    static const W SMALL[] = {{Kind::Church, .35}, {Kind::Medical, .2}, {Kind::Police, .15}, {Kind::Fire, .15}, {Kind::Civic, .15}};
    static const W MID[] = {{Kind::School, .3}, {Kind::Church, .3}, {Kind::Civic, .12}, {Kind::Library, .1},
                            {Kind::Medical, .08}, {Kind::Fire, .05}, {Kind::Police, .05}};
    static const W BIG[] = {{Kind::School, .55}, {Kind::Civic, .15}, {Kind::Medical, .15}, {Kind::Church, .1}, {Kind::Library, .05}};
    auto pick = [x](const W* w, size_t n) {
        double acc = 0;
        for (size_t i = 0; i < n; ++i) { acc += w[i].p; if (x < acc) return w[i].k; }
        return w[n - 1].k;
    };
    if (area < 900) return pick(SMALL, std::size(SMALL));
    if (area > 6000) return pick(BIG, std::size(BIG));
    return pick(MID, std::size(MID));
}

// footprint edge with outward normal (JS: sign of the area, then flipped if the point just outside lies inside)
struct PE { V2 a, b; double L, dx, dz, nx, nz, score = 0; };

inline std::vector<PE> EdgesOf(const Pts& pts) {
    std::vector<PE> E;
    const double sgn = PolyArea(pts) > 0 ? 1 : -1;
    const size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        const V2 &a = pts[i], &b = pts[(i + 1) % n];
        const double L = std::hypot(b[0] - a[0], b[1] - a[1]);
        if (L < .05) continue;
        PE e{a, b, L, (b[0] - a[0]) / L, (b[1] - a[1]) / L, (b[1] - a[1]) / L * -sgn, -(b[0] - a[0]) / L * -sgn};
        if (InPoly(a[0] + e.dx * L / 2 + e.nx * .5, a[1] + e.dz * L / 2 + e.nz * .5, pts)) { e.nx = -e.nx; e.nz = -e.nz; }
        E.push_back(e);
    }
    return E;
}
inline const PE& Longest(const std::vector<PE>& E) {
    size_t k = 0;
    for (size_t i = 1; i < E.size(); ++i) if (E[i].L > E[k].L) k = i;
    return E[k];
}
inline V3 At(const PE& e, double t, double y, double o = 0) { return {e.a[0] + e.dx * t + e.nx * o, y, e.a[1] + e.dz * t + e.nz * o}; }
// JS rect(): a wall-parallel quad on edge e from t0..t1, ya..yb, pushed out by o
inline void Face(GB& G, const PE& e, double t0, double t1, double ya, double yb, double o, double uv = 3) {
    G.Quad(At(e, t0, ya, o), At(e, t1, ya, o), At(e, t1, yb, o), At(e, t0, yb, o), {e.nx, 0, e.nz},
           {V2{t0 / uv, ya / uv}, V2{t1 / uv, ya / uv}, V2{t1 / uv, yb / uv}, V2{t0 / uv, yb / uv}});
}
inline double DistSeg(double x, double z, const V2& a, const V2& b) {
    const double dx = b[0] - a[0], dz = b[1] - a[1];
    double l = dx * dx + dz * dz; if (l == 0) l = 1;
    const double t = std::max(0.0, std::min(1.0, ((x - a[0]) * dx + (z - a[1]) * dz) / l));
    return std::hypot(x - a[0] - t * dx, z - a[1] - t * dz);
}

// three.js Object3D placement: position + rotation.y (local x -> (cos, -sin), local z -> (sin, cos))
struct XF {
    double x, y, z, c, s;
    XF(double px, double py, double pz, double th) : x(px), y(py), z(pz), c(std::cos(th)), s(std::sin(th)) {}
    V3 P(double lx, double ly, double lz) const { return {x + lx * c + lz * s, y + ly, z - lx * s + lz * c}; }
    V3 N(double lx, double ly, double lz) const { return {lx * c + lz * s, ly, -lx * s + lz * c}; }
};
// JS face(): origin at (x,y,z), local +z along the outward normal
inline XF Facing(double x, double y, double z, double nx, double nz) { return XF(x, y, z, std::atan2(nx, nz)); }
inline XF Facing(const V3& p, double nx, double nz) { return Facing(p[0], p[1], p[2], nx, nz); }

// BoxGeometry(sx, sy, sz) centred at local (cx, cy, cz)
inline void LBox(GB& g, const XF& f, double cx, double cy, double cz, double sx, double sy, double sz, bool bottom = true) {
    const double hx = sx / 2, hy = sy / 2, hz = sz / 2;
    auto p = [&](double a, double b, double c) { return f.P(cx + a * hx, cy + b * hy, cz + c * hz); };
    g.Quad(p(1, -1, -1), p(1, -1, 1), p(1, 1, 1), p(1, 1, -1), f.N(1, 0, 0), {V2{0, 0}, V2{sz / 3, 0}, V2{sz / 3, sy / 3}, V2{0, sy / 3}});
    g.Quad(p(-1, -1, 1), p(-1, -1, -1), p(-1, 1, -1), p(-1, 1, 1), f.N(-1, 0, 0), {V2{0, 0}, V2{sz / 3, 0}, V2{sz / 3, sy / 3}, V2{0, sy / 3}});
    g.Quad(p(-1, -1, 1), p(1, -1, 1), p(1, 1, 1), p(-1, 1, 1), f.N(0, 0, 1), {V2{0, 0}, V2{sx / 3, 0}, V2{sx / 3, sy / 3}, V2{0, sy / 3}});
    g.Quad(p(1, -1, -1), p(-1, -1, -1), p(-1, 1, -1), p(1, 1, -1), f.N(0, 0, -1), {V2{0, 0}, V2{sx / 3, 0}, V2{sx / 3, sy / 3}, V2{0, sy / 3}});
    g.Quad(p(-1, 1, -1), p(1, 1, -1), p(1, 1, 1), p(-1, 1, 1), f.N(0, 1, 0), {V2{0, 0}, V2{sx / 3, 0}, V2{sx / 3, sz / 3}, V2{0, sz / 3}});
    if (bottom) g.Quad(p(-1, -1, -1), p(1, -1, -1), p(1, -1, 1), p(-1, -1, 1), f.N(0, -1, 0));
}
// CylinderGeometry(rTop, rBot, h, seg) centred at local (cx, cy, cz), flat-shaded sides, optional top cap
inline void LCyl(GB& g, const XF& f, double cx, double cy, double cz, double rTop, double rBot, double h, int seg, bool top) {
    const double PI = 3.14159265358979323846, y0 = cy - h / 2, y1 = cy + h / 2, slope = (rBot - rTop) / h;
    for (int i = 0; i < seg; ++i) {
        const double a0 = 2 * PI * i / seg, a1 = 2 * PI * (i + 1) / seg, m = (a0 + a1) / 2;
        const V3 n = Nrm(f.N(std::sin(m), slope, std::cos(m)));
        g.Quad(f.P(cx + rBot * std::sin(a0), y0, cz + rBot * std::cos(a0)), f.P(cx + rBot * std::sin(a1), y0, cz + rBot * std::cos(a1)),
               f.P(cx + rTop * std::sin(a1), y1, cz + rTop * std::cos(a1)), f.P(cx + rTop * std::sin(a0), y1, cz + rTop * std::cos(a0)), n);
        if (top && rTop > 0)
            g.Tri(f.P(cx, y1, cz), f.P(cx + rTop * std::sin(a0), y1, cz + rTop * std::cos(a0)),
                  f.P(cx + rTop * std::sin(a1), y1, cz + rTop * std::cos(a1)), {0, 1, 0});
    }
}
// PlaneGeometry(w, h) in local XY at (cx, cy, cz), facing local +z (or -z with back = true)
inline void LPlane(GB& g, const XF& f, double cx, double cy, double cz, double w, double h, const RGB* c = nullptr, bool back = false) {
    const double hw = w / 2, hh = h / 2;
    g.Quad(f.P(cx - hw, cy - hh, cz), f.P(cx + hw, cy - hh, cz), f.P(cx + hw, cy + hh, cz), f.P(cx - hw, cy + hh, cz),
           f.N(0, 0, back ? -1 : 1), {V2{0, 0}, V2{1, 0}, V2{1, 1}, V2{0, 1}}, c);
}
// a blank sign: background panel plus a darker bar standing in for the lettering (the JS draws text into an atlas)
inline void Sign(GB& g, const XF& f, double cx, double cy, double cz, double w, double h, size_t letters, const RGB& bg, const RGB& fg) {
    LPlane(g, f, cx, cy, cz, w, h, &bg);
    const double lw = std::min(w * .86, std::max(.4, letters * h * .5));
    LPlane(g, f, cx, cy, cz + .01, lw, h * .42, &fg);
}

// JS clearRect(): largest axis-aligned rectangle (in the frame rotated by ang) of grid cells fully inside the footprint,
// at least 18 m on a side, aspect < 2.2, then capped to gym size (55 x 42 m)
struct ClearR { double a; Pts pts; };
inline std::optional<ClearR> ClearRect(const Pts& pts, double ang, double cell) {
    const double ca = std::cos(ang), sa = std::sin(ang);
    Pts loc;
    double x0 = 1e18, x1 = -1e18, z0 = 1e18, z1 = -1e18;
    for (const V2& p : pts) {
        const V2 q{p[0] * ca + p[1] * sa, -p[0] * sa + p[1] * ca};
        loc.push_back(q);
        x0 = std::min(x0, q[0]); x1 = std::max(x1, q[0]); z0 = std::min(z0, q[1]); z1 = std::max(z1, q[1]);
    }
    if (!(x1 - x0 >= 18) || !(z1 - z0 >= 18)) return std::nullopt;
    cell = std::max(cell, std::sqrt((x1 - x0) * (z1 - z0) / 40000)); // keep the grid <= ~40k cells on huge footprints
    const int nx = (int)std::ceil((x1 - x0) / cell), nz = (int)std::ceil((z1 - z0) / cell);
    std::vector<int> hgt((size_t)nx, 0);
    struct B { double a = 0; int i0 = 0, i1 = 0, j0 = 0, j1 = 0; } best;
    std::vector<std::pair<int, int>> st;
    for (int j = 0; j < nz; ++j) {
        for (int i = 0; i < nx; ++i) {
            const double x = x0 + i * cell, z = z0 + j * cell;
            const bool ok = InPoly(x + cell / 2, z + cell / 2, loc) && InPoly(x + .2, z + .2, loc) && InPoly(x + cell - .2, z + .2, loc) &&
                            InPoly(x + .2, z + cell - .2, loc) && InPoly(x + cell - .2, z + cell - .2, loc);
            hgt[(size_t)i] = ok ? hgt[(size_t)i] + 1 : 0;
        }
        st.clear();
        for (int i = 0; i <= nx; ++i) {
            const int hh = i < nx ? hgt[(size_t)i] : 0;
            int s0 = i;
            while (!st.empty() && st.back().second >= hh) {
                const auto [si, sh] = st.back();
                st.pop_back();
                const int w = i - si;
                const double a = w * sh * cell * cell;
                if (a > best.a && w * cell >= 18 && sh * cell >= 18 && (double)std::max(w, sh) / std::min(w, sh) < 2.2)
                    best = B{a, si, i, j - sh + 1, j + 1};
                s0 = si;
            }
            st.push_back({s0, hh});
        }
    }
    if (!best.a) return std::nullopt;
    double u0 = x0 + best.i0 * cell, u1 = x0 + best.i1 * cell, v0 = z0 + best.j0 * cell, v1 = z0 + best.j1 * cell;
    auto cap = [](double& a, double& b, double m) { if (b - a > m) { const double c = (a + b) / 2; a = c - m / 2; b = c + m / 2; } };
    if (u1 - u0 >= v1 - v0) { cap(u0, u1, 55); cap(v0, v1, 42); } else { cap(u0, u1, 42); cap(v0, v1, 55); }
    auto toW = [&](double u, double v) { return V2{u * ca - v * sa, u * sa + v * ca}; };
    return ClearR{(u1 - u0) * (v1 - v0), {toW(u0, v0), toW(u1, v0), toW(u1, v1), toW(u0, v1)}};
}

// [a,b] minus the skip intervals
inline std::vector<std::pair<double, double>> Cut(double a, double b, const std::vector<std::pair<double, double>>& skip) {
    std::vector<std::pair<double, double>> out{{a, b}};
    for (const auto& [s0, s1] : skip) {
        std::vector<std::pair<double, double>> nx;
        for (const auto& [p, q] : out) {
            if (s1 <= p || s0 >= q) { nx.push_back({p, q}); continue; }
            if (s0 - p > .3) nx.push_back({p, s0});
            if (q - s1 > .3) nx.push_back({s1, q});
        }
        out.swap(nx);
    }
    return out;
}

struct Ent { size_t e; bool main; double t, w; };

struct Mats {
    GB wall, glass, band, frame, roof, garage, unit, dark, canopy, white, walk, sign, flag, shingle;
};

// JS churchRoof(): gable over the oriented box of sp, steeple on one end
inline void ChurchRoof(Mats& m, const Pts& sp, double y1, Rng& r) {
    std::vector<PE> E;
    for (size_t i = 0; i < sp.size(); ++i) {
        const V2 &a = sp[i], &b = sp[(i + 1) % sp.size()];
        const double L = std::hypot(b[0] - a[0], b[1] - a[1]);
        if (L >= .05) E.push_back({a, b, L, (b[0] - a[0]) / L, (b[1] - a[1]) / L, 0, 0});
    }
    if (E.empty()) return;
    const PE& Le = Longest(E);
    const double ang = std::atan2(Le.dz, Le.dx), ca = std::cos(ang), sa = std::sin(ang);
    double u0 = 1e18, u1 = -1e18, v0 = 1e18, v1 = -1e18;
    for (const V2& p : sp) {
        const double u = p[0] * ca + p[1] * sa, v = -p[0] * sa + p[1] * ca;
        u0 = std::min(u0, u); u1 = std::max(u1, u); v0 = std::min(v0, v); v1 = std::max(v1, v);
    }
    u0 -= .4; u1 += .4; v0 -= .4; v1 += .4;
    const double vm = (v0 + v1) / 2, rise = std::min((v1 - v0) / 2 * .8, 9.0);
    auto W = [&](double u, double v, double y) { return V3{u * ca - v * sa, y, u * sa + v * ca}; };
    m.shingle.color = Hex(Pick(r, SHINGLE)); // (the JS picks after building the quads; no rng in between, same sequence)
    for (int sd : {-1, 1}) {
        const double ve = sd < 0 ? v0 : v1;
        m.shingle.Quad(W(u0, ve, y1), W(u1, ve, y1), W(u1, vm, y1 + rise), W(u0, vm, y1 + rise), {-sa * sd * .67, .74, ca * sd * .67},
                       {V2{0, 0}, V2{(u1 - u0) / 3, 0}, V2{(u1 - u0) / 3, rise / 2.2}, V2{0, rise / 2.2}});
    }
    for (double u : {u0 + .4, u1 - .4}) {
        const double sg = u < (u0 + u1) / 2 ? -1 : 1;
        m.wall.Tri(W(u, v0 + .4, y1), W(u, v1 - .4, y1), W(u, vm, y1 + rise - .3), {ca * sg, 0, sa * sg}, {0, 0}, {(v1 - v0) / 3, 0},
                   {(v1 - v0) / 6, rise / 3});
    }
    // steeple
    const double su = r() < .5 ? u0 + 2.5 : u1 - 2.5, tH = rise + 6, tw = 3.6;
    const V3 base = W(su, vm, 0);
    const XF st(base[0], y1 - .5, base[2], -ang);
    LBox(m.wall, st, 0, tH / 2, 0, tw, tH, tw, false);
    LBox(m.band, st, 0, tH, 0, tw + .3, .3, tw + .3);
    LBox(m.white, st, 0, tH + 1.2, 0, tw * .8, 2.4, tw * .8, false);
    { // 4-sided spire (ConeGeometry(tw*.55, 9, 4) turned 45 degrees: faces aligned with the tower)
        const double hs = tw * .55 / std::sqrt(2.0), yb = tH + 2.4, yt = yb + 9;
        const V3 c[4] = {st.P(-hs, yb, -hs), st.P(hs, yb, -hs), st.P(hs, yb, hs), st.P(-hs, yb, hs)}, apex = st.P(0, yt, 0);
        for (int i = 0; i < 4; ++i) m.dark.UpTri(c[i], c[(i + 1) % 4], apex, {0, 0}, {1, 0}, {.5, 3});
    }
    LBox(m.white, st, 0, tH + 2.4 + 9.6, 0, .12, 1.4, .12);
    LBox(m.white, st, 0, tH + 2.4 + 9.8, 0, .8, .12, .12);
}

// JS addEntrance(): glazed doors with mullions, canopy (posts on the main one), concrete apron, name sign above
inline void Entrance(Mats& m, const PE& e, const Ent& ent, double y0, double fh, Kind kind, const Structure& s, const HeightFn& hAt) {
    const double t0 = ent.t - ent.w / 2, t1 = ent.t + ent.w / 2, top = y0 + std::max(2.3, std::min(3.2, fh - .4));
    m.glass.Quad(At(e, t0, y0 + .1, .06), At(e, t1, y0 + .1, .06), At(e, t1, top, .06), At(e, t0, top, .06), {e.nx, 0, e.nz});
    const int mull = std::max(2, (int)std::lround(ent.w / 1.2));
    for (int k = 0; k <= mull; ++k) {
        const double t = t0 + k * ent.w / mull;
        m.frame.Quad(At(e, t - .05, y0, .08), At(e, t + .05, y0, .08), At(e, t + .05, top, .08), At(e, t - .05, top, .08), {e.nx, 0, e.nz});
    }
    m.frame.Quad(At(e, t0, top - .05, .08), At(e, t1, top - .05, .08), At(e, t1, top + .08, .08), At(e, t0, top + .08, .08), {e.nx, 0, e.nz});
    const double depth = ent.main ? 4.5 : 2.4, cw = ent.w + (ent.main ? 2 : 1), cy = top + .5;
    const V3 mp = At(e, ent.t, cy, 0);
    const XF c = Facing(mp, e.nx, e.nz);
    LBox(kind == Kind::Church ? m.white : m.canopy, c, 0, 0, depth / 2, cw, .45, depth);
    LBox(m.frame, c, 0, .27, depth / 2, cw + .1, .12, depth + .1, false);
    if (ent.main)
        for (int sx : {-1, 1}) LCyl(m.band, c, sx * (cw / 2 - .4), -(cy - y0) / 2, depth - .4, .16, .16, cy - y0, 8, false);
    // concrete apron
    const V3 a0 = At(e, ent.t, hAt(mp[0] + e.nx * 3, mp[2] + e.nz * 3) + .02, (depth + 3) / 2);
    LBox(m.walk, Facing(a0, e.nx, e.nz), 0, 0, 0, cw + 2, .2, depth + 3, false);
    if (ent.main && !s.name.empty()) {
        const double sh = .8, sw = std::min(e.L - 1, std::max(ent.w + 2, s.name.size() * sh * .55));
        const V3 q = At(e, ent.t, cy + .25 + .4 + sh / 2 + .1, .06);
        Sign(m.sign, Facing(q, e.nx, e.nz), 0, 0, 0, sw, sh, s.name.size(), col::signBg, col::signFg);
    }
}

inline std::vector<tbd::Vertex> Build(const Structure& s, const HeightFn& hAt) {
    const Pts& pts = s.pts;
    if (pts.size() < 3) return BuildFallback(s, hAt);
    for (const V2& p : pts) if (!std::isfinite(p[0]) || !std::isfinite(p[1])) return BuildFallback(s, hAt);
    const double area = std::abs(PolyArea(pts));
    if (!(area >= 12)) return BuildFallback(s, hAt);
    std::vector<PE> E = EdgesOf(pts);
    if (E.size() < 3) return BuildFallback(s, hAt);

    Rng r(s.seed ? s.seed : 1);
    const Kind kind = KindOf(s, area);
    const bool church = kind == Kind::Church, school = kind == Kind::School, library = kind == Kind::Library,
               medical = kind == Kind::Medical, fire = kind == Kind::Fire;
    int floors = s.floors > 0 ? (int)std::lround(s.floors) : (int)std::lround((s.height > 0 ? s.height : 4.5) / 4);
    floors = std::max(1, std::min(60, floors));
    if (!church && s.floors > 0 && s.height > 0 && s.height / floors > 6) floors = std::max(1, std::min(60, (int)std::lround(s.height / 4.2)));
    const double y0 = MinGround(pts, hAt) - .1;
    if (!std::isfinite(y0)) return BuildFallback(s, hAt);
    double H = s.height > 0 ? s.height : floors * 4.2;
    if (church) H = std::max(H, 6.0);
    H = std::min(H, 250.0);
    const double fh = std::min(4.5, H / floors), y1 = y0 + .1 + H, par = church ? 0 : .9;
    const char* brickHex = library || medical ? Pick(r, BRICK_LIB) : church ? Pick(r, BRICK_CHURCH) : Pick(r, BRICK_DEF);
    const RGB brickC = Hex(s.brickCol.empty() ? std::string(brickHex) : s.brickCol);
    const RGB roofC = Hex(s.roof.empty() ? std::string(Pick(r, ROOF_FLAT)) : s.roof); // JS: s.roofCol || pick(...) (no rng call when set)
    Mats m{GB(brickC), GB(col::glass), GB(col::coping), GB(col::frame), GB(roofC), GB(col::garage), GB(col::unit), GB(col::dark),
           GB(col::canopy), GB(col::white), GB(col::walk), GB(col::signBg), GB(col::flagWhite), GB(Hex(SHINGLE[0]))};

    // ---- entrance edges. The JS scores by distance to parking / roads in ctx.W; here only the courtyard test and edge
    // length remain, plus an optional s.front direction hint (toward the street).
    for (PE& e : E) {
        const double mx = e.a[0] + e.dx * e.L / 2 + e.nx * 12, mz = e.a[1] + e.dz * e.L / 2 + e.nz * 12;
        double d = 400;
        if (s.front) d -= 150 * (e.nx * (*s.front)[0] + e.nz * (*s.front)[1]);
        if (InPoly(mx, mz, pts)) d += 500; // facing into a courtyard
        e.score = d - std::min(e.L, 40.0) * .8 + r() * 3;
    }
    std::vector<size_t> byScore;
    for (size_t i = 0; i < E.size(); ++i) if (E[i].L >= 7) byScore.push_back(i);
    std::stable_sort(byScore.begin(), byScore.end(), [&](size_t p, size_t q) { return E[p].score < E[q].score; });
    const long mainI = byScore.empty() ? -1 : (long)byScore[0];
    std::vector<Ent> ents;
    if (mainI >= 0) {
        const PE& e = E[(size_t)mainI];
        ents.push_back({(size_t)mainI, true, e.L / 2, std::min(e.L - 2, area > 8000 ? 12.0 : 7.0)});
        const size_t maxExtra = area > 8000 ? 3 : area > 2500 ? 1 : 0;
        for (size_t k = 1, got = 0; k < byScore.size() && got < maxExtra; ++k) {
            const PE& x = E[byScore[k]];
            if (!(x.score < e.score + 60)) continue;
            ents.push_back({byScore[k], false, x.L * (.3 + r() * .4), std::min(x.L - 2, 4.5)});
            ++got;
        }
    }
    // fire station: apparatus bays on the main edge. The JS centres both the bays and the main door on that edge; here the
    // door moves beside the bays when it fits, else it is dropped (the bays are the front).
    int bays = fire && mainI >= 0 ? std::max(1, std::min(4, (int)std::floor((E[(size_t)mainI].L - 6) / 5))) : 0;
    if (bays) {
        const double L = E[(size_t)mainI].L, w = std::min(ents[0].w, 3.5), b1 = (L + bays * 5) / 2;
        if (b1 + 1 + w + .5 <= L) { ents[0].w = w; ents[0].t = b1 + 1 + w / 2; }
        else ents.erase(ents.begin());
    }
    auto entOf = [&](size_t i) -> const Ent* { for (const Ent& x : ents) if (x.e == i) return &x; return nullptr; };

    // ---- walls with window bands, floor lines, parapet
    const double wTop = y1 + par, winW = library ? 3 : church ? .9 : 1.6, gap = church ? 3 : school ? .8 : 1.1, step = winW + gap;
    long total = 0;
    for (const PE& e : E) if (e.L >= 3) total += floors * std::max(0L, (long)std::floor((e.L - 1.2) / step));
    const bool ribbon = total > 900; // budget: continuous glazing bands instead of hundreds of punched windows
    for (size_t ei = 0; ei < E.size(); ++ei) {
        const PE& e = E[ei];
        Face(m.wall, e, 0, e.L, y0, wTop, 0);
        if (par) {
            Face(m.band, e, 0, e.L, wTop - .25, wTop + .05, .04, 1);
            m.wall.Quad(At(e, 0, y1, -.35), At(e, e.L, y1, -.35), At(e, e.L, wTop, -.35), At(e, 0, wTop, -.35), {-e.nx, 0, -e.nz});
        }
        const Ent* ent = entOf(ei);
        const bool isMain = (long)ei == mainI;
        const double skip0 = ent ? ent->t - ent->w / 2 - .5 : -1, skip1 = ent ? ent->t + ent->w / 2 + .5 : -1;
        const double bay0 = isMain && bays ? (e.L - bays * 5) / 2 : -1, bay1 = bay0 >= 0 ? bay0 + bays * 5 : -1;
        for (int f = 0; f < floors; ++f) {
            const double fy = y0 + .1 + f * fh;
            if (f > 0 && !church) Face(m.band, e, 0, e.L, fy - .2, fy + .15, .03, 1);
            if (e.L < 3) continue;
            const double wy0 = fy + (church ? 1 : .95);
            const double wy1 = std::min(fy + fh - .55, church ? y1 - 1 : fy + (library && f == 0 ? 3.2 : 2.3));
            if (wy1 - wy0 < .4) continue;
            const int cnt = (int)std::floor((e.L - 1.2) / step);
            if (cnt <= 0) continue;
            const double t0 = (e.L - cnt * step + gap) / 2;
            if (ribbon) {
                std::vector<std::pair<double, double>> skip;
                if (f == 0 && ent) skip.push_back({skip0, skip1});
                if (f == 0 && bay0 >= 0) skip.push_back({bay0, bay1});
                for (const auto& [ta, tb] : Cut(t0, e.L - t0, skip)) {
                    Face(m.glass, e, ta, tb, wy0, wy1, .05, 1);
                    Face(m.frame, e, ta - .06, tb + .06, wy0 - .12, wy0, .07, 1);
                }
                continue;
            }
            for (int k = 0; k < cnt; ++k) {
                const double ta = t0 + k * step, tb = ta + winW;
                if (f == 0 && ((tb > skip0 && ta < skip1) || (tb > bay0 && ta < bay1))) continue;
                Face(m.glass, e, ta, tb, wy0, wy1, .05, 1);
                Face(m.frame, e, ta - .06, tb + .06, wy0 - .12, wy0, .07, 1);
                if (school && tb - ta > 1.2) Face(m.frame, e, ta + (tb - ta) / 2 - .04, ta + (tb - ta) / 2 + .04, wy0, wy1, .07, 1);
            }
        }
        if (ent) Entrance(m, e, *ent, y0, fh, kind, s, hAt);
        if (isMain && bays) {
            const double bh = std::max(2.5, std::min(4.3, wTop - y0 - .5));
            for (int k = 0; k < bays; ++k) {
                const double ta = bay0 + k * 5 + .6, tb = ta + 3.8;
                Face(m.frame, e, ta - .15, tb + .15, y0, y0 + bh + .2, .06, 1);
                Face(m.garage, e, ta, tb, y0, y0 + bh, .08, 4.3);
                Face(m.band, e, ta - .4, tb + .4, y0 - .02, y0 + .06, 4, 1);
            }
        }
    }

    // ---- roof. Church: gable the whole footprint when it is nearly a rectangle, else a flat-roofed plan with a gabled
    // sanctuary on its largest clear rectangle.
    std::optional<Pts> sanct;
    bool sanctWhole = false;
    if (church) {
        const PE& Le = Longest(E);
        const double an = std::atan2(Le.dz, Le.dx), ca = std::cos(an), sa = std::sin(an);
        double u0 = 1e18, u1 = -1e18, v0 = 1e18, v1 = -1e18;
        for (const V2& p : pts) {
            const double u = p[0] * ca + p[1] * sa, v = -p[0] * sa + p[1] * ca;
            u0 = std::min(u0, u); u1 = std::max(u1, u); v0 = std::min(v0, v); v1 = std::max(v1, v);
        }
        const double fill = area / std::max(1e-6, (u1 - u0) * (v1 - v0));
        if (fill > .82) { sanct = pts; sanctWhole = true; }
        else if (auto cr = ClearRect(pts, an, 2)) sanct = cr->pts;
    }
    if (sanct) ChurchRoof(m, *sanct, y1, r);
    if (!sanct || !sanctWhole) Cap(pts, y1, m.roof);

    // ---- gym / auditorium mass for big schools: the largest clear rectangle in the footprint, raised
    std::optional<ClearR> gym;
    if (school && area > 5000) {
        const PE& Le = Longest(E);
        const double ang = std::atan2(Le.dz, Le.dx);
        gym = ClearRect(pts, ang, 3);
        if (gym && gym->a > 650) {
            const Pts& gp = gym->pts;
            const double gH = std::max(H + 4.5, 11.5), gTop = y0 + .1 + gH;
            const std::vector<PE> GE = EdgesOf(gp);
            for (const PE& e : GE) {
                Face(m.wall, e, 0, e.L, y1 - .2, gTop + .6, 0);
                Face(m.band, e, 0, e.L, gTop + .35, gTop + .65, .04, 1);
                Face(m.band, e, 0, e.L, y1 + par + .6, y1 + par + 1.1, .03, 1);
                const int cnt = (int)std::floor((e.L - 4) / 4);
                for (int k = 0; k < cnt; ++k) { const double t = 2 + k * 4 + 1.2; Face(m.glass, e, t, t + 1.6, gTop - 1.6, gTop - .6, .05, 1); } // clerestory
            }
            Cap(gp, gTop, m.roof);
            double cx = 0, cz = 0;
            for (const V2& p : gp) { cx += p[0] / 4; cz += p[1] / 4; }
            for (int k = 0; k < 3; ++k)
                LBox(m.unit, XF(cx + (k - 1) * 6 * std::cos(ang), gTop + .7, cz + (k - 1) * 6 * std::sin(ang), -ang), 0, 0, 0, 2.2, 1.4, 3.2, false);
        } else gym.reset();
    }

    // ---- rooftop HVAC units on the flat roof
    if (!church) {
        double bx0 = 1e18, bx1 = -1e18, bz0 = 1e18, bz1 = -1e18;
        for (const V2& p : pts) { bx0 = std::min(bx0, p[0]); bx1 = std::max(bx1, p[0]); bz0 = std::min(bz0, p[1]); bz1 = std::max(bz1, p[1]); }
        const double ang = mainI >= 0 ? std::atan2(E[(size_t)mainI].dz, E[(size_t)mainI].dx) : 0;
        const int want = std::max(1, std::min(40, (int)std::lround(area / 550)));
        auto okAt = [&](double x, double z) {
            if (!InPoly(x, z, pts)) return false;
            for (const PE& e : E) if (DistSeg(x, z, e.a, e.b) <= 4) return false;
            if (gym) {
                if (InPoly(x, z, gym->pts)) return false;
                for (size_t i = 0; i < 4; ++i) if (DistSeg(x, z, gym->pts[i], gym->pts[(i + 1) % 4]) < 4) return false;
            }
            return true;
        };
        for (int k = 0, tries = 0; k < want && tries < want * 25; ++tries) {
            const double x = bx0 + r() * (bx1 - bx0), z = bz0 + r() * (bz1 - bz0);
            if (!okAt(x, z)) continue;
            ++k;
            const bool big = r() < .35;
            const double w = big ? 2.4 + r() * 1.2 : 1.2 + r() * .8;
            const double d = big ? 4 + r() * 2.5 : 1.4 + r() * 1;
            const double h = big ? 1.5 + r() * .5 : .9 + r() * .4;
            const XF u(x, y1, z, -ang);
            LBox(m.unit, u, 0, h / 2 + .15, 0, w, h, d, false);
            LBox(m.dark, u, 0, .07, 0, w + .3, .15, d + .3, false);
            if (big) for (int f = 0; f < 2; ++f) LCyl(m.dark, u, 0, h + .2, (f - .5) * d * .5, .55, .55, .12, 10, true);
        }
    }

    // ---- site: monument sign by the main entrance, flagpole (only for named structures, as in the JS)
    if (mainI >= 0 && !s.name.empty()) {
        const PE& e = E[(size_t)mainI];
        const double sd = r() < .5 ? -1 : 1;
        double mx = 0, mz = 0;
        bool ok = false;
        static const double SPOTS[6][2] = {{16, 1}, {10, 1}, {16, -1}, {24, 1}, {7, -1}, {24, -1}};
        for (const auto& sp : SPOTS) { // first spot not inside the building (the JS also avoids parking lots)
            const double t = e.L / 2 + sd * sp[1] * std::min(e.L / 2 - 2, 10.0);
            mx = e.a[0] + e.dx * t + e.nx * sp[0]; mz = e.a[1] + e.dz * t + e.nz * sp[0];
            if (!InPoly(mx, mz, pts)) { ok = true; break; }
        }
        if (ok) {
            const double gy = hAt(mx, mz);
            const XF mon = Facing(mx, gy, mz, e.nx, e.nz);
            LBox(m.wall, mon, 0, .8, 0, 4.6, 1.6, .7, false);
            LBox(m.band, mon, 0, 1.68, 0, 4.9, .15, .9);
            Sign(m.sign, mon, 0, .95, .37, 3.9, .75, s.name.size(), col::monBg, col::monFg);
            Sign(m.sign, XF(mx, gy, mz, std::atan2(e.nx, e.nz) + 3.14159265358979323846), 0, .95, .37, 3.9, .75, s.name.size(), col::monBg, col::monFg);
            if (!church && !medical) {
                const double fx = mx + e.dx * 4, fz = mz + e.dz * 4;
                const XF pole(fx, hAt(fx, fz), fz, std::atan2(e.nx, e.nz) + 3.14159265358979323846 / 2);
                LCyl(m.band, pole, 0, 5, 0, .05, .09, 10, 8, true);
                // flag: 7 stripes + canton (simplified from the 13-stripe canvas texture), both faces
                for (bool back : {false, true}) {
                    const double z = back ? -.004 : .004;
                    for (int i = 0; i < 7; ++i)
                        LPlane(m.flag, pole, .95, 9.2 + .55 - (i + .5) * 1.1 / 7, z, 1.8, 1.1 / 7, i % 2 ? &col::flagWhite : &col::flagRed, back);
                    LPlane(m.flag, pole, .05 + .36, 9.2 + .55 - .3, z * 2, .72, .6, &col::flagBlue, back);
                }
            }
        }
    }
    if (fire) {
        const PE& e = Longest(E);
        const V3 c = At(e, e.L * .85, y0 + (H + 6) / 2, -1.7);
        LBox(m.wall, XF(c[0], c[1], c[2], -std::atan2(e.dz, e.dx)), 0, 0, 0, 3.2, H + 6, 3.2); // hose-drying tower
    }

    std::vector<tbd::Vertex> out;
    m.wall.AppendTo(out, true);
    m.shingle.AppendTo(out, true);
    for (GB* g : {&m.glass, &m.band, &m.frame, &m.roof, &m.garage, &m.unit, &m.dark, &m.canopy, &m.white, &m.walk, &m.sign, &m.flag})
        g->AppendTo(out);
    return out;
}

} // namespace public_detail

inline std::vector<tbd::Vertex> BuildPublic(const Structure& s, const HeightFn& hAt) {
    for (const V2& p : s.pts) if (!std::isfinite(p[0]) || !std::isfinite(p[1])) return {}; // the box fallback would emit NaNs too
    try {
        std::vector<tbd::Vertex> v = public_detail::Build(s, hAt);
        if (!v.empty()) return v;
    } catch (...) {
    }
    return BuildFallback(s, hAt);
}

} // namespace optima
