// Optima-Earth commercial structures, C++ port of fly-optima engine/lib/gen/commercial.js + lib/gen/commercial/kit.js:
// strip malls, big boxes, restaurants / fast food, banks, offices (up to downtown towers), single retail, gas kiosks and
// fuel canopies. Flat colours per material (one GB per colour) instead of the textured three.js materials; sign text is
// drawn as a coloured text band (no glyph atlas). Facts the JS read from facts/commercial.py (sub, front, tn, drive) are
// inferred here with seeded defaults, since synthetic structures only carry pts / cat / height / floors / area / seed.
#pragma once
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "kit.h"

namespace optima {

inline std::vector<tbd::Vertex> BuildFallback(const Structure&, const HeightFn&); // defined in generators.h

namespace commercial_detail {

// ---------------- palettes (data/palettes.json) and guessed commercial colours ----------------
static const char* const BRICK[] = {"#8e4b3a", "#7a5546", "#b59a7b", "#9a6650"};
static const char* const STUCCO3[] = {"#eeeae0", "#e8dcc0", "#e3b998"};
static const char* const FLAT[] = {"#d8d8d4", "#e4e3de", "#cfcec8", "#b9b8b2", "#9a9b98", "#4a4a48"}; // white / light membrane mostly, a dark EPDM now and then
static const char* const METAL[] = {"#a7aaab", "#4a3d33", "#3f4246", "#3c5244", "#7a2e27", "#e2e2de"};
static const char* const ASPHALT[] = {"#2b2b2c", "#3c3d3f", "#55575b", "#5d6063", "#6a6c6c", "#7f7f7a", "#6d6457", "#7a6e5f",
                                      "#5a4a3c", "#6b5040", "#6e5844", "#8a7458", "#9c8a6e", "#4f5862", "#3f4c3c", "#3e4a5a"};
static const char* const METAL_ASPH4[] = {"#a7aaab", "#4a3d33", "#3f4246", "#3c5244", "#7a2e27", "#e2e2de",
                                          "#2b2b2c", "#3c3d3f", "#55575b", "#5d6063"};
// sign colours: common US commercial sign schemes (guessed; not data)
static const char* const SIGNS[][2] = {{"#b3121b", "#ffffff"}, {"#0d3a7a", "#ffffff"}, {"#ffffff", "#b3121b"}, {"#14532d", "#ffffff"},
                                       {"#1c1c1c", "#f4c430"}, {"#ffffff", "#123c73"}, {"#e25c12", "#ffffff"}, {"#5b1d6e", "#ffffff"},
                                       {"#f2f2f2", "#222222"}};
struct BrandRule { const char* const* keys; const char* bg; const char* fg; };
static const char* const B_SPEEDWAY[] = {"speedway", nullptr};
static const char* const B_MOBIL[] = {"mobil", "exxon", nullptr};
static const char* const B_MARATHON[] = {"marathon", nullptr};
static const char* const B_SHELL[] = {"shell", nullptr};
static const char* const B_BP[] = {"\x01" "bp", nullptr}; // \x01 = whole-word match
static const char* const B_SUNOCO[] = {"sunoco", nullptr};
static const char* const B_CITGO[] = {"citgo", nullptr};
static const char* const B_MEIJER[] = {"meijer", nullptr};
static const char* const B_MCD[] = {"mcdonald", nullptr};
static const char* const B_DUNKIN[] = {"dunkin", nullptr};
static const char* const B_COMERICA[] = {"comerica", nullptr};
static const char* const B_CHASE[] = {"chase", nullptr};
static const char* const B_ALDI[] = {"aldi", nullptr};
static const char* const B_KROGER[] = {"kroger", nullptr};
static const char* const B_TARGET[] = {"target", nullptr};
static const BrandRule BRAND[] = {{B_SPEEDWAY, "#d4151c", "#ffffff"}, {B_MOBIL, "#0b4ea2", "#ffffff"}, {B_MARATHON, "#1f3f8f", "#f15a22"},
                                  {B_SHELL, "#fbce07", "#dd1d21"},    {B_BP, "#00843d", "#ffffff"},    {B_SUNOCO, "#183a7c", "#fdb913"},
                                  {B_CITGO, "#e31b23", "#ffffff"},    {B_MEIJER, "#d71920", "#ffffff"}, {B_MCD, "#da291c", "#ffc72c"},
                                  {B_DUNKIN, "#ff671f", "#ffffff"},   {B_COMERICA, "#0a3c7d", "#ffffff"}, {B_CHASE, "#117aca", "#ffffff"},
                                  {B_ALDI, "#00005f", "#ffffff"},     {B_KROGER, "#134b97", "#ffffff"}, {B_TARGET, "#cc0000", "#ffffff"}};
static const char* const GENERIC[] = {"NAILS", "PIZZA", "CLEANERS", "TAX SERVICE", "CELL PHONES", "SALON", "SUBS", "MATTRESS",
                                      "VAPE", "DENTAL", "INSURANCE", "NUTRITION", "CHINESE FOOD", "TANNING", "LIQUOR", "PET SUPPLY"};

// flat stand-ins for mats.* in index.html
static const char* const GLASS = "#2e3e4c";   // mats.glass 0x1d2d3b, lifted a bit for its env reflection
static const char* const AC = "#b9bcbc";      // mats.ac
static const char* const WALK = "#b5b2aa";    // mats.walk (concrete tinted to the sampled sidewalk)
static const char* const MULL = "#3a3d40";
static const char* const COPING = "#c9c7c0";
static const char* const SOFFIT = "#d8d6cf";
static const char* const SIGNBACK = "#2a2a2a";

inline std::string Lower(std::string t) { for (char& ch : t) ch = (char)std::tolower((unsigned char)ch); return t; }
inline bool HasWord(const std::string& s, const std::string& w) {
    for (size_t p = s.find(w); p != std::string::npos; p = s.find(w, p + 1)) {
        const bool l = p == 0 || !std::isalnum((unsigned char)s[p - 1]), r = p + w.size() >= s.size() || !std::isalnum((unsigned char)s[p + w.size()]);
        if (l && r) return true;
    }
    return false;
}
// any of the keys (nullptr-terminated) in lower-case s; a leading \x01 asks for a whole word
inline bool HasAny(const std::string& s, const char* const* keys) {
    for (; *keys; ++keys) {
        if ((*keys)[0] == '\x01') { if (HasWord(s, *keys + 1)) return true; }
        else if (s.find(*keys) != std::string::npos) return true;
    }
    return false;
}

using Col2 = std::pair<std::string, std::string>;
inline Col2 BrandOf(const std::string& name, Rng& r) {
    const std::string l = Lower(name);
    for (const BrandRule& b : BRAND) if (HasAny(l, b.keys)) return {b.bg, b.fg};
    const auto& p = SIGNS[(size_t)std::floor(r() * 9) % 9];
    return {p[0], p[1]};
}
inline std::string PickS(Rng& r, std::initializer_list<const char*> a) { return *(a.begin() + (size_t)std::floor(r() * a.size()) % a.size()); }
template <size_t N> std::string PickA(Rng& r, const char* const (&a)[N]) { return a[(size_t)std::floor(r() * N) % N]; }
inline double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }

// ---------------- kit.js: edge frames, inset, oriented boxes/panels/fills ----------------
struct CE { int i = 0; V2 a{}, b{}; double L = 1e-6; V2 u{1, 0}, n{0, 1}; };

inline bool Pip(const V2& p, const Pts& poly) { return poly.size() >= 3 && InPoly(p[0], p[1], poly); }

// edges with outward normals (tested on the longest edge, as makeEdges in kit.js)
inline std::vector<CE> MakeEdges(const Pts& poly) {
    const size_t n = poly.size();
    std::vector<CE> E;
    if (n < 2) return E;
    E.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const V2 &a = poly[i], &b = poly[(i + 1) % n];
        double L = std::hypot(b[0] - a[0], b[1] - a[1]);
        if (L <= 0) L = 1e-6;
        CE e; e.i = (int)i; e.a = a; e.b = b; e.L = L; e.u = {(b[0] - a[0]) / L, (b[1] - a[1]) / L}; e.n = {e.u[1], -e.u[0]};
        E.push_back(e);
    }
    const CE* m = &E[0];
    for (const CE& x : E) if (x.L > m->L) m = &x;
    const V2 mid{(m->a[0] + m->b[0]) / 2 + m->n[0] * .05, (m->a[1] + m->b[1]) / 2 + m->n[1] * .05};
    if (Pip(mid, poly)) for (CE& x : E) x.n = {-x.n[0], -x.n[1]};
    return E;
}
inline V2 P(const CE& e, double t, double o = 0) { return {e.a[0] + e.u[0] * t + e.n[0] * o, e.a[1] + e.u[1] * t + e.n[1] * o}; }
inline CE Shifted(const CE& e, const V2& a) { CE x = e; x.a = a; return x; }

// miter offset, d > 0 shrinks (inward). Good for small d on simple polygons.
inline Pts Inset(const Pts& poly, double d) {
    const std::vector<CE> E = MakeEdges(poly);
    const size_t n = E.size();
    Pts out;
    for (size_t i = 0; i < n; ++i) {
        const CE &e0 = E[(i + n - 1) % n], &e1 = E[i];
        const V2 p0{e0.a[0] - e0.n[0] * d, e0.a[1] - e0.n[1] * d}, p1{e1.a[0] - e1.n[0] * d, e1.a[1] - e1.n[1] * d};
        const double den = e0.u[0] * e1.u[1] - e0.u[1] * e1.u[0];
        if (std::abs(den) < 1e-3) { out.push_back(p1); continue; }
        const double t = ((p1[0] - p0[0]) * e1.u[1] - (p1[1] - p0[1]) * e1.u[0]) / den;
        V2 q{p0[0] + e0.u[0] * t, p0[1] + e0.u[1] * t};
        if (std::hypot(q[0] - p1[0], q[1] - p1[1]) > std::abs(d) * 4) q = p1; // clamp spikes on sharp corners
        out.push_back(q);
    }
    return out;
}
// inset that refuses to flip, collapse or (for d > 0) grow; empty when the polygon is too small for d
inline Pts SafeInset(const Pts& poly, double d, double minArea = 1.0) {
    Pts q = Inset(poly, d);
    if (q.size() < 3) return {};
    for (const V2& p : q) if (!std::isfinite(p[0]) || !std::isfinite(p[1])) return {};
    const double a0 = PolyArea(poly), a1 = PolyArea(q);
    if (a0 * a1 <= 0 || std::abs(a1) < minArea) return {};
    if (d > 0 && std::abs(a1) >= std::abs(a0)) return {};
    if (d < 0 && std::abs(a1) <= std::abs(a0)) return {};
    return q;
}

inline std::array<V2, 4> Uvq(double w, double h, double sc = 3) { return {V2{0, 0}, V2{w / sc, 0}, V2{w / sc, h / sc}, V2{0, h / sc}}; }

// generic oriented box: centre c=[x,z], axis u (unit), half sizes hu (along u), hn (across), y from ya to yb;
// bottom face only where visible (thin slabs / canopies), as kit.js
inline void KBox(GB& b, const V2& c, const V2& u, double hu, double hn, double ya, double yb) {
    const V2 n{-u[1], u[0]};
    auto p = [&](double su, double sn, double y) { return V3{c[0] + u[0] * hu * su + n[0] * hn * sn, y, c[1] + u[1] * hu * su + n[1] * hn * sn}; };
    const double h = yb - ya;
    b.Quad(p(-1, 1, ya), p(1, 1, ya), p(1, 1, yb), p(-1, 1, yb), {n[0], 0, n[1]}, Uvq(2 * hu, h));
    b.Quad(p(1, -1, ya), p(-1, -1, ya), p(-1, -1, yb), p(1, -1, yb), {-n[0], 0, -n[1]}, Uvq(2 * hu, h));
    b.Quad(p(1, 1, ya), p(1, -1, ya), p(1, -1, yb), p(1, 1, yb), {u[0], 0, u[1]}, Uvq(2 * hn, h));
    b.Quad(p(-1, -1, ya), p(-1, 1, ya), p(-1, 1, yb), p(-1, -1, yb), {-u[0], 0, -u[1]}, Uvq(2 * hn, h));
    b.Quad(p(-1, -1, yb), p(-1, 1, yb), p(1, 1, yb), p(1, -1, yb), {0, 1, 0}, Uvq(2 * hu, 2 * hn));
    if (h < 1.2) b.Quad(p(-1, -1, ya), p(1, -1, ya), p(1, 1, ya), p(-1, 1, ya), {0, -1, 0}, Uvq(2 * hu, 2 * hn));
}
// box in an edge frame: t along edge, o outward from the wall plane
inline void KBoxE(GB& b, const CE& e, double t0, double t1, double o0, double o1, double ya, double yb) {
    KBox(b, P(e, (t0 + t1) / 2, (o0 + o1) / 2), e.u, std::abs(t1 - t0) / 2, std::abs(o1 - o0) / 2, ya, yb);
}
// fin against a wall (front + two ends only: mullions, piers). 6 triangles instead of 10-12 for tall facades.
inline void FinE(GB& b, const CE& e, double t0, double t1, double o0, double o1, double ya, double yb) {
    auto p = [&](double t, double o, double y) { const V2 q = P(e, t, o); return V3{q[0], y, q[1]}; };
    const double h = yb - ya;
    b.Quad(p(t0, o1, ya), p(t1, o1, ya), p(t1, o1, yb), p(t0, o1, yb), {e.n[0], 0, e.n[1]}, Uvq(t1 - t0, h));
    b.Quad(p(t1, o1, ya), p(t1, o0, ya), p(t1, o0, yb), p(t1, o1, yb), {e.u[0], 0, e.u[1]}, Uvq(o1 - o0, h));
    b.Quad(p(t0, o0, ya), p(t0, o1, ya), p(t0, o1, yb), p(t0, o0, yb), {-e.u[0], 0, -e.u[1]}, Uvq(o1 - o0, h));
}
// flat quad on the wall plane (offset o outward), facing outward
inline void Panel(GB& b, const CE& e, double t0, double t1, double ya, double yb, double o = .03) {
    auto p = [&](double t, double y) { const V2 q = P(e, t, o); return V3{q[0], y, q[1]}; };
    b.Quad(p(t0, ya), p(t1, ya), p(t1, yb), p(t0, yb), {e.n[0], 0, e.n[1]}, Uvq(t1 - t0, yb - ya));
}
// filled polygon at height Y(x,z), facing up (or down with dn)
template <class YF> void Fill(GB& b, const Pts& poly, YF&& Y, bool dn = false, double sc = 3) {
    if (poly.size() < 3) return;
    const std::vector<int> tr = Triangulate(poly);
    auto V = [&](int i) { return V3{poly[i][0], Y(poly[i][0], poly[i][1]), poly[i][1]}; };
    auto U = [&](int i) { return V2{poly[i][0] / sc, poly[i][1] / sc}; };
    for (size_t k = 0; k + 2 < tr.size(); k += 3)
        b.Tri(V(tr[k]), V(tr[k + 1]), V(tr[k + 2]), {0, dn ? -1.0 : 1.0, 0}, U(tr[k]), U(tr[k + 1]), U(tr[k + 2]));
}
inline void FillY(GB& b, const Pts& poly, double y, bool dn = false) { Fill(b, poly, [y](double, double) { return y; }, dn); }
// vertical walls around a polygon from ya to yb; inward = faces toward the inside (parapet backs)
inline void Walls(GB& b, const Pts& poly, double ya, double yb, bool inward = false) {
    for (const CE& e : MakeEdges(poly)) {
        if (e.L < .05) continue;
        auto p = [&](double t, double y) { const V2 q = P(e, t, 0); return V3{q[0], y, q[1]}; };
        const double sg = inward ? -1 : 1;
        b.Quad(p(0, ya), p(e.L, ya), p(e.L, yb), p(0, yb), {e.n[0] * sg, 0, e.n[1] * sg}, Uvq(e.L, yb - ya));
    }
}

// one GB per material colour: a building is a handful of colour batches
struct Kit {
    std::map<std::string, GB> G;
    GB& gb(const std::string& hex) {
        auto it = G.find(hex);
        if (it == G.end()) it = G.emplace(hex, GB(Hex(hex))).first;
        return it->second;
    }
};

enum class Sub { Strip, BigBox, Restaurant, Bank, Office, Retail, Kiosk, Canopy };

struct C {
    const Structure& s;
    const HeightFn& hAt;
    Rng& r;
    Rng& fr;   // facts rng: seeded defaults for the facts the JS read from tiles (tenants, drive-thru, anchors)
    Kit& K;
    Pts poly;
    std::vector<CE> E;
    CE f;      // front edge
    double y0 = 0, area = 0;
    std::string name, flatRoof, trim2;
    double gy(double x, double z) const { return hAt(x, z) + .07; }
    double gy(const V2& p) const { return gy(p[0], p[1]); }
};

// ---------------- shared pieces ----------------
inline void Copings(C& c, const Pts& poly, double y) {
    GB& b = c.K.gb(COPING);
    for (const CE& e : MakeEdges(poly)) if (e.L > .3) KBoxE(b, e, -.05, e.L + .05, -.32, .06, y, y + .12);
}
// walls + parapet + coping + flat roof over poly between yBase and yTop; returns the roof height
inline double Shell(C& c, const Pts& poly, double yBase, double yTop, const std::string& wall, double parapet, const std::string& roof = {}) {
    GB& wb = c.K.gb(wall);
    Walls(wb, poly, yBase, yTop + parapet);
    const Pts ip = parapet > 0 ? SafeInset(poly, .3, .5) : Pts{};
    if (!ip.empty()) Walls(wb, ip, yTop, yTop + parapet, true);
    if (parapet > 0) Copings(c, poly, yTop + parapet);
    const Pts rp = parapet > 0 ? SafeInset(poly, .29, .5) : Pts{};
    FillY(c.K.gb(roof.empty() ? c.flatRoof : roof), rp.empty() ? poly : rp, yTop);
    return yTop;
}
inline void Rtus(C& c, const Pts& poly, double roofY, int n, double keepOut = 2.5) {
    n = std::min(n, 60);
    if (n <= 0) return;
    const Pts ip = SafeInset(poly, keepOut, 2.0);
    if (ip.empty()) return;
    double x0 = 1e18, x1 = -1e18, z0 = 1e18, z1 = -1e18;
    for (const V2& p : ip) { x0 = std::min(x0, p[0]); x1 = std::max(x1, p[0]); z0 = std::min(z0, p[1]); z1 = std::max(z1, p[1]); }
    GB &b = c.K.gb(AC), &b2 = c.K.gb("#6f7372");
    const V2 u = c.f.u;
    for (int k = 0, tries = 0; k < n && tries < n * 20; tries++) {
        const V2 p{x0 + c.r() * (x1 - x0), z0 + c.r() * (z1 - z0)};
        if (!Pip(p, ip)) continue;
        k++;
        const double w = 1.2 + c.r() * 1, d = .9 + c.r() * .5, hh = .9 + c.r() * .5;
        KBox(b, p, u, w, d, roofY, roofY + hh);
        KBox(b2, {p[0] + u[0] * w * .4, p[1] + u[1] * w * .4}, u, .45, .45, roofY + hh, roofY + hh + .12);
    }
}
// concrete sidewalk along an edge, draped on terrain
inline void Apron(C& c, const CE& e, double t0, double t1, double depth) {
    const Pts q{P(e, t0, 0), P(e, t1, 0), P(e, t1, depth), P(e, t0, depth)};
    Fill(c.K.gb(WALK), q, [&](double x, double z) { return c.gy(x, z); });
    const double ym = c.gy(P(e, (t0 + t1) / 2, depth));
    KBoxE(c.K.gb("#b7b4ab"), e, t0, t1, depth - .15, depth, ym - .1, ym + .08);
}
// sign: bg panel, a text band in fg (stands in for the canvas-atlas lettering), dark backing box
inline void Sign(C& c, const CE& e, double t0, double t1, double ya, double yb, const std::string& text, const std::string& bg,
                 const std::string& fg, double o = .12) {
    if (t1 - t0 < .2 || yb - ya < .1) return;
    Panel(c.K.gb(bg), e, t0, t1, ya, yb, o);
    const double w = t1 - t0, hh = yb - ya, tw = std::min(w * .85, std::max(1.0, (double)text.size()) * hh * .5), th = hh * .42, tm = (t0 + t1) / 2, ym = (ya + yb) / 2;
    Panel(c.K.gb(fg), e, tm - tw / 2, tm + tw / 2, ym - th / 2, ym + th / 2, o + .01);
    KBoxE(c.K.gb(SIGNBACK), e, t0 - .05, t1 + .05, .02, o - .01, ya - .05, yb + .05);
}
// aluminium storefront: glass, mullions, door
inline void Storefront(C& c, const CE& e, double t0, double t1, double ya, double yb, bool door = true) {
    const double w = t1 - t0;
    if (w < .4 || yb - ya < 1) return;
    GB &gl = c.K.gb(GLASS), &mu = c.K.gb(MULL);
    Panel(gl, e, t0, t1, ya + .35, yb, .04);
    KBoxE(mu, e, t0, t1, 0, .08, ya, ya + .35);
    const int nm = std::max(1, (int)std::lround(w / 1.6));
    for (int k = 0; k <= nm; k++) { const double t = t0 + w * k / nm; KBoxE(mu, e, t - .04, t + .04, .02, .1, ya, yb); }
    KBoxE(mu, e, t0, t1, .02, .1, yb - .1, yb);
    KBoxE(mu, e, t0, t1, .02, .1, ya + 2.3, ya + 2.38);
    if (door) {
        const double tm = t0 + w * (c.r() < .5 ? .5 : .25);
        KBoxE(mu, e, tm - .95, tm + .95, .02, .12, ya + 2.18, ya + 2.3);
        KBoxE(mu, e, tm - .03, tm + .03, .02, .12, ya, ya + 2.2);
    }
}
// punched windows along an edge (skipping [s0,s1] along it, e.g. where a storefront goes)
inline void WindowsRow(C& c, const CE& e, double ya, double yb, double step, double wWin, const std::string& frame = "#2f3236",
                       double s0 = 1, double s1 = 0) {
    const int n = (int)std::floor((e.L - 1.5) / step);
    if (n < 1) return;
    GB &gl = c.K.gb(GLASS), &fb = c.K.gb(frame);
    const double t0 = (e.L - n * step) / 2;
    for (int k = 0; k < n; k++) {
        const double t = t0 + step * (k + .5);
        if (s1 > s0 && t + wWin / 2 > s0 && t - wWin / 2 < s1) continue;
        Panel(gl, e, t - wWin / 2, t + wWin / 2, ya, yb, .04);
        KBoxE(fb, e, t - wWin / 2 - .06, t + wWin / 2 + .06, 0, .09, ya - .08, ya);
        KBoxE(fb, e, t - wWin / 2 - .06, t + wWin / 2 + .06, 0, .09, yb, yb + .06);
    }
}
inline void ServiceDoors(C& c, const CE& e, int n, double y0) {
    GB& b = c.K.gb("#7d8085");
    for (int k = 0; k < n; k++) {
        const double t = e.L * (k + .5) / n + (c.r() - .5) * 2;
        if (t < 1.5 || t > e.L - 1.5) continue;
        Panel(b, e, t - .5, t + .5, y0, y0 + 2.15, .05);
    }
}
inline const CE& BackOf(const std::vector<CE>& E, const CE& f) {
    const CE* m = &E[0];
    auto d = [&](const CE& e) { return e.n[0] * f.n[0] + e.n[1] * f.n[1]; };
    for (const CE& e : E) if (d(e) < d(*m) || (d(e) == d(*m) && e.L > m->L)) m = &e;
    return *m;
}
inline const CE& SideOf(const std::vector<CE>& E, const CE& f, double dir) {
    auto sc = [&](const CE& x) { return (x.n[0] * -f.u[1] * dir + x.n[1] * f.u[0] * dir) * x.L; };
    const CE* m = nullptr;
    for (const CE& e : E) if (e.i != f.i && (!m || sc(e) > sc(*m))) m = &e;
    return m ? *m : f;
}
inline void Pylon(C& c, const V2& p, const std::string& text, const std::string& bg, const std::string& fg, double ht) {
    const CE& f = c.f;
    const double base = c.gy(p) - .05;
    KBox(c.K.gb("#4a4d50"), p, f.u, .18, .18, base, base + ht - 1.4);
    const double sw = std::min(3.6, text.size() * .3 + 1.2);
    const V2 n{-f.u[1], f.u[0]};
    GB &b = c.K.gb(bg), &tb = c.K.gb(fg);
    for (const double sd : {1.0, -1.0}) {
        auto q = [&](double su, double y, double off) { return V3{p[0] + f.u[0] * su + n[0] * off * sd, y, p[1] + f.u[1] * su + n[1] * off * sd}; };
        const V3 nn{n[0] * sd, 0, n[1] * sd};
        const double ya = base + ht - 1.4, yb = base + ht, tw = std::min(sw * .8, text.size() * .28 + .3) / 2;
        b.Quad(q(-sw / 2 * sd, ya, .2), q(sw / 2 * sd, ya, .2), q(sw / 2 * sd, yb, .2), q(-sw / 2 * sd, yb, .2), nn);
        tb.Quad(q(-tw, ya + .45, .21), q(tw, ya + .45, .21), q(tw, yb - .45, .21), q(-tw, yb - .45, .21), nn);
    }
    KBox(c.K.gb(SIGNBACK), p, f.u, sw / 2 + .08, .18, base + ht - 1.5, base + ht + .1);
}
inline void DriveThru(C& c, const CE& e, bool bankStyle) {
    const double y0 = c.y0;
    if (bankStyle) { // bank drive-up canopy with 2-3 lanes and teller/ATM pedestals
        const int nl = 2 + (c.r() < .5 ? 1 : 0);
        const double D = 3.6 * nl + .6, t0 = std::max(0.0, e.L / 2 - 4), t1 = std::min(e.L, e.L / 2 + 4), ch = y0 + 4.4;
        const std::string cm = c.trim2.empty() ? std::string("#e3dcc8") : c.trim2;
        KBoxE(c.K.gb(cm), e, t0 - .5, t1 + .5, 0, D, ch, ch + .8);
        KBoxE(c.K.gb(SOFFIT), e, t0 - .5, t1 + .5, 0, D, ch - .02, ch);
        GB &cl = c.K.gb(cm), &ped = c.K.gb("#2e4a7a"), &isl = c.K.gb(COPING);
        for (int k = 1; k <= nl; k++) {
            const double o = k * 3.6 + .3 - 3.6;
            for (const double t : {t0, t1}) KBoxE(cl, e, t - .25, t + .25, o + 3.4, o + 3.8, y0, ch);
            const double yi = c.gy(P(e, e.L / 2, o + 3.6));
            KBoxE(isl, e, t0 + 1, t1 - 1, o + 3.35, o + 3.85, yi - .05, yi + .15);
            KBoxE(ped, e, e.L / 2 - .35, e.L / 2 + .35, o + 3.4, o + 3.8, y0, y0 + 1.6);
        }
        return;
    }
    const double t = e.L * .5;
    GB& wb = c.K.gb("#3a3d40");
    KBoxE(wb, e, t - 1.1, t + 1.1, 0, .5, y0 + .9, y0 + 1.0);
    Panel(c.K.gb(GLASS), e, t - .9, t + .9, y0 + 1, y0 + 2.2, .06);
    KBoxE(c.K.gb(PickS(c.r, {"#b3121b", "#3c3d3f", "#e25c12"})), e, t - 1.3, t + 1.3, 0, 1.0, y0 + 2.6, y0 + 2.9);
    // menu board out along the lane
    const V2 p = P(e, e.L + 6, 4.5);
    const double base = c.gy(p);
    KBox(c.K.gb(SIGNBACK), p, e.u, .1, .1, base, base + 1);
    KBox(c.K.gb(SIGNBACK), p, e.u, .9, .1, base + 1, base + 2.2);
    const CE me = Shifted(e, {p[0] - e.u[0] * .85 + e.n[0] * .11, p[1] - e.u[1] * .85 + e.n[1] * .11});
    Panel(c.K.gb("#1c1c1c"), me, 0, 1.7, base + 1.05, base + 2.15, 0);
    Panel(c.K.gb("#f4c430"), me, .3, 1.4, base + 1.7, base + 1.95, .01);
}
// mansard band from ring ob (at yA) up to ring ib (at yT); both rings come from Inset so indices match
inline void Mansard(GB& rb, const Pts& ob, const Pts& ib, double yA, double yT) {
    const size_t n = std::min(ob.size(), ib.size());
    for (size_t i = 0; i < n; i++) {
        const size_t j = (i + 1) % n;
        const V3 a{ob[i][0], yA, ob[i][1]}, b{ob[j][0], yA, ob[j][1]}, cc{ib[j][0], yT, ib[j][1]}, d{ib[i][0], yT, ib[i][1]};
        rb.UpTri(a, b, cc, {0, 0}, {1, 0}, {1, 1});
        rb.UpTri(a, cc, d, {0, 0}, {1, 1}, {0, 1});
    }
}

// ---------------- subtypes ----------------
inline std::pair<std::string, std::string> WallsPick(C& c) {
    const std::string b = PickA(c.r, BRICK), s1 = PickA(c.r, STUCCO3), s2 = PickS(c.r, {"#b9ae98", "#a89f8e", "#c7bfae", "#8f8a80"});
    const double k = std::floor(c.r() * 3);
    return k < 1 ? std::make_pair(std::string("brick"), b) : k < 2 ? std::make_pair(std::string("stucco"), s1) : std::make_pair(std::string("stucco"), s2);
}

inline void Strip(C& c) {
    Rng& r = c.r;
    const CE& f = c.f;
    const double y0 = c.y0, h = Clamp(c.s.height > 0 ? c.s.height : 6, 4.8, 8.5);
    const auto wk = WallsPick(c);
    const std::string wm = wk.second;
    const double roofY = Shell(c, c.poly, y0, y0 + h, wm, .9);
    // tenants: one bay per ~8 m with generic signage (guessed)
    const double L = f.L;
    const int nb = (int)Clamp((double)std::lround(L / (7 + r() * 3)), 2, 18);
    std::vector<std::string> names;
    for (int k = 0; k < nb; k++) names.push_back(k == 0 && !c.name.empty() ? c.name : PickA(r, GENERIC));
    const double bw = L / nb, canH = y0 + 3.4, canD = 2.6 + r() * .8, fasc = .75;
    const std::string pier = PickA(r, BRICK);
    GB& pb = c.K.gb(pier);
    for (int k = 0; k < nb; k++) {
        const double t0 = k * bw, t1 = t0 + bw;
        KBoxE(pb, f, t0 - .3, t0 + .3, 0, .25, y0, canH);
        Storefront(c, f, t0 + .35, t1 - .35, y0, canH - .2, true);
        const Col2 col = BrandOf(names[k], r);
        const double sw = std::min(bw * .8, names[k].size() * .55 + 2.5);
        Sign(c, f, (t0 + t1) / 2 - sw / 2, (t0 + t1) / 2 + sw / 2, canH + fasc + .15, std::min(canH + fasc + 1.05, roofY + .75), names[k], col.first, col.second);
    }
    KBoxE(pb, f, L - .3, L + .3, 0, .25, y0, canH);
    // continuous canopy with fascia and columns
    const std::string fm = PickS(r, {"#5a4a3c", "#3c3d3f", "#7a2e27", "#3f4c3c", "#cfc8b8", "#2b2b2c"});
    KBoxE(c.K.gb(fm), f, -.2, L + .2, canD - .25, canD, canH, canH + fasc);
    KBoxE(c.K.gb(SOFFIT), f, -.2, L + .2, 0, canD, canH, canH + .12);
    KBoxE(c.K.gb(fm), f, -.2, L + .2, 0, canD, canH + fasc - .1, canH + fasc);
    for (int k = 0; k <= nb; k++) KBoxE(pb, f, k * bw - .22, k * bw + .22, canD - .7, canD - .26, y0, canH);
    // raised parapet "tower" over the centre or the end bays
    const double tw = std::min(L * .2, 12.0), tc = r() < .5 ? L / 2 : (r() < .5 ? tw / 2 + 1 : L - tw / 2 - 1);
    KBoxE(c.K.gb(wm), f, tc - tw / 2, tc + tw / 2, -.3, .35, roofY, roofY + 2.4);
    KBoxE(c.K.gb(COPING), f, tc - tw / 2 - .1, tc + tw / 2 + .1, -.4, .45, roofY + 2.4, roofY + 2.55);
    Apron(c, f, -1, L + 1, canD + 1.2);
    ServiceDoors(c, BackOf(c.E, f), nb, y0);
    Rtus(c, c.poly, roofY, (int)std::lround(c.area / 250));
}

inline void BigBox(C& c) {
    Rng& r = c.r;
    const CE& f = c.f;
    const double y0 = c.y0, h = Clamp(c.s.height > 0 ? c.s.height : 9, 7.5, 13);
    const std::string wm = PickS(r, {"#c2b49a", "#b8b2a3", "#d6c9ad", "#a7a9a6", "#cfc6b4", "#8f8a80"});
    const double roofY = Shell(c, c.poly, y0, y0 + h, wm, 1.2);
    // horizontal accent band (split-face block banding)
    GB& bb = c.K.gb(PickS(r, {"#7d7a72", "#9c8a6e", "#6a6c6c"}));
    for (const CE& e : c.E) if (e.L > 3) KBoxE(bb, e, 0, e.L, 0, .06, y0 + 1.1, y0 + 1.7);
    // entrance portal(s): anchors sharing the box (seeded: one anchor, or as many as the front allows)
    const int cap = (int)Clamp((double)std::lround(f.L / 45), 1, 4), na = c.fr() < .5 ? 1 : cap;
    std::vector<std::string> names{c.name.empty() ? std::string("SUPERSTORE") : c.name};
    static const char* const ANCH[] = {"HOME GOODS", "FOODS", "OUTLET", "SPORTS", "FURNITURE", "PETS", "CRAFTS"};
    while ((int)names.size() < na) names.push_back(PickA(c.fr, ANCH));
    const double seg = f.L / names.size();
    for (size_t k = 0; k < names.size(); k++) {
        const std::string& nm = names[k];
        const double tc = seg * (k + .5), pw = std::min(seg * .55, 22.0), ph = h + 2.5 + r() * 2.5;
        const Col2 col = BrandOf(nm, r);
        GB& pbx = c.K.gb(PickS(r, {"#e3dcc8", "#d9cbb0", "#ebe5d3", "#c9c3b5"}));
        KBoxE(pbx, f, tc - pw / 2, tc + pw / 2, -.3, 2.4, y0, y0 + ph);
        KBoxE(c.K.gb(COPING), f, tc - pw / 2 - .1, tc + pw / 2 + .1, -.3, 2.5, y0 + ph, y0 + ph + .15);
        Panel(c.K.gb(GLASS), f, tc - pw * .38, tc + pw * .38, y0 + .2, y0 + 4.2, 2.44);
        GB& mu = c.K.gb(MULL);
        for (int q = 0; q <= 6; q++) { const double t = tc - pw * .38 + pw * .76 * q / 6; KBoxE(mu, f, t - .05, t + .05, 2.4, 2.52, y0, y0 + 4.2); }
        KBoxE(mu, f, tc - pw * .38, tc + pw * .38, 2.4, 2.52, y0 + 4.1, y0 + 4.3);
        const double sw = std::min(pw * .9, nm.size() * 1.0 + 4);
        Sign(c, f, tc - sw / 2, tc + sw / 2, y0 + h - .4, y0 + std::min(ph - .3, h + 2.2), nm, col.first, col.second, 2.52);
        // cart corral / bollards
        GB& bl = c.K.gb("#e0b21a");
        for (int q = -2; q <= 2; q++) { const V2 p = P(f, tc + q * 2.2, 5.5); KBox(bl, p, f.u, .13, .13, c.gy(p), c.gy(p) + 1); }
    }
    Apron(c, f, -1, f.L + 1, 6);
    // loading docks on the rear
    const CE& back = BackOf(c.E, f);
    GB &db = c.K.gb("#9a9c9e"), &bp = c.K.gb("#1d1d1d"), &seal = c.K.gb("#2b2b2c");
    const int nd = (int)Clamp((double)std::lround(back.L / 25), 2, 8);
    const double t0 = back.L * (.15 + r() * .3);
    for (int k = 0; k < nd && t0 + k * 4.5 < back.L - 3; k++) {
        const double t = t0 + k * 4.5;
        Panel(db, back, t - 1.4, t + 1.4, y0 + 1.2, y0 + 4.2, .05);
        KBoxE(bp, back, t - 1.6, t - 1.3, 0, .2, y0 + 1, y0 + 1.5);
        KBoxE(bp, back, t + 1.3, t + 1.6, 0, .2, y0 + 1, y0 + 1.5);
        KBoxE(seal, back, t - 1.7, t + 1.7, 0, .35, y0 + 4.2, y0 + 4.6);
    }
    ServiceDoors(c, SideOf(c.E, f, 1), 2, y0);
    ServiceDoors(c, SideOf(c.E, f, -1), 2, y0);
    Rtus(c, c.poly, roofY, std::min(60, (int)std::lround(c.area / 500)), 4);
}

inline bool FastFoodName(const std::string& l) {
    static const char* const K[] = {"mcdonald", "burger", "wendy", "taco", "kfc", "arby", "dunkin", "subway", "popeye", "chick",
                                    "coffee", "starbucks", "tim horton", "hungry", "pizza", "domino", nullptr};
    return HasAny(l, K);
}

inline void Restaurant(C& c) {
    Rng& r = c.r;
    const CE& f = c.f;
    const double y0 = c.y0;
    const bool fast = FastFoodName(Lower(c.name)) || c.fr() < .45; // drive-thru: seeded default when the fact is missing
    const double h = Clamp(c.s.height > 0 ? c.s.height : 4.6, 3.8, 6.5);
    const std::string b = PickA(r, BRICK), st = PickS(r, {"#d9cbb0", "#c2b49a", "#e3dcc8", "#b8b2a3"});
    const std::string wm = r() < .5 ? b : st;
    const bool small = c.area < 900;
    // mansard / sloped fascia roof band (very common on US pad restaurants), else flat parapet
    bool mans = small && r() < .7;
    const double roofY = y0 + h;
    GB& wb = c.K.gb(wm);
    Walls(wb, c.poly, y0, roofY);
    for (const CE& e : c.E) if (e.L > 2.5) WindowsRow(c, e, y0 + .9, y0 + 2.6, 2.4 + r() * .6, 1.7);
    double yT = roofY;
    if (mans) {
        const bool metal = r() < .5;
        const std::string rm = metal ? PickA(r, METAL) : PickA(r, ASPHALT);
        const Pts ob = SafeInset(c.poly, -.6), ib = SafeInset(c.poly, 1.0, 4.0);
        if (ob.empty() || ib.empty() || ob.size() != ib.size()) mans = false;
        else {
            yT = roofY + 1.3;
            Mansard(c.K.gb(rm), ob, ib, roofY - .2, yT);
            FillY(c.K.gb(c.flatRoof), ib, yT);
            FillY(c.K.gb(SOFFIT), ob, roofY - .2, true);
        }
    }
    if (!mans) {
        Walls(wb, c.poly, roofY, roofY + 1.0);
        const Pts ip = SafeInset(c.poly, .3, .5), rp = SafeInset(c.poly, .29, .5);
        if (!ip.empty()) Walls(wb, ip, roofY, roofY + 1.0, true);
        Copings(c, c.poly, roofY + 1.0);
        FillY(c.K.gb(c.flatRoof), rp.empty() ? c.poly : rp, roofY);
        // coloured fascia stripe (brand band)
        const Col2 bc = BrandOf(c.name, r);
        GB& band = c.K.gb(bc.first);
        for (const CE& e : c.E) if (e.L > 1) KBoxE(band, e, 0, e.L, 0, .05, roofY + .2, roofY + .55);
    }
    // entry vestibule + sign
    const Col2 col = BrandOf(c.name, r);
    std::string nm = c.name.empty() ? PickS(r, {"GRILL", "DINER", "FAMILY RESTAURANT", "PIZZA", "SUBS"}) : c.name;
    const double tc = f.L * (.35 + r() * .3), vw = 3.2;
    KBoxE(c.K.gb(wm), f, tc - vw / 2, tc + vw / 2, 0, 2.2, y0, y0 + 3.3);
    Storefront(c, Shifted(f, P(f, 0, 2.2)), tc - vw / 2 + .3, tc + vw / 2 - .3, y0, y0 + 2.8, true);
    const double half = std::min(f.L * .4, nm.size() * .45 + 1.5);
    Sign(c, f, std::max(.5, tc - half), std::min(f.L - .5, tc + half), roofY + (mans ? .1 : .3), roofY + (mans ? 1.1 : .95), nm, col.first, col.second, mans ? .9 : .12);
    Apron(c, f, -1, f.L + 1, 3.6);
    // pylon sign at the front corner
    Pylon(c, P(f, -4, 9), nm, col.first, col.second, fast ? 7.5 : 5);
    if (fast) DriveThru(c, SideOf(c.E, f, r() < .5 ? 1 : -1), false);
    ServiceDoors(c, BackOf(c.E, f), 1, y0);
    Rtus(c, mans ? Pts(SafeInset(c.poly, 1.0, 4.0)) : c.poly, yT, std::max(1, (int)std::lround(c.area / 180)), mans ? 1 : 1.5);
}

inline void Bank(C& c) {
    Rng& r = c.r;
    const CE& f = c.f;
    const double y0 = c.y0, h = Clamp(c.s.height > 0 ? c.s.height : 5.5, 4.5, 9);
    const std::string wm = PickA(r, BRICK), trim = PickS(r, {"#ebe5d3", "#e3dcc8", "#f1f0eb"});
    c.trim2 = trim;
    const double roofY = y0 + h;
    Walls(c.K.gb(wm), c.poly, y0, roofY);
    for (const CE& e : c.E) if (e.L > 3) WindowsRow(c, e, y0 + .9, y0 + std::min(h - .8, 3.4), 3, 1.6, trim, f.L / 2 - 2, f.L / 2 + 2);
    // cornice + standing-seam mansard (banks along suburban arterials)
    GB& tb = c.K.gb(trim);
    for (const CE& e : c.E) if (e.L > .5) KBoxE(tb, e, -.15, e.L + .15, -.05, .35, roofY - .5, roofY);
    const std::string rm = PickA(r, METAL_ASPH4);
    const Pts ob = SafeInset(c.poly, -.4);
    Pts ib = SafeInset(c.poly, 2.2, 4.0);
    double rise = 2.2;
    if (ib.empty()) { ib = SafeInset(c.poly, 1.0, 2.0); rise = 1.2; }
    if (!ob.empty() && !ib.empty() && ob.size() == ib.size()) {
        Mansard(c.K.gb(rm), ob, ib, roofY, roofY + rise);
        FillY(c.K.gb(c.flatRoof), ib, roofY + rise);
        FillY(c.K.gb(SOFFIT), ob, roofY, true);
    } else FillY(c.K.gb(c.flatRoof), c.poly, roofY);
    // entrance: columned portico
    const double tc = f.L / 2, pw = std::min(6.0, f.L * .4);
    KBoxE(tb, f, tc - pw / 2, tc + pw / 2, 0, 2.6, y0 + 3.6, y0 + 4.3);
    for (const double t : {tc - pw / 2 + .3, tc + pw / 2 - .3}) KBox(tb, P(f, t, 2.3), f.u, .22, .22, y0, y0 + 3.6);
    Storefront(c, f, tc - 1.4, tc + 1.4, y0, y0 + 2.9, true);
    const Col2 col = BrandOf(c.name, r);
    const std::string nm = c.name.empty() ? std::string("BANK") : c.name;
    const double sw = std::min(f.L * .7, nm.size() * .45 + 1.5);
    Sign(c, f, tc - sw / 2, tc + sw / 2, y0 + 4.5, y0 + 5.2, nm, col.first, col.second, 2.62);
    Apron(c, f, -1, f.L + 1, 4);
    Pylon(c, P(f, -4, 10), nm, col.first, col.second, 4.5);
    DriveThru(c, SideOf(c.E, f, r() < .5 ? 1 : -1), true);
}

// Mid/high-rise facade, cheap enough for thousands of towers: per floor one ribbon (or spandrel) quad per edge, and
// full-height piers / mullions per bay. Triangles ~ 2*floors*edges + 6*bays*edges, independent of window count.
inline void TallFacade(C& c, const Pts& poly, double yBase, int k0, int k1, double fh, const std::string& style,
                       const std::string& wall, const std::string& glass, const std::string& spandrel, bool lobby) {
    const std::vector<CE> E = MakeEdges(poly);
    const double yTopFl = yBase + (k1 - k0) * fh;
    for (const CE& e : E) {
        if (e.L <= 3) continue;
        const bool front = lobby && e.i == c.f.i;
        const double yStart = front ? yBase + fh : yBase;
        if (yTopFl - yStart < 1) continue;
        if (style == "glass") { // curtain wall: glass, floor-line spandrels, vertical mullions
            GB &gl = c.K.gb(glass), &sp = c.K.gb(spandrel), &mu = c.K.gb(MULL);
            Panel(gl, e, .3, e.L - .3, yStart + .05, yTopFl, .04);
            for (int k = k0 + 1; k < k1; k++) {
                const double yl = yBase + (k - k0) * fh;
                if (yl <= yStart + .3) continue;
                Panel(sp, e, .3, e.L - .3, yl - .55, yl + .35, .06);
            }
            const int n = std::max(1, (int)std::floor((e.L - .6) / std::max(1.5, (e.L - .6) / 24)));
            const double st = (e.L - .6) / n;
            for (int q = 0; q <= n; q++) { const double t = .3 + st * q; FinE(mu, e, t - .05, t + .05, .02, .14, yStart, yTopFl); }
        } else { // masonry / precast grid: ribbon per floor behind full-height piers = punched windows
            const double step0 = style == "brick" ? 2.6 + c.r() * .6 : 2.8 + c.r() * .8;
            const double pw = style == "brick" ? .9 : .7;
            const int n = (int)std::floor((e.L - .8) / std::max(step0, (e.L - .8) / 30));
            if (n < 1) continue;
            const double st = (e.L - .8) / n, t0 = .4;
            GB &gl = c.K.gb(glass), &pb = c.K.gb(wall);
            for (int k = k0; k < k1; k++) {
                const double fb = yBase + (k - k0) * fh;
                if (fb < yStart - .01) continue;
                Panel(gl, e, t0, t0 + n * st, fb + fh * .27, fb + fh * .78, .04);
            }
            for (int q = 0; q <= n; q++) { const double t = t0 + st * q; FinE(pb, e, t - pw / 2, t + pw / 2, 0, .14, yStart, yTopFl); }
        }
    }
}

inline void Office(C& c) {
    const Structure& s = c.s;
    Rng& r = c.r;
    const CE& f = c.f;
    const double y0 = c.y0;
    int fl = s.floors > 0 ? (int)std::lround(Clamp(s.floors, 1, 200)) : std::max(1, (int)std::lround((s.height > 0 ? std::min(s.height, 800.0) : 4.5) / 3.9));
    double fh = 3.9;
    if (s.floors > 0 && s.height > 0) fh = Clamp((s.height - .4) / fl, 3.0, 4.8);
    const double h = std::max(std::min(s.height, 1000.0), fl * fh + .4);
    const bool tall = fl >= 2; // 2+ storeys: grid facade (per-window boxes cost ~26 tris each)
    const std::string style = fl < 12 ? PickS(r, {"brick", "brick", "stucco", "glass"}) : PickS(r, {"glass", "glass", "glass", "precast", "precast"});
    std::string wm;
    if (style == "brick") wm = PickA(r, BRICK);
    else if (style == "precast") wm = PickS(r, {"#d9d4c7", "#bfb8a8", "#a7a9a6", "#8f8a80", "#e3dcc8"});
    else if (style == "glass" && fl >= 12) wm = PickS(r, {"#5c636a", "#3f454b", "#8c9196", "#b9bcbc"}); // aluminium panel corners
    else wm = PickS(r, {"#d9cbb0", "#c2b49a", "#b8b2a3", "#e3dcc8", "#a7a9a6"});
    const double tc = f.L * (r() < .6 ? .5 : .3);
    Pts top = c.poly;
    double roofY;
    if (!tall) { // low-rise, as commercial.js
        roofY = Shell(c, c.poly, y0, y0 + h, wm, .8);
        for (int k = 0; k < fl; k++) {
            const double ya = y0 + k * fh + 1.0, yb = ya + 1.8;
            for (const CE& e : c.E) {
                if (e.L <= 3) continue;
                const bool skip = k == 0 && e.i == f.i;
                if (style == "glass") {
                    if (skip) {
                        Panel(c.K.gb(GLASS), e, .5, std::max(.5, tc - 2), ya - .3, yb + .5, .04);
                        Panel(c.K.gb(GLASS), e, std::min(e.L - .5, tc + 2), e.L - .5, ya - .3, yb + .5, .04);
                    } else Panel(c.K.gb(GLASS), e, .5, e.L - .5, ya - .3, yb + .5, .04);
                    GB& mu = c.K.gb(MULL);
                    for (double t = .5; t <= e.L - .5; t += 1.5) if (!skip || std::abs(t - tc) > 2) KBoxE(mu, e, t - .04, t + .04, .02, .1, ya - .3, yb + .5);
                } else WindowsRow(c, e, ya, yb, 2.6 + r() * .6, style == "brick" ? 1.6 : 2.2, "#2f3236", skip ? tc - 2 : 1, skip ? tc + 2 : 0);
            }
        }
        Storefront(c, f, tc - 1.6, tc + 1.6, y0, y0 + 2.8, true);
    } else { // mid/high-rise: optional podium + setback tower, curtain wall or punched grid
        const std::string glass = PickS(r, {"#2e3e4c", "#3a5068", "#2f4a52", "#4a5a66", "#28323c"});
        const std::string spandrel = PickS(r, {"#3c4650", "#55606a", "#22282e", "#6a7178"});
        int pf = 0;
        if (fl >= 15 && r() < .5) {
            const Frame F = MakeFrame(c.poly);
            const double d = Clamp(F.W * .12, 2, 6);
            Pts q = SafeInset(c.poly, d, std::abs(PolyArea(c.poly)) * .35);
            if (!q.empty() && q.size() == c.poly.size()) { top = q; pf = 2 + (int)std::floor(r() * 3); }
        }
        if (pf > 0) {
            const std::string pwall = style == "brick" ? wm : PickS(r, {"#d9d4c7", "#bfb8a8", "#c2b49a", "#8f8a80"});
            Shell(c, c.poly, y0, y0 + pf * fh, pwall, .8);
            TallFacade(c, c.poly, y0, 0, pf, fh, style == "glass" ? "precast" : style, pwall, glass, spandrel, true);
            const double yb = y0 + pf * fh;
            roofY = Shell(c, top, yb, y0 + h, wm, .8);
            TallFacade(c, top, yb, pf, fl, fh, style, wm, glass, spandrel, false);
        } else {
            roofY = Shell(c, c.poly, y0, y0 + h, wm, .8);
            TallFacade(c, c.poly, y0, 0, fl, fh, style, wm, glass, spandrel, true);
        }
        // lobby across the middle of the front
        const double w = Clamp(f.L * .45, 4, 24), lc = Clamp(tc, w / 2 + .5, std::max(w / 2 + .5, f.L - w / 2 - .5));
        Storefront(c, f, lc - w / 2, lc + w / 2, y0, y0 + std::min(fh - .4, 4.6), true);
        // crown: mechanical penthouse, antenna on the tallest
        const Frame F = MakeFrame(top);
        const double hl = F.L * .28, hw = F.W * .28;
        if (fl >= 5 && hl > 1.5 && hw > 1.5 && Pip({F.cx, F.cz}, top)) {
            const double ph = 3.5 + r() * 2.5;
            KBox(c.K.gb(style == "glass" ? spandrel : wm), {F.cx, F.cz}, {F.ux, F.uz}, hl, hw, roofY, roofY + ph);
            if (fl >= 25 && r() < .4) KBox(c.K.gb("#8a8d90"), {F.cx, F.cz}, {F.ux, F.uz}, .25, .25, roofY + ph, roofY + ph + 6 + r() * 14);
        }
    }
    const std::string cm = PickS(r, {"#3c3d3f", "#5a4a3c", "#c9c7c0"});
    const double ey = tall ? y0 + std::min(fh - .4, 4.6) + .2 : y0 + 3.05, ew = tall ? 3.2 : 2.2;
    KBoxE(c.K.gb(cm), f, tc - ew, tc + ew, 0, 2.2, ey, ey + .3);
    if (!c.name.empty()) {
        const double sw = std::min(f.L * .5, c.name.size() * .35 + 1);
        Sign(c, f, tc - sw / 2, tc + sw / 2, ey + .45, ey + 1.0, c.name, "#f2f2f2", "#222222");
        // monument sign (tenant directory) near the road
        const V2 p = P(f, f.L / 2, 10);
        const double b = c.gy(p) - .05;
        KBox(c.K.gb(wm), p, f.u, 1.4, .3, b, b + .5);
        KBox(c.K.gb(SIGNBACK), p, f.u, 1.25, .2, b + .5, b + 1.6);
        const CE me = Shifted(f, {p[0] - f.u[0] * 1.2 + f.n[0] * .21, p[1] - f.u[1] * 1.2 + f.n[1] * .21});
        Panel(c.K.gb(SIGNBACK), me, 0, 2.4, b + .55, b + 1.55, 0);
        Panel(c.K.gb("#ffffff"), me, .4, 2.0, b + .9, b + 1.2, .01);
    }
    Apron(c, f, -1, f.L + 1, tall ? 4 : 2.5);
    const double topA = std::abs(PolyArea(top));
    Rtus(c, top, roofY, fl >= 5 ? std::min(6, (int)std::lround(topA / 400)) : std::max(1, (int)std::lround(c.area / 300)));
}

inline void Retail(C& c) { // single-tenant store / small commercial box
    Rng& r = c.r;
    const CE& f = c.f;
    const double y0 = c.y0, h = Clamp(c.s.height > 0 ? c.s.height : 5, 4.2, 9);
    const auto wk = WallsPick(c);
    const double roofY = Shell(c, c.poly, y0, y0 + h, wk.second, .9);
    const double w = std::min(f.L - 1, std::max(4.0, f.L * .55)), tc = f.L / 2;
    Storefront(c, f, tc - w / 2, tc + w / 2, y0, y0 + 3, true);
    KBoxE(c.K.gb(PickS(r, {"#7a2e27", "#3f4c3c", "#2b2b2c", "#3e4a5a"})), f, tc - w / 2 - .2, tc + w / 2 + .2, 0, 1.2, y0 + 3.1, y0 + 3.45);
    const std::string nm = c.name.empty() ? PickA(r, GENERIC) : c.name;
    const Col2 col = BrandOf(nm, r);
    const double sw = std::min(f.L * .8, nm.size() * .45 + 1.5);
    Sign(c, f, tc - sw / 2, tc + sw / 2, y0 + 3.6, std::min(y0 + 4.5, roofY + .7), nm, col.first, col.second);
    Apron(c, f, -1, f.L + 1, 3);
    ServiceDoors(c, BackOf(c.E, f), 1, y0);
    Rtus(c, c.poly, roofY, std::max(1, (int)std::lround(c.area / 250)), 1.5);
}

inline bool GasName(const std::string& l) {
    static const char* const K[] = {"speedway", "mobil", "marathon", "shell", "\x01" "bp", "sunoco", "citgo", "exxon", "chevron",
                                    "texaco", "valero", "phillips 66", "circle k", "7-eleven", "wawa", "sheetz", "casey", "gas", "fuel", nullptr};
    return HasAny(l, K);
}

inline void Kiosk(C& c) { // gas-station convenience store: glass front, brand fascia
    Rng& r = c.r;
    const CE& f = c.f;
    const double y0 = c.y0, h = Clamp(c.s.height > 0 ? c.s.height : 4.5, 4, 6);
    const std::string wm = PickS(r, {"#e3dcc8", "#d9cbb0", "#c2b49a", "#ebe5d3"});
    const double roofY = Shell(c, c.poly, y0, y0 + h, wm, .9);
    Storefront(c, f, .6, f.L - .6, y0, y0 + 3, true);
    const Col2 col = BrandOf(c.name, r);
    GB& band = c.K.gb(col.first);
    for (const CE& e : c.E) if (e.L > 1) KBoxE(band, e, -.05, e.L + .05, 0, .08, roofY + .1, roofY + .7);
    const std::string nm = c.name.empty() || GasName(Lower(c.name)) ? std::string("FOOD MART") : c.name;
    const double sw = std::min(f.L * .6, nm.size() * .45 + 1.5);
    Sign(c, f, f.L / 2 - sw / 2, f.L / 2 + sw / 2, y0 + 3.15, y0 + 3.85, nm, "#ffffff", col.first);
    KBoxE(c.K.gb(SOFFIT), f, 0, f.L, 0, 1.6, y0 + 3.1, y0 + 3.25);
    KBoxE(c.K.gb("#e9eef2"), f, f.L - 2.2, f.L - .8, .1, .9, y0, y0 + 1.9);
    Apron(c, f, -1, f.L + 1, 3);
    ServiceDoors(c, BackOf(c.E, f), 1, y0);
    Rtus(c, c.poly, roofY, 2, 1.2);
}

inline void Canopy(C& c) { // fuel canopy: footprint = canopy roof; columns + pump islands under it
    Rng& r = c.r;
    const Pts& poly = c.poly;
    const double yg = MinGround(poly, c.hAt), ch = yg + 4.9, ft = Clamp((c.s.height > 0 ? c.s.height : 5.8) - 4.9, .8, 1.2);
    const Col2 col = BrandOf(c.name, r);
    const std::string& bg = col.first;
    // fascia: brand colour band with a white lower stripe, white soffit with recessed lights
    for (const CE& e : c.E)
        if (e.L > .3) {
            KBoxE(c.K.gb(bg), e, -.02, e.L + .02, -.25, 0, ch + .25, ch + ft);
            KBoxE(c.K.gb("#f4f4f2"), e, -.02, e.L + .02, -.25, 0, ch, ch + .25);
        }
    const Pts ip = SafeInset(poly, .2, .5);
    FillY(c.K.gb("#f2f2ef"), ip.empty() ? poly : ip, ch + .02, true);
    FillY(c.K.gb("#c9c9c6"), ip.empty() ? poly : ip, ch + ft - .1);
    // long axis of the canopy (pump islands run across it)
    const CE* Lp = &c.E[0];
    for (const CE& e : c.E) if (e.L > Lp->L) Lp = &e;
    const V2 u = Lp->u, n{-u[1], u[0]};
    double cx = 0, cz = 0;
    for (const V2& p : poly) { cx += p[0] / poly.size(); cz += p[1] / poly.size(); }
    double lo = 1e18, hi = -1e18, n0 = 1e18, n1 = -1e18;
    for (const V2& p : poly) {
        const double a = (p[0] - cx) * u[0] + (p[1] - cz) * u[1], b = (p[0] - cx) * n[0] + (p[1] - cz) * n[1];
        lo = std::min(lo, a); hi = std::max(hi, a); n0 = std::min(n0, b); n1 = std::max(n1, b);
    }
    const double wn = n1 - n0;
    const int nIs = std::min(12, std::max(1, (int)std::lround((hi - lo) / 7.5))), rows = wn > 16 ? 2 : 1;
    GB &li = c.K.gb("#fffbe8"), &colb = c.K.gb("#e9e9e6"), &isl = c.K.gb(COPING), &pm = c.K.gb("#eeeeec"), &pc = c.K.gb(bg), &scr = c.K.gb("#111417"),
       &boll = c.K.gb("#e0b21a");
    for (int k = 0; k < nIs; k++)
        for (int q = 0; q < rows; q++) {
            const double a = lo + (hi - lo) * (k + .5) / nIs, b = rows == 1 ? 0 : (q ? 1 : -1) * wn * .22;
            const V2 p{cx + u[0] * a + n[0] * b, cz + u[1] * a + n[1] * b};
            if (!Pip(p, poly)) continue;
            const double yb = c.gy(p);
            KBox(isl, p, n, 2.6, .65, yb - .05, yb + .18);
            KBox(colb, p, n, .22, .22, yb, ch);
            for (const double sd : {-1.4, 1.4}) {
                const V2 q2{p[0] + n[0] * sd, p[1] + n[1] * sd};
                KBox(pm, q2, n, .45, .3, yb + .18, yb + 1.85);
                KBox(pc, q2, n, .47, .32, yb + 1.5, yb + 1.95);
                KBox(scr, q2, n, .2, .33, yb + 1.15, yb + 1.4);
            }
            for (const double sd : {-2.5, 2.5}) { const V2 q2{p[0] + n[0] * sd, p[1] + n[1] * sd}; KBox(boll, q2, n, .1, .1, yb + .18, yb + 1); }
        }
    for (int k = 0; k < nIs * 2; k++)
        for (int q = -1; q <= 1; q += 2) {
            const double a = lo + (hi - lo) * (k + .5) / (nIs * 2);
            const V2 p{cx + u[0] * a + n[0] * q * wn * .28, cz + u[1] * a + n[1] * q * wn * .28};
            if (Pip(p, poly)) KBox(li, p, u, .6, .6, ch - .02, ch);
        }
    // concrete forecourt slab
    const Pts fc = SafeInset(poly, -5);
    if (!fc.empty()) Fill(c.K.gb("#bdbab2"), fc, [&](double x, double z) { return c.hAt(x, z) + .05; });
    // price sign: brand panel + LED price board
    const V2 p = P(*Lp, -6, 8);
    const double b0 = c.gy(p) - .05;
    KBox(c.K.gb("#4a4d50"), p, u, .15, .15, b0, b0 + 3.5);
    KBox(c.K.gb(SIGNBACK), p, u, 1.3, .15, b0 + 3.5, b0 + 6.1);
    for (const double sd : {1.0, -1.0}) {
        CE ef;
        ef.n = {-u[1] * sd, u[0] * sd};
        ef.u = {u[0] * sd, u[1] * sd};
        ef.a = {p[0] - u[0] * 1.2 * sd + ef.n[0] * .16, p[1] - u[1] * 1.2 * sd + ef.n[1] * .16};
        Panel(c.K.gb(bg), ef, 0, 2.4, b0 + 4.8, b0 + 6.0, 0);
        Panel(c.K.gb(col.second), ef, .5, 1.9, b0 + 5.2, b0 + 5.6, .01);
        Panel(c.K.gb("#111111"), ef, 0, 2.4, b0 + 3.6, b0 + 4.7, 0);
        for (int k = 0; k < 3; k++) Panel(c.K.gb("#ff4d2e"), ef, 1.2, 2.2, b0 + 3.72 + k * .33, b0 + 3.95 + k * .33, .01);
    }
}

// sub type: facts/commercial.py's s.sub when the tiles carry it; here inferred from cls / name / size / storeys
// (commercial.js inferSub), with seeded variety for small anonymous synthetic buildings.
inline Sub InferSub(const Structure& s, double a, Rng& fr) {
    const std::string cls = Lower(s.cls), nm = Lower(s.name);
    if (cls == "roof" || cls == "canopy" || cls == "fuel_canopy") return Sub::Canopy;
    const int fe = s.floors > 0 ? (int)std::lround(s.floors) : (s.height >= 9.5 ? (int)std::lround(s.height / 3.9) : 1);
    if (fe >= 4 || s.height > 14) return Sub::Office;
    static const char* const BANKK[] = {"bank", "credit union", "\x01" "cu", "chase", "comerica", "huntington", "fifth third",
                                        "wells fargo", "pnc", "citizens", "savings", nullptr};
    if ((cls == "fuel" || GasName(nm)) && a < 500) return Sub::Kiosk;
    if ((cls == "bank" || HasAny(nm, BANKK)) && a < 1500 && fe <= 2) return Sub::Bank;
    if ((cls == "restaurant" || cls == "fast_food" || cls == "cafe" || FastFoodName(nm)) && a < 900 && fe <= 1) return Sub::Restaurant;
    if (a > 5500) return Sub::BigBox;
    if (fe >= 2 || cls == "office") return Sub::Office;
    if (a > 900) return Sub::Strip;
    const double k = fr();
    if (a < 450) return k < .6 ? Sub::Restaurant : k < .75 ? Sub::Bank : Sub::Retail;
    return k < .7 ? Sub::Retail : k < .85 ? Sub::Restaurant : Sub::Bank;
}

// drop the closing duplicate, near-duplicate neighbours and non-finite points
inline Pts CleanPoly(const Pts& in) {
    Pts p;
    for (const V2& q : in) {
        if (!std::isfinite(q[0]) || !std::isfinite(q[1])) return {};
        if (p.empty() || std::hypot(q[0] - p.back()[0], q[1] - p.back()[1]) >= .01) p.push_back(q);
    }
    while (p.size() > 2 && std::hypot(p[0][0] - p.back()[0], p[0][1] - p.back()[1]) < .01) p.pop_back();
    return p;
}

} // namespace commercial_detail

inline std::vector<tbd::Vertex> BuildCommercial(const Structure& s, const HeightFn& hAt) {
    namespace cd = commercial_detail;
    try {
        const Pts poly = cd::CleanPoly(s.pts);
        const double pa = std::abs(PolyArea(poly));
        if (poly.size() < 3 || !(pa >= 6)) return BuildFallback(s, hAt);
        Frame F0 = MakeFrame(poly);
        if (F0.W < 2 || F0.L < 2) return BuildFallback(s, hAt); // slivers
        Rng r(s.seed != 0 ? s.seed : 1);
        Rng fr((s.seed != 0 ? s.seed : 1) * 2654435761LL + 977);
        cd::Kit K;
        cd::C c{s, hAt, r, fr, K, poly, cd::MakeEdges(poly), {}, 0, 0, s.name, {}, {}};
        c.area = s.area > 0 ? s.area : pa;
        // front: the edge facing s.front when given, else the longest edge
        const cd::CE* fe = &c.E[0];
        for (const cd::CE& e : c.E) if (e.L > fe->L) fe = &e;
        if (s.front) {
            const double fx = (*s.front)[0], fz = (*s.front)[1];
            const cd::CE* best = nullptr;
            for (const cd::CE& e : c.E)
                if (e.L > fe->L * .3 && (!best || e.n[0] * fx + e.n[1] * fz > best->n[0] * fx + best->n[1] * fz)) best = &e;
            if (best) fe = best;
        }
        c.f = *fe;
        c.y0 = MinGround(poly, hAt) - .1;
        if (!std::isfinite(c.y0)) return BuildFallback(s, hAt);
        c.flatRoof = cd::PickA(r, cd::FLAT);
        switch (cd::InferSub(s, c.area, fr)) {
            case cd::Sub::Strip: cd::Strip(c); break;
            case cd::Sub::BigBox: cd::BigBox(c); break;
            case cd::Sub::Restaurant: cd::Restaurant(c); break;
            case cd::Sub::Bank: cd::Bank(c); break;
            case cd::Sub::Office: cd::Office(c); break;
            case cd::Sub::Kiosk: cd::Kiosk(c); break;
            case cd::Sub::Canopy: cd::Canopy(c); break;
            default: cd::Retail(c); break;
        }
        std::vector<tbd::Vertex> out;
        size_t n = 0;
        for (const auto& kv : K.G) n += kv.second.v.size();
        out.reserve(n);
        for (const auto& kv : K.G) kv.second.AppendTo(out);
        for (const tbd::Vertex& v : out)
            if (!std::isfinite(v.px) || !std::isfinite(v.py) || !std::isfinite(v.pz)) return BuildFallback(s, hAt);
        if (out.empty()) return BuildFallback(s, hAt);
        return out;
    } catch (...) {
        return BuildFallback(s, hAt);
    }
}

} // namespace optima
