// C++ port of fly-optima engine/lib/gen/industrial.js + lib/gen/industrial/kit.js: tilt-up warehouses / distribution centres,
// light-industrial flex buildings, pole & gambrel barns, self storage (drive-up rows and climate-controlled boxes).
// Formulas and the order of rng() draws follow the JS so a structure with the same facts gives the same building.
// Facts the JS reads that Structure does not carry (sub, dock, office, barn, hsrc) are inferred / seeded here:
//   sub    from cls/name text, else footprint size and height (as the JS does); small anonymous buildings sometimes become pole barns
//   dock   the long side (O.v), flipped away from `front` when front is given
//   office the end of the most front-facing wall (one rng draw, as the JS)
//   barn   gambrel when style/roof says so, else ~18% gambrel from a side rng (does not disturb the main rng stream)
//   hsrc   'guess' when neither height nor floors is set
// Materials are flat colours (palettes.json values where the JS used PAL); textures (tilt-up panel joints, metal ribs, door
// slats, glazing mullions) are not reproduced.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

#include "kit.h"

namespace optima {

std::vector<tbd::Vertex> BuildFallback(const Structure&, const HeightFn&); // inline in generators.h

namespace industrial_detail {

// ---------- palettes (data/palettes.json) ----------
// wall.vinyl_siding stand-ins for tilt-up paint: Pebble, Gray, Linen, Sand, Colonial White, Beige
inline constexpr const char* const TILT_BODY[] = {"#b8b2a3", "#a7a9a6", "#e3dcc8", "#d6c9ad", "#ebe5d3", "#d9cbb0"};
// Slate Gray, Charcoal, Navy, Wedgewood Blue, Mocha
inline constexpr const char* const TILT_ACCENT[] = {"#7d8487", "#4d5156", "#3b4a5e", "#8496a5", "#8a7663"};
// roof.metal: Galvalume, White Metal, Charcoal Metal
inline constexpr const char* const METAL3[] = {"#a7aaab", "#e2e2de", "#3f4246"};
inline constexpr const char* const DOOR_COLS[] = {"#e8e6df", "#d9d6cc", "#3a4a5e", "#5a5f63"};
inline constexpr const char* const STORAGE_DOORS[] = {"#e2791f", "#2d5f9a", "#3b7d4f", "#a8322b", "#d9b23a"};
// roof.flat: White TPO x2, Gray Membrane / + Gravel BUR, Gray Membrane, Black EPDM
inline constexpr const char* const DC_ROOF[] = {"#d8d8d4", "#d8d8d4", "#9a9b98"};
inline constexpr const char* const FLEX_ROOF[] = {"#d8d8d4", "#d8d8d4", "#9a9b98", "#8e877c", "#9a9b98", "#2a2a2a"};
// barn walls: White Metal x2, Galvalume, Barn Red, Forest Green Metal, Charcoal Metal
inline constexpr const char* const BARN_WALL[] = {"#e2e2de", "#e2e2de", "#a7aaab", "#7a2e27", "#3c5244", "#3f4246"};
// roof.asphalt_shingle: Charcoal, Weathered Wood, Onyx Black
inline constexpr const char* const BARN_SHINGLE[] = {"#3c3d3f", "#6d6457", "#2b2b2c"};
// roof.metal: Galvalume, Charcoal Metal, Dark Bronze, Barn Red, Forest Green Metal
inline constexpr const char* const BARN_ROOF_METAL[] = {"#a7aaab", "#3f4246", "#4a3d33", "#7a2e27", "#3c5244"};
// trim on white barns: Charcoal Metal, Barn Red, Forest Green Metal, Dark Bronze
inline constexpr const char* const BARN_TRIM[] = {"#3f4246", "#7a2e27", "#3c5244", "#4a3d33"};
// sedan colours (index.html CAR_COLS)
inline constexpr const char* const CAR_COLS[] = {"#e8e8e6", "#1c1c1e", "#8d9196", "#5b5f64", "#2a3a5a", "#7a1f1f", "#c9c9c4", "#3d4a3a", "#b8a98c"};
inline constexpr const char* WHITE_METAL = "#e2e2de";
inline constexpr const char* WHITE_TPO = "#d8d8d4";
inline constexpr const char* GLASS = "#4d6a7e";   // glassMat default tint
inline constexpr const char* ROAD = "#6e7074";    // index.html asphaltTex base (mats.road)
inline constexpr const char* WALK = "#a9a79f";    // index.html concreteTex base (mats.walk)

constexpr double TILT_U = 7.6, TILT_V = 12, RIB_U = .9;

// ---------- footprint (industrial/kit.js) ----------
struct IE { V2 a, b; double L; V2 t, n; size_t i; };     // edge: tangent t, outward normal n
struct IO { V2 c, u, v; double L, W; };                   // min-area oriented rectangle, u = long axis

inline double Dot2(const V2& a, const V2& b) { return a[0] * b[0] + a[1] * b[1]; }
inline V2 Perp(const V2& u) { return {-u[1], u[0]}; }

// edges with outward normal tested against the polygon (winding does not matter); edges < 0.3 m are dropped
inline std::vector<IE> IEdges(const Pts& pts) {
    std::vector<IE> out;
    const size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        const V2 &a = pts[i], &b = pts[(i + 1) % n];
        const double L = std::hypot(b[0] - a[0], b[1] - a[1]);
        if (!(L >= .3)) continue;
        const V2 t{(b[0] - a[0]) / L, (b[1] - a[1]) / L};
        V2 nn{t[1], -t[0]};
        if (InPoly((a[0] + b[0]) / 2 + nn[0] * .3, (a[1] + b[1]) / 2 + nn[1] * .3, pts)) nn = {-nn[0], -nn[1]};
        out.push_back({a, b, L, t, nn, i});
    }
    return out;
}

inline IO Obb(const Pts& pts, const std::vector<IE>& E) {
    bool have = false;
    double bestA = 0, A0 = 0, A1 = 0, B0 = 0, B1 = 0;
    V2 bu{1, 0}, bv{0, 1};
    for (const IE& e : E) {
        const V2 eu = e.t, ev = Perp(eu);
        double a0 = 1e9, a1 = -1e9, b0 = 1e9, b1 = -1e9;
        for (const V2& p : pts) {
            const double pa = Dot2(p, eu), pb = Dot2(p, ev);
            a0 = std::min(a0, pa); a1 = std::max(a1, pa); b0 = std::min(b0, pb); b1 = std::max(b1, pb);
        }
        const double ar = (a1 - a0) * (b1 - b0);
        if (!have || ar < bestA) { have = true; bestA = ar; bu = eu; bv = ev; A0 = a0; A1 = a1; B0 = b0; B1 = b1; }
    }
    if (B1 - B0 > A1 - A0) {
        const V2 nu = bv, nv{-bu[0], -bu[1]};
        const double na0 = B0, na1 = B1, nb0 = -A1, nb1 = -A0;
        bu = nu; bv = nv; A0 = na0; A1 = na1; B0 = nb0; B1 = nb1;
    }
    const double ca = (A0 + A1) / 2, cb = (B0 + B1) / 2;
    return {{bu[0] * ca + bv[0] * cb, bu[1] * ca + bv[1] * cb}, bu, bv, A1 - A0, B1 - B0};
}

inline double RectRatio(const Pts& pts, const IO& o) { return std::abs(PolyArea(pts)) / std::max(1e-9, o.L * o.W); }

// ---------- geometry into GB batches ----------
// point on wall: edge e, distance s along it, height y, offset d outward
inline V3 Wp(const IE& e, double s, double y, double d = 0) {
    return {e.a[0] + e.t[0] * s + e.n[0] * d, y, e.a[1] + e.t[1] * s + e.n[1] * d};
}
// vertical quad on a wall from s0..s1, y0..y1, offset d; uv either metres/scale or 0..1 (unit)
inline void WallQuad(GB& g, const IE& e, double s0, double s1, double y0, double y1, double d, double su = 1, double sv = 1,
                     bool unit = false) {
    if (!(s1 > s0) || !(y1 > y0)) return;
    const std::array<V2, 4> uv = unit ? std::array<V2, 4>{V2{0, 0}, V2{1, 0}, V2{1, 1}, V2{0, 1}}
                                      : std::array<V2, 4>{V2{s0 / su, y0 / sv}, V2{s1 / su, y0 / sv}, V2{s1 / su, y1 / sv}, V2{s0 / su, y1 / sv}};
    g.Quad(Wp(e, s0, y0, d), Wp(e, s1, y0, d), Wp(e, s1, y1, d), Wp(e, s0, y1, d), {e.n[0], 0, e.n[1]}, uv);
}

// box faces; hidden faces are left out to stay within the triangle budget
enum : unsigned { FN = 1, BN = 2, FU = 4, BU = 8, TOP = 16, BOT = 32, ALL = 63, NOBOT = 31, SIDES = 15 };
// oriented box: centre c, size a along U, h up, b along N (U, N horizontal unit vectors)
inline void Box3(GB& g, const V3& c, double a, double h, double b, const V2& U, const V2& N, unsigned m = ALL,
                 const RGB* col = nullptr) {
    auto P = [&](double i, double j, double k) {
        return V3{c[0] + U[0] * a / 2 * i + N[0] * b / 2 * k, c[1] + h / 2 * j, c[2] + U[1] * a / 2 * i + N[1] * b / 2 * k};
    };
    auto uvq = [](double w, double hh) { return std::array<V2, 4>{V2{0, 0}, V2{w, 0}, V2{w, hh}, V2{0, hh}}; };
    if (m & FN) g.Quad(P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1), {N[0], 0, N[1]}, uvq(a, h), col);
    if (m & BN) g.Quad(P(1, -1, -1), P(-1, -1, -1), P(-1, 1, -1), P(1, 1, -1), {-N[0], 0, -N[1]}, uvq(a, h), col);
    if (m & FU) g.Quad(P(1, -1, 1), P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), {U[0], 0, U[1]}, uvq(b, h), col);
    if (m & BU) g.Quad(P(-1, -1, -1), P(-1, -1, 1), P(-1, 1, 1), P(-1, 1, -1), {-U[0], 0, -U[1]}, uvq(b, h), col);
    if (m & TOP) g.Quad(P(-1, 1, 1), P(1, 1, 1), P(1, 1, -1), P(-1, 1, -1), {0, 1, 0}, uvq(a, b), col);
    if (m & BOT) g.Quad(P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1), {0, -1, 0}, uvq(a, b), col);
}
// JS box(gb, x, y, z, a, h, b, u): v = perp(u)
inline void BoxU(GB& g, const V3& c, double a, double h, double b, const V2& u, unsigned m = ALL) { Box3(g, c, a, h, b, u, Perp(u), m); }

// triangle that skips degenerate input (profile fans produce some)
inline void Tri3(GB& g, const V3& a, const V3& b, const V3& c, const V3& n, V2 ua, V2 ub, V2 uc) {
    const V3 x = Cross(Sub(b, a), Sub(c, a));
    if (Dot(x, x) < 1e-10) return;
    g.Tri(a, b, c, n, ua, ub, uc);
}

// colour picked after the geometry was emitted (keeps the JS rng order)
inline void Recolor(GB& g, const RGB& c) {
    g.color = c;
    for (tbd::Vertex& q : g.v) { q.r = c[0]; q.g = c[1]; q.b = c[2]; }
}

inline std::string Lower(std::string t) {
    for (char& ch : t) ch = (char)std::tolower((unsigned char)ch);
    return t;
}
inline bool HasAny(const std::string& t, std::initializer_list<const char*> keys) {
    for (const char* k : keys) if (t.find(k) != std::string::npos) return true;
    return false;
}

// ---------- prep ----------
enum class Sub { DC, Flex, Barn, Storage };

struct Prep {
    Pts pts;
    std::vector<IE> E;
    IO O;
    V2 C;
    double y0 = 0, H = 0;
    Sub sub = Sub::Flex;
    bool hGuess = false;
};

inline bool MakePrep(const Structure& s, const HeightFn& hAt, Prep& P) {
    P.pts = s.pts;
    for (const V2& p : P.pts) if (!std::isfinite(p[0]) || !std::isfinite(p[1])) return false;
    P.E = IEdges(P.pts);
    if (P.E.size() < 3 || std::abs(PolyArea(P.pts)) < 4) return false;
    P.O = Obb(P.pts, P.E);
    if (!(P.O.L >= 1.5) || !(P.O.W >= 1.0) || !std::isfinite(P.O.L * P.O.W)) return false;
    double cx = 0, cz = 0;
    for (const V2& p : P.pts) { cx += p[0]; cz += p[1]; }
    P.C = {cx / P.pts.size(), cz / P.pts.size()};
    P.y0 = MinGround(P.pts, hAt) - .15;
    if (!std::isfinite(P.y0)) return false;
    const double a = s.area > 0 ? s.area : P.O.L * P.O.W, h = s.height > 0 ? s.height : 0;
    const std::string cls = Lower(s.cls) + Lower(s.name);
    if (HasAny(cls, {"storage", "garages"})) P.sub = Sub::Storage;
    else if (HasAny(cls, {"barn", "farm", "agricult", "stable", "shed"})) P.sub = Sub::Barn;
    else if (P.O.W <= 12.5 && P.O.L / P.O.W > 3 && a < 2500) P.sub = Sub::Storage;
    else if (a >= 9000 || (a >= 4500 && h >= 9.5)) P.sub = Sub::DC;
    else P.sub = Sub::Flex;
    // anonymous small buildings (synthetic structures): some are metal pole barns / machine sheds (side rng)
    if (P.sub == Sub::Flex && cls.empty() && a < 500 && (h == 0 || h < 7)) {
        Rng q(s.seed ^ 0x2545F491);
        if (q() < .35) P.sub = Sub::Barn;
    }
    P.hGuess = s.height <= 0 && s.floors <= 0;
    double def = P.sub == Sub::DC ? 12.2 : P.sub == Sub::Flex ? (a < 600 ? 5.0 : 7.3) : P.sub == Sub::Barn ? 6 : 3.2;
    double H = s.height > 0 ? s.height : s.floors > 0 ? std::max(def, s.floors * 4.2) : def;
    if (!std::isfinite(H)) H = def;
    P.H = std::max(2.5, std::min(60.0, H));
    return true;
}

// 53 ft van trailer (16.2 x 2.6 x 4.1 m), length along n
inline void Trailer(GB& body, GB& dark, const V3& p, const V2& n, bool parked) {
    const V2 u{n[1], -n[0]};
    const double L = 16.2, sg = parked ? -1 : 1;
    Box3(body, {p[0], p[1] + 1.2 + 1.45, p[2]}, 2.6, 2.9, L, u, n, NOBOT);
    // tandem axles at the tail end (the JS's two axle boxes at -.36 / -.28 L merged into one)
    const double ka = -.32 * L * sg;
    Box3(dark, {p[0] + n[0] * ka, p[1] + .55, p[2] + n[1] * ka}, 2.5, 1.1, 1.0 + .08 * L, u, n, SIDES);
    // landing gear near the nose (the JS put it at -.38 L, on the axle end)
    const double kg = .30 * L * sg;
    Box3(dark, {p[0] + n[0] * kg, p[1] + .6, p[2] + n[1] * kg}, .2, 1.2, .2, u, n, SIDES);
}

// index.html car(r): body, glass cabin, wheels (axle boxes instead of cylinders); length along (sin th, cos th)
inline void Car(GB& g, double x, double y, double z, const V2& n, Rng& r) {
    static const RGB glass = Hex("#2a3440"), tyre = Hex("#8d8a84");
    const RGB col = Hex(Pick(r, CAR_COLS));
    const double L = 4.2 + r() * .6;
    const double th = std::atan2(n[0], n[1]) + (r() < .5 ? 3.14159265358979323846 : 0);
    const V2 Z{std::sin(th), std::cos(th)}, X{std::cos(th), -std::sin(th)};
    Box3(g, {x, y + .55, z}, 1.8, .7, L, X, Z, NOBOT, &col);
    Box3(g, {x - Z[0] * L * .05, y + 1.15, z - Z[1] * L * .05}, 1.6, .55, L * .5, X, Z, NOBOT, &glass);
    for (const double k : {-.32, .32}) Box3(g, {x + Z[0] * L * k, y + .33, z + Z[1] * L * k}, 1.82, .66, .66, X, Z, SIDES, &tyre);
}

// ================= tilt-up warehouse / DC and flex =================
inline std::vector<tbd::Vertex> BoxBuilding(const Structure& s, const HeightFn& hAt, const Prep& P, Rng& r, Sub sub) {
    const Pts& pts = P.pts;
    const std::vector<IE>& E = P.E;
    const IO& O = P.O;
    const double y0 = P.y0, H = P.H, y1 = y0 + .15 + H;
    const bool dc = sub == Sub::DC;
    const double para = dc ? 1.0 : .7, yt = y1 + para;
    const bool flexMetal = !dc && H < 6.5 && r() < .35; // some small flex/shop buildings are metal-clad
    const char* body = Pick(r, TILT_BODY);
    const char* accent = Pick(r, TILT_ACCENT);
    const char* wallHex = flexMetal ? Pick(r, METAL3) : body;
    GB wallGB(Hex(wallHex)), accGB(Hex(accent)), copGB(Hex(flexMetal ? "#d0d0cc" : "#8c9196")), glGB(Hex(GLASS)),
        steelGB(Hex("#6e7378")), blackGB(Hex("#1d1d1d")), yelGB(Hex("#d9a91a"));
    GB courtGB(Hex(ROAD)), apronGB(Hex(WALK)), roofBits(Hex("#b9bcbc")), skyGB(Hex("#e9efe9")), trlGB(Hex("#e8e8e4")),
        trlDarkGB(Hex("#2a2c2e")), roofGB, carGB;
    const RGB doorC = Hex(Pick(r, DOOR_COLS));
    GB doorGB(doorC), gradeGB(doorC);
    // facts (or fallbacks): dock wall normal, front wall normal, office corner
    V2 dockV = O.v;
    V2 frontN{-dockV[0], -dockV[1]};
    if (s.front) {
        const double fl = std::hypot((*s.front)[0], (*s.front)[1]);
        if (fl > 1e-6 && std::isfinite(fl)) {
            frontN = {(*s.front)[0] / fl, (*s.front)[1] / fl};
            if (Dot2(frontN, dockV) > .5) dockV = {-dockV[0], -dockV[1]}; // docks away from the front
        }
    }
    const std::vector<V2> dockN{dockV};
    const IE* fe0 = &E[0];
    for (const IE& e : E) if (Dot2(e.n, frontN) * e.L > Dot2(fe0->n, frontN) * fe0->L) fe0 = &e;
    const V2 office = r() < .5 ? fe0->a : fe0->b;
    size_t oi = 0;
    for (size_t i = 0; i < pts.size(); ++i)
        if (std::hypot(pts[i][0] - office[0], pts[i][1] - office[1]) < std::hypot(pts[oi][0] - office[0], pts[oi][1] - office[1])) oi = i;
    const V2 OV = pts[oi];
    const double officeLen = dc ? std::min(30.0, 12 + r() * 16) : 0;
    auto distToOffice = [&](const IE& ed, double sAlong) {
        return std::hypot(ed.a[0] + ed.t[0] * sAlong - OV[0], ed.a[1] + ed.t[1] * sAlong - OV[1]);
    };
    auto isDockN = [&](const V2& n) {
        for (const V2& d : dockN) if (Dot2(d, n) > .92) return true;
        return false;
    };

    for (const IE& e : E) {
        const double L = e.L;
        // tilt-up wall panels to the parapet top
        WallQuad(wallGB, e, 0, L, y0, yt, 0, flexMetal ? RIB_U : TILT_U, flexMetal ? 30 : TILT_V);
        if (!flexMetal) {
            WallQuad(accGB, e, 0, L, yt - .55, yt - .25, .03, 4, 1);           // painted reveal band under the coping
            if (dc) WallQuad(accGB, e, 0, L, y0 + .15, y0 + 1.25, .02, 4, 1);  // darker base band
        }
        Box3(copGB, Wp(e, L / 2, yt + .06, -.05), L + .1, .12, .45, e.t, e.n); // metal coping cap
        const bool isDock = isDockN(e.n) && L > 14, isFront = Dot2(frontN, e.n) > .92;
        // ---- office glazing (DC: corner curtain wall; flex: storefront bays along the front)
        if (dc) {
            const double rx = OV[0] - e.a[0], rz = OV[1] - e.a[1], sp = rx * e.t[0] + rz * e.t[1], perp = std::abs(rx * e.n[0] + rz * e.n[1]);
            const double da = sp < 0 ? -sp : 0, db = sp > L ? sp - L : 0;
            if (perp < 1.5 && std::max(da, db) < officeLen - 3 && !isDock) {
                const double s0 = std::max(.6, sp - officeLen), s1 = std::min(L - .6, sp + officeLen);
                if (s1 - s0 > 3) {
                    WallQuad(glGB, e, s0, s1, y0 + .6, y0 + .6 + std::min(H * .72, 9.0), .04, 1.52, 2.0);
                    WallQuad(wallGB, e, s0 - .6, s1 + .6, yt, yt + 1.4, 0, TILT_U, TILT_V); // pop-up parapet over the office
                    Box3(copGB, Wp(e, (s0 + s1) / 2, yt + 1.46, -.05), s1 - s0 + 1.3, .12, .45, e.t, e.n);
                    if (r() < .8 && sp >= -1 && sp <= L + 1) { // entry canopy
                        const double m = sp < L / 2 ? s0 + 4 : s1 - 4;
                        Box3(steelGB, Wp(e, m, y0 + 3.4, 1.4), 5, .25, 2.8, e.t, e.n);
                    }
                }
            }
        } else if (isFront && L > 8) {
            const int nb = std::max(1, (int)std::round(L / 16));
            const double bw = L / nb;
            for (int k = 0; k < nb; ++k) {
                const double b0 = k * bw, sw = std::min(bw * .55, 7.0);
                WallQuad(glGB, e, b0 + 1.2, b0 + 1.2 + sw, y0 + .7, y0 + std::min(3.3, H * .5), .04, 1.5, 2.6);
                WallQuad(glGB, e, b0 + 1.6 + sw, b0 + 2.6 + sw, y0 + .15, y0 + 2.3, .05, 1, 2.15); // glass entry door
                Box3(steelGB, Wp(e, b0 + 2.1 + sw, y0 + 2.75, .7), 2.2, .15, 1.4, e.t, e.n);      // small canopy
                if (H > 5.5) WallQuad(accGB, e, b0 + 1, b0 + bw - 1, y0 + std::min(3.9, H * .58), y0 + std::min(4.6, H * .68), .03, 4, 1); // sign band
            }
        }
        // ---- clerestory windows high on long walls (DC)
        if (dc && L > 30 && !(isFront && std::min(distToOffice(e, 0), distToOffice(e, L)) < 2 && L < 60)) {
            for (double x = 6; x < L - 6; x += 9.1) {
                if (distToOffice(e, x) < officeLen + 4) continue;
                WallQuad(glGB, e, x - 1.8, x + 1.8, y1 - 2.3, y1 - 1.2, .04, 1.2, 1.1);
            }
        }
        // ---- loading docks
        if (isDock) {
            const double pitch = dc ? 4.27 : 15, m0 = 6;
            std::vector<double> doorsFrom;
            for (double x = m0; x <= L - m0; x += pitch) {
                if (distToOffice(e, x) < officeLen + 6 || (!dc && distToOffice(e, x) < 6)) continue;
                doorsFrom.push_back(x);
            }
            // DC: run of dock doors with a gap every 13th; flex: per bay one grade-level door + one dock door
            for (size_t k = 0; k < doorsFrom.size(); ++k) {
                const double x = doorsFrom[k];
                if (dc && k % 13 == 12) continue;
                if (dc || k % 2 == 1) {
                    // dock-high door 2.74 x 3.05 m, sill at 1.22 m, black seal, leveler lip
                    const double yb = y0 + .15 + 1.22;
                    WallQuad(doorGB, e, x - 1.37, x + 1.37, yb, yb + 3.05, .06, 1, 1, true);
                    Box3(blackGB, Wp(e, x - 1.55, yb + 1.5, .22), .3, 3.4, .4, e.t, e.n, FN | FU | BU | TOP);
                    Box3(blackGB, Wp(e, x + 1.55, yb + 1.5, .22), .3, 3.4, .4, e.t, e.n, FN | FU | BU | TOP);
                    Box3(blackGB, Wp(e, x, yb + 3.25, .22), 3.4, .4, .4, e.t, e.n, FN | FU | BU | TOP | BOT);
                    Box3(yelGB, Wp(e, x, y0 + .15 + .6, .06), 2.6, .12, .12, e.t, e.n, FN | TOP);
                    if (dc && r() < .3) Trailer(trlGB, trlDarkGB, Wp(e, x, y0 + .15, 8.6), e.n, false);
                } else {
                    // grade-level drive-in door 3.66 x 4.27 m
                    const double gh = std::min(4.27, H - 1.6);
                    WallQuad(gradeGB, e, x - 1.83, x + 1.83, y0 + .15, y0 + .15 + gh, .06, 1, 1, true);
                    Box3(yelGB, Wp(e, x - 2.1, y0 + .15 + .6, .25), .2, 1.2, .2, e.t, e.n, NOBOT); // bollards
                    Box3(yelGB, Wp(e, x + 2.1, y0 + .15 + .6, .25), .2, 1.2, .2, e.t, e.n, NOBOT);
                }
            }
            if (dc && !doorsFrom.empty()) { // dock canopy
                const double da0 = doorsFrom.front(), da1 = doorsFrom.back();
                Box3(steelGB, Wp(e, (da0 + da1) / 2, y0 + .15 + 1.22 + 3.85, 1.0), da1 - da0 + 4, .2, 2.0, e.t, e.n);
            }
            // truck court (asphalt) and concrete apron in front of the docks
            const double d1 = dc ? 40 : 22, s0 = 1, s1 = L - 1, yc = y0 + .2;
            if (s1 > s0) {
                courtGB.Quad(Wp(e, s0, yc, 4), Wp(e, s1, yc, 4), Wp(e, s1, yc, d1), Wp(e, s0, yc, d1), {0, 1, 0},
                             {V2{0, 0}, V2{L / 4, 0}, V2{L / 4, d1 / 4}, V2{0, d1 / 4}});
                apronGB.Quad(Wp(e, s0, yc + .02, 0), Wp(e, s1, yc + .02, 0), Wp(e, s1, yc + .02, 4), Wp(e, s0, yc + .02, 4), {0, 1, 0},
                             {V2{0, 0}, V2{L / 4, 0}, V2{L / 4, 1}, V2{0, 1}});
            }
            if (dc) // trailer stall paint
                for (double x = 4; x < L - 4; x += 4.27)
                    yelGB.Quad(Wp(e, x - .06, yc + .05, 22), Wp(e, x + .06, yc + .05, 22), Wp(e, x + .06, yc + .05, 38), Wp(e, x - .06, yc + .05, 38), {0, 1, 0});
            if (dc)
                for (double x = 10; x < L - 10; x += 4.27 * (2 + std::floor(r() * 3)))
                    if (r() < .45) Trailer(trlGB, trlDarkGB, Wp(e, x, yc, 30.5), {-e.n[0], -e.n[1]}, true);
        }
        // man doors here and there on long blank walls
        if (!isDock && !isFront && L > 25)
            for (double x = 12 + r() * 10; x < L - 8; x += 40 + r() * 30) {
                WallQuad(gradeGB, e, x - .5, x + .5, y0 + .15, y0 + 2.3, .05, 1, 1, true);
                Box3(steelGB, Wp(e, x, y0 + 2.55, .4), 1.6, .1, .8, e.t, e.n);
            }
    }
    // ---- roof: white TPO / gray membrane, skylights, RTUs
    Recolor(roofGB, Hex(dc ? Pick(r, DC_ROOF) : Pick(r, FLEX_ROOF)));
    Cap(pts, y1, roofGB);
    const V2 u = O.u, v = O.v;
    const double st0 = dc ? 24 : 18, st1 = dc ? 15 : 14;
    for (double a = -O.L / 2 + 10; a < O.L / 2 - 8; a += st0)
        for (double b = -O.W / 2 + 8; b < O.W / 2 - 6; b += st1) {
            const double x = O.c[0] + u[0] * a + v[0] * b, z = O.c[1] + u[1] * a + v[1] * b;
            if (!InPoly(x, z, pts)) continue;
            if (dc) BoxU(skyGB, {x, y1 + .15, z}, 2.4, .3, 1.2, u, NOBOT);
            else if (r() < .35) BoxU(roofBits, {x, y1 + .6, z}, 2.2, 1.2, 1.6, u, NOBOT);
        }
    const int nRtu = dc ? 2 + (int)std::floor(r() * 4) : 0;
    for (int k = 0; k < nRtu; ++k) {
        const double px = OV[0] + (P.C[0] - OV[0]) * (.08 + r() * .12);
        const double pz = OV[1] + (P.C[1] - OV[1]) * (.08 + r() * .12);
        if (InPoly(px, pz, pts)) BoxU(roofBits, {px, y1 + .7, pz}, 3, 1.4, 2, u, NOBOT);
    }
    // ---- employee parking with a few cars in front of the office
    {
        auto okE = [&](const IE& ed) { return ed.L > 12 && !isDockN(ed.n); };
        const IE* fe = nullptr;
        for (const IE& e : E) if (okE(e) && Dot2(e.n, frontN) > .92) { fe = &e; break; }
        if (!fe) {
            double best = 1e18;
            for (const IE& e : E) {
                if (!okE(e)) continue;
                const double dd = std::min(std::hypot(e.a[0] - OV[0], e.a[1] - OV[1]), std::hypot(e.b[0] - OV[0], e.b[1] - OV[1]));
                if (dd < best) { best = dd; fe = &e; }
            }
        }
        if (fe) {
            const int n = dc ? 6 + (int)std::floor(r() * 10) : 2 + (int)std::floor(r() * 5);
            for (int k = 0; k < n; ++k) {
                const double x = std::min(fe->L - 2, 2 + r() * std::min(fe->L - 4, dc ? 70 : fe->L));
                const double sAl = std::hypot(fe->a[0] - OV[0], fe->a[1] - OV[1]) < 1 ? x : fe->L - x;
                const V3 p = Wp(*fe, sAl, 0, 9 + (k % 2) * 6);
                Car(carGB, p[0], hAt(p[0], p[2]), p[2], fe->n, r);
            }
        }
    }
    std::vector<tbd::Vertex> out;
    wallGB.AppendTo(out, true); // parapet insides are seen from above
    for (GB* g : {&accGB, &copGB, &glGB, &doorGB, &gradeGB, &steelGB, &blackGB, &yelGB, &courtGB, &apronGB, &roofGB, &roofBits, &skyGB,
                  &trlGB, &trlDarkGB, &carGB})
        g->AppendTo(out);
    return out;
}

// ================= pole barn / gambrel barn =================
inline std::vector<tbd::Vertex> Barn(const Structure& s, const Prep& P, Rng& r) {
    const IO& O = P.O;
    const V2 u = O.u, v = O.v, c = O.c;
    const double L = O.L, W = O.W;
    bool gamb = Lower(s.style).find("gambrel") != std::string::npos || Lower(s.roof).find("gambrel") != std::string::npos;
    if (!gamb && s.style.empty() && s.roof.empty()) { Rng q(s.seed ^ 0x6A09E667); gamb = q() < .18 && W >= 6; }
    const double yb = P.y0 + .15, pitch = gamb ? 0 : .333;
    const double eave = gamb ? std::max(3.2, std::min(5.0, P.H * .45)) : std::max(2.8, std::min(6.0, P.H - W / 2 * pitch));
    const char* wallHex = gamb ? "#7a2e27" : Pick(r, BARN_WALL);
    const char* roofHex = gamb ? Pick(r, BARN_SHINGLE) : Pick(r, BARN_ROOF_METAL);
    const char* trimHex = std::strcmp(wallHex, WHITE_METAL) == 0 ? Pick(r, BARN_TRIM) : "#ecebe6";
    GB wGB(Hex(wallHex)), rGB(Hex(roofHex)), tGB(Hex(trimHex)), dGB, gGB(Hex("#33475a")), wainGB(Hex(trimHex));
    auto Cw = [&](double a, double b, double y) { return V3{c[0] + u[0] * a + v[0] * b, y, c[1] + u[1] * a + v[1] * b}; };
    // profile across the width (b, y) from one eave to the other
    const std::vector<V2> prof = gamb ? std::vector<V2>{{-W / 2, eave}, {-W * .32, eave + W * .3}, {0, eave + W * .42}, {W * .32, eave + W * .3}, {W / 2, eave}}
                                      : std::vector<V2>{{-W / 2, eave}, {0, eave + W / 2 * pitch}, {W / 2, eave}};
    const double ov = .35;
    // long walls (along u at b = +-W/2); wainscot pushed 3 cm out (the JS used b*1.002)
    for (const double sg : {1.0, -1.0}) {
        const double a0 = -L / 2, a1 = L / 2, b = sg * W / 2, bw = b + sg * .03;
        const V3 n{v[0] * sg, 0, v[1] * sg};
        const std::array<V2, 4> uv{V2{0, 0}, V2{L / RIB_U, 0}, V2{L / RIB_U, 1}, V2{0, 1}};
        wGB.Quad(Cw(a0, b, yb), Cw(a1, b, yb), Cw(a1, b, yb + eave), Cw(a0, b, yb + eave), n, uv);
        wainGB.Quad(Cw(a0, bw, yb), Cw(a1, bw, yb), Cw(a1, bw, yb + .9), Cw(a0, bw, yb + .9), n, uv);
    }
    // gable ends (fan of the profile)
    for (const double sg : {1.0, -1.0}) {
        const double a = sg * L / 2, aw = a + sg * .03;
        const V3 n{u[0] * sg, 0, u[1] * sg};
        const std::array<V2, 4> uv{V2{0, 0}, V2{W / RIB_U, 0}, V2{W / RIB_U, 1}, V2{0, 1}};
        wGB.Quad(Cw(a, -W / 2, yb), Cw(a, W / 2, yb), Cw(a, W / 2, yb + eave), Cw(a, -W / 2, yb + eave), n, uv);
        wainGB.Quad(Cw(aw, -W / 2, yb), Cw(aw, W / 2, yb), Cw(aw, W / 2, yb + .9), Cw(aw, -W / 2, yb + .9), n, uv);
        for (size_t i = 0; i + 1 < prof.size(); ++i) {
            const V2 &p = prof[i], &q = prof[i + 1];
            Tri3(wGB, Cw(a, p[0], yb + eave), Cw(a, q[0], yb + eave), Cw(a, q[0], yb + q[1]), n, {p[0] / RIB_U, 0}, {q[0] / RIB_U, 0}, {q[0] / RIB_U, 1});
            Tri3(wGB, Cw(a, p[0], yb + eave), Cw(a, q[0], yb + q[1]), Cw(a, p[0], yb + p[1]), n, {p[0] / RIB_U, 0}, {q[0] / RIB_U, 1}, {p[0] / RIB_U, 1});
        }
        // big door on one gable (sliding or overhead), hay door for gambrel
        if (sg == 1 || r() < .4) {
            const double dw = std::min(W * .45, W > 12 ? 4.9 : 3.7), dh = std::min(eave - .4, 4.3), bc = (r() - .5) * (W - dw - 2) * .5;
            auto qd = [&](double qb0, double qb1, double qy0, double qy1, GB& g) {
                const double ad = a + sg * .04;
                g.Quad(Cw(ad, qb0, yb + qy0), Cw(ad, qb1, yb + qy0), Cw(ad, qb1, yb + qy1), Cw(ad, qb0, yb + qy1), n);
            };
            qd(bc - dw / 2, bc + dw / 2, 0, dh, dGB);
            for (const double b : {bc - dw / 2 - .08, bc + dw / 2 + .08}) BoxU(tGB, Cw(a + sg * .06, b, yb + dh / 2), .12, dh, .16, u, NOBOT);
            BoxU(tGB, Cw(a + sg * .06, bc, yb + dh + .08), .12, .16, dw + .3, u);
            if (gamb && W >= 8) { qd(-.8, .8, eave + .5, eave + 2.0, dGB); BoxU(tGB, Cw(a + sg * .06, 0, yb + eave + 2.1), .1, .15, 1.9, u); }
        }
    }
    // roof planes with overhang
    for (size_t i = 0; i + 1 < prof.size(); ++i) {
        double b0 = prof[i][0], h0 = prof[i][1], b1 = prof[i + 1][0], h1 = prof[i + 1][1];
        if (i == 0) { const double k = ov / std::hypot(b1 - b0, h1 - h0); b0 -= (b1 - b0) * k; h0 -= (h1 - h0) * k; }
        if (i + 2 == prof.size()) { const double k = ov / std::hypot(b1 - b0, h1 - h0); b1 += (b1 - b0) * k; h1 += (h1 - h0) * k; }
        const double sl = std::hypot(b1 - b0, h1 - h0), nb0 = -(h1 - h0), nb1 = b1 - b0, nl = std::hypot(nb0, nb1);
        const V2 nn{nb0 / nl, nb1 / nl};
        const V3 N{v[0] * nn[0], nn[1], v[1] * nn[0]};
        const double a0 = -L / 2 - ov, a1 = L / 2 + ov;
        const std::array<V2, 4> uv = gamb ? std::array<V2, 4>{V2{0, 0}, V2{L / 3, 0}, V2{L / 3, sl / 3}, V2{0, sl / 3}}
                                          : std::array<V2, 4>{V2{0, 0}, V2{L / RIB_U, 0}, V2{L / RIB_U, 1}, V2{0, 1}};
        rGB.Quad(Cw(a0, b0, yb + h0 + .05), Cw(a1, b0, yb + h0 + .05), Cw(a1, b1, yb + h1 + .05), Cw(a0, b1, yb + h1 + .05), N, uv);
    }
    V2 top = prof[0];
    for (const V2& p : prof) if (p[1] > top[1]) top = p;
    BoxU(tGB, Cw(0, top[0], yb + top[1] + .1), L + 2 * ov, .14, .35, u, NOBOT); // ridge cap
    for (const V2& ab : {V2{-L / 2, -W / 2}, V2{-L / 2, W / 2}, V2{L / 2, -W / 2}, V2{L / 2, W / 2}})
        BoxU(tGB, Cw(ab[0], ab[1], yb + eave / 2), .14, eave, .14, u, SIDES); // corner trim
    // walk door + windows on a long side
    const double sd = r() < .5 ? 1 : -1;
    const V3 nS{v[0] * sd, 0, v[1] * sd};
    auto wq = [&](double qa0, double qa1, double qy0, double qy1, GB& g) {
        const double bs = sd * (W / 2 + .04);
        g.Quad(Cw(qa0, bs, yb + qy0), Cw(qa1, bs, yb + qy0), Cw(qa1, bs, yb + qy1), Cw(qa0, bs, yb + qy1), nS);
    };
    const double da = (r() - .5) * L * .6;
    wq(da - .45, da + .45, 0, 2.1, dGB);
    for (double a = -L / 2 + 3; a < L / 2 - 2; a += 6 + r() * 4)
        if (std::abs(a - da) > 2 && eave > 2.6) wq(a - .6, a + .6, std::min(1.5, eave - 1.3), std::min(2.4, eave - .3), gGB);
    if (gamb && L > 14 && r() < .6) { // cupola
        BoxU(tGB, Cw(0, 0, yb + top[1] + .7), 1.3, 1.2, 1.3, u, SIDES);
        BoxU(rGB, Cw(0, 0, yb + top[1] + 1.45), 1.6, .3, 1.6, u);
    }
    if (gamb) Recolor(dGB, Hex("#6d2620"));
    else {
        const char* const opts[3] = {"#ecebe6", "#d9d6cc", trimHex};
        Recolor(dGB, Hex(Pick(r, opts)));
    }
    std::vector<tbd::Vertex> out;
    wGB.AppendTo(out);
    rGB.AppendTo(out, true); // overhang undersides
    for (GB* g : {&wainGB, &tGB, &dGB, &gGB}) g->AppendTo(out);
    return out;
}

// ================= self storage =================
inline std::vector<tbd::Vertex> Storage(const Structure& s, const Prep& P, Rng& r) {
    const IO& O = P.O;
    const Pts& pts = P.pts;
    const std::vector<IE>& E = P.E;
    const double y0 = P.y0, area = s.area > 0 ? s.area : O.L * O.W;
    // one door colour per facility: hashed from the name like the JS (synthetic, unnamed ones hash the seed)
    const char* doorHex;
    {
        double hs = 7;
        for (const unsigned char ch : (s.name.empty() ? std::string("x") : s.name)) hs = hs * 31 + ch;
        double sd = std::isfinite(hs) ? std::fmod(std::floor(hs), 99991.0) : 0;
        if (s.name.empty()) sd = (double)((uint64_t)s.seed * 2654435761ull % 99991ull);
        Rng q((int64_t)sd);
        doorHex = STORAGE_DOORS[std::min<size_t>(4, (size_t)std::floor(q() * 5))];
    }
    // wall.vinyl_siding: Linen, Beige, Sand, Colonial White
    static constexpr const char* const WALLS[] = {"#e3dcc8", "#d9cbb0", "#d6c9ad", "#ebe5d3"};
    const char* wallHex = Pick(r, WALLS);
    std::vector<tbd::Vertex> out;
    if (area > 2500 || RectRatio(pts, O) < .8) {
        // climate-controlled multi-storey storage: tilt/block box, glazed office corner, accent band, doors to loading bay
        const double H = P.hGuess ? (area > 6000 ? 9.5 : 6.5) : P.H, y1 = y0 + .15 + H;
        GB wGB(Hex(wallHex)), aGB(Hex(doorHex)), glGB(Hex(GLASS)), dGB(Hex(doorHex)), cGB(Hex("#8c9196")), roofGB(Hex(WHITE_TPO));
        for (const IE& e : E) {
            WallQuad(wGB, e, 0, e.L, y0, y1 + .6, 0, TILT_U, TILT_V);
            WallQuad(aGB, e, 0, e.L, y1 - .9, y1 + .3, .03, 4, 1);
            Box3(cGB, Wp(e, e.L / 2, y1 + .66, -.05), e.L + .1, .12, .4, e.t, e.n);
            if (e.L > 20)
                for (double x = 8; x < e.L - 6; x += 10)
                    for (int f = 0; f < (int)std::floor(H / 3.2); ++f)
                        WallQuad(glGB, e, x - .9, x + .9, y0 + 1.2 + f * 3.2, y0 + 2.3 + f * 3.2, .04, 1.8, 1.1);
        }
        auto score = [&](const IE& ed) { return s.front ? Dot2(ed.n, *s.front) * ed.L : ed.L; };
        const IE* fe = &E[0];
        for (const IE& e : E) if (score(e) > score(*fe)) fe = &e;
        WallQuad(glGB, *fe, 2, std::min(fe->L - 2, 14.0), y0 + .3, y0 + 3.4, .05, 1.5, 3.1);
        for (int k = 0; k < std::min(4, (int)std::floor((fe->L - 20) / 4)); ++k)
            WallQuad(dGB, *fe, 18 + k * 4, 21 + k * 4, y0 + .15, y0 + 3.2, .05, 1, 1, true);
        Cap(pts, y1, roofGB);
        wGB.AppendTo(out, true);
        for (GB* g : {&aGB, &glGB, &dGB, &cGB, &roofGB}) g->AppendTo(out);
        return out;
    }
    // drive-up rows: single-storey metal building, low-slope gable, roll-up doors every 3.05 m (10 ft units) on the long side(s)
    const V2 u = O.u, v = O.v, c = O.c;
    const double L = O.L, W = O.W, yb = y0 + .15, eave = std::max(2.7, std::min(3.6, P.H - W / 2 * .083)), ridge = eave + W / 2 * .083;
    auto Cw = [&](double a, double b, double y) { return V3{c[0] + u[0] * a + v[0] * b, y, c[1] + u[1] * a + v[1] * b}; };
    GB wGB(Hex(wallHex)), rGB(Hex(Pick(r, METAL3))), dGB(Hex(doorHex)), tGB(Hex(doorHex)), pGB(Hex(ROAD));
    const bool both = W >= 7.5;
    const double oneSide = s.front && std::abs(Dot2(*s.front, v)) > .7 ? (Dot2(*s.front, v) > 0 ? 1 : -1) : 1;
    for (const double sg : {1.0, -1.0}) {
        const double b = sg * W / 2;
        const V3 n{v[0] * sg, 0, v[1] * sg};
        wGB.Quad(Cw(-L / 2, b, yb), Cw(L / 2, b, yb), Cw(L / 2, b, yb + eave), Cw(-L / 2, b, yb + eave), n,
                 {V2{0, 0}, V2{L / RIB_U, 0}, V2{L / RIB_U, 1}, V2{0, 1}});
        pGB.Quad(Cw(-L / 2, b + sg * .02, yb + .03), Cw(L / 2, b + sg * .02, yb + .03), Cw(L / 2, b + sg * 7, yb + .03), Cw(-L / 2, b + sg * 7, yb + .03),
                 {0, 1, 0}, {V2{0, 0}, V2{L / 4, 0}, V2{L / 4, 7.0 / 4}, V2{0, 7.0 / 4}}); // drive aisle
        if (both || sg == oneSide) {
            const int nU = std::max(1, (int)std::floor((L - 1) / 3.05));
            const double st = (L - nU * 3.05) / 2;
            for (int k = 0; k < nU; ++k) {
                const double a = -L / 2 + st + k * 3.05 + 1.525, dw = 2.45, dh = std::min(2.45, eave - .35), bd = b + sg * .04;
                dGB.Quad(Cw(a - dw / 2, bd, yb), Cw(a + dw / 2, bd, yb), Cw(a + dw / 2, bd, yb + dh), Cw(a - dw / 2, bd, yb + dh), n);
                BoxU(tGB, Cw(a + 1.525, b + sg * .05, yb + eave / 2), .1, eave, .1, u, SIDES);
            }
        }
        // gable ends
        const double a = sg * L / 2;
        const V3 ng{u[0] * sg, 0, u[1] * sg};
        wGB.Quad(Cw(a, -W / 2, yb), Cw(a, W / 2, yb), Cw(a, W / 2, yb + eave), Cw(a, -W / 2, yb + eave), ng,
                 {V2{0, 0}, V2{W / RIB_U, 0}, V2{W / RIB_U, 1}, V2{0, 1}});
        Tri3(wGB, Cw(a, -W / 2, yb + eave), Cw(a, W / 2, yb + eave), Cw(a, 0, yb + ridge), ng, {0, 0}, {W / RIB_U, 0}, {W / 2 / RIB_U, .1});
    }
    for (const double sg : {1.0, -1.0}) {
        const V3 N = Nrm({v[0] * sg * .083, 1, v[1] * sg * .083});
        rGB.Quad(Cw(-L / 2 - .2, 0, yb + ridge + .03), Cw(L / 2 + .2, 0, yb + ridge + .03), Cw(L / 2 + .2, sg * (W / 2 + .4), yb + eave - .01),
                 Cw(-L / 2 - .2, sg * (W / 2 + .4), yb + eave - .01), N, {V2{0, 0}, V2{L / RIB_U, 0}, V2{L / RIB_U, 1}, V2{0, 1}});
    }
    BoxU(tGB, Cw(0, W / 2 + .3, yb + eave - .05), L + .4, .18, .14, u); // gutters / fascia
    BoxU(tGB, Cw(0, -W / 2 - .3, yb + eave - .05), L + .4, .18, .14, u);
    wGB.AppendTo(out);
    rGB.AppendTo(out, true);
    for (GB* g : {&dGB, &tGB, &pGB}) g->AppendTo(out);
    return out;
}

} // namespace industrial_detail

inline std::vector<tbd::Vertex> BuildIndustrial(const Structure& s, const HeightFn& hAt) {
    using namespace industrial_detail;
    if (s.pts.size() < 3) return {};
    for (const V2& p : s.pts) if (!std::isfinite(p[0]) || !std::isfinite(p[1])) return {}; // the box fallback would emit NaNs too
    try {
        Prep P;
        if (!MakePrep(s, hAt, P)) return BuildFallback(s, hAt);
        Rng r(s.seed != 0 ? s.seed : 1);
        std::vector<tbd::Vertex> out;
        if (P.sub == Sub::Barn) out = (P.E.size() <= 6 || RectRatio(P.pts, P.O) > .85) ? Barn(s, P, r) : BoxBuilding(s, hAt, P, r, Sub::Flex);
        else if (P.sub == Sub::Storage) out = Storage(s, P, r);
        else out = BoxBuilding(s, hAt, P, r, P.sub);
        if (out.empty()) return BuildFallback(s, hAt);
        for (const tbd::Vertex& q : out)
            if (!std::isfinite(q.px) || !std::isfinite(q.py) || !std::isfinite(q.pz) || !std::isfinite(q.nx)) return BuildFallback(s, hAt);
        return out;
    } catch (...) {
        return BuildFallback(s, hAt);
    }
}

} // namespace optima
