// C++ port of fly-optima engine/lib/gen/apartments.js: garden apartments (breezeway stair cores, balconies, pitched roof),
// townhouse rows (garages, stoops, front gables), mid-rise condos and senior living (double-loaded block, lobby,
// porte-cochere). The JS models the building on the footprint's oriented box; here footprints that fill their box poorly
// (L shapes, courtyards) are split into up to 3 rectangles (kit Rects) and each one is built as its own block, so the notch
// of an L is not filled in. Facts that the tiles usually lack (apt type, floors, roof type, wall) get seeded defaults
// mirroring facts/apartments.py. Textures become flat colours; per-building colours come from data/palettes.json.
#pragma once
#include <cctype>
#include <map>

#include "kit.h"

namespace optima {

inline std::vector<tbd::Vertex> BuildFallback(const Structure& s, const HeightFn& hAt); // defined in generators.h

namespace apartments_detail {

constexpr double FH = 3.0, BASE = .35; // floor-to-floor, foundation reveal

// data/palettes.json (object order kept so picks match the JS)
inline constexpr const char* SIDING[] = {"#f1f0eb", "#ebe5d3", "#e3dcc8", "#d9cbb0", "#d6c9ad", "#c2b49a", "#b8b2a3",
                                         "#a7a9a6", "#7d8487", "#8496a5", "#a3ad94", "#8a7663", "#4d5156", "#3b4a5e"};
inline constexpr const char* BRICKS[] = {"#8e4b3a", "#7a5546", "#b59a7b", "#9a6650"};
inline constexpr const char* SHINGLES12[] = {"#2b2b2c", "#3c3d3f", "#55575b", "#5d6063", "#6a6c6c", "#7f7f7a",
                                             "#6d6457", "#7a6e5f", "#5a4a3c", "#6b5040", "#6e5844", "#8a7458"};
inline constexpr const char* FLATS[] = {"#d8d8d4", "#8e877c", "#2a2a2a", "#9a9b98"};
inline constexpr const char* DOORS[] = {"#f1f0eb", "#3b4a5e", "#5a2a27", "#2f3b2f", "#4d5156"};

inline bool IsHex(const std::string& h) {
    if (h.size() != 7 || h[0] != '#') return false;
    for (size_t i = 1; i < 7; ++i) if (!std::isxdigit((unsigned char)h[i])) return false;
    return true;
}
inline std::string Lower(std::string t) { for (char& c : t) c = (char)std::tolower((unsigned char)c); return t; }
inline bool Has(const std::string& t, std::initializer_list<const char*> ws) {
    for (const char* w : ws) if (t.find(w) != std::string::npos) return true;
    return false;
}
inline const char* PickV(Rng& r, const std::vector<const char*>& a) { return a[(size_t)std::floor(r() * a.size()) % a.size()]; }
inline double Lightness(const char* h) { const RGB c = Hex(h); return (std::max({c[0], c[1], c[2]}) + std::min({c[0], c[1], c[2]})) / 2.0; }

// one GB per material/colour key (the JS "bags"); std::map nodes are stable so GB& stays valid
struct Bags {
    std::map<std::string, GB> m;
    GB& Get(const std::string& k, const RGB& c) {
        auto it = m.find(k);
        if (it == m.end()) it = m.emplace(k, GB(c)).first;
        return it->second;
    }
};

// axis-aligned box in the block frame (x = along the building, z = across, +z front), no bottom face
inline void Bx(GB& g, double x0, double x1, double ya, double yb, double z0, double z1, const RGB* col = nullptr, double sc = 3) {
    if (x1 < x0) std::swap(x0, x1);
    if (z1 < z0) std::swap(z0, z1);
    const double w = x1 - x0, h = yb - ya, d = z1 - z0;
    auto uv = [sc](double a, double b) { return std::array<V2, 4>{V2{0, 0}, V2{a / sc, 0}, V2{a / sc, b / sc}, V2{0, b / sc}}; };
    g.Quad({x0, ya, z1}, {x1, ya, z1}, {x1, yb, z1}, {x0, yb, z1}, {0, 0, 1}, uv(w, h), col);
    g.Quad({x1, ya, z0}, {x0, ya, z0}, {x0, yb, z0}, {x1, yb, z0}, {0, 0, -1}, uv(w, h), col);
    g.Quad({x1, ya, z1}, {x1, ya, z0}, {x1, yb, z0}, {x1, yb, z1}, {1, 0, 0}, uv(d, h), col);
    g.Quad({x0, ya, z0}, {x0, ya, z1}, {x0, yb, z1}, {x0, yb, z0}, {-1, 0, 0}, uv(d, h), col);
    g.Quad({x0, yb, z1}, {x1, yb, z1}, {x1, yb, z0}, {x0, yb, z0}, {0, 1, 0}, uv(w, d), col);
}
// flat panel facing +z/-z (side) at depth z, or facing +x/-x when ax
inline void Panel(GB& g, double a0, double a1, double ya, double yb, double z, double side, const RGB* col = nullptr, bool ax = false) {
    auto q = [&](double a, double y) { return ax ? V3{z, y, a} : V3{a, y, z}; };
    const V3 n = ax ? V3{side, 0, 0} : V3{0, 0, side};
    g.Quad(q(a0, ya), q(a1, ya), q(a1, yb), q(a0, yb), n, {V2{0, 0}, V2{1, 0}, V2{1, 1}, V2{0, 1}}, col);
}
// thin two-faced bar (railing baluster) at (x, z), faces +-z or +-x
inline void Post(GB& g, double x, double z, double y0, double y1, double hw, bool facesZ) {
    for (double sd : {1.0, -1.0}) {
        if (facesZ) Panel(g, x - hw, x + hw, y0, y1, z, sd);
        else Panel(g, z - hw, z + hw, y0, y1, x, sd, nullptr, true);
    }
}
// square beam of thickness t from a to b (sloped stair handrail)
inline void Beam(GB& g, const V3& a, const V3& b, double t) {
    const V3 d = Nrm(Sub(b, a));
    V3 p = Cross(d, {0, 1, 0});
    p = Dot(p, p) < 1e-6 ? V3{1, 0, 0} : Nrm(p);
    const V3 q = Nrm(Cross(p, d));
    const double h = t / 2;
    auto off = [&](const V3& o, double s1, double s2) { return V3{o[0] + (p[0] * s1 + q[0] * s2) * h, o[1] + (p[1] * s1 + q[1] * s2) * h, o[2] + (p[2] * s1 + q[2] * s2) * h}; };
    const double S[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        const V3 n = Nrm({p[0] * (S[i][0] + S[j][0]) + q[0] * (S[i][1] + S[j][1]), p[1] * (S[i][0] + S[j][0]) + q[1] * (S[i][1] + S[j][1]),
                          p[2] * (S[i][0] + S[j][0]) + q[2] * (S[i][1] + S[j][1])});
        g.Quad(off(a, S[i][0], S[i][1]), off(a, S[j][0], S[j][1]), off(b, S[j][0], S[j][1]), off(b, S[i][0], S[i][1]), n);
    }
}

struct Gen {
    Rng& r;
    Bags& B;
    bool brickW = false;
    std::string sid1, sid2, sid3;
    RGB brickC{}, roofC{}, trimC{}, doorC{}, flatC{};
    bool flatPicked = false;

    GB& WALL(const std::string& hex) { return B.Get("s" + hex, Hex(hex)); }
    GB& BRICK() { return B.Get("b", brickC); }
    GB& TRIM() { return B.Get("trim", trimC); }
    GB& GLASS() { return B.Get("glass", Hex("#1d2d3b")); }
    GB& CONC() { return B.Get("conc", Hex("#a9a59c")); }
    GB& ROOF() { return B.Get("roof", roofC); }
    GB& DOOR() { return B.Get("door", doorC); }
    GB& GAR() { return B.Get("gar", Hex("#f2f1ec")); }
    GB& METAL() { return B.Get("metal", Hex("#555a60")); }
    GB& WOOD() { return B.Get("wood", Hex("#9c7a52")); }
    GB& WALLB(const std::string& hex) { return brickW ? BRICK() : WALL(hex); }

    // glass + trim surround + mullion (the JS frame of 5 trim boxes is flattened to panels: 6 tris a window)
    void WindowAt(double u, double y, double w, double h, double z, double side, bool ax = false) {
        const double t = .09;
        Panel(TRIM(), u - w / 2 - t, u + w / 2 + t, y - t - .03, y + h + t, z + .03 * side, side, nullptr, ax);
        Panel(GLASS(), u - w / 2, u + w / 2, y, y + h, z + .05 * side, side, nullptr, ax);
        Panel(TRIM(), u - .03, u + .03, y, y + h, z + .06 * side, side, nullptr, ax);
    }
    void Door(double u, double w, double h, double z, double side) {
        Panel(TRIM(), u - w / 2 - .08, u + w / 2 + .08, BASE, BASE + h + .08, z + .015 * side, side);
        Panel(DOOR(), u - w / 2, u + w / 2, BASE, BASE + h, z + .03 * side, side);
    }
    // balcony railing on 3 open sides (z1 = outer edge): rails + balusters every ~1.2 m (JS: every 0.45 m)
    void Railing(double x0, double x1, double z0, double z1, double y, double h = 1.05) {
        GB& M = METAL();
        const double t = .05, zo = z1, zi = z0, sg = zo >= zi ? 1 : -1;
        Bx(M, x0, x1, y + h - t, y + h, zo, zo - t * sg, nullptr, 1);
        Bx(M, x0, x1, y + .08, y + .13, zo, zo - t * sg, nullptr, 1);
        for (double x : {x0, x1}) Bx(M, x - t / 2, x + t / 2, y + h - t, y + h, zi, zo, nullptr, 1);
        const int n = std::max(2, (int)std::lround((x1 - x0) / 1.2));
        for (int i = 0; i <= n; ++i) Post(M, x0 + (x1 - x0) * i / n, zo - .025 * sg, y, y + h, .02, true);
        const int n2 = std::max(1, (int)std::lround(std::abs(zo - zi) / 1.2));
        for (double x : {x0, x1})
            for (int i = 0; i < n2; ++i) Post(M, x, zi + (zo - zi) * i / n2, y, y + h, .02, false);
    }
    void Balcony(double u, double w, double dep, double y, double wallZ, double side, const RGB* slabCol = nullptr) {
        const double z1 = wallZ + dep * side;
        Bx(TRIM(), u - w / 2, u + w / 2, y - .2, y, wallZ, z1, slabCol, 1);
        Railing(u - w / 2, u + w / 2, wallZ, z1, y);
        Panel(GLASS(), u - 1.1, u + 1.1, y, y + 2.15, wallZ + .02 * side, side);
        Bx(TRIM(), u - 1.18, u + 1.18, y + 2.15, y + 2.25, wallZ, wallZ + .06 * side, nullptr, 1);
    }
    // pitched roof over [x0,x1]x[-d/2,d/2] at eave H: gable or hip, overhang .45
    double Roof(double x0, double x1, double d, double H, bool hip, double pitch, const std::string& wallHex) {
        const double ov = .45, rise = std::min(d / 2 * pitch, 4.2), X0 = x0 - ov, X1 = x1 + ov, Z = d / 2 + ov, Hr = H + rise, He = H - ov * pitch;
        GB& R = ROOF();
        const double sl = std::hypot(1.0, pitch);
        auto uvf = [&](double x, double z) { return V2{x / 3, (Z - std::abs(z)) / 3 * sl}; };
        if (hip && x1 - x0 > d * .6) {
            const double ra = x0 + d / 2, rb = std::max(ra + .01, x1 - d / 2);
            for (double sd : {1.0, -1.0}) {
                const V3 a{X0, He, Z * sd}, b{X1, He, Z * sd}, c{rb, Hr, 0}, e{ra, Hr, 0};
                R.UpTri(a, b, c, uvf(a[0], a[2]), uvf(b[0], b[2]), uvf(c[0], 0));
                R.UpTri(a, c, e, uvf(a[0], a[2]), uvf(c[0], 0), uvf(e[0], 0));
            }
            const double ends[2][2] = {{X0, ra}, {X1, rb}};
            for (const auto& en : ends) {
                const V3 a{en[0], He, -Z}, b{en[0], He, Z}, c{en[1], Hr, 0};
                R.UpTri(a, b, c, {a[2] / 3, 0}, {b[2] / 3, 0}, {0, std::abs(en[1] - en[0]) / 3});
            }
        } else {
            for (double sd : {1.0, -1.0}) {
                const V3 a{X0, He, Z * sd}, b{X1, He, Z * sd}, c{X1, Hr, 0}, e{X0, Hr, 0};
                R.UpTri(a, b, c, uvf(a[0], a[2]), uvf(b[0], b[2]), uvf(c[0], 0));
                R.UpTri(a, c, e, uvf(a[0], a[2]), uvf(c[0], 0), uvf(e[0], 0));
            }
            for (double x : {x0, x1}) { // gable-end wall triangles + attic window
                const double sd = x == x0 ? -1 : 1;
                WALLB(wallHex).Tri({x, H, -d / 2}, {x, H, d / 2}, {x, H + rise, 0}, {sd, 0, 0}, {-d / 6, H / 3}, {d / 6, H / 3}, {0, (H + rise) / 3});
                if (rise > 1.6) WindowAt(0, H + .35, .7, .7, x, sd, true);
            }
        }
        Bx(TRIM(), X0, X1, He - .22, He, Z - .03, Z, nullptr, 1); // fascia
        Bx(TRIM(), X0, X1, He - .22, He, -Z, -Z + .03, nullptr, 1);
        return Hr;
    }
    void FlatRoof(double x0, double x1, double d, double H, double par) {
        const RGB fc = Hex(PickV(r, std::vector<const char*>(std::begin(FLATS), std::end(FLATS))));
        if (!flatPicked) { flatC = fc; flatPicked = true; }
        Bx(B.Get("flat", flatC), x0 + .2, x1 - .2, H, H + .05, -d / 2 + .2, d / 2 - .2);
        GB& W = WALLB(sid1);
        const double t = .25;
        Bx(W, x0, x1, H, H + par, d / 2 - t, d / 2); Bx(W, x0, x1, H, H + par, -d / 2, -d / 2 + t);
        Bx(W, x0, x0 + t, H, H + par, -d / 2, d / 2); Bx(W, x1 - t, x1, H, H + par, -d / 2, d / 2);
        // coping (the JS used one slab over the whole roof, which hid the membrane; here it rides the parapet only)
        const double c = .08, y = H + par;
        Bx(TRIM(), x0 - c, x1 + c, y, y + .12, d / 2 - t - c, d / 2 + c, nullptr, 1);
        Bx(TRIM(), x0 - c, x1 + c, y, y + .12, -d / 2 - c, -d / 2 + t + c, nullptr, 1);
        Bx(TRIM(), x0 - c, x0 + t + c, y, y + .12, -d / 2 + t + c, d / 2 - t - c, nullptr, 1);
        Bx(TRIM(), x1 - t - c, x1 + c, y, y + .12, -d / 2 + t + c, d / 2 - t - c, nullptr, 1);
        const int n = std::min(12, std::max(1, (int)std::lround((x1 - x0) * d / 260))); // rooftop condensers / RTUs
        for (int i = 0; i < n; ++i) {
            const double x = x0 + 3 + r() * (x1 - x0 - 6), z = (r() - .5) * (d - 6);
            Bx(B.Get("ac", Hex("#b9bcbc")), x - .6, x + .6, H, H + 1, z - .6, z + .6, nullptr, 1);
        }
    }
    // body wall: siding or brick, optional brick water-table band, concrete foundation, corner boards
    void Body(double x0, double x1, double z0, double z1, double H, const std::string& hex, double brickBase) {
        if (brickW && brickBase <= 0) { Bx(BRICK(), x0, x1, 0, H, z0, z1); return; }
        Bx(CONC(), x0 - .02, x1 + .02, 0, BASE, z0 - .02, z1 + .02, nullptr, 2);
        if (brickBase > 0) { Bx(BRICK(), x0 - .01, x1 + .01, BASE, brickBase, z0 - .01, z1 + .01); Bx(WALL(hex), x0, x1, brickBase, H, z0, z1); }
        else Bx(WALL(hex), x0, x1, BASE, H, z0, z1);
        const double cs[4][2] = {{x0, z0}, {x1, z0}, {x1, z1}, {x0, z1}};
        for (const auto& c : cs) Bx(TRIM(), c[0] - .09, c[0] + .09, BASE, H, c[1] - .09, c[1] + .09, nullptr, 1);
    }
    void StairRun(double xA, double xB, double yA, double yB, double zc, double w) {
        const int n = std::max(3, (int)std::lround(std::abs(yB - yA) / .3));
        GB& C = CONC();
        for (int i = 0; i < n; ++i) {
            const double xa = xA + (xB - xA) * i / n, xb = xA + (xB - xA) * (i + 1) / n, y = yA + (yB - yA) * (i + 1) / n;
            Bx(C, xa, xb, y - .18, y, zc - w / 2, zc + w / 2, nullptr, 1);
        }
        for (double z : {zc - w / 2, zc + w / 2}) Beam(METAL(), {xA, yA + 1, z}, {xB, yB + 1, z}, .05);
    }
    void RoofCanopy(double u, double w, double z, double y) { // small shed roof over a door
        GB& R = ROOF();
        const double d = 1.1;
        R.UpTri({u - w / 2, y, z + d}, {u + w / 2, y, z + d}, {u + w / 2, y + .45, z}, {0, 0}, {w / 3, 0}, {w / 3, .4});
        R.UpTri({u - w / 2, y, z + d}, {u + w / 2, y + .45, z}, {u - w / 2, y + .45, z}, {0, 0}, {w / 3, .4}, {0, .4});
        for (double x : {u - w / 2 + .1, u + w / 2 - .1}) Bx(TRIM(), x - .05, x + .05, y - .5, y + .2, z, z + .12, nullptr, 1);
    }
    void FrontGable(double x0, double x1, double zf, double H, double pitch, const std::string& hex) { // ridge toward +z
        const double w = x1 - x0, xc = (x0 + x1) / 2, rise = w / 2 * pitch, z0 = 0, z1 = zf + .35;
        GB& R = ROOF();
        WALLB(hex).Tri({x0, H, zf}, {x1, H, zf}, {xc, H + rise, zf}, {0, 0, 1}, {0, 0}, {w / 3, 0}, {w / 6, rise / 3});
        const double ab[2][2] = {{x0 - .3, xc}, {x1 + .3, xc}};
        for (const auto& p : ab) {
            const double a = p[0], b = p[1];
            R.UpTri({a, H - .2, z1}, {b, H + rise, z1}, {b, H + rise, z0}, {0, 0}, {1, 1}, {1, 0});
            R.UpTri({a, H - .2, z1}, {b, H + rise, z0}, {a, H - .2, z0}, {0, 0}, {1, 0}, {0, 1});
        }
        if (rise > 1.1) WindowAt(xc, H + .25, .6, .55, zf, 1);
    }

    void Townhouse(double L, double D, int fl, const std::string& roofType) {
        const double hz = D / 2;
        const int n = std::max(2, std::min(14, (int)std::lround(L / 6.5)));
        const double w = L / n, Ht = BASE + std::min(fl, 3) * FH;
        const bool garages = w >= 5.8 && D >= 11;
        const std::string cols[3] = {sid1, sid2, sid3};
        const RGB garC = Hex("#f2f1ec");
        for (int i = 0; i < n; ++i) {
            const double x0 = -L / 2 + i * w, x1 = x0 + w, xc = (x0 + x1) / 2;
            const double off = (i % 2 ? -.5 : .4) * (r() < .8 ? 1 : 0), zf = hz + off;
            const std::string& hex = cols[i % 3];
            const bool band = r() < .4 && !brickW;
            Body(x0, x1, -hz, zf, Ht, hex, band ? BASE + 1.1 : 0);
            if (garages) {
                const double gw = std::min(w - 2.2, 4.9 > w - 2.2 ? 2.7 : 4.9), gx = xc + (i % 2 ? 1 : -1) * (w / 2 - gw / 2 - .4);
                Panel(GAR(), gx - gw / 2, gx + gw / 2, BASE * .3, BASE * .3 + 2.2, zf + .03, 1, &garC);
                Bx(TRIM(), gx - gw / 2 - .12, gx + gw / 2 + .12, 2.3, 2.42, zf, zf + .07, nullptr, 1);
                Bx(CONC(), gx - gw / 2, gx + gw / 2, -.05, .04, zf, zf + 6, nullptr, 2); // driveway apron
                const double dx = xc - (i % 2 ? 1 : -1) * (w / 2 - .9);
                Door(dx, .95, 2.1, zf, 1);
                Bx(CONC(), dx - .8, dx + .8, 0, BASE, zf, zf + 1.3, nullptr, 1); // stoop
                RoofCanopy(dx, 1.7, zf, 2.7);
            } else {
                Door(xc - w / 4, .95, 2.1, zf, 1);
                Bx(CONC(), xc - w / 4 - .8, xc - w / 4 + .8, 0, BASE, zf, zf + 1.3, nullptr, 1);
                RoofCanopy(xc - w / 4, 1.7, zf, 2.7);
                WindowAt(xc + w / 5, BASE + .9, 1.4, 1.5, zf, 1);
            }
            for (int f = 1; f < std::min(fl, 3); ++f) {
                const int nw = w > 7 ? 2 : 1;
                for (int k = 0; k < nw; ++k) WindowAt(x0 + w * (k + .5) / nw, BASE + f * FH + .85, 1.0, 1.45, zf, 1);
            }
            // rear: patio door + windows, privacy fence between patios
            Panel(GLASS(), xc - 1.1, xc + 1.1, BASE, BASE + 2.1, -hz - .02, -1);
            for (int f = 1; f < std::min(fl, 3); ++f) WindowAt(xc, BASE + f * FH + .85, 1.0, 1.45, -hz, -1);
            if (i > 0) Bx(TRIM(), x0 - .1, x0 + .1, BASE, Ht, zf - .2, zf + .08, nullptr, 1); // party-wall trim
            Bx(WOOD(), x0 + .1, x0 + .2, 0, 1.8, -hz - 3.5, -hz, nullptr, 1);
        }
        const double pitch = .42 + r() * .12;
        Roof(-L / 2, L / 2, D + .9, Ht, roofType == "hip", pitch, sid1);
        // front-facing gables over alternating units break up the long roof
        if (D > 9)
            for (int i = (n % 2 ? 0 : 1); i < n; i += 2) { const double x0 = -L / 2 + i * w; FrontGable(x0 + .2, x0 + w - .2, hz + .4, Ht, pitch, cols[i % 3]); }
    }

    void Garden(double L, double D, int fl, const std::string& roofType) {
        const double hz = D / 2, H = BASE + fl * FH, bw = 3.6;
        const int nb = std::max(1, (int)std::lround(L / 32));
        std::vector<double> cores;
        for (int k = 0; k < nb; ++k) cores.push_back(-L / 2 + L * (k + .5) / nb);
        std::vector<std::array<double, 2>> segs;
        double x = -L / 2;
        for (double c : cores) { segs.push_back({x, c - bw / 2}); x = c + bw / 2; }
        segs.push_back({x, L / 2});
        const std::string hex = sid1;
        const bool bandR = r() < .5;
        const double band = bandR && !brickW ? BASE + FH : 0;
        for (const auto& sg : segs) if (sg[1] - sg[0] > 1) Body(sg[0], sg[1], -hz, hz, H, hex, band);
        for (double c : cores) { // breezeway: header, utility wall, landings, switchback stairs, unit doors, entry walk
            GB& C = CONC();
            const double a = c - bw / 2, b = c + bw / 2;
            Bx(WALL(hex), a, b, H - .6, H, -hz, hz);
            Bx(B.Get("dark", Hex("#3a3b3c")), a + .05, b - .05, 0, H - .6, -.15, .15);
            for (int f = 1; f < fl; ++f) {
                Bx(C, a, b, BASE + f * FH - .2, BASE + f * FH, -hz, hz, nullptr, 2);
                for (double sd : {1.0, -1.0}) Railing(a, b, sd * (hz - .2), sd * hz, BASE + f * FH);
            }
            for (int f = 0; f < fl - 1; ++f) {
                const double yA = BASE + f * FH, yB = yA + FH, zc = (f % 2 ? -1 : 1) * hz * .45;
                StairRun(f % 2 ? b - .2 : a + .2, f % 2 ? a + .2 : b - .2, yA, yB, zc, 1.1);
            }
            for (double sd : {1.0, -1.0})
                for (int f = 0; f < fl; ++f) {
                    Panel(DOOR(), -sd * hz * .2 - .48, -sd * hz * .2 + .48, BASE + f * FH, BASE + f * FH + 2.1, a + .01, 1, nullptr, true);
                    Panel(DOOR(), sd * hz * .2 - .48, sd * hz * .2 + .48, BASE + f * FH, BASE + f * FH + 2.1, b - .01, -1, nullptr, true);
                }
            Bx(C, a - .5, b + .5, -.05, BASE, hz, hz + 2.5, nullptr, 2);
        }
        // units: ~8 m bays; ground-floor patios, balconies above, on both long sides
        for (const auto& sg : segs) {
            const double a = sg[0], b = sg[1];
            if (b - a < 3.5) continue;
            const int n = std::max(1, (int)std::lround((b - a) / 8));
            const double w = (b - a) / n;
            for (int i = 0; i < n; ++i) {
                const double u = a + w * (i + .5);
                for (int f = 0; f < fl; ++f) {
                    const double y = BASE + f * FH;
                    for (double sd : {1.0, -1.0}) {
                        const double z = sd * hz, bu = u + (sd > 0 ? -1 : 1) * w * .18, wu = u + (sd > 0 ? 1 : -1) * w * .27;
                        if (f == 0) {
                            Panel(GLASS(), bu - 1.1, bu + 1.1, y, y + 2.15, z + .02 * sd, sd);
                            Bx(CONC(), bu - 1.7, bu + 1.7, -.05, BASE, z, z + 2.4 * sd, nullptr, 2);
                            Railing(bu - 1.7, bu + 1.7, z, z + 2.4 * sd, BASE, .9);
                        } else Balcony(bu, 3.4, 1.7, y + .02, z, sd);
                        if (w > 6.5) WindowAt(wu, y + .9, 1.2, 1.4, z, sd);
                    }
                }
            }
            for (double sd : {1.0, -1.0})
                for (double xe : {a, b})
                    if (std::abs(xe) > L / 2 - .1)
                        for (int f = 0; f < fl; ++f) WindowAt(sd * hz * .4, BASE + f * FH + .9, 1.0, 1.3, xe, xe > 0 ? 1 : -1, true);
        }
        if (roofType == "flat") FlatRoof(-L / 2, L / 2, D, H, .6);
        else {
            const double pitch = .33 + r() * .12;
            Roof(-L / 2, L / 2, D, H, roofType == "hip", pitch, hex);
            if (L > 40) for (double c : cores) FrontGable(c - bw / 2 - 2, c + bw / 2 + 2, hz + .05, H, pitch + .1, hex);
        }
    }

    void MidRise(double L, double D, int fl, const std::string& roofType, bool senior) {
        const double hz = D / 2, H = BASE + fl * FH;
        const std::string hex = senior ? sid3 : sid1;
        const double brickBase = !brickW ? (senior ? BASE + FH : BASE + FH * (fl >= 5 ? 2 : 1)) : 0;
        Body(-L / 2, L / 2, -hz, hz, H, hex, brickBase);
        const double bay = senior ? 4.2 : 7.5;
        const int n = std::max(2, (int)std::lround(L / bay));
        const double w = L / n, lob = std::min(10.0, L * .2);
        const RGB slab = Hex("#c9c9c4");
        for (int i = 0; i < n; ++i) {
            const double u = -L / 2 + w * (i + .5);
            if (std::abs(u) < lob / 2 + .5) continue;
            for (int f = 0; f < fl; ++f) {
                const double y = BASE + f * FH;
                for (double sd : {1.0, -1.0}) {
                    const double z = sd * hz;
                    if (!senior && f > 0 && (i + (sd > 0 ? 0 : 1)) % 2 == 0) Balcony(u, std::min(w - .8, 3.8), 1.8, y + .02, z, sd, &slab);
                    else if (senior) WindowAt(u, y + .85, 1.5, 1.45, z, sd);
                    else { WindowAt(u - w * .2, y + .85, 1.3, 1.5, z, sd); WindowAt(u + w * .2, y + .85, 1.3, 1.5, z, sd); }
                }
            }
        }
        for (double xe : {-L / 2, L / 2})
            for (int f = 0; f < fl; ++f) {
                const std::vector<double> zs = D > 16 ? std::vector<double>{-hz * .5, hz * .5} : std::vector<double>{0};
                for (double zz : zs) WindowAt(zz, BASE + f * FH + .85, 1.2, 1.45, xe, xe > 0 ? 1 : -1, true);
            }
        // lobby: tall glazed entry with mullions, canopy (condo) or porte-cochere (senior)
        Panel(GLASS(), -lob / 2 + .4, lob / 2 - .4, BASE, BASE + FH * 1.6, hz + .03, 1);
        for (int k = 0; k <= 4; ++k) { const double x = -lob / 2 + .4 + (lob - .8) * k / 4; Bx(METAL(), x - .05, x + .05, BASE, BASE + FH * 1.6, hz, hz + .08, nullptr, 1); }
        for (int f = 2; f < fl; ++f) WindowAt(0, BASE + f * FH + .5, lob * .6, 2, hz, 1);
        Bx(CONC(), -lob / 2 - 2, lob / 2 + 2, -.05, BASE, hz, hz + 3, nullptr, 2);
        if (senior) {
            const double dz = 7.5, y = BASE + 4.2, rise = lob / 2 * .5;
            GB& R = ROOF();
            Bx(TRIM(), -lob / 2, lob / 2, y, y + .6, hz, hz + dz, nullptr, 1);
            for (double x : {-lob / 2 + .3, lob / 2 - .3}) Bx(B.Get("col", Hex("#e8e8e8")), x - .25, x + .25, 0, y, hz + dz - .55, hz + dz - .05, nullptr, 1);
            const double ab[2] = {-lob / 2 - .3, lob / 2 + .3};
            for (double a : ab) {
                R.UpTri({a, y + .6, hz + dz + .3}, {0, y + .6 + rise, hz + dz + .3}, {0, y + .6 + rise, hz}, {0, 0}, {1, 1}, {1, 0});
                R.UpTri({a, y + .6, hz + dz + .3}, {0, y + .6 + rise, hz}, {a, y + .6, hz}, {0, 0}, {1, 0}, {0, 1});
            }
            WALL(hex).Tri({-lob / 2, y + .6, hz + dz + .3}, {lob / 2, y + .6, hz + dz + .3}, {0, y + .6 + rise, hz + dz + .3}, {0, 0, 1}, {0, 0}, {lob / 3, 0}, {lob / 6, rise / 3});
            Bx(B.Get("drive", Hex("#4b4c4c")), -lob, lob, -.05, .03, hz + 3, hz + dz + 4, nullptr, 4);
        } else {
            Bx(METAL(), -lob / 2 - .5, lob / 2 + .5, BASE + FH - .3, BASE + FH, hz, hz + 2.6, nullptr, 1);
        }
        if (roofType == "flat") FlatRoof(-L / 2, L / 2, D, H, 1.0);
        else {
            Roof(-L / 2, L / 2, D, H, roofType == "hip", senior ? .4 : .33, hex);
            if (senior && L > 30) FrontGable(-lob / 2 - 1.5, lob / 2 + 1.5, hz + .05, H, .55, hex);
        }
    }
};

struct Block { double cx, cz, ux, uz, L, D; };

} // namespace apartments_detail

inline std::vector<tbd::Vertex> BuildApartments(const Structure& s, const HeightFn& hAt) {
    using namespace apartments_detail;
    const Pts& pts = s.pts;
    if (pts.size() < 3) return BuildFallback(s, hAt);
    for (const V2& p : pts) if (!std::isfinite(p[0]) || !std::isfinite(p[1])) return BuildFallback(s, hAt);
    try {
        const double pa = std::abs(PolyArea(pts));
        const Frame F = MakeFrame(pts);
        if (pa < 40 || F.L < 6 || F.W < 3 || F.L > 600 || !std::isfinite(F.L)) return BuildFallback(s, hAt);

        // blocks: the oriented box (as in the JS), or a rectangle cover when the footprint fills its box poorly
        std::vector<Block> blocks;
        if (pa / std::max(1e-6, F.L * F.W) < .8) {
            for (const Rect& rc : Rects(pts, F, 3)) {
                const double du = rc.u1 - rc.u0, dv = rc.v1 - rc.v0;
                if (std::min(du, dv) < 5 || std::max(du, dv) < 6) continue;
                const V2 c = F.ToW((rc.u0 + rc.u1) / 2, (rc.v0 + rc.v1) / 2);
                if (du >= dv) blocks.push_back({c[0], c[1], F.ux, F.uz, du, dv});
                else blocks.push_back({c[0], c[1], F.vx, F.vz, dv, du});
            }
            if (blocks.size() < 2) blocks.clear();
            if (blocks.empty() && pa / std::max(1e-6, F.L * F.W) < .55) return BuildFallback(s, hAt); // triangles, odd shapes
        }
        const bool single = blocks.empty();
        if (single) { const V2 c = F.ToW(0, 0); blocks.push_back({c[0], c[1], F.ux, F.uz, F.L, F.W}); }

        Rng r(s.seed);
        // classify (JS mirror of facts/apartments.py) on the main block
        const Block& m0 = blocks[0];
        const std::string t = Lower(s.name + " " + s.cls + " " + s.style);
        const double A = s.area > 0 ? s.area : pa;
        const double flGuess = s.floors > 0 ? s.floors : (s.height > 0 ? std::round(s.height / 3.3) : 0);
        std::string apt;
        if (Has(t, {"senior", "retire", "assisted", "nursing", "memory_care"})) apt = "senior";
        else if (flGuess >= 4 || s.height > 13.5) apt = "condo";
        else if (Has(t, {"terrace", "townhouse"}) || (m0.D <= 14.5 && m0.L / std::max(1.0, m0.D) >= 2.6 && A < 2200)) apt = "townhouse";
        else apt = "garden";
        const double flDef = apt == "garden" ? 3 : apt == "townhouse" ? 2 : apt == "condo" ? 4 : 3;
        const int fl = std::max(1, std::min(8, (int)std::lround(flGuess > 0 ? flGuess : flDef)));

        // palette (data/palettes.json), same pick order as the JS
        std::vector<const char*> siding(std::begin(SIDING), std::end(SIDING)), light, shingles(std::begin(SHINGLES12), std::end(SHINGLES12)),
            bricks(std::begin(BRICKS), std::end(BRICKS)), doors(std::begin(DOORS), std::end(DOORS));
        for (const char* h : siding) if (Lightness(h) > .55) light.push_back(h);
        if (light.empty()) light = siding;
        Bags bags;
        Gen g{r, bags};
        g.sid1 = PickV(r, apt == "townhouse" ? siding : light);
        g.sid2 = PickV(r, siding);
        g.sid3 = PickV(r, light);
        const std::string brickHex = PickV(r, bricks), roofHex = PickV(r, shingles);
        const std::string wall = Lower(s.wall);
        if (wall == "brick" || wall == "siding") g.brickW = wall == "brick";
        else if (s.brick) g.brickW = true;
        else g.brickW = r() < .35;
        if (IsHex(s.wall)) { g.sid1 = g.sid2 = g.sid3 = s.wall; g.brickW = false; } // a known wall colour: the whole facade, no random brick
        g.brickC = Hex(IsHex(s.brickCol) ? s.brickCol : IsHex(s.wall) ? s.wall : brickHex);
        g.roofC = Hex(roofHex);
        g.trimC = Hex(r() < .8 ? "#f4f3ee" : "#3b3a38");
        g.doorC = Hex(PickV(r, doors));
        const std::string rt = Lower(s.roof);
        const std::string roofType = (rt == "flat" || rt == "hip" || rt == "gable") ? rt : apt == "condo" ? "flat" : r() < .35 ? "hip" : "gable";
        if (rt != "flat" && rt != "hip" && rt != "gable" && IsHex(s.roof)) g.roofC = Hex(s.roof);
        if (IsHex(s.roof)) { g.flatC = Hex(s.roof); g.flatPicked = true; } // and a known flat roof

        // ground: lowest of the footprint and the block corners so nothing floats
        double y0 = MinGround(pts, hAt);
        std::vector<Block> fb = blocks;
        for (Block& b : fb) {
            if (single) b.D = std::max(8.0, b.D);
            const double vx = -b.uz, vz = b.ux;
            const double cs[5][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}, {0, 0}};
            for (const auto& c : cs) y0 = std::min(y0, hAt(b.cx + c[0] * b.L / 2 * b.ux + c[1] * b.D / 2 * vx, b.cz + c[0] * b.L / 2 * b.uz + c[1] * b.D / 2 * vz));
        }
        if (!std::isfinite(y0)) y0 = 0;
        y0 -= .05;

        for (Block b : fb) {
            // front (+z local) faces s.front when given (the JS 'ent' side), else the frame's +v
            if (s.front && (*s.front)[0] * -b.uz + (*s.front)[1] * b.ux < 0) { b.ux = -b.ux; b.uz = -b.uz; }
            std::map<std::string, size_t> mark;
            for (auto& [k, gb] : bags.m) mark[k] = gb.v.size();
            if (apt == "townhouse") g.Townhouse(b.L, b.D, fl, roofType == "flat" ? "gable" : roofType);
            else if (apt == "garden") g.Garden(b.L, b.D, fl, roofType);
            else g.MidRise(b.L, b.D, fl, roofType, apt == "senior");
            // block frame -> structure local metres: x' = cx + x*u - z*... (v = (-uz, ux)), y + ground
            const float ux = (float)b.ux, uz = (float)b.uz;
            for (auto& [k, gb] : bags.m) {
                const auto it = mark.find(k);
                for (size_t i = it == mark.end() ? 0 : it->second; i < gb.v.size(); ++i) {
                    tbd::Vertex& v = gb.v[i];
                    const float x = v.px, z = v.pz, nx = v.nx, nz = v.nz;
                    v.px = (float)b.cx + x * ux - z * uz;
                    v.pz = (float)b.cz + x * uz + z * ux;
                    v.py = (float)(v.py + y0);
                    v.nx = nx * ux - nz * uz;
                    v.nz = nx * uz + nz * ux;
                }
            }
        }
        std::vector<tbd::Vertex> out;
        for (const auto& [k, gb] : bags.m) gb.AppendTo(out, k == "roof");
        if (out.empty()) return BuildFallback(s, hAt);
        return out;
    } catch (...) {
        return BuildFallback(s, hAt);
    }
}

} // namespace optima
