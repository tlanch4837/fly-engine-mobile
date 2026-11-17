// Optima-Earth structure generators, C++ port of fly-optima engine/lib/gen/{_fallback,residential}.js.
// Build(s, hAt) returns triangles in the structure's local metres (x east, y up, z south). The other categories are in
// gen_<category>.h (public, commercial, apartments, industrial, utility); unknown categories get the extruded box.
#pragma once
#include <regex>

#include "kit.h"

namespace optima {

// Flat stand-ins for the three.js materials in index.html (mats.*).
namespace mat {
inline const RGB concrete = Hex("#a9a59c"), stone = Hex("#8d8a84"), trim = Hex("#f2f1ec"), glass = Hex("#2a3440"),
                 garage = Hex("#e8e6e0"), door = Hex("#5a4632"), white = Hex("#f4f4f2"), brick = Hex("#8a4b38");
}

inline std::vector<tbd::Vertex> BuildFallback(const Structure& s, const HeightFn& hAt) {
    const Pts& pts = s.pts;
    const size_t n = pts.size();
    const double h = s.height > 0 ? s.height : (s.floors > 0 ? s.floors * 3.6 : 4.5);
    const double y0 = MinGround(pts, hAt) - .1, y1 = y0 + .1 + h;
    GB walls(mat::concrete), roof(mat::stone);
    double area = 0;
    for (size_t i = 0; i < n; ++i) { const V2 &a = pts[i], &b = pts[(i + 1) % n]; area += a[0] * b[1] - b[0] * a[1]; }
    const double sgn = area > 0 ? 1 : -1;
    for (size_t i = 0; i < n; ++i) {
        const V2 &a = pts[i], &b = pts[(i + 1) % n];
        const double L = std::hypot(b[0] - a[0], b[1] - a[1]);
        if (L < .05) continue;
        const double nx = (b[1] - a[1]) / L * -sgn, nz = -(b[0] - a[0]) / L * -sgn;
        walls.Quad({a[0], y0, a[1]}, {b[0], y0, b[1]}, {b[0], y1, b[1]}, {a[0], y1, a[1]}, {nx, 0, nz},
                   {V2{0, 0}, V2{L / 3, 0}, V2{L / 3, h / 3}, V2{0, h / 3}});
    }
    Cap(pts, y1, roof);
    std::vector<tbd::Vertex> out;
    walls.AppendTo(out, true);
    roof.AppendTo(out);
    return out;
}

inline std::vector<tbd::Vertex> BuildResidential(const Structure& s, const HeightFn& hAt) {
    static const char* const DOORS[] = {"#6b1f1a", "#1f2f4a", "#24402c", "#ece8e0", "#202020", "#7a5a3a"};
    static const char* const SHUT[] = {"#1d3a2a", "#1c1c1e", "#1f2f4a", "#5a1f1a"};
    Rng r(s.seed);
    const Pts& pts = s.pts;
    std::string style = s.style;
    for (char& c : style) c = (char)std::tolower((unsigned char)c);
    const bool ranch = std::regex_search(style, std::regex("ranch|bungalow|1 story|one story"));
    const bool split = std::regex_search(style, std::regex("bi-?level|split|tri"));
    const bool colonial = std::regex_search(style, std::regex("colonial|2 story|two story"));
    double floorsD = ranch ? 1 : colonial ? 2 : s.floors > 0 ? s.floors : (s.height > 0 ? (s.height > 6.4 ? 2 : 1) : (s.area > 170 ? 2 : 1));
    const int floors = std::max(1, std::min(3, (int)std::lround(floorsD)));
    const double fh = 2.75, found = .45 + r() * .2, yG = MinGround(pts, hAt) - .05, yF = yG + found;
    const double eave = yF + floors * fh + (split ? .9 : 0);
    const Frame F = MakeFrame(pts);
    std::vector<Rect> R = Rects(pts, F, 4);
    const double pitch = ranch ? .38 + r() * .08 : .45 + r() * .14;
    const bool hip = r() < (ranch ? .5 : R.size() > 2 ? .4 : .25);
    const RGB roofC = Hex(s.roof.empty() ? "#55575b" : s.roof);
    GB sGB(Hex(s.wall.empty() ? "#e3dcc8" : s.wall)), bGB(s.brickCol.empty() ? mat::brick : Hex(s.brickCol)), rGB(roofC), fGB(mat::concrete),
        tGB(mat::trim), glGB(mat::glass), gdGB(mat::garage), drGB(mat::door), shGB(mat::trim), pGB(mat::white), capGB(R.empty() ? mat::stone : roofC);
    // walls: foundation band + siding/brick to eave
    const std::vector<Edge> E = Edges(pts);
    if (E.empty()) return BuildFallback(s, hAt);
    const V2 fr = s.front ? *s.front : V2{F.vx, F.vz};
    auto facing = [&](const Edge& e) { return e.n[0] * fr[0] + e.n[1] * fr[1]; };
    const Edge* frontE = &E[0];
    for (const Edge& e : E)
        if (e.L > 2.4 && facing(e) * std::min(e.L, 10.0) > facing(*frontE) * std::min(frontE->L, 10.0)) frontE = &e;
    const bool brickAll = s.brick && ranch, brickFront = s.brick && !ranch;
    for (const Edge& e : E) {
        WallStrip(fGB, e, yG, yF);
        if (brickAll || (brickFront && &e == frontE)) {
            const double yb = brickAll ? eave : yF + fh;
            WallStrip(bGB, e, yF, yb);
            if (yb < eave) WallStrip(sGB, e, yb, eave);
        } else WallStrip(sGB, e, yF, eave);
    }
    const double top = !R.empty() ? Roofs(R, F, {eave, pitch, hip, .35, &rGB, brickAll ? &bGB : &sGB}) : eave;
    Cap(pts, eave - .02, capGB);
    // openings
    const RGB trim = Hex("#f2f1ec"), white = Hex("#efeeea"), doorC = Hex(Pick(r, DOORS)), shutC = Hex(Pick(r, SHUT));
    const bool shutters = !ranch && r() < .55;
    struct Op { const Edge* e; double t, w; };
    std::optional<Op> gar, door;
    if (s.area > 135 && frontE->L >= 9.5) {
        const bool side = r() < .5;
        const double w = frontE->L >= 13 && s.area > 170 ? 4.9 : 2.75;
        gar = Op{frontE, side ? w / 2 + .7 : frontE->L - w / 2 - .7, w};
    }
    if (frontE->L >= 2.2) {
        double t = frontE->L / 2;
        if (gar) t = gar->t < frontE->L / 2 ? std::min(frontE->L - 1.2, gar->t + gar->w / 2 + 2.6) : std::max(1.2, gar->t - gar->w / 2 - 2.6);
        door = Op{frontE, t, 0};
    }
    if (gar) { OnWall(tGB, *gar->e, gar->t, yG + .1, gar->w + .3, 2.3, .02, &trim); OnWall(gdGB, *gar->e, gar->t, yG + .1, gar->w, 2.15, .04, &white); }
    if (door) {
        OnWall(tGB, *door->e, door->t, yF, 1.15, 2.22, .02, &trim);
        OnWall(drGB, *door->e, door->t, yF, .92, 2.03, .04, &doorC);
        const Edge& e = *door->e;
        const V2 p{e.a[0] + e.d[0] * door->t + e.n[0] * .75, e.a[1] + e.d[1] * door->t + e.n[1] * .75};
        Box(fGB, p[0], p[1], e.d[0], e.d[1], .9, .7, yG - .2, yF - .02); // stoop
        if (!ranch && r() < .45) { // little porch roof over the door
            const V2 q{e.a[0] + e.d[0] * door->t + e.n[0] * .6, e.a[1] + e.d[1] * door->t + e.n[1] * .6};
            Box(pGB, q[0], q[1], e.d[0], e.d[1], 1.1, .62, yF + 2.45, yF + 2.6);
        }
    }
    const double winW = .9, winH = 1.35;
    for (const Edge& e : E) {
        if (e.L < 2) continue;
        const bool front = &e == frontE;
        const int cnt = std::max(1, (int)std::floor(e.L / (front ? 2.7 : 3.6)));
        const double sp = e.L / cnt;
        for (int f = 0; f < floors; ++f) {
            const double y = yF + f * fh + .85 + (split && f == 0 ? .5 : 0);
            for (int k = 0; k < cnt; ++k) {
                const double t = sp * (k + .5);
                if (f == 0 && door && door->e == &e && std::abs(t - door->t) < 1.4) continue;
                if (f == 0 && gar && gar->e == &e && std::abs(t - gar->t) < gar->w / 2 + .9) continue;
                if (!front && r() < .3) continue; // side/back walls: fewer windows
                const double ww = front && f == 0 && r() < .25 ? 1.8 : winW;
                OnWall(tGB, e, t, y - .08, ww + .18, winH + .16, .02, &trim);
                OnWall(glGB, e, t, y, ww, winH, .035);
                if (shutters && front && ww < 1) {
                    OnWall(shGB, e, t - ww / 2 - .3, y, .42, winH, .03, &shutC);
                    OnWall(shGB, e, t + ww / 2 + .3, y, .42, winH, .03, &shutC);
                }
            }
        }
    }
    // chimney on the main ridge
    if (!R.empty() && R[0].hasRidge && r() < .45) {
        const V3 &a = R[0].ridge[0], &b = R[0].ridge[1];
        const double t = r() < .5 ? .15 : .85, cx = a[0] + (b[0] - a[0]) * t, cz = a[2] + (b[2] - a[2]) * t;
        Box(bGB, cx, cz, F.ux, F.uz, .55, .45, a[1] - 1.4, top + .9);
    }
    std::vector<tbd::Vertex> out;
    sGB.AppendTo(out, true); bGB.AppendTo(out); rGB.AppendTo(out, true); capGB.AppendTo(out);
    for (GB* g : {&fGB, &tGB, &glGB, &gdGB, &drGB, &shGB, &pGB}) g->AppendTo(out);
    return out;
}

} // namespace optima

#include "gen_apartments.h"
#include "gen_commercial.h"
#include "gen_industrial.h"
#include "gen_public.h"
#include "gen_utility.h"

namespace optima {

inline std::vector<tbd::Vertex> Build(const Structure& s, const HeightFn& hAt) {
    if (s.pts.size() < 3) return {};
    if (s.cat == "residential") return BuildResidential(s, hAt);
    if (s.cat == "commercial") return BuildCommercial(s, hAt);
    if (s.cat == "apartments") return BuildApartments(s, hAt);
    if (s.cat == "industrial") return BuildIndustrial(s, hAt);
    if (s.cat == "public") return BuildPublic(s, hAt);
    if (s.cat == "utility") return BuildUtility(s, hAt);
    return BuildFallback(s, hAt);
}

} // namespace optima
