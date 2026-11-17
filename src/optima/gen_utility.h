// C++ port of fly-optima engine/lib/gen/utility.js: sheds, detached garages, pool houses, pump houses / substations.
// The footprint is simplified to its oriented box (walls + gable/hip roof, ~30-60 tris), the door goes on the wall facing
// s.front. Facts normally added by facts/utility.py (kind, roof/wall colours, brick) get seeded defaults that mirror it;
// they use their own rng stream so the geometry draws stay identical to the JS.
#pragma once
#include <cctype>

#include "kit.h"

namespace optima {

inline std::vector<tbd::Vertex> BuildFallback(const Structure& s, const HeightFn& hAt); // defined in generators.h

namespace utility_detail {

inline constexpr const char* SHINGLE[] = {"#2b2b2c", "#3c3d3f", "#55575b", "#5d6063", "#6a6c6c", "#7f7f7a", "#6d6457", "#7a6e5f",
                                          "#5a4a3c", "#6b5040", "#6e5844", "#8a7458", "#9c8a6e", "#4f5862", "#3f4c3c", "#3e4a5a"};
inline constexpr const char* METALR[] = {"#a7aaab", "#4a3d33", "#3f4246", "#3c5244", "#7a2e27", "#e2e2de"};
inline constexpr const char* SIDING[] = {"#f1f0eb", "#ebe5d3", "#e3dcc8", "#d9cbb0", "#d6c9ad", "#c2b49a", "#b8b2a3",
                                         "#a7a9a6", "#7d8487", "#8496a5", "#a3ad94", "#8a7663", "#4d5156", "#3b4a5e"};
inline constexpr const char* BRICKS[] = {"#8e4b3a", "#7a5546", "#b59a7b", "#9a6650"};

inline bool IsHex(const std::string& h) {
    if (h.size() != 7 || h[0] != '#') return false;
    for (size_t i = 1; i < 7; ++i) if (!std::isxdigit((unsigned char)h[i])) return false;
    return true;
}
inline bool Has(const std::string& t, std::initializer_list<const char*> ws) {
    for (const char* w : ws) if (t.find(w) != std::string::npos) return true;
    return false;
}

} // namespace utility_detail

inline std::vector<tbd::Vertex> BuildUtility(const Structure& s, const HeightFn& hAt) {
    using namespace utility_detail;
    if (s.pts.size() < 3) return BuildFallback(s, hAt);
    for (const V2& p : s.pts) if (!std::isfinite(p[0]) || !std::isfinite(p[1])) return BuildFallback(s, hAt);
    try {
        Rng r(s.seed);
        const Frame F = MakeFrame(s.pts);
        if (!(F.L >= 1.2 && F.W >= .9) || F.L > 250) return BuildFallback(s, hAt);
        const double area = s.area > 0 ? s.area : std::abs(PolyArea(s.pts));

        // facts/utility.py defaults on a separate stream (no road / parcel data here)
        Rng fr((int64_t)((uint64_t)s.seed * 0x9E3779B1ull + 0x7F4A7C15ull));
        std::string t = s.cls + " " + s.name + " " + s.style;
        for (char& c : t) c = (char)std::tolower((unsigned char)c);
        std::string kind;
        if (Has(t, {"transformer", "substation", "pump", "utility", "water", "sewage", "lift station", "well"})) kind = "pump";
        else if (Has(t, {"garage", "carport"})) kind = "garage";
        else if (Has(t, {"pool"})) kind = "poolhouse";
        else if (Has(t, {"shed", "storage"})) kind = "shed";
        else if (area >= 30 && fr() < .5) kind = "garage";
        else if (area >= 18 && area < 45 && fr() < .15) kind = "poolhouse";
        else if (area >= 80) kind = "pump"; // big lone outbuilding: municipal/utility hut
        else kind = "shed";

        const bool brick = s.brick || (kind == "pump" && !IsHex(s.wall));
        const std::string wallHex = IsHex(s.wall) ? s.wall : brick ? Pick(fr, BRICKS) : Pick(fr, SIDING);
        const bool metalRoof = kind == "pump" || (kind == "shed" && fr() < .3);
        const std::string roofHex = kind == "pump" ? "#8e877c" : IsHex(s.roof) ? s.roof : metalRoof ? Pick(fr, METALR) : Pick(fr, SHINGLE);

        const double hl = F.L / 2, hw = F.W / 2;
        const Pts pts = {F.ToW(-hl, -hw), F.ToW(hl, -hw), F.ToW(hl, hw), F.ToW(-hl, hw)};
        double yG = MinGround(pts, hAt);
        if (!std::isfinite(yG)) yG = 0;
        yG -= .05;
        const double yF = yG + .12;
        double eh = kind == "shed" ? 2.1 : kind == "garage" ? 2.7 : kind == "poolhouse" ? 2.6 : 3.0;
        if (s.height > 0) eh = std::clamp(kind == "pump" ? s.height - .2 : s.height * .7, 2.0, 12.0); // measured height wins
        const double eave = yF + eh;
        const std::vector<Edge> E = Edges(pts);
        if (E.size() < 3) return BuildFallback(s, hAt);
        const V2 frn = s.front ? *s.front : V2{F.vx, F.vz};

        GB wGB(Hex(wallHex)), rGB(Hex(roofHex)), dGB(Hex("#efeeea")),
            mGB(Hex(kind == "shed" ? "#9c7a52" : kind == "poolhouse" ? "#1d2d3b" : "#555a60"));
        for (const Edge& e : E) WallStrip(wGB, e, yG, eave);
        if (kind == "pump") Box(rGB, F.cx, F.cz, F.ux, F.uz, hl + .05, hw + .05, eave, eave + .2); // flat roof slab with coping
        else {
            std::vector<Rect> R{Rect{-hl, hl, -hw, hw}};
            const double pitch = kind == "shed" ? .42 + r() * .2 : .4 + r() * .12;
            const bool hip = kind == "poolhouse" && r() < .6;
            Roofs(R, F, {eave, pitch, hip, kind == "shed" ? .2 : .3, &rGB, &wGB});
        }
        auto score = [&](const Edge& e) { return (e.n[0] * frn[0] + e.n[1] * frn[1]) * std::min(e.L, 7.0); };
        const Edge* fe = &E[0];
        for (const Edge& e : E) if (score(e) > score(*fe)) fe = &e;
        if (kind == "garage") {
            const double w = fe->L >= 6.6 ? 4.9 : std::min(2.75, fe->L - .6);
            if (w > .4) OnWall(dGB, *fe, fe->L / 2, yF, w, std::min(2.15, eh - .1), .03);
        } else if (kind == "shed") {
            const double w = std::min(1.8, fe->L - .5);
            if (w > .4) OnWall(mGB, *fe, fe->L / 2, yF, w, std::min(1.85, eh - .1), .03);
        } else if (kind == "poolhouse") {
            OnWall(mGB, *fe, fe->L * .35, yF, std::min(.9, fe->L * .5), std::min(2.05, eh - .1), .03);
        } else {
            OnWall(mGB, *fe, fe->L * .3, yF, std::min(1.0, fe->L * .5), std::min(2.15, eh - .1), .03);
            const V2 p{F.cx + frn[0] * (hw + 2.2), F.cz + frn[1] * (hw + 2.2)}; // transformer / control cabinet out front
            Box(mGB, p[0], p[1], F.ux, F.uz, .7, .55, yG, yG + 1.4);
        }
        std::vector<tbd::Vertex> out;
        wGB.AppendTo(out, true);
        rGB.AppendTo(out, true);
        dGB.AppendTo(out);
        mGB.AppendTo(out);
        return out;
    } catch (...) {
        return BuildFallback(s, hAt);
    }
}

} // namespace optima
