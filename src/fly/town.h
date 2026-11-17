// Town and aircraft builders for the game (portable): Optima fixtures from JSON, a generated residential grid, flat quads,
// and the low-poly trainer. Included once, by fly/game.cpp.
#pragma once
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../core/mini_json.h"
#include "../optima/generators.h"
#include "../render/renderer.h"
#include "flight_model.h"

namespace fly::town {

using tbd::city::scene::Json;

optima::Structure FromJson(const Json& j) {
    optima::Structure s;
    auto num = [&](const char* k, double d) { const Json* v = j.Get(k); return v && v->t == Json::Num ? v->n : d; };
    auto str = [&](const char* k) { const Json* v = j.Get(k); return v && v->t == Json::Str ? v->s : std::string(); };
    if (const Json* p = j.Get("pts"))
        for (const Json& q : p->a) if (q.a.size() >= 2) s.pts.push_back({q.a[0].n, q.a[1].n});
    s.cat = str("cat"); s.cls = str("cls"); s.name = str("name"); s.style = str("style"); s.roof = str("roof"); s.wall = str("wall");
    s.brickCol = str("brickCol");
    s.height = num("height", 0); s.floors = num("floors", 0); s.area = num("area", 0); s.base = num("base", 0);
    s.seed = (int64_t)num("seed", 1);
    if (const Json* b = j.Get("brick")) s.brick = b->t == Json::Bool ? b->b : b->t != Json::Null;
    if (const Json* f = j.Get("front"); f && f->a.size() == 2) s.front = optima::V2{f->a[0].n, f->a[1].n};
    if (s.area <= 0) s.area = std::abs(optima::PolyArea(s.pts));
    return s;
}

std::vector<optima::Structure> LoadFixtures(const std::filesystem::path& dir) {
    std::vector<optima::Structure> out;
    std::error_code ec;
    int slot = 0;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
        if (e.path().extension() != ".json" || e.path().filename().string()[0] == '_') continue;
        std::ifstream f(e.path(), std::ios::binary);
        std::stringstream ss; ss << f.rdbuf();
        Json root; std::string err;
        if (!tbd::city::scene::ParseJson(ss.str(), root, err)) { tbd::Log("fixture %s: %s", e.path().string().c_str(), err.c_str()); continue; }
        const Json* list = root.Get("structures");
        if (!list) continue;
        // each category's fixtures sit in their own block east of the houses
        const double ox = 450 + (slot % 2) * 400, oz = -300 + (slot / 2) * 400;
        ++slot;
        for (const Json& j : list->a) {
            optima::Structure s = FromJson(j);
            for (auto& p : s.pts) { p[0] += ox; p[1] += oz; }
            out.push_back(std::move(s));
        }
        tbd::Log("fixtures: %s, %zu structures", e.path().filename().string().c_str(), list->a.size());
    }
    return out;
}

// Footprints for the residential generator: a street grid of lots, houses rectangular or L-shaped, facing the street.
std::vector<optima::Structure> HouseGrid(double x0, double z0, int nx, int nz, int64_t seed) {
    static const char* const STYLES[] = {"Ranch", "Colonial", "Bi-Level", "Cape Cod", "2 Story", "Bungalow"};
    static const char* const ROOFS[] = {"#55575b", "#3e3f42", "#6b5a4a", "#4a4f57", "#7a6e62"};
    static const char* const WALLS[] = {"#e3dcc8", "#d8d2c4", "#c9cfd3", "#e8e2d0", "#b9c2b0", "#d9c9a8"};
    std::vector<optima::Structure> out;
    optima::Rng r(seed);
    for (int j = 0; j < nz; ++j)
        for (int i = 0; i < nx; ++i) {
            optima::Structure s;
            s.cat = "residential"; s.seed = seed * 1000 + j * nx + i;
            const double cx = x0 + i * 32, cz = z0 + j * 44;
            const bool northSide = (j % 2) == 0; // pairs of rows back to back, street between pairs
            const double w = 11 + r() * 6, d = 8 + r() * 4;
            const double fz = northSide ? 1 : -1; // front normal z
            if (r() < 0.4) { // L: main box plus a wing toward the street
                const double ww = 4 + r() * 3, wd = 3 + r() * 2;
                const double y0 = -d / 2, y1 = d / 2, yf = fz > 0 ? y1 + wd : y0 - wd;
                if (fz > 0) s.pts = {{-w / 2, y0}, {w / 2, y0}, {w / 2, y1}, {-w / 2 + ww, y1}, {-w / 2 + ww, yf}, {-w / 2, yf}};
                else s.pts = {{-w / 2, yf}, {-w / 2 + ww, yf}, {-w / 2 + ww, y0}, {w / 2, y0}, {w / 2, y1}, {-w / 2, y1}};
            } else s.pts = {{-w / 2, -d / 2}, {w / 2, -d / 2}, {w / 2, d / 2}, {-w / 2, d / 2}};
            for (auto& p : s.pts) { p[0] += cx; p[1] += cz; }
            s.front = optima::V2{0, fz};
            s.style = optima::Pick(r, STYLES); s.roof = optima::Pick(r, ROOFS); s.wall = optima::Pick(r, WALLS);
            s.brick = r() < 0.3; s.area = std::abs(optima::PolyArea(s.pts));
            out.push_back(std::move(s));
        }
    return out;
}

void AddFlatQuad(std::vector<tbd::Vertex>& v, double x0, double z0, double x1, double z1, double y, optima::RGB c) {
    optima::GB g(c);
    g.Quad({x0, y, z0}, {x1, y, z0}, {x1, y, z1}, {x0, y, z1}, {0, 1, 0});
    g.AppendTo(v);
}

// ---- the aircraft: a low-poly trainer in body space (-z forward)
std::vector<tbd::Vertex> AircraftMesh() {
    optima::GB body(optima::Hex("#e9e9e6")), stripe(optima::Hex("#b3262c")), glass(optima::Hex("#1d2a36")), dark(optima::Hex("#2b2b2b"));
    optima::Box(body, 0, 0.5, 0, 1, 3.6, 0.55, -0.55, 0.7);    // fuselage (Box: centre x,z; u axis along z)
    optima::Box(glass, 0, -0.6, 0, 1, 0.9, 0.45, 0.7, 1.15);    // canopy
    optima::Box(body, 0, -0.4, 1, 0, 5.2, 0.75, 0.55, 0.68);   // main wing
    optima::Box(stripe, 0, -0.4, 1, 0, 5.25, 0.2, 0.69, 0.7);  // wing stripe
    optima::Box(body, 0, 3.6, 1, 0, 1.8, 0.45, 0.2, 0.3);      // tailplane
    optima::Box(stripe, 0, 3.7, 0, 1, 0.45, 0.06, 0.3, 1.9);   // fin
    optima::Box(dark, 0, -3.2, 1, 0, 0.9, 0.05, -0.4, 0.5);    // propeller disc
    std::vector<tbd::Vertex> v;
    for (auto* g : {&body, &stripe, &glass, &dark}) g->AppendTo(v);
    for (auto& q : v) q.py -= 0.1f;
    return v;
}

tbd::Mat4 BodyMatrix(const fly::Aircraft& a, const fly::WorldPosition& origin) {
    tbd::Mat4 m = tbd::TRS({0, 0, 0}, a.rot, {1, 1, 1});
    m.At(0, 3) = (float)(a.pos.x - origin.x); m.At(1, 3) = (float)(a.pos.y - origin.y); m.At(2, 3) = (float)(a.pos.z - origin.z);
    return m;
}

} // namespace fly::town
