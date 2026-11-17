// The basic build's baked airports (tools/bake_airport.py -> data/airports/<icao>.json): runways from OpenStreetMap as flat
// strips with painted markings, on a flattened plateau (WorldGen::airfield). The satellite imagery draped on the ground shows the rest
// (taxiways, aprons, terminals), so an airport is a few hundred triangles and one draw. No OS headers.
#pragma once
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../core/mini_json.h"
#include "../render/renderer.h"
#include "../render/vertex.h"
#include "geo.h"

namespace fly {

struct Airport {
    struct Runway {
        std::string ref, ends[2];
        double width = 45, lat[2] = {0, 0}, lon[2] = {0, 0};
    };
    std::string icao, name;
    double lat = 0, lon = 0, flatRadius = 3000;
    std::vector<Runway> runways;

    // every data/airports/*.json, sorted by ICAO code
    static std::vector<Airport> LoadAll(const std::filesystem::path& dir) {
        using tbd::city::scene::Json;
        std::vector<Airport> out;
        std::error_code ec;
        for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
            if (e.path().extension() != ".json") continue;
            std::ifstream f(e.path(), std::ios::binary);
            std::stringstream ss; ss << f.rdbuf();
            Json root; std::string err;
            if (!tbd::city::scene::ParseJson(ss.str(), root, err)) { tbd::Log("airport %s: %s", e.path().string().c_str(), err.c_str()); continue; }
            Airport a;
            auto str = [](const Json* j) { return j ? j->s : std::string(); };
            auto num = [](const Json* j, double d) { return j ? j->n : d; };
            a.icao = str(root.Get("icao")); a.name = str(root.Get("name"));
            a.lat = num(root.Get("lat"), 0); a.lon = num(root.Get("lon"), 0); a.flatRadius = num(root.Get("flat_radius_m"), 3000);
            if (const Json* rws = root.Get("runways"))
                for (const Json& j : rws->a) {
                    Runway rw;
                    rw.ref = str(j.Get("ref")); rw.width = num(j.Get("width_m"), 45);
                    const Json *pa = j.Get("a"), *pb = j.Get("b"), *en = j.Get("ends");
                    if (!pa || !pb || pa->a.size() < 2 || pb->a.size() < 2) continue;
                    rw.lat[0] = pa->a[0].n; rw.lon[0] = pa->a[1].n; rw.lat[1] = pb->a[0].n; rw.lon[1] = pb->a[1].n;
                    if (en && en->a.size() == 2) { rw.ends[0] = en->a[0].s; rw.ends[1] = en->a[1].s; }
                    a.runways.push_back(rw);
                }
            if (!a.runways.empty()) out.push_back(std::move(a));
        }
        std::sort(out.begin(), out.end(), [](const Airport& x, const Airport& y) { return x.icao < y.icao; });
        return out;
    }

    // A runway end by name ("22R"): its threshold and the unit vector down the runway from it (local x, z). False if there is none.
    bool FindEnd(const geo::Anchor& anchor, const std::string& end, double& x, double& z, double& dx, double& dz, double& length) const {
        for (const Runway& rw : runways)
            for (int k = 0; k < 2; ++k) {
                if (!end.empty() && rw.ends[k] != end) continue;
                double x0, z0, x1, z1;
                anchor.ToLocal(rw.lat[k], rw.lon[k], x0, z0);
                anchor.ToLocal(rw.lat[1 - k], rw.lon[1 - k], x1, z1);
                length = std::hypot(x1 - x0, z1 - z0);
                if (length < 1) continue;
                x = x0; z = z0; dx = (x1 - x0) / length; dz = (z1 - z0) / length;
                return true;
            }
        return false;
    }

    // Every runway as a flat strip at height y (local metres round `anchor`): asphalt, then the markings a little above it (edge
    // lines, threshold bars, centreline dashes, aiming points, touchdown zone bars). Lit, untextured (vertex alpha 1).
    std::vector<tbd::Vertex> BuildMesh(const geo::Anchor& anchor, double y) const {
        std::vector<tbd::Vertex> v;
        const float asphalt[3] = {0.36f, 0.36f, 0.35f}, paint[3] = {0.88f, 0.88f, 0.86f}; // weathered concrete / asphalt, white paint
        for (const Runway& rw : runways) {
            double x0, z0, x1, z1;
            anchor.ToLocal(rw.lat[0], rw.lon[0], x0, z0);
            anchor.ToLocal(rw.lat[1], rw.lon[1], x1, z1);
            const double L = std::hypot(x1 - x0, z1 - z0);
            if (L < 100) continue;
            const double ux = (x1 - x0) / L, uz = (z1 - z0) / L, sx = -uz, sz = ux; // along, across
            const double W = rw.width;
            // a rectangle from s0 to s1 along the runway (metres from end a), t0 to t1 across it (from the centreline)
            auto rect = [&](double s0, double s1, double t0, double t1, double h, const float c[3]) {
                const double px[4] = {x0 + ux * s0 + sx * t0, x0 + ux * s1 + sx * t0, x0 + ux * s1 + sx * t1, x0 + ux * s0 + sx * t1};
                const double pz[4] = {z0 + uz * s0 + sz * t0, z0 + uz * s1 + sz * t0, z0 + uz * s1 + sz * t1, z0 + uz * s0 + sz * t1};
                // wound like the town's flat quads, whichever way the runway runs
                const double cr = (px[1] - px[0]) * (pz[2] - pz[0]) - (pz[1] - pz[0]) * (px[2] - px[0]);
                const int order[6] = {0, 1, 2, 0, 2, 3}, flip[6] = {0, 2, 1, 0, 3, 2};
                for (int k = 0; k < 6; ++k) {
                    const int i = cr > 0 ? order[k] : flip[k]; // the same turn as optima::GB::Quad
                    v.push_back({(float)px[i], (float)(y + h), (float)pz[i], 0, 1, 0, c[0], c[1], c[2], 1.0f});
                }
            };
            rect(-30, L + 30, -W * 0.5 - 1.5, W * 0.5 + 1.5, 0.25, asphalt); // clear of the plateau it lies on (else the two z-fight) // the strip, a little past the thresholds
            const double m = 0.35; // the paint: 10 cm above the asphalt
            rect(0, L, -W * 0.5, -W * 0.5 + 0.9, m, paint); // edge lines
            rect(0, L, W * 0.5 - 0.9, W * 0.5, m, paint);
            for (int e = 0; e < 2; ++e) { // each end: threshold bars, aiming point, touchdown zone
                auto at = [&](double s0, double s1, double t0, double t1) { // s from this end's threshold
                    if (e == 0) rect(s0, s1, t0, t1, m, paint); else rect(L - s1, L - s0, t0, t1, m, paint);
                };
                const int bars = W > 50 ? 16 : W > 40 ? 12 : 8;
                const double bw = 1.8, gap = (W - 6.0 - bars * bw) / (bars - 1 + 2.0); // bars evenly across, a wider gap in the middle
                for (int b = 0; b < bars / 2; ++b) {
                    const double t = 3.0 + b * (bw + gap);
                    at(6, 51, W * 0.5 - t - bw, W * 0.5 - t);
                    at(6, 51, -W * 0.5 + t, -W * 0.5 + t + bw);
                }
                if (L > 1200) { at(300, 345, W * 0.25 - 4.5, W * 0.25 + 4.5); at(300, 345, -W * 0.25 - 4.5, -W * 0.25 + 4.5); } // aiming points
                if (L > 2100)
                    for (int k : {150, 450, 600, 750, 900}) // touchdown zone bars, three, two, two, one, one each side
                        for (int n = 0; n < (k == 150 ? 3 : k <= 600 ? 2 : 1); ++n) {
                            const double t = W * 0.25 - 4.0 + n * 3.0;
                            at(k, k + 22.5, t, t + 1.8); at(k, k + 22.5, -t - 1.8, -t);
                        }
            }
            for (double s = 80; s + 36 < L - 80; s += 60) rect(s, s + 36, -0.45, 0.45, m, paint); // centreline dashes
        }
        return v;
    }
};

} // namespace fly
