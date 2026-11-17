// Procedural Earth around an anchor: terrain height, coast, biome colour and urban density, all generated on the client from the
// land mask and the populated places (geo.h). Deterministic everywhere: the noise is sampled on the globe's 3D surface, so the same
// place gets the same hills whichever anchor it is seen from. Immutable once built, so worker threads read it freely.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geo.h"
#include "real.h"

namespace fly {

// ---- 3D value noise on the globe surface
inline uint32_t Hash3(int32_t x, int32_t y, int32_t z, uint32_t seed) {
    uint32_t h = (uint32_t)x * 374761393u + (uint32_t)y * 668265263u + (uint32_t)z * 2147483647u + seed * 2246822519u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}
inline double ValueNoise3(double x, double y, double z, uint32_t seed) {
    const double fx = std::floor(x), fy = std::floor(y), fz = std::floor(z);
    const int32_t ix = (int32_t)fx, iy = (int32_t)fy, iz = (int32_t)fz;
    double tx = x - fx, ty = y - fy, tz = z - fz;
    tx = tx * tx * (3 - 2 * tx); ty = ty * ty * (3 - 2 * ty); tz = tz * tz * (3 - 2 * tz);
    auto v = [&](int a, int b, int c) { return Hash3(ix + a, iy + b, iz + c, seed) * (1.0 / 4294967295.0); };
    const double x00 = v(0, 0, 0) + (v(1, 0, 0) - v(0, 0, 0)) * tx, x10 = v(0, 1, 0) + (v(1, 1, 0) - v(0, 1, 0)) * tx;
    const double x01 = v(0, 0, 1) + (v(1, 0, 1) - v(0, 0, 1)) * tx, x11 = v(0, 1, 1) + (v(1, 1, 1) - v(0, 1, 1)) * tx;
    const double y0 = x00 + (x10 - x00) * ty, y1 = x01 + (x11 - x01) * ty;
    return y0 + (y1 - y0) * tz;
}
// fbm in 0..1 of a point in metres, first octave `scale` metres across
inline double Fbm3(const geo::D3& p, double scale, int octaves, uint32_t seed) {
    double h = 0, amp = 1, sum = 0, f = 1.0 / scale;
    for (int o = 0; o < octaves; ++o) {
        h += ValueNoise3(p.x * f, p.y * f, p.z * f, seed + o * 101) * amp;
        sum += amp; amp *= 0.5; f *= 2.03;
    }
    return h / sum;
}
inline double Smooth(double e0, double e1, double x) { const double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0); return t * t * (3 - 2 * t); }

// What the ground is at one spot.
struct GroundSample {
    double height;  // metres above sea level (sea level is y = 0)
    double coastKm; // + inland
    double density; // urban 0..1
    double lat, lon;
    double moisture;
    double var = 0.5; // slow 0..1 noise at block scale: parks, roof tones, field patches
    uint8_t cover = 0; // real land cover (real::Area class + 1), 0 where there is no map data
    double fine = 0.5; // 0..1 noise at field scale (tens of metres): soil and sprouting crop, mown and rough grass
};

// Climate, roughly: wet tropics, dry subtropics (the deserts' belt), wet temperate westerlies, dry poles; interiors drier than
// coasts; noise breaks it up. 0 desert .. 1 rainforest.
// eastSea 0..1: open sea some hundreds of km to the east or the south (a continent's warm east coast: the US Southeast, east China,
// south-east Brazil and Australia are wet where the west coasts and interiors at the same latitudes are dry).
inline double Moisture(double lat, double coastKm, const geo::D3& P, uint32_t seed, double eastSea = 0.0) {
    static const double A[] = {0, 8, 16, 23, 30, 38, 48, 60, 72, 90};
    static const double M[] = {0.95, 0.85, 0.50, 0.18, 0.15, 0.42, 0.66, 0.60, 0.40, 0.30};
    const double a = std::min(89.9, std::abs(lat));
    int i = 0;
    while (A[i + 1] < a) ++i;
    const double t = (a - A[i]) / (A[i + 1] - A[i]);
    double m = M[i] + (M[i + 1] - M[i]) * t * t * (3 - 2 * t);
    m += (Fbm3(P, 900000.0, 4, seed + 21) - 0.5) * 0.55;
    // continental interiors dry out, most in the subtropical belts (the deserts); at mid latitudes the westerlies still bring enough
    // rain for steppe and prairie (the Great Plains, the Ukrainian and Kazakh steppes, the Pampas), which is farmed
    m -= (0.10 + 0.12 * Smooth(42.0, 30.0, a)) * Smooth(40.0, 250.0, coastKm);
    m += 0.12 * std::exp(-std::max(0.0, coastKm) / 50.0); // sea air on the coasts
    m += 0.32 * eastSea * Smooth(14.0, 22.0, a) * Smooth(44.0, 36.0, a); // the warm east coasts: humid, wooded
    return std::clamp(m, 0.0, 1.0);
}
// The large shape of the land (no small hills): continental rise plus mountain ranges. Also drawn on the globe map.
inline double MacroHeight(const geo::D3& P, double km, uint32_t seed) {
    if (km < 0) return std::max(-400.0, km * 30.0);
    const double base = 420.0 * (1.0 - std::exp(-km / 220.0));
    const double ridge = 1.0 - std::abs(ValueNoise3(P.x / 60000.0, P.y / 60000.0, P.z / 60000.0, seed + 999) * 2.0 - 1.0);
    const double range = Smooth(0.45, 0.75, Fbm3(P, 900000.0, 3, seed + 31)) * Smooth(5.0, 60.0, km);
    return base + std::pow(ridge, 4.0) * 2600.0 * range * (0.6 + 0.8 * Fbm3(P, 20000.0, 4, seed + 41));
}

class WorldGen {
public:
    geo::Anchor anchor;
    const geo::LandMask* land = nullptr;
    bool home = false;               // the Optima test town and runway sit at the world origin (flattened)
    double townHeight = 40, townRadius = 900;
    // An airfield fence (local x0, z0, x1, z1; empty when x1 <= x0): the map's buildings, roads and the city's trees stay outside it
    // (city.h BuildReal / BuildTile). The ground-ops sequence sets it round the home runway (ground_ops.h).
    double keepOut[4] = {0, 0, 0, 0};
    // The basic build's baked airport (fly/airport.h): the ground inside airfield[2] metres of (airfield[0], airfield[1]) sits at airfield[3],
    // ramping back into the generated hills over the next 1500 m. airfield[2] <= 0: none.
    double airfield[4] = {0, 0, 0, 0};
    bool KeepOut(double x, double z) const { return keepOut[2] > keepOut[0] && x > keepOut[0] && x < keepOut[2] && z > keepOut[1] && z < keepOut[3]; }
    double streetAngle = 0;          // radians: the street grid's rotation, from the main city here
    std::string placeName;           // the main city here, for the HUD and the loading screen
    uint32_t seed = 7;

    // Urban density on a grid around the anchor
    static constexpr int kGrid = 800;
    static constexpr double kCell = 250.0, kHalf = kGrid * kCell * 0.5;
    std::vector<float> density, floorCap;
    std::shared_ptr<const real::Local> real; // map data near the anchor (roads everywhere; footprints and land use in extracts)
    // The map coverage the ground's height comes from: the trip's, kept by the copies StreamMap makes as tiles stream in (only
    // `real` changes there). The terrain keeps the trip's WorldGen while the city takes each new copy, so both must see the same
    // heights; a newly mapped area's ground flattens at the next trip or re-anchor.
    std::shared_ptr<const real::Local> ground;

    static std::shared_ptr<WorldGen> Make(const geo::Earth& earth, double lat, double lon, bool home, const real::Data* realData = nullptr) {
        auto g = std::make_shared<WorldGen>();
        g->anchor = geo::Anchor::At(lat, lon);
        if (realData) g->real = real::Local::Make(*realData, g->anchor);
        g->ground = g->real;
        g->land = &earth.land;
        g->home = home;
        g->density.assign((size_t)kGrid * kGrid, 0.0f);
        g->floorCap.assign((size_t)kGrid * kGrid, 3.0f);
        // every place within the grid stamps a disk: radius from its population, peak from its size (a town's centre is a few
        // shops, a metropolis's is towers)
        int main = -1; double mainScore = 0;
        earth.places.Near(lat, lon, kHalf * 1.5, [&](uint32_t i) {
            const geo::Place& p = earth.places.list[i];
            double x, z;
            if (!g->anchor.ToLocal(p.lat, p.lon, x, z)) return;
            const double pop = std::max<double>(p.pop, 1000);
            const double r = 50.0 * std::pow(pop, 0.45) * (p.kind == 4 ? 0.5 : 1.0);
            // suburbs everywhere (density under 0.33); only big cities get a dense core on top, a fraction of their radius across
            const double peak = std::clamp(0.16 + 0.07 * std::log10(pop / 1000.0), 0.12, 0.42) * (p.kind == 4 ? 0.85 : 1.0);
            const double coreAmp = 0.42 * Smooth(5.2, 6.6, std::log10(pop)) * (p.kind == 4 ? 0.0 : 1.0), coreR2 = (r * 0.16) * (r * 0.16);
            const double lg = std::clamp((std::log10(pop) - 4.0) / 3.0, 0.0, 1.0);
            const float cap = (float)(3.0 + 60.0 * lg * lg);
            const int i0 = (int)std::floor((x - r + kHalf) / kCell), i1 = (int)std::floor((x + r + kHalf) / kCell);
            const int j0 = (int)std::floor((z - r + kHalf) / kCell), j1 = (int)std::floor((z + r + kHalf) / kCell);
            for (int j = std::max(0, j0); j <= std::min(kGrid - 1, j1); ++j)
                for (int ii = std::max(0, i0); ii <= std::min(kGrid - 1, i1); ++ii) {
                    const double cx = (ii + 0.5) * kCell - kHalf, cz = (j + 0.5) * kCell - kHalf;
                    const double t2 = ((cx - x) * (cx - x) + (cz - z) * (cz - z)) / (r * r);
                    if (t2 >= 1) continue;
                    const double d2 = (cx - x) * (cx - x) + (cz - z) * (cz - z);
                    const double d = peak * (1 - t2) * (1 - t2) + coreAmp * std::exp(-d2 / std::max(1.0, coreR2));
                    float& dst = g->density[(size_t)j * kGrid + ii];
                    if (d > dst) { dst = (float)d; g->floorCap[(size_t)j * kGrid + ii] = std::max(g->floorCap[(size_t)j * kGrid + ii], cap); }
                }
            const double dist = std::sqrt(x * x + z * z);
            const double score = std::sqrt(pop) / (1.0 + dist / 3000.0) * (p.kind == 4 ? 0.3 : 1.0);
            if (score > mainScore) { mainScore = score; main = (int)i; }
        });
        if (main >= 0) {
            const geo::Place& p = earth.places.list[main];
            g->placeName = p.name + (p.cc[0] && p.cc[0] != '-' ? std::string(", ") + p.cc : std::string());
            uint32_t h = 2166136261u;
            for (char c : p.name) h = (h ^ (uint8_t)c) * 16777619u;
            // most cities run close to north-south; some are turned
            const double r01 = (h % 1000) / 1000.0;
            g->streetAngle = r01 < 0.45 ? 0.0 : (r01 - 0.725) * 1.2; // 0, or within about +-19 degrees
        }
        // break the disks' perfect circles up: noise eats the edges, so suburbs fray into fields
        for (int j = 0; j < kGrid; ++j)
            for (int ii = 0; ii < kGrid; ++ii) {
                float& d = g->density[(size_t)j * kGrid + ii];
                if (d <= 0) continue;
                const double cx = (ii + 0.5) * kCell - kHalf, cz = (j + 0.5) * kCell - kHalf;
                const geo::D3 P = g->anchor.UnitAt(cx, cz) * geo::kEarthR;
                const double n = Fbm3(P, 3000.0, 3, 555) - 0.5;
                d = (float)std::max(0.0, d + n * 0.35 * std::min(1.0, 0.25 + d * 2.0) - 0.02);
            }
        return g;
    }

    // Bilinear urban density at local x, z.
    double Density(double x, double z) const {
        const double fx = (x + kHalf) / kCell - 0.5, fz = (z + kHalf) / kCell - 0.5;
        const int i = (int)std::floor(fx), j = (int)std::floor(fz);
        if (i < 0 || j < 0 || i >= kGrid - 1 || j >= kGrid - 1) return 0.0;
        const double tx = fx - i, tz = fz - j;
        auto at = [&](int a, int b) { return (double)density[(size_t)b * kGrid + a]; };
        return (at(i, j) * (1 - tx) + at(i + 1, j) * tx) * (1 - tz) + (at(i, j + 1) * (1 - tx) + at(i + 1, j + 1) * tx) * tz;
    }
    double FloorCap(double x, double z) const {
        const int i = (int)std::floor((x + kHalf) / kCell), j = (int)std::floor((z + kHalf) / kCell);
        if (i < 0 || j < 0 || i >= kGrid || j >= kGrid) return 3.0;
        return floorCap[(size_t)j * kGrid + i];
    }

    GroundSample Sample(double x, double z) const {
        GroundSample s{};
        const geo::D3 u = anchor.UnitAt(x, z);
        const geo::D3 P = u * geo::kEarthR;
        geo::LatLon(u, s.lat, s.lon);
        const double coast = land ? land->CoastKm(s.lat, s.lon) : 60.0;
        // the mask is ~10 km per texel: fractal noise gives the coast its bays, headlands and islands
        const double near = std::clamp(1.0 - std::abs(coast) / 60.0, 0.15, 1.0);
        const double km = coast + ((Fbm3(P, 38000.0, 4, seed + 11) - 0.5) * 22.0 + (Fbm3(P, 5000.0, 4, seed + 12) - 0.5) * 4.0) * near;
        s.coastKm = km;
        double eastSea = 0.0; // the sea to the east (6 degrees) or the south (5 degrees), from the land mask
        if (land) {
            const double ce = land->CoastKm(s.lat, s.lon + 6.0), cs = land->CoastKm(s.lat - (s.lat >= 0 ? 5.0 : -5.0), s.lon);
            eastSea = std::max(Smooth(0.0, -150.0, ce), Smooth(0.0, -150.0, cs) * 0.8);
        }
        s.moisture = Moisture(s.lat, km, P, seed, eastSea);
        s.density = Density(x, z);
        s.var = ValueNoise3(P.x / 450.0, P.y / 450.0, P.z / 450.0, seed + 61);
        s.fine = ValueNoise3(P.x / 38.0, P.y / 38.0, P.z / 38.0, seed + 71) * 0.65 + ValueNoise3(P.x / 11.0, P.y / 11.0, P.z / 11.0, seed + 72) * 0.35;
        const bool mapped = real && real->Detailed(x, z);
        if (real) s.cover = real->CoverAt(x, z);
        if (mapped && s.cover == 0) s.cover = 8; // mapped, but no land use drawn here: in practice open grass (lawns, verges, rough lots)
        if (km < 0) { // sea floor, deepening offshore; the sea surface is drawn at y = 0
            s.height = std::max(-400.0, km * 30.0);
            return s;
        }
        // continents rise inland; hills everywhere; ranges where a slow ridge field says so
        const double base = 420.0 * (1.0 - std::exp(-km / 220.0));
        const double hills = (Fbm3(P, 9000.0, 8, seed) - 0.5) * 2.0;
        const double amp = 30.0 + 220.0 * Smooth(0.0, 80.0, km);
        const double ridge = 1.0 - std::abs(ValueNoise3(P.x / 60000.0, P.y / 60000.0, P.z / 60000.0, seed + 999) * 2.0 - 1.0);
        const double range = Smooth(0.45, 0.75, Fbm3(P, 900000.0, 3, seed + 31)) * Smooth(5.0, 60.0, km);
        const double mountains = std::pow(ridge, 4.0) * 2600.0 * range * (0.6 + 0.8 * Fbm3(P, 20000.0, 4, seed + 41));
        double h = base + hills * amp + mountains;
        const double shore = Smooth(0.0, 1.8, km); // beaches: the land slides down to the water's edge
        h = h * shore + 1.2 * (1.0 - shore) * Smooth(0.0, 0.3, km);
        // how mapped the ground round here is, continuously: "mapped" sampled on a fixed 700 m lattice, each node tent-blurred over
        // its 3 x 3 neighbours, then interpolated bilinearly. The mapped ground's flat plain then ramps into the procedural hills over
        // about 2 km at the map data's edge instead of stepping into a cliff (a packed area's border, Midland's at 43.6 N).
        double mappedW = 0.0;
        if (const real::Local* cov = ground.get()) {
            constexpr double L = 700.0;
            const double fx = x / L, fz = z / L, ix = std::floor(fx), iz = std::floor(fz), tx = fx - ix, tz = fz - iz;
            const auto node = [&](double i, double j) {
                double v = 0.0;
                for (int b = -1; b <= 1; ++b)
                    for (int a = -1; a <= 1; ++a)
                        if (cov->Detailed((i + a) * L, (j + b) * L)) v += (2 - std::abs(a)) * (2 - std::abs(b));
                return v / 16.0;
            };
            const double n00 = node(ix, iz), n10 = node(ix + 1, iz), n01 = node(ix, iz + 1), n11 = node(ix + 1, iz + 1);
            mappedW = Smooth(0.0, 1.0, (n00 + (n10 - n00) * tx) * (1 - tz) + (n01 + (n11 - n01) * tx) * tz);
        }
        if (mappedW > 0.0) { // mapped country is drawn as it is; here that means the flat lake plain (no elevation data yet)
            const double flat = base * shore + hills * amp * 0.05 * shore + 2.0;
            h += (flat - h) * Smooth(0.0, 0.5, km) * 0.97 * mappedW;
        }
        if (mappedW < 1.0 && s.density > 0.02) { // towns sit on smoothed, gentle ground
            const double flat = base * shore + hills * amp * 0.12 * shore + 2.0;
            h += (flat - h) * std::min(1.0, s.density * 2.2) * (1.0 - mappedW);
        }
        if (home) { // the test town and its runway on a flat plateau
            const double d = std::sqrt(x * x + z * z);
            if (d < townRadius * 2) {
                const double t = std::clamp((d - townRadius) / townRadius, 0.0, 1.0), sm = t * t * (3 - 2 * t);
                h = townHeight * (1 - sm) + h * sm;
            }
        }
        if (airfield[2] > 0.0) { // the airport's plateau
            const double d = std::hypot(x - airfield[0], z - airfield[1]);
            const double t = std::clamp((d - airfield[2]) / 1500.0, 0.0, 1.0), sm = t * t * (3 - 2 * t);
            h = airfield[3] * (1 - sm) + h * sm;
        }
        s.height = h;
        return s;
    }
    double Height(double x, double z) const {
        const double h = Sample(x, z).height;
        return h;
    }

    // The ground under real land cover, as it looks from the air in late spring (muted: the air does the rest).
    static bool CoverColour(const GroundSample& s, float out[3]) {
        const double v = s.var, f = s.fine;
        auto set = [&](double r, double g, double b) { out[0] = (float)r; out[1] = (float)g; out[2] = (float)b; return true; };
        // vegetated ground reads much darker than its paved neighbours from the air (blades shade each other): measured against the DTW
        // photo, lawns are about a third as bright as the lots beside them
        auto veg = [&](double r, double g, double b) { return set(r * 0.36, g * 0.56, b * 0.24); }; // the air greys dark ground: keep green ahead of red and blue
        auto mix = [](double a, double b, double t) { return a + (b - a) * t; };
        switch (s.cover - 1) {
        case 0: return set(0.27 + 0.03 * f, 0.27 + 0.03 * f, 0.28 + 0.03 * f);                               // parking (cars drawn over it)
        case 1: return set(0.50 + 0.05 * f, 0.50 + 0.05 * f, 0.49 + 0.05 * f);                               // apron concrete
        case 2: return set(0.40, 0.40, 0.40);                                                                // runway and taxiway strips
        case 3: { const double t = (0.25 + 0.5 * f) * 0.35;                                                  // residential: mowed lawns (roofs drawn)
            return veg(mix(0.21, 0.30, t), mix(0.32, 0.30, t), mix(0.09, 0.16, t)); }
        case 4: case 5: case 15: case 16: { const double t = 0.82 + 0.18 * f; // mostly mown green; a little wear (not grey blotches)                               // commercial: mowed verges and lawns between the lots
            return set(mix(0.30, 0.21 * 0.36, t), mix(0.30, 0.32 * 0.56, t), mix(0.25, 0.09 * 0.24, t)); }
        case 6: { const double green = Smooth(0.35, 0.75, f * 0.7 + v * 0.3);                                   // fields: soil with a fresh crop coming
            return set(mix(0.33, 0.20, green), mix(0.28, 0.30, green), mix(0.20, 0.11, green)); }
        case 11: case 14: { const double t = Smooth(0.3, 0.8, f);                                               // parks, cemeteries: mowed, a rich spring green
            return veg(mix(0.19, 0.24, t), mix(0.31, 0.34, t), mix(0.08, 0.11, t)); }
        case 7: { // open grass land: rough meadow, last year's straw with green coming through, patchy at field scale (the DTW photo's open lots)
            // broad, gentle: the colour lives on terrain vertices metres apart, so fine noise here aliases into triangle blotches; the
            // shader adds the small-scale tufts and dry patches per pixel
            const double green = 0.62 + 0.22 * (v - 0.5) + 0.08 * (f - 0.5), straw = 1.0 - green;
            return veg(0.36 * straw + 0.21 * green, 0.34 * straw + 0.31 * green, 0.21 * straw + 0.10 * green); }
        case 12: case 13: return set(0.17 + 0.03 * f, 0.33 + 0.03 * f, 0.10);                                 // golf, pitches
        case 8: return set(0.16 + 0.04 * f, 0.24 + 0.04 * f, 0.07 + 0.02 * f);                               // woods in May: fresh yellow-green leaf (trees drawn too)
        case 9: return set(0.17, 0.22, 0.23);                                                                // ponds and basins
        case 10: return set(0.20 + 0.05 * f, 0.25 + 0.04 * f, 0.13);                                          // wetland
        default: return false;
        }
    }

    // How much of the ground here is worked farmland, 0..1 (terrain vertex alpha 0.08 .. 0.10, mesh.hlsl FarmField draws the fields):
    // the mapped land use where there is some; else the temperate and subtropical belts with enough rain for crops (or irrigation:
    // the semi-arid plains are farmed edge to edge), flat or gently rolling, out of the towns, below the high country, with some
    // land left as pasture, woods and prairie.
    static double FarmWeight(const GroundSample& s, double slope) {
        if (s.cover) return s.cover == 7 ? 1.0 : 0.0; // mapped: farmland (land use class 6)
        if (s.height < 1.5) return 0.0;
        const double alat = std::abs(s.lat), m = s.moisture;
        const double belt = Smooth(22.0, 28.0, alat) * Smooth(58.0, 52.0, alat);
        const double rain = Smooth(0.02, 0.1, m) * Smooth(0.9, 0.75, m) * (1.0 - 0.7 * Smooth(0.08, 0.02, m) * Smooth(40.0, 33.0, alat)); // dry farming reaches far into semi-arid land
        const double rural = 1.0 - Smooth(0.01, 0.045, s.density);
        const double flat = Smooth(0.2, 0.08, slope) * Smooth(1700.0, 1000.0, s.height);
        const double kept = Smooth(0.18, 0.32, s.var); // pasture, woods, prairie left between the farms
        // wet temperate country stays largely wooded (the US Southeast's piedmont, much of Europe's hill country): fields in its
        // clearings only, one place in three
        const double woods = 1.0 - Smooth(0.5, 0.72, m) * Smooth(0.45, 0.62, s.var) * 0.85;
        return std::clamp(belt * rain * rural * flat * kept * woods, 0.0, 1.0);
    }

    // Ground colour for terrain vertices: biome from latitude and moisture, snow by a latitude-dependent snowline, rock on slopes,
    // grey where the town is.
    static void Colour(const GroundSample& s, double slope, float out[3]) {
        const double h = s.height, alat = std::abs(s.lat);
        if (s.cover && h >= 0.8 && CoverColour(s, out)) return;
        if (h < 0.8) { out[0] = 0.74f; out[1] = 0.68f; out[2] = 0.52f; return; } // beach / sea floor
        const double snowline = std::max(0.0, 5200.0 - 85.0 * std::max(0.0, alat - 18.0));
        double r, g, b;
        const double m = s.moisture;
        // true desert only where it is very dry: the semi-arid interiors (the Great Plains, the steppes) are grass and crops
        const double desert = Smooth(0.14, 0.05, m) * Smooth(6.0, 14.0, alat) * Smooth(40.0, 33.0, alat);
        const double boreal = Smooth(48.0, 58.0, alat), tundra = Smooth(62.0, 70.0, alat);
        // temperate grass -> forest by moisture; deep green in the wet tropics
        r = 0.26 - 0.13 * m; g = 0.36 - 0.07 * m; b = 0.14 - 0.04 * m;
        const double jungle = Smooth(0.72, 0.9, m) * Smooth(24.0, 12.0, alat);
        r += (0.07 - r) * jungle; g += (0.24 - g) * jungle; b += (0.08 - b) * jungle;
        // farmland patches in the temperate belts
        const double farm = Smooth(0.35, 0.55, m) * Smooth(28.0, 34.0, alat) * Smooth(58.0, 52.0, alat) * Smooth(0.55, 0.75, s.var);
        r += (0.46 - r) * farm * 0.5; g += (0.44 - g) * farm * 0.5; b += (0.22 - b) * farm * 0.5;
        // dry grass / scrub
        const double dry = Smooth(0.5, 0.2, m); // short-grass steppe: grey-green with straw, not sand
        r += (0.40 - r) * dry * 0.7; g += (0.40 - g) * dry * 0.7; b += (0.24 - b) * dry * 0.7;
        // desert sand
        r += (0.78 - r) * desert; g += (0.64 - g) * desert; b += (0.45 - b) * desert;
        // boreal: dark conifer green; tundra: grey-brown
        r += (0.12 - r) * boreal * (1 - tundra); g += (0.22 - g) * boreal * (1 - tundra); b += (0.13 - b) * boreal * (1 - tundra);
        r += (0.42 - r) * tundra; g += (0.40 - g) * tundra; b += (0.34 - b) * tundra;
        // uplands dry out
        const double up = Smooth(600.0, 1800.0, h) * 0.6;
        r += (0.45 - r) * up; g += (0.41 - g) * up; b += (0.34 - b) * up;
        // rock on steep slopes and high up
        const double rock = std::max(Smooth(0.45, 0.7, slope), Smooth(snowline - 900.0, snowline - 200.0, h) * 0.8);
        r += (0.40 - r) * rock; g += (0.37 - g) * rock; b += (0.34 - b) * rock;
        // snow and ice
        const double snow = std::max(Smooth(snowline - 150.0, snowline + 150.0, h) * Smooth(0.75, 0.55, slope), Smooth(74.0, 80.0, alat));
        r += (0.93 - r) * snow; g += (0.94 - g) * snow; b += (0.96 - b) * snow;
        // suburbs from afar: lawns, tree canopy and roofs speckled together, mostly green
        const double sub = Smooth(0.03, 0.12, s.density) * (1.0 - Smooth(0.30, 0.45, s.density));
        const double roofs = Smooth(0.35, 0.75, s.var);
        r += ((0.24 + 0.16 * roofs) - r) * sub * 0.75; g += ((0.33 + 0.04 * roofs) - g) * sub * 0.75; b += ((0.17 + 0.14 * roofs) - b) * sub * 0.75;
        // dense towns: a mosaic of roofs, paving and park trees, warmer or cooler block by block
        const double urban = std::min(0.8, std::max(0.0, s.density - 0.25) * 2.5);
        const double v = s.var;
        double ur = 0.46 + 0.10 * (v - 0.5), ug = 0.43 + 0.04 * (v - 0.5), ub = 0.40 - 0.06 * (v - 0.5);
        const double park = Smooth(0.78, 0.9, v) * Smooth(0.75, 0.3, s.density); // parks, fewer in the core
        ur += (0.17 - ur) * park; ug += (0.29 - ug) * park; ub += (0.13 - ub) * park;
        r += (ur - r) * urban; g += (ug - g) * urban; b += (ub - b) * urban;
        out[0] = (float)r; out[1] = (float)g; out[2] = (float)b;
    }
};

} // namespace fly
