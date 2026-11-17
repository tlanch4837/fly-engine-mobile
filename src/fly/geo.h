// The whole Earth for Fly-engine: lat/lon <-> a local flat frame around an anchor, the populated places (GeoNames cities1000,
// CC-BY 4.0) and the land mask (Natural Earth, public domain), both packed by tools/pack_world.py into data/world/.
// Local frame (as everywhere in the game): x east, y up, z south, metres; the anchor is local (0, 0). The frame is the tangent
// plane at the anchor with exact orthographic projection onto a spherical Earth, good for a few hundred km. No OS headers.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fly::geo {

constexpr double kEarthR = 6371000.0;
constexpr double kPi = 3.14159265358979323846;
inline double Rad(double d) { return d * kPi / 180.0; }
inline double Deg(double r) { return r * 180.0 / kPi; }

struct D3 { double x, y, z; };
inline D3 operator+(D3 a, D3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline D3 operator-(D3 a, D3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline D3 operator*(D3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double Dot(D3 a, D3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline D3 Norm(D3 a) { const double l = std::sqrt(Dot(a, a)); return l > 0 ? a * (1.0 / l) : D3{0, 0, 1}; }

// Unit vector for a latitude / longitude in degrees (ECEF axes: x through 0,0; z through the north pole).
inline D3 Unit(double lat, double lon) {
    const double a = Rad(lat), b = Rad(lon);
    return {std::cos(a) * std::cos(b), std::cos(a) * std::sin(b), std::sin(a)};
}
inline void LatLon(D3 u, double& lat, double& lon) {
    u = Norm(u);
    lat = Deg(std::asin(std::clamp(u.z, -1.0, 1.0)));
    lon = Deg(std::atan2(u.y, u.x));
}
// Great-circle distance in metres.
inline double Distance(double lat0, double lon0, double lat1, double lon1) {
    return kEarthR * std::acos(std::clamp(Dot(Unit(lat0, lon0), Unit(lat1, lon1)), -1.0, 1.0));
}

// The tangent frame at an anchor.
struct Anchor {
    double lat = 0, lon = 0;
    D3 up{1, 0, 0}, east{0, 1, 0}, north{0, 0, 1};
    static Anchor At(double la, double lo) {
        Anchor a;
        a.lat = la; a.lon = lo;
        a.up = Unit(la, lo);
        const double b = Rad(lo), c = Rad(la);
        a.east = {-std::sin(b), std::cos(b), 0};
        a.north = {-std::sin(c) * std::cos(b), -std::sin(c) * std::sin(b), std::cos(c)};
        return a;
    }
    // Local x (east), z (south) -> the unit vector of the point on the sphere below it.
    D3 UnitAt(double x, double z) const {
        const double r2 = kEarthR * kEarthR - x * x - z * z;
        return Norm(east * x + north * (-z) + up * std::sqrt(std::max(r2, 0.0)));
    }
    void ToLatLon(double x, double z, double& la, double& lo) const { LatLon(UnitAt(x, z), la, lo); }
    // Lat/lon -> local x, z (orthographic onto the tangent plane). `front` is false on the far side of the planet.
    bool ToLocal(double la, double lo, double& x, double& z) const {
        const D3 p = Unit(la, lo) * kEarthR;
        x = Dot(p, east); z = -Dot(p, north);
        return Dot(p, up) > 0;
    }
    // How far the sphere drops below the tangent plane at local x, z (metres): the horizon's curve.
    double Drop(double x, double z) const { const double d2 = x * x + z * z; return d2 / (2.0 * kEarthR); }
};

// ---- populated places, most populous first
struct Place {
    float lat, lon;
    uint32_t pop;
    uint8_t kind;    // 0 town, 1 county seat, 2 regional capital, 3 national capital, 4 neighbourhood
    char cc[3];      // country code
    std::string name;
};

class Places {
public:
    bool Load(const std::filesystem::path& file) {
        std::ifstream f(file, std::ios::binary);
        if (!f) return false;
        char magic[4]; uint32_t n = 0;
        f.read(magic, 4); f.read((char*)&n, 4);
        if (std::memcmp(magic, "FLYP", 4) != 0) return false;
        list.resize(n);
        for (uint32_t i = 0; i < n && f; ++i) {
            Place& p = list[i];
            uint8_t len = 0;
            f.read((char*)&p.lat, 4); f.read((char*)&p.lon, 4); f.read((char*)&p.pop, 4); f.read((char*)&p.kind, 1);
            f.read(p.cc, 2); p.cc[2] = 0; f.read((char*)&len, 1);
            p.name.resize(len); f.read(p.name.data(), len);
        }
        // a coarse lat/lon bucket grid (1 degree) for "what is near here" queries
        for (uint32_t i = 0; i < n; ++i) buckets_[Bucket(list[i].lat, list[i].lon)].push_back(i);
        return !list.empty();
    }
    // Every place within `radius` metres of lat/lon (indices into list), any order.
    template <class F> void Near(double lat, double lon, double radius, F&& f) const {
        const double dLat = Deg(radius / kEarthR) + 1.0;
        const double dLon = std::min(180.0, dLat / std::max(0.05, std::cos(Rad(std::min(89.0, std::abs(lat) + dLat)))));
        const D3 c = Unit(lat, lon);
        const double cosR = std::cos(std::min(kPi, radius / kEarthR));
        for (int bl = (int)std::floor(lat - dLat); bl <= (int)std::floor(lat + dLat); ++bl) {
            if (bl < -90 || bl > 89) continue;
            for (int bo = (int)std::floor(lon - dLon); bo <= (int)std::floor(lon + dLon); ++bo) {
                const int wo = ((bo + 180) % 360 + 360) % 360 - 180;
                const auto& b = buckets_[(bl + 90) * 360 + (wo + 180)];
                for (uint32_t i : b)
                    if (Dot(Unit(list[i].lat, list[i].lon), c) >= cosR) f(i);
            }
        }
    }
    // The most fitting name for a spot: the biggest place, weighted by how close it is.
    int Nearest(double lat, double lon, double radius) const {
        int best = -1; double bestScore = 0;
        Near(lat, lon, radius, [&](uint32_t i) {
            const double d = Distance(lat, lon, list[i].lat, list[i].lon);
            const double score = std::sqrt((double)list[i].pop) / (1.0 + d / 1500.0) * (list[i].kind == 4 ? 0.3 : 1.0);
            if (score > bestScore) { bestScore = score; best = (int)i; }
        });
        return best;
    }
    std::vector<Place> list;

private:
    static int Bucket(double lat, double lon) {
        const int bl = std::clamp((int)std::floor(lat) + 90, 0, 179), bo = std::clamp((int)std::floor(lon) + 180, 0, 359);
        return bl * 360 + bo;
    }
    std::vector<uint32_t> buckets_[180 * 360];
};

// ---- the land mask: signed distance to the coast in km (+ on land), 2 km per step, on a lon/lat grid
class LandMask {
public:
    bool Load(const std::filesystem::path& file) {
        std::ifstream f(file, std::ios::binary);
        if (!f) return false;
        char magic[4];
        f.read(magic, 4); f.read((char*)&w, 4); f.read((char*)&h, 4);
        if (std::memcmp(magic, "FLYL", 4) != 0 || w <= 0 || h <= 0) return false;
        sdf.resize((size_t)w * h);
        f.read((char*)sdf.data(), (std::streamsize)sdf.size());
        return (bool)f;
    }
    bool Ok() const { return !sdf.empty(); }
    // Bilinear distance to the coast in km; positive on land.
    double CoastKm(double lat, double lon) const {
        if (sdf.empty()) return 50.0;
        const double fx = (lon + 180.0) / 360.0 * w - 0.5, fy = (90.0 - lat) / 180.0 * h - 0.5;
        const int x0 = (int)std::floor(fx), y0 = (int)std::floor(fy);
        const double tx = fx - x0, ty = fy - y0;
        auto at = [&](int x, int y) { x = ((x % w) + w) % w; y = std::clamp(y, 0, h - 1); return ((double)sdf[(size_t)y * w + x] - 128.5) * 2.0; };
        return (at(x0, y0) * (1 - tx) + at(x0 + 1, y0) * tx) * (1 - ty) + (at(x0, y0 + 1) * (1 - tx) + at(x0 + 1, y0 + 1) * tx) * ty;
    }
    int w = 0, h = 0;
    std::vector<uint8_t> sdf;
};

// ---- coastline and lake shore rings, lon/lat pairs
struct Coast {
    std::vector<std::vector<float>> rings; // lon, lat, lon, lat, ...
    bool Load(const std::filesystem::path& file) {
        std::ifstream f(file, std::ios::binary);
        if (!f) return false;
        char magic[4]; uint32_t n = 0;
        f.read(magic, 4); f.read((char*)&n, 4);
        if (std::memcmp(magic, "FLYC", 4) != 0) return false;
        rings.resize(n);
        for (auto& r : rings) {
            uint32_t m = 0;
            f.read((char*)&m, 4);
            r.resize((size_t)m * 2);
            f.read((char*)r.data(), (std::streamsize)(r.size() * 4));
        }
        return (bool)f;
    }
};

// Everything loaded once at start.
struct Earth {
    Places places;
    LandMask land;
    Coast coast;
    bool Load(const std::filesystem::path& dir) {
        const bool a = places.Load(dir / "places.bin"), b = land.Load(dir / "land.bin"), c = coast.Load(dir / "coast.bin");
        return a && b && c;
    }
};

} // namespace fly::geo
