// Real places: map data that overrides the procedural guesswork where we have it. Packed by tools/pack_roads.py into data/world/:
//   roads_ne.bin   major roads of the whole Earth (Natural Earth, public domain): freeways like I-94 wherever you are
//   osm_*.bin      OpenStreetMap extracts (ODbL; credit "OpenStreetMap contributors"): every road, runway, parking area, land use
//                  patch and building footprint inside their boxes, for 1:1 places
//   tiles/         the same map data for the whole world as streamed 0.1 degree tiles (tools/pack_tiles.py, tools/pack_overture.py;
//                  layout in tools/tile_layout.py): cells.bin lists the 1-degree cells that have tiles, each cell's index.bin is read
//                  when the camera comes near; a tile's roads and land use load for the area round the anchor, its buildings only
//                  near where the camera looks (Tiles::Want), so memory stays bounded wherever you fly
// RealLocal is one anchor's view: features near it in local metres (x east, z south), a coarse grid index, and a land cover
// raster for the ground's colour. Immutable once made; worker threads read it freely. No OS headers.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geo.h"

namespace fly::real {

enum Kind : uint8_t { Road = 0, Area = 1, Building = 2, Aeroway = 3, NeRoad = 4 };
// Road classes (OSM): 0 motorway 1 trunk 2 primary 3 secondary 4 tertiary 5 unclassified 6 residential 7 service 8 motorway link
// 9 trunk/primary link 10 other link 11 track. Area classes: 0 parking 1 apron 2 runway/taxiway area 3 residential 4 commercial
// 5 industrial 6 farmland 7 grass 8 forest 9 water 10 wetland 11 park 12 golf 13 pitch 14 cemetery 15 railway 16 construction 17 other.
// Building classes: 0 house 1 apartments 2 commercial 3 industrial 4 public 5 garage/shed 6 unknown. Aeroway: 0 runway 1 taxiway.
// NE roads: cls 0 major highway (flags bit0: expressway) 1 secondary highway 2 road.
enum : uint8_t { kOneway = 1, kBridge = 2, kTunnel = 4, kParkingAisle = 8, kDriveway = 16 };

struct GeoFeature { uint8_t kind, cls, flags, lanes; float height; std::vector<float> latlon; uint32_t wall = 0, roof = 0; }; // colours 0xRRGGBB + 1, 0 = none

struct Source {
    double s = 0, w = 0, n = 0, e = 0; // the box it covers
    std::vector<GeoFeature> f;
};

// The world's tiles beyond the local pack: our CDN behind a disk cache (tile_stream.h), by path relative to the release root
// ("cells.bin", "43_-85/index.bin", "43_-85/436_-843.bin"). A file not on disk yet is Pending: asked for, nearest `priority`
// metres first; Tiles asks again on a later Want.
struct TileFiles {
    enum Result { Ready, Pending, Missing };
    virtual ~TileFiles() = default;
    virtual Result Get(const std::string& rel, double priority, std::vector<uint8_t>& out) = 0;
    virtual void BeginRound() {} // a new Want starts: requests it doesn't repeat may be dropped
};

// The streamed map tiles. Format "FLYT" v1 (tools/pack_tiles.py): header, then section A (roads, areas, aeroways) and section B
// (buildings), each a count, a byte size and varint-coded features with microdegree point deltas from the tile's south-west corner.
class Tiles {
public:
    static constexpr double kDeg = 0.1;
    struct Entry { int32_t iy, ix; uint32_t bytesA, bytesB; };
    struct Loaded { Source a, b; bool hasA = false, hasB = false; };
    // A 1-degree cell with tiles in the local pack, on the CDN, or both. Its two indexes merge (a local tile wins), so a region
    // packed by hand (a few tiles of a cell) never hides the rest of the cell from the world release.
    struct Cell { bool local = false, remote = false, localRead = false, remoteRead = false; };
    std::unordered_map<int64_t, Cell> cells;    // every cell with tiles
    std::unordered_map<int64_t, Entry> index;   // the tiles of the cells read so far
    std::unordered_map<int64_t, Loaded> loaded; // the tiles in memory now
    std::filesystem::path dir;                  // the local pack (data/world/tiles)
    TileFiles* files = nullptr;                 // the CDN and its cache (tile_stream.h); null = the local pack only
    bool remoteCells = false;                   // the CDN's cells.bin is merged in (asked for on every Want until it is)

    static int64_t Key(int32_t iy, int32_t ix) { return ((int64_t)iy << 32) ^ (int64_t)(uint32_t)ix; }
    static int64_t KeyAt(double lat, double lon) { return Key((int32_t)std::floor(lat / kDeg), (int32_t)std::floor(lon / kDeg)); }
    static int32_t CellOf(int32_t i) { return (int32_t)std::floor(i / 10.0); }
    static std::string Name(int32_t a, int32_t b) { return std::to_string(a) + "_" + std::to_string(b); }
    bool Any() const { return !cells.empty() || files; }

    bool LoadIndex(const std::filesystem::path& d) {
        dir = d;
        std::vector<uint8_t> buf;
        return LocalFile("cells.bin", buf) && AddCells(buf, false) && !cells.empty();
    }
    // Reads the indexes of every cell overlapping the box (degrees), once (an index still on its way from the CDN is asked again).
    void EnsureCells(double lat0, double lon0, double lat1, double lon1, double lat, double lon) {
        std::vector<uint8_t> buf;
        for (int32_t cy = (int32_t)std::floor(lat0); cy <= (int32_t)std::floor(lat1); ++cy)
            for (int32_t cx = (int32_t)std::floor(lon0); cx <= (int32_t)std::floor(lon1); ++cx) {
                auto c = cells.find(Key(cy, cx));
                if (c == cells.end()) continue;
                Cell& C = c->second;
                const std::string rel = Name(cy, cx) + "/index.bin";
                if (C.local && !C.localRead) { C.localRead = true; if (LocalFile(rel, buf)) AddIndex(buf, true); }
                if (C.remote && !C.remoteRead && files) {
                    const double dy = (std::clamp(lat, (double)cy, cy + 1.0) - lat) * 110574.0;
                    const double dx = (std::clamp(lon, (double)cx, cx + 1.0) - lon) * 111320.0 * std::cos(geo::Rad(lat));
                    const TileFiles::Result got = files->Get(rel, std::sqrt(dx * dx + dy * dy), buf);
                    if (got == TileFiles::Pending) continue;
                    C.remoteRead = true;
                    if (got == TileFiles::Ready) AddIndex(buf, false);
                }
            }
    }

    // Section A of every tile within rA metres of (latA, lonA) and section B within rB of (latB, lonB); the rest leave memory.
    // Returns the tiles whose buildings, roads or land use came, or whose buildings went (the city rebuilds what stands on them).
    // With a CDN, tiles not on disk yet are only asked for: they come in on a later Want.
    std::vector<int64_t> Want(double latA, double lonA, double rA, double latB, double lonB, double rB) {
        std::vector<int64_t> changed;
        if (files) files->BeginRound();
        if (files && !remoteCells) { // the world's cell list from the CDN, merged with the local one
            std::vector<uint8_t> buf;
            const TileFiles::Result got = files->Get("cells.bin", 0, buf);
            if (got != TileFiles::Pending) remoteCells = true;
            if (got == TileFiles::Ready) AddCells(buf, true);
        }
        if (cells.empty()) return changed;
        auto dist2 = [](const Entry& e, double lat, double lon) { // the tile's nearest point to (lat, lon), in metres squared
            const double s = e.iy * kDeg, w = e.ix * kDeg;
            const double dy = (std::clamp(lat, s, s + kDeg) - lat) * 110574.0;
            const double dx = (std::clamp(lon, w, w + kDeg) - lon) * 111320.0 * std::cos(geo::Rad(lat));
            return dx * dx + dy * dy;
        };
        auto within = [&](const Entry& e, double lat, double lon, double r) { return dist2(e, lat, lon) <= r * r; };
        for (auto it = loaded.begin(); it != loaded.end();) { // they leave a little further out than they come in (no flapping)
            const Entry& e = index[it->first];
            Loaded& L = it->second;
            if (L.hasB && !within(e, latB, lonB, rB * 1.25)) { L.b = {}; L.hasB = false; changed.push_back(it->first); }
            if (L.hasA && !within(e, latA, lonA, rA * 1.25)) { L.a = {}; L.hasA = false; }
            it = (!L.hasA && !L.hasB) ? loaded.erase(it) : std::next(it);
        }
        const double dLat = std::max(rA, rB) / 110574.0 + kDeg;
        const double lat0 = std::min(latA, latB), lat1 = std::max(latA, latB), lon0 = std::min(lonA, lonB), lon1 = std::max(lonA, lonB);
        const double dLon = dLat / std::max(0.1, std::cos(geo::Rad(std::clamp((lat0 + lat1) * 0.5, -85.0, 85.0))));
        EnsureCells(lat0 - dLat, lon0 - dLon, lat1 + dLat, lon1 + dLon, latB, lonB);
        for (int32_t iy = (int32_t)std::floor((lat0 - dLat) / kDeg); iy <= (int32_t)std::floor((lat1 + dLat) / kDeg); ++iy)
            for (int32_t ix = (int32_t)std::floor((lon0 - dLon) / kDeg); ix <= (int32_t)std::floor((lon1 + dLon) / kDeg); ++ix) {
                auto e = index.find(Key(iy, ix));
                if (e == index.end()) continue;
                const double dA = dist2(e->second, latA, lonA), dB = dist2(e->second, latB, lonB);
                const bool a = dA <= rA * rA, b = dB <= rB * rB;
                if (!a && !b) continue;
                auto have = loaded.find(e->first);
                if (have != loaded.end() && (!a || have->second.hasA) && (!b || have->second.hasB)) continue;
                Loaded& L = loaded[e->first];
                const bool needA = a && !L.hasA, needB = b && !L.hasB;
                // fetched nearest first: by distance to where the buildings are wanted, roads-only tiles after them
                const double prio = std::sqrt(b ? std::min(dA, dB) : dA + rB * rB);
                if (Read(e->second, needA, needB, L, prio)) changed.push_back(e->first);
                else if (!L.hasA && !L.hasB) loaded.erase(e->first);
            }
        return changed;
    }

private:
    bool LocalFile(const std::string& rel, std::vector<uint8_t>& out) const {
        std::ifstream f(dir / rel, std::ios::binary);
        if (!f) return false;
        out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
        return !out.empty();
    }
    // A tile: the local pack's copy, else the CDN's (Pending while it is fetched).
    TileFiles::Result TileFile(const std::string& rel, double prio, std::vector<uint8_t>& out) {
        if (LocalFile(rel, out)) return TileFiles::Ready;
        return files ? files->Get(rel, prio, out) : TileFiles::Missing;
    }
    bool AddCells(const std::vector<uint8_t>& buf, bool remote) {
        if (buf.size() < 12 || std::memcmp(buf.data(), "FLYC", 4) != 0) return false;
        uint32_t n = 0;
        std::memcpy(&n, buf.data() + 8, 4);
        n = (uint32_t)std::min<size_t>(n, (buf.size() - 12) / 8);
        for (uint32_t i = 0; i < n; ++i) {
            int32_t c[2];
            std::memcpy(c, buf.data() + 12 + i * 8, 8);
            Cell& C = cells[Key(c[0], c[1])];
            (remote ? C.remote : C.local) = true;
        }
        return true;
    }
    void AddIndex(const std::vector<uint8_t>& buf, bool local) {
        if (buf.size() < 12 || std::memcmp(buf.data(), "FLYI", 4) != 0) return;
        uint32_t n = 0;
        std::memcpy(&n, buf.data() + 8, 4);
        n = (uint32_t)std::min<size_t>(n, (buf.size() - 12) / sizeof(Entry));
        for (uint32_t i = 0; i < n; ++i) {
            Entry e{};
            std::memcpy(&e, buf.data() + 12 + i * sizeof(Entry), sizeof e);
            if (local) index[Key(e.iy, e.ix)] = e;
            else index.emplace(Key(e.iy, e.ix), e); // the local pack's entry wins
        }
    }
    static uint64_t Varint(const uint8_t*& p, const uint8_t* end) {
        uint64_t v = 0; int shift = 0;
        while (p < end) { const uint8_t b = *p++; v |= (uint64_t)(b & 0x7F) << shift; if (!(b & 0x80)) break; shift += 7; }
        return v;
    }
    static int64_t Unzig(uint64_t v) { return (int64_t)(v >> 1) ^ -(int64_t)(v & 1); }
    static void Decode(const std::vector<uint8_t>& buf, uint32_t count, std::vector<GeoFeature>& out) {
        const uint8_t* p = buf.data(); const uint8_t* end = p + buf.size();
        out.reserve(count);
        for (uint32_t k = 0; k < count && end - p >= 4; ++k) {
            GeoFeature g{};
            g.kind = p[0]; g.cls = p[1]; g.flags = p[2]; g.lanes = p[3]; p += 4;
            g.height = (float)(Varint(p, end) * 0.1);
            const uint64_t n = Varint(p, end);
            if (g.kind == Building) { g.wall = (uint32_t)Varint(p, end); g.roof = (uint32_t)Varint(p, end); }
            g.latlon.resize((size_t)n * 2);
            int64_t y = 0, x = 0;
            for (uint64_t i = 0; i < n; ++i) {
                y += Unzig(Varint(p, end)); x += Unzig(Varint(p, end));
                g.latlon[i * 2] = (float)(y * 1e-6); g.latlon[i * 2 + 1] = (float)(x * 1e-6); // offsets from the corner (as Source)
            }
            out.push_back(std::move(g));
        }
    }
    bool Read(const Entry& e, bool wantA, bool wantB, Loaded& L, double prio) {
        std::vector<uint8_t> file;
        if (TileFile(Name(CellOf(e.iy), CellOf(e.ix)) + "/" + Name(e.iy, e.ix) + ".bin", prio, file) != TileFiles::Ready) return false;
        if (file.size() < 16 || std::memcmp(file.data(), "FLYT", 4) != 0) return false;
        size_t at = 16; // magic, version, iy, ix
        uint32_t count = 0, bytes = 0;
        std::vector<uint8_t> buf;
        for (int sec = 0; sec < 2; ++sec) {
            if (file.size() - at < 8) return false;
            std::memcpy(&count, file.data() + at, 4); std::memcpy(&bytes, file.data() + at + 4, 4); at += 8;
            if (file.size() - at < bytes) return false;
            if (!(sec == 0 ? wantA : wantB)) { at += bytes; continue; }
            buf.assign(file.begin() + (std::ptrdiff_t)at, file.begin() + (std::ptrdiff_t)(at + bytes));
            at += bytes;
            Source& s = sec == 0 ? L.a : L.b;
            s.s = e.iy * kDeg; s.w = e.ix * kDeg; s.n = s.s + kDeg; s.e = s.w + kDeg;
            s.f.clear();
            Decode(buf, count, s.f);
            (sec == 0 ? L.hasA : L.hasB) = true;
        }
        return true;
    }
};

class Data {
public:
    std::vector<GeoFeature> ne;   // world major roads
    std::vector<Source> osm;      // detailed extracts
    Tiles tiles;                  // the streamed world (changed by Want under `m`; Local::Make reads it under `m`)
    mutable std::mutex m;
    // How far the streamed layers reach: roads and land use round the anchor (the land cover raster, and the city's reach as the
    // camera drifts before the world re-anchors), buildings round where the camera looks (the city draws them out to 13 km).
    static constexpr double kRoadsRadius = 34000.0, kBuildingsRadius = 18000.0, kCoverHalf = 32000.0;
    void Want(double anchorLat, double anchorLon, double lookLat, double lookLon, std::vector<int64_t>* changed = nullptr) {
        std::lock_guard<std::mutex> l(m);
        auto c = tiles.Want(anchorLat, anchorLon, kRoadsRadius, lookLat, lookLon, kBuildingsRadius);
        if (changed) *changed = std::move(c);
    }
    bool Load(const std::filesystem::path& dir) {
        bool any = tiles.LoadIndex(dir / "tiles");
        { // Natural Earth
            std::ifstream f(dir / "roads_ne.bin", std::ios::binary);
            char m[4]; uint32_t n = 0;
            if (f && f.read(m, 4) && std::memcmp(m, "FLYR", 4) == 0 && f.read((char*)&n, 4)) {
                ne.resize(n);
                for (auto& g : ne) {
                    uint8_t t, x; uint32_t c;
                    f.read((char*)&t, 1); f.read((char*)&x, 1); f.read((char*)&c, 4);
                    std::vector<float> lonlat(c * 2);
                    f.read((char*)lonlat.data(), (std::streamsize)lonlat.size() * 4);
                    g.kind = NeRoad; g.cls = t; g.flags = x; g.lanes = 0; g.height = 0;
                    g.latlon.resize(c * 2);
                    for (uint32_t i = 0; i < c; ++i) { g.latlon[i * 2] = lonlat[i * 2 + 1]; g.latlon[i * 2 + 1] = lonlat[i * 2]; }
                }
                any = true;
            }
        }
        std::error_code ec;
        for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
            const std::string name = e.path().filename().string();
            if (name.rfind("osm_", 0) != 0 || e.path().extension() != ".bin") continue;
            std::ifstream f(e.path(), std::ios::binary);
            char m[4]; uint32_t ver = 0, n = 0;
            if (!f.read(m, 4) || std::memcmp(m, "FLYO", 4) != 0) continue;
            Source s;
            f.read((char*)&ver, 4); f.read((char*)&s.s, 8); f.read((char*)&s.w, 8); f.read((char*)&s.n, 8); f.read((char*)&s.e, 8); f.read((char*)&n, 4);
            s.f.resize(n);
            std::vector<int32_t> buf;
            for (auto& g : s.f) {
                uint32_t c = 0;
                f.read((char*)&g.kind, 1); f.read((char*)&g.cls, 1); f.read((char*)&g.flags, 1); f.read((char*)&g.lanes, 1);
                f.read((char*)&g.height, 4); f.read((char*)&c, 4);
                if (ver >= 2) { f.read((char*)&g.wall, 4); f.read((char*)&g.roof, 4); }
                buf.resize(c * 2);
                f.read((char*)buf.data(), (std::streamsize)buf.size() * 4);
                g.latlon.resize(c * 2);
                for (size_t i = 0; i < buf.size(); ++i) g.latlon[i] = (float)(buf[i] * 1e-7); // float: ~1 m; converted to local in double below
                // keep full precision for the local conversion: store the offset from the box corner instead
                for (size_t i = 0; i < c; ++i) { g.latlon[i * 2] = (float)(buf[i * 2] * 1e-7 - s.s); g.latlon[i * 2 + 1] = (float)(buf[i * 2 + 1] * 1e-7 - s.w); }
            }
            osm.push_back(std::move(s));
            any = true;
        }
        return any;
    }
};

// A feature in local metres.
struct Feature {
    uint8_t kind, cls, flags, lanes;
    float height;
    uint32_t wall, roof; // 0xRRGGBB + 1, 0 = not known
    std::vector<std::array<double, 2>> p; // x, z
    double x0, z0, x1, z1;                // bounds
};

// Land cover classes for the ground's colour (an Area class + 1; 0 = no data here).
class Local {
public:
    std::vector<Feature> f;
    // grid index: features whose bounds touch each cell
    static constexpr double kCell = 360.0;
    int gx0 = 0, gz0 = 0, gn = 0;
    std::vector<std::vector<uint32_t>> cells;
    // land cover raster around the anchor
    static constexpr double kCover = 20.0;
    int cn = 0;
    double coverHalf = 0;
    std::vector<uint8_t> cover;
    // where the detailed (OSM) data is, in local metres: inside it, towns are built from real footprints
    struct Box { double x0, z0, x1, z1; };
    std::vector<Box> detailed;

    // and where the streamed tiles have map data (from the tile index, so it stays the same as tiles load and leave)
    static constexpr double kMask = 250.0;
    int mn = 0;
    double maskHalf = 0;
    std::vector<uint8_t> mask;

    bool Detailed(double x, double z) const {
        if (!mask.empty()) {
            const int i = (int)std::floor((x + maskHalf) / kMask), j = (int)std::floor((z + maskHalf) / kMask);
            if (i >= 0 && j >= 0 && i < mn && j < mn && mask[(size_t)j * mn + i]) return true;
        }
        for (const Box& b : detailed) if (x > b.x0 && x < b.x1 && z > b.z0 && z < b.z1) return true;
        return false;
    }
    uint8_t CoverAt(double x, double z) const {
        if (cover.empty()) return 0;
        const int i = (int)std::floor((x + coverHalf) / kCover), j = (int)std::floor((z + coverHalf) / kCover);
        if (i < 0 || j < 0 || i >= cn || j >= cn) return 0;
        return cover[(size_t)j * cn + i];
    }
    // Calls fn(feature) once for every feature whose bounds touch the box.
    template <class F> void Query(double x0, double z0, double x1, double z1, F&& fn) const {
        if (cells.empty()) return;
        const int i0 = std::max(0, (int)std::floor(x0 / kCell) - gx0), i1 = std::min(gn - 1, (int)std::floor(x1 / kCell) - gx0);
        const int j0 = std::max(0, (int)std::floor(z0 / kCell) - gz0), j1 = std::min(gn - 1, (int)std::floor(z1 / kCell) - gz0);
        std::vector<uint32_t> seen;
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i)
                for (uint32_t k : cells[(size_t)j * gn + i]) {
                    const Feature& q = f[k];
                    if (q.x1 < x0 || q.x0 > x1 || q.z1 < z0 || q.z0 > z1) continue;
                    if (std::find(seen.begin(), seen.end(), k) != seen.end()) continue;
                    seen.push_back(k);
                    fn(q);
                }
    }

    static std::shared_ptr<Local> Make(const Data& d, const geo::Anchor& a, double radius = 45000.0) {
        auto L = std::make_shared<Local>();
        auto add = [&](const GeoFeature& g, double baseLat, double baseLon) {
            Feature q{g.kind, g.cls, g.flags, g.lanes, g.height, g.wall, g.roof, {}, 1e18, 1e18, -1e18, -1e18};
            const size_t n = g.latlon.size() / 2;
            bool near = false;
            q.p.reserve(n);
            for (size_t i = 0; i < n; ++i) {
                double x, z;
                if (!a.ToLocal(baseLat + g.latlon[i * 2], baseLon + g.latlon[i * 2 + 1], x, z)) return;
                q.p.push_back({x, z});
                q.x0 = std::min(q.x0, x); q.z0 = std::min(q.z0, z); q.x1 = std::max(q.x1, x); q.z1 = std::max(q.z1, z);
                if (std::abs(x) < radius && std::abs(z) < radius) near = true;
            }
            if (near && q.p.size() >= 2) L->f.push_back(std::move(q));
        };
        const double dLat = geo::Deg(radius / geo::kEarthR) * 1.2, dLon = dLat / std::max(0.1, std::cos(geo::Rad(a.lat)));
        bool hasOsm = false;
        std::lock_guard<std::mutex> lock(d.m);
        if (!d.tiles.index.empty()) { // where the streamed world has data (the cells round the anchor, read by Want): local metres
            L->maskHalf = Data::kCoverHalf; L->mn = (int)std::ceil(2 * L->maskHalf / kMask);
            L->mask.assign((size_t)L->mn * L->mn, 0);
            bool anyMask = false;
            for (int j = 0; j < L->mn; ++j)
                for (int i = 0; i < L->mn; ++i) {
                    double la, lo;
                    a.ToLatLon((i + 0.5) * kMask - L->maskHalf, (j + 0.5) * kMask - L->maskHalf, la, lo);
                    if (d.tiles.index.count(Tiles::KeyAt(la, lo))) { L->mask[(size_t)j * L->mn + i] = 1; anyMask = true; }
                }
            if (!anyMask) { L->mask.clear(); L->mn = 0; }
        }
        // a hand-made extract (the photo-matched airports) wins over the streamed tiles inside its box; the tiles fill round it
        auto inExtract = [&](const GeoFeature& g, double baseLat, double baseLon) {
            if (g.latlon.size() < 2) return false;
            const double la = baseLat + g.latlon[0], lo = baseLon + g.latlon[1];
            for (const Source& s : d.osm) if (la > s.s && la < s.n && lo > s.w && lo < s.e) return true;
            return false;
        };
        for (const auto& kv : d.tiles.loaded) {
            const Tiles::Loaded& t = kv.second;
            if (t.hasA) for (const GeoFeature& g : t.a.f) if (!inExtract(g, t.a.s, t.a.w)) add(g, t.a.s, t.a.w);
            if (t.hasB) for (const GeoFeature& g : t.b.f) if (!inExtract(g, t.b.s, t.b.w)) add(g, t.b.s, t.b.w);
            hasOsm = true;
        }
        for (const Source& s : d.osm) {
            if (s.n < a.lat - dLat || s.s > a.lat + dLat || s.e < a.lon - dLon || s.w > a.lon + dLon) continue;
            for (const GeoFeature& g : s.f) add(g, s.s, s.w);
            double bx0, bz0, bx1, bz1, t;
            a.ToLocal(s.n, s.w, bx0, bz0); a.ToLocal(s.s, s.e, bx1, bz1);
            if (bx0 > bx1) { t = bx0; bx0 = bx1; bx1 = t; }
            if (bz0 > bz1) { t = bz0; bz0 = bz1; bz1 = t; }
            L->detailed.push_back({bx0 + 300, bz0 + 300, bx1 - 300, bz1 - 300});
            hasOsm = true;
        }
        for (const GeoFeature& g : d.ne) { // world freeways, where no extract has them already
            bool inBox = false;
            for (size_t i = 0; i < g.latlon.size(); i += 2) {
                if (std::abs(g.latlon[i] - a.lat) < dLat && std::abs(g.latlon[i + 1] - a.lon) < dLon) { inBox = true; break; }
            }
            if (!inBox) continue;
            const size_t before = L->f.size();
            add(g, 0.0, 0.0);
            if (hasOsm && L->f.size() > before) { // drop the parts the detailed data covers
                Feature& q = L->f.back();
                const double cx = (q.x0 + q.x1) * 0.5, cz = (q.z0 + q.z1) * 0.5;
                if (L->Detailed(cx, cz)) L->f.pop_back();
            }
        }
        // grid index
        const double R = radius + 5000.0;
        L->gx0 = (int)std::floor(-R / kCell); L->gz0 = L->gx0; L->gn = (int)std::ceil(2 * R / kCell) + 1;
        L->cells.assign((size_t)L->gn * L->gn, {});
        for (uint32_t k = 0; k < L->f.size(); ++k) {
            const Feature& q = L->f[k];
            const int i0 = std::max(0, (int)std::floor(q.x0 / kCell) - L->gx0), i1 = std::min(L->gn - 1, (int)std::floor(q.x1 / kCell) - L->gx0);
            const int j0 = std::max(0, (int)std::floor(q.z0 / kCell) - L->gz0), j1 = std::min(L->gn - 1, (int)std::floor(q.z1 / kCell) - L->gz0);
            for (int j = j0; j <= j1; ++j)
                for (int i = i0; i <= i1; ++i) L->cells[(size_t)j * L->gn + i].push_back(k);
        }
        if (hasOsm) L->RasterCover(std::min(radius, L->mask.empty() ? 26000.0 : Data::kCoverHalf));
        return L;
    }

private:
    // Fills the land cover raster from the area polygons (scanline, even-odd), broad classes first so the specific ones win.
    void RasterCover(double half) {
        coverHalf = half;
        cn = (int)std::ceil(2 * half / kCover);
        cover.assign((size_t)cn * cn, 0);
        static const int order[] = {3, 4, 5, 15, 16, 6, 7, 14, 11, 12, 13, 8, 10, 9, 1, 2, 0};
        std::vector<double> xs;
        for (int cls : order)
            for (const Feature& q : f) {
                if (q.kind != Area || q.cls != cls) continue;
                const int j0 = std::max(0, (int)std::floor((q.z0 + half) / kCover)), j1 = std::min(cn - 1, (int)std::floor((q.z1 + half) / kCover));
                for (int j = j0; j <= j1; ++j) {
                    const double z = (j + 0.5) * kCover - half;
                    xs.clear();
                    for (size_t i = 0, n = q.p.size(); i < n; ++i) {
                        const auto& A = q.p[i];
                        const auto& B = q.p[(i + 1) % n];
                        if ((A[1] <= z) != (B[1] <= z)) xs.push_back(A[0] + (z - A[1]) / (B[1] - A[1]) * (B[0] - A[0]));
                    }
                    std::sort(xs.begin(), xs.end());
                    for (size_t k = 0; k + 1 < xs.size(); k += 2) {
                        const int i0 = std::max(0, (int)std::ceil((xs[k] + half) / kCover - 0.5)), i1 = std::min(cn - 1, (int)std::floor((xs[k + 1] + half) / kCover - 0.5));
                        for (int i = i0; i <= i1; ++i) cover[(size_t)j * cn + i] = (uint8_t)(cls + 1);
                    }
                }
            }
    }
};

} // namespace fly::real
