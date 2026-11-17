// Satellite imagery draped on the ground where we have it: USDA NAIP aerial photographs (public domain), fetched and baked offline
// (tools/fetch_naip.py, tools/bake_imagery.py) into data/world/imagery_<name>.jpg with a sidecar giving its Web Mercator box. The
// texture is bound as the surface map of the terrain and town draws; each frame the mapping from scene x, z (floating-origin local)
// to its uv goes to the shader (Renderer::SetGroundImagery, mesh.hlsl GroundImagery), which takes the ground's colour from it: the
// real fields, lots, tree lines and soil patches under the vector roads and the 3D buildings and trees. Portable: std I/O and
// platform::DecodeImage only.
#pragma once
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "../floating_origin.h"
#include "../platform/platform.h"
#include "../render/renderer.h"
#include "geo.h"

namespace fly {

struct GroundImagery {
    tbd::TextureId tex = tbd::kNoTexture;
    std::string name;              // which set is loaded (imagery_<name>)
    double merc[4] = {0, 0, 1, 1}; // Web Mercator box: left, top, right, bottom (metres)
    int width = 0, height = 0;
    float strength = 1.0f, gain = 1.0f;

    bool Ok() const { return tex != tbd::kNoTexture; }

    static bool ReadBox(const std::filesystem::path& json, double box[4]) {
        std::ifstream js(json);
        if (!js) return false;
        const std::string text((std::istreambuf_iterator<char>(js)), std::istreambuf_iterator<char>());
        const size_t at = text.find("\"merc\"");
        return at != std::string::npos && std::sscanf(text.c_str() + text.find('[', at) + 1, "%lf , %lf , %lf , %lf", &box[0], &box[1], &box[2], &box[3]) == 4;
    }
    // The set (data/world/imagery_<name>.json + .jpg) covering a place, loaded in place of the current one; false when none covers it
    // (the ground keeps its procedural colours). Outside a frame.
    bool LoadFor(tbd::Renderer& r, const std::filesystem::path& dir, double lat, double lon) {
        std::error_code ec;
        for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
            const std::string fn = e.path().filename().string();
            if (fn.rfind("imagery_", 0) != 0 || e.path().extension() != ".json") continue;
            double box[4];
            if (!ReadBox(e.path(), box)) continue;
            double u, v;
            const double keep[4] = {merc[0], merc[1], merc[2], merc[3]};
            for (int c = 0; c < 4; ++c) merc[c] = box[c];
            UV(lat, lon, u, v);
            for (int c = 0; c < 4; ++c) merc[c] = keep[c];
            if (u < 0.02 || u > 0.98 || v < 0.02 || v > 0.98) continue;
            const std::string n = fn.substr(8, fn.size() - 8 - 5);
            if (n == name && Ok()) return true;
            if (Ok()) r.RetireTexture(tex);
            tex = tbd::kNoTexture;
            name.clear();
            if (Load(r, dir, n)) return true;
        }
        if (Ok()) { r.RetireTexture(tex); tex = tbd::kNoTexture; name.clear(); }
        return false;
    }

    bool Load(tbd::Renderer& r, const std::filesystem::path& dir, const std::string& setName) {
        if (!ReadBox(dir / ("imagery_" + setName + ".json"), merc)) return false;
        name = setName;
        std::ifstream f(dir / ("imagery_" + setName + ".jpg"), std::ios::binary);
        if (!f) return false;
        const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        std::vector<uint8_t> rgba;
        if (!tbd::platform::DecodeImage(bytes.data(), bytes.size(), rgba, width, height)) return false;
        tex = r.CreateTexture(rgba.data(), width, height);
        return tex != tbd::kNoTexture;
    }

    // The uv of a point on the globe.
    void UV(double lat, double lon, double& u, double& v) const {
        constexpr double kR = 6378137.0, kDeg = 3.14159265358979323846 / 180.0;
        const double x = kR * lon * kDeg, y = kR * std::log(std::tan(3.14159265358979323846 / 4 + lat * kDeg / 2));
        u = (x - merc[0]) / (merc[2] - merc[0]);
        v = (merc[1] - y) / (merc[1] - merc[3]);
    }
    // The frame's mapping, scene x, z -> uv, as SetGroundImagery takes it: an affine fit at the floating origin (the camera is within
    // a kilometre or two of it; the fit's error 5 km out is a few metres). Zero strength when there is no imagery.
    void Mapping(const geo::Anchor& anchor, const tbd::WorldPosition& origin, float a[4], float b[4]) const {
        for (int c = 0; c < 4; ++c) a[c] = b[c] = 0.0f;
        if (!Ok() || strength <= 0.0f) return;
        double u[3], v[3];
        const double off[3][2] = {{0, 0}, {1000, 0}, {0, 1000}};
        for (int k = 0; k < 3; ++k) {
            double lat, lon;
            anchor.ToLatLon(origin.x + off[k][0], origin.z + off[k][1], lat, lon);
            UV(lat, lon, u[k], v[k]);
        }
        a[0] = (float)u[0]; a[1] = (float)v[0]; a[2] = (float)((u[1] - u[0]) / 1000.0); a[3] = (float)((u[2] - u[0]) / 1000.0);
        b[0] = (float)((v[1] - v[0]) / 1000.0); b[1] = (float)((v[2] - v[0]) / 1000.0); b[2] = strength; b[3] = gain;
    }
};

} // namespace fly
