// The basic game (src_basic, fly_basic.exe): the lean fork for phones and standalone headsets. Generated terrain with satellite
// imagery draped on it all the way to the eye, a baked airport (fly/airport.h: flattened ground, runway strips and markings), the
// trainer aircraft, chase / cockpit / orbit cameras and a small HUD. No towns, buildings, trees, ground vehicles, cockpit model,
// water system or baked shadows: everything here is meant to hold a phone / Quest frame budget, with features added back one at a
// time only while it does. Same Game interface as the full build (game.h), so every platform shell and the OpenXR layer work as is.
// No OS headers.
//
// Env: FLY_AIRPORT=KDTW (an ICAO code from data/airports; default the first), FLY_RUNWAY=22R (the end to use; default the first),
// FLY_SPAWN=approach|runway (default approach: 6 km out on a 3 degree glide), FLY_SAT=0 (no imagery), FLY_SAT_NEAR_ZOOM (14),
// FLY_HOUR (15.6), FLY_FOG (0 clear, 1 haze, 2 fog), FLY_HUD=0, FLY_CLOUDS (0..1), FLY_POST_TIER (0..4, default 1).
#include "game.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../platform/platform.h"
#include "../sun.h"
#include "airport.h"
#include "earth_gen.h"
#include "flight_model.h"
#include "frustum.h"
#include "geo.h"
#include "imagery.h"
#include "sat_stream.h"
#include "town.h"
#include "ui_kit.h"
#include "workers.h"
#include "world.h"

// Globals the renderers read (every backend: D3D12, Vulkan, Metal).
float g_brightness = 1.15f;   // tone-map brightness
float g_sceneExposure = 0.0f; // 0 = auto exposure

namespace fly {

static int EnvInt(const char* name, int fallback) {
    char b[32] = {};
    return tbd::platform::GetEnvA(name, b, sizeof b) > 0 ? std::atoi(b) : fallback;
}
static float EnvFloat(const char* name, float fallback) {
    char b[32] = {};
    return tbd::platform::GetEnvA(name, b, sizeof b) > 0 ? (float)std::atof(b) : fallback;
}
static std::string EnvStr(const char* name) {
    char b[64] = {};
    return tbd::platform::GetEnvA(name, b, sizeof b) > 0 ? std::string(b) : std::string();
}
static double Seconds(int64_t ticks) { return (double)ticks / (double)tbd::platform::TickFrequency(); }

struct Game::Impl {
    Settings settings;
    Terrain terrain;
    tbd::Sun sun;
    Aircraft plane, start;
    Controls ctl;
    WorldPosition origin{0, 0, 0};
    tbd::MeshId planeMesh = tbd::kInvalidMesh, runwayMesh = tbd::kInvalidMesh;
    int camMode = 0, w = 1, h = 1; // 0 chase, 1 cockpit, 2 orbit
    float orbitYaw = 0, lookYaw = 0, lookPitch = -0.08f;
    double simTime = 0;
    float lastDt = 0;
    uint64_t frame = 0;
    int fog = 0;
    geo::Earth earth;
    bool earthOk = false;
    std::shared_ptr<WorldGen> gen;
    std::vector<Airport> airports;
    int airport = -1;                 // index into airports, -1 none (FLY_GEO somewhere else)
    double fieldHeight = 0;           // the airport plateau's height
    SatStream sat;                    // satellite tiles streamed round the camera: the whole ground's colour (sat_stream.h)
    UiKit ui;
    bool loading = true;
    int64_t loadStart = 0;
    struct Perf { int64_t last = 0; double cpuMs = 0, gpuMs = 0; } perf;
    // last: its threads are joined first when the game goes, before the terrain they write into
    std::unique_ptr<Workers> workers;
};

Game::Game() : m_(std::make_unique<Impl>()) {}
Game::~Game() = default;
float Game::Throttle() const { return m_->ctl.throttle; }
bool Game::HandlesEscape() const { return false; }
bool Game::WantsFullscreen() const { return false; }
bool Game::WantsMouseLook() const { return false; }
void Game::Resize(int width, int height) { m_->w = std::max(1, width); m_->h = std::max(1, height); }

// The aircraft on the chosen runway end: on final 6 km out on a 3 degree glide (FLY_SPAWN=approach), or on the threshold, stopped.
static void PlaceAircraft(Game::Impl& g) {
    g.plane = Aircraft{};
    g.ctl = {};
    if (g.airport < 0) { // no airport here: cruising north at 450 m
        g.plane.pos = {0, std::max(0.0, g.terrain.field.Height(0, 0)) + 450.0, 0};
        g.ctl.throttle = 0.72f;
    } else {
        const Airport& ap = g.airports[(size_t)g.airport];
        double x = 0, z = 0, dx = 0, dz = -1, len = 0;
        ap.FindEnd(g.gen->anchor, EnvStr("FLY_RUNWAY"), x, z, dx, dz, len) || ap.FindEnd(g.gen->anchor, "", x, z, dx, dz, len);
        const float yaw = std::atan2(-(float)dx, -(float)dz); // body -z forward
        g.plane.rot = tbd::QuatAxisAngle({0, 1, 0}, yaw);
        if (EnvStr("FLY_SPAWN") == "runway") {
            g.plane.pos = {x + dx * 60.0, g.fieldHeight + 1.2, z + dz * 60.0};
            g.plane.vel = {0, 0, 0};
            g.ctl.throttle = 0.0f;
        } else {
            const double back = 6000.0;
            g.plane.pos = {x - dx * back, g.fieldHeight + back * std::tan(3.0 * 3.14159265 / 180.0), z - dz * back};
            g.plane.vel = tbd::Rotate(g.plane.rot, tbd::Vec3{0, 0, -52.0f});
            g.ctl.throttle = 0.45f;
        }
        tbd::Log("spawn: %s %s, %s", ap.icao.c_str(), EnvStr("FLY_RUNWAY").c_str(), EnvStr("FLY_SPAWN") == "runway" ? "on the runway" : "on final");
    }
    g.start = g.plane;
    g.origin = {std::round(g.plane.pos.x / 1000) * 1000, std::round(g.plane.pos.y / 1000) * 1000, std::round(g.plane.pos.z / 1000) * 1000};
}

bool Game::Start(tbd::Renderer& r, const Settings& settings) {
    Impl& g = *m_;
    g.settings = settings;
    g.camMode = settings.vr ? 1 : std::clamp(settings.startCamera, 0, 2);
    g.sun.hour = EnvFloat("FLY_HOUR", 15.6f);
    g.fog = EnvInt("FLY_FOG", 0) % 3;
    g.workers = std::make_unique<Workers>();
    g.earthOk = g.earth.Load(settings.dataDir / "world");
    tbd::Log("earth: %s, %zu places", g.earthOk ? "loaded" : "MISSING data/world (run tools/pack_world.py)", g.earth.places.list.size());
    g.airports = Airport::LoadAll(settings.dataDir / "airports");
    tbd::Log("airports: %zu baked (data/airports)", g.airports.size());
    double lat = 42.2042, lon = -83.3529; // DTW
    if (settings.hasStartGeo) { lat = settings.startLat; lon = settings.startLon; }
    else if (!g.airports.empty()) {
        g.airport = 0;
        const std::string want = EnvStr("FLY_AIRPORT");
        for (size_t k = 0; k < g.airports.size(); ++k)
            if (!want.empty() && std::equal(want.begin(), want.end(), g.airports[k].icao.begin(), g.airports[k].icao.end(),
                                            [](char x, char y) { return std::toupper((unsigned char)x) == std::toupper((unsigned char)y); }))
                g.airport = (int)k;
        lat = g.airports[(size_t)g.airport].lat; lon = g.airports[(size_t)g.airport].lon;
    }
    // the world round the airport: generated ground only (no map data: no roads, land cover or buildings)
    g.gen = WorldGen::Make(g.earth, lat, lon, false, nullptr);
    if (g.airport >= 0) { // the plateau at the ground's own height there, kept above the sea
        const Airport& ap = g.airports[(size_t)g.airport];
        g.fieldHeight = std::max(3.0, g.gen->Height(0, 0));
        g.gen->airfield[0] = 0; g.gen->airfield[1] = 0; g.gen->airfield[2] = ap.flatRadius; g.gen->airfield[3] = g.fieldHeight;
        const std::vector<tbd::Vertex> rv = ap.BuildMesh(g.gen->anchor, g.fieldHeight);
        if (!rv.empty()) g.runwayMesh = r.CreateMesh(rv.data(), (uint32_t)rv.size(), tbd::Topology::Triangles);
        tbd::Log("airport: %s %s, %zu runways, %zu vertices, field %.0f m, flat %.0f m round", ap.icao.c_str(), ap.name.c_str(), ap.runways.size(),
                 rv.size(), g.fieldHeight, ap.flatRadius);
    }
    g.terrain.Reset(r, g.gen);
    const std::vector<tbd::Vertex> planeVerts = town::AircraftMesh();
    g.planeMesh = r.CreateMesh(planeVerts.data(), (uint32_t)planeVerts.size(), tbd::Topology::Triangles);
    if (!g.ui.Init(r, settings.dataDir)) tbd::Log("ui: smooth path unavailable, classic pixel font");
    g.sat.enabled = EnvInt("FLY_SAT", 1) != 0;
    g.sat.nearZoom = std::clamp(EnvInt("FLY_SAT_NEAR_ZOOM", 14), 10, 17);
    g.sat.img.strength = 1.0f;
    g.sat.img.gain = EnvFloat("FLY_SAT_GAIN", 0.3f);
    if (g.sat.enabled) g.sat.Start(tbd::platform::CacheRoot() / "satcache");
    PlaceAircraft(g);
    g.loadStart = tbd::platform::Ticks();
    return g.planeMesh != tbd::kInvalidMesh;
}

bool Game::Frame(tbd::Renderer& r, float dt, const Input& in) {
    Simulate(dt, in);
    Render(r, nullptr);
    return true;
}

void Game::Simulate(float dt, const Input& in) {
    Impl& g = *m_;
    g.lastDt = dt;
    g.simTime += dt;
    ++g.frame;
    if (g.loading) return;
    auto ease = [&](float& v, float target) { v += (target - v) * std::min(1.0f, dt * 6.0f); };
    ease(g.ctl.pitch, std::clamp(in.pitch, -1.0f, 1.0f));
    ease(g.ctl.roll, std::clamp(in.roll, -1.0f, 1.0f));
    ease(g.ctl.yaw, std::clamp(in.yaw, -1.0f, 1.0f));
    if (in.throttleAbs >= 0) g.ctl.throttle = std::clamp(in.throttleAbs, 0.0f, 1.0f);
    g.ctl.throttle = std::clamp(g.ctl.throttle + in.throttleRate * dt * 0.5f, 0.0f, 1.0f);
    if (in.cycleFog) g.fog = (g.fog + 1) % 3;
    if (in.reset) { g.plane = g.start; g.ctl = {}; g.ctl.throttle = g.start.vel.z == 0 && g.start.vel.x == 0 ? 0.0f : 0.45f; }
    if (in.nextCamera) { g.camMode = (g.camMode + 1) % 3; g.lookYaw = 0; g.lookPitch = -0.08f; }
    if (g.camMode == 1) {
        g.lookYaw = std::clamp(g.lookYaw - in.lookX, -2.6f, 2.6f);
        g.lookPitch = std::clamp(g.lookPitch - in.lookY, -1.3f, 1.3f);
    }
    for (float t = 0; t < dt; t += 1.0f / 240.0f) g.plane.Step(g.ctl, std::min(1.0f / 240.0f, dt - t), g.terrain.field);
    if (g.camMode == 2) g.orbitYaw += dt * 0.3f;
    // floating origin: rebase in 1 km steps so local floats stay small wherever the aircraft goes
    const Aircraft& p = g.plane;
    if (std::abs(p.pos.x - g.origin.x) > 1000 || std::abs(p.pos.z - g.origin.z) > 1000 || std::abs(p.pos.y - g.origin.y) > 1000)
        g.origin = {std::round(p.pos.x / 1000) * 1000, std::round(p.pos.y / 1000) * 1000, std::round(p.pos.z / 1000) * 1000};
}

// Reversed-Z projection from the four half-angle tangents (OpenXR's asymmetric field of view), matching PerspectiveRH.
static tbd::Mat4 ProjectionFov(float tl, float tr, float tu, float td, float nearZ, float farZ) {
    tbd::Mat4 m;
    m.At(0, 0) = 2.0f / (tr - tl);
    m.At(0, 2) = (tr + tl) / (tr - tl);
    m.At(1, 1) = 2.0f / (tu - td);
    m.At(1, 2) = (tu + td) / (tu - td);
    m.At(2, 2) = nearZ / (farZ - nearZ);
    m.At(2, 3) = farZ * nearZ / (farZ - nearZ);
    m.At(3, 2) = -1.0f;
    m.At(3, 3) = 0.0f;
    return m;
}

static void RenderLoading(tbd::Renderer& r, Game::Impl& g) {
    tbd::FrameView fv;
    fv.viewProj = tbd::Mat4::Identity(); fv.invJittered = tbd::Mat4::Identity(); fv.sky = false; fv.postTier = 0; fv.motionVectors = false;
    r.SetFrameView(fv);
    const float clear[4] = {0.01f, 0.012f, 0.02f, 1};
    r.BeginFrame(clear);
    r.ResolveScene();
    if (g.settings.vr) return;
    UiKit& u = g.ui;
    u.Begin();
    const float w = (float)g.w, h = (float)g.h, S = std::max(0.75f, h / 900.0f), cx = w * 0.5f;
    u.Rect(0, 0, w, h, Rgba(14, 22, 40), Rgba(46, 38, 52), 0);
    const std::string name = g.airport >= 0 ? g.airports[(size_t)g.airport].name : g.gen->placeName.empty() ? std::string("Open country") : g.gen->placeName;
    u.TextCentered(cx, h * 0.44f - 64 * S, 15 * S, "F L Y I N G   T O", Rgba(255, 210, 120), FontFace::Bold);
    u.TextCentered(cx, h * 0.44f, 46 * S, name, Rgba(255, 255, 255), FontFace::Bold, 0.15f, Rgba(0, 0, 0, 120));
    char status[96];
    std::snprintf(status, sizeof status, "Terrain %zu    %s", g.terrain.Missing(), g.sat.Status().c_str());
    u.TextCentered(cx, h * 0.44f + 48 * S, 13 * S, status, Rgba(200, 205, 225, 170));
    u.End(r, g.w, g.h, false);
}

void Game::Render(tbd::Renderer& r, const EyeView* eyeView) {
    Impl& g = *m_;
    const HeightField& field = g.terrain.field;
    const Aircraft& plane = g.plane;
    const WorldPosition origin = g.origin;
    const double agl = std::max(0.0, plane.pos.y - std::max(0.0, field.Height(plane.pos.x, plane.pos.z)));
    { double la, lo; g.gen->anchor.ToLatLon(plane.pos.x, plane.pos.z, la, lo); if (g.sat.enabled) g.sat.Update(r, la, lo, agl); }
    if (g.loading) {
        g.terrain.Update(r, plane.pos, *g.workers, 64);
        const double since = Seconds(tbd::platform::Ticks() - g.loadStart);
        // the near terrain, and the first satellite mosaic (or 20 s: offline, the generated colours show)
        if ((g.terrain.Missing() == 0 && (g.sat.img.Ok() || !g.sat.enabled) && since > 0.5) || since > 20.0) {
            tbd::Log("loading: done in %.1f s, %zu terrain chunks, imagery %s", since, g.terrain.Count(), g.sat.img.Ok() ? g.sat.img.name.c_str() : "none");
            g.loading = false;
        } else {
            RenderLoading(r, g);
            return;
        }
    }
    g.terrain.Update(r, plane.pos, *g.workers, g.frame < 5 ? 64 : 6);

    // camera, in local (origin-relative) coordinates
    const tbd::Vec3 p{(float)(plane.pos.x - origin.x), (float)(plane.pos.y - origin.y), (float)(plane.pos.z - origin.z)};
    const tbd::Vec3 f = plane.Forward(), u = plane.Up();
    tbd::Vec3 eye, target, up{0, 1, 0};
    tbd::Mat4 proj;
    const float farZ = std::max(120000.0f, 1.2f * 3570.0f * std::sqrt(std::max(1.0f, (float)plane.pos.y)));
    if (eyeView) { // headset: the seat is the tracking origin; cockpit seat moves with the aircraft, chase / orbit keep the horizon level
        tbd::Quat seatRot = plane.rot;
        tbd::Vec3 seatPos = p + tbd::Rotate(plane.rot, {0, 0.75f, -0.3f});
        if (g.camMode != 1) {
            seatRot = tbd::QuatAxisAngle({0, 1, 0}, std::atan2(-f.x, -f.z));
            seatPos = p + tbd::Rotate(seatRot, {0, 2.5f, 12.0f});
        }
        const tbd::Quat eyeRot = seatRot * eyeView->orientation;
        eye = seatPos + tbd::Rotate(seatRot, eyeView->position);
        target = eye + tbd::Rotate(eyeRot, {0, 0, -1});
        up = tbd::Rotate(eyeRot, {0, 1, 0});
        proj = ProjectionFov(eyeView->tanLeft, eyeView->tanRight, eyeView->tanUp, eyeView->tanDown, 0.1f, farZ);
    } else {
        if (g.camMode == 0) { eye = p - f * 16.0f + tbd::Vec3{0, 4.5f, 0}; target = p + f * 30.0f; }
        else if (g.camMode == 1) {
            eye = p + u * 1.05f - f * 0.3f;
            const float cp = std::cos(g.lookPitch);
            target = eye + tbd::Rotate(plane.rot, tbd::Vec3{-std::sin(g.lookYaw) * cp, std::sin(g.lookPitch), -std::cos(g.lookYaw) * cp}) * 10.0f;
            up = u;
        } else { eye = p + tbd::Vec3{std::sin(g.orbitYaw) * 25.0f, 8.0f, std::cos(g.orbitYaw) * 25.0f}; target = p; }
        proj = tbd::PerspectiveRH(tbd::Radians(g.camMode == 1 ? 70.0f : 60.0f), (float)g.w / (float)g.h, g.camMode == 1 ? 0.2f : 0.5f, farZ);
    }
    const tbd::Mat4 view = tbd::LookAtRH(eye, target, up);
    const tbd::Mat4 viewProj = proj * view;

    tbd::FrameView fv;
    fv.viewProj = viewProj; fv.invJittered = tbd::Inverse(viewProj); fv.eye = eye; fv.time = (float)g.simTime; fv.sky = true;
    fv.postTier = std::clamp(EnvInt("FLY_POST_TIER", 1), 0, 4); fv.dt = g.lastDt; fv.motionVectors = false;
    { // the air: aerial perspective and height fog; clear, haze, fog (F)
        static const float kAir[3][3] = {{1.0f, 0.8e-5f, 1500.0f}, {1.6f, 2.0e-5f, 1200.0f}, {3.0f, 3.0e-4f, 260.0f}};
        static const float kTint[3][4] = {{0.55f, 0.70f, 0.96f, 0.40f}, {0.64f, 0.74f, 0.90f, 0.22f}, {0.76f, 0.79f, 0.84f, 0.65f}};
        for (int c = 0; c < 3; ++c) { fv.air[c] = kAir[g.fog][c]; fv.air2[c] = std::pow(kTint[g.fog][c], 2.2f); }
        fv.air[3] = 1.85f; fv.air2[3] = kTint[g.fog][3];
        fv.shade[3] = 1.0f - std::clamp(EnvFloat("FLY_CLOUDS", 1.0f), 0.0f, 1.0f);
        fv.horizon[0] = 1.0f; fv.horizon[1] = 1.08f; fv.horizon[2] = 0.3f; fv.horizon[3] = 0.3f;
        fv.sunGap[7][3] = (float)origin.y; // post.hlsl kOriginY: scene y back to altitude for the air
    }
    r.SetFrameView(fv);
    tbd::Lighting light = tbd::ComputeLighting(g.sun);
    light.eye[0] = eye.x; light.eye[1] = eye.y; light.eye[2] = eye.z;
    // the only shadow: the aircraft's own, in a small sun map round it (it lands on the runway on approach and on the ground)
    const bool shadows = tbd::ComputeShadow(g.sun, p, 40.0f, light);
    tbd::ComputeCloudMap(eye, false, light);
    r.SetWindowPortals(nullptr, 0);
    r.SetInteriorRooms(nullptr, 0);
    r.SetLighting(light);
    { // the satellite mosaic is the ground's whole colour, right up to the eye
        float ia[4], ib[4];
        g.sat.img.Mapping(g.gen->anchor, origin, ia, ib);
        const float look[4] = {1.0f, 1.0f, 0.0f, 1.0f}; // all of its hue, all of its brightness on pavement, no near fade
        r.SetGroundImagery(ia, ib, look);
        const float gs[4] = {0, 0, 0, 140.0f};
        r.SetGroundStyle(gs);
        const float wt[4] = {0, 0, 0, 0}; // no world atlas
        r.SetWorldTexOrigin(wt);
    }
    const float clear[4] = {0.55f, 0.7f, 0.9f, 1.0f};
    r.BeginFrame(clear);
    if (shadows) {
        r.BeginShadowPass(tbd::ShadowMap::Sun);
        r.DrawShadowCaster(g.planeMesh, town::BodyMatrix(plane, origin));
        r.EndShadowPass();
    }
    const Frustum frustum(viewProj);
    const bool cull = !g.settings.vr; // a headset's two eyes share no one viewProj here
    r.SetMaterial(tbd::kNoTexture, tbd::kNoTexture, g.sat.img.Ok() ? g.sat.img.tex : tbd::kNoTexture);
    g.terrain.ForEach([&](const Terrain::Chunk& ch) {
        const tbd::Vec3 d{(float)(ch.origin.x - origin.x), (float)(ch.origin.y - origin.y), (float)(ch.origin.z - origin.z)};
        if (cull && !frustum.BoxVisible({d.x + ch.lo[0], d.y + ch.lo[1], d.z + ch.lo[2]}, {d.x + ch.hi[0], d.y + ch.hi[1], d.z + ch.hi[2]})) return;
        const tbd::Mat4 model = Relative(ch.origin, origin);
        r.Draw(ch.mesh, viewProj * model, model, true, true);
    });
    static double coastKm = 0.0; // the sea: one big quad at sea level, only within reach of a coast
    if (g.frame % 30 == 0 || coastKm == 0.0) coastKm = g.gen->Sample(plane.pos.x, plane.pos.z).coastKm;
    if (coastKm < 150.0) {
        const float S = 120000.0f, y = (float)(0 - origin.y);
        const tbd::Vertex sea[6] = {{-S, y, -S, 0, 1, 0, .05f, .16f, .25f, 1}, {-S, y, S, 0, 1, 0, .05f, .16f, .25f, 1}, {S, y, S, 0, 1, 0, .05f, .16f, .25f, 1},
                                    {-S, y, -S, 0, 1, 0, .05f, .16f, .25f, 1}, {S, y, S, 0, 1, 0, .05f, .16f, .25f, 1}, {S, y, -S, 0, 1, 0, .05f, .16f, .25f, 1}};
        tbd::Mat4 m = tbd::Mat4::Identity();
        m.At(0, 3) = p.x; m.At(2, 3) = p.z;
        r.DrawTransient(sea, 6, tbd::Topology::Triangles, viewProj * m, m, true);
    }
    r.SetMaterial(tbd::kNoTexture, tbd::kNoTexture);
    if (g.runwayMesh != tbd::kInvalidMesh) {
        const tbd::Mat4 model = Relative({0, 0, 0}, origin);
        r.Draw(g.runwayMesh, viewProj * model, model, true, true);
    }
    if (g.camMode != 1 || eyeView) {
        const tbd::Mat4 model = town::BodyMatrix(plane, origin);
        r.Draw(g.planeMesh, viewProj * model, model, true, true);
    }
    r.ResolveScene();

    static const bool hudOn = EnvInt("FLY_HUD", 1) != 0;
    if (!hudOn || g.settings.vr || eyeView) return;
    { // frame times: the CPU frame, and the GPU's passes summed
        const int64_t now = tbd::platform::Ticks();
        if (g.perf.last) {
            const double ms = Seconds(now - g.perf.last) * 1000.0;
            const double* gp = r.GpuPasses();
            double gsum = 0; for (int k = 0; k < 10; ++k) gsum += gp[k];
            g.perf.cpuMs = g.perf.cpuMs > 0 ? g.perf.cpuMs * 0.95 + ms * 0.05 : ms;
            g.perf.gpuMs = g.perf.gpuMs > 0 ? g.perf.gpuMs * 0.95 + gsum * 0.05 : gsum;
            if (g.frame % 240 == 0) {
                const tbd::FrameCounts c = r.Counts();
                tbd::Log("perf: %.0f fps cpu %.2f ms gpu %.2f ms | draws %u tris %.2fM | terrain chunks %zu", 1000.0 / std::max(0.1, g.perf.cpuMs), g.perf.cpuMs, g.perf.gpuMs,
                         c.draws, (double)c.triangles / 1e6, g.terrain.Count());
            }
        }
        g.perf.last = now;
    }
    UiKit& hud = g.ui;
    hud.Begin();
    const float S = std::max(0.75f, g.h / 900.0f), m = 18 * S;
    const float hdg = std::fmod(tbd::Degrees(std::atan2(f.x, -f.z)) + 360.0f, 360.0f);
    char b[160];
    hud.Glass(m, m, m + 330 * S, m + 112 * S, 10 * S);
    std::snprintf(b, sizeof b, "%s", g.airport >= 0 ? (g.airports[(size_t)g.airport].icao + "  " + g.airports[(size_t)g.airport].name).c_str() : g.gen->placeName.c_str());
    hud.Text(m + 14 * S, m + 26 * S, 15 * S, b, Rgba(255, 220, 150), FontFace::Bold);
    std::snprintf(b, sizeof b, "IAS %3.0f kt   ALT %5.0f ft   HDG %03.0f", plane.Airspeed() * 1.944f, (plane.pos.y) * 3.281, hdg);
    hud.Text(m + 14 * S, m + 52 * S, 14 * S, b, Rgba(255, 255, 255));
    std::snprintf(b, sizeof b, "AGL %5.0f ft   THR %3.0f %%   VS %+5.0f fpm", agl * 3.281, g.ctl.throttle * 100.0f, plane.vel.y * 196.85f);
    hud.Text(m + 14 * S, m + 74 * S, 14 * S, b, Rgba(255, 255, 255));
    std::snprintf(b, sizeof b, "%.0f fps   cpu %.1f ms   gpu %.1f ms   %s", 1000.0 / std::max(0.1, g.perf.cpuMs), g.perf.cpuMs, g.perf.gpuMs, g.sat.Status().c_str());
    hud.Text(m + 14 * S, m + 96 * S, 12 * S, b, Rgba(170, 230, 170));
    if (plane.crashed) hud.TextCentered(g.w * 0.5f, g.h * 0.4f, 34 * S, "CRASHED  -  R to reset", Rgba(255, 120, 100), FontFace::Bold);
    hud.Text(m, g.h - m, 12 * S, g.settings.touchHints ? "Sticks fly  |  slider throttle" : "Arrows / WASD fly  |  +/- throttle  |  C camera  |  R reset  |  F fog",
             Rgba(220, 225, 235, 190));
    hud.TextCentered(g.w * 0.5f, g.h - m, 11 * S, g.sat.Credit(), Rgba(220, 225, 235, 140));
    hud.End(r, g.w, g.h, false);
}

} // namespace fly
