// Streamed satellite imagery for the far ground: Web Mercator (XYZ) image tiles fetched in the background around the camera,
// cached on disk, and stitched into one mosaic texture that rides in the same ground-imagery slot as the baked NAIP sets
// (imagery.h, mesh.hlsl GroundImagery). The near ground stays procedural: the shader fades the imagery in only past a distance
// (the look's [2] [3]), so the towns, fields and roads generated around the camera keep their own detail, and the satellite
// picture takes over the distant terrain out to the horizon.
//
// LOD: the tile zoom follows the camera's height above the ground (lower = finer tiles, a smaller mosaic), and the texture's mip
// chain picks the detail per pixel. The mosaic is rebuilt (on its own thread, then swapped in) when the camera leaves its middle
// half or the zoom changes; the old one keeps drawing meanwhile.
//
// Sources, tried per tile in order (FLY_SAT_URL={z}/{x}/{y} template replaces them, e.g. our own CDN for shipping builds):
//   USGS The National Map "USGSImageryOnly" (public domain; US only), then EOX Sentinel-2 cloudless 2016 (CC BY 4.0, worldwide).
// Free-data rule: a shipping build must point FLY_SAT_URL at our own tile copies, not at these public servers from players.
// Portable: std threads and platform::HttpGet / DecodeImage only.
#pragma once
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "../platform/platform.h"
#include "../render/renderer.h"
#include "imagery.h"

namespace fly {

class SatStream {
public:
    GroundImagery img;          // the live mosaic: texture + Web Mercator box (img.Ok() once the first one is in)
    bool enabled = true;
    int tilesPerSide = 16;      // 16 x 256 px = a 4096 px mosaic
    int nearZoom = 13;          // the zoom near the ground (the basic build: 14, its only ground detail); one less per height band

    ~SatStream() { Stop(); }

    void Start(const std::filesystem::path& cacheDir) {
        cacheDir_ = cacheDir;
        char b[512] = {};
        if (tbd::platform::GetEnvA("FLY_SAT_URL", b, sizeof b) > 0) sources_ = {{"custom", b, "Imagery: custom source"}};
        else sources_ = {{"usgs", "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/tile/{z}/{y}/{x}", "Imagery: USGS The National Map"},
                         {"s2", "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless_3857/default/g/{z}/{y}/{x}.jpg",
                          "Sentinel-2 cloudless 2016 by EOX (CC BY 4.0), contains modified Copernicus Sentinel data"}};
        if (tbd::platform::GetEnvA("FLY_SAT_TILES", b, sizeof b) > 0) tilesPerSide = std::clamp(std::atoi(b), 4, 32);
        if (tbd::platform::GetEnvA("FLY_SAT_ZOOM", b, sizeof b) > 0) fixedZoom_ = std::clamp(std::atoi(b), 3, 17);
        thread_ = std::thread([this] { Run(); });
        tbd::platform::LowerThreadPriority(thread_);
    }
    void Stop() {
        { std::lock_guard<std::mutex> l(m_); stop_ = true; }
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }

    // Frame thread, outside BeginFrame..EndFrame: asks for a mosaic around the camera when the live one no longer fits, and swaps a
    // finished one in.
    void Update(tbd::Renderer& r, double lat, double lon, double aboveGround) {
        if (!enabled || !thread_.joinable()) return;
        { // a finished mosaic -> the live texture
            std::unique_lock<std::mutex> l(m_);
            if (ready_) {
                Result res = std::move(*ready_);
                ready_.reset();
                l.unlock();
                const tbd::TextureId tex = r.CreateTextureNoWait(res.levels, res.dims);
                if (tex != tbd::kNoTexture) {
                    if (img.Ok()) r.RetireTexture(img.tex);
                    img.tex = tex;
                    for (int c = 0; c < 4; ++c) img.merc[c] = res.merc[c];
                    img.width = res.dims[0].first; img.height = res.dims[0].second;
                    img.name = "sat_z" + std::to_string(res.zoom);
                    liveZoom_ = res.zoom; liveX0_ = res.x0; liveY0_ = res.y0;
                    credit_ = res.credit;
                    tbd::Log("sat: z%d mosaic %dx%d live (%d tiles fetched, %d from cache, %d missing) in %.1f s", res.zoom, img.width, img.height,
                             res.fetched, res.cached, res.missing, res.seconds);
                }
            }
        }
        // the zoom for this height, with hysteresis so it doesn't flap at a boundary
        int z = fixedZoom_;
        if (z < 0) {
            static const double kUp[] = {2500, 9000, 30000, 90000}; // above kUp[k] use zoom 13-k-1 (13 near the ground .. 9 very high)
            const int shift = nearZoom - 13; // the bands below are written for zoom 13 at the ground
            const int cur = wantZoom_ > 0 ? wantZoom_ - shift : 13;
            int target = 13;
            for (double t : kUp) if (aboveGround > t) --target;
            if (target < cur && aboveGround < kUp[std::clamp(13 - cur, 0, 3)] * 1.25) target = cur;              // climbing: a little past the line first
            if (target > cur && aboveGround > kUp[std::clamp(12 - cur, 0, 3)] * 0.75) target = cur; // descending: likewise
            z = target + shift;
        }
        wantZoom_ = z;
        const double n = std::pow(2.0, z);
        const double fx = (lon + 180.0) / 360.0 * n;
        const double la = std::clamp(lat, -85.0, 85.0) * kPi / 180.0;
        const double fy = (1.0 - std::log(std::tan(la) + 1.0 / std::cos(la)) / kPi) * 0.5 * n;
        const int N = tilesPerSide;
        const auto fits = [&](int zz, int x0, int y0) {
            return zz == z && fx > x0 + N * 0.25 && fx < x0 + N * 0.75 && fy > y0 + N * 0.25 && fy < y0 + N * 0.75;
        };
        std::lock_guard<std::mutex> l(m_);
        if (fits(liveZoom_, liveX0_, liveY0_) || (busy_ && fits(jobZoom_, jobX0_, jobY0_)) || (job_ && fits(job_->zoom, job_->x0, job_->y0))) return;
        job_ = Job{z, (int)std::floor(fx) - N / 2, (int)std::floor(fy) - N / 2, N};
        cv_.notify_all();
    }

    const std::string& Credit() const { return credit_; }
    // "fetching 37/256" while a mosaic is being made, else empty
    std::string Status() const {
        if (!busy_) return {};
        char b[64];
        std::snprintf(b, sizeof b, "satellite z%d  %d/%d", jobZoom_.load(), progress_.load(), jobTotal_.load());
        return b;
    }

private:
    static constexpr double kPi = 3.14159265358979323846, kR = 6378137.0;
    struct Source { std::string id, url, credit; };
    struct Job { int zoom, x0, y0, n; };
    struct Result {
        std::vector<std::vector<uint8_t>> levels; std::vector<std::pair<int, int>> dims;
        double merc[4]; int zoom, x0, y0, fetched = 0, cached = 0, missing = 0; double seconds = 0; std::string credit;
    };

    std::filesystem::path cacheDir_;
    std::vector<Source> sources_;
    std::thread thread_;
    std::mutex m_;
    std::condition_variable cv_;
    bool stop_ = false;
    std::optional<Job> job_;
    std::optional<Result> ready_;
    std::atomic<bool> busy_{false};
    std::atomic<int> jobZoom_{0}, progress_{0}, jobTotal_{0};
    int jobX0_ = 0, jobY0_ = 0;
    int fixedZoom_ = -1, wantZoom_ = 0, liveZoom_ = -1, liveX0_ = 0, liveY0_ = 0;
    std::string credit_;

    static std::string Fill(std::string t, int z, int x, int y) {
        auto rep = [&](const char* k, int v) { for (size_t p; (p = t.find(k)) != std::string::npos;) t.replace(p, std::strlen(k), std::to_string(v)); };
        rep("{z}", z); rep("{x}", x); rep("{y}", y);
        return t;
    }
    static bool ReadFile(const std::filesystem::path& p, std::vector<uint8_t>& out) {
        std::ifstream f(p, std::ios::binary);
        if (!f) return false;
        out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
        return !out.empty();
    }
    // A server's "no data here" tile: near one flat colour.
    static bool Blank(const std::vector<uint8_t>& rgba) {
        if (rgba.size() < 16) return true;
        double s = 0, s2 = 0; size_t n = 0;
        for (size_t i = 0; i + 3 < rgba.size(); i += 4 * 97) { const double v = rgba[i] + rgba[i + 1] + rgba[i + 2]; s += v; s2 += v * v; ++n; }
        const double mean = s / n, var = s2 / n - mean * mean;
        return var < 9.0;
    }

    // One tile, 256 x 256 RGBA: from the disk cache, else from the first source that has it (written to the cache). -1 none, 0 cache, 1 net.
    int Tile(int z, int x, int y, std::vector<uint8_t>& rgba, int& src) {
        const int n = 1 << z;
        x = ((x % n) + n) % n;
        if (y < 0 || y >= n) return -1;
        for (size_t k = 0; k < sources_.size(); ++k) {
            const std::filesystem::path dir = cacheDir_ / sources_[k].id / std::to_string(z);
            const std::filesystem::path file = dir / (std::to_string(x) + "_" + std::to_string(y) + ".img"), none = dir / (std::to_string(x) + "_" + std::to_string(y) + ".none");
            std::error_code ec;
            if (std::filesystem::exists(none, ec)) continue;
            std::vector<uint8_t> bytes;
            int w = 0, h = 0, from = 0;
            if (!ReadFile(file, bytes)) {
                int status = 0;
                if (!tbd::platform::HttpGet(Fill(sources_[k].url, z, x, y), bytes, &status)) {
                    if (status == 404 || status == 400) { std::filesystem::create_directories(dir, ec); std::ofstream(none).put('0'); }
                    continue;
                }
                from = 1;
                // a flat "no data" tile sends us on to the next source; the last source's tiles are taken as they are (open sea)
                const bool last = k + 1 == sources_.size();
                if (tbd::platform::DecodeImage(bytes.data(), bytes.size(), rgba, w, h) && w == 256 && h == 256 && (last || !Blank(rgba))) {
                    std::filesystem::create_directories(dir, ec);
                    std::ofstream(file, std::ios::binary).write((const char*)bytes.data(), (std::streamsize)bytes.size());
                    src = (int)k;
                    return from;
                }
                std::filesystem::create_directories(dir, ec); std::ofstream(none).put('0');
                continue;
            }
            if (tbd::platform::DecodeImage(bytes.data(), bytes.size(), rgba, w, h) && w == 256 && h == 256) { src = (int)k; return from; }
        }
        return -1;
    }

    void Run() {
        for (;;) {
            Job job;
            {
                std::unique_lock<std::mutex> l(m_);
                cv_.wait(l, [&] { return stop_ || job_.has_value(); });
                if (stop_) return;
                job = *job_; job_.reset();
                jobZoom_ = job.zoom; jobX0_ = job.x0; jobY0_ = job.y0; jobTotal_ = job.n * job.n; progress_ = 0;
                busy_ = true;
            }
            const int64_t t0 = tbd::platform::Ticks();
            const int N = job.n, W = N * 256;
            Result res;
            res.zoom = job.zoom; res.x0 = job.x0; res.y0 = job.y0;
            std::vector<uint8_t> mosaic((size_t)W * W * 4, 0);
            std::vector<char> have((size_t)N * N, 0);
            std::vector<int> used(sources_.size(), 0);
            std::atomic<int> next{0}, fetched{0}, cached{0};
            std::mutex usedM;
            // nearest tiles first, so a cancelled or slow build still has the middle
            std::vector<int> order(N * N);
            for (int i = 0; i < N * N; ++i) order[i] = i;
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                const auto d = [&](int i) { const double dx = i % N - (N - 1) * 0.5, dy = i / N - (N - 1) * 0.5; return dx * dx + dy * dy; };
                return d(a) < d(b);
            });
            auto worker = [&] {
                std::vector<uint8_t> rgba;
                for (int i; (i = next.fetch_add(1)) < N * N;) {
                    { std::lock_guard<std::mutex> l(m_); if (stop_) return; }
                    const int t = order[i], tx = t % N, ty = t / N;
                    int src = -1;
                    const int got = Tile(job.zoom, job.x0 + tx, job.y0 + ty, rgba, src);
                    if (got >= 0) {
                        (got ? fetched : cached)++;
                        for (int row = 0; row < 256; ++row)
                            std::memcpy(&mosaic[(((size_t)ty * 256 + row) * W + (size_t)tx * 256) * 4], &rgba[(size_t)row * 256 * 4], 256 * 4);
                        have[t] = 1;
                        std::lock_guard<std::mutex> l(usedM);
                        ++used[src];
                    }
                    ++progress_;
                }
            };
            std::vector<std::thread> pool;
            for (int k = 0; k < 8; ++k) pool.emplace_back(worker);
            for (auto& th : pool) th.join();
            { std::lock_guard<std::mutex> l(m_); if (stop_) return; }
            // holes (no source has them): the mean colour of what came, so the far ground doesn't go black
            double mean[3] = {0, 0, 0}; size_t cnt = 0;
            for (int t = 0; t < N * N; ++t) {
                if (!have[t]) continue;
                const size_t at = (((size_t)(t / N) * 256 + 128) * W + (size_t)(t % N) * 256 + 128) * 4;
                for (int c = 0; c < 3; ++c) mean[c] += mosaic[at + c];
                ++cnt;
            }
            res.missing = N * N - (int)cnt;
            if (cnt == 0) { tbd::Log("sat: z%d no tiles from any source (offline?)", job.zoom); busy_ = false; continue; }
            for (int t = 0; t < N * N; ++t) {
                if (have[t]) continue;
                for (int row = 0; row < 256; ++row)
                    for (int col = 0; col < 256; ++col) {
                        uint8_t* p = &mosaic[(((size_t)(t / N) * 256 + row) * W + (size_t)(t % N) * 256 + col) * 4];
                        for (int c = 0; c < 3; ++c) p[c] = (uint8_t)(mean[c] / cnt);
                    }
            }
            for (size_t i = 3; i < mosaic.size(); i += 4) mosaic[i] = 255;
            // the mip chain (a box filter), for the renderer to upload without waiting
            res.dims.push_back({W, W});
            res.levels.push_back(std::move(mosaic));
            for (int w = W; w > 1;) {
                const int h = w / 2;
                const std::vector<uint8_t>& a = res.levels.back();
                std::vector<uint8_t> b((size_t)h * h * 4);
                for (int y = 0; y < h; ++y)
                    for (int x = 0; x < h; ++x)
                        for (int c = 0; c < 4; ++c) {
                            const size_t i0 = (((size_t)y * 2) * w + (size_t)x * 2) * 4 + c, i1 = i0 + (size_t)w * 4;
                            b[((size_t)y * h + x) * 4 + c] = (uint8_t)((a[i0] + a[i0 + 4] + a[i1] + a[i1 + 4] + 2) / 4);
                        }
                res.levels.push_back(std::move(b));
                res.dims.push_back({h, h});
                w = h;
            }
            const double n = std::pow(2.0, job.zoom), span = 2.0 * kPi * kR;
            res.merc[0] = job.x0 / n * span - kPi * kR; res.merc[2] = (job.x0 + N) / n * span - kPi * kR;
            res.merc[1] = kPi * kR - job.y0 / n * span; res.merc[3] = kPi * kR - (job.y0 + N) / n * span;
            res.fetched = fetched; res.cached = cached;
            for (size_t k = 0; k < sources_.size(); ++k)
                if (used[k]) res.credit += (res.credit.empty() ? "" : "   ") + sources_[k].credit;
            res.seconds = (double)(tbd::platform::Ticks() - t0) / (double)tbd::platform::TickFrequency();
            {
                std::lock_guard<std::mutex> l(m_);
                ready_ = std::move(res);
                busy_ = false;
            }
        }
    }
};

} // namespace fly
