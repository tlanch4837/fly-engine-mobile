// The sound output stream (audio_out.h) on miniaudio's device layer only (no decoders, no engine).
#define MINIAUDIO_IMPLEMENTATION
#define MA_NO_DECODING
#define MA_NO_ENCODING
#define MA_NO_RESOURCE_MANAGER
#define MA_NO_NODE_GRAPH
#define MA_NO_ENGINE
#define MA_NO_GENERATION
#include "../third_party/miniaudio.h"

#include <cstdlib>

#include "audio_out.h"

namespace tbd::audio_out {
namespace {
ma_device g_device;
bool g_open = false;
RenderFn g_fn = nullptr;
void* g_user = nullptr;

void Callback(ma_device* d, void* out, const void*, ma_uint32 frames) {
    if (g_fn) g_fn(g_user, static_cast<float*>(out), frames, d->playback.channels, d->sampleRate);
}
} // namespace

bool Start(RenderFn fn, void* user) {
    if (g_open) return true;
    if (std::getenv("TBD_NOAUDIO") || std::getenv("FLY_CAPTURE") || std::getenv("FLY_RECORD")) return false;
    g_fn = fn; g_user = user;
    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format = ma_format_f32;
    cfg.playback.channels = 2;
    cfg.sampleRate = 48000;
    cfg.dataCallback = Callback;
    if (ma_device_init(nullptr, &cfg, &g_device) != MA_SUCCESS) return false;
    if (ma_device_start(&g_device) != MA_SUCCESS) { ma_device_uninit(&g_device); return false; }
    g_open = true;
    return true;
}

void Stop() {
    if (!g_open) return;
    ma_device_uninit(&g_device);
    g_open = false;
    g_fn = nullptr;
}

} // namespace tbd::audio_out
