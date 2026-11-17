// A sound output stream: the platform opens the default device (miniaudio: WASAPI on Windows, Core Audio on Apple) and
// calls the render callback on its own thread for interleaved float frames. Built only into the game build (FLY_GAME_AUDIO).
#pragma once
#include <cstdint>

namespace tbd::audio_out {

using RenderFn = void (*)(void* user, float* out, uint32_t frames, uint32_t channels, uint32_t rate);
bool Start(RenderFn fn, void* user); // false: no device (or TBD_NOAUDIO / a capture run); nothing plays, nothing fails
void Stop();

} // namespace tbd::audio_out
