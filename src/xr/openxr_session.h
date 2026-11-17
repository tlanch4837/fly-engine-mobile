// OpenXR layer, shared by PCVR (Windows, a Quest 3 over Link / Air or any OpenXR headset) and later Quest standalone (Android).
// Graphics: the Vulkan renderer in offscreen mode (Renderer::SetXrBridge / SetEndFrameCopyTarget), so PCVR runs the same
// rendering code as the Quest will. No OpenXR or Vulkan types leak out of this header.
#pragma once
#include <memory>

#include "../fly/game.h"
#include "../render/renderer.h"

namespace fly::xr {

class Session {
public:
    Session();
    ~Session();
    // Finds the runtime and the headset. False when there is none (no runtime installed, headset off or not connected).
    bool CreateInstance();
    // Pass to the Vulkan renderer before Init(nullptr, w, h): lets the runtime create the instance and device.
    const tbd::Renderer::XrVulkanBridge* Bridge() const;
    // The runtime's recommended per-eye render size.
    void EyeSize(int& width, int& height) const;
    // After the renderer's Init: the session, reference space, swapchains and controller actions.
    bool Start(tbd::Renderer& r);
    // One headset frame: events, input, simulate once, render both eyes, submit. False when the app should quit.
    // `extra` is input from other devices (keyboard, gamepad) merged under the controllers'.
    bool Frame(tbd::Renderer& r, Game& game, const Input& extra);

private:
    struct Impl;
    std::unique_ptr<Impl> m_;
};

} // namespace fly::xr
