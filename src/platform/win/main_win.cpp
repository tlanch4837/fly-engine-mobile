// Windows shell: a Win32 window, the Direct3D 12 (default) or Vulkan renderer, keyboard and XInput gamepad -> fly::Input.
// Everything else is the portable game (src/fly/game.h).
//
// Keys: W/S throttle, arrows or numpad pitch/roll, A/D rudder, C camera, R reset, Esc quit.
// Gamepad (Xbox / XInput): left stick pitch/roll, triggers rudder, right stick Y throttle, Y camera, Back reset.
// VR: FLY_VR=1 renders to an OpenXR headset (PCVR; a Quest 3 over Link / Air): right stick pitch/roll, left stick throttle/rudder,
//     A or X camera, B or Y reset.
// Env: FLY_VR=1, FLY_API=vulkan, FLY_START=x,y,z, FLY_GEO=lat,lon, FLY_CAM=0|1|2, FLY_CAPTURE=<file.bmp> (writes frame
// FLY_CAPTURE_FRAME, default 120, and quits), FLY_MAP=1 (open on the world map).
#define NOMINMAX
#include <windows.h>
#include <xinput.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <atomic>
#include <vector>

#include "../../fly/game.h"
#include "../../xr/openxr_session.h"
#include "../../render/renderer.h"

namespace {

HWND g_window = nullptr;
int g_w = 1600, g_h = 900;
bool g_resized = false, g_quit = false, g_esc = false, g_fsKey = false;
float g_wheel = 0;

LRESULT CALLBACK WndProc(HWND h, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_SIZE: g_w = std::max(1, (int)LOWORD(lp)); g_h = std::max(1, (int)HIWORD(lp)); g_resized = true; return 0;
    case WM_CLOSE: g_quit = true; return 0;
    case WM_KEYDOWN: if (wp == VK_ESCAPE) g_esc = true; if (wp == VK_F11) g_fsKey = true; return 0;
    case WM_SYSKEYDOWN: if (wp == VK_RETURN && (lp & (1 << 29))) { g_fsKey = true; return 0; } break;
    case WM_MOUSEWHEEL: g_wheel += (float)GET_WHEEL_DELTA_WPARAM(wp) / WHEEL_DELTA; return 0;
    }
    return DefWindowProcW(h, msg, wp, lp);
}

// Borderless fullscreen on the window's monitor, or back to the window it was.
void SetFullscreen(bool on) {
    static WINDOWPLACEMENT placed{sizeof(WINDOWPLACEMENT)};
    const LONG style = GetWindowLongW(g_window, GWL_STYLE);
    if (on) {
        MONITORINFO mi{sizeof(MONITORINFO)};
        if (!GetWindowPlacement(g_window, &placed) || !GetMonitorInfoW(MonitorFromWindow(g_window, MONITOR_DEFAULTTOPRIMARY), &mi)) return;
        SetWindowLongW(g_window, GWL_STYLE, style & ~WS_OVERLAPPEDWINDOW);
        SetWindowPos(g_window, HWND_TOP, mi.rcMonitor.left, mi.rcMonitor.top, mi.rcMonitor.right - mi.rcMonitor.left, mi.rcMonitor.bottom - mi.rcMonitor.top,
                     SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
    } else {
        SetWindowLongW(g_window, GWL_STYLE, style | WS_OVERLAPPEDWINDOW);
        SetWindowPlacement(g_window, &placed);
        SetWindowPos(g_window, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
    }
}

bool Key(int vk) { return GetForegroundWindow() == g_window && (GetAsyncKeyState(vk) & 0x8000) != 0; }

std::filesystem::path ExeDir() {
    wchar_t buf[MAX_PATH];
    GetModuleFileNameW(nullptr, buf, MAX_PATH);
    return std::filesystem::path(buf).parent_path();
}

float Stick(SHORT v) { // XInput axis with a dead zone, -1..1
    const float f = std::max(-1.0f, v / 32767.0f);
    return std::abs(f) < 0.12f ? 0.0f : (f - std::copysign(0.12f, f)) / 0.88f;
}

bool WriteBmp(const char* path, const std::vector<uint8_t>& bgra, int w, int h) { // CapturedFrame is BGRA already
    FILE* f = nullptr;
    if (fopen_s(&f, path, "wb") != 0 || !f) return false;
    const uint32_t row = w * 3 + (4 - (w * 3) % 4) % 4, size = 54 + row * h;
    uint8_t hdr[54] = {'B', 'M'};
    memcpy(hdr + 2, &size, 4); uint32_t off = 54, dib = 40; memcpy(hdr + 10, &off, 4); memcpy(hdr + 14, &dib, 4);
    memcpy(hdr + 18, &w, 4); int nh = -h; memcpy(hdr + 22, &nh, 4); hdr[26] = 1; hdr[28] = 24;
    fwrite(hdr, 1, 54, f);
    std::vector<uint8_t> line(row);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) { const uint8_t* p = &bgra[(y * w + x) * 4]; line[x * 3] = p[0]; line[x * 3 + 1] = p[1]; line[x * 3 + 2] = p[2]; }
        fwrite(line.data(), 1, row, f);
    }
    fclose(f);
    return true;
}

} // namespace

int WINAPI wWinMain(HINSTANCE inst, HINSTANCE, PWSTR, int) {
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    WNDCLASSW wc{};
    wc.lpfnWndProc = WndProc; wc.hInstance = inst; wc.lpszClassName = L"FlyEngine"; wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    RegisterClassW(&wc);
    { char sz[32] = {}; int sw = 0, sh = 0; // FLY_SIZE=w,h: the window's client size (recordings)
      if (GetEnvironmentVariableA("FLY_SIZE", sz, sizeof sz) > 0 && sscanf_s(sz, "%d,%d", &sw, &sh) == 2 && sw > 64 && sh > 64) { g_w = sw; g_h = sh; } }
    RECT rc{0, 0, g_w, g_h};
    AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
    g_window = CreateWindowW(L"FlyEngine", L"Fly-engine", WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT,
                             rc.right - rc.left, rc.bottom - rc.top, nullptr, nullptr, inst, nullptr);
    { RECT cr; GetClientRect(g_window, &cr); g_w = std::max(1L, cr.right); g_h = std::max(1L, cr.bottom); }

    // FLY_VR=1: PCVR through OpenXR (a Quest 3 over Link / Air, or any OpenXR headset). Vulkan, offscreen, one image per eye.
    std::unique_ptr<fly::xr::Session> xr;
    if (GetEnvironmentVariableA("FLY_VR", nullptr, 0) > 0) {
        xr = std::make_unique<fly::xr::Session>();
        if (!xr->CreateInstance()) {
            MessageBoxW(g_window, L"No OpenXR headset found. Is the Meta Quest Link app running and the headset connected (Link or Air Link)? "
                                  L"Details in engine.log.", L"Fly-engine VR", MB_ICONERROR);
            return 1;
        }
    }
    char api[16] = {};
    GetEnvironmentVariableA("FLY_API", api, sizeof(api));
    std::unique_ptr<tbd::Renderer> r = xr || _stricmp(api, "vulkan") == 0 ? tbd::CreateVulkanRenderer() : tbd::CreateD3D12Renderer();
    if (!(xr || _stricmp(api, "vulkan") == 0)) { // fill the shader cache on every core first: a changed shader file otherwise compiles one entry point at a time (minutes)
        std::atomic<int> done{0}, total{0}, failed{0};
        const std::atomic<bool> stop{false};
        const auto t0 = std::chrono::steady_clock::now();
        tbd::PrecompileD3D12Shaders(done, total, stop, &failed);
        tbd::Log("D3D12: shader cache ready, %d shaders (%d failed) in %.1f s on all cores", total.load(), failed.load(),
                 std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    }
    int renderW = g_w, renderH = g_h;
    if (xr) {
        xr->EyeSize(renderW, renderH);
        r->SetXrBridge(xr->Bridge());
    }
    // FLY_OFFSCREEN_TEST=1 (with FLY_API=vulkan): the headset's offscreen path without a headset, at a Quest 3 eye size; use with FLY_CAPTURE.
    const bool offscreenTest = !xr && GetEnvironmentVariableA("FLY_OFFSCREEN_TEST", nullptr, 0) > 0;
    if (offscreenTest) { renderW = 1680; renderH = 1760; r->SetTemporalHistory(false); }
    if (!r || !r->Init(xr || offscreenTest ? nullptr : g_window, renderW, renderH, false) || (xr && !xr->Start(*r))) {
        MessageBoxW(g_window, L"Renderer failed to start (see engine.log).", L"Fly-engine", MB_ICONERROR);
        return 1;
    }
    tbd::Log("Fly-engine on %s%s", r->Name(), xr ? " (OpenXR)" : "");
    if (xr) SetWindowTextW(g_window, L"Fly-engine VR - the picture is in the headset. Keyboard and gamepad still work while this window has focus.");

    fly::Settings settings;
    settings.vr = xr != nullptr;
    settings.dataDir = ExeDir() / ".." / "data";
    char b[96] = {};
    if (GetEnvironmentVariableA("FLY_START", b, sizeof b) > 0 &&
        sscanf_s(b, "%lf,%lf,%lf", &settings.startX, &settings.startY, &settings.startZ) == 3) settings.hasStart = true;
    if (GetEnvironmentVariableA("FLY_GEO", b, sizeof b) > 0 && sscanf_s(b, "%lf,%lf", &settings.startLat, &settings.startLon) == 2) settings.hasStartGeo = true;
    if (GetEnvironmentVariableA("FLY_CAM", b, sizeof b) > 0) settings.startCamera = atoi(b);
    char capturePath[MAX_PATH] = {};
    const bool capture = GetEnvironmentVariableA("FLY_CAPTURE", capturePath, MAX_PATH) > 0;
    // FLY_RECORD=<file.mp4>: every frame at a fixed FLY_RECORD_FPS (default 30) piped to ffmpeg (on the PATH), from frame
    // FLY_RECORD_SKIP on (default 0); FLY_SEQ_QUIT=1 ends it with the ground sequence, else Esc.
    char recordPath[MAX_PATH] = {};
    const bool record = !capture && GetEnvironmentVariableA("FLY_RECORD", recordPath, MAX_PATH) > 0;
    int recordFps = 30, recordSkip = 0;
    { char f[16] = {}; if (GetEnvironmentVariableA("FLY_RECORD_FPS", f, sizeof f) > 0) recordFps = std::clamp(atoi(f), 1, 240);
      if (GetEnvironmentVariableA("FLY_RECORD_SKIP", f, sizeof f) > 0) recordSkip = std::max(0, atoi(f)); }
    FILE* recordPipe = nullptr;
    int recordW = 0, recordH = 0;
    uint64_t recorded = 0;

    fly::Game game;
    game.Resize(renderW, renderH);
    if (!game.Start(*r, settings)) return 1;
    if (game.HandlesEscape() && !xr) SetWindowTextW(g_window, L"Optima Earth - A320 Flight");

    auto last = std::chrono::steady_clock::now();
    bool cWas = false, rWas = false;
    WORD padWas = 0;
    uint64_t frame = 0;
    while (!g_quit) {
        MSG msg;
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) { TranslateMessage(&msg); DispatchMessageW(&msg); }
        if (g_quit) break;
        if (g_resized) { if (!xr && !offscreenTest) { r->Resize(g_w, g_h); game.Resize(g_w, g_h); } g_resized = false; }
        const auto now = std::chrono::steady_clock::now();
        const float wallDt = std::chrono::duration<float>(now - last).count();
        float dt = std::min(0.05f, wallDt);
        last = now;
        if (capture) dt = 1.0f / 60.0f;
        if (record) dt = 1.0f / (float)recordFps;

        fly::Input in;
        auto axis = [](bool neg, bool pos) { return (pos ? 1.0f : 0.0f) - (neg ? 1.0f : 0.0f); };
        in.pitch = axis(Key(VK_UP) || Key(VK_NUMPAD8), Key(VK_DOWN) || Key(VK_NUMPAD2)); // stick back (down arrow) = nose up
        in.roll = axis(Key(VK_LEFT) || Key(VK_NUMPAD4), Key(VK_RIGHT) || Key(VK_NUMPAD6));
        in.yaw = axis(Key('A'), Key('D'));
        in.throttleRate = axis(Key('S'), Key('W'));
        const bool c = Key('C'), rk = Key('R');
        in.nextCamera = c && !cWas; in.reset = rk && !rWas;
        in.moveUp = axis(Key('Q'), Key('E')); in.fast = Key(VK_SHIFT);
        { static bool nWas = false; const bool nk = Key('N'); in.toggleNight = nk && !nWas; nWas = nk; }
        { static bool lWas = false; const bool lk = Key('L'); in.toggleLights = lk && !lWas; lWas = lk; }
        { static bool gWas = false; const bool gk = Key('G'); in.toggleGear = gk && !gWas; gWas = gk; }
        { static bool vWas = false; const bool vk = Key('V'); in.toggleFreeCam = vk && !vWas; vWas = vk; }
        { // the game build's systems and menus (gameplay.h): edges of Z X B T Q Backspace and the arrows
            static bool was[10] = {};
            const bool now[10] = {Key('Z'), Key('X'), Key('B'), Key('T'), Key('Q'), Key(VK_BACK), Key(VK_UP), Key(VK_DOWN), Key(VK_LEFT), Key(VK_RIGHT)};
            bool* edge[10] = {&in.flapsUp, &in.flapsDown, &in.toggleParkBrake, &in.toggleReverse, &in.toggleSpeedbrake, &in.back, &in.menuUp, &in.menuDown, &in.menuLeft, &in.menuRight};
            for (int k = 0; k < 10; ++k) { *edge[k] = now[k] && !was[k]; was[k] = now[k]; }
            in.brake = Key(VK_SPACE) ? 1.0f : 0.0f;
            in.trimRate = axis(Key(VK_NEXT), Key(VK_PRIOR));
            if (g_esc) { g_esc = false; if (game.HandlesEscape()) in.pause = true; else g_quit = true; }
            if (g_fsKey) { g_fsKey = false; in.toggleFullscreen = true; }
        }
        if (Key(VK_SPACE)) in.moveUp = 1.0f; // the free camera: Space rises, Ctrl sinks (Q / E too)
        if (Key(VK_CONTROL)) in.moveUp = -1.0f;
        { // head look in the cockpit: drag with the right mouse button
            static POINT lastCursor{};
            POINT pt{}; GetCursorPos(&pt);
            if (Key(VK_RBUTTON)) { in.lookX = (pt.x - lastCursor.x) * 0.0035f; in.lookY = (pt.y - lastCursor.y) * 0.0035f; }
            // the free camera: the mouse is captured (hidden, held in the middle of the window) and every move looks; Tab frees it,
            // a click in the window or Tab again takes it back. Never while capturing a test frame or when the window isn't in front.
            static bool captured = false, released = false, tabWas = false;
            const bool tab = Key(VK_TAB);
            if (tab && !tabWas) released = !released;
            tabWas = tab;
            const bool front = GetForegroundWindow() == g_window;
            if (released && front && Key(VK_LBUTTON)) { POINT c = pt; ScreenToClient(g_window, &c); RECT rc; GetClientRect(g_window, &rc); if (PtInRect(&rc, c)) released = false; }
            const bool want = game.WantsMouseLook() && front && !released && !capture;
            RECT rc; GetClientRect(g_window, &rc);
            POINT mid{(rc.left + rc.right) / 2, (rc.top + rc.bottom) / 2};
            ClientToScreen(g_window, &mid);
            if (want != captured) {
                captured = want;
                ShowCursor(want ? FALSE : TRUE);
                if (want) { POINT tl{rc.left, rc.top}, br{rc.right, rc.bottom}; ClientToScreen(g_window, &tl); ClientToScreen(g_window, &br); RECT clip{tl.x, tl.y, br.x, br.y}; ClipCursor(&clip); SetCursorPos(mid.x, mid.y); pt = mid; }
                else ClipCursor(nullptr);
            } else if (captured) {
                in.lookX = (pt.x - mid.x) * 0.0022f; in.lookY = (pt.y - mid.y) * 0.0022f;
                SetCursorPos(mid.x, mid.y); pt = mid;
            }
            lastCursor = pt;
        }
        { static bool mWas = false, fWas = false, eWas = false; const bool mk = Key('M'), fk = Key('F'), ek = Key(VK_RETURN);
          in.toggleMap = mk && !mWas; in.cycleFog = fk && !fWas; in.confirm = ek && !eWas; mWas = mk; fWas = fk; eWas = ek; }
        { // the map's mouse: wheel, left-drag, and a click when the button comes up without having dragged
            in.wheel = g_wheel; g_wheel = 0;
            static POINT prev{}; static bool down = false; static float travelled = 0;
            POINT pt{}; GetCursorPos(&pt);
            POINT cl = pt; ScreenToClient(g_window, &cl);
            in.cursorX = (float)cl.x; in.cursorY = (float)cl.y;
            const bool lb = Key(VK_LBUTTON);
            if (lb && down) { in.dragX = (float)(pt.x - prev.x); in.dragY = (float)(pt.y - prev.y); travelled += std::abs(in.dragX) + std::abs(in.dragY); }
            if (lb && !down) travelled = 0;
            if (!lb && down && travelled < 5) in.click = true;
            down = lb; prev = pt;
        }
        cWas = c; rWas = rk;
        XINPUT_STATE pad{};
        if (XInputGetState(0, &pad) == ERROR_SUCCESS) {
            const XINPUT_GAMEPAD& gp = pad.Gamepad;
            if (const float lx = Stick(gp.sThumbLX), ly = Stick(gp.sThumbLY); lx != 0 || ly != 0) { in.roll = lx; in.pitch = -ly; }
            if (gp.bLeftTrigger > 20 || gp.bRightTrigger > 20) in.yaw = (gp.bRightTrigger - gp.bLeftTrigger) / 255.0f;
            if (const float ry = Stick(gp.sThumbRY); ry != 0) in.throttleRate = ry;
            in.nextCamera |= (gp.wButtons & XINPUT_GAMEPAD_Y) && !(padWas & XINPUT_GAMEPAD_Y);
            in.reset |= (gp.wButtons & XINPUT_GAMEPAD_BACK) && !(padWas & XINPUT_GAMEPAD_BACK);
            auto press = [&](WORD b) { return (gp.wButtons & b) && !(padWas & b); };
            in.confirm |= press(XINPUT_GAMEPAD_A); in.back |= press(XINPUT_GAMEPAD_B); in.pause |= press(XINPUT_GAMEPAD_START);
            in.flapsUp |= press(XINPUT_GAMEPAD_LEFT_SHOULDER); in.flapsDown |= press(XINPUT_GAMEPAD_RIGHT_SHOULDER);
            in.toggleGear |= press(XINPUT_GAMEPAD_RIGHT_THUMB); in.toggleReverse |= press(XINPUT_GAMEPAD_LEFT_THUMB);
            in.menuUp |= press(XINPUT_GAMEPAD_DPAD_UP); in.menuDown |= press(XINPUT_GAMEPAD_DPAD_DOWN);
            in.menuLeft |= press(XINPUT_GAMEPAD_DPAD_LEFT); in.menuRight |= press(XINPUT_GAMEPAD_DPAD_RIGHT);
            if (gp.wButtons & XINPUT_GAMEPAD_X) in.brake = 1.0f;
            if (gp.wButtons & XINPUT_GAMEPAD_DPAD_UP) in.trimRate = -1.0f;
            if (gp.wButtons & XINPUT_GAMEPAD_DPAD_DOWN) in.trimRate = 1.0f;
            padWas = gp.wButtons;
        }

        // A capture (FLY_CAPTURE) takes no input: its window grabs focus, and keys or the mouse used meanwhile (in another
        // app) flew the free camera off its FLY_FREECAM_POSE, so the same settings captured a different place run to run.
        if (capture) in = fly::Input{};
        if (xr) { // the headset paces the loop (xrWaitFrame) and renders both eyes
            if (!xr->Frame(*r, game, in)) break;
            continue;
        }
        static const int captureFrame = [] { char f[16] = {}; return GetEnvironmentVariableA("FLY_CAPTURE_FRAME", f, sizeof f) > 0 ? atoi(f) : 120; }();
        const bool shot = capture && (int)frame == captureFrame;
        const bool rec = record && (int64_t)frame >= recordSkip;
        if (shot || rec) r->SetFrameCapture(true);
        const auto t0 = std::chrono::steady_clock::now();
        const bool more = game.Frame(*r, dt, in); // simulates and renders, up to the HUD
        const auto t1 = std::chrono::steady_clock::now();
        r->EndFrame();
        { // the CPU side of the frame, next to the renderer's GPU timings: wall time per frame, the game's own work (simulation,
          // streaming, culling, recording draws) and EndFrame (submit, present, waiting on the GPU), 120-frame means
            static double sumWall = 0, sumGame = 0, sumEnd = 0; static int n = 0;
            sumWall += wallDt; sumGame += std::chrono::duration<double>(t1 - t0).count();
            sumEnd += std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
            if (++n == 120) {
                tbd::Log("cpu: frame %.2f ms wall (%.0f fps), game %.2f ms, end+present %.2f ms (120-frame mean)", sumWall / n * 1e3, n / std::max(sumWall, 1e-6), sumGame / n * 1e3, sumEnd / n * 1e3);
                sumWall = sumGame = sumEnd = 0; n = 0;
            }
        }
        if (shot) {
            std::vector<uint8_t> px; int w = 0, h = 0;
            if (r->CapturedFrame(px, w, h)) WriteBmp(capturePath, px, w, h);
            break;
        }
        if (rec) {
            static std::vector<uint8_t> px; int w = 0, h = 0;
            if (r->CapturedFrame(px, w, h) && w > 0 && h > 0) {
                if (!recordPipe) { // even sizes for yuv420p
                    recordW = w & ~1; recordH = h & ~1;
                    char cmd[MAX_PATH + 512];
                    snprintf(cmd, sizeof cmd, "ffmpeg -y -loglevel error -f rawvideo -pix_fmt bgra -s %dx%d -r %d -i - -vf crop=%d:%d:0:0 "
                             "-c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p -movflags +faststart \"%s\"", w, h, recordFps, recordW, recordH, recordPath);
                    recordPipe = _popen(cmd, "wb");
                    tbd::Log("record: %dx%d at %d fps to %s (%s)", recordW, recordH, recordFps, recordPath, recordPipe ? "ffmpeg" : "FAILED to start ffmpeg");
                    if (recordPipe) recordW = w, recordH = h;
                }
                if (recordPipe && w == recordW && h == recordH) { fwrite(px.data(), 1, px.size(), recordPipe); ++recorded; }
            }
        }
        if (!more) break;
        { static bool fs = false; const bool want = game.WantsFullscreen() && !capture && !record; if (want != fs) { fs = want; SetFullscreen(fs); } }
        ++frame;
    }
    if (recordPipe) { _pclose(recordPipe); tbd::Log("record: %llu frames written to %s", (unsigned long long)recorded, recordPath); }
    return 0;
}
