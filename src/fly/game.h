// The portable game: everything except the OS shell. No Windows, UIKit or Android headers here or in game.cpp.
// Each platform shell (src/platform/win/main_win.cpp, src/platform/ios/main_ios.mm, later Android/Quest) makes the renderer,
// turns its input devices into Input, and calls Start / Resize / Frame.
#pragma once
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "../render/renderer.h"

namespace fly {

// Input as analog axes, whatever produced it: keyboard (eased), touch sticks, gamepads, HOTAS, VR controllers.
struct Input {
    float pitch = 0, roll = 0, yaw = 0; // -1..1; pitch +1 = stick back (nose up), roll +1 = right, yaw +1 = right rudder
    float throttleRate = 0;             // -1..1: keyboard-style, throttle moves at this rate
    float throttleAbs = -1;             // 0..1 sets the throttle directly (touch slider, HOTAS lever); < 0 = not used
    bool reset = false;                 // edge: pressed this frame
    bool nextCamera = false;            // edge
    float lookX = 0, lookY = 0;         // head look this frame, radians (mouse drag); cockpit view only
    float moveUp = 0;                   // -1..1 head down / up (cockpit viewer: Q / E)
    bool fast = false;                  // move faster (Shift)
    bool toggleLights = false;          // edge: cockpit lights off / panel / panel + dome + flood (L)
    bool toggleNight = false;           // edge: day <-> night (N)
    bool toggleGear = false;            // edge: landing gear lever up / down (G)
    bool toggleFreeCam = false;         // edge: the free camera on / off (V)
    // the world map (M) and the mouse
    bool toggleMap = false;             // edge (M)
    bool cycleFog = false;              // edge (F): clear / haze / fog
    bool confirm = false;               // edge (Enter)
    float wheel = 0;                    // mouse wheel notches this frame, + = away from you (zoom in)
    float dragX = 0, dragY = 0;         // pixels moved with the left button held, this frame
    bool click = false;                 // edge: left button released without dragging
    float cursorX = 0, cursorY = 0;     // pixels in the window
    // the game build's flight systems and menus (gameplay.h)
    bool flapsUp = false, flapsDown = false;        // edges (Z / X; gamepad LB / RB)
    float brake = 0;                                // wheel brakes held, 0..1 (Space; gamepad X)
    bool toggleParkBrake = false;                   // edge (B)
    bool toggleReverse = false;                     // edge (T; gamepad left stick click)
    bool toggleSpeedbrake = false;                  // edge (Q)
    float trimRate = 0;                             // -1..1 pitch trim (PgDn / PgUp; d-pad)
    bool pause = false;                             // edge (Esc; gamepad Start)
    bool back = false;                              // edge (Backspace; gamepad B)
    bool menuUp = false, menuDown = false, menuLeft = false, menuRight = false; // edges (arrows; d-pad)
    bool toggleFullscreen = false;                  // edge (F11, Alt+Enter)
};

// One eye of a headset (OpenXR): the eye's pose relative to the seat / tracking origin, and its field of view as tangents of
// the four half-angles (left and down negative). The game puts the seat in the cockpit, or behind the aircraft for the chase view.
struct EyeView {
    tbd::Vec3 position{0, 0, 0}; // metres, in tracking space (x right, y up, -z forward)
    tbd::Quat orientation{0, 0, 0, 1};
    float tanLeft = -1, tanRight = 1, tanUp = 1, tanDown = -1;
};

struct Settings {
    std::filesystem::path dataDir; // folder holding fixtures/ (Windows: <exe>\..\data, iOS: the app bundle / Documents)
    int startCamera = 0;           // 0 chase, 1 cockpit, 2 orbit
    bool hasStart = false;
    double startX = 0, startY = 0, startZ = 0;
    float hudScale = 2.0f;         // pixel font cell size; shells raise it on high-DPI phones
    bool touchHints = false;       // show touch controls help instead of keyboard help
    bool vr = false;               // headset: no flat HUD, cockpit view first
    bool hasStartGeo = false;      // FLY_GEO=lat,lon: start somewhere on Earth instead of the home field
    double startLat = 0, startLon = 0;
};

class Game {
public:
    Game();
    ~Game();
    bool Start(tbd::Renderer& r, const Settings& settings);
    void Resize(int width, int height);
    // Simulates dt seconds and records one frame: BeginFrame, scene, ResolveScene, HUD. The shell then calls EndFrame
    // (so it can capture or add its own overlays first). Returns false when the game wants to quit.
    bool Frame(tbd::Renderer& r, float dt, const Input& in);
    // The same in two halves, for stereo: simulate once per headset frame, then Render once per eye (each followed by EndFrame).
    void Simulate(float dt, const Input& in);
    void Render(tbd::Renderer& r, const EyeView* eye); // eye == nullptr: the flat camera
    // For shells that draw their own touch controls: where the throttle currently is.
    float Throttle() const;
    // The free camera is on and flying: the shell may capture the mouse and send its motion as lookX / lookY every frame.
    bool WantsMouseLook() const;
    // The game build: Esc opens the pause menu instead of quitting, and the window follows the Display setting.
    bool HandlesEscape() const;
    bool WantsFullscreen() const;

    struct Impl; // public so game.cpp's free helpers can take it; its contents stay private to game.cpp

private:
    std::unique_ptr<Impl> m_;
};

} // namespace fly
