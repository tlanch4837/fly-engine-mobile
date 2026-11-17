// Weather, seasons and night lighting as pure logic (P10, P11). No renderer, no engine types except Vec3: tests/weather_test.cpp
// runs it headless. city_weather.inl is the glue that feeds Sun and Lighting, weather.hlsli / weather.hlsl are the GPU half.
//
// Calendar: the HUD's (city_hud.h): 12 months of 30 days, a 360-day year, day 0 = 1 Jan. Winter Dec-Feb, spring Mar-May, summer
// Jun-Aug, autumn Sep-Nov.
//
// What the GPU gets (all packed into the spare .w of Lighting's vectors, so no layout change; see city_weather.inl):
//   sunDir.w      season phase + 1, 0 = unset (phase 0..4: 0 winter, 1 spring, 2 summer, 3 autumn, 4 = winter again), continuous: foliage and ground tint
//   sunColor.w    snow cover 0..1
//   skyColor.w    fraction of windows lit now 0..1
//   groundColor.w window glow strength 0..1 (night factor)
//   probePrev.w   wetness (already read by mesh.hlsl)
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "vecmath.h"

namespace tbd::weather {

enum class Kind : int { Clear, Cloudy, Rain, Fog, Storm, Snow, Count };
inline const char* KindName(Kind k) {
    static const char* const n[] = {"Clear", "Cloudy", "Rain", "Fog", "Thunderstorm", "Snow"};
    return n[(int)k];
}
inline const char* SeasonName(int s) { static const char* const n[] = {"Winter", "Spring", "Summer", "Autumn"}; return n[s & 3]; }

constexpr int kDaysPerMonth = 30, kMonths = 12, kDaysPerYear = kDaysPerMonth * kMonths;

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }
inline float Smooth(float a, float b, float x) { const float t = Clamp01((x - a) / (b - a)); return t * t * (3.0f - 2.0f * t); }
inline float Mixf(float a, float b, float t) { return a + (b - a) * t; }
inline Vec3 MixV(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }

// Integer hash to 0..1; the per-building and per-window randomness.
inline float Hash01(uint32_t x) {
    x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16;
    return (float)(x >> 8) * (1.0f / 16777216.0f);
}

// -------------------------------------------------------------------------------------------------------------- calendar
inline int MonthOfDay(int day0) { return ((day0 % kDaysPerYear + kDaysPerYear) % kDaysPerYear) / kDaysPerMonth; }
// 0 winter, 1 spring, 2 summer, 3 autumn; matches HudSeason.
inline int SeasonOfMonth(int m) { return (m == 11 || m <= 1) ? 0 : m <= 4 ? 1 : m <= 7 ? 2 : 3; }
// Continuous season phase 0..4 from a fractional day of the year: centres of the seasons are 0 = 15 Jan, 1 = 15 Apr, 2 = 15 Jul,
// 3 = 15 Oct, so a tint eases between seasons over ~45 days instead of jumping on the 1st.
inline float SeasonPhase(float dayOfYear) {
    float d = std::fmod(dayOfYear, (float)kDaysPerYear);
    if (d < 0) d += kDaysPerYear;
    float p = (d - 15.0f) / 90.0f; // 0 at 15 Jan
    if (p < 0) p += 4.0f;
    return p;
}

// -------------------------------------------------------------------------------------------------------------- climate
// Per-map climate range (rubric 1.4): yearly mean, half the winter-summer swing, half the day-night swing, wetness, and whether
// the map ever snows. A desert map has a small snow chance, a temperate one the defaults.
struct Climate {
    float meanC = 12.0f;
    float annualAmpC = 10.0f; // winter mean 2, summer mean 22
    float dailyAmpC = 5.0f;
    float rainy = 0.5f;       // 0 dry .. 1 wet: share of days with precipitation or cloud
    float latitudeDeg = 45.0f;// drives day length
    bool snows = true;
};

// Temperature at a moment. Coldest around mid January, warmest mid July; the day peaks near 15:00 and bottoms near 05:00.
inline float Temperature(const Climate& c, float dayOfYear, float hour, float cloud = 0.0f, float precip = 0.0f, bool snowing = false) {
    const float annual = -std::cos((dayOfYear - 15.0f) / (float)kDaysPerYear * 6.2831853f); // -1 mid Jan .. +1 mid Jul
    const float daily = std::sin((hour - 9.0f) / 24.0f * 6.2831853f);                           // peak 15:00, trough 03:00
    const float damp = 1.0f - 0.45f * cloud;                                                     // cloud flattens the day swing
    float t = c.meanC + c.annualAmpC * annual + c.dailyAmpC * damp * daily - 2.0f * cloud - 2.5f * precip;
    if (snowing) t = std::min(t, 0.5f);
    return t;
}

// Day length in hours: 12 at the equinox, swings with latitude. Returns sunrise and sunset clock hours centred on 12.
struct DayLength { float sunrise, sunset, hours; };
inline DayLength DaylightHours(const Climate& c, float dayOfYear) {
    const float decl = 23.44f * std::sin((dayOfYear - 80.0f) / (float)kDaysPerYear * 6.2831853f); // degrees, peaks at the June solstice
    const float lat = c.latitudeDeg * 0.0174533f, d = decl * 0.0174533f;
    const float cosH = std::clamp(-std::tan(lat) * std::tan(d), -0.98f, 0.98f); // polar day/night clamped: the game never goes fully dark/bright
    const float half = std::acos(cosH) * 12.0f / 3.14159265f;
    return {12.0f - half, 12.0f + half, 2.0f * half};
}
// Map the game clock hour onto the sun's own hour (sun.h: 6 sunrise, 12 noon, 18 sunset) so sunrise and sunset move with the season.
inline float SunHourFromClock(float clockHour, const DayLength& dl) {
    const float h = std::fmod(clockHour + 24.0f, 24.0f);
    if (h >= dl.sunrise && h <= dl.sunset) return 6.0f + (h - dl.sunrise) / std::max(dl.hours, 0.1f) * 12.0f;
    const float night = 24.0f - dl.hours;
    const float since = h > dl.sunset ? h - dl.sunset : h + 24.0f - dl.sunset; // hours since sunset
    return std::fmod(18.0f + since / std::max(night, 0.1f) * 12.0f, 24.0f);
}

// -------------------------------------------------------------------------------------------------------------- tints
// Foliage multiplier on the albedo by season phase (weather.hlsli SeasonFoliage mirrors it).
inline Vec3 FoliageTint(float phase) {
    static const Vec3 k[5] = {{0.62f, 0.62f, 0.60f}, {1.02f, 1.12f, 0.80f}, {0.95f, 0.98f, 0.82f}, {1.55f, 0.82f, 0.38f}, {0.62f, 0.62f, 0.60f}};
    phase = std::fmod(std::max(phase, 0.0f), 4.0f);
    const int i = (int)phase;
    return MixV(k[i], k[i + 1], Smooth(0.0f, 1.0f, phase - (float)i));
}
inline Vec3 GroundTint(float phase) {
    static const Vec3 k[5] = {{0.90f, 0.90f, 0.92f}, {0.96f, 1.06f, 0.88f}, {1.04f, 1.00f, 0.86f}, {1.10f, 0.92f, 0.72f}, {0.90f, 0.90f, 0.92f}};
    phase = std::fmod(std::max(phase, 0.0f), 4.0f);
    const int i = (int)phase;
    return MixV(k[i], k[i + 1], Smooth(0.0f, 1.0f, phase - (float)i));
}

// -------------------------------------------------------------------------------------------------------------- the weather machine
// Condition parameters at full strength; the live state eases toward them.
struct Params {
    float cloud = 0.0f;     // sky cover 0..1 (0.15 is the engine's clear default)
    float precip = 0.0f;    // 0..1 intensity of rain or snow
    float fog = 0.0f;       // 0..1 extra haze
    float dark = 0.0f;      // storm/overcast dimming of the sun, 0..1
    float wind = 0.2f;      // 0..1
};
inline Params ParamsOf(Kind k) {
    switch (k) {
        case Kind::Clear:  return {0.12f, 0.0f, 0.0f, 0.0f, 0.15f};
        case Kind::Cloudy: return {0.75f, 0.0f, 0.05f, 0.20f, 0.30f};
        case Kind::Rain:   return {0.90f, 0.65f, 0.12f, 0.45f, 0.45f};
        case Kind::Fog:    return {0.55f, 0.0f, 0.85f, 0.25f, 0.05f};
        case Kind::Storm:  return {1.00f, 1.00f, 0.20f, 0.75f, 0.90f}; // lightning-free: only the darkening and heavier rain
        case Kind::Snow:   return {0.85f, 0.55f, 0.25f, 0.35f, 0.25f};
        default: return {};
    }
}

struct State {
    Kind kind = Kind::Clear, next = Kind::Clear;
    float blend = 1.0f;       // 0 just switched .. 1 arrived at `kind` (eases from the previous look)
    Params from{}, now{};     // `from` the look when the change began; `now` the eased value
    float hoursLeft = 6.0f;   // game hours until the next change
    float wetness = 0.0f;     // 0 dry .. 1 soaked: rises with rain, falls as it dries
    float snowCover = 0.0f;   // 0..1 on the ground: builds under snowfall below freezing, melts above
    float tempC = 12.0f;
    uint32_t rng = 1234567u;
    Kind forced = Kind::Count; // Count = automatic
};

inline uint32_t NextRand(uint32_t& s) { s = s * 1664525u + 1013904223u; return s; }
inline float Rand01(uint32_t& s) { return (float)(NextRand(s) >> 8) * (1.0f / 16777216.0f); }

// Next condition, drawn by season and climate. Snow only when cold enough; storms only when warm; fog likes cold mornings.
inline Kind PickNext(State& st, const Climate& c, int season, float tempC) {
    float w[(int)Kind::Count] = {};
    const float wet = 0.4f + 1.2f * c.rainy;
    const bool cold = tempC < 1.5f && c.snows;
    w[(int)Kind::Clear] = (season == 2 ? 3.0f : 2.0f) * (1.6f - c.rainy);
    w[(int)Kind::Cloudy] = 2.0f;
    w[(int)Kind::Rain] = cold ? 0.0f : 1.2f * wet;
    w[(int)Kind::Fog] = (season == 0 || season == 3) ? 0.7f : 0.25f;
    w[(int)Kind::Storm] = (tempC > 10.0f && season >= 1 && season <= 2) ? 0.35f * wet : 0.0f;
    w[(int)Kind::Snow] = cold ? 1.6f * wet : 0.0f;
    float total = 0; for (float x : w) total += x;
    float r = Rand01(st.rng) * total;
    for (int i = 0; i < (int)Kind::Count; ++i) { if ((r -= w[i]) <= 0.0f) return (Kind)i; }
    return Kind::Clear;
}

// Advance by `dtGameHours` at game moment (dayOfYear fractional, hour of day). Cheap: a handful of flops, no allocation.
inline void Step(State& st, const Climate& c, float dayOfYear, float hour, float dtGameHours) {
    const int season = SeasonOfMonth(MonthOfDay((int)dayOfYear));
    if (st.forced != Kind::Count && st.forced != st.next) { st.from = st.now; st.kind = st.next = st.forced; st.blend = 0.0f; }
    st.hoursLeft -= dtGameHours;
    if (st.forced == Kind::Count && st.hoursLeft <= 0.0f) {
        st.from = st.now;
        st.kind = st.next;
        st.next = PickNext(st, c, season, st.tempC);
        st.hoursLeft = 4.0f + Rand01(st.rng) * 14.0f; // a condition lasts 4..18 game hours
        st.blend = 0.0f;
    }
    st.blend = std::min(1.0f, st.blend + dtGameHours / 1.5f); // 1.5 game hours to ease between looks
    const Params to = ParamsOf(st.kind);
    const float e = Smooth(0.0f, 1.0f, st.blend);
    st.now = {Mixf(st.from.cloud, to.cloud, e), Mixf(st.from.precip, to.precip, e), Mixf(st.from.fog, to.fog, e), Mixf(st.from.dark, to.dark, e), Mixf(st.from.wind, to.wind, e)};
    st.tempC = Temperature(c, dayOfYear, hour, st.now.cloud, st.now.precip, st.kind == Kind::Snow);
    const bool snowing = st.kind == Kind::Snow && st.tempC < 1.5f;
    const bool raining = st.now.precip > 0.05f && !snowing;
    // wetness: soaks within about an hour of rain, dries over about eight; snow melting wets too
    const float melt = st.tempC > 1.0f ? Clamp01(st.snowCover) : 0.0f;
    st.wetness = Clamp01(st.wetness + (raining ? st.now.precip * 1.0f : 0.0f) * dtGameHours + melt * 0.3f * dtGameHours - (raining ? 0.0f : 0.125f * dtGameHours * (1.0f + st.now.wind)));
    // snow cover: ~0.4 per hour of full snowfall; melts about 0.05/h per degree above 0
    if (snowing) st.snowCover = Clamp01(st.snowCover + st.now.precip * 0.4f * dtGameHours);
    else if (st.tempC > 0.5f) st.snowCover = Clamp01(st.snowCover - (0.04f + 0.03f * st.tempC) * dtGameHours);
}

// Numbers the renderer needs this frame.
struct Frame {
    float phase = 2.0f;         // season phase for foliage/ground tint
    float snowCover = 0.0f;
    float wetness = 0.0f;
    float windowsLit = 0.0f;    // fraction of windows on
    float windowGlow = 0.0f;    // night factor driving emissive strength
    float streetLights = 0.0f;  // 0..1 lamp pool strength
    float headlights = 0.0f;
    float sunScale = 1.0f;      // multiply the direct sun
    float skyScale = 1.0f;      // multiply ambient sky (overcast flattens, storm darkens)
    float skyDesat = 0.0f;      // 0..1 pull of sky colour toward grey
    float fogDensity = 0.0f;    // multiplier on the haze (1 = the engine's default, up to ~4)
    Vec3 fogTint = {1, 1, 1};
    float precip = 0.0f;        // overlay intensity 0..1
    bool snowFall = false;
    float wind = 0.2f;
    float tempC = 12.0f;
};

// Night factor from the sun's height (y of the direction to the sun): 0 in day, 1 in full night, a dusk ramp between.
inline float NightFactor(float sunUp) { return 1.0f - Smooth(-0.14f, 0.10f, sunUp); }

// Share of windows lit by hour of day (weekday-ish): evening peak, small hours dark, a bump around breakfast.
inline float WindowLitFraction(float hour) {
    float h = std::fmod(hour + 24.0f, 24.0f);
    if (h < 5.0f) h += 24.0f; // the small hours continue the evening
    float f = 0.06f;
    f = std::max(f, 0.88f * Smooth(16.5f, 19.5f, h) * (1.0f - Smooth(22.5f, 27.0f, h)));
    f = std::max(f, 0.45f * Smooth(5.0f, 6.5f, h) * (1.0f - Smooth(7.0f, 8.5f, h)));
    return Clamp01(f);
}
// A window is lit if its own random number is below the fraction scaled by the building's habit (0.5 .. 1.3: some buildings stay
// dark, some blaze). One fixed hash per window and a threshold that only moves with the hour: a window turns on or off once
// as the fraction crosses it and never flickers.
inline bool WindowLit(uint32_t buildingId, uint32_t windowId, float fraction) {
    const float habit = 0.5f + 0.8f * Hash01(buildingId * 2654435761u + 101u);
    const float w = Hash01((buildingId * 40503u) ^ (windowId * 9973u + 17u));
    return w < fraction * habit;
}

inline Frame MakeFrame(const State& st, const Climate& c, float dayOfYear, float clockHour, float sunUp) {
    (void)c;
    Frame f;
    f.phase = SeasonPhase(dayOfYear);
    f.snowCover = st.snowCover;
    f.wetness = st.wetness;
    const float night = NightFactor(sunUp);
    const float gloom = st.now.dark;
    f.windowGlow = Clamp01(night + gloom * 0.35f * (1.0f - night));
    f.windowsLit = Clamp01(WindowLitFraction(clockHour) * night + gloom * 0.25f * (1.0f - night));
    f.streetLights = Clamp01(night * 1.0f + gloom * 0.5f * (1.0f - night));
    f.headlights = Smooth(0.25f, 0.8f, night + gloom * 0.4f);
    f.sunScale = 1.0f - 0.85f * st.now.dark;
    f.skyScale = 1.0f - 0.45f * st.now.dark;
    f.skyDesat = Clamp01(st.now.cloud * 0.65f + st.now.fog * 0.4f);
    f.fogDensity = 1.0f + 3.0f * st.now.fog + 0.8f * st.now.precip;
    f.fogTint = MixV({1.0f, 1.0f, 1.0f}, {0.78f, 0.82f, 0.88f}, Clamp01(st.now.fog + st.now.cloud * 0.4f));
    f.precip = st.now.precip;
    f.snowFall = st.kind == Kind::Snow && st.tempC < 1.5f;
    f.wind = st.now.wind;
    f.tempC = st.tempC;
    return f;
}

// -------------------------------------------------------------------------------------------------------------- per-preset cost
// Tier = LookTier(): 0 Low, 1 Med, 2 High, 3 Ultra.
struct Cost {
    int precipLayers;      // parallax layers of streaks/flakes in the one fullscreen pass (fragment cost scales with it)
    int drawCallsPrecip;   // fullscreen draws while precipitating; 0 when clear
    int drawCallsIdle;     // draws added when no precipitation (always 0)
    bool wetGloss;         // wet-road gloss (a shader branch on a constant: no extra draws on any tier)
    bool streetPools;      // street light pools as emissive on props plus ground decals (no extra draws; Low fakes with a baked glow)
    bool headlights;
    bool fog;              // the ground-hugging fog term in the composite
    float renderScale;     // the precipitation pass runs at this fraction of the screen
};
inline Cost CostOf(int tier) {
    switch (std::clamp(tier, 0, 3)) {
        case 0:  return {1, 1, 0, true, false, false, false, 0.5f};
        case 1:  return {2, 1, 0, true, true, false, true, 0.5f};
        case 2:  return {3, 1, 0, true, true, true, true, 0.75f};
        default: return {4, 1, 0, true, true, true, true, 1.0f};
    }
}
inline int DrawCalls(int tier, float precip) { const Cost c = CostOf(tier); return precip > 0.02f ? c.drawCallsPrecip : c.drawCallsIdle; }

// -------------------------------------------------------------------------------------------------------------- HUD
inline Climate& CurrentClimate() { static Climate c; return c; }
inline State& CurrentState() { static State s; return s; }

// The status bar's temperature: the live simulation value when one is running, else the pure climate curve for that date and time.
inline float HudTemperature(float dayOfYear, float hour) {
    const State& s = CurrentState();
    return s.blend < 0.0f ? 0.0f : Temperature(CurrentClimate(), dayOfYear, hour, s.now.cloud, s.now.precip, s.kind == Kind::Snow);
}
// "Winter  Snow" for the season cell.
inline const char* HudLabel(int month) {
    static char buf[48];
    std::snprintf(buf, sizeof(buf), "%s  %s", SeasonName(SeasonOfMonth(month)), KindName(CurrentState().kind));
    return buf;
}

} // namespace tbd::weather
