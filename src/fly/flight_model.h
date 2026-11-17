// The flight model. Two models share one Aircraft:
//  - the first, simple model (StepSimple): a light trainer, controls as torques scaled by dynamic pressure, one ground plane.
//    Every test build keeps it (captures and sequences were tuned against it).
//  - the game model (StepGame, Aircraft::realistic, on in the game build): an A320-class rigid body. Six-degree-of-freedom
//    dynamics with a diagonal inertia tensor about the centre of gravity, a lift curve with slats / flaps, stall and post-stall,
//    drag from flaps / gear / spoilers, ground effect, two turbofans with spool lag and reversers, three sprung and damped
//    wheels (nose-wheel steering, brakes, tyre friction), body contact points (tail, pods, wingtips, belly, nose), wind and
//    turbulence, and the Airbus normal law (fly-by-wire: the stick asks for flight-path rate and roll rate; neutral holds them;
//    high-angle-of-attack, pitch, bank and load-factor protections), or direct law.
// Body axes: -z forward, +y up, +x right. Position is double precision; the rest is float.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>

#include "../vecmath.h"
#include "world.h"

namespace fly {

using tbd::Quat;
using tbd::Vec3;

struct Controls {
    float pitch = 0, roll = 0, yaw = 0, throttle = 0.6f; // -1..1, throttle 0..1
    // the game model's
    int flaps = 0;            // flap lever: 0 UP, 1 CONF 1+F, 2 CONF 2, 3 CONF 3, 4 FULL
    float brake = 0;          // toe brakes 0..1 (both mains)
    bool parkBrake = false;
    bool reverse = false;     // thrust reversers selected (deploy only with weight on the wheels)
    bool speedbrake = false;  // speed brake lever out (in flight: half spoilers)
    bool gearDown = true;     // gear lever (without the cockpit's own gear sequence)
    float trim = 0;           // pitch trim -1..1 (direct law; the normal law trims itself)
    bool fbw = true;          // normal law (fly-by-wire) or direct law
};

// The game build turns the game model on for every Aircraft made from then on (the default member below).
inline bool gAircraftRealistic = false;

struct Aircraft {
    // ---- the simple model's airframe
    float mass = 1100, wingArea = 16.2f, maxThrust = 3200, cl0 = 0.3f, clAlpha = 5.0f, cd0 = 0.03f, k = 0.05f;
    float stallAlpha = 0.28f; // radians
    // ---- state (both models)
    WorldPosition pos{0, 450, 3500};
    Vec3 vel{0, 0, -55};          // m/s; heading north (-z)
    Quat rot{0, 0, 0, 1};         // body: -z forward, +y up, +x right
    Vec3 omega{0, 0, 0};          // body rates, rad/s (x pitch, y yaw, z roll)
    bool crashed = false;

    // ---- the game model
    bool realistic = gAircraftRealistic;
    Vec3 wind{0, 0, 0};           // steady wind, m/s (the air's velocity), set by the game
    float turbulence = 0;         // gust strength, m/s (1 sigma)
    float toughness = 1;          // crash thresholds x this (the game's Forgiving setting: 2)
    float gearExt = -1;           // gear extension 0 up .. 1 down from the cockpit's gear sequence; < 0: the model moves it
    // obstacles above the terrain (buildings): the roof height at (x, z), or a very low number. Optional.
    std::function<double(double, double)> obstacle;
    // what the game reads
    float n1[2] = {0.2f, 0.2f};   // engine fan speed 0..1 (0.2 idle)
    float flapPos = 0;            // flap / slat extension 0..1 (lever / 4, travelling)
    float gearPos = 1;            // 0 up .. 1 down and locked
    float spoilers = 0;           // 0..1
    bool reverserOut = false;
    float alpha = 0, beta = 0;    // angle of attack, sideslip (rad)
    float nz = 1;                 // load factor along the body's up (aerodynamic + thrust)
    float ias = 0;                // indicated (equivalent) airspeed, m/s
    float elevator = 0, aileron = 0, rudder = 0; // what the surfaces are doing, -1..1 (after the flight control law)
    float steer = 0;              // nose-wheel angle, rad
    bool onGround = false;        // any wheel loaded
    bool mainsOnGround = false;
    float wheelLoad[3] = {0, 0, 0}; // N: nose, left main, right main
    float airborneTime = 0, groundTime = 0;
    bool stallWarning = false, overspeed = false;
    float lawBlend = 0;           // 0 ground (direct) .. 1 flight (normal law)
    // events since the game last cleared them
    struct Events {
        bool touchdown = false; float tdSink = 0, tdIas = 0, tdBank = 0, tdPitch = 0; double tdX = 0, tdZ = 0;
        bool tailStrike = false;
        bool crash = false; const char* why = "";
        float impact = 0;           // largest contact speed this step (m/s), for the thump sound
    } ev;
    const char* crashWhy = "";

    Vec3 Forward() const { return tbd::Rotate(rot, {0, 0, -1}); }
    Vec3 Up() const { return tbd::Rotate(rot, {0, 1, 0}); }
    Vec3 Right() const { return tbd::Rotate(rot, {1, 0, 0}); }
    float Airspeed() const { return std::sqrt(tbd::Dot(vel, vel)); }

    static float AirDensity(double alt) { return 1.225f * (float)std::exp(-alt / 8500.0); }

    void Step(const Controls& c, float dt, const HeightField& ground) {
        if (realistic) StepGame(c, dt, ground);
        else StepSimple(c, dt, ground);
    }

    // ================================================================================================ the simple model
    void StepSimple(const Controls& c, float dt, const HeightField& ground) {
        if (crashed) return;
        const float rho = AirDensity(pos.y), V = std::max(0.1f, Airspeed()), q = 0.5f * rho * V * V;
        const Vec3 f = Forward(), u = Up(), rgt = Right();
        const Vec3 vdir = vel * (1.0f / V);
        // angle of attack: the airflow's angle below the nose in the body's vertical plane
        const float alpha = std::atan2(-tbd::Dot(vdir, u), tbd::Dot(vdir, f));
        float cl = cl0 + clAlpha * alpha;
        if (std::abs(alpha) > stallAlpha) cl *= std::max(0.2f, 1.0f - (std::abs(alpha) - stallAlpha) * 4.0f); // stall
        const float cd = cd0 + k * cl * cl;
        // lift is perpendicular to the airflow, in the plane of the airflow and the wing's up
        Vec3 liftDir = tbd::Cross(tbd::Cross(vdir, u), vdir);
        const float ll = std::sqrt(tbd::Dot(liftDir, liftDir));
        liftDir = ll > 1e-4f ? liftDir * (1.0f / ll) : u;
        const float beta = tbd::Dot(vdir, rgt); // sideslip: sideways force pushes the body back into the wind
        Vec3 force = liftDir * (q * wingArea * cl) - vdir * (q * wingArea * cd) + f * (maxThrust * c.throttle * rho / 1.225f) -
                     rgt * (q * wingArea * 0.6f * beta) + Vec3{0, -9.81f * mass, 0};
        const Vec3 acc = force * (1.0f / mass);
        vel = vel + acc * dt;
        pos.x += vel.x * dt; pos.y += vel.y * dt; pos.z += vel.z * dt;
        // rotation: control authority grows with dynamic pressure; damping keeps it flyable; weathervane on slip and alpha
        const float auth = std::min(q / 1800.0f, 2.0f);
        const Vec3 torque{c.pitch * 1.6f * auth - alpha * 2.0f * auth - omega.x * 2.5f,
                          -c.yaw * 0.7f * auth - beta * 1.5f * auth - omega.y * 2.0f,
                          -c.roll * 2.8f * auth - omega.z * 3.0f};
        omega = omega + torque * dt;
        const float w = std::sqrt(tbd::Dot(omega, omega));
        if (w > 1e-6f) rot = tbd::Normalize(rot * tbd::QuatAxisAngle(omega * (1.0f / w), w * dt));
        // ground
        const double g = std::max(0.0, ground.Height(pos.x, pos.z));
        if (pos.y < g + 1.5) {
            const float sink = -vel.y;
            if (sink > 4.0f || V > 75.0f || tbd::Dot(u, {0, 1, 0}) < 0.8f) crashed = true;
            pos.y = g + 1.5;
            vel.y = std::max(0.0f, vel.y);
            vel = vel * (1.0f - std::min(1.0f, 0.4f * dt)); // rolling friction
        }
    }

    // ================================================================================================ the game model (A320-200)
    // Airframe: wing 122.6 m^2, span 34.1 m, mean chord 4.29 m; 62 t; two 120 kN turbofans. Gear from the cockpit model
    // (ground_ops.h measured it): nose at z -13.7, mains at z -1.08, track 7.6 m, the body origin 3.91 m above the ground at rest.
    static constexpr float kMass = 62000, kS = 122.6f, kB = 34.1f, kC = 4.29f, kThrust = 120000;
    static constexpr float kIpitch = 3.0e6f, kIyaw = 4.0e6f, kIroll = 1.6e6f;
    static constexpr float kCgZ = -3.0f;              // the centre of gravity, body z (ahead of the mains: the nose stays down)
    static constexpr float kGearH = 3.91f;            // body origin above the ground on the gear, static
    static constexpr float kStroke0 = 0.20f;          // static strut compression
    static constexpr float kStrokeMax = 0.55f;
    static constexpr float kVmo = 180.0f;             // m/s indicated (350 kt)
    static Vec3 CgB() { return {0, 0, kCgZ}; }

    struct Wheel { Vec3 p; float k, c, mu; bool braked, steered; };
    static const Wheel* Wheels() {
        static const Wheel w[3] = {
            {{0.0f, -kGearH - kStroke0, -13.7f}, 0.15f * kMass * 9.81f / kStroke0, 2 * 0.6f * std::sqrt(0.15f * kMass * 9.81f / kStroke0 * 0.15f * kMass), 0.6f, false, true},
            {{-3.8f, -kGearH - kStroke0, -1.08f}, 0.425f * kMass * 9.81f / kStroke0, 2 * 0.6f * std::sqrt(0.425f * kMass * 9.81f / kStroke0 * 0.425f * kMass), 0.8f, true, false},
            {{3.8f, -kGearH - kStroke0, -1.08f}, 0.425f * kMass * 9.81f / kStroke0, 2 * 0.6f * std::sqrt(0.425f * kMass * 9.81f / kStroke0 * 0.425f * kMass), 0.8f, true, false}};
        return w;
    }
    // Points of the airframe that must not touch the ground: what happens when one does.
    enum Hit { kTail, kPodL, kPodR, kTipL, kTipR, kBellyF, kBellyA, kNose, kHits };
    static Vec3 HitPoint(int k) {
        static const Vec3 p[kHits] = {{0, -0.6f, 15.5f}, {-5.75f, -3.35f, -4.0f}, {5.75f, -3.35f, -4.0f}, {-17.0f, -0.9f, 2.5f},
                                      {17.0f, -0.9f, 2.5f}, {0, -2.95f, -9.0f}, {0, -2.95f, 5.0f}, {0, -1.2f, -19.5f}};
        return p[k];
    }

    // the private state of the game model
    float stroke_[3] = {0, 0, 0};
    bool wheelWas_[3] = {false, false, false};
    float gammaRef_ = 0, phiHold_ = 0, pitchI_ = 0;
    Vec3 gust_{0, 0, 0};
    uint32_t rng_ = 12345u;
    float NoiseN() { // ~ unit normal (sum of uniforms)
        float s = 0;
        for (int i = 0; i < 4; ++i) { rng_ = rng_ * 1664525u + 1013904223u; s += (float)(rng_ >> 8) / 16777216.0f; }
        return (s - 2.0f) * 1.732f;
    }

    // Lift coefficient: linear to the stall, a drop past it, flat plate beyond.
    static float LiftCoef(float a, float flap, float& alphaStall) {
        static const float dcl[5] = {0.0f, 0.35f, 0.60f, 0.85f, 1.10f};
        const float fi = std::clamp(flap * 4.0f, 0.0f, 4.0f);
        const int i0 = std::min(3, (int)fi);
        const float dCl = dcl[i0] + (dcl[i0 + 1] - dcl[i0]) * (fi - i0);
        alphaStall = 0.24f + 0.05f * std::min(1.0f, fi); // slats add ~3 degrees
        const float cl0 = 0.25f, cla = 5.5f, negStall = -0.20f;
        const float flat = 1.15f * std::sin(2.0f * a);
        if (a >= negStall && a <= alphaStall) return cl0 + dCl + cla * a;
        const bool pos = a > 0;
        const float as = pos ? alphaStall : negStall, past = std::abs(a - as);
        const float clmax = cl0 + dCl + cla * as;
        const float dip = clmax * (1.0f - 0.35f * std::clamp(past / 0.1f, 0.0f, 1.0f));
        const float t = std::clamp((past - 0.1f) / 0.4f, 0.0f, 1.0f), s = t * t * (3 - 2 * t);
        return dip + (flat - dip) * s;
    }

    float Bank() const { const Vec3 r = Right(), u = Up(); return std::atan2(-r.y, u.y); }  // right wing down +
    float PitchAngle() const { return std::asin(std::clamp(Forward().y, -1.0f, 1.0f)); }

    void StepGame(const Controls& c, float dt, const HeightField& ground) {
        ev.impact = 0;
        if (crashed) return;
        const float g0 = 9.81f;
        // ---- actuators
        auto toward = [&](float& v, float target, float rate) { v += std::clamp(target - v, -rate * dt, rate * dt); };
        toward(flapPos, std::clamp(c.flaps, 0, 4) / 4.0f, 0.12f);   // about 8 s from UP to FULL
        if (gearExt >= 0) gearPos = gearExt;
        else if (!(mainsOnGround && !c.gearDown)) toward(gearPos, c.gearDown ? 1.0f : 0.0f, 1.0f / 9.0f);
        reverserOut = c.reverse && mainsOnGround;
        const float idle = 0.2f;
        const float thr = reverserOut ? std::min(c.throttle, 0.75f) : c.throttle;
        for (float& n : n1) {
            const float target = idle + (1.0f - idle) * std::clamp(thr, 0.0f, 1.0f);
            const float tau = target > n ? (n < 0.5f ? 3.5f : 1.8f) : 1.5f; // slow off idle
            n += (target - n) * (1.0f - std::exp(-dt / tau));
        }
        // ---- the air
        const double gh = ground.Height(pos.x, pos.z);
        const double gy = std::max(0.0, gh);
        const float agl = (float)(pos.y - gy - kGearH);
        const float rho = AirDensity(pos.y);
        if (turbulence > 0) { // first-order gusts (time constant ~2 s), quieter near the ground
            const float tau = 2.0f, s = turbulence * std::clamp(0.35f + agl / 300.0f, 0.35f, 1.0f) * std::sqrt(2.0f / tau);
            const float sq = std::sqrt(dt);
            gust_ = gust_ + (gust_ * (-1.0f / tau)) * dt + Vec3{NoiseN() * s, NoiseN() * s * 0.6f, NoiseN() * s} * sq;
        } else gust_ = {};
        const float windScale = std::clamp(0.55f + 0.45f * agl / 200.0f, 0.55f, 1.0f); // the surface layer
        const Vec3 air = wind * windScale + gust_;
        const Vec3 va = vel - air;
        const Quat inv = tbd::Conjugate(rot);
        const Vec3 vb = tbd::Rotate(inv, va);
        const float V = std::max(0.5f, std::sqrt(tbd::Dot(vb, vb)));
        const float Vd = std::max(V, 12.0f); // for the rate derivatives
        alpha = std::atan2(-vb.y, -vb.z);
        beta = std::asin(std::clamp(vb.x / V, -1.0f, 1.0f));
        const float q = 0.5f * rho * V * V;
        ias = V * std::sqrt(rho / 1.225f);
        const float P = -omega.z, Q = omega.x, R = -omega.y;
        const float ph = P * kB / (2 * Vd), qh = Q * kC / (2 * Vd), rh = R * kB / (2 * Vd);
        // ---- spoilers: ground spoilers on touchdown with the levers at idle, half in flight with the speed brake
        float spTarget = c.speedbrake ? 0.5f : 0.0f;
        if (mainsOnGround && c.throttle < 0.08f && ias > 15.0f && groundTime > 0.15f) spTarget = 1.0f;
        toward(spoilers, spTarget, 1.6f);
        // ---- aero coefficients (aerodynamic sign convention: L roll right wing down +, M pitch nose up +, N yaw nose right +)
        float alphaStall = 0.24f;
        const float clw = LiftCoef(alpha, flapPos, alphaStall);
        const float hb = std::max(0.0f, agl + kGearH - 2.4f) / kB; // the wing above the ground, in spans
        const float geLift = 1.0f + 0.10f * std::exp(-4.0f * hb), geInd = (16 * hb) * (16 * hb) / (1 + (16 * hb) * (16 * hb));
        float CL = clw * geLift * (1.0f - 0.55f * spoilers);
        static const float fcd[5] = {0.0f, 0.010f, 0.022f, 0.036f, 0.065f};
        const float fi = std::clamp(flapPos * 4.0f, 0.0f, 4.0f);
        const int f0 = std::min(3, (int)fi);
        const float mach = V / 340.0f;
        float CD = 0.021f + 0.042f * CL * CL * std::max(0.35f, geInd) + fcd[f0] + (fcd[f0 + 1] - fcd[f0]) * (fi - f0) + 0.018f * gearPos + 0.07f * spoilers;
        if (mach > 0.76f) CD += 18.0f * (mach - 0.76f) * (mach - 0.76f);
        if (alpha > alphaStall || alpha < -0.2f) { const float s = std::sin(alpha); CD += 1.5f * s * s; }
        const float aEff = std::clamp(alpha, -0.6f, 0.6f);
        // ---- the flight control law -> surfaces
        float de = c.pitch + c.trim * 0.45f, da = c.roll, dr = c.yaw;
        const float phi = Bank(), theta = PitchAngle();
        // the law flies when airborne a moment; on the ground (and at the rotation) the stick moves the surfaces directly
        toward(lawBlend, c.fbw && !onGround && airborneTime > 0.4f ? 1.0f : 0.0f, onGround ? 2.0f : 0.8f);
        const float Vh = std::max(1.0f, std::sqrt(vel.x * vel.x + vel.z * vel.z));
        const float gamma = std::atan2(vel.y, Vh);
        const float qS = std::max(q, 150.0f) * kS;
        const float CmOther = 0.045f - 2.2f * aEff - 28.0f * qh - 0.06f * flapPos;
        if (lawBlend > 0.0f) {
            // pitch: the stick asks for flight-path rate; neutral holds the path (it trims itself)
            const float cphi = std::cos(std::clamp(phi, -1.2f, 1.2f));
            float gdot = c.pitch * 0.11f;
            gammaRef_ += gdot * dt;
            gammaRef_ = std::clamp(gammaRef_, gamma - 0.08f, gamma + 0.08f);
            const float qTurn = g0 / Vd * (1.0f / cphi - cphi);
            float qDes = gdot + 1.2f * (gammaRef_ - gamma) + qTurn;
            // protections: angle of attack (alpha prot / max), pitch attitude, load factor
            const float aProt = alphaStall - 0.045f, aMax = alphaStall - 0.012f;
            if (alpha > aProt) qDes = std::min(qDes, c.pitch > 0 ? (aMax - alpha) * 1.5f : (aProt - alpha) * 1.5f);
            const float thMax = flapPos > 0.6f ? 0.44f : 0.52f, thMin = -0.26f;
            qDes = std::min(qDes, (thMax - theta) * 0.8f);
            qDes = std::max(qDes, (thMin - theta) * 0.8f);
            qDes = std::clamp(qDes, -1.2f * g0 / Vd, (flapPos > 0.05f ? 1.0f : 1.5f) * g0 / Vd + qTurn);
            const float qdotDes = (qDes - Q) * 2.6f;
            pitchI_ = std::clamp(pitchI_ + (qDes - Q) * 1.2f * dt, -0.6f, 0.6f);
            const float CmReq = qdotDes * kIpitch / (qS * kC);
            const float deLaw = std::clamp((CmReq - CmOther) / 0.55f + pitchI_, -1.6f, 1.6f);
            // roll: the stick asks for roll rate (15 deg/s); neutral holds the bank up to 33 deg; 67 deg at most
            const float k33 = 0.576f, k67 = 1.17f;
            float pDes;
            if (std::abs(c.roll) > 0.04f) { pDes = c.roll * 0.26f; phiHold_ = phi; }
            else { phiHold_ = std::clamp(phiHold_, -k33, k33); pDes = std::clamp((phiHold_ - phi) * 0.9f, -0.15f, 0.15f); }
            if (phi > k67) pDes = std::min(pDes, (k67 - phi) * 1.5f);
            if (phi < -k67) pDes = std::max(pDes, (-k67 - phi) * 1.5f);
            const float ClReq = (pDes - P) * 3.0f * kIroll / (qS * kB);
            const float daLaw = std::clamp((ClReq - (-0.09f * beta - 0.45f * ph + 0.11f * rh)) / 0.055f, -1.0f, 1.0f);
            // yaw: turn coordination and the yaw damper; pedals add a little
            const float rDes = g0 / Vd * std::sin(phi) * std::cos(theta);
            const float CnReq = ((rDes - R) * 1.8f + beta * 1.2f) * kIyaw / (qS * kB);
            const float drLaw = std::clamp((CnReq - (0.11f * beta - 0.16f * rh - 0.03f * ph - 0.004f * daLaw)) / 0.065f + c.yaw * 0.4f, -1.0f, 1.0f);
            de = de + (deLaw - de) * lawBlend; da = da + (daLaw - da) * lawBlend; dr = dr + (drLaw - dr) * lawBlend;
        } else {
            gammaRef_ = gamma; phiHold_ = phi; pitchI_ = 0;
        }
        de = std::clamp(de, -1.6f, 1.6f); da = std::clamp(da, -1.0f, 1.0f); dr = std::clamp(dr, -1.0f, 1.0f);
        elevator = de; aileron = da; rudder = dr;
        float Cm = CmOther + 0.55f * de;
        if (std::abs(alpha) > 0.6f) Cm -= 0.5f * std::sin(alpha); // deep stall: the tail pushes the nose down
        const float Cl = -0.09f * beta - 0.45f * ph + 0.11f * rh + 0.055f * da + 0.008f * dr;
        const float Cn = 0.11f * beta - 0.16f * rh - 0.03f * ph - 0.004f * da + 0.065f * dr;
        const float CY = -0.85f * beta + 0.12f * dr;
        // ---- forces (world)
        const Vec3 f = Forward(), u = Up(), rgt = Right();
        const Vec3 vdir = va * (1.0f / V);
        Vec3 liftDir = tbd::Cross(tbd::Cross(vdir, u), vdir);
        const float ll = std::sqrt(tbd::Dot(liftDir, liftDir));
        liftDir = ll > 1e-4f ? liftDir * (1.0f / ll) : u;
        const Vec3 aero = liftDir * (q * kS * CL) - vdir * (q * kS * CD) + rgt * (q * kS * CY);
        float thrust = 0;
        for (const float n : n1) {
            const float s = std::clamp((n - idle) / (1.0f - idle), 0.0f, 1.0f);
            thrust += kThrust * (0.045f + 0.955f * std::pow(s, 1.7f)) * std::pow(rho / 1.225f, 0.75f) * std::max(0.45f, 1.0f - 0.0022f * V);
        }
        if (reverserOut) thrust *= -0.45f;
        Vec3 force = aero + f * thrust + Vec3{0, -g0 * kMass, 0};
        nz = tbd::Dot(aero + f * thrust, u) / (kMass * g0);
        // ---- moments (body: x pitch up, y yaw left, z roll left)
        Vec3 torque{q * kS * kC * Cm + thrust * 1.0f, -q * kS * kB * Cn, -q * kS * kB * Cl};
        // ---- the ground: three wheels, then the parts of the airframe that should never touch it
        const Vec3 cgB = CgB();
        const Vec3 cgW = tbd::Rotate(rot, cgB);
        auto groundAt = [&](double x, double z) { return std::max(0.0, ground.Height(x, z)); };
        auto pointVel = [&](Vec3 pb) { return vel + tbd::Rotate(rot, tbd::Cross(omega, pb - cgB)); };
        auto applyAt = [&](Vec3 pb, Vec3 fw) { force = force + fw; torque = torque + tbd::Cross(pb - cgB, tbd::Rotate(inv, fw)); };
        const bool water = gh < 0.3;
        onGround = false; mainsOnGround = false;
        const bool gearLocked = gearPos > 0.97f;
        const float steerMax = 1.2f + (0.10f - 1.2f) * std::clamp(Vh / 22.0f, 0.0f, 1.0f); // tiller 70 deg slow, pedals 6 deg fast
        steer = c.yaw * steerMax;
        const Wheel* W = Wheels();
        for (int i = 0; i < 3; ++i) {
            wheelLoad[i] = 0;
            if (!gearLocked) { stroke_[i] = 0; wheelWas_[i] = false; continue; }
            const Vec3 pb = W[i].p;
            const Vec3 pw = tbd::Rotate(rot, pb);
            const double wx = pos.x + pw.x, wy = pos.y + pw.y, wz = pos.z + pw.z;
            const double h = groundAt(wx, wz);
            const float comp = (float)(h - wy);
            if (comp <= 0) { stroke_[i] = 0; wheelWas_[i] = false; continue; }
            const Vec3 vp = pointVel(pb);
            const float rate = -vp.y;
            if (!wheelWas_[i]) {
                ev.impact = std::max(ev.impact, rate);
                if (water) { Crash("Ditched in the water"); return; }
                if (rate > (i == 0 ? 3.0f : 3.7f) * toughness) { Crash(i == 0 ? "Nose gear collapsed: hard touchdown" : "Main gear collapsed: hard landing"); return; }
            }
            if (comp > kStrokeMax * (toughness > 1 ? 1.6f : 1.0f)) { Crash("Gear collapsed: hit the ground too hard"); return; }
            stroke_[i] = comp; wheelWas_[i] = true;
            const float N = std::max(0.0f, W[i].k * comp + W[i].c * rate);
            wheelLoad[i] = N;
            onGround = true;
            if (i > 0) mainsOnGround = true;
            // tyre: rolling along the wheel's heading (the nose wheel steers), gripping across it
            Vec3 wf = tbd::Rotate(rot, W[i].steered ? Vec3{std::sin(steer), 0, -std::cos(steer)} : Vec3{0, 0, -1});
            wf.y = 0; wf = tbd::Normalize(wf);
            const Vec3 ws{-wf.z, 0, wf.x};
            const float vl = tbd::Dot(vp, wf), vs = tbd::Dot(vp, ws);
            const float br = W[i].braked ? std::max(c.brake, c.parkBrake ? 1.0f : 0.0f) : 0.0f;
            const float muLong = 0.012f + 0.55f * br;
            const Vec3 fr = wf * (-muLong * N * std::tanh(vl / (br > 0.5f ? 0.08f : 0.3f))) + ws * (-W[i].mu * N * std::tanh(vs / 0.15f));
            applyAt({pb.x, pb.y, pb.z}, Vec3{0, N, 0} + fr);
        }
        for (int k = 0; k < kHits; ++k) {
            const Vec3 pb = HitPoint(k);
            const Vec3 pw = tbd::Rotate(rot, pb);
            const double wx = pos.x + pw.x, wy = pos.y + pw.y, wz = pos.z + pw.z;
            const float depth = (float)(groundAt(wx, wz) - wy);
            if (depth <= 0) continue;
            const Vec3 vp = pointVel(pb);
            const float sp = std::sqrt(tbd::Dot(vp, vp));
            ev.impact = std::max(ev.impact, -vp.y);
            if (water && sp > 2.0f) { Crash("Ditched in the water"); return; }
            if (k == kTail) { ev.tailStrike = true; if (-vp.y > 3.0f * toughness || depth > 0.6f * toughness) { Crash("Tail strike: the tail hit the runway hard"); return; } }
            else if ((k == kPodL || k == kPodR) && sp > 3.0f * toughness * toughness) { Crash("An engine pod hit the ground"); return; }
            else if ((k == kTipL || k == kTipR) && sp > 3.0f * toughness * toughness) { Crash("A wingtip hit the ground"); return; }
            else if ((k == kBellyF || k == kBellyA) && sp > 6.0f * toughness) { Crash(gearLocked ? "Belly hit the ground" : "Belly landing: the gear was not down"); return; }
            else if (k == kNose && sp > 3.0f * toughness * toughness) { Crash("Flew into the ground"); return; }
            const float N = std::max(0.0f, 3.0e6f * depth + 2.5e5f * -vp.y);
            Vec3 vt{vp.x, 0, vp.z};
            const float vtl = std::sqrt(tbd::Dot(vt, vt));
            const Vec3 fr = vtl > 1e-3f ? vt * (-0.45f * N * std::tanh(vtl / 0.3f) / vtl) : Vec3{};
            applyAt(pb, Vec3{0, N, 0} + fr);
            onGround = true;
        }
        // buildings and other obstacles above the terrain: a few points of the airframe against their roofs
        if (obstacle) {
            static const Vec3 probe[6] = {{0, -1.0f, -19.0f}, {0, -2.0f, -3.0f}, {-16.5f, -0.9f, 2.5f}, {16.5f, -0.9f, 2.5f}, {0, 0.5f, 16.0f}, {0, -kGearH, -1.0f}};
            for (const Vec3& pb : probe) {
                const Vec3 pw = tbd::Rotate(rot, pb);
                const double wx = pos.x + pw.x, wy = pos.y + pw.y, wz = pos.z + pw.z;
                const double roof = obstacle(wx, wz);
                if (roof > groundAt(wx, wz) + 3.0 && wy < roof - 0.3) { Crash("Hit a building"); return; }
            }
        }
        if (nz > 3.9f * toughness || nz < -2.2f * toughness) { Crash("Structural failure: pulled too many g"); return; }
        if (ias > kVmo + 35.0f) { Crash("Structural failure: far beyond the maximum speed"); return; }
        // ---- integrate about the centre of gravity
        const double cx = pos.x + cgW.x, cy = pos.y + cgW.y, cz = pos.z + cgW.z;
        vel = vel + force * (dt / kMass);
        const Vec3 Iw{kIpitch * omega.x, kIyaw * omega.y, kIroll * omega.z};
        const Vec3 net = torque - tbd::Cross(omega, Iw);
        omega = omega + Vec3{net.x / kIpitch, net.y / kIyaw, net.z / kIroll} * dt;
        const float w = std::sqrt(tbd::Dot(omega, omega));
        if (w > 1e-7f) rot = tbd::Normalize(rot * tbd::QuatAxisAngle(omega * (1.0f / w), w * dt));
        const Vec3 cg2 = tbd::Rotate(rot, cgB);
        pos.x = cx + vel.x * dt - cg2.x; pos.y = cy + vel.y * dt - cg2.y; pos.z = cz + vel.z * dt - cg2.z;
        // ---- what the game reads
        const bool wasAir = airborneTime > 1.5f;
        if (mainsOnGround) {
            if (wasAir) { ev.touchdown = true; ev.tdSink = std::max(0.0f, ev.impact); ev.tdIas = ias; ev.tdBank = phi; ev.tdPitch = theta; ev.tdX = pos.x; ev.tdZ = pos.z; }
            airborneTime = 0; groundTime += dt;
        } else if (!onGround) { airborneTime += dt; groundTime = 0; }
        stallWarning = !onGround && alpha > alphaStall - 0.03f && airborneTime > 1.0f;
        overspeed = ias > kVmo;
    }

    void Crash(const char* why) { crashed = true; crashWhy = why; ev.crash = true; ev.why = why; }

    // Puts the aircraft on its wheels at (x, z), nose toward heading (radians, 0 north, clockwise), standing still.
    void PlaceOnGround(double x, double groundY, double z, float heading) {
        pos = {x, groundY + kGearH, z};
        rot = tbd::QuatAxisAngle({0, 1, 0}, -heading);
        vel = {0, 0, 0}; omega = {0, 0, 0};
        ResetSystems(1.0f);
        airborneTime = 0; groundTime = 1; onGround = mainsOnGround = true;
        for (bool& b : wheelWas_) b = true;
    }
    // In the air at speed, along heading, climbing at gammaRad, pitched up by alpha for level-ish flight.
    void PlaceInAir(double x, double y, double z, float heading, float speed, float gammaRad, float alphaRad, bool gearDown) {
        pos = {x, y, z};
        rot = tbd::QuatAxisAngle({0, 1, 0}, -heading) * tbd::QuatAxisAngle({1, 0, 0}, gammaRad + alphaRad);
        const Vec3 dir = tbd::Rotate(tbd::QuatAxisAngle({0, 1, 0}, -heading), Vec3{0, std::sin(gammaRad), -std::cos(gammaRad)});
        vel = dir * speed; omega = {0, 0, 0};
        ResetSystems(gearDown ? 1.0f : 0.0f);
        airborneTime = 10; groundTime = 0; onGround = mainsOnGround = false; lawBlend = 1;
        gammaRef_ = gammaRad; phiHold_ = 0;
    }
    void ResetSystems(float gear) {
        crashed = false; crashWhy = ""; ev = {};
        gearPos = gear; spoilers = 0; reverserOut = false; lawBlend = 0; pitchI_ = 0; gust_ = {};
        for (float& s : stroke_) s = 0;
        for (bool& b : wheelWas_) b = false;
    }
};

} // namespace fly
