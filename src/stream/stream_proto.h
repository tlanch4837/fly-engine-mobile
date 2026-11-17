#pragma once
// Remote-play stream protocol: one host runs the game and streams its frames; a viewer (the phone) shows them and sends touch / mouse /
// keyboard back. Transport-neutral (the same bytes go over Wi-Fi TCP or the USB tunnel, stream_net.h). Portable C++, no platform headers.
//
// Packet on the wire: u32 payloadLen (LE) | u8 type | payload.   All numbers little-endian.
//   Hello  (host->viewer) u16 version, u16 w, u16 h, u8 codec
//   Frame  (host->viewer) u32 seq, u8 key(1 = full frame), then codec data
//   Input  (viewer->host) u16 count, then count * InputEvent (16 bytes)
//   Ping/Pong u32 token (round-trip timing), Bye (no payload)
// Codecs: 0 = raw RGBA, 1 = delta (XOR with the previous frame, then zero-run RLE; lossless, cheap, used by tests and as a fallback),
// 2 = H.264 (reserved: Media Foundation encode on Windows, VideoToolbox decode on iOS; the platform layer fills the bytes).
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace tbd::stream {

constexpr uint16_t kVersion = 1;
constexpr uint32_t kMaxPayload = 64u * 1024u * 1024u;

enum PacketType : uint8_t { kHello = 1, kFrame = 2, kInput = 3, kPing = 4, kPong = 5, kBye = 6 };
enum Codec : uint8_t { kRaw = 0, kDelta = 1, kH264 = 2 };
enum InputKind : uint8_t { kPointerDown = 1, kPointerMove = 2, kPointerUp = 3, kKeyDown = 4, kKeyUp = 5, kWheel = 6 };

// Pointer positions are 0..1 of the streamed frame, so the viewer's own size never matters. `code` is a key code or the pointer button
// (0 = left / primary touch, 1 = right, 2 = middle); `id` tells touch fingers apart; wheel puts its delta in y.
struct InputEvent {
    uint8_t kind = 0;
    uint8_t id = 0;
    uint16_t mods = 0; // bit0 shift, bit1 ctrl, bit2 alt
    float x = 0, y = 0;
    uint32_t code = 0;
    bool operator==(const InputEvent& o) const { return kind == o.kind && id == o.id && mods == o.mods && x == o.x && y == o.y && code == o.code; }
};
struct Hello { uint16_t version = kVersion, w = 0, h = 0; uint8_t codec = kDelta; };

using Bytes = std::vector<uint8_t>;

inline void Put16(Bytes& b, uint16_t v) { b.push_back(uint8_t(v)); b.push_back(uint8_t(v >> 8)); }
inline void Put32(Bytes& b, uint32_t v) { for (int i = 0; i < 4; ++i) b.push_back(uint8_t(v >> (8 * i))); }
inline void PutF(Bytes& b, float f) { uint32_t u; std::memcpy(&u, &f, 4); Put32(b, u); }
inline uint16_t Get16(const uint8_t* p) { return uint16_t(p[0] | (p[1] << 8)); }
inline uint32_t Get32(const uint8_t* p) { return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); }
inline float GetF(const uint8_t* p) { uint32_t u = Get32(p); float f; std::memcpy(&f, &u, 4); return f; }

// ---- packets -------------------------------------------------------------------------------------------------------------------
inline Bytes Wrap(PacketType t, const Bytes& payload) {
    Bytes out; out.reserve(5 + payload.size());
    Put32(out, uint32_t(payload.size())); out.push_back(t);
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}
inline Bytes MakeHello(const Hello& h) { Bytes p; Put16(p, h.version); Put16(p, h.w); Put16(p, h.h); p.push_back(h.codec); return Wrap(kHello, p); }
inline bool ParseHello(const Bytes& p, Hello& h) {
    if (p.size() < 7) return false;
    h.version = Get16(&p[0]); h.w = Get16(&p[2]); h.h = Get16(&p[4]); h.codec = p[6];
    return true;
}
inline Bytes MakeFrame(uint32_t seq, bool key, const Bytes& data) {
    Bytes p; Put32(p, seq); p.push_back(key ? 1 : 0); p.insert(p.end(), data.begin(), data.end());
    return Wrap(kFrame, p);
}
inline bool ParseFrame(const Bytes& p, uint32_t& seq, bool& key, const uint8_t*& data, size_t& n) {
    if (p.size() < 5) return false;
    seq = Get32(&p[0]); key = p[4] != 0; data = p.data() + 5; n = p.size() - 5;
    return true;
}
inline Bytes MakeInput(const std::vector<InputEvent>& ev) {
    Bytes p; Put16(p, uint16_t(ev.size()));
    for (const InputEvent& e : ev) { p.push_back(e.kind); p.push_back(e.id); Put16(p, e.mods); PutF(p, e.x); PutF(p, e.y); Put32(p, e.code); }
    return Wrap(kInput, p);
}
inline bool ParseInput(const Bytes& p, std::vector<InputEvent>& out) {
    if (p.size() < 2) return false;
    size_t n = Get16(&p[0]);
    if (p.size() != 2 + n * 16) return false;
    out.clear();
    for (size_t i = 0; i < n; ++i) {
        const uint8_t* q = &p[2 + i * 16];
        InputEvent e; e.kind = q[0]; e.id = q[1]; e.mods = Get16(q + 2); e.x = GetF(q + 4); e.y = GetF(q + 8); e.code = Get32(q + 12);
        out.push_back(e);
    }
    return true;
}
inline Bytes MakeToken(PacketType t, uint32_t token) { Bytes p; Put32(p, token); return Wrap(t, p); }

// ---- delta codec (lossless) ----------------------------------------------------------------------------------------------------
// XOR against the previous frame (all zero when nothing changed), then runs: 0x00 <u16 n> = n zero bytes, 0x01 <u16 n> <n literals>.
inline Bytes DeltaEncode(const uint8_t* cur, const uint8_t* prev, size_t n) {
    Bytes out; size_t i = 0;
    auto at = [&](size_t k) { return uint8_t(cur[k] ^ (prev ? prev[k] : 0)); };
    while (i < n) {
        size_t run = 0;
        while (i + run < n && run < 65535 && at(i + run) == 0) ++run;
        if (run >= 4 || (run > 0 && i + run == n)) { out.push_back(0); Put16(out, uint16_t(run)); i += run; continue; }
        size_t lit = 0;
        while (i + lit < n && lit < 65535) {
            if (at(i + lit) == 0) { size_t z = 0; while (i + lit + z < n && z < 4 && at(i + lit + z) == 0) ++z; if (z >= 4 || i + lit + z == n) break; }
            ++lit;
        }
        out.push_back(1); Put16(out, uint16_t(lit));
        for (size_t k = 0; k < lit; ++k) out.push_back(at(i + k));
        i += lit;
    }
    return out;
}
// Decodes into cur (n bytes); prev may be null for a key frame. False on malformed or short / long data.
inline bool DeltaDecode(const uint8_t* d, size_t dn, uint8_t* cur, const uint8_t* prev, size_t n) {
    size_t i = 0, o = 0;
    while (i < dn) {
        if (i + 3 > dn) return false;
        uint8_t t = d[i]; size_t len = Get16(d + i + 1); i += 3;
        if (o + len > n) return false;
        if (t == 0) { for (size_t k = 0; k < len; ++k) cur[o + k] = prev ? prev[o + k] : 0; }
        else if (t == 1) { if (i + len > dn) return false; for (size_t k = 0; k < len; ++k) cur[o + k] = uint8_t(d[i + k] ^ (prev ? prev[o + k] : 0)); i += len; }
        else return false;
        o += len;
    }
    return o == n;
}

} // namespace tbd::stream
