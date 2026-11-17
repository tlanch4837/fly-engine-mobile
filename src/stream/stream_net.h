#pragma once
// TCP transport for the remote-play stream (stream_proto.h). The same code serves Wi-Fi (the viewer connects to the host's IP) and USB:
// Apple's usbmuxd service on the laptop tunnels a laptop-side TCP port to a port the phone listens on, so over USB the *phone* is the
// server and the laptop dials 127.0.0.1:<muxd port>; over Wi-Fi either side can be the server. This file only does blocking sockets.
#include "stream_proto.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using SockT = SOCKET;
static constexpr SockT kBadSock = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
using SockT = int;
static constexpr SockT kBadSock = -1;
#endif

namespace tbd::stream {

inline void NetInit() {
#ifdef _WIN32
    static bool done = false;
    if (!done) { WSADATA w; WSAStartup(MAKEWORD(2, 2), &w); done = true; }
#endif
}
inline void CloseSock(SockT s) {
#ifdef _WIN32
    closesocket(s);
#else
    close(s);
#endif
}

// One connected peer. Send/Recv are blocking and whole-packet; use one thread per direction (or poll Readable first).
class Link {
public:
    Link() = default;
    explicit Link(SockT s) : s_(s) { NoDelay(); }
    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;
    Link(Link&& o) noexcept : s_(o.s_) { o.s_ = kBadSock; }
    Link& operator=(Link&& o) noexcept { if (this != &o) { Close(); s_ = o.s_; o.s_ = kBadSock; } return *this; }
    ~Link() { Close(); }

    bool Open() const { return s_ != kBadSock; }
    void Close() { if (s_ != kBadSock) { CloseSock(s_); s_ = kBadSock; } }

    bool Send(const Bytes& packet) { return SendAll(packet.data(), packet.size()); }
    // Reads one packet. False on close, error or an oversize / malformed length.
    bool Recv(PacketType& type, Bytes& payload) {
        uint8_t h[5];
        if (!RecvAll(h, 5)) return false;
        uint32_t n = Get32(h);
        if (n > kMaxPayload) return false;
        type = PacketType(h[4]);
        payload.resize(n);
        return n == 0 || RecvAll(payload.data(), n);
    }
    // True when a read would not block (data or a close is waiting), within timeoutMs.
    bool Readable(int timeoutMs) {
        fd_set fs; FD_ZERO(&fs); FD_SET(s_, &fs);
        timeval tv{ timeoutMs / 1000, (timeoutMs % 1000) * 1000 };
        return select(int(s_) + 1, &fs, nullptr, nullptr, &tv) > 0;
    }

private:
    void NoDelay() { int one = 1; setsockopt(s_, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one)); }
    bool SendAll(const uint8_t* p, size_t n) {
        while (n > 0) {
            int k = int(send(s_, reinterpret_cast<const char*>(p), int(n > (1u << 20) ? (1u << 20) : n), 0));
            if (k <= 0) return false;
            p += k; n -= size_t(k);
        }
        return true;
    }
    bool RecvAll(uint8_t* p, size_t n) {
        while (n > 0) {
            int k = int(recv(s_, reinterpret_cast<char*>(p), int(n > (1u << 20) ? (1u << 20) : n), 0));
            if (k <= 0) return false;
            p += k; n -= size_t(k);
        }
        return true;
    }
    SockT s_ = kBadSock;
};

class Listener {
public:
    ~Listener() { if (s_ != kBadSock) CloseSock(s_); }
    // port 0 picks a free port (see Port()). any = listen on all interfaces (Wi-Fi), else loopback only.
    bool Listen(uint16_t port, bool any) {
        NetInit();
        s_ = socket(AF_INET, SOCK_STREAM, 0);
        if (s_ == kBadSock) return false;
        int one = 1; setsockopt(s_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));
        sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(port); a.sin_addr.s_addr = htonl(any ? INADDR_ANY : INADDR_LOOPBACK);
        if (bind(s_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0 || listen(s_, 1) != 0) return false;
        socklen_t len = sizeof(a);
        getsockname(s_, reinterpret_cast<sockaddr*>(&a), &len);
        port_ = ntohs(a.sin_port);
        return true;
    }
    uint16_t Port() const { return port_; }
    // Waits for one viewer; an empty Link on error.
    Link Accept() {
        SockT c = accept(s_, nullptr, nullptr);
        return c == kBadSock ? Link() : Link(c);
    }
private:
    SockT s_ = kBadSock;
    uint16_t port_ = 0;
};

inline Link Connect(const char* host, uint16_t port) {
    NetInit();
    SockT s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == kBadSock) return Link();
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &a.sin_addr) != 1 || connect(s, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) { CloseSock(s); return Link(); }
    return Link(s);
}

} // namespace tbd::stream
