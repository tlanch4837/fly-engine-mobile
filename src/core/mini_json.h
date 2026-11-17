// A minimal JSON reader (objects, arrays, strings, numbers, bools, null; // line comments and a BOM are accepted). Platform free.
// Shared by the scene loader (scene_spec.h) and the building catalog (building_catalog.h).
#pragma once
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace tbd {
namespace city {
namespace scene {

// ------------------------------------------------------------------------------------------------ minimal JSON
struct Json {
    enum Type { Null, Bool, Num, Str, Arr, Obj } t = Null;
    bool b = false;
    double n = 0;
    std::string s;
    std::vector<Json> a;
    std::vector<std::pair<std::string, Json>> o;
    const Json* Get(const char* k) const {
        for (const auto& kv : o)
            if (kv.first == k) return &kv.second;
        return nullptr;
    }
};

namespace detail {
struct Parser {
    const std::string& s;
    size_t i = 0;
    std::string err;
    explicit Parser(const std::string& text) : s(text) {}
    bool Fail(const char* what) {
        if (err.empty()) {
            int line = 1, col = 1;
            for (size_t k = 0; k < i && k < s.size(); ++k) s[k] == '\n' ? (++line, col = 1) : ++col;
            char buf[160];
            std::snprintf(buf, sizeof(buf), "JSON error at line %d, column %d: %s", line, col, what);
            err = buf;
        }
        return false;
    }
    void Ws() {
        for (;;) {
            while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) ++i;
            if (i + 1 < s.size() && s[i] == '/' && s[i + 1] == '/') { // // line comments are allowed (hand-authored files)
                while (i < s.size() && s[i] != '\n') ++i;
                continue;
            }
            break;
        }
    }
    static void Utf8(std::string& out, unsigned cp) {
        if (cp < 0x80) out += (char)cp;
        else if (cp < 0x800) out += (char)(0xC0 | (cp >> 6)), out += (char)(0x80 | (cp & 0x3F));
        else out += (char)(0xE0 | (cp >> 12)), out += (char)(0x80 | ((cp >> 6) & 0x3F)), out += (char)(0x80 | (cp & 0x3F));
    }
    bool String(std::string& out) {
        ++i; // opening quote
        while (i < s.size() && s[i] != '"') {
            char c = s[i++];
            if (c != '\\') { out += c; continue; }
            if (i >= s.size()) return Fail("unterminated string");
            c = s[i++];
            switch (c) {
            case 'n': out += '\n'; break;
            case 't': out += '\t'; break;
            case 'r': out += '\r'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case '"': case '\\': case '/': out += c; break;
            case 'u': {
                if (i + 4 > s.size()) return Fail("bad \\u escape");
                unsigned cp = 0;
                for (int k = 0; k < 4; ++k) {
                    const char h = s[i++];
                    cp = cp * 16 + (unsigned)(h >= '0' && h <= '9' ? h - '0' : h >= 'a' && h <= 'f' ? h - 'a' + 10 : h >= 'A' && h <= 'F' ? h - 'A' + 10 : 0);
                }
                Utf8(out, cp);
                break;
            }
            default: return Fail("bad escape");
            }
        }
        if (i >= s.size()) return Fail("unterminated string");
        ++i;
        return true;
    }
    bool Value(Json& v, int depth) {
        if (depth > 64) return Fail("nested too deeply");
        Ws();
        if (i >= s.size()) return Fail("unexpected end of file");
        const char c = s[i];
        if (c == '{') {
            v.t = Json::Obj;
            ++i;
            Ws();
            if (i < s.size() && s[i] == '}') return ++i, true;
            for (;;) {
                Ws();
                if (i >= s.size() || s[i] != '"') return Fail("expected a field name");
                std::string k;
                if (!String(k)) return false;
                Ws();
                if (i >= s.size() || s[i] != ':') return Fail("expected ':'");
                ++i;
                Json child;
                if (!Value(child, depth + 1)) return false;
                v.o.emplace_back(std::move(k), std::move(child));
                Ws();
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == '}') return ++i, true;
                return Fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            v.t = Json::Arr;
            ++i;
            Ws();
            if (i < s.size() && s[i] == ']') return ++i, true;
            for (;;) {
                Json child;
                if (!Value(child, depth + 1)) return false;
                v.a.push_back(std::move(child));
                Ws();
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == ']') return ++i, true;
                return Fail("expected ',' or ']'");
            }
        }
        if (c == '"') return v.t = Json::Str, String(v.s);
        if (s.compare(i, 4, "true") == 0) return v.t = Json::Bool, v.b = true, i += 4, true;
        if (s.compare(i, 5, "false") == 0) return v.t = Json::Bool, v.b = false, i += 5, true;
        if (s.compare(i, 4, "null") == 0) return v.t = Json::Null, i += 4, true;
        if (c == '-' || (c >= '0' && c <= '9')) {
            char* end = nullptr;
            v.n = std::strtod(s.c_str() + i, &end);
            if (end == s.c_str() + i) return Fail("bad number");
            v.t = Json::Num;
            i = (size_t)(end - s.c_str());
            return true;
        }
        return Fail("unexpected character");
    }
};
} // namespace detail

inline bool ParseJson(const std::string& text, Json& out, std::string& err) {
    detail::Parser p(text);
    out = Json();
    if (text.size() >= 3 && (unsigned char)text[0] == 0xEF && (unsigned char)text[1] == 0xBB && (unsigned char)text[2] == 0xBF) p.i = 3; // BOM
    if (!p.Value(out, 0)) return err = p.err, false;
    p.Ws();
    if (p.i != text.size()) return p.Fail("trailing characters after the document"), err = p.err, false;
    return true;
}

} // namespace scene
} // namespace city
} // namespace tbd
