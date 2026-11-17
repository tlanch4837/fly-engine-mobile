// The game's 2D look: anti-aliased SDF text (DejaVu, data/ui) with outlines, frosted-glass panels, rings and soft dots, on the
// renderer's smooth UI path (Renderer::DrawUi). Falls back to the 5x7 pixel font where that path is missing (gui::DrawList
// classic mode). Shared by the world map, the flight HUD and the loading screen. No OS headers.
#pragma once
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "../gui/draw_list.h"
#include "../render/renderer.h"

namespace fly {

using tbd::gui::Color;
using tbd::gui::FontFace;
using tbd::gui::Rgba;

class UiKit {
public:
    // Loads the SDF font atlas (512 x 1024 RGBA, raw) and hands it to the renderer. Without it, or without the smooth path,
    // everything draws the classic way.
    bool Init(tbd::Renderer& r, const std::filesystem::path& dataDir) {
        if (!r.UiSupported()) return false;
        std::ifstream f(dataDir / "ui" / "ui_font_sdf.rgba", std::ios::binary);
        const size_t bytes = (size_t)tbd::gui::font_data::kAtlasWidth * tbd::gui::font_data::kAtlasHeight * 4;
        std::vector<uint8_t> px(bytes);
        if (!f.read((char*)px.data(), (std::streamsize)bytes)) return false;
        atlas_ = r.CreateTexture(px.data(), tbd::gui::font_data::kAtlasWidth, tbd::gui::font_data::kAtlasHeight);
        if (atlas_ == tbd::kNoTexture) return false;
        r.SetUiAtlas(atlas_);
        smooth_ = true;
        return true;
    }
    bool Smooth() const { return smooth_; }

    void Begin() { d.Begin(!smooth_); }
    // Draws everything added since Begin. `glass`: copy the frame first so frosted panels can blur it.
    void End(tbd::Renderer& r, int w, int h, bool glass) {
        const tbd::Mat4 ortho = tbd::OrthoPixels((float)w, (float)h);
        if (smooth_) {
            if (d.Ui().empty()) return;
            if (glass) r.CaptureUiBackdrop();
            r.DrawUi(d.Ui().data(), (uint32_t)d.Ui().size(), ortho);
        } else if (!d.Flat().empty()) {
            r.DrawTransient(d.Flat().data(), (uint32_t)d.Flat().size(), tbd::Topology::Triangles, ortho, tbd::Mat4::Identity(), false);
        }
    }

    float Width(const std::string& s, float size, FontFace face = FontFace::Regular) const { return d.TextWidth(s.c_str(), s.size(), size, face); }
    float Cap(float size, FontFace face = FontFace::Regular) const { return d.CapHeight(size, face); }

    // Text with its pen at x on the baseline; `outline` 0..0.35 of the glyph's distance range, in `stroke`.
    void Text(float x, float baseline, float size, const std::string& s, Color fill, FontFace face = FontFace::Regular, float outline = 0.0f,
              Color stroke = Rgba(0, 0, 0, 200)) {
        if (s.empty()) return;
        const size_t from = d.Ui().size();
        d.Text(x, baseline, size, face, s.c_str(), s.size(), fill);
        if (smooth_ && outline > 0.0f) {
            auto& ui = const_cast<std::vector<tbd::UiVertex>&>(d.Ui());
            for (size_t i = from; i < ui.size(); ++i) { ui[i].border = outline; ui[i].line = stroke; }
        }
    }
    void TextCentered(float cx, float baseline, float size, const std::string& s, Color fill, FontFace face = FontFace::Regular, float outline = 0.0f,
                      Color stroke = Rgba(0, 0, 0, 200)) {
        Text(cx - Width(s, size, face) * 0.5f, baseline, size, s, fill, face, outline, stroke);
    }
    void Rect(float x0, float y0, float x1, float y1, Color top, Color bottom, float radius, float border = 0, Color borderColor = 0) {
        d.Rect(x0, y0, x1, y1, top, bottom, radius, border, borderColor);
    }
    void Shadow(float x0, float y0, float x1, float y1, float radius, float blur, Color c) { if (smooth_) d.Shadow(x0, y0, x1, y1, radius, blur, c); }
    // A frosted panel (blurred frame behind a tint) with a hairline border; a plain translucent panel where glass is missing.
    void Glass(float x0, float y0, float x1, float y1, float radius, Color tint = Rgba(14, 20, 32, 150), Color border = Rgba(255, 255, 255, 38)) {
        if (!smooth_) { d.Rect(x0, y0, x1, y1, tint, tint, 0); return; }
        Shadow(x0, y0 + 3, x1, y1 + 3, radius, 14, Rgba(0, 0, 0, 70));
        auto& ui = const_cast<std::vector<tbd::UiVertex>&>(d.Ui());
        const size_t from = ui.size();
        d.Rect(x0, y0, x1, y1, tint, tint, radius, 1.0f, border);
        for (size_t i = from; i < ui.size(); ++i) { ui[i].mode = (float)tbd::UiMode::Glass; ui[i].softness = 120.0f * 1000.0f + 10.0f; }
    }
    void Circle(float cx, float cy, float rad, Color fill, float border = 0, Color borderColor = 0) { d.Circle(cx, cy, rad, fill, border, borderColor); }
    void Ring(float cx, float cy, float rad, float width, Color c) { d.Circle(cx, cy, rad + width * 0.5f, Rgba(0, 0, 0, 0), width, c); }
    void Line(float x0, float y0, float x1, float y1, float thick, Color c) { d.Line(x0, y0, x1, y1, thick, c); }
    // A key cap: the key's name in a small rounded box, then what it does. Returns the x after it.
    float Key(float x, float baseline, float size, const std::string& key, const std::string& what) {
        const float kw = Width(key, size * 0.85f, FontFace::Bold) + size * 0.9f, cap = Cap(size);
        Rect(x, baseline - cap - size * 0.35f, x + kw, baseline + size * 0.35f, Rgba(255, 255, 255, 46), Rgba(255, 255, 255, 26), size * 0.3f, 1.0f,
             Rgba(255, 255, 255, 60));
        Text(x + size * 0.45f, baseline - (cap - Cap(size * 0.85f)) * 0.5f, size * 0.85f, key, Rgba(255, 255, 255, 235), FontFace::Bold);
        x += kw + size * 0.45f;
        Text(x, baseline, size, what, Rgba(205, 214, 228, 220));
        return x + Width(what, size) + size * 1.3f;
    }

    tbd::gui::DrawList d;

private:
    tbd::TextureId atlas_ = tbd::kNoTexture;
    bool smooth_ = false;
};

} // namespace fly
