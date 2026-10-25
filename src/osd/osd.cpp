// libfpv - Betaflight OSD rendering.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "osd/osd.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <string>

#include "util/embedded.h"
#include "util/fs.h"
#include "util/log.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_STDIO  // fonts are read with libfpv's UTF-8 path helpers
#include "stb/stb_image.h"

namespace fpv {

bool parseMcmFont(const char *text, size_t size, std::vector<uint8_t> &rgba, int atlasW, int xOffset) {
    std::vector<uint8_t> bytes;
    bytes.reserve(256 * 64);
    size_t i = 0;
    // skip header line
    while (i < size && text[i] != '\n') i++;
    while (i < size && bytes.size() < 256 * 64) {
        while (i < size && (text[i] == '\n' || text[i] == '\r' || text[i] == ' ')) i++;
        if (i + 8 > size) break;
        uint8_t b = 0;
        for (int k = 0; k < 8; k++) {
            char c = text[i + k];
            if (c != '0' && c != '1') return false;
            b = (uint8_t)((b << 1) | (c == '1'));
        }
        bytes.push_back(b);
        i += 8;
    }
    if (bytes.size() < 256 * 64) return false;
    for (int g = 0; g < 256; g++) {
        int gx = (g % 16) * OsdRenderer::kGlyphW + xOffset, gy = (g / 16) * OsdRenderer::kGlyphH;
        for (int p = 0; p < OsdRenderer::kGlyphW * OsdRenderer::kGlyphH; p++) {
            uint8_t byte = bytes[g * 64 + p / 4];
            int v = (byte >> (6 - 2 * (p % 4))) & 3;
            int px = gx + p % OsdRenderer::kGlyphW, py = gy + p / OsdRenderer::kGlyphW;
            uint8_t *o = &rgba[((size_t)py * atlasW + px) * 4];
            if (v == 0) {  // black (outline)
                o[0] = o[1] = o[2] = 0;
                o[3] = 255;
            } else if (v == 2) {  // white
                o[0] = o[1] = o[2] = 255;
                o[3] = 255;
            } else {  // transparent
                o[0] = o[1] = o[2] = o[3] = 0;
            }
        }
    }
    return true;
}

bool loadFontSheet(const std::string &bytes, bool png, OsdRenderer::Sheet &out, std::string &error) {
    OsdRenderer::Sheet s;
    if (png) {
        int w = 0, h = 0, n = 0;
        uint8_t *px = stbi_load_from_memory((const stbi_uc *)bytes.data(), (int)bytes.size(), &w, &h, &n, 4);
        if (!px) {
            error = std::string("not a readable PNG: ") + stbi_failure_reason();
            return false;
        }
        s.rgba.assign(px, px + (size_t)w * h * 4);
        stbi_image_free(px);
        s.w = w;
        s.h = h;
        // 256 glyphs of 2:3 cells, one per row (DJI O3 / SNEAKY_FPV sheets)
        s.glyphH = h / 256;
        s.glyphW = s.glyphH * 2 / 3;
        if (h % 256 || s.glyphH < 3 || s.glyphW > w) {
            error = "expected 256 rows of 2:3 glyphs, got " + std::to_string(w) + "x" + std::to_string(h);
            return false;
        }
    } else {
        // WTFOS msp-osd: raw RGBA, 256 glyphs of 36 x 54 (SD) or 24 x 36 (HD)
        const size_t sd = 256u * 36 * 54 * 4, hd = 256u * 24 * 36 * 4;
        if (bytes.size() != sd && bytes.size() != hd) {
            error = "expected a WTFOS font of " + std::to_string(sd) + " or " + std::to_string(hd) + " bytes, got " +
                    std::to_string(bytes.size());
            return false;
        }
        s.glyphW = bytes.size() == sd ? 36 : 24;
        s.glyphH = bytes.size() == sd ? 54 : 36;
        s.w = s.glyphW;
        s.h = s.glyphH * 256;
        s.rgba.assign(bytes.begin(), bytes.end());
    }
    out = std::move(s);
    return true;
}

bool OsdRenderer::loadPackage(const std::string &dirUtf8, std::string &error) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::path dir = pathFromUtf8(dirUtf8);
    if (!fs::is_directory(dir, ec)) {
        error = "no folder " + dirUtf8;
        return false;
    }
    // Best file per canvas: the O3 sheet suits DJI's HD goggles, then WTFOS HD,
    // then the generic HD sheet; for analog the SD sheet, then WTFOS SD.
    fs::path hd[3], sd[2];
    for (const auto &e : fs::directory_iterator(dir, ec)) {
        if (!e.is_regular_file(ec)) continue;
        std::string n = pathToUtf8(e.path().filename());
        std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return (char)std::tolower(c); });
        auto ends = [&](const char *s) {
            size_t k = std::strlen(s);
            return n.size() >= k && n.compare(n.size() - k, k, s) == 0;
        };
        if (ends("_o3.png")) hd[0] = e.path();
        else if (n == "font_bf_hd.bin") hd[1] = e.path();
        else if (ends("_hd.png")) hd[2] = e.path();
        else if (ends("_sd.png")) sd[0] = e.path();
        else if (n == "font_bf.bin") sd[1] = e.path();
    }
    auto load = [&](const fs::path *candidates, int count, Sheet &out) {
        for (int i = 0; i < count; i++) {
            std::string bytes, err;
            if (candidates[i].empty() || !readTextFile(candidates[i], bytes)) continue;
            if (loadFontSheet(bytes, candidates[i].extension() == ".png" || candidates[i].extension() == ".PNG", out, err))
                return true;
            FPV_LOGW("OSD font %s skipped: %s", pathToUtf8(candidates[i]).c_str(), err.c_str());
        }
        return false;
    };
    Sheet newHd, newSd;
    bool anyHd = load(hd, 3, newHd), anySd = load(sd, 2, newSd);
    if (!anyHd && !anySd) {
        error = "no usable goggle font (*_SD.png, *_HD.png, *_O3.png, font_bf.bin, font_bf_hd.bin) in " + dirUtf8;
        return false;
    }
    m_hd = std::move(newHd);
    m_sd = std::move(newSd);
    m_revision++;
    FPV_LOGI("OSD font %s: HD %dx%d, SD %dx%d glyphs", dirUtf8.c_str(), m_hd.glyphW, m_hd.glyphH, m_sd.glyphW,
             m_sd.glyphH);
    return true;
}

void OsdRenderer::clearPackage() {
    m_hd = Sheet{};
    m_sd = Sheet{};
    m_revision++;
}

const OsdRenderer::Sheet *OsdRenderer::sheet(int style) const {
    const Sheet &s = style == FPV_OSD_STYLE_HD ? m_hd : m_sd;
    return s.w ? &s : nullptr;
}

void OsdRenderer::texture(int style, const uint8_t *&rgba, int &w, int &h) const {
    if (const Sheet *s = sheet(style)) {
        rgba = s->rgba.data(), w = s->w, h = s->h;
    } else {
        rgba = m_atlas.data(), w = kAtlasW, h = kAtlasH;
    }
}

void OsdRenderer::glyphUv(uint8_t g, int style, float &u0, float &v0, float &u1, float &v1) const {
    if (const Sheet *s = sheet(style)) {
        // the white column; half a texel in, so filtering never bleeds the next glyph in
        u0 = 0.5f / s->w;
        u1 = (s->glyphW - 0.5f) / s->w;
        v0 = (g * s->glyphH + 0.5f) / s->h;
        v1 = ((g + 1) * s->glyphH - 0.5f) / s->h;
        return;
    }
    int font = style == FPV_OSD_STYLE_HD ? 1 : 0;
    u0 = (float)((g % 16) * kGlyphW + font * kGlyphW * 16) / kAtlasW;
    v0 = (float)((g / 16) * kGlyphH) / kAtlasH;
    u1 = u0 + (float)kGlyphW / kAtlasW;
    v1 = v0 + (float)kGlyphH / kAtlasH;
}

OsdRenderer::OsdRenderer() {
    m_atlas.assign((size_t)kAtlasW * kAtlasH * 4, 0);
    const char *names[kFonts] = {"betaflight-default.mcm", "betaflight-bold.mcm"};
    int loaded = 0;
    for (int f = 0; f < kFonts; f++) {
        for (size_t i = 0; i < kEmbeddedFontsCount; i++) {
            if (std::string(kEmbeddedFonts[i].path) == names[f] &&
                parseMcmFont((const char *)kEmbeddedFonts[i].data, kEmbeddedFonts[i].size, m_atlas, kAtlasW,
                             f * kGlyphW * 16))
                loaded++;
        }
    }
    m_ok = loaded == kFonts;
    if (!m_ok) FPV_LOGE("OSD font missing or corrupt");
}

void OsdRenderer::viewport(float width, float height, int style, float &x, float &y, float &w, float &h) {
    float aspect = style == FPV_OSD_STYLE_HD ? 16.0f / 9.0f : 4.0f / 3.0f;
    w = width;
    h = width / aspect;
    if (h > height) {
        h = height;
        w = height * aspect;
    }
    x = (width - w) * 0.5f;
    y = (height - h) * 0.5f;
}

static bool blank(uint8_t c) { return c == 0 || c == ' '; }

const fpv_draw_data *OsdRenderer::drawData(const uint8_t *osd, int rows, int cols, float width, float height,
                                           int style) {
    m_buffer.clear();
    float vx, vy, vw, vh;
    viewport(width, height, style, vx, vy, vw, vh);
    // Analog keeps the PAL character cell inside a small margin; the HD
    // canvas (53 x 20) spans the 16:9 picture the way DJI goggles draw it.
    float marginX = cols > FPV_OSD_COLS ? vw * 0.015f : style == FPV_OSD_STYLE_HD ? vw * 0.06f : vw * 0.03f;
    float marginY = cols > FPV_OSD_COLS ? vh * 0.02f : vh * 0.04f;
    float cw = (vw - 2 * marginX) / cols, ch = (vh - 2 * marginY) / rows;
    DrawListStorage &l = m_buffer.newList();
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            uint8_t g = osd[r * cols + c];
            if (blank(g)) continue;
            float x0 = vx + marginX + c * cw, y0 = vy + marginY + r * ch;
            float u0, v0, u1, v1;
            glyphUv(g, style, u0, v0, u1, v1);
            uint32_t base = (uint32_t)l.vertices.size();
            const uint32_t white = 0xFFFFFFFFu;
            l.vertices.push_back({{x0, y0}, {u0, v0}, white});
            l.vertices.push_back({{x0 + cw, y0}, {u1, v0}, white});
            l.vertices.push_back({{x0 + cw, y0 + ch}, {u1, v1}, white});
            l.vertices.push_back({{x0, y0 + ch}, {u0, v1}, white});
            uint32_t q[6] = {base, base + 1, base + 2, base, base + 2, base + 3};
            l.indices.insert(l.indices.end(), q, q + 6);
        }
    }
    if (!l.indices.empty()) {
        fpv_draw_cmd cmd{};
        cmd.clip_rect[0] = vx;
        cmd.clip_rect[1] = vy;
        cmd.clip_rect[2] = vx + vw;
        cmd.clip_rect[3] = vy + vh;
        cmd.texture_id = FPV_TEXTURE_OSD_FONT;
        cmd.index_count = (uint32_t)l.indices.size();
        l.commands.push_back(cmd);
    }
    return m_buffer.finish(width, height);
}

void OsdRenderer::rasterize(const uint8_t *osd, int rows, int cols, uint8_t *rgba, uint32_t width, uint32_t height,
                            uint32_t stride, int style) {
    for (uint32_t y = 0; y < height; y++) std::memset(rgba + (size_t)y * stride, 0, (size_t)width * 4);
    const fpv_draw_data *dd = drawData(osd, rows, cols, (float)width, (float)height, style);
    if (!dd->list_count) return;
    const uint8_t *tex;
    int tw, th;
    texture(style, tex, tw, th);
    const fpv_draw_list &l = dd->lists[0];
    // every glyph is an axis-aligned quad: 4 vertices
    for (uint32_t q = 0; q + 3 < l.vertex_count; q += 4) {
        const fpv_draw_vertex &a = l.vertices[q], &c = l.vertices[q + 2];
        int x0 = (int)std::floor(a.pos[0]), y0 = (int)std::floor(a.pos[1]);
        int x1 = (int)std::ceil(c.pos[0]), y1 = (int)std::ceil(c.pos[1]);
        for (int y = std::max(0, y0); y < std::min((int)height, y1); y++) {
            float tv = (y + 0.5f - a.pos[1]) / (c.pos[1] - a.pos[1]);
            int ay = (int)((a.uv[1] + (c.uv[1] - a.uv[1]) * tv) * th);
            for (int x = std::max(0, x0); x < std::min((int)width, x1); x++) {
                float tu = (x + 0.5f - a.pos[0]) / (c.pos[0] - a.pos[0]);
                int ax = (int)((a.uv[0] + (c.uv[0] - a.uv[0]) * tu) * tw);
                ax = std::clamp(ax, 0, tw - 1);
                ay = std::clamp(ay, 0, th - 1);
                const uint8_t *s = &tex[((size_t)ay * tw + ax) * 4];
                if (s[3]) std::memcpy(rgba + (size_t)y * stride + (size_t)x * 4, s, 4);
            }
        }
    }
}

}  // namespace fpv
