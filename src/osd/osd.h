// libfpv - Betaflight OSD rendering (font atlas, quads, CPU raster).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "osd/draw_buffer.h"

namespace fpv {

class OsdRenderer {
public:
    static constexpr int kGlyphW = 12, kGlyphH = 18;
    static constexpr int kFonts = 2;  // 0 analog (default), 1 HD (bold)
    static constexpr int kAtlasW = kGlyphW * 16 * kFonts, kAtlasH = kGlyphH * 16;

    // A goggle font sheet: 256 glyphs stacked one per row, the plain (white)
    // glyph in the first column. DJI O3 sheets add colour columns for
    // Betaflight's OSD severities.
    struct Sheet {
        std::vector<uint8_t> rgba;
        int w = 0, h = 0, glyphW = 0, glyphH = 0;
    };

    OsdRenderer();
    bool ok() const { return m_ok; }

    // A goggle font package: a folder with the files that go on the goggles'
    // SD card, unchanged. DJI O3 sheets (*_SD.png, *_HD.png, *_O3.png) or
    // WTFOS msp-osd fonts (font_bf.bin, font_bf_hd.bin). False (and the
    // built-in fonts stay) when the folder has none.
    bool loadPackage(const std::string &dirUtf8, std::string &error);
    void clearPackage();

    // The image FPV_TEXTURE_OSD_FONT holds for `style`, and its revision.
    void texture(int style, const uint8_t *&rgba, int &w, int &h) const;
    uint32_t revision(int style) const { return m_revision * 2 + (style ? 1 : 0); }
    // A boot logo image (PNG bytes), drawn over Betaflight's glyph logo block.
    bool loadLogo(const std::string &png, std::string &error);
    void clearLogo();
    const Sheet &logo() const { return m_logo; }
    uint32_t logoRevision() const { return m_logoRevision; }
    // Texture coordinates of glyph g for `style`.
    void glyphUv(uint8_t g, int style, float &u0, float &v0, float &u1, float &v1) const;

    // osd: rows x cols characters, row major (30 x 16 analog, 53 x 20 HD).
    const fpv_draw_data *drawData(const uint8_t *osd, int rows, int cols, float width, float height, int style);
    void rasterize(const uint8_t *osd, int rows, int cols, uint8_t *rgba, uint32_t width, uint32_t height,
                   uint32_t stride, int style);

    // Viewport of the OSD grid inside a width x height target.
    static void viewport(float width, float height, int style, float &x, float &y, float &w, float &h);

private:
    // The package sheet for a style, or null for the built-in atlas.
    const Sheet *sheet(int style) const;

    bool m_ok = false;
    std::vector<uint8_t> m_atlas;  // RGBA, the built-in fonts
    Sheet m_sd, m_hd;              // the package's, empty without one
    uint32_t m_revision = 1;
    Sheet m_logo;  // glyphW/glyphH unused
    uint32_t m_logoRevision = 1;
    DrawBuffer m_buffer;
};

// Parses a MAX7456 .mcm font into 256 glyphs of 12x18 2-bit pixels written
// into an RGBA atlas region (16 x 16 glyphs) at column offset xOffset.
bool parseMcmFont(const char *text, size_t size, std::vector<uint8_t> &rgba, int atlasW, int xOffset);

// A goggle font sheet from a PNG (any width, 256 rows of glyphs) or a WTFOS
// raw RGBA .bin (36 x 54 or 24 x 36 glyphs). Exposed for the tests.
bool loadFontSheet(const std::string &bytes, bool png, OsdRenderer::Sheet &out, std::string &error);

}  // namespace fpv
