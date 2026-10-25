// libfpv - Betaflight OSD rendering (font atlas, quads, CPU raster).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <vector>

#include "osd/draw_buffer.h"

namespace fpv {

class OsdRenderer {
public:
    static constexpr int kGlyphW = 12, kGlyphH = 18;
    static constexpr int kFonts = 2;  // 0 analog (default), 1 HD (bold)
    static constexpr int kAtlasW = kGlyphW * 16 * kFonts, kAtlasH = kGlyphH * 16;

    OsdRenderer();
    bool ok() const { return m_ok; }
    const std::vector<uint8_t> &atlas() const { return m_atlas; }

    // osd: rows x cols characters, row major (30 x 16 analog, 53 x 20 HD).
    const fpv_draw_data *drawData(const uint8_t *osd, int rows, int cols, float width, float height, int style);
    void rasterize(const uint8_t *osd, int rows, int cols, uint8_t *rgba, uint32_t width, uint32_t height,
                   uint32_t stride, int style);

    // Viewport of the OSD grid inside a width x height target.
    static void viewport(float width, float height, int style, float &x, float &y, float &w, float &h);

private:
    bool m_ok = false;
    std::vector<uint8_t> m_atlas;  // RGBA
    DrawBuffer m_buffer;
};

// Parses a MAX7456 .mcm font into 256 glyphs of 12x18 2-bit pixels written
// into an RGBA atlas region (16 x 16 glyphs) at column offset xOffset.
bool parseMcmFont(const char *text, size_t size, std::vector<uint8_t> &rgba, int atlasW, int xOffset);

}  // namespace fpv
