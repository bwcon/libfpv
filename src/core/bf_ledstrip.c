/*
 * libfpv - the WS2811 LED strip of the simulated flight controller.
 *
 * Betaflight's LED strip code (io/ledstrip.c, drivers/light_ws2811strip.c)
 * runs unchanged: its colours, warnings, larson scanner, rainbow and so on
 * come out here instead of going to a timer and DMA, and the host reads the
 * last whole frame with bfb_led_strip(). Compiled with the Betaflight sources;
 * simulation thread only.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "bf_bridge.h"

#if defined(USE_LED_STRIP)

#include "common/color.h"
#include "config/feature.h"
#include "drivers/io.h"
#include "drivers/light_ws2811strip.h"
#include "io/ledstrip.h"

extern volatile bool ws2811LedDataTransferInProgress;

static rgbColor24bpp_t pending[WS2811_LED_STRIP_LENGTH];  // the frame being written
static rgbColor24bpp_t shown[WS2811_LED_STRIP_LENGTH];    // the last whole frame

void ws2811LedStripInit(ioTag_t ioTag, ledStripFormatRGB_e ledFormat)
{
    UNUSED(ioTag);  // no pin: the strip is captured, not driven
    UNUSED(ledFormat);
}

bool ws2811LedStripHardwareInit(void)
{
    return true;
}

void ws2811LedStripUpdateTransferBuffer(const rgbColor24bpp_t *color, unsigned ledIndex)
{
    if (ledIndex < WS2811_LED_STRIP_LENGTH) pending[ledIndex] = *color;
}

void ws2811LedStripStartTransfer(void)
{
    memcpy(shown, pending, sizeof(shown));
    ws2811LedDataTransferInProgress = false;  // a real strip is done in about a millisecond
}

int bfb_led_strip(uint8_t *rgb, uint8_t *xy, int max)
{
    if (!featureIsEnabled(FEATURE_LED_STRIP)) return 0;
    int n = 0;
#if defined(USE_LED_STRIP_STATUS_MODE)
    // The configured LEDs, in strip order: Betaflight keeps them contiguous.
    for (int i = 0; i < LED_STRIP_MAX_LENGTH && n < max; i++) {
        const ledConfig_t *c = &ledStripStatusModeConfig()->ledConfigs[i];
        if (!*c) break;
        memcpy(&rgb[n * 3], shown[i].raw, 3);
        xy[n] = (uint8_t)(ledGetX(c) << 4 | ledGetY(c));
        n++;
    }
#endif
    return n;
}

#else

int bfb_led_strip(uint8_t *rgb, uint8_t *xy, int max)
{
    (void)rgb;
    (void)xy;
    (void)max;
    return 0;
}

#endif
