// libfpv - C API: OSD, audio, textures, overlay.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>

#include "audio/synth.h"
#include "lib.h"
#include "osd/osd.h"
#include "overlay/overlay.h"

using namespace fpv;

namespace fpv {

// The audio synth is read from the host's audio thread without the API
// mutex, so it lives behind an atomic pointer and is never destroyed.
static std::atomic<AudioSynth *> g_audio{nullptr};

AudioSynth &audioSynth() {
    AudioSynth *a = g_audio.load();
    if (!a) {
        AudioSynth *fresh = new AudioSynth();
        if (g_audio.compare_exchange_strong(a, fresh)) {
            a = fresh;
        } else {
            delete fresh;
        }
    }
    return *a;
}

OsdRenderer &osdRenderer() {
    Lib &L = lib();
    if (!L.osd) L.osd = new OsdRenderer();
    return *L.osd;
}

// Called by fpv_drone_tick with the API mutex held.
void updatePresentation(const fpv_frame &f) {
    Lib &L = lib();
    const DronePreset &p = currentPreset();
    AudioParams ap;
    float arm = std::sqrt(p.init.quadMotorPos[0].x * p.init.quadMotorPos[0].x +
                          p.init.quadMotorPos[0].z * p.init.quadMotorPos[0].z);
    for (int i = 0; i < 4; i++) {
        ap.rpm[i] = f.motors[i].rpm;
        ap.load[i] = std::clamp(std::fabs(f.motors[i].current_amp) / 35.0f, 0.0f, 1.0f);
        ap.damage[i] = f.motors[i].prop_damage;
        ap.pan[i] = arm > 0 ? std::clamp(p.init.quadMotorPos[i].x / arm, -1.0f, 1.0f) : 0.0f;
    }
    ap.maxRpm = p.soundMaxRpm;
    ap.blades = p.propBladeCount;
    ap.beeper = (f.flags & FPV_DRONE_BEEPER) != 0;
    ap.paused = (f.flags & FPV_DRONE_PAUSED) != 0;
    audioSynth().setParams(ap);
    audioSynth().setVolume(L.settings.audioVolume);
}

}  // namespace fpv

extern "C" {

FPV_API fpv_result fpv_texture_get(uint64_t id, fpv_texture *out) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!out) return setError(FPV_ERROR_INVALID_ARGUMENT, "out is NULL");
    if (id == FPV_TEXTURE_OSD_FONT) {
        OsdRenderer &r = osdRenderer();
        out->width = OsdRenderer::kAtlasW;
        out->height = OsdRenderer::kAtlasH;
        out->rgba = r.atlas().data();
        out->revision = 1;
        return FPV_OK;
    }
    if (id == FPV_TEXTURE_OVERLAY_FONT) {
        Overlay &o = overlay();
        return o.fontTexture(*out) ? FPV_OK : setError(FPV_ERROR_INTERNAL, "overlay font unavailable");
    }
    return setError(FPV_ERROR_NOT_FOUND, "unknown texture id %llu", (unsigned long long)id);
}

FPV_API const fpv_draw_data *fpv_osd_draw_data(const fpv_frame *frame, float width, float height,
                                              fpv_osd_style style) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!frame || !(width > 0) || !(height > 0)) {
        setError(FPV_ERROR_INVALID_ARGUMENT, "frame and a positive size are required");
        return nullptr;
    }
    return osdRenderer().drawData(frame->osd, width, height, style);
}

FPV_API fpv_result fpv_osd_render_rgba(const fpv_frame *frame, uint8_t *rgba, uint32_t width, uint32_t height,
                                       uint32_t stride, fpv_osd_style style) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!frame || !rgba || !width || !height) return setError(FPV_ERROR_INVALID_ARGUMENT, "frame, buffer and size required");
    if (stride == 0) stride = width * 4;
    if (stride < width * 4) return setError(FPV_ERROR_INVALID_ARGUMENT, "stride smaller than width * 4");
    osdRenderer().rasterize(frame->osd, rgba, width, height, stride, style);
    return FPV_OK;
}

FPV_API fpv_result fpv_audio_configure(uint32_t sampleRate, uint32_t channels) {
    if (sampleRate < 8000 || channels < 1) return setError(FPV_ERROR_INVALID_ARGUMENT, "bad audio format");
    audioSynth().configure(sampleRate, channels);
    return FPV_OK;
}

FPV_API uint32_t fpv_audio_read(float *out, uint32_t frames) {
    if (!out || !frames) return 0;
    return audioSynth().read(out, frames);
}

FPV_API void fpv_audio_set_volume(float v) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    L.settings.audioVolume = std::clamp(std::isfinite(v) ? v : 0.1f, 0.0f, 2.0f);
    audioSynth().setVolume(L.settings.audioVolume);
}

// --- input and overlay ---------------------------------------------------------

FPV_API void fpv_input_key(int key, int down) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (overlay().capturingKey()) {
        overlay().key(key, down != 0);
        return;
    }
    if (key == L.settings.overlayToggleKey && down) {
        overlay().toggle();
        return;
    }
    if (overlay().isOpen()) {
        overlay().key(key, down != 0);
        L.input.key(key, false);  // no flying while typing
        return;
    }
    L.input.key(key, down != 0);
}

FPV_API void fpv_overlay_set_open(int open) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().setOpen(open != 0);
}
FPV_API int fpv_overlay_is_open(void) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    return overlay().isOpen() ? 1 : 0;
}
FPV_API int fpv_overlay_wants_input(void) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    return overlay().isOpen() ? 1 : 0;
}
FPV_API void fpv_overlay_set_toggle_key(int key) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    L.settings.overlayToggleKey = key;
    if (L.initialized) saveSettings();
}
FPV_API void fpv_overlay_set_display(float w, float h, float scale) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().setDisplay(w, h, scale);
}
FPV_API void fpv_overlay_mouse_move(float x, float y) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().mouseMove(x, y);
}
FPV_API void fpv_overlay_mouse_button(int b, int down) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().mouseButton(b, down != 0);
}
FPV_API void fpv_overlay_mouse_wheel(float dx, float dy) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().mouseWheel(dx, dy);
}
FPV_API void fpv_overlay_text(const char *utf8) {
    if (!utf8) return;
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().text(utf8);
}
FPV_API void fpv_overlay_set_action(uint32_t id, const char *label) {
    if (!id) return;
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    auto &v = L.hostActions;
    auto it = std::find_if(v.begin(), v.end(), [id](const Lib::HostAction &a) { return a.id == id; });
    if (!label || !label[0]) {
        if (it != v.end()) v.erase(it);
        L.clickedActions.erase(std::remove(L.clickedActions.begin(), L.clickedActions.end(), id),
                               L.clickedActions.end());
    } else if (it != v.end()) {
        it->label = label;
    } else {
        v.push_back({id, label});
    }
}
FPV_API uint32_t fpv_overlay_poll_action(void) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (L.clickedActions.empty()) return 0;
    uint32_t id = L.clickedActions.front();
    L.clickedActions.erase(L.clickedActions.begin());
    return id;
}
FPV_API const fpv_draw_data *fpv_overlay_render(float dt) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return nullptr;
    L.runner.pump();  // browsers: the radio wizard reads live sticks with no drone
    return overlay().render(dt);
}

}  // extern "C"
