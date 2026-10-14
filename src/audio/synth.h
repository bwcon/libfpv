// libfpv - procedural motor sound, pulled from the host's audio thread.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <mutex>

namespace fpv {

struct AudioParams {
    float rpm[4] = {0, 0, 0, 0};
    float load[4] = {0, 0, 0, 0};      // 0..1, from current draw
    float damage[4] = {0, 0, 0, 0};    // prop damage 0..1
    float pan[4] = {0, 0, 0, 0};       // -1 left .. 1 right
    float maxRpm = 30000;
    float blades = 3;
    bool beeper = false;
    bool paused = false;
};

class AudioSynth {
public:
    void configure(uint32_t sampleRate, uint32_t channels);
    void setParams(const AudioParams &p);
    void setVolume(float v);
    // Interleaved float samples; always writes `frames` frames.
    uint32_t read(float *out, uint32_t frames);

private:
    std::mutex m_mutex;  // short critical sections only (parameter copy)
    AudioParams m_target, m_cur;
    uint32_t m_rate = 48000, m_channels = 2;
    float m_volume = 0.15f;
    double m_phase[4] = {0, 0, 0, 0};
    double m_whine[4] = {0, 0, 0, 0};
    double m_wobble[4] = {0, 0, 0, 0};
    float m_noiseLp[4] = {0, 0, 0, 0};
    double m_beepPhase = 0;
    uint32_t m_seed = 0x12345678u;
};

}  // namespace fpv
