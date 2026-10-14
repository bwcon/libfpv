// libfpv - input manager.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "input/input.h"

#include <algorithm>
#include <cmath>

#include "libfpv.h"
#include "nlohmann/json.hpp"
#include "util/fs.h"
#include "util/log.h"

namespace fpv {

using json = nlohmann::json;

InputManager::InputManager() {}
InputManager::~InputManager() { stopDevices(); }

void InputManager::setDataDir(const std::string &dir) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_dataDir = dir;
    loadMappings();
}

bool InputManager::startDevices(std::string &error) {
    auto backend = createSdlBackend();
    if (!backend) {
        error = "device input not compiled in";
        return false;
    }
    if (!backend->init(error)) return false;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_backend = std::move(backend);
    return true;
}

void InputManager::stopDevices() {
    std::unique_ptr<DeviceBackend> b;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        b = std::move(m_backend);
        m_devices.clear();
    }
    if (b) b->shutdown();
}

void InputManager::setHostChannels(const float *ch, uint32_t count, double wallTime) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (uint32_t i = 0; i < kChannelCount; i++) m_host[i] = (i < count && std::isfinite(ch[i])) ? std::clamp(ch[i], -1.0f, 1.0f) : (i == 2 || i >= 4 ? -1.0f : 0.0f);
    m_hostTime = wallTime;
}

void InputManager::clearHostChannels() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hostTime = -1e9;
}

void InputManager::key(int key, bool down) {
    if (key < 0 || key >= (int)m_keys.size()) return;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_keys[key] = down;
}

// --- persistence ---------------------------------------------------------

void InputManager::loadMappings() {
    m_mappings.clear();
    if (m_dataDir.empty()) return;
    std::string text;
    if (!readTextFile(pathFromUtf8(m_dataDir) / "radio_mappings.json", text)) return;
    try {
        json j = json::parse(text);
        for (auto &jm : j.at("devices")) {
            DeviceMapping m;
            m.guid = jm.value("guid", "");
            m.name = jm.value("name", "");
            m.complete = jm.value("complete", false);
            auto &chs = jm.at("channels");
            for (size_t i = 0; i < (size_t)kChannelCount && i < chs.size(); i++) {
                ChannelBinding &b = m.channels[i];
                b.axis = chs[i].value("axis", -1);
                b.button = chs[i].value("button", -1);
                b.inverted = chs[i].value("inverted", false);
                b.min = chs[i].value("min", -1.0f);
                b.center = chs[i].value("center", 0.0f);
                b.max = chs[i].value("max", 1.0f);
            }
            if (!m.guid.empty()) m_mappings.push_back(m);
        }
    } catch (const std::exception &e) {
        FPV_LOGW("radio_mappings.json ignored: %s", e.what());
    }
}

void InputManager::saveMappings() {
    if (m_dataDir.empty()) return;
    json j;
    j["version"] = 1;
    j["devices"] = json::array();
    for (auto &m : m_mappings) {
        json jm;
        jm["guid"] = m.guid;
        jm["name"] = m.name;
        jm["complete"] = m.complete;
        jm["channels"] = json::array();
        for (auto &b : m.channels) {
            jm["channels"].push_back({{"axis", b.axis}, {"button", b.button}, {"inverted", b.inverted},
                                      {"min", b.min}, {"center", b.center}, {"max", b.max}});
        }
        j["devices"].push_back(jm);
    }
    writeFileAtomic(pathFromUtf8(m_dataDir) / "radio_mappings.json", j.dump(2));
}

bool InputManager::getMapping(const std::string &guid, DeviceMapping &out) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto &m : m_mappings) {
        if (m.guid == guid) {
            out = m;
            return true;
        }
    }
    return false;
}

void InputManager::setMapping(const DeviceMapping &mapping) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = std::find_if(m_mappings.begin(), m_mappings.end(), [&](auto &m) { return m.guid == mapping.guid; });
    if (it != m_mappings.end()) {
        *it = mapping;
    } else {
        m_mappings.push_back(mapping);
    }
    saveMappings();
}

void InputManager::forgetMapping(const std::string &guid) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_mappings.erase(std::remove_if(m_mappings.begin(), m_mappings.end(), [&](auto &m) { return m.guid == guid; }),
                     m_mappings.end());
    saveMappings();
}

// --- mapping ---------------------------------------------------------------

static float applyCalibration(const ChannelBinding &b, float raw) {
    float v;
    if (raw >= b.center) {
        float span = b.max - b.center;
        v = span > 1e-4f ? (raw - b.center) / span : 0.0f;
    } else {
        float span = b.center - b.min;
        v = span > 1e-4f ? (raw - b.center) / span : 0.0f;
    }
    v = std::clamp(v, -1.0f, 1.0f);
    return b.inverted ? -v : v;
}

bool InputManager::mapRadio(const RawDevice &d, const DeviceMapping &m, float out[kChannelCount]) {
    for (int c = 0; c < kChannelCount; c++) {
        const ChannelBinding &b = m.channels[c];
        float v = (c == kThrottle || c >= kAux1) ? -1.0f : 0.0f;
        if (b.axis >= 0 && b.axis < (int)d.axes.size()) {
            v = c >= kAux1 ? switchPosition(m, c, d.axes[b.axis]) : applyCalibration(b, d.axes[b.axis]);
        } else if (b.button >= 0 && b.button < (int)d.buttons.size()) {
            bool pressed = d.buttons[b.button] != 0;
            v = (pressed != b.inverted) ? 1.0f : -1.0f;
        }
        out[c] = v;
    }
    return true;
}

static float deadband(float v, float db) {
    if (std::fabs(v) < db) return 0;
    return (v - std::copysign(db, v)) / (1.0f - db);
}

// Gamepad (degraded): mode 2 sticks, throttle from the left stick where the
// centre is idle, A toggles arm, Y toggles turtle, ANGLE mode on (AUX2 high).
void InputManager::mapGamepad(const RawDevice &d, float dt, float out[kChannelCount]) {
    (void)dt;
    const float db = 0.08f;
    float thr = std::max(0.0f, deadband(-d.pad[1], db));
    out[kThrottle] = thr * 2.0f - 1.0f;
    out[kYaw] = deadband(d.pad[0], db);
    out[kRoll] = deadband(d.pad[2], db);
    out[kPitch] = deadband(-d.pad[3], db);
    bool armBtn = (d.padButtons & 1u) != 0;  // SDL_GAMEPAD_BUTTON_SOUTH
    if (armBtn && !m_padArmPrev) m_padArmed = !m_padArmed;
    m_padArmPrev = armBtn;
    bool turtleBtn = (d.padButtons & 8u) != 0;  // SDL_GAMEPAD_BUTTON_NORTH
    if (turtleBtn && !m_padTurtlePrev) m_padTurtle = !m_padTurtle;
    m_padTurtlePrev = turtleBtn;
    out[kAux1] = m_padArmed ? 1.0f : -1.0f;
    out[kAux2] = 1.0f;
    out[kAux3] = (d.padButtons & 2u) ? 1.0f : -1.0f;  // EAST = beeper
    out[kAux4] = m_padTurtle ? 1.0f : -1.0f;
}

// Keyboard (degraded): W/S throttle, A/D yaw, arrows pitch/roll, Enter arms,
// T toggles turtle, ANGLE mode on.
void InputManager::mapKeyboard(float dt, float out[kChannelCount]) {
    auto k = [&](int code) { return m_keys[code]; };
    float thrRate = 1.2f;
    if (k('W')) m_kbThrottle += thrRate * dt;
    if (k('S')) m_kbThrottle -= thrRate * dt;
    m_kbThrottle = std::clamp(m_kbThrottle, -1.0f, 1.0f);
    out[kThrottle] = m_kbThrottle;
    out[kYaw] = (k('D') ? 1.0f : 0.0f) - (k('A') ? 1.0f : 0.0f);
    out[kRoll] = (k(FPV_KEY_RIGHT) ? 0.6f : 0.0f) - (k(FPV_KEY_LEFT) ? 0.6f : 0.0f);
    out[kPitch] = (k(FPV_KEY_UP) ? 0.6f : 0.0f) - (k(FPV_KEY_DOWN) ? 0.6f : 0.0f);
    bool armKey = k(FPV_KEY_ENTER);
    if (armKey && !m_kbArmKeyPrev) {
        m_kbArmed = !m_kbArmed;
        if (m_kbArmed) m_kbThrottle = -1.0f;
    }
    m_kbArmKeyPrev = armKey;
    bool turtleKey = k('T');
    if (turtleKey && !m_kbTurtleKeyPrev) m_kbTurtle = !m_kbTurtle;
    m_kbTurtleKeyPrev = turtleKey;
    out[kAux1] = m_kbArmed ? 1.0f : -1.0f;
    out[kAux2] = 1.0f;
    out[kAux3] = k('B') ? 1.0f : -1.0f;
    out[kAux4] = m_kbTurtle ? 1.0f : -1.0f;
}

void InputManager::update(double wallTime, float dt, float out[kChannelCount], bool &radioLost) {
    std::lock_guard<std::mutex> lock(m_mutex);
    radioLost = false;
    if (m_backend) m_backend->poll(m_devices);

    InputStatus st;
    st.deviceCount = (int)m_devices.size();
    float ch[kChannelCount] = {0, 0, -1, 0, -1, -1, -1, -1, -1, -1};

    // Host channels win while fresh.
    if (wallTime - m_hostTime < 0.5) {
        std::copy(m_host, m_host + kChannelCount, ch);
        st.source = InputSource::Host;
        st.deviceName = "host";
    } else {
        // A mapped radio, then any unmapped radio (needs setup), then a gamepad.
        const RawDevice *radio = nullptr;
        const DeviceMapping *radioMap = nullptr;
        const RawDevice *unmapped = nullptr;
        const RawDevice *pad = nullptr;
        for (const RawDevice &d : m_devices) {
            const DeviceMapping *m = nullptr;
            for (auto &mm : m_mappings) {
                if (mm.guid == d.guid && mm.complete) m = &mm;
            }
            if (m && !radio) {
                radio = &d;
                radioMap = m;
            } else if (!m && d.looksLikeRadio && !unmapped) {
                unmapped = &d;
            } else if (!m && d.isGamepad && !pad) {
                pad = &d;
            }
        }
        if (radio) {
            mapRadio(*radio, *radioMap, ch);
            st.source = InputSource::Radio;
            st.deviceName = radio->name;
            m_activeGuid = radio->guid;
            m_radioWasActive = true;
        } else {
            if (m_radioWasActive) {
                // The radio that was flying disappeared: let Betaflight
                // failsafe, exactly like a real receiver losing link.
                radioLost = true;
                std::copy(m_lastOut, m_lastOut + kChannelCount, ch);
                st.source = InputSource::Radio;
                st.deviceName = "radio disconnected";
            } else if (pad) {
                mapGamepad(*pad, dt, ch);
                st.source = InputSource::Gamepad;
                st.deviceName = pad->name;
            } else if (m_keyboardEnabled) {
                mapKeyboard(dt, ch);
                st.source = InputSource::Keyboard;
                st.deviceName = "keyboard";
            }
            if (unmapped) {
                st.needsSetup = true;
                if (st.source != InputSource::Gamepad && st.source != InputSource::Radio) st.deviceName = unmapped->name;
            }
        }
    }
    if (m_wizardActive && st.source != InputSource::Host) {
        // hold idle and disarmed while the player wiggles sticks in the wizard
        ch[kRoll] = ch[kPitch] = ch[kYaw] = 0;
        ch[kThrottle] = -1;
        ch[kAux1] = -1;
    }
    std::copy(ch, ch + kChannelCount, out);
    if (!radioLost) std::copy(ch, ch + kChannelCount, m_lastOut);
    m_status = st;
}

void InputManager::resetRadioLatch() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_radioWasActive = false;
}

InputStatus InputManager::status() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_status;
}

std::vector<RawDevice> InputManager::devicesSnapshot() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_devices;
}

}  // namespace fpv
