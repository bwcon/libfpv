// libfpv - input manager: radios, gamepads, keyboard fallback and host
// channels, mapped to Betaflight's AETR + AUX1..6 (AUX1..8 in kamikaze builds).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace fpv {

enum class InputSource { None = 0, Radio = 1, Gamepad = 2, Keyboard = 3, Host = 4 };

// Channel indices as Betaflight sees them (default map AETR1234).
enum Channel { kRoll = 0, kPitch = 1, kThrottle = 2, kYaw = 3, kAux1 = 4, kAux2 = 5, kAux3 = 6, kAux4 = 7, kAux5 = 8, kAux6 = 9 };
#if defined(LIBFPV_KAMIKAZE)
// kamikaze builds: bomb arm switch on AUX7, explode switch on AUX8
constexpr int kAux7 = 10, kAux8 = 11;
constexpr int kChannelCount = 12;
#else
constexpr int kChannelCount = 10;
#endif

// Channel value at rest: sticks centred, throttle idle, every switch off.
constexpr float idleChannel(int c) { return (c == kThrottle || c >= kAux1) ? -1.0f : 0.0f; }
inline void setIdleChannels(float *ch) {
    for (int c = 0; c < kChannelCount; c++) ch[c] = idleChannel(c);
}

// Mapping of one raw device axis/button to a channel.
struct ChannelBinding {
    int axis = -1;          // raw axis index, -1 = none
    bool inverted = false;
    float min = -1, center = 0, max = 1;  // calibrated raw endpoints
    // switches can also be bound to buttons: value = +1 while pressed
    int button = -1;
    // AUX switches on an axis are positions: "on" at one raw value, "off" at
    // rest. Several switches can share one multi-position switch.
    float switchOn() const { return inverted ? min : max; }
    float switchOff() const { return inverted ? max : min; }
};

struct DeviceMapping {
    std::string guid;
    std::string name;
    std::array<ChannelBinding, kChannelCount> channels{};
    bool complete = false;
};

// AUX channel `c` (bound to an axis) for the raw axis value: +1 when the
// switch is nearer its "on" position than half the way to any other known
// position of that axis (rest, or another switch's "on"), else -1.
inline float switchPosition(const DeviceMapping &m, int c, float raw) {
    const ChannelBinding &b = m.channels[c];
    float on = b.switchOn();
    float nearest = std::fabs(b.switchOff() - on);
    for (int o = kAux1; o < kChannelCount; o++) {
        const ChannelBinding &ob = m.channels[o];
        if (o == c || ob.axis != b.axis) continue;
        float d = std::fabs(ob.switchOn() - on);
        if (d > 1e-3f) nearest = std::min(nearest, d);
    }
    return std::fabs(raw - on) < 0.5f * nearest ? 1.0f : -1.0f;
}

// Raw state of one connected device, produced by the backend.
struct RawDevice {
    uint32_t instanceId = 0;
    std::string guid;
    std::string name;
    bool isGamepad = false;   // recognised by SDL as a gamepad (fixed layout)
    bool looksLikeRadio = false;
    std::vector<float> axes;  // -1..1
    std::vector<uint8_t> buttons;
    // gamepad layout (when isGamepad): leftX, leftY, rightX, rightY, lt, rt
    float pad[6] = {0, 0, 0, 0, 0, 0};
    uint32_t padButtons = 0;  // bit per SDL_GamepadButton
};

class DeviceBackend {
public:
    virtual ~DeviceBackend() = default;
    virtual bool init(std::string &error) = 0;
    virtual void shutdown() = 0;
    // Polls devices; called at the simulation rate on the simulation thread.
    virtual void poll(std::vector<RawDevice> &devices) = 0;
};

std::unique_ptr<DeviceBackend> createSdlBackend();

// What the keyboard's hover hold needs to know about the drone, taken from
// the previous simulation step.
struct HoverFeedback {
    std::string preset;     // the learned hover throttle is kept per preset
    float climbRate = 0;    // m/s, up positive
    float upright = 1;      // cosine of the tilt from level (< 0 upside down)
    bool airborne = false;  // a drone exists and nothing touches the ground
};

// Keyboard throttle and yaw exponent defaults (1/s), see mapKeyboard.
constexpr float kDefaultKeyboardThrottleExpo = 3.0f;
constexpr float kDefaultKeyboardYawExpo = 4.0f;

struct InputStatus {
    InputSource source = InputSource::None;
    std::string deviceName;
    bool needsSetup = false;
    int deviceCount = 0;
};

class InputManager {
public:
    InputManager();
    ~InputManager();

    void setDataDir(const std::string &dir);
    bool startDevices(std::string &error);  // optional; without it only host/keyboard input
    void stopDevices();

    // --- called from any thread -----------------------------------------
    void setHostChannels(const float *ch, uint32_t count, double wallTime);
    void clearHostChannels();
    void key(int key, bool down);
    void setKeyboardEnabled(bool enabled) { m_keyboardEnabled = enabled; }
    // GTA-style hover throttle (see mapKeyboard); off = W/S move the throttle.
    void setKeyboardHover(bool on) { m_kbHover = on; }
    // How fast W raises the throttle: it grows by e^(expo * seconds held).
    void setKeyboardThrottleExpo(float expo) { m_kbThrottleExpo = expo; }
    // How fast A/D raise the yaw, the same way from a small start.
    void setKeyboardYawExpo(float expo) { m_kbYawExpo = expo; }

    // --- simulation thread ----------------------------------------------
    void setHoverFeedback(const HoverFeedback &f);

    // Polls devices and produces Betaflight channels. `radioLost` is true
    // when the active radio disappeared while it was the input source.
    void update(double wallTime, float dt, float out[kChannelCount], bool &radioLost);

    // After the radio vanished, failsafe holds until Betaflight disarmed;
    // then other sources (gamepad, keyboard) may take over.
    void resetRadioLatch();

    InputStatus status();
    std::vector<RawDevice> devicesSnapshot();
    // Mapping access for the setup wizard (any thread).
    bool getMapping(const std::string &guid, DeviceMapping &out);
    void setMapping(const DeviceMapping &m);  // persists to data dir
    void forgetMapping(const std::string &guid);
    // While the wizard is reading raw sticks, flight input is held at idle.
    void setWizardActive(bool active) { m_wizardActive = active; }

private:
    void loadMappings();
    void saveMappings();
    bool mapRadio(const RawDevice &d, const DeviceMapping &m, float out[kChannelCount]);
    void mapGamepad(const RawDevice &d, float dt, float out[kChannelCount]);
    void mapKeyboard(float dt, float out[kChannelCount]);

    std::mutex m_mutex;
    std::string m_dataDir;
    std::unique_ptr<DeviceBackend> m_backend;
    std::vector<RawDevice> m_devices;
    std::vector<DeviceMapping> m_mappings;

    float m_host[kChannelCount];
    double m_hostTime = -1e9;
    std::array<bool, 512> m_keys{};
    std::atomic<bool> m_keyboardEnabled{true};
    std::atomic<bool> m_wizardActive{false};
    std::atomic<bool> m_kbHover{false};
    std::atomic<float> m_kbThrottleExpo{kDefaultKeyboardThrottleExpo};
    std::atomic<float> m_kbYawExpo{kDefaultKeyboardYawExpo};

    // fallback state
    float m_kbThrottle = 0;  // 0..1
    float m_kbYaw = 0;
    HoverFeedback m_hover;
    std::unordered_map<std::string, float> m_hoverThrottle;  // learned, per preset
    bool m_kbArmed = false, m_kbArmKeyPrev = false;
    bool m_kbTurtle = false, m_kbTurtleKeyPrev = false;
    bool m_padArmed = false, m_padArmPrev = false;
    bool m_padTurtle = false, m_padTurtlePrev = false;
#if defined(LIBFPV_KAMIKAZE)
    bool m_kbBomb = false, m_kbBombKeyPrev = false;
    bool m_padBomb = false, m_padBombPrev = false;
#endif
    float m_lastOut[kChannelCount];

    InputStatus m_status;
    std::string m_activeGuid;
    bool m_radioWasActive = false;
};

}  // namespace fpv
