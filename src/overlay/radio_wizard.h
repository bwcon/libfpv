// libfpv - guided radio setup: move each stick, the wizard finds the axis,
// its direction and endpoints, then the switches for ARM and modes.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

#include "input/input.h"

namespace fpv {

class RadioWizard {
public:
    enum class Step {
        PickDevice,
        Rest,
        Throttle,
        Yaw,
        Pitch,
        Roll,
        Calibrate,
        ArmSwitch,
        AngleSwitch,
        BeeperSwitch,
        TurtleSwitch,
        AirModeSwitch,
        OsdSwitch,
        Done
    };

    void start(const RawDevice &dev);
    void cancel() { m_active = false; }
    bool active() const { return m_active; }
    Step step() const { return m_step; }
    const std::string &guid() const { return m_mapping.guid; }

    // Feeds the latest raw state of the device being set up.
    void update(const RawDevice &dev);
    // "Next" button; returns false with `hint` set when it cannot proceed.
    bool next(std::string &hint);
    void skip();  // optional switches only
    void back();

    const char *title() const;
    const char *instruction() const;
    bool canSkip() const { return m_step >= Step::AngleSwitch && m_step <= Step::OsdSwitch; }
    // Live detection feedback for the UI.
    bool detected() const { return m_candidate >= 0; }
    std::string detectedLabel() const;  // e.g. "stick 3, reversed"
    float progress() const;             // 0..1 over the steps

    const DeviceMapping &mapping() const { return m_mapping; }

private:
    struct Candidate {
        int axis = -1, button = -1;
        float value = 0;  // raw value at the detected position
    };
    void detectAxis(const RawDevice &dev, bool allowButtons);
    bool axisUsed(int axis) const;
    bool stickAxisUsed(int axis) const;
    bool positionFree(int axis, float value) const;
    int sharedWith(int axis) const;  // first AUX channel on this axis, or -1
    bool buttonUsed(int button) const;
    int channelForStep(Step s) const;

    bool m_active = false;
    Step m_step = Step::PickDevice;
    DeviceMapping m_mapping;
    std::vector<float> m_rest;
    std::vector<uint8_t> m_restButtons;
    std::vector<float> m_min, m_max;
    int m_candidate = -1;  // axis index, or 1000 + button
    float m_candidateValue = 0;
};

}  // namespace fpv
