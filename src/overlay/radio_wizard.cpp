// libfpv - guided radio setup.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "overlay/radio_wizard.h"

#include <algorithm>
#include <cmath>

namespace fpv {

namespace {
constexpr float kMoveThreshold = 0.35f;  // raw units (-1..1) a stick must move to count
}

void RadioWizard::start(const RawDevice &dev, bool bombSwitches) {
    m_active = true;
    m_bombSwitches = bombSwitches;
    m_step = Step::Rest;
    m_mapping = DeviceMapping{};
    m_mapping.guid = dev.guid;
    m_mapping.name = dev.name;
    m_rest = dev.axes;
    m_restButtons = dev.buttons;
    m_min = dev.axes;
    m_max = dev.axes;
    m_candidate = -1;
}

int RadioWizard::channelForStep(Step s) const {
    switch (s) {
    case Step::Throttle: return kThrottle;
    case Step::Yaw: return kYaw;
    case Step::Pitch: return kPitch;
    case Step::Roll: return kRoll;
    case Step::ArmSwitch: return kAux1;
    case Step::AngleSwitch: return kAux2;
    case Step::BeeperSwitch: return kAux3;
    case Step::TurtleSwitch: return kAux4;
    case Step::AirModeSwitch: return kAux5;
    case Step::OsdSwitch: return kAux6;
#if defined(LIBFPV_KAMIKAZE)
    case Step::BombArmSwitch: return kAux7;
    case Step::ExplodeSwitch: return kAux8;
#endif
    default: return -1;
    }
}

RadioWizard::Step RadioWizard::lastSwitch() const {
#if defined(LIBFPV_KAMIKAZE)
    if (m_bombSwitches) return Step::ExplodeSwitch;
#endif
    return Step::OsdSwitch;
}

bool RadioWizard::axisUsed(int axis) const {
    for (auto &b : m_mapping.channels)
        if (b.axis == axis) return true;
    return false;
}

bool RadioWizard::stickAxisUsed(int axis) const {
    for (int c = kRoll; c <= kYaw; c++)
        if (m_mapping.channels[c].axis == axis) return true;
    return false;
}

// A switch may share an axis with switches mapped earlier (a 3-position
// switch: beeper in the middle, turtle at the top), but only in a position
// none of them uses.
bool RadioWizard::positionFree(int axis, float value) const {
    for (int c = kAux1; c < kChannelCount; c++) {
        const ChannelBinding &b = m_mapping.channels[c];
        if (b.axis == axis && std::fabs(value - b.switchOn()) < kMoveThreshold) return false;
    }
    return true;
}

int RadioWizard::sharedWith(int axis) const {
    for (int c = kAux1; c < kChannelCount; c++)
        if (m_mapping.channels[c].axis == axis) return c;
    return -1;
}

bool RadioWizard::buttonUsed(int button) const {
    for (auto &b : m_mapping.channels)
        if (b.button == button) return true;
    return false;
}

// The axis (or button) that moved furthest from its rest position and is
// not assigned yet. Switches may reuse a switch axis in a new position.
void RadioWizard::detectAxis(const RawDevice &dev, bool allowButtons) {
    m_candidate = -1;
    float best = kMoveThreshold;
    for (size_t i = 0; i < dev.axes.size() && i < m_rest.size(); i++) {
        if (allowButtons ? (stickAxisUsed((int)i) || !positionFree((int)i, dev.axes[i])) : axisUsed((int)i)) continue;
        float d = std::fabs(dev.axes[i] - m_rest[i]);
        if (d > best) {
            best = d;
            m_candidate = (int)i;
            m_candidateValue = dev.axes[i];
        }
    }
    if (m_candidate < 0 && allowButtons) {
        for (size_t i = 0; i < dev.buttons.size() && i < m_restButtons.size(); i++) {
            if (buttonUsed((int)i)) continue;
            if (dev.buttons[i] != m_restButtons[i]) {
                m_candidate = 1000 + (int)i;
                m_candidateValue = dev.buttons[i] ? 1.0f : 0.0f;
                break;
            }
        }
    }
}

void RadioWizard::update(const RawDevice &dev) {
    if (!m_active) return;
    if (m_step == Step::Rest) {
        m_rest = dev.axes;
        m_restButtons = dev.buttons;
        m_min = m_max = dev.axes;
        return;
    }
    if (m_step == Step::Calibrate) {
        for (size_t i = 0; i < dev.axes.size() && i < m_min.size(); i++) {
            m_min[i] = std::min(m_min[i], dev.axes[i]);
            m_max[i] = std::max(m_max[i], dev.axes[i]);
        }
        return;
    }
    if (channelForStep(m_step) >= 0) {
        bool sw = m_step >= Step::ArmSwitch;
        detectAxis(dev, sw);
        // keep endpoints up to date for sticks while they are moved
        for (size_t i = 0; i < dev.axes.size() && i < m_min.size(); i++) {
            m_min[i] = std::min(m_min[i], dev.axes[i]);
            m_max[i] = std::max(m_max[i], dev.axes[i]);
        }
    }
}

bool RadioWizard::next(std::string &hint) {
    hint.clear();
    switch (m_step) {
    case Step::PickDevice:
        return false;
    case Step::Rest:
        if (m_rest.size() < 4) {
            hint = "This device has fewer than 4 axes; it cannot be used as a radio.";
            return false;
        }
        m_step = Step::Throttle;
        return true;
    case Step::Calibrate: {
        // Sticks: endpoints from the calibration sweep; centre from rest
        // (throttle has no centre: use the middle of its travel).
        for (int c : {kRoll, kPitch, kYaw, kThrottle}) {
            ChannelBinding &b = m_mapping.channels[c];
            if (b.axis < 0) continue;
            float lo = m_min[b.axis], hi = m_max[b.axis];
            if (hi - lo < 0.5f) {
                hint = "Move every stick all the way to its corners, then press Next.";
                return false;
            }
            b.min = lo;
            b.max = hi;
            b.center = (c == kThrottle) ? 0.5f * (lo + hi) : m_rest[b.axis];
        }
        m_step = Step::ArmSwitch;
        m_candidate = -1;
        return true;
    }
    case Step::Done:
        return false;
    default:
        break;
    }
    int ch = channelForStep(m_step);
    if (ch < 0) return false;
    if (m_candidate < 0) {
        hint = m_step >= Step::ArmSwitch ? "Flip the switch first (the wizard has not seen it move yet)."
                                         : "Move the stick as shown and hold it there.";
        return false;
    }
    ChannelBinding &b = m_mapping.channels[ch];
    b = ChannelBinding{};
    if (m_candidate >= 1000) {
        b.button = m_candidate - 1000;
        // the switch is "on" in its current (flipped) position
        b.inverted = m_candidateValue < 0.5f;
    } else {
        b.axis = m_candidate;
        float rest = m_rest[b.axis];
        bool positive = m_candidateValue > rest;
        if (m_step >= Step::ArmSwitch) {
            // switch: "on" where it is now; endpoints are rest and now
            b.inverted = !positive;
            b.min = std::min(rest, m_candidateValue);
            b.max = std::max(rest, m_candidateValue);
            b.center = 0.5f * (b.min + b.max);
        } else {
            // stick: requested direction (up/right) must read +1
            b.inverted = !positive;
            b.min = -1;
            b.max = 1;
            b.center = rest;
        }
    }
    m_candidate = -1;
    switch (m_step) {
    case Step::Throttle: m_step = Step::Yaw; break;
    case Step::Yaw: m_step = Step::Pitch; break;
    case Step::Pitch: m_step = Step::Roll; break;
    case Step::Roll: m_step = Step::Calibrate; break;
    case Step::ArmSwitch: m_step = Step::AngleSwitch; break;
    case Step::AngleSwitch: m_step = Step::BeeperSwitch; break;
    case Step::BeeperSwitch: m_step = Step::TurtleSwitch; break;
    case Step::TurtleSwitch: m_step = Step::AirModeSwitch; break;
    case Step::AirModeSwitch: m_step = Step::OsdSwitch; break;
    default:
        if (m_step == lastSwitch()) {
            m_step = Step::Done;
            m_mapping.complete = true;
        } else if (m_step > Step::ArmSwitch && m_step < lastSwitch()) {
            m_step = (Step)((int)m_step + 1);
        }
        break;
    }
    return true;
}

void RadioWizard::skip() {
    if (!canSkip()) return;
    m_mapping.channels[channelForStep(m_step)] = ChannelBinding{};
    m_candidate = -1;
    if (m_step == lastSwitch()) {
        m_step = Step::Done;
        m_mapping.complete = true;
    } else {
        m_step = (Step)((int)m_step + 1);
    }
}

void RadioWizard::back() {
    if (m_step <= Step::Rest) return;
    m_step = m_step == Step::Done ? lastSwitch() : (Step)((int)m_step - 1);
    int ch = channelForStep(m_step);
    if (ch >= 0) m_mapping.channels[ch] = ChannelBinding{};
    m_candidate = -1;
    m_mapping.complete = false;
}

const char *RadioWizard::title() const {
    switch (m_step) {
    case Step::PickDevice: return "Choose your radio";
    case Step::Rest: return "Starting position";
    case Step::Throttle: return "Throttle";
    case Step::Yaw: return "Yaw";
    case Step::Pitch: return "Pitch";
    case Step::Roll: return "Roll";
    case Step::Calibrate: return "Stick endpoints";
    case Step::ArmSwitch: return "Arm switch";
    case Step::AngleSwitch: return "Angle (self-level) switch";
    case Step::BeeperSwitch: return "Beeper switch";
    case Step::TurtleSwitch: return "Turtle (crash flip) switch";
    case Step::AirModeSwitch: return "Air mode switch";
    case Step::OsdSwitch: return "OSD disable switch";
#if defined(LIBFPV_KAMIKAZE)
    case Step::BombArmSwitch: return "Bomb arm switch";
    case Step::ExplodeSwitch: return "Explode switch";
#endif
    case Step::Done: return "All set";
    }
    return "";
}

const char *RadioWizard::instruction() const {
    switch (m_step) {
    case Step::PickDevice: return "Select the radio you want to fly with.";
    case Step::Rest:
        return "Centre the right stick and the yaw, pull the throttle all the way down, and put every switch in "
               "its OFF position. Then press Next.";
    case Step::Throttle: return "Push the THROTTLE stick all the way UP and hold it there.";
    case Step::Yaw: return "Centre the throttle-side stick, then push YAW all the way to the RIGHT and hold it.";
    case Step::Pitch: return "Push PITCH all the way UP (forward) and hold it.";
    case Step::Roll: return "Push ROLL all the way to the RIGHT and hold it.";
    case Step::Calibrate:
        return "Move both sticks around their full range a few times (corners included), then centre them and press "
               "Next.";
    case Step::ArmSwitch: return "Flip the switch you want to ARM with to its ARMED position.";
    case Step::AngleSwitch:
        return "Optional: flip a switch to the position that should turn on ANGLE (self-level) mode, or Skip.";
    case Step::BeeperSwitch:
        return "Optional: flip a switch for the BEEPER (find a crashed drone by sound), or Skip. A 3-position switch "
               "can carry two modes: use one position here and another one for the next switch.";
    case Step::TurtleSwitch:
        return "Optional: flip a switch to the position that should turn on TURTLE mode (flip over after crash), or "
               "Skip. It can be another position of the beeper's switch. To use it upside down: switch on, arm, hold "
               "the stick towards the side to roll over, then switch it off to fly away.";
    case Step::AirModeSwitch:
        return "Optional: flip a switch to the position that should force AIR MODE on (full stick authority at zero "
               "throttle), or Skip. It only matters when the AIRMODE feature is off, as on quads that keep air mode "
               "on a switch. A 3-position switch can carry this and the next switch.";
    case Step::OsdSwitch:
        return "Optional: flip a switch to the position that should DISABLE THE OSD (hide everything drawn over the "
               "video), or Skip. It can be another position of the air mode switch.";
#if defined(LIBFPV_KAMIKAZE)
    case Step::BombArmSwitch:
        return "Flip the switch that should ARM THE BOMB to its armed position. Use a switch of its own, away from "
               "the arm switch. The motors only arm with it off: take off, then flip it; the warhead is live after "
               "the arming delay and explodes on impact. Skip it and the bomb can never be armed.";
    case Step::ExplodeSwitch:
        return "Optional: flip a switch (a momentary one is best) that should EXPLODE the bomb on command, or Skip. "
               "It only works while the bomb is armed and live.";
#endif
    case Step::Done:
        return "Your radio is mapped. Check the bars below, then close this window and fly. Throttle down and flip "
               "the arm switch to arm.";
    }
    return "";
}

std::string RadioWizard::detectedLabel() const {
    if (m_candidate < 0) return "";
    if (m_candidate >= 1000) return "button " + std::to_string(m_candidate - 1000 + 1);
    float rest = m_rest[m_candidate];
    std::string label = "axis " + std::to_string(m_candidate + 1);
    if (m_step >= Step::ArmSwitch) {
        int other = sharedWith(m_candidate);
        if (other >= 0) label += ", same switch as AUX" + std::to_string(other - kAux1 + 1) + " in another position";
        return label;
    }
    return label + (m_candidateValue > rest ? "" : ", reversed");
}

float RadioWizard::progress() const {
    if (m_step == Step::Done) return 1;
    return (float)m_step / (float)((int)lastSwitch() + 1);
}

}  // namespace fpv
