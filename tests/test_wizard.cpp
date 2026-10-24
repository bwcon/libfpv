// libfpv - radio setup wizard unit test: drives the wizard with a fake radio
// and checks the resulting switch mapping. No Betaflight, no SDL.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <string>

#include "input/input.h"
#include "overlay/radio_wizard.h"

using namespace fpv;

namespace {

int g_failures = 0, g_checks = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_failures++;                                                   \
            std::printf("  FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
            std::printf(__VA_ARGS__);                                       \
            std::printf("\n");                                              \
        }                                                                   \
    } while (0)

// Axes: 0 roll, 1 pitch, 2 throttle, 3 yaw, 4 arm (2-pos), 5 angle (2-pos),
// 6 beeper + turtle (3-pos: -1 off, 0 beeper, +1 turtle),
// 7 air mode + OSD disable (3-pos: -1 off, 0 air mode, +1 OSD off),
// kamikaze builds: 8 bomb arm (2-pos), button 0 explode (momentary), set up
// in their own wizard (startBombSwitches).
struct FakeRadio {
    RawDevice d;
    FakeRadio() {
        d.guid = "fake";
        d.name = "Fake radio";
        d.axes = {0, 0, -1, 0, -1, -1, -1, -1, -1};
        d.buttons = {0, 0};
    }
};

// Feeds the device to the wizard, then presses Next.
bool step(RadioWizard &w, FakeRadio &r) {
    w.update(r.d);
    std::string hint;
    bool ok = w.next(hint);
    if (!ok) std::printf("    next refused at \"%s\": %s\n", w.title(), hint.c_str());
    return ok;
}

void testSharedThreePositionSwitch() {
    FakeRadio r;
    RadioWizard w;
    w.start(r.d);
    CHECK(step(w, r), "rest");
    r.d.axes[2] = 1;
    CHECK(step(w, r), "throttle");
    r.d.axes[2] = -1;
    r.d.axes[3] = 1;
    CHECK(step(w, r), "yaw");
    r.d.axes[3] = 0;
    r.d.axes[1] = 1;
    CHECK(step(w, r), "pitch");
    r.d.axes[1] = 0;
    r.d.axes[0] = 1;
    CHECK(step(w, r), "roll");
    // calibration sweep
    for (float v : {-1.0f, 1.0f, 0.0f}) {
        for (int a = 0; a < 4; a++) r.d.axes[a] = v;
        w.update(r.d);
    }
    r.d.axes[0] = r.d.axes[1] = r.d.axes[3] = 0;
    r.d.axes[2] = -1;
    CHECK(step(w, r), "calibrate");
    r.d.axes[4] = 1;
    CHECK(step(w, r), "arm");
    r.d.axes[5] = 1;
    CHECK(step(w, r), "angle");
    r.d.axes[6] = 0;  // 3-pos switch to the middle: beeper
    CHECK(step(w, r), "beeper");
    // still in the middle: the wizard must not take the beeper position again
    w.update(r.d);
    CHECK(!w.detected(), "beeper's position is not offered for turtle");
    r.d.axes[6] = 1;  // all the way up: turtle
    w.update(r.d);
    CHECK(w.detected(), "same switch in another position is detected");
    std::printf("    turtle detected as: %s\n", w.detectedLabel().c_str());
    CHECK(step(w, r), "turtle");
    CHECK(w.step() == RadioWizard::Step::AirModeSwitch, "air mode step after turtle");
    r.d.axes[7] = 0;  // second 3-pos switch to the middle: air mode
    CHECK(step(w, r), "air mode");
    w.update(r.d);
    CHECK(!w.detected(), "air mode's position is not offered for OSD");
    r.d.axes[7] = 1;  // all the way up: OSD disable
    CHECK(step(w, r), "OSD disable");
    // the full setup never asks for the bomb switches
    CHECK(w.step() == RadioWizard::Step::Done, "wizard done after OSD disable");

    const DeviceMapping &m = w.mapping();
    CHECK(m.channels[kAux3].axis == 6 && m.channels[kAux4].axis == 6, "beeper and turtle share axis 6: %d %d",
          m.channels[kAux3].axis, m.channels[kAux4].axis);
    struct {
        float raw, beeper, turtle;
    } cases[] = {{-1, -1, -1}, {0, 1, -1}, {1, -1, 1}, {-0.1f, 1, -1}, {0.9f, -1, 1}};
    for (auto &c : cases) {
        float b = switchPosition(m, kAux3, c.raw), t = switchPosition(m, kAux4, c.raw);
        CHECK(b == c.beeper && t == c.turtle, "raw %.1f: beeper %.0f (want %.0f), turtle %.0f (want %.0f)", c.raw, b,
              c.beeper, t, c.turtle);
    }
    CHECK(m.channels[kAux5].axis == 7 && m.channels[kAux6].axis == 7, "air mode and OSD share axis 7: %d %d",
          m.channels[kAux5].axis, m.channels[kAux6].axis);
    for (auto &c : cases) {
        float a = switchPosition(m, kAux5, c.raw), o = switchPosition(m, kAux6, c.raw);
        CHECK(a == c.beeper && o == c.turtle, "raw %.1f: air mode %.0f (want %.0f), OSD off %.0f (want %.0f)", c.raw,
              a, c.beeper, o, c.turtle);
    }
    // plain 2-position switches still work
    CHECK(switchPosition(m, kAux1, 1) == 1 && switchPosition(m, kAux1, -1) == -1, "arm switch");
    CHECK(switchPosition(m, kAux2, 0.8f) == 1 && switchPosition(m, kAux2, -0.8f) == -1, "angle switch");
#if defined(LIBFPV_KAMIKAZE)
    CHECK(m.channels[kAux7].axis < 0 && m.channels[kAux8].button < 0, "no bomb switches in the full setup");

    // the bomb switches' own setup, on top of that mapping
    r.d.axes = {0, 0, -1, 0, -1, -1, -1, -1, -1};
    RadioWizard b;
    b.startBombSwitches(r.d, m);
    CHECK(b.bombSwitchesOnly() && b.step() == RadioWizard::Step::Rest, "bomb setup starts at rest");
    CHECK(step(b, r), "rest");
    CHECK(b.step() == RadioWizard::Step::BombArmSwitch, "straight to the bomb arm switch");
    r.d.axes[6] = 1;  // the turtle position: taken
    b.update(r.d);
    CHECK(!b.detected(), "a position the radio setup uses is not offered");
    r.d.axes[6] = -1;
    r.d.axes[8] = 1;
    CHECK(step(b, r), "bomb arm");
    r.d.buttons[0] = 1;  // momentary explode button, held while pressing Next
    CHECK(step(b, r), "explode");
    r.d.buttons[0] = 0;
    CHECK(b.step() == RadioWizard::Step::Done, "bomb setup done");
    DeviceMapping merged = m;
    RadioWizard::mergeBombSwitches(merged, b.mapping(), false);
    CHECK(merged.channels[kAux7].axis == 8 && switchPosition(merged, kAux7, 1) == 1 &&
              switchPosition(merged, kAux7, -1) == -1,
          "bomb arm switch on axis 8 (axis %d)", merged.channels[kAux7].axis);
    CHECK(merged.channels[kAux8].button == 0, "explode on button 1 (button %d)", merged.channels[kAux8].button);
    bool same = merged.complete == m.complete;
    for (int c = 0; c < kAux7; c++)
        same = same && merged.channels[c].axis == m.channels[c].axis &&
               merged.channels[c].button == m.channels[c].button && merged.channels[c].min == m.channels[c].min &&
               merged.channels[c].max == m.channels[c].max && merged.channels[c].inverted == m.channels[c].inverted;
    CHECK(same, "the rest of the radio setup is untouched");

    // a new full setup keeps the bomb switches that do not clash with it
    DeviceMapping fresh = m;
    fresh.channels[kAux2] = ChannelBinding{};
    fresh.channels[kAux2].button = 0;  // angle now on the explode button
    RadioWizard::mergeBombSwitches(fresh, merged, true);
    CHECK(fresh.channels[kAux7].axis == 8, "bomb arm kept (axis %d)", fresh.channels[kAux7].axis);
    CHECK(fresh.channels[kAux8].button < 0 && fresh.channels[kAux8].axis < 0, "explode dropped: its button is taken");
    DeviceMapping stick = m;
    stick.channels[kRoll].axis = 8;
    RadioWizard::mergeBombSwitches(stick, merged, true);
    CHECK(stick.channels[kAux7].axis < 0, "bomb arm dropped: its axis is a stick now");
#endif
}

// A middle-only mapping on a 3-position switch is off at the top.
void testMiddleOnly() {
    DeviceMapping m;
    ChannelBinding &b = m.channels[kAux3];
    b.axis = 6;
    b.inverted = false;
    b.min = -1;  // rest
    b.max = 0;   // on
    CHECK(switchPosition(m, kAux3, 0) == 1, "on in the middle");
    CHECK(switchPosition(m, kAux3, 1) == -1, "off at the top");
    CHECK(switchPosition(m, kAux3, -1) == -1, "off at rest");
}

}  // namespace

// The optional switches can all be skipped; the bomb switches are not part
// of the full setup.
void testSkipOptionalSwitches() {
    FakeRadio r;
    RadioWizard w;
    w.start(r.d);
    step(w, r);
    for (int a : {2, 3, 1, 0}) {
        r.d.axes[a] = 1;
        step(w, r);
        r.d.axes[a] = a == 2 ? -1 : 0;
    }
    for (float v : {-1.0f, 1.0f, 0.0f}) {
        for (int a = 0; a < 4; a++) r.d.axes[a] = v;
        w.update(r.d);
    }
    r.d.axes[0] = r.d.axes[1] = r.d.axes[3] = 0;
    r.d.axes[2] = -1;
    step(w, r);
    r.d.axes[4] = 1;
    CHECK(step(w, r), "arm");
    int skipped = 0;
    while (w.canSkip()) {
        w.skip();
        skipped++;
    }
    CHECK(skipped == 5, "angle, beeper, turtle, air mode and OSD are optional (%d skipped)", skipped);
    CHECK(w.step() == RadioWizard::Step::Done && w.mapping().complete, "done after skipping");
    for (int c = kAux2; c < kChannelCount; c++)
        CHECK(w.mapping().channels[c].axis < 0 && w.mapping().channels[c].button < 0, "AUX%d unbound", c - kAux1 + 1);
    w.back();
    CHECK(w.step() == RadioWizard::Step::OsdSwitch, "back from done returns to OSD");
#if defined(LIBFPV_KAMIKAZE)
    // the bomb switches' setup: both optional, back from the first one is rest
    RadioWizard b;
    b.startBombSwitches(r.d, w.mapping());
    step(b, r);
    CHECK(b.canSkip(), "bomb arm optional");
    b.back();
    CHECK(b.step() == RadioWizard::Step::Rest, "back from bomb arm returns to rest");
    step(b, r);
    b.skip();
    CHECK(b.step() == RadioWizard::Step::ExplodeSwitch && b.canSkip(), "explode optional");
    b.skip();
    CHECK(b.step() == RadioWizard::Step::Done, "bomb setup done after skipping");
#endif
}

int main() {
    testSharedThreePositionSwitch();
    testSkipOptionalSwitches();
    testMiddleOnly();
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
