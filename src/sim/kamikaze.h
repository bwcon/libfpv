// libfpv - kamikaze builds (LIBFPV_KAMIKAZE): the warhead carried by the
// drone. Bomb arm switch on AUX7, explode switch on AUX8. The flight side
// runs on the simulation thread; detonations are queued for the host.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

#include "fpv_math.h"
#include "input/input.h"

namespace fpv {

enum class WarheadState { Safe = 0, Arming = 1, Armed = 2, Detonated = 3 };
enum class DetonationCause { Impact = 0, Switch = 1, Host = 2 };

struct WarheadConfig {
    float armingDelay = 0.0f;    // seconds from switch on (motors armed) to live
    float minImpactSpeed = 0.25f;  // m/s, the contact fuse threshold (0.1..30)
    bool osd = true;             // the state in Betaflight's custom message 1 element
};

// Internal frame (metres, X right, Y up, Z forward).
struct DetonationEvent {
    double simTime = 0;
    DetonationCause cause = DetonationCause::Impact;
    Vec3 position, velocity, normal;  // normal is zero unless it was an impact
    uint8_t material = 0;
    uint32_t objectId = 0;
    int motorIndex = -1;
};

struct WarheadSnapshot {
    bool installed = false;      // the drone carries the bomb (an overlay/host choice, per session)
    WarheadState state = WarheadState::Safe;
    float armingRemaining = 0;
    bool bombSwitch = false, explodeSwitch = false;
    bool blocksArming = false;   // the motors cannot arm (bomb switch on, or destroyed)
    bool needsCycle = false;     // the bomb switch must go off before it can arm
    double refusedUntil = -1;    // explode switch flipped while not armed: "NOT ARMED" until then
    double stateSince = 0;       // sim time of the last state change
    uint32_t detonations = 0;
};

class Warhead {
public:
    // Simulation thread -------------------------------------------------
    // Before Betaflight sees the channels: holds AUX1 (arm) low while the
    // motors must not arm, until the pilot cycles the arm switch, like
    // Betaflight's own ARMSWITCH safety.
    void filterChannels(float ch[kChannelCount], bool fcArmed);
    // After the flight step: switch logic, arming countdown, explode switch.
    // Returns true when it detonated in this step.
    bool update(float dt, double simTime, const float ch[kChannelCount], bool fcArmed, Vec3 pos, Vec3 vel);
    // A contact from the collision solver; detonates a live warhead.
    bool impact(double simTime, Vec3 pos, Vec3 vel, Vec3 point, Vec3 normal, float speed, uint8_t material,
                uint32_t objectId, int motor);
    // Host request (fpv_kamikaze_detonate). Returns false if not detonated.
    bool detonate(double simTime, Vec3 pos, Vec3 vel, bool evenIfSafe);
    // A new drone or a repaired one: fresh warhead, safe; the bomb switch
    // must be seen off before it can arm again.
    void reset(double simTime);
    // Installs or removes the bomb. Without it the bomb switches do nothing;
    // a destroyed drone stays destroyed until repaired.
    void setInstalled(bool on, double simTime);
    // The state line of the OSD ("SAFE", "ARMING 3", "STEADY"...), "" when
    // there is nothing to show (no bomb, blink off, destroyed).
    void statusText(char *out, size_t size, double simTime, bool fcArmed);

    bool installed() const { return m_snap.installed; }
    bool detonated() const { return m_snap.state == WarheadState::Detonated; }
    const WarheadSnapshot &snapshot() const { return m_snap; }

    // Any thread ------------------------------------------------------------
    void setConfig(const WarheadConfig &c);
    WarheadConfig config();
    bool pollDetonation(DetonationEvent &out);

private:
    void setState(WarheadState s, double simTime);
    void fire(const DetonationEvent &e);

    std::mutex m_mutex;  // config and the event queue
    WarheadConfig m_config;
    std::deque<DetonationEvent> m_events;

    WarheadSnapshot m_snap;
    bool m_armHold = false;      // AUX1 held low until the arm switch goes off
    bool m_explodePrev = true;   // edge detection; true = must be seen off first
};

}  // namespace fpv
