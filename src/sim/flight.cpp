// libfpv - one drone: SimITL flight model + libfpv rigid body and collisions.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "sim/flight.h"

#include <algorithm>
#include <cstring>

#include "core/bf_bridge.h"
#include "sim/bf.h"
#include "sim/sim.h"
#include "simitl.h"
#include "util/log.h"

namespace fpv {

namespace {
constexpr float kImpactReportSpeed = 1.5f;   // m/s, below this a contact is not an "impact"
constexpr float kRestingSpeed = 0.6f;        // m/s, no bounce below this
constexpr float kPropDiscHalfThickness = 0.012f;
constexpr float kPropNutHeight = 0.006f;     // motor top above the prop disc
constexpr float kMaxSpeed = 120.0f;          // m/s, sanity clamp
constexpr float kMaxAngularSpeed = 100.0f;   // rad/s, SimITL's own clamp

Mat3 toSimBasis(Quat q) { return toMat3(q); }

void fillRotation(StateInput &in, Quat q) {
    Mat3 m = toSimBasis(q);
    for (int i = 0; i < 3; i++) in.rotation[i] = {m.r[i].x, m.r[i].y, m.r[i].z};
}
}  // namespace

void Flight::initBetaflight(const std::string &eepromPath, const DronePreset &parkedPreset) {
    if (m_bfReady) return;
    m_preset = parkedPreset;
    StateInit init = parkedPreset.init;
    std::memset(init.eepromName, 0, sizeof(init.eepromName));
    std::strncpy((char *)init.eepromName, eepromPath.c_str(), sizeof(init.eepromName) - 1);
    simitl_init(init);
    m_bfReady = true;

    m_input = StateInput{};
    fillRotation(m_input, Quat{});
    for (float &c : m_input.rcData) c = -1.0f;
    m_input.rcData[0] = m_input.rcData[1] = m_input.rcData[3] = 0.0f;
    m_input.contact = 2;
    buildShape();
}

// The props spin the way the flight controller is configured, like a
// correctly built quad: Betaflight's Quad X default is "props in"
// (motor 1 rear right clockwise); yaw_motors_reversed means "props out".
// SimITL's motor direction is +1 clockwise, -1 counter-clockwise.
void Flight::syncPropDirection() {
    int rev = bfb_yaw_motors_reversed();
    if (rev == m_yawReversed) return;
    m_yawReversed = rev;
    static const float kPropsIn[4] = {1, -1, -1, 1};
    StateInit &live = SimITL::Sim::getInstance().mSimState.stateInit;
    for (int i = 0; i < 4; i++) {
        float d = rev ? -kPropsIn[i] : kPropsIn[i];
        m_preset.init.quadMotorDir[i] = d;
        live.quadMotorDir[i] = d;
    }
}

// Collision shape: a sphere at the centre of mass and one under each motor
// bell. Their bottoms are level, so the drone rests on five points and
// ground friction can stop it from spinning. One more sphere on top of each
// motor (bell and prop nut, just above the prop disc): upside down the
// drone rests on those with its props clear of the ground, and in turtle
// mode it pivots on an arm like a real quad instead of rolling on the body.
void Flight::buildShape() {
    m_shape.clear();
    const CollisionSettings &c = m_preset.collision;
    m_shape.push_back({{0, 0, 0}, c.bodyRadius, -1});
    for (int i = 0; i < 4; i++) {
        const Vec3F &p = m_preset.init.quadMotorPos[i];
        float y = -c.bodyRadius + c.motorRadius;
        m_shape.push_back({{p.x, std::min(p.y, y), p.z}, c.motorRadius, i});
        float top = std::max(c.bodyRadius, p.y + kPropDiscHalfThickness + kPropNutHeight);
        m_shape.push_back({{p.x, top - c.motorRadius, p.z}, c.motorRadius, i});
    }
    m_mass = std::max(0.01f, m_preset.init.quadMass);
    const Vec3F &ii = m_preset.init.quadInvInertia;
    m_invInertiaBody = {std::fabs(ii.x), std::fabs(ii.y), std::fabs(ii.z)};
}

float Flight::restHeight() const {
    float h = 0;
    for (const Sphere &s : m_shape) h = std::max(h, s.radius - s.local.y);
    return h;
}

void Flight::applyPreset(const DronePreset &preset) {
    m_preset = preset;
    StateInit init = preset.init;
    std::memset(init.eepromName, 0, sizeof(init.eepromName));
    simitl_reinit_physics(init);
    buildShape();
    m_yawReversed = -1;
    syncPropDirection();
    for (int i = 0; i < 4; i++) {
        m_input.motorImbalance[i] = {preset.motorImbalance.x, preset.motorImbalance.y, preset.motorImbalance.z};
    }
    m_input.gyroBaseNoiseAmp = preset.gyroBaseNoiseAmp;
    m_input.gyrobaseNoiseFreq = preset.gyrobaseNoiseFreq;
    m_input.frameHarmonic1Amp = preset.frameHarmonic1Amp;
    m_input.frameHarmonic1Freq = preset.frameHarmonic1Freq;
    m_input.frameHarmonic2Amp = preset.frameHarmonic2Amp;
    m_input.frameHarmonic2Freq = preset.frameHarmonic2Freq;
    m_input.vbat = preset.init.quadBatCellCount * 4.2f;
}

// Stops the propellers and clears the ESC models (filters, delay line).
// SimITL's Reset/Repair commands keep them, so a drone deleted while flying
// would otherwise respawn with spinning motors.
void Flight::stopMotors() {
    SimITL::SimState &ss = SimITL::Sim::getInstance().mSimState;
    for (auto &m : ss.motorsState) {
        m.rpm = 0;
        m.pwm = 0;
        m.thrust = 0;
        m.current = 0;
        m.mTorque = m.pTorque = m.netTorque = 0;
        m.pwmLowPassFilter = SimITL::LowPassFilter{};
        m.propWashLowPassFilter = SimITL::LowPassFilter{};
        m.pwmHistory.clear();
    }
    ss.acceleration = {0, 0, 0};
}

void Flight::spawn(const DronePreset &preset, Vec3 pos, Quat rot, Vec3 pilotPos) {
    applyPreset(preset);
    repair();
    stopMotors();
    m_active = true;
    m_pilotPos = pilotPos;
    teleport(pos, rot);
}

void Flight::despawn() {
    m_active = false;
    stopMotors();
    teleport({0, 0, 0}, Quat{});
}

void Flight::teleport(Vec3 pos, Quat rot) {
    m_pos = pos;
    m_rot = normalize(rot);
    m_vel = {};
    m_angVel = {};
    simitl_command(CommandType::Reset);
    m_lastGroundContact = -1;
}

void Flight::repair() {
    simitl_command(CommandType::Repair);
    for (float &d : m_propDamage) d = 0;
}

Mat3 Flight::worldInvInertia() const {
    Mat3 r = toMat3(m_rot);
    return r * Mat3::diag(m_invInertiaBody) * transpose(r);
}

void Flight::recordImpact(const Contact &c, float speed, float impulse, int motor) {
    int slot = motor + 1;
    if (m_simTime - m_lastMotorStrike[slot] < 0.05) return;  // debounce per sphere
    m_lastMotorStrike[slot] = m_simTime;
    if (m_impacts.size() >= 64) m_impacts.erase(m_impacts.begin());
    ImpactEvent e;
    e.point = c.point;
    e.normal = c.normal;
    e.speed = speed;
    e.impulse = impulse;
    e.material = c.material;
    e.objectId = c.objectId;
    e.motorIndex = motor;
    m_impacts.push_back(e);
}

void Flight::takeImpacts(std::vector<ImpactEvent> &out) {
    out.insert(out.end(), m_impacts.begin(), m_impacts.end());
    m_impacts.clear();
}

// Sequential impulses over the contacts of all collision spheres at the
// current pose; also pushes the body out of penetration.
void Flight::resolveContacts(float dt) {
    (void)dt;
    struct Item {
        Contact c;
        int motor;
        float accN = 0;
        float approach = 0;
    };
    std::vector<Item> items;
    Mat3 rot = toMat3(m_rot);
    for (const Sphere &s : m_shape) {
        m_contacts.clear();
        Vec3 centre = m_pos + rot * s.local;
        m_world->sphereContacts(centre, s.radius, m_simTime, m_contacts);
        for (const Contact &c : m_contacts) items.push_back({c, s.motor});
    }
    if (items.empty()) return;

    // positional correction: deepest penetration along each normal
    Vec3 push{};
    for (const Item &it : items) {
        float need = it.c.depth - dot(push, it.c.normal);
        if (need > 0) push += it.c.normal * need;
    }
    m_pos += push;

    Mat3 invI = worldInvInertia();
    float invM = 1.0f / m_mass;
    for (Item &it : items) {
        Vec3 r = it.c.point - m_pos;
        Vec3 v = m_vel + cross(m_angVel, r) - it.c.surfaceVel;
        it.approach = -dot(v, it.c.normal);
        if (it.c.normal.y > 0.5f) m_lastGroundContact = m_simTime;
    }

    for (int iter = 0; iter < 6; iter++) {
        for (Item &it : items) {
            const Contact &c = it.c;
            Material mat = m_world->material(c.material);
            Vec3 r = c.point - m_pos;
            Vec3 v = m_vel + cross(m_angVel, r) - c.surfaceVel;
            float vn = dot(v, c.normal);
            // arms and props soak up hits on a motor: only the body bounces
            float e = (it.approach > kRestingSpeed && it.motor < 0) ? mat.restitution : 0.0f;
            Vec3 rn = cross(r, c.normal);
            float kN = invM + dot(c.normal, cross(invI * rn, r));
            float target = it.approach > 0 ? e * it.approach : 0.0f;
            float jn = (target - vn) / kN;
            float newAcc = std::max(it.accN + jn, 0.0f);
            jn = newAcc - it.accN;
            it.accN = newAcc;
            m_vel += c.normal * (jn * invM);
            m_angVel += invI * cross(r, c.normal * jn);

            // Coulomb friction against the accumulated normal impulse
            v = m_vel + cross(m_angVel, r) - c.surfaceVel;
            Vec3 vt = v - c.normal * dot(v, c.normal);
            float vtLen = length(vt);
            if (vtLen > 1e-5f) {
                Vec3 t = vt / vtLen;
                Vec3 rt = cross(r, t);
                float kT = invM + dot(t, cross(invI * rt, r));
                float jt = std::min(vtLen / kT, mat.friction * it.accN);
                m_vel -= t * (jt * invM);
                m_angVel -= invI * cross(r, t * jt);
            }
        }
    }

    for (const Item &it : items) {
        Material mat = m_world->material(it.c.material);
        if (it.motor >= 0 && (mat.flags & kMatKillsProps)) m_propDamage[it.motor] = 1.0f;
        if (it.approach > kImpactReportSpeed) recordImpact(it.c, it.approach, it.accN, it.motor);
    }
}

// Propeller discs touching geometry damage the props.
void Flight::propStrikes(float dt) {
    (void)dt;
    Mat3 rot = toMat3(m_rot);
    Vec3 up = rot.col(1);
    const StateOutput &out = SimITL::Sim::getInstance().getStateUpdate();
    float maxRpm = std::max(1000.0f, m_preset.init.propMaxRpm);
    for (int i = 0; i < 4; i++) {
        if (m_propDamage[i] >= 1.0f) continue;
        const Vec3F &lp = m_preset.init.quadMotorPos[i];
        Vec3 hub = m_pos + rot * Vec3{lp.x, lp.y, lp.z};
        m_contacts.clear();
        m_world->sphereContacts(hub, m_preset.collision.propRadius, m_simTime, m_contacts);
        for (const Contact &c : m_contacts) {
            if (std::fabs(dot(c.point - hub, up)) > kPropDiscHalfThickness) continue;
            Material mat = m_world->material(c.material);
            if (mat.flags & kMatKillsProps) {
                m_propDamage[i] = 1.0f;
                break;
            }
            if (m_simTime - m_lastMotorStrike[i + 1] < 0.02) continue;
            Vec3 r = c.point - m_pos;
            Vec3 v = m_vel + cross(m_angVel, r) - c.surfaceVel;
            float closing = std::max(0.0f, -dot(v, c.normal));
            float spin = std::fabs(out.motorRpm[i]) / maxRpm;
            float severity = closing + spin * 3.0f;
            if (severity < 0.5f) continue;
            m_propDamage[i] = std::min(1.0f, m_propDamage[i] + severity / std::max(1.0f, m_preset.collision.propDurability));
            recordImpact(c, closing, 0.0f, i);
            break;
        }
    }
}

void Flight::updateGroundEffect(float out[4]) {
    Mat3 rot = toMat3(m_rot);
    Vec3 down = -rot.col(1);
    float maxD = std::max(0.01f, m_preset.maxGroundEffectDist);
    for (int i = 0; i < 4; i++) {
        out[i] = 0;
        const Vec3F &lp = m_preset.init.quadMotorPos[i];
        Vec3 hub = m_pos + rot * Vec3{lp.x, lp.y, lp.z};
        float t;
        Vec3 n;
        uint8_t mat;
        uint32_t obj;
        if (m_world->raycast(hub, down, maxD, m_simTime, t, n, mat, obj)) out[i] = std::clamp(1.0f - t / maxD, 0.0f, 1.0f);
    }
}

void Flight::updateVideo() {
    // occlusion between pilot (receiver) and drone (transmitter), 50 Hz
    if (m_videoTick++ % 20 == 0) {
        Vec3 rx = m_pilotPos + Vec3{0, 1.5f, 0};
        Vec3 d = m_pos - rx;
        float dist = length(d);
        float blocked = 0;
        if (dist > 0.5f) {
            float t;
            Vec3 n;
            uint8_t mat;
            uint32_t obj;
            if (m_world->raycast(rx, d / dist, dist - 0.3f, m_simTime, t, n, mat, obj, true)) blocked = 1;
        }
        m_videoOcclusion += (blocked - m_videoOcclusion) * 0.25f;
    }
    float dist = length(m_pos - m_pilotPos);
    const VtxSettings &v = m_preset.vtx;
    float rangeFrac = dist / std::max(1.0f, v.rangeM);
    float noise = v.noiseFloor + v.noiseFactor * 0.1f * rangeFrac * rangeFrac * 10.0f * 0.1f + m_videoOcclusion * 0.45f;
    noise = std::clamp(noise + std::max(0.0f, rangeFrac - 0.8f) * 2.5f, 0.0f, 1.0f);
    m_snap.videoNoise = noise;
    m_snap.videoBreakup = std::clamp((noise - 0.55f) * 2.2f, 0.0f, 1.0f);
    m_snap.videoSignal = std::clamp(1.0f - noise, 0.0f, 1.0f);
}

void Flight::setHome(const Vec3 *home) {
    std::lock_guard<std::mutex> lock(m_homeMutex);
    m_hasHome = home != nullptr;
    if (home) m_home = *home;
}

void Flight::step(float dt, const float channels[kChannelCount], bool radioLost) {
    std::unique_lock<std::mutex> worldLock(m_world->mutex());

    m_simTime += dt;
    // --- inputs for SimITL ----------------------------------------------
    for (int i = 0; i < kChannelCount; i++) m_input.rcData[i] = std::clamp(channels[i], -1.0f, 1.0f);
    for (int i = kChannelCount; i < 16; i++) m_input.rcData[i] = -1.0f;  // unused AUX7+
    if (!m_active) {
        // parked: never armed, Betaflight still runs for the Configurator
        m_input.rcData[4] = -1.0f;
        m_input.rcData[2] = -1.0f;
        m_pos = {};
        m_rot = Quat{};
        m_vel = {};
        m_angVel = {};
    }
    SimITL::BF::setRcFailsafe(radioLost && m_active);
    // turtle mode: Betaflight picks it at arming, SimITL spins the props backwards
    m_input.motorsReversed = bfb_crash_flip_active() ? 1 : 0;

    if ((m_videoTick & 63) == 0) syncPropDirection();  // follows Configurator edits

    float ge[4] = {0, 0, 0, 0};
    if (m_active) updateGroundEffect(ge);
    m_input.delta = dt;
    m_input.position = {m_pos.x, m_pos.y, m_pos.z};
    fillRotation(m_input, m_rot);
    m_input.linearVelocity = {m_vel.x, m_vel.y, m_vel.z};
    m_input.angularVelocity = {m_angVel.x, m_angVel.y, m_angVel.z};
    m_input.contact = 2;  // our velocities are authoritative (patch 0002)
    for (int i = 0; i < 4; i++) {
        m_input.groundEffect[i] = ge[i];
        m_input.propDamage[i] = m_propDamage[i];
    }

    // internal X right, Y up, Z forward = east, up, north
    bfb_gps_update(m_pos.x, m_pos.z, m_pos.y, m_vel.x, m_vel.z, m_vel.y);
    bfb_baro_update(m_pos.y);

    simitl_update(m_input);
    // after the update: arming inside it may have just moved home
    {
        std::lock_guard<std::mutex> lock(m_homeMutex);
        if (m_hasHome) bfb_gps_set_home(m_home.x, m_home.z, m_home.y);
    }
    const StateOutput &out = SimITL::Sim::getInstance().getStateUpdate();

    // --- integrate pose with collisions -----------------------------------
    if (m_active) {
        m_rot = normalize(Quat{out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w});
        m_vel = {out.linearVelocity.x, out.linearVelocity.y, out.linearVelocity.z};
        m_angVel = {out.angularVelocity.x, out.angularVelocity.y, out.angularVelocity.z};
        if (!finite(m_vel) || !finite(m_angVel) || !std::isfinite(m_rot.w)) {
            FPV_LOGW("flight model produced a non-finite state; resetting velocities");
            m_vel = {};
            m_angVel = {};
            m_rot = Quat{};
            simitl_command(CommandType::Reset);
        }
        float speed = length(m_vel);
        if (speed > kMaxSpeed) m_vel *= kMaxSpeed / speed;
        if (length(m_angVel) > kMaxAngularSpeed) m_angVel *= kMaxAngularSpeed / length(m_angVel);

        // Substeps keep every sphere moving less than half its radius, so
        // thin geometry cannot be skipped (conservative advancement).
        float minRadius = m_preset.collision.motorRadius;
        for (const Sphere &s : m_shape) minRadius = std::min(minRadius, s.radius);
        float travel = length(m_vel) * dt + length(m_angVel) * dt * 0.15f;
        int sub = std::clamp((int)std::ceil(travel / (0.5f * minRadius)), 1, 64);
        float h = dt / (float)sub;
        resolveContacts(h);
        for (int k = 0; k < sub; k++) {
            m_pos += m_vel * h;
            resolveContacts(h);
        }
        propStrikes(dt);
    }

    // --- snapshot -----------------------------------------------------------
    const SimITL::SimState &ss = SimITL::Sim::getInstance().mSimState;
    m_snap.simTime = m_simTime;
    m_snap.active = m_active;
    m_snap.pos = m_pos;
    m_snap.rot = m_rot;
    m_snap.vel = m_vel;
    m_snap.angVel = m_angVel;
    bool anyBurned = false, anyBroken = false;
    for (int i = 0; i < 4; i++) {
        MotorSnapshot &m = m_snap.motors[i];
        m.rpm = out.motorRpm[i];
        m.thrust = out.thrust[i];
        m.current = out.motorCurrent[i];
        m.temperature = out.motorT[i];
        m.command = std::clamp(out.motorOutput[i], 0.0f, 1.0f);
        m.propDamage = m_propDamage[i];
        m.flags = 0;
        if (out.motorStatus[i] & MotorStatus::MotorBurnedOut) {
            m.flags |= 1u;
            anyBurned = true;
        }
        if (out.motorStatus[i] & MotorStatus::MotorDamaged || m_propDamage[i] > 0.05f) m.flags |= 2u;
        if (m_propDamage[i] >= 0.99f) {
            m.flags |= 4u;
            anyBroken = true;
        }
    }
    m_snap.batVoltage = out.batteryVoltage;
    m_snap.batVoltageSag = ss.batteryState.batVoltageSag;
    m_snap.batCurrent = (float)ss.batteryState.amperage;
    m_snap.mahDrawn = (float)ss.batteryState.mAhDrawn;
    m_snap.armed = bfb_is_armed();
    m_snap.failsafe = bfb_failsafe_active();
    m_snap.beeper = out.beep != 0;
    m_snap.armingDisableFlags = bfb_arming_disable_flags();
    uint32_t modes = bfb_flight_mode_flags();
    m_snap.modeAngle = (modes & 1u) != 0;    // ANGLE_MODE
    m_snap.modeHorizon = (modes & 2u) != 0;  // HORIZON_MODE
    m_snap.turtle = m_input.motorsReversed != 0;
    m_snap.osdDisabled = bfb_osd_disabled() != 0;
    m_snap.onGround = m_active && m_lastGroundContact >= 0 && m_simTime - m_lastGroundContact < 0.05;
    m_snap.crashed = m_active && (anyBurned || anyBroken);
    if (std::memcmp(m_snap.osd.data(), out.osd, m_snap.osd.size()) != 0) {
        std::memcpy(m_snap.osd.data(), out.osd, m_snap.osd.size());
        m_snap.osdRevision++;
    }
    for (int i = 0; i < kChannelCount; i++) m_snap.channels[i] = m_input.rcData[i];
    if (m_active) {
        float t;
        Vec3 n;
        uint8_t mat;
        uint32_t obj;
        if ((m_videoTick & 7) == 0) {
            m_snap.altitude = m_world->raycast(m_pos, {0, -1, 0}, 1000.0f, m_simTime, t, n, mat, obj) ? t : -1.0f;
        }
        updateVideo();
    }
}

}  // namespace fpv
