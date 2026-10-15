// libfpv - C API of kamikaze builds (LIBFPV_KAMIKAZE): warhead state,
// detonation events, host detonation and configuration.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <cstring>

#include "lib.h"

using namespace fpv;

namespace {

void put3(float out[3], Vec3 v) {
    out[0] = v.x;
    out[1] = v.y;
    out[2] = v.z;
}

}  // namespace

namespace fpv {

void applyKamikazeSettings() {
    Lib &L = lib();
    const Settings &s = L.settings;
    WarheadConfig w;
    w.armingDelay = s.kamikazeArmingDelay;
    w.minImpactSpeed = s.kamikazeMinImpactSpeed;
    w.osd = s.kamikazeOsd;
    L.flight.warhead().setConfig(w);
    float kg = s.kamikazePayloadKg, drop = s.kamikazePayloadDrop;
    bool installed = L.kamikazeInstalled;
    L.runner.call([&] {
        L.flight.setBombInstalled(installed);
        L.flight.setPayload(kg, drop);
    });
}

}  // namespace fpv

extern "C" {

FPV_API fpv_result fpv_kamikaze_get_state(fpv_drone *d, fpv_kamikaze_state *state) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    if (!d || d != &L.drone || !L.droneExists) return setError(FPV_ERROR_INVALID_ARGUMENT, "invalid or deleted drone handle");
    if (!state || state->struct_size < offsetof(fpv_kamikaze_state, flags))
        return setError(FPV_ERROR_ABI_MISMATCH, "set state->struct_size = sizeof(fpv_kamikaze_state)");
    FlightSnapshot prev, cur;
    double pw, cw;
    L.runner.snapshots(prev, pw, cur, cw);
    const WarheadSnapshot &w = cur.warhead;
    fpv_kamikaze_state out{};
    out.struct_size = sizeof(out);
    out.warhead = (fpv_warhead_state)w.state;
    out.arming_remaining = w.armingRemaining;
    if (w.installed) out.flags |= FPV_KAMIKAZE_INSTALLED;
    if (w.bombSwitch) out.flags |= FPV_KAMIKAZE_BOMB_SWITCH;
    if (w.explodeSwitch) out.flags |= FPV_KAMIKAZE_EXPLODE_SWITCH;
    if (w.blocksArming) out.flags |= FPV_KAMIKAZE_BLOCKS_ARMING;
    if (w.needsCycle && w.bombSwitch) out.flags |= FPV_KAMIKAZE_NEEDS_SWITCH_CYCLE;
    out.detonation_count = w.detonations;
    out.payload_mass = cur.payloadMass;
    out.total_mass = cur.totalMass;
    uint32_t size = std::min<uint32_t>(state->struct_size, sizeof(out));
    memcpy(state, &out, size);
    state->struct_size = size;
    return FPV_OK;
}

FPV_API int fpv_kamikaze_poll_detonation(fpv_detonation *out) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!out || out->struct_size < offsetof(fpv_detonation, material)) return 0;
    DetonationEvent e;
    if (!L.flight.warhead().pollDetonation(e)) return 0;
    const CoordConverter &cv = L.conv;
    fpv_detonation d{};
    d.struct_size = sizeof(d);
    d.sim_time = e.simTime;
    d.cause = (fpv_detonation_cause)e.cause;
    put3(d.position, cv.posToHost(e.position));
    put3(d.velocity, cv.dirToHost(e.velocity) * cv.unitsPerMeter());
    if (length2(e.normal) > 0) put3(d.normal, cv.dirToHost(e.normal));
    d.material = e.material;
    d.object_id = e.objectId;
    d.motor_index = e.motorIndex;
    uint32_t size = std::min<uint32_t>(out->struct_size, sizeof(d));
    memcpy(out, &d, size);
    out->struct_size = size;
    return 1;
}

FPV_API fpv_result fpv_kamikaze_set_installed(int installed) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    L.kamikazeInstalled = installed != 0;
    applyKamikazeSettings();
    return FPV_OK;
}

FPV_API int fpv_kamikaze_is_installed(void) { return lib().kamikazeInstalled ? 1 : 0; }

FPV_API fpv_result fpv_kamikaze_detonate(fpv_drone *d, int evenIfSafe) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    if (!d || d != &L.drone || !L.droneExists) return setError(FPV_ERROR_INVALID_ARGUMENT, "invalid or deleted drone handle");
    bool done = L.runner.call([&] { return L.flight.detonate(evenIfSafe != 0); });
    return done ? FPV_OK : setError(FPV_ERROR_NOT_FOUND, "the warhead is not live (or already exploded, or not installed)");
}

FPV_API void fpv_kamikaze_config_defaults(fpv_kamikaze_config *c) {
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->struct_size = sizeof(*c);
    Settings def;
    c->arming_delay = def.kamikazeArmingDelay;
    Lib &L = lib();
    c->min_impact_speed = L.initialized ? L.conv.lenToHost(def.kamikazeMinImpactSpeed) : def.kamikazeMinImpactSpeed;
    c->payload_mass = def.kamikazePayloadKg;
    c->payload_drop = def.kamikazePayloadDrop;
}

FPV_API fpv_result fpv_kamikaze_set_config(const fpv_kamikaze_config *in) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    if (!in) return setError(FPV_ERROR_INVALID_ARGUMENT, "config is NULL");
    fpv_kamikaze_config c;
    fpv_kamikaze_config_defaults(&c);
    memcpy(&c, in, std::min<size_t>(sizeof(c), in->struct_size ? in->struct_size : sizeof(c)));
    if (!std::isfinite(c.arming_delay) || c.arming_delay < 0 || c.arming_delay > 600)
        return setError(FPV_ERROR_INVALID_ARGUMENT, "arming_delay must be 0..600 s");
    float fuse = L.conv.lenToInternal(c.min_impact_speed);
    if (!std::isfinite(fuse) || fuse < 0.1f - 1e-4f || fuse > 30.0f)
        return setError(FPV_ERROR_INVALID_ARGUMENT, "min_impact_speed must be 0.1..30 m/s (in host units)");
    if (!std::isfinite(c.payload_mass) || c.payload_mass > 50)
        return setError(FPV_ERROR_INVALID_ARGUMENT, "payload_mass must be up to 50 kg (negative = automatic)");
    if (!std::isfinite(c.payload_drop)) return setError(FPV_ERROR_INVALID_ARGUMENT, "payload_drop is not finite");
    Settings &s = L.settings;
    s.kamikazeArmingDelay = c.arming_delay;
    s.kamikazeMinImpactSpeed = std::max(0.1f, fuse);
    s.kamikazePayloadKg = c.payload_mass < 0 ? -1.0f : c.payload_mass;
    s.kamikazePayloadDrop = c.payload_drop < 0 ? -1.0f : std::min(L.conv.lenToInternal(c.payload_drop), 1.0f);
    s.kamikazeOsd = !(c.flags & FPV_KAMIKAZE_NO_OSD);
    applyKamikazeSettings();
    saveSettings();
    return FPV_OK;
}

FPV_API fpv_result fpv_kamikaze_get_config(fpv_kamikaze_config *out) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    if (!out || out->struct_size < offsetof(fpv_kamikaze_config, flags))
        return setError(FPV_ERROR_ABI_MISMATCH, "set config->struct_size = sizeof(fpv_kamikaze_config)");
    const Settings &s = L.settings;
    fpv_kamikaze_config c;
    fpv_kamikaze_config_defaults(&c);
    c.arming_delay = s.kamikazeArmingDelay;
    c.min_impact_speed = L.conv.lenToHost(s.kamikazeMinImpactSpeed);
    c.flags = s.kamikazeOsd ? 0 : FPV_KAMIKAZE_NO_OSD;
    c.payload_mass = s.kamikazePayloadKg;
    c.payload_drop = s.kamikazePayloadDrop < 0 ? -1.0f : L.conv.lenToHost(s.kamikazePayloadDrop);
    uint32_t size = std::min<uint32_t>(out->struct_size, sizeof(c));
    memcpy(out, &c, size);
    out->struct_size = size;
    return FPV_OK;
}

}  // extern "C"
