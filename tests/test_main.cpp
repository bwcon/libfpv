// libfpv - headless test suite. Runs in CI on every platform.
//
// Betaflight is a process-wide singleton, so all tests share one process and
// one fpv_global_init; each test resets the world and recreates the drone.
// Timing is FPV_TIMING_MANUAL, so every test is deterministic.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "libfpv.h"

#if defined(_WIN32)
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

namespace {

int g_failures = 0;
int g_checks = 0;
std::string g_current;

#define CHECK(cond, ...)                                                                      \
    do {                                                                                      \
        g_checks++;                                                                           \
        if (!(cond)) {                                                                        \
            g_failures++;                                                                     \
            std::printf("  FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond);                   \
            std::printf(__VA_ARGS__);                                                         \
            std::printf("\n");                                                                \
        }                                                                                     \
    } while (0)

void logCb(void *, fpv_log_level level, const char *msg) {
    if (level >= FPV_LOG_WARN || std::getenv("LIBFPV_TEST_VERBOSE")) std::printf("    [libfpv] %s\n", msg);
}

struct Rc {
    float ch[8] = {0, 0, -1, 0, -1, -1, -1, -1};
    void send() { fpv_input_set_channels(ch, 8); }
};

// Simple test world in a given convention. Ground at "up" = 0 and a wall
// `wallDist` metres ahead along +forward (host convention).
struct World {
    std::vector<float> v;
    std::vector<uint32_t> idx;
    void quad(const float a[3], const float b[3], const float c[3], const float d[3]) {
        uint32_t base = (uint32_t)(v.size() / 3);
        for (const float *p : {a, b, c, d}) v.insert(v.end(), p, p + 3);
        uint32_t q[6] = {base, base + 1, base + 2, base, base + 2, base + 3};
        idx.insert(idx.end(), q, q + 6);
    }
    fpv_mesh mesh() const {
        fpv_mesh m{};
        m.positions = v.data();
        m.vertex_count = (uint32_t)(v.size() / 3);
        m.indices = idx.data();
        m.triangle_count = (uint32_t)(idx.size() / 3);
        return m;
    }
};

// Builds axis vectors of the host convention for a fpv_coord_system.
void axisVec(fpv_axis a, float s, float out[3]) {
    out[0] = out[1] = out[2] = 0;
    out[a / 2] = (a % 2) ? -s : s;
}

struct Host {
    fpv_coord_system cs;
    float R[3], U[3], F[3];  // unit host vectors for right/up/forward
    float upm;
    explicit Host(fpv_coord_system c) : cs(c), upm(c.units_per_meter) {
        axisVec(c.right, 1, R);
        axisVec(c.up, 1, U);
        axisVec(c.forward, 1, F);
    }
    // host position from (right, up, forward) metres
    void pos(float r, float u, float f, float out[3]) const {
        for (int i = 0; i < 3; i++) out[i] = (R[i] * r + U[i] * u + F[i] * f) * upm;
    }
    float along(const float v[3], const float axis[3]) const { return (v[0] * axis[0] + v[1] * axis[1] + v[2] * axis[2]) / upm; }
    float dir(const float v[3], const float axis[3]) const { return v[0] * axis[0] + v[1] * axis[1] + v[2] * axis[2]; }
    float right(const float v[3]) const { return along(v, R); }
    float up(const float v[3]) const { return along(v, U); }
    float fwd(const float v[3]) const { return along(v, F); }
};

World makeLevel(const Host &h, float size, float wallFwd) {
    World w;
    float a[3], b[3], c[3], d[3];
    h.pos(-size, 0, -size, a);
    h.pos(size, 0, -size, b);
    h.pos(size, 0, size, c);
    h.pos(-size, 0, size, d);
    w.quad(a, b, c, d);
    if (wallFwd > 0) {
        // a thin (zero thickness) wall across the forward axis
        h.pos(-50, -1, wallFwd, a);
        h.pos(50, -1, wallFwd, b);
        h.pos(50, 60, wallFwd, c);
        h.pos(-50, 60, wallFwd, d);
        w.quad(a, b, c, d);
    }
    return w;
}

struct Session {
    fpv_drone *drone = nullptr;
    fpv_frame f{};
    Rc rc;
    Session() { f.struct_size = sizeof(f); }
    void spawn(const float pos[3], const char *preset = nullptr) {
        fpv_drone_desc d;
        fpv_drone_desc_defaults(&d);
        std::memcpy(d.position, pos, sizeof(d.position));
        d.preset = preset ? preset : "propeater-5";
        fpv_result r = fpv_drone_create(&d, &drone);
        CHECK(r == FPV_OK, "drone_create: %s", fpv_last_error());
    }
    void run(float seconds, const std::function<void(float t)> &each = nullptr) {
        int n = (int)std::lround(seconds / 0.01f);
        for (int i = 0; i < n; i++) {
            if (each) each(i * 0.01f);
            rc.send();
            fpv_step(0.01f);
            fpv_drone_tick(drone, 0.01f, &f);
        }
    }
    bool armed() const { return (f.flags & FPV_DRONE_ARMED) != 0; }
    void arm() {
        rc.ch[2] = -1;
        rc.ch[4] = -1;
        run(0.3f);
        rc.ch[4] = 1;
        run(0.3f);
    }
    void destroy() {
        if (drone) fpv_drone_delete(drone);
        drone = nullptr;
    }
};

std::string g_dataDir;

bool init(fpv_coord_system cs) {
    fpv_global_config cfg;
    fpv_global_config_defaults(&cfg);
    cfg.data_dir = g_dataDir.c_str();
    cfg.coords = cs;
    cfg.timing = FPV_TIMING_MANUAL;
    cfg.flags = FPV_INIT_NO_DEVICE_INPUT | FPV_INIT_NO_CONFIGURATOR;
    cfg.log = logCb;
    fpv_result r = fpv_global_init(&cfg);
    CHECK(r == FPV_OK, "init: %s", fpv_last_error());
    return r == FPV_OK;
}

void loadWorld(const World &w) {
    fpv_mesh m = w.mesh();
    fpv_result r = fpv_static_surfaces_load(&m, 1);
    CHECK(r == FPV_OK, "static load: %s", fpv_last_error());
}

// --- tests ------------------------------------------------------------------------

void testVersionAndErrors() {
    CHECK(fpv_abi_version() == LIBFPV_ABI_VERSION, "abi %u", fpv_abi_version());
    CHECK(std::strcmp(fpv_version_string(), LIBFPV_VERSION_STRING) == 0, "%s", fpv_version_string());
    fpv_drone *d = nullptr;
    CHECK(fpv_drone_create(nullptr, &d) == FPV_ERROR_NOT_INITIALIZED, "create before init must fail");
    CHECK(std::strlen(fpv_last_error()) > 0, "last error set");
    fpv_global_config bad;
    fpv_global_config_defaults(&bad);
    CHECK(fpv_global_init(&bad) == FPV_ERROR_INVALID_ARGUMENT, "missing data_dir must fail");
    bad.data_dir = g_dataDir.c_str();
    bad.coords.up = bad.coords.right;
    CHECK(fpv_global_init(&bad) == FPV_ERROR_INVALID_ARGUMENT, "degenerate axes must fail");
}

void testArmDisarm() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 100, 0));
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.run(0.5f);
    CHECK(!s.armed(), "starts disarmed");
    CHECK(s.f.arming_disable_flags == 0, "armable right after spawn, got \"%s\"", s.f.arming_disable_text);
    CHECK(s.f.flags & FPV_DRONE_ON_GROUND, "rests on the ground");
    CHECK(std::fabs(h.up(s.f.position)) < 0.08f, "rest height %.3f", h.up(s.f.position));
    s.arm();
    CHECK(s.armed(), "arms on AUX1 (%s)", s.f.arming_disable_text);
    CHECK(s.f.motors[0].rpm > 500, "motors idle when armed: %.0f rpm", s.f.motors[0].rpm);
    s.rc.ch[4] = -1;
    s.run(0.2f);
    CHECK(!s.armed(), "disarms");
    // Arm switch held on while not armable must not arm (Betaflight safety).
    s.rc.ch[2] = 0.5f;
    s.run(0.2f);
    s.rc.ch[4] = 1;
    s.run(0.2f);
    CHECK(!s.armed(), "does not arm with throttle up");
    CHECK(std::strstr(s.f.arming_disable_text, "THROTTLE") != nullptr, "reports THROTTLE: \"%s\"", s.f.arming_disable_text);
    s.destroy();
}

// Upside down on the ground: it will not arm normally; in turtle mode
// (AUX4) it arms, the props spin backwards and a roll stick flips it over.
// libfpv's crashflip_rate cuts the power once it has turned over, so the
// stick can simply be held. Every preset must make it.
void testTurtle() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 100, 0));
    // body up, as a host vector, from the frame's quaternion
    auto bodyUp = [&](const fpv_frame &f) {
        float x = f.rotation[0], y = f.rotation[1], z = f.rotation[2], w = f.rotation[3];
        float u[3] = {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)};
        return h.dir(u, h.U);
    };
    for (uint32_t i = 0; i < fpv_preset_count(); i++) {
        const char *name = fpv_preset_name(i);
        Session s;
        fpv_drone_desc d;
        fpv_drone_desc_defaults(&d);
        h.pos(0, 0.1f, 0, d.position);
        float flip[4] = {h.F[0], h.F[1], h.F[2], 0};  // 180 degrees about forward
        std::memcpy(d.rotation, flip, sizeof(flip));
        d.preset = name;
        CHECK(fpv_drone_create(&d, &s.drone) == FPV_OK, "drone_create: %s", fpv_last_error());
        s.run(1.0f);
        CHECK(bodyUp(s.f) < -0.9f, "%s rests upside down, up %.2f", name, bodyUp(s.f));
        s.arm();
        CHECK(!s.armed(), "%s does not arm upside down without turtle", name);
        CHECK(std::strstr(s.f.arming_disable_text, "ANGLE") != nullptr, "reports ANGLE: \"%s\"",
              s.f.arming_disable_text);
        s.rc.ch[4] = -1;
        s.rc.ch[7] = 1;  // turtle switch
        s.run(0.3f);
        s.arm();
        CHECK(s.armed(), "%s arms in turtle mode (%s)", name, s.f.arming_disable_text);
        CHECK(s.f.flags & FPV_DRONE_TURTLE, "turtle flag set while armed");
        s.rc.ch[0] = 0.6f;
        float upright = -1, damage = 0;
        s.run(1.5f, [&](float t) {
            if (upright < 0 && bodyUp(s.f) > 0.9f) upright = t;
        });
        s.rc.ch[0] = 0;
        s.run(0.5f);
        for (auto &m : s.f.motors) damage = std::max(damage, m.prop_damage);
        std::printf("    %s: upright after %.2f s, final up %.2f, prop damage %.2f\n", name, upright, bodyUp(s.f),
                    damage);
        CHECK(bodyUp(s.f) > 0.9f, "%s flipped back upright, up %.2f", name, bodyUp(s.f));
        CHECK(damage < 0.3f, "%s props survive the flip, damage %.2f", name, damage);
        // after a successful flip, turning turtle off re-arms in normal mode
        // at once (Betaflight's quick restart): it can fly away
        s.rc.ch[7] = -1;
        s.run(0.3f);
        CHECK(s.armed(), "%s armed again after turtle off (%s)", name, s.f.arming_disable_text);
        CHECK(!(s.f.flags & FPV_DRONE_TURTLE), "turtle flag cleared");
        s.rc.ch[2] = 0.3f;
        s.run(1.0f);
        CHECK(h.up(s.f.position) > 0.5f, "%s flies away, %.2f m", name, h.up(s.f.position));
        s.destroy();
    }
}

// An EEPROM from before turtle mode gets the AUX4 switch and the crash flip
// cut-off once; after that the player's own edits stay.
void testEepromUpgrade() {
    namespace fs = std::filesystem;
    auto cli = [](const char *cmd) {
        char *o = nullptr;
        fpv_cli_exec(cmd, &o);
        std::string r = o ? o : "";
        fpv_free(o);
        return r;
    };
    auto activate = [](const char *preset) {
        Session s;
        float p[3] = {0, 0, 0};
        s.spawn(p, preset);
        s.run(0.05f);
        s.destroy();
    };
    std::string first = fpv_preset_name(0), other = fpv_preset_name(1);
    activate(first.c_str());
    fs::path marker = fs::path(g_dataDir) / "eeprom";
    for (auto &e : fs::directory_iterator(marker))
        if (e.path().extension() == ".upgrade") fs::remove(e.path());
    cli("aux 3 0 0 900 900 0 0\nset crashflip_rate = 0\nsave");  // an old EEPROM
    activate(other.c_str());
    activate(first.c_str());
    CHECK(cli("aux").find("aux 3 35 3 1700 2100") != std::string::npos, "turtle switch added");
    CHECK(cli("get crashflip_rate").find("crashflip_rate = 30") != std::string::npos, "crash flip cut-off set");
    cli("aux 3 0 0 900 900 0 0\nsave");  // the player removes it again
    activate(other.c_str());
    activate(first.c_str());
    CHECK(cli("aux").find("aux 3 35 3") == std::string::npos, "removed turtle switch stays removed");
}

void testSecondDroneRejected() {
    Session a;
    float p[3] = {0, 0, 0};
    a.spawn(p);
    fpv_drone *second = nullptr;
    fpv_result r = fpv_drone_create(nullptr, &second);
    CHECK(r == FPV_ERROR_DRONE_EXISTS, "second drone: %s", fpv_result_string(r));
    CHECK(second == nullptr, "no handle for the second drone");
    a.destroy();
    fpv_frame f{};
    f.struct_size = sizeof(f);
    CHECK(fpv_drone_tick(a.drone, 0.01f, &f) == FPV_ERROR_INVALID_ARGUMENT, "deleted handle rejected");
    Session b;
    b.spawn(p);
    CHECK(b.drone != nullptr, "a drone can be created again after delete");
    b.destroy();
}

// Finds the throttle that hovers by bisection in a clean world, then checks
// that this throttle holds altitude.
// Arms in the air at a given throttle, lets the motors settle, then
// teleports back to the start with zero velocity (staying armed). Returns
// the session ready to measure from rest.
void hoverFromRest(Session &s, const Host &h, float thr) {
    float p[3];
    h.pos(0, 50, 0, p);  // high up: no ground effect
    s.spawn(p);
    s.rc.ch[5] = 1;  // ANGLE mode (AUX2) keeps it level
    s.arm();
    s.rc.ch[2] = thr;
    s.run(1.0f);
    float rot[4] = {0, 0, 0, 1};
    fpv_drone_reset(s.drone, p, rot);
    s.run(0.01f);
}

// Hover throttle by bisection on the direction of motion from rest.
float findHoverThrottle(const Host &h) {
    float lo = -1.0f, hi = 0.2f;
    for (int iter = 0; iter < 14; iter++) {
        float mid = 0.5f * (lo + hi);
        Session s;
        hoverFromRest(s, h, mid);
        float y0 = h.up(s.f.position);
        s.run(1.0f);
        bool rising = h.up(s.f.position) > y0;
        s.destroy();
        (rising ? hi : lo) = mid;
    }
    return 0.5f * (lo + hi);
}

void testHover() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 200, 0));
    float thr = findHoverThrottle(h);
    std::printf("    hover throttle %.4f (stick), %.1f%%\n", thr, (thr + 1) * 50);
    CHECK(thr > -0.9f && thr < 0.2f, "plausible hover throttle %.3f", thr);
    Session s;
    hoverFromRest(s, h, thr);
    CHECK(s.armed(), "armed after reset");
    float y0 = h.up(s.f.position), maxDev = 0;
    s.run(3.0f, [&](float) { maxDev = std::max(maxDev, std::fabs(h.up(s.f.position) - y0)); });
    std::printf("    hover altitude deviation over 3 s: %.2f m\n", maxDev);
    CHECK(maxDev < 1.0f, "hover holds altitude within 1 m, deviation %.2f m", maxDev);
    CHECK(s.armed(), "still armed");
    s.destroy();
}

void testPunchOut() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 300, 0));
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.arm();
    s.rc.ch[2] = 1.0f;
    float maxV = 0;
    s.run(2.0f, [&](float) { maxV = std::max(maxV, s.f.speed_kmh); });
    float height = h.up(s.f.position);
    std::printf("    punch-out: %.0f km/h, %.1f m after 2 s\n", maxV, height);
    CHECK(maxV > 60.0f && maxV < 320.0f, "plausible punch-out speed %.0f km/h", maxV);
    CHECK(height > 20.0f, "climbed %.1f m", height);
    s.destroy();
}

// Stick directions in every host convention: pitch forward flies forward,
// roll right flies right, yaw right turns clockwise seen from above.
void testAxes(fpv_coords_preset_id preset, float upm, const char *name) {
    Host h(fpv_coords_preset(preset, upm));
    fpv_global_terminate();
    init(h.cs);
    loadWorld(makeLevel(h, 300, 0));

    auto fly = [&](int channel, float value, float seconds) {
        Session s;
        float p[3];
        h.pos(0, 30, 0, p);
        s.spawn(p);
        s.rc.ch[5] = 1;  // ANGLE
        s.arm();
        s.rc.ch[2] = -0.2f;
        s.rc.ch[channel] = value;
        s.run(seconds);
        fpv_frame f = s.f;
        s.destroy();
        return f;
    };
    fpv_frame f = fly(1, 0.5f, 1.5f);  // pitch forward
    CHECK(h.fwd(f.velocity) > 3.0f, "%s: pitch forward -> forward velocity %.2f m/s", name, h.fwd(f.velocity));
    CHECK(h.dir(f.camera.forward, h.F) > 0.3f, "%s: FPV camera looks forward", name);
    f = fly(0, 0.5f, 1.5f);  // roll right
    CHECK(h.right(f.velocity) > 3.0f, "%s: roll right -> rightward velocity %.2f m/s", name, h.right(f.velocity));
    f = fly(3, 0.5f, 0.6f);  // yaw right
    // drone forward axis expressed via the FPV camera forward (uptilt only pitches)
    float fr = h.dir(f.camera.forward, h.R), ff = h.dir(f.camera.forward, h.F);
    CHECK(fr > 0.2f, "%s: yaw right turns the nose right (r=%.2f f=%.2f)", name, fr, ff);
    // the camera basis must be right-handed in the host's own sense
    float u[3], r[3], fw[3];
    std::memcpy(u, f.camera.up, sizeof u);
    std::memcpy(r, f.camera.right, sizeof r);
    std::memcpy(fw, f.camera.forward, sizeof fw);
    CHECK(std::fabs(r[0] * u[0] + r[1] * u[1] + r[2] * u[2]) < 1e-3f, "%s: camera axes orthogonal", name);
}

void testWallCrash() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 300, 40));
    Session s;
    float p[3];
    h.pos(0, 3, 0, p);
    s.spawn(p);
    s.arm();
    s.rc.ch[2] = -0.3f;
    s.run(0.3f);
    // Teleport the drone just in front of the wall at 30 m/s using the
    // physics: point it at the wall and let it fly level at speed.
    s.rc.ch[1] = 0.0f;
    // Set velocity via reset + fast forward flight: pitch hard forward with high throttle.
    s.rc.ch[5] = 1;      // ANGLE so it stays level
    s.rc.ch[1] = 1.0f;   // full forward
    s.rc.ch[2] = 0.6f;
    int impacts = 0;
    float maxSpeed = 0, maxFwd = -1e9f, impactSpeed = 0;
    s.run(6.0f, [&](float) {
        for (uint32_t i = 0; i < s.f.impact_count; i++) {
            impacts++;
            impactSpeed = std::max(impactSpeed, s.f.impacts[i].speed);
        }
        maxSpeed = std::max(maxSpeed, s.f.speed_kmh / 3.6f);
        maxFwd = std::max(maxFwd, h.fwd(s.f.position));
    });
    std::printf("    wall: max speed %.1f m/s, impact speed %.1f m/s, closest %.3f m before the wall\n", maxSpeed,
                impactSpeed, 40.0f - maxFwd);
    CHECK(impacts > 0, "impact reported");
    CHECK(maxFwd < 40.0f, "never passed through the wall (max %.3f)", maxFwd);
    s.destroy();

    // Direct 30 m/s shot: spawn in front of the wall and fire the drone at it
    // with an explicit velocity through a moving platform-free setup.
    World w = makeLevel(h, 300, 40);
    loadWorld(w);
    Session t;
    h.pos(0, 5, 39.0f, p);
    t.spawn(p);
    // Use reset to align, then let the sim run with injected forward speed by
    // repeatedly flying full forward; check substepping at high speed.
    t.rc.ch[5] = 1;
    t.arm();
    t.rc.ch[1] = 1.0f;
    t.rc.ch[2] = 1.0f;
    float maxF = -1e9f;
    t.run(2.0f, [&](float) { maxF = std::max(maxF, h.fwd(t.f.position)); });
    CHECK(maxF < 40.0f, "thin wall holds at full power from 1 m (max %.3f)", maxF);
    t.destroy();
}

// 30 m/s through a thin wall using a moving surface object: the wall flies
// at the resting drone at 30 m/s (relative motion is what matters).
void testThirtyMetersPerSecond() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 300, 0));
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.run(0.3f);
    // thin wall, 4 m wide, 3 m tall, zero thickness, in object space at fwd=0
    World wall;
    float a[3], b[3], c[3], d[3];
    h.pos(-2, -0.5f, 0, a);
    h.pos(2, -0.5f, 0, b);
    h.pos(2, 3, 0, c);
    h.pos(-2, 3, 0, d);
    wall.quad(a, b, c, d);
    fpv_mesh m = wall.mesh();
    fpv_transform t{};
    t.rotation[3] = 1;
    h.pos(0, 0, 10, t.position);
    uint32_t id = 0;
    CHECK(fpv_surface_object_create(&m, &t, &id) == FPV_OK, "object create: %s", fpv_last_error());
    // move the wall towards the drone at 30 m/s with 100 Hz updates
    int impacts = 0;
    float minRel = 1e9f;
    for (int i = 0; i < 100; i++) {
        float fwd = 10.0f - 30.0f * 0.01f * (i + 1);
        h.pos(0, 0, fwd, t.position);
        fpv_surface_object_move(id, &t);
        s.rc.send();
        fpv_step(0.01f);
        fpv_drone_tick(s.drone, 0.01f, &s.f);
        impacts += (int)s.f.impact_count;
        minRel = std::min(minRel, h.fwd(s.f.position) - fwd);  // drone must stay in front (< 0 = behind wall)
    }
    std::printf("    30 m/s wall: %d impacts, drone ended %.2f m along forward\n", impacts, h.fwd(s.f.position));
    CHECK(impacts > 0, "impact at 30 m/s reported");
    CHECK(h.fwd(s.f.position) < -5.0f, "the wall carried the drone (pos %.2f)", h.fwd(s.f.position));
    fpv_surface_object_delete(id);
    s.destroy();
}

void testMotorBurnout() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    // A ceiling made of a prop-killing material: the drone takes off, loses
    // its propellers, drops, and the "pilot" keeps the throttle pinned. With
    // broken props (extra friction current, no prop-wash cooling) the
    // windings overheat until a motor burns out.
    fpv_material kill{};
    kill.restitution = 0.1f;
    kill.friction = 0.5f;
    kill.flags = FPV_MATERIAL_KILLS_PROPS;
    CHECK(fpv_material_set(7, &kill) == FPV_OK, "material");
    World w = makeLevel(h, 100, 0);
    float a[3], b[3], c[3], d[3];
    h.pos(-5, 0.6f, -5, a);
    h.pos(5, 0.6f, -5, b);
    h.pos(5, 0.6f, 5, c);
    h.pos(-5, 0.6f, 5, d);
    w.quad(a, b, c, d);
    std::vector<uint8_t> mats(w.idx.size() / 3, 0);
    mats[2] = mats[3] = 7;  // the ceiling quad
    fpv_mesh m = w.mesh();
    m.materials = mats.data();
    CHECK(fpv_static_surfaces_load(&m, 1) == FPV_OK, "load");
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.arm();
    s.rc.ch[2] = 1.0f;
    float maxT = 0;
    bool burned = false, crashed = false, propsBroken = false;
    s.run(240.0f, [&](float) {
        for (auto &mo : s.f.motors) {
            maxT = std::max(maxT, mo.temperature_c);
            if (mo.flags & FPV_MOTOR_BURNED_OUT) burned = true;
            if (mo.flags & FPV_MOTOR_PROP_BROKEN) propsBroken = true;
        }
        if (s.f.flags & FPV_DRONE_CRASHED) crashed = true;
    });
    std::printf("    burnout: props broken=%d, max motor temperature %.0f C, burned=%d\n", propsBroken, maxT, burned);
    CHECK(propsBroken, "prop-killing material breaks the propellers");
    CHECK(crashed, "broken props mark the drone crashed");
    CHECK(burned, "a motor burns out at full throttle on broken props (max %.0f C)", maxT);
    s.rc.ch[2] = -1.0f;
    s.run(0.1f);
    // like a player respawning: repair, then put it back on the ground
    // (it may still be touching the prop-killing ceiling)
    fpv_drone_repair(s.drone);
    fpv_drone_reset(s.drone, p, nullptr);
    s.run(0.1f);
    bool still = false;
    for (auto &mo : s.f.motors)
        if (mo.flags & (FPV_MOTOR_BURNED_OUT | FPV_MOTOR_PROP_BROKEN)) still = true;
    CHECK(!still, "repair restores motors and propellers (flags %u %u %u %u, %.0f C, damage %.2f)", s.f.motors[0].flags,
          s.f.motors[1].flags, s.f.motors[2].flags, s.f.motors[3].flags, s.f.motors[0].temperature_c,
          s.f.motors[0].prop_damage);
    s.destroy();
    fpv_material def{0.2f, 0.6f, 0};
    fpv_material_set(7, &def);
}

void testPresets() {
    uint32_t n = fpv_preset_count();
    CHECK(n >= 5, "built-in presets found: %u", n);
    for (uint32_t i = 0; i < n; i++) {
        const char *name = fpv_preset_name(i);
        CHECK(name && name[0], "preset %u has a name", i);
        Session s;
        float p[3] = {0, 0, 0};
        s.spawn(p, name);
        s.run(0.3f);
        s.arm();
        s.rc.ch[2] = 0.0f;
        s.run(1.0f);
        CHECK(s.armed(), "preset \"%s\" arms (%s)", name, s.f.arming_disable_text);
        CHECK(s.f.position[1] > 1.0f, "preset \"%s\" flies (%.2f m)", name, s.f.position[1]);
        CHECK(s.f.battery.cell_count >= 1, "preset \"%s\" battery", name);
        s.destroy();
    }
    Session s;
    float p[3] = {0, 0, 0};
    fpv_drone_desc d;
    fpv_drone_desc_defaults(&d);
    d.preset = "no such quad";
    CHECK(fpv_drone_create(&d, &s.drone) == FPV_ERROR_NOT_FOUND, "unknown preset rejected");
    (void)p;
}

void testCli() {
    char *out = nullptr;
    CHECK(fpv_cli_exec("get motor_pwm_protocol", &out) == FPV_OK, "cli: %s", fpv_last_error());
    CHECK(out && std::strstr(out, "PWM"), "PWM protocol: %s", out ? out : "(null)");
    fpv_free(out);
    CHECK(fpv_cli_exec("bl", &out) == FPV_OK, "refused commands do not fail the call");
    CHECK(out && std::strstr(out, "refused"), "bootloader refused: %s", out ? out : "");
    fpv_free(out);
    CHECK(fpv_cli_exec("set roll_rc_rate = 123\nsave", &out) == FPV_OK, "set+save");
    fpv_free(out);
    CHECK(fpv_cli_exec("get roll_rc_rate", &out) == FPV_OK, "get");
    CHECK(out && std::strstr(out, "123"), "value stored: %s", out ? out : "");
    fpv_free(out);    CHECK(fpv_cli_exec("aux", &out) == FPV_OK, "aux");
    CHECK(out && std::strstr(out, "aux 3 35 3 1700 2100"), "turtle on AUX4 by default: %s", out ? out : "");
    fpv_free(out);
    CHECK(fpv_cli_exec("get crashflip_rate", &out) == FPV_OK, "get crashflip_rate");
    CHECK(out && std::strstr(out, "crashflip_rate = 30"), "turtle cut-off on by default: %s", out ? out : "");
    fpv_free(out);
}

// Corpus of real-world style `diff all` outputs in tests/diffs/.
void testDiffImport(const std::string &corpusDir) {
    namespace fs = std::filesystem;
    loadWorld(makeLevel(Host(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1)), 100, 0));
    int files = 0;
    std::error_code ec;
    for (auto &e : fs::directory_iterator(corpusDir, ec)) {
        if (e.path().extension() != ".txt") continue;
        files++;
        std::string name = e.path().filename().string();
        FILE *fp = std::fopen(e.path().string().c_str(), "rb");
        std::string text;
        char buf[4096];
        size_t n;
        while ((n = std::fread(buf, 1, sizeof buf, fp)) > 0) text.append(buf, n);
        std::fclose(fp);
        char *report = nullptr;
        fpv_result r = fpv_import_betaflight_diff(text.c_str(), &report);
        CHECK(r == FPV_OK, "%s: import %s", name.c_str(), fpv_last_error());
        CHECK(report && std::strstr(report, "Applied"), "%s: report", name.c_str());
        if (std::getenv("LIBFPV_TEST_VERBOSE") && report) std::printf("%s\n", report);
        // Renamed settings from older firmware are reported, not fatal.
        const char *rej = report ? std::strstr(report, "Rejected by Betaflight") : nullptr;
        if (rej) std::printf("    %s: %.*s\n", name.c_str(), (int)(std::strchr(rej, '\n') ? std::strchr(rej, '\n') - rej : 40), rej);
        fpv_free(report);

        char *out = nullptr;
        fpv_cli_exec("get motor_pwm_protocol\naux", &out);
        CHECK(out && std::strstr(out, "= PWM"), "%s: PWM forced", name.c_str());
        CHECK(out && std::strstr(out, "aux 0 0 0 1700 2100"), "%s: ARM on AUX1", name.c_str());
        fpv_free(out);

        // and it flies
        Session s;
        float p[3] = {0, 0, 0};
        s.spawn(p);
        s.run(0.3f);
        s.arm();
        CHECK(s.armed(), "%s: arms after import (%s)", name.c_str(), s.f.arming_disable_text);
        s.rc.ch[2] = 0.0f;
        s.run(1.0f);
        CHECK(s.f.position[1] > 1.0f, "%s: flies after import (%.2f m)", name.c_str(), s.f.position[1]);
        s.destroy();
    }
    CHECK(files >= 3, "diff corpus has %d files", files);
    char *report = nullptr;
    CHECK(fpv_import_betaflight_diff("hello world", &report) != FPV_OK || (report && std::strstr(report, "Rejected")),
          "garbage is not silently accepted");
    fpv_free(report);
}

void testPinned() {
#if defined(_WIN32)
    HMODULE h = nullptr;
    CHECK(GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, (LPCWSTR)(void *)&fpv_global_init, &h), "module");
    // FreeLibrary on a pinned module is a no-op; it must stay loaded.
    FreeLibrary(h);
    FreeLibrary(h);
    HMODULE again = nullptr;
    CHECK(GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                             (LPCWSTR)(void *)&fpv_global_init, &again),
          "libfpv still mapped after FreeLibrary");
#else
    Dl_info info;
    CHECK(dladdr((void *)&fpv_global_init, &info) != 0, "dladdr");
    void *h = dlopen(info.dli_fname, RTLD_NOW | RTLD_NOLOAD);
    CHECK(h != nullptr, "loaded");
    dlclose(h);
    dlclose(h);
    dlclose(h);
    void *again = dlopen(info.dli_fname, RTLD_NOW | RTLD_NOLOAD);
    CHECK(again != nullptr, "libfpv still mapped after dlclose");
    if (again) dlclose(again);
#endif
    CHECK(fpv_abi_version() == LIBFPV_ABI_VERSION, "still callable");
}

void testTerminateReinit() {
    fpv_global_terminate();
    fpv_global_terminate();  // twice is fine
    CHECK(!fpv_is_initialized(), "terminated");
    CHECK(init(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1)), "re-init after terminate");
    Session s;
    float p[3] = {0, 0, 0};
    loadWorld(makeLevel(Host(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1)), 100, 0));
    s.spawn(p);
    s.arm();
    CHECK(s.armed(), "arms after re-init (%s)", s.f.arming_disable_text);
    s.destroy();
}

void testThreadedMode() {
    fpv_global_terminate();
    fpv_global_config cfg;
    fpv_global_config_defaults(&cfg);
    cfg.data_dir = g_dataDir.c_str();
    // real device input (SDL) and the Configurator server, as in a game
    cfg.flags = 0;
    cfg.configurator_port = 5799;
    cfg.log = logCb;
    CHECK(fpv_global_init(&cfg) == FPV_OK, "threaded init: %s", fpv_last_error());
    fpv_input_status ist{};
    ist.struct_size = sizeof(ist);
    CHECK(fpv_input_get_status(&ist) == FPV_OK, "input status");
    Host h(cfg.coords);
    loadWorld(makeLevel(h, 100, 0));
    fpv_drone *d = nullptr;
    CHECK(fpv_drone_create(nullptr, &d) == FPV_OK, "create");
    fpv_frame f{};
    f.struct_size = sizeof(f);
    float ch[8] = {0, 0, -1, 0, -1, -1, -1, -1};
    auto t0 = std::chrono::steady_clock::now();
    double simStart = -1;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(1500)) {
        double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        ch[4] = el > 0.3 ? 1.0f : -1.0f;
        ch[2] = el > 0.8 ? 0.0f : -1.0f;
        fpv_input_set_channels(ch, 8);
        fpv_drone_tick(d, 0.016f, &f);
        if (simStart < 0) simStart = f.sim_time;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    double simElapsed = f.sim_time - simStart;
    std::printf("    threaded: %.3f s simulated in 1.5 s wall\n", simElapsed);
    CHECK(simElapsed > 1.3 && simElapsed < 1.6, "real-time pacing (%.3f s)", simElapsed);
    CHECK(f.flags & FPV_DRONE_ARMED, "armed in threaded mode (%s)", f.arming_disable_text);
    CHECK(f.position[1] > 0.5f, "flying in threaded mode (%.2f)", f.position[1]);
    // auto-pause: stop ticking, sim time must stop advancing
    double before = f.sim_time;
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    fpv_drone_tick(d, 0.016f, &f);
    std::printf("    auto-pause: %.3f s advanced while the host did not tick for 1.2 s\n", f.sim_time - before);
    CHECK(f.sim_time - before < 0.7, "auto-paused when the host stopped ticking");
    fpv_drone_delete(d);
    fpv_global_terminate();
}

void testOsdAudioOverlay() {
    Session s;
    float p[3] = {0, 0, 0};
    s.spawn(p);
    s.run(0.5f);
    int chars = 0;
    for (uint8_t c : s.f.osd) chars += (c != 0 && c != ' ');
    CHECK(chars > 10, "Betaflight OSD has content (%d characters)", chars);
    fpv_texture tex{};
    CHECK(fpv_texture_get(FPV_TEXTURE_OSD_FONT, &tex) == FPV_OK && tex.width > 0 && tex.rgba, "OSD font texture");
    for (int style = 0; style < 2; style++) {
        const fpv_draw_data *dd = fpv_osd_draw_data(&s.f, 1280, 720, (fpv_osd_style)style);
        CHECK(dd && dd->list_count == 1 && dd->total_index_count == (uint32_t)chars * 6, "OSD quads style %d", style);
        if (dd && dd->list_count) CHECK(dd->lists[0].commands[0].texture_id == FPV_TEXTURE_OSD_FONT, "OSD texture id");
    }
    std::vector<uint8_t> img(640 * 480 * 4);
    CHECK(fpv_osd_render_rgba(&s.f, img.data(), 640, 480, 0, FPV_OSD_STYLE_ANALOG) == FPV_OK, "OSD raster");
    size_t opaque = 0;
    for (size_t i = 3; i < img.size(); i += 4) opaque += img[i] != 0;
    CHECK(opaque > 500, "OSD raster has pixels (%zu)", opaque);

    // audio: silent when disarmed, motor sound when spinning
    CHECK(fpv_audio_configure(48000, 2) == FPV_OK, "audio configure");
    std::vector<float> buf(4800 * 2);
    auto energy = [&] {
        fpv_audio_read(buf.data(), 4800);
        double e = 0;
        for (float v : buf) e += v * v;
        return e / buf.size();
    };
    energy();
    double quiet = energy();
    s.arm();
    s.rc.ch[2] = 0.0f;
    s.run(0.5f);
    energy();
    double loud = energy();
    std::printf("    audio energy disarmed %.2e, flying %.2e\n", quiet, loud);
    CHECK(quiet < 1e-6, "silent when the motors are stopped");
    CHECK(loud > 1e-3, "motor sound when flying");
    for (float v : buf) {
        if (!(std::fabs(v) <= 1.0f)) {
            CHECK(false, "audio sample out of range %f", v);
            break;
        }
    }

    // overlay
    CHECK(fpv_overlay_render(0.016f) == nullptr, "closed overlay draws nothing");
    fpv_overlay_set_display(1280, 720, 1.0f);
    fpv_overlay_set_open(1);
    CHECK(fpv_overlay_wants_input(), "open overlay wants input");
    const fpv_draw_data *od = nullptr;
    for (int i = 0; i < 3; i++) od = fpv_overlay_render(0.016f);
    CHECK(od && od->total_vertex_count > 100, "overlay draw data (%u vertices)", od ? od->total_vertex_count : 0);
    bool fontCmd = false;
    for (uint32_t l = 0; od && l < od->list_count; l++)
        for (uint32_t c = 0; c < od->lists[l].command_count; c++) {
            const fpv_draw_cmd &cmd = od->lists[l].commands[c];
            fontCmd |= cmd.texture_id == FPV_TEXTURE_OVERLAY_FONT;
            CHECK(cmd.index_offset + cmd.index_count <= od->lists[l].index_count, "command within index buffer");
        }
    CHECK(fontCmd, "overlay uses its font texture");
    CHECK(fpv_texture_get(FPV_TEXTURE_OVERLAY_FONT, &tex) == FPV_OK && tex.width >= 256, "overlay font %ux%u", tex.width,
          tex.height);
    // host buttons render in the Fly tab; nothing is clicked yet
    fpv_overlay_set_action(7, "Move player here");
    fpv_overlay_set_action(7, "Move the player here");
    CHECK(fpv_overlay_render(0.016f) != nullptr, "renders with a host button");
    CHECK(fpv_overlay_poll_action() == 0, "no host button clicked");
    fpv_overlay_set_action(7, nullptr);
    CHECK(fpv_overlay_render(0.016f) != nullptr, "renders after removing the host button");
    // click through every tab (the tab bar is at the top of the centred window)
    for (int tab = 0; tab < 10; tab++) {
        fpv_overlay_mouse_move(0, 0);
        od = fpv_overlay_render(0.016f);
        CHECK(od != nullptr, "renders with tab %d", tab);
    }
    // the toggle key closes it; flight keys do not leak while it is open
    fpv_input_key(FPV_KEY_F8, 1);
    fpv_input_key(FPV_KEY_F8, 0);
    CHECK(!fpv_overlay_is_open(), "F8 closes the overlay");
    s.destroy();
}

struct Test {
    const char *name;
    std::function<void()> fn;
};

}  // namespace

int main(int argc, char **argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // keep output if a test crashes
    std::string filter = argc > 1 ? argv[1] : "";
    namespace fs = std::filesystem;
    g_dataDir = (fs::temp_directory_path() / ("libfpv-test-" + std::to_string((long long)std::time(nullptr)))).string();
    std::string corpus = LIBFPV_TEST_DIFF_DIR;

    testVersionAndErrors();
    if (!init(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1))) return 1;

    std::vector<Test> tests = {
        {"arm_disarm", testArmDisarm},
        {"turtle", testTurtle},
        {"eeprom_upgrade", testEepromUpgrade},
        {"second_drone_rejected", testSecondDroneRejected},
        {"hover", testHover},
        {"punch_out", testPunchOut},
        {"wall_crash", testWallCrash},
        {"thirty_mps_thin_wall", testThirtyMetersPerSecond},
        {"motor_burnout", testMotorBurnout},
        {"presets", testPresets},
        {"cli", testCli},
        {"diff_import", [&] { testDiffImport(corpus); }},
        {"axes_y_up_right", [] { testAxes(FPV_COORDS_Y_UP_RIGHT_HANDED, 1, "Y-up right-handed"); }},
        {"axes_y_up_left", [] { testAxes(FPV_COORDS_Y_UP_LEFT_HANDED, 1, "Y-up left-handed"); }},
        {"axes_z_up_left_cm", [] { testAxes(FPV_COORDS_Z_UP_LEFT_HANDED, 100, "Z-up left-handed (cm)"); }},
        {"axes_z_up_right_in", [] { testAxes(FPV_COORDS_Z_UP_RIGHT_HANDED, 39.37f, "Z-up right-handed (in)"); }},
        {"osd_audio_overlay", testOsdAudioOverlay},
        {"pinned", testPinned},
        {"terminate_reinit", testTerminateReinit},
        {"threaded", testThreadedMode},
    };
    int ran = 0;
    for (auto &t : tests) {
        if (!filter.empty() && ("," + filter + ",").find("," + std::string(t.name) + ",") == std::string::npos) continue;
        if (!fpv_is_initialized() && std::string(t.name) != "threaded")
            init(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
        // every test starts from libfpv's default Betaflight configuration
        if (fpv_is_initialized()) {
            char *o = nullptr;
            fpv_cli_exec("defaults\nsave", &o);
            fpv_free(o);
            // and from a plain 200 m ground in the default convention
            loadWorld(makeLevel(Host(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1)), 100, 0));
        }
        int before = g_failures;
        auto t0 = std::chrono::steady_clock::now();
        std::printf("[ RUN  ] %s\n", t.name);
        std::fflush(stdout);
        t.fn();
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("[ %s ] %s (%.0f ms)\n", g_failures == before ? " OK " : "FAIL", t.name, ms);
        ran++;
    }
    fpv_global_terminate();
    std::error_code ec;
    fs::remove_all(g_dataDir, ec);
    std::printf("\n%d tests, %d checks, %d failures\n", ran, g_checks, g_failures);
    return g_failures ? 1 : 0;
}
