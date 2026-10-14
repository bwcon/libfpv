// examples/standalone - a minimal "game" that integrates libfpv exactly the
// way a mod would: SDL3 window, OpenGL 3.3 renderer, a test level (ground,
// gates, buildings, a thin wall, a moving platform), and every libfpv hook:
//
//   1. fpv_global_init                       once, after the game starts
//   2. fpv_static_surfaces_load              level collision
//      fpv_surface_object_create/move        the moving platform
//   3. fpv_drone_create                      spawn
//      fpv_gps_set_home                      the launch pad is GPS home
//   4. every frame: fpv_drone_tick           pose + camera + OSD + impacts
//      place the game camera at frame.camera (or chase/LOS)
//      draw the drone at frame.position/rotation, props by frame.motors[].rpm
//      lens distortion + video noise post-process from frame.camera/video
//      fpv_osd_draw_data + fpv_overlay_render drawn by fpv_render_gl3.h
//   5. forward keyboard/mouse to fpv_input_key / fpv_overlay_*,
//      and stop game input while fpv_overlay_wants_input()
//   6. audio: fpv_audio_read from the audio callback
//   7. fpv_global_terminate on exit
//
// Keys: F8 overlay, V camera (FPV/chase/LOS), R respawn, F11 or Alt+Enter
// fullscreen, Esc quit.
// Verification flags: --autopilot (scripted flight), --screenshot out.ppm
// --frames N (save the N-th frame and exit), --overlay (open it), --camera.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <SDL3/SDL.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#define GL_MIN_IMPLEMENTATION
#include "gl_min.h"
#define FPV_RENDER_GL3_IMPLEMENTATION
#include "fpv_render_gl3.h"
#include "libfpv.h"

// --- tiny math ---------------------------------------------------------------------

struct V3 {
    float x, y, z;
};
static V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static V3 norm(V3 a) {
    float l = std::sqrt(dot(a, a));
    return l > 0 ? a * (1 / l) : a;
}

struct M4 {
    float m[16];  // column major
};
static M4 mul(const M4 &a, const M4 &b) {
    M4 r{};
    for (int c = 0; c < 4; c++)
        for (int rr = 0; rr < 4; rr++) {
            float s = 0;
            for (int k = 0; k < 4; k++) s += a.m[k * 4 + rr] * b.m[c * 4 + k];
            r.m[c * 4 + rr] = s;
        }
    return r;
}
static M4 identity() {
    M4 r{};
    r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1;
    return r;
}
static M4 translate(V3 t) {
    M4 r = identity();
    r.m[12] = t.x, r.m[13] = t.y, r.m[14] = t.z;
    return r;
}
// rotation matrix from a quaternion {x, y, z, w}
static M4 fromQuat(const float q[4]) {
    float x = q[0], y = q[1], z = q[2], w = q[3];
    M4 r = identity();
    r.m[0] = 1 - 2 * (y * y + z * z), r.m[1] = 2 * (x * y + z * w), r.m[2] = 2 * (x * z - y * w);
    r.m[4] = 2 * (x * y - z * w), r.m[5] = 1 - 2 * (x * x + z * z), r.m[6] = 2 * (y * z + x * w);
    r.m[8] = 2 * (x * z + y * w), r.m[9] = 2 * (y * z - x * w), r.m[10] = 1 - 2 * (x * x + y * y);
    return r;
}
static M4 rotY(float a) {
    M4 r = identity();
    r.m[0] = std::cos(a), r.m[2] = -std::sin(a), r.m[8] = std::sin(a), r.m[10] = std::cos(a);
    return r;
}
static M4 perspective(float fovYDeg, float aspect, float n, float f) {
    float t = 1.0f / std::tan(fovYDeg * 3.14159265f / 360.0f);
    M4 r{};
    r.m[0] = t / aspect, r.m[5] = t, r.m[10] = (f + n) / (n - f), r.m[11] = -1, r.m[14] = 2 * f * n / (n - f);
    return r;
}
// view matrix from libfpv's camera basis (OpenGL style: camera looks down -Z)
static M4 viewFromCamera(const fpv_camera &c) {
    V3 r{c.right[0], c.right[1], c.right[2]}, u{c.up[0], c.up[1], c.up[2]}, f{c.forward[0], c.forward[1], c.forward[2]};
    V3 p{c.position[0], c.position[1], c.position[2]};
    M4 v = identity();
    v.m[0] = r.x, v.m[4] = r.y, v.m[8] = r.z;
    v.m[1] = u.x, v.m[5] = u.y, v.m[9] = u.z;
    v.m[2] = -f.x, v.m[6] = -f.y, v.m[10] = -f.z;
    v.m[12] = -dot(r, p), v.m[13] = -dot(u, p), v.m[14] = dot(f, p);
    return v;
}

// --- meshes ------------------------------------------------------------------------

struct Mesh {
    std::vector<float> v;  // pos3 normal3 color3
    std::vector<uint32_t> idx;
    GLuint vao = 0, vbo = 0, ebo = 0;

    void tri(V3 a, V3 b, V3 c, V3 col) {
        V3 n = norm(cross(b - a, c - a));
        uint32_t base = (uint32_t)(v.size() / 9);
        for (V3 p : {a, b, c}) {
            float d[9] = {p.x, p.y, p.z, n.x, n.y, n.z, col.x, col.y, col.z};
            v.insert(v.end(), d, d + 9);
        }
        idx.insert(idx.end(), {base, base + 1, base + 2});
    }
    void quad(V3 a, V3 b, V3 c, V3 d, V3 col) {
        tri(a, b, c, col);
        tri(a, c, d, col);
    }
    // box with centre, half extents and yaw
    void box(V3 c, V3 h, float yaw, V3 col) {
        float cs = std::cos(yaw), sn = std::sin(yaw);
        auto P = [&](float x, float y, float z) {
            float rx = x * cs + z * sn, rz = -x * sn + z * cs;
            return V3{c.x + rx, c.y + y, c.z + rz};
        };
        V3 p[8] = {P(-h.x, -h.y, -h.z), P(h.x, -h.y, -h.z), P(h.x, h.y, -h.z), P(-h.x, h.y, -h.z),
                   P(-h.x, -h.y, h.z),  P(h.x, -h.y, h.z),  P(h.x, h.y, h.z),  P(-h.x, h.y, h.z)};
        quad(p[4], p[5], p[6], p[7], col);  // +z
        quad(p[1], p[0], p[3], p[2], col);  // -z
        quad(p[5], p[1], p[2], p[6], col);  // +x
        quad(p[0], p[4], p[7], p[3], col);  // -x
        quad(p[7], p[6], p[2], p[3], col);  // +y
        quad(p[0], p[1], p[5], p[4], col);  // -y
    }
    void upload() {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(v.size() * 4), v.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(idx.size() * 4), idx.data(), GL_STATIC_DRAW);
        for (int i = 0; i < 3; i++) {
            glEnableVertexAttribArray(i);
            glVertexAttribPointer(i, 3, GL_FLOAT, GL_FALSE, 36, (void *)(size_t)(i * 12));
        }
        glBindVertexArray(0);
    }
    void draw() const {
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, (GLsizei)idx.size(), GL_UNSIGNED_INT, nullptr);
    }
    // positions only, for libfpv collision
    fpv_mesh collision() const {
        fpv_mesh m{};
        m.positions = v.data();
        m.vertex_count = (uint32_t)(v.size() / 9);
        m.vertex_stride = 36;
        m.indices = idx.data();
        m.triangle_count = (uint32_t)(idx.size() / 3);
        return m;
    }
};

static void buildLevel(Mesh &m) {
    // ground (also drawn with a checker in the shader)
    m.quad({-300, 0, 300}, {300, 0, 300}, {300, 0, -300}, {-300, 0, -300}, {0.35f, 0.5f, 0.3f});
    // a loop of race gates (1.8 m opening) around the spawn
    for (int i = 0; i < 6; i++) {
        float a = i * 6.2831853f / 6, r = 35;
        V3 c{std::sin(a) * r, 0, -std::cos(a) * r - 35};
        float yaw = -a;
        V3 col{1.0f, 0.45f, 0.1f};
        auto P = [&](float x, float y) { return V3{c.x + x * std::cos(yaw), y, c.z - x * std::sin(yaw)}; };
        m.box(P(-1.0f, 1.2f), {0.1f, 1.2f, 0.1f}, yaw, col);
        m.box(P(1.0f, 1.2f), {0.1f, 1.2f, 0.1f}, yaw, col);
        m.box(P(0, 2.3f), {1.1f, 0.1f, 0.1f}, yaw, col);
        m.box(P(0, 0.4f), {1.1f, 0.05f, 0.1f}, yaw, col);
    }
    // buildings
    struct B {
        float x, z, w, h, d;
    } const bs[] = {{-30, -10, 8, 14, 8}, {25, -5, 10, 9, 6}, {-15, -80, 12, 22, 12}, {40, -70, 6, 30, 6},
                    {0, 40, 20, 6, 4}};
    for (auto &b : bs) m.box({b.x, b.h / 2, b.z}, {b.w / 2, b.h / 2, b.d / 2}, 0.2f, {0.55f, 0.55f, 0.6f});
    // a thin (zero thickness) wall: collision must still hold at speed
    V3 wc{0.75f, 0.85f, 0.95f};
    m.quad({-6, 0, -15}, {6, 0, -15}, {6, 5, -15}, {-6, 5, -15}, wc);
    m.quad({6, 0, -15}, {-6, 0, -15}, {-6, 5, -15}, {6, 5, -15}, wc);
}

static void buildDrone(Mesh &body, Mesh &prop, const fpv_frame &f) {
    V3 dark{0.12f, 0.12f, 0.14f}, red{0.85f, 0.1f, 0.1f};
    body.box({0, 0.01f, 0}, {0.035f, 0.018f, 0.06f}, 0, dark);
    body.box({0, 0.035f, -0.045f}, {0.015f, 0.012f, 0.012f}, 0, red);  // camera, front (-Z)
    for (int i = 0; i < 4; i++) {
        V3 m{f.motors[i].position[0], f.motors[i].position[1], f.motors[i].position[2]};
        V3 mid = m * 0.5f;
        float yaw = std::atan2(m.x, m.z);
        float len = std::sqrt(m.x * m.x + m.z * m.z);
        body.box({mid.x, 0.0f, mid.z}, {0.008f, 0.004f, len / 2}, yaw, dark);
        body.box(m + V3{0, 0.006f, 0}, {0.013f, 0.012f, 0.013f}, 0, {0.6f, 0.6f, 0.65f});
    }
    // one propeller (3 blades) in its local frame
    for (int b = 0; b < 3; b++) {
        float a = b * 2.0943951f;
        V3 tip{std::sin(a) * 0.064f, 0.02f, std::cos(a) * 0.064f}, side{std::cos(a) * 0.009f, 0, -std::sin(a) * 0.009f};
        V3 hub{0, 0.02f, 0};
        prop.tri(hub, tip + side, tip - side, {0.2f, 0.7f, 1.0f});
        prop.tri(hub, tip - side, tip + side, {0.2f, 0.7f, 1.0f});
    }
}

// --- shaders -------------------------------------------------------------------------

static GLuint program(const char *vs, const char *fs) {
    auto sh = [](GLenum t, const char *src) {
        GLuint s = glCreateShader(t);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
        GLint ok;
        glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[2048];
            glGetShaderInfoLog(s, sizeof log, nullptr, log);
            std::fprintf(stderr, "shader: %s\n", log);
        }
        return s;
    };
    GLuint p = glCreateProgram();
    glAttachShader(p, sh(GL_VERTEX_SHADER, vs));
    glAttachShader(p, sh(GL_FRAGMENT_SHADER, fs));
    glLinkProgram(p);
    return p;
}

static const char *kSceneVs = R"(#version 330 core
layout(location=0) in vec3 pos; layout(location=1) in vec3 nrm; layout(location=2) in vec3 col;
uniform mat4 viewProj; uniform mat4 model;
out vec3 v_nrm; out vec3 v_col; out vec3 v_world;
void main() {
  vec4 w = model * vec4(pos, 1.0);
  v_world = w.xyz; v_nrm = mat3(model) * nrm; v_col = col;
  gl_Position = viewProj * w;
})";
static const char *kSceneFs = R"(#version 330 core
in vec3 v_nrm; in vec3 v_col; in vec3 v_world; uniform vec3 eye; out vec4 o;
void main() {
  vec3 n = normalize(v_nrm);
  float l = 0.35 + 0.65 * abs(dot(n, normalize(vec3(0.4, 1.0, 0.3))));
  vec3 c = v_col * l;
  if (n.y > 0.99 && v_world.y < 0.01) { // ground checker, 2 m tiles
    float k = mod(floor(v_world.x * 0.5) + floor(v_world.z * 0.5), 2.0);
    c *= 0.85 + 0.15 * k;
  }
  float fog = clamp(length(v_world - eye) / 400.0, 0.0, 1.0);
  o = vec4(mix(c, vec3(0.62, 0.74, 0.86), fog * fog), 1.0);
})";
// Post-process: Brown-Conrady lens distortion and analog video noise, as in
// docs/INTEGRATION.md#lens-distortion.
static const char *kPostVs = R"(#version 330 core
out vec2 uv;
void main() { vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2); uv = p; gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0); })";
static const char *kPostFs = R"(#version 330 core
in vec2 uv; uniform sampler2D scene; uniform vec2 k; uniform float zoom; uniform float aspect;
uniform float noise; uniform float breakup; uniform float time; uniform float analog; out vec4 o;
float hash(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }
void main() {
  vec2 scale = vec2(aspect, 1.0) / (0.5 * sqrt(aspect * aspect + 1.0)); // r = 1 at the corners
  vec2 c = (uv - 0.5) * scale;
  float r2 = dot(c, c);
  vec2 d = c * (1.0 + k.x * r2 + k.y * r2 * r2) / zoom;
  vec2 suv = d / scale + 0.5;
  if (breakup > 0.0) { // rolling tear lines
    float line = step(1.0 - breakup * 0.25, hash(vec2(floor(uv.y * 120.0), floor(time * 20.0))));
    suv.x += line * (hash(vec2(time, uv.y)) - 0.5) * 0.15;
  }
  vec3 col = (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) ? vec3(0.0) : texture(scene, suv).rgb;
  float n = hash(uv * 513.0 + time) - 0.5;
  col += n * noise * 0.9 * analog;
  col = mix(col, vec3(dot(col, vec3(0.3, 0.59, 0.11))), noise * 0.6 * analog);
  o = vec4(col, 1.0);
})";

// --- input mapping ----------------------------------------------------------------------

static int toFpvKey(SDL_Keycode k) {
    if (k >= SDLK_A && k <= SDLK_Z) return 'A' + (int)(k - SDLK_A);
    if (k >= SDLK_0 && k <= SDLK_9) return '0' + (int)(k - SDLK_0);
    if (k >= SDLK_F1 && k <= SDLK_F12) return FPV_KEY_F1 + (int)(k - SDLK_F1);
    switch (k) {
    case SDLK_TAB: return FPV_KEY_TAB;
    case SDLK_RETURN: return FPV_KEY_ENTER;
    case SDLK_ESCAPE: return FPV_KEY_ESCAPE;
    case SDLK_SPACE: return FPV_KEY_SPACE;
    case SDLK_LEFT: return FPV_KEY_LEFT;
    case SDLK_RIGHT: return FPV_KEY_RIGHT;
    case SDLK_UP: return FPV_KEY_UP;
    case SDLK_DOWN: return FPV_KEY_DOWN;
    case SDLK_PAGEUP: return FPV_KEY_PAGE_UP;
    case SDLK_PAGEDOWN: return FPV_KEY_PAGE_DOWN;
    case SDLK_HOME: return FPV_KEY_HOME;
    case SDLK_END: return FPV_KEY_END;
    case SDLK_INSERT: return FPV_KEY_INSERT;
    case SDLK_DELETE: return FPV_KEY_DELETE;
    case SDLK_BACKSPACE: return FPV_KEY_BACKSPACE;
    case SDLK_LSHIFT: return FPV_KEY_LSHIFT;
    case SDLK_RSHIFT: return FPV_KEY_RSHIFT;
    case SDLK_LCTRL: return FPV_KEY_LCTRL;
    case SDLK_RCTRL: return FPV_KEY_RCTRL;
    case SDLK_LALT: return FPV_KEY_LALT;
    case SDLK_RALT: return FPV_KEY_RALT;
    default: return FPV_KEY_NONE;
    }
}

static void logCb(void *, fpv_log_level level, const char *msg) {
    if (level >= FPV_LOG_INFO) SDL_Log("[libfpv] %s", msg);
}

static void SDLCALL audioCb(void *, SDL_AudioStream *stream, int additional, int) {
    static std::vector<float> buf;
    int frames = additional / (int)(2 * sizeof(float));
    if (frames <= 0) return;
    buf.resize((size_t)frames * 2);
    fpv_audio_read(buf.data(), (uint32_t)frames);
    SDL_PutAudioStreamData(stream, buf.data(), frames * 2 * (int)sizeof(float));
}

static void savePpm(const char *path, int w, int h) {
    std::vector<unsigned char> px((size_t)w * h * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
    FILE *f = std::fopen(path, "wb");
    if (!f) return;
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int y = h - 1; y >= 0; y--) std::fwrite(&px[(size_t)y * w * 3], 1, (size_t)w * 3, f);
    std::fclose(f);
}

// --- main -------------------------------------------------------------------------------

int main(int argc, char **argv) {
    std::string dataDir = "data", screenshot;
    int frames = 0, width = 1280, height = 720, camMode = 0;
    bool autopilot = false, openOverlay = false;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto nextArg = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--data") dataDir = nextArg();
        else if (a == "--screenshot") screenshot = nextArg();
        else if (a == "--frames") frames = std::atoi(nextArg().c_str());
        else if (a == "--autopilot") autopilot = true;
        else if (a == "--overlay") openOverlay = true;
        else if (a == "--width") width = std::atoi(nextArg().c_str());
        else if (a == "--height") height = std::atoi(nextArg().c_str());
        else if (a == "--camera") {
            std::string c = nextArg();
            camMode = c == "chase" ? 1 : c == "los" ? 2 : 0;
        }
    }

    // The game's own SDL (video + audio). libfpv uses a separate, hidden SDL
    // instance for radios; the game does not initialise joysticks for it.
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO)) {
        std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_Window *win = SDL_CreateWindow("libfpv standalone", width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (!win) {
        std::fprintf(stderr, "window: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GLContext gl = SDL_GL_CreateContext(win);
    if (!gl || !gl_min_load((void *(*)(const char *))SDL_GL_GetProcAddress)) {
        std::fprintf(stderr, "OpenGL 3.3: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_SetSwapInterval(screenshot.empty() ? 1 : 0);
    SDL_StartTextInput(win);

    // 1. libfpv
    fpv_global_config cfg;
    fpv_global_config_defaults(&cfg);
    cfg.data_dir = dataDir.c_str();
    cfg.coords = fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1.0f);  // OpenGL: X right, Y up, -Z forward
    cfg.log = logCb;
    if (fpv_global_init(&cfg) != FPV_OK) {
        std::fprintf(stderr, "libfpv: %s\n", fpv_last_error());
        return 1;
    }

    // 2. world
    Mesh level;
    buildLevel(level);
    level.upload();
    fpv_mesh lm = level.collision();
    fpv_static_surfaces_load(&lm, 1);
    Mesh platform;
    platform.box({0, 0, 0}, {2.0f, 0.15f, 2.0f}, 0, {0.2f, 0.45f, 0.9f});
    platform.upload();
    fpv_transform pt{};
    pt.rotation[3] = 1;
    pt.position[0] = 12, pt.position[1] = 2, pt.position[2] = -25;
    uint32_t platformId = 0;
    fpv_mesh pm = platform.collision();
    fpv_surface_object_create(&pm, &pt, &platformId);

    // 3. drone
    fpv_drone_desc dd;
    fpv_drone_desc_defaults(&dd);
    dd.position[0] = 0, dd.position[1] = 0, dd.position[2] = 0;
    fpv_drone *drone = nullptr;
    if (fpv_drone_create(&dd, &drone) != FPV_OK) {
        std::fprintf(stderr, "drone: %s\n", fpv_last_error());
        return 1;
    }
    // the home arrow and distance point at the launch pad, wherever the drone
    // is armed (without this, home is where it last armed)
    fpv_gps_set_home(dd.position);
    fpv_frame frame;
    std::memset(&frame, 0, sizeof frame);
    frame.struct_size = sizeof frame;
    fpv_drone_tick(drone, 0, &frame);
    Mesh body, prop;
    buildDrone(body, prop, frame);
    body.upload();
    prop.upload();

    // renderers
    GLuint sceneProg = program(kSceneVs, kSceneFs), postProg = program(kPostVs, kPostFs);
    GLuint emptyVao;
    glGenVertexArrays(1, &emptyVao);
    fpv_gl3 ui;
    if (!fpv_gl3_init(&ui)) std::fprintf(stderr, "overlay renderer failed\n");
    GLuint fbo = 0, fboTex = 0, fboDepth = 0;
    int fboW = 0, fboH = 0;

    // 6. audio
    SDL_AudioSpec spec{SDL_AUDIO_F32, 2, 48000};
    SDL_AudioStream *audio = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, audioCb, nullptr);
    if (audio) {
        fpv_audio_configure(48000, 2);
        SDL_ResumeAudioStreamDevice(audio);
    }
    if (openOverlay) fpv_overlay_set_open(1);

    double propAngle[4] = {0, 0, 0, 0};
    Uint64 last = SDL_GetPerformanceCounter();
    double time = 0;
    bool running = true;
    int frameNo = 0;
    while (running) {
        // 5. input
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
            case SDL_EVENT_QUIT: running = false; break;
            case SDL_EVENT_KEY_DOWN:
            case SDL_EVENT_KEY_UP: {
                int k = toFpvKey(e.key.key);
                bool down = e.type == SDL_EVENT_KEY_DOWN;
                // fullscreen toggle; Alt+Enter must not reach libfpv (Enter arms)
                bool altEnter = (e.key.key == SDLK_RETURN || e.key.key == SDLK_KP_ENTER) && (e.key.mod & SDL_KMOD_ALT);
                if (altEnter || e.key.key == SDLK_F11) {
                    if (down && !e.key.repeat)
                        SDL_SetWindowFullscreen(win, !(SDL_GetWindowFlags(win) & SDL_WINDOW_FULLSCREEN));
                    break;
                }
                if (k != FPV_KEY_NONE && !e.key.repeat) fpv_input_key(k, down);
                if (down && !fpv_overlay_wants_input()) {  // the game's own keys
                    if (e.key.key == SDLK_ESCAPE) running = false;
                    if (e.key.key == SDLK_V) camMode = (camMode + 1) % 3;
                    if (e.key.key == SDLK_R) {
                        fpv_drone_repair(drone);
                        fpv_drone_reset(drone, dd.position, dd.rotation);
                    }
                }
                break;
            }
            case SDL_EVENT_TEXT_INPUT: fpv_overlay_text(e.text.text); break;
            case SDL_EVENT_MOUSE_MOTION: fpv_overlay_mouse_move(e.motion.x, e.motion.y); break;
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
            case SDL_EVENT_MOUSE_BUTTON_UP: {
                int b = e.button.button == SDL_BUTTON_LEFT ? 0 : e.button.button == SDL_BUTTON_RIGHT ? 1 : 2;
                fpv_overlay_mouse_button(b, e.type == SDL_EVENT_MOUSE_BUTTON_DOWN);
                break;
            }
            case SDL_EVENT_MOUSE_WHEEL: fpv_overlay_mouse_wheel(e.wheel.x, e.wheel.y); break;
            default: break;
            }
        }
        Uint64 now = SDL_GetPerformanceCounter();
        float dt = (float)((double)(now - last) / (double)SDL_GetPerformanceFrequency());
        last = now;
        if (!screenshot.empty()) dt = 1.0f / 60.0f;
        time += dt;

        if (autopilot) {
            // scripted pilot: arm, climb, fly forward through the gate loop
            float ch[8] = {0, 0, -1, 0, -1, 1, -1, -1};
            if (time > 0.5) ch[4] = 1;
            if (time > 1.0) ch[2] = 0.1f;
            if (time > 2.0) {
                ch[2] = -0.25f;
                ch[1] = 0.35f;
                ch[3] = 0.12f;
            }
            fpv_input_set_channels(ch, 8);
            if (!screenshot.empty()) SDL_Delay(16);  // real-time pacing for the sim thread
        }

        // moving platform: up and down 2..6 m
        pt.position[1] = 4 + 2 * (float)std::sin(time * 0.6);
        fpv_surface_object_move(platformId, &pt);

        // 4. tick
        fpv_drone_tick(drone, dt, &frame);
        for (int i = 0; i < 4; i++)
            propAngle[i] += frame.motors[i].rpm / 60.0 * 6.2831853 * dt * frame.motors[i].spin_direction;
        for (uint32_t i = 0; i < frame.impact_count; i++)
            if (frame.impacts[i].speed > 4) SDL_Log("impact %.1f m/s", frame.impacts[i].speed);

        int fbw, fbh;
        SDL_GetWindowSizeInPixels(win, &fbw, &fbh);
        int ww, wh;
        SDL_GetWindowSize(win, &ww, &wh);
        if (fbw != fboW || fbh != fboH) {
            if (fbo) {
                glDeleteFramebuffers(1, &fbo);
                glDeleteTextures(1, &fboTex);
                glDeleteRenderbuffers(1, &fboDepth);
            }
            glGenFramebuffers(1, &fbo);
            glBindFramebuffer(GL_FRAMEBUFFER, fbo);
            glGenTextures(1, &fboTex);
            glBindTexture(GL_TEXTURE_2D, fboTex);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, fbw, fbh, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTex, 0);
            glGenRenderbuffers(1, &fboDepth);
            glBindRenderbuffer(GL_RENDERBUFFER, fboDepth);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, fbw, fbh);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, fboDepth);
            fboW = fbw, fboH = fbh;
        }

        // scene into the offscreen target
        const fpv_camera &cam = camMode == 0 ? frame.camera : camMode == 1 ? frame.chase_camera : frame.los_camera;
        float aspect = (float)fbw / (float)std::max(1, fbh);
        M4 viewProj = mul(perspective(cam.fov_y_degrees, aspect, std::max(0.01f, cam.near_plane), 2000), viewFromCamera(cam));
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, fbw, fbh);
        glClearColor(0.62f, 0.74f, 0.86f, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glUseProgram(sceneProg);
        glUniformMatrix4fv(glGetUniformLocation(sceneProg, "viewProj"), 1, GL_FALSE, viewProj.m);
        glUniform3f(glGetUniformLocation(sceneProg, "eye"), cam.position[0], cam.position[1], cam.position[2]);
        GLint locModel = glGetUniformLocation(sceneProg, "model");
        M4 I = identity();
        glUniformMatrix4fv(locModel, 1, GL_FALSE, I.m);
        level.draw();
        M4 pmod = translate({pt.position[0], pt.position[1], pt.position[2]});
        glUniformMatrix4fv(locModel, 1, GL_FALSE, pmod.m);
        platform.draw();
        if (camMode != 0) {  // the FPV camera sits inside the frame
            M4 dm = mul(translate({frame.position[0], frame.position[1], frame.position[2]}), fromQuat(frame.rotation));
            glUniformMatrix4fv(locModel, 1, GL_FALSE, dm.m);
            body.draw();
            for (int i = 0; i < 4; i++) {
                if (frame.motors[i].flags & FPV_MOTOR_PROP_BROKEN) continue;
                const float *mp = frame.motors[i].position;
                M4 pm4 = mul(dm, mul(translate({mp[0], mp[1], mp[2]}), rotY((float)propAngle[i])));
                glUniformMatrix4fv(locModel, 1, GL_FALSE, pm4.m);
                prop.draw();
            }
        }

        // post-process to the window: lens distortion + analog noise
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, fbw, fbh);
        glDisable(GL_DEPTH_TEST);
        glUseProgram(postProg);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, fboTex);
        glUniform1i(glGetUniformLocation(postProg, "scene"), 0);
        bool fpv = camMode == 0;
        glUniform2f(glGetUniformLocation(postProg, "k"), fpv ? frame.camera.distortion_k1 : 0, fpv ? frame.camera.distortion_k2 : 0);
        glUniform1f(glGetUniformLocation(postProg, "zoom"), fpv ? frame.camera.distortion_zoom : 1.0f);
        glUniform1f(glGetUniformLocation(postProg, "aspect"), aspect);
        glUniform1f(glGetUniformLocation(postProg, "noise"), fpv ? frame.video.noise : 0);
        glUniform1f(glGetUniformLocation(postProg, "breakup"), fpv ? frame.video.breakup : 0);
        glUniform1f(glGetUniformLocation(postProg, "analog"), frame.video.is_digital ? 0.0f : 1.0f);
        glUniform1f(glGetUniformLocation(postProg, "time"), (float)time);
        glBindVertexArray(emptyVao);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        // OSD (FPV view only) and overlay, in window coordinates
        fpv_overlay_set_display((float)ww, (float)wh, (float)fbw / (float)std::max(1, ww));
        if (fpv) fpv_gl3_render(&ui, fpv_osd_draw_data(&frame, (float)ww, (float)wh, FPV_OSD_STYLE_ANALOG), fbw, fbh);
        fpv_gl3_render(&ui, fpv_overlay_render(dt), fbw, fbh);

        frameNo++;
        if (!screenshot.empty() && frameNo >= frames) {
            savePpm(screenshot.c_str(), fbw, fbh);
            SDL_Log("saved %s (sim %.2f s, %s, height %.1f m, %.0f km/h)", screenshot.c_str(), frame.sim_time,
                    (frame.flags & FPV_DRONE_ARMED) ? "armed" : "disarmed", frame.position[1], frame.speed_kmh);
            running = false;
        }
        SDL_GL_SwapWindow(win);
    }

    // 7. shutdown
    if (audio) SDL_DestroyAudioStream(audio);
    fpv_drone_delete(drone);
    fpv_global_terminate();
    fpv_gl3_shutdown(&ui);
    SDL_GL_DestroyContext(gl);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
