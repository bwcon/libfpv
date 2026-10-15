// libfpv - built-in Dear ImGui overlay.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "overlay/overlay.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/bf_bridge.h"
#include "imgui.h"
#include "lib.h"
#include "osd/osd.h"
#include "util/log.h"

namespace fpv {

OsdRenderer &osdRenderer();  // api_present.cpp

Overlay &overlay() {
    Lib &L = lib();
    if (!L.overlay) L.overlay = new Overlay();
    return *L.overlay;
}

namespace {

ImGuiKey toImGuiKey(int k) {
    if (k >= 'A' && k <= 'Z') return (ImGuiKey)(ImGuiKey_A + (k - 'A'));
    if (k >= 'a' && k <= 'z') return (ImGuiKey)(ImGuiKey_A + (k - 'a'));
    if (k >= '0' && k <= '9') return (ImGuiKey)(ImGuiKey_0 + (k - '0'));
    switch (k) {
    case FPV_KEY_TAB: return ImGuiKey_Tab;
    case FPV_KEY_ENTER: return ImGuiKey_Enter;
    case FPV_KEY_ESCAPE: return ImGuiKey_Escape;
    case FPV_KEY_SPACE: return ImGuiKey_Space;
    case FPV_KEY_LEFT: return ImGuiKey_LeftArrow;
    case FPV_KEY_RIGHT: return ImGuiKey_RightArrow;
    case FPV_KEY_UP: return ImGuiKey_UpArrow;
    case FPV_KEY_DOWN: return ImGuiKey_DownArrow;
    case FPV_KEY_PAGE_UP: return ImGuiKey_PageUp;
    case FPV_KEY_PAGE_DOWN: return ImGuiKey_PageDown;
    case FPV_KEY_HOME: return ImGuiKey_Home;
    case FPV_KEY_END: return ImGuiKey_End;
    case FPV_KEY_INSERT: return ImGuiKey_Insert;
    case FPV_KEY_DELETE: return ImGuiKey_Delete;
    case FPV_KEY_BACKSPACE: return ImGuiKey_Backspace;
    case FPV_KEY_LSHIFT: return ImGuiKey_LeftShift;
    case FPV_KEY_RSHIFT: return ImGuiKey_RightShift;
    case FPV_KEY_LCTRL: return ImGuiKey_LeftCtrl;
    case FPV_KEY_RCTRL: return ImGuiKey_RightCtrl;
    case FPV_KEY_LALT: return ImGuiKey_LeftAlt;
    case FPV_KEY_RALT: return ImGuiKey_RightAlt;
    default: break;
    }
    if (k >= FPV_KEY_F1 && k <= FPV_KEY_F12) return (ImGuiKey)(ImGuiKey_F1 + (k - FPV_KEY_F1));
    return ImGuiKey_None;
}

const char *keyName(int k) {
    static char buf[16];
    if ((k >= 'A' && k <= 'Z') || (k >= '0' && k <= '9')) {
        buf[0] = (char)k;
        buf[1] = 0;
        return buf;
    }
    if (k >= FPV_KEY_F1 && k <= FPV_KEY_F12) {
        snprintf(buf, sizeof buf, "F%d", k - FPV_KEY_F1 + 1);
        return buf;
    }
    switch (k) {
    case FPV_KEY_TAB: return "Tab";
    case FPV_KEY_ENTER: return "Enter";
    case FPV_KEY_ESCAPE: return "Esc";
    case FPV_KEY_SPACE: return "Space";
    case FPV_KEY_INSERT: return "Insert";
    case FPV_KEY_DELETE: return "Delete";
    case FPV_KEY_HOME: return "Home";
    case FPV_KEY_END: return "End";
    case FPV_KEY_PAGE_UP: return "Page Up";
    case FPV_KEY_PAGE_DOWN: return "Page Down";
    default: break;
    }
    snprintf(buf, sizeof buf, "key %d", k);
    return buf;
}

// Plain-language reasons for Betaflight's arming-disable flags.
const char *armingHelp(const char *flag) {
    struct H {
        const char *f, *h;
    } static const help[] = {
        {"THROTTLE", "Throttle is not at the bottom."},
        {"ARM_SWITCH", "The arm switch was on when the drone became armable: switch it off, then on again."},
        {"ANGLE", "The drone is tilted too much (more than small_angle). Reset it upright."},
        {"RXLOSS", "No radio signal. Check the Radio tab."},
        {"FAILSAFE", "Failsafe is active: the radio signal was lost."},
        {"CALIB", "Sensors are calibrating; keep the drone still for a moment."},
        {"CLI", "The Betaflight Configurator is in its CLI tab."},
        {"MSP", "The Configurator is connected and blocking arming (normal while connected)."},
        {"BOOTGRACE", "Betaflight's power-on delay."},
        {"NOPREARM", "A PREARM switch is configured and not on."},
        {"CRASH", "Crash detection triggered: disarm and re-arm."},
        {"RUNAWAY", "Runaway takeoff prevention triggered: disarm and re-arm."},
        {"REBOOT_REQD", "A setting needs a reboot: libfpv reboots Betaflight when you import; otherwise press Reboot in the Configurator tab."},
        {"MOTOR_PROTO", "Motor protocol was changed: only PWM works in the simulator (libfpv restores it on next start)."},
        {"PARALYZE", "PARALYZE mode is active."},
        {"LOAD", "CPU load too high."},
        {"BOMB", "The bomb arm switch is on (or the drone was destroyed): the motors only arm with the bomb safe. "
                 "Switch it off, then switch the arm switch off and on again."},
    };
    for (auto &x : help)
        if (std::strcmp(x.f, flag) == 0) return x.h;
    return "";
}

void sectionTitle(const char *t) {
    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.55f, 0.8f, 1.0f, 1.0f), "%s", t);
    ImGui::Separator();
}

}  // namespace

Overlay::Overlay() {
    m_bf.watch("rates_type");
    m_bf.watch("_rc_rate");
    m_bf.watch("_srate");
    m_bf.watch("_expo");
    m_bf.watch("_rate_limit");
    m_bf.watch("quickrates_rc_expo");
    m_bf.watch("thr_mid");
    m_bf.watch("thr_expo");
    m_bf.watch("simplified_");
    m_bf.watch("angle_limit");
    m_bf.watch("osd_");
    m_bf.watch("yaw_motors_reversed");
    m_bf.watch("craft_name");
}

void Overlay::ensureContext() {
    if (m_ctx) return;
    IMGUI_CHECKVERSION();
    ImGuiContext *prev = ImGui::GetCurrentContext();
    m_ctx = ImGui::CreateContext();
    ImGui::SetCurrentContext(m_ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = nullptr;  // nothing written next to the game
    io.LogFilename = nullptr;
    io.BackendPlatformName = "libfpv";
    io.BackendRendererName = "libfpv-host";
    io.MouseDrawCursor = true;  // games usually hide the OS cursor
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGuiStyle &st = ImGui::GetStyle();
    st.WindowRounding = 6;
    st.FrameRounding = 4;
    st.GrabRounding = 4;
    rebuildFonts();
    ImGui::SetCurrentContext(prev);
}

void Overlay::rebuildFonts() {
    ImGuiIO &io = ImGui::GetIO();
    ImFontConfig cfg;
    cfg.SizePixels = 32.0f;  // baked large, drawn at 16 * ui_scale
    io.Fonts->AddFontDefaultVector(&cfg);
    unsigned char *px = nullptr;
    io.Fonts->GetTexDataAsRGBA32(&px, &m_fontW, &m_fontH);
    m_fontPixels.assign(px, px + (size_t)m_fontW * m_fontH * 4);
    io.Fonts->SetTexID((ImTextureID)FPV_TEXTURE_OVERLAY_FONT);
    ImGui::GetStyle().FontSizeBase = 16.0f;
    m_fontRevision++;
}

bool Overlay::fontTexture(fpv_texture &out) {
    ensureContext();
    if (m_fontPixels.empty()) return false;
    out.width = (uint32_t)m_fontW;
    out.height = (uint32_t)m_fontH;
    out.rgba = m_fontPixels.data();
    out.revision = m_fontRevision;
    return true;
}

void Overlay::setOpen(bool open) {
    if (open == m_open) return;
    m_open = open;
    if (open) m_bf.invalidate();
    if (!open && m_wizard.active() && m_wizard.step() != RadioWizard::Step::Done) {
        m_wizard.cancel();
        lib().input.setWizardActive(false);
    }
}

void Overlay::setDisplay(float w, float h, float scale) {
    if (w > 0) m_w = w;
    if (h > 0) m_h = h;
    if (scale > 0 && std::isfinite(scale)) m_scale = std::clamp(scale, 0.5f, 4.0f);
}

#define WITH_CTX                                     \
    ensureContext();                                 \
    ImGuiContext *prev_ = ImGui::GetCurrentContext(); \
    ImGui::SetCurrentContext(m_ctx)
#define END_CTX ImGui::SetCurrentContext(prev_)

void Overlay::mouseMove(float x, float y) {
    WITH_CTX;
    ImGui::GetIO().AddMousePosEvent(x, y);
    END_CTX;
}
void Overlay::mouseButton(int b, bool down) {
    if (b < 0 || b > 4) return;
    WITH_CTX;
    ImGui::GetIO().AddMouseButtonEvent(b, down);
    END_CTX;
}
void Overlay::mouseWheel(float dx, float dy) {
    WITH_CTX;
    ImGui::GetIO().AddMouseWheelEvent(dx, dy);
    END_CTX;
}
void Overlay::text(const char *utf8) {
    if (!m_open) return;
    WITH_CTX;
    ImGui::GetIO().AddInputCharactersUTF8(utf8);
    END_CTX;
}

bool Overlay::capturingKey() const { return m_rebindingToggle; }

void Overlay::key(int k, bool down) {
    if (m_rebindingToggle && down) {
        if (k != FPV_KEY_ESCAPE) {
            lib().settings.overlayToggleKey = k;
            saveSettings();
        }
        m_rebindingToggle = false;
        return;
    }
    WITH_CTX;
    ImGuiIO &io = ImGui::GetIO();
    ImGuiKey ik = toImGuiKey(k);
    if (ik != ImGuiKey_None) io.AddKeyEvent(ik, down);
    if (k == FPV_KEY_LCTRL || k == FPV_KEY_RCTRL) io.AddKeyEvent(ImGuiMod_Ctrl, down);
    if (k == FPV_KEY_LSHIFT || k == FPV_KEY_RSHIFT) io.AddKeyEvent(ImGuiMod_Shift, down);
    if (k == FPV_KEY_LALT || k == FPV_KEY_RALT) io.AddKeyEvent(ImGuiMod_Alt, down);
    END_CTX;
}

void Overlay::notifyUnmappedRadio(const std::string &guid) {
    if (std::find(m_promptedRadios.begin(), m_promptedRadios.end(), guid) != m_promptedRadios.end()) return;
    m_promptedRadios.push_back(guid);
    setOpen(true);
    m_requestTab = 1;  // Radio
}

// --- tabs ------------------------------------------------------------------------

void Overlay::channelBars(const float *ch) {
    static const char *names[kChannelCount] = {"Roll",        "Pitch",       "Throttle",     "Yaw",
                                               "AUX1 arm",    "AUX2 angle",  "AUX3 beeper",  "AUX4 turtle",
                                               "AUX5 air mode", "AUX6 OSD off"
#if defined(LIBFPV_KAMIKAZE)
                                               , "AUX7 bomb arm", "AUX8 explode"
#endif
    };
    for (int i = 0; i < kChannelCount; i++) {
        char label[32];
        snprintf(label, sizeof label, "%d", (int)std::lround(1500 + ch[i] * 500));
        ImGui::ProgressBar((ch[i] + 1) * 0.5f, ImVec2(ImGui::GetFontSize() * 14, 0), label);
        ImGui::SameLine();
        ImGui::TextUnformatted(names[i]);
    }
}

void Overlay::tabFly() {
    Lib &L = lib();
    const fpv_frame &f = L.lastFrame;
    if (!L.droneExists) {
        ImGui::TextWrapped("No drone is spawned. The game spawns it; settings below still apply.");
    } else {
        bool armed = f.flags & FPV_DRONE_ARMED;
        ImGui::TextColored(armed ? ImVec4(1, 0.4f, 0.3f, 1) : ImVec4(0.5f, 1, 0.5f, 1), "%s",
                           armed ? "ARMED" : "Disarmed");
        if (!armed && f.arming_disable_text[0]) {
            sectionTitle("Why it does not arm");
            char flags[96];
            snprintf(flags, sizeof flags, "%s", f.arming_disable_text);
            for (char *tok = std::strtok(flags, " "); tok; tok = std::strtok(nullptr, " ")) {
                const char *h = armingHelp(tok);
                ImGui::BulletText("%s%s%s", tok, h[0] ? ": " : "", h);
            }
        } else if (!armed) {
            ImGui::TextWrapped("Ready: throttle down, flip the arm switch.");
        }
        if (f.flags & FPV_DRONE_FAILSAFE) ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "FAILSAFE");
#if defined(LIBFPV_KAMIKAZE)
        {
            bool installed = fpv_kamikaze_is_installed() != 0;
            if (ImGui::Checkbox("Bomb installed", &installed)) fpv_kamikaze_set_installed(installed);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Carry the warhead: its weight, its OSD line and the bomb arm (AUX7) and explode "
                                  "(AUX8) switches. Kept for this session, also on respawned drones.");
            fpv_kamikaze_state ks{};
            ks.struct_size = sizeof ks;
            if (fpv_kamikaze_get_state(&L.drone, &ks) == FPV_OK && (installed || ks.warhead == FPV_WARHEAD_DETONATED)) {
                static const char *names[] = {"SAFE", "ARMING", "ARMED (live)", "DETONATED"};
                ImVec4 col = ks.warhead == FPV_WARHEAD_SAFE ? ImVec4(0.5f, 1, 0.5f, 1) : ImVec4(1, 0.4f, 0.3f, 1);
                ImGui::TextColored(col, "Warhead: %s", names[ks.warhead]);
                if (ks.warhead == FPV_WARHEAD_ARMING) {
                    ImGui::SameLine();
                    ImGui::Text("(%.1f s)", ks.arming_remaining);
                }
                if (ks.warhead == FPV_WARHEAD_DETONATED) {
                    ImGui::SameLine();
                    ImGui::TextUnformatted("- press Repair for a new drone");
                }
            }
            if (installed) {
                // the payload is real weight; the player can tune it
                Settings &st = L.settings;
                ImGui::Text("Payload %.0f g, all-up weight %.0f g", ks.payload_mass * 1000, ks.total_mass * 1000);
                bool changed = false;
                bool autoMass = st.kamikazePayloadKg < 0;
                if (ImGui::Checkbox("Automatic payload (the quad's preset)", &autoMass)) {
                    st.kamikazePayloadKg = autoMass ? -1.0f : ks.payload_mass;
                    changed = true;
                }
                if (!autoMass) {
                    float g = st.kamikazePayloadKg * 1000;
                    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 14);
                    if (ImGui::SliderFloat("Payload (g)", &g, 0, 3000, "%.0f")) {
                        st.kamikazePayloadKg = g / 1000;
                        changed = true;
                    }
                }
                ImGui::SetNextItemWidth(ImGui::GetFontSize() * 14);
                if (ImGui::SliderFloat("Arming delay (s)", &st.kamikazeArmingDelay, 0, 10, "%.1f")) changed = true;
                ImGui::SetNextItemWidth(ImGui::GetFontSize() * 14);
                if (ImGui::SliderFloat("Impact fuse (m/s)", &st.kamikazeMinImpactSpeed, 0.1f, 30.0f, "%.2f",
                                       ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp))
                    changed = true;
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("The slowest touch that sets the live warhead off. 0.25 = a light tap on the "
                                      "ground; higher needs a real hit.");
                if (ImGui::Checkbox("Warhead state on the OSD", &st.kamikazeOsd)) changed = true;
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Its place: \"Bomb status\" in the OSD tab (\"Custom message 1\" in the "
                                      "Betaflight Configurator).");
                if (changed) {
                    applyKamikazeSettings();
                    saveSettings();
                }
            }
        }
#endif
        if (f.flags & FPV_DRONE_CRASHED)
            ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "Crashed (broken props or burned motor): press Repair.");
        sectionTitle("Drone");
        ImGui::Text("Speed %.0f km/h   Battery %.2f V (%.2f V/cell), %.0f mAh used", f.speed_kmh, f.battery.voltage,
                    f.battery.cell_voltage, f.battery.mah_drawn);
        for (int i = 0; i < 4; i++) {
            ImGui::Text("Motor %d: %5.0f rpm  %4.1f A  %3.0f C  prop %3.0f%%", i + 1, f.motors[i].rpm,
                        f.motors[i].current_amp, f.motors[i].temperature_c, (1 - f.motors[i].prop_damage) * 100);
        }
        if (ImGui::Button("Repair")) fpv_drone_repair(&L.drone);
        ImGui::SameLine();
        if (ImGui::Button("Respawn")) {
            float p[3], q[4];
            Vec3 hp = L.conv.posToHost(L.spawnPos);
            Quat hq = L.conv.rotToHost(L.spawnRot);
            p[0] = hp.x, p[1] = hp.y, p[2] = hp.z;
            q[0] = hq.x, q[1] = hq.y, q[2] = hq.z, q[3] = hq.w;
            fpv_drone_repair(&L.drone);
            fpv_drone_reset(&L.drone, p, q);
        }
        ImGui::SameLine();
        bool paused = L.runner.paused();
        if (ImGui::Checkbox("Pause", &paused)) L.runner.setPaused(paused);
    }
    if (!L.hostActions.empty()) {
        sectionTitle("Game");
        for (size_t i = 0; i < L.hostActions.size(); i++) {
            const Lib::HostAction &a = L.hostActions[i];
            if (i) ImGui::SameLine();
            ImGui::PushID((int)a.id);
            if (ImGui::Button(a.label.c_str())) L.clickedActions.push_back(a.id);
            ImGui::PopID();
        }
    }
    sectionTitle("Input");
    InputStatus st = L.input.status();
    static const char *src[] = {"none", "radio", "gamepad (degraded)", "keyboard (degraded)", "game/script"};
    ImGui::Text("Source: %s - %s", src[(int)st.source], st.deviceName.c_str());
    if (st.needsSetup) {
        ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "A radio is connected but not set up.");
        ImGui::SameLine();
        if (ImGui::SmallButton("Set it up")) m_requestTab = 1;
    }
    ImGui::TextDisabled("Overlay: %s", keyName(L.settings.overlayToggleKey));
}

void Overlay::tabRadio() {
    Lib &L = lib();
    std::vector<RawDevice> devs = L.input.devicesSnapshot();
    if (m_wizard.active()) {
        const RawDevice *dev = nullptr;
        for (auto &d : devs)
            if (d.guid == m_wizard.guid()) dev = &d;
        if (!dev) {
            ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), "The radio was disconnected. Plug it back in to continue.");
            if (ImGui::Button("Cancel setup")) {
                m_wizard.cancel();
                L.input.setWizardActive(false);
            }
            return;
        }
        m_wizard.update(*dev);
        ImGui::ProgressBar(m_wizard.progress(), ImVec2(-1, 0), "");
        ImGui::TextColored(ImVec4(0.55f, 0.8f, 1, 1), "%s", m_wizard.title());
        ImGui::TextWrapped("%s", m_wizard.instruction());
        ImGui::Spacing();
        if (m_wizard.step() != RadioWizard::Step::Done && m_wizard.step() != RadioWizard::Step::Rest &&
            m_wizard.step() != RadioWizard::Step::Calibrate) {
            if (m_wizard.detected())
                ImGui::TextColored(ImVec4(0.5f, 1, 0.5f, 1), "Detected%s%s", m_showRawAxes ? ": " : "",
                                   m_showRawAxes ? m_wizard.detectedLabel().c_str() : "");
            else
                ImGui::TextDisabled("Waiting for movement...");
        }
        if (m_wizard.step() == RadioWizard::Step::Done) {
            if (!m_wizardSaved) {
                // persisted once; flight input switches to the mapped radio
                L.input.setMapping(m_wizard.mapping());
                L.input.setWizardActive(false);
                m_wizardSaved = true;
            }
            channelBars(L.lastFrame.channels);
            if (ImGui::Button("Finish")) m_wizard.cancel();
            ImGui::SameLine();
            if (ImGui::Button("Start over")) {
                L.input.forgetMapping(m_wizard.guid());
                m_wizard.start(*dev);
                m_wizardSaved = false;
                L.input.setWizardActive(true);
            }
        } else {
            if (ImGui::Button("Back")) m_wizard.back();
            ImGui::SameLine();
            if (ImGui::Button("Next")) m_wizard.next(m_wizardHint);
            if (m_wizard.canSkip()) {
                ImGui::SameLine();
                if (ImGui::Button("Skip")) m_wizard.skip();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) {
                m_wizard.cancel();
                L.input.setWizardActive(false);
            }
            if (!m_wizardHint.empty()) ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "%s", m_wizardHint.c_str());
        }
        ImGui::Checkbox("Show raw axes", &m_showRawAxes);
        if (m_showRawAxes) {
            for (size_t i = 0; i < dev->axes.size(); i++) {
                char lbl[16];
                snprintf(lbl, sizeof lbl, "axis %zu", i + 1);
                ImGui::ProgressBar((dev->axes[i] + 1) * 0.5f, ImVec2(ImGui::GetFontSize() * 10, 0), lbl);
            }
        }
        return;
    }

    if (devs.empty()) {
        ImGui::TextWrapped(
            "No radio or gamepad detected. Connect your radio with USB and choose \"USB Joystick (HID)\" on the radio "
            "(EdgeTX/OpenTX), or plug in a simulator dongle. Keyboard works as a fallback: W/S throttle, A/D yaw, "
            "arrow keys pitch/roll, Enter arms.");
    }
    for (auto &d : devs) {
        ImGui::PushID(d.guid.c_str());
        DeviceMapping m;
        bool mapped = L.input.getMapping(d.guid, m) && m.complete;
        ImGui::Text("%s", d.name.c_str());
        ImGui::SameLine();
        if (mapped) ImGui::TextColored(ImVec4(0.5f, 1, 0.5f, 1), "(set up)");
        else if (d.isGamepad) ImGui::TextDisabled("(gamepad: default layout, degraded)");
        else ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "(not set up)");
        if (ImGui::Button(mapped ? "Set up again" : "Set up")) {
            m_wizard.start(d);
            m_wizardHint.clear();
            m_wizardSaved = false;
            L.input.setWizardActive(true);
        }
        if (mapped) {
            ImGui::SameLine();
            if (ImGui::Button("Forget")) L.input.forgetMapping(d.guid);
        }
        ImGui::PopID();
        ImGui::Separator();
    }
    sectionTitle("Channels Betaflight receives");
    channelBars(L.lastFrame.channels);
}

void Overlay::tabDrone() {
    Lib &L = lib();
    bool armed = L.droneExists && (L.lastFrame.flags & FPV_DRONE_ARMED);
    for (size_t i = 0; i < L.presets.size(); i++) {
        const DronePreset &p = L.presets[i];
        ImGui::PushID((int)i);
        bool sel = (int)i == L.presetIndex;
        if (ImGui::RadioButton(p.name.c_str(), sel) && !sel) {
            if (armed) {
                m_status = "Disarm before switching drones.";
            } else {
                selectPreset(p.fileName, true);
                m_status = "Switched to " + p.name;
            }
        }
        ImGui::PopID();
    }
    if (!m_status.empty()) ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "%s", m_status.c_str());
    const DronePreset &p = currentPreset();
    sectionTitle(p.name.c_str());
    ImGui::TextWrapped("%s", p.description.c_str());
    const StateInit &s = p.init;
    float arm = std::sqrt(s.quadMotorPos[0].x * s.quadMotorPos[0].x + s.quadMotorPos[0].z * s.quadMotorPos[0].z);
    ImGui::Text("All-up weight   %.0f g", s.quadMass * 1000);
    ImGui::Text("Motors          %.0f kV", s.motorKV[0]);
    ImGui::Text("Propellers      %.1f\" %d-blade", p.collision.propRadius * 2 / 0.0254f, (int)p.propBladeCount);
    ImGui::Text("Battery         %dS %.0f mAh", (int)s.quadBatCellCount, s.quadBatCapacity);
    ImGui::Text("Motor spacing   %.0f mm diagonal", arm * 2000);
    ImGui::Text("Video           %s", p.vtx.digital ? "digital (HD)" : "analog");
    ImGui::Spacing();
    ImGui::TextWrapped("Physical parameters live in JSON files you can copy and edit: %s/quad (restart the game to "
                       "reload). Each drone keeps its own Betaflight configuration.",
                       L.presetDir.c_str());
}

void Overlay::tabRates() {
    Lib &L = lib();
    m_bf.refresh(wallSeconds());
    static const char *types[] = {"BETAFLIGHT", "RACEFLIGHT", "KISS", "ACTUAL", "QUICK"};
    std::string cur = m_bf.str("rates_type", "ACTUAL");
    int t = 3;
    for (int i = 0; i < 5; i++)
        if (cur == types[i]) t = i;
    if (ImGui::Combo("Rates type", &t, types, 5)) m_bf.set("rates_type", types[t], true);
    static const char *axes[] = {"roll", "pitch", "yaw"};
    ImGui::RadioButton("Roll", &m_rateAxis, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Pitch", &m_rateAxis, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Yaw", &m_rateAxis, 2);
    std::string ax = axes[m_rateAxis];
    // labels and ranges per rates type (as in Betaflight Configurator)
    const char *l1 = "RC rate", *l2 = "Super rate", *l3 = "Expo";
    int max1 = 255, max2 = 100, max3 = 100;
    if (t == 3) l1 = "Center sensitivity (x10 deg/s)", l2 = "Max rate (x10 deg/s)", max2 = 200;
    if (t == 4) l1 = "RC rate (x2 deg/s)", l2 = "Max rate (x10 deg/s)", max2 = 200;
    if (t == 1) l1 = "Rate (x10 deg/s)", l2 = "Acro+", max2 = 255;
    int rc = m_bf.num(ax + "_rc_rate", 7), sr = m_bf.num(ax + "_srate", 67), ex = m_bf.num(ax + "_expo", 0);
    auto slider = [&](const char *label, const std::string &name, int &v, int maxv) {
        if (ImGui::SliderInt(label, &v, 0, maxv)) m_bf.set(name, v, false);
        if (ImGui::IsItemDeactivatedAfterEdit()) m_bf.set(name, v, true);
    };
    slider(l1, ax + "_rc_rate", rc, max1);
    slider(l2, ax + "_srate", sr, max2);
    slider(l3, ax + "_expo", ex, max3);
    if (!m_bf.lastError().empty()) ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), "%s", m_bf.lastError().c_str());

    RateParams rp;
    rp.type = t;
    rp.rcRate = (float)rc;
    rp.rate = (float)sr;
    rp.expo = (float)ex;
    rp.limit = (float)m_bf.num(ax + "_rate_limit", 1998);
    rp.quickRcExpo = m_bf.str("quickrates_rc_expo", "OFF") == "ON";
    float maxRate = betaflightRate(rp, 1.0f);
    ImGui::Text("Max rate: %.0f deg/s", maxRate);

    // live curve with the current stick position
    ImVec2 size(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 12);
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImDrawList *dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), IM_COL32(20, 24, 30, 255));
    float top = std::max(200.0f, std::ceil(maxRate / 200.0f) * 200.0f);
    for (float g = 200; g < top; g += 200) {
        float y = p0.y + size.y * (1 - g / top);
        dl->AddLine(ImVec2(p0.x, y), ImVec2(p0.x + size.x, y), IM_COL32(60, 60, 70, 255));
    }
    ImVec2 prevPt;
    for (int i = 0; i <= 100; i++) {
        float x = i / 100.0f;
        ImVec2 pt(p0.x + x * size.x, p0.y + size.y * (1 - betaflightRate(rp, x) / top));
        if (i) dl->AddLine(prevPt, pt, IM_COL32(90, 200, 255, 255), 2.0f);
        prevPt = pt;
    }
    int chIdx[3] = {0, 1, 3};
    float stick = std::fabs(L.lastFrame.channels[chIdx[m_rateAxis]]);
    ImVec2 dot(p0.x + stick * size.x, p0.y + size.y * (1 - betaflightRate(rp, stick) / top));
    dl->AddCircleFilled(dot, 5, IM_COL32(255, 200, 60, 255));
    ImGui::Dummy(size);
    ImGui::TextDisabled("Grid: 200 deg/s. The dot follows your %s stick.", axes[m_rateAxis]);

    sectionTitle("Throttle");
    int mid = m_bf.num("thr_mid", 50), expo = m_bf.num("thr_expo", 0);
    slider("Throttle mid", "thr_mid", mid, 100);
    slider("Throttle expo", "thr_expo", expo, 100);
}

void Overlay::tabTuning() {
    m_bf.refresh(wallSeconds());
    ImGui::TextWrapped("Betaflight's simplified tuning. Sliders apply instantly; 100 is the default.");
    struct S {
        const char *label, *name;
        int lo, hi;
    } static const sliders[] = {
        {"Master multiplier", "simplified_master_multiplier", 0, 200},
        {"Damping (D gain)", "simplified_d_gain", 0, 200},
        {"Tracking (P & I gain)", "simplified_pi_gain", 0, 200},
        {"Stick response (feedforward)", "simplified_feedforward_gain", 0, 200},
        {"Dynamic damping (D max)", "simplified_d_max_gain", 0, 200},
        {"Drift / wobble (I gain)", "simplified_i_gain", 0, 200},
        {"Pitch damping", "simplified_pitch_d_gain", 0, 200},
        {"Pitch tracking", "simplified_pitch_pi_gain", 0, 200},
    };
    bool changed = false, release = false;
    for (auto &s : sliders) {
        int v = m_bf.num(s.name, 100);
        if (ImGui::SliderInt(s.label, &v, s.lo, s.hi)) {
            m_bf.set(s.name, v, false);
            changed = true;
        }
        if (ImGui::IsItemDeactivatedAfterEdit()) release = true;
    }
    sectionTitle("Filters");
    int gyro = m_bf.num("simplified_gyro_filter_multiplier", 100), dterm = m_bf.num("simplified_dterm_filter_multiplier", 100);
    if (ImGui::SliderInt("Gyro filter multiplier", &gyro, 10, 200)) {
        m_bf.set("simplified_gyro_filter_multiplier", gyro, false);
        changed = true;
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) release = true;
    if (ImGui::SliderInt("D-term filter multiplier", &dterm, 10, 200)) {
        m_bf.set("simplified_dterm_filter_multiplier", dterm, false);
        changed = true;
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) release = true;
    if (changed || release) m_bf.run(std::string("simplified_tuning apply\n") + (release ? "save\n" : ""));
    sectionTitle("Angle mode");
    int angle = m_bf.num("angle_limit", 60);
    if (ImGui::SliderInt("Max angle", &angle, 10, 85)) m_bf.set("angle_limit", angle, false);
    if (ImGui::IsItemDeactivatedAfterEdit()) m_bf.set("angle_limit", angle, true);
    if (ImGui::Button("Reset tuning to defaults")) {
        for (auto &s : sliders) m_bf.set(s.name, 100, false);
        m_bf.set("simplified_gyro_filter_multiplier", 100, false);
        m_bf.set("simplified_dterm_filter_multiplier", 100, false);
        m_bf.run("simplified_tuning apply\nsave\n");
    }
    ImGui::TextDisabled("Full PID and filter control: Betaflight Configurator (see its tab).");
}

void Overlay::tabOsd() {
    Lib &L = lib();
    m_bf.refresh(wallSeconds());
    int style = L.settings.osdStyle;
    if (ImGui::RadioButton("Analog look", style == FPV_OSD_STYLE_ANALOG)) style = FPV_OSD_STYLE_ANALOG;
    ImGui::SameLine();
    if (ImGui::RadioButton("HD look", style == FPV_OSD_STYLE_HD)) style = FPV_OSD_STYLE_HD;
    if (style != L.settings.osdStyle) {
        L.settings.osdStyle = style;
        saveSettings();
    }
    struct E {
        const char *label, *name;
    } static const elems[] = {
        {"Battery voltage", "osd_vbat_pos"},     {"Cell voltage", "osd_avg_cell_voltage_pos"},
        {"Current", "osd_current_pos"},          {"mAh used", "osd_mah_drawn_pos"},
        {"Flight time", "osd_tim_2_pos"},        {"On time", "osd_tim_1_pos"},
        {"Flight mode", "osd_flymode_pos"},      {"Throttle", "osd_throttle_pos"},
        {"Crosshairs", "osd_crosshairs_pos"},    {"Artificial horizon", "osd_ah_pos"},
        {"Horizon sidebars", "osd_ah_sbar_pos"}, {"Warnings", "osd_warnings_pos"},
        {"Craft name", "osd_craft_name_pos"},    {"Speed", "osd_gps_speed_pos"},
        {"Altitude", "osd_altitude_pos"},        {"Disarmed", "osd_disarmed_pos"},
        {"G-force", "osd_g_force_pos"},          {"Power", "osd_power_pos"},
        {"Stick overlay L", "osd_stick_overlay_left_pos"}, {"Stick overlay R", "osd_stick_overlay_right_pos"},
#if defined(LIBFPV_KAMIKAZE)
        {"Bomb status", "osd_custom_msg0_pos"},  // the warhead state (Flight::step)
#endif
    };
    const int n = (int)(sizeof(elems) / sizeof(elems[0]));
    ImGui::BeginChild("elems", ImVec2(ImGui::GetFontSize() * 13, 0), ImGuiChildFlags_Borders);
    for (int i = 0; i < n; i++) {
        int pos = m_bf.num(elems[i].name, 0);
        bool vis = (pos & 0x800) != 0;
        ImGui::PushID(i);
        if (ImGui::Checkbox("##v", &vis)) m_bf.set(elems[i].name, vis ? (pos | 0x800) : (pos & ~0x800), true);
        ImGui::SameLine();
        if (ImGui::Selectable(elems[i].label, m_osdSelected == i)) m_osdSelected = i;
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::TextWrapped("Select an element, then click where it should go. Live preview:");
    // preview grid drawn with the OSD font texture
    float cw = ImGui::GetFontSize() * 0.9f, chh = cw * 1.5f;
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 size(cw * FPV_OSD_COLS, chh * FPV_OSD_ROWS);
    ImDrawList *dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), IM_COL32(60, 70, 60, 255));
    const float aw = (float)OsdRenderer::kAtlasW, ah = (float)OsdRenderer::kAtlasH;
    int fontOff = style == FPV_OSD_STYLE_HD ? OsdRenderer::kGlyphW * 16 : 0;
    for (int r = 0; r < FPV_OSD_ROWS; r++) {
        for (int c = 0; c < FPV_OSD_COLS; c++) {
            uint8_t g = L.lastFrame.osd[r * FPV_OSD_COLS + c];
            if (g == 0 || g == ' ') continue;
            ImVec2 a(p0.x + c * cw, p0.y + r * chh), b(a.x + cw, a.y + chh);
            ImVec2 uv0(((g % 16) * OsdRenderer::kGlyphW + fontOff) / aw, (g / 16) * OsdRenderer::kGlyphH / ah);
            ImVec2 uv1(uv0.x + OsdRenderer::kGlyphW / aw, uv0.y + OsdRenderer::kGlyphH / ah);
            dl->AddImage(ImTextureRef((ImTextureID)FPV_TEXTURE_OSD_FONT), a, b, uv0, uv1);
        }
    }
    ImGui::InvisibleButton("osdgrid", size);
    if (ImGui::IsItemClicked()) {
        ImVec2 m = ImGui::GetIO().MousePos;
        int c = std::clamp((int)((m.x - p0.x) / cw), 0, FPV_OSD_COLS - 1);
        int r = std::clamp((int)((m.y - p0.y) / chh), 0, FPV_OSD_ROWS - 1);
        int pos = c | (r << 5) | 0x800;
        m_bf.set(elems[m_osdSelected].name, pos, true);
    }
    int pos = m_bf.num(elems[m_osdSelected].name, 0);
    ImGui::Text("%s: column %d, row %d%s", elems[m_osdSelected].label, pos & 31, (pos >> 5) & 31,
                (pos & 0x800) ? "" : " (hidden)");
    ImGui::EndGroup();
}

void Overlay::tabImport() {
    ImGui::TextWrapped(
        "Paste the output of `diff all` from your real quad's Betaflight CLI. Hardware lines (resources, serial, "
        "motor protocol, VTX...) are skipped, ARM moves to AUX1 and other switches to AUX2-4. The rest of your "
        "tune is applied to the current drone.");
    static char buf[64 * 1024];
    if (m_importText.size() < sizeof(buf)) std::memcpy(buf, m_importText.c_str(), m_importText.size() + 1);
    if (ImGui::InputTextMultiline("##diff", buf, sizeof buf, ImVec2(-1, ImGui::GetFontSize() * 10)))
        m_importText = buf;
    if (ImGui::Button("Import")) {
        char *report = nullptr;
        fpv_result r = fpv_import_betaflight_diff(m_importText.c_str(), &report);
        m_importReport = r == FPV_OK ? (report ? report : "") : std::string("Import failed: ") + fpv_last_error();
        fpv_free(report);
        m_bf.invalidate();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) m_importText.clear();
    if (!m_importReport.empty()) {
        ImGui::BeginChild("report", ImVec2(0, ImGui::GetFontSize() * 10), ImGuiChildFlags_Borders);
        ImGui::TextUnformatted(m_importReport.c_str());
        ImGui::EndChild();
    }
    sectionTitle("Betaflight CLI");
    static char line[256];
    bool enter = ImGui::InputText("##cli", line, sizeof line, ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    if (ImGui::Button("Run") || enter) {
        char *out = nullptr;
        fpv_cli_exec(line, &out);
        m_cliOutput = out ? out : fpv_last_error();
        fpv_free(out);
        line[0] = 0;
        m_bf.invalidate();
    }
    if (!m_cliOutput.empty()) {
        ImGui::BeginChild("cliout", ImVec2(0, 0), ImGuiChildFlags_Borders);
        ImGui::TextUnformatted(m_cliOutput.c_str());
        ImGui::EndChild();
    }
}

void Overlay::tabCamera() {
    Lib &L = lib();
    const DronePreset &p = currentPreset();
    float tilt = L.settings.cameraUptilt >= 0 ? L.settings.cameraUptilt : p.camera.uptiltDeg;
    float fov = L.settings.cameraFov > 0 ? L.settings.cameraFov : p.camera.fovYDeg;
    bool changed = ImGui::SliderFloat("Camera uptilt (deg)", &tilt, 0, 70, "%.0f");
    bool done = ImGui::IsItemDeactivatedAfterEdit();
    changed |= ImGui::SliderFloat("Field of view (vertical, deg)", &fov, 60, 150, "%.0f");
    done |= ImGui::IsItemDeactivatedAfterEdit();
    if (changed) {
        L.settings.cameraUptilt = tilt;
        L.settings.cameraFov = fov;
    }
    if (L.config.flags & FPV_INIT_NO_LENS_DISTORTION) {
        ImGui::TextDisabled("Fisheye lens: not drawn by this game");
    } else {
        ImGui::SliderFloat("Fisheye lens", &L.settings.cameraFisheye, 0, 1, L.settings.cameraFisheye > 0 ? "%.2f" : "off");
        done |= ImGui::IsItemDeactivatedAfterEdit();
    }
    if (done) saveSettings();
    if (ImGui::Button("Use the drone's defaults")) {
        L.settings.cameraUptilt = -1;
        L.settings.cameraFov = -1;
        L.settings.cameraFisheye = Settings{}.cameraFisheye;
        saveSettings();
    }
    sectionTitle("Sound");
    if (ImGui::SliderFloat("Motor sound volume", &L.settings.audioVolume, 0, 1.5f)) fpv_audio_set_volume(L.settings.audioVolume);
    sectionTitle("Video link");
    const fpv_frame &f = L.lastFrame;
    ImGui::Text("%s link, signal %.0f%%, noise %.0f%%", f.video.is_digital ? "Digital" : "Analog",
                f.video.signal_strength * 100, f.video.noise * 100);
    ImGui::TextDisabled("Noise grows with distance from the pilot and behind obstacles.");
}

void Overlay::tabControls() {
    Lib &L = lib();
    ImGui::Text("Open/close this overlay: %s", keyName(L.settings.overlayToggleKey));
    ImGui::SameLine();
    if (m_rebindingToggle) {
        ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "press a key (Esc cancels)");
    } else if (ImGui::Button("Change")) {
        m_rebindingToggle = true;
    }
    sectionTitle("Keyboard (degraded fallback)");
    ImGui::BulletText("W / S: throttle up / down");
    ImGui::BulletText("A / D: yaw");
    ImGui::BulletText("Arrow keys: pitch and roll (self-level mode)");
    ImGui::BulletText("Enter: arm / disarm, B: beeper, T: turtle mode on / off");
    sectionTitle("Gamepad (degraded fallback)");
    ImGui::BulletText("Left stick up: throttle (centre = idle), left stick sideways: yaw");
    ImGui::BulletText("Right stick: pitch and roll (self-level mode)");
    ImGui::BulletText("A / Cross: arm / disarm, B / Circle: beeper, Y / Triangle: turtle mode on / off");
    ImGui::TextWrapped("For real FPV flying use a radio in USB joystick mode: see the Radio tab.");
}

void Overlay::tabConfigurator() {
    fpv_configurator_info info{};
    info.struct_size = sizeof(info);
    fpv_configurator_get_info(&info);
    if (!info.enabled) {
        ImGui::TextWrapped("The Configurator connection is disabled by the game.");
        return;
    }
    ImGui::Text("Address: %s", info.url);
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy")) ImGui::SetClipboardText(info.url);
    ImGui::Text("Connected: %s", info.client_count > 0 ? "yes" : "no");
    sectionTitle("Connect Betaflight Configurator");
    ImGui::TextWrapped("1. Open Betaflight Configurator (app.betaflight.com or the desktop app).");
    ImGui::TextWrapped("2. In Options, enable \"manual connection mode\".");
    ImGui::TextWrapped("3. Choose \"Manual Selection\", enter %s as the port and press Connect.", info.url);
    ImGui::TextWrapped("Everything you change there is the same configuration as this overlay. Keep the motor "
                       "protocol on PWM and ARM on AUX1.");
    if (ImGui::Button("Reboot Betaflight")) {
        lib().runner.call([] { bfb_reboot(); });
    }
}

// --- frame -------------------------------------------------------------------------

void Overlay::buildUi() {
    Lib &L = lib();
    ImGuiIO &io = ImGui::GetIO();
    ImVec2 size(std::min(m_w - 20, 820 * m_scale), std::min(m_h - 20, 620 * m_scale));
    ImGui::SetNextWindowPos(ImVec2(m_w * 0.5f, m_h * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(size, ImGuiCond_Appearing);
    bool open = true;
    if (ImGui::Begin("libfpv - FPV drone", &open, ImGuiWindowFlags_NoCollapse)) {
        if (ImGui::BeginTabBar("tabs")) {
            static const char *names[] = {"Fly", "Radio", "Drone", "Rates", "Tuning", "OSD", "Import & CLI",
                                          "Camera & sound", "Controls", "Configurator"};
            void (Overlay::*fns[])() = {&Overlay::tabFly,    &Overlay::tabRadio,  &Overlay::tabDrone,
                                        &Overlay::tabRates,  &Overlay::tabTuning, &Overlay::tabOsd,
                                        &Overlay::tabImport, &Overlay::tabCamera, &Overlay::tabControls,
                                        &Overlay::tabConfigurator};
            for (int i = 0; i < 10; i++) {
                ImGuiTabItemFlags fl = (m_requestTab == i) ? ImGuiTabItemFlags_SetSelected : 0;
                if (ImGui::BeginTabItem(names[i], nullptr, fl)) {
                    (this->*fns[i])();
                    ImGui::EndTabItem();
                }
            }
            m_requestTab = -1;
            ImGui::EndTabBar();
        }
        if (m_bf.busy())
            ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1),
                               "The Configurator is in CLI mode: values here are paused until you leave its CLI tab.");
    }
    ImGui::End();
    (void)io;
    if (!open) setOpen(false);
    (void)L;
}

const fpv_draw_data *Overlay::convert() {
    m_draw.clear();
    ImDrawData *dd = ImGui::GetDrawData();
    if (!dd) return m_draw.finish(m_w, m_h);
    for (int n = 0; n < dd->CmdLists.Size; n++) {
        const ImDrawList *cl = dd->CmdLists[n];
        DrawListStorage &s = m_draw.newList();
        s.vertices.resize(cl->VtxBuffer.Size);
        static_assert(sizeof(ImDrawVert) == sizeof(fpv_draw_vertex), "vertex layout");
        std::memcpy(s.vertices.data(), cl->VtxBuffer.Data, sizeof(ImDrawVert) * cl->VtxBuffer.Size);
        s.indices.resize(cl->IdxBuffer.Size);
        for (int i = 0; i < cl->IdxBuffer.Size; i++) s.indices[i] = cl->IdxBuffer.Data[i];
        for (int c = 0; c < cl->CmdBuffer.Size; c++) {
            const ImDrawCmd &pc = cl->CmdBuffer[c];
            if (pc.UserCallback || pc.ElemCount == 0) continue;
            fpv_draw_cmd o{};
            o.clip_rect[0] = pc.ClipRect.x - dd->DisplayPos.x;
            o.clip_rect[1] = pc.ClipRect.y - dd->DisplayPos.y;
            o.clip_rect[2] = pc.ClipRect.z - dd->DisplayPos.x;
            o.clip_rect[3] = pc.ClipRect.w - dd->DisplayPos.y;
            o.texture_id = (uint64_t)pc.GetTexID();
            o.index_offset = pc.IdxOffset;
            o.index_count = pc.ElemCount;
            o.vertex_offset = pc.VtxOffset;
            s.commands.push_back(o);
        }
    }
    return m_draw.finish(m_w, m_h);
}

const fpv_draw_data *Overlay::render(float dt) {
    Lib &L = lib();
    // a newly connected, unmapped radio opens the setup wizard once
    InputStatus st = L.input.status();
    if (st.needsSetup) {
        for (auto &d : L.input.devicesSnapshot()) {
            DeviceMapping m;
            if (d.looksLikeRadio && !L.input.getMapping(d.guid, m)) notifyUnmappedRadio(d.guid);
        }
    }
    if (!m_open) return nullptr;
    WITH_CTX;
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(m_w, m_h);
    io.DeltaTime = (dt > 0 && std::isfinite(dt)) ? std::min(dt, 0.25f) : 1.0f / 60.0f;
    ImGui::GetStyle().FontScaleMain = m_scale;
    m_time += io.DeltaTime;
    ImGui::NewFrame();
    buildUi();
    ImGui::Render();
    const fpv_draw_data *out = convert();
    END_CTX;
    return out;
}

}  // namespace fpv
