# Pull request notes

Each branch below starts from upstream `main` (0318810), except for three
branches that build on another branch. The base of each branch is in its
section. Merge a stacked branch after its base. Every branch builds with
MinGW-w64 GCC (Release). Both `fpv_tests` and `fpv_wizard_tests` pass on
every branch.

| Branch | Base |
|---|---|
| `pr/osd-style` | `main` |
| `pr/raycast-ground` | `main` |
| `pr/rssi-fov` | `main` |
| `pr/replace-props` | `main` |
| `pr/overlay-embedded` | `main` |
| `pr/chase-sliders` | `main` |
| `pr/respawn-switch` | `main` |
| `pr/osd-element-list` | `main` |
| `pr/hd-osd` | `pr/osd-style` |
| `pr/goggle-fonts` | `pr/hd-osd` |
| `pr/logo-layouts` | `pr/goggle-fonts` |

---

## pr/osd-style

Title: Add fpv_osd_get_style and fpv_osd_set_style

The overlay's OSD tab saves the player's choice between the analog look
and the HD look in `settings.json`. Before this change, a host had no way
to read that choice. It had to keep its own setting for the style that
it passes to `fpv_osd_draw_data`.

Every game that draws the OSD can now use the player's choice directly.
A game can also change the style from its own hotkey or menu. The overlay
shows the change, and libfpv saves it.

API added:

- `fpv_osd_style fpv_osd_get_style(void)`
- `void fpv_osd_set_style(fpv_osd_style style)`

Test: `osd_style` sets each style and reads it back.

---

## pr/raycast-ground

Title: Cast the raycast ground under a new drone before its first tick

libfpv refreshed the raycast collision planes only in `fpv_drone_tick`.
But the simulation steps as soon as `fpv_drone_create` returns. If the
first tick from the host came late, the drone fell with no ground under
it. For example, a host can tick late while it sets up its camera. If the
game hitches, the first tick also comes late. When the drone was below the surface, the downward probes
started inside the ground and missed, so the drone fell forever.

Now `fpv_drone_create`, `fpv_drone_reset` and
`fpv_world_set_raycast_callback` cast the planes at the new position at
once. This helps every game that uses the raycast callback, because games
often tick late while they load.

API added: none. The behavior of three existing calls changes as
described above.

Test: `raycast_ground_at_spawn` steps the simulation for 0.5 s before the
first tick, with the callback set before and after `fpv_drone_create`.
The drone must rest on the raycast ground. Without the fix, the drone
ends 11 m below it.

---

## pr/rssi-fov

Title: RSSI from the link model, and a camera field of view API

Betaflight's RSSI showed 0 because nothing set it. The link model
(the distance to the pilot and the obstructions) already controls the
video noise. Each step now also sets Betaflight's RSSI and RSSI dBm,
from -40 dBm up close to -115 dBm. The OSD RSSI elements, the RSSI
warnings and the logs now show real values. If the model uses an ADC or
channel RSSI source, this change does nothing.

Two new calls give access to the field of view setting of the overlay.
A game can show this setting in its own menu.

API added:

- `float fpv_camera_get_fov(void)`
- `fpv_result fpv_camera_set_fov(float degrees)`: 60 to 150, or 0 for
  the value of the preset. libfpv saves the value in `settings.json`.

Tests: `rssi` (99 next to the pilot, 0 at 3 km) and `camera_fov`.

---

## pr/replace-props

Title: Add fpv_drone_replace_props: new propellers, same battery

A crash bends the props, and turtle mode damages them more. The drone
then flew badly until a full repair, which also charges the battery
again. The new call resets only the prop damage, like a prop swap in the
field. A game can use it for a "replace props" key or menu item that
does not reset the flight.

API added: `fpv_result fpv_drone_replace_props(fpv_drone *drone)`

Test: `replace_props`. A wall crash bends a prop. After the call, the
damage is 0, and the used mAh value does not change.

---

## pr/overlay-embedded

Title: Overlay embedded mode (fpv_overlay_set_embedded)

Some hosts show the overlay in a panel of their own, for example the
add-on settings of ReShade. In such a panel, the overlay had its own
movable window with a title bar and a close button. In embedded mode,
the overlay fills the display area without a window frame. The mode is
off by default.

API added: `void fpv_overlay_set_embedded(int embedded)`

Tests: no new test. The existing overlay tests pass.

---

## pr/chase-sliders

Title: Chase camera: distance, height and follow sliders

The chase camera stayed 1.3 m behind and 0.45 m above the drone. It moved
toward that point at a rate of 6/s, so it swung wide in turns. Three new
settings are on the Camera page of the overlay: `chaseDistance`,
`chaseHeight` and `chaseFollow`. The default follow rate is now 12/s,
which keeps the camera closer behind the heading. A button sets the
default values again. libfpv saves the settings in `settings.json`.

Every game that uses the chase camera gets these settings. The game does
not need any change.

API added: none. The new values are in `settings.json`.

Tests: no new test. The existing tests pass.

---

## pr/respawn-switch

Title: Radio respawn switch, and a fix for the radio setup

The Radio tab can set up one more switch on each radio: a respawn switch.
It has its own short setup wizard, like the kamikaze bomb switches.
libfpv saves the switch with the radio mapping, after the Betaflight
channels. Betaflight never sees it. Each time the switch goes on, the
next `fpv_frame` has `FPV_DRONE_RESPAWN_REQUEST`, and the game runs its
own reset. If the switch is already on when the radio appears or during
a setup, it must go off first. The tab shows the live state of the
switch. If the radio has no free axis or button, the tab tells the
pilot.

A pilot can then respawn without taking the hands off the sticks. Every
game gets this with one check of a frame flag.

The second commit fixes the setup wizard. Before this change, a close of
the overlay stopped the setup. Some hosts close and open the overlay for
a moment, for example if their panel misses a frame. Now a close pauses the
setup and frees the flight input, and an open continues the setup.

API added: `FPV_DRONE_RESPAWN_REQUEST` (bit 11 of `fpv_frame.flags`).

Tests: new cases in `fpv_wizard_tests` (52 checks before, 62 after). The
kamikaze build compiles, and its tests pass.

---

## pr/osd-element-list

Title: Overlay: list every OSD element that the simulator build has

The OSD editor in the overlay listed only 20 elements. A pilot had no
way to place the home arrow, the home distance, RSSI and most other elements.
The list now has every `osd_*_pos` setting that the simulator build of
Betaflight has. The stick overlay entries are removed, because the
simulator build does not compile them.

API added: none.

Tests: no new test. The existing tests pass.

---

## pr/hd-osd (base: pr/osd-style)

Title: HD OSD: Betaflight's 53 x 20 canvas (vcd_video_system = HD)

The OSD can now use the HD canvas of DJI O3 and O4 goggles. As on a real
quad, Betaflight's `vcd_video_system` setting selects the canvas. HD
gives 53 x 20. Any other value gives the analog 30 x 16.

- SimITL patch 0018: the fake display port holds a 53 x 20 buffer. It
  gets its size from `vcd_video_system` at init. The OSD state carries
  its size. `USE_OSD_HD` is defined for the simITL target.
- Betaflight patch 0011: on `SIMULATOR_BUILD`, Betaflight keeps the fake
  display instead of the MSP display port for HD, and keeps the analog
  defaults.
- `fpv_frame` gets `osd_rows`, `osd_cols` and `osd_hd` (53 x 20) at its
  end. The `osd` field keeps the top-left 30 x 16 for older hosts. The
  renderer, `fpv_osd_draw_data` and `fpv_osd_render_rgba` draw a grid of
  any size.
- `fpv_osd_get_style` now reads Betaflight's video system.
  `fpv_osd_set_style` sets it, saves, and reboots Betaflight in the
  process, because Betaflight sets up the canvas at boot. The call does
  nothing while the drone is armed.
- The OSD tab of the overlay uses the same API. It shows the real canvas
  and can place elements past column 31.

Both patches apply to the Betaflight and SimITL commits that `main` uses.

API added: `FPV_OSD_MAX_ROWS`, `FPV_OSD_MAX_COLS`, and the
`fpv_frame` fields `osd_rows`, `osd_cols` and `osd_hd`.

Test: `osd_style` switches to HD and puts an element at column 45. It
then finds the element in `osd_hd`, and switches back to 30 x 16.

---

## pr/goggle-fonts (base: pr/hd-osd)

Title: Load goggle OSD font packages (DJI O3 PNG, WTFOS .bin)

A font package is a folder in `<data_dir>/fonts`. It holds the same files
that go on the SD card of the goggles. The analog canvas uses the SD sheet of the package. The HD
canvas uses its HD sheet. This applies to the OSD texture and to the
overlay preview. A missing sheet falls back to the built-in
font. libfpv saves the selection. PNG sheets load with stb_image (v2.30,
in `third_party/stb`).

A pilot can use the same font in every libfpv game as on the goggles.

API added:

- `fpv_result fpv_osd_set_font(const char *name)`
- `const char *fpv_osd_get_font(void)`

Test: `osd_font` loads a WTFOS package and checks the size of the SD
sheet. It also checks that a missing folder and a path fail, and that the
built-in font comes back.

---

## pr/logo-layouts (base: pr/goggle-fonts)

Title: Custom OSD boot logo, and one OSD layout for each canvas

Boot logo: the host can set a PNG file. It replaces Betaflight's 24 x 4
glyph logo (glyphs 160 to 255) wherever that block shows: at boot, and
on arming with `osd_logo_on_arming`. The image keeps its own aspect
ratio. The draw data gets a second command with `FPV_TEXTURE_OSD_LOGO`,
and the CPU rasterizer draws both textures. Betaflight does not change:
libfpv finds the block by its glyph pattern. A game can show its own
logo when the drone powers up.

Layouts: Betaflight keeps one set of element positions. Before this
change, a layout for the HD canvas fell off the analog canvas. Now a
style change saves the current positions in `settings.json` (`osdLayoutHd`
and `osdLayoutSd`) and restores the layout of the other canvas. The first
time a pilot uses a canvas, libfpv scales the current positions to it.

API added: `fpv_result fpv_osd_set_logo(const char *png_path_utf8)` and
`FPV_TEXTURE_OSD_LOGO`.

Tests: `osd_font` checks the logo quad and its texture.
`osd_layout_per_style` checks that each canvas keeps its own layout.

---

## Left out

None of the 11 features was left out. These items are not on any branch:

- The LED strip colour changes in commit 1d3fa23 need the LED strip page,
  which `main` does not have. `pr/respawn-switch` does not contain them.
- The port to Betaflight 2026.6 and the work that needs it are not on any
  branch. That work includes Betaflight's USER1 respawn mode, the blackbox
  log files and the LED strip.
