# libfpv design decisions

This is the log of decisions not fixed by the original brief. Each entry
gives the decision, why it was made and what it would take to reverse it.
Newest entries go at the bottom of their section.

## Dependencies and repository

### D1. Flattened HTTPS submodules, SimITL pinned, patches as a series

SimITL's own submodules use `git@github.com:` URLs and nest Betaflight,
libwebsockets and fmt inside SimITL. libfpv adds each of them as a top-level
submodule under `third_party/` with an HTTPS URL, pinned to exactly the commit
SimITL `c881d55` (2026-09-07) references:

| Submodule | Commit / tag |
|---|---|
| `third_party/SimITL` | `c881d55` |
| `third_party/betaflight` (AJ92 fork, 2025.12.0 pre-release) | `7bd98d8` |
| `third_party/libwebsockets` | `f69835e` |
| `third_party/fmt` | `9869971` |
| `third_party/SDL` | `release-3.4.18` |
| `third_party/imgui` | `v1.92.9` |
| `third_party/nlohmann/json.hpp` (vendored single header) | `v3.12.0` |

libfpv does not use SimITL's CMake files; `cmake/SimITLCore.cmake`
reproduces SimITL's Betaflight source list against the flattened paths. It
builds only a static, position-independent `simitl_core` library with hidden
symbols.

Changes to SimITL and Betaflight are kept as a patch series in
`patches/simitl/` and `patches/betaflight/`. CMake applies them at configure
time, and a patch that already reverse-applies is skipped. Patched
submodules are marked `ignore = dirty`. We use a patch series rather than a
GitHub fork so the changes stay reviewable in this repository and can be
offered upstream one at a time.

Betaflight upstream is at 2026.6.2 while SimITL's fork tracks 2025.12.0. The
brief fixes SimITL as the flight core, so libfpv stays on SimITL's fork.
Moving to a newer Betaflight means rebasing that fork, which is out of scope
for v1.

### D2. SimITL patches

* `0001-lifecycle-logging-failsafe.patch`
  * Root cause of "stopping corrupts memory": `wsStop()` destroyed the
    libwebsockets contexts while the WebSocket thread was still inside
    `lws_service()`. The new order is: signal, `lws_cancel_service()`,
    join, then destroy. Contexts can be re-created (`wsRestart`), so the
    Configurator service can be stopped and restarted without
    re-initialising Betaflight.
  * Root cause of "game exit crashes": the `Sim` singleton owns a joinable
    `std::thread` and its destructor neither joined nor detached it, so
    process exit ran `std::terminate` (reproduced: "terminate called without
    an active exception"). The destructor now detaches. It never joins,
    because on Windows the thread is already killed by `ExitProcess` and
    joining under the loader lock can deadlock.
  * The WebSocket base port is now configurable (`wsSetBasePort`; UART1 is
    base + 1, so the default stays at 5761). `lws_service` timeout was
    lowered from 1000 ms to 50 ms so shutdown is prompt.
  * Failsafe: `rxRcFrameStatus` can report `RX_FRAME_PENDING`, which makes
    Betaflight's own RX-loss and failsafe logic run when the radio
    disappears.
  * SimITL's `fmt::print` and Betaflight's `printf`/`fprintf` are routed to
    libfpv's log callback. Betaflight C files are compiled with a
    force-included header, `src/core/bf_log_redirect.h`. A game's stdout is
    never written to.
* `0002-exact-contact-velocity.patch`: `StateInput::contact == 2` makes
  SimITL take the host's linear and angular velocity exactly. The existing
  value 1 blends 75 %/50 %. libfpv resolves collisions with impulses and
  needs the result applied exactly, otherwise the drone sinks into the
  ground or tunnels.
* `0004-reversed-motors.patch`: `StateInput::motorsReversed` spins the
  props backwards for turtle mode (D21): thrust is negated and scaled by
  0.5 (an inverted airfoil), and the reaction torque flips sign.
* `0007-gps-from-host.patch`: SimITL's `updateGps` wrote `gpsSol`
  directly every step, which Betaflight's virtual GPS provider overwrote
  with its own (empty) solution at 10 Hz. It is no longer called; libfpv
  feeds the virtual GPS instead (D22).
* `0008-monotonic-clock-across-reboot.patch`: `systemInit` reset the clock
  to 0 on every in-process reboot (`systemReset`), while Betaflight's
  statics survive it. Timestamps they hold then lay in the future: the
  virtual GPS stayed silent for as long as the uptime before the reboot.

## Simulation

### D3. SimITL coordinate frame

Verified from `physics.cpp`, `bf.cpp` and the presets: X right, Y up,
Z forward, left-handed (Unity's convention), SI units, gravity −Y. Motor 1
(rear right in Betaflight's Quad X) is at (+x, −z), and motor 2 (front
right) is at (+x, +z). SimITL integrates velocities and orientation, but
**not position**. The host must feed the rotation back each update, because
`StateInput` overwrites it.

### D4. Betaflight configuration goes through Betaflight's own CLI, in-process

`src/core/bf_bridge.c` implements an in-memory `serialPort_t` and drives
`cliEnter`/`cliProcess` on the simulation thread. Preset defaults, overlay
edits and `diff all` imports are therefore just CLI commands. They go
through the same parameter-group code as the Configurator (which uses MSP
over the WebSocket), so there is one source of truth and no copy of any
Betaflight setting inside libfpv. `save` is always issued as
`save noreboot`. When the Configurator owns the CLI (`cliMode`), libfpv
reports "busy" instead of interleaving.

### D5. Default EEPROM settings

Measured: a fresh EEPROM does not arm. There is no ARM mode, there is a 5 s
power-on grace (`BOOTGRACE`), and gyro calibration (`CALIB`) takes about
4 s of sim time. libfpv's defaults set `pwr_on_arm_grace = 0`, PWM, and
`aux 0 0 0 1700 2100 0 0` (ARM on AUX1).

### D6. Betaflight patches

* `0001-cli-array-set-null-deref.patch`: `cliSet` parsed array settings
  with `valPtr = strchr(valPtr, ',') + 1`. When a diff gives fewer values
  than the array length (common in diffs from older firmware), this turns
  NULL into `(char *)1` and the next iteration reads address 1.
  Found by importing the 4.0.6 corpus diff under AddressSanitizer.
* `0002-sitl-crash-flip.patch`: crash flip (turtle) mode is compiled only
  with `USE_DSHOT`, which SITL builds leave out. The patch enables the
  mode box, arming in crash flip and its beeper/disarm handling for SITL.
  The motor reversal a DShot command would do is applied by the simulator
  (D21).
* `0004-virtual-gps-keeps-feature.patch`: config validation turned
  `FEATURE_GPS` off on every save and boot when no serial port has the GPS
  function, exempting only the MSP provider. The virtual provider needs no
  port either.
* `0005-altitude-elapsed-time.patch`: the altitude estimate's filters and
  the vario's derivative assumed the ALTITUDE task runs at its nominal
  100 Hz. In the simulator it runs at about 33 Hz (D22), so altitude lagged
  and the vario read 3× too high. Both now use the measured interval.

### D7. One EEPROM per quad preset

Each preset has its own Betaflight EEPROM at
`<data_dir>/eeprom/<preset>.eeprom`, like one flight controller per real
quad. Switching presets saves pending changes and then re-reads the other
file (`initEEPROM` → `ensureEEPROMStructureIsValid` → `readEEPROM`). A
missing file is first reset to Betaflight defaults, so it never inherits
the previous quad's memory image. Then libfpv's first-run script and the
preset's own tune (`<preset>.diff.txt` next to the JSON, if present) are
applied. `diff all` imports go to the active preset's EEPROM.

### D8. Props follow `yaw_motors_reversed`

Measured: with SimITL's preset motor directions (−1, 1, 1, −1) and
Betaflight's default "props in" mixer, the yaw loop has positive feedback.
The drone spins up to 18 rad/s and runaway-takeoff prevention disarms it.
SimITL's own tools work because pr0p's EEPROMs set
`yaw_motors_reversed = ON`. libfpv does not force that setting, because an
imported real "props in" diff would then spin the wrong way. Instead the
simulated prop directions follow Betaflight's `yaw_motors_reversed`, read
every 64 steps so Configurator edits apply live. It flies stably both ways
(tested).

### D9. Collision shape

The collision shape is a sphere at the centre of mass plus one under each
motor bell, with level bottoms. With only a body sphere the drone rested
on a single point and ground friction could not stop yaw spin (measured:
12.8 rad/s forever). Propeller discs are not collided with; they are tested
separately for prop strikes, which damage the props (`durability` from the
prop JSON). `FPV_MATERIAL_KILLS_PROPS` destroys them on contact.

A second sphere sits on top of each motor (bell and prop nut, 6 mm above
the prop disc). Without them an upside-down drone balanced on the body
sphere: it rocked until a prop tip touched the ground, so props spun up in
turtle mode were destroyed by strikes, and a turtle flip pivoted on the
centre and spun far too easily. Now it rests on the motor tops with props
clear and pivots on an arm, like a real quad. Contacts on motor spheres
never bounce (restitution 0): arms and props soak up the hit. With the
material's restitution a quad landing on an arm after a flip cartwheeled
on over it.

### D10. Warm-up, so a fresh drone can arm immediately

Betaflight needs about 3.7 s of simulated time after boot for gyro, acc
and baro calibration, and RX is suspended briefly whenever an EEPROM is
read. `fpv_global_init`, EEPROM switches and the reboot after an import
step the parked simulation until `CALIBRATING`, `RXLOSS`, `FAILSAFE` and
`BOOTGRACE` clear. This costs about 40 ms of wall time.

### D11. Timing model: dedicated simulation thread at 1 kHz

Measured: one 1 ms SimITL step (8 Betaflight PID loops) costs about 10 µs
on a desktop CPU, so a 1 kHz thread uses about 1 % of a core. Collision
substeps keep every sphere moving less than half its radius, so a 30 m/s
drone takes about 4 substeps per step.

* `FPV_TIMING_THREAD` (default): a libfpv thread steps the simulation at
  `sim_rate_hz` and polls the radio at the same rate. Pacing uses a
  high-resolution waitable timer on Windows and `sleep_until` elsewhere. A
  fixed-step accumulator gives frame-rate independence and is capped at
  50 ms, so a host hitch cannot cause a spiral. Each step carries the wall
  time it represents. `fpv_drone_tick` renders one step (1 ms) in the past
  and interpolates (pos lerp, rot slerp), so there is no visible jitter at
  any host frame rate and no extrapolation error.
* `FPV_TIMING_MANUAL`: the host calls `fpv_step` itself. This is
  deterministic and used by the tests and lockstep engines.
* Auto-pause: when the host stops calling `fpv_drone_tick` for 0.5 s (a
  pause menu, loading screen or debugger), the simulation stops, unless the
  drone is disarmed on the ground. In that case Betaflight keeps running so
  the Configurator stays responsive.
* All Betaflight access (CLI, EEPROM, presets) is queued to the simulation
  thread. The public API is serialised by one recursive mutex.

The alternative, stepping inside the host's tick, would tie radio sampling
and PID timing to the game's frame rate (16 ms at 60 FPS) and break the
"radio polled at sim rate" requirement.

### D12. libfpv does not retune SimITL's aerodynamics

Measured with PropEater 5" (6S, 1900 kV): hover at 14 % throttle, and a
vertical punch-out reaches about 275 km/h after 2 s (thrust-to-weight
about 15:1 from the prop JSON). That is high but within what real 6S
racing quads do, and the model is SimITL's by design. Tests check
plausibility ranges, not exact values.

### D13. `diff all` import rules

* The import starts from `defaults nosave`, then applies the diff, clears
  all 20 mode slots, applies libfpv's base settings, re-adds the modes and
  saves without reboot. If Betaflight then reports
  `REBOOT_REQUIRED`, libfpv reboots it in-process (Betaflight's own
  `systemReset` → `init()`, the same path as the Configurator's
  "Save and Reboot") and warms it up again.
* Dropped as hardware-specific, each with its reason in the report:
  resources, timers, DMA, serial ports, board identity, LED strip,
  VTX/vtxtable, beacon, receiver settings, sensor hardware and
  calibration, OSD chip, battery/current sensor scaling, and `map` and
  `rxrange` (libfpv's radio wizard owns channel order and endpoints).
* `profile`/`rateprofile` indices this Betaflight does not have (newer
  firmware has 6 rate profiles, the embedded one has 4) are skipped
  together with every `set` that follows them, so those settings cannot
  land in the wrong profile.
* ARM is moved to AUX1 with range 1700–2100, the wizard's "switch on". The
  other mode channels are packed into AUX2–AUX4 in order of appearance. A
  fifth distinct switch channel is dropped and reported.
* Settings Betaflight rejects (renamed since older firmware) are listed
  under "Rejected by Betaflight". They are not an error.
* Corpus: six real diffs from Betaflight 4.0.6 to 4.5.1
  (`tests/diffs/README.md`). Every one imports, arms and flies in the test
  suite.

## Presentation

### D14. Overlay: libfpv is a "legacy" ImGui backend

Dear ImGui 1.92 supports dynamic font atlases (`RendererHasTextures`), but
that would push texture create/update/destroy requests onto every host
renderer. libfpv does not set the flag. It bakes ImGui's scalable default
font once at 32 px into one RGBA atlas (`FPV_TEXTURE_OVERLAY_FONT`) and
draws it at `16 px × ui_scale`. The host uploads at most two static
textures (overlay font, OSD font) and draws plain indexed triangles, so its
renderer snippet stays about 60 lines. The overlay draws its own mouse
cursor (`io.MouseDrawCursor`), because games usually hide the OS cursor.
No `imgui.ini` is written. ImGui is compiled inside libfpv with hidden
symbols, so it cannot clash with a game's own ImGui.

Overlay values for Betaflight settings are read and written with CLI
`get`/`set` on the simulation thread (`BfSettingsCache`) and refreshed
every second while the overlay is open. Configurator edits therefore show
up, and overlay edits show up in the Configurator after its next read.
Sliders write live and `save` on release.

### D15. OSD

Betaflight Configurator's MAX7456 fonts (`default.mcm` for the analog look,
`bold.mcm` for the HD look) are embedded and parsed at runtime into one
384×288 atlas. `fpv_osd_draw_data` returns quads in the same draw-data
format as the overlay, so a host needs only one renderer.
`fpv_osd_render_rgba` rasterises on the CPU for hosts that prefer a
texture. SimITL's OSD canvas is the 30×16 analog grid. "HD look" uses the
bold font in a 16:9 viewport; it does not add an HD (53×20) canvas.

### D16. Audio

`fpv_audio_read` is called from the host's audio thread. It never takes
the API lock: parameters (RPM, load from current, prop damage, pan from
motor position) are copied under a small mutex once per host frame and
glided over 15 ms. Each motor mixes a band-limited blade-pass saw
(harmonics above Nyquist dropped), the motor's electrical whine (7 pole
pairs), RPM-tracking low-passed noise scaled by load, and a
once-per-revolution wobble from prop damage. Betaflight's beeper is a
2.7 kHz tone. The output is soft-clipped with `tanh`.

### D17. Motor state is cleared on spawn and despawn

SimITL's `Reset`/`Repair` commands keep the motor model (RPM, ESC filters,
2 ms delay line). A drone deleted at hover throttle came back with
spinning props, popped up 5 cm and tripped ANGLE (found by test ordering).
`Flight::stopMotors` clears that state.

### D18. Shipped presets use Betaflight's stock tune

SimITL's per-quad `.bin` EEPROMs load fine (version 178), but differ from
defaults only in `feature -GPS -TELEMETRY`: there are no modes and no
tune. So libfpv ships no per-preset tune. The `<preset>.diff.txt`
mechanism (D7) is there for preset authors.

## Platforms

### D19. Windows: MinGW-w64 cross build, fully static runtimes

`cmake/toolchains/mingw-w64-x86_64.cmake` uses the `-posix` GCC
(winpthreads, needed for `std::thread`) and links libgcc, libstdc++ and
winpthreads statically into every binary. Without that, executables need
`libgcc_s_seh-1.dll`, `libstdc++-6.dll` and `libwinpthread-1.dll` and fail
to start silently: found when `fpv_tests.exe` exited with code 53. Now
`libfpv.dll` imports only Windows system DLLs. `libfpv.def` is emitted so
MSVC hosts can build an import library with `lib /def:libfpv.def
/machine:x64`. They can also `LoadLibrary` it, since the API is pure C.
The full test suite passes on Windows 11 (run from WSL interop).

SimITL's `string_light.c` re-implements libc's ctype and `strcasecmp`
functions. With whole-archive linking (D20) these collide with msvcrt, so
it is excluded on every platform and the platform libc is used. MinGW's
missing `strcasestr` comes from `src/core/bf_compat.c`. Betaflight C files
are compiled with `_GNU_SOURCE`, so `strcasestr` has a real prototype
rather than an implicit `int` that would truncate the pointer. Betaflight's
structs use GCC bitfield layout (`-mno-ms-bitfields`), applied to C and
C++ alike inside the core.

### D19b. 32-bit Windows (i686) for 32-bit host games

`cmake/toolchains/mingw-w64-i686.cmake` mirrors the x86_64 one for 32-bit
hosts such as GTA IV (a PE32 executable, so it can only load a 32-bit
DLL). It adds `-msse2 -mfpmath=sse`: with GCC's default x87 maths,
`wall_crash` failed (a 63 m/s drone tunnelled through the wall). With SSE2
the full suite passes (20 tests, 321 checks) on Windows 11 through WSL
interop. MSVC hosts use `lib /def:libfpv.def /machine:x86`.

### D20. Whole-archive linking of the core

Betaflight's parameter groups register themselves in a linker section that
no code references by name. `simitl_core` is therefore linked with
`$<LINK_LIBRARY:WHOLE_ARCHIVE,...>`, so no PG can be dropped by the linker.

## Modes

### D21. Turtle mode (flip over after crash)

Betaflight decides crash flip when arming with the FLIP OVER AFTER CRASH
switch on, then reverses the motors with a DShot command. libfpv drives
PWM (D5), so instead `bfb_crash_flip_active()` (armed and
`isCrashFlipModeActive()`) feeds `StateInput::motorsReversed` every step.
Betaflight's own crash flip mixer picks the motors from the sticks.
Fresh EEPROMs get turtle on AUX4 (`aux 3 35 3 1700 2100 0 0`). The F8
radio wizard's turtle switch step binds AUX4, the keyboard fallback toggles
it with T and the gamepad fallback with Y / Triangle.
`FPV_DRONE_TURTLE` is set while armed in turtle mode.

AUX switches on an axis are positions, not ranges: a switch is on when the
axis is closer to its "on" value than half the way to any other known
position of that axis (rest, or another switch's "on"). So one 3-position
switch can carry beeper in the middle and turtle at the top, and a mode
mapped to the middle alone is off at the top. The wizard offers an axis
already used by a switch again only in a position no switch uses yet.
Saved mappings keep their format (on = `max`, or `min` when inverted).

libfpv sets `crashflip_rate = 30`, the default of Betaflight's Race Pro
builds (stock is 0). It cuts the turtle power as the quad spins faster
and once it has turned about 90°, so holding the stick works and
Betaflight can tell the flip succeeded; turning the switch off then
re-arms in normal mode at once (quick restart). Measured with stock 0:
releasing the stick at the right moment was needed, overflips were
common and props were often destroyed. With 30, every shipped preset
flips upright in about 0.5 s with 60 % stick held, with no prop damage.

EEPROMs made before this get, once, the turtle switch on AUX4 (if there is
no crash flip mode yet and AUX4 is free) and `crashflip_rate = 30` (if it
is 0). A `<eeprom>.upgrade` file records that this ran, so later edits by
the player stay.

### D23. Air mode and OSD disable switches (AUX5, AUX6)

Two more Betaflight modes that pilots keep on switches: AIR MODE (box 28)
on AUX5 and OSD DISABLE (box 19) on AUX6, both on the high position, in
mode slots 4 and 5 of fresh EEPROMs. The F8 radio wizard binds them after
turtle, both optional, with the same position logic as beeper and turtle
(D21): one 3-position switch can carry air mode in the middle and OSD
disable at the top. The keyboard and gamepad fallbacks leave both off.

libfpv now drives 10 channels (AETR + AUX1..6, `FPV_CHANNEL_COUNT`).
SimITL passed only 8 to Betaflight; `0009-sixteen-rc-channels.patch`
widens `StateInput::rcData` and `BF::setRcData` to Betaflight's
`SIMULATOR_MAX_RC_CHANNELS` (16). libfpv sends -1 on AUX7 and up.
`fpv_frame.channels` grew to 10 at the end of the struct, so hosts built
against the old header still get the first 8 (`struct_size`), and
`fpv_input_set_channels` with 8 channels leaves AUX5/AUX6 off.

Air mode: Betaflight's AIRMODE feature is on by default, and then air
mode is always on and the switch changes nothing, exactly like on a real
quad. The switch matters when the player turns the feature off
(`feature -AIRMODE`, or a `diff all` from a quad flown that way).

OSD disable: Betaflight clears the whole OSD while the mode is on
(including warnings and the post-flight statistics), so `frame.osd` is
blank. `FPV_DRONE_OSD_DISABLED` (from `IS_RC_MODE_ACTIVE(BOXOSD)`) tells
the host, which must then hide every bit of its own UI that it still
draws over the FPV view (INTEGRATION.md, "OSD disable switch"); the
player asked for a clean picture.

Older EEPROMs get both switches once, each only if that mode is not
configured yet and its channel is free, through the same `.upgrade` file.
Upgrades are now an ordered list and the file holds the last one applied
(`turtle-2`, then `air-osd-3`), so an EEPROM already upgraded for turtle
gets only the new switches, and turtle is never re-added after the player
removed it. A `diff all` import may now move modes to AUX2..AUX6.

## Sensors

### D22. GPS and barometer

Every simulated quad has a GPS and a barometer, so Betaflight's GPS and
altitude features work: speed, home arrow and distance, distance flown,
altitude, vario, efficiency, their post-flight statistics, the GPS lap
timer and GPS Rescue.

* `Flight::step` feeds Betaflight's virtual GPS provider (`setVirtualGPS`,
  the target's default) and its virtual barometer every step. Betaflight
  samples the GPS at 10 Hz like a real module. Satellites are always 12.
* The local frame is laid on a flat patch of earth at 0° N 0° E, with
  internal +Z (host forward) as north and +X (host right) as east. At the
  equator a degree of latitude and of longitude have the same length
  (`EARTH_ANGLE_TO_CM`), so Betaflight's distance and bearing maths gives
  exact metres. Latitude, longitude and plus code are therefore
  meaningless and not supported.
* Home is Betaflight's own (where the drone arms) until the host calls
  `fpv_gps_set_home`. The host's home is re-applied after every step,
  because arming moves it. When the arming point replaced it, the
  distance and direction to home are recomputed at once, so the OSD never
  shows the arming point for a GPS cycle. The value is kept behind a mutex
  rather than a simulation-thread job, so hosts can set a moving home
  every frame without waiting.
* GPS is a libfpv essential, like PWM: `ensureEssentials` turns
  `feature GPS` back on and reboots Betaflight in-process when it is not
  running (features only apply at boot). The first-run script and every
  import include `feature GPS`.
* The barometer is fed too. Without it the virtual baro reads sea level,
  and Betaflight's altitude mix, which prefers the baro when the two
  disagree by more than 1 m, kept the altitude near 0.
* `bfb_cli_run` calls `osdAnalyzeActiveElements` after each CLI session.
  A real CLI `save` reboots, which rebuilds the list of elements to draw;
  libfpv's `save noreboot` did not, so elements moved or enabled through
  `fpv_cli_exec` stayed hidden until the next reboot.

Measured limits of the simulator's scheduler: SimITL calls Betaflight's
scheduler once per 125 µs gyro step, and each call runs at most one
non-realtime task. RX takes about half of those slots, so low-priority
tasks run below their rate (ALTITUDE 33 Hz instead of 100, GPS 50 instead
of 100), and each OSD element is redrawn only about every 0.4 s. Altitude
on the OSD trails the drone by up to about 0.9 s. Calling the scheduler
four times per step brought every task to its rate, but it changes
Betaflight's timing everywhere (RX then ran at 8 kHz) and was not
evaluated beyond that, so it was not kept. Patch 0005 fixes the part that
was wrong rather than late.

## Kamikaze builds

### D24. Warhead, bomb arm and explode switches (`LIBFPV_KAMIKAZE`)

Kamikaze (strike) FPV is a separate build, `-DLIBFPV_KAMIKAZE=ON`, off by
default. Without it nothing changes: no new symbols, no new channels, same
header (the kamikaze declarations sit behind `#if defined(LIBFPV_KAMIKAZE)`),
same behaviour. The option is a PUBLIC compile definition of `libfpv::fpv`,
so hosts linking through CMake see the API automatically. A host built
with it against a library built without it fails to link or to resolve
`fpv_kamikaze_*`, which is the feature check.

**Installed or not.** The bomb is a per-session choice, off by default:
`fpv_kamikaze_set_installed` or the overlay's "Bomb installed" (F8 >
Fly). Uninstalled, the drone is the plain quad of other builds: no
payload weight or collision sphere, no warhead OSD, and `Warhead` ignores
AUX7 and AUX8 (it does not hold AUX1, reports both switches off, and
neither the switches nor `fpv_kamikaze_detonate` can fire it). The flag
lives in `Lib` and on the `Warhead` of the one `Flight`, which outlives
drones, so a new, respawned or repaired drone keeps it; it is not saved in
`settings.json`, since a game decides per mission whether the drone
carries a bomb. Installing or removing it safes the warhead (a switch left
on must be cycled), except on a wreck, which stays one until repaired.

**Switches.** AUX7 is the bomb arm switch and AUX8 the explode switch.
`FPV_CHANNEL_COUNT` becomes 12 and `fpv_frame.channels` grows at its end,
the same `struct_size`-safe way as in D23. Betaflight receives both
channels like any other (the Configurator's Receiver tab shows them) but
has no mode on them: on real strike drones the firing board, not the
flight controller, reads them. The radio wizard adds both steps after the
OSD switch, both skippable; the keyboard fallback uses K (bomb toggle) and
X (explode, held), the gamepad West (toggle) and the right shoulder.

**Rules,** in `src/sim/kamikaze.cpp` (`Warhead`, simulation thread):

* The motors do not arm while the bomb arm switch is on, and a destroyed
  drone never arms. libfpv holds AUX1 low towards Betaflight, and keeps
  holding it until the pilot cycles the arm switch, like Betaflight's own
  ARMSWITCH rule; otherwise turning the bomb off with the arm switch on
  would arm the motors at once. Betaflight has no arming-disable flag for
  this and none was patched in, so `arming_disable_flags` stays 0 and
  `arming_disable_text` reads "BOMB".
* Switch on with the motors armed: ARMED at once, or ARMING for
  `arming_delay` first when one is set (default 0: strike pilots fly with
  a live warhead the moment they flip it; a delay is a game's or a
  player's choice). Switch off at any time: SAFE. Motors disarmed while ARMING: SAFE
  and the switch must be cycled. Motors disarmed while ARMED: it stays
  live (a falling live drone is still a bomb).
* A switch that is on when the drone spawns or is repaired must be turned
  off first, so a respawn never comes with a live or arming warhead.
* Only ARMED explodes, on a contact, on the explode switch's off-to-on
  edge (an explode switch already on when it goes live does nothing until
  cycled), or on `fpv_kamikaze_detonate`.
* The fuse is a contact fuse: every collision contact approaching faster
  than `min_impact_speed` (default 0.25 m/s, 0.1..30, tunable in the
  overlay) fires it, and so does a spinning prop striking something. That
  is far below the 1.5 m/s of `fpv_frame.impacts`, so a light tap on the
  ground counts. Contacts slower than 0.1 m/s never reach it: a drone
  resting on the ground with a live warhead does not explode.

**After the explosion** the drone is destroyed but not deleted, so every
kind of game can build on it: the wreck stays frozen where it exploded,
the props are broken (CRASHED), the motors are stopped, AUX1 is held low,
the video reports no signal (noise 1, signal 0), and the OSD shows only
"SIGNAL LOST" until the repair, the way goggles do when the video drops. It
is the goggles' message, not the flight controller's, so the OSD disable
switch doesn't hide it; games can still draw their own with
`FPV_KAMIKAZE_NO_OSD`. `fpv_drone_repair` is the new drone with a
new, safe warhead. libfpv never respawns on its own: whether the game
cuts to a cutscene, respawns at once, or leaves the pilot on static until
they take the next drone is the game's choice (INTEGRATION.md, "Kamikaze
builds"). Detonations are queued (`fpv_kamikaze_poll_detonation`, up to
16) rather than put in `fpv_frame`, so frames keep their layout and a
host polling less often loses nothing.

**OSD.** The warhead state is a Betaflight OSD element: `SAFE`,
`BOMB SW ON` (blinking, the motors are blocked), `ARMING 3`, `STEADY`
(live) and `NOT ARMED` (explode switch on a safe bomb). These are the words
seen on strike-drone footage; the exact wording of real firing boards is
not public, so libfpv follows the convention rather than a specific board.
At first libfpv wrote the text into `frame.osd` over Betaflight's grid,
which put it above everything, the post-flight stats included, and fixed
on row 3. It now takes Betaflight's "Custom message 1" (`OSD_CUSTOM_MSG0`,
meant for text sent over MSPv2): Betaflight patch 0006 lets the host
supply that element's text (`bfb_osd_set_host_message`, set every step by
`Flight`), and SimITL patch 0010 adds `osd_custom_msg0_pos`..`3_pos` to
the CLI, which upstream lacks (the Configurator moves it over MSP), so the
overlay's OSD tab ("Bomb status") and scripts can place it too. Betaflight
then layers it like any element, as the pilot name became one after
patch 0003. The text is centred in a 10-character field so it stays put as
it changes, the field defaults to columns 10..19 of row 3, and EEPROM
upgrade `bomb-osd-4` shows it there unless the player already placed it.
Blinking is 1 Hz because Betaflight redraws the simulated OSD only every
~0.3 s. `SIGNAL LOST` (row 7) is still drawn by libfpv over a blank grid:
it is the goggles' message, not the flight controller's. Both can be
turned off (`FPV_KAMIKAZE_NO_OSD`) for games that draw their own, and
since Betaflight's `osd_revision` cannot see `SIGNAL LOST`, kamikaze
builds count revisions of the composed grid instead.

**Payload.** The warhead is real weight. Its mass is added to the quad's
in SimITL, its inertia is added as a point mass `drop` below the centre
(parallel axis) plus its own solid sphere, and it adds a collision sphere.
The default is the preset's `"payload": {"mass", "drop"}`, or else 60 % of
the quad's own mass strapped under the frame, flush with the landing
points; a sphere hanging below them made the drone balance on it and tip
over (ANGLE). The centre of mass does not move, because SimITL has no
offset for it. Changing the payload patches SimITL's live `stateInit`
instead of re-initialising the physics, so it can change in flight.
`fpv_kamikaze_config` and the overlay (F8 > Fly) edit the same values
(payload, arming delay, fuse threshold, OSD), saved in `settings.json`;
those settings only show with the bomb installed.

Tests: `kamikaze_*` in `fpv_tests` (only in kamikaze builds; the other
tests run there without the bomb installed, since they check a bare quad
and Betaflight's own OSD), the wizard test with both switches,
and `fpv_standalone --kamikaze-demo`, which flies a scripted strike in the
real app and exits 1 if any rule fails.
