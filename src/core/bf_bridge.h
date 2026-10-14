/*
 * libfpv - Betaflight-side glue, C interface usable from libfpv's C++ code.
 * All functions must be called on the simulation thread.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef LIBFPV_BF_BRIDGE_H
#define LIBFPV_BF_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    BFB_CLI_OK = 0,
    BFB_CLI_BUSY = 1,   /* the Configurator is in CLI mode */
    BFB_CLI_ERROR = 2,
};

/* Runs CLI commands (newline separated) and returns the CLI output in
 * *output (free with bfb_free). Never send save/exit/defaults without
 * "noreboot"/"nosave": libfpv filters those before calling this. */
int bfb_cli_run(const char *script, char **output);
void bfb_free(void *p);

typedef struct {
    uint8_t ratesType;
    uint8_t rcRate[3];
    uint8_t rate[3];
    uint8_t expo[3];
    uint16_t rateLimit[3];
    uint8_t profileIndex;
} bfbRates_t;

bool bfb_is_armed(void);
uint32_t bfb_arming_disable_flags(void);
const char *bfb_arming_disable_flag_name(int index);
int bfb_arming_disable_flag_count(void);
bool bfb_failsafe_active(void);
uint32_t bfb_flight_mode_flags(void);
bool bfb_config_dirty(void);
void bfb_write_eeprom(void);
int bfb_cli_mode(void);
bfbRates_t bfb_get_rates(void);

/* Saves pending changes, then makes `path` the active EEPROM (one file per
 * quad preset). A missing file is created with Betaflight defaults.
 * Returns 1 if the file existed, 0 if created, -1 on error. */
int bfb_switch_eeprom(const char *path);
/* Betaflight's own reboot path (re-runs init in-process, like the
 * Configurator's "Save and Reboot"). */
void bfb_reboot(void);
int bfb_reboot_required(void);

/* Gyro/accelerometer/baro calibration after boot still running. */
int bfb_calibrating(void);
/* Betaflight's "props out" setting; the simulated props follow it. */
int bfb_yaw_motors_reversed(void);
/* Armed in crash flip (turtle) mode: the motors spin backwards. */
int bfb_crash_flip_active(void);
/* Calibration, RX recovery or boot grace still pending. */
int bfb_settling(void);

/* The GPS feature is on in the active EEPROM. */
int bfb_gps_configured(void);
/* Betaflight booted with the GPS feature on (features apply at boot). */
int bfb_gps_running(void);
/* Feeds Betaflight's virtual GPS: position (metres) and velocity (m/s) in
 * a local east/north/up frame. */
void bfb_gps_update(float east, float north, float up, float vEast, float vNorth, float vUp);
/* Feeds Betaflight's virtual barometer: altitude in metres. */
void bfb_baro_update(float up);
/* Makes the home point (same frame) override the one Betaflight sets when
 * arming. Call every step while the host's home applies; when it stops,
 * Betaflight takes the next arming point again. */
void bfb_gps_set_home(float east, float north, float up);

void bfb_ws_set_enabled(int enabled);
void bfb_ws_set_base_port(int basePort);
int bfb_ws_base_port(void);
int bfb_ws_client_count(void);

#ifdef __cplusplus
}
#endif

#endif
