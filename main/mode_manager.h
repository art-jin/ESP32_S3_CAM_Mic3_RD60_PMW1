#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Dual-mode manager: TRACK (auto sound-source tracking) and COMMAND
 * (REST-directed servo positioning). Default is TRACK on boot.
 *
 * TRACK has three sub-modes (Phase 3, PRD US-005):
 *   audio_only  — only sound drives the servo (base behaviour, default)
 *   fusion      — sound has priority; with no recent sound the servo
 *                 follows the radar primary target
 *   radar_follow— only the radar target drives the servo
 *
 * Out-of-range policy (US-006): what the servo does when the target
 * azimuth is beyond the mechanical ±90° (9点~3点):
 *   hold (default) / clamp to boundary / return home / boundary scan (v2.5
 *   base behaviour, kept as an option).
 *
 * Sub-mode and policy persist in NVS across reboots. */

typedef enum {
    MODE_TRACK = 0,     /* tracker ON, /point rejected */
    MODE_COMMAND,       /* tracker OFF, /point accepted */
} app_mode_t;

typedef enum {
    TRACK_AUDIO_ONLY = 0,
    TRACK_FUSION,
    TRACK_RADAR_FOLLOW,
} track_submode_t;

typedef enum {
    OOR_HOLD = 0,
    OOR_CLAMP,
    OOR_HOME,
    OOR_SCAN,
} oor_policy_t;

void mode_manager_init(void);

/* Atomic read of current mode. */
app_mode_t mode_manager_get(void);

/* Switch mode. Handles tracker enable/disable + state reset.
 * timeout_s: command mode auto-return timeout (0 = never). */
void mode_manager_set(app_mode_t mode, int timeout_s);

/* Get current effective timeout (0 = no auto-return). */
int mode_manager_get_timeout(void);

/* Track sub-mode (only meaningful in MODE_TRACK). Resets tracker state
 * on change for a clean transition. Persisted in NVS. */
void mode_manager_set_submode(track_submode_t submode);
track_submode_t mode_manager_get_submode(void);

/* Out-of-range policy. Persisted in NVS. */
void mode_manager_set_oor_policy(oor_policy_t policy);
oor_policy_t mode_manager_get_oor_policy(void);

/* Long-stillness care alarm threshold in minutes (US-009): fire
 * STILL_ALARM when the radar target stays in breath/micro-motion state
 * continuously for this long. 0 = disabled (default). Persisted in NVS. */
void mode_manager_set_still_min(uint16_t minutes);
uint16_t mode_manager_get_still_min(void);

/* Called from mic_task at 20Hz. Checks command-mode timeout. */
void mode_manager_tick(void);

/* Called from /api/point handler. Resets the command-mode timer. */
void mode_manager_register_command(void);
