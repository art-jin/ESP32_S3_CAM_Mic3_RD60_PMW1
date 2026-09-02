#pragma once
#include <stdbool.h>

/* Dual-mode manager: TRACK (auto sound-source tracking) and COMMAND
 * (REST-directed servo positioning). Default is TRACK on boot. */

typedef enum {
    MODE_TRACK = 0,     /* tracker ON, /point rejected */
    MODE_COMMAND,       /* tracker OFF, /point accepted */
} app_mode_t;

void mode_manager_init(void);

/* Atomic read of current mode. */
app_mode_t mode_manager_get(void);

/* Switch mode. Handles tracker enable/disable + state reset.
 * timeout_s: command mode auto-return timeout (0 = never). */
void mode_manager_set(app_mode_t mode, int timeout_s);

/* Get current effective timeout (0 = no auto-return). */
int mode_manager_get_timeout(void);

/* Called from mic_task at 20Hz. Checks command-mode timeout. */
void mode_manager_tick(void);

/* Called from /api/point handler. Resets the command-mode timer. */
void mode_manager_register_command(void);
