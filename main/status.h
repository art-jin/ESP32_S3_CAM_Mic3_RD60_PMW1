#pragma once
#include <stdbool.h>
#include "mode_manager.h"
#include "doa.h"

/* Thread-safe global status shared between mic_task (writer, 20Hz) and
 * httpd task (reader, on-demand). Copy-in/copy-out under mutex. */

typedef struct {
    /* DOA (updated by mic_task at 20Hz) */
    float    azimuth;
    int      sextant;
    int      stable_sect;
    float    confidence;
    float    lr_corr;

    /* Servo */
    float    servo_angle;
    bool     servo_moving;

    /* Mode */
    app_mode_t mode;

    /* Network */
    bool     wifi_connected;
    char     ip[16];
    char     hostname[32];
} device_status_t;

void status_init(void);

/* Called from mic_task every frame (20Hz). Safe to call rapidly — just
 * takes a mutex, copies fields, releases. */
void status_update(const doa_result_t *r);

/* Called from HTTP handler. Copies out a consistent snapshot. */
void status_get(device_status_t *out);
