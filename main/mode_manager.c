#include "mode_manager.h"
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "tracker.h"
#include "evlog.h"

static const char *TAG = "mode";

/* Default command-mode timeout: 5 minutes. 0 = never auto-return. */
#define COMMAND_DEFAULT_TIMEOUT_S  300

static _Atomic app_mode_t s_mode = MODE_TRACK;
static int64_t s_last_command_us = 0;
static int     s_timeout_s = COMMAND_DEFAULT_TIMEOUT_S;

void mode_manager_init(void)
{
    atomic_store(&s_mode, MODE_TRACK);
    s_last_command_us = 0;
    s_timeout_s = COMMAND_DEFAULT_TIMEOUT_S;
    ESP_LOGI(TAG, "init: default TRACK, command timeout %ds", s_timeout_s);
}

app_mode_t mode_manager_get(void)
{
    return atomic_load(&s_mode);
}

void mode_manager_set(app_mode_t mode, int timeout_s)
{
    app_mode_t old = atomic_exchange(&s_mode, mode);
    if (old != mode) {
        evlog_record(EV_MODE_CHG, (uint8_t)old, (int16_t)mode);
    }

    if (mode == MODE_COMMAND) {
        tracker_set_enabled(false);
        s_timeout_s = (timeout_s >= 0) ? timeout_s : COMMAND_DEFAULT_TIMEOUT_S;
        s_last_command_us = esp_timer_get_time();
        ESP_LOGI(TAG, "TRACK -> COMMAND (timeout=%ds)", s_timeout_s);
    } else {
        tracker_reset_state();
        tracker_set_enabled(true);
        ESP_LOGI(TAG, "COMMAND -> TRACK");
    }
    (void)old;
}

int mode_manager_get_timeout(void)
{
    return s_timeout_s;
}

void mode_manager_tick(void)
{
    if (atomic_load(&s_mode) != MODE_COMMAND) return;
    if (s_timeout_s == 0) return;  /* no auto-return */

    int64_t now = esp_timer_get_time();
    int64_t elapsed_s = (now - s_last_command_us) / 1000000;
    if (elapsed_s > s_timeout_s) {
        ESP_LOGI(TAG, "command timeout (%ds), returning to TRACK", s_timeout_s);
        mode_manager_set(MODE_TRACK, 0);
    }
}

void mode_manager_register_command(void)
{
    s_last_command_us = esp_timer_get_time();
}
