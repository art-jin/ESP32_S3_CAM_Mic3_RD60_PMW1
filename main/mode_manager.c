#include "mode_manager.h"
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "tracker.h"
#include "evlog.h"

static const char *TAG = "mode";

/* Default command-mode timeout: 5 minutes. 0 = never auto-return. */
#define COMMAND_DEFAULT_TIMEOUT_S  300

static _Atomic app_mode_t s_mode = MODE_TRACK;
static int64_t s_last_command_us = 0;
static int     s_timeout_s = COMMAND_DEFAULT_TIMEOUT_S;

static track_submode_t s_submode = TRACK_AUDIO_ONLY;
static oor_policy_t    s_oor_policy = OOR_HOLD;
static uint16_t        s_still_min = 0;   /* 0 = still-alarm disabled */

static const char *submode_name(track_submode_t s)
{
    switch (s) {
    case TRACK_AUDIO_ONLY:  return "audio_only";
    case TRACK_FUSION:      return "fusion";
    case TRACK_RADAR_FOLLOW:return "radar_follow";
    default:                return "?";
    }
}

static const char *oor_name(oor_policy_t p)
{
    switch (p) {
    case OOR_HOLD:  return "hold";
    case OOR_CLAMP: return "clamp";
    case OOR_HOME:  return "home";
    case OOR_SCAN:  return "scan";
    default:        return "?";
    }
}

static void cfg_save(const char *key, uint8_t val)
{
    nvs_handle_t h;
    if (nvs_open("mmode", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, key, val);
        nvs_commit(h);
        nvs_close(h);
    }
}

/* Load persisted sub-mode/policy; fall back to defaults on anything
 * invalid (first boot, corrupt, or future enum growth). */
static void cfg_load(void)
{
    nvs_handle_t h;
    if (nvs_open("mmode", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t v;
    if (nvs_get_u8(h, "submode", &v) == ESP_OK && v <= TRACK_RADAR_FOLLOW)
        s_submode = (track_submode_t)v;
    if (nvs_get_u8(h, "oorpol", &v) == ESP_OK && v <= OOR_SCAN)
        s_oor_policy = (oor_policy_t)v;
    uint16_t m;
    if (nvs_get_u16(h, "stillmin", &m) == ESP_OK && m <= 1440)
        s_still_min = m;
    nvs_close(h);
}

void mode_manager_init(void)
{
    atomic_store(&s_mode, MODE_TRACK);
    s_last_command_us = 0;
    s_timeout_s = COMMAND_DEFAULT_TIMEOUT_S;
    cfg_load();
    ESP_LOGI(TAG, "init: TRACK/%s oor=%s, command timeout %ds",
             submode_name(s_submode), oor_name(s_oor_policy), s_timeout_s);
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

void mode_manager_set_submode(track_submode_t submode)
{
    if (submode == s_submode) return;
    ESP_LOGI(TAG, "submode: %s -> %s", submode_name(s_submode),
             submode_name(submode));
    evlog_record(EV_MODE_CHG, (uint8_t)s_submode, (int16_t)submode);
    s_submode = submode;
    cfg_save("submode", (uint8_t)submode);
    tracker_reset_state();
}

track_submode_t mode_manager_get_submode(void)
{
    return s_submode;
}

void mode_manager_set_oor_policy(oor_policy_t policy)
{
    if (policy == s_oor_policy) return;
    ESP_LOGI(TAG, "oor policy: %s -> %s", oor_name(s_oor_policy),
             oor_name(policy));
    s_oor_policy = policy;
    cfg_save("oorpol", (uint8_t)policy);
}

oor_policy_t mode_manager_get_oor_policy(void)
{
    return s_oor_policy;
}

void mode_manager_set_still_min(uint16_t minutes)
{
    if (minutes > 1440) minutes = 1440;   /* cap at 24 h */
    if (minutes == s_still_min) return;
    ESP_LOGI(TAG, "still alarm: %u min (%s)", minutes,
             minutes ? "enabled" : "disabled");
    s_still_min = minutes;
    nvs_handle_t h;
    if (nvs_open("mmode", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u16(h, "stillmin", minutes);
        nvs_commit(h);
        nvs_close(h);
    }
}

uint16_t mode_manager_get_still_min(void)
{
    return s_still_min;
}
