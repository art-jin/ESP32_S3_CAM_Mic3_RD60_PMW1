/* evlog.c — persistent event ring buffer in NVS for post-mortem diagnostics.
 *
 * 32-slot ring of 8-byte events. Survives reboots. On every boot the
 * previous boot's events are printed to UART, then the buffer keeps
 * accumulating (oldest events get overwritten cyclically).
 *
 * NVS layout (namespace "evlog"):
 *   "seq"  (u16)  — next sequence number
 *   "idx"  (u8)   — next write slot (0..31)
 *   "boot" (u32)  — boot counter, incremented on each evlog_init
 *   "e00".."e31"  — one 8-byte blob per slot
 *
 * The buffer is intentionally NOT cleared on init — repeated boots keep
 * showing the same events until they're overwritten. Use the seq field
 * to disambiguate.
 *
 * Thread-safety: a single mutex serializes all access. NVS handles are
 * opened/closed per call (cheap) to avoid holding a long-lived RW handle.
 */
#include "evlog.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "evlog";

#define EVLOG_NS    "evlog"
#define EVLOG_CAP   32

static SemaphoreHandle_t s_mutex = NULL;
static uint16_t s_seq = 0;
static uint8_t  s_idx = 0;
static uint32_t s_boot = 0;

/* Lookup table for human-readable event type. Exposed via evlog_type_str()
 * for both UART boot-print and /api/logs JSON output. */
const char *evlog_type_str(uint8_t type)
{
    switch (type) {
        case EV_BOOT:        return "BOOT";
        case EV_SWEEP_DONE:  return "SWEEP_DONE";
        case EV_DOA_FIRST:   return "DOA_FIRST";
        case EV_SERVO_CMD:   return "SERVO_CMD";
        case EV_WIFI_UP:     return "WIFI_UP";
        case EV_MODE_CHG:    return "MODE_CHG";
        case EV_SHAKE_START: return "SHAKE_START";
        case EV_SHAKE_END:   return "SHAKE_END";
        case EV_RADAR_UP:    return "RADAR_UP";
        case EV_RADAR_DOWN:  return "RADAR_DOWN";
        case EV_STILL:       return "STILL";
        case EV_OOR:         return "OOR";
        default:             return "?";
    }
}

void evlog_init(void)
{
    s_mutex = xSemaphoreCreateMutex();

    nvs_handle_t h;
    if (nvs_open(EVLOG_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed, event logging disabled");
        return;
    }

    /* Read existing indices (defaults to 0 on first boot). */
    uint16_t seq = 0;
    uint8_t  idx = 0;
    uint32_t boot = 0;
    nvs_get_u16(h, "seq",  &seq);
    nvs_get_u8 (h, "idx",  &idx);
    nvs_get_u32(h, "boot", &boot);

    /* Increment boot counter. */
    boot++;
    nvs_set_u32(h, "boot", boot);
    nvs_commit(h);
    s_boot = boot;   /* cache for evlog_get_boot_count() */

    ESP_LOGI(TAG, "=== event log: boot #%lu, seq=%u, idx=%u ===",
             (unsigned long)boot, seq, idx);

    /* Print all 32 slots in write-order (starting from idx, wrapping).
     * Empty slots (type == 0) are skipped. */
    for (int i = 0; i < EVLOG_CAP; i++) {
        uint8_t k = (uint8_t)((idx + i) % EVLOG_CAP);
        char key[5];
        snprintf(key, sizeof(key), "e%02u", k);
        event_t ev;
        size_t len = sizeof(ev);
        if (nvs_get_blob(h, key, &ev, &len) == ESP_OK
            && len == sizeof(ev)
            && ev.type != 0) {
            ESP_LOGI(TAG, "  [%8lu ms] seq=%5u  %-11s  flags=%u  value=%d",
                     (unsigned long)ev.uptime_ms, ev.seq,
                     evlog_type_str(ev.type), ev.flags, ev.value);
        }
    }
    ESP_LOGI(TAG, "=== end event log ===");

    nvs_close(h);

    /* Cache seq/idx in memory for fast subsequent evlog_record calls. */
    s_seq = seq;
    s_idx = idx;
}

void evlog_record(uint8_t type, uint8_t flags, int16_t value)
{
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    event_t ev = {
        .seq       = s_seq,
        .type      = type,
        .flags     = flags,
        .value     = value,
        .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000),
    };

    nvs_handle_t h;
    if (nvs_open(EVLOG_NS, NVS_READWRITE, &h) == ESP_OK) {
        char key[5];
        snprintf(key, sizeof(key), "e%02u", s_idx);
        nvs_set_blob(h, key, &ev, sizeof(ev));
        nvs_set_u16(h, "seq", (uint16_t)(s_seq + 1));
        nvs_set_u8 (h, "idx", (uint8_t)((s_idx + 1) % EVLOG_CAP));
        nvs_commit(h);
        nvs_close(h);

        s_seq++;
        s_idx = (uint8_t)((s_idx + 1) % EVLOG_CAP);
    }
    xSemaphoreGive(s_mutex);
}

/* ---- Read API (for /api/logs) ---- */

int evlog_capacity(void)
{
    return EVLOG_CAP;
}

uint32_t evlog_get_boot_count(void)
{
    if (!s_mutex) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint32_t b = s_boot;
    xSemaphoreGive(s_mutex);
    return b;
}

/* Returns string literal or NULL. Uses a static buffer for sextant formatting
 * (single-threaded caller in httpd; non-reentrant but safe in this usage). */
const char *evlog_flags_str(uint8_t type, uint8_t flags)
{
    if (type == EV_SERVO_CMD) {
        switch (flags) {
            case SRC_TRACKER: return "TRACKER";
            case SRC_REST:    return "REST";
            case SRC_SHAKE:   return "SHAKE";
            case SRC_IDLE:    return "IDLE";
            case SRC_BOOT:    return "BOOT";
            default:          return "?";
        }
    }
    if (type == EV_DOA_FIRST && flags <= 5) {
        static char s[4];
        snprintf(s, sizeof(s), "S%u", flags);
        return s;
    }
    return NULL;
}

int evlog_get_events(event_t *out, int max_count)
{
    if (!s_mutex) return 0;
    if (max_count <= 0) return 0;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Single NVS open reads all 32 blobs (mirrors evlog_init pattern). */
    nvs_handle_t h;
    int count = 0;
    if (nvs_open(EVLOG_NS, NVS_READONLY, &h) == ESP_OK) {
        int limit = (max_count < EVLOG_CAP) ? max_count : EVLOG_CAP;
        for (int i = 0; i < limit; i++) {
            uint8_t k = (uint8_t)((s_idx + i) % EVLOG_CAP);
            char key[5];
            snprintf(key, sizeof(key), "e%02u", k);
            event_t ev;
            size_t len = sizeof(ev);
            if (nvs_get_blob(h, key, &ev, &len) == ESP_OK
                && len == sizeof(ev)
                && ev.type != 0) {
                out[count++] = ev;
            }
        }
        nvs_close(h);
    }

    xSemaphoreGive(s_mutex);
    return count;
}
