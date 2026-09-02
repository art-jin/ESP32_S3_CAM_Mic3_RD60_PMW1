#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "events.h"
#include "evlog.h"

#define EV_RING_N 32

static const char *TAG = "events";

static app_event_t s_ring[EV_RING_N];
static int   s_head;              /* next write slot */
static uint32_t s_seq;
static SemaphoreHandle_t s_lock;

/* NVS evlog mirror only for events that matter after a reboot:
 * link state, care alarms, out-of-range. Transient scene events
 * (enter/leave/assoc) stay RAM-only — they would evict crash forensics
 * from the 32-slot NVS ring for little diagnostic value. */
static void mirror_to_evlog(uint8_t type, int16_t v1, int16_t v2)
{
    switch (type) {
    case AEVT_RADAR_OFFLINE: evlog_record(EV_RADAR_DOWN, 0, v1); break;
    case AEVT_RADAR_ONLINE:  evlog_record(EV_RADAR_UP, 0, v1); break;
    case AEVT_STILL_ALARM:   evlog_record(EV_STILL, 1, v1); break;
    case AEVT_STILL_RECOVER: evlog_record(EV_STILL, 0, v1); break;
    case AEVT_OOR:           evlog_record(EV_OOR, (uint8_t)v2, v1); break;
    default: break;
    }
}

void events_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_head = 0;
    s_seq = 0;
    ESP_LOGI(TAG, "init: %d-slot ring", EV_RING_N);
}

void events_push(uint8_t type, int16_t v1, int16_t v2)
{
    if (!s_lock || type >= AEVT_TYPE_COUNT) return;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    app_event_t *e = &s_ring[s_head];
    e->seq = ++s_seq;
    e->uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    e->type = type;
    e->v1 = v1;
    e->v2 = v2;
    s_head = (s_head + 1) % EV_RING_N;
    xSemaphoreGive(s_lock);

    mirror_to_evlog(type, v1, v2);
}

int events_get(app_event_t *out, int max, uint32_t since, uint32_t *next)
{
    if (!s_lock || !out || max <= 0) return 0;
    int n = 0;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (next) *next = s_seq;
    /* Ring holds the newest EV_RING_N events; iterate oldest → newest. */
    for (int k = EV_RING_N; k > 0 && n < max; k--) {
        int idx = (s_head + EV_RING_N - k) % EV_RING_N;
        const app_event_t *e = &s_ring[idx];
        /* Empty slots (before first wrap) have seq == 0. */
        if (e->seq == 0 || e->seq <= since) continue;
        out[n++] = *e;
    }
    xSemaphoreGive(s_lock);
    return n;
}

const char *events_type_name(uint8_t type)
{
    switch (type) {
    case AEVT_TARGET_ENTER:   return "TARGET_ENTER";
    case AEVT_TARGET_LEAVE:   return "TARGET_LEAVE";
    case AEVT_SOUND_ASSOC:    return "SOUND_ASSOC";
    case AEVT_SOUND_UNASSOC:  return "SOUND_UNASSOC";
    case AEVT_OOR:            return "OOR";
    case AEVT_RADAR_OFFLINE:  return "RADAR_OFFLINE";
    case AEVT_RADAR_ONLINE:   return "RADAR_ONLINE";
    case AEVT_STILL_ALARM:    return "STILL_ALARM";
    case AEVT_STILL_RECOVER:  return "STILL_RECOVER";
    default:                  return "?";
    }
}
