/* Sound-source ↔ radar-target association — Phase 2 (PRD US-004).
 *
 * Rule: on a tracker-accepted DOA azimuth, compare against the radar
 * primary target's mapped azimuth. Associated when the radar is online,
 * the target is valid with trustworthy range/angle (rb_conf >= 12,
 * angle_conf >= 8) and the wrapped angle difference is within
 * FUSION_ASSOC_GATE_DEG. Otherwise the sound event is reported as
 * unassociated — per FR-6 the servo still follows the sound. */

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "fusion.h"
#include "events.h"

static const char *TAG = "fusion";

static SemaphoreHandle_t s_lock;
static fusion_result_t s_last;

/* Association-state hysteresis: DOA azimuth jitters ±15° in speech and
 * crosses the 20° gate every few hundred ms, which otherwise floods the
 * 32-slot event ring (observed 31 flips in 80 s). A flip is committed —
 * and pushed as an event — only after the new state persists 1 s, and
 * assoc/unassoc pushes are rate-limited to one per 10 s. */
#define FUSION_FLIP_HOLD_US  1000000LL
#define FUSION_EVT_MIN_US    10000000LL
static bool     s_flip_pending;
static bool     s_flip_to;
static int64_t  s_flip_since_us;
static int64_t  s_last_assoc_evt_us;

static float wrap180(float d)
{
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

void fusion_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    memset(&s_last, 0, sizeof(s_last));
    ESP_LOGI(TAG, "init: gate=%.0f°", FUSION_ASSOC_GATE_DEG);
}

void fusion_evaluate(float doa_az_deg, float confidence)
{
    /* Speech = substantial activity for the care alarm (US-009). Gate on
     * confidence: ambient noise (fans, street) passes the tracker's
     * 0.35 + 2-of-3 filters but not 0.5 — noise DOAs at az 12-75° were
     * observed resetting the stillness timer every few seconds. */
    if (confidence >= 0.5f) {
        radar_notify_sound();
    }

    radar_target_t t;
    bool has_radar = radar_get_target(&t);
    bool online = radar_is_online();

    fusion_result_t r = {
        .evaluated = true,
        .radar_online = online,
        .doa_az_deg = doa_az_deg,
        .timestamp_ms = esp_timer_get_time() / 1000,
    };

    if (online && has_radar && t.rb_conf >= 12 && t.angle_conf >= 8) {
        float diff = wrap180(doa_az_deg - t.azimuth_deg);
        r.radar_az_deg = t.azimuth_deg;
        r.angle_diff_deg = fabsf(diff);
        r.range_mm = t.range_mm;
        r.state = t.state;
        r.associated = r.angle_diff_deg <= FUSION_ASSOC_GATE_DEG;
    }

    bool was_assoc = false;
    int64_t now_us = esp_timer_get_time();
    r.assoc_instant = r.associated;   /* raw verdict, before hysteresis */

    /* Hysteresis commit: the reported association only flips (and logs /
     * pushes an event) when the desired state has held for 1 s and the
     * rate-limit window allows. Until then keep the previous state. */
    if (r.associated != s_last.associated) {
        if (!s_flip_pending || s_flip_to != r.associated) {
            s_flip_pending = true;
            s_flip_to = r.associated;
            s_flip_since_us = now_us;
        }
        if (now_us - s_flip_since_us >= FUSION_FLIP_HOLD_US &&
            now_us - s_last_assoc_evt_us >= FUSION_EVT_MIN_US) {
            r.associated = s_flip_to;
            s_flip_pending = false;
            s_last_assoc_evt_us = now_us;
        } else {
            r.associated = s_last.associated;   /* hold previous state */
        }
    } else {
        s_flip_pending = false;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    was_assoc = s_last.associated;
    s_last = r;
    xSemaphoreGive(s_lock);

    if (r.associated != was_assoc) {
        if (r.associated) {
            events_push(AEVT_SOUND_ASSOC, (int16_t)r.doa_az_deg,
                        (int16_t)(r.range_mm / 10));
            ESP_LOGI(TAG, "ASSOC doa=%.0f° radar=%.0f° diff=%.0f° range=%umm (%s)",
                     r.doa_az_deg, r.radar_az_deg, r.angle_diff_deg,
                     r.range_mm,
                     t.state == RADAR_TGT_MOTION ? "运动" :
                     t.state == RADAR_TGT_BREATH ? "呼吸" : "目标");
        } else {
            events_push(AEVT_SOUND_UNASSOC, (int16_t)r.doa_az_deg,
                        (int16_t)r.angle_diff_deg);
            if (r.evaluated && online) {
                ESP_LOGI(TAG, "UNASSOC doa=%.0f° (radar az=%.0f° diff=%.0f° > gate)",
                         r.doa_az_deg, r.radar_az_deg, r.angle_diff_deg);
            } else {
                ESP_LOGI(TAG, "UNASSOC doa=%.0f° (radar %s)", r.doa_az_deg,
                         online ? "no target" : "offline");
            }
        }
    }
}

void fusion_get_last(fusion_result_t *out)
{
    if (!out) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_last;
    xSemaphoreGive(s_lock);
}
