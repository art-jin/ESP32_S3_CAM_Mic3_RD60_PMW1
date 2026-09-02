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

static const char *TAG = "fusion";

static SemaphoreHandle_t s_lock;
static fusion_result_t s_last;

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

void fusion_evaluate(float doa_az_deg)
{
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
    xSemaphoreTake(s_lock, portMAX_DELAY);
    was_assoc = s_last.associated;
    s_last = r;
    xSemaphoreGive(s_lock);

    if (r.associated != was_assoc) {
        if (r.associated) {
            ESP_LOGI(TAG, "ASSOC doa=%.0f° radar=%.0f° diff=%.0f° range=%umm (%s)",
                     r.doa_az_deg, r.radar_az_deg, r.angle_diff_deg,
                     r.range_mm,
                     t.state == RADAR_TGT_MOTION ? "运动" :
                     t.state == RADAR_TGT_BREATH ? "呼吸" : "目标");
        } else if (r.evaluated && online) {
            ESP_LOGI(TAG, "UNASSOC doa=%.0f° (radar az=%.0f° diff=%.0f° > gate)",
                     r.doa_az_deg, r.radar_az_deg, r.angle_diff_deg);
        } else {
            ESP_LOGI(TAG, "UNASSOC doa=%.0f° (radar %s)", r.doa_az_deg,
                     online ? "no target" : "offline");
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
