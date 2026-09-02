#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "doa.h"
#include "mic_capture.h"
#include "servo.h"
#include "tracker.h"
#include "wifi.h"
#include "mode_manager.h"
#include "status.h"
#include "evlog.h"

static const char *TAG = "main";

/* Device ID: generated once on first boot, stored in NVS, never changes.
 * 6 chars from [A-Z0-9]. Accessed by rest_api.c for auth. */
char g_device_id[8] = {0};

static void device_id_init(void)
{
    nvs_handle_t h;
    size_t len = sizeof(g_device_id);
    if (nvs_open("device", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_str(h, "id", g_device_id, &len) == ESP_OK) {
            nvs_close(h);
            ESP_LOGI(TAG, "Device ID (from NVS): %s", g_device_id);
            return;
        }
        nvs_close(h);
    }
    /* Not in NVS yet — generate random 6-char ID. */
    static const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    for (int i = 0; i < 6; i++) {
        g_device_id[i] = charset[esp_random() % 36];
    }
    g_device_id[6] = '\0';
    /* Save to NVS so it persists across reboots. */
    if (nvs_open("device", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "id", g_device_id);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "Device ID (newly generated): %s", g_device_id);
}

/* DMA window + per-channel scratch. All static — the mic task is the only
 * consumer, so there's no need for heap allocation or locking. */
static int16_t s_dma[MIC_WINDOW_SAMPLES];
static int16_t s_c0[DOA_FFT_N];
static int16_t s_c1[DOA_FFT_N];
static int16_t s_c2[DOA_FFT_N];

static float ac_rms(const int16_t *x, size_t n)
{
    if (n == 0) return 0.0f;
    int64_t sum = 0;
    for (size_t i = 0; i < n; i++) sum += x[i];
    float mean = (float)sum / (float)n;
    float acc = 0.0f;
    for (size_t i = 0; i < n; i++) {
        float d = (float)x[i] - mean;
        acc += d * d;
    }
    return sqrtf(acc / (float)n);
}

/* Deinterleave the 3-channel DMA buffer into per-channel scratch buffers,
 * taking the first DOA_FFT_N samples of each, then apply pre-emphasis
 * in-place (y[i] = x[i] - 0.97*x[i-1]). Pre-emphasis is applied to ALL
 * downstream consumers: L/R correlation (ρ01), GCC-PHAT, AC RMS. This
 * is critical — applying it only inside GCC-PHAT (as an earlier version
 * did) left ρ01 operating on raw PCM, so speech (dominated by low-freq
 * voiced energy) still showed ρ01 > 0.95 → L/R collapse → no 3-mic. */
static void deinterleave(const int16_t *dma, size_t n_per_ch)
{
    size_t take = n_per_ch < DOA_FFT_N ? n_per_ch : DOA_FFT_N;
    float prev0 = 0.0f, prev1 = 0.0f, prev2 = 0.0f;
    for (size_t i = 0; i < take; i++) {
        float r0 = dma[i * MIC_NUM_CHANNELS + 0];
        float r1 = dma[i * MIC_NUM_CHANNELS + 1];
        float r2 = dma[i * MIC_NUM_CHANNELS + 2];
        s_c0[i] = (int16_t)(r0 - 0.97f * prev0);
        s_c1[i] = (int16_t)(r1 - 0.97f * prev1);
        s_c2[i] = (int16_t)(r2 - 0.97f * prev2);
        prev0 = r0; prev1 = r1; prev2 = r2;
    }
}

static void mic_task(void *arg)
{
    (void)arg;
    int64_t last_print   = esp_timer_get_time();
    int64_t stats_start  = esp_timer_get_time();
    int frames_3 = 0, frames_2 = 0, frames_bad = 0;
    int first = 1;

    while (1) {
        size_t got = 0;
        esp_err_t err = mic_capture_read(s_dma, sizeof(s_dma), &got);
        if (err != ESP_OK || got != sizeof(s_dma)) {
            ESP_LOGW(TAG, "i2s read err=%s got=%u", esp_err_to_name(err), (unsigned)got);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        /* Drop the first window — boot-phase DMA garbage. */
        if (first) { first = 0; taskYIELD(); continue; }

        deinterleave(s_dma, MIC_WINDOW_PER_CH);

        doa_result_t r;
        doa_process(s_c0, s_c1, s_c2, DOA_FFT_N, &r);

        /* Phase 2: hand the DOA result to the tracker, which decides
         * whether to move the servo. Tracker applies deadband + clamp
         * internally. (Phase 3 will add motion-pause to skip DOA updates
         * while the servo is moving, so servo whine doesn't feed back.) */
        tracker_update(&r);
        mode_manager_tick();    /* check command-mode timeout (20Hz) */
        status_update(&r);     /* publish to global status for REST API */

        if      (r.mode == DOA_MODE_3MIC) frames_3++;
        else if (r.mode == DOA_MODE_2MIC) frames_2++;
        else                              frames_bad++;

        int64_t now = esp_timer_get_time();

        /* 5-second mode histogram — surfaces whether we're getting 3-mic
         * localization at all, or stuck in 2-mic fallback, or rejecting
         * most frames as noise. */
        if (now - stats_start > 5 * 1000 * 1000) {
            int total = frames_3 + frames_2 + frames_bad;
            ESP_LOGI(TAG, "[5s] %d frames: 3-mic=%d (%d%%)  2-mic=%d (%d%%)  bad=%d  DeviceID=%s",
                     total,
                     frames_3, total ? frames_3 * 100 / total : 0,
                     frames_2, total ? frames_2 * 100 / total : 0,
                     frames_bad, g_device_id);
            frames_3 = frames_2 = frames_bad = 0;
            stats_start = now;
        }

        /* Per-frame line throttled to 2 Hz. */
        if (now - last_print < 500 * 1000) {
            taskYIELD();
            continue;
        }
        last_print = now;

        float ac0 = ac_rms(s_c0, DOA_FFT_N);
        float ac1 = ac_rms(s_c1, DOA_FFT_N);
        float ac2 = ac_rms(s_c2, DOA_FFT_N);

        const char *sext = doa_sextant_label(r.sextant, r.mode);
        const char *stable = doa_sextant_label(r.stable_sextant,
                                               r.stable_sextant >= 0 ? DOA_MODE_3MIC : DOA_MODE_INVALID);
        const char *tmode;
        switch (tracker_get_mode()) {
            case TRACKER_MODE_TRACKING:   tmode = "TRK"; break;
            case TRACKER_MODE_IDLE:       tmode = "idl"; break;
            case TRACKER_MODE_SUPPRESSED: tmode = "sup"; break;
            case TRACKER_MODE_DISABLED:   tmode = "OFF"; break;
            default:                      tmode = "?";
        }

        if (r.mode == DOA_MODE_3MIC) {
            ESP_LOGI(TAG,
                ">>> 3-MIC  az=%6.1f°  sect=%d (%s)  stable=%d (%s)  conf=%.2f  "
                "lag01=%+5.2f lag02=%+5.2f lag12=%+5.2f  "
                "| ac %.0f/%.0f/%.0f  ρ01=%.2f peak01=%.2f  "
                "| servo=%+.1f° %s [%s]",
                r.azimuth_deg, r.sextant, sext,
                r.stable_sextant, stable, r.confidence,
                r.lag_pair[0], r.lag_pair[1], r.lag_pair[2],
                ac0, ac1, ac2, r.lr_corr, r.peak_pair[0],
                servo_get_angle_deg(), servo_is_moving() ? "[MOTION]" : "",
                tmode);
        } else if (r.mode == DOA_MODE_2MIC) {
            ESP_LOGI(TAG,
                " .  2-MIC  half=%5.1f°  bin=%d (%s)  conf=%.2f  "
                "lag02=%+5.2f  mask=0x%x  "
                "| ac %.0f/%.0f/%.0f  ρ01=%.2f (L/R collapsed)  "
                "| servo=%+.1f° %s [%s]",
                r.azimuth_deg, r.sextant, sext, r.confidence,
                r.lag_pair[1], r.mics_valid_mask,
                ac0, ac1, ac2, r.lr_corr,
                servo_get_angle_deg(), servo_is_moving() ? "[MOTION]" : "",
                tmode);
        } else {
            ESP_LOGI(TAG,
                "    -----  conf=%.2f  stable=%d (%s)  "
                "| ac %.0f/%.0f/%.0f  ρ01=%.2f  (silent / noisy)  "
                "| servo=%+.1f° %s [%s]",
                r.confidence, r.stable_sextant, stable,
                ac0, ac1, ac2, r.lr_corr,
                servo_get_angle_deg(), servo_is_moving() ? "[MOTION]" : "",
                tmode);
        }

        /* Display update removed — TFT SPI uses GPIO 19/20 which conflicts
         * with USB-CDC. Will need WiFi-based display or alternative pins. */

        taskYIELD();
    }
}

void app_main(void)
{
    /* NVS — required by WiFi driver. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    device_id_init();  /* load or generate persistent device ID */

    evlog_init();  /* load prior events from NVS, increment boot counter */
    evlog_record(EV_BOOT, 0, (int16_t)esp_reset_reason());

    ESP_LOGI(TAG, "=== 3DMIC-291 DOA + servo + REST API starting ===");
    ESP_LOGI(TAG, "  pins: CLK0=%d CLK1=%d DAT0=%d DAT1=%d",
             MIC_CLK0_GPIO, MIC_CLK1_GPIO, MIC_DAT0_GPIO, MIC_DAT1_GPIO);
    ESP_LOGI(TAG, "  PCM=%d Hz  FFT=%d pts  K=%.3f samp  max_lag=±%d",
             MIC_SAMPLE_RATE_HZ, DOA_FFT_N, DOA_K, DOA_MAX_LAG);

    doa_init();
    ESP_ERROR_CHECK(mic_capture_init());
    ESP_ERROR_CHECK(servo_init());
    servo_boot_sweep();   /* visual confirmation of home + range before tracking */
    tracker_init(NULL);
    mode_manager_init();  /* default: MODE_TRACK */
    status_init();        /* global status for REST API */

    /* WiFi — non-blocking, connects in background. REST API starts
     * automatically once IP is obtained. Track mode works without WiFi. */
    wifi_init();

    /* 8 KB stack covers the FFT scratch (static) + libc math + ESP_LOG. */
    xTaskCreate(mic_task, "mic", 8192, NULL,
                configMAX_PRIORITIES - 2, NULL);
}
