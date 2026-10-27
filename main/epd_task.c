/* epd_task.c — e-ink bring-up task (M2). Boot self-test: white clear,
 * then a tri-color pattern that verifies all three colors, full X/Y span
 * and orientation. M3 will replace the pattern with the status page. */
#include "epd_task.h"
#include "board_config.h"

#if ENABLE_EPD

#include "epd.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "evlog.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

#define TAG "EPD"

enum { WHITE = 0, BLACK, RED };

static TaskHandle_t s_epd_task;
static epd_result_t s_result;   /* latest run snapshot, read by REST */

const epd_result_t *epd_last_result(void) { return &s_result; }

static void fill_rect(uint8_t *bw, uint8_t *red, int x0, int y0, int w, int h,
                      int color)
{
    for (int y = y0; y < y0 + h && y < EPD_HEIGHT; y++) {
        for (int x = x0; x < x0 + w && x < EPD_WIDTH; x++) {
            int idx = y * (EPD_WIDTH / 8) + x / 8;
            uint8_t bit = 1u << (7 - x % 8);
            switch (color) {
            case BLACK: bw[idx] &= ~bit; red[idx] &= ~bit; break;
            case RED:   bw[idx] &= ~bit; red[idx] |= bit;  break;
            default:    bw[idx] |= bit;  red[idx] &= ~bit; break; /* white */
            }
        }
    }
}

/* init -> clear -> tri-color pattern. Returns false when the panel didn't
 * answer (logged by epd_init). */
static bool epd_selftest(void)
{
    s_result.run_id++;
    s_result.running = true;
    s_result.init_ok = s_result.clear_engaged = s_result.pat_engaged = false;
    s_result.clear_ms = s_result.pat_ms = 0;
    s_result.started_uptime_ms = (int32_t)(esp_timer_get_time() / 1000);

    if (!epd_init()) {
        ESP_LOGE(TAG, "selftest aborted — panel not responding");
        s_result.running = false;
        return false;
    }
    s_result.init_ok = true;

    int64_t t0 = esp_timer_get_time();
    epd_clear();   /* white frame, also wipes panel RAM ghosts */
    s_result.clear_ms = (int)((esp_timer_get_time() - t0) / 1000);
    s_result.clear_engaged = true;
    evlog_record(EV_EPD, EPD_PHASE_CLEAR_DONE, (int16_t)s_result.clear_ms);

    uint8_t *bw = malloc(EPD_BUF_SIZE);
    uint8_t *red = malloc(EPD_BUF_SIZE);
    if (!bw || !red) {
        ESP_LOGE(TAG, "no heap for framebuffers");
        evlog_record(EV_EPD, EPD_PHASE_NO_HEAP, 0);
        free(bw); free(red);
        s_result.running = false;
        return false;
    }

    /* --- Bisect layer 1: solid black frame (memset only, no fill_rect).
     * Visible black => buffer transport + plane encoding are fine. --- */
    memset(bw, 0x00, EPD_BUF_SIZE);
    memset(red, 0x00, EPD_BUF_SIZE);
    ESP_LOGI(TAG, "bisect frame 1: solid black");
    epd_display(bw, red);

    /* --- Bisect layer 2: black bars via fill_rect (no red plane use). --- */
    memset(bw, 0xFF, EPD_BUF_SIZE);   /* all white */
    fill_rect(bw, red, 0, 0, EPD_WIDTH, 40, BLACK);          /* top bar */
    fill_rect(bw, red, 0, EPD_HEIGHT - 40, EPD_WIDTH, 40, BLACK); /* bottom */
    fill_rect(bw, red, 0, 0, 40, EPD_HEIGHT, BLACK);         /* left bar */
    fill_rect(bw, red, EPD_WIDTH - 40, 0, 40, EPD_HEIGHT, BLACK); /* right */
    ESP_LOGI(TAG, "bisect frame 2: black bars via fill_rect");
    epd_display(bw, red);

    /* --- Bisect layer 3: full tri-color pattern (resolution-relative). --- */
    memset(bw, 0xFF, EPD_BUF_SIZE);   /* all white */
    memset(red, 0x00, EPD_BUF_SIZE);
    int cx = EPD_WIDTH / 2, cy = EPD_HEIGHT / 2;

    /* Test pattern: edge bars prove span+orientation, center squares prove
     * both planes and both colors render where expected. */
    fill_rect(bw, red, 0, 0, EPD_WIDTH, 40, BLACK);          /* top bar */
    fill_rect(bw, red, 0, EPD_HEIGHT - 40, EPD_WIDTH, 40, RED); /* bottom */
    fill_rect(bw, red, 0, 0, 40, EPD_HEIGHT, BLACK);         /* left bar */
    fill_rect(bw, red, EPD_WIDTH - 40, 0, 40, EPD_HEIGHT, RED); /* right */
    fill_rect(bw, red, cx - 40, cy - 40, 80, 80, RED);       /* center red */
    fill_rect(bw, red, cx - 50, cy - 50, 100, 4, BLACK);     /* black frame */
    fill_rect(bw, red, cx - 50, cy + 46, 100, 4, BLACK);
    fill_rect(bw, red, cx - 50, cy - 50, 4, 100, BLACK);
    fill_rect(bw, red, cx + 46, cy - 50, 4, 100, BLACK);

    ESP_LOGI(TAG, "displaying test pattern (full refresh)...");
    t0 = esp_timer_get_time();
    epd_display(bw, red);
    s_result.pat_ms = (int)((esp_timer_get_time() - t0) / 1000);
    s_result.pat_engaged = true;
    evlog_record(EV_EPD, EPD_PHASE_PAT_DONE, (int16_t)s_result.pat_ms);
    ESP_LOGI(TAG, "test pattern done");
    free(bw); free(red);
    s_result.running = false;
    return true;
}

static void epd_task(void *arg)
{
    (void)arg;
    evlog_record(EV_EPD, EPD_PHASE_TASK_ALIVE, 0);
    for (;;) {
        epd_selftest();   /* runs once at boot for bring-up */
        /* Park until re-triggered (REST /api/epd/test). M3: status page
         * loop replaces the park. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        evlog_record(EV_EPD, EPD_PHASE_TASK_ALIVE, 1);  /* woken by trigger */
    }
}

void epd_task_start(void)
{
    xTaskCreate(epd_task, "epd", 8192, NULL, 2, &s_epd_task);
}

void epd_task_test_trigger(void)
{
    if (s_epd_task) xTaskNotifyGive(s_epd_task);
}

#else /* !ENABLE_EPD */

void epd_task_start(void) {}
void epd_task_test_trigger(void) {}
const epd_result_t *epd_last_result(void)
{
    static const epd_result_t zero = {0};
    return &zero;
}

#endif
