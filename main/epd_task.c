/* epd_task.c — e-ink page owner (M3): lunar calendar ported from ArtInk58.
 * Boot: init panel -> wait for NTP time -> render calendar. Then re-render
 * on date rollover (hourly check); /api/epd/test shows the bench tri-color
 * card, after which the calendar returns. */
#include "epd_task.h"
#include "board_config.h"

#if ENABLE_EPD

#include "epd.h"
#include "epd_calendar.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "evlog.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TAG "EPD"

enum { WHITE = 0, BLACK, RED };

#define NOTIFY_TEST_PATTERN 1u

static TaskHandle_t s_epd_task;
static epd_result_t s_result;   /* latest run snapshot, read by REST */

const epd_result_t *epd_last_result(void) { return &s_result; }

/* Resolution-relative tri-color card: edge bars prove span+orientation,
 * center squares prove both planes and both colors. Bench tool only. */
static void draw_test_pattern(uint8_t *bw, uint8_t *red)
{
    memset(bw, 0xFF, EPD_BUF_SIZE);
    memset(red, 0x00, EPD_BUF_SIZE);
    int cx = EPD_WIDTH / 2, cy = EPD_HEIGHT / 2;
    struct { int x, y, w, h, c; } r[] = {
        {0, 0, EPD_WIDTH, 40, BLACK},
        {0, EPD_HEIGHT - 40, EPD_WIDTH, 40, RED},
        {0, 0, 40, EPD_HEIGHT, BLACK},
        {EPD_WIDTH - 40, 0, 40, EPD_HEIGHT, RED},
        {cx - 40, cy - 40, 80, 80, RED},
    };
    for (unsigned k = 0; k < sizeof(r) / sizeof(r[0]); k++) {
        for (int y = r[k].y; y < r[k].y + r[k].h && y < EPD_HEIGHT; y++) {
            for (int x = r[k].x; x < r[k].x + r[k].w && x < EPD_WIDTH; x++) {
                int idx = y * (EPD_WIDTH / 8) + x / 8;
                uint8_t bit = 1u << (7 - x % 8);
                if (r[k].c == BLACK) { bw[idx] &= ~bit; red[idx] &= ~bit; }
                else                 { bw[idx] &= ~bit; red[idx] |= bit; }
            }
        }
    }
    /* black frame around the red square */
    int fx = cx - 50, fy = cy - 50, fw = 100, fh = 100;
    for (int y = fy; y < fy + fh; y++)
        for (int x = fx; x < fx + fw; x++) {
            bool edge = (x < fx + 4 || x >= fx + fw - 4 ||
                         y < fy + 4 || y >= fy + fh - 4);
            if (!edge) continue;
            int idx = y * (EPD_WIDTH / 8) + x / 8;
            bw[idx] &= ~(1u << (7 - x % 8));
            red[idx] &= ~(1u << (7 - x % 8));
        }
}

static int day_key(void)
{
    time_t now;
    struct tm t;
    time(&now);
    localtime_r(&now, &t);
    return (t.tm_year << 9) | t.tm_yday;
}

static void epd_task(void *arg)
{
    (void)arg;
    evlog_record(EV_EPD, EPD_PHASE_TASK_ALIVE, 0);
    s_result.run_id++;
    s_result.running = true;
    s_result.started_uptime_ms = (int32_t)(esp_timer_get_time() / 1000);

    if (!epd_init()) {
        ESP_LOGE(TAG, "panel not responding — EPD disabled until reboot");
        s_result.running = false;
        vTaskDelete(NULL);
        return;
    }
    s_result.init_ok = true;

    uint8_t *bw = malloc(EPD_BUF_SIZE);
    uint8_t *red = malloc(EPD_BUF_SIZE);
    if (!bw || !red) {
        ESP_LOGE(TAG, "no heap for framebuffers");
        evlog_record(EV_EPD, EPD_PHASE_NO_HEAP, 0);
        free(bw); free(red);
        s_result.running = false;
        vTaskDelete(NULL);
        return;
    }

    /* Wait for NTP so the calendar page shows the right day (the panel
     * keeps its old image meanwhile — e-ink's killer feature). */
    for (int i = 0; i < 18 && !epd_calendar_time_ready(); i++) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    int shown_day = -1;
    for (;;) {
        int64_t t0 = esp_timer_get_time();
        epd_calendar_render(bw, red);
        epd_display(bw, red);
        s_result.clear_ms = (int)((esp_timer_get_time() - t0) / 1000);
        evlog_record(EV_EPD, EPD_PHASE_CLEAR_DONE, (int16_t)s_result.clear_ms);
        ESP_LOGI(TAG, "calendar rendered (%d ms)", s_result.clear_ms);
        shown_day = day_key();
        s_result.running = false;

        /* Park up to 1 h: re-render on date rollover or notify. */
        for (;;) {
            uint32_t val = 0;
            if (xTaskNotifyWait(0, 0xFFFFFFFF, &val,
                                pdMS_TO_TICKS(3600 * 1000)) == pdTRUE) {
                s_result.running = true;
                if (val == NOTIFY_TEST_PATTERN) {
                    t0 = esp_timer_get_time();
                    draw_test_pattern(bw, red);
                    epd_display(bw, red);
                    s_result.pat_ms = (int)((esp_timer_get_time() - t0) / 1000);
                    evlog_record(EV_EPD, EPD_PHASE_PAT_DONE,
                                 (int16_t)s_result.pat_ms);
                }
                break;   /* calendar re-renders next outer iteration */
            }
            if (day_key() != shown_day) {
                ESP_LOGI(TAG, "date rolled over — re-rendering calendar");
                s_result.running = true;
                break;
            }
        }
    }
}

void epd_task_start(void)
{
    xTaskCreate(epd_task, "epd", 8192, NULL, 2, &s_epd_task);
}

void epd_task_test_trigger(void)
{
    if (s_epd_task) xTaskNotify(s_epd_task, NOTIFY_TEST_PATTERN,
                                eSetValueWithOverwrite);
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
