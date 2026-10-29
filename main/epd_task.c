/* epd_task.c — e-ink page owner.74-M1:模式框架 — 默认月历,display_mode
 * (REST /api/display | MQTT cmd/display)可切临时页,TTL 到期回月历;
 * /api/epd/test 调试三色卡不变。 */
#include "epd_task.h"
#include "board_config.h"

#if ENABLE_EPD

#include "display_mode.h"
#include "epd.h"
#include "epd_calendar.h"
#include "epd_pages.h"
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

/* 用户裁决(10-29):切换需等待 — 两次全刷之间的最小间隔。该面板有
 * BUSY 卡死前科(M2 "panel died" + 今日 120s timeout 实录),连刷会把它
 * 推进卡死态;新请求在间隔内先排队(到点重查,新 seq 自然优先)。 */
#define EPD_MIN_REFRESH_GAP_MS 30000

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

    display_mode_set_notify_task(xTaskGetCurrentTaskHandle());

    int shown_day = -1;
    int last_mode = -1;
    uint32_t last_seq = 0;
    int64_t last_refresh_ms = -EPD_MIN_REFRESH_GAP_MS;  /* 首刷不等待 */
    for (;;) {
        /* 74-M1:渲染生效模式(月历=TTL 回退目标;临时页到期自动回)。 */
        int mode = display_mode_active();
        uint32_t seq = display_mode_seq();
        if (mode != last_mode || seq != last_seq ||
            (mode == DMODE_CALENDAR && day_key() != shown_day)) {
            /* 切换等待:距上刷不足间隔 → 先睡差值,醒来重查(新请求
             * 自然以最新 seq 胜出,不排队旧页)。 */
            int64_t now_ms = esp_timer_get_time() / 1000;
            int64_t since = now_ms - last_refresh_ms;
            if (since < EPD_MIN_REFRESH_GAP_MS) {
                s_result.running = false;
                vTaskDelay(pdMS_TO_TICKS(EPD_MIN_REFRESH_GAP_MS - since));
                continue;
            }
            int64_t t0 = esp_timer_get_time();
            epd_pages_render(mode, display_mode_payload(), bw, red);
            epd_display(bw, red);
            display_mode_rendered(mode);   /* TTL 自上屏完成起算 */
            s_result.clear_ms = (int)((esp_timer_get_time() - t0) / 1000);
            last_refresh_ms = t0 / 1000;
            evlog_record(EV_EPD, EPD_PHASE_CLEAR_DONE, (int16_t)s_result.clear_ms);
            ESP_LOGI(TAG, "page %s rendered (%d ms)",
                     display_mode_name(mode), s_result.clear_ms);
            last_mode = mode;
            last_seq = seq;
            shown_day = day_key();
        }
        s_result.running = false;

        /* Park:min(TTL 剩余, 1h) — 到点重查(过期回退/日期翻转/通知)。 */
        TickType_t wait = pdMS_TO_TICKS(3600 * 1000);
        int64_t remain = display_mode_ttl_remaining_ms();
        if (remain >= 0 && remain < 3600 * 1000)
            wait = pdMS_TO_TICKS((remain > 0 ? remain : 0) + 100);

        uint32_t val = 0;
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &val, wait) == pdTRUE &&
            val == NOTIFY_TEST_PATTERN) {
            s_result.running = true;
            int64_t t0 = esp_timer_get_time();
            draw_test_pattern(bw, red);
            epd_display(bw, red);
            s_result.pat_ms = (int)((esp_timer_get_time() - t0) / 1000);
            evlog_record(EV_EPD, EPD_PHASE_PAT_DONE, (int16_t)s_result.pat_ms);
            last_mode = -1;   /* 强制下一轮重渲染生效页 */
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
