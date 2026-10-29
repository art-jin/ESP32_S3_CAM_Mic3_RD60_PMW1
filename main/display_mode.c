/* display_mode.c — docs/74 M1 显示模式注册表实现。
 * 状态只有一份(互斥锁保护),REST /api/display 与 MQTT cmd/display 都汇
 * 到 display_mode_show();EPD task 用 (mode,seq) 检测变更,TTL 到期由
 * display_mode_active() 的询问方自行发现(EPD park 超时取 min(ttl,1h))。 */
#include "display_mode.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"

#include "mqtt_publisher.h"

#define TAG "DMODE"
#define NOTIFY_SHOW 2u

static const char *k_names[DMODE_COUNT] = {
    "calendar", "calendar_overlay", "weather", "todo",
    "status", "medgrid", "msg",
};

static SemaphoreHandle_t s_lock;
static int       s_mode = DMODE_CALENDAR;
static int64_t   s_expires_ms = -1;      /* -1 = 默认页/无 TTL */
static char      s_payload[DISPLAY_PAYLOAD_MAX] = "{}";
static uint32_t  s_seq = 0;
static TaskHandle_t s_epd_task;

int display_mode_from_name(const char *name)
{
    if (!name) return -1;
    for (int i = 0; i < DMODE_COUNT; i++)
        if (strcmp(name, k_names[i]) == 0) return i;
    return -1;
}

const char *display_mode_name(int mode)
{
    return (mode >= 0 && mode < DMODE_COUNT) ? k_names[mode] : "?";
}

static void lock_init(void)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
}

bool display_mode_show(int mode, const char *payload_json, int ttl_sec)
{
    if (mode < 0 || mode >= DMODE_COUNT) return false;
    lock_init();
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) return false;

    s_mode = mode;
    if (payload_json && payload_json[0]) {
        strncpy(s_payload, payload_json, DISPLAY_PAYLOAD_MAX - 1);
        s_payload[DISPLAY_PAYLOAD_MAX - 1] = '\0';
    } else {
        strcpy(s_payload, "{}");
    }
    if (mode == DMODE_CALENDAR || mode == DMODE_CALENDAR_OVERLAY) {
        s_expires_ms = -1;
    } else {
        if (ttl_sec <= 0) ttl_sec = DISPLAY_TTL_DEFAULT_SEC;
        s_expires_ms = esp_timer_get_time() / 1000 + (int64_t)ttl_sec * 1000;
    }
    s_seq++;

    TaskHandle_t task = s_epd_task;
    xSemaphoreGive(s_lock);

    if (task) xTaskNotify(task, NOTIFY_SHOW, eSetValueWithOverwrite);
    ESP_LOGI(TAG, "show %s ttl=%ds seq=%lu",
             k_names[mode],
             (s_expires_ms < 0) ? -1 : (int)((s_expires_ms - esp_timer_get_time() / 1000) / 1000),
             (unsigned long)s_seq);
    return true;
}

int display_mode_active(void)
{
    lock_init();
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) return DMODE_CALENDAR;
    int mode = s_mode;
    if (s_expires_ms >= 0 && esp_timer_get_time() / 1000 >= s_expires_ms) {
        mode = DMODE_CALENDAR;   /* TTL 到期 — 回退页语义(revert 恒 calendar) */
    }
    xSemaphoreGive(s_lock);
    return mode;
}

const char *display_mode_payload(void)
{
    lock_init();
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) return "{}";
    /* 载荷属于最近一次 show;TTL 回退后 active()=CALENDAR,其渲染器
     * (及 overlay 的本地月历部分)不读临时页字段,过期载荷无害。 */
    const char *p = s_payload;
    xSemaphoreGive(s_lock);
    return p;
}

uint32_t display_mode_seq(void)
{
    lock_init();
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) return 0;
    uint32_t seq = s_seq;
    xSemaphoreGive(s_lock);
    return seq;
}

int64_t display_mode_ttl_remaining_ms(void)
{
    lock_init();
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) return -1;
    int64_t r = -1;
    if (s_expires_ms >= 0)
        r = s_expires_ms - esp_timer_get_time() / 1000;
    xSemaphoreGive(s_lock);
    return r;
}

void display_mode_set_notify_task(void *task_handle)
{
    s_epd_task = (TaskHandle_t)task_handle;
}

/* ---- MQTT cmd/display 下行 ---- */

static void on_mqtt_display(const char *body, int len)
{
    /* 载荷与 REST /api/display 同构(docs/74 §4) */
    char *buf = malloc(len + 1);
    if (!buf) return;
    memcpy(buf, body, len);
    buf[len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        ESP_LOGW(TAG, "cmd/display: bad json (%d bytes)", len);
        free(buf);
        return;
    }
    const char *mode = NULL;
    int ttl = 0;
    cJSON *m = cJSON_GetObjectItem(root, "mode");
    if (cJSON_IsString(m)) mode = m->valuestring;
    cJSON *t = cJSON_GetObjectItem(root, "ttl_sec");
    if (cJSON_IsNumber(t)) ttl = t->valueint;

    int id = display_mode_from_name(mode);
    if (id < 0) {
        ESP_LOGW(TAG, "cmd/display: unknown mode '%s'", mode ? mode : "(null)");
    } else {
        display_mode_show(id, buf, ttl);
    }
    cJSON_Delete(root);
    free(buf);
}

void display_mode_mqtt_init(void)
{
    mqtt_publisher_set_display_handler(on_mqtt_display);
    ESP_LOGI(TAG, "cmd/display channel armed");
}
