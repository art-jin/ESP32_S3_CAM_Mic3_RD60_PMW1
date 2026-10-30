/* display_mode.c — docs/74 M1 显示模式注册表实现。
 * 状态只有一份(互斥锁保护),REST /api/display 与 MQTT cmd/display 都汇
 * 到 display_mode_show();EPD task 用 (mode,seq) 检测变更,TTL 到期由
 * display_mode_active() 的询问方自行发现(EPD park 超时取 min(ttl,1h))。
 * docs/74 M2：最近一次 calendar_overlay 载荷暂存于 s_overlay_json，临时
 * 页 TTL 到期回退时套用 → 月历常驻主页始终带节假日/待办框线(重启丢失，
 * 等 guardian 次推恢复；NVS 持久化留 M1.5)。 */
#include "display_mode.h"

#include <string.h>
#include <time.h>

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
static int64_t   s_expires_ms = -1;      /* -1 = 无 TTL;PENDING = 待上屏 */
static int       s_ttl_req_sec = DISPLAY_TTL_DEFAULT_SEC;
static char      s_payload[DISPLAY_PAYLOAD_MAX] = "{}";
static char      s_overlay_json[DISPLAY_PAYLOAD_MAX];  /* 归一化 {"overlay":…} */
static int       s_overlay_day = -1;   /* 暂存日戳 yyyymmdd;-1=无;隔天失效(用户裁决 10-30) */
static uint32_t  s_seq = 0;
static TaskHandle_t s_epd_task;

#define TTL_PENDING INT64_MAX

static int today_key(void)
{
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    return (tm.tm_year + 1900) * 10000 + (tm.tm_mon + 1) * 100 + tm.tm_mday;
}

/* 从任意载荷提取 "overlay" 对象归一化暂存(用户裁决 10-30):
 * - calendar_overlay 页:载荷本体即含 overlay;
 * - 待办页等临时页:guardian 以旁带字段随页下发最新 overlay,收到即无感
 *   更新暂存,不切页——回退月历立即反映新代办框线。
 * 暂存带日戳,隔天失效(guardian 每晨 07:10 与每次待办意图都会重推)。 */
static void stash_overlay_from(const char *payload_json)
{
    if (!payload_json || !strstr(payload_json, "\"overlay\""))
        return;
    cJSON *root = cJSON_Parse(payload_json);
    if (!root) return;
    cJSON *ov = cJSON_GetObjectItem(root, "overlay");
    if (cJSON_IsObject(ov)) {
        char *s = cJSON_PrintUnformatted(ov);
        if (s) {
            int rc = snprintf(s_overlay_json, DISPLAY_PAYLOAD_MAX,
                              "{\"overlay\":%s}", s);
            if (rc < 0) s_overlay_json[0] = '\0';
            s_overlay_day = today_key();
            cJSON_free(s);
        }
    }
    cJSON_Delete(root);
}

static bool overlay_stash_valid(void)
{
    return s_overlay_json[0] != '\0' && s_overlay_day == today_key();
}

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
    stash_overlay_from(s_payload);
    if (mode == DMODE_CALENDAR || mode == DMODE_CALENDAR_OVERLAY) {
        s_expires_ms = -1;
        s_ttl_req_sec = DISPLAY_TTL_DEFAULT_SEC;
    } else {
        if (ttl_sec < DISPLAY_TTL_MIN_SEC) ttl_sec = DISPLAY_TTL_MIN_SEC;
        s_ttl_req_sec = ttl_sec;
        s_expires_ms = TTL_PENDING;   /* 上屏完成(display_mode_rendered)起算 */
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

void display_mode_rendered(int mode)
{
    lock_init();
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) return;
    if (mode == s_mode && s_expires_ms == TTL_PENDING) {
        s_expires_ms = esp_timer_get_time() / 1000 +
                       (int64_t)s_ttl_req_sec * 1000;
    }
    xSemaphoreGive(s_lock);
}

int display_mode_active(void)
{
    lock_init();
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) return DMODE_CALENDAR;
    int mode = s_mode;
    if (s_expires_ms != TTL_PENDING && s_expires_ms >= 0 &&
        esp_timer_get_time() / 1000 >= s_expires_ms) {
        /* TTL 到期 — 回退页语义：当日 overlay 暂存有效 → 带框线月历
         * (M2 常驻主页),否则纯月历。 */
        mode = overlay_stash_valid() ? DMODE_CALENDAR_OVERLAY : DMODE_CALENDAR;
    }
    xSemaphoreGive(s_lock);
    return mode;
}

const char *display_mode_payload(void)
{
    lock_init();
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) return "{}";
    /* 载荷属于最近一次 show；TTL 回退后的 active()=CALENDAR_OVERLAY 时，
     * s_payload 已是过期临时页字段，须改供当日 overlay 暂存。 */
    const char *p = s_payload;
    if (s_mode != DMODE_CALENDAR_OVERLAY &&
        s_expires_ms != TTL_PENDING && s_expires_ms >= 0 &&
        esp_timer_get_time() / 1000 >= s_expires_ms && overlay_stash_valid()) {
        p = s_overlay_json;
    }
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
    if (s_expires_ms == TTL_PENDING) {
        r = (int64_t)s_ttl_req_sec * 1000;   /* 未上屏 — 报请求值 */
    } else if (s_expires_ms >= 0) {
        r = s_expires_ms - esp_timer_get_time() / 1000;
    }
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
