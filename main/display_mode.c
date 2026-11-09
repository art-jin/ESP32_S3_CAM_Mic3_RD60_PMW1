/* display_mode.c — docs/74 M1 显示模式注册表实现。
 * 状态只有一份(互斥锁保护),REST /api/display 与 MQTT cmd/display 都汇
 * 到 display_mode_show();EPD task 用 (mode,seq) 检测变更,TTL 到期由
 * display_mode_active() 的询问方自行发现(EPD park 超时取 min(ttl,1h))。
 * docs/74 M2：最近一次 calendar_overlay 载荷暂存于 s_overlay_json，临时
 * 页 TTL 到期回退时套用 → 月历常驻主页始终带节假日/待办框线。
 * R2 屏持久化(78 §4.4,2026-11-09)：当前页+overlay 暂存落 NVS(ns "dmode")，
 * 重启后 display_mode_replay_nvs() 恢复；overlay 按天戳记沿用原日戳——
 * 隔天不复活(回退纯月历,等云补推)。写入在锁内(载荷推送低频,EPD 渲染
 * 本身百秒级,毫秒级 flash 写不构成争用)。 */
#include "display_mode.h"

#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "nvs.h"

#include "mqtt_publisher.h"

#define TAG "DMODE"
#define NOTIFY_SHOW 2u

static const char *k_names[DMODE_COUNT] = {
    "calendar", "calendar_overlay", "weather", "todo",
    "status", "medgrid", "msg", "month_view",
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

/* ---- R2 NVS 持久化 ---- */

static void nvs_save_locked(int mode, const char *payload)
{
    nvs_handle_t h;
    if (nvs_open("dmode", NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "nvs open failed (skip persist)");
        return;
    }
    bool ok = true;
    if (nvs_set_u8(h, "mode", (uint8_t)mode) != ESP_OK) ok = false;
    if (nvs_set_blob(h, "json", payload, strlen(payload) + 1) != ESP_OK)
        ok = false;
    if (s_overlay_json[0]) {
        if (nvs_set_blob(h, "ovl", s_overlay_json,
                         strlen(s_overlay_json) + 1) != ESP_OK) ok = false;
        if (nvs_set_i32(h, "ovl_day", s_overlay_day) != ESP_OK) ok = false;
    }
    if (nvs_commit(h) != ESP_OK) ok = false;
    nvs_close(h);
    if (!ok) ESP_LOGW(TAG, "nvs persist partial failure");
}

/* 开机重放(EPD task 注册 notify 后、首渲染前调用):
 * - 恢复 overlay 暂存但保留原日戳——隔天不复活,诚实过期;
 * - 当前页恢复:calendar_overlay 跨日不重放(回退纯月历);临时页恢复后
 *   TTL 重新起算,到期自然回退。 */
void display_mode_replay_nvs(void)
{
    lock_init();
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) return;

    nvs_handle_t h;
    if (nvs_open("dmode", NVS_READONLY, &h) != ESP_OK) {
        xSemaphoreGive(s_lock);
        return;                       /* 无缓存=首次烧录,正常走默认月历 */
    }
    uint8_t mode = DMODE_CALENDAR;
    size_t len = 0;
    char *json = NULL, *ovl = NULL;
    int32_t ovl_day = -1;
    bool have = nvs_get_u8(h, "mode", &mode) == ESP_OK;
    if (have) {
        if (nvs_get_blob(h, "json", NULL, &len) == ESP_OK && len > 1 &&
            len <= DISPLAY_PAYLOAD_MAX) {
            json = malloc(len);
            if (json && nvs_get_blob(h, "json", json, &len) == ESP_OK &&
                json[len - 1] != '\0') json[len - 1] = '\0';
        }
        size_t olen = 0;
        if (nvs_get_blob(h, "ovl", NULL, &olen) == ESP_OK && olen > 1 &&
            olen <= DISPLAY_PAYLOAD_MAX) {
            ovl = malloc(olen);
            if (ovl && nvs_get_blob(h, "ovl", ovl, &olen) == ESP_OK &&
                ovl[olen - 1] != '\0') ovl[olen - 1] = '\0';
            nvs_get_i32(h, "ovl_day", &ovl_day);
        }
    }
    nvs_close(h);

    if (!json || !json[0]) { free(json); free(ovl); xSemaphoreGive(s_lock); return; }
    if (mode >= DMODE_COUNT) { free(json); free(ovl); xSemaphoreGive(s_lock); return; }

    /* 先恢复暂存(原日戳) */
    if (ovl && ovl[0]) {
        strncpy(s_overlay_json, ovl, DISPLAY_PAYLOAD_MAX - 1);
        s_overlay_json[DISPLAY_PAYLOAD_MAX - 1] = '\0';
        s_overlay_day = (int)ovl_day;
    }
    free(ovl);

    bool replay_page = true;
    if (mode == DMODE_CALENDAR_OVERLAY && s_overlay_day != today_key())
        replay_page = false;          /* 跨日:让 active() 回退纯月历 */

    if (replay_page) {
        strncpy(s_payload, json, DISPLAY_PAYLOAD_MAX - 1);
        s_payload[DISPLAY_PAYLOAD_MAX - 1] = '\0';
        s_mode = (int)mode;
        if (mode == DMODE_CALENDAR || mode == DMODE_CALENDAR_OVERLAY) {
            s_expires_ms = -1;
            s_ttl_req_sec = DISPLAY_TTL_DEFAULT_SEC;
        } else {
            s_ttl_req_sec = DISPLAY_TTL_MIN_SEC;   /* 原 ttl 不落盘,按最小 */
            s_expires_ms = TTL_PENDING;
        }
        /* 暂存保持上面恢复的原日戳(勿走 stash_overlay_from 重打今日戳) */
        s_seq++;
        ESP_LOGI(TAG, "replay from nvs: %s (overlay_day=%d)",
                 k_names[s_mode], s_overlay_day);
    } else {
        ESP_LOGI(TAG, "replay: overlay stale (day %d != %d) -> plain calendar",
                 s_overlay_day, today_key());
    }
    free(json);
    xSemaphoreGive(s_lock);
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
    nvs_save_locked(mode, s_payload);   /* R2:当前页+overlay 暂存落 NVS */

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
