/* radar_turn.c — 97（docs/97 v1.1）：雷达直驱转向（脖子本地反射）。
 *
 * 转头是反射、开口才是决定：zone_enter 时脖子自检门控后立刻切 TRACK/FUSION
 * 看人（不等 CAM 经 broker 批准），hold 内 CAM 会话 start 接管则转为会话期
 * （96 机制不变）；无人应答 hold 到期归 6 点回 COMMAND。
 *
 * 门控（97a H-1 白名单）：mode==COMMAND && src ∈ {NONE, SESSION}；
 *   MANUAL/MQTT 拒绝（人工锁定/巡检窗）。夜间窗 + cooldown + 总开关。
 * 参数来源=cmd/wake_cfg（guardian 双发到脖子前缀），NVS "wcfg"。
 * 归位路径（zone_leave / hold 到期）均判 src==RADAR（97a M-5）。
 */
#include "radar_turn.h"

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "nvs.h"
#include "servo.h"
#include "mode_manager.h"
#include "evlog.h"
#include "mqtt_publisher.h"

static const char *TAG = "radar_turn";

#define WCFG_NS "wcfg"

/* 默认=97 §2.2 */
static radar_turn_cfg_t s_cfg = {
    .enabled = 1, .hold_s = 10, .cooldown_s = 30,
    .night_start = 22, .night_end = 7,
};
static int64_t  s_hold_deadline_us = 0;    /* 0 = 无 hold 进行中 */
static int64_t  s_last_enter_us = 0;       /* cooldown 起算（首次 enter） */
static bool     s_time_synced = false;

void radar_turn_set_time_synced(void) { s_time_synced = true; }

static void load_cfg(void)
{
    nvs_handle_t h;
    if (nvs_open(WCFG_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t u;
    uint16_t v;
    if (nvs_get_u8(h, "rt_en", &u) == ESP_OK) s_cfg.enabled = u != 0;
    if (nvs_get_u16(h, "rt_hold", &v) == ESP_OK && v >= 3 && v <= 60) s_cfg.hold_s = v;
    if (nvs_get_u16(h, "rt_cool", &v) == ESP_OK && v <= 600) s_cfg.cooldown_s = v;
    if (nvs_get_u8(h, "night_s", &u) == ESP_OK && u <= 23) s_cfg.night_start = u;
    if (nvs_get_u8(h, "night_e", &u) == ESP_OK && u <= 23) s_cfg.night_end = u;
    nvs_close(h);
    ESP_LOGI(TAG, "cfg: enabled=%d hold=%ds cooldown=%ds night=%u-%u",
             s_cfg.enabled, s_cfg.hold_s, s_cfg.cooldown_s,
             s_cfg.night_start, s_cfg.night_end);
}

/* cmd/wake_cfg 载荷子集（97 §2.2 新字段；guardian 双发）。只写出现的键。 */
void radar_turn_apply_cfg(const char *body,
                          bool (*get_int)(const char *, const char *, int *))
{
    nvs_handle_t h;
    if (nvs_open(WCFG_NS, NVS_READWRITE, &h) != ESP_OK) return;
    int v;
    if (get_int(body, "radar_turn_enabled", &v)) {
        s_cfg.enabled = v != 0; nvs_set_u8(h, "rt_en", s_cfg.enabled);
    }
    if (get_int(body, "radar_turn_hold_s", &v) && v >= 3 && v <= 60) {
        s_cfg.hold_s = v; nvs_set_u16(h, "rt_hold", (uint16_t)v);
    }
    if (get_int(body, "radar_turn_cooldown_s", &v) && v >= 0 && v <= 600) {
        s_cfg.cooldown_s = v; nvs_set_u16(h, "rt_cool", (uint16_t)v);
    }
    if (get_int(body, "night_start", &v) && v >= 0 && v <= 23) {
        s_cfg.night_start = (uint8_t)v; nvs_set_u8(h, "night_s", (uint8_t)v);
    }
    if (get_int(body, "night_end", &v) && v >= 0 && v <= 23) {
        s_cfg.night_end = (uint8_t)v; nvs_set_u8(h, "night_e", (uint8_t)v);
    }
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "cfg applied: enabled=%d hold=%ds cooldown=%ds night=%u-%u",
             s_cfg.enabled, s_cfg.hold_s, s_cfg.cooldown_s,
             s_cfg.night_start, s_cfg.night_end);
}

void radar_turn_cfg_json(char *out, size_t sz)
{
    snprintf(out, sz,
             "\"radar_turn_enabled\":%d,\"radar_turn_hold_s\":%d,"
             "\"radar_turn_cooldown_s\":%d,\"night_start\":%u,\"night_end\":%u",
             s_cfg.enabled, s_cfg.hold_s, s_cfg.cooldown_s,
             s_cfg.night_start, s_cfg.night_end);
}

/* 97a L-3：SNTP 未同步=按非夜间处理（避免 1970 误判） */
static bool in_night_window(void)
{
    if (!s_time_synced || s_cfg.night_start == s_cfg.night_end) return false;
    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    int h = tm_now.tm_hour;
    if (s_cfg.night_start > s_cfg.night_end)        /* 跨午夜 22-7 */
        return h >= s_cfg.night_start || h < s_cfg.night_end;
    return h >= s_cfg.night_start && h < s_cfg.night_end;
}

static bool hold_active(void)
{
    return s_hold_deadline_us > 0;
}

/* 归位：归 0° + 回 COMMAND + src=NONE。只在本模块持有（src==RADAR）时执行。 */
static void radar_home(void)
{
    s_hold_deadline_us = 0;
    if (mode_manager_oneshot_src() != ONESHOT_RADAR) return;   /* 97a M-5 */
    servo_set_angle_deg(0.0f);
    mode_manager_set_oneshot_src(ONESHOT_NONE);
    mode_manager_set(MODE_COMMAND, 0);   /* 永久指令态（裁决 B/C） */
    evlog_record(EV_SERVO_CMD, SRC_RADAR, 0);
    ESP_LOGI(TAG, "home → COMMAND");
}

void radar_turn_init(void)
{
    setenv("TZ", "CST-8", 1);
    tzset();
    load_cfg();
}

/* zone_enter 边沿（radar.c 调用，MQTT 发布照常不受影响）。
 * 返回 true=已切 TRACK（本地反射生效）。 */
bool radar_turn_on_enter(void)
{
    int64_t now = esp_timer_get_time();
    if (hold_active()) {
        ESP_LOGI(TAG, "enter during hold: ignore (no hold reset)");
        return false;
    }
    oneshot_src_t src = mode_manager_oneshot_src();
    if (!s_cfg.enabled) { evlog_record(EV_CMD_REJECT, SRC_RADAR, 1); return false; }
    if (mode_manager_get() != MODE_COMMAND ||
        (src != ONESHOT_NONE && src != ONESHOT_SESSION)) {
        evlog_record(EV_CMD_REJECT, SRC_RADAR, 2);   /* 人工/巡检/会话中 */
        return false;
    }
    if (in_night_window()) { evlog_record(EV_CMD_REJECT, SRC_RADAR, 3); return false; }
    if (s_last_enter_us > 0 &&
        (now - s_last_enter_us) < (int64_t)s_cfg.cooldown_s * 1000000LL) {
        ESP_LOGI(TAG, "in cooldown, ignored");
        return false;
    }
    s_last_enter_us = now;
    /* 97a M-1：先 TRACK 再 FUSION（反序被 restore_submode 还原） */
    mode_manager_snapshot_submode();
    mode_manager_set_oneshot_src(ONESHOT_RADAR);
    mode_manager_set(MODE_TRACK, 0);
    mode_manager_set_submode(TRACK_FUSION);
    s_hold_deadline_us = now + (int64_t)s_cfg.hold_s * 1000000LL;
    evlog_record(EV_MODE_CHG, SRC_RADAR, (int16_t)TRACK_FUSION);
    ESP_LOGI(TAG, "enter → TRACK/FUSION (hold %ds)", s_cfg.hold_s);
    return true;
}

/* zone_leave 边沿：仅本模块持有时归位（start 已接管则忽略）。 */
void radar_turn_on_leave(void)
{
    if (mode_manager_oneshot_src() == ONESHOT_RADAR && hold_active()) {
        ESP_LOGI(TAG, "leave during hold → home");
        radar_home();
    }
}

/* cmd/wake_cfg 下行（guardian 双发到脖子前缀，97 §2.2）：只取本模块
 * 5 字段，其余（absence/cooldown/grace 是 CAM 的）忽略；回执 status/wake
 * 与 CAM 对称（E-2），KinCal "已存未生效"语义一致。 */
void radar_turn_on_wake_cfg(const char *body)
{
    extern bool json_get_int(const char *, const char *, int *);
    radar_turn_apply_cfg(body, json_get_int);
    char topic[80], receipt[256], cfgj[192];
    radar_turn_cfg_json(cfgj, sizeof(cfgj));
    snprintf(topic, sizeof(topic), "%s/status/wake", mqtt_publisher_prefix());
    snprintf(receipt, sizeof(receipt), "{%s}", cfgj);
    bool ok = mqtt_publisher_publish_raw(topic, receipt, 1, 0);
    ESP_LOGI(TAG, "wake_cfg receipt %s: %s", ok ? "sent" : "FAIL", receipt);
}

/* 周期 tick（radar/main 任务每 ~500ms）：hold 到期归位。 */
void radar_turn_tick(void)
{
    if (!hold_active()) return;
    if (mode_manager_oneshot_src() != ONESHOT_RADAR) {
        s_hold_deadline_us = 0;            /* start 已接管，撤 hold */
        return;
    }
    if (esp_timer_get_time() >= s_hold_deadline_us) {
        ESP_LOGI(TAG, "hold expired → home");
        radar_home();
    }
}
