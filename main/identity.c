/* identity.c — F-1b 脖子 NVS 身份实现（C 版，模式对齐 CAM 仓 identity.cc） */
#include "identity.h"

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "nvs.h"
#include "sdkconfig.h"
#include "mqtt_creds.h"
#include "wifi_creds.h"

static const char *TAG = "identity";

#define NS "identity"
#define K_BROKER "mqtt_broker"   /* NVS key 上限 15 字符 */
#define K_USER   "mqtt_user"
#define K_PASS   "mqtt_pass"
#define K_PREFIX "mqtt_prefix"
#define K_EXPECT "expect_dev"
#define K_W0SSID "wifi0_ssid"
#define K_W0PASS "wifi0_pass"
#define K_W1SSID "wifi1_ssid"
#define K_W1PASS "wifi1_pass"

/* 内存缓存（首次加载后只读） */
static char s_broker[128] = {0};
static char s_user[64]    = {0};
static char s_pass[64]    = {0};
static char s_prefix[64]  = {0};
static char s_expect[24]  = {0};
static char s_wssid[2][40] = {{0}};
static char s_wpass[2][72] = {{0}};
static bool s_loaded = false;

static void nvs_read_str(nvs_handle_t h, const char *key, char *dst, size_t dst_sz)
{
    size_t sz = dst_sz;
    if (nvs_get_str(h, key, dst, &sz) != ESP_OK) dst[0] = '\0';
}

static esp_err_t nvs_write_str(nvs_handle_t h, const char *key, const char *val)
{
    if (!val || !val[0]) return ESP_OK;   /* 空值不写（保留现有） */
    return nvs_set_str(h, key, val);
}

/* 惰性加载：wifi/mqtt 初始化不保证在 identity_init 之后 */
static void ensure_loaded(void)
{
    if (s_loaded) return;
    s_loaded = true;
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "NVS ns '%s' not found — compile-time defaults "
                 "(存量兼容 or 空身份待注入)", NS);
        return;
    }
    nvs_read_str(h, K_BROKER, s_broker, sizeof(s_broker));
    nvs_read_str(h, K_USER,   s_user,   sizeof(s_user));
    nvs_read_str(h, K_PASS,   s_pass,   sizeof(s_pass));
    nvs_read_str(h, K_PREFIX, s_prefix, sizeof(s_prefix));
    nvs_read_str(h, K_EXPECT, s_expect, sizeof(s_expect));
    nvs_read_str(h, K_W0SSID, s_wssid[0], sizeof(s_wssid[0]));
    nvs_read_str(h, K_W0PASS, s_wpass[0], sizeof(s_wpass[0]));
    nvs_read_str(h, K_W1SSID, s_wssid[1], sizeof(s_wssid[1]));
    nvs_read_str(h, K_W1PASS, s_wpass[1], sizeof(s_wpass[1]));
    nvs_close(h);

    ESP_LOGI(TAG, "identity loaded: %s (user=%s prefix=%s wifi0=%s)",
             identity_is_provisioned() ? "PROVISIONED" : "EMPTY",
             s_user[0] ? s_user : "(default)",
             s_prefix[0] ? s_prefix : "(default)",
             s_wssid[0][0] ? s_wssid[0] : WIFI0_SSID);
}

void identity_init(void)
{
    ensure_loaded();
}

const char *identity_mqtt_broker(void)
{
    ensure_loaded();
    return s_broker[0] ? s_broker : MQTT_BROKER_URI;
}
const char *identity_mqtt_user(void)
{
    ensure_loaded();
    return s_user[0] ? s_user : MQTT_USERNAME;
}
const char *identity_mqtt_pass(void)
{
    ensure_loaded();
    return s_pass[0] ? s_pass : MQTT_PASSWORD;
}
const char *identity_mqtt_prefix(void)
{
    ensure_loaded();
    return s_prefix[0] ? s_prefix : MQTT_ROBOT_PREFIX;
}
const char *identity_expect_device_id(void)
{
    ensure_loaded();
    return s_expect[0] ? s_expect : MQTT_EXPECT_DEVICE_ID;
}
const char *identity_wifi_ssid(int slot)
{
    ensure_loaded();
    if (slot == 0) return s_wssid[0][0] ? s_wssid[0] : WIFI0_SSID;
    return s_wssid[1][0] ? s_wssid[1] : WIFI1_SSID;
}
const char *identity_wifi_pass(int slot)
{
    ensure_loaded();
    if (slot == 0) return s_wpass[0][0] ? s_wpass[0] : WIFI0_PASSWORD;
    return s_wpass[1][0] ? s_wpass[1] : WIFI1_PASSWORD;
}

bool identity_is_provisioned(void)
{
    return s_user[0] && s_pass[0] && s_prefix[0] && s_broker[0] && s_expect[0];
}

esp_err_t identity_write_all(const char *broker, const char *user,
                             const char *pass, const char *prefix,
                             const char *expect_device_id,
                             const char *wifi0_ssid, const char *wifi0_pass,
                             const char *wifi1_ssid, const char *wifi1_pass)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open write: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_write_str(h, K_BROKER, broker);
    if (err == ESP_OK) err = nvs_write_str(h, K_USER, user);
    if (err == ESP_OK) err = nvs_write_str(h, K_PASS, pass);
    if (err == ESP_OK) err = nvs_write_str(h, K_PREFIX, prefix);
    if (err == ESP_OK) err = nvs_write_str(h, K_EXPECT, expect_device_id);
    if (err == ESP_OK) err = nvs_write_str(h, K_W0SSID, wifi0_ssid);
    if (err == ESP_OK) err = nvs_write_str(h, K_W0PASS, wifi0_pass);
    if (err == ESP_OK) err = nvs_write_str(h, K_W1SSID, wifi1_ssid);
    if (err == ESP_OK) err = nvs_write_str(h, K_W1PASS, wifi1_pass);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "identity written: user=%s prefix=%s",
                 user ? user : "(keep)", prefix ? prefix : "(keep)");
    } else {
        ESP_LOGE(TAG, "identity write failed: %s", esp_err_to_name(err));
    }
    return err;
}

bool identity_verify(const char *broker, const char *user,
                     const char *pass, const char *prefix,
                     const char *expect_device_id,
                     const char *wifi0_ssid, const char *wifi0_pass,
                     const char *wifi1_ssid, const char *wifi1_pass)
{
    /* 重新从 NVS 读（不用内存缓存，确保落盘验证） */
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) return false;
    char buf[128];
    bool ok = true;

    struct { const char *key; const char *want; } checks[] = {
        {K_BROKER, broker}, {K_USER, user}, {K_PASS, pass},
        {K_PREFIX, prefix}, {K_EXPECT, expect_device_id},
        {K_W0SSID, wifi0_ssid}, {K_W0PASS, wifi0_pass},
        {K_W1SSID, wifi1_ssid}, {K_W1PASS, wifi1_pass},
    };
    for (size_t i = 0; i < sizeof(checks) / sizeof(checks[0]); i++) {
        if (!checks[i].want || !checks[i].want[0]) continue;
        size_t sz = sizeof(buf);
        if (nvs_get_str(h, checks[i].key, buf, &sz) != ESP_OK ||
            strcmp(buf, checks[i].want) != 0) {
            ESP_LOGE(TAG, "verify mismatch on %s: want='%s'",
                     checks[i].key, checks[i].want);
            ok = false;
        }
    }
    nvs_close(h);
    ESP_LOGI(TAG, "identity verify: %s", ok ? "PASS" : "FAIL");
    return ok;
}

const char *identity_summary_json(void)
{
    static char json[512];
    snprintf(json, sizeof(json),
             "{\"provisioned\":%s,"
             "\"broker\":\"%s\",\"user\":\"%s\",\"pass\":\"***\","
             "\"prefix\":\"%s\",\"expect_device_id\":\"%s\","
             "\"wifi0_ssid\":\"%s\",\"wifi1_ssid\":\"%s\"}",
             identity_is_provisioned() ? "true" : "false",
             identity_mqtt_broker(), identity_mqtt_user(),
             identity_mqtt_prefix(), identity_expect_device_id(),
             identity_wifi_ssid(0), identity_wifi_ssid(1));
    return json;
}
