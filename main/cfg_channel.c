#include "cfg_channel.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

#include "mqtt_publisher.h"
#include "zone_tracker.h"
#include "mode_manager.h"

static const char *TAG = "cfg";

/* 轻量 JSON 字段提取（与 rest_api.c 同款手写风格，避免引 cJSON）。 */
static bool jget_str(const char *body, const char *key, char *out, size_t n)
{
    char pat[40];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(body, pat);
    if (!p) return false;
    p += strlen(pat);
    while (*p && (*p == ':' || *p == ' ')) p++;
    if (*p == '"') {
        p++;
        size_t i = 0;
        while (*p && *p != '"' && i < n - 1) out[i++] = *p++;
        out[i] = '\0';
        return true;
    }
    /* 裸值（数字/bool）——拷到 , } 或空白为止（rest_api.c 同款）。 */
    if (*p && *p != ',' && *p != '}') {
        size_t i = 0;
        while (*p && *p != ',' && *p != '}' && *p != ' ' && i < n - 1)
            out[i++] = *p++;
        out[i] = '\0';
        return true;
    }
    return false;
}

static bool jget_int(const char *body, const char *key, int *out)
{
    char b[16];
    if (!jget_str(body, key, b, sizeof(b))) return false;
    *out = atoi(b);
    return true;
}

static void cfg_handler(const char *payload, int len)
{
    (void)len;
    ESP_LOGI(TAG, "cfg_handler raw: %s", payload);
    char req_id[40] = {0};
    jget_str(payload, "req_id", req_id, sizeof(req_id));

    char applied[160] = "";
    char rejected[96] = "";
    size_t alen = 0, rlen = 0;
    int tmp;

    /* zone 参数（合并当前值再校验落盘，与 REST /api/radar 同路径） */
    zone_cfg_t z = *zone_cfg();
    bool zone_touched = false;
    if (jget_int(payload, "zone_enter_mm", &tmp)) { z.enter_mm = tmp; zone_touched = true; }
    if (jget_int(payload, "zone_debounce", &tmp)) { z.debounce = tmp; zone_touched = true; }
    if (jget_int(payload, "zone_hyst_mm", &tmp)) { z.hyst_mm = tmp; zone_touched = true; }
    if (jget_int(payload, "zone_leave", &tmp)) { z.leave = tmp; zone_touched = true; }
    if (jget_int(payload, "zone_lost_ms", &tmp)) { z.lost_ms = tmp; zone_touched = true; }
    if (zone_touched) {
        if (zone_cfg_set(&z)) {
            alen += snprintf(applied + alen, sizeof(applied) - alen,
                             "%s\"zone\"", alen ? "," : "");
        } else {
            rlen += snprintf(rejected + rlen, sizeof(rejected) - rlen,
                             "%s\"zone(out_of_range)\"", rlen ? "," : "");
        }
    }

    if (jget_int(payload, "still_min", &tmp)) {
        mode_manager_set_still_min((uint16_t)tmp);
        alen += snprintf(applied + alen, sizeof(applied) - alen,
                         "%s\"still_min\"", alen ? "," : "");
    }
    if (jget_int(payload, "fall_detect", &tmp)) {
        mode_manager_set_fall_detect(tmp != 0);
        alen += snprintf(applied + alen, sizeof(applied) - alen,
                         "%s\"fall_detect\"", alen ? "," : "");
    }
    if (jget_int(payload, "assoc_gate", &tmp)) {
        mode_manager_set_assoc_gate(tmp != 0);
        alen += snprintf(applied + alen, sizeof(applied) - alen,
                         "%s\"assoc_gate\"", alen ? "," : "");
    }

    /* 回执（guardian 对账锚点；req_id 原样透传） */
    char topic[80];
    char receipt[384];
    snprintf(topic, sizeof(topic), "%s/status/cfg", mqtt_publisher_prefix());
    snprintf(receipt, sizeof(receipt),
             "{\"req_id\":\"%s\",\"applied\":[%s],\"rejected\":[%s]}",
             req_id, applied, rejected);
    bool ok = mqtt_publisher_publish_raw(topic, receipt, 1, 0);
    ESP_LOGI(TAG, "cfg_set req=%s applied=[%s] rejected=[%s] receipt=%s",
             req_id, applied, rejected, ok ? "sent" : "FAIL(not connected)");
}

void cfg_channel_init(void)
{
    mqtt_publisher_set_cfg_handler(cfg_handler);
    ESP_LOGI(TAG, "cmd/cfg_set channel armed");
}
