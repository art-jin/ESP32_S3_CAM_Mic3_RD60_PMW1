/* ota_neck.c — 脖子板 OTA（docs/49 §12.6 设计；R26-B1 v1.1 重写）。
 *
 * cmd/ota {url, version, force?, sha256?} → esp_https_ota（crt_bundle 公共
 * CA，与 MQTT 同套；CAM 11-10⑩ 同源实证 41s/3.2MB）→ 双槽写入 → 重启。
 * 自研 HTTP 下载循环退役（2026-11-21 评审 R26-9：esp_https_ota 自带镜像
 * 校验/断点/统一错误处理，且与 B2 rollback 集成更顺）。
 *
 * 安全闸（与 guardian config_api E3 双层，防御纵深）：
 *   - url 必须 https + kincal.cn 域（crt_bundle 之外的运营线）
 *   - version 单调闸 YYYYMMDDNN 数字序；force=true 跳过（bench 演练用）
 *   - sha256 字段协议保留暂不校验（49 S-4 有意简化，TLS 承载完整性）
 * 进度/结果上报 <prefix>/status/ota：downloading{progress} / failed{error}
 * / rebooting / boot_pending_verify / boot_confirmed（B3）。
 */
#include "ota_neck.h"
#include "mqtt_publisher.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "cJSON.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota_neck";

static void ota_publish(const char *prefix, const char *payload) {
    /* 11-23 存活案终破：原死码把 strlen 当 qos 传（真签名 qos/retain）→
     * 每次上报 QoS≈50 → mosquitto "Invalid QoS in PUBLISH, disconnecting"
     * → 连接刚建即被踢（三连复现的 EOF 环） */
    char topic[128];
    snprintf(topic, sizeof(topic), "%s/status/ota", prefix);
    mqtt_publisher_publish_raw(topic, payload, 1, 0);
}

static bool url_allowed(const char *url) {
    if (strncmp(url, "https://", 8) != 0) return false;
    const char *host = url + 8;
    const char *slash = strchr(host, '/');
    size_t hlen = slash ? (size_t)(slash - host) : strlen(host);
    const char *colon = memchr(host, ':', hlen);
    if (colon) hlen = (size_t)(colon - host);
    char h[64];
    if (hlen >= sizeof(h)) return false;
    memcpy(h, host, hlen);
    h[hlen] = '\0';
    return strcmp(h, "kincal.cn") == 0 ||
           (hlen > 9 && strcmp(h + hlen - 9, ".kincal.cn") == 0);
}

/* YYYYMMDDNN 数字单调比较；cur 可为任意 git-describe 串（非纯数字=旧版，
 * 视为 0——首版 OTA 总是放行） */
static bool version_newer(const char *cur, const char *next) {
    if (!next || strlen(next) != 10 || strspn(next, "0123456789") != 10)
        return false;
    if (!cur || strspn(cur, "0123456789") != strlen(cur) || strlen(cur) != 10)
        return true;
    return strtoul(next, NULL, 10) > strtoul(cur, NULL, 10);
}

static void ota_task(void *arg) {
    /* arg 指向堆上 {url[512], prefix[80]} — 本 task 独占并负责释放 */
    char *url = ((char **)arg)[0];
    char *prefix = ((char **)arg)[1];

    esp_http_client_config_t http_cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t cfg = {
        .http_config = &http_cfg,   /* IDF5: 指针 */
        .partial_http_download = false,
    };
    esp_https_ota_handle_t h = NULL;
    esp_err_t err = esp_https_ota_begin(&cfg, &h);
    if (err != ESP_OK || h == NULL) {
        ESP_LOGE(TAG, "ota_begin: %s", esp_err_to_name(err));
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"begin\"}");
        free(url); free(prefix); free(arg);
        vTaskDelete(NULL);
        return;
    }

    int last_pct = -1;
    while ((err = esp_https_ota_perform(h)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
        int total = esp_https_ota_get_image_size(h);
        int read = esp_https_ota_get_image_len_read(h);
        if (total > 0) {
            int pct = read * 100 / total;
            if (pct / 10 != last_pct / 10) {
                last_pct = pct;
                char st[80];
                snprintf(st, sizeof(st),
                         "{\"state\":\"downloading\",\"progress\":%d}", pct);
                ota_publish(prefix, st);
                ESP_LOGI(TAG, "OTA progress: %d%%", pct);
            }
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota perform/verify: %s", esp_err_to_name(err));
        esp_https_ota_abort(h);
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"download_or_verify\"}");
        free(url); free(prefix); free(arg);
        vTaskDelete(NULL);
        return;
    }
    err = esp_https_ota_finish(h);   /* IDF5.5: finish（含镜像校验） */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota_end: %s", esp_err_to_name(err));
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"end_or_image\"}");
        free(url); free(prefix); free(arg);
        vTaskDelete(NULL);
        return;
    }

    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    err = esp_ota_set_boot_partition(part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_boot_partition: %s", esp_err_to_name(err));
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"set_boot\"}");
        free(url); free(prefix); free(arg);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGW(TAG, "OTA complete (%s), rebooting in 3s", url);
    ota_publish(prefix, "{\"state\":\"rebooting\"}");
    vTaskDelay(pdMS_TO_TICKS(3000));
    free(url); free(prefix); free(arg);
    esp_restart();
}

void ota_neck_start(const char *url, const char *prefix) {
    /* 独立任务执行（不占 MQTT 事件回调；R26-B1） */
    char **args = malloc(sizeof(char *) * 2);
    char *u = strdup(url), *p = strdup(prefix);
    if (!args || !u || !p) { free(args); free(u); free(p); return; }
    args[0] = u; args[1] = p;
    if (xTaskCreate(ota_task, "ota", 8192, args, 3, NULL) != pdPASS) {
        free(u); free(p); free(args);
    }
}

void ota_neck_handle_cmd(const char *payload, int len, const char *prefix) {
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        ESP_LOGE(TAG, "cmd/ota: bad JSON");
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"bad_json\"}");
        return;
    }
    /* 11-27：cmd/ota 双板同 topic，target 分拣——"cam" → 脖子忽略；
     * "neck"/缺省 → 处理（兼容旧脚本） */
    const cJSON *tgt_item = cJSON_GetObjectItem(root, "target");
    if (cJSON_IsString(tgt_item) && strcmp(tgt_item->valuestring, "cam") == 0) {
        ESP_LOGI(TAG, "cmd/ota target=cam → neck ignore");
        cJSON_Delete(root);
        return;
    }
    const cJSON *url_item = cJSON_GetObjectItem(root, "url");
    const cJSON *ver_item = cJSON_GetObjectItem(root, "version");
    const cJSON *force_item = cJSON_GetObjectItem(root, "force");
    if (!cJSON_IsString(url_item) || !url_item->valuestring[0] ||
        !cJSON_IsString(ver_item) || !ver_item->valuestring[0]) {
        ESP_LOGE(TAG, "cmd/ota: missing url/version");
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"missing_fields\"}");
        cJSON_Delete(root);
        return;
    }
    const char *url = url_item->valuestring;
    const char *ver = ver_item->valuestring;
    bool force = cJSON_IsTrue(force_item);

    if (!url_allowed(url)) {
        ESP_LOGE(TAG, "cmd/ota: url rejected (https+kincal.cn only): %s", url);
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"url_not_allowed\"}");
        cJSON_Delete(root);
        return;
    }
    const esp_app_desc_t *desc = esp_app_get_description();
    if (!force && !version_newer(desc->version, ver)) {
        ESP_LOGW(TAG, "cmd/ota: version gate (cur='%s' next='%s', no force)",
                 desc->version, ver);
        char st[128];
        snprintf(st, sizeof(st),
                 "{\"state\":\"failed\",\"error\":\"version_not_newer\",\"cur\":\"%s\"}",
                 desc->version);
        ota_publish(prefix, st);
        cJSON_Delete(root);
        return;
    }

    ESP_LOGW(TAG, "OTA start: %s (v%s%s)", url, ver, force ? " FORCE" : "");
    char url_copy[512], prefix_copy[80];
    snprintf(url_copy, sizeof(url_copy), "%s", url);
    snprintf(prefix_copy, sizeof(prefix_copy), "%s", prefix);
    cJSON_Delete(root);
    ota_neck_start(url_copy, prefix_copy);
}

/* B2/B3：boot 状态上报 + mark_valid（WiFi+首次 MQTT CONNECTED 时机，
 * mqtt_publisher 调用）。R26-3 永久契约：此后每版固件必须保留本调用——
 * 漏一次=该版 OTA 后自动回滚。 */
void ota_neck_boot_check(const char *prefix) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) == ESP_OK &&
        st == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGW(TAG, "boot: OTA PENDING_VERIFY (mark_valid on first MQTT connect)");
        ota_publish(prefix, "{\"state\":\"boot_pending_verify\"}");
    }
}

void ota_neck_mark_valid(const char *prefix) {
    static bool done = false;
    if (done) return;
    done = true;
    /* 首次 MQTT CONNECTED = 生命体征达标（R26-3）；无 pending verify 时
     * 返回 ESP_ERR_NOT_FOUND（正常旧槽启动），boot_confirmed 照发 */
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    ota_publish(prefix, "{\"state\":\"boot_confirmed\"}");
    ESP_LOGI(TAG, "mark_app_valid: %s", esp_err_to_name(err));
}
