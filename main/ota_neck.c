/* ota_neck.c — 脖子板 OTA（docs/49 §12.6，C 实现）。
 *
 * cmd/ota {url} → esp_http_client 下载 → esp_ota_ops 写另一槽 → 重启。
 * TLS 用 crt_bundle（与 MQTT 同套 CA）。进度上报 status/ota。
 */
#include "ota_neck.h"
#include "mqtt_publisher.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "cJSON.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota_neck";
#define OTA_BUF_SIZE 4096

/* 简单发布辅助（mqtt_publisher 内部用 esp_mqtt_client_publish） */
static void ota_publish(const char *prefix, const char *payload) {
    char topic[128];
    snprintf(topic, sizeof(topic), "%s/status/ota", prefix);
    mqtt_publisher_publish_raw(topic, payload, (int)strlen(payload), 1);
}

esp_err_t ota_neck_start(const char *url, const char *prefix) {
    ESP_LOGI(TAG, "OTA start: %s", url);

    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) {
        ESP_LOGE(TAG, "No OTA partition（分区表需 ota_0+ota_1）");
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"no_partition\"}");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Writing to %s at 0x%lx", part->label, (unsigned long)part->address);

    esp_http_client_config_t config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "HTTP init failed");
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"http_init\"}");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open: %s", esp_err_to_name(err));
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"http_open\"}");
        esp_http_client_cleanup(client);
        return err;
    }

    int status = esp_http_client_get_status_code(client);
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP status %d", status);
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"http_status\"}");
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    int64_t content_length = esp_http_client_fetch_headers(client);
    if (content_length <= 0) {
        ESP_LOGE(TAG, "No content length");
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"no_length\"}");
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Firmware size: %lld bytes", (long long)content_length);

    esp_ota_handle_t handle;
    err = esp_ota_begin(part, OTA_SIZE_UNKNOWN, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota_begin: %s", esp_err_to_name(err));
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"ota_begin\"}");
        esp_http_client_cleanup(client);
        return err;
    }

    char *buf = malloc(OTA_BUF_SIZE);
    if (!buf) {
        esp_ota_abort(handle);
        esp_http_client_cleanup(client);
        return ESP_ERR_NO_MEM;
    }

    int total_read = 0;
    int last_percent = -1;

    while (1) {
        int read_len = esp_http_client_read(client, buf, OTA_BUF_SIZE);
        if (read_len < 0) {
            ESP_LOGE(TAG, "HTTP read failed");
            free(buf);
            esp_ota_abort(handle);
            esp_http_client_cleanup(client);
            ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"read\"}");
            return ESP_FAIL;
        }
        if (read_len == 0) break;

        err = esp_ota_write(handle, buf, read_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ota_write: %s", esp_err_to_name(err));
            free(buf);
            esp_ota_abort(handle);
            esp_http_client_cleanup(client);
            ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"write\"}");
            return err;
        }

        total_read += read_len;
        int percent = (int)((total_read * 100) / content_length);
        if (percent / 10 != last_percent / 10) {
            last_percent = percent;
            char status_buf[128];
            snprintf(status_buf, sizeof(status_buf),
                "{\"state\":\"downloading\",\"progress\":%d}", percent);
            ota_publish(prefix, status_buf);
            ESP_LOGI(TAG, "OTA progress: %d%%", percent);
        }
    }

    free(buf);
    esp_http_client_cleanup(client);

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota_end: %s", esp_err_to_name(err));
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"ota_end\"}");
        return err;
    }

    err = esp_ota_set_boot_partition(part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_boot_partition: %s", esp_err_to_name(err));
        ota_publish(prefix, "{\"state\":\"failed\",\"error\":\"set_boot\"}");
        return err;
    }

    ESP_LOGI(TAG, "OTA complete, rebooting in 3s");
    ota_publish(prefix, "{\"state\":\"rebooting\"}");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
    return ESP_OK;
}

void ota_neck_handle_cmd(const char *payload, int len, const char *prefix) {
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        ESP_LOGE(TAG, "cmd/ota: bad JSON");
        return;
    }
    cJSON *url_item = cJSON_GetObjectItem(root, "url");
    if (!cJSON_IsString(url_item) || !url_item->valuestring[0]) {
        ESP_LOGE(TAG, "cmd/ota: missing url");
        cJSON_Delete(root);
        return;
    }
    char url[512];
    snprintf(url, sizeof(url), "%s", url_item->valuestring);
    cJSON_Delete(root);

    /* P1：同步调用（MQTT 事件回调里跑，阻塞期间不收新消息——种子期可接受） */
    ESP_LOGW(TAG, "OTA sync start: %s", url);
    ota_neck_start(url, prefix);
}
