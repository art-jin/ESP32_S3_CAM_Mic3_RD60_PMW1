#pragma once
#include "esp_err.h"

/* OTA P1.5（脖子板，C 实现）——cmd/ota handler + 固件下载烧写 */
esp_err_t ota_neck_start(const char *url, const char *prefix);
void ota_neck_handle_cmd(const char *payload, int len, const char *prefix);
