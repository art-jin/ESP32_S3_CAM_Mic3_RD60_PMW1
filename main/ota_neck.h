#pragma once

/* 脖子 OTA（docs/49 §12.6 + R26-B1 v1.1：esp_https_ota 内核）。
 * cmd/ota {url,version,force?} → 下载双槽 → 重启；进度/结果上报
 * <prefix>/status/ota。安全闸=https+kincal.cn 域+版本单调（force 跳过）。 */
void ota_neck_handle_cmd(const char *payload, int len, const char *prefix);

/* B2/B3（R26-3 永久契约：每版固件必含 mark_valid 调用，漏一次=该版
 * OTA 后被自动回滚）。mqtt_publisher 在首次 MQTT CONNECTED 时调用。 */
void ota_neck_boot_check(const char *prefix);   /* PENDING_VERIFY 上报 */
void ota_neck_mark_valid(const char *prefix);   /* 生命体征达标+boot_confirmed */
