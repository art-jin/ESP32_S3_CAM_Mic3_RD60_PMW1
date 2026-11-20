/* identity.h — F-1b 脖子 NVS 身份后置（89 v1.1 §1.3 脖子同步改造）
 *
 * mqtt_creds.h 五项 + wifi_creds.h 双槽从 NVS "identity" namespace 读取；
 * NVS 为空则回落编译期默认（存量脖子零重烧兼容，红线①）。getter 惰性
 * 加载——wifi/mqtt 初始化早于任何显式 identity_init 调用也能拿到值。
 *
 * 注入：/api/identity_config（rest_api.c，?device_id= 认证 + 写后回读
 * 校验 + 重启生效）。EXPECT_DEVICE_ID 护栏随身份同源——错身份烧录仍会被
 * mqtt_publisher 的 device_id 比对拒绝。
 */
#pragma once
#include <stdbool.h>
#include "esp_err.h"

void identity_init(void);   /* 幂等；getter 也会惰性触发 */

/* MQTT（NVS 优先，mqtt_creds.h 编译期回落） */
const char *identity_mqtt_broker(void);
const char *identity_mqtt_user(void);
const char *identity_mqtt_pass(void);
const char *identity_mqtt_prefix(void);
const char *identity_expect_device_id(void);

/* WiFi 双槽（NVS 优先，wifi_creds.h 编译期回落；静态 IP 标志仍编译期） */
const char *identity_wifi_ssid(int slot);   /* slot 0/1 */
const char *identity_wifi_pass(int slot);

/* NVS 中是否有完整 MQTT 身份（五项全非空） */
bool identity_is_provisioned(void);

/* 写入（注入路径）：NULL/空串 = 该字段不写（保留现有） */
esp_err_t identity_write_all(const char *broker, const char *user,
                             const char *pass, const char *prefix,
                             const char *expect_device_id,
                             const char *wifi0_ssid, const char *wifi0_pass,
                             const char *wifi1_ssid, const char *wifi1_pass);

/* 回读校验：比对 NVS 落盘值与期望值（空期望跳过）；true = 全部一致 */
bool identity_verify(const char *broker, const char *user,
                     const char *pass, const char *prefix,
                     const char *expect_device_id,
                     const char *wifi0_ssid, const char *wifi0_pass,
                     const char *wifi1_ssid, const char *wifi1_pass);

/* REST 只读摘要（密码掩码） */
const char *identity_summary_json(void);
