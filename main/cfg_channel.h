/* cfg_channel.h — docs/52 P0-6: 产品配置全量 MQTT 下发通道 + 设备回执。
 *
 * 话题（49 §12.4 Phase-A）：
 *   下行 <prefix>/cmd/cfg_set   {"req_id":"...","zone_enter_mm":1500,...}
 *   回执 <prefix>/status/cfg    {"req_id":"...","applied":[...],"rejected":[...]}
 * 回执由 guardian 侧对账（下发-回执闭环）。REST 配置入口保留但定位为
 * 「台架/开发」（rest_api.c 头注）。
 *
 * v1 字段集 = zone 参数 + 模式参数（still_min/fall_detect/assoc_gate）；
 * bed_* 床区窗字段随 P0-2 分支合入时并入（同一 handler 扩展）。 */
#ifndef CFG_CHANNEL_H
#define CFG_CHANNEL_H

void cfg_channel_init(void);   /* 注册 mqtt_publisher 回调 */

#endif /* CFG_CHANNEL_H */
