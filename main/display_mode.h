#ifndef DISPLAY_MODE_H
#define DISPLAY_MODE_H

#include <stdbool.h>
#include <stdint.h>

/* docs/74 §4/M1 — 显示模式注册表。REST /api/display 与 MQTT cmd/display
 * 同构入口:两者最终都调 display_mode_show(),载荷即 74 §4 的完整 JSON
 * (渲染器自取 mode 专属字段)。TTL 到期自动回默认页(月历)。 */

enum {
    DMODE_CALENDAR = 0,        /* 默认页:月历(本地渲染,回退目标) */
    DMODE_CALENDAR_OVERLAY,    /* 月历 + overlay(todo_days/holidays 框线) */
    DMODE_WEATHER,             /* title+lines 文本页 */
    DMODE_TODO,                /* title+items 文本页 */
    DMODE_STATUS,              /* 本地状态自渲染(不依赖载荷) */
    DMODE_MEDGRID,             /* 3×7 药盒格 */
    DMODE_MSG,                 /* title+lines 确认页 */
    DMODE_COUNT
};

#define DISPLAY_PAYLOAD_MAX 1024
#define DISPLAY_TTL_DEFAULT_SEC 90

int         display_mode_from_name(const char *name);
const char *display_mode_name(int mode);

/* 请求切换模式。payload_json 为完整原始 JSON(可为 NULL → "{}")。
 * ttl_sec<=0 → 90s 默认。返回 false = 未知模式。
 * 线程安全(REST task / MQTT task / EPD task 三方)。 */
bool display_mode_show(int mode, const char *payload_json, int ttl_sec);

/* 生效模式(TTL 过期自动回 DMODE_CALENDAR)。 */
int         display_mode_active(void);
/* 当前载荷(display_mode_active() 对应;恒非 NULL)。 */
const char *display_mode_payload(void);
/* 每次 show 自增 — EPD task 用 (mode,seq) 变更检测。 */
uint32_t    display_mode_seq(void);
/* 当前临时页剩余毫秒;默认页返回 -1。 */
int64_t     display_mode_ttl_remaining_ms(void);

/* EPD task 启动时注册自己,show() 通过 notify 唤醒渲染。 */
void display_mode_set_notify_task(void *task_handle);

/* MQTT cmd/display 通道接线(mqtt_publisher 下行 → show)。 */
void display_mode_mqtt_init(void);

#endif
