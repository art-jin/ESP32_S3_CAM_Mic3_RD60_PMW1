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
/* 用户裁决(10-29):非月历内容页至少维持 90s — 全刷本身 ~14s,短 TTL
 * 页面刚上屏就过期回刷,观感是"闪一下就没了"。 */
#define DISPLAY_TTL_MIN_SEC 90

int         display_mode_from_name(const char *name);
const char *display_mode_name(int mode);

/* 请求切换模式。payload_json 为完整原始 JSON(可为 NULL → "{}")。
 * ttl_sec<=0 → 90s 默认。返回 false = 未知模式。
 * **TTL 自上屏完成起算**(EPD 全刷 ~14s,自请求起算会把短页刚刷上
 * 就过期):show() 置 pending,EPD 渲染完成回调 display_mode_rendered()
 * 才开始倒计时。线程安全(REST/MQTT/EPD 三方)。 */
bool display_mode_show(int mode, const char *payload_json, int ttl_sec);

/* EPD task:某模式上屏完成后调用 — 启动其 TTL 倒计时。 */
void display_mode_rendered(int mode);

/* 生效模式(TTL 过期自动回 DMODE_CALENDAR)。 */
int         display_mode_active(void);
/* 当前载荷(display_mode_active() 对应;恒非 NULL)。 */
const char *display_mode_payload(void);
/* 每次 show 自增 — EPD task 用 (mode,seq) 变更检测。 */
uint32_t    display_mode_seq(void);
/* 当前临时页剩余毫秒;默认页返回 -1。pending(未上屏)返回 ttl 请求值。 */
int64_t     display_mode_ttl_remaining_ms(void);

/* EPD task 启动时注册自己,show() 通过 notify 唤醒渲染。 */
void display_mode_set_notify_task(void *task_handle);

/* MQTT cmd/display 通道接线(mqtt_publisher 下行 → show)。 */
void display_mode_mqtt_init(void);

#endif
