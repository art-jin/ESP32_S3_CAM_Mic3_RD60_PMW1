/* radar_turn.h — 97：雷达直驱转向（脖子本地反射）。 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    bool     enabled;
    int      hold_s;
    int      cooldown_s;
    uint8_t  night_start;
    uint8_t  night_end;
} radar_turn_cfg_t;

void radar_turn_init(void);
void radar_turn_set_time_synced(void);
bool radar_turn_on_enter(void);
void radar_turn_on_leave(void);
void radar_turn_tick(void);
/* cmd/wake_cfg 子集应用（get_int=调用方的 JSON int 取值器）。 */
void radar_turn_apply_cfg(const char *body,
                          bool (*get_int)(const char *, const char *, int *));
void radar_turn_on_wake_cfg(const char *body);
/* 回执 JSON 片段（无外层花括号）。 */
void radar_turn_cfg_json(char *out, size_t sz);
