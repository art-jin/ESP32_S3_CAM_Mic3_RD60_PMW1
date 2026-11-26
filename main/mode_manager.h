#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Dual-mode manager: TRACK (auto sound-source tracking) and COMMAND
 * (REST-directed servo positioning). Default is TRACK on boot.
 *
 * TRACK has three sub-modes (Phase 3, PRD US-005):
 *   audio_only  — only sound drives the servo (base behaviour, default)
 *   fusion      — sound has priority; with no recent sound the servo
 *                 follows the radar primary target
 *   radar_follow— only the radar target drives the servo
 *
 * Out-of-range policy (US-006): what the servo does when the target
 * azimuth is beyond the mechanical ±90° (9点~3点):
 *   hold (default) / clamp to boundary / return home / boundary scan (v2.5
 *   base behaviour, kept as an option).
 *
 * Sub-mode and policy persist in NVS across reboots. */

typedef enum {
    MODE_TRACK = 0,     /* tracker ON, /point rejected */
    MODE_COMMAND,       /* tracker OFF, /point accepted */
} app_mode_t;

typedef enum {
    TRACK_AUDIO_ONLY = 0,
    TRACK_FUSION,
    TRACK_RADAR_FOLLOW,
} track_submode_t;

typedef enum {
    OOR_HOLD = 0,
    OOR_CLAMP,
    OOR_HOME,
    OOR_SCAN,
} oor_policy_t;

void mode_manager_init(void);

/* Atomic read of current mode. */
app_mode_t mode_manager_get(void);

/* Switch mode. Handles tracker enable/disable + state reset.
 * timeout_s: command mode auto-return timeout (0 = never). */
void mode_manager_set(app_mode_t mode, int timeout_s);

/* Get current effective timeout (0 = no auto-return). */
int mode_manager_get_timeout(void);

/* 96-S1：COMMAND one-shot 的来源（谁把脖子切到 COMMAND）。start 会话
 * 不抢占 SRC_MQTT（R27 巡检拍照窗）；来源随回 TRACK 自动清零。 */
typedef enum {
    ONESHOT_NONE = 0,
    ONESHOT_MQTT,      /* cmd/point（巡检/语音转向） */
    ONESHOT_SESSION,   /* cmd/session end（会话结束归位） */
} oneshot_src_t;
oneshot_src_t mode_manager_oneshot_src(void);
void mode_manager_set_oneshot_src(oneshot_src_t src);
/* 96-S1：会话期强制 FUSION，会话前子模式快照供结束还原。 */
void mode_manager_snapshot_submode(void);
void mode_manager_restore_submode(void);

/* Track sub-mode (only meaningful in MODE_TRACK). Resets tracker state
 * on change for a clean transition. Persisted in NVS. */
void mode_manager_set_submode(track_submode_t submode);
track_submode_t mode_manager_get_submode(void);

/* Out-of-range policy. Persisted in NVS. */
void mode_manager_set_oor_policy(oor_policy_t policy);
oor_policy_t mode_manager_get_oor_policy(void);

/* Long-stillness care alarm threshold in minutes (US-009): fire
 * STILL_ALARM when the radar target stays in breath/micro-motion state
 * continuously for this long. 0 = disabled (default). Persisted in NVS. */
void mode_manager_set_still_min(uint16_t minutes);
uint16_t mode_manager_get_still_min(void);

/* Sound association gate (robot-speaker immunity): when enabled, sound
 * only drives the servo when its DOA coincides with the radar target
 * (the person). Non-coincident sources (e.g. a robot speaker at 3oc)
 * are logged as events but never move the servo. Bypassed while the
 * radar is offline. Persisted in NVS, default off. */
void mode_manager_set_assoc_gate(bool on);
bool mode_manager_get_assoc_gate(void);

/* Fall-suspect detector (weak-feature range-step chain, bench-validated
 * 2026-09-07): emits FALL_SUSPECT / FALL_CLEAR events. Default ON.
 * Persisted in NVS. */
void mode_manager_set_fall_detect(bool on);
bool mode_manager_get_fall_detect(void);

/* R25-P0 影子模式：跌倒判定照跑、事件不外发（3-7 天系数采集期），
 * 默认 false=生产行为不变；cfg_set {"fall_shadow":1} 远程开启 */
void mode_manager_set_fall_shadow(bool on);
bool mode_manager_get_fall_shadow(void);

/* Called from mic_task at 20Hz. Checks command-mode timeout. */
void mode_manager_tick(void);

/* Called from /api/point handler. Resets the command-mode timer. */
void mode_manager_register_command(void);
