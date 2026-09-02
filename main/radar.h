#pragma once

#include <stdbool.h>
#include <stdint.h>

/* MS60-1211S80M (AT6010) radar driver. Single aggregated target via 5 Hz
 * polling of command 0x30 (3.2.6). Active reporting is not functional on
 * this firmware — see tasks/radar-protocol-notes.md. */

typedef enum {
    RADAR_TGT_NONE = 0,
    RADAR_TGT_APPROACH = 0x01,  /* 靠近 */
    RADAR_TGT_DEPART   = 0x02,  /* 远离 */
    RADAR_TGT_MOTION   = 0x04,  /* 运动 */
    RADAR_TGT_BREATH   = 0x10,  /* 呼吸/微动（静止人体存在） */
} radar_tgt_state_t;

/* Radar angle -> clock azimuth mapping, 3-point fit measured 2026-09-02
 * (tasks/radar-protocol-notes.md): 6点/5点/7点 at ~1m gave pairs
 * (angle 0 -> az 183°, -20 -> 148°, +18 -> 227°), residuals ±4°.
 * Slope ~2.1 ≈ 1/sin(30°): the firmware reports a sine-compressed angle.
 * Linear fit is adequate for |angle| <= 35° against a ±20° association gate. */
#define RADAR_AZ_OFFSET_DEG   187.0f
#define RADAR_AZ_SCALE        2.1f

typedef struct {
    bool valid;                 /* target present (det_result != 0) */
    uint8_t det_result;         /* raw bitmask from fmcw_det_info_t */
    radar_tgt_state_t state;    /* decoded primary state, motion has priority */
    uint16_t range_mm;          /* valid when rb_conf >= 12 */
    int16_t angle_deg;          /* raw radar angle; yaw mapping is Phase 2 */
    int16_t angle_filt_deg;     /* median of last conf-qualified samples */
    uint8_t filt_n;             /* samples behind the median (0-5) */
    float azimuth_deg;          /* mapped clock azimuth 0-360, 12点=0 */
    uint8_t rb_conf;            /* 0-16 */
    uint8_t angle_conf;         /* 0-16; angle trustworthy when >= 8 */
    uint32_t frame_idx;
    int64_t last_seen_ms;       /* esp_timer ms of the last 0x30 reply */
} radar_target_t;

/* Starts the poll task. Never blocks or fails the app — the radar is an
 * enhancement, not a dependency: on link failure the system degrades to
 * audio-only tracking. */
void radar_init(void);

/* Link considered alive (0x30 replies arriving within the last ~600 ms). */
bool radar_is_online(void);

/* Snapshot of the current primary target. Returns false when no target is
 * detected; *out is still filled with state=NONE fields in that case. */
bool radar_get_target(radar_target_t *out);

/* Notify the radar module that speech/DOA activity was observed (used by
 * the stillness care alarm as a recovery trigger). */
void radar_notify_sound(void);
