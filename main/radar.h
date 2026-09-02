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

/* ---- Configuration requests (PRD US-010, V2) ----
 * All UART I/O stays inside the radar task: callers post a request and
 * wait on a semaphore; the poll loop executes it and fills the result. */

typedef struct {
    /* factory boundaries (cm), from cmd 0x32 */
    uint16_t b_mot_min, b_mot_max, b_micro_min, b_micro_max, b_bhr_min, b_bhr_max;
    /* current user config (cm + levels 0-10), from cmd 0x33 */
    uint16_t mot_min, mot_max, micro_min, micro_max, bhr_min, bhr_max;
    uint16_t mot_lvl, micro_lvl, bhr_lvl;
    bool sensing;       /* from 0xD0 */
    bool online;
} radar_cfg_t;

/* Bitmask of which SET fields the caller wants applied (radar_set_mask_t) */
#define RAD_SET_SENSING   (1u << 0)
#define RAD_SET_MOT_MIN   (1u << 1)
#define RAD_SET_MOT_MAX   (1u << 2)
#define RAD_SET_MOT_LVL   (1u << 3)
#define RAD_SET_BHR_MIN   (1u << 4)
#define RAD_SET_BHR_MAX   (1u << 5)
#define RAD_SET_BHR_LVL   (1u << 6)
#define RAD_SET_SAVE      (1u << 7)

typedef struct {
    uint32_t mask;      /* RAD_SET_* of fields valid below */
    bool     sensing;
    uint16_t mot_min, mot_max, mot_lvl;
    uint16_t bhr_min, bhr_max, bhr_lvl;
} radar_set_req_t;

/* POST a config read. Returns false on timeout (radar busy/offline). */
bool radar_req_get_cfg(radar_cfg_t *out, uint32_t timeout_ms);

/* POST a config write (+optional save). *out receives the readback
 * config afterwards. Returns false on timeout; per-field verification
 * is the caller's job (compare request vs out). */
bool radar_req_set_cfg(const radar_set_req_t *req, radar_cfg_t *out,
                       uint32_t timeout_ms);

/* POST a radar system reset (clears the phantom target). Async-ish:
 * returns once the reset command is queued; the radar re-inits for
 * ~5 s afterwards. */
bool radar_req_reset(uint32_t timeout_ms);
