/* zone_tracker.h — presence-zone state machine for docs/48 Tier 1.
 *
 * Turns the 5 Hz radar target stream into one-shot zone_enter / zone_leave
 * edges (person within zone_enter_mm) with debounce + range hysteresis +
 * target-loss timeout, so a person lingering at the 1.5 m boundary or a
 * momentary detection dropout does not flap the MQTT event stream.
 *
 * Feed from the radar task's per-frame path (pure arithmetic, ~µs).
 * Config persists in NVS namespace "zcfg" (REST-configurable, §6.4). */
#ifndef ZONE_TRACKER_H
#define ZONE_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ZONE_EDGE_NONE = 0,
    ZONE_EDGE_ENTER,
    ZONE_EDGE_LEAVE,
} zone_edge_t;

typedef struct {
    uint16_t enter_mm;     /* in-zone threshold                        */
    uint16_t debounce;     /* consecutive in-range frames to confirm   */
    uint16_t hyst_mm;      /* extra margin before leaving counts       */
    uint16_t leave;        /* consecutive out-of-range frames to leave */
    uint16_t lost_ms;      /* untrusted/absent for this long ⇒ leave   */
} zone_cfg_t;

/* Defaults: 1500 mm, 3 frames (600 ms), 400 mm hysteresis, 5 frames (1 s),
 * 2000 ms loss timeout — docs/48 §13. */
#define ZONE_CFG_DEFAULT { .enter_mm = 1500, .debounce = 3, \
                           .hyst_mm = 400, .leave = 5, .lost_ms = 2000 }

/* Load persisted config (falls back to defaults). Call once from app_main. */
void zone_tracker_init(void);

/* Feed one radar frame. valid = target detected this frame; range_mm /
 * rb_conf straight from the 0x30 reply. Frames with rb_conf < 12 count as
 * untrusted (same gate as fusion.c). Returns at most one edge per call. */
zone_edge_t zone_sm_feed(bool valid, uint32_t range_mm, uint8_t rb_conf,
                         int64_t now_ms);

/* Force the machine to the out-zone state (radar went offline / target was
 * cleared). Returns ZONE_EDGE_LEAVE iff it was in-zone, so the caller can
 * publish the trailing zone_leave before the offline event. */
zone_edge_t zone_sm_reset(void);

bool zone_in(void);

/* Validated config access. cfg_set persists to NVS and returns false on
 * out-of-range values (caller answers HTTP 400). */
const zone_cfg_t *zone_cfg(void);
bool zone_cfg_set(const zone_cfg_t *c);

#endif /* ZONE_TRACKER_H */
