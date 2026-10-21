/* bed_zone.h — bed-zone suppression for docs/52 P0-2 (49 §3.1.4).
 *
 * A fall_suspect whose burst anchor lands inside the calibrated bed
 * distance window is demoted to in_bed_still ONLY when BOTH necessary
 * conditions hold (S-7 frozen, "双必要条件"):
 *   ① anchor (window median) within [min_mm, max_mm]
 *   ② entry-trajectory absent: zone continuously in for ≥ cont_s AND no
 *      upright-motion episode within the last traj_s seconds
 * An upright episode is the compact-and-moving signature of a person who
 * got up (spread < UPRIGHT_SPREAD_MM sustained UPRIGHT_FRAMES frames in
 * MOTION state) — in-bed rolling never gets compact while moving, so it
 * cannot masquerade as a get-up.
 *
 * Any condition failing ⇒ fall_suspect is published unchanged. With
 * min_mm == 0 the whole mechanism is disabled (default: pre-P0-2
 * behaviour is preserved until the bedside calibration writes a window).
 *
 * Also owns the night window (wall clock) used to weight the fall
 * detector's confirmation at night. Clock unsynced ⇒ treated as day
 * (fail-safe: normal sensitivity). */
#ifndef BED_ZONE_H
#define BED_ZONE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t min_mm;        /* 0 = suppression disabled (default)          */
    uint16_t max_mm;        /* must be > min_mm and <= 6000 when enabled   */
    uint16_t cont_s;        /* continuous in-zone seconds before ② holds   */
    uint16_t traj_s;        /* lookback seconds for upright episodes       */
    uint8_t  night_start_h; /* night window start hour, local time         */
    uint8_t  night_end_h;   /* night window end hour (wrap allowed)        */
} bed_cfg_t;

#define BED_CFG_DEFAULT { .min_mm = 0, .max_mm = 0, .cont_s = 120, \
                          .traj_s = 90, .night_start_h = 22, .night_end_h = 6 }

void bed_zone_init(void);

/* Per-frame feed from the radar task (µs clock, same as fall_feed).
 * zone_in = current zone state machine output; trusted = rb_conf ≥ 12
 * and target valid; motion = aggregated target in MOTION state;
 * spread_mm = fall-ring spread (radar_get_spread_mm()). */
void bed_zone_feed(bool zone_in, bool trusted, bool motion,
                   uint16_t spread_mm, int64_t now_us);

/* Suppression verdict for a burst anchored at anchor_mm. Pure read of
 * the tracked state — call from fall_fire before latching. */
bool bed_zone_should_suppress(uint16_t anchor_mm, int64_t now_us);

bool bed_zone_is_night(void);

const bed_cfg_t *bed_cfg(void);
bool bed_cfg_set(const bed_cfg_t *c);   /* validates + persists to NVS */

/* One-shot parameter profiles (docs/52 P0-2 dev item 2). "bedside"
 * widens the presence zone to cover the calibrated bed window and arms
 * the night window; "desk" restores the docs/48 defaults and disables
 * suppression. Bed window itself is calibration data, not profile data. */
void bed_zone_profile(const char *name);

uint32_t bed_zone_suppressed_count(void);

#endif /* BED_ZONE_H */
