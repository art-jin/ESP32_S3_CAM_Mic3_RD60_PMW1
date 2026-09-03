#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "radar.h"

/* Sound-source ↔ radar-target association (PRD US-004).
 *
 * Evaluated on every tracker-accepted 3-mic DOA event. The result is
 * metadata only in Phase 2 — it never gates servo motion (that is the
 * Phase 3 sub-mode work). 2-mic DOA frames are not associated (half-plane
 * only, no trustworthy azimuth). */

#define FUSION_ASSOC_GATE_DEG 20.0f   /* default association gate */

typedef struct {
    bool evaluated;           /* at least one DOA event since boot */
    bool associated;
    bool assoc_instant;       /* pre-hysteresis association of the last DOA
                               * (used by the tracker's association gate) */
    bool radar_online;
    float doa_az_deg;         /* tracker-accepted DOA azimuth */
    float radar_az_deg;       /* radar target azimuth at that moment */
    float angle_diff_deg;     /* wrapped |doa - radar| */
    uint16_t range_mm;        /* from radar target (rb_conf gated) */
    radar_tgt_state_t state;  /* radar target state */
    int64_t timestamp_ms;
} fusion_result_t;

void fusion_init(void);

/* Called from the tracker accept path with the accepted DOA azimuth and
 * its frame confidence. Confidence also gates the stillness-alarm speech
 * trigger (noise peaks pass the tracker at 0.35-0.45; real speech is
 * higher — base-project calibration). */
void fusion_evaluate(float doa_az_deg, float confidence);

/* Snapshot for status/REST (Phase 4). */
void fusion_get_last(fusion_result_t *out);
