#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "doa.h"

/* Sound-source tracker: maps DOA results to servo target angles.
 *
 * Strategy A from SERVO_PLAN.md (fixed home + ±27° fine-tune):
 *   target = (azimuth_deg - 180°) + home_deg
 *   clamp to [SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG]
 *
 * Goal: rotate the mic array so that its 6 o'clock direction (M3, α=180°)
 * points toward the sound source.
 *
 * Only 3-mic DOA results drive the servo. 2-mic results have front/back
 * ambiguity that's too coarse for servo control — those frames are logged
 * but don't move the servo. INVALID frames also don't move the servo.
 *
 * Deadband: if |target - current| < deadband_deg, the servo is not
 * commanded. This prevents tracking oscillation when the source is near
 * a target boundary (each servo motion shifts the array, which shifts
 * the DOA reading, which commands another motion, etc.). */

typedef enum {
    TRACKER_MODE_IDLE,        /* No valid DOA — servo holds last target */
    TRACKER_MODE_TRACKING,    /* 3-mic DOA accepted, servo tracking */
    TRACKER_MODE_OUT_OF_RANGE_SCAN, /* Source outside mechanical range — servo oscillates at boundary */
    TRACKER_MODE_SUPPRESSED,  /* 2-mic or invalid — log only, don't move */
    TRACKER_MODE_DISABLED,    /* Manually disabled for debugging */
} tracker_mode_t;

typedef struct {
    /* home_deg: offset added to target. 0 = home points array's 6oc at the
     * "neutral" direction in the room. Increase/decrease to shift the
     * servo's tracking window. */
    float    home_deg;
    /* deadband_deg: minimum target change to trigger motion. */
    float    deadband_deg;
    /* min_confidence: 3-mic frames with conf below this are SUPPRESSED. */
    float    min_confidence;
    /* use_stable_sextant: if true, only update target when stable_sextant
     * changes (very conservative, no within-sextant tracking). If false,
     * use raw azimuth_deg for smoother tracking (subject to deadband). */
    bool     conservative_mode;
    /* out_of_range_deg: if |source α - 180°| exceeds this, the source is
     * too far from M3's home direction for the servo to track (mechanical
     * range is ±20°, so anything outside ±45° is definitely unreachable).
     * Tracker holds the last command instead of oscillating between
     * mechanical limits. Set to 0 to disable this check. */
    float    out_of_range_deg;
    /* target_agreement_deg: require two consecutive frames' raw targets
     * to agree within this tolerance before commanding motion. Filters
     * out single-frame GCC-PHAT transients (e.g., servo-motor buzz
     * leaking through the motion-pause window). 0 disables the check.
     * (Phase A: replaced by 2-of-3 ring-buffer check in tracker.c.) */
    float    target_agreement_deg;
    /* idle_return_threshold_s: if no valid 3-mic DOA frame is seen for
     * this many seconds, slowly return the servo to home (0°). Prevents
     * the servo from sitting at ±33° indefinitely after the user stops
     * speaking — both a UX issue and a cause of limit-position buzz that
     * couples into the mic PCB. 0 disables. */
    float    idle_return_threshold_s;
    /* idle_return_rate_deg_per_s: how fast to return to home during idle.
     * Slow (2-3°/s) avoids startling the user and keeps motion-pause
     * intervals short so DOA recovers quickly if speech resumes. */
    float    idle_return_rate_deg_per_s;

    /* ---- Boundary scan (when source is outside mechanical range) ----
     *
     * When the un-clamped target exceeds scan_enter_deg, the servo
     * oscillates ±scan_amplitude_deg around the nearest mechanical limit
     * (±SERVO_ANGLE_MAX_DEG) at scan_rate_deg_per_s. This signals "I hear
     * you but can't reach you" instead of sitting still.
     *
     * Hysteresis: enter at scan_enter_deg, exit at scan_exit_deg. The gap
     * prevents oscillation between TRACKING and SCAN at the boundary under
     * noisy DOA. Set scan_enter_deg=0 to disable the feature entirely. */
    float    scan_enter_deg;
    float    scan_exit_deg;
    float    scan_amplitude_deg;
    float    scan_rate_deg_per_s;
} tracker_config_t;

#define TRACKER_DEFAULT_CONFIG  { \
    .home_deg             = 0.0f,    \
    .deadband_deg         = 3.0f,    \
    .min_confidence       = 0.35f,   \
    .conservative_mode    = false,   \
    .out_of_range_deg     = 150.0f,  \
    .target_agreement_deg = 5.0f,    \
    .idle_return_threshold_s    = 10.0f, \
    .idle_return_rate_deg_per_s = 2.5f,  \
    .scan_enter_deg       = 95.0f,    \
    .scan_exit_deg        = 85.0f,    \
    .scan_amplitude_deg   = 5.0f,     \
    .scan_rate_deg_per_s  = 10.0f,    \
}

void tracker_init(const tracker_config_t *cfg);

/* Per-frame update. Called from mic_task after doa_process(). */
void tracker_update(const doa_result_t *doa);

/* Current tracker mode (informational; tracker_update sets it). */
tracker_mode_t tracker_get_mode(void);

/* Enable / disable tracker. When disabled, servo stays at last target. */
void tracker_set_enabled(bool enabled);
bool tracker_is_enabled(void);

/* Reset all internal tracking state (EMA, agreement buffer, target history).
 * Called when switching from COMMAND back to TRACK to ensure the tracker
 * starts fresh from the current servo position, without stale state. */
void tracker_reset_state(void);

/* Update config at runtime (Phase 4 will wire this to UART). */
void tracker_set_config(const tracker_config_t *cfg);
const tracker_config_t *tracker_get_config(void);
