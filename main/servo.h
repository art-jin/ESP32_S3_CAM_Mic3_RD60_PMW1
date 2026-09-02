#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "board_config.h"

/* Servo driver via LEDC PWM. Hardware details: see CLAUDE.md "Servo hardware".
 *
 * Two servo models are supported (pick one in board_config.h via SERVO_MODEL_*):
 *
 *   JS6620_EXTERNAL_GEAR (default):
 *     - 270° rotation hobby PWM servo
 *     - 50 Hz PWM, pulse 500–2500 µs
 *     - 15T pinion → 20T external spur gear, reduction 1.333:1
 *     - 270° servo travel = ~202.5° gear travel, mechanical limit ±101.25°
 *     - Shaft-down + external mesh: two direction inversions cancel → net 0
 *
 *   MG90S_DIRECT_DRIVE:
 *     - 180° MG90S/SG90 metal-gear micro servo
 *     - 50 Hz PWM, pulse 500–2500 µs (same electrical interface)
 *     - No gear reduction (1:1), shaft-up, rigid disc → robot head
 *     - Direction empirically inverted (clone MG90S or disc mount offset);
 *       boot sweep 0→+60→0→−60→0 observed as head going 6oc→4oc→6oc→8oc→6oc
 *       (expected 6oc→8oc→6oc→4oc→6oc), so the slope sign must flip.
 *
 * Any board (BOARD_*) can drive either servo. GPIO comes from board_config.h;
 * travel / gear / direction / limits / boot sweep range come from servo.h
 * based on the SERVO_MODEL_* selection.
 *
 * Angle sign convention: looking down from above, positive angle = CW. */

/* SERVO_GPIO defined in board_config.h */

#define SERVO_PWM_FREQ_HZ      50
#define SERVO_PERIOD_US        (1000000 / SERVO_PWM_FREQ_HZ)   /* 20000 µs */

/* Pulse-width range (identical for both supported servos). */
#define SERVO_PULSE_MIN_US     500
#define SERVO_PULSE_CENTER_US  1500
#define SERVO_PULSE_MAX_US     2500

#if defined(SERVO_MODEL_JS6620_EXTERNAL_GEAR)

    /* 270° JS6620 with 1.333:1 external spur gear reduction. */
    #define SERVO_TRAVEL_DEG           270.0f
    #define SERVO_GEAR_RATIO           1.333f        /* 20T / 15T */
    /* Mechanical limit at gear output: ±101.25° (270° / 1.333 / 2). Battery-
     * brownout incidents at end-of-travel led to ±90° clamp (10° margin).
     * ±80° was over-conservative (lost real 3oc/9oc coverage); ±100° hit the
     * brownout-prone region. Coverage from 6oc home: ~3:00 → 9:00 o'clock. */
    #define SERVO_ANGLE_MIN_DEG        (-90.0f)
    #define SERVO_ANGLE_MAX_DEG        (+90.0f)
    /* Two inversions (shaft-down + external mesh) cancel → no negate. */
    #define SERVO_DIRECTION_INVERTED   0
    /* Battery brownout (3x ESP_RST_BROWNOUT in 60 s) at ±100° boot sweep led
     * to ±60° cap. Still gives a clear visual range confirmation. */
    #define SERVO_BOOT_SWEEP_DEG       60.0f

#elif defined(SERVO_MODEL_MG90S_DIRECT_DRIVE)

    /* 180° MG90S/SG90, direct drive, no gear reduction. */
    #define SERVO_TRAVEL_DEG           180.0f
    #define SERVO_GEAR_RATIO           1.0f
    /* 180° servo hard limit is ±90°. Clamp to ±85° leaves a 5° margin from
     * the mechanical end-stop to avoid buzzing, current spikes, and the
     * "past 9oc / before 3oc" overshoot caused by typical MG90S clone travel
     * tolerance (±5%, often 190–200° actual travel). If still overshooting,
     * drop to ±80°; if buzzing couples into the 3DMIC-291 PCB and triggers
     * L/R collapse, drop further. */
    #define SERVO_ANGLE_MIN_DEG        (-85.0f)
    #define SERVO_ANGLE_MAX_DEG        (+85.0f)
    /* Empirical: boot sweep +60 took head to 4oc instead of 8oc, meaning
     * +command → head CCW (opposite of JS6620). Negate slope to undo. */
    #define SERVO_DIRECTION_INVERTED   1
    /* MG90S draws less current than JS6620; ±80° boot sweep is brownout-safe
     * and gives a clearer visual confirmation than ±60°. */
    #define SERVO_BOOT_SWEEP_DEG       80.0f

#endif

esp_err_t servo_init(void);

/* Set raw pulse width in microseconds. Clamped to [SERVO_PULSE_MIN_US,
 * SERVO_PULSE_MAX_US]. Updates the internal motion-state used by
 * servo_is_moving(). */
void servo_set_pulse_us(uint32_t us);

/* Set target angle in degrees at the ring gear. 0 = home.
 * Positive = clockwise (viewed from above). Clamped to
 * [SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG]. */
void servo_set_angle_deg(float angle_deg);

/* Current target angle (last commanded, after clamping). Returns the angle
 * at the driven element (gear output for JS6620, shaft/disc for MG90S direct
 * drive). For JS6620, raw servo horn angle = this × SERVO_GEAR_RATIO. */
float servo_get_angle_deg(void);

/* Boot-time sweep: 0 -> ±SERVO_BOOT_SWEEP_DEG -> 0 -> ∓SERVO_BOOT_SWEEP_DEG
 * -> 0, ~1200 ms dwell at each waypoint so the user can visually confirm the
 * home direction and range before tracking starts. Ramp runs at 100 deg/s
 * (3x slower than tracking) for visibility; tracking speed is restored after.
 * Call once after servo_init(), before starting the mic task. Total ~8.8 s. */
void servo_boot_sweep(void);

/* Override the smooth-ramp step size (deg per 20 ms timer tick). Default
 * is SERVO_SMOOTH_STEP_DEG (6 deg = 300 deg/s). Smaller = slower, more
 * visible motion. Clamped to [0.2, 10] — 0.2 deg/20ms = 10 deg/s, used
 * by the tracker's boundary-scan mode. */
void servo_set_smooth_step_deg(float deg_per_step);

/* True if the servo was commanded to a new target within the holdoff
 * window. The tracker uses this to freeze DOA updates during motion +
 * settle, so servo-motor whine doesn't corrupt GCC-PHAT.
 *
 * Phase B2 (2026-06-25): holdoff is now ADAPTIVE based on the magnitude
 * of the last commanded step:
 *   |delta| <  5°   →  200 ms
 *   |delta| < 15°   →  350 ms
 *   |delta| ≥ 15°   →  500 ms (this constant)
 * Small idle-return steps no longer trigger the full 500 ms pause, so
 * idle return rate matches the configured 2.5°/s instead of being
 * capped at ~0.5°/s by the holdoff. */
#define SERVO_MOTION_HOLDOFF_MS  300
bool servo_is_moving(void);
