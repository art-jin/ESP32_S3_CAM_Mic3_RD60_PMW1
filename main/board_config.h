/* board_config.h — compile-time board selection.
 *
 * Centralizes the GPIO that differ between supported boards, plus two
 * orthogonal switches: SERVO_MODEL_* (mechanical/electrical params, see
 * servo.h) and MIC_ARRAY_MOUNTED_ON_SERVO (whether the 3DMIC-291 rotates
 * with the servo or is fixed in the room, see tracker.c). Other constants
 * (PWM timing, DOA geometry) live in their owning modules — they're
 * identical across boards given the same 3DMIC-291 orientation.
 *
 * To switch boards: uncomment exactly one #define below, rebuild, flash.
 * No sdkconfig change needed. */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* === Pick exactly one board === */
#define BOARD_GOOUUU_S3_CAM    1
// #define BOARD_WAVESHARE_S3_ZERO_M   1
// #define BOARD_ESP32_S3_SUPERMINI    1

#if defined(BOARD_GOOUUU_S3_CAM)
    /* GOOUUU ESP32-S3-CAM (WROOM-1 N16R8) + 3DMIC-291 + radar + EPD, used as
     * a plain ESP32-S3 board: camera, LCD and onboard LED are NOT connected.
     * Measured schematic facts (ESP32-S3CAM.pdf): camera occupies GPIO 4-13
     * and 15-18 (FPC must be detached), TF slot occupies 38/39/40 (CMD/CLK/
     * DATA, each with a 10K pull-up — never insert a card), strapping pins
     * 0/45/46 and the octal-PSRAM zone 35-37 stay unused.
     * Bench wiring recorded 2026-10-26 (wire colors in CLAUDE.md):
     * radar TX->9 (orange), radar RX<-10 (yellow), servo=38. */
    #define MIC_CLK0_GPIO       41
    #define MIC_DAT0_GPIO       42
    #define MIC_CLK1_GPIO       2
    #define MIC_DAT1_GPIO       1
    #define SERVO_GPIO          38   /* shares the SD_CMD net (10K pull-up) */
    #define RAD_TX_GPIO         10   /* ESP TX -> radar RX */
    #define RAD_RX_GPIO         9    /* ESP RX <- radar TX */
    #define LED_GPIO            -1   /* dropped for the CAM phase; 48 free */

#elif defined(BOARD_WAVESHARE_S3_ZERO_M)
    /* Waveshare ESP32-S3-Zero + 3DMIC-291 + ZP10S (PWM mode, 15T/20T external gear) */
    #define MIC_CLK0_GPIO       1
    #define MIC_DAT0_GPIO       2
    #define MIC_CLK1_GPIO       3
    #define MIC_DAT1_GPIO       4
    #define SERVO_GPIO          5
    #define RAD_TX_GPIO         8
    #define RAD_RX_GPIO         9
    #define LED_GPIO            48   /* On-board WS2812, may not light correctly */

#elif defined(BOARD_ESP32_S3_SUPERMINI)
    /* ESP32-S3 SuperMini + 3DMIC-291 + ZP10S (PWM mode, 15T/20T external gear)
     * Silkscreen numbers on this board are offset +2 from actual ESP32 GPIO
     * numbers (silkscreen "1" = GPIO3, "3" = GPIO5, "5" = GPIO7, etc.). */
    #define MIC_CLK0_GPIO       3    /* silkscreen "1" */
    #define MIC_DAT0_GPIO       4    /* silkscreen "2" */
    #define MIC_CLK1_GPIO       5    /* silkscreen "3" */
    #define MIC_DAT1_GPIO       6    /* silkscreen "4" */
    #define SERVO_GPIO          7    /* silkscreen "5" */
    #define RAD_TX_GPIO         8
    #define RAD_RX_GPIO         9
    #define LED_GPIO            48   /* On-board LED, may not match */

    /* GPIO 10-15 (silkscreen "8"-"13") are FREE — previously reserved for the
     * EPD e-ink experiment, removed 2026-09-02 (hardware never verified). */

#else
    #error "Pick a board in board_config.h"
#endif

#if !defined(RAD_TX_GPIO) || !defined(RAD_RX_GPIO)
    #error "Board block must define RAD_TX_GPIO / RAD_RX_GPIO"
#endif

/* Static IP — pinned so external callers can always reach this device at
 * the same address regardless of which physical board is flashed. The
 * per-robot address (redwolf=.110, heizai=.113) lives in the gitignored
 * wifi_creds.h next to the WiFi password — one local file per neck
 * deployment, flip the active block when flashing a different robot.
 * Gateway/netmask validated against the host network (192.168.1.1 / 24)
 * on 2026-08-02. */
#define USE_STATIC_IP       1

/* Dual-slot WiFi failover tuning (see tasks/wifi-dual-network.md).
 * RETRY_MAX: quick reconnect attempts on the SAME slot before rotating.
 *   6 (docs/52 P0-6 T8): a full router power cycle ≈ 10-15s and each
 *   NO_AP_FOUND attempt burns 2-3s — 3 retries rotated mid-bounce
 *   (measured 2026-10-22); 6 keeps ~15-20s bounces on the same slot.
 * SWITCH_DELAY_MS: pause after rotating to the next slot before
 * connecting (full no-network cycle ≈ 30s, avoids hammering the radio). */
#define WIFI_SLOT_RETRY_MAX       6
#define WIFI_SLOT_SWITCH_DELAY_MS 5000

/* === Pick exactly one servo model ===
 *
 * Decoupled from BOARD_* above: any board can drive any servo. The servo
 * model selects mechanical/electrical params (travel, gear ratio, direction
 * inversion, mechanical clamp, boot sweep range) in servo.h, and the
 * feed-forward sign in tracker.c.
 *
 * Default: JS6620 (the original S3-CAM configuration).
 * Switch to MG90S_DIRECT for the new S3-Zero + MG90S/SG90 direct-drive robot. */
// #define SERVO_MODEL_JS6620_EXTERNAL_GEAR   1
#define SERVO_MODEL_MG90S_DIRECT_DRIVE   1

#if !defined(SERVO_MODEL_JS6620_EXTERNAL_GEAR) && !defined(SERVO_MODEL_MG90S_DIRECT_DRIVE)
    #error "Pick a servo model in board_config.h"
#endif
#if defined(SERVO_MODEL_JS6620_EXTERNAL_GEAR) && defined(SERVO_MODEL_MG90S_DIRECT_DRIVE)
    #error "Pick only one servo model in board_config.h"
#endif

/* === Mic array mounting ===
 *
 * Decoupled from BOARD_* and SERVO_MODEL_*: any board can run in either mode.
 *
 *   1 = array rotates with the servo (mounted on the rotating element).
 *       Feed-forward compensation is required in tracker.c to convert
 *       α_array → α_room; without it the closed-loop tracker oscillates
 *       as servo motion shifts the array.
 *
 *   0 = array is FIXED in the room; only the servo + payload (radar,
 *       laser, etc.) rotates. α_array IS α_room, so feed-forward is
 *       disabled entirely (not just sign-flipped).
 *
 * Set explicitly (was per-board default): the 2026-10 CAM migration swaps
 * the brain board only — the physical rig is unchanged from the SuperMini
 * build (fixed array, payload-only rotation). Flip to 1 if a future rig
 * mounts the array on the rotating neck. */
#define MIC_ARRAY_MOUNTED_ON_SERVO  0

#endif /* BOARD_CONFIG_H */
