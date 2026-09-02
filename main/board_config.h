/* board_config.h — compile-time board selection.
 *
 * Centralizes the 4 GPIO that differ between supported boards, plus two
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
// #define BOARD_GOOUUU_S3_CAM    1
// #define BOARD_WAVESHARE_S3_ZERO_M   1
#define BOARD_ESP32_S3_SUPERMINI    1

#if defined(BOARD_GOOUUU_S3_CAM)
    /* GOOUUU ESP32-S3-CAM + 3DMIC-291 + JS6620 (270° PWM servo, 15T/20T external gear) */
    #define MIC_CLK0_GPIO       1
    #define MIC_DAT0_GPIO       2
    #define MIC_CLK1_GPIO       14
    #define MIC_DAT1_GPIO       42
    #define SERVO_GPIO          38
    #define LED_GPIO            48

#elif defined(BOARD_WAVESHARE_S3_ZERO_M)
    /* Waveshare ESP32-S3-Zero + 3DMIC-291 + ZP10S (PWM mode, 15T/20T external gear) */
    #define MIC_CLK0_GPIO       1
    #define MIC_DAT0_GPIO       2
    #define MIC_CLK1_GPIO       3
    #define MIC_DAT1_GPIO       4
    #define SERVO_GPIO          5
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
    #define LED_GPIO            48   /* On-board LED, may not match */

    /* Static IP — pinned so external callers can always reach this device at
     * the same address regardless of which physical board is flashed (the
     * original "红太狼" board was 4T1VMF @ 192.168.1.110; this replacement
     * inherits that identity). Gateway/netmask validated against the host
     * network (192.168.1.1 / 24) on 2026-08-02. */
    #define USE_STATIC_IP       1
    #define STATIC_IP_ADDR      "192.168.1.110"
    #define STATIC_IP_GW        "192.168.1.1"
    #define STATIC_IP_NETMASK   "255.255.255.0"
    #define STATIC_IP_DNS       "192.168.1.1"

    /* GPIO 10-15 (silkscreen "8"-"13") are FREE — previously reserved for the
     * EPD e-ink experiment, removed 2026-09-02 (hardware never verified). */

#else
    #error "Pick a board in board_config.h"
#endif

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
 *       The original S3-CAM / S3-Zero builds. Feed-forward compensation is
 *       required in tracker.c to convert α_array → α_room; without it the
 *       closed-loop tracker oscillates as servo motion shifts the array.
 *
 *   0 = array is FIXED in the room; only the servo + payload (camera, laser,
 *       etc.) rotates. The SuperMini build. α_array IS α_room, so feed-forward
 *       is disabled entirely (not just sign-flipped).
 *
 * Default per board:
 *   GOOUUU S3-CAM, Waveshare S3-Zero: 1 (array-on-servo, original behaviour)
 *   ESP32-S3 SuperMini:               0 (fixed array, payload-only rotation)
 *
 * Override by commenting the #if/#else below if your physical setup differs. */
#if defined(BOARD_ESP32_S3_SUPERMINI)
    #define MIC_ARRAY_MOUNTED_ON_SERVO  0
#else
    #define MIC_ARRAY_MOUNTED_ON_SERVO  1
#endif

#endif /* BOARD_CONFIG_H */
