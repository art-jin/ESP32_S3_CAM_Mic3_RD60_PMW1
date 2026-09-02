#pragma once

#include <stddef.h>
#include <stdint.h>

/* GCC-PHAT direction-of-arrival for the 3DMIC-291 3-mic array.
 *
 * Geometry (equilateral triangle, side d = 10 mm, centred at origin).
 *
 * The 3DMIC-291 PCB is installed **component-side down, sound holes up**
 * (flipped relative to the silkscreen's intended orientation). This mirrors
 * the layout across the 12oc-6oc axis: M1 and M2 swap clock positions
 * compared to the silkscreen, M3 stays at 6oc.
 *
 *     M2 (c0) @ 10oc        12oc (north, α=0°)
 *          ●
 *         /   \                  ↗ 0°
 *        /     \              ↑
 *       /       \             |
 *      ●─────────●            +────→ east
 *   M3 (c2) @ 6oc    M1 (c1) @ 2oc
 *
 * Channel → physical mic mapping (tap-test verified 2026-06-21; the channel
 * wiring is independent of board orientation, only the positions change):
 *   c0 (DAT0 L-slot) = M2 @ 10oc → (-d/2,   +d/(2√3))
 *   c1 (DAT0 R-slot) = M1 @ 2oc  → (+d/2,   +d/(2√3))
 *   c2 (DAT1)        = M3 @ 6oc  → (0,      -d/√3)
 *
 * Pairwise TDOA (gcc_phat(a,b) returns arrival(a) − arrival(b), in samples;
 * K = d·fs/c ≈ 1.40 at 48 kHz):
 *   lag_01 = gcc_phat(c0,c1) = arrival(M2) − arrival(M1) = +K · sin(α)
 *   lag_02 = gcc_phat(c0,c2) = arrival(M2) − arrival(M3) = +K · sin(α − 60°)
 *   lag_12 = gcc_phat(c1,c2) = arrival(M1) − arrival(M3) = −K · sin(α + 60°)
 *
 * 3-mic solve:
 *   sin α = +lag_01 / K
 *   cos α = (lag_01 − 2 · lag_02) / (K · √3)
 *   α = atan2(sin α, cos α)
 *
 * 2-mic fallback (only c0=M2 + c2=M3 usable, DAT0 L/R collapsed):
 *   lag_02 = +K · sin(α − 60°). Linear baseline M2↔M3, front/back ambiguous.
 *   Half-plane bearing θ ∈ [0°, 180°]:
 *     θ = 0°    source on M2 side       (lag_02 = −K, α ≈ 330° = 10-11oc)
 *     θ = 90°   broadside                (lag_02 = 0,  α ≈ 60° / 240°)
 *     θ = 180°  source on M3 side        (lag_02 = +K, α ≈ 150° = 4-5oc)
 *   Quantized to 3 bins of 60° each. */

#define DOA_FS_HZ            48000
#define DOA_MIC_SPACING_MM   10.0f
#define DOA_SPEED_OF_SOUND   343.0f

/* K = d · fs / c — max pairwise TDOA in samples for this geometry. */
#define DOA_K                (DOA_MIC_SPACING_MM * 1e-3f * DOA_FS_HZ / DOA_SPEED_OF_SOUND)

/* FFT size — must be a power of two and ≤ the per-channel sample count
 * passed to doa_process. At 48 kHz, 1024 points = 21.3 ms window — long
 * enough to lock onto voice/whistle through ambient noise. */
#define DOA_FFT_N            1024

/* TDOA search range. Physical max is K≈1.4 samples; ±8 gives the parabolic
 * interpolation room and tolerates small noise spikes. */
#define DOA_MAX_LAG          8

#define DOA_N_PAIRS          3   /* 31, 32, 12 */
/* Median-smoothing window, in frames. Was 5 (500 ms @ 10 Hz); reduced to 3
 * (300 ms) in Phase A to lower response latency. Feed-forward compensation
 * in the tracker provides system-level stability, so per-frame lag smoothing
 * can be shorter without risking oscillation. */
#define DOA_HIST_N           3

typedef enum {
    DOA_MODE_INVALID = 0,
    DOA_MODE_3MIC,        /* full 360° localization, 6 sextants */
    DOA_MODE_2MIC,        /* front/back-ambiguous half-plane, 3 bins */
} doa_mode_t;

typedef struct {
    doa_mode_t mode;
    float azimuth_deg;                  /* 0..360 (3-mic) or 0..180 (2-mic) */
    int   sextant;                      /* 0..5 (3-mic) or 0..2 (2-mic), -1 if invalid */
    int   stable_sextant;               /* hysteresis-smoothed sextant; requires
                                         * DOA_STABLE_STREAK consecutive identical
                                         * raw sextant readings before updating.
                                         * Suppresses per-frame jitter. -1 if no
                                         * stable reading yet. */
    float confidence;                   /* 0..1, mean GCC-PHAT peak height */
    float peak_pair[DOA_N_PAIRS];       /* per-pair GCC-PHAT peak heights [01,02,12] */
    float lag_pair[DOA_N_PAIRS];        /* smoothed TDOAs [01, 02, 12], samples */
    float lag_pair_raw[DOA_N_PAIRS];    /* latest single-frame TDOAs, for debug */
    int   mics_valid_mask;              /* bit0=M2, bit1=M1, bit2=M3 */
    int   lr_independent;               /* 1 if port-0 L/R slots differ, 0 if collapsed */
    int   lr_max_diff;                  /* max |c0 - c1| over the window, for debug */
    int   lr_same_pct;                  /* % of samples where c0 == c1, for debug */
    float lr_corr;                      /* normalized cross-corr coef of c0,c1 in [-1,1] */
} doa_result_t;

/* Hysteresis: require this many consecutive identical sextant readings
 * before stable_sextant updates. 3 frames @ 2 Hz print = 1.5 s settle time. */
#define DOA_STABLE_STREAK   3

/* GCC-PHAT peak thresholds for accepting a frame.
 *   - 3-mic solve requires BOTH peak_01 (M1↔M2) and peak_02 (M3↔M2) sharp,
 *     since the 3-mic math uses both.
 *   - 2-mic fallback only uses peak_02.
 * Below these the GCC "peak" is most likely a noise artifact. */
#define DOA_PEAK_THRESH_3MIC  0.35f
#define DOA_PEAK_THRESH_2MIC  0.30f

/* One-time init: precompute Hann window + twiddle factors. */
void doa_init(void);

/* Run one DOA pass on deinterleaved PCM.
 *   c0 = LINE0_L  (DAT0 left slot)  → M3 candidate
 *   c1 = LINE0_R  (DAT0 right slot) → M1 candidate (or duplicate of c0)
 *   c2 = LINE1_L  (DAT1)            → M2 candidate
 * n = samples per channel (will use min(n, DOA_FFT_N)). */
void doa_process(const int16_t *c0, const int16_t *c1, const int16_t *c2,
                 size_t n, doa_result_t *out);

/* "12 o'clock", "2 o'clock", ... "10 o'clock" for 3-mic sextants.
 * "M2 side", "broadside (front/back ambig)", "M3 side" for 2-mic bins. */
const char *doa_sextant_label(int sextant, doa_mode_t mode);
