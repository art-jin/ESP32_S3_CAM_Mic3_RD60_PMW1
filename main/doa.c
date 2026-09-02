#include "doa.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { float re, im; } cplx_t;

/* Precomputed tables. */
static float s_hann[DOA_FFT_N];
static float s_tw_cos[DOA_FFT_N / 2];
static float s_tw_sin[DOA_FFT_N / 2];

/* Per-call scratch. doa_process is single-threaded (only the mic task calls
 * it), so static storage avoids blowing the task stack. */
static cplx_t s_A[DOA_FFT_N];
static cplx_t s_B[DOA_FFT_N];
static cplx_t s_G[DOA_FFT_N];

/* Median-smoothing history per pair: [pair][frame]. */
static float s_lag_hist[DOA_N_PAIRS][DOA_HIST_N];
static int   s_hist_head   = 0;
static int   s_hist_filled = 0;

void doa_init(void)
{
    for (int i = 0; i < DOA_FFT_N; i++) {
        s_hann[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (DOA_FFT_N - 1)));
    }
    for (int k = 0; k < DOA_FFT_N / 2; k++) {
        s_tw_cos[k] = cosf(-2.0f * (float)M_PI * k / DOA_FFT_N);
        s_tw_sin[k] = sinf(-2.0f * (float)M_PI * k / DOA_FFT_N);
    }
}

/* In-place iterative radix-2 Cooley-Tukey FFT, length DOA_FFT_N. */
static void fft_radix2(cplx_t *x)
{
    /* Bit-reversal permutation. */
    for (int i = 1, j = 0; i < DOA_FFT_N; i++) {
        int bit = DOA_FFT_N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { cplx_t t = x[i]; x[i] = x[j]; x[j] = t; }
    }
    /* Butterfly stages. */
    for (int len = 2; len <= DOA_FFT_N; len <<= 1) {
        int half = len >> 1;
        int step = DOA_FFT_N / len;
        for (int i = 0; i < DOA_FFT_N; i += len) {
            for (int k = 0; k < half; k++) {
                float c = s_tw_cos[k * step];
                float s = s_tw_sin[k * step];
                cplx_t u = x[i + k];
                cplx_t v;
                v.re = x[i + k + half].re * c - x[i + k + half].im * s;
                v.im = x[i + k + half].re * s + x[i + k + half].im * c;
                x[i + k].re = u.re + v.re;
                x[i + k].im = u.im + v.im;
                x[i + k + half].re = u.re - v.re;
                x[i + k + half].im = u.im - v.im;
            }
        }
    }
}

static void ifft_radix2(cplx_t *x)
{
    /* Conjugate, forward FFT, conjugate, scale by 1/N. */
    for (int i = 0; i < DOA_FFT_N; i++) x[i].im = -x[i].im;
    fft_radix2(x);
    float inv = 1.0f / DOA_FFT_N;
    for (int i = 0; i < DOA_FFT_N; i++) {
        x[i].re *= inv;
        x[i].im = -x[i].im * inv;
    }
}

/* GCC-PHAT between a and b. Returns lag (arrival(b) − arrival(a)) in samples
 * with sub-sample precision via parabolic interpolation. *peak_out gets the
 * GCC peak value in [-1, 1] — high when a sharp alignment exists, near 0 or
 * negative when noise dominates. */
static float gcc_phat(const int16_t *a, const int16_t *b, int n, float *peak_out)
{
    if (n > DOA_FFT_N) n = DOA_FFT_N;
    for (int i = 0; i < DOA_FFT_N; i++) {
        float va = (i < n) ? (float)a[i] * s_hann[i] : 0.0f;
        float vb = (i < n) ? (float)b[i] * s_hann[i] : 0.0f;
        s_A[i].re = va; s_A[i].im = 0;
        s_B[i].re = vb; s_B[i].im = 0;
    }
    fft_radix2(s_A);
    fft_radix2(s_B);
    /* Cross-spectrum G = A · conj(B), then PHAT weighting (divide by |·|). */
    for (int i = 0; i < DOA_FFT_N; i++) {
        float re = s_A[i].re * s_B[i].re + s_A[i].im * s_B[i].im;
        float im = s_A[i].im * s_B[i].re - s_A[i].re * s_B[i].im;
        float mag = sqrtf(re * re + im * im) + 1e-10f;
        s_G[i].re = re / mag;
        s_G[i].im = im / mag;
    }
    ifft_radix2(s_G);
    /* Find most-positive peak in ±DOA_MAX_LAG (NOT largest |R| — that would
     * match anti-correlation). */
    int   best_idx = 0;
    float best_val = -2.0f;
    for (int lag = -DOA_MAX_LAG; lag <= DOA_MAX_LAG; lag++) {
        int idx = (lag + DOA_FFT_N) % DOA_FFT_N;
        float v = s_G[idx].re;
        if (v > best_val) { best_val = v; best_idx = idx; }
    }
    int idx_p = (best_idx - 1 + DOA_FFT_N) % DOA_FFT_N;
    int idx_n = (best_idx + 1) % DOA_FFT_N;
    float y0 = s_G[idx_p].re, y1 = s_G[best_idx].re, y2 = s_G[idx_n].re;
    float denom = (y0 - 2.0f * y1 + y2);
    float delta = (denom != 0.0f) ? 0.5f * (y0 - y2) / denom : 0.0f;
    int lag_int = (best_idx > DOA_FFT_N / 2) ? best_idx - DOA_FFT_N : best_idx;
    *peak_out = best_val;
    return (float)lag_int + delta;
}

static float median_n(const float *h, int head, int filled)
{
    int n = filled < DOA_HIST_N ? filled : DOA_HIST_N;
    if (n == 0) return 0.0f;
    float tmp[DOA_HIST_N];
    for (int i = 0; i < n; i++) {
        tmp[i] = h[(head - n + 1 + i + DOA_HIST_N) % DOA_HIST_N];
    }
    /* Insertion sort (n is tiny). */
    for (int i = 1; i < n; i++) {
        float v = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = v;
    }
    return tmp[n / 2];
}

static int azimuth_to_sextant(float az)
{
    while (az <    0.0f) az += 360.0f;
    while (az >= 360.0f) az -= 360.0f;
    int s = (int)lroundf(az / 60.0f);
    if (s >= 6) s = 0;
    return s;
}

/* Per-sextant azimuth calibration offsets (Phase A).
 *
 * Values are the NEGATIVE of measured bias — i.e., what we ADD to a raw
 * azimuth to remove systematic error. Updated 2026-06-25 after a fresh
 * 6-position whistle calibration run (tracker disabled, servo at 0°):
 *
 *   sextant  meaning           raw meas  true   raw err   correction
 *   0        12oc (α=0°)       357.3°    360°   -2.7°     +3.0
 *   1        2oc  (α=60°)       66.3°     60°   +6.3°     -6.0   (s=1 unreliable — see note)
 *   2        4oc  (α=120°)    117.3°    120°   -2.7°     +3.0
 *   3        6oc  (α=180°)    179.8°    180°   -0.2°      0.0
 *   4        8oc  (α=240°)    239.3°    240°   -0.7°     +1.0
 *   5        10oc (α=300°)    295.3°    300°   -4.7°     +5.0
 *
 * Note on s=1: the v1.3 (2026-06-22) measurement at 3oc (α=90°, also s=1)
 * showed raw 83.9° → -6.1° bias, OPPOSITE sign to today's 2oc reading.
 * This means the bias within s=1 is not monotonic; a single offset
 * cannot fully correct it. Today's value favors 2oc since 3oc is the
 * s=1/s=2 boundary and effectively shared. Future work: geometric
 * calibration (Phase C, Levenberg-Marquardt) to replace per-sextant table.
 *
 * Day-to-day variation: v1.3 had 12oc at +6.6° bias, today -2.7°.
 * ~5° swing. Treat this table as "best single-day estimate," not absolute.
 *
 * 9oc (α=270°) excluded — array's geometric blind spot (M1 shadowed by
 * PCB), reported via 2-mic fallback, not 3-mic. */
static const float s_sextant_offset[6] = {
    +3.0f,   /* s=0  12oc */
    -6.0f,   /* s=1  2oc  */
    +3.0f,   /* s=2  4oc  */
     0.0f,   /* s=3  6oc  */
    +1.0f,   /* s=4  8oc  */
    +5.0f,   /* s=5  10oc */
};

/* ---- Output hysteresis state ----
 * Require DOA_STABLE_STREAK consecutive identical raw sextant readings
 * before stable_sextant updates. Suppresses per-frame jitter. */
static int s_stable_sextant = -1;
static int s_pending_sextant = -1;
static int s_pending_streak  = 0;

static void update_stable_sextant(doa_result_t *out)
{
    out->stable_sextant = s_stable_sextant;
    int raw = out->sextant;
    if (raw < 0) {
        /* Invalid frame — don't advance the streak, but don't reset it
         * either. A few invalid frames in a row shouldn't clear a
         * previously-stable reading. */
        return;
    }
    if (raw == s_pending_sextant) {
        if (s_pending_streak < DOA_STABLE_STREAK) s_pending_streak++;
    } else {
        s_pending_sextant = raw;
        s_pending_streak  = 1;
    }
    if (s_pending_streak >= DOA_STABLE_STREAK) {
        s_stable_sextant = s_pending_sextant;
    }
}

static float compute_ac_rms(const int16_t *x, int n)
{
    if (n == 0) return 0.0f;
    int64_t sum = 0;
    for (int i = 0; i < n; i++) sum += x[i];
    float mean = (float)sum / n;
    float acc = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = (float)x[i] - mean;
        acc += d * d;
    }
    return sqrtf(acc / n);
}

void doa_process(const int16_t *c0, const int16_t *c1, const int16_t *c2,
                 size_t n_in, doa_result_t *out)
{
    memset(out, 0, sizeof(*out));
    int n = (int)n_in;
    if (n < 64) { out->mode = DOA_MODE_INVALID; return; }

    /* ---- Detect port-0 L/R slot collapse ----
     * Two complementary metrics:
     *   1. max |c0 - c1| scaled by AC RMS of c0. Independent mics hearing the
     *      same source differ by O(RMS) per sample (TDOA-induced); collapsed
     *      mics differ only by PDM2PCM quantization noise (< RMS/3).
     *      → threshold: max_diff > max(25, 0.5*RMS_c0).
     *   2. normalized cross-correlation coef ρ(c0,c1). Collapsed mics give
     *      ρ > 0.95 at zero lag; independent mics give ρ < 0.9.
     * Independent iff both metrics agree. */
    int   max_diff = 0;
    int   same     = 0;
    int64_t c0_sum = 0, c1_sum = 0;
    int64_t c0_sq_sum = 0, c1_sq_sum = 0;
    int64_t cross_sum = 0;
    for (int i = 0; i < n; i++) {
        int a = c0[i];
        int b = c1[i];
        int d = a - b;
        if (d < 0) d = -d;
        if (d > max_diff) max_diff = d;
        if (d == 0) same++;
        c0_sum += a;
        c1_sum += b;
        cross_sum += (int64_t)a * b;
        c0_sq_sum += (int64_t)a * a;
        c1_sq_sum += (int64_t)b * b;
    }
    float c0_mean = (float)c0_sum / n;
    float c1_mean = (float)c1_sum / n;
    /* AC RMS (DC-removed) and Pearson correlation. */
    float c0_var = (float)c0_sq_sum / n - c0_mean * c0_mean;
    float c1_var = (float)c1_sq_sum / n - c1_mean * c1_mean;
    float cov    = (float)cross_sum / n - c0_mean * c1_mean;
    float c0_ac_rms = (c0_var > 0.0f) ? sqrtf(c0_var) : 0.0f;
    float c1_ac_rms = (c1_var > 0.0f) ? sqrtf(c1_var) : 0.0f;

    /* VAD gate: if all channels are below noise floor, skip DOA entirely.
     * Saves ~15ms CPU (3× GCC-PHAT) and prevents noise-driven false DOA. */
    float c2_ac_rms = compute_ac_rms(c2, n);
    if (c0_ac_rms < 25.0f && c1_ac_rms < 25.0f && c2_ac_rms < 25.0f) {
        out->mode = DOA_MODE_INVALID;
        out->sextant = -1;
        update_stable_sextant(out);
        return;
    }

    float corr = 0.0f;
    if (c0_var > 1.0f && c1_var > 1.0f) {
        corr = cov / sqrtf(c0_var * c1_var);
        if (corr >  1.0f) corr =  1.0f;
        if (corr < -1.0f) corr = -1.0f;
    }
    int same_pct = (same * 100) / n;
    /* Independent iff (signal-proportional diff is large) AND (low correlation). */
    int diff_thresh = 25;
    int rms_thresh  = (int)(c0_ac_rms * 0.5f);
    if (rms_thresh > diff_thresh) diff_thresh = rms_thresh;
    int lr_independent = (same_pct < 95) && (max_diff > diff_thresh) && (corr < 0.90f);
    out->lr_independent = lr_independent;
    out->lr_max_diff    = max_diff;
    out->lr_same_pct    = same_pct;
    out->lr_corr        = corr;

    /* ---- Pairwise GCC-PHAT ---- */
    float peak_01 = 0, peak_02 = 0, peak_12 = 0;
    float lag_01  = gcc_phat(c0, c1, n, &peak_01);   /* M1 − M2 */
    float lag_02  = gcc_phat(c0, c2, n, &peak_02);   /* M3 − M2 */
    float lag_12  = gcc_phat(c1, c2, n, &peak_12);   /* M3 − M1 */

    out->lag_pair_raw[0] = lag_01;
    out->lag_pair_raw[1] = lag_02;
    out->lag_pair_raw[2] = lag_12;
    out->peak_pair[0] = peak_01;
    out->peak_pair[1] = peak_02;
    out->peak_pair[2] = peak_12;

    /* ---- Median smoothing across the last DOA_HIST_N frames ----
     * Single-frame GCC outliers (peak jumps to a harmonic) are common; the
     * median kills them without introducing the lag of a moving average. */
    s_lag_hist[0][s_hist_head] = lag_01;
    s_lag_hist[1][s_hist_head] = lag_02;
    s_lag_hist[2][s_hist_head] = lag_12;
    s_hist_head = (s_hist_head + 1) % DOA_HIST_N;
    if (s_hist_filled < DOA_HIST_N) s_hist_filled++;

    float sm_01 = median_n(s_lag_hist[0], s_hist_head, s_hist_filled);
    float sm_02 = median_n(s_lag_hist[1], s_hist_head, s_hist_filled);
    float sm_12 = median_n(s_lag_hist[2], s_hist_head, s_hist_filled);
    out->lag_pair[0] = sm_01;
    out->lag_pair[1] = sm_02;
    out->lag_pair[2] = sm_12;

    /* ---- Confidence: mean of GCC peaks, skipping M1-pairs if collapsed ---- */
    float conf;
    if (lr_independent) {
        conf = (peak_01 + peak_02 + peak_12) / 3.0f;
    } else {
        conf = peak_02;   /* only the M2↔M3 pair carries real signal */
    }
    if (conf < 0.0f) conf = 0.0f;
    if (conf > 1.0f) conf = 1.0f;
    out->confidence = conf;

    /* Mic validity mask: bit0=M2, bit1=M1, bit2=M3. */
    out->mics_valid_mask = 0x01;             /* M2 (c0) always present */
    if (lr_independent) out->mics_valid_mask |= 0x02;
    out->mics_valid_mask |= 0x04;            /* M3 (c2) always present */

    /* Reject silent / noise-only frames. */
    if (conf < 0.15f) {
        out->mode = DOA_MODE_INVALID;
        out->sextant = -1;
        update_stable_sextant(out);
        return;
    }

    /* Gate 3-mic solve on per-pair GCC sharpness. Even with low ρ01
     * (so the L/R detector says "independent"), pure-noise frames give
     * weak GCC peaks that produce spurious azimuths. Require both pairs
     * that the 3-mic math actually uses (peak_01 and peak_02) to be
     * sharp. At extreme angles (3oc/9oc), one mic has weak signal so
     * one peak may be low — require at least ONE to pass (OR not AND).
     * Downstream EMA + plausibility check filters any noise that leaks
     * through with a single-good-peak frame. */
    int three_mic_eligible = lr_independent
                          && ((peak_01 >= DOA_PEAK_THRESH_3MIC)
                           || (peak_02 >= DOA_PEAK_THRESH_3MIC));

    if (three_mic_eligible) {
        /* ---- 3-mic solve (c0=M2@10oc, c1=M1@2oc, c2=M3@6oc) ----
         *   sin α = +lag_01 / K
         *   cos α = (lag_01 - 2·lag_02) / (K · √3)
         * (Sign convention: gcc_phat(a,b) returns arrival(a) − arrival(b).
         * Geometry is mirrored across the 12oc-6oc axis because the 3DMIC-291
         * PCB is installed component-side down — M1 and M2 swap clock positions
         * relative to the silkscreen, M3 stays at 6oc.)
         * For ideal far-field, sin² + cos² ≤ 1. If much greater, the three
         * pairwise measurements are inconsistent (noise / multi-source).
         * Normalize and penalize confidence; reject if wildly off. */
        float sin_a = sm_01 / DOA_K;
        float cos_a = (sm_01 - 2.0f * sm_02) / (DOA_K * sqrtf(3.0f));
        float mag2 = sin_a * sin_a + cos_a * cos_a;
        if (mag2 >= 4.0f) {
            out->mode = DOA_MODE_INVALID;
            out->sextant = -1;
            return;
        }
        if (mag2 > 1.0f) {
            float mag = sqrtf(mag2);
            sin_a /= mag;
            cos_a /= mag;
            out->confidence *= 1.0f / mag2;
        }
        float alpha_rad = atan2f(sin_a, cos_a);
        float alpha_deg = alpha_rad * 180.0f / (float)M_PI;
        if (alpha_deg < 0.0f) alpha_deg += 360.0f;

        /* Apply per-sextant calibration correction (Phase A).
         * Look up by the RAW sextant (pre-correction) so the offset matches
         * the geometry that produced it. Correction magnitudes are small
         * (<10°) and the boundaries are 30°/90°/150°... so a correction
         * never pushes the result across a sextant boundary except at the
         * 90° case (3oc, α=89.9° post-correction stays s=1 by lroundf). */
        int raw_sextant = azimuth_to_sextant(alpha_deg);
        alpha_deg += s_sextant_offset[raw_sextant];
        if (alpha_deg <    0.0f) alpha_deg += 360.0f;
        if (alpha_deg >= 360.0f) alpha_deg -= 360.0f;

        out->azimuth_deg = alpha_deg;
        out->sextant     = azimuth_to_sextant(alpha_deg);
        out->mode        = DOA_MODE_3MIC;
        update_stable_sextant(out);
    } else if (peak_02 >= DOA_PEAK_THRESH_2MIC) {
        /* ---- 2-mic solve (c0=M2@10oc, c2=M3@6oc only) ----
         * lag_02 = +K · sin(α − 60°). Linear baseline M2↔M3, front/back
         * ambiguous. Map lag_02 ∈ [-K, +K] linearly to θ ∈ [0°, 180°]:
         *   θ = 0°    source on M2 side       (lag_02 = -K, α ≈ 330° = 10-11oc)
         *   θ = 90°   broadside                (lag_02 = 0,  α ≈ 60° / 240°)
         *   θ = 180°  source on M3 side        (lag_02 = +K, α ≈ 150° = 4-5oc) */
        float ratio = sm_02 / DOA_K;
        if (ratio >  1.0f) ratio =  1.0f;
        if (ratio < -1.0f) ratio = -1.0f;
        float half_deg = (1.0f + ratio) * 90.0f;   /* [0°, 180°] */
        out->azimuth_deg = half_deg;
        out->sextant     = (int)(half_deg / 60.0f);
        if (out->sextant > 2) out->sextant = 2;
        out->mode        = DOA_MODE_2MIC;
        /* Don't update stable_sextant from 2-mic mode: its 0..2 bin scale
         * differs from the 3-mic 0..5 sextant scale and would corrupt
         * the hysteresis state. The previously-stable 3-mic sextant (if
         * any) is preserved in out->stable_sextant by update_stable_sextant
         * being a no-op for streak advancement when raw<0 — so we set
         * sextant to -1 here purely for the hysteresis call, then restore. */
        {
            int saved = out->sextant;
            out->sextant = -1;
            update_stable_sextant(out);
            out->sextant = saved;
        }
    } else {
        /* Even the M2↔M3 pair has no usable peak — pure noise. */
        out->mode = DOA_MODE_INVALID;
        out->sextant = -1;
        update_stable_sextant(out);
    }
}

const char *doa_sextant_label(int sextant, doa_mode_t mode)
{
    static const char *clock6[6] = {
        "12 o'clock", " 2 o'clock", " 4 o'clock",
        " 6 o'clock", " 8 o'clock", "10 o'clock",
    };
    static const char *half3[3] = {
        "M2 side", "broadside (12oc/6oc ambig)", "M3 side",
    };
    if (mode == DOA_MODE_3MIC) {
        if (sextant < 0 || sextant >= 6) return "?";
        return clock6[sextant];
    }
    if (mode == DOA_MODE_2MIC) {
        if (sextant < 0 || sextant >= 3) return "?";
        return half3[sextant];
    }
    return "--";
}
