/* MS60-1211S80M radar driver — Phase 1.
 *
 * Link: UART1 115200 8N1, radar RX <- GPIO8 (our TX), radar TX -> GPIO9
 * (our RX). The AT6010 protocol doc claims a 921600 default; this module's
 * firmware actually ships 115200 (measured, tasks/radar-protocol-notes.md).
 *
 * Data path: poll 3.2.6 (cmd 0x30) at 5 Hz. Every reply carries the full
 * fmcw_det_info_t. No-target signature: det_result=0, range=0, conf=0
 * (is_detected stays 1 — it means "frame valid", not "target present").
 *
 * Sensing is switched on at every boot (58 D1 01 01 2B 01) instead of being
 * saved to the radar's flash once: same end state, zero flash wear.
 *
 * Offline detection: 3 consecutive poll cycles (~600 ms) without any valid
 * 0x59 reply -> offline. Radar is then ignored until replies resume.
 */

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"

#include "radar.h"
#include "events.h"
#include "mode_manager.h"

#define RAD_UART_NUM    UART_NUM_1
#define RAD_BAUD        115200
#define RAD_TX_GPIO     8
#define RAD_RX_GPIO     9
#define RAD_RX_BUF      4096
#define RAD_ACC_MAX     256

#define RAD_POLL_MS     200      /* 5 Hz */
#define RAD_OFFLINE_MISS 3       /* consecutive missed polls -> offline */
#define RAD_INIT_RETRIES 3

static const char *TAG = "radar";

static const uint8_t VER_REQ[5]  = {0x58, 0xFE, 0x00, 0x56, 0x01};
static const uint8_t SENS_Q[5]   = {0x58, 0xD0, 0x00, 0x28, 0x01};  /* 3.2.2 sensing status */
/* 3.1.11 波特率切换 -> 115200 (u32 LE). The module can revert to its
 * factory-default 921600 baud (observed after config-flash writes +
 * overnight power cycle) — the driver self-heals this at bring-up. */
static const uint8_t BAUD_115200[9] = {0x58, 0x19, 0x04, 0x00, 0xC2, 0x01, 0x00, 0x38, 0x01};
static const uint8_t SENS_ON[6]  = {0x58, 0xD1, 0x01, 0x01, 0x2B, 0x01};
static const uint8_t DET_Q[5]    = {0x58, 0x30, 0x00, 0x88, 0x00};
/* 3.2.16 breath min distance = 80 (cm) — gates the static ~0.6m desk echo
 * that the breath detector keeps reporting as a phantom presence target.
 * 3.2.10 motion min distance = 50 (cm) trims extreme near-field noise.
 * NB: the micro-motion min-distance command (0x37) is marked ineffective
 * in this firmware version, which is why we gate on breath instead. */
static const uint8_t BR_MIN[7]   = {0x58, 0x3A, 0x02, 0x50, 0x00, 0xE4, 0x00};
static const uint8_t MO_MIN[7]   = {0x58, 0x34, 0x02, 0x32, 0x00, 0xC0, 0x00};
/* 3.2.7 algorithm boundaries (factory, read-only) / 3.2.8 user config. */
static const uint8_t Q_BOUND[5]  = {0x58, 0x32, 0x00, 0x8A, 0x00};
static const uint8_t Q_CFG[5]    = {0x58, 0x33, 0x00, 0x8B, 0x00};
/* 3.1.5 save settings + 3.1.9 system reset. Distance config only takes
 * effect after save + reload on this firmware (verified empirically:
 * 0x34 motion-min reads back live, 0x3A breath-min reads back 0 until
 * save+reset). Saving once per boot is negligible flash wear. */
static const uint8_t SAVE[6]     = {0x58, 0x08, 0x01, 0x01, 0x62, 0x00};
static const uint8_t SYSRST[6]   = {0x58, 0x13, 0x01, 0x01, 0x6D, 0x00};

static SemaphoreHandle_t s_lock;
static radar_target_t s_target;         /* guarded by s_lock */
static volatile bool s_online;
static bool s_ver_ok;

/* rx accumulator + counters (task-local, no locking needed) */
static uint8_t s_acc[RAD_ACC_MAX];
static size_t s_len;
static uint32_t c_rx, c_reply, c_30, c_bad_ck, c_stray, c_miss;

static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int16_t  rd_s16(const uint8_t *p) { return (int16_t)rd_u16(p); }
static uint32_t rd_u32(const uint8_t *p) { return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24)); }

/* Speech-activity flag: set by fusion (mic_task) on each accepted DOA,
 * consumed by the still-alarm check in the radar task. Talking counts
 * as substantial activity for the care alarm. */
static volatile bool s_sound_activity;

void radar_notify_sound(void)
{
    s_sound_activity = true;
}

/* ---- Configuration request mailbox (US-010) ----
 * REST handlers post a request; the radar task (sole UART owner)
 * executes it inside its poll loop and signals completion. */
typedef enum {
    RAD_REQ_NONE = 0,
    RAD_REQ_GET_CFG,
    RAD_REQ_SET_CFG,
    RAD_REQ_RESET,
} rad_req_type_t;

typedef struct {
    rad_req_type_t type;
    radar_set_req_t set;
    radar_cfg_t cfg;        /* filled by the radar task */
    bool ok;
} rad_req_t;

static rad_req_t s_req;                     /* guarded by s_req_lock */
static SemaphoreHandle_t s_req_lock;
static SemaphoreHandle_t s_req_done;        /* binary, given on completion */
static volatile bool s_req_pending;

static bool service_link(void);             /* fwd decls: defined below */
static void hist_reset(void);

/* Reply capture targets: when set, handle_reply copies matching payloads */
static volatile uint8_t  s_capture_cmd;     /* 0x32 / 0x33 / 0xD0 */
static radar_cfg_t      *s_capture_dst;
static volatile bool     s_got_bounds, s_got_cfg, s_got_sensing;

static bool req_post(rad_req_type_t type, const radar_set_req_t *set,
                     radar_cfg_t *out, uint32_t timeout_ms)
{
    if (!s_req_lock) return false;
    if (xSemaphoreTake(s_req_lock, pdMS_TO_TICKS(200)) != pdTRUE) return false;
    if (s_req_pending) { xSemaphoreGive(s_req_lock); return false; }  /* busy */
    s_req.type = type;
    s_req.ok = false;
    if (set) s_req.set = *set;
    memset(&s_req.cfg, 0, sizeof(s_req.cfg));
    s_req_pending = true;
    xSemaphoreGive(s_req_lock);

    xSemaphoreTake(s_req_done, 0);          /* clear stale */
    /* Wake the radar task promptly: it polls every <=20 ms anyway. */
    if (xSemaphoreTake(s_req_done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        /* timeout — reclaim the slot so future requests can proceed */
        xSemaphoreTake(s_req_lock, portMAX_DELAY);
        s_req_pending = false;
        xSemaphoreGive(s_req_lock);
        return false;
    }
    xSemaphoreTake(s_req_lock, portMAX_DELAY);
    bool ok = s_req.ok;
    if (out) *out = s_req.cfg;
    s_req_pending = false;
    xSemaphoreGive(s_req_lock);
    return ok;
}

bool radar_req_get_cfg(radar_cfg_t *out, uint32_t timeout_ms)
{
    return req_post(RAD_REQ_GET_CFG, NULL, out, timeout_ms);
}

bool radar_req_set_cfg(const radar_set_req_t *req, radar_cfg_t *out,
                       uint32_t timeout_ms)
{
    return req_post(RAD_REQ_SET_CFG, req, out, timeout_ms);
}

bool radar_req_reset(uint32_t timeout_ms)
{
    return req_post(RAD_REQ_RESET, NULL, NULL, timeout_ms);
}

/* Build a host command frame: 58 CMD PLEN params... CK16(LE). */
static size_t mk_cmd(uint8_t *buf, uint8_t cmd, const uint8_t *p, uint8_t plen)
{
    buf[0] = 0x58; buf[1] = cmd; buf[2] = plen;
    if (plen) memcpy(buf + 3, p, plen);
    uint16_t sum = 0;
    for (size_t i = 0; i < 3u + plen; i++) sum += buf[i];
    buf[3 + plen] = (uint8_t)(sum & 0xFF);
    buf[4 + plen] = (uint8_t)(sum >> 8);
    return 5u + plen;
}

static void send_u16_cmd(uint8_t cmd, uint16_t val)
{
    uint8_t b[8], p[2] = {(uint8_t)(val & 0xFF), (uint8_t)(val >> 8)};
    uart_write_bytes(RAD_UART_NUM, b, mk_cmd(b, cmd, p, 2));
}

static void send_u8_cmd(uint8_t cmd, uint8_t val)
{
    uint8_t b[8], p[1] = {val};
    uart_write_bytes(RAD_UART_NUM, b, mk_cmd(b, cmd, p, 1));
}

/* Pump service_link until cond becomes true or timeout. Runs in the
 * radar task only. */
static bool wait_for(volatile bool *cond, uint32_t timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        service_link();
        if (*cond) return true;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return false;
}

static bool query_cfg_into(radar_cfg_t *c)
{
    s_capture_dst = c;
    s_got_bounds = s_got_cfg = s_got_sensing = false;
    s_capture_cmd = 0x32;
    uart_write_bytes(RAD_UART_NUM, Q_BOUND, sizeof(Q_BOUND));
    bool b1 = wait_for(&s_got_bounds, 400);
    s_capture_cmd = 0x33;
    uart_write_bytes(RAD_UART_NUM, Q_CFG, sizeof(Q_CFG));
    bool b2 = wait_for(&s_got_cfg, 400);
    s_capture_cmd = 0xD0;
    uart_write_bytes(RAD_UART_NUM, SENS_Q, sizeof(SENS_Q));
    bool b3 = wait_for(&s_got_sensing, 400);
    s_capture_cmd = 0;
    c->online = s_online && (b1 || b2);
    return b1 && b2 && b3;
}

static void exec_set_cfg(const radar_set_req_t *r)
{
    if (r->mask & RAD_SET_SENSING) send_u8_cmd(0xD1, r->sensing ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    if (r->mask & RAD_SET_MOT_MIN) send_u16_cmd(0x34, r->mot_min);
    if (r->mask & RAD_SET_MOT_MAX) send_u16_cmd(0xD2, r->mot_max);
    if (r->mask & RAD_SET_MOT_LVL) send_u8_cmd(0x35, (uint8_t)r->mot_lvl);
    if (r->mask & RAD_SET_BHR_MIN) send_u16_cmd(0x3A, r->bhr_min);
    if (r->mask & RAD_SET_BHR_MAX) send_u16_cmd(0x39, r->bhr_max);
    if (r->mask & RAD_SET_BHR_LVL) send_u8_cmd(0x3B, (uint8_t)r->bhr_lvl);
    vTaskDelay(pdMS_TO_TICKS(120));
    if (r->mask & RAD_SET_SAVE) {
        uint8_t b[8];
        uart_write_bytes(RAD_UART_NUM, b, mk_cmd(b, 0x08, (const uint8_t *)"\x01", 1));
        vTaskDelay(pdMS_TO_TICKS(1200));   /* doc: wait >=1 s after save */
    }
}

static void exec_request(void)
{
    switch (s_req.type) {
    case RAD_REQ_GET_CFG:
        s_req.ok = query_cfg_into(&s_req.cfg);
        break;
    case RAD_REQ_SET_CFG:
        exec_set_cfg(&s_req.set);
        s_req.ok = query_cfg_into(&s_req.cfg);
        break;
    case RAD_REQ_RESET:
        /* Ack immediately (queue semantics): the reset sequence itself
         * runs ~5 s, longer than any sane HTTP timeout. */
        s_req.ok = true;
        xSemaphoreGive(s_req_done);
        /* Reset to the module's stored config, then re-apply our
         * standard init (sensing on + distance gates). */
        uart_write_bytes(RAD_UART_NUM, SYSRST, sizeof(SYSRST));
        vTaskDelay(pdMS_TO_TICKS(4000));
        uart_write_bytes(RAD_UART_NUM, SENS_ON, sizeof(SENS_ON));
        vTaskDelay(pdMS_TO_TICKS(200));
        uart_write_bytes(RAD_UART_NUM, BR_MIN, sizeof(BR_MIN));
        vTaskDelay(pdMS_TO_TICKS(200));
        uart_write_bytes(RAD_UART_NUM, MO_MIN, sizeof(MO_MIN));
        vTaskDelay(pdMS_TO_TICKS(200));
        hist_reset();
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_target = (radar_target_t){0};
        xSemaphoreGive(s_lock);
        return;   /* done semaphore already given */
    default:
        break;
    }
    xSemaphoreGive(s_req_done);
}

static const char *state_name(radar_tgt_state_t s)
{
    switch (s) {
    case RADAR_TGT_MOTION:   return "运动";
    case RADAR_TGT_BREATH:   return "呼吸";
    case RADAR_TGT_APPROACH: return "靠近";
    case RADAR_TGT_DEPART:   return "远离";
    default:                 return "无目标";
    }
}

static radar_tgt_state_t decode_state(uint8_t det)
{
    if (det & RADAR_TGT_MOTION) return RADAR_TGT_MOTION;
    if (det & RADAR_TGT_BREATH) return RADAR_TGT_BREATH;
    if (det & RADAR_TGT_APPROACH) return RADAR_TGT_APPROACH;
    if (det & RADAR_TGT_DEPART) return RADAR_TGT_DEPART;
    return RADAR_TGT_NONE;
}

/* Raw angles scatter ±20°+ while the target moves/speaks (torso sway +
 * multipath); confidence-qualified still frames are stable. Median of the
 * last RAD_ANG_HIST qualified samples gives a usable azimuth for fusion. */
#define RAD_ANG_HIST 5
static int16_t s_ang_hist[RAD_ANG_HIST];
static uint8_t s_ang_hist_n;

static int16_t median_hist(void)
{
    int16_t tmp[RAD_ANG_HIST];
    memcpy(tmp, s_ang_hist, s_ang_hist_n * sizeof(int16_t));
    for (uint8_t i = 1; i < s_ang_hist_n; i++) {
        int16_t k = tmp[i];
        int8_t j = i - 1;
        while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = k;
    }
    return tmp[s_ang_hist_n / 2];
}

static void hist_reset(void)
{
    s_ang_hist_n = 0;
}

static void publish(uint8_t det_result, uint16_t range_mm, int16_t angle_deg,
                    uint8_t rb_conf, uint8_t angle_conf, uint32_t frame_idx)
{
    bool was_valid;
    radar_tgt_state_t was_state;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    was_valid = s_target.valid;
    was_state = s_target.state;
    s_target.valid = det_result != 0;
    s_target.det_result = det_result;
    s_target.state = decode_state(det_result);
    s_target.range_mm = range_mm;
    s_target.angle_deg = angle_deg;
    if (det_result && rb_conf >= 12 && angle_conf >= 8) {
        if (s_ang_hist_n < RAD_ANG_HIST) {
            s_ang_hist[s_ang_hist_n++] = angle_deg;
        } else {
            memmove(s_ang_hist, s_ang_hist + 1, (RAD_ANG_HIST - 1) * sizeof(int16_t));
            s_ang_hist[RAD_ANG_HIST - 1] = angle_deg;
        }
    }
    s_target.filt_n = s_ang_hist_n;
    s_target.angle_filt_deg = s_ang_hist_n ? median_hist() : angle_deg;
    float az = RADAR_AZ_OFFSET_DEG + RADAR_AZ_SCALE * (float)s_target.angle_filt_deg;
    if (az < 0) az += 360;
    if (az >= 360) az -= 360;
    s_target.azimuth_deg = az;
    s_target.rb_conf = rb_conf;
    s_target.angle_conf = angle_conf;
    s_target.frame_idx = frame_idx;
    s_target.last_seen_ms = esp_timer_get_time() / 1000;
    xSemaphoreGive(s_lock);

    if (was_valid != s_target.valid) {
        if (s_target.valid) {
            events_push(AEVT_TARGET_ENTER, (int16_t)s_target.azimuth_deg,
                        (int16_t)(range_mm / 10));
        } else {
            events_push(AEVT_TARGET_LEAVE, (int16_t)s_target.azimuth_deg, 0);
        }
    }
    if (was_valid != s_target.valid || was_state != s_target.state) {
        ESP_LOGI(TAG, "%s: %s range=%umm angle=%+d° filt=%+d° az=%.0f° conf=%u/%u",
                 s_target.valid ? "target" : "cleared",
                 state_name(s_target.state),
                 s_target.valid ? range_mm : 0, s_target.valid ? angle_deg : 0,
                 s_target.valid ? s_target.angle_filt_deg : 0,
                 s_target.valid ? s_target.azimuth_deg : 0.0f,
                 rb_conf, angle_conf);
    }
}

static void handle_reply(const uint8_t *cmd_pos, uint8_t plen)
{
    /* cmd_pos[0]=CMD, cmd_pos[1]=PARAMLEN, params at cmd_pos+2 */
    uint8_t cmd = cmd_pos[0];
    c_reply++;

    if (cmd == 0x30 && plen >= 20) {
        const uint8_t *f = cmd_pos + 2;
        c_30++;
        publish(f[1],                   /* det_result */
                rd_u16(f + 2),          /* range */
                rd_s16(f + 4),          /* angle */
                f[14],                  /* rb_conf */
                f[15],                  /* angle_conf */
                rd_u32(f + 16));        /* frame_idx */
    } else if (cmd == 0xFE && !s_ver_ok && plen >= 8) {
        s_ver_ok = true;
        ESP_LOGI(TAG, "online: SDK v%d.%d.%d HW v%d.%d",
                 cmd_pos[2], cmd_pos[3], cmd_pos[4], cmd_pos[7], cmd_pos[8]);
    } else if (cmd == 0x32 && plen >= 14) {
        const uint8_t *f = cmd_pos + 2;  /* u16s, cm */
        ESP_LOGI(TAG, "bounds(cm): mot %u-%u micro %u-%u bhr %u-%u",
                 rd_u16(f), rd_u16(f + 2), rd_u16(f + 4),
                 rd_u16(f + 6), rd_u16(f + 8), rd_u16(f + 10));
        if (s_capture_cmd == 0x32 && s_capture_dst) {
            radar_cfg_t *c = s_capture_dst;
            c->b_mot_min = rd_u16(f);      c->b_mot_max = rd_u16(f + 2);
            c->b_micro_min = rd_u16(f + 4); c->b_micro_max = rd_u16(f + 6);
            c->b_bhr_min = rd_u16(f + 8);  c->b_bhr_max = rd_u16(f + 10);
            s_got_bounds = true;
        }
    } else if (cmd == 0x33 && plen >= 18) {
        const uint8_t *f = cmd_pos + 2;  /* 6 u16 cm + 3 u16 levels */
        ESP_LOGI(TAG, "cfg(cm): mot %u-%u micro %u-%u bhr %u-%u lvl %u/%u/%u",
                 rd_u16(f), rd_u16(f + 2), rd_u16(f + 4), rd_u16(f + 6),
                 rd_u16(f + 8), rd_u16(f + 10),
                 rd_u16(f + 12), rd_u16(f + 14), rd_u16(f + 16));
        if (s_capture_cmd == 0x33 && s_capture_dst) {
            radar_cfg_t *c = s_capture_dst;
            c->mot_min = rd_u16(f);       c->mot_max = rd_u16(f + 2);
            c->micro_min = rd_u16(f + 4); c->micro_max = rd_u16(f + 6);
            c->bhr_min = rd_u16(f + 8);   c->bhr_max = rd_u16(f + 10);
            c->mot_lvl = rd_u16(f + 12);  c->micro_lvl = rd_u16(f + 14);
            c->bhr_lvl = rd_u16(f + 16);
            s_got_cfg = true;
        }
    } else if (cmd == 0xD0 && plen >= 1) {
        if (s_capture_cmd == 0xD0 && s_capture_dst) {
            s_capture_dst->sensing = cmd_pos[2] != 0;
            s_got_sensing = true;
        }
    }
}

/* Walk the accumulator, extracting complete 0x59 reply frames. */
static void parse_frames(void)
{
    size_t i = 0;
    while (i < s_len) {
        if (s_acc[i] != 0x59) { c_stray++; i++; continue; }
        if (i + 3 > s_len) break;
        uint8_t plen = s_acc[i + 2];
        size_t total = 3u + plen + 2u;
        if (i + total > s_len) break;
        uint16_t sum = 0;
        for (size_t k = 0; k < 3u + plen; k++) sum += s_acc[i + k];
        if (sum == rd_u16(&s_acc[i + 3 + plen])) {
            handle_reply(&s_acc[i + 1], plen);
            i += total;
        } else { c_bad_ck++; i++; }
    }
    if (i) { memmove(s_acc, s_acc + i, s_len - i); s_len -= i; }
}

/* Non-blocking: drain UART + parse. Returns true if any valid reply arrived. */
static bool service_link(void)
{
    uint32_t replies_before = c_reply;
    size_t room = RAD_ACC_MAX - s_len;
    if (!room) { s_len = 0; c_stray += RAD_ACC_MAX; }
    int n = uart_read_bytes(RAD_UART_NUM, s_acc + s_len, room, pdMS_TO_TICKS(20));
    if (n > 0) { s_len += (size_t)n; c_rx += (uint32_t)n; }
    parse_frames();
    return c_reply > replies_before;
}

static bool wait_reply(uint32_t timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        if (service_link() && s_ver_ok) return true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

static void radar_task(void *arg)
{
    (void)arg;
    int64_t last_poll = 0, last_stats = 0;
    uint32_t miss = 0;

    /* Bring-up: confirm link with version query, then force sensing on. */
    for (int i = 0; i < RAD_INIT_RETRIES && !s_ver_ok; i++) {
        uart_write_bytes(RAD_UART_NUM, VER_REQ, sizeof(VER_REQ));
        wait_reply(500);
    }

    /* Baud self-heal: if the module sits at its factory 921600 (seen
     * after config-flash writes), switch it back to 115200 and
     * re-verify before continuing. */
    if (!s_ver_ok) {
        ESP_LOGW(TAG, "no reply at %d — trying 921600 recovery", RAD_BAUD);
        uart_set_baudrate(RAD_UART_NUM, 921600);
        uart_flush_input(RAD_UART_NUM);
        for (int i = 0; i < 3 && !s_ver_ok; i++) {
            uart_write_bytes(RAD_UART_NUM, BAUD_115200, sizeof(BAUD_115200));
            vTaskDelay(pdMS_TO_TICKS(300));
            uart_write_bytes(RAD_UART_NUM, VER_REQ, sizeof(VER_REQ));
            wait_reply(400);
        }
        uart_set_baudrate(RAD_UART_NUM, RAD_BAUD);
        uart_flush_input(RAD_UART_NUM);
        if (s_ver_ok) {
            s_ver_ok = false;
            for (int i = 0; i < 3 && !s_ver_ok; i++) {
                uart_write_bytes(RAD_UART_NUM, VER_REQ, sizeof(VER_REQ));
                wait_reply(400);
            }
            ESP_LOGI(TAG, "baud recovery %s",
                     s_ver_ok ? "OK (radar back at 115200)" : "FAILED");
        } else {
            ESP_LOGW(TAG, "radar silent at 921600 too — continuing, "
                     "will keep polling at %d", RAD_BAUD);
        }
    }
    if (!s_ver_ok)
        ESP_LOGW(TAG, "no version reply — continuing, will keep polling");
    uart_write_bytes(RAD_UART_NUM, SENS_ON, sizeof(SENS_ON));
    vTaskDelay(pdMS_TO_TICKS(200));
    uart_write_bytes(RAD_UART_NUM, BR_MIN, sizeof(BR_MIN));
    vTaskDelay(pdMS_TO_TICKS(200));
    uart_write_bytes(RAD_UART_NUM, MO_MIN, sizeof(MO_MIN));
    vTaskDelay(pdMS_TO_TICKS(200));
    uart_write_bytes(RAD_UART_NUM, SAVE, sizeof(SAVE));
    vTaskDelay(pdMS_TO_TICKS(1200));
    uart_write_bytes(RAD_UART_NUM, SYSRST, sizeof(SYSRST));
    ESP_LOGI(TAG, "radar reset to apply distance config (mot>=50cm bhr>=80cm)");
    vTaskDelay(pdMS_TO_TICKS(4000));
    uart_write_bytes(RAD_UART_NUM, Q_BOUND, sizeof(Q_BOUND));
    vTaskDelay(pdMS_TO_TICKS(200));
    uart_write_bytes(RAD_UART_NUM, Q_CFG, sizeof(Q_CFG));
    vTaskDelay(pdMS_TO_TICKS(200));
    service_link();
    s_online = true;

    while (1) {
        /* Execute a posted configuration request (UART stays in this
         * task). Polling pauses for the duration — bounded by the
         * request type (GET ~1.2 s, SET ~2.5 s, RESET ~5 s). */
        if (s_req_pending) {
            exec_request();
        }

        int64_t now = esp_timer_get_time();
        if (now - last_poll >= RAD_POLL_MS * 1000LL) {
            uart_write_bytes(RAD_UART_NUM, DET_Q, sizeof(DET_Q));
            last_poll = now;
            if (service_link()) {
                miss = 0;
            } else {
                /* Give the reply the remainder of this cycle to land. */
                bool late = false;
                int64_t give = now + RAD_POLL_MS * 1000LL;
                while (esp_timer_get_time() < give) {
                    if (service_link()) { late = true; break; }
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                if (late) miss = 0; else miss++;
            }
            bool was_online = s_online;
            s_online = miss < RAD_OFFLINE_MISS;
            if (was_online && !s_online) {
                ESP_LOGW(TAG, "OFFLINE (%u polls unanswered) — degrading", miss);
                events_push(AEVT_RADAR_OFFLINE, 0, 0);
                xSemaphoreTake(s_lock, portMAX_DELAY);
                s_target.valid = false;
                s_target.state = RADAR_TGT_NONE;
                s_target.range_mm = 0;
                s_target.angle_deg = 0;
                s_target.angle_filt_deg = 0;
                s_target.filt_n = 0;
                hist_reset();
                s_target.rb_conf = 0;
                s_target.angle_conf = 0;
                xSemaphoreGive(s_lock);
            } else if (!was_online && s_online) {
                ESP_LOGI(TAG, "back ONLINE");
                events_push(AEVT_RADAR_ONLINE, 0, 0);
            }
            c_miss = miss;
        } else {
            service_link();
        }

        if (now - last_stats >= 5000 * 1000LL) {
            radar_target_t t;
            radar_get_target(&t);

            /* Long-stillness care alarm (US-009). Alarm: target present
             * with no sustained motion for still_min. Activity (resets
             * timer / clears alarm): sustained MOTION state (2+ ticks,
             * any range — the aggregated target reports hands during
             * voluntary movement, so range attribution is unreliable),
             * speech (confident DOA), torso position change (breath
             * anchor shift >15 cm), or the person leaving.
             *
             * Known limitation: in a multi-reflector scene (desk with
             * phantom/hand motion source) the alarm effectively stays
             * off — correct, since that scene is continuously "moving".
             * The care scenario (chair/bed, single person) is the target.
             * Disabled when still_min == 0 (default). */
            static int64_t last_motion_us = -1;
            static int64_t target_gone_us = -1;
            static uint8_t motion_votes = 0;
            static bool still_fired = false;
            static int16_t breath_anchor_cm = -1;
            if (last_motion_us < 0) last_motion_us = now;   /* boot */

#define STILL_RECOVER(why) do {                                        \
        still_fired = false;                                           \
        last_motion_us = now;    /* restart clock on ANY recovery */   \
        events_push(AEVT_STILL_RECOVER, (int16_t)t.azimuth_deg, 0);    \
        ESP_LOGI(TAG, "still alarm cleared (%s)", why);                \
    } while (0)

            int16_t range_cm = (int16_t)(t.range_mm / 10);
            bool consumed_sound = false;
            if (s_sound_activity) {           /* speech beats everything */
                s_sound_activity = false;
                consumed_sound = true;
                if (still_fired) STILL_RECOVER("speech");
                else last_motion_us = now;
            }

            if (t.valid) {
                target_gone_us = -1;
                if (t.state == RADAR_TGT_BREATH) {
                    if (breath_anchor_cm < 0) {
                        breath_anchor_cm = range_cm;
                    } else {
                        if (still_fired &&
                            (range_cm > breath_anchor_cm + 15 ||
                             range_cm < breath_anchor_cm - 15)) {
                            STILL_RECOVER("position change");
                        }
                        /* slow anchor follow for breathing wander */
                        breath_anchor_cm =
                            (int16_t)((breath_anchor_cm * 7 + range_cm * 3) / 10);
                    }
                }
                if (t.state == RADAR_TGT_MOTION) {
                    if (motion_votes < 5) motion_votes++;
                } else if (motion_votes) {
                    motion_votes--;
                }
                if (motion_votes >= 2) {
                    last_motion_us = now;
                    if (still_fired) STILL_RECOVER("motion");
                }
            } else {
                if (target_gone_us < 0) target_gone_us = now;
                if (now - target_gone_us > 10 * 1000000LL) {
                    if (still_fired) STILL_RECOVER("left");
                    breath_anchor_cm = -1;
                }
            }

            uint16_t still_min = mode_manager_get_still_min();
            if (still_min > 0 && t.valid && !still_fired && !consumed_sound &&
                now - last_motion_us >= (int64_t)still_min * 60 * 1000000LL) {
                still_fired = true;
                events_push(AEVT_STILL_ALARM, (int16_t)(t.range_mm / 10),
                            (int16_t)t.azimuth_deg);
                ESP_LOGW(TAG, "STILL ALARM: no movement for %u min "
                         "(range=%ucm az=%.0f°)", still_min,
                         t.range_mm / 10, t.azimuth_deg);
            }
#undef STILL_RECOVER

            ESP_LOGI(TAG, "[5s] online=%d tgt=%d(%s) range=%umm angle=%+d° "
                     "filt=%+d° az=%.0f° conf=%u/%u rx=%uB r30=%u miss=%u bad_ck=%u stray=%u",
                     s_online, t.valid, state_name(t.state),
                     t.valid ? t.range_mm : 0, t.valid ? t.angle_deg : 0,
                     t.valid ? t.angle_filt_deg : 0,
                     t.valid ? t.azimuth_deg : 0.0f,
                     t.rb_conf, t.angle_conf,
                     c_rx, c_30, c_miss, c_bad_ck, c_stray);
            c_rx = c_30 = 0;
            last_stats = now;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void radar_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_req_lock = xSemaphoreCreateMutex();
    s_req_done = xSemaphoreCreateBinary();
    s_req_pending = false;

    const uart_config_t cfg = {
        .baud_rate = RAD_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RAD_UART_NUM, RAD_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RAD_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RAD_UART_NUM, RAD_TX_GPIO, RAD_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "init: UART%d %d 8N1 tx=%d rx=%d, 5Hz poll",
             RAD_UART_NUM, RAD_BAUD, RAD_TX_GPIO, RAD_RX_GPIO);

    xTaskCreate(radar_task, "radar", 4096, NULL, 5, NULL);
}

bool radar_is_online(void)
{
    return s_online;
}

bool radar_get_target(radar_target_t *out)
{
    if (!out) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_target;
    xSemaphoreGive(s_lock);
    return out->valid;
}
