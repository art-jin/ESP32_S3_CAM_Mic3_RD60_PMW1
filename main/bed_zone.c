#include "bed_zone.h"

#include <string.h>
#include <time.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "zone_tracker.h"

static const char *TAG = "bed";

/* Upright-episode signature: compact (standing bodies are; lying bodies
 * are bimodal at 638-1122mm per protocol notes §11) AND moving, sustained
 * UPRIGHT_FRAMES frames at 5 Hz (~1.6 s of walking). */
#define UPRIGHT_SPREAD_MM  350
#define UPRIGHT_FRAMES     8

#define BED_MIN_MM_LO  300
#define BED_MIN_MM_HI  5900
#define BED_MAX_MM_LO  400
#define BED_MAX_MM_HI  6000
#define BED_CONT_LO    10
#define BED_CONT_HI    3600
#define BED_TRAJ_LO    10
#define BED_TRAJ_HI    1800

static bed_cfg_t s_cfg = BED_CFG_DEFAULT;

/* Radar-task-local state (sole feeder, no locking). */
static bool     s_zone_in;
static int64_t  s_zone_since_us;      /* when current in-zone stretch began */
static uint16_t s_up_cnt;             /* consecutive compact+motion frames  */
static int64_t  s_last_upright_us;    /* end of last upright episode        */
static uint32_t s_suppressed;

static void cfg_save(void)
{
    nvs_handle_t h;
    if (nvs_open("bcfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u16(h, "bmin", s_cfg.min_mm);
        nvs_set_u16(h, "bmax", s_cfg.max_mm);
        nvs_set_u16(h, "bcont", s_cfg.cont_s);
        nvs_set_u16(h, "btraj", s_cfg.traj_s);
        nvs_set_u8(h, "bnst", s_cfg.night_start_h);
        nvs_set_u8(h, "bnen", s_cfg.night_end_h);
        nvs_commit(h);
        nvs_close(h);
    }
}

void bed_zone_init(void)
{
    nvs_handle_t h;
    if (nvs_open("bcfg", NVS_READONLY, &h) == ESP_OK) {
        uint16_t v;
        if (nvs_get_u16(h, "bmin", &v) == ESP_OK) s_cfg.min_mm = v;
        if (nvs_get_u16(h, "bmax", &v) == ESP_OK) s_cfg.max_mm = v;
        if (nvs_get_u16(h, "bcont", &v) == ESP_OK && v >= BED_CONT_LO && v <= BED_CONT_HI)
            s_cfg.cont_s = v;
        if (nvs_get_u16(h, "btraj", &v) == ESP_OK && v >= BED_TRAJ_LO && v <= BED_TRAJ_HI)
            s_cfg.traj_s = v;
        uint8_t h8;
        if (nvs_get_u8(h, "bnst", &h8) == ESP_OK && h8 < 24) s_cfg.night_start_h = h8;
        if (nvs_get_u8(h, "bnen", &h8) == ESP_OK && h8 < 24) s_cfg.night_end_h = h8;
        nvs_close(h);
    }
    if (s_cfg.min_mm == 0) s_cfg.max_mm = 0;   /* disabled is disabled */
    ESP_LOGI(TAG, "init: window=[%u,%u]mm cont=%us traj=%us night=%u-%u "
             "(suppression %s)",
             s_cfg.min_mm, s_cfg.max_mm, s_cfg.cont_s, s_cfg.traj_s,
             s_cfg.night_start_h, s_cfg.night_end_h,
             s_cfg.min_mm ? "ARMED" : "OFF");
}

void bed_zone_feed(bool zone_in, bool trusted, bool motion,
                   uint16_t spread_mm, int64_t now_us)
{
    if (zone_in && !s_zone_in) s_zone_since_us = now_us;
    s_zone_in = zone_in;

    bool upright_frame = trusted && motion && spread_mm < UPRIGHT_SPREAD_MM;
    if (upright_frame) {
        if (++s_up_cnt >= UPRIGHT_FRAMES) {
            s_last_upright_us = now_us;   /* episode ended (so far) now */
        }
    } else if (s_up_cnt >= UPRIGHT_FRAMES) {
        s_last_upright_us = now_us;       /* episode just completed */
        s_up_cnt = 0;
    } else {
        s_up_cnt = 0;
    }
}

bool bed_zone_should_suppress(uint16_t anchor_mm, int64_t now_us)
{
    if (s_cfg.min_mm == 0 || s_cfg.max_mm <= s_cfg.min_mm) return false;
    if (anchor_mm < s_cfg.min_mm || anchor_mm > s_cfg.max_mm) return false;
    if (!s_zone_in ||
        now_us - s_zone_since_us < (int64_t)s_cfg.cont_s * 1000000LL)
        return false;
    if (now_us - s_last_upright_us < (int64_t)s_cfg.traj_s * 1000000LL)
        return false;
    s_suppressed++;
    return true;
}

bool bed_zone_is_night(void)
{
    time_t t = time(NULL);
    if (t < 1704067200) return false;   /* 2024-01-01: unsynced ⇒ day */
    struct tm tm_now;
    localtime_r(&t, &tm_now);
    uint8_t h = (uint8_t)tm_now.tm_hour;
    uint8_t a = s_cfg.night_start_h, b = s_cfg.night_end_h;
    return a <= b ? (h >= a && h < b) : (h >= a || h < b);
}

const bed_cfg_t *bed_cfg(void) { return &s_cfg; }

bool bed_cfg_set(const bed_cfg_t *c)
{
    if (c->min_mm != 0 &&
        (c->min_mm < BED_MIN_MM_LO || c->min_mm > BED_MIN_MM_HI ||
         c->max_mm < BED_MAX_MM_LO || c->max_mm > BED_MAX_MM_HI ||
         c->max_mm <= c->min_mm))
        return false;
    if (c->cont_s < BED_CONT_LO || c->cont_s > BED_CONT_HI ||
        c->traj_s < BED_TRAJ_LO || c->traj_s > BED_TRAJ_HI ||
        c->night_start_h > 23 || c->night_end_h > 23)
        return false;
    s_cfg = *c;
    if (s_cfg.min_mm == 0) s_cfg.max_mm = 0;
    cfg_save();
    ESP_LOGI(TAG, "cfg: window=[%u,%u]mm cont=%us traj=%us night=%u-%u",
             s_cfg.min_mm, s_cfg.max_mm, s_cfg.cont_s, s_cfg.traj_s,
             s_cfg.night_start_h, s_cfg.night_end_h);
    return true;
}

void bed_zone_profile(const char *name)
{
    if (strcmp(name, "bedside") == 0) {
        /* Zone covers the bed: bed window + hysteresis margin, capped by
         * the zone validator. Night weighting armed with cfg hours. */
        zone_cfg_t z = *zone_cfg();
        z.enter_mm = s_cfg.max_mm ? (uint16_t)(s_cfg.max_mm + 400) : z.enter_mm;
        if (z.enter_mm > 6000) z.enter_mm = 6000;
        zone_cfg_set(&z);
        ESP_LOGI(TAG, "profile bedside: zone_enter=%umm", z.enter_mm);
    } else if (strcmp(name, "desk") == 0) {
        bed_cfg_t c = s_cfg;
        c.min_mm = 0;
        c.max_mm = 0;
        bed_cfg_set(&c);
        zone_cfg_t z = ZONE_CFG_DEFAULT;
        zone_cfg_set(&z);
        ESP_LOGI(TAG, "profile desk: suppression off, zone defaults");
    }
}

uint32_t bed_zone_suppressed_count(void) { return s_suppressed; }
