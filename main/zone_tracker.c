#include "zone_tracker.h"

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "zone";

/* rb_conf below this is untrusted — same threshold as fusion.c. */
#define ZONE_RB_CONF_MIN  12

/* Validation ranges for REST-set values (§6.4 — sane bounds, not just types). */
#define Z_ENTER_MIN 300
#define Z_ENTER_MAX 6000
#define Z_DEB_MIN     1
#define Z_DEB_MAX    10
#define Z_HYST_MIN   50
#define Z_HYST_MAX  2000
#define Z_LEAVE_MIN   1
#define Z_LEAVE_MAX  20
#define Z_LOST_MIN  200
#define Z_LOST_MAX 10000

static zone_cfg_t s_cfg = ZONE_CFG_DEFAULT;

/* State — only ever touched from the radar task (sole feeder). */
static bool     s_in_zone = false;
static uint16_t s_in_cnt  = 0;
static uint16_t s_out_cnt = 0;
static int64_t  s_last_trusted_ms = 0;

static void cfg_save(const zone_cfg_t *c)
{
    nvs_handle_t h;
    if (nvs_open("zcfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u16(h, "zent", c->enter_mm);
        nvs_set_u16(h, "zdeb", c->debounce);
        nvs_set_u16(h, "zhys", c->hyst_mm);
        nvs_set_u16(h, "zlve", c->leave);
        nvs_set_u16(h, "zlst", c->lost_ms);
        nvs_commit(h);
        nvs_close(h);
    }
}

void zone_tracker_init(void)
{
    nvs_handle_t h;
    if (nvs_open("zcfg", NVS_READONLY, &h) == ESP_OK) {
        uint16_t v;
        if (nvs_get_u16(h, "zent", &v) == ESP_OK && v >= Z_ENTER_MIN && v <= Z_ENTER_MAX)
            s_cfg.enter_mm = v;
        if (nvs_get_u16(h, "zdeb", &v) == ESP_OK && v >= Z_DEB_MIN && v <= Z_DEB_MAX)
            s_cfg.debounce = v;
        if (nvs_get_u16(h, "zhys", &v) == ESP_OK && v >= Z_HYST_MIN && v <= Z_HYST_MAX)
            s_cfg.hyst_mm = v;
        if (nvs_get_u16(h, "zlve", &v) == ESP_OK && v >= Z_LEAVE_MIN && v <= Z_LEAVE_MAX)
            s_cfg.leave = v;
        if (nvs_get_u16(h, "zlst", &v) == ESP_OK && v >= Z_LOST_MIN && v <= Z_LOST_MAX)
            s_cfg.lost_ms = v;
        nvs_close(h);
    }
    ESP_LOGI(TAG, "init: enter=%umm deb=%u hyst=%umm leave=%u lost=%ums",
             s_cfg.enter_mm, s_cfg.debounce, s_cfg.hyst_mm, s_cfg.leave,
             s_cfg.lost_ms);
}

zone_edge_t zone_sm_feed(bool valid, uint32_t range_mm, uint8_t rb_conf,
                         int64_t now_ms)
{
    bool trusted = valid && rb_conf >= ZONE_RB_CONF_MIN;
    if (trusted) s_last_trusted_ms = now_ms;

    if (!s_in_zone) {
        if (trusted && range_mm < s_cfg.enter_mm) {
            if (++s_in_cnt >= s_cfg.debounce) {
                s_in_zone = true;
                s_in_cnt = s_out_cnt = 0;
                return ZONE_EDGE_ENTER;
            }
        } else {
            s_in_cnt = 0;
        }
    } else {
        if (trusted && range_mm > (uint32_t)s_cfg.enter_mm + s_cfg.hyst_mm) {
            if (++s_out_cnt >= s_cfg.leave) {
                s_in_zone = false;
                s_in_cnt = s_out_cnt = 0;
                return ZONE_EDGE_LEAVE;
            }
        } else if (!trusted && now_ms - s_last_trusted_ms > s_cfg.lost_ms) {
            s_in_zone = false;
            s_in_cnt = s_out_cnt = 0;
            return ZONE_EDGE_LEAVE;
        } else {
            s_out_cnt = 0;
        }
    }
    return ZONE_EDGE_NONE;
}

zone_edge_t zone_sm_reset(void)
{
    s_in_cnt = s_out_cnt = 0;
    if (s_in_zone) {
        s_in_zone = false;
        return ZONE_EDGE_LEAVE;
    }
    return ZONE_EDGE_NONE;
}

bool zone_in(void)
{
    return s_in_zone;
}

const zone_cfg_t *zone_cfg(void)
{
    return &s_cfg;
}

bool zone_cfg_set(const zone_cfg_t *c)
{
    if (c->enter_mm < Z_ENTER_MIN || c->enter_mm > Z_ENTER_MAX ||
        c->debounce < Z_DEB_MIN  || c->debounce  > Z_DEB_MAX  ||
        c->hyst_mm  < Z_HYST_MIN  || c->hyst_mm  > Z_HYST_MAX  ||
        c->leave    < Z_LEAVE_MIN || c->leave    > Z_LEAVE_MAX ||
        c->lost_ms  < Z_LOST_MIN  || c->lost_ms  > Z_LOST_MAX) {
        return false;
    }
    s_cfg = *c;
    cfg_save(&s_cfg);
    ESP_LOGI(TAG, "cfg: enter=%umm deb=%u hyst=%umm leave=%u lost=%ums",
             s_cfg.enter_mm, s_cfg.debounce, s_cfg.hyst_mm, s_cfg.leave,
             s_cfg.lost_ms);
    return true;
}
