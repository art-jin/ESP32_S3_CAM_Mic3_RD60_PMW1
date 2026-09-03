#include "rest_api.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mode_manager.h"
#include "status.h"
#include "servo.h"
#include "doa.h"
#include "evlog.h"
#include "radar.h"
#include "fusion.h"
#include "events.h"

static const char *TAG = "rest";
static httpd_handle_t s_server = NULL;

/* Device ID from NVS (generated on first boot, persistent).
 * Defined in main.c, declared extern here. */
extern char g_device_id[8];

/* Rate limiter for /api/point and /api/shake (shared, min 500ms). */
#define POINT_MIN_INTERVAL_US  500000
static int64_t s_last_point_us = 0;

/* Shake-in-progress flag: blocks /api/point and /api/mode during shake. */
static volatile bool s_shaking = false;

/* Clock-direction → servo angle mapping. 2oc and 10oc are clamped to
 * the ±100° mechanical limit. */
static const struct {
    const char *name;
    float angle;
} s_dir_map[] = {
    {"2oc",  -100.0f},
    {"3oc",   -90.0f},
    {"4oc",   -60.0f},
    {"5oc",   -30.0f},
    {"6oc",     0.0f},
    {"7oc",    30.0f},
    {"8oc",    60.0f},
    {"9oc",    90.0f},
    {"10oc",  100.0f},
};
#define N_DIR  (sizeof(s_dir_map) / sizeof(s_dir_map[0]))

#define MAX_BODY_LEN 128

/* ---- CORS + error helpers ---- */
static void set_cors(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

static esp_err_t send_json_ok(httpd_req_t *req, const char *body)
{
    set_cors(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, body);
    return ESP_OK;
}

static esp_err_t send_error(httpd_req_t *req, int code, const char *error, const char *message)
{
    set_cors(req);
    httpd_resp_set_type(req, "application/json");
    const char *status = code == 401 ? "401 Unauthorized" :
                         code == 400 ? "400 Bad Request" :
                         code == 403 ? "403 Forbidden" :
                         code == 429 ? "429 Too Many Requests" :
                                       "500 Internal Server Error";
    httpd_resp_set_status(req, status);
    char body[200];
    snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\",\"message\":\"%s\"}",
             error, message);
    httpd_resp_sendstr(req, body);
    return ESP_OK;
}

static bool check_auth(httpd_req_t *req)
{
    char query[128];
    char dev_id[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "device_id", dev_id, sizeof(dev_id)) == ESP_OK &&
        strcmp(dev_id, g_device_id) == 0) {
        return true;
    }
    send_error(req, 401, "unauthorized", "invalid or missing device_id");
    return false;
}

/* Read POST body into buf (null-terminated). Returns true on success. */
static bool read_body(httpd_req_t *req, char *buf, size_t bufsize)
{
    int len = req->content_len;
    if (len <= 0) {
        send_error(req, 400, "bad_request", "empty body");
        return false;
    }
    if (len >= (int)bufsize) {
        send_error(req, 400, "bad_request", "body too long");
        return false;
    }
    int ret = httpd_req_recv(req, buf, bufsize - 1);
    if (ret <= 0) {
        send_error(req, 400, "bad_request", "failed to read body");
        return false;
    }
    buf[ret] = '\0';
    return true;
}

/* Extract string value for key from JSON-like body.
 * Body format: {"key":"value"} or {"key":123}
 * Returns true if found and copies value (null-terminated). */
static bool json_get_str(const char *body, const char *key, char *out, size_t outsize)
{
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(body, pattern);
    if (!p) return false;
    p += strlen(pattern);
    /* skip : and whitespace */
    while (*p && (*p == ':' || *p == ' ' || *p == '\t')) p++;
    if (*p == '"') {
        p++;
        size_t i = 0;
        while (*p && *p != '"' && i < outsize - 1) out[i++] = *p++;
        out[i] = '\0';
    } else {
        size_t i = 0;
        while (*p && *p != ',' && *p != '}' && *p != ' ' && i < outsize - 1)
            out[i++] = *p++;
        out[i] = '\0';
    }
    return true;
}

static bool json_get_int(const char *body, const char *key, int *out)
{
    char buf[16];
    if (!json_get_str(body, key, buf, sizeof(buf))) return false;
    *out = atoi(buf);
    return true;
}

/* ---- Handlers ---- */

/* GET /api/ping — no auth, heartbeat only. */
static esp_err_t handler_ping(httpd_req_t *req)
{
    return send_json_ok(req, "{\"ok\":true}");
}

/* OPTIONS catch-all for CORS preflight. */
static esp_err_t handler_options(httpd_req_t *req)
{
    set_cors(req);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* GET /api/status?device_id=XXXX */
static esp_err_t handler_status(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;

    device_status_t st;
    status_get(&st);

    const char *sect_label = doa_sextant_label(st.stable_sect, DOA_MODE_3MIC);
    if (!sect_label) sect_label = "--";

    /* Phase 4 (US-007): radar / fusion / sub-mode snapshot fields,
     * read directly from the owning modules at request time. */
    radar_target_t rt;
    bool rt_valid = radar_get_target(&rt);
    bool rt_online = radar_is_online();
    fusion_result_t fr;
    fusion_get_last(&fr);
    static const char *subnames[] = {"audio_only", "fusion", "radar_follow"};
    static const char *oornames[] = {"hold", "clamp", "home", "scan"};
    /* radar_tgt_state_t values are bit masks, not sequential — map by switch. */
    const char *rt_state;
    switch (rt.state) {
    case RADAR_TGT_APPROACH: rt_state = "approach"; break;
    case RADAR_TGT_DEPART:   rt_state = "depart";   break;
    case RADAR_TGT_MOTION:   rt_state = "motion";   break;
    case RADAR_TGT_BREATH:   rt_state = "breath";   break;
    default:                 rt_state = "none";     break;
    }

    char body[800];
    snprintf(body, sizeof(body),
        "{\"ok\":true,"
        "\"mode\":\"%s\","
        "\"submode\":\"%s\","
        "\"oor\":\"%s\","
        "\"still_min\":%u,"
        "\"assoc_gate\":%s,"
        "\"servo\":%.1f,"
        "\"moving\":%s,"
        "\"azimuth\":%.0f,"
        "\"sect\":\"%s\","
        "\"conf\":%.2f,"
        "\"radar\":{\"online\":%s,\"target\":{\"valid\":%s,\"state\":\"%s\","
        "\"range_mm\":%u,\"azimuth\":%.0f,\"rb_conf\":%u,\"ang_conf\":%u}},"
        "\"fusion\":{\"evaluated\":%s,\"associated\":%s,\"doa_az\":%.0f,"
        "\"radar_az\":%.0f,\"diff\":%.0f,\"range_mm\":%u},"
        "\"wifi\":\"%s\","
        "\"ip\":\"%s\","
        "\"host\":\"%s\""
        "}",
        st.mode == MODE_COMMAND ? "command" : "track",
        subnames[mode_manager_get_submode()],
        oornames[mode_manager_get_oor_policy()],
        (unsigned)mode_manager_get_still_min(),
        mode_manager_get_assoc_gate() ? "true" : "false",
        st.servo_angle,
        st.servo_moving ? "true" : "false",
        st.azimuth,
        sect_label,
        st.confidence,
        rt_online ? "true" : "false",
        rt_valid ? "true" : "false",
        rt_valid ? rt_state : "none",
        rt_valid ? rt.range_mm : 0,
        rt_valid ? rt.azimuth_deg : 0.0f,
        rt_valid ? rt.rb_conf : 0,
        rt_valid ? rt.angle_conf : 0,
        fr.evaluated ? "true" : "false",
        fr.associated ? "true" : "false",
        fr.doa_az_deg, fr.radar_az_deg, fr.angle_diff_deg,
        fr.range_mm,
        st.wifi_connected ? "connected" : "disconnected",
        st.ip[0] ? st.ip : "",
        st.hostname[0] ? st.hostname : "");

    return send_json_ok(req, body);
}

/* Embedded visualization page (main/index.html via EMBED_FILES). Served
 * without auth — the page itself carries no data; every /api endpoint
 * still requires device_id. */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static esp_err_t handler_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start);
}

/* GET /api/events?device_id=XXXX[&since=SEQ]
 * Returns the newest 32 scene events; pass since=<last seq seen> for
 * incremental polling. Events older than the ring window are skipped. */
static esp_err_t handler_events(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;

    uint32_t since = 0;
    char query[128], val[12];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "since", val, sizeof(val)) == ESP_OK) {
        since = (uint32_t)strtoul(val, NULL, 10);
    }

    app_event_t evs[32];
    uint32_t next = 0;
    int n = events_get(evs, 32, since, &next);

    char *body = malloc(256 + (size_t)n * 80);
    if (!body) {
        return send_error(req, 500, "oom", "cannot allocate response buffer");
    }
    int pos = snprintf(body, 256, "{\"ok\":true,\"next\":%lu,\"events\":[",
                       (unsigned long)next);
    for (int i = 0; i < n && pos > 0; i++) {
        pos += snprintf(body + pos, 80,
            "%s{\"seq\":%lu,\"ms\":%lu,\"type\":\"%s\",\"v1\":%d,\"v2\":%d}",
            i ? "," : "",
            (unsigned long)evs[i].seq,
            (unsigned long)evs[i].uptime_ms,
            events_type_name(evs[i].type),
            evs[i].v1, evs[i].v2);
    }
    if (pos > 0 && pos < 256 + n * 80) {
        snprintf(body + pos, 16, "]}");
    } else {
        free(body);
        return send_error(req, 500, "oom", "response overflow");
    }
    esp_err_t r = send_json_ok(req, body);
    free(body);
    return r;
}

/* GET /api/logs?device_id=XXXX
 * Returns last 32 evlog events as compact JSON. Heap-allocates the response
 * (up to ~3 KB); caller (httpd task) frees on completion. */
static esp_err_t handler_logs(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;

    int cap = evlog_capacity();
    event_t *events = malloc((size_t)cap * sizeof(event_t));
    if (!events) {
        return send_error(req, 500, "oom", "cannot allocate events buffer");
    }
    int n = evlog_get_events(events, cap);
    uint32_t boot = evlog_get_boot_count();

    /* Worst case: 32 events × ~85 bytes + envelope ≈ 2.8 KB */
    char *body = malloc(3200);
    if (!body) {
        free(events);
        return send_error(req, 500, "oom", "cannot allocate response buffer");
    }

    int pos = snprintf(body, 3200,
        "{\"ok\":true,\"boot\":%lu,\"events\":[",
        (unsigned long)boot);

    for (int i = 0; i < n && pos < 3000; i++) {
        const char *tstr = evlog_type_str(events[i].type);
        const char *fstr = evlog_flags_str(events[i].type, events[i].flags);
        pos += snprintf(body + pos, (size_t)(3200 - pos),
            "%s{\"seq\":%u,\"ms\":%lu,\"type\":\"%s\"",
            (i > 0 ? "," : ""),
            events[i].seq,
            (unsigned long)events[i].uptime_ms,
            tstr);
        if (fstr) {
            pos += snprintf(body + pos, (size_t)(3200 - pos),
                ",\"flags\":\"%s\"", fstr);
        }
        pos += snprintf(body + pos, (size_t)(3200 - pos),
            ",\"value\":%d}", events[i].value);
    }

    pos += snprintf(body + pos, (size_t)(3200 - pos), "]}");
    (void)pos;

    free(events);
    esp_err_t r = send_json_ok(req, body);
    free(body);
    return r;
}

/* POST /api/mode?device_id=XXXX
 * Body: {"mode":"command"} or {"mode":"track"}
 * Optional: {"mode":"command","timeout":0}  (0 = no auto-return)
 * Optional (Phase 3, US-005/US-006): "submode":"audio_only|fusion|radar_follow"
 * and/or "oor":"hold|clamp|home|scan". At least one of mode/submode/oor
 * must be present; fields are applied independently.
 */
static esp_err_t handler_mode(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;

    /* Reject mode switch during shake. */
    if (s_shaking) {
        return send_error(req, 409, "shaking",
                          "shake in progress, wait for completion");
    }

    char body[MAX_BODY_LEN];
    if (!read_body(req, body, sizeof(body))) return ESP_OK;

    char mode_str[16] = {0};
    char sub_str[16] = {0};
    char oor_str[8] = {0};
    bool has_mode = json_get_str(body, "mode", mode_str, sizeof(mode_str));
    bool has_sub  = json_get_str(body, "submode", sub_str, sizeof(sub_str));
    bool has_oor  = json_get_str(body, "oor", oor_str, sizeof(oor_str));
    int still_probe = -1;
    bool has_still = json_get_int(body, "still_min", &still_probe);
    int gate_probe = -1;
    bool has_gate = json_get_int(body, "assoc_gate", &gate_probe);

    if (!has_mode && !has_sub && !has_oor && !has_still && !has_gate) {
        return send_error(req, 400, "bad_request",
                          "need at least one of 'mode'/'submode'/'oor'/'still_min'/'assoc_gate'");
    }

    if (has_mode) {
        if (strcmp(mode_str, "command") == 0) {
            int timeout = -1;  /* -1 = use default */
            json_get_int(body, "timeout", &timeout);
            mode_manager_set(MODE_COMMAND, timeout);
        } else if (strcmp(mode_str, "track") == 0) {
            mode_manager_set(MODE_TRACK, 0);
        } else {
            return send_error(req, 400, "bad_request",
                              "mode must be 'track' or 'command'");
        }
    }

    if (has_sub) {
        if (strcmp(sub_str, "audio_only") == 0) {
            mode_manager_set_submode(TRACK_AUDIO_ONLY);
        } else if (strcmp(sub_str, "fusion") == 0) {
            mode_manager_set_submode(TRACK_FUSION);
        } else if (strcmp(sub_str, "radar_follow") == 0) {
            mode_manager_set_submode(TRACK_RADAR_FOLLOW);
        } else {
            return send_error(req, 400, "bad_request",
                              "submode must be audio_only|fusion|radar_follow");
        }
    }

    if (has_oor) {
        if (strcmp(oor_str, "hold") == 0) {
            mode_manager_set_oor_policy(OOR_HOLD);
        } else if (strcmp(oor_str, "clamp") == 0) {
            mode_manager_set_oor_policy(OOR_CLAMP);
        } else if (strcmp(oor_str, "home") == 0) {
            mode_manager_set_oor_policy(OOR_HOME);
        } else if (strcmp(oor_str, "scan") == 0) {
            mode_manager_set_oor_policy(OOR_SCAN);
        } else {
            return send_error(req, 400, "bad_request",
                              "oor must be hold|clamp|home|scan");
        }
    }

    int still_min = -1;   /* -1 = field absent */
    json_get_int(body, "still_min", &still_min);
    if (still_min > 1440) {
        return send_error(req, 400, "bad_request", "still_min must be 0..1440");
    }
    if (still_min >= 0) {
        mode_manager_set_still_min((uint16_t)still_min);
    }

    int gate = -1;
    json_get_int(body, "assoc_gate", &gate);
    if (gate > 1) {
        return send_error(req, 400, "bad_request", "assoc_gate must be 0 or 1");
    }
    if (gate >= 0) {
        mode_manager_set_assoc_gate(gate != 0);
    }

    static const char *subnames[] = {"audio_only", "fusion", "radar_follow"};
    static const char *oornames[] = {"hold", "clamp", "home", "scan"};
    char resp[144];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"mode\":\"%s\",\"submode\":\"%s\",\"oor\":\"%s\","
             "\"still_min\":%u,\"assoc_gate\":%s}",
             mode_manager_get() == MODE_COMMAND ? "command" : "track",
             subnames[mode_manager_get_submode()],
             oornames[mode_manager_get_oor_policy()],
             (unsigned)mode_manager_get_still_min(),
             mode_manager_get_assoc_gate() ? "true" : "false");
    return send_json_ok(req, resp);
}

/* GET /api/radar?device_id=XXXX — radar config readback (US-010):
 * factory bounds + user config + sensing state, queried on demand. */
static esp_err_t handler_radar_get(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;

    radar_cfg_t c;
    if (!radar_req_get_cfg(&c, 3000)) {
        return send_error(req, 503, "radar_busy",
                          "radar busy/offline, try again");
    }
    char body[480];
    snprintf(body, sizeof(body),
        "{\"ok\":true,\"online\":%s,\"sensing\":%s,"
        "\"bounds\":{\"mot\":[%u,%u],\"micro\":[%u,%u],\"bhr\":[%u,%u]},"
        "\"cfg\":{\"mot\":[%u,%u],\"mot_lvl\":%u,"
        "\"micro\":[%u,%u],\"micro_lvl\":%u,"
        "\"bhr\":[%u,%u],\"bhr_lvl\":%u}}",
        c.online ? "true" : "false",
        c.sensing ? "true" : "false",
        c.b_mot_min, c.b_mot_max, c.b_micro_min, c.b_micro_max,
        c.b_bhr_min, c.b_bhr_max,
        c.mot_min, c.mot_max, c.mot_lvl,
        c.micro_min, c.micro_max, c.micro_lvl,
        c.bhr_min, c.bhr_max, c.bhr_lvl);
    return send_json_ok(req, body);
}

/* POST /api/radar?device_id=XXXX — apply config fields (all optional,
 * cm units, levels 0-10) and optionally save to the radar's flash.
 * Body: {"sensing":1,"mot_min":50,"mot_max":1000,"mot_lvl":5,
 *        "bhr_min":80,"bhr_max":255,"bhr_lvl":3,"save":1} */
static esp_err_t handler_radar_post(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;

    char body[MAX_BODY_LEN];
    if (!read_body(req, body, sizeof(body))) return ESP_OK;

    radar_set_req_t r = {0};
    int tmp;
    if (json_get_int(body, "sensing", &tmp)) {
        r.mask |= RAD_SET_SENSING;
        r.sensing = tmp != 0;
    }
    if (json_get_int(body, "mot_min", &tmp)) {
        if (tmp < 0 || tmp > 1400) return send_error(req, 400, "bad_request", "mot_min 0-1400 cm");
        r.mask |= RAD_SET_MOT_MIN; r.mot_min = (uint16_t)tmp;
    }
    if (json_get_int(body, "mot_max", &tmp)) {
        if (tmp < 0 || tmp > 1400) return send_error(req, 400, "bad_request", "mot_max 0-1400 cm");
        r.mask |= RAD_SET_MOT_MAX; r.mot_max = (uint16_t)tmp;
    }
    if (json_get_int(body, "bhr_min", &tmp)) {
        if (tmp < 0 || tmp > 650) return send_error(req, 400, "bad_request", "bhr_min 0-650 cm");
        r.mask |= RAD_SET_BHR_MIN; r.bhr_min = (uint16_t)tmp;
    }
    if (json_get_int(body, "bhr_max", &tmp)) {
        if (tmp < 0 || tmp > 650) return send_error(req, 400, "bad_request", "bhr_max 0-650 cm");
        r.mask |= RAD_SET_BHR_MAX; r.bhr_max = (uint16_t)tmp;
    }
    if (json_get_int(body, "mot_lvl", &tmp)) {
        if (tmp < 0 || tmp > 10) return send_error(req, 400, "bad_request", "mot_lvl 0-10");
        r.mask |= RAD_SET_MOT_LVL; r.mot_lvl = (uint16_t)tmp;
    }
    if (json_get_int(body, "bhr_lvl", &tmp)) {
        if (tmp < 0 || tmp > 10) return send_error(req, 400, "bad_request", "bhr_lvl 0-10");
        r.mask |= RAD_SET_BHR_LVL; r.bhr_lvl = (uint16_t)tmp;
    }
    if (json_get_int(body, "save", &tmp) && tmp) r.mask |= RAD_SET_SAVE;

    if (!r.mask) {
        return send_error(req, 400, "bad_request",
                          "no valid fields (sensing/mot_min/mot_max/mot_lvl/bhr_min/bhr_max/bhr_lvl/save)");
    }

    radar_cfg_t c;
    if (!radar_req_set_cfg(&r, &c, 8000)) {
        return send_error(req, 503, "radar_busy", "radar busy/offline, try again");
    }

    /* per-field verification against readback */
    bool v_ok = true;
    if ((r.mask & RAD_SET_MOT_MIN) && c.mot_min != r.mot_min) v_ok = false;
    if ((r.mask & RAD_SET_MOT_MAX) && c.mot_max != r.mot_max) v_ok = false;
    if ((r.mask & RAD_SET_BHR_MIN) && c.bhr_min != r.bhr_min) v_ok = false;
    if ((r.mask & RAD_SET_BHR_MAX) && c.bhr_max != r.bhr_max) v_ok = false;
    if ((r.mask & RAD_SET_SENSING) && c.sensing != r.sensing) v_ok = false;

    char out[480];
    snprintf(out, sizeof(out),
        "{\"ok\":true,\"verified\":%s,"
        "\"cfg\":{\"mot\":[%u,%u],\"mot_lvl\":%u,\"bhr\":[%u,%u],\"bhr_lvl\":%u,"
        "\"sensing\":%s}}",
        v_ok ? "true" : "false (readback differs — some settings are "
                       "ineffective in this firmware)",
        c.mot_min, c.mot_max, c.mot_lvl, c.bhr_min, c.bhr_max, c.bhr_lvl,
        c.sensing ? "true" : "false");
    return send_json_ok(req, out);
}

/* POST /api/radar/reset?device_id=XXXX — radar system reset. Also the
 * operator's "clear phantom target" button. The radar re-inits for ~5 s;
 * polling resumes automatically. */
static esp_err_t handler_radar_reset(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;
    if (!radar_req_reset(1000)) {
        return send_error(req, 503, "radar_busy", "radar busy, try again");
    }
    return send_json_ok(req,
        "{\"ok\":true,\"restarting\":true,\"eta_s\":5,"
         "\"note\":\"re-applies sensing-on + distance gates\"}");
}

/* POST /api/point?device_id=XXXX
 * Body: {"dir":"7oc"} or {"angle":30}
 * Only works in COMMAND mode. Rate-limited to 1 per 500ms.
 */
static esp_err_t handler_point(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;

    /* Reject during shake. */
    if (s_shaking) {
        return send_error(req, 409, "shaking",
                          "shake in progress, wait for completion");
    }

    /* Mode check: /point only in COMMAND mode. */
    if (mode_manager_get() != MODE_COMMAND) {
        return send_error(req, 403, "mode_is_track",
                          "switch to command mode first");
    }

    /* Rate limit. */
    int64_t now = esp_timer_get_time();
    if (now - s_last_point_us < POINT_MIN_INTERVAL_US) {
        return send_error(req, 429, "rate_limited",
                          "min 500ms between commands");
    }

    char body[MAX_BODY_LEN];
    if (!read_body(req, body, sizeof(body))) return ESP_OK;

    /* Parse direction: try "dir" first, then "angle". */
    float target = 0.0f;
    bool  have_target = false;

    char dir_str[8] = {0};
    if (json_get_str(body, "dir", dir_str, sizeof(dir_str))) {
        for (int i = 0; i < (int)N_DIR; i++) {
            if (strcmp(dir_str, s_dir_map[i].name) == 0) {
                target = s_dir_map[i].angle;
                have_target = true;
                break;
            }
        }
        if (!have_target) {
            return send_error(req, 400, "bad_request", "unknown dir value");
        }
    } else if (json_get_str(body, "angle", dir_str, sizeof(dir_str))) {
        target = strtof(dir_str, NULL);
        have_target = true;
    }

    if (!have_target) {
        return send_error(req, 400, "bad_request",
                          "missing 'dir' or 'angle' field");
    }

    /* Clamp to mechanical range, flag if clamped. */
    bool clamped = false;
    if (target > SERVO_ANGLE_MAX_DEG) {
        target = SERVO_ANGLE_MAX_DEG;
        clamped = true;
    }
    if (target < SERVO_ANGLE_MIN_DEG) {
        target = SERVO_ANGLE_MIN_DEG;
        clamped = true;
    }

    /* Command the servo (async — returns immediately). */
    s_last_point_us = now;
    mode_manager_register_command();
    servo_set_angle_deg(target);
    evlog_record(EV_SERVO_CMD, SRC_REST, (int16_t)target);

    ESP_LOGI(TAG, "point: target=%.1f clamped=%d", target, clamped);

    /* Build response. */
    char resp[80];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"servo\":%.1f%s}",
             target, clamped ? ",\"clamped\":true" : "");
    return send_json_ok(req, resp);
}

/* POST /api/shake?device_id=XXXX
 * Shake the servo ±10° (boundary-aware) around the current position.
 * Pattern: 3 oscillations, pause 2s, 2 oscillations, return to start.
 * Blocking: HTTP response returns after shake completes (~7s).
 * Only in COMMAND mode. /api/point and /api/mode rejected during shake.
 */
static esp_err_t handler_shake(httpd_req_t *req)
{
    if (!check_auth(req)) return ESP_OK;

    /* Mode check. */
    if (mode_manager_get() != MODE_COMMAND) {
        return send_error(req, 403, "mode_is_track",
                          "switch to command mode first");
    }

    /* Rate limit (shared with /api/point). */
    int64_t now = esp_timer_get_time();
    if (now - s_last_point_us < POINT_MIN_INTERVAL_US) {
        return send_error(req, 429, "rate_limited",
                          "min 500ms between commands");
    }

    /* Compute shake bounds from current position. */
    float center = servo_get_angle_deg();
    float hi = fminf(center + 10.0f, SERVO_ANGLE_MAX_DEG);
    float lo = fmaxf(center - 10.0f, SERVO_ANGLE_MIN_DEG);

    ESP_LOGI(TAG, "shake: center=%.1f hi=%.1f lo=%.1f", center, hi, lo);

    /* Set shaking flag — blocks /api/point and /api/mode. */
    s_shaking = true;
    s_last_point_us = now;
    mode_manager_register_command();
    evlog_record(EV_SHAKE_START, 0, (int16_t)center);

    /* Group 1: 3 oscillations (hi-lo-hi-lo-hi-lo). */
    for (int i = 0; i < 3; i++) {
        servo_set_angle_deg(hi);
        vTaskDelay(pdMS_TO_TICKS(400));
        servo_set_angle_deg(lo);
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    /* Return to center and pause. */
    servo_set_angle_deg(center);
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Group 2: 2 oscillations. */
    for (int i = 0; i < 2; i++) {
        servo_set_angle_deg(hi);
        vTaskDelay(pdMS_TO_TICKS(400));
        servo_set_angle_deg(lo);
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    /* Return to original position. */
    servo_set_angle_deg(center);
    vTaskDelay(pdMS_TO_TICKS(300));  /* let servo settle */

    s_shaking = false;
    ESP_LOGI(TAG, "shake done, back at %.1f", center);
    evlog_record(EV_SHAKE_END, 0, (int16_t)center);

    return send_json_ok(req, "{\"ok\":true}");
}

/* ---- Server start ---- */
esp_err_t rest_api_start(void)
{
    if (s_server) {
        ESP_LOGW(TAG, "HTTP server already running");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 6144;
    config.max_uri_handlers = 16;   /* 12 registered; over-limit handlers fail silently */
    config.task_priority = configMAX_PRIORITIES - 5;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        return err;
    }

    static const httpd_uri_t uri_root = {
        .uri = "/", .method = HTTP_GET, .handler = handler_root
    };
    static const httpd_uri_t uri_ping = {
        .uri = "/api/ping", .method = HTTP_GET, .handler = handler_ping
    };
    static const httpd_uri_t uri_status = {
        .uri = "/api/status", .method = HTTP_GET, .handler = handler_status
    };
    static const httpd_uri_t uri_logs = {
        .uri = "/api/logs", .method = HTTP_GET, .handler = handler_logs
    };
    static const httpd_uri_t uri_events = {
        .uri = "/api/events", .method = HTTP_GET, .handler = handler_events
    };
    static const httpd_uri_t uri_mode = {
        .uri = "/api/mode", .method = HTTP_POST, .handler = handler_mode
    };
    static const httpd_uri_t uri_radar_get = {
        .uri = "/api/radar", .method = HTTP_GET, .handler = handler_radar_get
    };
    static const httpd_uri_t uri_radar_post = {
        .uri = "/api/radar", .method = HTTP_POST, .handler = handler_radar_post
    };
    static const httpd_uri_t uri_radar_reset = {
        .uri = "/api/radar/reset", .method = HTTP_POST, .handler = handler_radar_reset
    };
    static const httpd_uri_t uri_point = {
        .uri = "/api/point", .method = HTTP_POST, .handler = handler_point
    };
    static const httpd_uri_t uri_shake = {
        .uri = "/api/shake", .method = HTTP_POST, .handler = handler_shake
    };
    static const httpd_uri_t uri_options = {
        .uri = "/*", .method = HTTP_OPTIONS, .handler = handler_options
    };

    httpd_register_uri_handler(s_server, &uri_root);
    httpd_register_uri_handler(s_server, &uri_ping);
    httpd_register_uri_handler(s_server, &uri_status);
    httpd_register_uri_handler(s_server, &uri_logs);
    httpd_register_uri_handler(s_server, &uri_events);
    httpd_register_uri_handler(s_server, &uri_mode);
    httpd_register_uri_handler(s_server, &uri_radar_get);
    httpd_register_uri_handler(s_server, &uri_radar_post);
    httpd_register_uri_handler(s_server, &uri_radar_reset);
    httpd_register_uri_handler(s_server, &uri_point);
    httpd_register_uri_handler(s_server, &uri_shake);
    httpd_register_uri_handler(s_server, &uri_options);

    ESP_LOGI(TAG, "REST API started: /api/ping /api/status /api/logs /api/events /api/mode /api/radar /api/radar/reset /api/point /api/shake");
    return ESP_OK;
}
