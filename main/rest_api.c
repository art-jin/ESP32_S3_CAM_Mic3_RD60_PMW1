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

    char body[400];
    snprintf(body, sizeof(body),
        "{\"ok\":true,"
        "\"mode\":\"%s\","
        "\"servo\":%.1f,"
        "\"moving\":%s,"
        "\"azimuth\":%.0f,"
        "\"sect\":\"%s\","
        "\"conf\":%.2f,"
        "\"wifi\":\"%s\","
        "\"ip\":\"%s\","
        "\"host\":\"%s\""
        "}",
        st.mode == MODE_COMMAND ? "command" : "track",
        st.servo_angle,
        st.servo_moving ? "true" : "false",
        st.azimuth,
        sect_label,
        st.confidence,
        st.wifi_connected ? "connected" : "disconnected",
        st.ip[0] ? st.ip : "",
        st.hostname[0] ? st.hostname : "");

    return send_json_ok(req, body);
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

    if (!has_mode && !has_sub && !has_oor) {
        return send_error(req, 400, "bad_request",
                          "need at least one of 'mode'/'submode'/'oor'");
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

    static const char *subnames[] = {"audio_only", "fusion", "radar_follow"};
    static const char *oornames[] = {"hold", "clamp", "home", "scan"};
    char resp[96];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"mode\":\"%s\",\"submode\":\"%s\",\"oor\":\"%s\"}",
             mode_manager_get() == MODE_COMMAND ? "command" : "track",
             subnames[mode_manager_get_submode()],
             oornames[mode_manager_get_oor_policy()]);
    return send_json_ok(req, resp);
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
    config.max_uri_handlers = 8;
    config.task_priority = configMAX_PRIORITIES - 5;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        return err;
    }

    static const httpd_uri_t uri_ping = {
        .uri = "/api/ping", .method = HTTP_GET, .handler = handler_ping
    };
    static const httpd_uri_t uri_status = {
        .uri = "/api/status", .method = HTTP_GET, .handler = handler_status
    };
    static const httpd_uri_t uri_logs = {
        .uri = "/api/logs", .method = HTTP_GET, .handler = handler_logs
    };
    static const httpd_uri_t uri_mode = {
        .uri = "/api/mode", .method = HTTP_POST, .handler = handler_mode
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

    httpd_register_uri_handler(s_server, &uri_ping);
    httpd_register_uri_handler(s_server, &uri_status);
    httpd_register_uri_handler(s_server, &uri_logs);
    httpd_register_uri_handler(s_server, &uri_mode);
    httpd_register_uri_handler(s_server, &uri_point);
    httpd_register_uri_handler(s_server, &uri_shake);
    httpd_register_uri_handler(s_server, &uri_options);

    ESP_LOGI(TAG, "REST API started: /api/ping /api/status /api/logs /api/mode /api/point /api/shake");
    return ESP_OK;
}
