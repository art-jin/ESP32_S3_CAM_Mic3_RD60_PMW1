/* Phase 0 protocol probe for the MS60-1211S80M (AT6010) 60GHz radar.
 *
 * Wiring (see ArthurReadMe.md): radar RX <- GPIO8 (ESP32 TX),
 * radar TX -> GPIO9 (ESP32 RX).
 *
 * Two modes selected by RP_SWEEP:
 *  - RP_SWEEP=1 (default): baud-rate sweep. For each candidate baud, listen
 *    3 s (passive), then 3 s while sending the version query every second.
 *    Prints raw RX byte counts + first bytes hex per baud, forever. Any
 *    nonzero count means the link is alive at that baud.
 *  - RP_SWEEP=0: fixed 921600 8N1 framing + decode of 0x5A active reports
 *    and 0x59 replies, with a 10 s version heartbeat and 10 s stats.
 *
 * Frame formats (reference/雷达通信协议.pdf):
 *  - Active report 0x5A: HEAD LEN payload(TYPE+struct, LEN bytes) CHECK8.
 *    Sent only while a target is detected — silence = no target.
 *  - Host command 0x58 / reply 0x59: HEAD CMD PARAMLEN params CK16 (LE).
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "radar_probe.h"

#define RP_UART_NUM   UART_NUM_1
#define RP_BAUD       115200  /* module default; NOT the 921600 in the AT6010 doc */
#define RP_TX_GPIO    8    /* ESP32 TX -> radar RX */
#define RP_RX_GPIO    9    /* ESP32 RX <- radar TX */
#define RP_RX_BUF     4096
#define RP_ACC_MAX    1024

#ifndef RP_SWEEP
#define RP_SWEEP      0
#endif

static const char *TAG = "rprobe";

/* 3.1.10 获取软硬件版本号 / 3.2.2 感应状态查询 / 3.2.1 打开感应 /
 * 3.2.6 获取感应信息 (same 20-byte struct as TYPE=0 report). */
static const uint8_t VER_REQ[5]  = {0x58, 0xFE, 0x00, 0x56, 0x01};
static const uint8_t SENS_Q[5]   = {0x58, 0xD0, 0x00, 0x28, 0x01};
static const uint8_t SENS_ON[6]  = {0x58, 0xD1, 0x01, 0x01, 0x2B, 0x01};
static const uint8_t DET_Q[5]    = {0x58, 0x30, 0x00, 0x88, 0x00};

static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int16_t  rd_s16(const uint8_t *p) { return (int16_t)rd_u16(p); }
static uint32_t rd_u32(const uint8_t *p) { return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24)); }

static void hex_str(const uint8_t *p, size_t n, char *out, size_t cap)
{
    size_t o = 0;
    for (size_t i = 0; i < n && o + 4 < cap; i++)
        o += (size_t)snprintf(out + o, cap - o, "%02X ", p[i]);
    if (o && o < cap) out[o - 1] = '\0';
    else if (o < cap) out[0] = '\0';
}

static void det_str(uint8_t det, char *out, size_t cap)
{
    out[0] = '\0';
    if (det & 0x01) strlcat(out, "靠近", cap);
    if (det & 0x02) strlcat(out, "远离", cap);
    if (det & 0x04) strlcat(out, "运动", cap);
    if (det & 0x08) strlcat(out, "微动", cap);
    if (det & 0x10) strlcat(out, "呼吸", cap);
    if (!out[0])    snprintf(out, cap, "0x%02X", det);
}

#if !RP_SWEEP

static uint8_t  s_acc[RP_ACC_MAX];
static size_t   s_len;
static uint32_t c_bytes, c_stray, c_bad_ck, c_f59, c_hb_reply, c_ver_printed;
static uint32_t c_type[8];

static void handle_5a(const uint8_t *payload, uint8_t len)
{
    uint8_t type = payload[0];
    if (type < 8) c_type[type]++;
    uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000);
    char hex[160];
    hex_str(payload, len, hex, sizeof(hex));

    switch (type) {
    case 0: {  /* fmcw_det_info_t: full detection, payload = 1+20 bytes */
        char ds[32];
        det_str(payload[2], ds, sizeof(ds));
        ESP_LOGI(TAG, "[%6ums] T0 fidx=%-6u det=%u(%s) range=%umm angle=%+d° "
                 "velo=%d rb_conf=%u ang_conf=%u | %s",
                 ms, rd_u32(payload + 17), payload[1], ds,
                 rd_u16(payload + 3), rd_s16(payload + 5), rd_s16(payload + 7),
                 payload[15], payload[16], hex);
        break;
    }
    case 3: {  /* mot_det_info_t: motion/presence, payload = 1+7 bytes */
        char ds[32];
        det_str(payload[2], ds, sizeof(ds));
        ESP_LOGI(TAG, "[%6ums] T3 det=%u(%s) range=%umm angle=%+d° velo=%d | %s",
                 ms, payload[1], ds, rd_u16(payload + 3),
                 rd_s16(payload + 5), rd_s16(payload + 7), hex);
        break;
    }
    case 5: {  /* rgn_det_info_t: zone detection, payload = 1+16 bytes */
        uint32_t n = rd_u32(payload + 1);
        ESP_LOGI(TAG, "[%6ums] T5 obj=%u", ms, n);
        for (uint32_t k = 0; k < 3; k++) {
            const uint8_t *o = payload + 5 + k * 4;
            ESP_LOGI(TAG, "    zone%u: range=%umm angle=%+d°",
                     k, rd_u16(o), rd_s16(o + 2));
        }
        break;
    }
    default:
        ESP_LOGI(TAG, "[%6ums] T%u len=%u | %s", ms, type, len, hex);
        break;
    }
}

/* Decode a 3.2.6 (cmd 0x30) reply payload: same fmcw_det_info_t as TYPE=0. */
static void decode_fmcw(const uint8_t *f, uint32_t ms, const char *src)
{
    char ds[32];
    det_str(f[1], ds, sizeof(ds));
    ESP_LOGI(TAG, "[%6ums] %s fidx=%-6u det=%u(%s) range=%umm angle=%+d° "
             "velo=%d rb_conf=%u ang_conf=%u",
             ms, src, rd_u32(f + 16), f[0], ds,
             rd_u16(f + 2), rd_s16(f + 4), rd_s16(f + 6), f[14], f[15]);
}

static void handle_59(const uint8_t *buf, uint8_t plen)
{
    c_f59++;
    char hex[160];
    hex_str(buf, (size_t)plen + 2, hex, sizeof(hex));
    uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (buf[0] == 0xFE && !c_ver_printed && plen >= 8) {
        c_ver_printed = 1;
        c_hb_reply++;
        ESP_LOGI(TAG, "radar version: SDK v%d.%d.%d cust v%d.%d HW v%d.%d",
                 buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8]);
    } else if (buf[0] == 0x30 && plen >= 20) {
        decode_fmcw(buf + 2, ms, "POLL");
        return;
    } else if (buf[0] == 0xD0) {
        ESP_LOGI(TAG, "sensing status: %s", plen >= 1 && buf[2] ? "ON" : "OFF");
    }
    ESP_LOGI(TAG, "reply cmd=0x%02X | %s", buf[0], hex);
}

static void probe_task(void *arg)
{
    (void)arg;
    int64_t last_hb = 0, last_stats = 0;
    int64_t last_5a_ms = -30000;   /* pretend last report was 30s ago */
    int64_t last_poll = 0;
    int sent_sens_q = 0, sent_sens_on = 0;

    while (1) {
        /* Startup sequence: query sensing state, then force sensing on. */
        int64_t now0 = esp_timer_get_time() / 1000;
        if (!sent_sens_q && now0 > 2000) {
            uart_write_bytes(RP_UART_NUM, SENS_Q, sizeof(SENS_Q));
            sent_sens_q = 1;
        }
        if (sent_sens_q && !sent_sens_on && now0 > 5000) {
            uart_write_bytes(RP_UART_NUM, SENS_ON, sizeof(SENS_ON));
            sent_sens_on = 1;
            ESP_LOGI(TAG, "sent sensing-ON command (58 D1 01 01 2B 01)");
        }

        size_t room = RP_ACC_MAX - s_len;
        if (!room) { s_len = 0; c_stray += RP_ACC_MAX; }
        int n = uart_read_bytes(RP_UART_NUM, s_acc + s_len, room, pdMS_TO_TICKS(20));
        if (n > 0) { s_len += (size_t)n; c_bytes += (uint32_t)n; }

        size_t i = 0;
        while (i < s_len) {
            uint8_t h = s_acc[i];
            if (h == 0x5A) {
                if (i + 2 > s_len) break;
                uint8_t len = s_acc[i + 1];
                size_t total = 2u + len + 1u;
                if (i + total > s_len) break;
                uint8_t ck = 0;
                for (size_t k = 0; k < 2u + len; k++) ck += s_acc[i + k];
                if (ck == s_acc[i + 2 + len]) {
                    handle_5a(&s_acc[i + 2], len);
                    last_5a_ms = (int64_t)(esp_timer_get_time() / 1000);
                    i += total;
                }
                else { c_bad_ck++; i++; }
            } else if (h == 0x59) {
                if (i + 3 > s_len) break;
                uint8_t plen = s_acc[i + 2];
                size_t total = 3u + plen + 2u;
                if (i + total > s_len) break;
                uint16_t sum = 0;
                for (size_t k = 0; k < 3u + plen; k++) sum += s_acc[i + k];
                if (sum == rd_u16(&s_acc[i + 3 + plen])) { handle_59(&s_acc[i + 1], plen); i += total; }
                else { c_bad_ck++; i++; }
            } else { c_stray++; i++; }
        }
        if (i) { memmove(s_acc, s_acc + i, s_len - i); s_len -= i; }

        int64_t now = esp_timer_get_time();
        if (now - last_hb >= 10 * 1000 * 1000LL) {
            uart_write_bytes(RP_UART_NUM, VER_REQ, sizeof(VER_REQ));
            last_hb = now;
        }
        /* No 0x5A stream for 30 s -> fall back to polling 3.2.6 every 1 s. */
        int64_t now_ms = now / 1000;
        if (sent_sens_on && now_ms - last_5a_ms > 30000 &&
            now_ms - last_poll >= 1000) {
            uart_write_bytes(RP_UART_NUM, DET_Q, sizeof(DET_Q));
            last_poll = now_ms;
        }
        if (now - last_stats >= 10 * 1000 * 1000LL) {
            ESP_LOGI(TAG, "STAT rx=%uB T0=%u T1=%u T2=%u T3=%u T4=%u T5=%u "
                     "reply=%u hb_ok=%u bad_ck=%u stray=%u",
                     c_bytes, c_type[0], c_type[1], c_type[2], c_type[3],
                     c_type[4], c_type[5], c_f59, c_hb_reply, c_bad_ck, c_stray);
            c_bytes = c_stray = c_bad_ck = c_f59 = 0;
            for (int k = 0; k < 8; k++) c_type[k] = 0;
            last_stats = now;
        }
    }
}

#else  /* RP_SWEEP */

/* SuperMini silkscreen is offset +2 from true GPIO numbers (silkscreen N =
 * GPIO N+2), so wires placed "on GPIO8/9" may actually land on GPIO10/11.
 * Also covers the TX/RX-swapped case. Each combo is tried at both the
 * documented default baud and the common 115200. */
static const struct { int tx, rx; } k_combos[] = {
    {8, 9}, {9, 8}, {10, 11}, {11, 10},
};
static const int k_bauds[] = {921600, 115200};

/* One listen window. Returns bytes received; fills first bytes hex. */
static uint32_t listen_window(uint32_t ms, int send_hb, char *first_hex, size_t hex_cap)
{
    uint8_t buf[256];
    uint32_t got = 0;
    size_t first_n = 0;
    uint8_t first[16] = {0};
    int64_t start = esp_timer_get_time();
    int64_t last_tx = 0;

    while ((uint32_t)(esp_timer_get_time() - start) < ms * 1000LL) {
        if (send_hb && esp_timer_get_time() - last_tx >= 800 * 1000LL) {
            uart_write_bytes(RP_UART_NUM, VER_REQ, sizeof(VER_REQ));
            last_tx = esp_timer_get_time();
        }
        int n = uart_read_bytes(RP_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (n > 0) {
            got += (uint32_t)n;
            if (first_n < sizeof(first)) {
                size_t take = (size_t)n;
                if (take > sizeof(first) - first_n) take = sizeof(first) - first_n;
                memcpy(first + first_n, buf, take);
                first_n += take;
            }
        }
    }
    hex_str(first, first_n, first_hex, hex_cap);
    return got;
}

static void probe_task(void *arg)
{
    (void)arg;
    char hex[64];

    while (1) {
        for (size_t c = 0; c < sizeof(k_combos) / sizeof(k_combos[0]); c++) {
            uart_set_pin(RP_UART_NUM, k_combos[c].tx, k_combos[c].rx,
                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            for (size_t b = 0; b < sizeof(k_bauds) / sizeof(k_bauds[0]); b++) {
                uart_set_baudrate(RP_UART_NUM, k_bauds[b]);
                uart_flush_input(RP_UART_NUM);
                uint32_t rx_passive = listen_window(2500, 0, hex, sizeof(hex));
                uint32_t rx_active  = listen_window(2500, 1, hex, sizeof(hex));
                ESP_LOGI(TAG, "tx=%-2d rx=%-2d %-7u listen=%uB txpoll=%uB %s",
                         k_combos[c].tx, k_combos[c].rx, k_bauds[b],
                         rx_passive, rx_active,
                         (rx_passive || rx_active) ? hex : "");
            }
        }
        ESP_LOGI(TAG, "--- combo sweep done, restarting ---");
    }
}

#endif

void radar_probe_start(void)
{
    const uart_config_t cfg = {
        .baud_rate = RP_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RP_UART_NUM, RP_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RP_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RP_UART_NUM, RP_TX_GPIO, RP_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

#if RP_SWEEP
    ESP_LOGI(TAG, "=== radar baud sweep: UART1 tx=%d rx=%d, 8N1, %u bauds ===",
             RP_TX_GPIO, RP_RX_GPIO,
             (unsigned)(sizeof(k_bauds) / sizeof(k_bauds[0])));
#else
    ESP_LOGI(TAG, "=== radar protocol probe: UART1 921600 8N1 tx=%d rx=%d ===",
             RP_TX_GPIO, RP_RX_GPIO);
    ESP_LOGI(TAG, "heartbeat 10s; empty room = no 0x5A frames");
#endif

    xTaskCreate(probe_task, "rprobe", 4096, NULL, 5, NULL);
}
