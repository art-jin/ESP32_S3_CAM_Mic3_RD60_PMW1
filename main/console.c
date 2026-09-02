#include "console.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "doa.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo.h"
#include "tracker.h"

static const char *TAG = "console";

#define CONSOLE_UART       UART_NUM_0
#define CONSOLE_BAUD       115200
#define CONSOLE_BUF_LEN    256
#define CONSOLE_STACK      4096

static char s_line[CONSOLE_BUF_LEN];

/* Output helper: write directly to UART (bypasses any stdio buffering).
 * ESP_LOG also uses this UART, so our output interleaves with log lines
 * — that's fine for a debug console. */
static void out(const char *s)
{
    uart_write_bytes(CONSOLE_UART, s, strlen(s));
}

static void outf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) uart_write_bytes(CONSOLE_UART, buf, n);
}

/* ---- Command handlers ---- */

static int cmd_help(void)
{
    out("Commands:\r\n");
    out("  help                    Show this list\r\n");
    out("  status                  Print servo / tracker / DOA snapshot\r\n");
    out("  servo <deg>             Manual: move servo (clamped ±20°), disables tracker\r\n");
    out("  tracker <on|off>        Enable / disable auto-tracking\r\n");
    out("  cfg                     Print current tracker config\r\n");
    out("  cfg <key> <val>         Set config: home|deadband|minconf|range|agree\r\n");
    return 0;
}

static int cmd_status(void)
{
    out("=== status ===\r\n");
    outf("servo:        %+6.2f deg  %s\r\n",
          servo_get_angle_deg(), servo_is_moving() ? "[MOTION]" : "");
    outf("tracker:      %s  mode=%d\r\n",
          tracker_is_enabled() ? "enabled" : "DISABLED",
          tracker_get_mode());
    const tracker_config_t *c = tracker_get_config();
    outf("  home:       %+.1f deg\r\n", c->home_deg);
    outf("  deadband:   %.1f deg\r\n",  c->deadband_deg);
    outf("  minconf:    %.2f\r\n",      c->min_confidence);
    outf("  range:      %.1f deg\r\n",  c->out_of_range_deg);
    outf("  agree:      %.1f deg\r\n",  c->target_agreement_deg);
    outf("  conserv:    %s\r\n",        c->conservative_mode ? "on" : "off");
    return 0;
}

static int cmd_servo(const char *arg)
{
    if (!arg || !*arg) {
        out("usage: servo <deg>\r\n");
        return 1;
    }
    float angle = strtof(arg, NULL);
    tracker_set_enabled(false);
    servo_set_angle_deg(angle);
    outf("servo -> %+.2f deg (tracker disabled; 'tracker on' to resume)\r\n",
         servo_get_angle_deg());
    return 0;
}

static int cmd_tracker(const char *arg)
{
    if (!arg || !*arg) {
        out("usage: tracker <on|off>\r\n");
        return 1;
    }
    if (strcmp(arg, "on") == 0) {
        tracker_set_enabled(true);
        out("tracker enabled\r\n");
    } else if (strcmp(arg, "off") == 0) {
        tracker_set_enabled(false);
        out("tracker disabled\r\n");
    } else {
        out("usage: tracker <on|off>\r\n");
        return 1;
    }
    return 0;
}

static int cmd_cfg(const char *arg)
{
    tracker_config_t cfg = *tracker_get_config();

    if (!arg || !*arg) {
        outf("home=%.2f deadband=%.2f minconf=%.2f range=%.2f agree=%.2f conserv=%s\r\n",
             cfg.home_deg, cfg.deadband_deg, cfg.min_confidence,
             cfg.out_of_range_deg, cfg.target_agreement_deg,
             cfg.conservative_mode ? "on" : "off");
        out("Usage: cfg <key> <val>  keys: home|deadband|minconf|range|agree\r\n");
        return 0;
    }

    char key[32];
    const char *p = arg;
    size_t klen = 0;
    while (*p && !isspace((unsigned char)*p) && klen < sizeof(key) - 1) {
        key[klen++] = *p++;
    }
    key[klen] = 0;
    while (*p && isspace((unsigned char)*p)) p++;

    if (!*p) {
        outf("Missing value for '%s'\r\n", key);
        return 1;
    }
    float val = strtof(p, NULL);

    int ok = 1;
    if      (strcmp(key, "home")     == 0) cfg.home_deg             = val;
    else if (strcmp(key, "deadband") == 0) cfg.deadband_deg         = val;
    else if (strcmp(key, "minconf")  == 0) cfg.min_confidence       = val;
    else if (strcmp(key, "range")    == 0) cfg.out_of_range_deg     = val;
    else if (strcmp(key, "agree")    == 0) cfg.target_agreement_deg = val;
    else {
        outf("Unknown key '%s'. Keys: home|deadband|minconf|range|agree\r\n", key);
        ok = 0;
    }
    if (ok) {
        tracker_set_config(&cfg);
        outf("cfg %s = %.2f (applied)\r\n", key, val);
    }
    return ok ? 0 : 1;
}

static void process_line(char *line)
{
    size_t n = strlen(line);
    while (n > 0 && isspace((unsigned char)line[n - 1])) line[--n] = 0;
    if (n == 0) return;

    char cmd[32];
    size_t clen = 0;
    const char *p = line;
    while (*p && !isspace((unsigned char)*p) && clen < sizeof(cmd) - 1) {
        cmd[clen++] = tolower((unsigned char)*p++);
    }
    cmd[clen] = 0;
    while (*p && isspace((unsigned char)*p)) p++;

    if      (strcmp(cmd, "help")    == 0) cmd_help();
    else if (strcmp(cmd, "status")  == 0) cmd_status();
    else if (strcmp(cmd, "servo")   == 0) cmd_servo(p);
    else if (strcmp(cmd, "tracker") == 0) cmd_tracker(p);
    else if (strcmp(cmd, "cfg")     == 0) cmd_cfg(p);
    else {
        outf("Unknown command '%s'. Type 'help'.\r\n", cmd);
    }
}

static void console_task(void *arg)
{
    (void)arg;
    /* Install UART driver. We use uart_read_bytes for input and
     * uart_write_bytes for output (via out()/outf()), bypassing the
     * VFS / stdio layer entirely. ESP_LOG output coexists on the same
     * UART via ROM printf — both pathways share the hardware FIFO. */
    esp_err_t err = uart_driver_install(CONSOLE_UART, CONSOLE_BUF_LEN * 2,
                                        CONSOLE_BUF_LEN * 2, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    uart_set_rx_timeout(CONSOLE_UART, 1);

    out("\r\nType 'help' for commands.\r\n> ");

    size_t pos = 0;
    while (1) {
        uint8_t c;
        int n = uart_read_bytes(CONSOLE_UART, &c, 1, portMAX_DELAY);
        if (n <= 0) continue;

        if (c == '\r' || c == '\n') {
            s_line[pos] = 0;
            out("\r\n");
            process_line(s_line);
            out("> ");
            pos = 0;
        } else if (c == 0x7f || c == 0x08) {
            if (pos > 0) {
                pos--;
                out("\b \b");
            }
        } else if (pos < sizeof(s_line) - 1 && isprint(c)) {
            s_line[pos++] = (char)c;
            char echo[2] = { (char)c, 0 };
            out(echo);
        }
    }
}

void console_init(void)
{
    xTaskCreate(console_task, "console", CONSOLE_STACK, NULL, 1, NULL);
    ESP_LOGI(TAG, "console ready on UART0 (%d baud). Type 'help'.", CONSOLE_BAUD);
}
