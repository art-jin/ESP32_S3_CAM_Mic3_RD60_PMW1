/* epd_pages.c — docs/74 M1 模式渲染器。
 * 像素/字体原语与 epd_calendar.c 同款(set_pixel/fill_rect/u8g2 桥),
 * 刻意各留一份静态拷贝:月历页是已验收代码,不动它。
 * 布局全部由 EPD_WIDTH/HEIGHT 推导,4.2/5.83 吋同源(74 §7)。 */
#include "epd_pages.h"
#include "display_mode.h"
#include "epd.h"
#include "epd_calendar.h"
#include "fonts.h"
#include "u8g2_font.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "evlog.h"
#include "mqtt_publisher.h"
#include "radar.h"
#include "status.h"

#define CLR_BLACK 0
#define CLR_RED   1
#define CLR_WHITE 2
#define U_BLACK  0x0000
#define U_RED    0xF800
#define U_WHITE  0xFFFF

static uint8_t *g_bw, *g_red;

/* ---- 像素/文本原语(epd_calendar.c 同款) ---- */

static void set_pixel(int x, int y, int color)
{
    if (x < 0 || x >= EPD_WIDTH || y < 0 || y >= EPD_HEIGHT) return;
    int idx = y * (EPD_WIDTH / 8) + x / 8;
    int bit = 7 - (x % 8);
    switch (color) {
    case CLR_BLACK: g_bw[idx] &= ~(1 << bit); g_red[idx] &= ~(1 << bit); break;
    case CLR_RED:   g_bw[idx] &= ~(1 << bit); g_red[idx] |=  (1 << bit); break;
    default:        g_bw[idx] |=  (1 << bit); g_red[idx] &= ~(1 << bit); break;
    }
}

static void fill_rect(int x0, int y0, int x1, int y1, int color)
{
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            set_pixel(x, y, color);
}

static void frame_rect(int x0, int y0, int x1, int y1, int color, int t)
{
    fill_rect(x0, y0, x1, y0 + t - 1, color);
    fill_rect(x0, y1 - t + 1, x1, y1, color);
    fill_rect(x0, y0, x0 + t - 1, y1, color);
    fill_rect(x1 - t + 1, y0, x1, y1, color);
}

static void u8g2_draw_hv_line(u8g2_font_t *u, int16_t x, int16_t y,
                              int16_t len, uint8_t dir, uint16_t color)
{
    int c = (color == U_RED) ? CLR_RED
          : (color == U_WHITE) ? CLR_WHITE : CLR_BLACK;
    for (int i = 0; i < len; i++) {
        if (dir == 0) set_pixel(x + i, y, c);
        else          set_pixel(x, y + i, c);
    }
}

static uint16_t utf8_next(const char **s)
{
    const uint8_t *p = (const uint8_t *)*s;
    uint16_t cp;
    if (p[0] < 0x80) { cp = p[0]; *s += 1; }
    else if ((p[0] & 0xE0) == 0xC0) {
        cp = (uint16_t)((p[0] & 0x1F) << 6 | (p[1] & 0x3F)); *s += 2;
    } else {
        cp = (uint16_t)((p[0] & 0x0F) << 12 | (p[1] & 0x3F) << 6 | (p[2] & 0x3F));
        *s += 3;
    }
    return cp;
}

static int16_t draw_utf8(u8g2_font_t *font, int16_t x, int16_t y, const char *s)
{
    while (*s) {
        uint16_t cp = utf8_next(&s);
        x += u8g2_DrawGlyph(font, x, y, cp);
    }
    return x;
}

static int16_t utf8_width(u8g2_font_t *font, const char *s)
{
    int16_t w = 0;
    while (*s) {
        uint16_t cp = utf8_next(&s);
        w += u8g2_GetGlyphWidth(font, cp);
    }
    return w;
}

/* ---- 文本页:weather / msg / todo 共用(title + 行列表) ---- */

static void draw_title_bar(u8g2_font_t *font, const char *title)
{
    int h = (EPD_HEIGHT >= 400) ? 34 : 26;
    fill_rect(0, 0, EPD_WIDTH - 1, h - 1, CLR_BLACK);
    u8g2_SetFont(font, u8g2_font_wqy12_t_gb2312a);
    u8g2_SetFontMode(font, 1);
    u8g2_SetForegroundColor(font, U_WHITE);
    draw_utf8(font, 10, h - (EPD_HEIGHT >= 400 ? 10 : 8), title);
}

static void draw_lines_page(uint8_t *bw, uint8_t *red,
                            const char *title, const char *const *lines,
                            int n_lines)
{
    memset(bw, 0xFF, EPD_BUF_SIZE);
    memset(red, 0x00, EPD_BUF_SIZE);
    g_bw = bw; g_red = red;

    u8g2_font_t font;
    memset(&font, 0, sizeof(font));
    font.draw_hv_line = u8g2_draw_hv_line;

    draw_title_bar(&font, title);

    u8g2_SetFont(&font, u8g2_font_wqy12_t_gb2312a);
    u8g2_SetFontMode(&font, 1);
    u8g2_SetBackgroundColor(&font, U_WHITE);

    int line_h = (EPD_HEIGHT >= 400) ? 26 : 22;
    int y = (EPD_HEIGHT >= 400 ? 34 : 26) + line_h;
    for (int i = 0; i < n_lines && y < EPD_HEIGHT - 6; i++) {
        u8g2_SetForegroundColor(&font, U_BLACK);
        /* 简单截断(不折行)— 载荷侧负责行宽 */
        char buf[96];
        strncpy(buf, lines[i], sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        draw_utf8(&font, 12, y, buf);
        y += line_h;
    }
}

static void render_lines_from_json(uint8_t *bw, uint8_t *red,
                                   const char *payload, const char *mode_name,
                                   const char *items_key)
{
    const char *lines[12];
    int n = 0;
    char title[48] = "";

    cJSON *root = cJSON_Parse(payload);
    if (root) {
        cJSON *obj = cJSON_GetObjectItem(root, mode_name);
        if (obj) {
            cJSON *t = cJSON_GetObjectItem(obj, "title");
            if (cJSON_IsString(t))
                snprintf(title, sizeof(title), "%s", t->valuestring);
        }
        /* weather/msg: lines[];todo: items[] → "HH:MM 文本 [✓]" */
        const char *key = items_key ? items_key : "lines";
        cJSON *arr = cJSON_GetObjectItem(obj ? obj : root, key);
        if (cJSON_IsArray(arr)) {
            cJSON *it;
            cJSON_ArrayForEach(it, arr) {
                if (n >= 12) break;
                static char row[12][96];
                if (cJSON_IsString(it)) {
                    snprintf(row[n], 96, "%s", it->valuestring);
                } else if (cJSON_IsObject(it)) {
                    cJSON *tm = cJSON_GetObjectItem(it, "time");
                    cJSON *tx = cJSON_GetObjectItem(it, "text");
                    cJSON *dn = cJSON_GetObjectItem(it, "done");
                    snprintf(row[n], 96, "%s %s%s",
                             cJSON_IsString(tm) ? tm->valuestring : "  ",
                             cJSON_IsString(tx) ? tx->valuestring : "",
                             cJSON_IsTrue(dn) ? " [✓]" : "");
                } else continue;
                lines[n] = row[n];
                n++;
            }
        }
        cJSON_Delete(root);
    }
    if (!title[0]) snprintf(title, sizeof(title), "%s", mode_name);
    if (n == 0) { lines[n++] = "(无内容)"; }
    draw_lines_page(bw, red, title, lines, n);
}

/* ---- 状态页:本地自渲染(74 §4: status 为 null) ---- */

static void render_status(uint8_t *bw, uint8_t *red)
{
    char l[10][96];
    int n = 0;

    device_status_t st;
    status_get(&st);

    const esp_app_desc_t *app = esp_app_get_description();
    snprintf(l[n++], 96, "版本  %s", app->version);
    snprintf(l[n++], 96, "启动   第 %lu 次 · 运行 %ld 分钟",
             (unsigned long)evlog_get_boot_count(),
             (long)(esp_timer_get_time() / 1000 / 60000));
    snprintf(l[n++], 96, "WiFi   %s · %s",
             st.wifi_connected ? "已连接" : "断开",
             st.ip[0] ? st.ip : "-");
    snprintf(l[n++], 96, "MQTT   %s · 事件序号 %lu",
             mqtt_publisher_is_connected() ? "在线" : "离线",
             (unsigned long)mqtt_publisher_seq());
    snprintf(l[n++], 96, "雷达   %s · 伺服 %+.0f°",
             radar_is_online() ? "在线" : "离线", st.servo_angle);
    snprintf(l[n++], 96, "内存   %u KB 空闲",
             (unsigned)(xPortGetFreeHeapSize() / 1024));

    const char *lines[10];
    for (int i = 0; i < n; i++) lines[i] = l[i];
    draw_lines_page(bw, red, "设备状态", lines, n);
}

/* ---- 药盒格 medgrid:3 行 × 7 天 ---- */

static void render_medgrid(uint8_t *bw, uint8_t *red, const char *payload)
{
    memset(bw, 0xFF, EPD_BUF_SIZE);
    memset(red, 0x00, EPD_BUF_SIZE);
    g_bw = bw; g_red = red;

    u8g2_font_t font;
    memset(&font, 0, sizeof(font));
    font.draw_hv_line = u8g2_draw_hv_line;
    u8g2_SetBackgroundColor(&font, U_WHITE);

    draw_title_bar(&font, "本周服药");

    cJSON *root = cJSON_Parse(payload);
    cJSON *grid = root ? cJSON_GetObjectItem(root, "medgrid") : NULL;
    cJSON *days = cJSON_IsObject(grid) ? cJSON_GetObjectItem(grid, "days") : NULL;
    cJSON *today = cJSON_IsObject(grid) ? cJSON_GetObjectItem(grid, "today_idx") : NULL;
    int today_idx = cJSON_IsNumber(today) ? today->valueint : -1;

    int top = (EPD_HEIGHT >= 400) ? 40 : 30;
    int bottom = EPD_HEIGHT - 6;
    int left = 6, right = EPD_WIDTH - 6;
    int colw = (right - left) / 7;
    int grid_h = bottom - top - 16;
    int rowh = grid_h / 3;

    for (int d = 0; d < 7; d++) {
        int cx0 = left + d * colw;
        const char *date = "";
        const char *slots[3] = {"", "", ""};
        cJSON *day = (cJSON_IsArray(days) && d < cJSON_GetArraySize(days))
                         ? cJSON_GetArrayItem(days, d) : NULL;
        if (cJSON_IsObject(day)) {
            cJSON *dt = cJSON_GetObjectItem(day, "date");
            if (cJSON_IsString(dt)) date = dt->valuestring;
            cJSON *sl = cJSON_GetObjectItem(day, "slots");
            if (cJSON_IsArray(sl))
                for (int s = 0; s < 3 && s < cJSON_GetArraySize(sl); s++) {
                    cJSON *e = cJSON_GetArrayItem(sl, s);
                    if (cJSON_IsString(e)) slots[s] = e->valuestring;
                }
        }
        /* 日期标签 */
        u8g2_SetFont(&font, u8g2_font_wqy9_t_lunar);
        u8g2_SetFontMode(&font, 1);
        u8g2_SetForegroundColor(&font, (d == today_idx) ? U_RED : U_BLACK);
        draw_utf8(&font, cx0 + 4, top + 11, date);
        /* 3 格:taken=实心黑 missed=实心红 pending=空心黑 */
        for (int s = 0; s < 3; s++) {
            int y0 = top + 16 + s * rowh;
            int x0 = cx0 + 5, x1 = cx0 + colw - 6, y1 = y0 + rowh - 4;
            if (strcmp(slots[s], "taken") == 0) {
                fill_rect(x0, y0, x1, y1, CLR_BLACK);
            } else if (strcmp(slots[s], "missed") == 0) {
                fill_rect(x0, y0, x1, y1, CLR_RED);
            } else {
                frame_rect(x0, y0, x1, y1, CLR_BLACK, 2);
            }
        }
        /* 今天整列红框高亮 */
        if (d == today_idx)
            frame_rect(cx0 + 2, top + 13, cx0 + colw - 3, bottom - 2, CLR_RED, 3);
    }
    if (root) cJSON_Delete(root);
}

/* ---- calendar_overlay:月历 + 待办框线/节假日点 ---- */

static int day_of_week(int y, int m, int d)
{
    static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (m < 3) y--;
    return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

static void render_calendar_overlay(uint8_t *bw, uint8_t *red,
                                    const char *payload)
{
    /* 先画标准月历(已验收路径),再按同一布局公式补 overlay 标记 */
    epd_calendar_render(bw, red);
    g_bw = bw; g_red = red;

    cJSON *root = cJSON_Parse(payload);
    cJSON *ov = root ? cJSON_GetObjectItem(root, "overlay") : NULL;
    if (!cJSON_IsObject(ov)) { if (root) cJSON_Delete(root); return; }

    time_t now;
    struct tm tm_now;
    time(&now);
    localtime_r(&now, &tm_now);
    int year = tm_now.tm_year + 1900, month = tm_now.tm_mon + 1;

    /* 与 epd_calendar.c draw_month_days 同一套几何 */
    bool large = (EPD_HEIGHT >= 400);
    int margin = 6;
    int wh_y = large ? 44 : 32;
    int wh_h = large ? 30 : 22;
    int dim = 31;
    static const int mdays[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    dim = mdays[month - 1];
    if (month == 2 && (year % 400 == 0 || (year % 4 == 0 && year % 100)))
        dim = 29;
    int fw = day_of_week(year, month, 1);
    int nrows = (fw + dim + 6) / 7;
    int colw = (EPD_WIDTH - margin * 2) / 7;
    int rowh = (EPD_HEIGHT - wh_y - wh_h - 4) / nrows;
    int sq = large ? 34 : 30;

    /* todo_days:细红框(框住日期方块,不遮数字) */
    cJSON *td = cJSON_GetObjectItem(ov, "todo_days");
    if (cJSON_IsArray(td)) {
        cJSON *it;
        cJSON_ArrayForEach(it, td) {
            if (!cJSON_IsNumber(it)) continue;
            int day = it->valueint;
            if (day < 1 || day > dim) continue;
            int col = (fw + day - 1) % 7, row = (fw + day - 1) / 7;
            int cx = margin + col * colw + colw / 2;
            int cy = wh_y + wh_h + row * rowh + rowh / 2;
            frame_rect(cx - sq / 2 - 3, cy - sq / 2 - 5,
                       cx + sq / 2 + 3, cy + sq / 2 + 5, CLR_RED, 2);
        }
    }
    /* holidays:红色小方块(日格右上角) */
    cJSON *hol = cJSON_GetObjectItem(ov, "holidays");
    if (cJSON_IsArray(hol)) {
        cJSON *it;
        cJSON_ArrayForEach(it, hol) {
            cJSON *d = cJSON_GetObjectItem(it, "d");
            if (!cJSON_IsNumber(d)) continue;
            int day = d->valueint;
            if (day < 1 || day > dim) continue;
            int col = (fw + day - 1) % 7, row = (fw + day - 1) / 7;
            int cx = margin + col * colw + colw / 2;
            int cy = wh_y + wh_h + row * rowh + rowh / 2;
            fill_rect(cx + sq / 2 - 4, cy - sq / 2 - 4,
                      cx + sq / 2 + 2, cy - sq / 2 + 2, CLR_RED);
        }
    }
    cJSON_Delete(root);
}

/* ---- 分发 ---- */

void epd_pages_render(int mode, const char *payload_json,
                      uint8_t *bw, uint8_t *red)
{
    const char *payload = (payload_json && payload_json[0]) ? payload_json : "{}";
    switch (mode) {
    case DMODE_CALENDAR:
        epd_calendar_render(bw, red);
        break;
    case DMODE_CALENDAR_OVERLAY:
        render_calendar_overlay(bw, red, payload);
        break;
    case DMODE_WEATHER:
        render_lines_from_json(bw, red, payload, "weather", NULL);
        break;
    case DMODE_MSG:
        render_lines_from_json(bw, red, payload, "msg", NULL);
        break;
    case DMODE_TODO:
        render_lines_from_json(bw, red, payload, "todo", "items");
        break;
    case DMODE_STATUS:
        render_status(bw, red);
        break;
    case DMODE_MEDGRID:
        render_medgrid(bw, red, payload);
        break;
    default:
        epd_calendar_render(bw, red);
        break;
    }
}
