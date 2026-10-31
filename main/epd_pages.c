/* epd_pages.c — docs/74 M1 模式渲染器(排版升级版,用户 10-29 裁决:
 * 大字/图标/图例/3D麦状态)。
 * - 文本:wqy16 全量(16px),标题与天气主信息 2x 像素放大(32px)
 * - 天气页:图标(晴/云/雨 原语绘制,关键词匹配)+大字主行
 * - 药格:格内 早/中/晚(实心格反白),底部图例(黑=已服 红=漏服 空=待服)
 * - 状态页:含 3D 麦克风阵列状态
 * 像素/字体原语与 epd_calendar.c 同款,刻意各留一份静态拷贝。 */
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
#include "mode_manager.h"
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
/* 放大绘制:v3 修正 — 缩放只作用于字形像素,原点 (g_ox,g_oy) 不缩放。
 * (旧实现把坐标一起乘,2x 文字实际画到 y*2 处出界=全裁空) */
static int g_scale = 1;
static int g_ox = 0, g_oy = 0;

/* ---- 像素/文本原语 ---- */

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
    for (int i = 0; i < len; i++)
        for (int dy = 0; dy < g_scale; dy++)
            for (int dx = 0; dx < g_scale; dx++) {
                /* 字形空间 (x+i,y) → 画布 (g_ox+(x+i)*s, g_oy+y*s)+块内偏移 */
                int px = g_ox + (int)(dir == 0 ? x + i : x) * g_scale + dx;
                int py = g_oy + (int)(dir == 0 ? y : y + i) * g_scale + dy;
                set_pixel(px, py, c);
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

/* 放大绘制:font 的每个逻辑像素画 scale×scale 块(g_scale 全局)。 */
static int16_t draw_text(u8g2_font_t *font, int16_t x, int16_t y,
                         const char *s, int scale)
{
    g_ox = x; g_oy = y; g_scale = scale;
    int16_t fx = 0, fy = 0;   /* 字形空间局部坐标(相对原点) */
    while (*s) {
        uint16_t cp = utf8_next(&s);
        fx += u8g2_DrawGlyph(font, fx, fy, cp);
    }
    g_scale = 1;
    return x + fx * scale;
}

static int text_w(u8g2_font_t *font, const char *s, int scale)
{
    g_scale = 1;
    int16_t w = 0;
    while (*s) {
        uint16_t cp = utf8_next(&s);
        w += u8g2_GetGlyphWidth(font, cp);
    }
    return w * scale;
}

/* ---- 天气图标(原语绘制,关键词选择) ---- */

static void draw_icon_sun(int cx, int cy, int r, int color)
{
    for (int a = 0; a < 360; a += 4)
        fill_rect(cx + r * 0.66 * __builtin_cos(a * 3.14159 / 180) - 1,
                  cy + r * 0.66 * __builtin_sin(a * 3.14159 / 180) - 1,
                  cx + r * 0.66 * __builtin_cos(a * 3.14159 / 180) + 1,
                  cy + r * 0.66 * __builtin_sin(a * 3.14159 / 180) + 1, color);
    for (int a = 0; a < 360; a += 45) {
        double c = __builtin_cos(a * 3.14159 / 180), s = __builtin_sin(a * 3.14159 / 180);
        fill_rect(cx + r * 0.85 * c, cy + r * 0.85 * s,
                  cx + r * 1.25 * c, cy + r * 1.25 * s, color);
    }
}

static void draw_icon_cloud(int cx, int cy, int w, int color)
{
    int r = w / 5;
    fill_rect(cx - w / 2 + r, cy, cx + w / 2 - r, cy + r, color);
    for (int dx = -w / 2 + 2 * r; dx <= w / 2 - 2 * r; dx += r)
        for (int a = 0; a < 360; a += 6)
            fill_rect(cx + dx + r * 0.7 * __builtin_cos(a * 3.14159 / 180),
                      cy + r * 0.7 * __builtin_sin(a * 3.14159 / 180),
                      cx + dx + r * 0.7 * __builtin_cos(a * 3.14159 / 180) + 1,
                      cy + r * 0.7 * __builtin_sin(a * 3.14159 / 180) + 1, color);
}

static void draw_weather_icon(const char *cond, int cx, int cy)
{
    if (strstr(cond, "雨")) {
        draw_icon_cloud(cx, cy - 18, 72, CLR_BLACK);
        for (int i = -1; i <= 1; i++)
            fill_rect(cx + i * 16, cy + 2, cx + i * 16 + 5, cy + 18, CLR_RED);
    } else if (strstr(cond, "雪")) {
        draw_icon_cloud(cx, cy - 18, 72, CLR_BLACK);
        for (int i = -1; i <= 1; i++)
            fill_rect(cx + i * 16, cy + 6, cx + i * 16 + 4, cy + 10, CLR_RED);
    } else if (strstr(cond, "云") || strstr(cond, "阴")) {
        draw_icon_cloud(cx, cy - 8, 80, CLR_BLACK);
    } else {   /* 晴(默认) */
        draw_icon_sun(cx, cy, 26, CLR_RED);
    }
}

/* ---- 文本页骨架 ---- */

#define TITLE_H      38   /* 用户裁决:窄栏 */
#define TITLE_SCALE  1    /* 反白 16px 标题(黑底白字已够醒目) */
#define BODY_SCALE   1
#define LINE_H       34

static void draw_title_bar(u8g2_font_t *font, const char *title)
{
    fill_rect(0, 0, EPD_WIDTH - 1, TITLE_H - 1, CLR_BLACK);
    u8g2_SetFont(font, u8g2_font_wqy16_t_gb2312a);
    u8g2_SetFontMode(font, 1);
    u8g2_SetForegroundColor(font, U_WHITE);
    draw_text(font, 12, TITLE_H - 9, title, TITLE_SCALE);
}

static void draw_body(u8g2_font_t *font, int x, int y, const char *s,
                      int scale, int red)
{
    u8g2_SetFont(font, u8g2_font_wqy16_t_gb2312a);
    u8g2_SetFontMode(font, 1);
    u8g2_SetBackgroundColor(font, U_WHITE);
    u8g2_SetForegroundColor(font, red ? U_RED : U_BLACK);
    draw_text(font, x, y, s, scale);
}

static void draw_lines_page(uint8_t *bw, uint8_t *red,
                            const char *title, const char *const *lines,
                            int n_lines, int hero_index)
{
    memset(bw, 0xFF, EPD_BUF_SIZE);
    memset(red, 0x00, EPD_BUF_SIZE);
    g_bw = bw; g_red = red;

    u8g2_font_t font;
    memset(&font, 0, sizeof(font));
    font.draw_hv_line = u8g2_draw_hv_line;

    draw_title_bar(&font, title);

    int y = TITLE_H + LINE_H;
    for (int i = 0; i < n_lines && y < EPD_HEIGHT - 8; i++) {
        char buf[96];
        strncpy(buf, lines[i], sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        if (i == hero_index) {
            draw_body(&font, 14, y + 8, buf, 2, 1);   /* 2x 红色主信息 */
            y += 56;
        } else {
            draw_body(&font, 14, y, buf, BODY_SCALE, 0);
            y += LINE_H;
        }
    }
}

/* 从 JSON 抽 lines[](或 items[]→行)进静态行缓冲 */
static int json_lines(const char *payload, const char *mode_name,
                      const char *items_key, char title[48],
                      char rows[12][96])
{
    int n = 0;
    title[0] = '\0';
    cJSON *root = cJSON_Parse(payload);
    if (root) {
        cJSON *obj = cJSON_GetObjectItem(root, mode_name);
        if (obj) {
            cJSON *t = cJSON_GetObjectItem(obj, "title");
            if (cJSON_IsString(t))
                snprintf(title, 48, "%s", t->valuestring);
        }
        const char *key = items_key ? items_key : "lines";
        cJSON *arr = cJSON_GetObjectItem(obj ? obj : root, key);
        if (cJSON_IsArray(arr)) {
            cJSON *it;
            cJSON_ArrayForEach(it, arr) {
                if (n >= 12) break;
                if (cJSON_IsString(it)) {
                    snprintf(rows[n], 96, "%s", it->valuestring);
                } else if (cJSON_IsObject(it)) {
                    cJSON *tm = cJSON_GetObjectItem(it, "time");
                    cJSON *tx = cJSON_GetObjectItem(it, "text");
                    cJSON *dn = cJSON_GetObjectItem(it, "done");
                    snprintf(rows[n], 96, "%s %s%s",
                             cJSON_IsString(tm) ? tm->valuestring : "  ",
                             cJSON_IsString(tx) ? tx->valuestring : "",
                             cJSON_IsTrue(dn) ? " [已毕]" : "");
                } else continue;
                n++;
            }
        }
        cJSON_Delete(root);
    }
    if (!title[0]) snprintf(title, 48, "%s", mode_name);
    return n;
}

static void render_lines_from_json(uint8_t *bw, uint8_t *red,
                                   const char *payload, const char *mode_name,
                                   const char *items_key, int hero_index)
{
    static char rows[12][96];
    static char title[48];
    const char *lines[12];
    int n = json_lines(payload, mode_name, items_key, title, rows);
    if (n == 0) strcpy(rows[n++], "(无内容)");
    for (int i = 0; i < n; i++) lines[i] = rows[i];

    /* 天气页专用排版 v3:图标居中上区,主信息(2x红)整行居中于图标下,
     * 分隔线后为明细行 — 任何行宽都不会出界。 */
    if (hero_index >= 0 && mode_name[0] == 'w' && n > 0) {
        memset(bw, 0xFF, EPD_BUF_SIZE);
        memset(red, 0x00, EPD_BUF_SIZE);
        g_bw = bw; g_red = red;
        u8g2_font_t font;
        memset(&font, 0, sizeof(font));
        font.draw_hv_line = u8g2_draw_hv_line;
        draw_title_bar(&font, title);

        draw_weather_icon(rows[0], EPD_WIDTH / 2, TITLE_H + 52);

        char cond[40];
        strncpy(cond, rows[0], sizeof(cond) - 1);
        cond[sizeof(cond) - 1] = 0;
        /* UTF-8 安全截到 12 字(2x 居中行宽上限) */
        int cnt = 0;
        char *p = cond;
        while (*p && cnt < 12) { p += (*p & 0x80) ? 3 : 1; cnt++; }
        *p = 0;

        int hero_scale = 2;
        int hw = text_w(&font, cond, hero_scale);
        if (hw > EPD_WIDTH - 24) hero_scale = 1;
        hw = text_w(&font, cond, hero_scale);
        draw_body(&font, (EPD_WIDTH - hw) / 2, TITLE_H + 148, cond,
                  hero_scale, 1);

        int div_y = TITLE_H + 162;
        fill_rect(10, div_y, EPD_WIDTH - 10, div_y + 1, CLR_BLACK);

        int y = div_y + 32;
        for (int i = 1; i < n && y < EPD_HEIGHT - 10; i++) {
            draw_body(&font, 14, y, rows[i], BODY_SCALE, 0);
            y += LINE_H;
        }
        return;
    }
    draw_lines_page(bw, red, title, lines, n, hero_index);
}

/* ---- 状态页 v3:卡片表驱动,2×4 自适应网格(8 卡含舵机) ---- */

static void card_icon_clock(int cx, int cy)
{
    for (int a = 0; a < 360; a += 6)
        fill_rect(cx + 9 * __builtin_cos(a * 3.14159 / 180),
                  cy + 9 * __builtin_sin(a * 3.14159 / 180),
                  cx + 9 * __builtin_cos(a * 3.14159 / 180) + 1,
                  cy + 9 * __builtin_sin(a * 3.14159 / 180) + 1, CLR_BLACK);
    fill_rect(cx, cy, cx + 5, cy + 1, CLR_BLACK);
    fill_rect(cx, cy, cx, cy - 5, CLR_BLACK);
}

static void card_icon_mic(int cx, int cy)
{
    fill_rect(cx - 3, cy - 8, cx + 3, cy + 1, CLR_BLACK);
    fill_rect(cx - 6, cy + 1, cx + 6, cy + 2, CLR_BLACK);
    fill_rect(cx, cy + 3, cx + 1, cy + 5, CLR_BLACK);
    fill_rect(cx - 3, cy + 6, cx + 3, cy + 7, CLR_BLACK);
}

static void card_icon_wifi(int cx, int cy)
{
    for (int ring = 1; ring <= 3; ring++) {
        int r = ring * 4;
        for (int a = 200; a <= 340; a += 10)
            fill_rect(cx + r * __builtin_cos(a * 3.14159 / 180),
                      cy - r * __builtin_sin(a * 3.14159 / 180),
                      cx + r * __builtin_cos(a * 3.14159 / 180) + 1,
                      cy - r * __builtin_sin(a * 3.14159 / 180) + 1, CLR_BLACK);
    }
    fill_rect(cx - 1, cy - 1, cx + 1, cy + 1, CLR_BLACK);
}

static void card_icon_cloud(int cx, int cy)
{
    draw_icon_cloud(cx, cy, 22, CLR_BLACK);
}

static void card_icon_radar(int cx, int cy)
{
    for (int ring = 1; ring <= 2; ring++) {
        int r = ring * 5;
        for (int a = 0; a <= 90; a += 12)
            fill_rect(cx + r * __builtin_cos(a * 3.14159 / 180),
                      cy - r * __builtin_sin(a * 3.14159 / 180),
                      cx + r * __builtin_cos(a * 3.14159 / 180) + 1,
                      cy - r * __builtin_sin(a * 3.14159 / 180) + 1, CLR_BLACK);
    }
    fill_rect(cx, cy, cx + 7, cy - 5, CLR_BLACK);
    fill_rect(cx - 1, cy - 1, cx + 1, cy + 1, CLR_RED);
}

static void card_icon_servo(int cx, int cy)
{
    frame_rect(cx - 7, cy - 3, cx + 7, cy + 9, CLR_BLACK, 2);
    for (int a = 0; a < 360; a += 8)
        fill_rect(cx + 3 * __builtin_cos(a * 3.14159 / 180),
                  cy - 7 + 3 * __builtin_sin(a * 3.14159 / 180),
                  cx + 3 * __builtin_cos(a * 3.14159 / 180) + 1,
                  cy - 7 + 3 * __builtin_sin(a * 3.14159 / 180) + 1, CLR_BLACK);
    fill_rect(cx, cy - 7, cx + 8, cy - 6, CLR_RED);   /* 输出轴臂 */
}

static void card_icon_chip(int cx, int cy)
{
    frame_rect(cx - 6, cy - 5, cx + 6, cy + 5, CLR_BLACK, 2);
    fill_rect(cx - 2, cy - 1, cx + 2, cy + 1, CLR_BLACK);
    for (int i = -4; i <= 4; i += 4) {
        fill_rect(cx + i, cy - 8, cx + i + 1, cy - 6, CLR_BLACK);
        fill_rect(cx + i, cy + 6, cx + i + 1, cy + 8, CLR_BLACK);
    }
}

static void card_icon_info(int cx, int cy)
{
    for (int a = 0; a < 360; a += 8)
        fill_rect(cx + 8 * __builtin_cos(a * 3.14159 / 180),
                  cy + 8 * __builtin_sin(a * 3.14159 / 180),
                  cx + 8 * __builtin_cos(a * 3.14159 / 180) + 1,
                  cy + 8 * __builtin_sin(a * 3.14159 / 180) + 1, CLR_BLACK);
    fill_rect(cx, cy - 3, cx + 1, cy + 2, CLR_BLACK);
    fill_rect(cx, cy + 4, cx + 1, cy + 5, CLR_BLACK);
}

/* 卡片内容描述(表驱动) */
typedef struct {
    void (*icon)(int, int);
    const char *label;
    char value[40];
    char sub[48];
    bool alarm;
} status_card_t;

static void draw_card(u8g2_font_t *font, int x0, int y0, int w, int h,
                      const status_card_t *c)
{
    if (c->alarm) frame_rect(x0, y0, x0 + w - 1, y0 + h - 1, CLR_RED, 4);
    else          frame_rect(x0, y0, x0 + w - 1, y0 + h - 1, CLR_BLACK, 2);
    c->icon(x0 + w / 2, y0 + 22);

    u8g2_SetBackgroundColor(font, U_WHITE);
    u8g2_SetFontMode(font, 1);
    u8g2_SetFont(font, u8g2_font_wqy12_t_gb2312a);
    u8g2_SetForegroundColor(font, U_BLACK);
    draw_text(font, x0 + (w - text_w(font, c->label, 1)) / 2,
              y0 + 48, c->label, 1);
    /* 数值:按 '-' 切段组装行(每行 ≤13 半角字符,最多 3 行;12px 红字)。
     * 用户实例: v0.3.0- / supermini-final- / 95-g292c935 三行(10-29 裁决)。 */
    u8g2_SetForegroundColor(font, U_RED);
    u8g2_SetFont(font, u8g2_font_wqy12_t_gb2312a);
    char vlines[3][24];
    int nv = 0;
    {
        const char *p = c->value;
        vlines[0][0] = vlines[1][0] = vlines[2][0] = '\0';
        while (*p && nv < 3) {
            const char *dash = strchr(p, '-');
            size_t seg = dash ? (size_t)(dash - p + 1) : strlen(p);
            if (strlen(vlines[nv]) + seg > 13 && vlines[nv][0]) {
                if (++nv >= 3) break;
            }
            strncat(vlines[nv], p, seg);
            p += seg;
        }
        if (nv == 0) nv = 1;
    }
    if (strlen(vlines[0]) <= 13 && !vlines[1][0]) {
        u8g2_SetFont(font, u8g2_font_wqy16_t_gb2312a);
        int vw = text_w(font, c->value, 1);
        if (vw > w - 10) {
            u8g2_SetFont(font, u8g2_font_wqy12_t_gb2312a);
            vw = text_w(font, c->value, 1);
        }
        draw_text(font, x0 + (w - vw) / 2, y0 + h - 30, c->value, 1);
    } else {
        int ybase = y0 + h - 16 - 15 * (nv);
        for (int i = 0; i <= nv && i < 3; i++) {
            if (!vlines[i][0]) break;
            int lw = text_w(font, vlines[i], 1);
            draw_text(font, x0 + (w - lw) / 2, ybase + i * 15, vlines[i], 1);
        }
    }
    u8g2_SetFont(font, u8g2_font_wqy12_t_gb2312a);
    u8g2_SetForegroundColor(font, U_BLACK);
    draw_text(font, x0 + (w - text_w(font, c->sub, 1)) / 2,
              y0 + h - 10, c->sub, 1);
}

static void render_status(uint8_t *bw, uint8_t *red)
{
    memset(bw, 0xFF, EPD_BUF_SIZE);
    memset(red, 0x00, EPD_BUF_SIZE);
    g_bw = bw; g_red = red;

    u8g2_font_t font;
    memset(&font, 0, sizeof(font));
    font.draw_hv_line = u8g2_draw_hv_line;
    extern char g_device_id[8];              /* main.c NVS 身份 */
    char stitle[40];
    snprintf(stitle, sizeof(stitle), "设备状态 · %s", g_device_id);
    draw_title_bar(&font, stitle);

    device_status_t st;
    status_get(&st);
    const esp_app_desc_t *app = esp_app_get_description();

    status_card_t cards[8];
    memset(cards, 0, sizeof(cards));

    cards[0].icon = card_icon_clock;
    cards[0].label = "运行";
    snprintf(cards[0].value, 40, "%ld 分",
             (long)(esp_timer_get_time() / 1000 / 60000));
    snprintf(cards[0].sub, 48, "第 %lu 次启动",
             (unsigned long)evlog_get_boot_count());

    cards[1].icon = card_icon_mic;
    cards[1].label = "3D麦克风";
    snprintf(cards[1].value, 40, "%.0f%%", st.confidence * 100);
    snprintf(cards[1].sub, 48, "方位 %+.0f°", st.azimuth);

    cards[2].icon = card_icon_wifi;
    cards[2].label = "WiFi";
    snprintf(cards[2].value, 40, "%s",
             st.wifi_connected ? "已连接" : "断开");
    snprintf(cards[2].sub, 48, "%s", st.ip[0] ? st.ip : "-");
    cards[2].alarm = !st.wifi_connected;

    cards[3].icon = card_icon_cloud;
    cards[3].label = "MQTT";
    snprintf(cards[3].value, 40, "%s",
             mqtt_publisher_is_connected() ? "在线" : "离线");
    snprintf(cards[3].sub, 48, "事件 %lu",
             (unsigned long)mqtt_publisher_seq());
    cards[3].alarm = !mqtt_publisher_is_connected();

    cards[4].icon = card_icon_radar;
    cards[4].label = "雷达";
    snprintf(cards[4].value, 40, "%s", radar_is_online() ? "在线" : "离线");
    snprintf(cards[4].sub, 48, "检测 %s",
             mode_manager_get_fall_detect() ? "开" : "关");
    cards[4].alarm = !radar_is_online();

    cards[5].icon = card_icon_servo;
    cards[5].label = "舵机";
    snprintf(cards[5].value, 40, "%+.0f°", st.servo_angle);
    snprintf(cards[5].sub, 48, "%s", st.servo_moving ? "转动中" : "静止");

    cards[6].icon = card_icon_chip;
    cards[6].label = "内存";
    snprintf(cards[6].value, 40, "%u KB",
             (unsigned)(xPortGetFreeHeapSize() / 1024));
    snprintf(cards[6].sub, 48, "低水位 %u KB",
             (unsigned)(xPortGetMinimumEverFreeHeapSize() / 1024));

    cards[7].icon = card_icon_info;
    cards[7].label = "固件";
    snprintf(cards[7].value, 40, "%s", app->version);
    snprintf(cards[7].sub, 48, "脖颈 RD60");

    int top = TITLE_H + 5;
    int gap = 5;
    int cols = 4, rows = 2;                    /* 8 卡 → 2×4 自适应 */
    int cw = (EPD_WIDTH - 12 - gap * (cols - 1)) / cols;
    int ch = (EPD_HEIGHT - top - 6 - gap * (rows - 1)) / rows;
    for (int i = 0; i < 8; i++) {
        int r = i / cols, c = i % cols;
        draw_card(&font, 6 + c * (cw + gap), top + r * (ch + gap),
                  cw, ch, &cards[i]);
    }
}


/* ---- 药盒格 medgrid:格内早/中/晚 + 底部图例 ---- */

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

    static const char *slot_name[3] = {"早", "中", "晚"};
    int legend_h = 24;
    int top = TITLE_H + 20;
    int bottom = EPD_HEIGHT - legend_h - 4;
    int left = 6, right = EPD_WIDTH - 6;
    int colw = (right - left) / 7;
    int rowh = (bottom - top - 14) / 3;

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
        u8g2_SetFont(&font, u8g2_font_wqy16_t_gb2312a);
        u8g2_SetFontMode(&font, 1);
        u8g2_SetForegroundColor(&font, (d == today_idx) ? U_RED : U_BLACK);
        draw_text(&font, cx0 + 4, top + 14, date, 1);

        for (int s = 0; s < 3; s++) {
            int x0 = cx0 + 5, x1 = cx0 + colw - 6;
            int y0 = top + 18 + s * rowh, y1 = y0 + rowh - 5;
            int fg = CLR_WHITE, bg = CLR_WHITE;
            if (strcmp(slots[s], "taken") == 0) {
                fill_rect(x0, y0, x1, y1, CLR_BLACK); bg = CLR_BLACK;
            } else if (strcmp(slots[s], "missed") == 0) {
                fill_rect(x0, y0, x1, y1, CLR_RED); bg = CLR_RED;
            } else {
                frame_rect(x0, y0, x1, y1, CLR_BLACK, 2); bg = CLR_WHITE;
            }
            (void)bg;
            /* 格内早/中/晚:实心格反白 */
            u8g2_SetForegroundColor(&font,
                (bg == CLR_WHITE) ? U_BLACK : U_WHITE);
            int tw = text_w(&font, slot_name[s], 1);
            int cell_cx = (x0 + x1) / 2, cell_cy = (y0 + y1) / 2;
            draw_text(&font, cell_cx - tw / (2 * 1), cell_cy + 7,
                      slot_name[s], 1);
        }
        /* 今天整列红框高亮 */
        if (d == today_idx)
            frame_rect(cx0 + 2, top + 15, cx0 + colw - 3, bottom - 2, CLR_RED, 3);
    }

    /* 底部图例:■已服(黑) ■漏服(红) □待服(空心) */
    int ly = EPD_HEIGHT - legend_h - 2;
    int lx = 10;
    static const char *legend[] = {"已服", "漏服", "待服"};
    int lcolor[] = {CLR_BLACK, CLR_RED, CLR_WHITE};
    u8g2_SetFont(&font, u8g2_font_wqy16_t_gb2312a);
    for (int i = 0; i < 3; i++) {
        if (lcolor[i] == CLR_WHITE) frame_rect(lx, ly, lx + 13, ly + 13, CLR_BLACK, 2);
        else fill_rect(lx, ly, lx + 13, ly + 13, lcolor[i]);
        u8g2_SetFontMode(&font, 1);
        u8g2_SetForegroundColor(&font, U_BLACK);
        draw_text(&font, lx + 18, ly + 13, legend[i], 1);
        lx += 18 + text_w(&font, legend[i], 1) + 22;
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

/* ---- 月历装饰层:todo 红框 + 节假日/补班重上色(M3.2 从 render_calendar_overlay
 * 抽出,calendar_overlay 本月页与 month_view 浏览页共用)。month_obj 为含
 * todo_days/holidays/workdays 的 cJSON 对象;today_mday<0 = 浏览月(基座没画
 * "今天"黑底,重上色无需跳过任何日)。 ---- */

static void apply_month_decor(uint8_t *bw, uint8_t *red,
                              int year, int month, int today_mday,
                              const cJSON *month_obj)
{
    g_bw = bw; g_red = red;

    bool large = (EPD_HEIGHT >= 400);
    int margin = 6;
    int wh_y = large ? 44 : 32;
    int wh_h = large ? 30 : 22;
    static const int mdays[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    int dim = mdays[month - 1];
    if (month == 2 && (year % 400 == 0 || (year % 4 == 0 && year % 100)))
        dim = 29;
    int fw = day_of_week(year, month, 1);
    int nrows = (fw + dim + 6) / 7;
    int colw = (EPD_WIDTH - margin * 2) / 7;
    int rowh = (EPD_HEIGHT - wh_y - wh_h - 4) / nrows;
    int sq = large ? 34 : 30;

    const cJSON *td = cJSON_GetObjectItem(month_obj, "todo_days");
    if (cJSON_IsArray(td)) {
        const cJSON *it;
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
    /* ---- 日期数字重上色(用户裁决 10-30,替代旧节假日黑框) ----
     * 法定节假日=红字;补班工作日=黑字;今天=黑底白字(基座已画,跳过)。
     * 基座默认:周末列红字、工作日列黑字 → 只重画会变色的组合:
     *   节假日落在工作日列(黑→红)、补班落在周末列(红→黑)。 */
    {
        u8g2_font_t font;
        memset(&font, 0, sizeof(font));
        font.draw_hv_line = u8g2_draw_hv_line;
        u8g2_SetFont(&font, u8g2_font_helvB18_tn);
        u8g2_SetBackgroundColor(&font, U_WHITE);
        u8g2_SetFontMode(&font, 0);

        const cJSON *hol = cJSON_GetObjectItem(month_obj, "holidays");
        if (cJSON_IsArray(hol)) {
            const cJSON *it;
            cJSON_ArrayForEach(it, hol) {
                cJSON *d = cJSON_GetObjectItem(it, "d");
                if (!cJSON_IsNumber(d)) continue;
                int day = d->valueint;
                if (day < 1 || day > dim) continue;
                int col = (fw + day - 1) % 7;
                if (col == 0 || col == 6) continue;  /* 周末列本就红字 */
                if (today_mday > 0 && day == today_mday)
                    continue;   /* 今天黑底白字优先 */
                int row = (fw + day - 1) / 7;
                int cx = margin + col * colw + colw / 2;
                int cy = wh_y + wh_h + row * rowh + rowh / 2;
                fill_rect(cx - sq / 2, cy - sq / 2,
                          cx + sq / 2, cy + sq / 2, CLR_WHITE);
                char nb[8];
                snprintf(nb, sizeof(nb), "%u", (unsigned)(uint8_t)day);
                u8g2_SetForegroundColor(&font, U_RED);
                int nw = text_w(&font, nb, 1);
                draw_text(&font, cx - nw / 2, cy + 6, nb, 1);
            }
        }
        const cJSON *wd = cJSON_GetObjectItem(month_obj, "workdays");
        if (cJSON_IsArray(wd)) {
            const cJSON *it;
            cJSON_ArrayForEach(it, wd) {
                if (!cJSON_IsNumber(it)) continue;
                int day = it->valueint;
                if (day < 1 || day > dim) continue;
                int col = (fw + day - 1) % 7;
                if (col != 0 && col != 6) continue;  /* 工作日列本就黑字 */
                if (today_mday > 0 && day == today_mday) continue;
                int row = (fw + day - 1) / 7;
                int cx = margin + col * colw + colw / 2;
                int cy = wh_y + wh_h + row * rowh + rowh / 2;
                fill_rect(cx - sq / 2, cy - sq / 2,
                          cx + sq / 2, cy + sq / 2, CLR_WHITE);
                char nb[8];
                snprintf(nb, sizeof(nb), "%u", (unsigned)(uint8_t)day);
                u8g2_SetForegroundColor(&font, U_BLACK);
                int nw = text_w(&font, nb, 1);
                draw_text(&font, cx - nw / 2, cy + 6, nb, 1);
            }
        }
    }
}

static void render_calendar_overlay(uint8_t *bw, uint8_t *red,
                                    const char *payload)
{
    time_t now;
    struct tm tm_now;
    time(&now);
    localtime_r(&now, &tm_now);
    int year = tm_now.tm_year + 1900, month = tm_now.tm_mon + 1;

    epd_calendar_render_month(bw, red, year, month, tm_now.tm_mday);

    cJSON *root = cJSON_Parse(payload);
    cJSON *ov = root ? cJSON_GetObjectItem(root, "overlay") : NULL;
    if (!cJSON_IsObject(ov)) { if (root) cJSON_Delete(root); return; }
    apply_month_decor(bw, red, year, month, tm_now.tm_mday, ov);
    cJSON_Delete(root);
}

/* ---- month_view:任意月浏览页(docs/74 M3.2,语音"看下个月") ----
 * 载荷 {"mode":"month_view","month":{"year":2026,"month":11,
 *   "todo_days":[21,23],"holidays":[{"d":1,"name":"元旦"}],"workdays":[…]}}
 * 键名刻意不用顶层 "overlay" — display_mode stash_overlay_from() 会把
 * 顶层 overlay 收编进当日暂存,下月节假日就会画到本月常驻主页上。 */
static void render_month_view(uint8_t *bw, uint8_t *red, const char *payload)
{
    cJSON *root = cJSON_Parse(payload);
    cJSON *mv = root ? cJSON_GetObjectItem(root, "month") : NULL;
    cJSON *y = cJSON_IsObject(mv) ? cJSON_GetObjectItem(mv, "year") : NULL;
    cJSON *m = cJSON_IsObject(mv) ? cJSON_GetObjectItem(mv, "month") : NULL;
    if (!cJSON_IsNumber(y) || !cJSON_IsNumber(m) ||
        y->valueint < 2000 || y->valueint > 2099 ||
        m->valueint < 1 || m->valueint > 12) {
        epd_calendar_render(bw, red);   /* 载荷残缺 → 本月兜底 */
        if (root) cJSON_Delete(root);
        return;
    }
    epd_calendar_render_month(bw, red, y->valueint, m->valueint, -1);
    apply_month_decor(bw, red, y->valueint, m->valueint, -1, mv);
    if (root) cJSON_Delete(root);
}

/* ---- todo 表格页(用户裁决 10-30:日期|时间|事项 三列,今天红/未来黑) ---- */

static void utf8_trunc_to_width(u8g2_font_t *font, char *s, int max_w)
{
    while (*s && text_w(font, s, 1) > max_w) {
        int len = (int)strlen(s);
        while (len > 0 && (s[len - 1] & 0xC0) == 0x80) len--;
        if (len > 0) len--;
        if (len <= 0) { s[0] = '\0'; break; }
        s[len] = '\0';
    }
}

static void render_todo_table(uint8_t *bw, uint8_t *red, const char *payload)
{
    memset(bw, 0xFF, EPD_BUF_SIZE);
    memset(red, 0x00, EPD_BUF_SIZE);
    g_bw = bw; g_red = red;

    static struct { char date[16]; char time[12]; char text[80]; int today; }
        rows[12];
    int n = 0;
    char title[48] = "待办";

    cJSON *root = cJSON_Parse(payload);
    if (root) {
        cJSON *obj = cJSON_GetObjectItem(root, "todo");
        cJSON *t = obj ? cJSON_GetObjectItem(obj, "title") : NULL;
        if (cJSON_IsString(t)) snprintf(title, sizeof(title), "%s", t->valuestring);
        cJSON *arr = cJSON_GetObjectItem(obj ? obj : root, "items");
        cJSON *it;
        cJSON_ArrayForEach(it, arr) {
            if (n >= 12) break;
            rows[n].date[0] = rows[n].time[0] = rows[n].text[0] = '\0';
            rows[n].today = 0;
            cJSON *d = cJSON_GetObjectItem(it, "date");
            if (cJSON_IsString(d)) {                  /* v2 行 {date,time,text,today} */
                snprintf(rows[n].date, sizeof(rows[n].date), "%s", d->valuestring);
                cJSON *tm = cJSON_GetObjectItem(it, "time");
                cJSON *tx = cJSON_GetObjectItem(it, "text");
                cJSON *td = cJSON_GetObjectItem(it, "today");
                if (cJSON_IsString(tm))
                    snprintf(rows[n].time, sizeof(rows[n].time), "%s", tm->valuestring);
                if (cJSON_IsString(tx))
                    snprintf(rows[n].text, sizeof(rows[n].text), "%s", tx->valuestring);
                rows[n].today = cJSON_IsTrue(td);
                n++;
            } else if (cJSON_IsObject(it)) {          /* 旧行 {time,text,done} */
                cJSON *tm = cJSON_GetObjectItem(it, "time");
                cJSON *tx = cJSON_GetObjectItem(it, "text");
                cJSON *dn = cJSON_GetObjectItem(it, "done");
                if (cJSON_IsString(tm))
                    snprintf(rows[n].time, sizeof(rows[n].time), "%s", tm->valuestring);
                if (cJSON_IsString(tx))
                    snprintf(rows[n].text, sizeof(rows[n].text), "%s%s",
                             tx->valuestring, cJSON_IsTrue(dn) ? " [已毕]" : "");
                n++;
            } else if (cJSON_IsString(it)) {          /* 裸字符串 */
                snprintf(rows[n].text, sizeof(rows[n].text), "%s", it->valuestring);
                n++;
            }
        }
        cJSON_Delete(root);
    }
    if (n == 0) { snprintf(rows[0].text, sizeof(rows[0].text), "今天没有安排"); n = 1; }

    u8g2_font_t font;
    memset(&font, 0, sizeof(font));
    font.draw_hv_line = u8g2_draw_hv_line;
    u8g2_SetFont(&font, u8g2_font_wqy16_t_gb2312a);
    u8g2_SetFontMode(&font, 1);
    u8g2_SetBackgroundColor(&font, U_WHITE);

    draw_title_bar(&font, title);

    const int top = TITLE_H + 4, hdr_h = 26;
    const int c1 = 64, c2 = 122, x_end = EPD_WIDTH - 1;
    int avail = EPD_HEIGHT - 3 - (top + hdr_h);
    int max_rows = avail / 26;               /* 最密 26px/行 */
    if (max_rows > 12) max_rows = 12;
    if (n > max_rows) n = max_rows;
    int row_h = avail / n;                   /* 行距伸展铺满(26..44) */
    if (row_h > 44) row_h = 44;
    const int bot = top + hdr_h + n * row_h - 1;

    /* 表头黑底反白 + 列线 + 行线 */
    fill_rect(1, top, x_end, top + hdr_h - 1, CLR_BLACK);
    fill_rect(c1, top, c1 + 1, bot, CLR_BLACK);
    fill_rect(c2, top, c2 + 1, bot, CLR_BLACK);
    frame_rect(1, top, x_end, bot, CLR_BLACK, 2);
    for (int i = 1; i < n; i++)
        fill_rect(2, top + hdr_h + i * row_h - 1, x_end - 1,
                  top + hdr_h + i * row_h - 1, CLR_BLACK);

    u8g2_SetForegroundColor(&font, U_WHITE);
    draw_text(&font, 10, top + 19, "日期", 1);
    draw_text(&font, c1 + 8, top + 19, "时间", 1);
    draw_text(&font, c2 + 8, top + 19, "事项", 1);

    for (int i = 0; i < n; i++) {
        int ry = top + hdr_h + i * row_h + row_h / 2 + 8;   /* 16px 字基线 */
        /* 用户裁决 10-30: 今天的代办整行红(日期+时间+事项),未来行黑 */
        u8g2_SetForegroundColor(&font, rows[i].today ? U_RED : U_BLACK);
        draw_text(&font, 8, ry, rows[i].date, 1);
        draw_text(&font, c1 + 6, ry, rows[i].time, 1);
        utf8_trunc_to_width(&font, rows[i].text, x_end - (c2 + 8) - 6);
        draw_text(&font, c2 + 8, ry, rows[i].text, 1);
    }
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
        render_lines_from_json(bw, red, payload, "weather", NULL, 0);
        break;
    case DMODE_MSG:
        render_lines_from_json(bw, red, payload, "msg", NULL, 0);
        break;
    case DMODE_TODO:
        if (EPD_WIDTH >= 400)   /* 4.2吋(400×300)横置:按宽判,HEIGHT=300 会误走旧分支(10-30 实录) */
            render_todo_table(bw, red, payload);
        else
            render_lines_from_json(bw, red, payload, "todo", "items", -1);
        break;
    case DMODE_STATUS:
        render_status(bw, red);
        break;
    case DMODE_MEDGRID:
        render_medgrid(bw, red, payload);
        break;
    case DMODE_MONTH_VIEW:
        render_month_view(bw, red, payload);
        break;
    default:
        epd_calendar_render(bw, red);
        break;
    }
}
