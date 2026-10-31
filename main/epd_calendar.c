/* epd_calendar.c — lunar-calendar page, ported verbatim from ArtInk58's
 * main.c rendering pipeline (u8g2 glyph bridge + calendar layout). The
 * neck keeps this on the e-ink between daily re-renders. */
#include "epd_calendar.h"
#include "epd.h"
#include "fonts.h"
#include "lunar.h"
#include "u8g2_font.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define CLR_BLACK 0
#define CLR_RED   1
#define CLR_WHITE 2

#define U_BLACK  0x0000
#define U_RED    0xF800
#define U_WHITE  0xFFFF

/* ---- Pixel operations ---- */

static uint8_t *g_bw, *g_red;

static void set_pixel(uint8_t *bw, uint8_t *red, int x, int y, int color)
{
    if (x < 0 || x >= EPD_WIDTH || y < 0 || y >= EPD_HEIGHT) return;
    int idx = y * (EPD_WIDTH / 8) + x / 8;
    int bit = 7 - (x % 8);
    switch (color) {
        case CLR_BLACK: bw[idx] &= ~(1 << bit); red[idx] &= ~(1 << bit); break;
        case CLR_RED:   bw[idx] &= ~(1 << bit); red[idx] |=  (1 << bit); break;
        case CLR_WHITE: bw[idx] |=  (1 << bit); red[idx] &= ~(1 << bit); break;
    }
}

static void fill_rect(uint8_t *bw, uint8_t *red,
                      int x0, int y0, int x1, int y1, int color)
{
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            set_pixel(bw, red, x, y, color);
}

/* ---- u8g2 integration ---- */

static void u8g2_draw_hv_line(u8g2_font_t *u, int16_t x, int16_t y,
                               int16_t len, uint8_t dir, uint16_t color)
{
    int c = (color == U_RED) ? CLR_RED : (color == U_WHITE) ? CLR_WHITE : CLR_BLACK;
    for (int i = 0; i < len; i++) {
        switch (dir) {
            case 0: set_pixel(g_bw, g_red, x + i, y, c); break;
            case 1: set_pixel(g_bw, g_red, x, y + i, c); break;
        }
    }
}

static uint16_t utf8_next(const char **s)
{
    const uint8_t *p = (const uint8_t *)*s;
    uint16_t cp;
    if (p[0] < 0x80) {
        cp = p[0];
        *s += 1;
    } else if ((p[0] & 0xE0) == 0xC0) {
        cp = (p[0] & 0x1F) << 6 | (p[1] & 0x3F);
        *s += 2;
    } else {
        cp = (p[0] & 0x0F) << 12 | (p[1] & 0x3F) << 6 | (p[2] & 0x3F);
        *s += 3;
    }
    return cp;
}

static int16_t draw_utf8(u8g2_font_t *font, int16_t x, int16_t y, const char *s)
{
    while (*s) {
        uint16_t cp = utf8_next(&s);
        int16_t dx = u8g2_DrawGlyph(font, x, y, cp);
        x += dx;
    }
    return x;
}

static int16_t get_utf8_width(u8g2_font_t *font, const char *s)
{
    int16_t w = 0;
    while (*s) {
        uint16_t cp = utf8_next(&s);
        w += u8g2_GetGlyphWidth(font, cp);
    }
    return w;
}

/* ---- Date helpers ---- */

static int month_from_str(const char *s)
{
    static const char *m[] = {
        "Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"
    };
    for (int i = 0; i < 12; i++)
        if (s[0]==m[i][0] && s[1]==m[i][1] && s[2]==m[i][2]) return i + 1;
    return 1;
}

static int day_of_week(int y, int m, int d)
{
    static const int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
    if (m < 3) y--;
    return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

static int days_in_month(int y, int m)
{
    static const int d[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    int n = d[m-1];
    if (m==2 && (y%400==0 || (y%4==0 && y%100!=0))) n = 29;
    return n;
}

/* ---- Calendar rendering (ArtInk58 layout) ---- */

static void draw_date_header(u8g2_font_t *font, int year, int month,
                              int y, const struct Lunar_Date *lunar_today)
{
    int x = 6;
    char buf[24];

    u8g2_SetBackgroundColor(font, U_WHITE);
    u8g2_SetFontMode(font, 1);
    u8g2_SetForegroundColor(font, U_BLACK);

    snprintf(buf, sizeof(buf), "%d", year);
    u8g2_SetFont(font, u8g2_font_helvB18_tn);
    x = draw_utf8(font, x, y, buf);
    u8g2_SetFont(font, u8g2_font_wqy12_t_lunar);
    x = draw_utf8(font, x, y, "年");

    snprintf(buf, sizeof(buf), "%d", month);
    u8g2_SetFont(font, u8g2_font_helvB18_tn);
    x = draw_utf8(font, x, y, buf);
    u8g2_SetFont(font, u8g2_font_wqy12_t_lunar);
    x = draw_utf8(font, x, y, "月");

    int tx = x + 6;
    u8g2_SetFont(font, u8g2_font_wqy9_t_lunar);
    if (lunar_today->IsLeap) tx = draw_utf8(font, tx, y, Lunar_MonthLeapString[1]);
    snprintf(buf, sizeof(buf), "%s%s",
             Lunar_MonthString[lunar_today->Month],
             Lunar_DateString[lunar_today->Date]);
    draw_utf8(font, tx, y, buf);

    int y2 = y - 13;
    snprintf(buf, sizeof(buf), "%s%s年[%s]",
             Lunar_StemStrig[LUNAR_GetStem(lunar_today)],
             Lunar_BranchStrig[LUNAR_GetBranch(lunar_today)],
             Lunar_ZodiacString[LUNAR_GetZodiac(lunar_today)]);
    draw_utf8(font, tx, y2, buf);
}

static void draw_week_header(u8g2_font_t *font, int x0, int y, int cell_w)
{
    bool large = (EPD_HEIGHT >= 400);
    int h = large ? 30 : 22;
    int x1 = x0 + 7 * cell_w - 1;

    fill_rect(g_bw, g_red, x0, y, x1, y + h - 1, CLR_BLACK);
    fill_rect(g_bw, g_red, x0, y, x0 + cell_w - 1, y + h - 1, CLR_RED);
    fill_rect(g_bw, g_red, x0 + 6 * cell_w, y, x1, y + h - 1, CLR_RED);

    u8g2_SetFont(font, large ? u8g2_font_wqy12_t_lunar : u8g2_font_wqy9_t_lunar);
    u8g2_SetFontMode(font, 0);

    for (int d = 0; d < 7; d++) {
        u8g2_SetForegroundColor(font, U_WHITE);
        u8g2_SetBackgroundColor(font, (d == 0 || d == 6) ? U_RED : U_BLACK);

        int cw = get_utf8_width(font, Lunar_DayString[d]);
        int cx = x0 + d * cell_w + (cell_w - cw) / 2;
        int cy = y + (h + 9) / 2;
        draw_utf8(font, cx, cy, Lunar_DayString[d]);
    }
}

static void draw_month_days(u8g2_font_t *font, int x0, int y,
                             int today, int fw, int dim, int nrows)
{
    int bw = (EPD_WIDTH - x0 * 2) / 7;
    int bh = (EPD_HEIGHT - y - 4) / nrows;
    int sq = (EPD_HEIGHT >= 400) ? 34 : 30;

    u8g2_SetBackgroundColor(font, U_WHITE);
    u8g2_SetFontMode(font, 0);

    for (int day = 1; day <= dim; day++) {
        int col = (fw + day - 1) % 7;
        int row = (fw + day - 1) / 7;

        int cx = x0 + col * bw + bw / 2;
        int cy = y + row * bh + bh / 2;

        bool weekend = (col == 0 || col == 6);
        bool is_today = (day == today);

        char num_buf[8];
        snprintf(num_buf, sizeof(num_buf), "%u", (unsigned)(uint8_t)day);

        if (is_today) {
            /* 用户裁决 10-30:当前日期=黑底白字(原红底);今日有代办时
             * overlay 层在其外围再画红框 */
            fill_rect(g_bw, g_red, cx - sq/2, cy - sq/2 - 2, cx + sq/2, cy + sq/2 + 2, CLR_BLACK);
            u8g2_SetFont(font, u8g2_font_helvB18_tn);
            u8g2_SetFontMode(font, 1);
            u8g2_SetForegroundColor(font, U_WHITE);
            int nw = get_utf8_width(font, num_buf);
            draw_utf8(font, cx - nw / 2, cy + 6, num_buf);
            u8g2_SetFontMode(font, 0);
        } else {
            u8g2_SetFont(font, u8g2_font_helvB18_tn);
            u8g2_SetForegroundColor(font, weekend ? U_RED : U_BLACK);
            int nw = get_utf8_width(font, num_buf);
            draw_utf8(font, cx - nw / 2, cy + 6, num_buf);
        }
    }
}

/* ---- Public API ---- */

static void ensure_tz(void)
{
    static bool tz_done;
    if (!tz_done) {
        setenv("TZ", "CST-8", 1);   /* China, no DST */
        tzset();
        tz_done = true;
    }
}

bool epd_calendar_time_ready(void)
{
    time_t now;
    time(&now);
    return now > 1700000000;   /* after 2023-11: NTP-synced */
}

void epd_calendar_render_month(uint8_t *bw, uint8_t *red,
                               int year, int month, int today_day)
{
    ensure_tz();

    if (month < 1) { month += 12; year--; }
    if (month > 12) { month -= 12; year++; }

    int fw    = day_of_week(year, month, 1);
    int dim   = days_in_month(year, month);
    int nrows = (fw + dim + 6) / 7;

    memset(bw, 0xFF, EPD_BUF_SIZE);
    memset(red, 0x00, EPD_BUF_SIZE);
    g_bw = bw;
    g_red = red;

    u8g2_font_t font;
    memset(&font, 0, sizeof(font));
    font.draw_hv_line = u8g2_draw_hv_line;

    /* 表头农历参照日:本月=今天;浏览月=该月 1 日(用户裁决 10-31) */
    struct Lunar_Date lunar_ref;
    int lunar_day = (today_day > 0) ? today_day : 1;
    LUNAR_SolarToLunar(&lunar_ref, year, month, lunar_day);

    bool large = (EPD_HEIGHT >= 400);
    int margin = 6;
    int cell_w = (EPD_WIDTH - margin * 2) / 7;

    int header_y = large ? 38 : 28;
    draw_date_header(&font, year, month, header_y, &lunar_ref);

    int wh_y = large ? 44 : 32;
    int wh_h = large ? 30 : 22;
    draw_week_header(&font, margin, wh_y, cell_w);

    draw_month_days(&font, margin, wh_y + wh_h, today_day, fw, dim, nrows);
}

void epd_calendar_render(uint8_t *bw, uint8_t *red)
{
    ensure_tz();
    int year, month, today;
    if (epd_calendar_time_ready()) {
        time_t now;
        struct tm tm_now;
        time(&now);
        localtime_r(&now, &tm_now);
        year  = tm_now.tm_year + 1900;
        month = tm_now.tm_mon + 1;
        today = tm_now.tm_mday;
    } else {
        char mstr[4] = {0};
        int mday;
        sscanf(__DATE__, "%3s %d %d", mstr, &mday, &year);
        month = month_from_str(mstr);
        today = mday;
    }
    epd_calendar_render_month(bw, red, year, month, today);
}
