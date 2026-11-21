/* epd.h — 4.2" 400x300 B/W/R e-ink (Hink-E042A48-A1, SSD1683, 24-pin FPC
 * via the GYSDISPLAY adapter). Driver ported from ArtInk4 (bench-verified
 * there 2026-10; same panel). All functions compile to no-ops when
 * ENABLE_EPD=0 in board_config.h.
 *
 * Pixel packing: MSB = leftmost pixel, idx = y*(EPD_WIDTH/8) + x/8,
 * bit = 7 - x%8. Plane encoding: black = B0/R0, white = B1/R0, red = B0/R1.
 * Full refresh takes ~14 s (OTP tri-color waveform) — no partial refresh.
 */
#ifndef EPD_H
#define EPD_H

#include "board_config.h"   /* 2026-11-17: 宏可见性自携带——EPD_PANEL_* 由本头
                              * 自带，杜绝各 .c 因 include 顺序不同而对屏幕几何
                              * 认知分裂（当日实录：epd.c 活在 400x300 只发左
                              * 62%，渲染/任务在 648x480——左侧月历右侧残影）。 */
#include <stdbool.h>
#include <stdint.h>
#include "epd_geom.h"   /* R26-A3: EPD_WIDTH/HEIGHT/BUF_SIZE 宏退役——
                         * 运行时访问器 epd_width()/epd_height()/epd_buf_size() */

/* Panel variant: 运行时选择（epd_geom.c 面板表，identity NVS epd_panel 注入，
 * 编译默认 4.2）。5.83" FPC-8601B UC8179-V2 / 4.2" P420016-MF1。 */

/* Returns false when init failed (BUSY timeout — usually panel absent). */
bool epd_init(void);
void epd_clear(void);                              /* white, ~14 s */
void epd_display(const uint8_t *bw, const uint8_t *red);  /* ~14 s */
void epd_sleep(void);
void epd_display_planes(const uint8_t *bw, const uint8_t *red, int which);

/* Wiring self-check (~2 s, repeatable). BUSY is the panel's only feedback
 * line: a RES pulse proves panel power + RES + BUSY; a SW-reset command
 * proves the SPI chain (SCK/SDO/CS/DC; a wrong BS shows up here too). */
typedef struct {
    int  busy_flips;       /* BUSY line flips while idle; >5 = floating */
    int  busy_idle_level;  /* last idle sample */
    int  busy_pullup;      /* BUSY read with internal pull-up: 1 = line was
                            * floating (nothing drives it), 0 = actively
                            * pulled low (panel asserting busy) */
    bool reset_reaction;   /* power + RES + BUSY wiring verified */
    bool cmd_reaction;     /* SCK/SDO/CS/DC wiring verified */
} epd_wiring_t;

bool epd_wiring_check(epd_wiring_t *out);

/* Hard panel reset for a stuck-busy panel: 100 ms RES low. Returns the
 * BUSY level right after (1 = recovered to idle). */
int epd_deep_recovery(void);

/* Wire-matrix sweep: pulse each signal GPIO low in turn and watch all
 * others for a reaction. Fills reacts[7] with the number of LOW samples
 * seen on each pin index (0 = SCK..6 = BUSY per epd_pin_list()). Used to
 * locate the true wiring when the panel stays silent. Returns the pin
 * index whose pulse produced the strongest reaction, -1 if none. */
const int *epd_pin_list(void);   /* {SCK,SDO,DC,BUSY,CS,RES,BS} GPIOs */
int epd_wire_sweep(int reacts[7]);

#endif
