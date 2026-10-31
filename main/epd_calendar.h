#ifndef EPD_CALENDAR_H
#define EPD_CALENDAR_H

#include <stdbool.h>
#include <stdint.h>

/* Lunar calendar renderer ported from ArtInk58 (bench-verified on this
 * very panel). Renders the current month into the caller's EPD planes. */

/* True once wall-clock time looks valid (NTP-synced). */
bool epd_calendar_time_ready(void);

/* Render the current month (header + lunar info + weekday bar + day grid,
 * today highlighted in red). Falls back to the compile date when time is
 * not yet valid. */
void epd_calendar_render(uint8_t *bw, uint8_t *red);

/* Render an arbitrary month (docs/74 M3.2 month_view). today_day < 0 =
 * browse view: no today highlight, header lunar info taken from the 1st
 * of that month (用户裁决 10-31:浏览月没有"今天"概念). */
void epd_calendar_render_month(uint8_t *bw, uint8_t *red,
                               int year, int month, int today_day);

#endif
