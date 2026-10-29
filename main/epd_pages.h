#ifndef EPD_PAGES_H
#define EPD_PAGES_H

#include <stdint.h>

/* docs/74 M1 模式渲染器 — 按 display_mode 的 (mode,payload) 出帧。
 * 分辨率自适应:所有布局以 EPD_WIDTH/HEIGHT 推导(74 §7)。 */
void epd_pages_render(int mode, const char *payload_json,
                      uint8_t *bw, uint8_t *red);

#endif
