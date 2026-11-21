/* epd_geom.c — 面板表 + 运行时几何访问器（host/firmware 共用：harness
 * 直接编译本文件，EPD_PANEL env 注入；固件侧 epd_geom_set 由 app init
 * 从 identity NVS 调用）。表错误由编译期 _Static_assert 锁死（91 R26-1）。 */
#include "epd_geom.h"

#include <stddef.h>
#include <stdlib.h>

typedef struct {
    epd_panel_t id;
    int width;
    int height;
} panel_row_t;

/* C 的 const 非常量表达式——表与断言共用同一组宏（R26-1 双轨：宏锁
 * 编译期，表供运行期，二者结构上不可能失配） */
#define P42_W 400
#define P42_H 300
#define P583_W 648
#define P583_H 480

static const panel_row_t k_panels[] = {
    { EPD_PANEL_P42_V2, P42_W, P42_H },
    { EPD_PANEL_P583,   P583_W, P583_H },
};

_Static_assert(P42_W % 8 == 0 && P583_W % 8 == 0,
               "panel width must be byte-aligned");
_Static_assert(P42_W / 8 * P42_H == 15000, "4.2 plane bytes");
_Static_assert(P583_W / 8 * P583_H == 38880, "5.83 plane bytes");
_Static_assert(P42_W / 8 * P42_H <= EPD_BUF_MAX && P583_W / 8 * P583_H <= EPD_BUF_MAX,
               "EPD_BUF_MAX must cover table");

static epd_panel_t s_panel = EPD_PANEL_COMPILE_DEFAULT;

void epd_geom_set(epd_panel_t p)
{
    if (p == EPD_PANEL_P42_V2 || p == EPD_PANEL_P583)
        s_panel = p;
}

epd_panel_t epd_panel_current(void)
{
#if defined(EPD_GEOM_HOST)
    /* harness：env 注入（"1"=4.2 "2"=5.83），覆盖编译默认 */
    const char *e = getenv("EPD_PANEL");
    if (e && e[0] == '1') return EPD_PANEL_P42_V2;
    if (e && e[0] == '2') return EPD_PANEL_P583;
#endif
    return s_panel;
}

bool epd_panel_is_583(void)
{
    return epd_panel_current() == EPD_PANEL_P583;
}

static const panel_row_t *row(void)
{
    for (size_t i = 0; i < sizeof(k_panels)/sizeof(k_panels[0]); i++)
        if (k_panels[i].id == epd_panel_current())
            return &k_panels[i];
    return &k_panels[0];
}

int epd_width(void)    { return row()->width; }
int epd_height(void)   { return row()->height; }
int epd_buf_size(void) { return row()->width / 8 * row()->height; }
int epd_buf_max(void)  { return 38880; }
