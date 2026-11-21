/* epd_geom.h — R26-A：面板几何运行时化（91 v1.1 切片 A）。
 *
 * EPD_WIDTH/HEIGHT/BUF_SIZE 宏退役（删宏编译报名），一律经本模块访问器。
 * 选择源：epd_geom_set()（app 启动时从 identity NVS epd_panel 键注入，
 * 缺省=编译默认 4.2——失败可见原则 R26-2）；host 构建读 EPD_PANEL env。
 */
#ifndef EPD_GEOM_H
#define EPD_GEOM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    EPD_PANEL_P42_V2 = 1,   /* 4.2" P420016-MF1 UC8179 家族 400x300 */
    EPD_PANEL_P583   = 2,   /* 5.83" FPC-8601B UC8179-V2 648x480 */
} epd_panel_t;

/* 编译默认（R26-2 用户裁决：4.2 失败可见——5.83 几何喂 4.2 屏=PON 挂死，
 * 4.2 几何喂 5.83 屏=局部渲染可见可纠） */
#define EPD_PANEL_COMPILE_DEFAULT EPD_PANEL_P42_V2

void          epd_geom_set(epd_panel_t p);   /* app init 注入（幂等） */
epd_panel_t   epd_panel_current(void);
bool          epd_panel_is_583(void);

int           epd_width(void);               /* px */
int           epd_height(void);
int           epd_buf_size(void);            /* width/8*height 每平面字节 */

/* 全表最大平面字节数（静态/编译期缓冲上界；表驱断言保证 ≤ 此值） */
#define EPD_BUF_MAX 38880

#endif
