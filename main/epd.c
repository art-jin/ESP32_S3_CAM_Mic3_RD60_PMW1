/* epd.c — e-ink register-level drivers, ported from ArtInk4 (both panels
 * bench-verified there on the same bench):
 *   - 5.83" 648x480 B/W/R, FPC-8601B, UC8179/GD7965 (BUSY read needs the
 *     0x71 FLG command first; LOW = busy; ~20 s full refresh)
 *   - 4.2" 400x300 B/W/R, Hink-E042A48-A1, SSD1683 (BUSY LOW = busy; RST
 *     low pulse must stay ~2 ms — longer may power-cycle the adapter LDO)
 * Panel variant is selected in board_config.h (EPD_PANEL_5IN83).
 *
 * Bench-earned invariants (2026-10-27):
 *   - spi_bus_initialize must run exactly once per boot (s_spi_inited).
 *   - Sub-10 ms waits need esp_rom_delay_us: the FreeRTOS tick is 100 Hz,
 *     so pdMS_TO_TICKS(2) == 0 and tick-based "2 ms polls" spin instantly.
 *   - After an update command, pace with the two-phase wait (engage +
 *     release); if the panel never signals, fall back to a fixed settle
 *     or back-to-back frames stomp each other.
 */
#include "epd.h"
#include "board_config.h"

#if ENABLE_EPD

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "evlog.h"
#include <string.h>

/* GYSDISPLAY 24-pin adapter: brown=SCK(4) black=SDO/DIN(5) white=D/C(6)
 * gray=BUSY(7) green=CS(15) blue=RES(16) purple=BS(17, LOW=4-wire SPI). */
#define PIN_SCK   EPD_SCK_GPIO
#define PIN_MOSI  EPD_SDO_GPIO
#define PIN_CS    EPD_CS_GPIO
#define PIN_DC    EPD_DC_GPIO
#define PIN_RES   EPD_RES_GPIO
#define PIN_BS    EPD_BS_GPIO
#define PIN_BUSY  EPD_BUSY_GPIO

#define TAG "EPD"

static spi_device_handle_t spi;
static bool s_spi_inited;      /* spi_bus_initialize must run exactly once */
static bool s_busy_timeout;

static void send_cmd(uint8_t c)
{
    gpio_set_level(PIN_DC, 0);
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &c;
    spi_device_polling_transmit(spi, &t);
}

static void send_data(uint8_t d)
{
    gpio_set_level(PIN_DC, 1);
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &d;
    spi_device_polling_transmit(spi, &t);
}

/* Framebuffer staging: SPI DMA must only read INTERNAL RAM. With
 * SPIRAM_USE_MALLOC and MALLOC_ALWAYSINTERNAL=16384, the 5.83" 38880-byte
 * plane buffers land in PSRAM — DMA reads from PSRAM come back
 * cache-incoherent (busy handshake + refresh healthy, pixels garbled;
 * 2026-11-17 MENGOR 花屏实录, two full attempts). Plain CPU loads from
 * PSRAM are fine, so bounce every chunk through this internal staging
 * buffer before handing it to the SPI engine. Cost: one 4 KB memcpy per
 * chunk — negligible against a 20 s panel refresh. */
static uint8_t s_tx_stage[4096] __attribute__((aligned(4)));

static void send_buf(const uint8_t *buf, int len)
{
    gpio_set_level(PIN_DC, 1);
    int off = 0;
    while (off < len) {
        int chunk = len - off;
        if (chunk > sizeof(s_tx_stage)) chunk = sizeof(s_tx_stage);
        memcpy(s_tx_stage, buf + off, chunk);
        spi_transaction_t t = {0};
        t.length = chunk * 8;
        t.tx_buffer = s_tx_stage;
        spi_device_polling_transmit(spi, &t);
        off += chunk;
    }
}

/* BUSY pin: LOW = busy on all panels. UC8179 family requires the 0x71 (FLG)
 * command before each pin read — confirmed by the Waveshare 4.2b-V2 protocol
 * (pengwon/epd42, bench-proven on the same P420016-MF1-A panel). SSD1683 has
 * no FLG and must NOT receive it (an intermediate edit did exactly that and
 * garbled the 4.2 panel — 上黑下红花屏, measured 2026-11-04). */
static int read_busy(void)
{
    return gpio_get_level(PIN_BUSY);
}

/* GPIO + one-time SPI setup shared by all entry points. */
static void ensure_buses(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_DC) | (1ULL << PIN_RES) | (1ULL << PIN_BS),
        .mode = GPIO_MODE_INPUT_OUTPUT,   /* IDF v6: needed for gpio_get_level */
    };
    gpio_config(&out);
    gpio_set_level(PIN_BS, 0);   /* LOW = 4-wire SPI (adapter has a BS pin) */

    gpio_config_t in = {
        .pin_bit_mask = 1ULL << PIN_BUSY,
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&in);

    if (!s_spi_inited) {
        spi_bus_config_t bus = {
            .sclk_io_num = PIN_SCK,
            .mosi_io_num = PIN_MOSI,
            .miso_io_num = -1,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4096,
        };
        spi_device_interface_config_t dev = {
            .clock_speed_hz = 1000000,   /* 2026-11-17: 2 MHz garbles long 5.83" streams in the noisy neck env (PSRAM octal on 35-37 + I2S + radar); 1 MHz for margin */
            .mode = 0,
            .spics_io_num = PIN_CS,
            .queue_size = 1,
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &spi));
        s_spi_inited = true;
    }
}

/* Two-phase refresh wait (ArtInk58-verified): LUT load + temperature
 * conversion can delay the busy ASSERT by seconds — allow a 15 s engage
 * window, then wait out the refresh (120 s cap for tri-color). */
static bool wait_busy2(int engage_ms)
{
    int us = 0;
    while (read_busy()) {                       /* HIGH = not busy yet */
        esp_rom_delay_us(500);
        us += 500;
        if (us % 10000 == 0) vTaskDelay(pdMS_TO_TICKS(10));  /* feed IDLE/WDT */
        if (us > engage_ms * 1000) return false;   /* never engaged */
    }
    int ms = 0;
    while (!read_busy()) {                      /* LOW = busy */
        vTaskDelay(pdMS_TO_TICKS(10));
        ms += 10;
        if (ms > 120000) {
            s_busy_timeout = true;
            ESP_LOGE(TAG, "BUSY release timeout %d ms", ms);
            return false;
        }
    }
    return true;
}


/* ===== 5.83" 648x480 BWR, UC8179-V2 protocol (Waveshare epd5in83b_V2) ====
 * 2026-11-17: NEW panel (the 2026-11-04 Hanshow salvage was judged dead —
 * RAM not holding writes — see acf4d65). Protocol empirically verified on
 * the new panel in ArtInk58 (same GOOUUU board, same EPD pins):
 *   - red plane is 0x13 (0x26 write silently ignored — NOT the SSD1683 map)
 *   - 0x50 VCOM/data interval takes TWO bytes (0x11 0x07); one byte leaves
 *     every band speckled
 *   - a 0x71 FLG command must precede EACH BUSY pin read (branch-local —
 *     the 42_V2 and SSD1683 panels must NOT receive it)
 *   - init order: power setting → power-on → remaining registers
 * Planes 0x10 BW (1=white) / 0x13 red (1=red), update 0x12, LOW=busy.
 * Per-frame reinit (hw_reset_5 + registers + PON) retained from the dead-
 * panel rewrite: COG write-pointer drift insurance, same as the 4.2". */

static void hw_reset_5(void)
{
    gpio_set_level(PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(PIN_RES, 0);
    vTaskDelay(pdMS_TO_TICKS(20));   /* ArtInk58-verified pulse */
    gpio_set_level(PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
}

/* LOW = busy. Each pin read is preceded by 0x71 FLG (V2 requirement). */
static bool wait_idle_5(int cap_ms)
{
    int ms = 0;
    while (1) {
        send_cmd(0x71);
        if (gpio_get_level(PIN_BUSY)) break;
        vTaskDelay(pdMS_TO_TICKS(10));
        ms += 10;
        if (ms > cap_ms) return false;
    }
    return true;
}

static bool start_update_and_wait_5(void)
{
    send_cmd(0x12);               /* master update */
    bool engaged = false;
    for (int ms = 0; ms < 15000; ms += 10) {   /* engage: LOW within 15 s */
        send_cmd(0x71);
        if (!gpio_get_level(PIN_BUSY)) { engaged = true; break; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    bool done = wait_idle_5(120000);           /* release: HIGH within 120 s */
    evlog_record(EV_EPD, EPD_PHASE_UPDATE_ENGAGED, engaged ? 1 : 0);
    return done;
}

/* ArtInk58-verified order: power setting → PON → remaining registers;
 * re-armed before EVERY frame. */
static bool load_registers_and_power_on(void)
{
    send_cmd(0x01);               /* power setting */
    send_data(0x07);
    send_data(0x07);
    send_data(0x3F);
    send_data(0x3F);
    send_cmd(0x04);               /* power on — EARLY, before config regs */
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!wait_idle_5(10000)) {
        ESP_LOGE(TAG, "PON no idle — power/panel issue");
        return false;
    }
    send_cmd(0x00);               /* panel setting: BW OTP LUT */
    send_data(0x0F);
    send_cmd(0x61);               /* resolution — derive from geometry */
    send_data((epd_width() >> 8) & 0xFF);
    send_data(epd_width() & 0xFF);
    send_data((epd_height() >> 8) & 0xFF);
    send_data(epd_height() & 0xFF);
    send_cmd(0x15);               /* VGL/VGH option */
    send_data(0x00);
    send_cmd(0x50);               /* VCOM + data interval — TWO bytes (V2) */
    send_data(0x11);
    send_data(0x07);
    send_cmd(0x60);               /* TCON */
    send_data(0x22);
    /* 2026-11-17: 显式全屏窗口+退出局部模式。这块屏今天早期跑过带窗口
     * 命令的实验固件+DC 抖动期的垃圾命令流，COG 可能锁存了窄窗口/
     * partial-in 状态（左侧月历正确+右侧 1/3 永不更新的形态吻合）。
     * ArtInk58 的极简 init 无窗口命令——但它的屏从未被污染过。每次
     * init 强制拉回已知全屏态，防锁存残留。 */
    send_cmd(0x92);               /* partial-window OUT */
    send_cmd(0x11);               /* data entry: X inc, Y inc */
    send_data(0x03);
    send_cmd(0x44);               /* X window: FULL width (bytes) */
    send_data(0x00);
    send_data((epd_width() / 8) - 1);
    send_cmd(0x45);               /* Y window: FULL height */
    send_data(0x00); send_data(0x00);
    send_data((epd_height() - 1) & 0xFF);
    send_data((epd_height() - 1) >> 8);
    send_cmd(0x4E);               /* X cursor = 0 */
    send_data(0x00);
    send_cmd(0x4F);               /* Y cursor = 0 */
    send_data(0x00); send_data(0x00);
    return true;
}

/* Per-frame reinit: zero the write pointer + reload scan geometry (the COG
 * drifts both across refreshes, exactly like the 4.2 sibling). */
static bool panel_reinit_5(void)
{
    hw_reset_5();
    return load_registers_and_power_on();
}

/* 2026-11-17 花屏实录（第一版移植）: 4.2 时代的"保险"——每帧 panel_reinit
 * + set_ram_window 寻址 + 帧后 0x02 断电——在这块 V2 COG 上全部致命：
 * BUSY 握手正常、20 s 全刷照走，但像素映射花屏。ArtInk58 在同一块屏上
 * 的实证序列是极简流：init 一次（复位+寄存器+PON），之后每帧只写
 * 0x10/0x13 两平面 + 0x12 触发，无 reinit/无窗口/无断电，连续多帧
 * display 稳定。display 路径必须保持这个最小形态。 */

bool epd_init_5(void)
{
    ensure_buses();
    if (!panel_reinit_5()) {
        evlog_record(EV_EPD, EPD_PHASE_INIT_FAIL, read_busy());
        return false;
    }
    ESP_LOGI(TAG, "UC8179-V2 init done");
    evlog_record(EV_EPD, EPD_PHASE_INIT_OK, read_busy());
    return true;
}

void epd_clear_5(void)
{
    uint8_t *buf = malloc(epd_buf_size());
    if (!buf) return;
    memset(buf, 0xFF, epd_buf_size());
    send_cmd(0x10);               /* BW plane: all white */
    send_buf(buf, epd_buf_size());
    send_cmd(0x13);               /* RED plane: none */
    memset(buf, 0x00, epd_buf_size());
    send_buf(buf, epd_buf_size());
    free(buf);
    start_update_and_wait_5();
}

void epd_display_5(const uint8_t *bw, const uint8_t *red)
{
    send_cmd(0x10);
    send_buf(bw, epd_buf_size());
    send_cmd(0x13);
    send_buf(red, epd_buf_size());
    start_update_and_wait_5();
}

void epd_sleep_5(void)
{
    send_cmd(0x02);
    wait_idle_5(5000);
    send_cmd(0x07);
    send_data(0xA5);
}

/* Bench diagnostic (2026-11-17 花屏 hunt): plane-selective display.
 * which: 0=both planes, 1=BW only (red forced none), 2=RED only (BW forced
 * white). Isolates whether corruption begins with the second plane write. */
void epd_display_planes_5(const uint8_t *bw, const uint8_t *red, int which);

void epd_display_planes_5(const uint8_t *bw, const uint8_t *red, int which)
{
    /* bench 专用：堆分配替代 38.9KB .bss（11-22 实案：bss 膨胀挤压内部
     * DRAM → TLS 握手 -0x3000 两连俱亡——epd 几何合并没有罪，是这块
     * 恒定上限静态缓冲的锅） */
    uint8_t *blank = malloc(epd_buf_size());
    if (!blank) return;
    if (which == 1) {
        memset(blank, 0x00, epd_buf_size());
        red = blank;
    } else if (which == 2) {
        memset(blank, 0xFF, epd_buf_size());
        bw = blank;
    }
    send_cmd(0x10);
    send_buf(bw, epd_buf_size());
    send_cmd(0x13);
    send_buf(red, epd_buf_size());
    start_update_and_wait_5();
    free(blank);
}


/* ====== 4.2" 400x300 tri-color P420016-MF1-A (ESL salvage, anan bench
 * 2026-11-04). SSD1683-style command set with THREE divergences measured by
 * Admin-233/displayDrivers epd4in2b_V2 (bench-proven on this exact panel,
 * see xfox.fun/archives/1750):
 *   1. BUSY polarity INVERTED: HIGH = busy, LOW = idle (and no 0x71 FLG)
 *   2. NO 0x01 driver-output-control (misconfigures this COG's gate scan)
 *   3. extra 0x21 (display update control) ← 0x80 0x80
 * Planes 0x24/0x26, update 0x22←0xF7 + 0x20 — same as the SSD1683 branch. */

static void hw_reset_42(void)
{
    gpio_set_level(PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(PIN_RES, 0);
    esp_rom_delay_us(2000);
    gpio_set_level(PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
}

/* This panel: HIGH = busy. Wait for the idle (LOW) level. */
static bool wait_idle_42(int cap_ms)
{
    int ms = 0;
    while (gpio_get_level(PIN_BUSY)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ms += 10;
        if (ms > cap_ms) return false;
    }
    return true;
}

static void set_ram_window_42(void)
{
    send_cmd(0x11);               /* data entry: X inc, Y inc */
    send_data(0x03);
    send_cmd(0x44);               /* x window: bytes 0..49 */
    send_data(0x00);
    send_data((epd_width() / 8) - 1);
    send_cmd(0x45);               /* y window: 0..299 */
    send_data(0x00); send_data(0x00);
    send_data((epd_height() - 1) & 0xFF);
    send_data((epd_height() - 1) >> 8);
    send_cmd(0x4E);               /* cursor (0,0) */
    send_data(0x00);
    send_cmd(0x4F);
    send_data(0x00); send_data(0x00);
}

static bool start_update_and_wait_42(void)
{
    send_cmd(0x22);               /* OTP full refresh */
    send_data(0xF7);
    send_cmd(0x20);
    /* Two-phase wait (ArtInk58 bench invariant): busy assert can lag by
     * seconds on tri-color LUTs — a single immediate poll reads "idle"
     * during the assert window and returns early, the next frame then
     * writes into a refreshing panel and the image lands wrap-shifted
     * (measured: calendar header at lower-middle, 2026-11-04). */
    bool engaged = false;
    for (int ms = 0; ms < 15000; ms += 10) {
        if (gpio_get_level(PIN_BUSY)) { engaged = true; break; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    bool done = wait_idle_42(120000);
    evlog_record(EV_EPD, EPD_PHASE_UPDATE_ENGAGED, engaged ? 1 : 0);
    return done;
}

/* This COG advances its write pointer across refresh cycles and never
 * auto-wraps to the window origin (0x24 does not home the counter, and
 * lone 0x4E/0x4F re-sends don't stick after a refresh). Only a full
 * reset+SWRESET+window run zeros it — the boot render is always correct,
 * every later frame wraps shifted (calendar header to lower-middle,
 * measured 2026-11-04). So: re-init before every frame. At the 30 s
 * refresh cadence the ~1 s cost is free. */
static bool panel_reinit_42(void)
{
    hw_reset_42();
    wait_idle_42(5000);
    send_cmd(0x12);               /* SW reset */
    if (!wait_idle_42(15000)) return false;
    set_ram_window_42();
    send_cmd(0x3C);
    send_data(0x05);
    send_cmd(0x18);
    send_data(0x80);
    send_cmd(0x21);
    send_data(0x80);
    send_data(0x80);
    wait_idle_42(5000);
    return true;
}

bool epd_init_42(void)
{
    ensure_buses();
    if (!panel_reinit_42()) {
        ESP_LOGE(TAG, "no idle after SW reset — check power/wiring (BUSY=%d)",
                 read_busy());
        evlog_record(EV_EPD, EPD_PHASE_INIT_FAIL, read_busy());
        return false;
    }
    ESP_LOGI(TAG, "P420016 4.2 V2 init done");
    evlog_record(EV_EPD, EPD_PHASE_INIT_OK, read_busy());
    return true;
}

void epd_clear_42(void)
{
    uint8_t *buf = malloc(epd_buf_size());
    if (!buf) return;
    panel_reinit_42();
    memset(buf, 0xFF, epd_buf_size());
    send_cmd(0x24);               /* BW plane: all white */
    send_buf(buf, epd_buf_size());
    send_cmd(0x26);               /* RED plane: wire 1 = no-red here */
    send_buf(buf, epd_buf_size());
    free(buf);
    start_update_and_wait_42();
}

void epd_display_42(const uint8_t *bw, const uint8_t *red)
{
    static uint8_t inv[4096];     /* wire 0=red / 1=white ⇒ invert our red */
    panel_reinit_42();
    send_cmd(0x24);
    send_buf(bw, epd_buf_size());
    send_cmd(0x26);
    for (int off = 0; off < epd_buf_size(); off += sizeof(inv)) {
        int n = epd_buf_size() - off;
        if (n > (int)sizeof(inv)) n = sizeof(inv);
        for (int i = 0; i < n; i++) inv[i] = ~red[off + i];
        send_buf(inv, n);
    }
    start_update_and_wait_42();
}

void epd_sleep_42(void)
{
    send_cmd(0x10);
    send_data(0x01);
}


/* ===== R26-A2: 运行时面板分派（91 v1.1 切片 A）===== */
bool epd_init(void)
{
    return epd_panel_is_583() ? epd_init_5() : epd_init_42();
}

void epd_clear(void)
{
    if (epd_panel_is_583()) epd_clear_5(); else epd_clear_42();
}

void epd_display(const uint8_t *bw, const uint8_t *red)
{
    if (epd_panel_is_583()) epd_display_5(bw, red); else epd_display_42(bw, red);
}

void epd_sleep(void)
{
    if (epd_panel_is_583()) epd_sleep_5(); else epd_sleep_42();
}

void epd_display_planes(const uint8_t *bw, const uint8_t *red, int which)
{
    if (!epd_panel_is_583()) return;   /* 平面隔离 = 5.83 bench 专用 */
    epd_display_planes_5(bw, red, which);
}


/* ---- Diagnostics (panel-agnostic) ---- */

bool epd_wiring_check(epd_wiring_t *out)
{
    memset(out, 0, sizeof(*out));
    ensure_buses();

    /* 0. Line-state test: BUSY with a weak internal pull-up. HIGH now =
     * nothing drives the line (broken wire / no contact). LOW = actively
     * pulled low (panel asserting busy — a wedge, not a wiring fault). */
    gpio_config_t pu = {
        .pin_bit_mask = 1ULL << PIN_BUSY,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&pu);
    vTaskDelay(pdMS_TO_TICKS(10));
    out->busy_pullup = gpio_get_level(PIN_BUSY);
    gpio_config(&(gpio_config_t){   /* restore plain input */
        .pin_bit_mask = 1ULL << PIN_BUSY,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    });

    /* 1. BUSY stability while idle. */
    int last = read_busy();
    out->busy_idle_level = last;
    for (int i = 0; i < 100; i++) {
        esp_rom_delay_us(2000);
        int v = read_busy();
        if (v != last) out->busy_flips++;
        last = v;
    }
    out->busy_idle_level = last;

    /* 2. RES pulse -> panel must pull BUSY low briefly (proves power +
     * RES + BUSY). */
    for (int i = 0; i < 500 && read_busy() == 0; i++)
        esp_rom_delay_us(2000);
    gpio_set_level(PIN_RES, 0);
    esp_rom_delay_us(10000);         /* ~10 ms, works for both panels */
    gpio_set_level(PIN_RES, 1);
    for (int i = 0; i < 250; i++) {  /* 500 ms reaction window */
        esp_rom_delay_us(2000);
        if (!read_busy()) { out->reset_reaction = true; break; }
    }

    /* 3. Reset command over SPI -> BUSY reaction proves SCK/SDA/CS/DC. */
    for (int i = 0; i < 500 && read_busy() == 0; i++)
        esp_rom_delay_us(2000);
    send_cmd(0x12);
    /* UC8179=update cmd(busy 长) / SSD1683=SW reset(busy 短) */
    out->cmd_reaction = wait_busy2(epd_panel_is_583() ? 15000 : 5000);
    /* Drain the post-reset busy period so a later init starts clean. */
    for (int i = 0; i < 500 && read_busy() == 0; i++)
        esp_rom_delay_us(2000);

    return out->reset_reaction && out->cmd_reaction;
}

/* Wedge-breaker for a stuck-busy panel: 100 ms RES low. */
int epd_deep_recovery(void)
{
    ensure_buses();
    gpio_set_level(PIN_RES, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    return read_busy();
}

/* Wire-matrix sweep: pulse each signal pin low for ~10 ms and watch every
 * other pin for LOW samples during the following 300 ms. A live panel
 * answers a RES pulse with a BUSY drop — wherever that shows up tells us
 * the true wiring. */
const int *epd_pin_list(void)
{
    static const int pins[7] = {
        EPD_SCK_GPIO, EPD_SDO_GPIO, EPD_DC_GPIO, EPD_BUSY_GPIO,
        EPD_CS_GPIO, EPD_RES_GPIO, EPD_BS_GPIO
    };
    return pins;
}

int epd_wire_sweep(int reacts[7])
{
    ensure_buses();
    const int *pins = epd_pin_list();
    memset(reacts, 0, sizeof(int) * 7);
    int best_watch = -1, best_watch_lows = 0;

    for (int p = 0; p < 7; p++) {
        if (pins[p] < 0) continue;
        /* All other pins become pulled-up inputs so we can watch them. */
        for (int i = 0; i < 7; i++) {
            if (i == p || pins[i] < 0) continue;
            gpio_config(&(gpio_config_t){
                .pin_bit_mask = 1ULL << pins[i],
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
            });
        }
        vTaskDelay(pdMS_TO_TICKS(5));

        /* Pulse pin p low ~10 ms, then high. */
        gpio_config(&(gpio_config_t){
            .pin_bit_mask = 1ULL << pins[p],
            .mode = GPIO_MODE_OUTPUT,
        });
        gpio_set_level(pins[p], 0);
        esp_rom_delay_us(10000);
        gpio_set_level(pins[p], 1);

        /* Watch the others for LOW samples over 300 ms (real 2 ms polls).
         * Attribution: record which pulse index caused them. */
        int lows[7] = {0};
        for (int s = 0; s < 150; s++) {
            esp_rom_delay_us(2000);
            for (int i = 0; i < 7; i++) {
                if (i == p || pins[i] < 0) continue;
                if (!gpio_get_level(pins[i])) lows[i]++;
            }
        }
        for (int i = 0; i < 7; i++) {
            if (lows[i] > 3) {
                reacts[i] += lows[i];
                ESP_LOGI(TAG, "sweep: pulse pin %d (idx %d) -> pin %d (idx %d) LOW x%d",
                         pins[p], p, pins[i], i, lows[i]);
                if (lows[i] > best_watch_lows) {
                    best_watch_lows = lows[i];
                    best_watch = i;
                }
            }
        }
    }
    return best_watch;
}

#else /* !ENABLE_EPD — board without the panel: no-op degradation */

bool epd_init(void) { return false; }
void epd_clear(void) {}
void epd_display(const uint8_t *bw, const uint8_t *red) { (void)bw; (void)red; }
void epd_sleep(void) {}
bool epd_wiring_check(epd_wiring_t *out) { *out = (epd_wiring_t){0}; return false; }
int epd_deep_recovery(void) { return 0; }
const int *epd_pin_list(void)
{
    static const int pins[7] = { -1, -1, -1, -1, -1, -1, -1 };
    return pins;
}
int epd_wire_sweep(int reacts[7]) { for (int i = 0; i < 7; i++) reacts[i] = 0; return -1; }

#endif
