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

static void send_buf(const uint8_t *buf, int len)
{
    gpio_set_level(PIN_DC, 1);
    int off = 0;
    while (off < len) {
        int chunk = len - off;
        if (chunk > 4096) chunk = 4096;
        spi_transaction_t t = {0};
        t.length = chunk * 8;
        t.tx_buffer = buf + off;
        spi_device_polling_transmit(spi, &t);
        off += chunk;
    }
}

/* UC8179 needs a 0x71 (FLG) command before each BUSY pin read. */
static int read_busy(void)
{
#if defined(EPD_PANEL_5IN83)
    send_cmd(0x71);
#endif
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
            .clock_speed_hz = 2000000,
            .mode = 0,
            .spics_io_num = PIN_CS,
            .queue_size = 1,
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &spi));
        s_spi_inited = true;
    }
}

/* ArtInk58 semantics for init resets: wait for BUSY release (LOW->HIGH),
 * cap at cap_ms; instant pass if already idle. */
static bool wait_release(int cap_ms)
{
    int ms = 0;
    while (!read_busy()) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ms += 10;
        if (ms > cap_ms) return false;
    }
    return true;
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

#if defined(EPD_PANEL_5IN83)

/* ============ UC8179/GD7965: 5.83" 648x480 FPC-8601B (ArtInk4 port) ==== */

static void hw_reset(void)
{
    gpio_set_level(PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(PIN_RES, 0);
    esp_rom_delay_us(10000);   /* ~10 ms */
    gpio_set_level(PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
}

static bool start_update_and_wait(void)
{
    send_cmd(0x12);               /* master update */
    bool engaged = wait_busy2(15000);
    evlog_record(EV_EPD, EPD_PHASE_UPDATE_ENGAGED, engaged ? 1 : 0);
    if (!engaged) vTaskDelay(pdMS_TO_TICKS(22000));   /* ~20 s refresh */
    return engaged;
}

bool epd_init(void)
{
    ensure_buses();
    hw_reset();

    send_cmd(0x01);               /* power setting */
    send_data(0x07);
    send_data(0x07);
    send_data(0x3F);
    send_data(0x3F);

    send_cmd(0x04);               /* power on */
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!wait_release(10000)) {
        ESP_LOGE(TAG, "PON no busy response — power/panel issue");
        evlog_record(EV_EPD, EPD_PHASE_INIT_FAIL, read_busy());
        return false;
    }

    send_cmd(0x00);               /* panel setting: tri-color OTP */
    send_data(0x0F);
    send_cmd(0x61);               /* resolution 648x480 */
    send_data(0x02);
    send_data(0x88);
    send_data(0x01);
    send_data(0xE0);
    send_cmd(0x15);               /* VGL/VGH option */
    send_data(0x00);
    send_cmd(0x50);               /* VCOM + data interval */
    send_data(0x11);
    send_data(0x07);
    send_cmd(0x60);               /* TCON */
    send_data(0x22);

    ESP_LOGI(TAG, "UC8179 init done");
    evlog_record(EV_EPD, EPD_PHASE_INIT_OK, read_busy());
    return true;
}

void epd_clear(void)
{
    uint8_t *buf = malloc(EPD_BUF_SIZE);
    if (!buf) return;
    memset(buf, 0xFF, EPD_BUF_SIZE);
    send_cmd(0x10);               /* OLD plane: all white */
    send_buf(buf, EPD_BUF_SIZE);
    memset(buf, 0x00, EPD_BUF_SIZE);
    send_cmd(0x13);               /* NEW plane: none */
    send_buf(buf, EPD_BUF_SIZE);
    free(buf);
    start_update_and_wait();
}

void epd_display(const uint8_t *bw, const uint8_t *red)
{
    send_cmd(0x10);
    send_buf(bw, EPD_BUF_SIZE);
    send_cmd(0x13);
    send_buf(red, EPD_BUF_SIZE);
    start_update_and_wait();
}

void epd_sleep(void)
{
    send_cmd(0x02);
    wait_busy2(2000);
    send_cmd(0x07);
    send_data(0xA5);
}

#else

/* ============ SSD1683: 4.2" 400x300 Hink-E042A48-A1 (ArtInk4 port) ===== */

static void hw_reset(void)
{
    gpio_set_level(PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(PIN_RES, 0);
    esp_rom_delay_us(2000);   /* exactly 2 ms — longer kills the adapter LDO */
    gpio_set_level(PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void set_ram_window(void)
{
    send_cmd(0x44);
    send_data(0x00);
    send_data(0x31);      /* 50 bytes per row */
    send_cmd(0x45);
    send_data(0x00);
    send_data(0x00);
    send_data(0x2B);      /* 300 lines */
    send_data(0x01);
    send_cmd(0x4E);
    send_data(0x00);
    send_cmd(0x4F);
    send_data(0x00);
    send_data(0x00);
}

static bool start_update_and_wait(void)
{
    send_cmd(0x22);                   /* OTP full refresh */
    send_data(0xF7);
    send_cmd(0x20);
    bool engaged = wait_busy2(15000);   /* assert may lag by seconds */
    evlog_record(EV_EPD, EPD_PHASE_UPDATE_ENGAGED, engaged ? 1 : 0);
    if (!engaged) vTaskDelay(pdMS_TO_TICKS(16000));   /* ~14 s refresh */
    return engaged;
}

bool epd_init(void)
{
    ensure_buses();

    hw_reset();
    int busy_raw = read_busy();
    ESP_LOGI(TAG, "BUSY raw after hw_reset: %d", busy_raw);
    vTaskDelay(pdMS_TO_TICKS(100));   /* BUSY is unreliable right after reset */

    send_cmd(0x12);                   /* SW reset */
    if (!wait_release(15000)) {
        ESP_LOGE(TAG, "no response to SW reset — check panel VCI/GND, "
                 "then SCK/SDA/CS wiring (BUSY=%d)", read_busy());
        evlog_record(EV_EPD, EPD_PHASE_INIT_FAIL, busy_raw);
        return false;
    }

    send_cmd(0x01);                   /* driver: 300 lines */
    send_data(0x2B);
    send_data(0x01);
    send_data(0x00);
    send_cmd(0x11);                   /* data entry: X inc, Y dec */
    send_data(0x03);
    send_cmd(0x44);
    send_data(0x00);
    send_data(0x31);
    send_cmd(0x45);
    send_data(0x00);
    send_data(0x00);
    send_data(0x2B);
    send_data(0x01);
    send_cmd(0x3C);                   /* border waveform */
    send_data(0x05);
    send_cmd(0x18);                   /* temp sensor: internal */
    send_data(0x80);
    send_cmd(0x4E);
    send_data(0x00);
    send_cmd(0x4F);
    send_data(0x00);
    send_data(0x00);

    bool ok = wait_release(15000);
    if (ok) {
        ESP_LOGI(TAG, "SSD1683 init done");
        evlog_record(EV_EPD, EPD_PHASE_INIT_OK, busy_raw);
    } else {
        evlog_record(EV_EPD, EPD_PHASE_INIT_FAIL, busy_raw);
    }
    return ok;
}

void epd_clear(void)
{
    set_ram_window();
    uint8_t *buf = malloc(EPD_BUF_SIZE);
    if (!buf) return;
    memset(buf, 0xFF, EPD_BUF_SIZE);
    send_cmd(0x24);                   /* BW plane: all white */
    send_buf(buf, EPD_BUF_SIZE);
    memset(buf, 0x00, EPD_BUF_SIZE);
    send_cmd(0x26);                   /* RED plane: none */
    send_buf(buf, EPD_BUF_SIZE);
    free(buf);
    start_update_and_wait();
}

void epd_display(const uint8_t *bw, const uint8_t *red)
{
    set_ram_window();
    send_cmd(0x24);
    send_buf(bw, EPD_BUF_SIZE);
    send_cmd(0x26);
    send_buf(red, EPD_BUF_SIZE);
    start_update_and_wait();
}

void epd_sleep(void)
{
    send_cmd(0x10);
    send_data(0x01);
}

#endif /* panel variant */

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
#if defined(EPD_PANEL_5IN83)
    send_cmd(0x12);                  /* on UC8179 this is the update cmd */
    out->cmd_reaction = wait_busy2(15000);
#else
    send_cmd(0x12);                  /* SSD1683 SW reset */
    out->cmd_reaction = wait_busy2(5000);
#endif
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
