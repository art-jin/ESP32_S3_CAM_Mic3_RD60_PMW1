#include "mic_capture.h"

#include "driver/gpio.h"
#include "driver/i2s_pdm.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "soc/gpio_sig_map.h"

static const char *TAG = "mic";

static i2s_chan_handle_t s_rx;

esp_err_t mic_capture_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel: %s", esp_err_to_name(err));
        return err;
    }

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_PDM_RX_SLOT_PCM_FMT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                            I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .clk = MIC_CLK0_GPIO,
            .invert_flags = { .clk_inv = false },
        },
    };
    pdm_cfg.gpio_cfg.dins[0] = MIC_DAT0_GPIO;
    pdm_cfg.gpio_cfg.dins[1] = MIC_DAT1_GPIO;
    /* Activate both L and R slots on DIN[0] (intended to give M3+M1 split
     * by PDM clock phase) plus L slot on DIN[1] (M2). If the on-chip
     * PDM2PCM collapses L and R, doa_process detects it at runtime and
     * falls back to 2-mic mode using only M3 (c0) and M2 (c2). */
    pdm_cfg.slot_cfg.slot_mask = I2S_PDM_RX_LINE0_SLOT_LEFT
                               | I2S_PDM_RX_LINE0_SLOT_RIGHT
                               | I2S_PDM_RX_LINE1_SLOT_LEFT;

    err = i2s_channel_init_pdm_rx_mode(s_rx, &pdm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_pdm_rx_mode: %s", esp_err_to_name(err));
        return err;
    }

    /* Fan the I²S0 RX WS (bit-clock / PDM-clock for RX) out to CLK1 so
     * both 3DMIC clock inputs share the same hardware edge. Phase drift
     * between CLK0 and CLK1 would corrupt the M3↔M1 TDOA. */
    gpio_set_direction(MIC_CLK1_GPIO, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(MIC_CLK1_GPIO, I2S0I_WS_OUT_IDX, false, false);

    err = i2s_channel_enable(s_rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "PDM RX up: %d Hz %d ch | CLK0=%d CLK1=%d(fanout) DAT0=%d DAT1=%d",
             MIC_SAMPLE_RATE_HZ, MIC_NUM_CHANNELS,
             MIC_CLK0_GPIO, MIC_CLK1_GPIO, MIC_DAT0_GPIO, MIC_DAT1_GPIO);
    return ESP_OK;
}

esp_err_t mic_capture_read(int16_t *buf, size_t buf_bytes, size_t *got_bytes)
{
    return i2s_channel_read(s_rx, buf, buf_bytes, got_bytes, portMAX_DELAY);
}
