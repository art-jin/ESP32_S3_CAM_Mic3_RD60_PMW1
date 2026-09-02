#include "status.h"
#include <string.h>
#include "servo.h"
#include "wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static device_status_t s_status;
static SemaphoreHandle_t s_mutex;

void status_init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_mutex = xSemaphoreCreateMutex();
}

void status_update(const doa_result_t *r)
{
    if (!s_mutex) return;
    if (xSemaphoreTake(s_mutex, 0) != pdTRUE) return;  /* non-blocking */
    s_status.azimuth     = r->azimuth_deg;
    s_status.sextant     = r->sextant;
    s_status.stable_sect = r->stable_sextant;
    s_status.confidence   = r->confidence;
    s_status.lr_corr      = r->lr_corr;
    s_status.servo_angle  = servo_get_angle_deg();
    s_status.servo_moving = servo_is_moving();
    s_status.mode         = mode_manager_get();
    s_status.wifi_connected = wifi_is_connected();
    const char *ip = wifi_get_ip_string();
    if (ip) strncpy(s_status.ip, ip, sizeof(s_status.ip) - 1);
    const char *hn = wifi_get_hostname();
    if (hn) strncpy(s_status.hostname, hn, sizeof(s_status.hostname) - 1);
    xSemaphoreGive(s_mutex);
}

void status_get(device_status_t *out)
{
    if (!s_mutex || !out) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_mutex);
}
