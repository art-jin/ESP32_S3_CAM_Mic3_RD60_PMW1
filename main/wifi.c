#include "wifi.h"
#include "rest_api.h"

#include <string.h>
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "mdns.h"
#include "wifi_creds.h"
#include "evlog.h"
#include "board_config.h"

/* LED indicator for WiFi status. GPIO defined in board_config.h
 * (typically GPIO 48 on both supported boards; on S3-Zero this is the
 * on-board WS2812, which may not light up correctly under simple
 * on/off GPIO drive — that's cosmetic, doesn't affect WiFi logic). */
#define WIFI_CONNECTED_BIT  BIT0

static const char *TAG = "wifi";
static EventGroupHandle_t s_wifi_eg;
static esp_netif_t *s_sta_netif = NULL;
static char s_ip_str[16] = {0};
static char s_hostname[32] = {0};
static int s_retry_count = 0;
static bool s_http_started = false;

/* Forward declarations for rest_api (avoid circular include). */
extern esp_err_t rest_api_start(void);

/* Static IP — when USE_STATIC_IP is set in board_config.h, pin the STA
 * interface to a fixed IPv4 instead of using DHCP. Used for the
 * "红太狼" replacement board so external callers can keep using
 * 192.168.1.110 across physical-board swaps without router DHCP
 * reservation. No-op (empty inline) when disabled — other boards
 * keep default DHCP behaviour. */
#if defined(USE_STATIC_IP) && USE_STATIC_IP
static void static_ip_apply(esp_netif_t *netif)
{
    /* DHCP client must be stopped before set_ip_info takes effect. */
    esp_netif_dhcpc_stop(netif);  /* ok if this errors — pre-start */

    esp_netif_ip_info_t ip = {0};
    ip4addr_aton(STATIC_IP_ADDR,    (ip4_addr_t *)&ip.ip);
    ip4addr_aton(STATIC_IP_GW,      (ip4_addr_t *)&ip.gw);
    ip4addr_aton(STATIC_IP_NETMASK, (ip4_addr_t *)&ip.netmask);
    esp_netif_set_ip_info(netif, &ip);

    esp_netif_dns_info_t dns = {0};
    ip4addr_aton(STATIC_IP_DNS, (ip4_addr_t *)&dns.ip.u_addr.ip4);
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);

    ESP_LOGI(TAG, "Static IP applied: %s / %s / gw %s / dns %s",
             STATIC_IP_ADDR, STATIC_IP_NETMASK, STATIC_IP_GW, STATIC_IP_DNS);
}
#else
static void static_ip_apply(esp_netif_t *netif) { (void)netif; }
#endif

/* ---- LED indicator ---- */
static void led_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(LED_GPIO, 0);  /* off = connecting */
}

static void led_set(bool on)
{
    gpio_set_level(LED_GPIO, on ? 1 : 0);
}

/* Slow blink via software timer while connecting. */
static void led_blink_task(void *arg)
{
    while (!wifi_is_connected()) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

/* ---- mDNS ---- */
static void mdns_start(void)
{
    mdns_init();
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(s_hostname, sizeof(s_hostname), "esp32-mic-%02x%02x", mac[4], mac[5]);
    mdns_hostname_set(s_hostname);
    mdns_instance_name_set("ESP32 Mic Array DOA");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS: http://%s.local", s_hostname);
}

/* ---- Event handler ---- */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_retry_count++;
        xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);
        led_set(0);  /* LED off = disconnected */

        if (s_retry_count <= 5) {
            ESP_LOGW(TAG, "WiFi disconnected, retry %d/5", s_retry_count);
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "WiFi max retries reached, retrying in 10s");
            vTaskDelay(pdMS_TO_TICKS(10000));
            s_retry_count = 0;
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
        led_set(1);  /* LED steady on = connected */
        ESP_LOGI(TAG, "WiFi connected! IP: %s", s_ip_str);
        evlog_record(EV_WIFI_UP, 0, 0);

        /* Start mDNS + HTTP server (once). */
        mdns_start();
        if (!s_http_started) {
            rest_api_start();
            s_http_started = true;
        }
    }
}

esp_err_t wifi_init(void)
{
    s_wifi_eg = xEventGroupCreate();

    led_init();
    /* Blink LED while connecting. */
    xTaskCreate(led_blink_task, "led_blink", 1024, NULL, 1, NULL);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    /* Apply static IP before wifi_start() so DHCP client doesn't grab a
     * router-assigned address first. */
    static_ip_apply(s_sta_netif);

    wifi_config_t wifi_cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_cfg.sta.ssid, WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASSWORD, sizeof(wifi_cfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA init: SSID=\"%s\"", WIFI_SSID);
    return ESP_OK;
}

bool wifi_is_connected(void)
{
    return (xEventGroupGetBits(s_wifi_eg) & WIFI_CONNECTED_BIT) != 0;
}

const char *wifi_get_ip_string(void)
{
    return s_ip_str[0] ? s_ip_str : NULL;
}

const char *wifi_get_hostname(void)
{
    return s_hostname[0] ? s_hostname : NULL;
}
