#include "wifi.h"
#include "rest_api.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
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
static int s_cur_slot = 0;       /* slot order = priority, boot starts at 0 */
static int s_slot_retry = 0;     /* disconnects on current slot */
static TimerHandle_t s_switch_timer = NULL;  /* one-shot delayed reconnect */
static bool s_http_started = false;

/* ---- Dual-slot network table ----
 * Order = priority: boot always starts at slot 0 (居家 / static robot
 * identity); the failover state machine rotates to the next slot only
 * after the current one exhausts its retry budget. Hotspot slots run
 * DHCP — their subnet varies and is never used as a fixed address.
 * Design + tuning: tasks/wifi-dual-network.md */
typedef struct {
    const char *ssid;
    const char *password;
    bool use_static_ip;
} wifi_net_slot_t;

#define WIFI_NET_COUNT 2

static const wifi_net_slot_t s_wifi_nets[WIFI_NET_COUNT] = {
    { WIFI0_SSID, WIFI0_PASSWORD, WIFI0_USE_STATIC },
    { WIFI1_SSID, WIFI1_PASSWORD, WIFI1_USE_STATIC },
};

/* Forward declarations for rest_api (avoid circular include). */
extern esp_err_t rest_api_start(void);

/* Static IP — when USE_STATIC_IP is set in board_config.h AND the current
 * slot opts in (WIFIx_USE_STATIC), pin the STA interface to a fixed IPv4
 * instead of using DHCP. Used for the home slot so external callers can
 * keep using the robot's address (heizai=.113) without router DHCP
 * reservation. */
#if defined(USE_STATIC_IP) && USE_STATIC_IP
static void apply_static_ip(void)
{
    /* DHCP client must be stopped before set_ip_info takes effect.
     * ALREADY_STOPPED is expected (boot path stops it before it ran). */
    esp_err_t err = esp_netif_dhcpc_stop(s_sta_netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGW(TAG, "dhcpc_stop: %s", esp_err_to_name(err));
    }

    esp_netif_ip_info_t ip = {0};
    ip4addr_aton(STATIC_IP_ADDR,    (ip4_addr_t *)&ip.ip);
    ip4addr_aton(STATIC_IP_GW,      (ip4_addr_t *)&ip.gw);
    ip4addr_aton(STATIC_IP_NETMASK, (ip4_addr_t *)&ip.netmask);
    esp_netif_set_ip_info(s_sta_netif, &ip);

    esp_netif_dns_info_t dns = {0};
    ip4addr_aton(STATIC_IP_DNS, (ip4_addr_t *)&dns.ip.u_addr.ip4);
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN, &dns);

    ESP_LOGI(TAG, "Static IP applied: %s / %s / gw %s / dns %s",
             STATIC_IP_ADDR, STATIC_IP_NETMASK, STATIC_IP_GW, STATIC_IP_DNS);
}
#else
static void apply_static_ip(void) { }
#endif

/* Per-slot IP mode — called in the disconnected window only (boot
 * pre-start, or mid-rotation), which is the safe time to reconfigure
 * the netif. */
static void apply_ip_mode(int slot)
{
    bool want_static = false;
#if defined(USE_STATIC_IP) && USE_STATIC_IP
    want_static = s_wifi_nets[slot].use_static_ip;
#endif
    if (want_static) {
        apply_static_ip();
    } else {
        esp_err_t err = esp_netif_dhcpc_start(s_sta_netif);
        /* ALREADY_STARTED is normal when re-entering a DHCP slot. */
        if (err == ESP_OK || err == ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
            ESP_LOGI(TAG, "Slot %d \"%s\": DHCP client started",
                     slot, s_wifi_nets[slot].ssid);
        } else {
            ESP_LOGW(TAG, "dhcpc_start: %s", esp_err_to_name(err));
        }
    }
}

static void set_sta_config(int slot)
{
    wifi_config_t cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)cfg.sta.ssid, s_wifi_nets[slot].ssid, sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, s_wifi_nets[slot].password, sizeof(cfg.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
}

/* Short decode for the disconnect reasons seen on the bench — separates
 * "AP invisible" (band wrong / out of range) from "auth failure" (wrong
 * password). Full table: esp_wifi_types.h / IEEE 802.11. */
static const char *reason_str(uint8_t reason)
{
    switch (reason) {
    case 2:   return "AUTH_EXPIRE";
    case 15:  return "4WAY_HANDSHAKE_TIMEOUT(密码错?)";
    case 200: return "BEACON_TIMEOUT";
    case 201: return "NO_AP_FOUND(网络不可见/频段错?)";
    case 202: return "AUTH_FAIL(密码错?)";
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    default:  return "";
    }
}

/* Delayed reconnect after a slot rotation — one-shot software timer, so
 * the event-loop task is never blocked (the pre-rewrite code slept 10s
 * inside the event handler, stalling all other events). */
static void switch_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    ESP_LOGI(TAG, "Connecting slot %d \"%s\"",
             s_cur_slot, s_wifi_nets[s_cur_slot].ssid);
    esp_wifi_connect();
}

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

/* ---- mDNS ----
 * docs/52 P0-6: anban-neck-<robot>.local, robot taken from the MQTT
 * prefix's last segment (firmware identity, mqtt_creds.h). Falls back
 * to the old mac-suffix name if the prefix is missing. */
#include "mqtt_creds.h"

static void mdns_start(void)
{
    mdns_init();
    const char *robot = NULL;
    const char *p = MQTT_ROBOT_PREFIX;
    const char *slash = strrchr(p, '/');
    robot = slash ? slash + 1 : p;
    if (robot[0] == '\0') {
        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        snprintf(s_hostname, sizeof(s_hostname), "esp32-mic-%02x%02x", mac[4], mac[5]);
    } else {
        snprintf(s_hostname, sizeof(s_hostname), "anban-neck-%s", robot);
    }
    mdns_hostname_set(s_hostname);
    mdns_instance_name_set("Anban neck (radar presence)");
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
        wifi_event_sta_disconnected_t *evt = (wifi_event_sta_disconnected_t *)data;
        xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);
        led_set(0);  /* LED off = disconnected */

        s_slot_retry++;
        ESP_LOGW(TAG, "WiFi disconnected from \"%s\" reason=%d %s, retry %d/%d",
                 s_wifi_nets[s_cur_slot].ssid, evt->reason,
                 reason_str(evt->reason), s_slot_retry, WIFI_SLOT_RETRY_MAX);

        if (s_slot_retry < WIFI_SLOT_RETRY_MAX) {
            /* Same slot, immediate — rides out transient drops
             * (router reboot blips) without rotating away. */
            esp_wifi_connect();
        } else {
            /* Budget exhausted — rotate to the next slot. Netif + STA
             * config are re-applied here in the disconnected window;
             * the actual connect fires later from the one-shot timer. */
            s_cur_slot = (s_cur_slot + 1) % WIFI_NET_COUNT;
            s_slot_retry = 0;
            apply_ip_mode(s_cur_slot);
            set_sta_config(s_cur_slot);
            ESP_LOGI(TAG, "Rotating to WiFi slot %d \"%s\" (connect in %d ms)",
                     s_cur_slot, s_wifi_nets[s_cur_slot].ssid,
                     WIFI_SLOT_SWITCH_DELAY_MS);
            xTimerStart(s_switch_timer, 0);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        s_slot_retry = 0;
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
    s_switch_timer = xTimerCreate("wifi_slot",
                                  pdMS_TO_TICKS(WIFI_SLOT_SWITCH_DELAY_MS),
                                  pdFALSE /* one-shot */, NULL, switch_timer_cb);

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

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    /* Boot always starts at slot 0 (priority order). Its IP mode is
     * applied pre-start so the DHCP client doesn't grab a router
     * address first on static slots. */
    apply_ip_mode(0);
    set_sta_config(0);

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA init: %d slots, boot from \"%s\"",
             WIFI_NET_COUNT, s_wifi_nets[0].ssid);
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
