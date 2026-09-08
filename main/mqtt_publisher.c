#include "mqtt_publisher.h"

#include <string.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "esp_crt_bundle.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mqtt_creds.h"
#include "evlog.h"

static const char *TAG = "mqtt";

/* ---- Credentials — single access point (§6.5 fleet-migration note) ----
 * Everything above this function is forbidden from touching mqtt_creds.h;
 * when fleet provisioning moves to an NVS "fleet" namespace, only
 * get_creds() changes. */
typedef struct {
    const char *uri;
    const char *username;
    const char *password;
    const char *robot_prefix;   /* e.g. "arthur139/redwolf" */
} neck_creds_t;

static const neck_creds_t *get_creds(void)
{
    static const neck_creds_t c = {
        .uri          = MQTT_BROKER_URI,
        .username     = MQTT_USERNAME,
        .password     = MQTT_PASSWORD,
        .robot_prefix = MQTT_ROBOT_PREFIX,
    };
    return &c;
}

/* Event queue depth: events are edge-triggered (a few per minute at most);
 * 16 slots only need to absorb short broker outages. Overflow ⇒ drop. */
#define EVT_QUEUE_LEN 16

typedef struct {
    mqtt_event_type_t type;
    uint32_t seq;
    uint32_t sec;
    uint32_t usec;
    uint32_t range_mm;
    int16_t  az_deg;
    uint8_t  conf;
} neck_evt_t;

static const char *evt_type_str(mqtt_event_type_t t)
{
    switch (t) {
    case MQTT_EVT_ZONE_ENTER:   return "zone_enter";
    case MQTT_EVT_ZONE_LEAVE:   return "zone_leave";
    case MQTT_EVT_RADAR_ONLINE: return "radar_online";
    case MQTT_EVT_RADAR_OFFLINE:return "radar_offline";
    default:                    return "?";
    }
}

/* After this epoch we consider SNTP synced (fail-safe: unsynced ⇒ sec=0). */
#define TIME_SYNC_MIN_SEC 1577836800LL   /* 2020-01-01 */

static QueueHandle_t s_evtq;
static esp_mqtt_client_handle_t s_client;
static volatile bool s_connected;
static uint32_t s_seq;
static uint32_t s_dropped;
static char s_topic_events[64];
static char s_topic_online[64];
static char s_client_id[24];
static bool s_started;

extern char g_device_id[8];   /* main.c — 6-char NVS identity */

static void on_ip_got(void *arg, esp_event_base_t base, int32_t id, void *data);

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    esp_mqtt_event_t *e = (esp_mqtt_event_t *)data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        esp_mqtt_client_publish(s_client, s_topic_online, "1", 1, 1, 1);
        ESP_LOGI(TAG, "connected; online=1 published; heap free=%u min=%u",
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)esp_get_minimum_free_heap_size());
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "disconnected");
        break;
    case MQTT_EVENT_ERROR:
        if (e->error_handle->error_type == MQTT_ERROR_TYPE_ESP_TLS) {
            ESP_LOGE(TAG, "TLS error (0x%x)", e->error_handle->esp_tls_last_esp_err);
        } else if (e->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGE(TAG, "connection refused (auth?)");
        } else {
            ESP_LOGE(TAG, "mqtt error type=%d", e->error_handle->error_type);
        }
        break;
    default:
        break;
    }
}

static void publisher_task(void *arg)
{
    neck_evt_t evt;
    char payload[192];
    char topic[80];
    while (true) {
        if (xQueueReceive(s_evtq, &evt, portMAX_DELAY) != pdTRUE) continue;

        if (!s_connected) {
            /* Event semantics (D7 spirit): a stale zone_enter replayed
             * after reconnect is worse than a dropped one. */
            s_dropped++;
            ESP_LOGW(TAG, "drop %s seq=%u (not connected), dropped=%u",
                     evt_type_str(evt.type), evt.seq, s_dropped);
            continue;
        }

        int n = snprintf(payload, sizeof(payload),
            "{\"boots\":%lu,\"seq\":%lu,\"sec\":%lu,\"usec\":%lu,"
            "\"type\":\"%s\",\"range_mm\":%lu,\"az_deg\":%d,\"conf\":%u}",
            (unsigned long)evlog_get_boot_count(),
            (unsigned long)evt.seq,
            (unsigned long)evt.sec, (unsigned long)evt.usec,
            evt_type_str(evt.type),
            (unsigned long)evt.range_mm, evt.az_deg, evt.conf);

        snprintf(topic, sizeof(topic), "%s", s_topic_events);
        int msg_id = esp_mqtt_client_publish(s_client, topic, payload, n, 1, 0);
        if (msg_id < 0) {
            s_dropped++;
            ESP_LOGW(TAG, "publish %s seq=%u failed (%d), dropped=%u",
                     evt_type_str(evt.type), evt.seq, msg_id, s_dropped);
        } else {
            ESP_LOGI(TAG, "→ %s %s", evt_type_str(evt.type), payload);
        }
    }
}

static void on_ip_got(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)id; (void)data;
    if (s_started) return;
    s_started = true;

    /* SNTP for epoch timestamps in the event payload (unsynced ⇒ sec=0). */
    esp_sntp_config_t sntp_cfg =
        ESP_NETIF_SNTP_DEFAULT_CONFIG("ntp.aliyun.com");
    esp_netif_sntp_init(&sntp_cfg);   /* .start=true: service starts now */

    esp_mqtt_client_start(s_client);
    ESP_LOGI(TAG, "start: %s user=%s prefix=%s heap free=%u",
             get_creds()->uri, get_creds()->username, get_creds()->robot_prefix,
             (unsigned)esp_get_free_heap_size());
}

void mqtt_publisher_init(void)
{
    const neck_creds_t *c = get_creds();

    snprintf(s_topic_events, sizeof(s_topic_events), "%s/neck/events", c->robot_prefix);
    snprintf(s_topic_online, sizeof(s_topic_online), "%s/neck/state/online", c->robot_prefix);
    snprintf(s_client_id, sizeof(s_client_id), "neck-%s",
             g_device_id[0] ? g_device_id : "esp32");

    s_evtq = xQueueCreate(EVT_QUEUE_LEN, sizeof(neck_evt_t));

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = c->uri,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.username = c->username,
        .credentials.client_id = s_client_id,
        .credentials.authentication.password = c->password,
        .session.last_will = {
            .topic = s_topic_online,
            .msg = "0",
            .msg_len = 1,
            .qos = 1,
            .retain = true,
        },
        .session.keepalive = 60,
        .network.reconnect_timeout_ms = 5000,
    };
    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "esp_mqtt_client_init failed — MQTT disabled");
        return;
    }
    esp_mqtt_client_register_event(s_client, MQTT_EVENT_ANY,
                                   mqtt_event_handler, NULL);

    /* Connect only once WiFi has an IP (esp_mqtt auto-reconnects after). */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_ip_got, NULL, NULL));

    xTaskCreate(publisher_task, "mqtt_pub", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "init: events→%s online→%s (connect deferred until IP)",
             s_topic_events, s_topic_online);
}

bool mqtt_publish_event(mqtt_event_type_t type, uint32_t range_mm,
                        int16_t az_deg, uint8_t conf)
{
    if (!s_evtq) return false;

    neck_evt_t evt = {
        .type = type,
        .seq = ++s_seq,
        .range_mm = range_mm,
        .az_deg = az_deg,
        .conf = conf,
    };
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec >= TIME_SYNC_MIN_SEC) {
        evt.sec = (uint32_t)tv.tv_sec;
        evt.usec = (uint32_t)tv.tv_usec;
    }
    /* else stays 0/0 — consumer treats unsynced timestamps as absent. */

    if (xQueueSend(s_evtq, &evt, 0) != pdTRUE) {
        s_dropped++;
        ESP_LOGW(TAG, "queue full — %s dropped (%u total)",
                 evt_type_str(type), s_dropped);
        return false;
    }
    return true;
}

bool mqtt_publisher_is_connected(void) { return s_connected; }
uint32_t mqtt_publisher_seq(void) { return s_seq; }
uint32_t mqtt_publisher_dropped(void) { return s_dropped; }
