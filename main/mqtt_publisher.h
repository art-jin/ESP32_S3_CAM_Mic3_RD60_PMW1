/* mqtt_publisher.h — neck-event MQTT publisher for docs/48 Tier 1.
 *
 * Publishes zone/radar-lifecycle events as non-retained QoS1 JSON to
 * <prefix>/neck/events, plus a retained <prefix>/neck/state/online
 * heartbeat with an LWT of "0" (§5.1).
 *
 * Non-blocking rule (§6.1): producers (radar task) only enqueue to an
 * internal FreeRTOS queue; the dedicated publisher task does the actual
 * esp_mqtt calls. Queue full / broker down ⇒ events are dropped and
 * counted — never block the 5 Hz sensing loop, servo or REST. */
#ifndef MQTT_PUBLISHER_H
#define MQTT_PUBLISHER_H

#include <stdbool.h>
#include <stdint.h>

/* v1 event types (§5.3 — only add, never rename; consumers must ignore
 * unknown types). String forms live in mqtt_publisher.c. */
typedef enum {
    MQTT_EVT_ZONE_ENTER = 0,
    MQTT_EVT_ZONE_LEAVE,
    MQTT_EVT_RADAR_ONLINE,
    MQTT_EVT_RADAR_OFFLINE,
} mqtt_event_type_t;

/* One-time setup: queue + publisher task + IP-event hook. Connects and
 * starts SNTP once WiFi has an IP. Call from app_main after wifi_init(). */
void mqtt_publisher_init(void);

/* Enqueue an event (timestamped now). Never blocks; returns false when
 * the queue is full (event dropped, counted in mqtt_publisher_dropped). */
bool mqtt_publish_event(mqtt_event_type_t type, uint32_t range_mm,
                        int16_t az_deg, uint8_t conf);

bool mqtt_publisher_is_connected(void);

/* Last assigned sequence number (dedup key low half, §5.2). */
uint32_t mqtt_publisher_seq(void);

/* Events dropped (queue full / not connected) since boot. */
uint32_t mqtt_publisher_dropped(void);

#endif /* MQTT_PUBLISHER_H */
