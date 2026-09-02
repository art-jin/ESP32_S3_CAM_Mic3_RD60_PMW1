#pragma once

#include <stdint.h>

/* Application-level scene events (PRD US-008): a 32-slot RAM ring,
 * monotonic seq, readable via GET /api/events?since=<seq> for incremental
 * polling. Low-frequency/high-value events are mirrored into the NVS
 * evlog (survives reboot); high-frequency ones (sound assoc transitions)
 * stay in RAM only so they cannot drown evlog's crash-forensics slots. */

typedef enum {
    AEVT_TARGET_ENTER = 0,   /* v1=az_deg,  v2=range_cm */
    AEVT_TARGET_LEAVE,       /* v1=az_deg,  v2=0 */
    AEVT_SOUND_ASSOC,        /* v1=doa_az,  v2=range_cm */
    AEVT_SOUND_UNASSOC,      /* v1=doa_az,  v2=diff_deg */
    AEVT_OOR,                /* v1=target_deg, v2=policy id */
    AEVT_RADAR_OFFLINE,      /* v1=0, v2=0 */
    AEVT_RADAR_ONLINE,       /* v1=0, v2=0 */
    AEVT_STILL_ALARM,        /* v1=range_cm, v2=az_deg (US-009) */
    AEVT_STILL_RECOVER,      /* v1=az_deg, v2=0 */
    AEVT_TYPE_COUNT
} app_event_type_t;

typedef struct {
    uint32_t seq;
    uint32_t uptime_ms;
    uint8_t  type;           /* app_event_type_t */
    int16_t  v1;
    int16_t  v2;
} app_event_t;

void events_init(void);

/* Push one event. Safe from any task. */
void events_push(uint8_t type, int16_t v1, int16_t v2);

/* Copy events with seq > since into out[] (up to max). Returns the count;
 * *next is set to the newest seq present (0 when the ring is empty). */
int events_get(app_event_t *out, int max, uint32_t since, uint32_t *next);

const char *events_type_name(uint8_t type);
