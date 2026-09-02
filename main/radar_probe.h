#pragma once

/* Phase 0 radar protocol probe. When RADAR_PROBE_MODE is 1, app_main skips
 * the normal DOA/tracker/REST startup and only runs the UART1 sniffer below,
 * so the console log contains nothing but radar traffic. Set to 0 (and
 * rebuild) to restore normal firmware behaviour. */
#define RADAR_PROBE_MODE 0

void radar_probe_start(void);
