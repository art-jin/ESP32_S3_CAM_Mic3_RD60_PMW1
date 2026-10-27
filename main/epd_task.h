#ifndef EPD_TASK_H
#define EPD_TASK_H

#include <stdbool.h>
#include <stdint.h>

/* Result snapshot of the last selftest run — readable via REST at any
 * time (independent of serial console and the small evlog ring). */
typedef struct {
    uint32_t run_id;        /* increments per run */
    bool     running;
    bool     init_ok;
    bool     clear_engaged;
    int      clear_ms;
    bool     pat_engaged;
    int      pat_ms;
    int32_t  started_uptime_ms;
} epd_result_t;

/* Starts the e-ink task (no-op when ENABLE_EPD=0). Boot bring-up:
 * init -> clear -> tri-color test pattern, all inside its own low-priority
 * task so the refreshes never block radar/mic/WiFi. */
void epd_task_start(void);

/* Re-run init+clear+test pattern on the parked task (bench bring-up;
 * called from REST /api/epd/test). */
void epd_task_test_trigger(void);

/* Snapshot of the latest/ongoing run (never NULL). */
const epd_result_t *epd_last_result(void);

#endif
