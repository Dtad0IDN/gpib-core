#include <stdint.h>

#include "gpib_internal.h"

/* Instance identity manager (R14/REQ-072): the counter lives outside the
 * reinitialized context memory, is unique within one continuous program run,
 * is never reused, and exhaustion refuses new initialization instead of
 * wrapping (anti-ABA; stale handles must not come back to life). */
static uint64_t s_epoch_counter;

gpib_status_t gpib_epoch_next(uint64_t *out) {
    if (s_epoch_counter == UINT64_MAX) {
        return GPIB_RESOURCE_EXHAUSTED;
    }
    s_epoch_counter++;
    *out = s_epoch_counter;
    return GPIB_OK;
}
