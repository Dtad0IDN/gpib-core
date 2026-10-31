#include <string.h>

#include "gpib_internal.h"

#if GPIB_CFG_TRACE_EVENTS > 0

void gpib_trace_write(struct gpib_context *ctx, uint16_t kind, uint16_t a,
    uint16_t b, uint32_t c) {
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return;
    }
    gpib_trace_event_t *slot = &ctx->trace[ctx->trace_head];
    slot->ts_us = ctx->port->now_us(ctx->port->user);
    slot->kind = kind;
    slot->a = a;
    slot->b = b;
    slot->c = c;
    ctx->trace_head = (ctx->trace_head + 1u) % GPIB_CFG_TRACE_EVENTS;
    if (ctx->trace_count < GPIB_CFG_TRACE_EVENTS) {
        ctx->trace_count++;
    } else {
        /* Full ring: the oldest record is dropped and counted, never
         * coalesced (REQ-058 wrap policy [P]). */
        ctx->trace_dropped++;
    }
}

gpib_status_t gpib_trace_query(gpib_context_t *ctx, gpib_trace_info_t *out) {
    if (ctx == NULL || out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    out->count = ctx->trace_count;
    out->dropped = ctx->trace_dropped;
    return GPIB_OK;
}

gpib_status_t gpib_trace_dump(gpib_context_t *ctx, gpib_trace_event_t *events,
    size_t cap, size_t *n) {
    if (ctx == NULL || events == NULL || n == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    size_t want = (cap < ctx->trace_count) ? cap : ctx->trace_count;
    /* Oldest valid record position; non-consuming copy. */
    size_t oldest = (ctx->trace_head + GPIB_CFG_TRACE_EVENTS - ctx->trace_count) %
                    GPIB_CFG_TRACE_EVENTS;
    for (size_t i = 0; i < want; i++) {
        events[i] = ctx->trace[(oldest + i) % GPIB_CFG_TRACE_EVENTS];
    }
    *n = want;
    return GPIB_OK;
}

#else /* tracing disabled at compile time */

gpib_status_t gpib_trace_query(gpib_context_t *ctx, gpib_trace_info_t *out) {
    if (ctx == NULL || out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    out->count = 0;
    out->dropped = 0;
    return GPIB_OK;
}

gpib_status_t gpib_trace_dump(gpib_context_t *ctx, gpib_trace_event_t *events,
    size_t cap, size_t *n) {
    if (ctx == NULL || events == NULL || n == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    *n = 0;
    return GPIB_OK;
}

#endif
