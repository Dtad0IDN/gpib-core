#include <string.h>

#include "gpib_internal.h"

static bool kind_valid(gpib_handle_t h) {
    return h.kind == (uint8_t)GPIB_HANDLE_TXN ||
           h.kind == (uint8_t)GPIB_HANDLE_BLOCK;
}

gpib_status_t gpib_result_peek(gpib_context_t *ctx, gpib_handle_t handle,
    gpib_result_t *out) {
    if (ctx == NULL || out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->lifecycle == GPIB_LC_INITIALIZED) {
        /* Ledger is empty: no side effects (LIF-03). */
        return kind_valid(handle) ? GPIB_NO_RESULT : GPIB_INVALID_HANDLE;
    }
    if (handle.kind == (uint8_t)GPIB_HANDLE_BLOCK) {
        int s = gpib_ledger_block_slot(ctx, handle);
        if (s < 0) {
            return GPIB_INVALID_HANDLE;
        }
        if (ctx->block[s].state != BLOCK_RESULT_READY) {
            return GPIB_NO_RESULT;
        }
        *out = ctx->block[s].result;
        return GPIB_OK;
    }
    if (handle.kind == (uint8_t)GPIB_HANDLE_TXN) {
        int s = gpib_ledger_txn_slot(ctx, handle);
        if (s < 0) {
            return GPIB_INVALID_HANDLE;
        }
        if (ctx->txn[s].state != TXN_RESULT_READY) {
            return GPIB_NO_RESULT;
        }
        *out = ctx->txn[s].result;
        return GPIB_OK;
    }
    return GPIB_INVALID_HANDLE;
}

gpib_status_t gpib_result_view(gpib_context_t *ctx, gpib_handle_t block,
    const uint8_t **data, size_t *len) {
    if (ctx == NULL || data == NULL || len == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->lifecycle == GPIB_LC_INITIALIZED) {
        return GPIB_NO_RESULT;
    }
    if (block.kind != (uint8_t)GPIB_HANDLE_BLOCK) {
        return GPIB_INVALID_HANDLE;
    }
    int s = gpib_ledger_block_slot(ctx, block);
    if (s < 0) {
        /* Includes the post-ACK case: the generation bump invalidated the
         * handle, so the borrowed view is contractually expired (FIX-03)
         * while the application's own pointer remains valid. */
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->block[s].state != BLOCK_RESULT_READY) {
        return GPIB_NO_RESULT;
    }
    if (!ctx->block[s].is_rx) {
        return GPIB_UNSUPPORTED; /* TX blocks expose no view */
    }
    *data = ctx->block[s].data;
    *len = ctx->block[s].result.bytes_confirmed; /* confirmed prefix only */
    return GPIB_OK;
}

gpib_status_t gpib_result_ack(gpib_context_t *ctx, gpib_handle_t handle) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->lifecycle == GPIB_LC_INITIALIZED) {
        return kind_valid(handle) ? GPIB_NO_RESULT : GPIB_INVALID_HANDLE;
    }
    if (handle.kind == (uint8_t)GPIB_HANDLE_BLOCK) {
        int s = gpib_ledger_block_slot(ctx, handle);
        if (s < 0) {
            return GPIB_INVALID_HANDLE; /* stale/double/wrong-kind */
        }
        if (ctx->block[s].state != BLOCK_RESULT_READY) {
            return GPIB_NO_RESULT;
        }
        /* Ownership returns to the application here and only here
         * (REQ-032/R9); the generation bump invalidates this handle.
         * R17-02 (anti-ABA): a generation must never wrap back into a
         * value it already held -- a full uint32 turn made a STALE
         * handle re-validate against a NEW block in the same slot. At
         * exhaustion the slot is RETIRED from circulation for the rest
         * of the instance (BLOCK_APP_OWNED is not findable as free
         * only if we mark it; use the quarantine flag semantics: the
         * simplest permanent-retire marker is a state the free-finder
         * skips). */
        struct gpib_block_record *b = &ctx->block[s];
        if (b->generation == UINT32_MAX) {
            /* Retire: this slot will never be handed out again. */
            b->generation = 0;
            b->state = BLOCK_WAIT_QUIESCENCE; /* not BLOCK_APP_OWNED:
                                               * find_free skips it */
            b->retired = true;
            memset(&b->result, 0, sizeof(b->result));
            b->txn_slot = 0;
            b->txn_generation = 0;
            GPIB_TRACE(ctx, GPIB_TRC_ACK_BLOCK, (uint16_t)s, 0, 1u);
            return GPIB_OK;
        }
        b->generation++;
        b->state = BLOCK_APP_OWNED;
        memset(&b->result, 0, sizeof(b->result));
        b->txn_slot = 0;
        b->txn_generation = 0;
        GPIB_TRACE(ctx, GPIB_TRC_ACK_BLOCK, (uint16_t)s, b->generation, 0);
        return GPIB_OK;
    }
    if (handle.kind == (uint8_t)GPIB_HANDLE_TXN) {
        int s = gpib_ledger_txn_slot(ctx, handle);
        if (s < 0) {
            return GPIB_INVALID_HANDLE;
        }
        if (ctx->txn[s].state != TXN_RESULT_READY) {
            return GPIB_NO_RESULT;
        }
        /* Releases ONLY the transaction slot; block records, block handles
         * and block memory are deliberately untouched (FIX-01/REQ-070). */
        struct gpib_txn_record *t = &ctx->txn[s];
        if (t->generation == UINT32_MAX) {
            /* R17-02: retire the slot instead of wrapping -- a wrapped
             * generation re-validates stale transaction handles. */
            t->generation = 0;
            t->state = TXN_RECORD_RELEASED;
            t->retired = true; /* gpib_ledger_find_free skips retired */
            memset(&t->address, 0, sizeof(t->address));
            t->kind = GPIB_REQ_WRITE;
            t->block_count = 0;
            t->blocks_terminal = 0;
            memset(&t->result, 0, sizeof(t->result));
            GPIB_TRACE(ctx, GPIB_TRC_ACK_TXN, (uint16_t)s, 0, 1u);
            return GPIB_OK;
        }
        t->generation++;
        t->state = TXN_RECORD_RELEASED;
        memset(&t->address, 0, sizeof(t->address));
        t->kind = GPIB_REQ_WRITE;
        t->block_count = 0;
        t->blocks_terminal = 0;
        memset(&t->result, 0, sizeof(t->result));
        GPIB_TRACE(ctx, GPIB_TRC_ACK_TXN, (uint16_t)s, t->generation, 0);
        return GPIB_OK;
    }
    return GPIB_INVALID_HANDLE;
}
