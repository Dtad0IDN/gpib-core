#include <stddef.h>

#include "gpib_internal.h"

int gpib_ledger_find_free_txn(const struct gpib_context *ctx) {
    for (int i = 0; i < GPIB_CFG_MAX_OPERATIONS; i++) {
        gpib_txn_state_t st = ctx->txn[i].state;
        if (ctx->txn[i].retired) {
            continue; /* R17-02: exhausted generation, never reused */
        }
        if (st == TXN_UNRESERVED || st == TXN_RECORD_RELEASED) {
            return i;
        }
    }
    return -1;
}

int gpib_ledger_find_free_block(const struct gpib_context *ctx) {
    for (int i = 0; i < GPIB_CFG_MAX_BLOCKS; i++) {
        if (ctx->block[i].state == BLOCK_APP_OWNED &&
            !ctx->block[i].retired) { /* R17-02: retired never reused */
            return i;
        }
    }
    return -1;
}

int gpib_ledger_txn_slot(const struct gpib_context *ctx, gpib_handle_t h) {
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return -1;
    }
    if (h.epoch != ctx->epoch) {
        return -1; /* handle from a previous instance never matches (TXN-05) */
    }
    if (h.kind != (uint8_t)GPIB_HANDLE_TXN) {
        return -1; /* wrong-kind handle */
    }
    if (h.slot >= (uint16_t)GPIB_CFG_MAX_OPERATIONS) {
        return -1;
    }
    if (ctx->txn[h.slot].state == TXN_UNRESERVED ||
        ctx->txn[h.slot].state == TXN_RECORD_RELEASED) {
        return -1;
    }
    if (ctx->txn[h.slot].generation != h.generation) {
        return -1; /* stale/double handle after ACK bumped the generation */
    }
    return (int)h.slot;
}

int gpib_ledger_block_slot(const struct gpib_context *ctx, gpib_handle_t h) {
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return -1;
    }
    if (h.epoch != ctx->epoch) {
        return -1;
    }
    if (h.kind != (uint8_t)GPIB_HANDLE_BLOCK) {
        return -1;
    }
    if (h.slot >= (uint16_t)GPIB_CFG_MAX_BLOCKS) {
        return -1;
    }
    if (ctx->block[h.slot].state == BLOCK_APP_OWNED) {
        return -1;
    }
    if (ctx->block[h.slot].generation != h.generation) {
        return -1;
    }
    return (int)h.slot;
}

gpib_handle_t gpib_make_handle(const struct gpib_context *ctx, uint8_t kind,
    uint16_t slot, uint32_t generation) {
    gpib_handle_t h;
    h.epoch = ctx->epoch;
    h.slot = slot;
    h.generation = generation;
    h.kind = kind;
    h.reserved = 0;
    return h;
}

void gpib_ledger_terminalize_txn_blocks(struct gpib_context *ctx,
    uint16_t txn_slot) {
    struct gpib_txn_record *t = &ctx->txn[txn_slot];
    for (int i = 0; i < GPIB_CFG_MAX_BLOCKS; i++) {
        struct gpib_block_record *b = &ctx->block[i];
        if (b->state == BLOCK_CORE_QUEUED && b->txn_slot == txn_slot &&
            b->txn_generation == t->generation) {
            b->result.status = GPIB_OK;
            b->result.reason = GPIB_TERM_NOT_STARTED;
            b->result.end_cause = GPIB_END_NONE;
            b->result.bytes_confirmed = 0;
            b->result.last_byte_uncertain = false;
            b->state = BLOCK_RESULT_READY;
            t->blocks_terminal++;
        }
    }
}

void gpib_ledger_try_publish_txn(struct gpib_context *ctx, uint16_t txn_slot,
    gpib_status_t status, gpib_term_reason_t reason, gpib_end_cause_t end) {
    struct gpib_txn_record *t = &ctx->txn[txn_slot];
    if (t->state != TXN_FINISHING) {
        return;
    }
    if (t->blocks_terminal != t->block_count) {
        /* Guard of TXN_FINISHING: every accepted block must own its terminal
         * record first; no falsely final result is published. */
        return;
    }
    t->result.status = status;
    t->result.reason = reason;
    t->result.end_cause = end;
    t->result.bytes_confirmed = t->stream_bytes;
    /* N11/D6: the transaction inherits the WORST byte-fate proof of its
     * blocks -- a mid-byte abort in ANY block (ATN takeover, IFC, FAULT)
     * must surface in the transaction result, not vanish in the
     * aggregation (the hardcoded false here used to mask exactly the
     * case the final audit's D6 reproduced). */
    t->result.last_byte_uncertain = false;
    for (int i = 0; i < GPIB_CFG_MAX_BLOCKS; i++) {
        struct gpib_block_record *b = &ctx->block[i];
        if (b->txn_slot == txn_slot &&
            b->txn_generation == t->generation &&
            b->result.last_byte_uncertain) {
            t->result.last_byte_uncertain = true;
            break;
        }
    }
    t->result.stb = (t->kind == GPIB_REQ_SPOLL) ? t->spoll_stb
        : (t->kind == GPIB_REQ_PPOLL) ? t->ppr_byte : 0u;
    t->state = TXN_RESULT_READY;
}

bool gpib_ledger_all_terminal(const struct gpib_context *ctx) {
    for (int i = 0; i < GPIB_CFG_MAX_OPERATIONS; i++) {
        gpib_txn_state_t st = ctx->txn[i].state;
        if (st == TXN_RESERVED || st == TXN_OPEN || st == TXN_FINISHING) {
            return false;
        }
    }
    for (int i = 0; i < GPIB_CFG_MAX_BLOCKS; i++) {
        gpib_block_state_t st = ctx->block[i].state;
        if (st == BLOCK_APP_OWNED || st == BLOCK_RESULT_READY) {
            continue;
        }
        /* R17-02: a RETIRED slot (generation exhausted) is terminally
         * out of circulation -- its WAIT_QUIESCENCE-like state must not
         * block the lifecycle drain. */
        if (ctx->block[i].retired) {
            continue;
        }
        return false;
    }
    return true;
}

size_t gpib_ledger_unacked(const struct gpib_context *ctx) {
    size_t n = 0;
    for (int i = 0; i < GPIB_CFG_MAX_OPERATIONS; i++) {
        if (ctx->txn[i].state == TXN_RESULT_READY) {
            n++;
        }
    }
    for (int i = 0; i < GPIB_CFG_MAX_BLOCKS; i++) {
        if (ctx->block[i].state == BLOCK_RESULT_READY) {
            n++;
        }
    }
    return n;
}
